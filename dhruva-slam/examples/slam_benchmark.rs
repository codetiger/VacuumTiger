//! Unified SLAM Algorithm Benchmarking Tool
//!
//! Benchmarks scan matching algorithms and full SLAM pipelines using recorded bag files.
//! Provides both scan-level accuracy metrics and end-to-end map quality assessment.
//!
//! # Modes
//!
//! - `scan-accuracy`: Tests scan matching using random scan pairs vs odometry ground truth
//! - `full-slam`: Runs complete SLAM pipeline and generates maps
//! - `all` (default): Runs both modes
//!
//! # Usage
//!
//! ```bash
//! # Run full benchmark (both modes)
//! slam-benchmark --bag recordings/square.bag --output results/
//!
//! # Scan matching accuracy only
//! slam-benchmark scan-accuracy --bag recordings/square.bag --num-tests 100
//!
//! # Full SLAM pipeline only
//! slam-benchmark full-slam --bag recordings/square.bag --output results/
//!
//! # Generate markdown summary from results
//! slam-benchmark summarize --input results/ --output summary.md
//! ```

use std::fs;
use std::io::Write as IoWrite;
use std::path::Path;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use image::{GrayImage, Luma};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use serde::{Deserialize, Serialize};
use sysinfo::System;

use dhruva_slam::algorithms::mapping::{
    CellState, FeatureExtractor, FeatureExtractorConfig, MapIntegrator, MapIntegratorConfig,
    OccupancyGrid, OccupancyGridConfig,
};
use dhruva_slam::algorithms::matching::{
    CorrelativeConfig, CorrelativeMatcher, DynMatcher, HybridIcpMatcher, HybridIcpMatcherConfig,
    HybridP2LMatcher, HybridP2LMatcherConfig, IcpConfig, MatcherType, MultiResolutionConfig,
    MultiResolutionMatcher, PointToLineIcp, PointToLineIcpConfig, PointToPointIcp,
    ScanMatchValidator, ScanMatchValidatorConfig, ScanMatcher,
};
use dhruva_slam::core::types::{LaserScan, Point2D, PointCloud2D, Pose2D, PoseTracker};
use dhruva_slam::io::bag::{BagMessage, BagPlayer};
use dhruva_slam::metrics::{
    MapNoiseMetrics, TransformError, TransformErrorStats, analyze_map_noise,
    compute_transform_error,
};
use dhruva_slam::sensors::odometry::{
    ComplementaryConfig, ComplementaryFilter, DynOdometry, DynOdometryConfig, OdometryType,
    WheelOdometry, WheelOdometryConfig,
};
use dhruva_slam::sensors::preprocessing::{LidarOffset, PreprocessorConfig, ScanPreprocessor};
use dhruva_slam::utils::{GYRO_SCALE, WHEEL_BASE, WHEEL_TICKS_PER_METER};

/// Lidar mounting offset from robot center (meters).
/// Calibrated from rotation test: lidar traces arc during in-place rotation.
/// Coordinate frame: X=forward, Y=left (ROS REP-103)

// 1. Lidar housing mounting position relative to robot center
const LIDAR_MOUNTING_X: f32 = -0.110; // 93mm behind center
const LIDAR_MOUNTING_Y: f32 = 0.0; // Centered on robot axis

// 2. Optical center offset (sensor specific)
// Distance from housing center to laser origin (rotating scanner)
// From calibration: total ≈ -91mm, mounting = -93mm, so optical ≈ 0
const LIDAR_OPTICAL_OFFSET: f32 = 0.02;

// 3. Angular offset (if lidar 0° is not aligned with robot forward)
const LIDAR_ANGLE_OFFSET: f32 = 12.5;

// Total X offset = -0.093 + (-0.020) = -0.113m (113mm behind robot center)

// ============================================================================
// CLI Arguments
// ============================================================================

#[derive(Parser)]
#[command(name = "slam-benchmark")]
#[command(about = "Unified SLAM algorithm benchmarking tool")]
struct Args {
    #[command(subcommand)]
    command: Option<Commands>,

    /// Input bag file (required for all modes except summarize)
    #[arg(short, long)]
    bag: Option<String>,

    /// Output directory for results
    #[arg(short, long, default_value = "results")]
    output: String,

    /// Run all algorithm combinations
    #[arg(long)]
    run_all: bool,

    /// Scan matcher type (for single runs)
    #[arg(long, value_enum, default_value = "hybrid-p2l")]
    matcher: MatcherType,

    /// Odometry filter type (for single runs)
    #[arg(long, value_enum, default_value = "complementary")]
    odometry: OdometryType,
}

#[derive(Subcommand)]
enum Commands {
    /// Test scan matching accuracy using random scan pairs
    ScanAccuracy {
        /// Input bag file
        #[arg(short, long)]
        bag: String,

        /// Number of random test pairs
        #[arg(long, default_value = "100")]
        num_tests: usize,

        /// Minimum scan separation
        #[arg(long, default_value = "1")]
        min_offset: usize,

        /// Maximum scan separation
        #[arg(long, default_value = "30")]
        max_offset: usize,

        /// Random seed for reproducibility
        #[arg(long)]
        seed: Option<u64>,

        /// Output directory
        #[arg(short, long, default_value = "results")]
        output: String,
    },

    /// Run full SLAM pipeline and generate maps
    FullSlam {
        /// Input bag file
        #[arg(short, long)]
        bag: String,

        /// Output directory
        #[arg(short, long, default_value = "results")]
        output: String,

        /// Run all algorithm combinations
        #[arg(long)]
        run_all: bool,

        /// Scan matcher type
        #[arg(long, value_enum, default_value = "hybrid-p2l")]
        matcher: MatcherType,

        /// Odometry filter type
        #[arg(long, value_enum, default_value = "complementary")]
        odometry: OdometryType,
    },

    /// Generate markdown summary from results
    Summarize {
        /// Directory containing JSON result files
        #[arg(short, long)]
        input: String,

        /// Output markdown file
        #[arg(short, long, default_value = "BENCHMARK_RESULTS.md")]
        output: String,
    },
}

// ============================================================================
// Result Structures
// ============================================================================

/// Scan matching accuracy results for a single algorithm.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanAccuracyResult {
    pub matcher_name: String,
    pub num_tests: usize,
    pub convergence_rate: f32,
    pub mean_trans_error_cm: f32,
    pub median_trans_error_cm: f32,
    pub mean_rot_error_deg: f32,
    pub median_rot_error_deg: f32,
    pub avg_time_ms: f32,
}

/// Full SLAM pipeline benchmark result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FullSlamResult {
    pub bag_file: String,
    pub matcher_type: MatcherType,
    pub odometry_type: OdometryType,

    // Performance metrics
    pub total_time_ms: f64,
    pub scans_processed: u64,
    pub avg_scan_time_ms: f64,
    pub peak_memory_mb: f64,

    // Map quality metrics
    pub map_noise: MapNoiseMetrics,

    // Accuracy metrics (new!)
    pub scan_accuracy: Option<ScanAccuracyStats>,

    // Final pose (for loop closure scenarios)
    pub final_pose: (f64, f64, f64),

    // Match statistics
    pub avg_match_score: f64,
    pub failed_matches: u64,
}

/// Inline scan accuracy stats during SLAM.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanAccuracyStats {
    pub mean_trans_error_cm: f32,
    pub mean_rot_error_deg: f32,
    pub samples: usize,
}

/// Combined benchmark report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkReport {
    pub bag_file: String,
    pub scan_accuracy: Vec<ScanAccuracyResult>,
    pub full_slam: Vec<FullSlamResult>,
}

// ============================================================================
// Main Entry Point
// ============================================================================

fn main() {
    env_logger::init();
    let args = Args::parse();

    if let Err(e) = run(args) {
        eprintln!("Error: {}", e);
        std::process::exit(1);
    }
}

fn run(args: Args) -> Result<(), Box<dyn std::error::Error>> {
    match args.command {
        Some(Commands::ScanAccuracy {
            bag,
            num_tests,
            min_offset,
            max_offset,
            seed,
            output,
        }) => {
            run_scan_accuracy_benchmark(&bag, num_tests, min_offset, max_offset, seed, &output)?;
        }

        Some(Commands::FullSlam {
            bag,
            output,
            run_all,
            matcher,
            odometry,
        }) => {
            fs::create_dir_all(&output)?;
            if run_all {
                run_all_full_slam(&bag, &output)?;
            } else {
                let (result, map) = run_single_full_slam(&bag, matcher, odometry)?;
                save_full_slam_result(&result, &output)?;
                save_map_png(&result, &map, &output)?;
                print_full_slam_result(&result);
            }
        }

        Some(Commands::Summarize { input, output }) => {
            generate_summary(&input, &output)?;
        }

        None => {
            // Default: run both benchmarks
            let bag_path = args.bag.ok_or("Missing --bag argument")?;
            fs::create_dir_all(&args.output)?;

            println!("╔══════════════════════════════════════════════════════════════════╗");
            println!("║           UNIFIED SLAM BENCHMARK SUITE                           ║");
            println!("╚══════════════════════════════════════════════════════════════════╝\n");

            // Phase 1: Scan accuracy
            println!("=== Phase 1: Scan Matching Accuracy ===\n");
            run_scan_accuracy_benchmark(&bag_path, 100, 1, 30, None, &args.output)?;

            println!("\n=== Phase 2: Full SLAM Pipeline ===\n");
            if args.run_all {
                run_all_full_slam(&bag_path, &args.output)?;
            } else {
                let (result, map) = run_single_full_slam(&bag_path, args.matcher, args.odometry)?;
                save_full_slam_result(&result, &args.output)?;
                save_map_png(&result, &map, &args.output)?;
                print_full_slam_result(&result);
            }

            // Generate summary
            generate_summary(
                &args.output,
                &format!("{}/BENCHMARK_RESULTS.md", args.output),
            )?;
        }
    }

    Ok(())
}

// ============================================================================
// Scan Accuracy Benchmark
// ============================================================================

/// Tracks accuracy stats for an algorithm.
struct AlgorithmAccuracyTracker {
    name: String,
    error_stats: TransformErrorStats,
    time_sum_us: u128,
    trans_threshold: f32,
    rot_threshold: f32,
}

impl AlgorithmAccuracyTracker {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            error_stats: TransformErrorStats::new(),
            time_sum_us: 0,
            trans_threshold: 0.05, // 5cm
            rot_threshold: 0.05,   // ~3 degrees
        }
    }

    fn record(&mut self, error: TransformError, offset: usize, time_us: u128) {
        let converged = error.converged(self.trans_threshold, self.rot_threshold);
        self.error_stats.record(error, offset, converged);
        self.time_sum_us += time_us;
    }

    fn avg_time_ms(&self) -> f32 {
        let n = self.error_stats.total_pairs();
        if n == 0 {
            0.0
        } else {
            self.time_sum_us as f32 / n as f32 / 1000.0
        }
    }

    fn to_result(&self) -> ScanAccuracyResult {
        ScanAccuracyResult {
            matcher_name: self.name.clone(),
            num_tests: self.error_stats.total_pairs(),
            convergence_rate: self.error_stats.convergence_rate(),
            mean_trans_error_cm: self.error_stats.mean_translation_error() * 100.0,
            median_trans_error_cm: self.error_stats.median_translation_error() * 100.0,
            mean_rot_error_deg: self.error_stats.mean_rotation_error().to_degrees(),
            median_rot_error_deg: self.error_stats.median_rotation_error().to_degrees(),
            avg_time_ms: self.avg_time_ms(),
        }
    }
}

fn run_scan_accuracy_benchmark(
    bag_path: &str,
    num_tests: usize,
    min_offset: usize,
    max_offset: usize,
    seed: Option<u64>,
    output_dir: &str,
) -> Result<Vec<ScanAccuracyResult>, Box<dyn std::error::Error>> {
    println!("Bag file: {}", bag_path);
    println!(
        "Test pairs: {} random pairs (offset range: {}-{} scans)",
        num_tests, min_offset, max_offset
    );

    // Load bag and collect scans with odometry predictions
    let (scans, predictions) = load_scans_with_odometry(bag_path)?;

    println!(
        "Loaded {} scans, {} odometry predictions",
        scans.len(),
        predictions.len()
    );

    if scans.len() < min_offset + 2 {
        return Err("Not enough scans for requested offset range".into());
    }

    // Generate random test pairs
    let test_pairs = generate_random_pairs(scans.len(), num_tests, min_offset, max_offset, seed);
    println!("Generated {} test pairs\n", test_pairs.len());

    // Initialize all matchers
    let mut icp = PointToPointIcp::new(IcpConfig::default());
    let mut p2l = PointToLineIcp::new(PointToLineIcpConfig::default());
    let mut correlative = CorrelativeMatcher::new(CorrelativeConfig::default());
    let mut multi_res = MultiResolutionMatcher::new(MultiResolutionConfig::default());
    let mut hybrid_icp = HybridIcpMatcher::from_config(HybridIcpMatcherConfig::default());
    let mut hybrid_p2l = HybridP2LMatcher::from_config(HybridP2LMatcherConfig::default());

    // Initialize trackers
    let mut icp_tracker = AlgorithmAccuracyTracker::new("P2P ICP");
    let mut p2l_tracker = AlgorithmAccuracyTracker::new("P2L ICP");
    let mut corr_tracker = AlgorithmAccuracyTracker::new("Correlative");
    let mut multi_tracker = AlgorithmAccuracyTracker::new("Multi-Res");
    let mut hybrid_icp_tracker = AlgorithmAccuracyTracker::new("Hybrid ICP");
    let mut hybrid_p2l_tracker = AlgorithmAccuracyTracker::new("Hybrid P2L");

    print!("Running tests");
    std::io::stdout().flush().ok();

    for (idx, &(base_idx, target_idx)) in test_pairs.iter().enumerate() {
        let source = &scans[base_idx];
        let target = &scans[target_idx];
        let offset = target_idx - base_idx;

        // Ground truth: accumulated odometry
        // The scan matcher returns transform from source→target, which equals robot motion
        let expected_odom = accumulate_odometry(&predictions, base_idx, target_idx);
        let initial_guess = expected_odom;

        // Test each algorithm
        let start = Instant::now();
        let result = icp.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom);
        icp_tracker.record(error, offset, elapsed);

        let start = Instant::now();
        let result = p2l.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom);
        p2l_tracker.record(error, offset, elapsed);

        let start = Instant::now();
        let result = correlative.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom);
        corr_tracker.record(error, offset, elapsed);

        let start = Instant::now();
        let result = multi_res.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom);
        multi_tracker.record(error, offset, elapsed);

        let start = Instant::now();
        let result = hybrid_icp.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom);
        hybrid_icp_tracker.record(error, offset, elapsed);

        let start = Instant::now();
        let result = hybrid_p2l.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom);
        hybrid_p2l_tracker.record(error, offset, elapsed);

        if (idx + 1) % 10 == 0 {
            print!(".");
            std::io::stdout().flush().ok();
        }
    }

    println!(" done!\n");

    // Collect results
    let results = vec![
        icp_tracker.to_result(),
        p2l_tracker.to_result(),
        corr_tracker.to_result(),
        multi_tracker.to_result(),
        hybrid_icp_tracker.to_result(),
        hybrid_p2l_tracker.to_result(),
    ];

    // Print table
    println!("┌─────────────────┬────────┬─────────────┬─────────────┬──────────┐");
    println!("│ Algorithm       │ Conv%  │ Trans Error │ Rot Error   │ Avg Time │");
    println!("├─────────────────┼────────┼─────────────┼─────────────┼──────────┤");

    for r in &results {
        println!(
            "│ {:15} │ {:5.1}% │ {:4.1}/{:4.1}cm │ {:4.1}/{:4.1}° │ {:6.2}ms │",
            r.matcher_name,
            r.convergence_rate,
            r.mean_trans_error_cm,
            r.median_trans_error_cm,
            r.mean_rot_error_deg,
            r.median_rot_error_deg,
            r.avg_time_ms,
        );
    }

    println!("└─────────────────┴────────┴─────────────┴─────────────┴──────────┘");

    // Save to JSON
    fs::create_dir_all(output_dir)?;
    let json_path = Path::new(output_dir).join("scan_accuracy.json");
    let json = serde_json::to_string_pretty(&results)?;
    fs::write(&json_path, json)?;
    println!("\nResults saved to: {}", json_path.display());

    Ok(results)
}

fn load_scans_with_odometry(
    bag_path: &str,
) -> Result<(Vec<PointCloud2D>, Vec<Pose2D>), Box<dyn std::error::Error>> {
    // First pass: Estimate gyro bias from initial samples (assumes robot starts stationary)
    let gyro_bias = estimate_gyro_bias(bag_path)?;

    // Second pass: Load scans with bias-corrected complementary filter
    let mut player = BagPlayer::open(bag_path)?;
    let preprocessor = ScanPreprocessor::new(PreprocessorConfig {
        lidar_offset: LidarOffset::new(
            LIDAR_MOUNTING_X,
            LIDAR_MOUNTING_Y,
            LIDAR_OPTICAL_OFFSET,
            LIDAR_ANGLE_OFFSET,
        ),
        ..PreprocessorConfig::default()
    });

    let odom_config = WheelOdometryConfig {
        ticks_per_meter: WHEEL_TICKS_PER_METER,
        wheel_base: WHEEL_BASE,
    };
    let mut wheel_odom = WheelOdometry::new(odom_config);

    // Use ComplementaryFilter with estimated gyro bias
    // ROS REP-103: gyro_raw[2] is the yaw (Z) axis
    let comp_config = ComplementaryConfig {
        alpha: 0.8,
        gyro_scale: GYRO_SCALE,
        gyro_bias_z: gyro_bias, // Raw bias value (filter applies sign correction)
        ..ComplementaryConfig::default()
    };
    let mut comp_filter = ComplementaryFilter::new(comp_config);

    let mut scans: Vec<PointCloud2D> = Vec::new();
    let mut predictions: Vec<Pose2D> = Vec::new();
    let mut last_sensor_timestamp_us: Option<u64> = None;
    let mut accumulated_delta = Pose2D::identity();

    while let Ok(Some(msg)) = player.next_immediate() {
        match msg {
            BagMessage::SensorStatus(status) => {
                // Get wheel odometry delta
                if let Some(delta) = wheel_odom.update(status.encoder.left, status.encoder.right) {
                    // Update complementary filter with encoder delta and gyro
                    // ROS REP-103: gyro_raw[2] is the yaw (Z) axis
                    let timestamp_us = status.timestamp_us;
                    if last_sensor_timestamp_us.is_some() {
                        // ComplementaryFilter handles bias correction and stationary detection
                        comp_filter.update(delta, status.gyro_raw[2], timestamp_us);
                    }
                    accumulated_delta = accumulated_delta.compose(&delta);
                }
                last_sensor_timestamp_us = Some(status.timestamp_us);
            }
            BagMessage::Lidar(timestamped) => {
                let laser_scan = LaserScan::from_lidar_scan(&timestamped.data);
                let processed = preprocessor.process(&laser_scan);

                if processed.len() >= 50 {
                    if !scans.is_empty() {
                        // Get the fused pose from complementary filter
                        let fused_pose = comp_filter.pose();
                        // The prediction is the delta since last scan
                        predictions.push(fused_pose);
                    } else {
                        wheel_odom.reset();
                    }
                    scans.push(processed);
                    // Reset filter and accumulated delta for next interval
                    comp_filter.reset();
                    accumulated_delta = Pose2D::identity();
                }
            }
            _ => {}
        }
    }

    Ok((scans, predictions))
}

/// Estimate gyro bias from the first N samples of a bag file.
/// Assumes the robot starts stationary.
fn estimate_gyro_bias(bag_path: &str) -> Result<f32, Box<dyn std::error::Error>> {
    let mut player = BagPlayer::open(bag_path)?;
    const BIAS_SAMPLES: usize = 100; // ~2 seconds at 50Hz

    let mut gyro_samples: Vec<i16> = Vec::with_capacity(BIAS_SAMPLES);

    while let Ok(Some(msg)) = player.next_immediate() {
        if let BagMessage::SensorStatus(status) = msg {
            // ROS REP-103: gyro_raw[2] is the yaw (Z) axis
            gyro_samples.push(status.gyro_raw[2]);
            if gyro_samples.len() >= BIAS_SAMPLES {
                break;
            }
        }
    }

    if gyro_samples.is_empty() {
        return Ok(0.0);
    }

    let bias = gyro_samples.iter().map(|&g| g as f32).sum::<f32>() / gyro_samples.len() as f32;
    Ok(bias)
}

fn generate_random_pairs(
    num_scans: usize,
    num_tests: usize,
    min_offset: usize,
    max_offset: usize,
    seed: Option<u64>,
) -> Vec<(usize, usize)> {
    let mut rng: StdRng = match seed {
        Some(s) => StdRng::seed_from_u64(s),
        None => StdRng::from_os_rng(),
    };

    let max_valid_offset = max_offset.min(num_scans - 1);
    if max_valid_offset < min_offset {
        return Vec::new();
    }

    let max_base_idx = num_scans - max_valid_offset - 1;
    if max_base_idx == 0 {
        return Vec::new();
    }

    (0..num_tests)
        .map(|_| {
            let base_idx = rng.random_range(0..max_base_idx);
            let offset = rng.random_range(min_offset..=max_valid_offset);
            (base_idx, base_idx + offset)
        })
        .collect()
}

fn accumulate_odometry(predictions: &[Pose2D], start: usize, end: usize) -> Pose2D {
    predictions[start..end]
        .iter()
        .fold(Pose2D::identity(), |acc, delta| acc.compose(delta))
}

/// Extract occupied cells from occupancy grid as a point cloud for scan-to-map matching.
fn map_to_point_cloud(map: &OccupancyGrid) -> PointCloud2D {
    let mut points = Vec::new();
    let (width, height) = map.dimensions();

    for cy in 0..height {
        for cx in 0..width {
            if map.get_state(cx, cy) == CellState::Occupied {
                let (x, y) = map.cell_to_world(cx, cy);
                points.push(Point2D::new(x, y));
            }
        }
    }

    PointCloud2D::from_points(points)
}

// ============================================================================
// Full SLAM Benchmark
// ============================================================================

fn run_all_full_slam(bag_path: &str, output_dir: &str) -> Result<(), Box<dyn std::error::Error>> {
    let matchers = [
        MatcherType::Icp,
        MatcherType::P2l,
        MatcherType::Correlative,
        MatcherType::MultiRes,
        MatcherType::HybridIcp,
        MatcherType::HybridP2l,
    ];

    let odometries = [
        OdometryType::Wheel,
        OdometryType::Complementary,
        OdometryType::Eskf,
        OdometryType::Mahony,
    ];

    let total = matchers.len() * odometries.len();
    let mut count = 0;

    println!("Bag file: {}", bag_path);
    println!("Running {} algorithm combinations...\n", total);

    for matcher in &matchers {
        for odometry in &odometries {
            count += 1;
            println!("[{}/{}] {} + {}...", count, total, matcher, odometry);

            match run_single_full_slam(bag_path, *matcher, *odometry) {
                Ok((result, map)) => {
                    save_full_slam_result(&result, output_dir)?;
                    save_map_png(&result, &map, output_dir)?;
                    println!(
                        "  -> {:.1}ms/scan, coherence: {:.1}%, density: {:.0}, area: {:.1}m²",
                        result.avg_scan_time_ms,
                        result.map_noise.wall_coherence * 100.0,
                        result.map_noise.occupied_density,
                        result.map_noise.map_area_m2
                    );
                }
                Err(e) => {
                    eprintln!("  -> FAILED: {}", e);
                }
            }
        }
    }

    println!("\nResults saved to: {}/", output_dir);
    Ok(())
}

fn run_single_full_slam(
    bag_path: &str,
    matcher_type: MatcherType,
    odometry_type: OdometryType,
) -> Result<(FullSlamResult, OccupancyGrid), Box<dyn std::error::Error>> {
    let mut player = BagPlayer::open(bag_path)?;
    player.set_speed(0.0);

    // Create dynamic odometry with robot-specific wheel configuration
    let odom_config = DynOdometryConfig {
        wheel: WheelOdometryConfig {
            ticks_per_meter: WHEEL_TICKS_PER_METER,
            wheel_base: WHEEL_BASE,
        },
        complementary: ComplementaryConfig {
            alpha: 0.8,
            gyro_scale: GYRO_SCALE,
            gyro_bias_z: 0.0,
            ..ComplementaryConfig::default()
        },
        ..DynOdometryConfig::default()
    };
    let mut odometry = DynOdometry::new(odometry_type, odom_config);
    let mut matcher = DynMatcher::new(matcher_type);
    let preprocessor = ScanPreprocessor::new(PreprocessorConfig {
        lidar_offset: LidarOffset::new(
            LIDAR_MOUNTING_X,
            LIDAR_MOUNTING_Y,
            LIDAR_OPTICAL_OFFSET,
            LIDAR_ANGLE_OFFSET,
        ),
        ..PreprocessorConfig::default()
    });
    let mut map = OccupancyGrid::new(OccupancyGridConfig::default());
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Use PoseTracker for odometry and SLAM pose management
    let mut odom_tracker = PoseTracker::new(); // Raw odometry
    let mut slam_pose = Pose2D::identity(); // Scan-matched pose
    let mut scans_processed = 0u64;
    let mut failed_matches = 0u64;
    let mut total_score = 0.0f64;

    let mut total_scan_time = Duration::ZERO;

    let mut sys = System::new();
    let mut peak_memory_mb = 0.0f64;
    let pid = sysinfo::get_current_pid().ok();

    // Use ScanMatchValidator for accepting/rejecting scan match results
    let validator = ScanMatchValidator::new(ScanMatchValidatorConfig::default());

    let start_time = Instant::now();

    while let Ok(Some(msg)) = player.next_immediate() {
        match msg {
            BagMessage::SensorStatus(status) => {
                // ROS REP-103: gyro_raw[2] is the yaw (Z) axis
                if let Some(pose) = odometry.update(
                    status.encoder.left,
                    status.encoder.right,
                    status.gyro_raw[2],
                    status.timestamp_us,
                ) {
                    odom_tracker.set(pose); // Always raw odometry
                }
            }
            BagMessage::Lidar(timestamped) => {
                let scan_start = Instant::now();
                let laser_scan = LaserScan::from_lidar_scan(&timestamped.data);
                let processed = preprocessor.process(&laser_scan);

                if processed.len() >= 50 {
                    // Compute robot motion from RAW odometry using PoseTracker
                    let odom_delta = odom_tracker.delta_since_snapshot();

                    // Predicted pose from odometry
                    let predicted_pose = slam_pose.compose(&odom_delta);

                    // SCAN-TO-MAP MATCHING: Match against accumulated map instead of previous scan
                    // This prevents drift accumulation since map is a stable reference
                    if scans_processed > 0 {
                        // Extract current map as point cloud for matching
                        let map_cloud = map_to_point_cloud(&map);

                        // Only match if map has enough points
                        if map_cloud.len() >= 50 {
                            // Transform map points to local frame (relative to predicted pose)
                            // This way scan (local) matches against map (transformed to local)
                            let map_local = map_cloud.transform(&predicted_pose.inverse());

                            // Match local scan against local-frame map
                            // Initial guess is identity since scan is at origin in local frame
                            let match_result =
                                matcher.match_scans(&processed, &map_local, &Pose2D::identity());

                            // The result is the correction in local frame
                            // Validate: the correction should be small (near identity)
                            let validation = validator.validate(&match_result, &Pose2D::identity());

                            if validation.is_valid {
                                // Apply correction: predicted_pose composed with local correction
                                slam_pose = predicted_pose.compose(&match_result.transform);
                                total_score += match_result.score as f64;
                            } else {
                                // Fall back to odometry prediction
                                slam_pose = predicted_pose;
                                failed_matches += 1;
                            }
                        } else {
                            // Not enough map points yet, use odometry
                            slam_pose = predicted_pose;
                        }
                    }

                    // Transform scan using SLAM pose (not raw odometry)
                    let global_scan = processed.transform(&slam_pose);
                    integrator.integrate_cloud(&mut map, &global_scan, &slam_pose);

                    // Take snapshot for next delta computation
                    odom_tracker.take_snapshot();
                    scans_processed += 1;
                }

                total_scan_time += scan_start.elapsed();

                if scans_processed % 10 == 0 {
                    sys.refresh_all();
                    if let Some(pid) = pid {
                        if let Some(process) = sys.process(pid) {
                            let mem_mb = process.memory() as f64 / (1024.0 * 1024.0);
                            peak_memory_mb = peak_memory_mb.max(mem_mb);
                        }
                    }
                }
            }
            BagMessage::Odometry(_) => {}
        }
    }

    let total_time = start_time.elapsed();
    let map_noise = analyze_map_noise(&map);

    // Extract features from the map
    let mut feature_extractor = FeatureExtractor::new(FeatureExtractorConfig::default());
    let features = feature_extractor.extract(&map);
    println!(
        "    Features: {} lines, {} corners",
        features.lines.len(),
        features.corners.len()
    );
    for (i, line) in features.lines.iter().enumerate() {
        println!(
            "      Line {}: ({:.2},{:.2}) -> ({:.2},{:.2}), len={:.2}m, qual={:.2}, pts={}",
            i,
            line.start.x,
            line.start.y,
            line.end.x,
            line.end.y,
            line.length,
            line.quality,
            line.point_count
        );
    }
    for (i, corner) in features.corners.iter().enumerate() {
        println!(
            "      Corner {}: ({:.2},{:.2}), angle={:.1}°, type={:?}",
            i,
            corner.position.x,
            corner.position.y,
            corner.angle.to_degrees(),
            corner.corner_type
        );
    }

    let avg_scan_time_ms = if scans_processed > 0 {
        total_scan_time.as_secs_f64() * 1000.0 / scans_processed as f64
    } else {
        0.0
    };

    let successful_matches = scans_processed
        .saturating_sub(failed_matches)
        .saturating_sub(1); // -1 for first scan
    let avg_match_score = if successful_matches > 0 {
        total_score / successful_matches as f64
    } else {
        0.0
    };

    let result = FullSlamResult {
        bag_file: Path::new(bag_path)
            .file_name()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| bag_path.to_string()),
        matcher_type,
        odometry_type,
        total_time_ms: total_time.as_secs_f64() * 1000.0,
        scans_processed,
        avg_scan_time_ms,
        peak_memory_mb,
        map_noise,
        scan_accuracy: None, // TODO: Add inline accuracy tracking
        final_pose: (
            slam_pose.x as f64,
            slam_pose.y as f64,
            slam_pose.theta as f64,
        ),
        avg_match_score,
        failed_matches,
    };

    Ok((result, map))
}

// ============================================================================
// Output Functions
// ============================================================================

fn save_full_slam_result(
    result: &FullSlamResult,
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let filename = format!(
        "{}_{:?}_{:?}.json",
        result.bag_file.replace(".bag", ""),
        result.matcher_type,
        result.odometry_type
    );
    let path = Path::new(output_dir).join(&filename);

    let json = serde_json::to_string_pretty(result)?;
    fs::write(&path, json)?;

    Ok(())
}

fn save_map_png(
    result: &FullSlamResult,
    map: &OccupancyGrid,
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let filename = format!(
        "{}_{:?}_{:?}.png",
        result.bag_file.replace(".bag", ""),
        result.matcher_type,
        result.odometry_type
    );
    let path = Path::new(output_dir).join(&filename);

    let (width, height, pixels) = map.to_grayscale();

    let img = GrayImage::from_fn(width as u32, height as u32, |x, y| {
        let flipped_y = height - 1 - y as usize;
        let idx = flipped_y * width + x as usize;
        Luma([pixels[idx]])
    });

    img.save(&path)?;

    Ok(())
}

fn print_full_slam_result(result: &FullSlamResult) {
    println!("\n=== Full SLAM Result ===");
    println!("Bag: {}", result.bag_file);
    println!("Matcher: {:?}", result.matcher_type);
    println!("Odometry: {:?}", result.odometry_type);
    println!();
    println!("Performance:");
    println!("  Total time: {:.1} ms", result.total_time_ms);
    println!("  Scans processed: {}", result.scans_processed);
    println!("  Avg scan time: {:.2} ms", result.avg_scan_time_ms);
    println!("  Peak memory: {:.1} MB", result.peak_memory_mb);
    println!();
    println!("Map Quality:");
    println!("  Occupied cells: {}", result.map_noise.occupied_cells);
    println!("  Free cells: {}", result.map_noise.free_cells);
    println!(
        "  Wall coherence: {:.1}%",
        result.map_noise.wall_coherence * 100.0
    );
    println!("  Isolated cells: {}", result.map_noise.isolated_cells);
    println!(
        "  Map area: {:.2} m² ({:.2}x{:.2}m)",
        result.map_noise.map_area_m2,
        result.map_noise.bounds.2 - result.map_noise.bounds.0,
        result.map_noise.bounds.3 - result.map_noise.bounds.1
    );
    println!(
        "  Occupied density: {:.0} cells/m²",
        result.map_noise.occupied_density
    );
    println!(
        "  Wall compactness: {:.3}",
        result.map_noise.wall_compactness
    );
    println!("  Aspect ratio: {:.2}", result.map_noise.aspect_ratio);
    println!();
    println!("Matching:");
    println!("  Avg match score: {:.3}", result.avg_match_score);
    println!("  Failed matches: {}", result.failed_matches);
    println!();
    println!(
        "Final Pose: ({:.3}, {:.3}, {:.1}°)",
        result.final_pose.0,
        result.final_pose.1,
        result.final_pose.2.to_degrees()
    );
}

fn generate_summary(input_dir: &str, output_file: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("Generating summary from: {}", input_dir);

    let mut full_slam_results: Vec<FullSlamResult> = Vec::new();
    let mut scan_accuracy_results: Vec<ScanAccuracyResult> = Vec::new();

    for entry in fs::read_dir(input_dir)? {
        let entry = entry?;
        let path = entry.path();

        if path.extension().map(|e| e == "json").unwrap_or(false) {
            let content = fs::read_to_string(&path)?;

            // Try to parse as full SLAM result
            if let Ok(result) = serde_json::from_str::<FullSlamResult>(&content) {
                full_slam_results.push(result);
            }
            // Try to parse as scan accuracy results
            else if let Ok(results) = serde_json::from_str::<Vec<ScanAccuracyResult>>(&content) {
                scan_accuracy_results = results;
            }
        }
    }

    // Sort full SLAM results
    full_slam_results.sort_by(|a, b| {
        let a_key = format!("{:?}_{:?}", a.matcher_type, a.odometry_type);
        let b_key = format!("{:?}_{:?}", b.matcher_type, b.odometry_type);
        a_key.cmp(&b_key)
    });

    // Generate markdown
    let mut md = String::new();

    md.push_str("# SLAM Benchmark Results\n\n");

    // Scan accuracy section
    if !scan_accuracy_results.is_empty() {
        md.push_str("## Scan Matching Accuracy\n\n");
        md.push_str("Tests random scan pairs against odometry ground truth.\n\n");
        md.push_str("| Algorithm | Conv% | Trans Error | Rot Error | Time |\n");
        md.push_str("|-----------|-------|-------------|-----------|------|\n");

        for r in &scan_accuracy_results {
            md.push_str(&format!(
                "| {} | {:.1}% | {:.1}/{:.1}cm | {:.1}/{:.1}° | {:.2}ms |\n",
                r.matcher_name,
                r.convergence_rate,
                r.mean_trans_error_cm,
                r.median_trans_error_cm,
                r.mean_rot_error_deg,
                r.median_rot_error_deg,
                r.avg_time_ms,
            ));
        }
        md.push('\n');
    }

    // Full SLAM section
    if !full_slam_results.is_empty() {
        md.push_str("## Full SLAM Pipeline\n\n");
        md.push_str("| Matcher | Odometry | Time/Scan | Coherence | Density | Area | Compact |\n");
        md.push_str("|---------|----------|-----------|-----------|---------|------|--------|\n");

        for r in &full_slam_results {
            md.push_str(&format!(
                "| {} | {} | {:.2}ms | {:.1}% | {:.0} | {:.1}m² | {:.2} |\n",
                r.matcher_type,
                r.odometry_type,
                r.avg_scan_time_ms,
                r.map_noise.wall_coherence * 100.0,
                r.map_noise.occupied_density,
                r.map_noise.map_area_m2,
                r.map_noise.wall_compactness,
            ));
        }
        md.push('\n');

        // Recommendations
        if let Some(best_speed) = full_slam_results
            .iter()
            .min_by(|a, b| a.avg_scan_time_ms.partial_cmp(&b.avg_scan_time_ms).unwrap())
        {
            md.push_str(&format!(
                "**Fastest**: {} + {} ({:.2}ms/scan)\n\n",
                best_speed.matcher_type, best_speed.odometry_type, best_speed.avg_scan_time_ms
            ));
        }

        if let Some(best_quality) = full_slam_results.iter().max_by(|a, b| {
            a.map_noise
                .wall_coherence
                .partial_cmp(&b.map_noise.wall_coherence)
                .unwrap()
        }) {
            md.push_str(&format!(
                "**Best Coherence**: {} + {} ({:.1}%)\n\n",
                best_quality.matcher_type,
                best_quality.odometry_type,
                best_quality.map_noise.wall_coherence * 100.0
            ));
        }

        // Best density (higher is better - less drift)
        if let Some(best_density) = full_slam_results.iter().max_by(|a, b| {
            a.map_noise
                .occupied_density
                .partial_cmp(&b.map_noise.occupied_density)
                .unwrap()
        }) {
            md.push_str(&format!(
                "**Best Density** (least drift): {} + {} ({:.0} cells/m², {:.1}m² area)\n",
                best_density.matcher_type,
                best_density.odometry_type,
                best_density.map_noise.occupied_density,
                best_density.map_noise.map_area_m2
            ));
        }
    }

    fs::write(output_file, md)?;
    println!("Summary written to: {}", output_file);

    Ok(())
}
