//! SLAM algorithm benchmarking tool.
//!
//! Runs SLAM with different algorithm combinations on recorded bag files
//! and measures performance and map quality metrics.
//!
//! # Usage
//!
//! ```bash
//! # Run single benchmark
//! slam-benchmark --bag recordings/square.bag --matcher p2l --odometry complementary
//!
//! # Run all 12 algorithm combinations
//! slam-benchmark --run-all --bag recordings/square.bag --output results/
//!
//! # Generate markdown summary
//! slam-benchmark summarize --input results/ --output summary.md
//! ```

use std::fs::{self, File};
use std::io::Write;
use std::path::Path;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand, ValueEnum};
use serde::{Deserialize, Serialize};
use sysinfo::System;

use dhruva_slam::algorithms::mapping::{
    MapIntegrator, MapIntegratorConfig, OccupancyGrid, OccupancyGridConfig,
};
use dhruva_slam::algorithms::matching::{
    CorrelativeConfig, CorrelativeMatcher, HybridMatcher, HybridMatcherConfig, HybridP2LMatcher,
    HybridP2LMatcherConfig, IcpConfig, MultiResolutionConfig, MultiResolutionMatcher,
    PointToLineIcp, PointToLineIcpConfig, PointToPointIcp, ScanMatchResult, ScanMatcher,
};
use dhruva_slam::core::types::{LaserScan, PointCloud2D, Pose2D};
use dhruva_slam::io::bag::{BagMessage, BagPlayer};
use dhruva_slam::metrics::{MapNoiseMetrics, analyze_map_noise};
use dhruva_slam::sensors::odometry::{
    ComplementaryConfig, ComplementaryFilter, Eskf, EskfConfig, WheelOdometry, WheelOdometryConfig,
};
use dhruva_slam::sensors::preprocessing::{PreprocessorConfig, ScanPreprocessor};

#[derive(Parser)]
#[command(name = "slam-benchmark")]
#[command(about = "Benchmark SLAM algorithm combinations")]
struct Args {
    #[command(subcommand)]
    command: Option<Commands>,

    /// Input bag file
    #[arg(short, long)]
    bag: Option<String>,

    /// Scan matcher type
    #[arg(long, value_enum, default_value = "hybrid-p2l")]
    matcher: MatcherType,

    /// Odometry filter type
    #[arg(long, value_enum, default_value = "complementary")]
    odometry: OdometryType,

    /// Output directory for results
    #[arg(short, long, default_value = "results")]
    output: String,

    /// Run all 12 algorithm combinations
    #[arg(long)]
    run_all: bool,
}

#[derive(Subcommand)]
enum Commands {
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

#[derive(Clone, Copy, ValueEnum, Debug, Serialize, Deserialize)]
pub enum MatcherType {
    /// Point-to-Point ICP only
    Icp,
    /// Point-to-Line ICP only
    P2l,
    /// Correlative matcher only
    Correlative,
    /// Multi-resolution correlative matcher
    MultiRes,
    /// Hybrid: Correlative + Point-to-Point ICP
    HybridIcp,
    /// Hybrid: Correlative + Point-to-Line ICP (recommended)
    HybridP2l,
}

#[derive(Clone, Copy, ValueEnum, Debug, Serialize, Deserialize)]
pub enum OdometryType {
    /// Wheel odometry only (encoder ticks)
    Wheel,
    /// Complementary filter (encoder + gyro fusion)
    Complementary,
    /// Error-State Kalman Filter (full IMU fusion)
    Eskf,
}

impl std::fmt::Display for MatcherType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MatcherType::Icp => write!(f, "P2P ICP"),
            MatcherType::P2l => write!(f, "P2L ICP"),
            MatcherType::Correlative => write!(f, "Correlative"),
            MatcherType::MultiRes => write!(f, "Multi-Res"),
            MatcherType::HybridIcp => write!(f, "Hybrid ICP"),
            MatcherType::HybridP2l => write!(f, "Hybrid P2L"),
        }
    }
}

impl std::fmt::Display for OdometryType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OdometryType::Wheel => write!(f, "Wheel"),
            OdometryType::Complementary => write!(f, "Complementary"),
            OdometryType::Eskf => write!(f, "ESKF"),
        }
    }
}

/// Benchmark result for a single algorithm combination.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkResult {
    /// Name of the bag file used
    pub bag_file: String,

    /// Scan matcher type
    pub matcher_type: MatcherType,

    /// Odometry filter type
    pub odometry_type: OdometryType,

    // Performance metrics
    pub total_time_ms: f64,
    pub scans_processed: u64,
    pub avg_scan_time_ms: f64,
    pub peak_memory_mb: f64,

    // Map quality metrics
    pub map_noise: MapNoiseMetrics,

    // Final pose (for loop closure scenarios)
    pub final_pose: (f64, f64, f64),

    // Match statistics
    pub avg_match_score: f64,
    pub failed_matches: u64,
}

fn main() {
    env_logger::init();

    let args = Args::parse();

    if let Err(e) = run(args) {
        eprintln!("Error: {}", e);
        std::process::exit(1);
    }
}

fn run(args: Args) -> Result<(), Box<dyn std::error::Error>> {
    // Handle summarize subcommand
    if let Some(Commands::Summarize { input, output }) = args.command {
        return generate_summary(&input, &output);
    }

    // Require bag file for benchmarking
    let bag_path = args.bag.ok_or("Missing --bag argument")?;

    // Create output directory
    fs::create_dir_all(&args.output)?;

    if args.run_all {
        run_all_combinations(&bag_path, &args.output)?;
    } else {
        let result = run_benchmark(&bag_path, args.matcher, args.odometry)?;
        save_result(&result, &args.output)?;
        print_result(&result);
    }

    Ok(())
}

/// Run all 12 algorithm combinations.
fn run_all_combinations(
    bag_path: &str,
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {
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
    ];

    let total = matchers.len() * odometries.len();
    let mut count = 0;

    println!("=== SLAM Benchmark Suite ===");
    println!("Bag file: {}", bag_path);
    println!("Running {} algorithm combinations...\n", total);

    for matcher in &matchers {
        for odometry in &odometries {
            count += 1;
            println!("[{}/{}] {} + {}...", count, total, matcher, odometry);

            match run_benchmark(bag_path, *matcher, *odometry) {
                Ok(result) => {
                    save_result(&result, output_dir)?;
                    println!(
                        "  -> {:.1}ms/scan, coherence: {:.1}%, isolated: {}",
                        result.avg_scan_time_ms,
                        result.map_noise.wall_coherence * 100.0,
                        result.map_noise.isolated_cells
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

/// Run a single benchmark with specified algorithm combination.
fn run_benchmark(
    bag_path: &str,
    matcher_type: MatcherType,
    odometry_type: OdometryType,
) -> Result<BenchmarkResult, Box<dyn std::error::Error>> {
    // Open bag file
    let mut player = BagPlayer::open(bag_path)?;
    player.set_speed(0.0); // Maximum speed (no timing)

    // Initialize components
    let mut odometry = create_odometry(odometry_type);
    let mut matcher = create_matcher(matcher_type);
    let preprocessor = ScanPreprocessor::new(PreprocessorConfig::default());
    let mut map = OccupancyGrid::new(OccupancyGridConfig::default());
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // State tracking
    let mut current_pose = Pose2D::identity();
    let mut previous_scan: Option<PointCloud2D> = None;
    let mut previous_odom_pose = Pose2D::identity(); // Pose at previous scan
    let mut scans_processed = 0u64;
    let mut failed_matches = 0u64;
    let mut total_score = 0.0f64;

    // Timing
    let mut total_scan_time = Duration::ZERO;

    // Memory tracking
    let mut sys = System::new();
    let mut peak_memory_mb = 0.0f64;
    let pid = sysinfo::get_current_pid().ok();

    let start_time = Instant::now();

    // Process bag messages
    while let Ok(Some(msg)) = player.next_immediate() {
        match msg {
            BagMessage::SensorStatus(status) => {
                // Update odometry with raw encoder ticks
                if let Some(pose) = odometry.update(
                    status.encoder.left,
                    status.encoder.right,
                    status.gyro_raw[0], // Yaw is in gyro_x on CRL-200S
                    status.timestamp_us,
                ) {
                    current_pose = pose;
                }
            }
            BagMessage::Lidar(timestamped) => {
                let scan_start = Instant::now();

                // Convert to LaserScan for preprocessing
                let laser_scan = LaserScan::from_lidar_scan(&timestamped.data);

                // Preprocess scan to PointCloud2D
                let processed = preprocessor.process(&laser_scan);

                if processed.len() >= 50 {
                    // Scan matching
                    if let Some(ref prev_scan) = previous_scan {
                        // Compute odometry delta as initial guess for scan matching.
                        // The initial guess is the transform from source (current) to target (previous).
                        // Since odom_delta = previous_pose.inverse().compose(current_pose) represents
                        // robot motion from previous to current, we need its inverse.
                        let odom_delta = previous_odom_pose.inverse().compose(&current_pose);
                        let initial_guess = odom_delta.inverse();

                        let result = matcher.match_scans(&processed, prev_scan, &initial_guess);

                        if result.converged {
                            // Apply scan match result: the transform maps current → previous,
                            // so to get the new pose we compose previous_pose with the inverse.
                            current_pose = previous_odom_pose.compose(&result.transform.inverse());
                            total_score += result.score as f64;
                        } else {
                            failed_matches += 1;
                            // On failure, keep using odometry-derived current_pose
                        }
                    }

                    // Transform scan to global frame
                    let global_scan = processed.transform(&current_pose);

                    // Integrate into map
                    integrator.integrate_cloud(&mut map, &global_scan, &current_pose);

                    // Update for next iteration
                    previous_scan = Some(processed);
                    previous_odom_pose = current_pose; // Track pose at this scan for next delta
                    scans_processed += 1;
                }

                total_scan_time += scan_start.elapsed();

                // Periodic memory check (every 10 scans)
                if scans_processed % 10 == 0 {
                    sys.refresh_all();
                    if let Some(pid) = pid
                        && let Some(process) = sys.process(pid)
                    {
                        let mem_mb = process.memory() as f64 / (1024.0 * 1024.0);
                        peak_memory_mb = peak_memory_mb.max(mem_mb);
                    }
                }
            }
            BagMessage::Odometry(_) => {
                // Ground truth odometry - not used in this benchmark
            }
        }
    }

    let total_time = start_time.elapsed();

    // Compute map quality metrics
    let map_noise = analyze_map_noise(&map);

    // Build result
    let avg_scan_time_ms = if scans_processed > 0 {
        total_scan_time.as_secs_f64() * 1000.0 / scans_processed as f64
    } else {
        0.0
    };

    let avg_match_score = if scans_processed > failed_matches && scans_processed > 0 {
        total_score / (scans_processed - failed_matches) as f64
    } else {
        0.0
    };

    Ok(BenchmarkResult {
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
        final_pose: (
            current_pose.x as f64,
            current_pose.y as f64,
            current_pose.theta as f64,
        ),
        avg_match_score,
        failed_matches,
    })
}

/// Trait object wrapper for different matcher types.
enum DynMatcher {
    Icp(PointToPointIcp),
    P2l(PointToLineIcp),
    Correlative(CorrelativeMatcher),
    MultiRes(MultiResolutionMatcher),
    HybridIcp(HybridMatcher),
    HybridP2l(HybridP2LMatcher),
}

impl DynMatcher {
    fn match_scans(
        &mut self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        match self {
            DynMatcher::Icp(m) => m.match_scans(source, target, initial_guess),
            DynMatcher::P2l(m) => m.match_scans(source, target, initial_guess),
            DynMatcher::Correlative(m) => m.match_scans(source, target, initial_guess),
            DynMatcher::MultiRes(m) => m.match_scans(source, target, initial_guess),
            DynMatcher::HybridIcp(m) => m.match_scans(source, target, initial_guess),
            DynMatcher::HybridP2l(m) => m.match_scans(source, target, initial_guess),
        }
    }
}

fn create_matcher(matcher_type: MatcherType) -> DynMatcher {
    match matcher_type {
        MatcherType::Icp => DynMatcher::Icp(PointToPointIcp::new(IcpConfig::default())),
        MatcherType::P2l => DynMatcher::P2l(PointToLineIcp::new(PointToLineIcpConfig::default())),
        MatcherType::Correlative => {
            DynMatcher::Correlative(CorrelativeMatcher::new(CorrelativeConfig::default()))
        }
        MatcherType::MultiRes => {
            DynMatcher::MultiRes(MultiResolutionMatcher::new(MultiResolutionConfig::default()))
        }
        MatcherType::HybridIcp => {
            DynMatcher::HybridIcp(HybridMatcher::new(HybridMatcherConfig::default()))
        }
        MatcherType::HybridP2l => {
            DynMatcher::HybridP2l(HybridP2LMatcher::new(HybridP2LMatcherConfig::default()))
        }
    }
}

/// Trait object wrapper for different odometry types.
enum DynOdometry {
    Wheel(WheelOdometryWrapper),
    Complementary(ComplementaryOdometry),
    Eskf(EskfOdometry),
}

impl DynOdometry {
    fn update(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
    ) -> Option<Pose2D> {
        match self {
            DynOdometry::Wheel(o) => o.update(left, right),
            DynOdometry::Complementary(o) => o.update(left, right, gyro_yaw, timestamp_us),
            DynOdometry::Eskf(o) => o.update(left, right, gyro_yaw, timestamp_us),
        }
    }
}

/// Wrapper for wheel odometry that accumulates pose.
struct WheelOdometryWrapper {
    wheel: WheelOdometry,
    pose: Pose2D,
}

impl WheelOdometryWrapper {
    fn new() -> Self {
        Self {
            wheel: WheelOdometry::new(WheelOdometryConfig::default()),
            pose: Pose2D::identity(),
        }
    }

    fn update(&mut self, left: u16, right: u16) -> Option<Pose2D> {
        if let Some(delta) = self.wheel.update(left, right) {
            self.pose = self.pose.compose(&delta);
            Some(self.pose)
        } else {
            None
        }
    }
}

/// Wrapper for complementary filter odometry.
struct ComplementaryOdometry {
    wheel: WheelOdometry,
    filter: ComplementaryFilter,
    pose: Pose2D,
    last_timestamp_us: Option<u64>,
}

impl ComplementaryOdometry {
    fn new() -> Self {
        Self {
            wheel: WheelOdometry::new(WheelOdometryConfig::default()),
            filter: ComplementaryFilter::new(ComplementaryConfig::default()),
            pose: Pose2D::identity(),
            last_timestamp_us: None,
        }
    }

    fn update(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
    ) -> Option<Pose2D> {
        if let Some(encoder_delta) = self.wheel.update(left, right) {
            // Use complementary filter to fuse encoder + gyro
            let fused = self.filter.update(encoder_delta, gyro_yaw, timestamp_us);
            self.pose = self.pose.compose(&fused);
            self.last_timestamp_us = Some(timestamp_us);
            Some(self.pose)
        } else {
            self.last_timestamp_us = Some(timestamp_us);
            None
        }
    }
}

/// Wrapper for ESKF odometry.
struct EskfOdometry {
    wheel: WheelOdometry,
    eskf: Eskf,
    pose: Pose2D,
    last_timestamp_us: Option<u64>,
}

impl EskfOdometry {
    fn new() -> Self {
        Self {
            wheel: WheelOdometry::new(WheelOdometryConfig::default()),
            eskf: Eskf::new(EskfConfig::default()),
            pose: Pose2D::identity(),
            last_timestamp_us: None,
        }
    }

    fn update(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
    ) -> Option<Pose2D> {
        if let Some(encoder_delta) = self.wheel.update(left, right) {
            // Use ESKF to fuse encoder + gyro
            let fused = self.eskf.update(encoder_delta, gyro_yaw, timestamp_us);
            self.pose = self.pose.compose(&fused);
            self.last_timestamp_us = Some(timestamp_us);
            Some(self.pose)
        } else {
            self.last_timestamp_us = Some(timestamp_us);
            None
        }
    }
}

fn create_odometry(odometry_type: OdometryType) -> DynOdometry {
    match odometry_type {
        OdometryType::Wheel => DynOdometry::Wheel(WheelOdometryWrapper::new()),
        OdometryType::Complementary => DynOdometry::Complementary(ComplementaryOdometry::new()),
        OdometryType::Eskf => DynOdometry::Eskf(EskfOdometry::new()),
    }
}

fn save_result(
    result: &BenchmarkResult,
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
    let mut file = File::create(&path)?;
    file.write_all(json.as_bytes())?;

    Ok(())
}

fn print_result(result: &BenchmarkResult) {
    println!("\n=== Benchmark Result ===");
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
        "  Avg neighbors: {:.2}",
        result.map_noise.avg_occupied_neighbors
    );
    println!();
    println!("Matching:");
    println!("  Avg match score: {:.3}", result.avg_match_score);
    println!("  Failed matches: {}", result.failed_matches);
    println!();
    println!(
        "Final Pose: ({:.3}, {:.3}, {:.1}deg)",
        result.final_pose.0,
        result.final_pose.1,
        result.final_pose.2.to_degrees()
    );
}

fn generate_summary(input_dir: &str, output_file: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("Generating summary from: {}", input_dir);

    // Load all result files
    let mut results: Vec<BenchmarkResult> = Vec::new();

    for entry in fs::read_dir(input_dir)? {
        let entry = entry?;
        let path = entry.path();

        if path.extension().map(|e| e == "json").unwrap_or(false) {
            let content = fs::read_to_string(&path)?;
            if let Ok(result) = serde_json::from_str::<BenchmarkResult>(&content) {
                results.push(result);
            }
        }
    }

    if results.is_empty() {
        return Err("No result files found".into());
    }

    // Sort by matcher then odometry
    results.sort_by(|a, b| {
        let a_key = format!("{:?}_{:?}", a.matcher_type, a.odometry_type);
        let b_key = format!("{:?}_{:?}", b.matcher_type, b.odometry_type);
        a_key.cmp(&b_key)
    });

    // Generate markdown
    let mut md = String::new();

    md.push_str("## Empirical Benchmark Results\n\n");

    // Environment info
    md.push_str("### Test Environment\n\n");
    md.push_str("- **Platform**: ARM Allwinner A33 (CRL-200S Robot)\n");
    md.push_str("- **Lidar**: Delta-2D, 360 points @ 5Hz\n");
    md.push_str("- **Odometry**: 500Hz encoder + gyro\n\n");

    // Results table
    md.push_str("### Algorithm Comparison\n\n");
    md.push_str("| Matcher | Odometry | Time/Scan (ms) | Memory (MB) | Wall Coherence | Isolated | Avg Score |\n");
    md.push_str("|---------|----------|----------------|-------------|----------------|----------|----------|\n");

    for r in &results {
        md.push_str(&format!(
            "| {} | {} | {:.2} | {:.1} | {:.1}% | {} | {:.3} |\n",
            r.matcher_type,
            r.odometry_type,
            r.avg_scan_time_ms,
            r.peak_memory_mb,
            r.map_noise.wall_coherence * 100.0,
            r.map_noise.isolated_cells,
            r.avg_match_score,
        ));
    }

    md.push('\n');

    // Find best performers
    let best_speed = results
        .iter()
        .min_by(|a, b| a.avg_scan_time_ms.partial_cmp(&b.avg_scan_time_ms).unwrap());

    let best_quality = results.iter().max_by(|a, b| {
        a.map_noise
            .wall_coherence
            .partial_cmp(&b.map_noise.wall_coherence)
            .unwrap()
    });

    md.push_str("### Recommendations\n\n");

    if let Some(r) = best_speed {
        md.push_str(&format!(
            "- **Best Performance**: {} + {} ({:.2} ms/scan)\n",
            r.matcher_type, r.odometry_type, r.avg_scan_time_ms
        ));
    }

    if let Some(r) = best_quality {
        md.push_str(&format!(
            "- **Best Map Quality**: {} + {} ({:.1}% coherence)\n",
            r.matcher_type,
            r.odometry_type,
            r.map_noise.wall_coherence * 100.0
        ));
    }

    // Write output
    let mut file = File::create(output_file)?;
    file.write_all(md.as_bytes())?;

    println!("Summary written to: {}", output_file);

    Ok(())
}
