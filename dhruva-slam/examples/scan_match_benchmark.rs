//! Scan Matching Algorithm Benchmark Tool
//!
//! Tests all scan matching algorithms using random scan pairs from a bag file
//! to compare accuracy against odometry ground truth.
//!
//! # New Benchmarking Approach
//!
//! Instead of comparing consecutive scans (which are too similar for meaningful testing),
//! this benchmark:
//! 1. Picks random scan pairs with variable separation (offset)
//! 2. Uses accumulated odometry as ground truth
//! 3. Measures how close the algorithm's transform estimate is to odometry
//!
//! This stress-tests algorithms with realistic pose differences and provides
//! meaningful accuracy metrics.
//!
//! # Algorithms Tested
//!
//! 1. Point-to-Point ICP (Legacy simple)
//! 2. Point-to-Point ICP (Robust with Welsch kernel)
//! 3. Point-to-Line ICP
//! 4. Correlative Scan Matcher
//! 5. Multi-Resolution Correlative Matcher
//! 6. Hybrid Matcher (Correlative + P2P ICP)
//! 7. Hybrid P2L Matcher (Correlative + P2L ICP)
//!
//! # Usage
//!
//! ```bash
//! # Random pair benchmark (100 tests, offsets 1-30)
//! cargo run --release --bin scan-match-benchmark -- bags/forward_1min.bag \
//!     --num-tests 100 --min-offset 1 --max-offset 30 --use-fused
//!
//! # Reproducible run with seed
//! cargo run --release --bin scan-match-benchmark -- bags/rotation_2min.bag \
//!     --num-tests 200 --seed 42 --use-fused
//! ```

use dhruva_slam::utils::{GYRO_SCALE, WHEEL_BASE, WHEEL_TICKS_PER_METER};
use std::fs::File;
use std::io::Write as IoWrite;
use std::path::Path;
use std::time::Instant;

use clap::Parser;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use serde::{Deserialize, Serialize};

use dhruva_slam::algorithms::matching::{
    CorrelativeConfig, CorrelativeMatcher, HybridMatcher, HybridMatcherConfig, HybridP2LMatcher,
    HybridP2LMatcherConfig, IcpConfig, MultiResolutionConfig, MultiResolutionMatcher,
    PointToLineIcp, PointToLineIcpConfig, PointToPointIcp, RobustKernel, ScanMatcher,
};
use dhruva_slam::core::types::{LaserScan, PointCloud2D, Pose2D};
use dhruva_slam::io::bag::{BagMessage, BagPlayer};
use dhruva_slam::metrics::{TransformError, TransformErrorStats, compute_transform_error};
use dhruva_slam::sensors::odometry::{WheelOdometry, WheelOdometryConfig};
use dhruva_slam::sensors::preprocessing::{PreprocessorConfig, ScanPreprocessor};

#[derive(Parser)]
#[command(name = "scan-match-benchmark")]
#[command(about = "Benchmark scan matching algorithms using random pairs vs odometry ground truth")]
struct Args {
    /// Input bag file
    bag: String,

    /// Number of random test pairs to evaluate
    #[arg(long, default_value = "100")]
    num_tests: usize,

    /// Minimum scan separation (offset) for test pairs
    #[arg(long, default_value = "1")]
    min_offset: usize,

    /// Maximum scan separation (offset) for test pairs
    #[arg(long, default_value = "30")]
    max_offset: usize,

    /// Random seed for reproducible results (optional)
    #[arg(long)]
    seed: Option<u64>,

    /// Skip first N scans (to skip initial warmup)
    #[arg(long, default_value = "0")]
    skip: usize,

    /// Use minimal preprocessing (range filter only, no outlier filter or downsampling)
    #[arg(long)]
    minimal: bool,

    /// Use fused encoder + gyro for initial guess (complementary filter)
    /// Gyro provides rotation, encoder provides translation
    #[arg(long)]
    use_fused: bool,

    /// Gyro weight for complementary filter (0.0 = encoder only, 1.0 = gyro only)
    #[arg(long, default_value = "0.8")]
    gyro_weight: f32,

    /// Translation error threshold for convergence (meters)
    #[arg(long, default_value = "0.05")]
    trans_threshold: f32,

    /// Rotation error threshold for convergence (radians, ~3 degrees default)
    #[arg(long, default_value = "0.05")]
    rot_threshold: f32,

    /// Export results to JSON file
    #[arg(long, value_name = "FILE")]
    output_json: Option<String>,

    /// Export results to CSV file
    #[arg(long, value_name = "FILE")]
    output_csv: Option<String>,

    /// Suppress stdout table output (useful with --output-json/csv)
    #[arg(long)]
    quiet: bool,
}

// ============================================================================
// Serializable Data Structures for Export
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BenchmarkMetadata {
    bag_file: String,
    num_tests: usize,
    min_offset: usize,
    max_offset: usize,
    seed: Option<u64>,
    skip_count: usize,
    preprocessing_mode: String,
    initial_guess_source: String,
    trans_threshold_m: f32,
    rot_threshold_rad: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct AlgorithmSummary {
    name: String,
    convergence_rate: f32,
    mean_trans_error_cm: f32,
    median_trans_error_cm: f32,
    mean_rot_error_deg: f32,
    median_rot_error_deg: f32,
    avg_time_ms: f32,
    total_tests: usize,
    converged_count: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct OffsetBucketStats {
    offset_range: String,
    mean_trans_error_cm: f32,
    mean_rot_error_deg: f32,
    count: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BenchmarkReport {
    metadata: BenchmarkMetadata,
    algorithms: Vec<AlgorithmSummary>,
    offset_analysis: Vec<Vec<OffsetBucketStats>>, // Per-algorithm offset buckets
}

// ============================================================================
// Algorithm Statistics with Transform Error Tracking
// ============================================================================

struct AlgorithmBenchmark {
    name: String,
    error_stats: TransformErrorStats,
    time_sum_us: u128,
    trans_threshold: f32,
    rot_threshold: f32,
}

impl AlgorithmBenchmark {
    fn new(name: &str, trans_threshold: f32, rot_threshold: f32) -> Self {
        Self {
            name: name.to_string(),
            error_stats: TransformErrorStats::new(),
            time_sum_us: 0,
            trans_threshold,
            rot_threshold,
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

    fn to_summary(&self) -> AlgorithmSummary {
        AlgorithmSummary {
            name: self.name.clone(),
            convergence_rate: self.error_stats.convergence_rate(),
            mean_trans_error_cm: self.error_stats.mean_translation_error() * 100.0,
            median_trans_error_cm: self.error_stats.median_translation_error() * 100.0,
            mean_rot_error_deg: self.error_stats.mean_rotation_error().to_degrees(),
            median_rot_error_deg: self.error_stats.median_rotation_error().to_degrees(),
            avg_time_ms: self.avg_time_ms(),
            total_tests: self.error_stats.total_pairs(),
            converged_count: self.error_stats.converged_count,
        }
    }

    fn offset_bucket_stats(&self, bucket_size: usize) -> Vec<OffsetBucketStats> {
        self.error_stats
            .errors_by_offset_bucket(bucket_size)
            .into_iter()
            .map(|(range, mean_trans, mean_rot, count)| OffsetBucketStats {
                offset_range: format!("{}-{}", range.start, range.end - 1),
                mean_trans_error_cm: mean_trans * 100.0,
                mean_rot_error_deg: mean_rot.to_degrees(),
                count,
            })
            .collect()
    }
}

// ============================================================================
// Random Pair Generation
// ============================================================================

/// Generate random (base_idx, target_idx) pairs for testing.
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
            let target_idx = base_idx + offset;
            (base_idx, target_idx)
        })
        .collect()
}

/// Accumulate odometry transforms between two scan indices.
///
/// The predictions array contains the transform from scan[i] to scan[i+1].
/// So accumulating from start to end gives the total transform.
fn accumulate_odometry(predictions: &[Pose2D], start: usize, end: usize) -> Pose2D {
    predictions[start..end]
        .iter()
        .fold(Pose2D::identity(), |acc, delta| acc.compose(delta))
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    let args = Args::parse();

    let bag_path = Path::new(&args.bag);
    if !bag_path.exists() {
        eprintln!("Error: Bag file not found: {}", args.bag);
        std::process::exit(1);
    }

    if !args.quiet {
        println!("╔══════════════════════════════════════════════════════════════════╗");
        println!("║       SCAN MATCHING ALGORITHM BENCHMARK                          ║");
        println!("╚══════════════════════════════════════════════════════════════════╝");
        println!();
        println!("Bag file: {}", args.bag);
        println!(
            "Test pairs: {} random pairs (offset range: {}-{} scans)",
            args.num_tests, args.min_offset, args.max_offset
        );
        if let Some(seed) = args.seed {
            println!("Random seed: {} (reproducible)", seed);
        }
        if args.minimal {
            println!("Mode: MINIMAL preprocessing (range filter only)");
        } else {
            println!("Mode: FULL preprocessing (range + outlier + downsampling)");
        }
        if args.use_fused {
            println!(
                "Initial guess: FUSED (encoder translation + gyro rotation, weight={:.1})",
                args.gyro_weight
            );
        } else {
            println!("Initial guess: IDENTITY (no motion prediction)");
        }
        println!("Ground truth: Fused encoder + gyro odometry");
        println!(
            "Convergence thresholds: trans < {:.1}cm, rot < {:.1}°",
            args.trans_threshold * 100.0,
            args.rot_threshold.to_degrees()
        );
        println!();
    }

    // Load bag file
    let mut player = BagPlayer::open(bag_path).expect("Failed to open bag file");
    let preprocessor = ScanPreprocessor::new(PreprocessorConfig::default());

    // Wheel odometry for computing ground truth
    let odom_config = WheelOdometryConfig {
        ticks_per_meter: WHEEL_TICKS_PER_METER,
        wheel_base: WHEEL_BASE,
    };
    let mut wheel_odom = WheelOdometry::new(odom_config);

    // Collect scans and fused odometry predictions between consecutive scans
    let mut scans: Vec<PointCloud2D> = Vec::new();
    let mut fused_predictions: Vec<Pose2D> = Vec::new();
    let mut accumulated_odom = Pose2D::identity();
    let use_minimal = args.minimal;
    let gyro_weight = args.gyro_weight;

    // Gyro integration state
    let mut integrated_gyro_theta: f32 = 0.0;
    let mut last_sensor_timestamp_us: Option<u64> = None;

    while let Ok(Some(msg)) = player.next_immediate() {
        match msg {
            BagMessage::SensorStatus(status) => {
                // Update wheel odometry
                if let Some(delta) = wheel_odom.update(status.encoder.left, status.encoder.right) {
                    accumulated_odom = accumulated_odom.compose(&delta);
                }

                // Integrate gyroscope for rotation
                if let Some(last_ts) = last_sensor_timestamp_us
                    && status.timestamp_us > last_ts
                {
                    let dt_s = (status.timestamp_us - last_ts) as f32 / 1_000_000.0;
                    let gyro_x_rad_s = -status.gyro_raw[0] as f32 * GYRO_SCALE;
                    integrated_gyro_theta += gyro_x_rad_s * dt_s;
                }
                last_sensor_timestamp_us = Some(status.timestamp_us);
            }
            BagMessage::Lidar(timestamped) => {
                let laser_scan = LaserScan::from_lidar_scan(&timestamped.data);
                let processed = if use_minimal {
                    preprocessor.process_minimal(&laser_scan)
                } else {
                    preprocessor.process(&laser_scan)
                };

                if processed.len() >= 50 {
                    if !scans.is_empty() {
                        // Create fused prediction: encoder translation + weighted gyro rotation
                        let fused_theta = gyro_weight * integrated_gyro_theta
                            + (1.0 - gyro_weight) * accumulated_odom.theta;
                        let fused_pose =
                            Pose2D::new(accumulated_odom.x, accumulated_odom.y, fused_theta);
                        fused_predictions.push(fused_pose);
                    } else {
                        wheel_odom.reset();
                    }
                    scans.push(processed);
                    accumulated_odom = Pose2D::identity();
                    integrated_gyro_theta = 0.0;
                }
            }
            _ => {}
        }
    }

    // Skip initial scans
    if args.skip > 0 && args.skip < scans.len() {
        scans = scans.split_off(args.skip);
        fused_predictions = fused_predictions.split_off(args.skip.min(fused_predictions.len()));
    }

    if !args.quiet {
        println!("Loaded {} scans from bag file", scans.len());
        println!("Computed {} odometry predictions", fused_predictions.len());
    }

    if scans.len() < args.min_offset + 2 {
        eprintln!("Not enough scans in bag file for requested offset range");
        std::process::exit(1);
    }

    // Generate random test pairs
    let test_pairs = generate_random_pairs(
        scans.len(),
        args.num_tests,
        args.min_offset,
        args.max_offset,
        args.seed,
    );

    if test_pairs.is_empty() {
        eprintln!("Could not generate any valid test pairs");
        std::process::exit(1);
    }

    if !args.quiet {
        println!("Generated {} random test pairs", test_pairs.len());
        println!();
        println!("Running benchmark...");
    }

    // ========================================================================
    // Initialize ALL scan matchers (7 algorithms)
    // ========================================================================

    // 1. Legacy ICP (simple, no robust features)
    let legacy_config = IcpConfig {
        max_iterations: 50,
        translation_epsilon: 0.001,
        rotation_epsilon: 0.001,
        max_correspondence_distance: 0.5,
        min_correspondences: 10,
        outlier_ratio: 0.1,
        robust_kernel: RobustKernel::None,
        kernel_scale: 0.10,
        bidirectional_check: false,
        damping_factor: 1.0,
    };
    let mut legacy_icp = PointToPointIcp::new(legacy_config);

    // 2. Robust ICP (Welsch kernel + bidirectional + damping)
    let robust_config = IcpConfig {
        max_iterations: 50,
        translation_epsilon: 0.001,
        rotation_epsilon: 0.001,
        max_correspondence_distance: 0.5,
        min_correspondences: 10,
        outlier_ratio: 0.1,
        robust_kernel: RobustKernel::Welsch,
        kernel_scale: 0.10,
        bidirectional_check: true,
        damping_factor: 0.8,
    };
    let mut robust_icp = PointToPointIcp::new(robust_config);

    // 3. Point-to-Line ICP
    let p2l_config = PointToLineIcpConfig {
        max_iterations: 50,
        translation_epsilon: 0.001,
        rotation_epsilon: 0.001,
        max_correspondence_distance: 0.5,
        min_correspondences: 10,
        outlier_ratio: 0.1,
        line_neighbors: 5,
        min_line_quality: 0.8,
    };
    let mut p2l_icp = PointToLineIcp::new(p2l_config);

    // 4. Correlative Scan Matcher
    let corr_config = CorrelativeConfig {
        search_window_x: 0.1,
        search_window_y: 0.1,
        search_window_theta: 0.1,
        linear_resolution: 0.005,
        angular_resolution: 0.005,
        grid_resolution: 0.02,
        min_score: 0.5,
    };
    let mut correlative = CorrelativeMatcher::new(corr_config);

    // 5. Multi-Resolution Correlative
    let multi_config = MultiResolutionConfig {
        num_levels: 3,
        coarse_search_window_xy: 0.1,
        coarse_search_window_theta: 0.1,
        fine_resolution: 0.005,
        fine_angular_resolution: 0.005,
        fine_grid_resolution: 0.02,
        min_score: 0.4,
        resolution_multiplier: 2.0,
        window_shrink_factor: 2.0,
    };
    let mut multi_res = MultiResolutionMatcher::new(multi_config);

    // 6. Hybrid Matcher (Correlative + P2P ICP)
    let hybrid_config = HybridMatcherConfig::default();
    let mut hybrid_matcher = HybridMatcher::new(hybrid_config);

    // 7. Hybrid P2L Matcher (Correlative + P2L ICP)
    let hybrid_p2l_config = HybridP2LMatcherConfig::default();
    let mut hybrid_p2l_matcher = HybridP2LMatcher::new(hybrid_p2l_config);

    // Initialize benchmark trackers
    let mut legacy_bench = AlgorithmBenchmark::new(
        "Legacy ICP (simple)",
        args.trans_threshold,
        args.rot_threshold,
    );
    let mut robust_bench = AlgorithmBenchmark::new(
        "Robust ICP (Welsch)",
        args.trans_threshold,
        args.rot_threshold,
    );
    let mut p2l_bench = AlgorithmBenchmark::new(
        "Point-to-Line ICP",
        args.trans_threshold,
        args.rot_threshold,
    );
    let mut corr_bench = AlgorithmBenchmark::new(
        "Correlative Matcher",
        args.trans_threshold,
        args.rot_threshold,
    );
    let mut multi_bench =
        AlgorithmBenchmark::new("Multi-Resolution", args.trans_threshold, args.rot_threshold);
    let mut hybrid_bench = AlgorithmBenchmark::new(
        "Hybrid (Corr+P2P)",
        args.trans_threshold,
        args.rot_threshold,
    );
    let mut hybrid_p2l_bench = AlgorithmBenchmark::new(
        "Hybrid (Corr+P2L)",
        args.trans_threshold,
        args.rot_threshold,
    );

    // ========================================================================
    // Run benchmark on all random pairs
    // ========================================================================

    for (idx, &(base_idx, target_idx)) in test_pairs.iter().enumerate() {
        let source = &scans[base_idx];
        let target = &scans[target_idx];
        let offset = target_idx - base_idx;

        // Compute ground truth: accumulated odometry from base to target
        let expected_odom = accumulate_odometry(&fused_predictions, base_idx, target_idx);

        // Initial guess: use odometry if requested, otherwise identity
        let initial_guess = if args.use_fused {
            expected_odom.inverse() // Scan matching finds source->target, odom is target->source
        } else {
            Pose2D::identity()
        };

        // Run each algorithm and compare to ground truth
        // The algorithm returns a transform that aligns source to target
        // The expected transform is the inverse of the robot motion (odometry)

        // 1. Legacy ICP
        let start = Instant::now();
        let result = legacy_icp.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom.inverse());
        legacy_bench.record(error, offset, elapsed);

        // 2. Robust ICP
        let start = Instant::now();
        let result = robust_icp.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom.inverse());
        robust_bench.record(error, offset, elapsed);

        // 3. Point-to-Line ICP
        let start = Instant::now();
        let result = p2l_icp.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom.inverse());
        p2l_bench.record(error, offset, elapsed);

        // 4. Correlative
        let start = Instant::now();
        let result = correlative.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom.inverse());
        corr_bench.record(error, offset, elapsed);

        // 5. Multi-Resolution
        let start = Instant::now();
        let result = multi_res.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom.inverse());
        multi_bench.record(error, offset, elapsed);

        // 6. Hybrid
        let start = Instant::now();
        let result = hybrid_matcher.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom.inverse());
        hybrid_bench.record(error, offset, elapsed);

        // 7. Hybrid P2L
        let start = Instant::now();
        let result = hybrid_p2l_matcher.match_scans(source, target, &initial_guess);
        let elapsed = start.elapsed().as_micros();
        let error = compute_transform_error(&result.transform, &expected_odom.inverse());
        hybrid_p2l_bench.record(error, offset, elapsed);

        // Progress indicator
        if !args.quiet && (idx + 1) % 10 == 0 {
            print!(".");
            std::io::stdout().flush().ok();
        }
    }

    if !args.quiet {
        println!(" done!\n");
    }

    // Collect all benchmarks
    let all_benchmarks = [
        &legacy_bench,
        &robust_bench,
        &p2l_bench,
        &corr_bench,
        &multi_bench,
        &hybrid_bench,
        &hybrid_p2l_bench,
    ];

    // ========================================================================
    // Print summary table (unless quiet mode)
    // ========================================================================

    if !args.quiet {
        println!("╔══════════════════════════════════════════════════════════════════╗");
        println!("║                         RESULTS SUMMARY                          ║");
        println!("╚══════════════════════════════════════════════════════════════════╝");
        println!();

        println!("┌─────────────────────────┬────────┬─────────────┬─────────────┬──────────┐");
        println!("│ Algorithm               │ Conv%  │ Trans Error │ Rot Error   │ Avg Time │");
        println!("│                         │        │ mean / med  │ mean / med  │          │");
        println!("├─────────────────────────┼────────┼─────────────┼─────────────┼──────────┤");

        for bench in &all_benchmarks {
            print_stats(bench);
        }

        println!("└─────────────────────────┴────────┴─────────────┴─────────────┴──────────┘");

        println!();
        println!("Legend:");
        println!("  Conv%      = Convergence rate (error within threshold, higher is better)");
        println!("  Trans Error = Translation error in cm (mean / median, lower is better)");
        println!("  Rot Error  = Rotation error in degrees (mean / median, lower is better)");
        println!("  Avg Time   = Average time per match in ms (lower is better)");
        println!();

        // Analysis
        let best_accuracy = all_benchmarks
            .iter()
            .filter(|b| b.error_stats.total_pairs() > 0)
            .min_by(|a, b| {
                a.error_stats
                    .median_translation_error()
                    .partial_cmp(&b.error_stats.median_translation_error())
                    .unwrap()
            });

        let best_speed = all_benchmarks
            .iter()
            .filter(|b| b.error_stats.convergence_rate() > 80.0)
            .min_by(|a, b| a.avg_time_ms().partial_cmp(&b.avg_time_ms()).unwrap());

        let best_convergence = all_benchmarks.iter().max_by(|a, b| {
            a.error_stats
                .convergence_rate()
                .partial_cmp(&b.error_stats.convergence_rate())
                .unwrap()
        });

        println!("Analysis:");
        if let Some(best) = best_accuracy {
            println!(
                "  Best Accuracy: {} ({:.2}cm median trans error)",
                best.name,
                best.error_stats.median_translation_error() * 100.0
            );
        }
        if let Some(best) = best_speed {
            println!(
                "  Best Speed: {} ({:.2}ms avg)",
                best.name,
                best.avg_time_ms()
            );
        }
        if let Some(best) = best_convergence {
            println!(
                "  Best Convergence: {} ({:.1}%)",
                best.name,
                best.error_stats.convergence_rate()
            );
        }

        // Offset bucket analysis
        println!();
        println!("Error vs Offset Analysis (Hybrid Corr+P2P):");
        let bucket_size = 5;
        let hybrid_buckets = hybrid_bench.offset_bucket_stats(bucket_size);
        for bucket in &hybrid_buckets {
            println!(
                "  Offset {:>5}: {:.2}cm trans, {:.2}° rot ({} samples)",
                bucket.offset_range,
                bucket.mean_trans_error_cm,
                bucket.mean_rot_error_deg,
                bucket.count
            );
        }

        println!();
    }

    // ========================================================================
    // Export Results
    // ========================================================================

    if args.output_json.is_some() || args.output_csv.is_some() {
        let bucket_size = 5;
        let report = BenchmarkReport {
            metadata: BenchmarkMetadata {
                bag_file: args.bag.clone(),
                num_tests: test_pairs.len(),
                min_offset: args.min_offset,
                max_offset: args.max_offset,
                seed: args.seed,
                skip_count: args.skip,
                preprocessing_mode: if args.minimal {
                    "minimal".to_string()
                } else {
                    "full".to_string()
                },
                initial_guess_source: if args.use_fused {
                    format!("fused_encoder_gyro_weight_{:.1}", args.gyro_weight)
                } else {
                    "identity".to_string()
                },
                trans_threshold_m: args.trans_threshold,
                rot_threshold_rad: args.rot_threshold,
            },
            algorithms: all_benchmarks.iter().map(|b| b.to_summary()).collect(),
            offset_analysis: all_benchmarks
                .iter()
                .map(|b| b.offset_bucket_stats(bucket_size))
                .collect(),
        };

        if let Some(ref path) = args.output_json
            && let Err(e) = export_json(&report, path)
        {
            eprintln!("Failed to write JSON: {}", e);
        }

        if let Some(ref path) = args.output_csv
            && let Err(e) = export_csv(&report, path)
        {
            eprintln!("Failed to write CSV: {}", e);
        }
    }
}

// ============================================================================
// Output Functions
// ============================================================================

fn print_stats(bench: &AlgorithmBenchmark) {
    let mean_trans = bench.error_stats.mean_translation_error() * 100.0;
    let med_trans = bench.error_stats.median_translation_error() * 100.0;
    let mean_rot = bench.error_stats.mean_rotation_error().to_degrees();
    let med_rot = bench.error_stats.median_rotation_error().to_degrees();

    let trans_str = if mean_trans.is_nan() {
        "   N/A    ".to_string()
    } else {
        format!("{:4.1}/{:4.1}cm", mean_trans, med_trans)
    };

    let rot_str = if mean_rot.is_nan() {
        "    N/A    ".to_string()
    } else {
        format!("{:4.1}/{:4.1}°", mean_rot, med_rot)
    };

    println!(
        "│ {:23} │ {:5.1}% │ {:>11} │ {:>11} │ {:6.2}ms │",
        bench.name,
        bench.error_stats.convergence_rate(),
        trans_str,
        rot_str,
        bench.avg_time_ms(),
    );
}

fn export_json(report: &BenchmarkReport, path: &str) -> std::io::Result<()> {
    let json = serde_json::to_string_pretty(report)?;
    let mut file = File::create(path)?;
    file.write_all(json.as_bytes())?;
    println!("JSON results written to: {}", path);
    Ok(())
}

fn export_csv(report: &BenchmarkReport, path: &str) -> std::io::Result<()> {
    let mut file = File::create(path)?;

    // Write header
    writeln!(
        file,
        "algorithm,convergence_rate,mean_trans_error_cm,median_trans_error_cm,mean_rot_error_deg,median_rot_error_deg,avg_time_ms,total_tests,converged_count"
    )?;

    // Write data for each algorithm
    for alg in &report.algorithms {
        writeln!(
            file,
            "{},{:.2},{:.4},{:.4},{:.4},{:.4},{:.4},{},{}",
            alg.name,
            alg.convergence_rate,
            alg.mean_trans_error_cm,
            alg.median_trans_error_cm,
            alg.mean_rot_error_deg,
            alg.median_rot_error_deg,
            alg.avg_time_ms,
            alg.total_tests,
            alg.converged_count,
        )?;
    }

    println!("CSV results written to: {}", path);
    Ok(())
}
