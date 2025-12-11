//! Map Representation Benchmark
//!
//! Compares occupancy grid vs feature-based map representations.
//!
//! Metrics:
//! - CPU time: Build time, query time, update time
//! - Memory: Storage size for equivalent map data
//! - Accuracy: Reconstruction error, localization accuracy
//!
//! # Usage
//!
//! ```bash
//! cargo run --release --example map_representation_benchmark -- --bag bags/static.bag
//! ```

use std::mem::size_of;
use std::path::Path;
use std::time::{Duration, Instant};

use clap::Parser;

use dhruva_slam::algorithms::mapping::{
    CellState, FeatureExtractor, FeatureExtractorConfig, LineSegment, MapFeatures, MapIntegrator,
    MapIntegratorConfig, OccupancyGrid, OccupancyGridConfig,
};
use dhruva_slam::algorithms::matching::{
    DynMatcher, MatcherType, ScanMatchValidator, ScanMatchValidatorConfig, ScanMatcher,
};
use dhruva_slam::core::types::{LaserScan, Point2D, PointCloud2D, Pose2D, PoseTracker};
use dhruva_slam::io::bag::{BagMessage, BagPlayer};
use dhruva_slam::sensors::odometry::{
    DynOdometry, DynOdometryConfig, OdometryType, WheelOdometryConfig,
};
use dhruva_slam::sensors::preprocessing::{LidarOffset, PreprocessorConfig, ScanPreprocessor};
use dhruva_slam::utils::{WHEEL_BASE, WHEEL_TICKS_PER_METER};

// Lidar calibration constants
const LIDAR_MOUNTING_X: f32 = -0.110;
const LIDAR_MOUNTING_Y: f32 = 0.0;
const LIDAR_OPTICAL_OFFSET: f32 = 0.02;
const LIDAR_ANGLE_OFFSET: f32 = 12.5;

#[derive(Parser, Debug)]
#[command(name = "map_representation_benchmark")]
#[command(about = "Compare occupancy grid vs feature-based map representations")]
struct Args {
    /// Path to bag file
    #[arg(short, long)]
    bag: String,

    /// Output directory for results
    #[arg(short, long, default_value = "results/map_benchmark")]
    output: String,
}

/// Results for occupancy grid representation
#[derive(Debug, Clone)]
struct OccupancyGridMetrics {
    /// Time to build the full map
    build_time_ms: f64,
    /// Time per scan integration
    avg_integration_time_us: f64,
    /// Memory used by the grid (bytes)
    memory_bytes: usize,
    /// Grid dimensions
    width: usize,
    height: usize,
    /// Number of occupied cells
    occupied_cells: usize,
    /// Number of free cells
    free_cells: usize,
    /// Time to query a single cell (average, nanoseconds)
    avg_query_time_ns: f64,
    /// Time to extract point cloud from grid
    point_cloud_extraction_time_us: f64,
}

/// Results for feature-based representation
#[derive(Debug, Clone)]
struct FeatureMapMetrics {
    /// Time to extract features from occupancy grid
    extraction_time_ms: f64,
    /// Memory used by features (bytes)
    memory_bytes: usize,
    /// Number of line segments
    num_lines: usize,
    /// Number of corners
    num_corners: usize,
    /// Total points represented by lines
    total_line_points: usize,
    /// Average line quality (R²)
    avg_line_quality: f32,
    /// Time to query nearest line (average, nanoseconds)
    avg_query_time_ns: f64,
    /// Reconstruction error: avg distance from original points to nearest line
    reconstruction_error_m: f32,
    /// Coverage: percentage of original occupied cells within threshold of a line
    coverage_percent: f32,
}

/// Combined benchmark results
#[derive(Debug)]
struct BenchmarkResults {
    bag_file: String,
    scans_processed: usize,
    occupancy: OccupancyGridMetrics,
    features: FeatureMapMetrics,
    // Comparison metrics
    memory_ratio: f64,      // feature / occupancy
    compression_ratio: f64, // occupied cells / feature count
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    println!("=== Map Representation Benchmark ===\n");
    println!("Bag file: {}", args.bag);

    // Create output directory
    std::fs::create_dir_all(&args.output)?;

    let results = run_benchmark(&args.bag)?;
    print_results(&results);
    save_results(&results, &args.output)?;

    Ok(())
}

fn run_benchmark(bag_path: &str) -> Result<BenchmarkResults, Box<dyn std::error::Error>> {
    // =========================================================================
    // Phase 1: Build occupancy grid (standard SLAM)
    // =========================================================================
    println!("\n--- Phase 1: Building Occupancy Grid ---");

    let (map, scans_processed, build_time, integration_times) = build_occupancy_grid(bag_path)?;

    let avg_integration_time_us = if !integration_times.is_empty() {
        integration_times.iter().sum::<Duration>().as_micros() as f64
            / integration_times.len() as f64
    } else {
        0.0
    };

    // Measure grid memory
    let (width, height) = map.dimensions();
    let grid_memory = width * height * size_of::<f32>() + size_of::<OccupancyGrid>();

    // Count cells
    let mut occupied_cells = 0;
    let mut free_cells = 0;
    for cy in 0..height {
        for cx in 0..width {
            match map.get_state(cx, cy) {
                CellState::Occupied => occupied_cells += 1,
                CellState::Free => free_cells += 1,
                CellState::Unknown => {}
            }
        }
    }

    // Measure query time
    let query_iterations = 10000;
    let query_start = Instant::now();
    for i in 0..query_iterations {
        let cx = (i * 7) % width;
        let cy = (i * 11) % height;
        let _ = map.get_state(cx, cy);
    }
    let avg_query_time_ns = query_start.elapsed().as_nanos() as f64 / query_iterations as f64;

    // Measure point cloud extraction time
    let extract_start = Instant::now();
    let occupied_points = map.occupied_points();
    let point_cloud_extraction_time_us = extract_start.elapsed().as_micros() as f64;

    let occupancy_metrics = OccupancyGridMetrics {
        build_time_ms: build_time.as_secs_f64() * 1000.0,
        avg_integration_time_us,
        memory_bytes: grid_memory,
        width,
        height,
        occupied_cells,
        free_cells,
        avg_query_time_ns,
        point_cloud_extraction_time_us,
    };

    println!("  Grid: {}x{} cells", width, height);
    println!("  Occupied: {}, Free: {}", occupied_cells, free_cells);
    println!("  Build time: {:.1}ms", occupancy_metrics.build_time_ms);
    println!("  Memory: {:.1} KB", grid_memory as f64 / 1024.0);

    // =========================================================================
    // Phase 2: Extract features
    // =========================================================================
    println!("\n--- Phase 2: Extracting Features ---");

    let extract_start = Instant::now();
    let mut extractor = FeatureExtractor::new(FeatureExtractorConfig::default());
    let features = extractor.extract(&map);
    let extraction_time = extract_start.elapsed();

    // Measure feature memory
    let line_memory = features.lines.len() * size_of::<LineSegment>();
    let corner_memory =
        features.corners.len() * size_of::<dhruva_slam::algorithms::mapping::Corner>();
    let feature_memory = line_memory + corner_memory + size_of::<MapFeatures>();

    // Calculate total points represented
    let total_line_points: usize = features.lines.iter().map(|l| l.point_count).sum();

    // Average line quality
    let avg_line_quality = if !features.lines.is_empty() {
        features.lines.iter().map(|l| l.quality).sum::<f32>() / features.lines.len() as f32
    } else {
        0.0
    };

    // Measure query time (find nearest line to a point)
    let query_iterations = 10000;
    let query_start = Instant::now();
    for i in 0..query_iterations {
        let test_point = Point2D::new((i as f32 * 0.1) % 5.0 - 2.5, (i as f32 * 0.07) % 5.0 - 2.5);
        let _ = find_nearest_line(&features.lines, &test_point);
    }
    let feature_query_time_ns = query_start.elapsed().as_nanos() as f64 / query_iterations as f64;

    // Calculate reconstruction error and coverage
    let (reconstruction_error, coverage) =
        calculate_reconstruction_metrics(&occupied_points, &features);

    let feature_metrics = FeatureMapMetrics {
        extraction_time_ms: extraction_time.as_secs_f64() * 1000.0,
        memory_bytes: feature_memory,
        num_lines: features.lines.len(),
        num_corners: features.corners.len(),
        total_line_points,
        avg_line_quality,
        avg_query_time_ns: feature_query_time_ns,
        reconstruction_error_m: reconstruction_error,
        coverage_percent: coverage,
    };

    println!(
        "  Lines: {}, Corners: {}",
        features.lines.len(),
        features.corners.len()
    );
    println!(
        "  Extraction time: {:.1}ms",
        feature_metrics.extraction_time_ms
    );
    println!("  Memory: {:.1} KB", feature_memory as f64 / 1024.0);
    println!("  Reconstruction error: {:.3}m", reconstruction_error);
    println!("  Coverage: {:.1}%", coverage);

    // =========================================================================
    // Phase 3: Compute comparison metrics
    // =========================================================================
    let memory_ratio = feature_memory as f64 / grid_memory as f64;
    let compression_ratio =
        occupied_cells as f64 / (features.lines.len() + features.corners.len()).max(1) as f64;

    Ok(BenchmarkResults {
        bag_file: Path::new(bag_path)
            .file_name()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| bag_path.to_string()),
        scans_processed,
        occupancy: occupancy_metrics,
        features: feature_metrics,
        memory_ratio,
        compression_ratio,
    })
}

fn build_occupancy_grid(
    bag_path: &str,
) -> Result<(OccupancyGrid, usize, Duration, Vec<Duration>), Box<dyn std::error::Error>> {
    let mut player = BagPlayer::open(bag_path)?;
    player.set_speed(0.0);

    let odom_config = DynOdometryConfig {
        wheel: WheelOdometryConfig {
            ticks_per_meter: WHEEL_TICKS_PER_METER,
            wheel_base: WHEEL_BASE,
        },
        ..DynOdometryConfig::default()
    };
    let mut odometry = DynOdometry::new(OdometryType::Wheel, odom_config);
    let mut matcher = DynMatcher::new(MatcherType::MultiRes);
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
    let validator = ScanMatchValidator::new(ScanMatchValidatorConfig::default());

    let mut odom_tracker = PoseTracker::new();
    let mut slam_pose = Pose2D::identity();
    let mut scans_processed = 0usize;
    let mut integration_times = Vec::new();

    let start_time = Instant::now();

    while let Ok(Some(msg)) = player.next_immediate() {
        match msg {
            BagMessage::SensorStatus(status) => {
                if let Some(pose) = odometry.update(
                    status.encoder.left,
                    status.encoder.right,
                    status.gyro_raw[2],
                    status.timestamp_us,
                ) {
                    odom_tracker.set(pose);
                }
            }
            BagMessage::Lidar(timestamped) => {
                let laser_scan = LaserScan::from_lidar_scan(&timestamped.data);
                let processed = preprocessor.process(&laser_scan);

                if processed.len() < 50 {
                    continue;
                }

                if scans_processed == 0 {
                    // First scan - just integrate
                    let integrate_start = Instant::now();
                    integrator.integrate_cloud(&mut map, &processed, &slam_pose);
                    integration_times.push(integrate_start.elapsed());
                    odom_tracker.take_snapshot();
                    scans_processed += 1;
                    continue;
                }

                let odom_delta = odom_tracker.delta_since_snapshot();
                let predicted_pose = slam_pose.compose(&odom_delta);

                // Scan-to-map matching
                let map_cloud = map_to_point_cloud(&map);
                if map_cloud.len() >= 50 {
                    let map_local = map_cloud.transform(&predicted_pose.inverse());
                    let match_result =
                        matcher.match_scans(&processed, &map_local, &Pose2D::identity());
                    let validation = validator.validate(&match_result, &Pose2D::identity());

                    if validation.is_valid {
                        slam_pose = predicted_pose.compose(&match_result.transform);
                    } else {
                        slam_pose = predicted_pose;
                    }
                } else {
                    slam_pose = predicted_pose;
                }

                let integrate_start = Instant::now();
                let global_scan = processed.transform(&slam_pose);
                integrator.integrate_cloud(&mut map, &global_scan, &slam_pose);
                integration_times.push(integrate_start.elapsed());

                odom_tracker.take_snapshot();
                scans_processed += 1;
            }
            BagMessage::Odometry(_) => {
                // Ignore legacy odometry messages
            }
        }
    }

    let build_time = start_time.elapsed();
    Ok((map, scans_processed, build_time, integration_times))
}

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

fn find_nearest_line<'a>(
    lines: &'a [LineSegment],
    point: &Point2D,
) -> Option<(&'a LineSegment, f32)> {
    let mut nearest: Option<(&LineSegment, f32)> = None;

    for line in lines {
        let dist = line.distance_to_line(point).abs();
        if nearest.is_none() || dist < nearest.unwrap().1 {
            nearest = Some((line, dist));
        }
    }

    nearest
}

fn calculate_reconstruction_metrics(points: &[Point2D], features: &MapFeatures) -> (f32, f32) {
    if points.is_empty() || features.lines.is_empty() {
        return (f32::INFINITY, 0.0);
    }

    let threshold = 0.05; // 5cm threshold for "covered"
    let mut total_error = 0.0f32;
    let mut covered_count = 0usize;

    for point in points {
        if let Some((line, dist)) = find_nearest_line(&features.lines, point) {
            // Only count if point projects onto the line segment
            if line.point_projects_onto_segment(point) {
                total_error += dist;
                if dist <= threshold {
                    covered_count += 1;
                }
            } else {
                // Point doesn't project - use distance to nearest endpoint
                let d_start =
                    ((point.x - line.start.x).powi(2) + (point.y - line.start.y).powi(2)).sqrt();
                let d_end =
                    ((point.x - line.end.x).powi(2) + (point.y - line.end.y).powi(2)).sqrt();
                let endpoint_dist = d_start.min(d_end);
                total_error += endpoint_dist;
                if endpoint_dist <= threshold {
                    covered_count += 1;
                }
            }
        }
    }

    let avg_error = total_error / points.len() as f32;
    let coverage = (covered_count as f32 / points.len() as f32) * 100.0;

    (avg_error, coverage)
}

fn print_results(results: &BenchmarkResults) {
    println!("\n{}", "=".repeat(60));
    println!("        MAP REPRESENTATION BENCHMARK RESULTS");
    println!("{}\n", "=".repeat(60));

    println!("Bag File: {}", results.bag_file);
    println!("Scans Processed: {}\n", results.scans_processed);

    println!("┌─────────────────────────────────────────────────────────┐");
    println!("│                    OCCUPANCY GRID                       │");
    println!("├─────────────────────────────────────────────────────────┤");
    println!(
        "│ Dimensions:        {:>6} x {:>6} cells                │",
        results.occupancy.width, results.occupancy.height
    );
    println!(
        "│ Occupied cells:    {:>10}                          │",
        results.occupancy.occupied_cells
    );
    println!(
        "│ Free cells:        {:>10}                          │",
        results.occupancy.free_cells
    );
    println!(
        "│ Build time:        {:>10.1} ms                       │",
        results.occupancy.build_time_ms
    );
    println!(
        "│ Avg integration:   {:>10.1} µs/scan                  │",
        results.occupancy.avg_integration_time_us
    );
    println!(
        "│ Memory usage:      {:>10.1} KB                       │",
        results.occupancy.memory_bytes as f64 / 1024.0
    );
    println!(
        "│ Query time:        {:>10.1} ns                       │",
        results.occupancy.avg_query_time_ns
    );
    println!(
        "│ Point extraction:  {:>10.1} µs                       │",
        results.occupancy.point_cloud_extraction_time_us
    );
    println!("└─────────────────────────────────────────────────────────┘\n");

    println!("┌─────────────────────────────────────────────────────────┐");
    println!("│                   FEATURE-BASED MAP                     │");
    println!("├─────────────────────────────────────────────────────────┤");
    println!(
        "│ Lines:             {:>10}                          │",
        results.features.num_lines
    );
    println!(
        "│ Corners:           {:>10}                          │",
        results.features.num_corners
    );
    println!(
        "│ Points represented:{:>10}                          │",
        results.features.total_line_points
    );
    println!(
        "│ Avg line quality:  {:>10.3}                          │",
        results.features.avg_line_quality
    );
    println!(
        "│ Extraction time:   {:>10.1} ms                       │",
        results.features.extraction_time_ms
    );
    println!(
        "│ Memory usage:      {:>10.1} KB                       │",
        results.features.memory_bytes as f64 / 1024.0
    );
    println!(
        "│ Query time:        {:>10.1} ns                       │",
        results.features.avg_query_time_ns
    );
    println!(
        "│ Reconstruction err:{:>10.3} m                        │",
        results.features.reconstruction_error_m
    );
    println!(
        "│ Coverage:          {:>10.1} %                        │",
        results.features.coverage_percent
    );
    println!("└─────────────────────────────────────────────────────────┘\n");

    println!("┌─────────────────────────────────────────────────────────┐");
    println!("│                     COMPARISON                          │");
    println!("├─────────────────────────────────────────────────────────┤");
    println!(
        "│ Memory ratio:      {:>10.4}x (feature/grid)         │",
        results.memory_ratio
    );
    println!(
        "│ Compression ratio: {:>10.1}:1 (cells/features)      │",
        results.compression_ratio
    );
    let speed_ratio = results.occupancy.avg_query_time_ns / results.features.avg_query_time_ns;
    println!(
        "│ Query speed ratio: {:>10.2}x (grid faster)          │",
        speed_ratio
    );
    println!("└─────────────────────────────────────────────────────────┘\n");

    // Summary
    println!("SUMMARY:");
    println!(
        "  - Features use {:.1}% of grid memory",
        results.memory_ratio * 100.0
    );
    println!(
        "  - Each feature represents ~{:.0} occupied cells",
        results.compression_ratio
    );
    println!(
        "  - Reconstruction covers {:.1}% of original map",
        results.features.coverage_percent
    );
    println!(
        "  - Average reconstruction error: {:.1} cm",
        results.features.reconstruction_error_m * 100.0
    );
}

fn save_results(
    results: &BenchmarkResults,
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let filename = format!("{}_benchmark.txt", results.bag_file.replace(".bag", ""));
    let path = Path::new(output_dir).join(&filename);

    let mut content = String::new();
    content.push_str(&format!(
        "Map Representation Benchmark: {}\n",
        results.bag_file
    ));
    content.push_str(&format!("Scans: {}\n\n", results.scans_processed));

    content.push_str("OCCUPANCY GRID:\n");
    content.push_str(&format!(
        "  Dimensions: {}x{}\n",
        results.occupancy.width, results.occupancy.height
    ));
    content.push_str(&format!(
        "  Occupied: {}\n",
        results.occupancy.occupied_cells
    ));
    content.push_str(&format!(
        "  Memory: {:.1} KB\n",
        results.occupancy.memory_bytes as f64 / 1024.0
    ));
    content.push_str(&format!(
        "  Build time: {:.1} ms\n",
        results.occupancy.build_time_ms
    ));
    content.push_str(&format!(
        "  Query time: {:.1} ns\n\n",
        results.occupancy.avg_query_time_ns
    ));

    content.push_str("FEATURE MAP:\n");
    content.push_str(&format!("  Lines: {}\n", results.features.num_lines));
    content.push_str(&format!("  Corners: {}\n", results.features.num_corners));
    content.push_str(&format!(
        "  Memory: {:.1} KB\n",
        results.features.memory_bytes as f64 / 1024.0
    ));
    content.push_str(&format!(
        "  Extraction time: {:.1} ms\n",
        results.features.extraction_time_ms
    ));
    content.push_str(&format!(
        "  Query time: {:.1} ns\n",
        results.features.avg_query_time_ns
    ));
    content.push_str(&format!(
        "  Reconstruction error: {:.3} m\n",
        results.features.reconstruction_error_m
    ));
    content.push_str(&format!(
        "  Coverage: {:.1}%\n\n",
        results.features.coverage_percent
    ));

    content.push_str("COMPARISON:\n");
    content.push_str(&format!("  Memory ratio: {:.4}x\n", results.memory_ratio));
    content.push_str(&format!(
        "  Compression: {:.1}:1\n",
        results.compression_ratio
    ));

    std::fs::write(&path, content)?;
    println!("Results saved to: {}", path.display());

    Ok(())
}
