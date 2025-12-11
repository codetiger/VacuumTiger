//! Focused SLAM Node: MultiRes + Mahony + Feature Maps
//!
//! Production-optimized SLAM daemon using:
//! - **Matcher**: MultiResolution correlative scan matching (99.7% wall coherence)
//! - **Odometry**: Mahony AHRS filter (auto-calibrating gyro fusion)
//! - **Maps**: Dual representation (Occupancy Grid + Feature-based)
//!
//! # Usage
//!
//! ```bash
//! # With default config
//! cargo run --release --example dhruva_slam_node
//!
//! # With custom config file
//! cargo run --release --example dhruva_slam_node -- --config dhruva-slam.toml
//!
//! # With command line overrides
//! cargo run --release --example dhruva_slam_node -- --sangam 192.168.68.101:5555 --port 5557
//! ```

use std::fs;
use std::io::Write;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use serde::Deserialize;

use dhruva_slam::SangamClient;
use dhruva_slam::algorithms::mapping::{
    CellState, FeatureExtractor, FeatureExtractorConfig, MapIntegrator, MapIntegratorConfig,
    OccupancyGrid, OccupancyGridConfig,
};
use dhruva_slam::algorithms::matching::{
    MultiResolutionConfig, MultiResolutionMatcher, ScanMatcher,
};
use dhruva_slam::core::types::{LaserScan, PointCloud2D, Pose2D, PoseTracker};
use dhruva_slam::io::bag::{BagMessage, BagPlayer};
use dhruva_slam::io::streaming::OdometryPublisher;
use dhruva_slam::sensors::odometry::{
    DynOdometry, DynOdometryConfig, MahonyConfig, OdometryType, WheelOdometryConfig,
};
use dhruva_slam::sensors::preprocessing::{
    AngularDownsamplerConfig, LidarOffset, OutlierFilterConfig, PreprocessorConfig,
    RangeFilterConfig, ScanPreprocessor,
};
use dhruva_slam::utils::{WHEEL_BASE, WHEEL_TICKS_PER_METER};

// ============================================================================
// Hardware Calibration Constants (from benchmarks)
// ============================================================================

/// Lidar housing mounting position relative to robot center (meters)
const LIDAR_MOUNTING_X: f32 = -0.110; // 110mm behind center
const LIDAR_MOUNTING_Y: f32 = 0.0; // Centered on robot axis

/// Optical center offset from housing center (meters)
const LIDAR_OPTICAL_OFFSET: f32 = 0.02;

/// Angular offset if lidar 0 degrees is not aligned with robot forward
const LIDAR_ANGLE_OFFSET: f32 = 12.5;

// ============================================================================
// Configuration
// ============================================================================

#[derive(Debug, Deserialize, Default)]
struct Config {
    #[serde(default)]
    source: SourceConfig,
    #[serde(default)]
    output: OutputConfig,
    #[serde(default)]
    slam: SlamConfig,
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct SourceConfig {
    sangam_address: String,
}

impl Default for SourceConfig {
    fn default() -> Self {
        Self {
            sangam_address: "192.168.68.101:5555".to_string(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct OutputConfig {
    bind_port: u16,
    odometry_rate_hz: f32,
    map_rate_hz: f32,
    feature_rate_hz: f32,
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            bind_port: 5557,
            odometry_rate_hz: 50.0,
            map_rate_hz: 1.0,
            feature_rate_hz: 0.2, // Every 5 seconds
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct SlamConfig {
    min_range: f32,
    max_range: f32,
}

impl Default for SlamConfig {
    fn default() -> Self {
        Self {
            min_range: 0.15, // 15cm minimum range
            max_range: 8.0,  // 8m maximum range
        }
    }
}

// ============================================================================
// CLI Arguments
// ============================================================================

struct Args {
    config_path: Option<String>,
    sangam_address: Option<String>,
    bind_port: Option<u16>,
    bag_file: Option<String>,
    loop_bag: bool,
}

fn parse_args() -> Args {
    let args: Vec<String> = std::env::args().collect();
    let mut result = Args {
        config_path: None,
        sangam_address: None,
        bind_port: None,
        bag_file: None,
        loop_bag: false,
    };

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--config" | "-c" => {
                if i + 1 < args.len() {
                    result.config_path = Some(args[i + 1].clone());
                    i += 1;
                }
            }
            "--sangam" | "-s" => {
                if i + 1 < args.len() {
                    result.sangam_address = Some(args[i + 1].clone());
                    i += 1;
                }
            }
            "--port" | "-p" => {
                if i + 1 < args.len() {
                    result.bind_port = args[i + 1].parse().ok();
                    i += 1;
                }
            }
            "--bag" | "-b" => {
                if i + 1 < args.len() {
                    result.bag_file = Some(args[i + 1].clone());
                    i += 1;
                }
            }
            "--loop" | "-l" => {
                result.loop_bag = true;
            }
            "--help" | "-h" => {
                print_help();
                std::process::exit(0);
            }
            _ => {
                eprintln!("Unknown argument: {}", args[i]);
                print_help();
                std::process::exit(1);
            }
        }
        i += 1;
    }

    result
}

fn print_help() {
    println!("dhruva-slam-node - Focused SLAM daemon (MultiRes + Mahony + Features)");
    println!();
    println!("USAGE:");
    println!("    dhruva-slam-node [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    -c, --config <FILE>     Configuration file (dhruva-slam.toml)");
    println!("    -s, --sangam <ADDR>     SangamIO address (192.168.68.101:5555)");
    println!("    -p, --port <PORT>       TCP publish port (5557)");
    println!("    -b, --bag <FILE>        Bag file for offline playback (instead of SangamIO)");
    println!("    -l, --loop              Loop bag file playback");
    println!("    -h, --help              Print help information");
}

fn load_config(args: &Args) -> Config {
    let config = match &args.config_path {
        Some(path) => match fs::read_to_string(path) {
            Ok(contents) => match basic_toml::from_str(&contents) {
                Ok(cfg) => {
                    log::info!("Loaded config from {}", path);
                    cfg
                }
                Err(e) => {
                    log::warn!("Failed to parse config {}: {}", path, e);
                    Config::default()
                }
            },
            Err(e) => {
                log::warn!("Failed to read config {}: {}", path, e);
                Config::default()
            }
        },
        None => {
            // Try default paths
            for path in &["dhruva-slam.toml", "/etc/dhruva-slam.toml"] {
                if let Ok(contents) = fs::read_to_string(path)
                    && let Ok(cfg) = basic_toml::from_str(&contents)
                {
                    log::info!("Loaded config from {}", path);
                    return apply_overrides(cfg, args);
                }
            }
            Config::default()
        }
    };

    apply_overrides(config, args)
}

fn apply_overrides(mut config: Config, args: &Args) -> Config {
    if let Some(addr) = &args.sangam_address {
        config.source.sangam_address = addr.clone();
    }
    if let Some(port) = args.bind_port {
        config.output.bind_port = port;
    }
    config
}

// ============================================================================
// Main Entry Point
// ============================================================================

fn main() {
    // Initialize logging
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format(|buf, record| {
            writeln!(
                buf,
                "[{}] {} - {}",
                record.level(),
                record.target(),
                record.args()
            )
        })
        .init();

    let args = parse_args();
    let config = load_config(&args);

    log::info!("dhruva-slam-node starting (focused implementation)");
    if let Some(ref bag) = args.bag_file {
        log::info!("  Input: Bag file {}", bag);
        if args.loop_bag {
            log::info!("  Loop: enabled");
        }
    } else {
        log::info!("  SangamIO: {}", config.source.sangam_address);
    }
    log::info!("  Publish port: {}", config.output.bind_port);
    log::info!("  Algorithm: MultiRes + Mahony");
    log::info!("  Output: Occupancy Grid + Features");

    // Setup signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        log::info!("Received shutdown signal");
        r.store(false, Ordering::Relaxed);
    })
    .expect("Error setting Ctrl-C handler");

    // Run main loop - bag or live mode
    if let Some(ref bag_path) = args.bag_file {
        // Bag file playback mode
        loop {
            if !running.load(Ordering::Relaxed) {
                break;
            }
            if let Err(e) = run_bag_loop(&config, bag_path, running.clone()) {
                log::error!("Bag playback error: {}", e);
            }
            if !args.loop_bag || !running.load(Ordering::Relaxed) {
                break;
            }
            log::info!("Looping bag file...");
        }
    } else {
        // Live SangamIO mode with reconnection
        while running.load(Ordering::Relaxed) {
            if let Err(e) = run_main_loop(&config, running.clone()) {
                log::error!("Main loop error: {}", e);
                if running.load(Ordering::Relaxed) {
                    log::info!("Reconnecting in 5 seconds...");
                    std::thread::sleep(Duration::from_secs(5));
                }
            }
        }
    }

    log::info!("dhruva-slam-node shutdown complete");
}

// ============================================================================
// Main Processing Loop
// ============================================================================

fn run_main_loop(
    config: &Config,
    running: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error>> {
    // Connect to SangamIO
    log::info!(
        "Connecting to SangamIO at {}...",
        config.source.sangam_address
    );
    let mut client = SangamClient::connect(&config.source.sangam_address)?;
    log::info!("Connected to SangamIO");

    // Start publisher
    let publisher = OdometryPublisher::new(config.output.bind_port)?;
    log::info!("Publisher listening on port {}", config.output.bind_port);

    // Initialize Mahony AHRS odometry (auto-calibrating)
    let mut odometry = DynOdometry::new(
        OdometryType::Mahony,
        DynOdometryConfig {
            wheel: WheelOdometryConfig {
                ticks_per_meter: WHEEL_TICKS_PER_METER,
                wheel_base: WHEEL_BASE,
            },
            mahony: MahonyConfig::default(),
            ..Default::default()
        },
    );
    log::info!("Odometry: Mahony AHRS (auto-calibrating gyro fusion)");

    // Initialize MultiResolution scan matcher
    let mut matcher = MultiResolutionMatcher::new(MultiResolutionConfig::default());
    log::info!("Matcher: MultiResolution correlative");

    // Initialize occupancy grid map
    let mut map = OccupancyGrid::new(OccupancyGridConfig::default());
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Initialize feature extractor
    let mut feature_extractor = FeatureExtractor::new(FeatureExtractorConfig::default());

    // Initialize scan preprocessor
    let preprocessor = ScanPreprocessor::new(PreprocessorConfig {
        range_filter: RangeFilterConfig {
            min_range: config.slam.min_range,
            max_range: config.slam.max_range,
        },
        outlier_filter: OutlierFilterConfig::default(),
        downsampler: AngularDownsamplerConfig {
            target_points: 180,
            min_angle_step: 1.0_f32.to_radians(),
        },
        lidar_offset: LidarOffset::new(
            LIDAR_MOUNTING_X,
            LIDAR_MOUNTING_Y,
            LIDAR_OPTICAL_OFFSET,
            LIDAR_ANGLE_OFFSET,
        ),
    });

    // Pose tracking
    let mut odom_tracker = PoseTracker::new();
    let mut slam_pose = Pose2D::identity();
    let mut slam_base_pose = Pose2D::identity();
    let mut slam_odom_snapshot = PoseTracker::new();

    // Rate limiting
    let map_interval = Duration::from_secs_f32(1.0 / config.output.map_rate_hz);
    let feature_interval = Duration::from_secs_f32(1.0 / config.output.feature_rate_hz);
    let mut last_map_time = Instant::now();
    let mut last_feature_time = Instant::now();
    let mut last_log_time = Instant::now();

    // Statistics
    let mut scans_processed = 0u64;
    let mut _odom_updates = 0u64;

    log::info!("Processing started");
    log::info!(
        "  Range filter: {:.2}-{:.2}m",
        config.slam.min_range,
        config.slam.max_range
    );
    log::info!("  Map rate: {} Hz", config.output.map_rate_hz);
    log::info!("  Feature rate: {} Hz", config.output.feature_rate_hz);

    // Main processing loop
    let mut msg_count = 0u64;
    while running.load(Ordering::Relaxed) {
        let msg = client.recv()?;
        let timestamp_us = msg.timestamp_us();
        msg_count += 1;

        // Log message types periodically
        if msg_count % 500 == 0 {
            log::info!(
                "Received {} messages, group_id: {}",
                msg_count,
                msg.group_id()
            );
        }

        // Process sensor_status messages (odometry @ 110Hz)
        if msg.group_id() == "sensor_status" {
            let (left, right) = match msg.encoder_ticks() {
                Some(ticks) => ticks,
                None => continue,
            };
            let gyro_yaw = msg.gyro_yaw_raw().unwrap_or(0);

            // Update odometry
            if let Some(odom_pose) = odometry.update(left, right, gyro_yaw, timestamp_us) {
                odom_tracker.set(odom_pose);

                // Compute SLAM-corrected pose for publishing
                let odom_delta = slam_odom_snapshot
                    .snapshot_pose()
                    .inverse()
                    .compose(&odom_pose);
                let published_pose = slam_base_pose.compose(&odom_delta);
                publisher.publish_pose(&published_pose, timestamp_us);
                _odom_updates += 1;
            }
        }
        // Process lidar messages (SLAM @ ~5Hz)
        else if msg.group_id() == "lidar" {
            let lidar_data = match msg.as_lidar() {
                Some(data) => data,
                None => {
                    log::warn!("Lidar message but as_lidar() returned None");
                    continue;
                }
            };

            // Preprocess scan
            let laser_scan = LaserScan::from_lidar_scan(lidar_data.data);
            let scan_cloud = preprocessor.process(&laser_scan);
            log::debug!(
                "Lidar scan: {} raw points -> {} processed",
                lidar_data.data.len(),
                scan_cloud.len()
            );

            // Skip sparse scans
            if scan_cloud.len() < 50 {
                continue;
            }

            // Compute odometry delta since last scan
            let odom_delta = odom_tracker.delta_since_snapshot();
            let predicted_pose = slam_pose.compose(&odom_delta);

            // Scan-to-map matching
            if scans_processed > 0 {
                let map_cloud = map_to_point_cloud(&map);
                if map_cloud.len() >= 50 {
                    // Transform map to local frame relative to predicted pose
                    let map_local = map_cloud.transform(&predicted_pose.inverse());

                    // Match scan against local-frame map
                    let match_result =
                        matcher.match_scans(&scan_cloud, &map_local, &Pose2D::identity());

                    if match_result.score > 0.3 {
                        slam_pose = predicted_pose.compose(&match_result.transform);
                    } else {
                        slam_pose = predicted_pose;
                    }
                } else {
                    slam_pose = predicted_pose;
                }
            }

            // Integrate scan into map
            let global_scan = scan_cloud.transform(&slam_pose);
            integrator.integrate_cloud(&mut map, &global_scan, &slam_pose);

            // Update pose tracking
            slam_base_pose = slam_pose;
            slam_odom_snapshot.set(odom_tracker.pose());
            slam_odom_snapshot.take_snapshot();
            odom_tracker.take_snapshot();

            // Publish scan
            publisher.publish_slam_scan(&scan_cloud, &slam_pose, timestamp_us);
            scans_processed += 1;
        }

        // Publish map periodically
        if last_map_time.elapsed() >= map_interval {
            let occupied_count = count_occupied_cells(&map);
            let (map_width, map_height) = map.dimensions();
            log::info!(
                "Publishing map: {}x{} cells, {} occupied, {} clients",
                map_width,
                map_height,
                occupied_count,
                publisher.client_count()
            );
            publisher.publish_slam_map(&map, timestamp_us);
            last_map_time = Instant::now();
        }

        // Extract and publish features periodically
        if last_feature_time.elapsed() >= feature_interval {
            let features = feature_extractor.extract(&map);
            publisher.publish_features(&features, timestamp_us);
            last_feature_time = Instant::now();

            log::debug!(
                "Features: {} lines, {} corners",
                features.lines.len(),
                features.corners.len()
            );
        }

        // Log statistics periodically
        if last_log_time.elapsed() >= Duration::from_secs(10) {
            let (width, height) = map.dimensions();
            let occupied = count_occupied_cells(&map);
            log::info!(
                "Pose: ({:.2}, {:.2}, {:.1}deg) | Map: {}x{} ({} occupied) | {} scans | {} clients",
                slam_pose.x,
                slam_pose.y,
                slam_pose.theta.to_degrees(),
                width,
                height,
                occupied,
                scans_processed,
                publisher.client_count()
            );
            last_log_time = Instant::now();
        }
    }

    Ok(())
}

// ============================================================================
// Bag File Playback Loop
// ============================================================================

fn run_bag_loop(
    config: &Config,
    bag_path: &str,
    running: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error>> {
    // Open bag file
    log::info!("Opening bag file: {}", bag_path);
    let mut player = BagPlayer::open(bag_path)?;
    player.set_speed(1.0); // Real-time playback

    // Start publisher
    let publisher = OdometryPublisher::new(config.output.bind_port)?;
    log::info!("Publisher listening on port {}", config.output.bind_port);

    // Initialize Mahony AHRS odometry (auto-calibrating)
    let mut odometry = DynOdometry::new(
        OdometryType::Mahony,
        DynOdometryConfig {
            wheel: WheelOdometryConfig {
                ticks_per_meter: WHEEL_TICKS_PER_METER,
                wheel_base: WHEEL_BASE,
            },
            mahony: MahonyConfig::default(),
            ..Default::default()
        },
    );

    // Initialize MultiResolution scan matcher
    let mut matcher = MultiResolutionMatcher::new(MultiResolutionConfig::default());

    // Initialize occupancy grid map
    let mut map = OccupancyGrid::new(OccupancyGridConfig::default());
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Initialize feature extractor
    let mut feature_extractor = FeatureExtractor::new(FeatureExtractorConfig::default());

    // Initialize scan preprocessor
    let preprocessor = ScanPreprocessor::new(PreprocessorConfig {
        range_filter: RangeFilterConfig {
            min_range: config.slam.min_range,
            max_range: config.slam.max_range,
        },
        outlier_filter: OutlierFilterConfig::default(),
        downsampler: AngularDownsamplerConfig {
            target_points: 180,
            min_angle_step: 1.0_f32.to_radians(),
        },
        lidar_offset: LidarOffset::new(
            LIDAR_MOUNTING_X,
            LIDAR_MOUNTING_Y,
            LIDAR_OPTICAL_OFFSET,
            LIDAR_ANGLE_OFFSET,
        ),
    });

    // Pose tracking
    let mut odom_tracker = PoseTracker::new();
    let mut slam_pose = Pose2D::identity();
    let mut slam_base_pose = Pose2D::identity();
    let mut slam_odom_snapshot = PoseTracker::new();

    // Rate limiting
    let map_interval = Duration::from_secs_f32(1.0 / config.output.map_rate_hz);
    let feature_interval = Duration::from_secs_f32(1.0 / config.output.feature_rate_hz);
    let mut last_map_time = Instant::now();
    let mut last_feature_time = Instant::now();
    let mut last_log_time = Instant::now();

    // Statistics
    let mut scans_processed = 0u64;
    let mut last_timestamp_us = 0u64;

    log::info!("Bag playback started");

    // Main processing loop
    while running.load(Ordering::Relaxed) {
        let msg = match player.next() {
            Ok(Some(m)) => m,
            Ok(None) => {
                log::info!("End of bag file reached");
                break;
            }
            Err(e) => {
                log::error!("Bag read error: {}", e);
                break;
            }
        };

        match msg {
            BagMessage::SensorStatus(status) => {
                let timestamp_us = status.timestamp_us;
                last_timestamp_us = timestamp_us;

                // Update odometry
                if let Some(odom_pose) = odometry.update(
                    status.encoder.left,
                    status.encoder.right,
                    status.gyro_raw[2],
                    timestamp_us,
                ) {
                    odom_tracker.set(odom_pose);

                    // Compute SLAM-corrected pose for publishing
                    let odom_delta = slam_odom_snapshot
                        .snapshot_pose()
                        .inverse()
                        .compose(&odom_pose);
                    let published_pose = slam_base_pose.compose(&odom_delta);
                    publisher.publish_pose(&published_pose, timestamp_us);
                }
            }
            BagMessage::Lidar(timestamped) => {
                let timestamp_us = timestamped.timestamp_us;
                last_timestamp_us = timestamp_us;

                // Preprocess scan
                let laser_scan = LaserScan::from_lidar_scan(&timestamped.data);
                let scan_cloud = preprocessor.process(&laser_scan);

                // Skip sparse scans
                if scan_cloud.len() < 50 {
                    continue;
                }

                // Compute odometry delta since last scan
                let odom_delta = odom_tracker.delta_since_snapshot();
                let predicted_pose = slam_pose.compose(&odom_delta);

                // Scan-to-map matching
                if scans_processed > 0 {
                    let map_cloud = map_to_point_cloud(&map);
                    if map_cloud.len() >= 50 {
                        // Transform map to local frame relative to predicted pose
                        let map_local = map_cloud.transform(&predicted_pose.inverse());

                        // Match scan against local-frame map
                        let match_result =
                            matcher.match_scans(&scan_cloud, &map_local, &Pose2D::identity());

                        if match_result.score > 0.3 {
                            slam_pose = predicted_pose.compose(&match_result.transform);
                        } else {
                            slam_pose = predicted_pose;
                        }
                    } else {
                        slam_pose = predicted_pose;
                    }
                }

                // Integrate scan into map
                let global_scan = scan_cloud.transform(&slam_pose);
                integrator.integrate_cloud(&mut map, &global_scan, &slam_pose);

                // Update pose tracking
                slam_base_pose = slam_pose;
                slam_odom_snapshot.set(odom_tracker.pose());
                slam_odom_snapshot.take_snapshot();
                odom_tracker.take_snapshot();

                // Publish scan
                publisher.publish_slam_scan(&scan_cloud, &slam_pose, timestamp_us);
                scans_processed += 1;
            }
            BagMessage::Odometry(_) => {
                // Skip raw odometry messages, we compute our own
            }
        }

        let timestamp_us = last_timestamp_us;

        // Publish map periodically
        if last_map_time.elapsed() >= map_interval {
            let occupied_count = count_occupied_cells(&map);
            let (map_width, map_height) = map.dimensions();
            log::info!(
                "Publishing map: {}x{} cells, {} occupied, {} clients",
                map_width,
                map_height,
                occupied_count,
                publisher.client_count()
            );
            publisher.publish_slam_map(&map, timestamp_us);
            last_map_time = Instant::now();
        }

        // Extract and publish features periodically
        if last_feature_time.elapsed() >= feature_interval {
            let features = feature_extractor.extract(&map);
            publisher.publish_features(&features, timestamp_us);
            last_feature_time = Instant::now();

            log::debug!(
                "Features: {} lines, {} corners",
                features.lines.len(),
                features.corners.len()
            );
        }

        // Log statistics periodically
        if last_log_time.elapsed() >= Duration::from_secs(10) {
            let (width, height) = map.dimensions();
            let occupied = count_occupied_cells(&map);
            log::info!(
                "Bag: Pose: ({:.2}, {:.2}, {:.1}deg) | Map: {}x{} ({} occupied) | {} scans | {} clients",
                slam_pose.x,
                slam_pose.y,
                slam_pose.theta.to_degrees(),
                width,
                height,
                occupied,
                scans_processed,
                publisher.client_count()
            );
            last_log_time = Instant::now();
        }
    }

    log::info!("Bag playback finished: {} scans processed", scans_processed);
    Ok(())
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Extract occupied cells from occupancy grid as point cloud for scan-to-map matching.
fn map_to_point_cloud(map: &OccupancyGrid) -> PointCloud2D {
    let mut points = Vec::new();
    let (width, height) = map.dimensions();

    for cy in 0..height {
        for cx in 0..width {
            if map.get_state(cx, cy) == CellState::Occupied {
                let (x, y) = map.cell_to_world(cx, cy);
                points.push(dhruva_slam::core::types::Point2D::new(x, y));
            }
        }
    }

    PointCloud2D::from_points(points)
}

/// Count occupied cells in the map.
fn count_occupied_cells(map: &OccupancyGrid) -> usize {
    let (width, height) = map.dimensions();
    let mut count = 0;
    for cy in 0..height {
        for cx in 0..width {
            if map.get_state(cx, cy) == CellState::Occupied {
                count += 1;
            }
        }
    }
    count
}
