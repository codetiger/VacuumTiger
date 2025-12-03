//! dhruva-slam-node daemon
//!
//! Connects to SangamIO, computes odometry and SLAM, and publishes data for visualization.
//!
//! # Usage
//!
//! ```bash
//! # With default config
//! cargo run --bin dhruva-slam-node
//!
//! # With custom config file
//! cargo run --bin dhruva-slam-node -- --config dhruva-slam.toml
//!
//! # With command line overrides
//! cargo run --bin dhruva-slam-node -- --sangam 192.168.68.101:5555 --port 5557
//! ```

use dhruva_slam::{
    ComplementaryConfig, OdometryPipeline, OdometryPipelineConfig, OdometryPublisher, OnlineSlam,
    OnlineSlamConfig, Point2D, PointCloud2D, Pose2D, SangamClient, SlamEngine, SlamMode,
    WheelOdometryConfig,
};
use serde::Deserialize;
use std::fs;
use std::io::Write;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// Configuration file structure
#[derive(Debug, Deserialize, Default)]
struct Config {
    #[serde(default)]
    source: SourceConfig,
    #[serde(default)]
    output: OutputConfig,
    #[serde(default)]
    odometry: OdometryConfig,
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
    slam_status_rate_hz: f32,
    slam_map_rate_hz: f32,
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            bind_port: 5557,
            odometry_rate_hz: 50.0,
            slam_status_rate_hz: 5.0,
            slam_map_rate_hz: 1.0,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct OdometryConfig {
    ticks_per_meter: f32,
    wheel_base: f32,
    alpha: f32,
    gyro_scale: f32,
    gyro_bias_z: f32,
}

impl Default for OdometryConfig {
    fn default() -> Self {
        Self {
            ticks_per_meter: 1000.0,
            wheel_base: 0.17,
            alpha: 0.98,
            // CRL-200S: raw units are 0.1 deg/s, convert to rad/s
            gyro_scale: 0.1 * (std::f32::consts::PI / 180.0),
            gyro_bias_z: 0.0,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct SlamConfig {
    enabled: bool,
    initial_mode: String,
    min_range: f32,
    max_range: f32,
}

impl Default for SlamConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            initial_mode: "Mapping".to_string(),
            min_range: 0.15, // 15cm minimum range (filter close returns)
            max_range: 8.0,  // 8m maximum range
        }
    }
}

/// Command line arguments
struct Args {
    config_path: Option<String>,
    sangam_address: Option<String>,
    bind_port: Option<u16>,
}

fn parse_args() -> Args {
    let args: Vec<String> = std::env::args().collect();
    let mut result = Args {
        config_path: None,
        sangam_address: None,
        bind_port: None,
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
    println!("dhruva-slam-node - SLAM and odometry daemon");
    println!();
    println!("USAGE:");
    println!("    dhruva-slam-node [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    -c, --config <FILE>     Configuration file (dhruva-slam.toml)");
    println!("    -s, --sangam <ADDR>     SangamIO address (192.168.68.101:5555)");
    println!("    -p, --port <PORT>       TCP publish port (5557)");
    println!("    -h, --help              Print help information");
}

fn load_config(args: &Args) -> Config {
    let config = match &args.config_path {
        Some(path) => match fs::read_to_string(path) {
            Ok(contents) => match toml::from_str(&contents) {
                Ok(cfg) => {
                    eprintln!("Loaded config from {}", path);
                    cfg
                }
                Err(e) => {
                    eprintln!("Failed to parse config {}: {}", path, e);
                    Config::default()
                }
            },
            Err(e) => {
                eprintln!("Failed to read config {}: {}", path, e);
                Config::default()
            }
        },
        None => {
            // Try default paths
            for path in &["dhruva-slam.toml", "/etc/dhruva-slam.toml"] {
                if let Ok(contents) = fs::read_to_string(path)
                    && let Ok(cfg) = toml::from_str(&contents)
                {
                    eprintln!("Loaded config from {}", path);
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

/// Convert lidar scan to PointCloud2D
fn lidar_to_pointcloud(scan: &[(f32, f32, u8)], min_range: f32, max_range: f32) -> PointCloud2D {
    let mut cloud = PointCloud2D::with_capacity(scan.len());

    for &(angle, distance, quality) in scan {
        // Filter by range and quality
        if distance < min_range || distance > max_range || quality < 10 {
            continue;
        }

        // Convert polar to Cartesian
        let x = distance * angle.cos();
        let y = distance * angle.sin();
        cloud.push(Point2D::new(x, y));
    }

    cloud
}

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

    // Parse arguments and load config
    let args = parse_args();
    let config = load_config(&args);

    log::info!("dhruva-slam-node starting...");
    log::info!("  SangamIO: {}", config.source.sangam_address);
    log::info!("  Wire format: Protobuf");
    log::info!("  Publish port: {}", config.output.bind_port);
    log::info!("  Output rate: {} Hz", config.output.odometry_rate_hz);
    log::info!("  SLAM enabled: {}", config.slam.enabled);
    if config.slam.enabled {
        log::info!("  SLAM mode: {}", config.slam.initial_mode);
    }

    // Setup signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        log::info!("Received shutdown signal");
        r.store(false, Ordering::Relaxed);
    })
    .expect("Error setting Ctrl-C handler");

    // Run main loop with reconnection
    while running.load(Ordering::Relaxed) {
        if let Err(e) = run_main_loop(&config, running.clone()) {
            log::error!("Main loop error: {}", e);
            if running.load(Ordering::Relaxed) {
                log::info!("Reconnecting in 5 seconds...");
                std::thread::sleep(Duration::from_secs(5));
            }
        }
    }

    log::info!("dhruva-slam-node shutdown complete");
}

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
    log::info!("Connected to SangamIO (format: Protobuf)");

    // Start publisher
    let publisher = OdometryPublisher::new(config.output.bind_port)?;
    log::info!("Publisher listening on port {}", config.output.bind_port);

    // Initialize odometry pipeline
    let pipeline_config = OdometryPipelineConfig {
        wheel_odom: WheelOdometryConfig {
            ticks_per_meter: config.odometry.ticks_per_meter,
            wheel_base: config.odometry.wheel_base,
        },
        filter: ComplementaryConfig {
            alpha: config.odometry.alpha,
            gyro_scale: config.odometry.gyro_scale,
            gyro_bias_z: config.odometry.gyro_bias_z,
        },
        output_rate_hz: config.output.odometry_rate_hz,
        calibration_samples: 1500, // ~3 seconds at 500Hz for gyro bias calibration
    };
    let mut pipeline = OdometryPipeline::new(pipeline_config);

    // Initialize SLAM if enabled
    let mut slam: Option<OnlineSlam> = if config.slam.enabled {
        let mut slam_engine = OnlineSlam::new(OnlineSlamConfig::default());

        // Set initial mode
        let mode = match config.slam.initial_mode.to_lowercase().as_str() {
            "mapping" => SlamMode::Mapping,
            "localization" => SlamMode::Localization,
            "idle" => SlamMode::Idle,
            _ => {
                log::warn!(
                    "Unknown SLAM mode '{}', defaulting to Mapping",
                    config.slam.initial_mode
                );
                SlamMode::Mapping
            }
        };
        slam_engine.set_mode(mode);
        Some(slam_engine)
    } else {
        None
    };

    log::info!("Pipeline initialized");
    log::info!("  ticks_per_meter: {}", config.odometry.ticks_per_meter);
    log::info!("  wheel_base: {} m", config.odometry.wheel_base);
    log::info!("  alpha: {}", config.odometry.alpha);
    log::info!("  gyro_scale: {}", config.odometry.gyro_scale);
    log::info!("Gyro calibration: collecting 1500 samples (~3 seconds)...");

    let mut last_log_time = Instant::now();
    let mut last_status_time = Instant::now();
    let mut last_map_time = Instant::now();
    let mut msg_count = 0u64;
    let mut lidar_count = 0u64;

    // Track previous pose for computing odometry delta
    let mut prev_pose = Pose2D::identity();

    // Rate limiting intervals
    let status_interval = Duration::from_secs_f32(1.0 / config.output.slam_status_rate_hz);
    let map_interval = Duration::from_secs_f32(1.0 / config.output.slam_map_rate_hz);

    // Main processing loop
    while running.load(Ordering::Relaxed) {
        // Receive message from SangamIO (blocking)
        let msg = client.recv()?;
        let timestamp_us = msg.timestamp_us();

        // Process sensor_status messages (odometry)
        if msg.group_id() == "sensor_status" {
            // Extract encoder and gyro data
            let (left, right) = match msg.encoder_ticks() {
                Some(ticks) => ticks,
                None => continue,
            };

            let gyro_yaw = msg.gyro_yaw_raw().unwrap_or(0);

            // Process through pipeline
            if let Some(pose) = pipeline.process(left, right, gyro_yaw, timestamp_us) {
                publisher.publish_pose(&pose, timestamp_us);
                msg_count += 1;
            }

            // Publish diagnostics periodically
            if let Some(diagnostics) = pipeline.diagnostics(timestamp_us) {
                publisher.publish_diagnostics(&diagnostics);
            }
        }
        // Process lidar messages (SLAM)
        else if msg.group_id() == "lidar"
            && let Some(slam_engine) = slam.as_mut()
            && let Some(lidar_data) = msg.as_lidar()
        {
            // Convert lidar scan to point cloud (already filtered by range/quality)
            let scan_cloud = lidar_to_pointcloud(
                lidar_data.data,
                config.slam.min_range,
                config.slam.max_range,
            );

            // Skip if scan is too sparse
            if scan_cloud.len() < 50 {
                continue;
            }

            // Compute odometry delta since last SLAM update
            let current_pose = pipeline.pose();
            let odom_delta = prev_pose.inverse().compose(&current_pose);
            prev_pose = current_pose;

            // Process scan through SLAM
            let result = slam_engine.process_scan(&scan_cloud, &odom_delta, timestamp_us);

            // Publish scan with SLAM-corrected pose
            publisher.publish_slam_scan(&scan_cloud, &result.pose, timestamp_us);

            lidar_count += 1;
        }

        // Publish SLAM status and diagnostics at configured rate
        if let Some(slam_engine) = slam.as_ref()
            && last_status_time.elapsed() >= status_interval
        {
            let status = slam_engine.status();
            publisher.publish_slam_status(&status);

            // Publish diagnostics at the same rate as status
            let diagnostics = slam_engine.diagnostics(timestamp_us);
            publisher.publish_slam_diagnostics(&diagnostics);

            last_status_time = Instant::now();
        }

        // Publish SLAM map at configured rate
        if let Some(slam_engine) = slam.as_mut()
            && last_map_time.elapsed() >= map_interval
        {
            let map = slam_engine.global_map();
            publisher.publish_slam_map(map, timestamp_us);
            last_map_time = Instant::now();
        }

        // Log statistics periodically
        if last_log_time.elapsed() >= Duration::from_secs(10) {
            let pose = pipeline.pose();
            if let Some(slam_engine) = slam.as_ref() {
                let status = slam_engine.status();
                log::info!(
                    "Pose: x={:.3}m y={:.3}m θ={:.1}° | SLAM: {:?} match={:.0}% kf={} | {} clients | {} odom {} lidar",
                    pose.x,
                    pose.y,
                    pose.theta.to_degrees(),
                    status.mode,
                    status.last_match_score * 100.0,
                    status.num_keyframes,
                    publisher.client_count(),
                    msg_count,
                    lidar_count
                );
            } else {
                log::info!(
                    "Pose: x={:.3}m y={:.3}m θ={:.1}° | {} clients | {} msgs",
                    pose.x,
                    pose.y,
                    pose.theta.to_degrees(),
                    publisher.client_count(),
                    msg_count
                );
            }
            last_log_time = Instant::now();
        }
    }

    Ok(())
}
