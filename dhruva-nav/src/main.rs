//! DhruvaNav - Navigation Controller for VacuumTiger
//!
//! A navigation controller that connects to SangamIO, performs autonomous
//! frontier-based exploration, and builds maps using VastuSLAM.
//!
//! ## Multi-Threaded Architecture
//!
//! DhruvaNav uses three threads for concurrent operation:
//!
//! - **Sensor Thread** (~100Hz): Reads UDP sensor data, updates odometry,
//!   checks safety sensors, sends velocity commands
//! - **Mapping Thread** (~5-10Hz): Receives lidar scans, performs scan matching,
//!   updates the occupancy grid map
//! - **Exploration Thread** (~20Hz): Reads the map, detects frontiers,
//!   plans paths, computes velocity commands

#![feature(portable_simd)]

mod client;
mod config;
mod error;
mod exploration;
mod odometry;
mod planning;
mod shared;
mod threads;
mod utils;

use config::DhruvaConfig;
use error::{DhruvaError, Result};
use shared::{SharedDebugViz, SharedGrid, SharedState, SharedTrajectory};
use threads::spawn_threads;

use std::path::Path;
use std::sync::{Arc, RwLock};
use std::time::Duration;
use tracing::{error, info, warn};
use vastu_slam::{GridStorage, MapConfig, Pose2D};

fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive("dhruva_nav=info".parse().unwrap()),
        )
        .init();

    // Parse command line arguments
    let args: Vec<String> = std::env::args().collect();

    let config = if args.len() > 1 && !args[1].starts_with("--") {
        // Load config from file
        let config_path = Path::new(&args[1]);
        info!("Loading configuration from {:?}", config_path);
        DhruvaConfig::load(config_path)?
    } else {
        // Check for --robot argument
        let robot_ip = args
            .iter()
            .position(|a| a == "--robot")
            .and_then(|i| args.get(i + 1))
            .cloned();

        let mut config = if Path::new("dhruva.toml").exists() {
            info!("Loading configuration from dhruva.toml");
            DhruvaConfig::load(Path::new("dhruva.toml"))?
        } else {
            info!("Using default configuration");
            DhruvaConfig::default()
        };

        // Override robot IP if provided
        if let Some(ip) = robot_ip {
            info!("Using robot IP: {}", ip);
            config.connection.robot_ip = ip;
        }

        config
    };

    info!("DhruvaNav v{}", env!("CARGO_PKG_VERSION"));
    info!(
        "Connecting to {}:{}",
        config.connection.robot_ip, config.connection.port
    );

    // Log configuration
    let total_clearance = config.exploration.robot_radius + config.exploration.safety_margin;
    info!(
        "Wall clearance: {:.2}m (robot radius {:.2}m + safety margin {:.2}m)",
        total_clearance, config.exploration.robot_radius, config.exploration.safety_margin
    );

    // Initialize shared state
    // Robot starts at origin - map is built relative to initial position
    let start_pose = Pose2D::new(0.0, 0.0, 0.0);
    info!("Start pose: origin (0, 0, 0°)");

    let shared_state = Arc::new(SharedState::new(start_pose));

    // Initialize shared grid storage
    let map_config = MapConfig::default();
    let initial_grid = GridStorage::new(
        map_config.grid.initial_width,
        map_config.grid.initial_height,
        map_config.grid.resolution,
        vastu_slam::WorldPoint::ZERO,
    );
    let shared_grid: SharedGrid = Arc::new(RwLock::new(initial_grid));

    // Spawn worker threads
    info!("Starting multi-threaded exploration...");
    let handles = spawn_threads(
        config.clone(),
        Arc::clone(&shared_state),
        Arc::clone(&shared_grid),
    )?;

    // Main thread: Monitor and wait for completion
    let check_interval = Duration::from_millis(500);

    loop {
        std::thread::sleep(check_interval);

        // Check for safety stop
        if shared_state.is_safety_stop() {
            warn!(
                "Safety stop: {}",
                shared_state
                    .safety_reason()
                    .unwrap_or_else(|| "unknown".to_string())
            );
            break;
        }

        // Check for exploration complete
        if shared_state.is_exploration_complete() {
            info!("Exploration completed successfully");
            break;
        }

        // Check if threads are still alive
        if handles.sensor.is_finished()
            || handles.mapping.is_finished()
            || handles.exploration.is_finished()
        {
            warn!("A worker thread exited unexpectedly");
            break;
        }
    }

    // Signal shutdown to all threads
    shared_state.signal_shutdown();

    // Wait for threads to finish (with timeout)
    info!("Waiting for threads to finish...");

    let join_timeout = Duration::from_secs(5);
    let join_start = std::time::Instant::now();

    // Join threads
    if let Err(e) = handles.sensor.join() {
        error!("Sensor thread panicked: {:?}", e);
    }

    if join_start.elapsed() < join_timeout
        && let Err(e) = handles.mapping.join()
    {
        error!("Mapping thread panicked: {:?}", e);
    }

    if join_start.elapsed() < join_timeout
        && let Err(e) = handles.exploration.join()
    {
        error!("Exploration thread panicked: {:?}", e);
    }

    // Save the map with trajectory and debug visualization
    info!("Saving map...");
    save_map(
        &config,
        &shared_grid,
        &handles.trajectory,
        &handles.debug_viz,
    )?;

    info!("DhruvaNav finished");
    Ok(())
}

/// Save the map to files.
fn save_map(
    config: &DhruvaConfig,
    shared_grid: &SharedGrid,
    shared_trajectory: &SharedTrajectory,
    shared_debug_viz: &SharedDebugViz,
) -> Result<()> {
    let grid_guard = shared_grid
        .read()
        .map_err(|e| DhruvaError::Slam(format!("Failed to lock grid: {}", e)))?;
    let grid: &GridStorage = &grid_guard;

    // Get statistics
    let counts = grid.count_by_type();
    let explored_area = (counts.floor + counts.wall) as f32 * grid.resolution() * grid.resolution();
    info!(
        "Map statistics: {:.2}m² explored, {} floor cells, {} wall cells",
        explored_area, counts.floor, counts.wall
    );

    // Create output directory if needed
    let map_path = Path::new(&config.output.map_path);
    if let Some(parent) = map_path.parent() {
        std::fs::create_dir_all(parent)
            .map_err(|e| DhruvaError::Slam(format!("Failed to create output directory: {}", e)))?;
    }

    // Save .vastu file
    vastu_slam::io::save_vastu(grid, map_path)
        .map_err(|e| DhruvaError::Slam(format!("Failed to save map: {}", e)))?;
    info!("Map saved to {:?}", map_path);

    // Read trajectory for SVG visualization
    let trajectory = shared_trajectory
        .read()
        .map_err(|e| DhruvaError::Slam(format!("Failed to lock trajectory: {}", e)))?
        .clone();

    // Read debug visualization data
    let debug_viz = shared_debug_viz
        .read()
        .map_err(|e| DhruvaError::Slam(format!("Failed to lock debug viz: {}", e)))?
        .clone();

    info!(
        "Visualization data: {} trajectory poses, {} frontier targets, {} waypoints",
        trajectory.len(),
        debug_viz.visited_frontiers.len(),
        debug_viz.waypoints.len()
    );

    // Save SVG visualization with trajectory and debug info
    let svg_path = Path::new(&config.output.svg_path);
    let svg_config = vastu_slam::io::SvgConfig {
        scale: 50.0,
        trajectory_width: 2.0,
        ..Default::default()
    };

    // Calculate markers at 0.5m intervals along trajectory
    let markers = vastu_slam::io::markers_by_distance(&trajectory, 0.5);

    let visualizer = vastu_slam::io::SvgVisualizer::new(grid.clone(), svg_config)
        .with_title("DhruvaNav Exploration")
        .with_estimated(trajectory, markers)
        .with_frontiers(debug_viz.visited_frontiers)
        .with_waypoints(debug_viz.waypoints);

    visualizer
        .save(svg_path)
        .map_err(|e| DhruvaError::Slam(format!("Failed to save SVG: {}", e)))?;
    info!("SVG saved to {:?}", svg_path);

    Ok(())
}
