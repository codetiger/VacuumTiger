//! Scenario-based SLAM example using SangamIO mock device.
//!
//! This example demonstrates:
//! - Loading a scenario file (YAML) with movement commands
//! - Running the SangamIO mock device simulation in-process
//! - Building a map using VastuSLAM from simulated lidar scans
//! - Exporting the result as SVG visualization
//!
//! # Usage
//!
//! ```bash
//! cargo run --example scenario_slam -- scenarios/square_path.yaml
//! ```
//!
//! The output SVG will be saved to the `output/` directory.

use std::collections::HashMap;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sangam_io::config::DeviceConfig;
use sangam_io::core::driver::DeviceDriver;
use sangam_io::core::types::{
    Command as SangamCommand, ComponentAction, SensorGroupData, SensorValue,
};
use sangam_io::devices::mock::MockDriver;
use sangam_io::devices::mock::config::SimulationConfig;

use vastu_slam::io::{Scenario, SvgConfig, SvgVisualizer, markers_by_distance};
use vastu_slam::matching::CorrelativeMatcherConfig;
use vastu_slam::{LidarScan, MapConfig, OccupancyGridMap, Pose2D};

/// Convert SangamIO PointCloud2D to VastuSLAM LidarScan
fn to_lidar_scan(points: &[(f32, f32, u8)]) -> LidarScan {
    let angles: Vec<f32> = points.iter().map(|(a, _, _)| *a).collect();
    let ranges: Vec<f32> = points.iter().map(|(_, d, _)| *d).collect();
    LidarScan::new(ranges, angles, 0.15, 8.0)
}

/// Get lidar scan from sensor data if available
fn get_lidar_scan(
    sensor_data: &HashMap<String, Arc<Mutex<SensorGroupData>>>,
    last_seq: &mut u64,
) -> Option<LidarScan> {
    let lidar = sensor_data.get("lidar")?;
    let data = lidar.lock().ok()?;

    // Only return if new data (sequence number changed)
    if data.sequence_number == *last_seq {
        return None;
    }
    *last_seq = data.sequence_number;

    if let Some(SensorValue::PointCloud2D(points)) = data.values.get("scan") {
        if !points.is_empty() {
            return Some(to_lidar_scan(points));
        }
    }
    None
}

/// Compute odometry pose from wheel encoders
fn compute_odometry(
    sensor_data: &HashMap<String, Arc<Mutex<SensorGroupData>>>,
    prev_left: &mut u16,
    prev_right: &mut u16,
    pose: &mut Pose2D,
    ticks_per_meter: f32,
    wheel_base: f32,
) {
    let sensor_status = match sensor_data.get("sensor_status") {
        Some(s) => s,
        None => return,
    };

    let data = match sensor_status.lock() {
        Ok(d) => d,
        Err(_) => return,
    };

    let left = match data.values.get("wheel_left") {
        Some(SensorValue::U16(v)) => *v,
        _ => return,
    };
    let right = match data.values.get("wheel_right") {
        Some(SensorValue::U16(v)) => *v,
        _ => return,
    };

    // Calculate tick deltas (handle wraparound)
    let dl = left.wrapping_sub(*prev_left) as i16;
    let dr = right.wrapping_sub(*prev_right) as i16;
    *prev_left = left;
    *prev_right = right;

    // Skip if no movement
    if dl == 0 && dr == 0 {
        return;
    }

    // Convert ticks to meters
    let dist_left = dl as f32 / ticks_per_meter;
    let dist_right = dr as f32 / ticks_per_meter;

    // Differential drive kinematics
    let dist = (dist_left + dist_right) / 2.0;
    let dtheta = (dist_right - dist_left) / wheel_base;

    // Update pose
    pose.x += dist * pose.theta.cos();
    pose.y += dist * pose.theta.sin();
    pose.theta += dtheta;
}

/// Create a command to enable lidar
fn lidar_enable() -> SangamCommand {
    SangamCommand::ComponentControl {
        id: "lidar".to_string(),
        action: ComponentAction::Enable { config: None },
    }
}

/// Create a command to enable drive
fn drive_enable() -> SangamCommand {
    SangamCommand::ComponentControl {
        id: "drive".to_string(),
        action: ComponentAction::Enable { config: None },
    }
}

/// Create a command to set velocity
fn set_velocity(linear: f32, angular: f32) -> SangamCommand {
    let mut config = HashMap::new();
    config.insert("linear".to_string(), SensorValue::F32(linear));
    config.insert("angular".to_string(), SensorValue::F32(angular));
    SangamCommand::ComponentControl {
        id: "drive".to_string(),
        action: ComponentAction::Configure { config },
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    // Parse CLI args
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <scenario.yaml> [--speed-factor N]", args[0]);
        eprintln!(
            "Example: {} scenarios/square_path.yaml --speed-factor 10",
            args[0]
        );
        std::process::exit(1);
    }

    let scenario_path = Path::new(&args[1]);
    let speed_factor = args
        .iter()
        .position(|a| a == "--speed-factor")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse::<f32>().ok())
        .unwrap_or(5.0); // Default 5x speed for faster simulation

    // Load scenario
    log::info!("Loading scenario from {:?}", scenario_path);
    let scenario = Scenario::load(scenario_path)?;
    log::info!(
        "Scenario: {} ({} commands, {:.1}s total)",
        scenario.name,
        scenario.commands.len(),
        scenario.total_duration_secs()
    );

    // Resolve map path (scenario map name -> maps/NAME.yaml)
    let maps_dir = scenario_path
        .parent()
        .unwrap_or(Path::new("."))
        .join("../maps");
    let map_file = maps_dir.join(format!("{}.yaml", scenario.map));
    log::info!("Using map: {:?}", map_file);

    // Get start pose from scenario or use defaults
    let (start_x, start_y, start_theta) = scenario
        .start_pose
        .as_ref()
        .map(|p| (p.x, p.y, p.theta))
        .unwrap_or((4.0, 4.0, 0.0));

    // Create simulation config
    let sim_config = SimulationConfig {
        map_file: map_file.to_string_lossy().to_string(),
        start_x,
        start_y,
        start_theta,
        speed_factor,
        random_seed: 42, // Deterministic for reproducibility
        ..Default::default()
    };

    // Create device config
    let device_config = DeviceConfig {
        device_type: "mock".to_string(),
        name: "Scenario SLAM Simulation".to_string(),
        hardware: None,
        simulation: Some(sim_config),
    };

    // Initialize mock driver
    log::info!("Initializing mock driver (speed factor: {}x)", speed_factor);
    let mut driver = MockDriver::new(device_config)?;
    let init_result = driver.initialize()?;
    let sensor_data = init_result.sensor_data;

    // Enable lidar and drive
    driver.send_command(lidar_enable())?;
    driver.send_command(drive_enable())?;

    // Wait for first lidar scan
    log::info!("Waiting for first lidar scan...");
    std::thread::sleep(Duration::from_millis(500));

    // Create SLAM map
    let map_config = MapConfig::default();
    let mut map = OccupancyGridMap::new(map_config);

    // Configure matcher with correct lidar offset (must match SensorConfig)
    let mut matcher_config = CorrelativeMatcherConfig::default();
    matcher_config.sensor_offset = (-0.110, 0.0); // Lidar 11cm behind robot center

    // Tracking state
    let mut trajectory: Vec<Pose2D> = vec![];
    let mut current_pose = Pose2D::new(start_x, start_y, start_theta);
    let mut prev_wheel_left: u16 = 0;
    let mut prev_wheel_right: u16 = 0;
    let mut last_lidar_seq: u64 = 0;

    // Robot parameters (CRL-200S defaults)
    let ticks_per_meter = 4464.0;
    let wheel_base = 0.233;

    // Initialize wheel encoder baseline
    if let Some(sensor_status) = sensor_data.get("sensor_status") {
        if let Ok(data) = sensor_status.lock() {
            if let Some(SensorValue::U16(l)) = data.values.get("wheel_left") {
                prev_wheel_left = *l;
            }
            if let Some(SensorValue::U16(r)) = data.values.get("wheel_right") {
                prev_wheel_right = *r;
            }
        }
    }

    // Seed the map with initial scans (rotate to get 360° coverage)
    log::info!("Seeding map with {} scans...", scenario.seed.scans);
    if scenario.seed.rotation {
        // Rotate slowly while collecting scans
        driver.send_command(set_velocity(0.0, 0.3))?;
    }

    let seed_start = Instant::now();
    let seed_duration_ms = (scenario.seed.scans as f32 * 200.0 / speed_factor) as u64;

    while seed_start.elapsed().as_millis() < seed_duration_ms as u128 {
        // Update odometry
        compute_odometry(
            &sensor_data,
            &mut prev_wheel_left,
            &mut prev_wheel_right,
            &mut current_pose,
            ticks_per_meter,
            wheel_base,
        );

        // During seeding, insert scans directly without matching
        // (matching against empty/sparse map gives poor results)
        if let Some(scan) = get_lidar_scan(&sensor_data, &mut last_lidar_seq) {
            map.observe_lidar(&scan, current_pose);
            trajectory.push(current_pose);
        }

        std::thread::sleep(Duration::from_millis(20));
    }

    // Stop rotation
    driver.send_command(set_velocity(0.0, 0.0))?;
    std::thread::sleep(Duration::from_millis(100));

    log::info!(
        "Map seeded with {} scans, {} cells known",
        trajectory.len(),
        map.coverage_stats().known_cells
    );

    // Execute scenario commands
    log::info!("Executing {} scenario commands...", scenario.commands.len());

    for (i, cmd) in scenario.commands.iter().enumerate() {
        let (linear, angular) = cmd.velocities();
        let duration_ms = cmd.duration_ms();
        let sim_duration_ms = (duration_ms as f32 / speed_factor) as u64;

        log::debug!(
            "Command {}/{}: v={:.2} m/s, ω={:.2} rad/s, duration={}ms (sim: {}ms)",
            i + 1,
            scenario.commands.len(),
            linear,
            angular,
            duration_ms,
            sim_duration_ms
        );

        // Send velocity command
        driver.send_command(set_velocity(linear, angular))?;

        let cmd_start = Instant::now();
        while cmd_start.elapsed().as_millis() < sim_duration_ms as u128 {
            // Update odometry
            compute_odometry(
                &sensor_data,
                &mut prev_wheel_left,
                &mut prev_wheel_right,
                &mut current_pose,
                ticks_per_meter,
                wheel_base,
            );

            // Process lidar scan with matching
            if let Some(scan) = get_lidar_scan(&sensor_data, &mut last_lidar_seq) {
                let (_result, corrected_pose, match_result) =
                    map.observe_lidar_with_matching(&scan, current_pose, &matcher_config);

                if match_result.converged && match_result.score >= 0.5 {
                    current_pose = corrected_pose;
                    log::trace!("Match accepted: score={:.3}", match_result.score);
                } else {
                    log::debug!(
                        "Match rejected: score={:.3}, converged={}",
                        match_result.score,
                        match_result.converged
                    );
                }
                trajectory.push(current_pose);
            }

            std::thread::sleep(Duration::from_millis(20));
        }
    }

    // Stop robot
    driver.send_command(set_velocity(0.0, 0.0))?;

    // Report results
    let stats = map.coverage_stats();
    log::info!(
        "SLAM complete: {} poses, {:.2}m² explored, {} wall cells",
        trajectory.len(),
        stats.explored_area_m2,
        stats.wall_cells
    );

    // Create output directory
    let output_dir = Path::new("output");
    std::fs::create_dir_all(output_dir)?;

    // Export SVG
    let svg_filename = scenario.svg_filename();
    let svg_path = output_dir.join(&svg_filename);

    let svg_config = SvgConfig {
        scale: scenario.svg_output.scale,
        trajectory_width: scenario.svg_output.trajectory_width,
        ..Default::default()
    };

    let markers = markers_by_distance(
        &trajectory,
        scenario.svg_output.marker_interval_m.unwrap_or(0.5),
    );

    let visualizer = SvgVisualizer::new(map.storage().clone(), svg_config)
        .with_title(&scenario.name)
        .with_estimated(trajectory, markers);

    visualizer.save(&svg_path)?;

    log::info!("SVG saved to: {:?}", svg_path);

    Ok(())
}
