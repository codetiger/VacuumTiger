//! Mock exploration example with encoder odometry and scan matching
//!
//! Demonstrates autonomous frontier-based exploration using a simulated robot
//! with realistic encoder drift and scan-to-map matching for correction.
//!
//! Usage:
//!   cargo run --example explore_mock -- --map simple_room
//!   cargo run --example explore_mock  # Uses random map
//!
//! Enable debug logging to see drift metrics:
//!   RUST_LOG=info cargo run --example explore_mock

use clap::Parser;
use rand::prelude::*;
use std::f32::consts::{PI, TAU};
use std::path::Path;
use std::time::Instant;

// SangamIO mock simulation components
use sangam_io::devices::mock::{
    config::{EncoderConfig, LidarConfig, RobotConfig},
    encoder_sim::EncoderSimulator,
    lidar_sim::LidarSimulator,
    map_loader::SimulationMap,
    noise::NoiseGenerator,
    physics::PhysicsState,
};

// Vastu-map components
use vastu_map::{
    LidarScan, OccupancyGridMap, Pose2D, VastuConfig,
    exploration::ExplorationController,
    io::{export_ros_map, save_vastu},
    slam::{CorrelativeMatcher, CorrelativeMatcherConfig},
};

/// Mock exploration example
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Map name to load from maps/ folder (e.g., "simple_room")
    /// If not specified, picks a random map
    #[arg(short, long)]
    map: Option<String>,

    /// Configuration file path
    #[arg(short, long, default_value = "configs/config.yaml")]
    config: String,

    /// Output directory for saved maps
    #[arg(short, long, default_value = "./output")]
    output: String,

    /// Maximum simulation steps (0 = unlimited)
    #[arg(long, default_value = "10000")]
    max_steps: usize,

    /// Simulation speed factor (1.0 = realtime)
    #[arg(long, default_value = "10.0")]
    speed: f32,

    /// Show progress every N steps
    #[arg(long, default_value = "100")]
    progress_interval: usize,
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    // List available maps
    let maps_dir = Path::new("maps");
    let available_maps = list_available_maps(maps_dir);

    if available_maps.is_empty() {
        eprintln!("No maps found in maps/ folder");
        std::process::exit(1);
    }

    // Select map
    let map_name = args.map.unwrap_or_else(|| {
        let mut rng = rand::rng();
        let selected = available_maps.choose(&mut rng).unwrap().clone();
        println!("Randomly selected map: {}", selected);
        selected
    });

    let map_yaml = maps_dir.join(format!("{}.yaml", map_name));
    if !map_yaml.exists() {
        eprintln!("Map not found: {}", map_yaml.display());
        eprintln!("Available maps: {:?}", available_maps);
        std::process::exit(1);
    }

    // Load configuration
    let config_path = Path::new(&args.config);
    let vastu_config = if config_path.exists() {
        VastuConfig::load(config_path).unwrap_or_else(|e| {
            eprintln!("Warning: Failed to load config: {}, using defaults", e);
            VastuConfig::default()
        })
    } else {
        println!("Config not found, using defaults");
        VastuConfig::default()
    };

    // Load simulation map
    println!("Loading map: {}", map_yaml.display());
    let sim_map = SimulationMap::load(&map_yaml).expect("Failed to load simulation map");
    println!(
        "Map size: {}x{} pixels, resolution: {}m/px",
        sim_map.width(),
        sim_map.height(),
        sim_map.resolution()
    );

    // Find starting position (search for free space in grid bounds)
    let start_pose = find_free_start_position(&sim_map, &vastu_config);
    println!(
        "Starting at: ({:.2}, {:.2}, {:.1}°)",
        start_pose.x,
        start_pose.y,
        start_pose.theta.to_degrees()
    );

    // Create output directory
    std::fs::create_dir_all(&args.output).ok();

    // Run exploration
    let result = run_exploration(
        &sim_map,
        start_pose,
        &vastu_config,
        args.max_steps,
        args.speed,
        args.progress_interval,
    );

    // Save map
    let output_base = Path::new(&args.output).join(&map_name);

    println!("\nSaving maps...");
    save_vastu(result.map.storage(), &output_base.with_extension("vastu"))
        .expect("Failed to save .vastu file");
    println!("  Saved: {}.vastu", output_base.display());

    export_ros_map(result.map.storage(), &output_base).expect("Failed to save ROS map");
    println!(
        "  Saved: {}.pgm + {}.yaml",
        output_base.display(),
        output_base.display()
    );

    // Print summary
    println!("\n=== Exploration Summary ===");
    println!("Status: {:?}", result.status);
    println!("Steps: {}", result.steps);
    println!("Time: {:.1}s", result.elapsed_secs);
    println!(
        "Distance traveled: {:.2}m",
        result.progress.distance_traveled
    );
    println!("Frontiers explored: {}", result.progress.frontiers_explored);

    let map_stats = result.map.coverage_stats();
    println!(
        "Cells explored: {} floor, {} wall, {} unknown",
        map_stats.floor_cells, map_stats.wall_cells, map_stats.unknown_cells
    );
    println!("Coverage: {:.2}m²", map_stats.explored_area_m2);

    // Drift and scan matching statistics
    println!("\n=== Scan Matching Stats ===");
    let ds = &result.drift_stats;
    let total_matches = ds.successful_matches + ds.failed_matches;
    let success_rate = if total_matches > 0 {
        100.0 * ds.successful_matches as f32 / total_matches as f32
    } else {
        0.0
    };
    println!(
        "Matches: {} successful, {} failed ({:.1}% success)",
        ds.successful_matches, ds.failed_matches, success_rate
    );
    println!(
        "Total drift corrected: {:.3}m translation, {:.2}° rotation",
        ds.total_translation_drift,
        ds.total_rotation_drift.to_degrees()
    );
    println!(
        "Max drift error: {:.3}m translation, {:.2}° rotation",
        ds.max_translation_error,
        ds.max_rotation_error.to_degrees()
    );
}

/// List available map names in directory
fn list_available_maps(dir: &Path) -> Vec<String> {
    let mut maps = Vec::new();
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().is_some_and(|e| e == "yaml") {
                if let Some(stem) = path.file_stem() {
                    maps.push(stem.to_string_lossy().to_string());
                }
            }
        }
    }
    maps.sort();
    maps
}

/// Find a free starting position in the map
/// Constrains to the overlapping region of simulation map and vastu-map grid
fn find_free_start_position(map: &SimulationMap, vastu_config: &VastuConfig) -> Pose2D {
    let mut rng = rand::rng();
    let sim_origin = map.origin();
    let sim_width_m = map.width() as f32 * map.resolution();
    let sim_height_m = map.height() as f32 * map.resolution();

    // Vastu-map grid bounds (centered)
    let grid_size_x = vastu_config.grid.initial_width as f32 * vastu_config.grid.resolution;
    let grid_size_y = vastu_config.grid.initial_height as f32 * vastu_config.grid.resolution;
    let grid_min_x = -grid_size_x / 2.0;
    let grid_min_y = -grid_size_y / 2.0;
    let grid_max_x = grid_size_x / 2.0;
    let grid_max_y = grid_size_y / 2.0;

    // Find overlap region
    let min_x = sim_origin.0.max(grid_min_x) + 0.5;
    let min_y = sim_origin.1.max(grid_min_y) + 0.5;
    let max_x = (sim_origin.0 + sim_width_m).min(grid_max_x) - 0.5;
    let max_y = (sim_origin.1 + sim_height_m).min(grid_max_y) - 0.5;

    // Try random positions within overlap region
    for _ in 0..1000 {
        let x = min_x + rng.random::<f32>() * (max_x - min_x);
        let y = min_y + rng.random::<f32>() * (max_y - min_y);

        // Check if position is free (not occupied, with margin)
        if !map.is_occupied(x, y)
            && !map.is_occupied(x + 0.2, y)
            && !map.is_occupied(x - 0.2, y)
            && !map.is_occupied(x, y + 0.2)
            && !map.is_occupied(x, y - 0.2)
        {
            let theta = rng.random::<f32>() * TAU;
            return Pose2D::new(x, y, theta);
        }
    }

    // Fallback: scan for any free position in overlap
    for py in 0..map.height() {
        for px in 0..map.width() {
            let (x, y) = map.pixel_to_world(px, py);
            if x >= min_x && x <= max_x && y >= min_y && y <= max_y && !map.is_occupied(x, y) {
                return Pose2D::new(x, y, 0.0);
            }
        }
    }

    // Last resort: center of overlap
    Pose2D::new((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, 0.0)
}

/// Exploration result
struct ExplorationResult {
    map: OccupancyGridMap,
    status: ExplorationStatus,
    steps: usize,
    elapsed_secs: f32,
    progress: vastu_map::exploration::ExplorationProgress,
    drift_stats: DriftStats,
}

/// Drift statistics for encoder vs corrected pose
#[derive(Debug, Default)]
struct DriftStats {
    /// Total translation drift (meters)
    total_translation_drift: f32,
    /// Total rotation drift (radians)
    total_rotation_drift: f32,
    /// Maximum translation error observed
    max_translation_error: f32,
    /// Maximum rotation error observed
    max_rotation_error: f32,
    /// Number of successful scan matches
    successful_matches: usize,
    /// Number of failed scan matches
    failed_matches: usize,
}

#[derive(Debug)]
enum ExplorationStatus {
    Complete,
    MaxStepsReached,
    #[allow(dead_code)]
    Failed(String),
}

/// Simple wheel odometry tracker
///
/// Computes pose from encoder ticks using differential drive kinematics.
struct WheelOdometry {
    wheel_base: f32,
    ticks_per_meter: f32,
    last_left: i32,
    last_right: i32,
    pose: Pose2D,
    initialized: bool,
}

impl WheelOdometry {
    fn new(wheel_base: f32, ticks_per_meter: f32, initial_pose: Pose2D) -> Self {
        Self {
            wheel_base,
            ticks_per_meter,
            last_left: 0,
            last_right: 0,
            pose: initial_pose,
            initialized: false,
        }
    }

    /// Update odometry from encoder ticks, returns pose delta
    fn update(&mut self, left_ticks: u16, right_ticks: u16) -> Pose2D {
        let left = left_ticks as i32;
        let right = right_ticks as i32;

        if !self.initialized {
            self.last_left = left;
            self.last_right = right;
            self.initialized = true;
            return Pose2D::new(0.0, 0.0, 0.0);
        }

        // Handle wrapping (u16 encoder ticks wrap at 65535)
        let delta_left = (left.wrapping_sub(self.last_left) as i16) as i32;
        let delta_right = (right.wrapping_sub(self.last_right) as i16) as i32;

        self.last_left = left;
        self.last_right = right;

        // Convert to meters
        let left_dist = delta_left as f32 / self.ticks_per_meter;
        let right_dist = delta_right as f32 / self.ticks_per_meter;

        // Differential drive kinematics
        let delta_theta = (right_dist - left_dist) / self.wheel_base;
        let delta_linear = (left_dist + right_dist) / 2.0;

        // Robot-frame delta
        let (dx, dy) = if delta_theta.abs() < 1e-6 {
            (delta_linear, 0.0)
        } else {
            let radius = delta_linear / delta_theta;
            (
                radius * delta_theta.sin(),
                radius * (1.0 - delta_theta.cos()),
            )
        };

        // Transform to world frame
        let (sin_t, cos_t) = self.pose.theta.sin_cos();
        let dx_world = dx * cos_t - dy * sin_t;
        let dy_world = dx * sin_t + dy * cos_t;

        // Update pose
        self.pose = Pose2D::new(
            self.pose.x + dx_world,
            self.pose.y + dy_world,
            normalize_angle(self.pose.theta + delta_theta),
        );

        Pose2D::new(dx, dy, delta_theta)
    }

    fn pose(&self) -> Pose2D {
        self.pose
    }

    /// Reset odometry to a corrected pose (after scan matching)
    fn reset_to(&mut self, pose: Pose2D) {
        self.pose = pose;
    }
}

/// Run the exploration simulation
fn run_exploration(
    sim_map: &SimulationMap,
    start_pose: Pose2D,
    config: &VastuConfig,
    max_steps: usize,
    speed_factor: f32,
    progress_interval: usize,
) -> ExplorationResult {
    // Create occupancy grid map
    let map_config = config.to_map_config();
    let mut map = OccupancyGridMap::new(map_config);

    // Create exploration controller
    let explore_config = config.to_exploration_config();
    let mut explorer = ExplorationController::new(explore_config);

    // Create physics simulation (ground truth)
    let robot_config = RobotConfig::default();
    let mut physics =
        PhysicsState::new(start_pose.x, start_pose.y, start_pose.theta, &robot_config);

    // Create encoder simulator (with noise/slip for realistic odometry)
    let encoder_config = EncoderConfig::default();
    let encoder_noise = NoiseGenerator::new(42);
    let mut encoder =
        EncoderSimulator::new(&encoder_config, robot_config.ticks_per_meter, encoder_noise);

    // Create wheel odometry tracker
    let mut odometry = WheelOdometry::new(
        robot_config.wheel_base,
        robot_config.ticks_per_meter,
        start_pose,
    );

    // Create lidar simulator
    let lidar_config = LidarConfig::default();

    // Create scan-to-map matcher with larger search window for encoder drift
    // Also set sensor offset to match lidar mounting position for accurate alignment
    let matcher_config = CorrelativeMatcherConfig {
        search_x: 0.5,     // ±0.5m (default: 0.3m)
        search_y: 0.5,     // ±0.5m
        search_theta: 1.0, // ±1.0 rad (~57°) - handles large rotation errors
        sensor_offset: (lidar_config.mounting_x, lidar_config.mounting_y),
        ..CorrelativeMatcherConfig::default()
    };
    let matcher = CorrelativeMatcher::new(matcher_config);
    let lidar_noise = NoiseGenerator::new(43); // Different seed for lidar
    let mut lidar = LidarSimulator::new(&lidar_config, lidar_noise);

    // Simulation parameters
    let dt = 0.05; // 20Hz physics
    let lidar_interval = 0.2; // 5Hz lidar
    let mut time_since_lidar = lidar_interval; // Trigger immediately
    let _ = speed_factor;

    // Velocity state
    let mut linear_vel = 0.0f32;
    let mut angular_vel = 0.0f32;

    // Corrected pose (from scan matching)
    let mut corrected_pose = start_pose;

    // Drift statistics
    let mut drift_stats = DriftStats::default();

    // Start exploration
    explorer.start();
    let start_time = Instant::now();

    for step in 0..max_steps.max(1) {
        // Update encoder simulation based on wheel velocities
        let (left_vel, right_vel) =
            physics.wheel_velocities(linear_vel, angular_vel, robot_config.wheel_base);
        let (left_ticks, right_ticks) = encoder.update(left_vel, right_vel, dt);

        // Update wheel odometry from encoder ticks
        odometry.update(left_ticks, right_ticks);
        let odom_pose = odometry.pose();

        // Generate lidar scan periodically
        time_since_lidar += dt;
        if time_since_lidar >= lidar_interval {
            time_since_lidar = 0.0;

            // Generate scan at ground truth position (what sensor actually sees)
            let gt_pose = Pose2D::new(physics.x(), physics.y(), physics.theta());
            let scan_data = lidar.generate_scan(sim_map, gt_pose.x, gt_pose.y, gt_pose.theta);

            if !scan_data.is_empty() {
                let ranges: Vec<f32> = scan_data.iter().map(|(_, d, _)| *d).collect();
                // Include lidar angle_offset so map observation matches ray-cast direction
                let angles: Vec<f32> = scan_data
                    .iter()
                    .map(|(a, _, _)| a + lidar_config.angle_offset)
                    .collect();

                let scan = LidarScan::new(
                    ranges,
                    angles,
                    lidar_config.min_range,
                    lidar_config.max_range,
                );

                // Use encoder odometry as initial guess for scan matching
                let match_result = matcher.match_scan(&scan, odom_pose, map.storage());

                // Check map has enough structure for reliable matching
                let map_stats = map.coverage_stats();
                let has_enough_structure = map_stats.wall_cells > 100;

                // Accept match if converged with reasonable score
                // Lower thresholds for closed-loop SLAM where map quality depends on matching
                let min_hit_ratio = if has_enough_structure { 0.25 } else { 0.35 };
                let accept_match =
                    match_result.converged && match_result.hit_ratio() > min_hit_ratio;

                // Use matched pose if successful, otherwise use odometry
                if accept_match {
                    corrected_pose = match_result.pose;
                    drift_stats.successful_matches += 1;

                    // Reset odometry to corrected pose to prevent drift accumulation
                    odometry.reset_to(corrected_pose);

                    // Track drift: difference between encoder and corrected pose
                    let dx = odom_pose.x - corrected_pose.x;
                    let dy = odom_pose.y - corrected_pose.y;
                    let trans_error = (dx * dx + dy * dy).sqrt();
                    let rot_error = (odom_pose.theta - corrected_pose.theta).abs();

                    drift_stats.total_translation_drift += trans_error;
                    drift_stats.total_rotation_drift += rot_error;
                    drift_stats.max_translation_error =
                        drift_stats.max_translation_error.max(trans_error);
                    drift_stats.max_rotation_error = drift_stats.max_rotation_error.max(rot_error);

                    log::debug!(
                        "[ScanMatch] SUCCESS: odom=({:.2},{:.2},{:.1}°) → corrected=({:.2},{:.2},{:.1}°), error={:.3}m, hits={:.0}%",
                        odom_pose.x,
                        odom_pose.y,
                        odom_pose.theta.to_degrees(),
                        corrected_pose.x,
                        corrected_pose.y,
                        corrected_pose.theta.to_degrees(),
                        trans_error,
                        match_result.hit_ratio() * 100.0
                    );
                } else {
                    // Match failed or not trusted - use odometry pose
                    corrected_pose = odom_pose;
                    drift_stats.failed_matches += 1;

                    log::warn!(
                        "[ScanMatch] FAILED: using odom pose ({:.2},{:.2},{:.1}°), score={:.2}",
                        odom_pose.x,
                        odom_pose.y,
                        odom_pose.theta.to_degrees(),
                        match_result.score
                    );
                }

                // Update map with estimated pose (closed-loop SLAM)
                map.observe_lidar(&scan, corrected_pose);
            }
        }

        // Update exploration controller with corrected pose
        let command = explorer.update(corrected_pose, map.storage());

        // Convert command to velocities
        // Note: We use corrected_pose for control, but physics uses ground truth internally
        match command {
            vastu_map::exploration::ExplorationCommand::MoveTo { target, max_speed } => {
                let (lin, ang) = compute_velocity_to_target(
                    corrected_pose.x,
                    corrected_pose.y,
                    corrected_pose.theta,
                    target.x,
                    target.y,
                    max_speed,
                    2.0, // max angular speed
                );
                linear_vel = lin;
                angular_vel = ang;
            }
            vastu_map::exploration::ExplorationCommand::Rotate {
                target_heading,
                max_angular_speed,
            } => {
                linear_vel = 0.0;
                angular_vel = compute_angular_velocity(
                    corrected_pose.theta,
                    target_heading,
                    max_angular_speed,
                );
            }
            vastu_map::exploration::ExplorationCommand::Stop => {
                linear_vel = 0.0;
                angular_vel = 0.0;
            }
            vastu_map::exploration::ExplorationCommand::ExplorationComplete => {
                println!("\nExploration complete!");
                return ExplorationResult {
                    map,
                    status: ExplorationStatus::Complete,
                    steps: step,
                    elapsed_secs: start_time.elapsed().as_secs_f32(),
                    progress: explorer.progress(),
                    drift_stats,
                };
            }
            vastu_map::exploration::ExplorationCommand::ExplorationFailed { reason } => {
                println!("\nExploration failed: {}", reason);
                return ExplorationResult {
                    map,
                    status: ExplorationStatus::Failed(reason),
                    steps: step,
                    elapsed_secs: start_time.elapsed().as_secs_f32(),
                    progress: explorer.progress(),
                    drift_stats,
                };
            }
            vastu_map::exploration::ExplorationCommand::None => {
                // Continue with current velocities
            }
        }

        // Update physics and check for collision
        let collision = physics.update(dt, linear_vel, angular_vel, sim_map, &robot_config);
        if collision {
            // Trigger obstacle detection event for the exploration controller
            explorer.handle_event(vastu_map::exploration::ExplorationEvent::ObstacleDetected);
        }

        // Progress output - also show when exploration changes state
        let progress = explorer.progress();
        if progress_interval > 0 && step % progress_interval == 0 && step > 0 {
            // Compare ground truth vs corrected pose for drift visualization
            let gt_pose = Pose2D::new(physics.x(), physics.y(), physics.theta());
            let pos_error = ((gt_pose.x - corrected_pose.x).powi(2)
                + (gt_pose.y - corrected_pose.y).powi(2))
            .sqrt();

            println!(
                "[Step {}] State: {}, Frontiers: {}, Dist: {:.1}m, Matches: {}/{}, PosErr: {:.3}m",
                step,
                progress.state,
                progress.frontiers_explored,
                progress.distance_traveled,
                drift_stats.successful_matches,
                drift_stats.successful_matches + drift_stats.failed_matches,
                pos_error
            );
        }

        // Debug: show completion or state changes
        if matches!(
            command,
            vastu_map::exploration::ExplorationCommand::ExplorationComplete
        ) {
            println!("[Step {}] Exploration complete!", step);
        }
    }

    ExplorationResult {
        map,
        status: ExplorationStatus::MaxStepsReached,
        steps: max_steps,
        elapsed_secs: start_time.elapsed().as_secs_f32(),
        progress: explorer.progress(),
        drift_stats,
    }
}

/// Compute velocity command to move towards target
///
/// Uses proportional control with concurrent linear + angular movement
/// for smooth path following.
fn compute_velocity_to_target(
    x: f32,
    y: f32,
    theta: f32,
    target_x: f32,
    target_y: f32,
    max_linear: f32,
    max_angular: f32,
) -> (f32, f32) {
    let dx = target_x - x;
    let dy = target_y - y;
    let distance = (dx * dx + dy * dy).sqrt();

    if distance < 0.05 {
        return (0.0, 0.0); // Close enough
    }

    // Angle to target
    let target_angle = dy.atan2(dx);
    let angle_error = normalize_angle(target_angle - theta);
    let abs_error = angle_error.abs();

    // Proportional angular velocity (gain = 1.5)
    let angular_vel = (angle_error * 1.5).clamp(-max_angular, max_angular);

    // Only pure rotation if angle is very large (>45°, ~0.78 rad)
    if abs_error > 0.78 {
        return (0.0, angular_vel);
    }

    // Move forward with concurrent angular correction
    // Scale linear velocity based on angle error (slower when turning more)
    let angle_factor = 1.0 - (abs_error / 0.78).min(0.8);
    let linear_vel = max_linear.min(distance) * angle_factor;

    (linear_vel, angular_vel)
}

/// Compute angular velocity to rotate to target heading
fn compute_angular_velocity(current: f32, target: f32, max_speed: f32) -> f32 {
    let error = normalize_angle(target - current);
    if error.abs() < 0.05 {
        0.0
    } else {
        (error * 2.0).clamp(-max_speed, max_speed)
    }
}

/// Normalize angle to [-PI, PI)
fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle % TAU;
    if a > PI {
        a -= TAU;
    } else if a < -PI {
        a += TAU;
    }
    a
}
