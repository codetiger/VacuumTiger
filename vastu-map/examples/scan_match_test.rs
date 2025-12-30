//! Scan Matching Accuracy Test
//!
//! Controlled tests to measure scan-to-map matching accuracy using YAML scenario files.
//!
//! Scenarios define:
//! - Which map to use
//! - Starting pose (optional)
//! - Sequence of movement commands (move/stay)
//! - SVG output configuration
//!
//! Usage:
//!   cargo run --example scan_match_test -- --scenario scenarios/straight_line.yaml
//!   cargo run --example scan_match_test -- -s scenarios/rotation_360.yaml
//!   cargo run --example scan_match_test -- -s scenarios/square_path.yaml -o ./my_output

use clap::Parser;
use std::f32::consts::{PI, TAU};
use std::path::Path;

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
    io::{Command, Scenario, SvgConfig, SvgVisualizer, markers_by_distance, markers_by_time},
    slam::{CorrelativeMatcher, CorrelativeMatcherConfig},
};

/// Scan matching accuracy test using YAML scenario files
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Scenario YAML file defining the test
    #[arg(short = 's', long)]
    scenario: String,

    /// Configuration file path
    #[arg(short, long, default_value = "configs/config.yaml")]
    config: String,

    /// Output directory for SVG audit file
    #[arg(short, long, default_value = "./output")]
    output: String,
}

/// Per-scan metrics
#[derive(Clone, Debug)]
struct ScanMetrics {
    /// Ground truth pose
    gt_pose: Pose2D,
    /// Encoder odometry pose
    odom_pose: Pose2D,
    /// Scan-matched corrected pose
    corrected_pose: Pose2D,
    /// Match score (0-1)
    match_score: f32,
    /// Hit ratio (0-1)
    hit_ratio: f32,
    /// Whether match converged
    converged: bool,
}

/// Test results
struct TestResults {
    test_name: String,
    start_pose: Pose2D,
    gt_final: Pose2D,
    odom_final: Pose2D,
    corrected_final: Pose2D,
    scans: Vec<ScanMetrics>,
    wall_cells: usize,
    map: OccupancyGridMap,
}

/// Simple wheel odometry tracker
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

    fn update(&mut self, left_ticks: u16, right_ticks: u16) {
        let left = left_ticks as i32;
        let right = right_ticks as i32;

        if !self.initialized {
            self.last_left = left;
            self.last_right = right;
            self.initialized = true;
            return;
        }

        let delta_left = (left.wrapping_sub(self.last_left) as i16) as i32;
        let delta_right = (right.wrapping_sub(self.last_right) as i16) as i32;

        self.last_left = left;
        self.last_right = right;

        let left_dist = delta_left as f32 / self.ticks_per_meter;
        let right_dist = delta_right as f32 / self.ticks_per_meter;

        let delta_theta = (right_dist - left_dist) / self.wheel_base;
        let delta_linear = (left_dist + right_dist) / 2.0;

        let (dx, dy) = if delta_theta.abs() < 1e-6 {
            (delta_linear, 0.0)
        } else {
            let radius = delta_linear / delta_theta;
            (
                radius * delta_theta.sin(),
                radius * (1.0 - delta_theta.cos()),
            )
        };

        let (sin_t, cos_t) = self.pose.theta.sin_cos();
        let dx_world = dx * cos_t - dy * sin_t;
        let dy_world = dx * sin_t + dy * cos_t;

        self.pose = Pose2D::new(
            self.pose.x + dx_world,
            self.pose.y + dy_world,
            normalize_angle(self.pose.theta + delta_theta),
        );
    }

    fn pose(&self) -> Pose2D {
        self.pose
    }

    fn reset_to(&mut self, pose: Pose2D) {
        self.pose = pose;
    }
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    // Load configuration
    let config_path = Path::new(&args.config);
    let vastu_config = if config_path.exists() {
        VastuConfig::load(config_path).unwrap_or_else(|e| {
            eprintln!("Warning: Failed to load config: {}, using defaults", e);
            VastuConfig::default()
        })
    } else {
        println!(
            "Config not found at {}, using defaults",
            config_path.display()
        );
        VastuConfig::default()
    };

    // Create output directory
    let output_dir = Path::new(&args.output);
    std::fs::create_dir_all(output_dir).ok();

    // Load scenario
    let scenario_file = Path::new(&args.scenario);
    if !scenario_file.exists() {
        eprintln!("Scenario file not found: {}", scenario_file.display());
        std::process::exit(1);
    }

    println!("Loading scenario: {}", scenario_file.display());
    let scenario = Scenario::load(scenario_file).unwrap_or_else(|e| {
        eprintln!("Failed to load scenario: {}", e);
        std::process::exit(1);
    });

    // Resolve and load map from scenario
    let maps_dir = Path::new("maps");
    let map_yaml = scenario.resolve_map_path(maps_dir);
    if !map_yaml.exists() {
        eprintln!("Map not found: {}", map_yaml.display());
        std::process::exit(1);
    }

    println!("Loading map: {}", map_yaml.display());
    let sim_map = SimulationMap::load(&map_yaml).expect("Failed to load simulation map");
    println!(
        "Map size: {}x{} pixels, resolution: {}m/px",
        sim_map.width(),
        sim_map.height(),
        sim_map.resolution()
    );

    // Run the scenario test
    let results = run_scenario_test(&sim_map, &scenario, &vastu_config);
    print_results(&results);
    save_output(&results, output_dir, &scenario);
}

/// Find a free starting position with good clearance
fn find_start_position(sim_map: &SimulationMap) -> Pose2D {
    let sim_origin = sim_map.origin();
    let sim_width_m = sim_map.width() as f32 * sim_map.resolution();
    let sim_height_m = sim_map.height() as f32 * sim_map.resolution();

    // Search for position with good clearance
    let clearance = 1.0; // 1m from walls
    for py in 0..sim_map.height() {
        for px in 0..sim_map.width() {
            let (x, y) = sim_map.pixel_to_world(px, py);

            // Check bounds
            if x < sim_origin.0 + clearance
                || x > sim_origin.0 + sim_width_m - clearance
                || y < sim_origin.1 + clearance
                || y > sim_origin.1 + sim_height_m - clearance
            {
                continue;
            }

            // Check clearance in all directions
            let mut clear = true;
            for angle in 0..8 {
                let a = angle as f32 * PI / 4.0;
                for dist in [0.3, 0.6, 0.9] {
                    if sim_map.is_occupied(x + dist * a.cos(), y + dist * a.sin()) {
                        clear = false;
                        break;
                    }
                }
                if !clear {
                    break;
                }
            }

            if clear {
                return Pose2D::new(x, y, 0.0);
            }
        }
    }

    // Fallback
    Pose2D::new(
        sim_origin.0 + sim_width_m / 2.0,
        sim_origin.1 + sim_height_m / 2.0,
        0.0,
    )
}

/// Run a scenario-based test from YAML specification
fn run_scenario_test(
    sim_map: &SimulationMap,
    scenario: &Scenario,
    vastu_config: &VastuConfig,
) -> TestResults {
    println!("\n=== Running Scenario: {} ===", scenario.name);
    if !scenario.description.is_empty() {
        println!("{}", scenario.description);
    }

    // Determine start pose
    let start_pose = if let Some(ref sp) = scenario.start_pose {
        Pose2D::new(sp.x, sp.y, sp.theta)
    } else {
        find_start_position(sim_map)
    };

    println!(
        "Start position: ({:.2}, {:.2}, {:.1}°)",
        start_pose.x,
        start_pose.y,
        start_pose.theta.to_degrees()
    );

    // Setup simulation
    let map_config = vastu_config.to_map_config();
    let mut map = OccupancyGridMap::new(map_config);

    let robot_config = RobotConfig::default();
    let mut physics =
        PhysicsState::new(start_pose.x, start_pose.y, start_pose.theta, &robot_config);

    let encoder_config = EncoderConfig::default();
    let encoder_noise = NoiseGenerator::new(42);
    let mut encoder =
        EncoderSimulator::new(&encoder_config, robot_config.ticks_per_meter, encoder_noise);

    let mut pure_odometry = WheelOdometry::new(
        robot_config.wheel_base,
        robot_config.ticks_per_meter,
        start_pose,
    );
    let mut corrected_odometry = WheelOdometry::new(
        robot_config.wheel_base,
        robot_config.ticks_per_meter,
        start_pose,
    );

    let lidar_config = LidarConfig::default();

    let matcher_config = CorrelativeMatcherConfig {
        sensor_offset: (lidar_config.mounting_x, lidar_config.mounting_y),
        ..CorrelativeMatcherConfig::default()
    };
    let matcher = CorrelativeMatcher::new(matcher_config);
    let lidar_noise = NoiseGenerator::new(43);
    let mut lidar = LidarSimulator::new(&lidar_config, lidar_noise);

    // Simulation parameters
    let dt = 0.05; // 20Hz physics
    let lidar_interval = 0.2; // 5Hz lidar

    let mut time_since_lidar = lidar_interval;
    let mut corrected_pose = start_pose;
    let mut scans: Vec<ScanMetrics> = Vec::new();

    // Phase 1: Seed the map
    if scenario.seed.rotation {
        println!(
            "Seeding map with {} rotational scans...",
            scenario.seed.scans
        );
        for i in 0..scenario.seed.scans {
            let angle = i as f32 * TAU / scenario.seed.scans as f32;
            let seed_pose = Pose2D::new(start_pose.x, start_pose.y, angle);
            let scan_data = lidar.generate_scan(sim_map, seed_pose.x, seed_pose.y, seed_pose.theta);
            if !scan_data.is_empty() {
                let ranges: Vec<f32> = scan_data.iter().map(|(_, d, _)| *d).collect();
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
                map.observe_lidar(&scan, seed_pose);
            }
        }
    }

    let map_stats = map.coverage_stats();
    println!(
        "Map seeded: {} wall cells, {} floor cells",
        map_stats.wall_cells, map_stats.floor_cells
    );

    // Phase 2: Execute scenario commands
    println!(
        "Executing {} commands (total: {:.1}s)...",
        scenario.commands.len(),
        scenario.total_duration_secs()
    );

    for (cmd_idx, command) in scenario.commands.iter().enumerate() {
        let (linear_vel, angular_vel) = command.velocities();
        let duration_secs = command.duration_secs();
        let steps = (duration_secs / dt) as usize;

        let cmd_type = match command {
            Command::Move { .. } => "move",
            Command::Stay { .. } => "stay",
        };
        println!(
            "  Command {}: {} (v={:.2}m/s, w={:.2}rad/s) for {:.1}s",
            cmd_idx + 1,
            cmd_type,
            linear_vel,
            angular_vel,
            duration_secs
        );

        for _ in 0..steps {
            // Update encoders
            let (left_vel, right_vel) =
                physics.wheel_velocities(linear_vel, angular_vel, robot_config.wheel_base);
            let (left_ticks, right_ticks) = encoder.update(left_vel, right_vel, dt);

            pure_odometry.update(left_ticks, right_ticks);
            corrected_odometry.update(left_ticks, right_ticks);

            // Update physics
            let collision = physics.update(dt, linear_vel, angular_vel, sim_map, &robot_config);
            if collision {
                println!("Warning: Collision detected, stopping command early");
                break;
            }

            // Lidar scan
            time_since_lidar += dt;
            if time_since_lidar >= lidar_interval {
                time_since_lidar = 0.0;

                let gt_pose = Pose2D::new(physics.x(), physics.y(), physics.theta());
                let pure_odom_pose = pure_odometry.pose();
                let corrected_odom_pose = corrected_odometry.pose();

                let scan_data = lidar.generate_scan(sim_map, gt_pose.x, gt_pose.y, gt_pose.theta);
                if !scan_data.is_empty() {
                    let ranges: Vec<f32> = scan_data.iter().map(|(_, d, _)| *d).collect();
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

                    let match_result =
                        matcher.match_scan(&scan, corrected_odom_pose, map.storage());

                    let match_dist = ((match_result.pose.x - corrected_odom_pose.x).powi(2)
                        + (match_result.pose.y - corrected_odom_pose.y).powi(2))
                    .sqrt();

                    let valid_match = match_result.converged
                        && match_result.hit_ratio() > 0.3
                        && match_dist < 0.5;

                    if valid_match {
                        corrected_pose = match_result.pose;
                        corrected_odometry.reset_to(corrected_pose);
                    } else {
                        corrected_pose = corrected_odom_pose;
                    }

                    map.observe_lidar(&scan, corrected_pose);

                    scans.push(ScanMetrics {
                        gt_pose,
                        odom_pose: pure_odom_pose,
                        corrected_pose,
                        match_score: match_result.score,
                        hit_ratio: match_result.hit_ratio(),
                        converged: match_result.converged,
                    });
                }
            }
        }
    }

    let map_stats = map.coverage_stats();

    // Use the last scan's poses for fair comparison
    // (corrected_pose is from the last scan, so odom should be too)
    let (odom_final, corrected_final, gt_final) = if let Some(last_scan) = scans.last() {
        (
            last_scan.odom_pose,
            last_scan.corrected_pose,
            last_scan.gt_pose,
        )
    } else {
        // Fallback if no scans
        (
            pure_odometry.pose(),
            corrected_pose,
            Pose2D::new(physics.x(), physics.y(), physics.theta()),
        )
    };

    TestResults {
        test_name: scenario.name.clone(),
        start_pose,
        gt_final,
        odom_final,
        corrected_final,
        scans,
        wall_cells: map_stats.wall_cells,
        map,
    }
}

/// Generate SVG audit file
fn save_output(results: &TestResults, output_dir: &Path, scenario: &Scenario) {
    println!("\nGenerating SVG audit file...");

    // Collect trajectories from scan metrics
    let gt_poses: Vec<Pose2D> = results.scans.iter().map(|s| s.gt_pose).collect();
    let est_poses: Vec<Pose2D> = results.scans.iter().map(|s| s.corrected_pose).collect();
    let odom_poses: Vec<Pose2D> = results.scans.iter().map(|s| s.odom_pose).collect();

    // Calculate markers based on scenario config
    let marker_indices = if let Some(interval_m) = scenario.svg_output.marker_interval_m {
        markers_by_distance(&gt_poses, interval_m)
    } else if let Some(interval_s) = scenario.svg_output.marker_interval_s {
        markers_by_time(gt_poses.len(), scenario.total_duration_secs(), interval_s)
    } else {
        markers_by_distance(&gt_poses, 0.5)
    };

    let svg_config = SvgConfig {
        scale: scenario.svg_output.scale,
        trajectory_width: scenario.svg_output.trajectory_width,
        ..SvgConfig::default()
    };

    let visualizer = SvgVisualizer::new(results.map.storage().clone(), svg_config)
        .with_title(&scenario.name)
        .with_ground_truth(gt_poses, marker_indices.clone())
        .with_estimated(est_poses, marker_indices.clone())
        .with_odometry(odom_poses, marker_indices);

    let svg_filename = scenario.svg_filename();
    let svg_path = output_dir.join(&svg_filename);

    if let Err(e) = visualizer.save(&svg_path) {
        eprintln!("  Failed to save SVG: {}", e);
    } else {
        println!("  Saved: {}", svg_path.display());
    }
}

/// Print test results
fn print_results(results: &TestResults) {
    println!("\n=== {} Results ===", results.test_name);
    println!("Scans collected: {}", results.scans.len());
    println!("Map wall cells: {}", results.wall_cells);

    // Pose comparison
    println!("\nPose Trajectory:");
    println!(
        "  Start:     ({:7.3}, {:7.3}, {:7.2}°)",
        results.start_pose.x,
        results.start_pose.y,
        results.start_pose.theta.to_degrees()
    );
    println!(
        "  GT Final:  ({:7.3}, {:7.3}, {:7.2}°)",
        results.gt_final.x,
        results.gt_final.y,
        results.gt_final.theta.to_degrees()
    );
    println!(
        "  Odom Final:({:7.3}, {:7.3}, {:7.2}°)",
        results.odom_final.x,
        results.odom_final.y,
        results.odom_final.theta.to_degrees()
    );
    println!(
        "  Corrected: ({:7.3}, {:7.3}, {:7.2}°)",
        results.corrected_final.x,
        results.corrected_final.y,
        results.corrected_final.theta.to_degrees()
    );

    // Calculate errors
    let odom_pos_error = ((results.odom_final.x - results.gt_final.x).powi(2)
        + (results.odom_final.y - results.gt_final.y).powi(2))
    .sqrt();
    let odom_ang_error = normalize_angle(results.odom_final.theta - results.gt_final.theta).abs();

    let corr_pos_error = ((results.corrected_final.x - results.gt_final.x).powi(2)
        + (results.corrected_final.y - results.gt_final.y).powi(2))
    .sqrt();
    let corr_ang_error =
        normalize_angle(results.corrected_final.theta - results.gt_final.theta).abs();

    println!("\nFinal Errors (vs Ground Truth):");
    println!(
        "  Encoder:   position = {:6.4}m, heading = {:6.2}°",
        odom_pos_error,
        odom_ang_error.to_degrees()
    );
    println!(
        "  Corrected: position = {:6.4}m, heading = {:6.2}°",
        corr_pos_error,
        corr_ang_error.to_degrees()
    );

    // Improvement ratio
    if odom_pos_error > 0.0 {
        let pos_improvement = (odom_pos_error - corr_pos_error) / odom_pos_error * 100.0;
        println!("\n  Position improvement: {:+.1}%", pos_improvement);
    }
    if odom_ang_error > 0.0 {
        let ang_improvement = (odom_ang_error - corr_ang_error) / odom_ang_error * 100.0;
        println!("  Heading improvement:  {:+.1}%", ang_improvement);
    }

    // Scan matching statistics
    let converged_count = results.scans.iter().filter(|s| s.converged).count();
    let avg_score: f32 = if !results.scans.is_empty() {
        results.scans.iter().map(|s| s.match_score).sum::<f32>() / results.scans.len() as f32
    } else {
        0.0
    };
    let avg_hit_ratio: f32 = if !results.scans.is_empty() {
        results.scans.iter().map(|s| s.hit_ratio).sum::<f32>() / results.scans.len() as f32
    } else {
        0.0
    };

    println!("\nScan Matching Statistics:");
    println!(
        "  Success rate: {}/{} ({:.1}%)",
        converged_count,
        results.scans.len(),
        if results.scans.is_empty() {
            0.0
        } else {
            100.0 * converged_count as f32 / results.scans.len() as f32
        }
    );
    println!("  Avg score:    {:.3}", avg_score);
    println!("  Avg hit ratio:{:.1}%", avg_hit_ratio * 100.0);

    // Per-scan error analysis
    if !results.scans.is_empty() {
        let mut odom_errors: Vec<f32> = Vec::new();
        let mut corr_errors: Vec<f32> = Vec::new();

        for scan in &results.scans {
            let odom_err = ((scan.odom_pose.x - scan.gt_pose.x).powi(2)
                + (scan.odom_pose.y - scan.gt_pose.y).powi(2))
            .sqrt();
            let corr_err = ((scan.corrected_pose.x - scan.gt_pose.x).powi(2)
                + (scan.corrected_pose.y - scan.gt_pose.y).powi(2))
            .sqrt();
            odom_errors.push(odom_err);
            corr_errors.push(corr_err);
        }

        let odom_max = odom_errors.iter().cloned().fold(0.0f32, f32::max);
        let corr_max = corr_errors.iter().cloned().fold(0.0f32, f32::max);
        let odom_avg = odom_errors.iter().sum::<f32>() / odom_errors.len() as f32;
        let corr_avg = corr_errors.iter().sum::<f32>() / corr_errors.len() as f32;

        println!("\nPer-Scan Position Error:");
        println!(
            "  Encoder:   avg = {:6.4}m, max = {:6.4}m",
            odom_avg, odom_max
        );
        println!(
            "  Corrected: avg = {:6.4}m, max = {:6.4}m",
            corr_avg, corr_max
        );
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
