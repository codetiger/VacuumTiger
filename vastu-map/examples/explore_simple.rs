//! Simple exploration example using VastuExplorer API.
//!
//! This example demonstrates the high-level exploration API that handles
//! all SLAM, scan matching, and navigation internally.
//!
//! Usage:
//!   cargo run --example explore_simple -- --map simple_room
//!   cargo run --example explore_simple  # Uses random map

use clap::Parser;
use rand::prelude::*;
use std::f32::consts::TAU;
use std::path::Path;

// SangamIO mock simulation components
use sangam_io::devices::mock::{
    config::{LidarConfig, RobotConfig},
    lidar_sim::LidarSimulator,
    map_loader::SimulationMap,
    noise::NoiseGenerator,
    physics::PhysicsState,
};

// Vastu-map components
use vastu_map::{
    LidarScan, Pose2D, VastuConfig,
    explore::{ExploreResult, SensorSource, VastuExplorer},
    io::{export_ros_map, save_vastu},
};

/// Simple exploration example
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Configuration file path
    #[arg(short, long, default_value = "configs/config.yaml")]
    config: String,

    /// Map name to load from maps/ folder (e.g., "simple_room")
    #[arg(short, long)]
    map: Option<String>,

    /// Output directory for saved maps
    #[arg(short, long, default_value = "./output")]
    output: String,
}

/// Mock sensor source that wraps the SangamIO simulation.
struct MockSensorSource<'a> {
    sim_map: &'a SimulationMap,
    physics: PhysicsState,
    lidar: LidarSimulator,
    lidar_config: LidarConfig,
    robot_config: RobotConfig,
    latest_scan: Option<LidarScan>,
    scan_counter: u32,
    /// Current commanded velocities
    linear_vel: f32,
    angular_vel: f32,
}

impl<'a> MockSensorSource<'a> {
    fn new(sim_map: &'a SimulationMap, start_pose: Pose2D) -> Self {
        let robot_config = RobotConfig::default();
        let lidar_config = LidarConfig::default();
        let noise = NoiseGenerator::new(42);
        let lidar = LidarSimulator::new(&lidar_config, noise);
        let physics =
            PhysicsState::new(start_pose.x, start_pose.y, start_pose.theta, &robot_config);

        Self {
            sim_map,
            physics,
            lidar,
            lidar_config,
            robot_config,
            latest_scan: None,
            scan_counter: 0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        }
    }

    fn update_physics(&mut self, dt: f32, linear: f32, angular: f32) {
        self.physics
            .update(dt, linear, angular, self.sim_map, &self.robot_config);
    }
}

impl SensorSource for MockSensorSource<'_> {
    fn get_pose(&self) -> Pose2D {
        Pose2D::new(self.physics.x(), self.physics.y(), self.physics.theta())
    }

    fn get_lidar_scan(&self) -> Option<LidarScan> {
        self.latest_scan.clone()
    }

    fn send_velocity(&mut self, linear: f32, angular: f32) {
        // Store commanded velocities (physics updated in poll)
        self.linear_vel = linear;
        self.angular_vel = angular;
    }

    fn is_connected(&self) -> bool {
        true
    }

    fn poll(&mut self) {
        // Update physics with current velocities (10Hz control loop)
        let dt = 0.1;
        self.update_physics(dt, self.linear_vel, self.angular_vel);

        // Generate lidar scan every 2nd poll (5Hz lidar at 10Hz control)
        // Start at 1 so first poll generates a scan
        self.scan_counter += 1;
        if self.scan_counter == 1 || self.scan_counter % 2 == 0 {
            let scan_data = self.lidar.generate_scan(
                self.sim_map,
                self.physics.x(),
                self.physics.y(),
                self.physics.theta(),
            );

            if !scan_data.is_empty() {
                let ranges: Vec<f32> = scan_data.iter().map(|(_, d, _)| *d).collect();
                let angles: Vec<f32> = scan_data.iter().map(|(a, _, _)| *a).collect();

                self.latest_scan = Some(LidarScan::new(
                    ranges,
                    angles,
                    self.lidar_config.min_range,
                    self.lidar_config.max_range,
                ));
            }
        } else {
            self.latest_scan = None;
        }
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

    // Load simulation map
    println!("Loading map: {}", map_yaml.display());
    let sim_map = SimulationMap::load(&map_yaml).expect("Failed to load simulation map");
    println!(
        "Map size: {}x{} pixels, resolution: {}m/px",
        sim_map.width(),
        sim_map.height(),
        sim_map.resolution()
    );

    // Find starting position
    let start_pose = find_free_start_position(&sim_map);
    println!(
        "Starting at: ({:.2}, {:.2}, {:.1}°)",
        start_pose.x,
        start_pose.y,
        start_pose.theta.to_degrees()
    );

    // Create output directory
    std::fs::create_dir_all(&args.output).ok();

    // Create explorer with loaded configuration
    let explorer_config = vastu_config.to_explorer_config();
    let explorer = VastuExplorer::new(explorer_config);

    // Create sensor source
    let mut source = MockSensorSource::new(&sim_map, start_pose);

    // Run exploration - the library handles everything!
    println!("\nStarting exploration...");
    match explorer.explore(&mut source) {
        ExploreResult::Complete(map) => {
            println!("\nExploration complete!");

            // Save maps
            let output_base = Path::new(&args.output).join(&map_name);

            save_vastu(map.storage(), &output_base.with_extension("vastu"))
                .expect("Failed to save .vastu file");
            println!("Saved: {}.vastu", output_base.display());

            export_ros_map(map.storage(), &output_base).expect("Failed to save ROS map");
            println!(
                "Saved: {}.pgm + {}.yaml",
                output_base.display(),
                output_base.display()
            );

            let stats = map.coverage_stats();
            println!("\n=== Map Statistics ===");
            println!("Floor cells: {}", stats.floor_cells);
            println!("Wall cells: {}", stats.wall_cells);
            println!("Coverage: {:.2}m²", stats.explored_area_m2);
        }
        ExploreResult::Failed { map, error } => {
            println!("\nExploration failed: {}", error);

            // Save partial map
            let output_base = Path::new(&args.output).join(format!("{}_partial", map_name));
            save_vastu(map.storage(), &output_base.with_extension("vastu")).ok();
            println!("Saved partial map: {}.vastu", output_base.display());
        }
    }
}

/// List available map names in directory.
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

/// Find a free starting position in the map.
fn find_free_start_position(map: &SimulationMap) -> Pose2D {
    let mut rng = rand::rng();
    let sim_origin = map.origin();
    let sim_width_m = map.width() as f32 * map.resolution();
    let sim_height_m = map.height() as f32 * map.resolution();

    // Search in center region
    let margin = 0.5;
    let min_x = sim_origin.0 + margin;
    let min_y = sim_origin.1 + margin;
    let max_x = sim_origin.0 + sim_width_m - margin;
    let max_y = sim_origin.1 + sim_height_m - margin;

    // Try random positions
    for _ in 0..1000 {
        let x = min_x + rng.random::<f32>() * (max_x - min_x);
        let y = min_y + rng.random::<f32>() * (max_y - min_y);

        // Check if position is free with margin
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

    // Fallback: center of map
    Pose2D::new(
        sim_origin.0 + sim_width_m / 2.0,
        sim_origin.1 + sim_height_m / 2.0,
        0.0,
    )
}
