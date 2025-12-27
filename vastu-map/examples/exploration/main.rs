//! Autonomous exploration example
//!
//! Demonstrates the complete exploration pipeline using vastu_map's public APIs:
//! 1. Frontier detection
//! 2. Path planning to frontiers
//! 3. Path following with collision detection
//! 4. Virtual wall insertion on unexpected collisions
//! 5. Exploration completion when no frontiers remain
//!
//! # Running
//!
//! ```bash
//! # With a specific map file:
//! cargo run --example exploration --release -- path/to/map.yaml
//!
//! # With random map selection:
//! cargo run --example exploration --release
//! ```
//!
//! Output SVG is saved to `results/exploration_test.svg`

mod harness;

use harness::{HarnessConfig, TestHarness};
use rand::seq::IndexedRandom;
use sangam_io::devices::mock::map_loader::SimulationMap;
use vastu_map::Map;
use vastu_map::Path as PlannedPath;
use vastu_map::config::ExplorationConfig as MapExplorationConfig;
use vastu_map::core::{Point2D, Pose2D};
use vastu_map::exploration::{
    CollisionEvent, CollisionType, ExplorationConfig, ExplorationController, ExplorationState,
    VelocityCommand,
};
use vastu_map::integration::CoplanarMergeConfig;
use vastu_map::query::cbvg::ClearanceVisibilityGraph;

/// Maximum simulation time in seconds (no limit).
const MAX_TIME: f32 = f32::MAX;

/// Directory containing available maps.
const MAPS_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/data/maps");

/// Select a random map file from the maps directory.
///
/// # Returns
/// The full path to a randomly selected .yaml map file.
///
/// # Panics
/// If no map files are found in the directory.
fn select_random_map() -> String {
    let maps_path = std::path::Path::new(MAPS_DIR);

    // Collect all .yaml files in the maps directory
    let map_files: Vec<_> = std::fs::read_dir(maps_path)
        .expect("Failed to read maps directory")
        .filter_map(|entry| {
            let entry = entry.ok()?;
            let path = entry.path();
            if path.extension().is_some_and(|ext| ext == "yaml") {
                Some(path)
            } else {
                None
            }
        })
        .collect();

    if map_files.is_empty() {
        panic!("No map files found in {}", MAPS_DIR);
    }

    // Pick a random map
    let mut rng = rand::rng();
    let selected = map_files.choose(&mut rng).unwrap();
    let map_name = selected.file_stem().unwrap().to_string_lossy();

    println!("Selected random map: {}", map_name);
    println!("  (from {} available maps)", map_files.len());

    selected.to_string_lossy().into_owned()
}

/// Find a random reachable (free) position in the map with sufficient clearance.
///
/// # Arguments
/// * `map_file` - Path to the map YAML file
/// * `robot_radius` - Robot radius in meters for clearance checking
///
/// # Returns
/// A tuple (x, y) of world coordinates for a valid start position.
///
/// # Panics
/// If no valid position can be found after sampling.
fn find_random_start_position(map_file: &str, robot_radius: f32) -> (f32, f32) {
    let map = SimulationMap::load(map_file).expect("Failed to load map for position sampling");

    // Collect all free pixels with sufficient clearance
    let mut valid_positions = Vec::new();

    // Calculate clearance in pixels (add some margin)
    let clearance_pixels = ((robot_radius * 1.5) / map.resolution()).ceil() as i32;

    for py in 0..map.height() {
        for px in 0..map.width() {
            let (x, y) = map.pixel_to_world(px, py);

            // Check if this position is free
            if map.is_occupied(x, y) {
                continue;
            }

            // Check clearance around the robot
            let mut has_clearance = true;
            'clearance: for dy in -clearance_pixels..=clearance_pixels {
                for dx in -clearance_pixels..=clearance_pixels {
                    let check_x = x + dx as f32 * map.resolution();
                    let check_y = y + dy as f32 * map.resolution();
                    if map.is_occupied(check_x, check_y) {
                        has_clearance = false;
                        break 'clearance;
                    }
                }
            }

            if has_clearance {
                valid_positions.push((x, y));
            }
        }
    }

    if valid_positions.is_empty() {
        panic!("No valid start positions found in map with sufficient clearance");
    }

    // Pick a random position
    let mut rng = rand::rng();
    let (x, y) = *valid_positions.choose(&mut rng).unwrap();

    println!("Selected random start position: ({:.2}, {:.2})", x, y);
    println!(
        "  (from {} valid positions with {:.2}m clearance)",
        valid_positions.len(),
        robot_radius * 1.5
    );

    (x, y)
}

/// Result of exploration.
#[derive(Debug)]
pub struct ExplorationResult {
    /// Whether exploration completed (no frontiers remain).
    pub completed: bool,
    /// Total simulation time.
    pub sim_time: f32,
    /// Number of exploration cycles.
    pub cycles: usize,
    /// Number of collisions handled.
    pub collisions: usize,
    /// Number of virtual walls added.
    pub virtual_walls: usize,
    /// Map line count before optimization.
    pub map_lines_before: usize,
    /// Final map line count (after optimization).
    pub map_lines: usize,
    /// Number of lines merged during optimization.
    pub lines_merged: usize,
    /// Initial frontier count.
    pub initial_frontiers: usize,
    /// Final frontier count (should be 0 on success).
    pub remaining_frontiers: usize,
}

fn main() {
    env_logger::init();

    // Ensure results directory exists
    std::fs::create_dir_all("results").ok();

    // Check for command-line argument for map file
    let args: Vec<String> = std::env::args().collect();
    let map_file = if args.len() > 1 {
        let provided_map = &args[1];
        // Check if file exists
        if !std::path::Path::new(provided_map).exists() {
            eprintln!("Error: Map file not found: {}", provided_map);
            std::process::exit(1);
        }
        println!("Using provided map: {}", provided_map);
        provided_map.clone()
    } else {
        // Select a random map from available maps
        select_random_map()
    };

    // Get default robot radius for clearance checking
    let default_config = HarnessConfig::default();
    let robot_radius = default_config.robot.robot_radius;

    // Find a random reachable start position
    let (start_x, start_y) = find_random_start_position(&map_file, robot_radius);

    // Setup harness with map file and random starting position
    let harness_config = HarnessConfig {
        map_file,
        start_x,
        start_y,
        ..Default::default()
    };

    // Enable VectorMap exploration mode for point-cloud-based line re-fitting
    // This will re-fit lines every 10 observations using accumulated scan data
    let map_exploration_config = MapExplorationConfig::default()
        .with_refit_interval(10)
        .with_gap_threshold(0.3);

    let mut harness = TestHarness::new(harness_config)
        .expect("Failed to create harness")
        .with_exploration_mode(Some(map_exploration_config))
        .with_visualization("results/exploration_test.svg");

    // Create robot exploration config (for path following)
    let exploration_config = ExplorationConfig::default()
        .with_max_linear_speed(0.3)
        .with_max_angular_speed(1.0)
        .with_backoff_distance(0.15)
        .with_robot_radius(0.15);

    // Run exploration
    let result = run_exploration_loop(&mut harness, exploration_config);

    // Print results
    println!("\n=== Exploration Results ===");
    println!("Completed: {}", result.completed);
    println!("Simulation time: {:.1}s", result.sim_time);
    println!("Cycles: {}", result.cycles);
    println!("Collisions handled: {}", result.collisions);
    println!("Virtual walls added: {}", result.virtual_walls);
    println!(
        "Map lines: {} before optimization -> {} after ({} merged)",
        result.map_lines_before, result.map_lines, result.lines_merged
    );
    println!(
        "Frontiers: {} initial -> {} remaining",
        result.initial_frontiers, result.remaining_frontiers
    );
    println!("\nVisualization saved to: results/exploration_test.svg");
}

/// Run the exploration loop using the ExplorationController with region-based TSP.
///
/// This uses a continuous control loop where the controller is called every step,
/// properly integrating with its state machine for path following and collision handling.
fn run_exploration_loop(harness: &mut TestHarness, config: ExplorationConfig) -> ExplorationResult {
    let mut cycles = 0;
    let mut collisions = 0;
    let mut virtual_walls = 0;

    // Variables to capture the last CBVG and path for visualization
    let mut last_cbvg: Option<ClearanceVisibilityGraph> = None;
    let mut last_planned_path: Option<PlannedPath> = None;

    println!("Starting exploration with region-based TSP planning...");

    // Create the exploration controller with region-based algorithm
    let mut controller = ExplorationController::new(config.clone());

    // Phase 1: Initial map update (longer for initial observation)
    wait_for_map_update(harness, 2.0);

    let initial_frontiers = harness.slam().frontiers().len();
    println!("Initial frontiers: {}", initial_frontiers);

    // Start exploration
    controller.start();

    let mut step_count = 0;
    let mut last_target: Option<Point2D> = None;
    let mut last_cycle_print = 0;
    let mut pending_collision: Option<CollisionEvent> = None;
    const MAX_STEPS: usize = usize::MAX;
    const LIDAR_INTERVAL: usize = 5; // Process lidar every N steps

    while step_count < MAX_STEPS && harness.sim_time() < MAX_TIME {
        step_count += 1;

        // Get current pose
        let physics_pose = harness.physics_pose();
        let current_pose = Pose2D::new(physics_pose.x, physics_pose.y, physics_pose.theta);

        // Update controller with any pending collision
        let step = controller.update(harness.slam(), current_pose, pending_collision.take());

        // Check completion states
        match &step.state {
            ExplorationState::Complete => {
                println!("  Exploration complete - no frontiers remain!");
                break;
            }
            ExplorationState::Failed { reason } => {
                println!("  Exploration failed - {}", reason);
                break;
            }
            ExplorationState::Idle => {
                break;
            }
            ExplorationState::SelectingFrontier => {
                // Step simulation forward while selecting
                harness.step(0.0, 0.0);
                // Process lidar while selecting
                if step_count % LIDAR_INTERVAL == 0 {
                    harness.process_lidar();
                }
            }
            ExplorationState::FollowingPath {
                target_frontier,
                path,
                ..
            } => {
                // Track cycles (new target = new cycle)
                let is_new_target = last_target
                    .map(|last| last.distance(*target_frontier) > 0.3)
                    .unwrap_or(true);

                if is_new_target {
                    cycles += 1;
                    last_target = Some(*target_frontier);

                    let robot_pos = current_pose.position();
                    println!(
                        "  Cycle {}: Path to ({:.2}, {:.2}), dist: {:.2}m, path: {:.2}m, {} waypoints",
                        cycles,
                        target_frontier.x,
                        target_frontier.y,
                        robot_pos.distance(*target_frontier),
                        path.length,
                        path.points.len()
                    );

                    // Capture CBVG for visualization using visibility-based building
                    // This ensures nodes are only in lidar-scanned areas
                    last_cbvg = Some(harness.slam().build_cbvg());
                    last_planned_path = Some(path.clone());
                }

                // Execute velocity command from controller (or stop if none)
                let vel = step.velocity.unwrap_or(VelocityCommand::new(0.0, 0.0));
                let step_result = harness.step(vel.linear, vel.angular);

                // Check for collision
                if step_result.any_bumper() {
                    collisions += 1;

                    let collision_type = match (step_result.left_bumper, step_result.right_bumper) {
                        (true, true) => CollisionType::BumperBoth,
                        (true, false) => CollisionType::BumperLeft,
                        (false, true) => CollisionType::BumperRight,
                        _ => CollisionType::BumperBoth,
                    };

                    let theta = step_result.physics_pose.theta;
                    let radius = harness.config().robot.robot_radius;
                    let point = Point2D::new(
                        step_result.physics_pose.x + (radius + 0.02) * theta.cos(),
                        step_result.physics_pose.y + (radius + 0.02) * theta.sin(),
                    );

                    // Create collision event for controller
                    pending_collision = Some(CollisionEvent::new(collision_type, point, theta));

                    // Also add virtual wall directly to map
                    let wall = pending_collision
                        .as_ref()
                        .unwrap()
                        .to_virtual_wall(config.virtual_wall_length);
                    harness.slam_mut().add_line(wall.line);
                    virtual_walls += 1;

                    println!(
                        "    Collision at ({:.2}, {:.2}) - added virtual wall",
                        point.x, point.y
                    );
                }

                // Process lidar periodically
                if step_count % LIDAR_INTERVAL == 0 {
                    harness.process_lidar();
                }

                // Log progress periodically
                if cycles > last_cycle_print && step_count % 100 == 0 {
                    let end_pos = harness.physics_pose().position();
                    let current_frontiers = harness.slam().frontiers().len();
                    println!(
                        "           Position: ({:.2}, {:.2}), frontiers: {}, unreachable: {}",
                        end_pos.x,
                        end_pos.y,
                        current_frontiers,
                        controller.unreachable_frontier_count()
                    );
                    last_cycle_print = cycles;
                }
            }
            ExplorationState::RecoveringFromCollision { .. } => {
                // Execute backoff velocity (or stop if none)
                let vel = step.velocity.unwrap_or(VelocityCommand::new(0.0, 0.0));
                let step_result = harness.step(vel.linear, vel.angular);

                // Check for collision during recovery (defensive - should be rare)
                if step_result.any_bumper() {
                    collisions += 1;

                    let collision_type = match (step_result.left_bumper, step_result.right_bumper) {
                        (true, true) => CollisionType::BumperBoth,
                        (true, false) => CollisionType::BumperLeft,
                        (false, true) => CollisionType::BumperRight,
                        _ => CollisionType::BumperBoth,
                    };

                    let theta = step_result.physics_pose.theta;
                    let radius = harness.config().robot.robot_radius;
                    let point = Point2D::new(
                        step_result.physics_pose.x + (radius + 0.02) * theta.cos(),
                        step_result.physics_pose.y + (radius + 0.02) * theta.sin(),
                    );

                    // Create collision event for controller
                    pending_collision = Some(CollisionEvent::new(collision_type, point, theta));

                    // Also add virtual wall directly to map
                    let wall = pending_collision
                        .as_ref()
                        .unwrap()
                        .to_virtual_wall(config.virtual_wall_length);
                    harness.slam_mut().add_line(wall.line);
                    virtual_walls += 1;

                    println!(
                        "    Recovery collision at ({:.2}, {:.2}) - added virtual wall",
                        point.x, point.y
                    );
                }
            }
        }
    }

    // Final log
    let end_pos = harness.physics_pose().position();
    let current_frontiers = harness.slam().frontiers().len();
    println!(
        "           Final position: ({:.2}, {:.2}), frontiers: {}, unreachable: {}",
        end_pos.x,
        end_pos.y,
        current_frontiers,
        controller.unreachable_frontier_count()
    );

    // Get line count before optimization
    let map_lines_before = harness.slam().lines().len();

    // Optimize coplanar lines with tight thresholds
    let coplanar_config = CoplanarMergeConfig::default()
        .with_max_angle_diff(3.0 * std::f32::consts::PI / 180.0)
        .with_max_perpendicular_dist(0.05);

    let lines_merged = harness.slam_mut().optimize_lines(&coplanar_config);

    println!(
        "\n  Line optimization: {} lines -> {} lines ({} merged)",
        map_lines_before,
        harness.slam().lines().len(),
        lines_merged
    );

    // Finalize to generate visualization with CBVG
    harness.finalize_with_graph(last_cbvg.as_ref(), last_planned_path.as_ref());

    ExplorationResult {
        completed: harness.slam().frontiers().is_empty(),
        sim_time: harness.sim_time(),
        cycles,
        collisions,
        virtual_walls,
        map_lines_before,
        map_lines: harness.slam().lines().len(),
        lines_merged,
        initial_frontiers,
        remaining_frontiers: harness.slam().frontiers().len(),
    }
}

/// Wait for map update by running lidar scans for specified duration.
fn wait_for_map_update(harness: &mut TestHarness, duration: f32) {
    let start_time = harness.sim_time();

    while harness.sim_time() - start_time < duration {
        // Stop the robot
        harness.step(0.0, 0.0);
        // Process lidar
        harness.process_lidar();
    }
}
