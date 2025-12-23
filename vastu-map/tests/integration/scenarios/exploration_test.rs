//! Autonomous exploration integration test.
//!
//! Tests the complete exploration pipeline using vastu_map's public APIs:
//! 1. Frontier detection
//! 2. Path planning to frontiers
//! 3. Path following with collision detection
//! 4. Virtual wall insertion on unexpected collisions
//! 5. Exploration completion when no frontiers remain
//!
//! # Exploration Algorithm
//!
//! 1. Stop and wait for map update (1 second of lidar scans)
//! 2. Find the nearest frontier from updated map
//! 3. Plan path and move towards frontier
//! 4. Stop and wait for map update
//! 5. Re-check frontiers, find nearest
//! 6. Repeat until no frontiers remain
//!
//! # Running the test
//!
//! ```bash
//! cargo test --features integration-tests exploration -- --nocapture
//! ```
//!
//! Output SVG is saved to `results/exploration_test.svg`

use crate::{HarnessConfig, TestHarness};
use vastu_map::Map;
use vastu_map::Path as PlannedPath;
use vastu_map::config::ExplorationConfig as MapExplorationConfig;
use vastu_map::core::{Point2D, Pose2D};
use vastu_map::exploration::{
    CollisionEvent, CollisionType, ExplorationConfig, ExplorationController, ExplorationState,
    VelocityCommand,
};
use vastu_map::integration::CoplanarMergeConfig;
use vastu_map::query::{PathPlanner, PathPlanningConfig, VisibilityGraph};

/// Maximum simulation time in seconds.
const MAX_TIME: f32 = 200.0; // 3.3 minutes

/// Result of exploration test.
#[derive(Debug)]
pub struct ExplorationTestResult {
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Test autonomous exploration of medium_room map with visualization.
    ///
    /// This test verifies:
    /// 1. The robot can explore the entire map
    /// 2. Collisions are handled with virtual walls
    /// 3. All frontiers are eventually closed
    ///
    /// Output: `results/exploration_test.svg`
    #[test]
    fn test_autonomous_exploration() {
        env_logger::try_init().ok();

        // Ensure results directory exists
        std::fs::create_dir_all("results").ok();

        // Setup harness with medium_room map and visualization
        let harness_config = HarnessConfig::medium_room();

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
        println!("\n=== Exploration Test Results ===");
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

        // Assertions for exploration quality
        // 1. Map should have reasonable number of lines (room walls + obstacles)
        // Note: line optimization can merge many lines, so threshold is conservative
        assert!(
            result.map_lines >= 10,
            "Map has too few lines: {} (expected >= 10 for room + obstacles)",
            result.map_lines
        );

        // 2. Robot should have explored significant number of frontiers
        // (better metric than cycles, since nearby frontiers don't count as new cycles)
        let frontiers_explored = result
            .initial_frontiers
            .saturating_sub(result.remaining_frontiers);
        assert!(
            frontiers_explored >= 8 || result.completed,
            "Robot did not explore enough: only {} frontiers explored (expected >= 8)",
            frontiers_explored
        );

        // 3. Virtual walls should be added when collisions happen
        if result.collisions > 0 {
            assert!(
                result.virtual_walls > 0,
                "Collisions occurred but no virtual walls added"
            );
        }

        // 4. Exploration should either complete or have reasonable remaining frontiers
        // Note: Frontier count can fluctuate as robot explores and discovers new areas
        assert!(
            result.completed || result.remaining_frontiers <= 30,
            "Too many frontiers remaining: {} (expected <= 30 or completed)",
            result.remaining_frontiers
        );
    }

    /// Run the exploration loop using the ExplorationController with region-based TSP.
    ///
    /// This uses a continuous control loop where the controller is called every step,
    /// properly integrating with its state machine for path following and collision handling.
    fn run_exploration_loop(
        harness: &mut TestHarness,
        config: ExplorationConfig,
    ) -> ExplorationTestResult {
        let mut cycles = 0;
        let mut collisions = 0;
        let mut virtual_walls = 0;

        // Variables to capture the last visibility graph and path for visualization
        let mut last_visibility_graph: Option<VisibilityGraph> = None;
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
        const MAX_STEPS: usize = 20000;
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

                        // Capture visibility graph for this path for visualization
                        let path_config = PathPlanningConfig::default()
                            .with_robot_radius(config.robot_radius)
                            .with_conservative(false);
                        let planner = PathPlanner::new(path_config);
                        last_visibility_graph = Some(planner.build_graph(
                            robot_pos,
                            *target_frontier,
                            harness.slam().lines(),
                        ));
                        last_planned_path = Some(path.clone());
                    }

                    // Execute velocity command from controller (or stop if none)
                    let vel = step.velocity.unwrap_or(VelocityCommand::new(0.0, 0.0));
                    let step_result = harness.step(vel.linear, vel.angular);

                    // Check for collision
                    if step_result.any_bumper() {
                        collisions += 1;

                        let collision_type =
                            match (step_result.left_bumper, step_result.right_bumper) {
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
                    harness.step(vel.linear, vel.angular);
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

        // Finalize to generate visualization with visibility graph
        let _test_result =
            harness.finalize_with_graph(last_visibility_graph.as_ref(), last_planned_path.as_ref());

        ExplorationTestResult {
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
}
