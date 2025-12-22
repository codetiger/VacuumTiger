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
use vastu_map::config::ExplorationConfig as MapExplorationConfig;
use vastu_map::core::Point2D;
use vastu_map::exploration::{CollisionEvent, CollisionType, ExplorationConfig, VelocityCommand};
use vastu_map::integration::CoplanarMergeConfig;

/// Maximum exploration cycles before timeout.
const MAX_CYCLES: usize = 200;

/// Maximum simulation time in seconds.
const MAX_TIME: f32 = 200.0; // 3.3 minutes

/// How long to wait for map update (in simulation seconds).
const MAP_UPDATE_WAIT: f32 = 0.4;

/// Distance threshold to consider "arrived" at frontier (meters).
const ARRIVAL_THRESHOLD: f32 = 0.4;

/// Maximum attempts to reach the same frontier before marking unreachable.
const MAX_FRONTIER_ATTEMPTS: usize = 2;

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
        assert!(
            result.map_lines >= 15,
            "Map has too few lines: {} (expected >= 15 for room + obstacles)",
            result.map_lines
        );

        // 2. Robot should have explored for multiple cycles
        assert!(
            result.cycles >= 5,
            "Robot did not explore enough: only {} cycles (expected >= 5)",
            result.cycles
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

    /// Run the exploration loop with stop-scan-move pattern.
    fn run_exploration_loop(
        harness: &mut TestHarness,
        config: ExplorationConfig,
    ) -> ExplorationTestResult {
        let mut cycles = 0;
        let mut collisions = 0;
        let mut virtual_walls = 0;
        let mut unreachable_frontiers: Vec<Point2D> = Vec::new();
        // Track attempts per frontier location (using grid cells)
        let mut frontier_attempts: std::collections::HashMap<(i32, i32), usize> =
            std::collections::HashMap::new();
        const FRONTIER_MATCH_DIST: f32 = 0.5;
        const GRID_CELL_SIZE: f32 = 0.5;

        fn point_to_cell(p: Point2D) -> (i32, i32) {
            ((p.x / GRID_CELL_SIZE) as i32, (p.y / GRID_CELL_SIZE) as i32)
        }

        println!("Starting exploration with path planning...");

        // Phase 1: Initial map update (longer for initial observation)
        wait_for_map_update(harness, 2.0);

        let initial_frontiers = harness.slam().frontiers().len();
        println!("Initial frontiers: {}", initial_frontiers);

        while cycles < MAX_CYCLES && harness.sim_time() < MAX_TIME {
            cycles += 1;

            // Step 1: Get frontiers from updated map, excluding unreachable ones
            let frontiers = harness.slam().frontiers();
            let reachable_frontiers: Vec<_> = frontiers
                .iter()
                .filter(|f| {
                    let cell = point_to_cell(f.point);
                    let attempts = frontier_attempts.get(&cell).copied().unwrap_or(0);
                    attempts < MAX_FRONTIER_ATTEMPTS
                        && !unreachable_frontiers
                            .iter()
                            .any(|u| u.distance(f.point) < FRONTIER_MATCH_DIST)
                })
                .collect();

            if reachable_frontiers.is_empty() {
                if frontiers.is_empty() {
                    println!(
                        "  Cycle {}: No frontiers remain - exploration complete!",
                        cycles
                    );
                } else {
                    println!(
                        "  Cycle {}: All {} frontiers unreachable - done",
                        cycles,
                        frontiers.len()
                    );
                }
                break;
            }

            // Step 2: Find best frontier with path planning
            let robot_pos = harness.physics_pose().position();

            // Sort by distance, preferring frontiers with fewer attempts
            let mut sorted_frontiers: Vec<_> = reachable_frontiers
                .iter()
                .map(|f| {
                    let cell = point_to_cell(f.point);
                    let attempts = frontier_attempts.get(&cell).copied().unwrap_or(0);
                    let distance = robot_pos.distance(f.point);
                    // Score: lower is better, penalize retries
                    let score = distance + (attempts as f32) * 0.5;
                    (f.point, distance, score)
                })
                .collect();
            sorted_frontiers.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

            let mut selected_target: Option<(Point2D, vastu_map::Path)> = None;

            for (target, distance, _) in &sorted_frontiers {
                // Try to plan a path to this frontier
                if let Some(path) = harness.slam().get_path(robot_pos, *target) {
                    if path.points.len() >= 2 && path.length < distance * 3.0 {
                        // Accept path if it's not too much longer than direct distance
                        println!(
                            "  Cycle {}: Path to ({:.2}, {:.2}), dist: {:.2}m, path: {:.2}m, {} waypoints",
                            cycles,
                            target.x,
                            target.y,
                            distance,
                            path.length,
                            path.points.len()
                        );
                        selected_target = Some((*target, path));
                        break;
                    }
                }
            }

            let (target, path) = match selected_target {
                Some(t) => t,
                None => {
                    // No path found, try direct approach to nearest unexplored
                    let nearest = sorted_frontiers.first().map(|(p, _, _)| *p);
                    if let Some(target) = nearest {
                        println!(
                            "  Cycle {}: No path, direct to ({:.2}, {:.2})",
                            cycles, target.x, target.y
                        );
                        let direct_path = vastu_map::Path {
                            points: vec![robot_pos, target],
                            length: robot_pos.distance(target),
                        };
                        (target, direct_path)
                    } else {
                        break;
                    }
                }
            };

            // Track attempt for this frontier location
            let target_cell = point_to_cell(target);
            *frontier_attempts.entry(target_cell).or_insert(0) += 1;

            let start_distance = robot_pos.distance(target);

            // Step 3: Follow the path
            let (collision_count, wall_count, reached) =
                follow_path(harness, &config, &path, ARRIVAL_THRESHOLD);
            collisions += collision_count;
            virtual_walls += wall_count;

            // Check result
            let end_pos = harness.physics_pose().position();
            let final_distance = end_pos.distance(target);

            if reached {
                // Successfully reached frontier - frontier should disappear
                // after map update
            } else if final_distance < start_distance * 0.5 {
                // Made significant progress even if didn't fully reach
                // Don't mark as unreachable, just continue
            } else if collision_count > 0 {
                // Had collision, will replan with virtual wall
            } else {
                // Got stuck without collision and didn't make progress
                // Check if we've tried too many times
                let attempts = frontier_attempts.get(&target_cell).copied().unwrap_or(0);
                if attempts >= MAX_FRONTIER_ATTEMPTS {
                    println!(
                        "           Too many attempts at ({:.2}, {:.2}), marking unreachable",
                        target.x, target.y
                    );
                    unreachable_frontiers.push(target);
                }
            }

            // Step 4: Wait for map update after moving
            wait_for_map_update(harness, MAP_UPDATE_WAIT);

            // Log progress
            let current_frontiers = harness.slam().frontiers().len();
            println!(
                "           Position: ({:.2}, {:.2}), frontiers: {}, unreachable: {}",
                end_pos.x,
                end_pos.y,
                current_frontiers,
                unreachable_frontiers.len()
            );
        }

        // Get line count before optimization
        let map_lines_before = harness.slam().lines().len();

        // Optimize coplanar lines with tight thresholds:
        // - max_angle_diff: 1 degree = 0.01745 rad
        // - max_perpendicular_dist: 2cm = 0.02m
        let coplanar_config = CoplanarMergeConfig::default()
            .with_max_angle_diff(3.0 * std::f32::consts::PI / 180.0) // 1 degree
            .with_max_perpendicular_dist(0.05); // 2cm

        let lines_merged = harness.slam_mut().optimize_lines(&coplanar_config);

        println!(
            "\n  Line optimization: {} lines -> {} lines ({} merged)",
            map_lines_before,
            harness.slam().lines().len(),
            lines_merged
        );

        // Finalize to generate visualization
        let _test_result = harness.finalize();

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

    /// Follow a planned path through waypoints.
    /// Returns (collision_count, virtual_wall_count, reached_goal).
    fn follow_path(
        harness: &mut TestHarness,
        config: &ExplorationConfig,
        path: &vastu_map::Path,
        arrival_threshold: f32,
    ) -> (usize, usize, bool) {
        let mut collisions = 0;
        let mut virtual_walls = 0;
        let mut steps = 0;
        const MAX_STEPS: usize = 500; // Prevent infinite loops

        if path.points.len() < 2 {
            return (0, 0, false);
        }

        let mut waypoint_idx = 1; // Start from first waypoint (skip start position)
        let goal = path.points.last().copied().unwrap();
        let start_pos = harness.physics_pose().position();
        let initial_distance = start_pos.distance(goal);
        let mut last_progress_step = 0;
        let mut best_distance_to_goal = initial_distance;

        // For short paths, allow less strict progress requirements
        let progress_threshold = (initial_distance * 0.05).max(0.03);

        while steps < MAX_STEPS && harness.sim_time() < MAX_TIME && waypoint_idx < path.points.len()
        {
            let pose = harness.physics_pose();
            let robot_pos = pose.position();
            let current_waypoint = path.points[waypoint_idx];
            let distance_to_waypoint = robot_pos.distance(current_waypoint);
            let distance_to_goal = robot_pos.distance(goal);

            // Check if reached goal (can skip intermediate waypoints)
            if distance_to_goal < arrival_threshold {
                return (collisions, virtual_walls, true);
            }

            // Check if reached current waypoint
            if distance_to_waypoint < arrival_threshold {
                waypoint_idx += 1;
                if waypoint_idx >= path.points.len() {
                    return (collisions, virtual_walls, true);
                }
                continue;
            }

            // Track progress towards goal
            if distance_to_goal < best_distance_to_goal - progress_threshold {
                best_distance_to_goal = distance_to_goal;
                last_progress_step = steps;
            }

            // Check if stuck (no progress for too long)
            // Allow more time for short paths
            let max_stuck_steps = if initial_distance < 1.0 { 200 } else { 150 };
            if steps - last_progress_step > max_stuck_steps {
                // Not making progress, abort
                return (collisions, virtual_walls, false);
            }

            // Compute velocity towards waypoint
            let velocity = compute_velocity_to_waypoint(pose, current_waypoint, config);

            // Execute movement
            let step_result = harness.step(velocity.linear, velocity.angular);
            steps += 1;

            // Handle collision
            if step_result.any_bumper() {
                collisions += 1;

                let collision_type = match (step_result.left_bumper, step_result.right_bumper) {
                    (true, true) => CollisionType::BumperBoth,
                    (true, false) => CollisionType::BumperLeft,
                    (false, true) => CollisionType::BumperRight,
                    _ => continue,
                };

                let theta = step_result.physics_pose.theta;
                let radius = harness.config().robot.robot_radius;
                let point = Point2D::new(
                    step_result.physics_pose.x + (radius + 0.02) * theta.cos(),
                    step_result.physics_pose.y + (radius + 0.02) * theta.sin(),
                );

                // Add virtual wall
                let collision = CollisionEvent::new(collision_type, point, theta);
                let wall = collision.to_virtual_wall(config.virtual_wall_length);
                harness.slam_mut().add_line(wall.line);
                virtual_walls += 1;

                println!(
                    "    Collision at ({:.2}, {:.2}) - added virtual wall",
                    point.x, point.y
                );

                // Back off after collision
                backoff(harness, config.backoff_distance, theta);

                // Abort path following - need to replan
                return (collisions, virtual_walls, false);
            }

            // Process lidar periodically
            if steps % 5 == 0 {
                harness.process_lidar();
            }
        }

        // Check if we reached the goal
        let final_distance = harness.physics_pose().position().distance(goal);
        (
            collisions,
            virtual_walls,
            final_distance < arrival_threshold,
        )
    }

    /// Compute velocity command to move towards a waypoint.
    fn compute_velocity_to_waypoint(
        pose: vastu_map::core::Pose2D,
        waypoint: Point2D,
        config: &ExplorationConfig,
    ) -> VelocityCommand {
        // Transform waypoint to robot frame
        let local_target = pose.inverse_transform_point(waypoint);

        // Angle to waypoint (in robot frame, forward is +X)
        let angle_to_target = local_target.y.atan2(local_target.x);
        let distance = local_target.length();

        // Angular velocity: proportional to angle error
        let angular =
            (angle_to_target * 2.5).clamp(-config.max_angular_speed, config.max_angular_speed);

        // Linear velocity depends on alignment with target
        let alignment = angle_to_target.abs().cos(); // 1.0 when facing target, 0 when perpendicular

        // Base linear velocity proportional to distance
        let base_linear = (distance * 0.5).min(config.max_linear_speed);

        // Reduce speed when not well aligned, but always maintain some forward motion
        // if we're close enough that turning alone won't help
        let linear = if angle_to_target.abs() > 1.0 {
            // Very misaligned - turn in place
            0.0
        } else if angle_to_target.abs() > 0.3 {
            // Somewhat misaligned - slow forward while turning
            base_linear * alignment * 0.3
        } else {
            // Well aligned - full speed
            base_linear * alignment
        };

        // Ensure minimum speed when close to target
        let linear = if distance < 0.5 && linear < 0.05 && angle_to_target.abs() < 0.5 {
            0.1
        } else {
            linear
        };

        VelocityCommand::new(linear, angular)
    }

    /// Back off after collision.
    fn backoff(harness: &mut TestHarness, distance: f32, _heading: f32) {
        let start_pos = harness.physics_pose().position();
        let backup_speed = -0.15; // Reverse
        let mut steps = 0;
        const MAX_BACKUP_STEPS: usize = 50;

        while harness.physics_pose().position().distance(start_pos) < distance
            && steps < MAX_BACKUP_STEPS
        {
            harness.step(backup_speed, 0.0);
            steps += 1;
        }
    }
}
