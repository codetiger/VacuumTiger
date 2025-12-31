//! Exploration controller implementation.

use crate::core::{Pose2D, WorldPoint};
use crate::grid::GridStorage;
use crate::pathfinding::{AStarConfig, AStarPlanner, PathFailure, PathSmoother, SmoothingConfig};
use crate::query::{Frontier, FrontierDetector, TraversabilityChecker};
use log::{debug, info, trace, warn};

use super::config::ExplorationConfig;
use super::state::{ExplorationCommand, ExplorationEvent, ExplorationState, RecoveryAction};

/// Exploration controller
///
/// Implements a state machine for autonomous frontier-based exploration.
pub struct ExplorationController {
    /// Current state
    state: ExplorationState,
    /// Configuration
    config: ExplorationConfig,
    /// Consecutive failure count
    consecutive_failures: usize,
    /// Total frontiers explored
    frontiers_explored: usize,
    /// Total distance traveled (approximate)
    distance_traveled: f32,
    /// Last known robot pose
    last_pose: Pose2D,
    /// Recently failed frontier centroids (to avoid re-selecting)
    failed_frontier_centroids: Vec<WorldPoint>,
    /// Steps since last significant progress during navigation
    steps_without_progress: usize,
    /// Last recorded position for stuck detection
    stuck_check_position: WorldPoint,
    /// Consecutive StartBlocked events (localization drift)
    start_blocked_count: usize,
}

impl ExplorationController {
    /// Create a new exploration controller
    pub fn new(config: ExplorationConfig) -> Self {
        Self {
            state: ExplorationState::Idle,
            config,
            consecutive_failures: 0,
            frontiers_explored: 0,
            distance_traveled: 0.0,
            last_pose: Pose2D::default(),
            failed_frontier_centroids: Vec::new(),
            steps_without_progress: 0,
            stuck_check_position: WorldPoint::ZERO,
            start_blocked_count: 0,
        }
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(ExplorationConfig::default())
    }

    /// Get current state
    pub fn state(&self) -> &ExplorationState {
        &self.state
    }

    /// Get configuration
    pub fn config(&self) -> &ExplorationConfig {
        &self.config
    }

    /// Get exploration progress
    pub fn progress(&self) -> ExplorationProgress {
        ExplorationProgress {
            state: self.state.name().to_string(),
            frontiers_explored: self.frontiers_explored,
            distance_traveled: self.distance_traveled,
            consecutive_failures: self.consecutive_failures,
            is_active: self.state.is_active(),
            is_complete: matches!(self.state, ExplorationState::Complete),
        }
    }

    /// Start exploration
    pub fn start(&mut self) {
        if matches!(self.state, ExplorationState::Idle) {
            self.state = ExplorationState::SearchingFrontiers;
            self.consecutive_failures = 0;
        }
    }

    /// Stop exploration
    pub fn stop(&mut self) {
        self.state = ExplorationState::Idle;
    }

    /// Update controller with current pose and map
    ///
    /// Returns the command to execute
    pub fn update(&mut self, pose: Pose2D, storage: &GridStorage) -> ExplorationCommand {
        // Track distance traveled
        let dist = pose.position().distance(&self.last_pose.position());
        if dist > 0.01 {
            // Only count movement > 1cm
            self.distance_traveled += dist;
        }
        self.last_pose = pose;

        match &self.state {
            ExplorationState::Idle => ExplorationCommand::None,

            ExplorationState::SearchingFrontiers => self.search_frontiers(pose, storage),

            ExplorationState::Planning { target } => self.plan_path(pose, target.clone(), storage),

            ExplorationState::Navigating {
                path,
                current_waypoint,
                target,
            } => self.navigate(
                pose,
                path.clone(),
                *current_waypoint,
                target.clone(),
                storage,
            ),

            ExplorationState::Scanning {
                position,
                rotation_remaining,
            } => self.handle_scanning(*position, *rotation_remaining, pose),

            ExplorationState::Recovering { action, attempts } => {
                self.handle_recovery(action.clone(), *attempts)
            }

            ExplorationState::Complete => ExplorationCommand::ExplorationComplete,

            ExplorationState::Failed { reason } => ExplorationCommand::ExplorationFailed {
                reason: reason.clone(),
            },
        }
    }

    /// Handle an external event.
    ///
    /// Events are used to notify the controller of external conditions
    /// that should trigger state transitions.
    pub fn handle_event(&mut self, event: ExplorationEvent) {
        match event {
            ExplorationEvent::Start => self.start(),
            ExplorationEvent::Stop => self.stop(),
            ExplorationEvent::RecoveryComplete => {
                // Go back to searching after recovery
                self.state = ExplorationState::SearchingFrontiers;
            }
            ExplorationEvent::ScanComplete => {
                // Finished scanning, look for next frontier
                self.frontiers_explored += 1;
                self.consecutive_failures = 0;
                self.state = ExplorationState::SearchingFrontiers;
            }
            ExplorationEvent::ObstacleDetected | ExplorationEvent::PathBlocked => {
                self.enter_recovery();
            }
        }
    }

    // Private state handlers

    fn search_frontiers(&mut self, pose: Pose2D, storage: &GridStorage) -> ExplorationCommand {
        let detector = FrontierDetector::with_min_size(self.config.min_frontier_size);
        let robot_grid = storage.world_to_grid(pose.position());

        // Get all frontiers and filter out recently failed ones
        let frontiers = detector.detect_frontiers(storage);
        let total_frontiers = frontiers.len();
        let blacklisted_count = frontiers
            .iter()
            .filter(|f| self.is_recently_failed_frontier(&f.centroid_world))
            .count();

        let valid_frontiers: Vec<_> = frontiers
            .into_iter()
            .filter(|f| !self.is_recently_failed_frontier(&f.centroid_world))
            .collect();

        debug!(
            "[Explore] SearchingFrontiers: {} total, {} blacklisted, {} valid",
            total_frontiers,
            blacklisted_count,
            valid_frontiers.len()
        );

        // Find the best frontier from the valid ones
        let best = valid_frontiers.into_iter().max_by(|a, b| {
            let distance_a = robot_grid.manhattan_distance(&a.centroid_grid) as f32;
            let distance_b = robot_grid.manhattan_distance(&b.centroid_grid) as f32;
            let score_a = if distance_a > 0.0 {
                a.exploration_score() / (1.0 + distance_a * 0.1)
            } else {
                a.exploration_score() * 2.0
            };
            let score_b = if distance_b > 0.0 {
                b.exploration_score() / (1.0 + distance_b * 0.1)
            } else {
                b.exploration_score() * 2.0
            };
            score_a.total_cmp(&score_b)
        });

        if let Some(frontier) = best {
            let distance = robot_grid.manhattan_distance(&frontier.centroid_grid);
            info!(
                "[Explore] Selected frontier: ({:.2}, {:.2}), size={}, score={:.1}, dist={}",
                frontier.centroid_world.x,
                frontier.centroid_world.y,
                frontier.size,
                frontier.exploration_score(),
                distance
            );
            // Reset stuck detection when starting to plan for a new frontier
            self.steps_without_progress = 0;
            self.stuck_check_position = pose.position();
            self.state = ExplorationState::Planning { target: frontier };
            ExplorationCommand::None
        } else {
            // Check if map has enough explored area before declaring complete
            // If we have very few floor cells, we haven't really started exploring yet
            let counts = storage.count_by_type();
            let resolution = storage.resolution();
            let explored_area_m2 = counts.floor as f32 * resolution * resolution;

            if counts.floor < 100 {
                // Not enough explored yet - wait for more sensor data
                debug!(
                    "[Explore] No valid frontiers, but only {} floor cells ({:.2}m²) - waiting",
                    counts.floor, explored_area_m2
                );
                ExplorationCommand::None
            } else {
                // No frontiers and we have explored a reasonable amount - complete
                info!(
                    "[Explore] COMPLETE: No frontiers remaining. Floor={} ({:.2}m²), Wall={}, Unknown={}",
                    counts.floor, explored_area_m2, counts.wall, counts.unknown
                );
                self.state = ExplorationState::Complete;
                ExplorationCommand::ExplorationComplete
            }
        }
    }

    /// Check if a frontier centroid is in the recently failed list
    fn is_recently_failed_frontier(&self, centroid: &WorldPoint) -> bool {
        self.failed_frontier_centroids
            .iter()
            .any(|failed| centroid.distance(failed) < self.config.frontier_blacklist_tolerance)
    }

    fn plan_path(
        &mut self,
        pose: Pose2D,
        target: Frontier,
        storage: &GridStorage,
    ) -> ExplorationCommand {
        let astar_config = AStarConfig {
            footprint: self.config.footprint.clone(),
            clearance_weight: self.config.path_clearance_weight,
            exploration_mode: true, // Allow Unknown cells in footprint during exploration
            ..Default::default()
        };

        let robot_pos = pose.position();
        let robot_grid = storage.world_to_grid(robot_pos);

        // Try to find a valid observation point for this frontier
        let goal_world = match self.find_observation_point(&target, robot_pos, robot_grid, storage)
        {
            Some(goal) => goal,
            None => {
                // No valid observation point found - blacklist this frontier but don't
                // count as a "real" failure (it's a goal selection issue, not pathfinding)
                debug!(
                    "[Explore] No valid observation point for frontier at ({:.2},{:.2}), skipping",
                    target.centroid_world.x, target.centroid_world.y
                );
                self.failed_frontier_centroids.push(target.centroid_world);
                const MAX_FAILED_FRONTIERS: usize = 20;
                if self.failed_frontier_centroids.len() > MAX_FAILED_FRONTIERS {
                    self.failed_frontier_centroids.remove(0);
                }
                self.state = ExplorationState::SearchingFrontiers;
                return ExplorationCommand::None;
            }
        };

        trace!(
            "[Explore] Planning: robot=({:.2},{:.2}) → goal=({:.2},{:.2}), frontier=({:.2},{:.2})",
            robot_pos.x,
            robot_pos.y,
            goal_world.x,
            goal_world.y,
            target.centroid_world.x,
            target.centroid_world.y
        );

        let planner = AStarPlanner::new(storage, astar_config.clone());
        let result = planner.find_path_world(robot_pos, goal_world);

        if result.success {
            // Reset failure counts on successful planning
            self.consecutive_failures = 0;
            self.start_blocked_count = 0;

            let path_length_m = result.length_meters();
            let path = if self.config.smooth_paths {
                let smoother_config = SmoothingConfig {
                    footprint: self.config.footprint.clone(),
                    ..Default::default()
                };
                let smoother = PathSmoother::new(storage, smoother_config);
                smoother.smooth(&result.path_world)
            } else {
                result.path_world
            };

            info!(
                "[Explore] Planning SUCCESS: path length={} waypoints, {:.2}m",
                path.len(),
                path_length_m
            );

            self.state = ExplorationState::Navigating {
                path,
                current_waypoint: 0,
                target,
            };
            ExplorationCommand::None
        } else {
            // Planning failed - handle different failure types
            let failure_reason = result.failure_reason.clone();

            // StartBlocked: Robot's current position is considered blocked.
            // This typically happens due to localization drift - trigger recovery
            // to move the robot to a safer position.
            let is_start_blocked = matches!(failure_reason, Some(PathFailure::StartBlocked));
            if is_start_blocked {
                self.start_blocked_count += 1;

                // After too many consecutive StartBlocked events, the robot is likely stuck
                // due to localization drift. Check if we've explored enough to call it complete.
                const MAX_START_BLOCKED: usize = 10;
                if self.start_blocked_count >= MAX_START_BLOCKED {
                    let counts = storage.count_by_type();
                    let explored_area_m2 =
                        counts.floor as f32 * storage.resolution() * storage.resolution();

                    if explored_area_m2 > 10.0 && self.frontiers_explored > 0 {
                        info!(
                            "[Explore] COMPLETE (localization drift): explored {:.2}m², {} frontiers done after {} StartBlocked events",
                            explored_area_m2, self.frontiers_explored, self.start_blocked_count
                        );
                        self.state = ExplorationState::Complete;
                        return ExplorationCommand::ExplorationComplete;
                    } else {
                        warn!(
                            "[Explore] {} consecutive StartBlocked events, only {:.2}m² explored - entering recovery",
                            self.start_blocked_count, explored_area_m2
                        );
                    }
                }

                debug!(
                    "[Explore] StartBlocked at robot position ({}/{})",
                    self.start_blocked_count, MAX_START_BLOCKED
                );
                self.enter_recovery();
                return ExplorationCommand::Stop;
            } else {
                // Reset counter on non-StartBlocked events
                self.start_blocked_count = 0;
            }

            // GoalBlocked means our observation point selection failed (shouldn't happen after
            // find_observation_point, but handle it gracefully). Don't count as consecutive failure.
            let is_goal_blocked = matches!(failure_reason, Some(PathFailure::GoalBlocked));

            if is_goal_blocked {
                debug!(
                    "[Explore] Goal blocked at ({:.2},{:.2}), blacklisting frontier",
                    goal_world.x, goal_world.y
                );
            } else {
                warn!(
                    "[Explore] Planning FAILED to ({:.2},{:.2}): {:?}, failures={}/{}",
                    goal_world.x,
                    goal_world.y,
                    failure_reason,
                    self.consecutive_failures + 1,
                    self.config.max_consecutive_failures
                );
                self.consecutive_failures += 1;
            }

            self.failed_frontier_centroids.push(target.centroid_world);
            const MAX_FAILED_FRONTIERS: usize = 20;
            if self.failed_frontier_centroids.len() > MAX_FAILED_FRONTIERS {
                self.failed_frontier_centroids.remove(0);
            }

            if self.consecutive_failures >= self.config.max_consecutive_failures {
                // Before failing, check if we've explored a significant area
                let counts = storage.count_by_type();
                let explored_area_m2 =
                    counts.floor as f32 * storage.resolution() * storage.resolution();

                if explored_area_m2 > 10.0 && self.frontiers_explored > 0 {
                    info!(
                        "[Explore] COMPLETE (planning failures): explored {:.2}m², {} frontiers done",
                        explored_area_m2, self.frontiers_explored
                    );
                    self.state = ExplorationState::Complete;
                    return ExplorationCommand::ExplorationComplete;
                } else if explored_area_m2 > 20.0 {
                    info!(
                        "[Explore] COMPLETE (large area): explored {:.2}m² with {} frontiers",
                        explored_area_m2, self.frontiers_explored
                    );
                    self.state = ExplorationState::Complete;
                    return ExplorationCommand::ExplorationComplete;
                } else {
                    warn!(
                        "[Explore] FAILED: {} consecutive planning failures, only {:.2}m² explored",
                        self.consecutive_failures, explored_area_m2
                    );
                    self.state = ExplorationState::Failed {
                        reason: "Too many planning failures".to_string(),
                    };
                }
            } else {
                self.state = ExplorationState::SearchingFrontiers;
            }
            ExplorationCommand::None
        }
    }

    /// Find a valid observation point for a frontier.
    ///
    /// Tries multiple candidate positions near the frontier and returns the first
    /// one that is traversable (robot can fit there without hitting obstacles).
    fn find_observation_point(
        &self,
        target: &Frontier,
        robot_pos: WorldPoint,
        robot_grid: crate::core::GridCoord,
        storage: &GridStorage,
    ) -> Option<WorldPoint> {
        let checker = TraversabilityChecker::new(storage, self.config.footprint.clone());
        let offset_distance = self.config.footprint.total_radius() + storage.resolution() * 2.0;

        // Get the closest frontier cell to the robot
        let frontier_world = target
            .closest_cell_to(robot_grid)
            .map(|c| storage.grid_to_world(c.coord))
            .unwrap_or(target.centroid_world);

        // Generate candidate observation points
        let candidates = self.generate_observation_candidates(
            frontier_world,
            robot_pos,
            offset_distance,
            storage,
        );

        // Find the first valid candidate (traversable in exploration mode)
        for candidate in candidates {
            if checker.is_position_safe_mode(candidate, true) {
                trace!(
                    "[Explore] Found valid observation point at ({:.2},{:.2})",
                    candidate.x, candidate.y
                );
                return Some(candidate);
            }
        }

        // If no candidate worked, try frontier cells directly
        // (the robot might be able to observe from multiple cells in the frontier)
        for cell in target.cells.iter().take(10) {
            let cell_world = storage.grid_to_world(cell.coord);
            let dx = robot_pos.x - cell_world.x;
            let dy = robot_pos.y - cell_world.y;
            let dist = (dx * dx + dy * dy).sqrt();

            if dist > offset_distance {
                let ratio = offset_distance / dist;
                let candidate =
                    WorldPoint::new(cell_world.x + dx * ratio, cell_world.y + dy * ratio);

                if checker.is_position_safe_mode(candidate, true) {
                    trace!(
                        "[Explore] Found valid observation point from cell at ({:.2},{:.2})",
                        candidate.x, candidate.y
                    );
                    return Some(candidate);
                }
            }
        }

        None
    }

    /// Generate candidate observation points around a frontier.
    fn generate_observation_candidates(
        &self,
        frontier_world: WorldPoint,
        robot_pos: WorldPoint,
        offset_distance: f32,
        _storage: &GridStorage,
    ) -> Vec<WorldPoint> {
        let mut candidates = Vec::with_capacity(9);

        let dx = robot_pos.x - frontier_world.x;
        let dy = robot_pos.y - frontier_world.y;
        let dist = (dx * dx + dy * dy).sqrt();

        // Primary candidate: offset toward robot
        if dist > offset_distance {
            let ratio = offset_distance / dist;
            candidates.push(WorldPoint::new(
                frontier_world.x + dx * ratio,
                frontier_world.y + dy * ratio,
            ));
        } else if dist > 0.01 {
            // Robot is close, just move slightly toward robot
            candidates.push(robot_pos);
        }

        // Additional candidates at different angles around the frontier
        let base_angle = dy.atan2(dx);
        let angles = [
            0.0,
            std::f32::consts::FRAC_PI_4,
            -std::f32::consts::FRAC_PI_4,
            std::f32::consts::FRAC_PI_2,
            -std::f32::consts::FRAC_PI_2,
            std::f32::consts::FRAC_PI_4 * 3.0,
            -std::f32::consts::FRAC_PI_4 * 3.0,
            std::f32::consts::PI,
        ];

        for angle_offset in angles {
            let angle = base_angle + angle_offset;
            candidates.push(WorldPoint::new(
                frontier_world.x + offset_distance * angle.cos(),
                frontier_world.y + offset_distance * angle.sin(),
            ));
        }

        candidates
    }

    fn navigate(
        &mut self,
        pose: Pose2D,
        path: Vec<WorldPoint>,
        current_waypoint: usize,
        target: Frontier,
        _storage: &GridStorage,
    ) -> ExplorationCommand {
        if current_waypoint >= path.len() {
            // Reached end of path - start scanning rotation
            self.state = ExplorationState::Scanning {
                position: pose.position(),
                rotation_remaining: std::f32::consts::TAU, // Full 360° rotation
            };
            return ExplorationCommand::Stop;
        }

        let waypoint = path[current_waypoint];
        let distance = pose.position().distance(&waypoint);

        // Check if waypoint reached
        if distance < self.config.waypoint_threshold {
            // Move to next waypoint
            let next_waypoint = current_waypoint + 1;

            // Check if we've reached the frontier (within threshold of centroid)
            let distance_to_target = pose.position().distance(&target.centroid_world);
            if distance_to_target < self.config.frontier_reached_threshold {
                self.state = ExplorationState::Scanning {
                    position: pose.position(),
                    rotation_remaining: std::f32::consts::TAU,
                };
                return ExplorationCommand::Stop;
            }

            if next_waypoint < path.len() {
                let next_target = path[next_waypoint];
                self.state = ExplorationState::Navigating {
                    path,
                    current_waypoint: next_waypoint,
                    target,
                };
                return ExplorationCommand::MoveTo {
                    target: next_target,
                    max_speed: self.config.navigation_max_speed,
                };
            } else {
                // Reached end of path - transition to Scanning
                // (we're close enough to observe the frontier)
                self.state = ExplorationState::Scanning {
                    position: pose.position(),
                    rotation_remaining: std::f32::consts::TAU,
                };
                return ExplorationCommand::Stop;
            }
        }

        // Stuck detection: only check when robot should be moving forward
        // Calculate angle to waypoint
        let dx = waypoint.x - pose.x;
        let dy = waypoint.y - pose.y;
        let target_angle = dy.atan2(dx);
        let angle_error = (target_angle - pose.theta).abs();
        // Normalize to [0, PI]
        let angle_error = if angle_error > std::f32::consts::PI {
            std::f32::consts::TAU - angle_error
        } else {
            angle_error
        };

        // Only check stuck if angle is small (robot should be moving forward)
        if angle_error < 0.5 {
            // ~30 degrees
            let progress = pose.position().distance(&self.stuck_check_position);
            if progress > self.config.min_progress_distance {
                // Made meaningful progress - reset stuck counter
                self.steps_without_progress = 0;
                self.stuck_check_position = pose.position();
            } else {
                self.steps_without_progress += 1;
            }

            // If stuck for too long, trigger recovery
            if self.steps_without_progress > self.config.max_stuck_steps {
                warn!(
                    "[Explore] Stuck detected! {} steps without progress at ({:.2},{:.2})",
                    self.steps_without_progress, pose.x, pose.y
                );
                self.steps_without_progress = 0;
                // Add this frontier to failed list and try recovery
                self.failed_frontier_centroids.push(target.centroid_world);
                self.enter_recovery();
                return ExplorationCommand::Stop;
            }
        } else {
            // Robot is rotating to face waypoint - reset stuck counter
            self.steps_without_progress = 0;
        }

        // Continue to current waypoint
        ExplorationCommand::MoveTo {
            target: waypoint,
            max_speed: self.config.navigation_max_speed,
        }
    }

    fn handle_scanning(
        &mut self,
        position: WorldPoint,
        rotation_remaining: f32,
        pose: Pose2D,
    ) -> ExplorationCommand {
        // Rotate to scan the area - each update rotates by a fixed amount
        let rotation_step = self.config.scan_rotation_step;
        let new_remaining = rotation_remaining - rotation_step;

        if new_remaining <= 0.0 {
            // Scan complete - frontier successfully explored
            self.frontiers_explored += 1;
            self.consecutive_failures = 0;
            self.failed_frontier_centroids.clear(); // Reset on successful exploration
            info!(
                "[Explore] Scan complete at ({:.2},{:.2}), frontiers_explored={}",
                position.x, position.y, self.frontiers_explored
            );
            self.state = ExplorationState::SearchingFrontiers;
            ExplorationCommand::None
        } else {
            // Continue rotating
            self.state = ExplorationState::Scanning {
                position,
                rotation_remaining: new_remaining,
            };
            ExplorationCommand::Rotate {
                target_heading: pose.theta + rotation_step,
                max_angular_speed: self.config.scan_angular_speed,
            }
        }
    }

    fn handle_recovery(&mut self, action: RecoveryAction, attempts: usize) -> ExplorationCommand {
        const MAX_RECOVERY_ATTEMPTS: usize = 3;

        debug!(
            "[Explore] Recovery: {:?}, attempt {}/{}",
            action,
            attempts + 1,
            MAX_RECOVERY_ATTEMPTS
        );

        if attempts >= MAX_RECOVERY_ATTEMPTS {
            // Too many recovery attempts
            self.consecutive_failures += 1;
            warn!(
                "[Explore] Recovery exhausted after {} attempts, failures={}/{}",
                MAX_RECOVERY_ATTEMPTS,
                self.consecutive_failures,
                self.config.max_consecutive_failures
            );
            if self.consecutive_failures >= self.config.max_consecutive_failures {
                self.state = ExplorationState::Failed {
                    reason: "Too many recovery failures".to_string(),
                };
            } else {
                self.state = ExplorationState::SearchingFrontiers;
            }
            return ExplorationCommand::None;
        }

        match action {
            RecoveryAction::BackUp(distance) => {
                // After backing up, try turning
                self.state = ExplorationState::Recovering {
                    action: RecoveryAction::TurnInPlace(self.config.recovery_turn_angle),
                    attempts: attempts + 1,
                };
                ExplorationCommand::MoveTo {
                    target: WorldPoint::new(
                        self.last_pose.x - distance * self.last_pose.theta.cos(),
                        self.last_pose.y - distance * self.last_pose.theta.sin(),
                    ),
                    max_speed: self.config.recovery_max_speed,
                }
            }
            RecoveryAction::TurnInPlace(angle) => {
                // After turning, go back to searching
                self.state = ExplorationState::SearchingFrontiers;
                ExplorationCommand::Rotate {
                    target_heading: self.last_pose.theta + angle,
                    max_angular_speed: self.config.scan_angular_speed,
                }
            }
            RecoveryAction::Wait => {
                // Just wait, then try again
                self.state = ExplorationState::SearchingFrontiers;
                ExplorationCommand::None
            }
        }
    }

    fn enter_recovery(&mut self) {
        self.state = ExplorationState::Recovering {
            action: RecoveryAction::BackUp(self.config.recovery_backup_distance),
            attempts: 0,
        };
    }
}

/// Exploration progress tracking
#[derive(Clone, Debug, Default)]
pub struct ExplorationProgress {
    /// Current state name
    pub state: String,
    /// Number of frontiers explored
    pub frontiers_explored: usize,
    /// Total distance traveled (meters)
    pub distance_traveled: f32,
    /// Current consecutive failure count
    pub consecutive_failures: usize,
    /// Is exploration active?
    pub is_active: bool,
    /// Is exploration complete?
    pub is_complete: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{CellType, GridCoord};

    fn create_test_storage() -> GridStorage {
        GridStorage::centered(50, 50, 0.1) // 5m x 5m at 10cm resolution
    }

    fn fill_floor_with_frontiers(storage: &mut GridStorage) {
        // Create a floor area with unknown edges (frontiers)
        for x in 10..40 {
            for y in 10..40 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }
    }

    #[test]
    fn test_controller_creation() {
        let controller = ExplorationController::with_defaults();
        assert!(matches!(controller.state(), ExplorationState::Idle));
    }

    #[test]
    fn test_start_exploration() {
        let mut controller = ExplorationController::with_defaults();
        controller.start();
        assert!(matches!(
            controller.state(),
            ExplorationState::SearchingFrontiers
        ));
    }

    #[test]
    fn test_stop_exploration() {
        let mut controller = ExplorationController::with_defaults();
        controller.start();
        controller.stop();
        assert!(matches!(controller.state(), ExplorationState::Idle));
    }

    #[test]
    fn test_exploration_finds_frontiers() {
        let mut storage = create_test_storage();
        fill_floor_with_frontiers(&mut storage);

        let config = ExplorationConfig {
            footprint: crate::query::RobotFootprint::new(0.05, 0.01),
            ..Default::default()
        };
        let mut controller = ExplorationController::new(config);

        controller.start();

        // Update with a pose inside the floor area
        let pose = Pose2D::new(2.0, 2.0, 0.0);
        let _cmd = controller.update(pose, &storage);

        // Should be planning or navigating after finding frontier
        assert!(
            matches!(controller.state(), ExplorationState::Planning { .. })
                || matches!(controller.state(), ExplorationState::Navigating { .. })
        );
    }

    #[test]
    fn test_exploration_completes_when_no_frontiers() {
        let mut storage = create_test_storage();
        // Larger fully explored area (no frontiers - all cells are known)
        // Need >100 floor cells for exploration to complete
        let center = storage.world_to_grid(WorldPoint::ZERO);
        for dx in -6..=6 {
            for dy in -6..=6 {
                let coord = center + GridCoord::new(dx, dy);
                storage.set_type(coord, CellType::Floor);
            }
        }

        let config = ExplorationConfig {
            footprint: crate::query::RobotFootprint::new(0.05, 0.01),
            // Very large min size means no frontiers found (floor area is ~13x13 = 169 cells,
            // frontier edges would be ~48 cells, so 100 ensures none qualify)
            min_frontier_size: 100,
            ..Default::default()
        };
        let mut controller = ExplorationController::new(config);

        controller.start();
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let cmd = controller.update(pose, &storage);

        assert!(matches!(controller.state(), ExplorationState::Complete));
        assert!(matches!(cmd, ExplorationCommand::ExplorationComplete));
    }

    #[test]
    fn test_exploration_progress() {
        let controller = ExplorationController::with_defaults();
        let progress = controller.progress();

        assert_eq!(progress.state, "Idle");
        assert_eq!(progress.frontiers_explored, 0);
        assert!(!progress.is_active);
        assert!(!progress.is_complete);
    }
}
