//! Exploration controller state machine.
//!
//! Manages frontier-based exploration with collision recovery.
//! The primary goal is complete map coverage, with obstacle avoidance
//! as the secondary priority.

use crate::core::{Point2D, Pose2D};
use crate::{Frontier, Map, Path};

use super::collision::{CollisionEvent, VirtualWall};
use super::config::ExplorationConfig;
use super::path_follower::{PathFollower, VelocityCommand};

/// Current exploration state.
#[derive(Clone, Debug)]
pub enum ExplorationState {
    /// Idle, waiting for start.
    Idle,

    /// Selecting next frontier to explore.
    SelectingFrontier,

    /// Following path to target frontier.
    FollowingPath {
        /// Current path being followed.
        path: Path,
        /// Current waypoint index.
        waypoint_idx: usize,
        /// Target frontier point.
        target_frontier: Point2D,
    },

    /// Recovering from collision (backing off).
    RecoveringFromCollision {
        /// Distance remaining to back off (meters).
        backoff_remaining: f32,
        /// Heading to back off towards (radians).
        backoff_heading: f32,
        /// Position when backoff started.
        start_position: Point2D,
    },

    /// Exploration complete - no frontiers remain.
    Complete,

    /// Exploration failed (stuck, unreachable frontiers).
    Failed {
        /// Reason for failure.
        reason: String,
    },
}

impl ExplorationState {
    /// Get a short description of the state.
    pub fn name(&self) -> &'static str {
        match self {
            ExplorationState::Idle => "Idle",
            ExplorationState::SelectingFrontier => "SelectingFrontier",
            ExplorationState::FollowingPath { .. } => "FollowingPath",
            ExplorationState::RecoveringFromCollision { .. } => "RecoveringFromCollision",
            ExplorationState::Complete => "Complete",
            ExplorationState::Failed { .. } => "Failed",
        }
    }
}

/// Result of a single exploration step.
#[derive(Clone, Debug)]
pub struct ExplorationStep {
    /// Velocity command to execute (None if no action needed).
    pub velocity: Option<VelocityCommand>,

    /// Current state after this step.
    pub state: ExplorationState,

    /// Virtual wall to add to map (if collision occurred).
    pub virtual_wall: Option<VirtualWall>,

    /// Whether a new path was planned this step.
    pub path_planned: bool,

    /// Current target frontier (if any).
    pub target_frontier: Option<Point2D>,

    /// Distance to current target.
    pub distance_to_target: f32,
}

impl ExplorationStep {
    /// Create a step with no action.
    fn idle(state: ExplorationState) -> Self {
        Self {
            velocity: None,
            state,
            virtual_wall: None,
            path_planned: false,
            target_frontier: None,
            distance_to_target: 0.0,
        }
    }

    /// Create a step with a velocity command.
    fn with_velocity(velocity: VelocityCommand, state: ExplorationState) -> Self {
        Self {
            velocity: Some(velocity),
            state,
            virtual_wall: None,
            path_planned: false,
            target_frontier: None,
            distance_to_target: 0.0,
        }
    }
}

/// Exploration controller.
///
/// Manages autonomous exploration using frontier-based strategy:
///
/// 1. Detect frontiers (unexplored areas)
/// 2. Select the best frontier based on distance and openness
/// 3. Plan a path to the frontier
/// 4. Follow the path while avoiding obstacles
/// 5. Handle collisions by adding virtual walls and replanning
/// 6. Repeat until no frontiers remain
///
/// # Primary Goals
///
/// 1. **Complete map coverage** - Explore every reachable area
/// 2. **Avoid obstacles** - Navigate safely using path planning with robot radius clearance
///
/// # Usage
///
/// ```rust,ignore
/// use vastu_map::exploration::{ExplorationController, ExplorationConfig, CollisionEvent};
/// use vastu_map::{VectorMap, Pose2D};
///
/// let config = ExplorationConfig::default();
/// let mut controller = ExplorationController::new(config);
/// controller.start();
///
/// loop {
///     let map = get_slam_map();
///     let pose = get_current_pose();
///     let collision = check_sensors();
///
///     let step = controller.update(&map, pose, collision);
///
///     if let Some(wall) = step.virtual_wall {
///         add_wall_to_map(wall);
///     }
///
///     if let Some(vel) = step.velocity {
///         send_velocity_command(vel);
///     }
///
///     if controller.is_complete() {
///         break;
///     }
/// }
/// ```
pub struct ExplorationController {
    /// Current exploration state.
    state: ExplorationState,

    /// Path follower for smooth motion.
    path_follower: PathFollower,

    /// Configuration.
    config: ExplorationConfig,

    /// Number of consecutive failed path planning attempts.
    failed_planning_attempts: usize,

    /// Maximum failed attempts before giving up on a frontier.
    max_failed_attempts: usize,

    /// Frontiers that have been marked as unreachable.
    unreachable_frontiers: Vec<Point2D>,

    /// Distance threshold to consider frontiers as same location.
    frontier_match_distance: f32,
}

impl ExplorationController {
    /// Create a new exploration controller.
    pub fn new(config: ExplorationConfig) -> Self {
        let path_follower = PathFollower::new(&config);

        Self {
            state: ExplorationState::Idle,
            path_follower,
            config,
            failed_planning_attempts: 0,
            max_failed_attempts: 3,
            unreachable_frontiers: Vec::new(),
            frontier_match_distance: 0.5,
        }
    }

    /// Start exploration.
    pub fn start(&mut self) {
        self.state = ExplorationState::SelectingFrontier;
        self.failed_planning_attempts = 0;
        self.unreachable_frontiers.clear();
    }

    /// Reset to idle state.
    pub fn reset(&mut self) {
        self.state = ExplorationState::Idle;
        self.failed_planning_attempts = 0;
        self.unreachable_frontiers.clear();
    }

    /// Get current state.
    pub fn state(&self) -> &ExplorationState {
        &self.state
    }

    /// Check if exploration is complete.
    pub fn is_complete(&self) -> bool {
        matches!(self.state, ExplorationState::Complete)
    }

    /// Check if exploration failed.
    pub fn is_failed(&self) -> bool {
        matches!(self.state, ExplorationState::Failed { .. })
    }

    /// Check if exploration is active (not idle, complete, or failed).
    pub fn is_active(&self) -> bool {
        !matches!(
            self.state,
            ExplorationState::Idle | ExplorationState::Complete | ExplorationState::Failed { .. }
        )
    }

    /// Main update function - call every control loop iteration.
    ///
    /// # Arguments
    /// * `map` - Current SLAM map (implements Map trait)
    /// * `current_pose` - Current robot pose estimate
    /// * `collision` - Collision event if any
    ///
    /// # Returns
    /// Exploration step with velocity command and state updates.
    pub fn update<M: Map>(
        &mut self,
        map: &M,
        current_pose: Pose2D,
        collision: Option<CollisionEvent>,
    ) -> ExplorationStep {
        // Handle collision first (highest priority for safety)
        if let Some(event) = collision {
            return self.handle_collision(event, current_pose);
        }

        // State machine
        match self.state.clone() {
            ExplorationState::Idle => ExplorationStep::idle(self.state.clone()),

            ExplorationState::SelectingFrontier => self.select_frontier(map, current_pose),

            ExplorationState::FollowingPath {
                path,
                waypoint_idx,
                target_frontier,
            } => self.follow_path(map, current_pose, path, waypoint_idx, target_frontier),

            ExplorationState::RecoveringFromCollision {
                backoff_remaining,
                backoff_heading,
                start_position,
            } => self.execute_backoff(
                current_pose,
                backoff_remaining,
                backoff_heading,
                start_position,
            ),

            ExplorationState::Complete => ExplorationStep::idle(self.state.clone()),

            ExplorationState::Failed { .. } => ExplorationStep::idle(self.state.clone()),
        }
    }

    /// Handle a collision event.
    fn handle_collision(&mut self, event: CollisionEvent, current_pose: Pose2D) -> ExplorationStep {
        log::info!(
            "Collision detected: {:?} at ({:.2}, {:.2})",
            event.collision_type,
            event.point.x,
            event.point.y
        );

        // Create virtual wall at collision point
        let virtual_wall = event.to_virtual_wall(self.config.virtual_wall_length);

        // Transition to recovery state
        self.state = ExplorationState::RecoveringFromCollision {
            backoff_remaining: self.config.backoff_distance,
            backoff_heading: event.backoff_heading(),
            start_position: current_pose.position(),
        };

        // Return stop command with virtual wall
        let mut step = ExplorationStep::with_velocity(VelocityCommand::stop(), self.state.clone());
        step.virtual_wall = Some(virtual_wall);
        step
    }

    /// Select the next frontier to explore.
    fn select_frontier<M: Map>(&mut self, map: &M, current_pose: Pose2D) -> ExplorationStep {
        let robot_pos = current_pose.position();

        // Get all frontiers
        let all_frontiers = map.frontiers();

        // Filter out unreachable frontiers
        let reachable_frontiers: Vec<&Frontier> = all_frontiers
            .iter()
            .filter(|f| !self.is_frontier_unreachable(f.point))
            .collect();

        if reachable_frontiers.is_empty() {
            if all_frontiers.is_empty() {
                // No frontiers at all - exploration complete!
                log::info!("Exploration complete: no frontiers remain");
                self.state = ExplorationState::Complete;
                return ExplorationStep::idle(self.state.clone());
            } else {
                // All frontiers are unreachable
                log::warn!(
                    "Exploration stuck: {} frontiers exist but all are unreachable",
                    all_frontiers.len()
                );
                self.state = ExplorationState::Failed {
                    reason: format!("{} frontiers unreachable", all_frontiers.len()),
                };
                return ExplorationStep::idle(self.state.clone());
            }
        }

        // Find closest reachable frontier
        let best_frontier = reachable_frontiers
            .iter()
            .min_by(|a, b| {
                let dist_a = robot_pos.distance(a.point);
                let dist_b = robot_pos.distance(b.point);
                dist_a
                    .partial_cmp(&dist_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|f| f.point);

        let target = match best_frontier {
            Some(p) => p,
            None => {
                self.state = ExplorationState::Complete;
                return ExplorationStep::idle(self.state.clone());
            }
        };

        log::debug!(
            "Selected frontier at ({:.2}, {:.2}), distance: {:.2}m",
            target.x,
            target.y,
            robot_pos.distance(target)
        );

        // Plan path to frontier
        match map.get_path(robot_pos, target) {
            Some(path) => {
                log::debug!(
                    "Path planned: {} waypoints, {:.2}m total",
                    path.points.len(),
                    path.length
                );

                self.failed_planning_attempts = 0;

                // Immediately start following the path
                let result = self.path_follower.follow(
                    current_pose,
                    &path,
                    0,
                    self.config.waypoint_tolerance,
                );

                self.state = ExplorationState::FollowingPath {
                    path: path.clone(),
                    waypoint_idx: result.waypoint_idx,
                    target_frontier: target,
                };

                let mut step = ExplorationStep::with_velocity(result.velocity, self.state.clone());
                step.path_planned = true;
                step.target_frontier = Some(target);
                step.distance_to_target = result.distance_to_waypoint;
                step
            }
            None => {
                log::warn!(
                    "Failed to plan path to frontier at ({:.2}, {:.2})",
                    target.x,
                    target.y
                );

                self.failed_planning_attempts += 1;

                if self.failed_planning_attempts >= self.max_failed_attempts {
                    // Mark this frontier as unreachable
                    self.unreachable_frontiers.push(target);
                    self.failed_planning_attempts = 0;
                    log::warn!(
                        "Frontier marked as unreachable after {} failed attempts",
                        self.max_failed_attempts
                    );
                }

                // Stay in SelectingFrontier to try another
                ExplorationStep::idle(self.state.clone())
            }
        }
    }

    /// Follow the current path.
    fn follow_path<M: Map>(
        &mut self,
        _map: &M,
        current_pose: Pose2D,
        path: Path,
        waypoint_idx: usize,
        target_frontier: Point2D,
    ) -> ExplorationStep {
        let result = self.path_follower.follow(
            current_pose,
            &path,
            waypoint_idx,
            self.config.waypoint_tolerance,
        );

        if result.path_complete {
            // Path complete - go back to selecting frontier
            log::debug!(
                "Reached frontier at ({:.2}, {:.2})",
                target_frontier.x,
                target_frontier.y
            );
            self.state = ExplorationState::SelectingFrontier;

            return ExplorationStep::idle(self.state.clone());
        }

        // Note: We don't check frontier existence every step for performance.
        // If the frontier closes while following, we'll select a new one when
        // the path completes or we get close enough.

        // Update state with new waypoint index
        self.state = ExplorationState::FollowingPath {
            path,
            waypoint_idx: result.waypoint_idx,
            target_frontier,
        };

        let mut step = ExplorationStep::with_velocity(result.velocity, self.state.clone());
        step.target_frontier = Some(target_frontier);
        step.distance_to_target = result.distance_to_waypoint;
        step
    }

    /// Execute backoff motion after collision.
    fn execute_backoff(
        &mut self,
        current_pose: Pose2D,
        backoff_remaining: f32,
        backoff_heading: f32,
        start_position: Point2D,
    ) -> ExplorationStep {
        // Calculate how far we've backed up
        let backed_up = current_pose.position().distance(start_position);
        let remaining = backoff_remaining - backed_up;

        if remaining <= 0.0 {
            // Backoff complete - go back to selecting frontier
            log::debug!("Backoff complete, selecting new frontier");
            self.state = ExplorationState::SelectingFrontier;
            return ExplorationStep::idle(self.state.clone());
        }

        // Update remaining distance
        self.state = ExplorationState::RecoveringFromCollision {
            backoff_remaining: remaining,
            backoff_heading,
            start_position,
        };

        // Command slow reverse motion
        let backup_speed = 0.1; // m/s
        ExplorationStep::with_velocity(VelocityCommand::backup(backup_speed), self.state.clone())
    }

    /// Check if a frontier point is marked as unreachable.
    fn is_frontier_unreachable(&self, point: Point2D) -> bool {
        self.unreachable_frontiers
            .iter()
            .any(|p| p.distance(point) < self.frontier_match_distance)
    }

    /// Get the number of unreachable frontiers.
    pub fn unreachable_frontier_count(&self) -> usize {
        self.unreachable_frontiers.len()
    }

    /// Clear unreachable frontiers (useful after map updates).
    pub fn clear_unreachable_frontiers(&mut self) {
        self.unreachable_frontiers.clear();
    }

    /// Get the current configuration.
    pub fn config(&self) -> &ExplorationConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Occupancy;
    use crate::core::Bounds;
    use crate::features::Line2D;

    /// Mock map for testing
    struct MockMap {
        frontiers: Vec<Frontier>,
        lines: Vec<Line2D>,
    }

    impl MockMap {
        fn new() -> Self {
            Self {
                frontiers: Vec::new(),
                lines: Vec::new(),
            }
        }

        fn with_frontier(mut self, x: f32, y: f32) -> Self {
            self.frontiers.push(Frontier {
                point: Point2D::new(x, y),
                line_idx: 0,
                is_start: true,
            });
            self
        }
    }

    impl Map for MockMap {
        fn observe(
            &mut self,
            _scan: &crate::core::PointCloud2D,
            _odometry: Pose2D,
        ) -> crate::ObserveResult {
            unimplemented!()
        }

        fn raycast(&self, _from: Point2D, _direction: Point2D, max_range: f32) -> f32 {
            max_range
        }

        fn query(&self, _point: Point2D) -> Occupancy {
            Occupancy::Free
        }

        fn frontiers(&self) -> Vec<Frontier> {
            self.frontiers.clone()
        }

        fn get_path(&self, from: Point2D, to: Point2D) -> Option<Path> {
            // Simple direct path
            Some(Path {
                points: vec![from, to],
                length: from.distance(to),
            })
        }

        fn is_path_clear(&self, _from: Point2D, _to: Point2D) -> bool {
            true
        }

        fn bounds(&self) -> Bounds {
            Bounds::new(Point2D::new(-10.0, -10.0), Point2D::new(10.0, 10.0))
        }

        fn clear(&mut self) {
            self.frontiers.clear();
            self.lines.clear();
        }
    }

    #[test]
    fn test_controller_initial_state() {
        let config = ExplorationConfig::default();
        let controller = ExplorationController::new(config);

        assert!(matches!(controller.state(), ExplorationState::Idle));
        assert!(!controller.is_active());
        assert!(!controller.is_complete());
    }

    #[test]
    fn test_controller_start() {
        let config = ExplorationConfig::default();
        let mut controller = ExplorationController::new(config);

        controller.start();

        assert!(matches!(
            controller.state(),
            ExplorationState::SelectingFrontier
        ));
        assert!(controller.is_active());
    }

    #[test]
    fn test_exploration_complete_no_frontiers() {
        let config = ExplorationConfig::default();
        let mut controller = ExplorationController::new(config);
        controller.start();

        let map = MockMap::new(); // No frontiers
        let pose = Pose2D::identity();

        let step = controller.update(&map, pose, None);

        assert!(controller.is_complete());
        assert!(step.velocity.is_none());
    }

    #[test]
    fn test_exploration_selects_frontier() {
        let config = ExplorationConfig::default();
        let mut controller = ExplorationController::new(config);
        controller.start();

        let map = MockMap::new().with_frontier(2.0, 0.0);
        let pose = Pose2D::identity();

        let step = controller.update(&map, pose, None);

        assert!(step.path_planned);
        assert!(step.target_frontier.is_some());
        assert!(matches!(
            controller.state(),
            ExplorationState::FollowingPath { .. }
        ));
    }

    #[test]
    fn test_collision_triggers_recovery() {
        let config = ExplorationConfig::default();
        let mut controller = ExplorationController::new(config);
        controller.start();

        let map = MockMap::new().with_frontier(2.0, 0.0);
        let pose = Pose2D::identity();

        // First update to start following path
        controller.update(&map, pose, None);

        // Simulate collision
        let collision = CollisionEvent::new(
            super::super::collision::CollisionType::BumperBoth,
            Point2D::new(0.5, 0.0),
            0.0,
        );

        let step = controller.update(&map, pose, Some(collision));

        assert!(step.virtual_wall.is_some());
        assert!(matches!(
            controller.state(),
            ExplorationState::RecoveringFromCollision { .. }
        ));
    }

    #[test]
    fn test_reset() {
        let config = ExplorationConfig::default();
        let mut controller = ExplorationController::new(config);
        controller.start();

        let map = MockMap::new().with_frontier(2.0, 0.0);
        let pose = Pose2D::identity();

        controller.update(&map, pose, None);
        controller.reset();

        assert!(matches!(controller.state(), ExplorationState::Idle));
        assert!(!controller.is_active());
    }
}
