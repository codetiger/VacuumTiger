//! Navigation executor - plans paths and follows them.
//!
//! The [`Navigator`] is the core navigation logic that:
//! 1. Plans paths to targets using A* planner
//! 2. Follows waypoints with rotation-first control
//! 3. Handles replanning on obstacles or significant deviation
//! 4. Manages the navigation state machine
//!
//! Note: Some config fields and methods are defined for future use.

use std::time::{Duration, Instant};

use crate::algorithms::planning::{AStarConfig, AStarPlanner, PlanningError};
use crate::core::math::normalize_angle;
use crate::core::types::Pose2D;
use crate::navigation::{
    MovementDirection, NavState, NavTarget, NavigationState, PathFailureReason, Waypoint,
};
use crate::state::CurrentMapData;

/// Configuration for the navigator.
#[derive(Debug, Clone)]
pub struct NavigatorConfig {
    /// A* planner configuration.
    pub planner: AStarConfig,

    /// Waypoint reached threshold (meters).
    ///
    /// How close the robot must be to a waypoint before advancing
    /// to the next one. Typically looser than goal tolerance.
    pub waypoint_reached_threshold: f32,

    /// Heading threshold for rotation (radians).
    ///
    /// Angle error threshold for switching from rotation to translation.
    pub heading_threshold: f32,

    /// Maximum linear velocity (m/s).
    pub max_linear_vel: f32,

    /// Maximum angular velocity (rad/s).
    pub max_angular_vel: f32,

    /// Default linear velocity (m/s).
    pub linear_vel: f32,

    /// Default angular velocity (rad/s).
    pub angular_vel: f32,

    /// Stuck timeout (seconds).
    ///
    /// If no progress is made for this duration, consider robot stuck.
    pub stuck_timeout_s: f32,

    /// Maximum replan attempts before failing.
    pub max_replan_attempts: u32,

    /// Minimum distance from path before replanning (meters).
    pub replan_deviation_threshold: f32,

    /// Escape duration (seconds).
    ///
    /// How long to back away from obstacle before replanning.
    pub escape_duration_s: f32,

    /// Escape velocity (m/s).
    ///
    /// Speed at which to back away from obstacles (negative = backward).
    pub escape_velocity: f32,

    /// Minimum clearance to achieve during escape (meters).
    pub escape_clearance_m: f32,
}

impl Default for NavigatorConfig {
    fn default() -> Self {
        Self {
            planner: AStarConfig::default(),
            waypoint_reached_threshold: 0.15,
            heading_threshold: 0.3, // ~17 degrees
            max_linear_vel: 0.3,
            max_angular_vel: 0.5,
            linear_vel: 0.2,
            angular_vel: 0.3,
            stuck_timeout_s: 5.0,
            max_replan_attempts: 3,
            replan_deviation_threshold: 0.3,
            escape_duration_s: 1.5,
            escape_velocity: -0.1, // Negative = backward
            escape_clearance_m: 0.15,
        }
    }
}

/// Result of a navigation update.
#[derive(Debug)]
pub struct NavUpdate {
    /// Commanded linear velocity (m/s).
    pub linear_vel: f32,

    /// Commanded angular velocity (rad/s).
    pub angular_vel: f32,

    /// Whether the navigation state changed.
    pub state_changed: bool,
}

impl NavUpdate {
    /// Create a stop command (zero velocities).
    pub fn stop() -> Self {
        Self {
            linear_vel: 0.0,
            angular_vel: 0.0,
            state_changed: false,
        }
    }

    /// Create a command with state change flag.
    pub fn with_state_change(mut self) -> Self {
        self.state_changed = true;
        self
    }
}

/// Navigation executor.
///
/// Call `update()` at a regular rate (e.g., 10Hz) with the current
/// robot pose and map data. The navigator will update the navigation
/// state and return velocity commands.
pub struct Navigator {
    /// Configuration.
    config: NavigatorConfig,

    /// A* path planner.
    planner: AStarPlanner,

    /// Last known position (for stuck detection).
    last_position: Option<(f32, f32)>,

    /// Last progress time (for stuck detection).
    last_progress_time: Instant,

    /// Distance threshold for progress (meters).
    progress_threshold: f32,
}

impl Navigator {
    /// Create a new navigator with the given configuration.
    pub fn new(config: NavigatorConfig) -> Self {
        let planner = AStarPlanner::new(config.planner.clone());

        Self {
            config,
            planner,
            last_position: None,
            last_progress_time: Instant::now(),
            progress_threshold: 0.02, // 2cm
        }
    }

    /// Get the configuration.
    pub fn config(&self) -> &NavigatorConfig {
        &self.config
    }

    /// Update the navigator with current pose and map.
    ///
    /// This should be called at a regular rate (e.g., 10Hz).
    ///
    /// # Returns
    ///
    /// - `NavUpdate` with velocity commands and state change flag.
    ///
    /// # Arguments
    ///
    /// - `nav_state`: Mutable reference to navigation state.
    /// - `pose`: Current robot pose.
    /// - `map`: Current occupancy grid map data.
    /// - `hazard`: Whether a hazard (bumper/cliff) is currently detected.
    pub fn update(
        &mut self,
        nav_state: &mut NavigationState,
        pose: &Pose2D,
        map: Option<&CurrentMapData>,
        hazard: bool,
    ) -> NavUpdate {
        // Handle hazard - escape before replanning
        if hazard && nav_state.is_navigating() {
            log::warn!("Hazard detected during navigation, escaping before replan...");
            // Calculate escape direction (opposite of current heading = backward)
            let escape_dir = pose.theta + std::f32::consts::PI;
            nav_state.start_escape(escape_dir);
            return NavUpdate {
                linear_vel: self.config.escape_velocity,
                angular_vel: 0.0,
                state_changed: true,
            };
        }

        match nav_state.nav_state {
            NavState::Idle | NavState::TargetReached | NavState::Failed | NavState::Cancelled => {
                // Nothing to do
                NavUpdate::stop()
            }

            NavState::Planning => {
                // Plan path to current target
                self.handle_planning(nav_state, pose, map)
            }

            NavState::Navigating => {
                // Follow current path
                self.handle_navigating(nav_state, pose)
            }

            NavState::Escaping => {
                // Back away from obstacle before replanning
                self.handle_escaping(nav_state, pose)
            }

            NavState::RotatingToHeading => {
                // Rotate to final heading
                self.handle_rotating_to_heading(nav_state, pose)
            }
        }
    }

    /// Handle the Planning state.
    fn handle_planning(
        &mut self,
        nav_state: &mut NavigationState,
        pose: &Pose2D,
        map: Option<&CurrentMapData>,
    ) -> NavUpdate {
        // Check if we've exceeded max replan attempts
        if nav_state.replan_attempts >= self.config.max_replan_attempts {
            log::warn!(
                "Navigation failed: exceeded {} replan attempts",
                self.config.max_replan_attempts
            );
            nav_state.set_failed(PathFailureReason::PathBlocked {
                replan_attempts: nav_state.replan_attempts,
            });
            nav_state.set_status(format!(
                "Path blocked after {} attempts",
                nav_state.replan_attempts
            ));
            return NavUpdate::stop().with_state_change();
        }

        // Get target info (clone what we need to avoid borrow issues)
        let (target_id, goal_x, goal_y, description) = match nav_state.current_target() {
            Some(t) => (t.id, t.position.x, t.position.y, t.description.clone()),
            None => {
                // No target, go idle
                nav_state.nav_state = NavState::Idle;
                return NavUpdate::stop().with_state_change();
            }
        };

        let map = match map {
            Some(m) => m,
            None => {
                // No map, fail
                nav_state.set_failed(PathFailureReason::NoPathFound);
                nav_state.set_status("No map available for planning");
                return NavUpdate::stop().with_state_change();
            }
        };

        // Check if we can find a path
        let start = (pose.x, pose.y);
        let goal = (goal_x, goal_y);

        log::info!(
            "Planning path from ({:.2}, {:.2}) to ({:.2}, {:.2})",
            start.0,
            start.1,
            goal.0,
            goal.1
        );

        // Plan the path
        match self.planner.plan(map, start, goal, target_id) {
            Ok(path) => {
                log::info!(
                    "Path found: {} waypoints, {:.2}m total",
                    path.len(),
                    path.total_length
                );

                nav_state.set_path(path);
                nav_state.set_status(format!("Navigating to {}", description));

                // Reset stuck detection
                self.last_position = Some((pose.x, pose.y));
                self.last_progress_time = Instant::now();

                NavUpdate::stop().with_state_change()
            }
            Err(PlanningError::NoPathFound) => {
                log::warn!("No path found after {} attempts", nav_state.replan_attempts);
                nav_state.set_failed(PathFailureReason::NoPathFound);
                NavUpdate::stop().with_state_change()
            }
            Err(PlanningError::StartInObstacle) => {
                log::warn!("Robot position is in obstacle");
                nav_state.set_failed(PathFailureReason::NoPathFound);
                nav_state.set_status("Robot position is blocked");
                NavUpdate::stop().with_state_change()
            }
            Err(PlanningError::GoalInObstacle) => {
                log::warn!("Goal position is in obstacle");
                nav_state.set_failed(PathFailureReason::TargetInvalid);
                nav_state.set_status("Goal is unreachable");
                NavUpdate::stop().with_state_change()
            }
            Err(PlanningError::StartOutOfBounds) | Err(PlanningError::GoalOutOfBounds) => {
                log::warn!("Position out of map bounds");
                nav_state.set_failed(PathFailureReason::NoPathFound);
                nav_state.set_status("Position out of bounds");
                NavUpdate::stop().with_state_change()
            }
            Err(PlanningError::MaxIterationsExceeded) => {
                log::warn!("Path planning exceeded max iterations");
                nav_state.set_failed(PathFailureReason::NoPathFound);
                nav_state.set_status("Path planning timeout");
                NavUpdate::stop().with_state_change()
            }
            Err(PlanningError::InvalidMap) => {
                log::warn!("Invalid map data");
                nav_state.set_failed(PathFailureReason::NoPathFound);
                nav_state.set_status("Invalid map");
                NavUpdate::stop().with_state_change()
            }
        }
    }

    /// Handle the Navigating state.
    fn handle_navigating(&mut self, nav_state: &mut NavigationState, pose: &Pose2D) -> NavUpdate {
        let target = match nav_state.current_target() {
            Some(t) => t.clone(),
            None => {
                nav_state.nav_state = NavState::Idle;
                return NavUpdate::stop().with_state_change();
            }
        };

        // Check if we've reached the goal position
        if target.is_position_reached(pose.x, pose.y) {
            log::info!(
                "Position reached for target {}: ({:.2}, {:.2})",
                target.id,
                target.position.x,
                target.position.y
            );

            // Check if we need to rotate to heading
            if target.heading.is_some() && !target.is_heading_reached(pose.theta) {
                nav_state.nav_state = NavState::RotatingToHeading;
                nav_state.set_status(format!("Rotating to heading for {}", target.description));
                return NavUpdate::stop().with_state_change();
            }

            // Target fully reached
            return self.complete_target(nav_state);
        }

        // Get current waypoint
        let (waypoint, waypoint_idx, total_waypoints) = match nav_state.current_waypoint() {
            Some(w) => {
                let idx = nav_state.current_waypoint_index;
                let total = nav_state.current_path().map(|p| p.len()).unwrap_or(0);
                (*w, idx, total)
            }
            None => {
                // No more waypoints but not at goal - replan
                log::debug!("NAV: No waypoints available, requesting replan");
                nav_state.request_replan();
                return NavUpdate::stop().with_state_change();
            }
        };

        // Check if waypoint reached
        let dx = waypoint.x - pose.x;
        let dy = waypoint.y - pose.y;
        let dist_to_waypoint = (dx * dx + dy * dy).sqrt();

        log::debug!(
            "NAV: Waypoint {}/{} at ({:.2},{:.2}), robot at ({:.2},{:.2},{:.1}°), dist={:.2}m, threshold={:.2}m",
            waypoint_idx + 1,
            total_waypoints,
            waypoint.x,
            waypoint.y,
            pose.x,
            pose.y,
            pose.theta.to_degrees(),
            dist_to_waypoint,
            self.config.waypoint_reached_threshold
        );

        if dist_to_waypoint < self.config.waypoint_reached_threshold {
            // Advance to next waypoint
            if !nav_state.advance_waypoint() {
                // Was last waypoint, check goal
                if target.is_position_reached(pose.x, pose.y) {
                    if target.heading.is_some() && !target.is_heading_reached(pose.theta) {
                        nav_state.nav_state = NavState::RotatingToHeading;
                        return NavUpdate::stop().with_state_change();
                    }
                    return self.complete_target(nav_state);
                }

                // Path is complete but not at exact goal position.
                // For frontiers/exploration, accept "close enough" to avoid getting stuck
                // trying to reach goals in obstacle inflation zones.
                // Use 1.5x waypoint threshold (typically ~22cm with default 15cm threshold).
                let goal_dx = target.position.x - pose.x;
                let goal_dy = target.position.y - pose.y;
                let goal_dist = (goal_dx * goal_dx + goal_dy * goal_dy).sqrt();
                let close_enough_threshold = self.config.waypoint_reached_threshold * 1.5;

                if goal_dist < close_enough_threshold {
                    log::info!(
                        "Path complete, accepting position as 'close enough' ({:.2}m from goal)",
                        goal_dist
                    );
                    if target.heading.is_some() && !target.is_heading_reached(pose.theta) {
                        nav_state.nav_state = NavState::RotatingToHeading;
                        return NavUpdate::stop().with_state_change();
                    }
                    return self.complete_target(nav_state);
                }
            }

            // Get new current waypoint
            let waypoint = match nav_state.current_waypoint() {
                Some(w) => *w,
                None => {
                    // Edge case: should go to goal
                    Waypoint::new(target.position.x, target.position.y)
                }
            };

            return self.compute_velocity_to_waypoint(pose, &waypoint, &target);
        }

        // Check for stuck condition
        if let Some((last_x, last_y)) = self.last_position {
            let moved = ((pose.x - last_x).powi(2) + (pose.y - last_y).powi(2)).sqrt();
            if moved > self.progress_threshold {
                self.last_position = Some((pose.x, pose.y));
                self.last_progress_time = Instant::now();
            } else if self.last_progress_time.elapsed()
                > Duration::from_secs_f32(self.config.stuck_timeout_s)
            {
                log::warn!(
                    "Robot stuck for {:.1}s, failing navigation",
                    self.config.stuck_timeout_s
                );
                nav_state.set_failed(PathFailureReason::Stuck {
                    stuck_duration_secs: self.config.stuck_timeout_s,
                });
                return NavUpdate::stop().with_state_change();
            }
        } else {
            self.last_position = Some((pose.x, pose.y));
            self.last_progress_time = Instant::now();
        }

        // Compute velocity to waypoint
        self.compute_velocity_to_waypoint(pose, &waypoint, &target)
    }

    /// Handle the Escaping state.
    ///
    /// Robot backs away from obstacle for a duration before replanning.
    fn handle_escaping(&mut self, nav_state: &mut NavigationState, pose: &Pose2D) -> NavUpdate {
        let elapsed = nav_state.escape_elapsed_secs();

        // Check if escape complete (duration elapsed)
        if elapsed >= self.config.escape_duration_s {
            log::info!(
                "Escape complete (elapsed={:.1}s), transitioning to replan",
                elapsed
            );
            nav_state.finish_escape_and_replan();

            // Reset stuck detection for the replan
            self.last_position = Some((pose.x, pose.y));
            self.last_progress_time = Instant::now();

            return NavUpdate::stop().with_state_change();
        }

        // Continue backing up
        log::debug!(
            "Escaping: {:.1}s / {:.1}s",
            elapsed,
            self.config.escape_duration_s
        );

        NavUpdate {
            linear_vel: self.config.escape_velocity,
            angular_vel: 0.0,
            state_changed: false,
        }
    }

    /// Handle the RotatingToHeading state.
    fn handle_rotating_to_heading(
        &mut self,
        nav_state: &mut NavigationState,
        pose: &Pose2D,
    ) -> NavUpdate {
        let target = match nav_state.current_target() {
            Some(t) => t.clone(),
            None => {
                nav_state.nav_state = NavState::Idle;
                return NavUpdate::stop().with_state_change();
            }
        };

        let target_heading = match target.heading {
            Some(h) => h,
            None => {
                // No heading required, target reached
                return self.complete_target(nav_state);
            }
        };

        // Check if heading reached
        if target.is_heading_reached(pose.theta) {
            log::info!("Heading reached for target {}", target.id);
            return self.complete_target(nav_state);
        }

        // Rotate toward target heading
        let angle_error = normalize_angle(target_heading - pose.theta);
        let angular_vel = angle_error.signum() * self.config.angular_vel;

        NavUpdate {
            linear_vel: 0.0,
            angular_vel: angular_vel
                .clamp(-self.config.max_angular_vel, self.config.max_angular_vel),
            state_changed: false,
        }
    }

    /// Complete the current target and move to next or idle.
    fn complete_target(&mut self, nav_state: &mut NavigationState) -> NavUpdate {
        let completed_target = nav_state.pop_target();

        if let Some(t) = completed_target {
            log::info!("Target {} completed: {}", t.id, t.description);
        }

        if nav_state.has_targets() {
            // More targets, will plan next
            nav_state.set_status("Planning next target...");
        } else {
            // All done
            nav_state.nav_state = NavState::TargetReached;
            nav_state.set_status("Navigation complete");
        }

        NavUpdate::stop().with_state_change()
    }

    /// Compute velocity command to reach a waypoint.
    fn compute_velocity_to_waypoint(
        &self,
        pose: &Pose2D,
        waypoint: &Waypoint,
        target: &NavTarget,
    ) -> NavUpdate {
        let dx = waypoint.x - pose.x;
        let dy = waypoint.y - pose.y;
        let distance = (dx * dx + dy * dy).sqrt();

        // Target angle to waypoint
        let target_angle = dy.atan2(dx);
        let angle_error = normalize_angle(target_angle - pose.theta);

        // Apply movement direction (forward vs backward)
        let direction_sign = target.movement_direction.velocity_sign();

        // For backward movement, we want to face away from the target
        let adjusted_angle_error = if target.movement_direction == MovementDirection::Backward {
            // When moving backward, the "front" of the robot should point away from target
            normalize_angle(angle_error + std::f32::consts::PI)
        } else {
            angle_error
        };

        // Rotation-first strategy
        if adjusted_angle_error.abs() > self.config.heading_threshold {
            // Need to rotate first
            let angular_vel = adjusted_angle_error.signum() * self.config.angular_vel;
            log::debug!(
                "NAV: Rotating to align - pose=({:.2},{:.2},{:.1}°) waypoint=({:.2},{:.2}) dist={:.2}m angle_err={:.1}° threshold={:.1}° -> angular={:.2}",
                pose.x,
                pose.y,
                pose.theta.to_degrees(),
                waypoint.x,
                waypoint.y,
                distance,
                adjusted_angle_error.to_degrees(),
                self.config.heading_threshold.to_degrees(),
                angular_vel
            );
            return NavUpdate {
                linear_vel: 0.0,
                angular_vel: angular_vel
                    .clamp(-self.config.max_angular_vel, self.config.max_angular_vel),
                state_changed: false,
            };
        }

        // Move toward waypoint with proportional angular correction
        let linear_vel = (distance * 0.5).min(self.config.linear_vel) * direction_sign;
        let angular_vel =
            (adjusted_angle_error * 0.5).clamp(-self.config.angular_vel, self.config.angular_vel);

        log::debug!(
            "NAV: Moving forward - pose=({:.2},{:.2},{:.1}°) waypoint=({:.2},{:.2}) dist={:.2}m -> linear={:.2} angular={:.2}",
            pose.x,
            pose.y,
            pose.theta.to_degrees(),
            waypoint.x,
            waypoint.y,
            distance,
            linear_vel,
            angular_vel
        );

        NavUpdate {
            linear_vel: linear_vel.clamp(-self.config.max_linear_vel, self.config.max_linear_vel),
            angular_vel: angular_vel
                .clamp(-self.config.max_angular_vel, self.config.max_angular_vel),
            state_changed: false,
        }
    }

    /// Reset navigator state (call when starting new navigation).
    pub fn reset(&mut self) {
        self.last_position = None;
        self.last_progress_time = Instant::now();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;
    use crate::navigation::{NavTargetSource, NavTargetType};

    fn make_test_target(x: f32, y: f32, heading: Option<f32>) -> NavTarget {
        NavTarget::new(
            Point2D::new(x, y),
            heading,
            NavTargetType::Waypoint,
            NavTargetSource::UserClick,
            format!("Test ({}, {})", x, y),
        )
    }

    #[test]
    fn test_navigator_creation() {
        let config = NavigatorConfig::default();
        let navigator = Navigator::new(config);

        assert!(navigator.config.waypoint_reached_threshold > 0.0);
    }

    #[test]
    fn test_nav_update_stop() {
        let update = NavUpdate::stop();

        assert_eq!(update.linear_vel, 0.0);
        assert_eq!(update.angular_vel, 0.0);
        assert!(!update.state_changed);
    }

    #[test]
    fn test_nav_update_with_state_change() {
        let update = NavUpdate::stop().with_state_change();

        assert!(update.state_changed);
    }

    #[test]
    fn test_navigator_idle_state() {
        let mut navigator = Navigator::new(NavigatorConfig::default());
        let mut nav_state = NavigationState::new();
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        let update = navigator.update(&mut nav_state, &pose, None, false);

        assert_eq!(update.linear_vel, 0.0);
        assert_eq!(update.angular_vel, 0.0);
        assert!(!update.state_changed);
    }

    #[test]
    fn test_navigator_planning_no_map() {
        let mut navigator = Navigator::new(NavigatorConfig::default());
        let mut nav_state = NavigationState::new();
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        // Add a target
        nav_state.push_target(make_test_target(1.0, 1.0, None));
        assert_eq!(nav_state.nav_state, NavState::Planning);

        // Update without map
        let update = navigator.update(&mut nav_state, &pose, None, false);

        assert!(update.state_changed);
        assert_eq!(nav_state.nav_state, NavState::Failed);
    }

    #[test]
    fn test_navigator_hazard_triggers_escape() {
        let mut navigator = Navigator::new(NavigatorConfig::default());
        let mut nav_state = NavigationState::new();
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        // Put in navigating state manually
        nav_state.push_target(make_test_target(1.0, 1.0, None));
        nav_state.nav_state = NavState::Navigating;

        // Hazard detected - should trigger escape (backing up)
        let update = navigator.update(&mut nav_state, &pose, None, true);

        // Robot should back up (negative velocity) and enter Escaping state
        assert_eq!(update.linear_vel, navigator.config.escape_velocity);
        assert_eq!(update.angular_vel, 0.0);
        assert!(update.state_changed);
        assert_eq!(nav_state.nav_state, NavState::Escaping);
    }

    #[test]
    fn test_movement_direction_backward() {
        let navigator = Navigator::new(NavigatorConfig::default());

        // Create a backward-moving target
        let target = NavTarget::dock_target(1.0, 0.0, 0.0);
        assert_eq!(target.movement_direction, MovementDirection::Backward);

        // Robot at origin, facing positive X
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let waypoint = Waypoint::new(1.0, 0.0);

        // The velocity should be negative (backward)
        let update = navigator.compute_velocity_to_waypoint(&pose, &waypoint, &target);

        // Robot needs to rotate to face away from target first (for backward movement)
        // Since robot is facing target directly, it needs ~180 degree rotation
        // This should result in either rotation or backward movement
        assert!(update.angular_vel.abs() > 0.0 || update.linear_vel < 0.0);
    }

    #[test]
    fn test_compute_velocity_forward() {
        let navigator = Navigator::new(NavigatorConfig::default());

        // Target directly ahead
        let target = make_test_target(1.0, 0.0, None);
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let waypoint = Waypoint::new(1.0, 0.0);

        let update = navigator.compute_velocity_to_waypoint(&pose, &waypoint, &target);

        assert!(update.linear_vel > 0.0);
        assert!(update.angular_vel.abs() < 0.1); // Nearly straight
    }

    #[test]
    fn test_compute_velocity_rotation_first() {
        let navigator = Navigator::new(NavigatorConfig::default());

        // Target to the left (90 degrees)
        let target = make_test_target(0.0, 1.0, None);
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let waypoint = Waypoint::new(0.0, 1.0);

        let update = navigator.compute_velocity_to_waypoint(&pose, &waypoint, &target);

        // Should rotate in place
        assert!(update.linear_vel.abs() < 0.01);
        assert!(update.angular_vel > 0.0); // CCW rotation
    }
}
