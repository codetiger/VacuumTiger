//! Pure pursuit path following for exploration.
//!
//! Implements a simple pure pursuit controller that generates velocity
//! commands to follow a path of waypoints.

use crate::Path;
use crate::core::{Point2D, Pose2D};

use super::config::ExplorationConfig;

/// Velocity command for the robot.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VelocityCommand {
    /// Linear velocity in m/s (positive = forward).
    pub linear: f32,
    /// Angular velocity in rad/s (positive = counter-clockwise).
    pub angular: f32,
}

impl VelocityCommand {
    /// Create a new velocity command.
    pub fn new(linear: f32, angular: f32) -> Self {
        Self { linear, angular }
    }

    /// Create a zero velocity command (stop).
    pub fn stop() -> Self {
        Self::default()
    }

    /// Create a command for backing up.
    pub fn backup(speed: f32) -> Self {
        Self {
            linear: -speed.abs(),
            angular: 0.0,
        }
    }

    /// Check if this is a stop command.
    pub fn is_stop(&self) -> bool {
        self.linear.abs() < 0.001 && self.angular.abs() < 0.001
    }
}

/// Result of a path following step.
#[derive(Clone, Debug)]
pub struct FollowResult {
    /// Velocity command to execute.
    pub velocity: VelocityCommand,
    /// Updated waypoint index.
    pub waypoint_idx: usize,
    /// Whether the path is complete (reached final waypoint).
    pub path_complete: bool,
    /// Distance to current target waypoint.
    pub distance_to_waypoint: f32,
}

/// Path follower for waypoint-based navigation.
///
/// Generates velocity commands to follow a path of waypoints:
///
/// 1. Transform target waypoint to robot frame
/// 2. Compute angle to target
/// 3. Generate proportional angular velocity for steering
/// 4. Reduce linear velocity when turning sharply
///
/// The follower visits each waypoint in sequence to ensure the robot
/// follows the planned CBVG path through the medial axis of free space.
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::exploration::{PathFollower, ExplorationConfig};
/// use vastu_map::{Path, Pose2D, Point2D};
///
/// let config = ExplorationConfig::default();
/// let follower = PathFollower::new(&config);
///
/// let path = Path {
///     points: vec![Point2D::new(1.0, 0.0), Point2D::new(2.0, 0.0)],
///     length: 1.0,
/// };
///
/// let pose = Pose2D::new(0.0, 0.0, 0.0);
/// let result = follower.follow(pose, &path, 0, 0.1);
/// ```
#[derive(Clone, Debug)]
pub struct PathFollower {
    /// Maximum linear velocity.
    max_linear: f32,
    /// Maximum angular velocity.
    max_angular: f32,
    /// Proportional gain for angular control.
    angular_gain: f32,
    /// Proportional gain for linear control based on distance.
    linear_gain: f32,
}

impl PathFollower {
    /// Create a new path follower from exploration config.
    pub fn new(config: &ExplorationConfig) -> Self {
        Self {
            max_linear: config.max_linear_speed,
            max_angular: config.max_angular_speed,
            angular_gain: 2.0, // Responsive steering
            linear_gain: 0.5,  // Smooth approach
        }
    }

    /// Create a path follower with custom parameters.
    pub fn with_params(max_linear: f32, max_angular: f32) -> Self {
        Self {
            max_linear,
            max_angular,
            angular_gain: 2.0,
            linear_gain: 0.5,
        }
    }

    /// Compute velocity command to follow path.
    ///
    /// # Arguments
    /// * `current_pose` - Current robot pose in world frame
    /// * `path` - Path to follow
    /// * `waypoint_idx` - Current waypoint index
    /// * `tolerance` - Distance to consider waypoint reached
    ///
    /// # Returns
    /// Follow result with velocity command and updated waypoint index.
    pub fn follow(
        &self,
        current_pose: Pose2D,
        path: &Path,
        waypoint_idx: usize,
        tolerance: f32,
    ) -> FollowResult {
        // Empty path or past end
        if path.points.is_empty() || waypoint_idx >= path.points.len() {
            return FollowResult {
                velocity: VelocityCommand::stop(),
                waypoint_idx,
                path_complete: true,
                distance_to_waypoint: 0.0,
            };
        }

        let robot_pos = current_pose.position();
        let mut current_idx = waypoint_idx;

        // Find the next waypoint to pursue (skip waypoints we've passed)
        while current_idx < path.points.len() {
            let waypoint = path.points[current_idx];
            let distance = robot_pos.distance(waypoint);

            if distance >= tolerance {
                break;
            }

            current_idx += 1;
        }

        // Check if path is complete
        if current_idx >= path.points.len() {
            return FollowResult {
                velocity: VelocityCommand::stop(),
                waypoint_idx: current_idx,
                path_complete: true,
                distance_to_waypoint: 0.0,
            };
        }

        // Target current waypoint directly (no lookahead skipping)
        // This ensures the robot follows each CBVG node in sequence
        let target = path.points[current_idx];

        // Transform target to robot frame
        let local_target = current_pose.inverse_transform_point(target);

        // Compute angle to target (in robot frame, forward is +X)
        let angle_to_target = local_target.y.atan2(local_target.x);
        let distance_to_waypoint = local_target.length();

        // Compute velocity command
        let velocity = self.compute_velocity(angle_to_target, distance_to_waypoint);

        FollowResult {
            velocity,
            waypoint_idx: current_idx,
            path_complete: false,
            distance_to_waypoint,
        }
    }

    /// Compute velocity command from angle and distance to target.
    fn compute_velocity(&self, angle_to_target: f32, distance: f32) -> VelocityCommand {
        // Angular velocity: proportional to angle error
        let angular =
            (angle_to_target * self.angular_gain).clamp(-self.max_angular, self.max_angular);

        // Speed reduction when turning sharply
        // At 90 degrees, reduce speed to 20% of max
        let turn_factor = 1.0 - (angle_to_target.abs() / std::f32::consts::PI).min(0.8);

        // Linear velocity: proportional to distance, reduced when turning
        let linear = (distance * self.linear_gain).min(self.max_linear) * turn_factor;

        // Ensure minimum forward velocity when not at target
        let linear = if distance > 0.05 && linear < 0.05 {
            0.05 * turn_factor
        } else {
            linear
        };

        VelocityCommand::new(linear, angular)
    }

    /// Generate a command to rotate in place towards a target.
    pub fn rotate_to_target(&self, current_pose: Pose2D, target: Point2D) -> VelocityCommand {
        let local_target = current_pose.inverse_transform_point(target);
        let angle_to_target = local_target.y.atan2(local_target.x);

        // Pure rotation
        let angular =
            (angle_to_target * self.angular_gain).clamp(-self.max_angular, self.max_angular);

        VelocityCommand::new(0.0, angular)
    }

    /// Check if the robot is approximately facing the target.
    pub fn is_facing_target(&self, current_pose: Pose2D, target: Point2D, tolerance: f32) -> bool {
        let local_target = current_pose.inverse_transform_point(target);
        let angle_to_target = local_target.y.atan2(local_target.x);
        angle_to_target.abs() < tolerance
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_4;

    fn make_path(points: &[(f32, f32)]) -> Path {
        let pts: Vec<Point2D> = points.iter().map(|(x, y)| Point2D::new(*x, *y)).collect();
        let mut length = 0.0;
        for i in 1..pts.len() {
            length += pts[i - 1].distance(pts[i]);
        }
        Path {
            points: pts,
            length,
        }
    }

    #[test]
    fn test_velocity_command() {
        let cmd = VelocityCommand::new(0.3, 0.5);
        assert_eq!(cmd.linear, 0.3);
        assert_eq!(cmd.angular, 0.5);
        assert!(!cmd.is_stop());

        let stop = VelocityCommand::stop();
        assert!(stop.is_stop());

        let backup = VelocityCommand::backup(0.2);
        assert_eq!(backup.linear, -0.2);
    }

    #[test]
    fn test_follow_empty_path() {
        let config = ExplorationConfig::default();
        let follower = PathFollower::new(&config);

        let path = Path::new();
        let pose = Pose2D::identity();

        let result = follower.follow(pose, &path, 0, 0.1);
        assert!(result.path_complete);
        assert!(result.velocity.is_stop());
    }

    #[test]
    fn test_follow_straight_path() {
        let config = ExplorationConfig::default();
        let follower = PathFollower::new(&config);

        let path = make_path(&[(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]);
        let pose = Pose2D::new(0.0, 0.0, 0.0); // At origin, facing +X

        let result = follower.follow(pose, &path, 0, 0.1);

        // Should move forward with minimal turning
        assert!(!result.path_complete);
        assert!(result.velocity.linear > 0.0);
        assert!(result.velocity.angular.abs() < 0.1); // Nearly straight
    }

    #[test]
    fn test_follow_requires_turn() {
        let config = ExplorationConfig::default();
        let follower = PathFollower::new(&config);

        let path = make_path(&[(0.0, 1.0), (0.0, 2.0)]); // Path to the left
        let pose = Pose2D::new(0.0, 0.0, 0.0); // At origin, facing +X

        let result = follower.follow(pose, &path, 0, 0.1);

        // Should turn left (positive angular)
        assert!(!result.path_complete);
        assert!(result.velocity.angular > 0.0);
        // Linear should be reduced due to turning
        assert!(result.velocity.linear < config.max_linear_speed);
    }

    #[test]
    fn test_follow_path_complete() {
        let config = ExplorationConfig::default();
        let follower = PathFollower::new(&config);

        let path = make_path(&[(0.0, 0.0)]);
        let pose = Pose2D::new(0.0, 0.0, 0.0); // Already at target

        let result = follower.follow(pose, &path, 0, 0.1);

        assert!(result.path_complete);
        assert!(result.velocity.is_stop());
    }

    #[test]
    fn test_waypoint_advancement() {
        let config = ExplorationConfig::default();
        let follower = PathFollower::new(&config);

        let path = make_path(&[(0.5, 0.0), (1.0, 0.0), (1.5, 0.0)]);

        // Robot at (0.45, 0) - very close to first waypoint
        let pose = Pose2D::new(0.45, 0.0, 0.0);

        let result = follower.follow(pose, &path, 0, 0.1);

        // Should skip first waypoint and target second
        assert_eq!(result.waypoint_idx, 1);
    }

    #[test]
    fn test_rotate_to_target() {
        let config = ExplorationConfig::default();
        let follower = PathFollower::new(&config);

        let pose = Pose2D::new(0.0, 0.0, 0.0); // Facing +X
        let target = Point2D::new(0.0, 1.0); // Target is to the left

        let cmd = follower.rotate_to_target(pose, target);

        assert_eq!(cmd.linear, 0.0); // No forward motion
        assert!(cmd.angular > 0.0); // Turn left (CCW)
    }

    #[test]
    fn test_is_facing_target() {
        let config = ExplorationConfig::default();
        let follower = PathFollower::new(&config);

        let pose = Pose2D::new(0.0, 0.0, 0.0);

        // Target directly ahead
        assert!(follower.is_facing_target(pose, Point2D::new(1.0, 0.0), 0.1));

        // Target to the side
        assert!(!follower.is_facing_target(pose, Point2D::new(0.0, 1.0), 0.1));

        // Target at 45 degrees
        assert!(follower.is_facing_target(pose, Point2D::new(1.0, 0.1), FRAC_PI_4));
    }
}
