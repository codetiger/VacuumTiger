//! Path follower using pure pursuit algorithm.
//!
//! Follows planned paths by computing velocity commands that steer
//! the robot toward a lookahead point on the path.

use vastu_slam::{Pose2D, WorldPoint};

use crate::planning::{PathSegment, SmoothedPath};

/// Configuration for path following.
#[derive(Clone, Debug)]
pub struct FollowerConfig {
    /// Distance tolerance for reaching waypoints (meters)
    pub waypoint_tolerance: f32,
    /// Distance tolerance for reaching final goal (meters)
    pub goal_tolerance: f32,
    /// Maximum linear velocity (m/s)
    pub max_linear_vel: f32,
    /// Maximum angular velocity (rad/s)
    pub max_angular_vel: f32,
    /// Proportional gain for angular control
    pub kp_angular: f32,
}

impl Default for FollowerConfig {
    fn default() -> Self {
        Self {
            waypoint_tolerance: 0.10,
            goal_tolerance: 0.15,
            max_linear_vel: 0.2,
            max_angular_vel: 0.5,
            kp_angular: 2.0,
        }
    }
}

/// State of path following.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FollowState {
    /// Following the path
    Following,
    /// Performing a point turn
    Turning,
    /// Reached the goal
    Complete,
    /// Path is empty or invalid
    NoPath,
}

/// Path follower using pure pursuit.
pub struct PathFollower {
    config: FollowerConfig,
    /// Current path to follow
    path: Option<SmoothedPath>,
    /// Current segment index
    current_segment: usize,
    /// Current state
    state: FollowState,
}

impl PathFollower {
    /// Create a new path follower with configuration.
    pub fn new(config: FollowerConfig) -> Self {
        Self {
            config,
            path: None,
            current_segment: 0,
            state: FollowState::NoPath,
        }
    }

    /// Set a new path to follow.
    pub fn set_path(&mut self, path: SmoothedPath) {
        if path.segments.is_empty() {
            tracing::warn!("set_path: received empty path (0 segments)");
            self.path = None;
            self.current_segment = 0;
            self.state = FollowState::NoPath;
        } else {
            tracing::info!(
                "set_path: {} segments, {:.2}m total",
                path.segments.len(),
                path.total_length
            );
            self.path = Some(path);
            self.current_segment = 0;
            self.state = FollowState::Following;
        }
    }

    /// Clear the current path.
    pub fn clear_path(&mut self) {
        self.path = None;
        self.current_segment = 0;
        self.state = FollowState::NoPath;
    }

    /// Check if path following is complete.
    pub fn is_complete(&self) -> bool {
        self.state == FollowState::Complete
    }

    /// Check if there's no path to follow.
    pub fn has_path(&self) -> bool {
        self.path.is_some() && !self.path.as_ref().unwrap().segments.is_empty()
    }

    /// Get the final goal point.
    pub fn goal(&self) -> Option<WorldPoint> {
        self.path
            .as_ref()
            .and_then(|p| p.segments.last().map(|s| s.end_point()))
    }

    /// Compute velocity command to follow the path.
    ///
    /// Returns (linear_velocity, angular_velocity).
    pub fn compute_velocity(&mut self, current_pose: Pose2D) -> (f32, f32) {
        let path = match &self.path {
            Some(p) => p,
            None => {
                self.state = FollowState::NoPath;
                tracing::warn!("compute_velocity: No path set on follower");
                return (0.0, 0.0);
            }
        };

        if self.current_segment >= path.segments.len() {
            self.state = FollowState::Complete;
            tracing::debug!("Path complete: all {} segments done", path.segments.len());
            return (0.0, 0.0);
        }

        let segment = &path.segments[self.current_segment];
        tracing::debug!(
            "Following segment {}/{}: {:?}",
            self.current_segment + 1,
            path.segments.len(),
            match segment {
                PathSegment::Line { start, end } => format!(
                    "Line ({:.2},{:.2})->({:.2},{:.2})",
                    start.x, start.y, end.x, end.y
                ),
                PathSegment::PointTurn { to_angle, .. } =>
                    format!("PointTurn to {:.1}°", to_angle.to_degrees()),
            }
        );
        let current_pos = WorldPoint::new(current_pose.x, current_pose.y);

        match segment {
            PathSegment::Line { start, end } => {
                self.follow_line(current_pose, current_pos, *start, *end)
            }
            PathSegment::PointTurn {
                position, to_angle, ..
            } => self.execute_point_turn(current_pose, *position, *to_angle),
        }
    }

    /// Follow a line segment - simplified direct approach.
    fn follow_line(
        &mut self,
        current_pose: Pose2D,
        current_pos: WorldPoint,
        _start: WorldPoint,
        end: WorldPoint,
    ) -> (f32, f32) {
        let distance_to_end = current_pos.distance(&end);

        // Check if we've reached the end of this segment
        if distance_to_end < self.config.waypoint_tolerance {
            tracing::debug!("Segment complete (dist {:.3}m)", distance_to_end);
            self.advance_segment();
            return self.compute_velocity(current_pose);
        }

        // Simple approach: drive directly toward the segment end point
        // This is more robust than pure pursuit for our use case
        self.drive_to_point(current_pose, end, distance_to_end)
    }

    /// Execute a point turn (rotate in place).
    fn execute_point_turn(
        &mut self,
        current_pose: Pose2D,
        _position: WorldPoint,
        target_angle: f32,
    ) -> (f32, f32) {
        // Compute angle error
        let angle_error = Self::normalize_angle(target_angle - current_pose.theta);

        // Check if turn is complete (10° tolerance)
        if angle_error.abs() < 0.17 {
            self.advance_segment();
            return self.compute_velocity(current_pose);
        }

        // Pure rotation with gentler control to prevent overshoot
        self.state = FollowState::Turning;
        let turn_max_angular = self.config.max_angular_vel * 0.5; // 50% of max for point turns
        let angular_gain = 0.6; // Gentle gain for point turns

        let angular = (angular_gain * angle_error).clamp(-turn_max_angular, turn_max_angular);

        tracing::debug!(
            "point_turn: target={:.1}°, current={:.1}°, error={:.1}°, angular={:.3}",
            target_angle.to_degrees(),
            current_pose.theta.to_degrees(),
            angle_error.to_degrees(),
            angular
        );

        (0.0, angular)
    }

    /// Simple drive-to-point controller.
    /// Uses proportional control with angle-dependent speed reduction.
    fn drive_to_point(
        &mut self,
        current_pose: Pose2D,
        target: WorldPoint,
        distance: f32,
    ) -> (f32, f32) {
        self.state = FollowState::Following;

        // Compute angle to target
        let dx = target.x - current_pose.x;
        let dy = target.y - current_pose.y;
        let target_angle = dy.atan2(dx);
        let angle_error = Self::normalize_angle(target_angle - current_pose.theta);
        let angle_error_deg = angle_error.to_degrees();
        let angle_error_abs = angle_error.abs();

        // Proportional control with angle-dependent speed scaling
        // Key insight: ALWAYS maintain some forward motion to prevent spinning loops

        // Angular velocity: proportional to angle error, clamped
        let kp_angular = self.config.kp_angular;
        let angular = (kp_angular * angle_error)
            .clamp(-self.config.max_angular_vel, self.config.max_angular_vel);

        // Linear velocity: scale down based on angle error
        // - At 0° error: full speed
        // - At 90° error: 20% speed (still moving forward!)
        // - At 180° error: 10% speed (prevents spinning in place)
        let angle_factor = if angle_error_abs < 0.17 {
            // < 10°: full speed
            1.0
        } else if angle_error_abs < 1.57 {
            // 10-90°: linear interpolation from 1.0 to 0.2
            1.0 - 0.8 * (angle_error_abs - 0.17) / (1.57 - 0.17)
        } else {
            // > 90°: minimum speed (10-20% to prevent pure spinning)
            0.1 + 0.1 * (std::f32::consts::PI - angle_error_abs) / (std::f32::consts::PI - 1.57)
        };

        // Distance-based speed: slow down as we approach target
        let distance_speed = (distance * 0.8).min(self.config.max_linear_vel);

        // Minimum forward speed when far from target to prevent getting stuck
        let min_forward_speed = if distance > 0.15 { 0.03 } else { 0.0 };

        let linear = (distance_speed * angle_factor).max(min_forward_speed);

        tracing::debug!(
            "drive_to_point: dist={:.2}m, angle_err={:.1}°, angle_factor={:.2}, vel=({:.3},{:.3})",
            distance,
            angle_error_deg,
            angle_factor,
            linear,
            angular
        );

        (linear, angular)
    }

    /// Advance to the next segment.
    fn advance_segment(&mut self) {
        self.current_segment += 1;

        if let Some(path) = &self.path {
            if self.current_segment >= path.segments.len() {
                self.state = FollowState::Complete;
            } else {
                self.state = FollowState::Following;
            }
        }
    }

    /// Normalize angle to [-pi, pi].
    #[inline]
    fn normalize_angle(angle: f32) -> f32 {
        let mut a = angle;
        while a > std::f32::consts::PI {
            a -= 2.0 * std::f32::consts::PI;
        }
        while a < -std::f32::consts::PI {
            a += 2.0 * std::f32::consts::PI;
        }
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::PathSegment;

    fn create_test_path() -> SmoothedPath {
        SmoothedPath {
            segments: vec![
                PathSegment::Line {
                    start: WorldPoint::new(0.0, 0.0),
                    end: WorldPoint::new(1.0, 0.0),
                },
                PathSegment::Line {
                    start: WorldPoint::new(1.0, 0.0),
                    end: WorldPoint::new(1.0, 1.0),
                },
            ],
            total_length: 2.0,
            estimated_time: 10.0,
        }
    }

    #[test]
    fn test_follower_creation() {
        let follower = PathFollower::new(FollowerConfig::default());
        assert!(!follower.has_path());
    }

    #[test]
    fn test_set_path() {
        let mut follower = PathFollower::new(FollowerConfig::default());
        let path = create_test_path();

        follower.set_path(path);
        assert!(follower.has_path());
    }

    #[test]
    fn test_follow_straight_line() {
        let mut follower = PathFollower::new(FollowerConfig::default());
        let path = SmoothedPath {
            segments: vec![PathSegment::Line {
                start: WorldPoint::new(0.0, 0.0),
                end: WorldPoint::new(1.0, 0.0),
            }],
            total_length: 1.0,
            estimated_time: 5.0,
        };

        follower.set_path(path);

        // Robot at origin, facing forward
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let (linear, angular) = follower.compute_velocity(pose);

        // Should move forward with minimal turning
        assert!(linear > 0.0);
        assert!(angular.abs() < 0.1);
    }

    #[test]
    fn test_complete_detection() {
        let mut follower = PathFollower::new(FollowerConfig::default());
        let path = SmoothedPath {
            segments: vec![PathSegment::Line {
                start: WorldPoint::new(0.0, 0.0),
                end: WorldPoint::new(0.1, 0.0),
            }],
            total_length: 0.1,
            estimated_time: 0.5,
        };

        follower.set_path(path);

        // Robot near the end
        let pose = Pose2D::new(0.09, 0.0, 0.0);
        let (_linear, _angular) = follower.compute_velocity(pose);

        assert!(follower.is_complete());
    }
}
