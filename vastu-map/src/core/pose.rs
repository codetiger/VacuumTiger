//! Robot pose representation.

use super::point::WorldPoint;
use serde::{Deserialize, Serialize};

/// Robot pose in world coordinates (x, y, theta)
///
/// Coordinate system: ROS REP-103
/// - X: Forward (positive ahead of robot)
/// - Y: Left (positive to robot's left)
/// - Theta: Rotation angle in radians, CCW positive from +X axis
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Pose2D {
    /// X position in meters
    pub x: f32,
    /// Y position in meters
    pub y: f32,
    /// Orientation in radians (CCW positive from +X)
    pub theta: f32,
}

impl Pose2D {
    /// Create a new pose
    #[inline]
    pub fn new(x: f32, y: f32, theta: f32) -> Self {
        Self { x, y, theta }
    }

    /// Create a pose from position only (theta = 0)
    #[inline]
    pub fn from_position(x: f32, y: f32) -> Self {
        Self { x, y, theta: 0.0 }
    }

    /// Create a pose from a WorldPoint (theta = 0)
    #[inline]
    pub fn from_point(point: WorldPoint) -> Self {
        Self {
            x: point.x,
            y: point.y,
            theta: 0.0,
        }
    }

    /// Get the position as a WorldPoint
    #[inline]
    pub fn position(&self) -> WorldPoint {
        WorldPoint::new(self.x, self.y)
    }

    /// Get the forward direction as a unit vector
    #[inline]
    pub fn forward(&self) -> WorldPoint {
        WorldPoint::new(self.theta.cos(), self.theta.sin())
    }

    /// Get the left direction as a unit vector
    #[inline]
    pub fn left(&self) -> WorldPoint {
        WorldPoint::new(-self.theta.sin(), self.theta.cos())
    }

    /// Transform a point from robot frame to world frame
    ///
    /// Robot frame: X forward, Y left
    /// World frame: absolute coordinates
    #[inline]
    pub fn transform_point(&self, robot_point: WorldPoint) -> WorldPoint {
        let cos_t = self.theta.cos();
        let sin_t = self.theta.sin();

        WorldPoint::new(
            self.x + robot_point.x * cos_t - robot_point.y * sin_t,
            self.y + robot_point.x * sin_t + robot_point.y * cos_t,
        )
    }

    /// Transform a point from world frame to robot frame
    #[inline]
    pub fn inverse_transform_point(&self, world_point: WorldPoint) -> WorldPoint {
        let cos_t = self.theta.cos();
        let sin_t = self.theta.sin();

        let dx = world_point.x - self.x;
        let dy = world_point.y - self.y;

        WorldPoint::new(dx * cos_t + dy * sin_t, -dx * sin_t + dy * cos_t)
    }

    /// Compose two poses: self * other
    /// Applies other's transform in self's frame
    #[inline]
    pub fn compose(&self, other: &Pose2D) -> Pose2D {
        let transformed = self.transform_point(other.position());
        Pose2D::new(
            transformed.x,
            transformed.y,
            normalize_angle(self.theta + other.theta),
        )
    }

    /// Inverse of this pose
    #[inline]
    pub fn inverse(&self) -> Pose2D {
        let cos_t = self.theta.cos();
        let sin_t = self.theta.sin();

        Pose2D::new(
            -self.x * cos_t - self.y * sin_t,
            self.x * sin_t - self.y * cos_t,
            -self.theta,
        )
    }

    /// Distance to another pose (position only)
    #[inline]
    pub fn distance(&self, other: &Pose2D) -> f32 {
        self.position().distance(&other.position())
    }

    /// Angular difference to another pose (in radians, normalized to [-pi, pi])
    #[inline]
    pub fn angle_diff(&self, other: &Pose2D) -> f32 {
        normalize_angle(other.theta - self.theta)
    }

    /// Normalize the theta to [-pi, pi]
    #[inline]
    pub fn normalize(&self) -> Pose2D {
        Pose2D::new(self.x, self.y, normalize_angle(self.theta))
    }
}

impl PartialEq for Pose2D {
    fn eq(&self, other: &Self) -> bool {
        (self.x - other.x).abs() < 1e-6
            && (self.y - other.y).abs() < 1e-6
            && (normalize_angle(self.theta - other.theta)).abs() < 1e-6
    }
}

/// Normalize an angle to [-pi, pi]
#[inline]
pub fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle;
    while a > std::f32::consts::PI {
        a -= 2.0 * std::f32::consts::PI;
    }
    while a < -std::f32::consts::PI {
        a += 2.0 * std::f32::consts::PI;
    }
    a
}

/// Interpolate between two poses
#[inline]
pub fn interpolate_pose(a: &Pose2D, b: &Pose2D, t: f32) -> Pose2D {
    let t = t.clamp(0.0, 1.0);

    // Linear interpolation for position
    let x = a.x + t * (b.x - a.x);
    let y = a.y + t * (b.y - a.y);

    // Angular interpolation (shortest path)
    let diff = normalize_angle(b.theta - a.theta);
    let theta = normalize_angle(a.theta + t * diff);

    Pose2D::new(x, y, theta)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::{FRAC_PI_2, PI};

    #[test]
    fn test_pose_transform() {
        // Robot at (1, 0) facing +Y (90 degrees)
        let pose = Pose2D::new(1.0, 0.0, FRAC_PI_2);

        // Point 1m ahead in robot frame should be at (1, 1) in world
        let robot_point = WorldPoint::new(1.0, 0.0);
        let world_point = pose.transform_point(robot_point);

        assert!((world_point.x - 1.0).abs() < 1e-5);
        assert!((world_point.y - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_pose_inverse_transform() {
        let pose = Pose2D::new(1.0, 2.0, FRAC_PI_2);
        let world_point = WorldPoint::new(1.0, 3.0);

        let robot_point = pose.inverse_transform_point(world_point);
        let back_to_world = pose.transform_point(robot_point);

        assert!((back_to_world.x - world_point.x).abs() < 1e-5);
        assert!((back_to_world.y - world_point.y).abs() < 1e-5);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-6);
        assert!((normalize_angle(2.0 * PI) - 0.0).abs() < 1e-6);
        assert!((normalize_angle(-2.0 * PI) - 0.0).abs() < 1e-6);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-5);
    }

    #[test]
    fn test_pose_compose() {
        let a = Pose2D::new(1.0, 0.0, FRAC_PI_2);
        let b = Pose2D::new(1.0, 0.0, 0.0);

        let c = a.compose(&b);

        // Moving 1m forward from (1,0) facing +Y should put us at (1,1)
        assert!((c.x - 1.0).abs() < 1e-5);
        assert!((c.y - 1.0).abs() < 1e-5);
        assert!((c.theta - FRAC_PI_2).abs() < 1e-5);
    }

    #[test]
    fn test_interpolate_pose() {
        let a = Pose2D::new(0.0, 0.0, 0.0);
        let b = Pose2D::new(2.0, 2.0, PI);

        let mid = interpolate_pose(&a, &b, 0.5);
        assert!((mid.x - 1.0).abs() < 1e-5);
        assert!((mid.y - 1.0).abs() < 1e-5);
        assert!((mid.theta - FRAC_PI_2).abs() < 1e-5);
    }
}
