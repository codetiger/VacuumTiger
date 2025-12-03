//! Pose and point types for 2D SLAM.

use serde::{Deserialize, Serialize};

/// A 2D point in meters.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Point2D {
    /// X coordinate in meters
    pub x: f32,
    /// Y coordinate in meters
    pub y: f32,
}

impl Point2D {
    /// Create a new point.
    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Squared distance to another point (avoids sqrt).
    #[inline]
    pub fn distance_squared(&self, other: &Point2D) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }

    /// Distance to another point.
    #[inline]
    pub fn distance(&self, other: &Point2D) -> f32 {
        self.distance_squared(other).sqrt()
    }
}

impl Default for Point2D {
    fn default() -> Self {
        Self { x: 0.0, y: 0.0 }
    }
}

/// Robot pose in 2D space.
///
/// Represents position (x, y) in meters and heading (theta) in radians.
/// Theta is normalized to [-π, π].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose2D {
    /// X position in meters
    pub x: f32,
    /// Y position in meters
    pub y: f32,
    /// Heading in radians, normalized to [-π, π]
    pub theta: f32,
}

impl Pose2D {
    /// Create a new pose with theta normalized to [-π, π].
    #[inline]
    pub fn new(x: f32, y: f32, theta: f32) -> Self {
        Self {
            x,
            y,
            theta: crate::core::math::normalize_angle(theta),
        }
    }

    /// Identity pose at origin with zero heading.
    #[inline]
    pub fn identity() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }

    /// Compose two poses: self ⊕ other
    ///
    /// Applies `other` transform relative to `self` frame.
    /// ```text
    /// C = A ⊕ B:
    ///   C.x = A.x + B.x * cos(A.θ) - B.y * sin(A.θ)
    ///   C.y = A.y + B.x * sin(A.θ) + B.y * cos(A.θ)
    ///   C.θ = normalize(A.θ + B.θ)
    /// ```
    #[inline]
    pub fn compose(&self, other: &Pose2D) -> Pose2D {
        let (sin_t, cos_t) = self.theta.sin_cos();
        Pose2D::new(
            self.x + other.x * cos_t - other.y * sin_t,
            self.y + other.x * sin_t + other.y * cos_t,
            self.theta + other.theta,
        )
    }

    /// Inverse of this pose.
    ///
    /// Returns the transform that undoes this pose.
    /// ```text
    /// A⁻¹:
    ///   x = -A.x * cos(A.θ) - A.y * sin(A.θ)
    ///   y =  A.x * sin(A.θ) - A.y * cos(A.θ)
    ///   θ = -A.θ
    /// ```
    #[inline]
    pub fn inverse(&self) -> Pose2D {
        let (sin_t, cos_t) = self.theta.sin_cos();
        Pose2D::new(
            -self.x * cos_t - self.y * sin_t,
            self.x * sin_t - self.y * cos_t,
            -self.theta,
        )
    }

    /// Transform a point from local frame to global frame.
    #[inline]
    pub fn transform_point(&self, point: &Point2D) -> Point2D {
        let (sin_t, cos_t) = self.theta.sin_cos();
        Point2D::new(
            self.x + point.x * cos_t - point.y * sin_t,
            self.y + point.x * sin_t + point.y * cos_t,
        )
    }

    /// Transform a point from global frame to local frame.
    #[inline]
    pub fn inverse_transform_point(&self, point: &Point2D) -> Point2D {
        let (sin_t, cos_t) = self.theta.sin_cos();
        let dx = point.x - self.x;
        let dy = point.y - self.y;
        Point2D::new(dx * cos_t + dy * sin_t, -dx * sin_t + dy * cos_t)
    }

    /// Interpolate between two timestamped poses.
    ///
    /// Returns the interpolated pose at `target_time_us`.
    /// Returns `None` if `target_time_us` is outside the range [start, end].
    ///
    /// Uses linear interpolation for x, y and shortest-path angular
    /// interpolation for theta.
    pub fn interpolate(
        start: &super::Timestamped<Pose2D>,
        end: &super::Timestamped<Pose2D>,
        target_time_us: u64,
    ) -> Option<Pose2D> {
        // Check bounds
        if target_time_us < start.timestamp_us || target_time_us > end.timestamp_us {
            return None;
        }

        // Handle same timestamp
        if start.timestamp_us == end.timestamp_us {
            return Some(start.data);
        }

        // Compute interpolation factor
        let t = (target_time_us - start.timestamp_us) as f32
            / (end.timestamp_us - start.timestamp_us) as f32;

        // Linear interpolation for x, y
        let x = start.data.x + t * (end.data.x - start.data.x);
        let y = start.data.y + t * (end.data.y - start.data.y);

        // Angular interpolation (shortest path)
        let theta = crate::core::math::angle_lerp(start.data.theta, end.data.theta, t);

        Some(Pose2D { x, y, theta })
    }
}

impl Default for Pose2D {
    fn default() -> Self {
        Self::identity()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Timestamped;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, PI};

    #[test]
    fn test_point2d_distance() {
        let a = Point2D::new(0.0, 0.0);
        let b = Point2D::new(3.0, 4.0);
        assert_relative_eq!(a.distance(&b), 5.0);
        assert_relative_eq!(a.distance_squared(&b), 25.0);
    }

    #[test]
    fn test_pose_compose_identity() {
        let p = Pose2D::new(1.0, 2.0, 0.5);
        let identity = Pose2D::identity();
        let result = p.compose(&identity);
        assert_relative_eq!(result.x, p.x);
        assert_relative_eq!(result.y, p.y);
        assert_relative_eq!(result.theta, p.theta);
    }

    #[test]
    fn test_pose_inverse_roundtrip() {
        let p = Pose2D::new(1.0, 2.0, 0.5);
        let inv = p.inverse();
        let result = p.compose(&inv);
        assert_relative_eq!(result.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(result.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(result.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_transform_point() {
        let pose = Pose2D::new(1.0, 0.0, FRAC_PI_2);
        let point = Point2D::new(1.0, 0.0);
        let result = pose.transform_point(&point);
        assert_relative_eq!(result.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(result.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_inverse_transform_point() {
        let pose = Pose2D::new(1.0, 0.0, FRAC_PI_2);
        let global_point = Point2D::new(1.0, 1.0);
        let local = pose.inverse_transform_point(&global_point);
        assert_relative_eq!(local.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(local.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_composition_order() {
        let move_forward = Pose2D::new(1.0, 0.0, 0.0);
        let rotate = Pose2D::new(0.0, 0.0, FRAC_PI_2);
        let result = move_forward.compose(&rotate);
        assert_relative_eq!(result.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(result.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(result.theta, FRAC_PI_2, epsilon = 1e-6);

        let result2 = rotate.compose(&move_forward);
        assert_relative_eq!(result2.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(result2.y, 1.0, epsilon = 1e-6);
        assert_relative_eq!(result2.theta, FRAC_PI_2, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_interpolation() {
        let start = Timestamped::new(Pose2D::new(0.0, 0.0, 0.0), 0);
        let end = Timestamped::new(Pose2D::new(2.0, 4.0, PI / 2.0), 1000);

        let p = Pose2D::interpolate(&start, &end, 0).unwrap();
        assert_relative_eq!(p.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(p.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(p.theta, 0.0, epsilon = 1e-6);

        let p = Pose2D::interpolate(&start, &end, 1000).unwrap();
        assert_relative_eq!(p.x, 2.0, epsilon = 1e-6);
        assert_relative_eq!(p.y, 4.0, epsilon = 1e-6);
        assert_relative_eq!(p.theta, PI / 2.0, epsilon = 1e-6);

        let p = Pose2D::interpolate(&start, &end, 500).unwrap();
        assert_relative_eq!(p.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(p.y, 2.0, epsilon = 1e-6);
        assert_relative_eq!(p.theta, PI / 4.0, epsilon = 1e-6);

        assert!(Pose2D::interpolate(&start, &end, 1001).is_none());
    }

    #[test]
    fn test_pose_interpolation_angle_wrap() {
        let start = Timestamped::new(Pose2D::new(0.0, 0.0, PI - 0.1), 0);
        let end = Timestamped::new(Pose2D::new(0.0, 0.0, -PI + 0.1), 1000);

        let p = Pose2D::interpolate(&start, &end, 500).unwrap();
        assert!(p.theta.abs() > PI - 0.2);
    }

    #[test]
    fn test_point_distance_to_self() {
        let p = Point2D::new(3.0, 4.0);
        assert_eq!(p.distance(&p), 0.0);
        assert_eq!(p.distance_squared(&p), 0.0);
    }

    #[test]
    fn test_pose_compose_with_zero_rotation() {
        let pose = Pose2D::new(1.0, 2.0, 0.0);
        let delta = Pose2D::new(3.0, 0.0, 0.0);

        let result = pose.compose(&delta);
        assert_relative_eq!(result.x, 4.0, epsilon = 1e-6);
        assert_relative_eq!(result.y, 2.0, epsilon = 1e-6);
        assert_relative_eq!(result.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_inverse_of_identity() {
        let identity = Pose2D::identity();
        let inv = identity.inverse();

        assert_relative_eq!(inv.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(inv.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(inv.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_interpolate_identical_timestamps() {
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let start = Timestamped::new(pose, 1000);
        let end = Timestamped::new(Pose2D::new(5.0, 6.0, 1.0), 1000);

        let result = Pose2D::interpolate(&start, &end, 1000);
        assert!(result.is_some());
        let interp = result.unwrap();
        assert_relative_eq!(interp.x, pose.x, epsilon = 1e-6);
        assert_relative_eq!(interp.y, pose.y, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_interpolate_out_of_bounds() {
        let start = Timestamped::new(Pose2D::new(0.0, 0.0, 0.0), 1000);
        let end = Timestamped::new(Pose2D::new(10.0, 10.0, 1.0), 2000);

        assert!(Pose2D::interpolate(&start, &end, 500).is_none());
        assert!(Pose2D::interpolate(&start, &end, 2500).is_none());
    }
}
