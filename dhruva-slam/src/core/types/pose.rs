//! Pose and point types for 2D SLAM.
//!
//! Note: Some utility methods are defined for future use.

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
}

impl Default for Pose2D {
    fn default() -> Self {
        Self::identity()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::FRAC_PI_2;

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
}

// ============================================================================
// Phase 1: Enhanced Pose Composition Tests
// ============================================================================

#[cfg(test)]
mod pose_composition_tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::PI;

    #[test]
    fn test_pose_compose_multiple_rotations() {
        let rotate_45 = Pose2D::new(0.0, 0.0, PI / 4.0);
        let rotate_90 = Pose2D::new(0.0, 0.0, PI / 2.0);

        let combined = rotate_45.compose(&rotate_45);
        assert_relative_eq!(combined.theta, PI / 2.0, epsilon = 1e-6);

        let triple = rotate_45.compose(&rotate_45).compose(&rotate_90);
        assert_relative_eq!(triple.theta, PI, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_inverse_compose_identity_multiple() {
        let poses = vec![
            Pose2D::new(0.0, 0.0, 0.0),
            Pose2D::new(1.0, 2.0, 0.5),
            Pose2D::new(-5.0, 3.0, -PI + 0.01),
            Pose2D::new(0.0, 0.0, PI - 0.001),
        ];

        for p in poses {
            let result = p.compose(&p.inverse());
            assert_relative_eq!(result.x, 0.0, epsilon = 1e-5);
            assert_relative_eq!(result.y, 0.0, epsilon = 1e-5);
            assert!(
                result.theta.abs() < 1e-5 || (result.theta.abs() - 2.0 * PI).abs() < 1e-5,
                "Theta not near zero: {} for pose {:?}",
                result.theta,
                p
            );
        }
    }

    #[test]
    fn test_pose_angle_wraparound_at_pi() {
        let near_pi = Pose2D::new(0.0, 0.0, PI - 0.01);
        let small_rotation = Pose2D::new(0.0, 0.0, 0.05);

        let result = near_pi.compose(&small_rotation);

        assert!(
            result.theta.abs() <= PI + 1e-5,
            "Theta should be normalized: {}",
            result.theta
        );
    }

    #[test]
    fn test_pose_angle_wraparound_negative_pi() {
        let near_neg_pi = Pose2D::new(0.0, 0.0, -PI + 0.01);
        let small_rotation = Pose2D::new(0.0, 0.0, -0.05);

        let result = near_neg_pi.compose(&small_rotation);

        assert!(
            result.theta.abs() <= PI + 1e-5,
            "Theta should be normalized: {}",
            result.theta
        );
    }

    #[test]
    fn test_pose_near_pi_boundary_positive() {
        let pose = Pose2D::new(0.0, 0.0, PI - 0.01);
        let delta = Pose2D::new(0.0, 0.0, 0.05);
        let result = pose.compose(&delta);

        // Should wrap to negative side
        assert!(result.theta < 0.0 || result.theta > PI - 0.1);
        assert!(result.theta.abs() <= PI + 1e-5);
    }

    #[test]
    fn test_pose_near_pi_boundary_negative() {
        let pose = Pose2D::new(0.0, 0.0, -PI + 0.01);
        let delta = Pose2D::new(0.0, 0.0, -0.05);
        let result = pose.compose(&delta);

        // Should wrap to positive side
        assert!(result.theta > 0.0 || result.theta < -PI + 0.1);
        assert!(result.theta.abs() <= PI + 1e-5);
    }
}
