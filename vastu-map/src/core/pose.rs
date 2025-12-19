//! 2D pose type for robot position and orientation.
//!
//! Coordinate frame follows ROS REP-103:
//! - X-forward, Y-left, Z-up (right-handed)
//! - Counter-clockwise positive rotation

use super::math::normalize_angle;
use super::point::Point2D;

/// A 2D pose representing position and orientation.
///
/// Uses the ROS REP-103 coordinate convention:
/// - Position: (x, y) in meters
/// - Theta: heading angle in radians, counter-clockwise from X-axis
///
/// # Composition
///
/// Poses can be composed using `*` operator (chain transformations):
/// ```
/// use vastu_map::core::{Pose2D, Point2D};
///
/// let pose_a = Pose2D::new(1.0, 0.0, std::f32::consts::FRAC_PI_2);
/// let pose_b = Pose2D::new(1.0, 0.0, 0.0);
/// let combined = pose_a * pose_b;  // Apply pose_b in pose_a's frame
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2D {
    /// X position in meters.
    pub x: f32,
    /// Y position in meters.
    pub y: f32,
    /// Heading angle in radians [-π, π), CCW positive from X-axis.
    pub theta: f32,
}

impl Pose2D {
    /// Create a new pose.
    ///
    /// # Arguments
    /// * `x` - X position in meters
    /// * `y` - Y position in meters
    /// * `theta` - Heading angle in radians (will be normalized to [-π, π))
    #[inline]
    pub fn new(x: f32, y: f32, theta: f32) -> Self {
        Self {
            x,
            y,
            theta: normalize_angle(theta),
        }
    }

    /// Create an identity pose (origin, facing forward).
    #[inline]
    pub const fn identity() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }

    /// Create a pose from position and angle.
    #[inline]
    pub fn from_position_angle(position: Point2D, theta: f32) -> Self {
        Self::new(position.x, position.y, theta)
    }

    /// Get the position as a Point2D.
    #[inline]
    pub fn position(self) -> Point2D {
        Point2D::new(self.x, self.y)
    }

    /// Get the forward direction (unit vector).
    #[inline]
    pub fn forward(self) -> Point2D {
        Point2D::new(self.theta.cos(), self.theta.sin())
    }

    /// Get the left direction (unit vector, perpendicular to forward).
    #[inline]
    pub fn left(self) -> Point2D {
        Point2D::new(-self.theta.sin(), self.theta.cos())
    }

    /// Transform a point from this pose's local frame to world frame.
    ///
    /// # Example
    /// ```
    /// use vastu_map::core::{Pose2D, Point2D};
    /// use std::f32::consts::FRAC_PI_2;
    ///
    /// let pose = Pose2D::new(1.0, 0.0, FRAC_PI_2);  // At (1,0), facing left
    /// let local = Point2D::new(1.0, 0.0);  // 1m forward in local frame
    /// let world = pose.transform_point(local);
    /// // Should be at (1, 1) in world frame
    /// ```
    #[inline]
    pub fn transform_point(self, point: Point2D) -> Point2D {
        let (sin, cos) = self.theta.sin_cos();
        Point2D {
            x: self.x + point.x * cos - point.y * sin,
            y: self.y + point.x * sin + point.y * cos,
        }
    }

    /// Transform a point from world frame to this pose's local frame.
    #[inline]
    pub fn inverse_transform_point(self, point: Point2D) -> Point2D {
        let (sin, cos) = self.theta.sin_cos();
        let dx = point.x - self.x;
        let dy = point.y - self.y;
        Point2D {
            x: dx * cos + dy * sin,
            y: -dx * sin + dy * cos,
        }
    }

    /// Compose this pose with another (chain transformations).
    ///
    /// Returns a new pose representing: apply `other` in `self`'s frame.
    /// This is equivalent to matrix multiplication: self * other.
    #[inline]
    pub fn compose(self, other: Pose2D) -> Self {
        let pos = self.transform_point(other.position());
        Self::new(pos.x, pos.y, self.theta + other.theta)
    }

    /// Compute the inverse of this pose.
    ///
    /// The inverse pose, when composed with the original, yields identity:
    /// `pose.compose(pose.inverse()) ≈ Pose2D::identity()`
    #[inline]
    pub fn inverse(self) -> Self {
        let (sin, cos) = self.theta.sin_cos();
        Self::new(
            -self.x * cos - self.y * sin,
            self.x * sin - self.y * cos,
            -self.theta,
        )
    }

    /// Compute the relative pose from `self` to `other`.
    ///
    /// Returns the pose that, when composed with `self`, yields `other`:
    /// `self.compose(self.relative_to(other)) ≈ other`
    #[inline]
    pub fn relative_to(self, other: Pose2D) -> Self {
        self.inverse().compose(other)
    }

    /// Linear interpolation between poses.
    ///
    /// Note: This interpolates theta linearly, which may not be ideal
    /// for large angular differences. For smooth rotation interpolation,
    /// consider using spherical interpolation.
    #[inline]
    pub fn lerp(self, other: Pose2D, t: f32) -> Self {
        use super::math::angle_diff;
        Self::new(
            self.x + (other.x - self.x) * t,
            self.y + (other.y - self.y) * t,
            self.theta + angle_diff(self.theta, other.theta) * t,
        )
    }

    /// Check if this pose is approximately equal to another.
    #[inline]
    pub fn approx_eq(self, other: Pose2D, pos_epsilon: f32, angle_epsilon: f32) -> bool {
        use super::math::angles_approx_equal;
        (self.x - other.x).abs() <= pos_epsilon
            && (self.y - other.y).abs() <= pos_epsilon
            && angles_approx_equal(self.theta, other.theta, angle_epsilon)
    }
}

impl std::ops::Mul for Pose2D {
    type Output = Self;

    /// Compose two poses (same as `compose`).
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        self.compose(rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, PI};

    #[test]
    fn test_new_normalizes_angle() {
        // At ±π boundary, floating-point may give +π or -π; both are valid
        let pose = Pose2D::new(0.0, 0.0, 3.0 * PI);
        assert!(pose.theta.abs() - PI < 1e-5);

        let pose = Pose2D::new(0.0, 0.0, -3.0 * PI);
        assert!(pose.theta.abs() - PI < 1e-5);
    }

    #[test]
    fn test_identity() {
        let pose = Pose2D::identity();
        assert_eq!(pose.x, 0.0);
        assert_eq!(pose.y, 0.0);
        assert_eq!(pose.theta, 0.0);
    }

    #[test]
    fn test_position() {
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let pos = pose.position();
        assert_eq!(pos.x, 1.0);
        assert_eq!(pos.y, 2.0);
    }

    #[test]
    fn test_forward_and_left() {
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let fwd = pose.forward();
        assert_relative_eq!(fwd.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(fwd.y, 0.0, epsilon = 1e-6);

        let left = pose.left();
        assert_relative_eq!(left.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(left.y, 1.0, epsilon = 1e-6);

        // At 90 degrees
        let pose90 = Pose2D::new(0.0, 0.0, FRAC_PI_2);
        let fwd90 = pose90.forward();
        assert_relative_eq!(fwd90.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(fwd90.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_transform_point() {
        // At origin, no rotation
        let pose = Pose2D::identity();
        let local = Point2D::new(1.0, 0.0);
        let world = pose.transform_point(local);
        assert_relative_eq!(world.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(world.y, 0.0, epsilon = 1e-6);

        // At (1, 0), rotated 90 degrees
        let pose = Pose2D::new(1.0, 0.0, FRAC_PI_2);
        let local = Point2D::new(1.0, 0.0);
        let world = pose.transform_point(local);
        assert_relative_eq!(world.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(world.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_inverse_transform_point() {
        let pose = Pose2D::new(1.0, 2.0, FRAC_PI_2);
        let world = Point2D::new(3.0, 4.0);

        let local = pose.inverse_transform_point(world);
        let back = pose.transform_point(local);

        assert_relative_eq!(back.x, world.x, epsilon = 1e-6);
        assert_relative_eq!(back.y, world.y, epsilon = 1e-6);
    }

    #[test]
    fn test_compose() {
        // Translate then rotate
        let translate = Pose2D::new(1.0, 0.0, 0.0);
        let rotate = Pose2D::new(0.0, 0.0, FRAC_PI_2);
        let combined = translate.compose(rotate);

        assert_relative_eq!(combined.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(combined.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(combined.theta, FRAC_PI_2, epsilon = 1e-6);

        // Rotate then translate (in rotated frame)
        let combined2 = rotate.compose(translate);
        assert_relative_eq!(combined2.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(combined2.y, 1.0, epsilon = 1e-6);
        assert_relative_eq!(combined2.theta, FRAC_PI_2, epsilon = 1e-6);
    }

    #[test]
    fn test_inverse() {
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let inv = pose.inverse();
        let identity = pose.compose(inv);

        assert_relative_eq!(identity.x, 0.0, epsilon = 1e-5);
        assert_relative_eq!(identity.y, 0.0, epsilon = 1e-5);
        assert_relative_eq!(identity.theta, 0.0, epsilon = 1e-5);
    }

    #[test]
    fn test_relative_to() {
        let pose_a = Pose2D::new(1.0, 0.0, 0.0);
        let pose_b = Pose2D::new(2.0, 1.0, FRAC_PI_2);

        let relative = pose_a.relative_to(pose_b);
        let reconstructed = pose_a.compose(relative);

        assert_relative_eq!(reconstructed.x, pose_b.x, epsilon = 1e-5);
        assert_relative_eq!(reconstructed.y, pose_b.y, epsilon = 1e-5);
        assert_relative_eq!(reconstructed.theta, pose_b.theta, epsilon = 1e-5);
    }

    #[test]
    fn test_lerp() {
        // Use non-boundary angle to avoid π ambiguity
        let a = Pose2D::new(0.0, 0.0, 0.0);
        let b = Pose2D::new(2.0, 4.0, FRAC_PI_2);

        let mid = a.lerp(b, 0.5);
        assert_relative_eq!(mid.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(mid.y, 2.0, epsilon = 1e-6);
        assert_relative_eq!(mid.theta, FRAC_PI_2 / 2.0, epsilon = 1e-6); // π/4

        // Test full interpolation
        assert_relative_eq!(a.lerp(b, 0.0).theta, 0.0, epsilon = 1e-6);
        assert_relative_eq!(a.lerp(b, 1.0).theta, FRAC_PI_2, epsilon = 1e-6);
    }

    #[test]
    fn test_mul_operator() {
        let a = Pose2D::new(1.0, 0.0, FRAC_PI_2);
        let b = Pose2D::new(1.0, 0.0, 0.0);

        let composed = a.compose(b);
        let multiplied = a * b;

        assert_eq!(composed, multiplied);
    }

    #[test]
    fn test_approx_eq() {
        let a = Pose2D::new(1.0, 2.0, 0.5);
        let b = Pose2D::new(1.001, 2.001, 0.501);

        assert!(a.approx_eq(b, 0.01, 0.01));
        assert!(!a.approx_eq(b, 0.0001, 0.0001));
    }
}
