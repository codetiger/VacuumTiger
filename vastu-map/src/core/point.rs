//! 2D point type for geometric operations.
//!
//! Coordinate frame follows ROS REP-103:
//! - X-forward, Y-left, Z-up (right-handed)
//! - Counter-clockwise positive rotation

use std::ops::{Add, Div, Mul, Neg, Sub};

/// A 2D point/vector in meters.
///
/// Uses the ROS REP-103 coordinate convention:
/// - X: forward (positive ahead of robot)
/// - Y: left (positive to the left of robot)
///
/// This type is used for both points and direction vectors.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point2D {
    /// X coordinate in meters (forward direction).
    pub x: f32,
    /// Y coordinate in meters (left direction).
    pub y: f32,
}

impl Point2D {
    /// Create a new point from x and y coordinates.
    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Create a zero point (origin).
    #[inline]
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    /// Create a point from polar coordinates.
    ///
    /// # Arguments
    /// * `angle` - Angle in radians from X axis (CCW positive)
    /// * `distance` - Distance from origin in meters
    #[inline]
    pub fn from_polar(angle: f32, distance: f32) -> Self {
        Self {
            x: distance * angle.cos(),
            y: distance * angle.sin(),
        }
    }

    /// Squared distance from origin (avoids sqrt).
    #[inline]
    pub fn length_squared(self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    /// Distance from origin.
    #[inline]
    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    /// Squared distance to another point (avoids sqrt).
    #[inline]
    pub fn distance_squared(self, other: Point2D) -> f32 {
        (self - other).length_squared()
    }

    /// Distance to another point.
    #[inline]
    pub fn distance(self, other: Point2D) -> f32 {
        self.distance_squared(other).sqrt()
    }

    /// Normalize to unit length. Returns zero vector if length is zero.
    #[inline]
    pub fn normalized(self) -> Self {
        let len = self.length();
        if len > f32::EPSILON {
            self / len
        } else {
            Self::zero()
        }
    }

    /// Dot product with another vector.
    #[inline]
    pub fn dot(self, other: Point2D) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// 2D cross product (returns scalar z-component).
    ///
    /// This is the signed area of the parallelogram formed by the two vectors.
    /// Positive if `other` is counter-clockwise from `self`.
    #[inline]
    pub fn cross(self, other: Point2D) -> f32 {
        self.x * other.y - self.y * other.x
    }

    /// Perpendicular vector (90° counter-clockwise rotation).
    #[inline]
    pub fn perpendicular(self) -> Self {
        Self {
            x: -self.y,
            y: self.x,
        }
    }

    /// Angle from X axis (in radians, range [-π, π]).
    #[inline]
    pub fn angle(self) -> f32 {
        self.y.atan2(self.x)
    }

    /// Rotate point by angle (radians, counter-clockwise positive).
    #[inline]
    pub fn rotate(self, angle: f32) -> Self {
        let (sin, cos) = angle.sin_cos();
        Self {
            x: self.x * cos - self.y * sin,
            y: self.x * sin + self.y * cos,
        }
    }

    /// Linear interpolation between this point and another.
    #[inline]
    pub fn lerp(self, other: Point2D, t: f32) -> Self {
        Self {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
        }
    }

    /// Component-wise minimum.
    #[inline]
    pub fn min(self, other: Point2D) -> Self {
        Self {
            x: self.x.min(other.x),
            y: self.y.min(other.y),
        }
    }

    /// Component-wise maximum.
    #[inline]
    pub fn max(self, other: Point2D) -> Self {
        Self {
            x: self.x.max(other.x),
            y: self.y.max(other.y),
        }
    }

    /// Check if this point is approximately equal to another.
    #[inline]
    pub fn approx_eq(self, other: Point2D, epsilon: f32) -> bool {
        (self.x - other.x).abs() <= epsilon && (self.y - other.y).abs() <= epsilon
    }
}

impl Add for Point2D {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub for Point2D {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Mul<f32> for Point2D {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Mul<Point2D> for f32 {
    type Output = Point2D;

    #[inline]
    fn mul(self, rhs: Point2D) -> Point2D {
        Point2D {
            x: self * rhs.x,
            y: self * rhs.y,
        }
    }
}

impl Div<f32> for Point2D {
    type Output = Self;

    #[inline]
    fn div(self, rhs: f32) -> Self {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl Neg for Point2D {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::PI;

    #[test]
    fn test_new_and_zero() {
        let p = Point2D::new(1.0, 2.0);
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);

        let z = Point2D::zero();
        assert_eq!(z.x, 0.0);
        assert_eq!(z.y, 0.0);
    }

    #[test]
    fn test_from_polar() {
        let p = Point2D::from_polar(0.0, 1.0);
        assert_relative_eq!(p.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(p.y, 0.0, epsilon = 1e-6);

        let p = Point2D::from_polar(PI / 2.0, 1.0);
        assert_relative_eq!(p.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(p.y, 1.0, epsilon = 1e-6);

        let p = Point2D::from_polar(PI / 4.0, 2.0_f32.sqrt());
        assert_relative_eq!(p.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(p.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_length() {
        let p = Point2D::new(3.0, 4.0);
        assert_eq!(p.length_squared(), 25.0);
        assert_eq!(p.length(), 5.0);
    }

    #[test]
    fn test_distance() {
        let a = Point2D::new(0.0, 0.0);
        let b = Point2D::new(3.0, 4.0);
        assert_eq!(a.distance_squared(b), 25.0);
        assert_eq!(a.distance(b), 5.0);
    }

    #[test]
    fn test_normalized() {
        let p = Point2D::new(3.0, 4.0);
        let n = p.normalized();
        assert_relative_eq!(n.length(), 1.0, epsilon = 1e-6);
        assert_relative_eq!(n.x, 0.6, epsilon = 1e-6);
        assert_relative_eq!(n.y, 0.8, epsilon = 1e-6);

        // Zero vector stays zero
        let z = Point2D::zero().normalized();
        assert_eq!(z, Point2D::zero());
    }

    #[test]
    fn test_dot() {
        let a = Point2D::new(1.0, 0.0);
        let b = Point2D::new(0.0, 1.0);
        assert_eq!(a.dot(b), 0.0); // Perpendicular

        let c = Point2D::new(1.0, 2.0);
        let d = Point2D::new(3.0, 4.0);
        assert_eq!(c.dot(d), 11.0); // 1*3 + 2*4 = 11
    }

    #[test]
    fn test_cross() {
        let a = Point2D::new(1.0, 0.0);
        let b = Point2D::new(0.0, 1.0);
        assert_eq!(a.cross(b), 1.0); // CCW
        assert_eq!(b.cross(a), -1.0); // CW
    }

    #[test]
    fn test_perpendicular() {
        let p = Point2D::new(1.0, 0.0);
        let perp = p.perpendicular();
        assert_relative_eq!(perp.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(perp.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_angle() {
        assert_relative_eq!(Point2D::new(1.0, 0.0).angle(), 0.0, epsilon = 1e-6);
        assert_relative_eq!(Point2D::new(0.0, 1.0).angle(), PI / 2.0, epsilon = 1e-6);
        assert_relative_eq!(Point2D::new(-1.0, 0.0).angle(), PI, epsilon = 1e-6);
        assert_relative_eq!(Point2D::new(0.0, -1.0).angle(), -PI / 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_rotate() {
        let p = Point2D::new(1.0, 0.0);

        let r90 = p.rotate(PI / 2.0);
        assert_relative_eq!(r90.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(r90.y, 1.0, epsilon = 1e-6);

        let r180 = p.rotate(PI);
        assert_relative_eq!(r180.x, -1.0, epsilon = 1e-6);
        assert_relative_eq!(r180.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_lerp() {
        let a = Point2D::new(0.0, 0.0);
        let b = Point2D::new(10.0, 20.0);

        let mid = a.lerp(b, 0.5);
        assert_eq!(mid.x, 5.0);
        assert_eq!(mid.y, 10.0);

        assert_eq!(a.lerp(b, 0.0), a);
        assert_eq!(a.lerp(b, 1.0), b);
    }

    #[test]
    fn test_min_max() {
        let a = Point2D::new(1.0, 5.0);
        let b = Point2D::new(3.0, 2.0);

        let min = a.min(b);
        assert_eq!(min.x, 1.0);
        assert_eq!(min.y, 2.0);

        let max = a.max(b);
        assert_eq!(max.x, 3.0);
        assert_eq!(max.y, 5.0);
    }

    #[test]
    fn test_operators() {
        let a = Point2D::new(1.0, 2.0);
        let b = Point2D::new(3.0, 4.0);

        assert_eq!(a + b, Point2D::new(4.0, 6.0));
        assert_eq!(b - a, Point2D::new(2.0, 2.0));
        assert_eq!(a * 2.0, Point2D::new(2.0, 4.0));
        assert_eq!(2.0 * a, Point2D::new(2.0, 4.0));
        assert_eq!(a / 2.0, Point2D::new(0.5, 1.0));
        assert_eq!(-a, Point2D::new(-1.0, -2.0));
    }

    #[test]
    fn test_approx_eq() {
        let a = Point2D::new(1.0, 2.0);
        let b = Point2D::new(1.001, 2.001);

        assert!(a.approx_eq(b, 0.01));
        assert!(!a.approx_eq(b, 0.0001));
    }
}
