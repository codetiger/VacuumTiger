//! Mathematical utilities for angles and geometry.
//!
//! All angles are in radians. Coordinate frame follows ROS REP-103:
//! - X-forward, Y-left, Z-up
//! - Counter-clockwise positive rotation

use std::f32::consts::PI;

/// Two times PI (full circle in radians).
pub const TWO_PI: f32 = 2.0 * PI;

/// Normalize angle to [-π, π).
///
/// # Example
/// ```
/// use vastu_map::core::math::normalize_angle;
/// use std::f32::consts::PI;
///
/// // Values near ±π may normalize to either +π or -π due to floating-point
/// assert!(normalize_angle(3.0 * PI).abs() - PI < 1e-5);
/// assert!(normalize_angle(-3.0 * PI).abs() - PI < 1e-5);
/// assert!((normalize_angle(PI / 2.0) - PI / 2.0).abs() < 1e-6);
/// ```
#[inline]
pub fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle % TWO_PI;
    if a >= PI {
        a -= TWO_PI;
    } else if a < -PI {
        a += TWO_PI;
    }
    a
}

/// Compute the signed angular difference between two angles.
///
/// Returns the shortest angular distance from `from` to `to`,
/// in the range [-π, π).
///
/// Positive result means counter-clockwise rotation from `from` to `to`.
///
/// # Example
/// ```
/// use vastu_map::core::math::angle_diff;
/// use std::f32::consts::PI;
///
/// // 90° difference
/// let diff = angle_diff(0.0, PI / 2.0);
/// assert!((diff - PI / 2.0).abs() < 1e-6);
///
/// // Crossing -π/π boundary
/// let diff = angle_diff(-0.9 * PI, 0.9 * PI);
/// assert!((diff - (-0.2 * PI)).abs() < 1e-5);
/// ```
#[inline]
pub fn angle_diff(from: f32, to: f32) -> f32 {
    normalize_angle(to - from)
}

/// Check if two angles are approximately equal (within tolerance).
///
/// Handles wrap-around at ±π correctly.
///
/// # Example
/// ```
/// use vastu_map::core::math::angles_approx_equal;
/// use std::f32::consts::PI;
///
/// assert!(angles_approx_equal(PI - 0.001, -PI + 0.001, 0.01));
/// assert!(!angles_approx_equal(0.0, PI, 0.1));
/// ```
#[inline]
pub fn angles_approx_equal(a: f32, b: f32, tolerance: f32) -> bool {
    angle_diff(a, b).abs() <= tolerance
}

/// Convert degrees to radians.
#[inline]
pub fn deg_to_rad(deg: f32) -> f32 {
    deg * PI / 180.0
}

/// Convert radians to degrees.
#[inline]
pub fn rad_to_deg(rad: f32) -> f32 {
    rad * 180.0 / PI
}

/// Clamp a value to a range.
#[inline]
pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}

/// Linear interpolation between two values.
#[inline]
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Square of a value. Useful for avoiding `pow(x, 2)`.
#[inline]
pub fn sq(x: f32) -> f32 {
    x * x
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_normalize_angle() {
        assert_relative_eq!(normalize_angle(0.0), 0.0, epsilon = 1e-6);
        // At ±π boundary, floating-point may give +π or -π; both are valid
        assert!(normalize_angle(PI).abs() - PI < 1e-6);
        assert!(normalize_angle(-PI).abs() - PI < 1e-6);
        assert_relative_eq!(normalize_angle(TWO_PI), 0.0, epsilon = 1e-6);
        // For 3π, should normalize to approximately ±π
        assert!(normalize_angle(3.0 * PI).abs() - PI < 1e-5);
        assert!(normalize_angle(-3.0 * PI).abs() - PI < 1e-5);
        assert_relative_eq!(normalize_angle(PI / 2.0), PI / 2.0, epsilon = 1e-6);
        assert_relative_eq!(normalize_angle(-PI / 2.0), -PI / 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_angle_diff() {
        assert_relative_eq!(angle_diff(0.0, PI / 2.0), PI / 2.0, epsilon = 1e-6);
        assert_relative_eq!(angle_diff(PI / 2.0, 0.0), -PI / 2.0, epsilon = 1e-6);
        assert_relative_eq!(angle_diff(0.0, PI), -PI, epsilon = 1e-6);
        assert_relative_eq!(angle_diff(0.0, -PI), -PI, epsilon = 1e-6);

        // Crossing boundary
        assert_relative_eq!(angle_diff(-0.9 * PI, 0.9 * PI), -0.2 * PI, epsilon = 1e-5);
        assert_relative_eq!(angle_diff(0.9 * PI, -0.9 * PI), 0.2 * PI, epsilon = 1e-5);
    }

    #[test]
    fn test_angles_approx_equal() {
        assert!(angles_approx_equal(0.0, 0.001, 0.01));
        assert!(angles_approx_equal(PI - 0.001, -PI + 0.001, 0.01));
        assert!(!angles_approx_equal(0.0, PI, 0.1));
    }

    #[test]
    fn test_deg_rad_conversion() {
        assert_relative_eq!(deg_to_rad(180.0), PI, epsilon = 1e-6);
        assert_relative_eq!(deg_to_rad(90.0), PI / 2.0, epsilon = 1e-6);
        assert_relative_eq!(rad_to_deg(PI), 180.0, epsilon = 1e-6);
        assert_relative_eq!(rad_to_deg(PI / 2.0), 90.0, epsilon = 1e-6);
    }

    #[test]
    fn test_clamp() {
        assert_eq!(clamp(5.0, 0.0, 10.0), 5.0);
        assert_eq!(clamp(-5.0, 0.0, 10.0), 0.0);
        assert_eq!(clamp(15.0, 0.0, 10.0), 10.0);
    }

    #[test]
    fn test_lerp() {
        assert_eq!(lerp(0.0, 10.0, 0.0), 0.0);
        assert_eq!(lerp(0.0, 10.0, 1.0), 10.0);
        assert_eq!(lerp(0.0, 10.0, 0.5), 5.0);
    }

    #[test]
    fn test_sq() {
        assert_eq!(sq(2.0), 4.0);
        assert_eq!(sq(-3.0), 9.0);
        assert_eq!(sq(0.0), 0.0);
    }
}
