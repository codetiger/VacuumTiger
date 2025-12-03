//! Mathematical primitives for 2D SLAM operations.
//!
//! Functions for angle normalization and angular arithmetic.

use std::f32::consts::PI;

/// Normalize angle to [-π, π].
///
/// # Example
/// ```
/// use dhruva_slam::core::math::normalize_angle;
/// use std::f32::consts::PI;
///
/// assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-6);
/// assert!((normalize_angle(-3.0 * PI) - (-PI)).abs() < 1e-6);
/// ```
#[inline]
pub fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Shortest angular difference from angle `a` to angle `b`.
///
/// Returns the signed angle you need to add to `a` to reach `b`,
/// taking the shortest path around the circle.
///
/// # Example
/// ```
/// use dhruva_slam::core::math::angle_diff;
/// use std::f32::consts::PI;
///
/// // From 0 to π/2 is +π/2
/// assert!((angle_diff(0.0, PI / 2.0) - PI / 2.0).abs() < 1e-6);
///
/// // Crossing the ±π boundary takes the short way
/// let diff = angle_diff(PI - 0.1, -PI + 0.1);
/// assert!((diff - 0.2).abs() < 1e-6);
/// ```
#[inline]
pub fn angle_diff(a: f32, b: f32) -> f32 {
    normalize_angle(b - a)
}

/// Linear interpolation between two angles, taking the shortest path.
///
/// `t` should be in [0, 1] where 0 returns `a` and 1 returns `b`.
#[inline]
pub fn angle_lerp(a: f32, b: f32, t: f32) -> f32 {
    normalize_angle(a + angle_diff(a, b) * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_normalize_angle_zero() {
        assert_relative_eq!(normalize_angle(0.0), 0.0);
    }

    #[test]
    fn test_normalize_angle_pi() {
        assert_relative_eq!(normalize_angle(PI), PI);
        assert_relative_eq!(normalize_angle(-PI), -PI);
    }

    #[test]
    fn test_normalize_angle_wrap_positive() {
        assert_relative_eq!(normalize_angle(2.0 * PI), 0.0, epsilon = 1e-6);
        assert_relative_eq!(normalize_angle(3.0 * PI), PI, epsilon = 1e-6);
        assert_relative_eq!(normalize_angle(4.0 * PI), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_normalize_angle_wrap_negative() {
        assert_relative_eq!(normalize_angle(-2.0 * PI), 0.0, epsilon = 1e-6);
        assert_relative_eq!(normalize_angle(-3.0 * PI), -PI, epsilon = 1e-6);
    }

    #[test]
    fn test_angle_diff_same_sign() {
        assert_relative_eq!(angle_diff(0.0, PI / 2.0), PI / 2.0);
        assert_relative_eq!(angle_diff(PI / 2.0, 0.0), -PI / 2.0);
    }

    #[test]
    fn test_angle_diff_crossing_pi() {
        // From just below π to just above -π (should be small positive)
        assert_relative_eq!(angle_diff(PI - 0.1, -PI + 0.1), 0.2, epsilon = 1e-6);
        // From just above -π to just below π (should be small negative)
        assert_relative_eq!(angle_diff(-PI + 0.1, PI - 0.1), -0.2, epsilon = 1e-6);
    }

    #[test]
    fn test_angle_lerp() {
        // Simple interpolation
        assert_relative_eq!(angle_lerp(0.0, PI / 2.0, 0.0), 0.0);
        assert_relative_eq!(angle_lerp(0.0, PI / 2.0, 1.0), PI / 2.0);
        assert_relative_eq!(angle_lerp(0.0, PI / 2.0, 0.5), PI / 4.0);

        // Crossing ±π boundary
        let result = angle_lerp(PI - 0.1, -PI + 0.1, 0.5);
        assert_relative_eq!(result, PI, epsilon = 1e-6);
    }

    #[test]
    fn test_normalize_angle_very_large_positive() {
        let result = normalize_angle(100.0 * PI);
        assert_relative_eq!(result, 0.0, epsilon = 1e-4);
    }

    #[test]
    fn test_normalize_angle_very_large_negative() {
        let result = normalize_angle(-100.0 * PI);
        assert_relative_eq!(result, 0.0, epsilon = 1e-4);
    }

    #[test]
    fn test_normalize_angle_exactly_at_boundary() {
        assert_relative_eq!(normalize_angle(PI), PI, epsilon = 1e-6);
        assert_relative_eq!(normalize_angle(-PI), -PI, epsilon = 1e-6);
    }

    #[test]
    fn test_normalize_angle_just_beyond_boundary() {
        let just_over = PI + 0.001;
        let result = normalize_angle(just_over);
        assert!(result < 0.0, "Should wrap to negative: {}", result);
        assert_relative_eq!(result, -PI + 0.001, epsilon = 1e-5);

        let just_under = -PI - 0.001;
        let result = normalize_angle(just_under);
        assert!(result > 0.0, "Should wrap to positive: {}", result);
        assert_relative_eq!(result, PI - 0.001, epsilon = 1e-5);
    }

    #[test]
    fn test_angle_diff_same_angle() {
        assert_relative_eq!(angle_diff(1.0, 1.0), 0.0, epsilon = 1e-6);
        assert_relative_eq!(angle_diff(PI, PI), 0.0, epsilon = 1e-6);
        assert_relative_eq!(angle_diff(-PI, -PI), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_angle_diff_opposite_directions() {
        assert_relative_eq!(angle_diff(0.0, PI), PI, epsilon = 1e-6);
        let diff = angle_diff(0.0, -PI);
        assert!(diff.abs() > PI - 0.01, "Should be near ±π: {}", diff);
    }

    #[test]
    fn test_angle_lerp_t_outside_range() {
        let result = angle_lerp(0.0, PI / 2.0, -1.0);
        assert_relative_eq!(result, -PI / 2.0, epsilon = 1e-6);

        let result = angle_lerp(0.0, PI / 2.0, 2.0);
        assert_relative_eq!(result, PI, epsilon = 1e-6);
    }

    #[test]
    fn test_angle_lerp_identical_angles() {
        let result = angle_lerp(1.5, 1.5, 0.5);
        assert_relative_eq!(result, 1.5, epsilon = 1e-6);
    }

    #[test]
    fn test_normalize_handles_nan_gracefully() {
        let result = normalize_angle(f32::NAN);
        assert!(result.is_nan());
    }

    #[test]
    fn test_normalize_handles_infinity() {
        let result = normalize_angle(f32::INFINITY);
        assert!(result.is_nan());
    }
}
