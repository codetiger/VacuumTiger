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

// ─────────────────────────────────────────────────────────────────────────────
// Geometry Utilities (shared by line fitting, RANSAC, etc.)
// ─────────────────────────────────────────────────────────────────────────────

use super::Point2D;

/// Covariance matrix elements for 2D point sets.
///
/// Represents a symmetric 2x2 matrix:
/// ```text
/// | cxx  cxy |
/// | cxy  cyy |
/// ```
#[derive(Debug, Clone, Copy, Default)]
pub struct Covariance2D {
    /// Sum of (x - cx)^2
    pub cxx: f32,
    /// Sum of (y - cy)^2
    pub cyy: f32,
    /// Sum of (x - cx)(y - cy)
    pub cxy: f32,
}

/// Compute the centroid of a set of points.
///
/// # Example
/// ```
/// use vastu_map::core::{Point2D, math::compute_centroid};
///
/// let points = [Point2D::new(0.0, 0.0), Point2D::new(2.0, 0.0), Point2D::new(1.0, 1.0)];
/// let centroid = compute_centroid(&points);
/// assert!((centroid.x - 1.0).abs() < 1e-6);
/// assert!((centroid.y - 1.0 / 3.0).abs() < 1e-6);
/// ```
#[inline]
pub fn compute_centroid(points: &[Point2D]) -> Point2D {
    if points.is_empty() {
        return Point2D::new(0.0, 0.0);
    }

    let n = points.len() as f32;
    let mut sum_x: f32 = 0.0;
    let mut sum_y: f32 = 0.0;

    for p in points {
        sum_x += p.x;
        sum_y += p.y;
    }

    Point2D::new(sum_x / n, sum_y / n)
}

/// Compute the 2x2 covariance matrix elements for a set of points.
///
/// The covariance matrix describes the spread and orientation of points
/// around their centroid. Used for Total Least Squares line fitting and PCA.
///
/// # Arguments
/// * `points` - Point set
/// * `centroid` - Pre-computed centroid (use [`compute_centroid`])
///
/// # Example
/// ```
/// use vastu_map::core::{Point2D, math::{compute_centroid, compute_covariance}};
///
/// let points = [
///     Point2D::new(0.0, 0.0),
///     Point2D::new(1.0, 1.0),
///     Point2D::new(2.0, 2.0),
/// ];
/// let centroid = compute_centroid(&points);
/// let cov = compute_covariance(&points, centroid);
///
/// // Points lie on y=x line, so cxx == cyy and cxy > 0
/// assert!((cov.cxx - cov.cyy).abs() < 1e-6);
/// assert!(cov.cxy > 0.0);
/// ```
#[inline]
pub fn compute_covariance(points: &[Point2D], centroid: Point2D) -> Covariance2D {
    let mut cxx: f32 = 0.0;
    let mut cyy: f32 = 0.0;
    let mut cxy: f32 = 0.0;

    for p in points {
        let dx = p.x - centroid.x;
        let dy = p.y - centroid.y;
        cxx += dx * dx;
        cyy += dy * dy;
        cxy += dx * dy;
    }

    Covariance2D { cxx, cyy, cxy }
}

/// Compute weighted covariance matrix elements.
///
/// Points with higher weights contribute more to the covariance.
/// Useful for range-weighted lidar point fitting.
///
/// # Arguments
/// * `points` - Point set
/// * `weights` - Weight for each point (must have same length as points)
/// * `centroid` - Pre-computed weighted centroid
///
/// # Panics
/// Panics if `points.len() != weights.len()`.
#[inline]
pub fn compute_covariance_weighted(
    points: &[Point2D],
    weights: &[f32],
    centroid: Point2D,
) -> Covariance2D {
    debug_assert_eq!(points.len(), weights.len());

    let mut cxx: f32 = 0.0;
    let mut cyy: f32 = 0.0;
    let mut cxy: f32 = 0.0;

    for (p, &w) in points.iter().zip(weights.iter()) {
        let dx = p.x - centroid.x;
        let dy = p.y - centroid.y;
        cxx += w * dx * dx;
        cyy += w * dy * dy;
        cxy += w * dx * dy;
    }

    Covariance2D { cxx, cyy, cxy }
}

/// Compute the principal direction from a 2x2 covariance matrix.
///
/// Returns the eigenvector corresponding to the larger eigenvalue,
/// which represents the direction of maximum variance.
///
/// Returns `None` if the covariance matrix is degenerate (e.g., all points
/// coincident or forming a circle).
///
/// # Example
/// ```
/// use vastu_map::core::{Point2D, math::{compute_centroid, compute_covariance, principal_direction}};
///
/// // Points on a horizontal line
/// let points = [Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0), Point2D::new(2.0, 0.0)];
/// let centroid = compute_centroid(&points);
/// let cov = compute_covariance(&points, centroid);
/// let dir = principal_direction(&cov).unwrap();
///
/// // Direction should be horizontal (1, 0) or (-1, 0)
/// assert!(dir.y.abs() < 1e-6);
/// assert!((dir.x.abs() - 1.0).abs() < 1e-6);
/// ```
#[inline]
pub fn principal_direction(cov: &Covariance2D) -> Option<Point2D> {
    // Find eigenvectors of 2x2 covariance matrix
    // For 2x2 symmetric matrix, eigenvalues are:
    // λ = (cxx + cyy)/2 ± sqrt(((cxx - cyy)/2)² + cxy²)

    let trace_half = (cov.cxx + cov.cyy) / 2.0;
    let det_sqrt = ((cov.cxx - cov.cyy) / 2.0).powi(2) + cov.cxy * cov.cxy;

    if det_sqrt < f32::EPSILON {
        // Degenerate case - points form a circle or single point
        return None;
    }

    let det_sqrt = det_sqrt.sqrt();
    let lambda1 = trace_half + det_sqrt; // Larger eigenvalue

    // Eigenvector for larger eigenvalue (line direction)
    // For eigenvalue λ1, eigenvector satisfies (cxx - λ1)x + cxy*y = 0
    // Direction: (cxy, λ1 - cxx) or (-cxy, cxx - λ1)

    if cov.cxy.abs() > f32::EPSILON {
        Some(Point2D::new(lambda1 - cov.cyy, cov.cxy).normalized())
    } else if cov.cxx > cov.cyy {
        Some(Point2D::new(1.0, 0.0)) // Horizontal line
    } else {
        Some(Point2D::new(0.0, 1.0)) // Vertical line
    }
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

    // ─────────────────────────────────────────────────────────────────────────
    // Geometry Utility Tests
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_compute_centroid() {
        // Empty points
        let empty: &[Point2D] = &[];
        let c = compute_centroid(empty);
        assert_eq!(c.x, 0.0);
        assert_eq!(c.y, 0.0);

        // Single point
        let single = [Point2D::new(3.0, 4.0)];
        let c = compute_centroid(&single);
        assert_relative_eq!(c.x, 3.0, epsilon = 1e-6);
        assert_relative_eq!(c.y, 4.0, epsilon = 1e-6);

        // Triangle
        let triangle = [
            Point2D::new(0.0, 0.0),
            Point2D::new(3.0, 0.0),
            Point2D::new(0.0, 3.0),
        ];
        let c = compute_centroid(&triangle);
        assert_relative_eq!(c.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(c.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_compute_covariance() {
        // Points on horizontal line (y=0)
        let horizontal = [
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
        ];
        let centroid = compute_centroid(&horizontal);
        let cov = compute_covariance(&horizontal, centroid);

        // cxx > 0, cyy = 0, cxy = 0
        assert!(cov.cxx > 0.0);
        assert_relative_eq!(cov.cyy, 0.0, epsilon = 1e-6);
        assert_relative_eq!(cov.cxy, 0.0, epsilon = 1e-6);

        // Points on diagonal (y=x)
        let diagonal = [
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
            Point2D::new(2.0, 2.0),
        ];
        let centroid = compute_centroid(&diagonal);
        let cov = compute_covariance(&diagonal, centroid);

        // cxx == cyy, cxy > 0
        assert_relative_eq!(cov.cxx, cov.cyy, epsilon = 1e-6);
        assert!(cov.cxy > 0.0);
    }

    #[test]
    fn test_principal_direction_horizontal() {
        let points = [
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
        ];
        let centroid = compute_centroid(&points);
        let cov = compute_covariance(&points, centroid);
        let dir = principal_direction(&cov).unwrap();

        // Direction should be horizontal
        assert!(dir.y.abs() < 1e-6);
        assert!((dir.x.abs() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_principal_direction_vertical() {
        let points = [
            Point2D::new(0.0, 0.0),
            Point2D::new(0.0, 1.0),
            Point2D::new(0.0, 2.0),
        ];
        let centroid = compute_centroid(&points);
        let cov = compute_covariance(&points, centroid);
        let dir = principal_direction(&cov).unwrap();

        // Direction should be vertical
        assert!(dir.x.abs() < 1e-6);
        assert!((dir.y.abs() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_principal_direction_diagonal() {
        let points = [
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
            Point2D::new(2.0, 2.0),
        ];
        let centroid = compute_centroid(&points);
        let cov = compute_covariance(&points, centroid);
        let dir = principal_direction(&cov).unwrap();

        // Direction should be 45° (normalized: ~0.707, ~0.707)
        assert_relative_eq!(dir.x.abs(), dir.y.abs(), epsilon = 1e-5);
        assert!((dir.length() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_principal_direction_degenerate() {
        // Single point - degenerate
        let cov = Covariance2D {
            cxx: 0.0,
            cyy: 0.0,
            cxy: 0.0,
        };
        assert!(principal_direction(&cov).is_none());
    }
}
