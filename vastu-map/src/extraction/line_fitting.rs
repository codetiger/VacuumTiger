//! Line fitting using Total Least Squares (Orthogonal Regression).
//!
//! TLS minimizes perpendicular distances to the line, not vertical distances
//! like ordinary least squares. This is appropriate for lidar data where
//! errors are isotropic.
//!
//! # Weighted TLS
//!
//! For lidar data, measurement uncertainty increases with range. Points closer
//! to the sensor are more reliable than distant points. Weighted TLS allows
//! incorporating this range-based uncertainty into the fitting.
//!
//! ```rust,ignore
//! use vastu_map::extraction::fit_line_from_sensor;
//! use vastu_map::core::Point2D;
//!
//! let sensor_pos = Point2D::new(0.0, 0.0);  // Robot position
//! let points = vec![/* lidar hits */];
//! let line = fit_line_from_sensor(&points, sensor_pos, None);
//! ```

use crate::config::LidarNoiseModel;
use crate::core::Point2D;
use crate::features::Line2D;

/// Fit a line to a set of points using Total Least Squares.
///
/// The algorithm:
/// 1. Compute centroid of points
/// 2. Compute covariance matrix
/// 3. Find eigenvector of smallest eigenvalue (perpendicular to line)
/// 4. Line direction is eigenvector of largest eigenvalue
///
/// Returns None if:
/// - Fewer than 2 points
/// - Points are collinear in one direction (degenerate case)
///
/// # Example
/// ```
/// use vastu_map::extraction::fit_line;
/// use vastu_map::core::Point2D;
///
/// let points = vec![
///     Point2D::new(0.0, 0.0),
///     Point2D::new(1.0, 0.1),  // Slight noise
///     Point2D::new(2.0, 0.0),
///     Point2D::new(3.0, -0.1),
///     Point2D::new(4.0, 0.0),
/// ];
///
/// let line = fit_line(&points).unwrap();
/// // Line should be approximately horizontal
/// ```
pub fn fit_line(points: &[Point2D]) -> Option<Line2D> {
    if points.len() < 2 {
        return None;
    }

    // Compute centroid
    let n = points.len() as f32;
    let mut sum_x: f32 = 0.0;
    let mut sum_y: f32 = 0.0;

    for p in points {
        sum_x += p.x;
        sum_y += p.y;
    }

    let centroid = Point2D::new(sum_x / n, sum_y / n);

    // Compute covariance matrix elements
    // Cov = | cxx  cxy |
    //       | cxy  cyy |
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

    // Find eigenvectors of 2x2 covariance matrix
    // For 2x2 symmetric matrix, eigenvalues are:
    // λ = (cxx + cyy)/2 ± sqrt(((cxx - cyy)/2)² + cxy²)

    let trace_half = (cxx + cyy) / 2.0;
    let det_sqrt = ((cxx - cyy) / 2.0).powi(2) + cxy * cxy;

    if det_sqrt < f32::EPSILON {
        // Degenerate case - points form a circle or single point
        // Use simple linear regression fallback
        return fit_line_simple(points);
    }

    let det_sqrt = det_sqrt.sqrt();
    let lambda1 = trace_half + det_sqrt; // Larger eigenvalue
    let _lambda2 = trace_half - det_sqrt; // Smaller eigenvalue

    // Eigenvector for larger eigenvalue (line direction)
    // For eigenvalue λ1, eigenvector satisfies (cxx - λ1)x + cxy*y = 0
    // Direction: (cxy, λ1 - cxx) or (-cxy, cxx - λ1)

    let direction = if cxy.abs() > f32::EPSILON {
        Point2D::new(lambda1 - cyy, cxy).normalized()
    } else if cxx > cyy {
        Point2D::new(1.0, 0.0) // Horizontal line
    } else {
        Point2D::new(0.0, 1.0) // Vertical line
    };

    // Find extent of points along line direction
    let mut t_min = f32::MAX;
    let mut t_max = f32::MIN;

    for p in points {
        let t = (p.x - centroid.x) * direction.x + (p.y - centroid.y) * direction.y;
        t_min = t_min.min(t);
        t_max = t_max.max(t);
    }

    // Create line from extent
    let start = Point2D::new(
        centroid.x + t_min * direction.x,
        centroid.y + t_min * direction.y,
    );
    let end = Point2D::new(
        centroid.x + t_max * direction.x,
        centroid.y + t_max * direction.y,
    );

    Some(Line2D::new(start, end))
}

/// Simple line fitting fallback using first and last points.
fn fit_line_simple(points: &[Point2D]) -> Option<Line2D> {
    if points.len() < 2 {
        return None;
    }
    Some(Line2D::new(points[0], points[points.len() - 1]))
}

/// Compute range-based weights for line fitting.
///
/// Points closer to the sensor receive higher weights because they have
/// lower measurement uncertainty. Uses the formula:
/// σ(r) = σ_base + σ_range × r²
/// weight = 1 / σ(r)²
///
/// # Arguments
/// * `points` - Points to compute weights for
/// * `sensor_pos` - Position of the sensor (typically robot position)
/// * `noise_model` - Lidar noise model parameters (uses default if None)
pub fn compute_range_weights(
    points: &[Point2D],
    sensor_pos: Point2D,
    noise_model: Option<&LidarNoiseModel>,
) -> Vec<f32> {
    let model = noise_model.cloned().unwrap_or_default();
    points
        .iter()
        .map(|p| {
            let range = p.distance(sensor_pos);
            model.weight(range)
        })
        .collect()
}

/// Fit a line to points with explicit weights using Weighted Total Least Squares.
///
/// Each point contributes to the fit proportionally to its weight.
/// Higher weights mean the point has more influence on the result.
///
/// # Algorithm
/// 1. Compute weighted centroid: c = Σ(w_i × p_i) / Σw_i
/// 2. Compute weighted covariance matrix: C = Σ(w_i × (p_i - c) × (p_i - c)ᵀ)
/// 3. Find eigenvector of largest eigenvalue (line direction)
/// 4. Project points onto line to find extent
///
/// # Arguments
/// * `points` - Points to fit line to
/// * `weights` - Weights for each point (must match points length)
///
/// # Returns
/// Fitted line segment, or None if points are degenerate
pub fn fit_line_weighted(points: &[Point2D], weights: &[f32]) -> Option<Line2D> {
    if points.len() < 2 || weights.len() != points.len() {
        return None;
    }

    // Compute weighted centroid
    let mut sum_w: f32 = 0.0;
    let mut sum_wx: f32 = 0.0;
    let mut sum_wy: f32 = 0.0;

    for (p, &w) in points.iter().zip(weights.iter()) {
        sum_w += w;
        sum_wx += w * p.x;
        sum_wy += w * p.y;
    }

    if sum_w < f32::EPSILON {
        return fit_line_simple(points);
    }

    let centroid = Point2D::new(sum_wx / sum_w, sum_wy / sum_w);

    // Compute weighted covariance matrix elements
    // Cov = | cxx  cxy |
    //       | cxy  cyy |
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

    // Find eigenvectors of 2x2 covariance matrix
    // For 2x2 symmetric matrix, eigenvalues are:
    // λ = (cxx + cyy)/2 ± sqrt(((cxx - cyy)/2)² + cxy²)

    let trace_half = (cxx + cyy) / 2.0;
    let det_sqrt = ((cxx - cyy) / 2.0).powi(2) + cxy * cxy;

    if det_sqrt < f32::EPSILON {
        // Degenerate case - points form a circle or single point
        return fit_line_simple(points);
    }

    let det_sqrt = det_sqrt.sqrt();
    let lambda1 = trace_half + det_sqrt; // Larger eigenvalue

    // Eigenvector for larger eigenvalue (line direction)
    let direction = if cxy.abs() > f32::EPSILON {
        Point2D::new(lambda1 - cyy, cxy).normalized()
    } else if cxx > cyy {
        Point2D::new(1.0, 0.0) // Horizontal line
    } else {
        Point2D::new(0.0, 1.0) // Vertical line
    };

    // Find extent of points along line direction
    let mut t_min = f32::MAX;
    let mut t_max = f32::MIN;

    for p in points {
        let t = (p.x - centroid.x) * direction.x + (p.y - centroid.y) * direction.y;
        t_min = t_min.min(t);
        t_max = t_max.max(t);
    }

    // Create line from extent
    let start = Point2D::new(
        centroid.x + t_min * direction.x,
        centroid.y + t_min * direction.y,
    );
    let end = Point2D::new(
        centroid.x + t_max * direction.x,
        centroid.y + t_max * direction.y,
    );

    Some(Line2D::new(start, end))
}

/// Fit a line to points using range-based weighting from sensor position.
///
/// This is the preferred method for fitting lines to lidar data, as it
/// accounts for the fact that closer measurements are more reliable.
///
/// # Arguments
/// * `points` - Points to fit line to
/// * `sensor_pos` - Position of the sensor (typically robot position)
/// * `noise_model` - Lidar noise model parameters (uses default if None)
///
/// # Example
/// ```
/// use vastu_map::extraction::fit_line_from_sensor;
/// use vastu_map::core::Point2D;
///
/// let sensor_pos = Point2D::new(0.0, 0.0);
/// let points = vec![
///     Point2D::new(1.0, 0.0),  // 1m away - high weight
///     Point2D::new(2.0, 0.05), // 2m away - medium weight
///     Point2D::new(4.0, 0.0),  // 4m away - lower weight
/// ];
///
/// let line = fit_line_from_sensor(&points, sensor_pos, None).unwrap();
/// // Line is influenced more by the closer point at (1.0, 0.0)
/// ```
pub fn fit_line_from_sensor(
    points: &[Point2D],
    sensor_pos: Point2D,
    noise_model: Option<&LidarNoiseModel>,
) -> Option<Line2D> {
    if points.len() < 2 {
        return None;
    }

    let weights = compute_range_weights(points, sensor_pos, noise_model);
    fit_line_weighted(points, &weights)
}

/// Compute weighted fitting error (RMS distance to line, weighted by measurement quality).
///
/// Returns the weighted root-mean-square of perpendicular distances.
pub fn fitting_error_weighted(points: &[Point2D], line: &Line2D, weights: &[f32]) -> f32 {
    if points.is_empty() || weights.len() != points.len() {
        return 0.0;
    }

    let mut sum_w: f32 = 0.0;
    let mut sum_w_sq: f32 = 0.0;

    for (p, &w) in points.iter().zip(weights.iter()) {
        let dist = line.distance_to_point(*p);
        sum_w += w;
        sum_w_sq += w * dist * dist;
    }

    if sum_w < f32::EPSILON {
        return fitting_error(points, line);
    }

    (sum_w_sq / sum_w).sqrt()
}

/// Compute the perpendicular fitting error (RMS distance to line).
///
/// Returns the root-mean-square of perpendicular distances from points to the line.
pub fn fitting_error(points: &[Point2D], line: &Line2D) -> f32 {
    if points.is_empty() {
        return 0.0;
    }

    let mut sum_sq: f32 = 0.0;
    for p in points {
        let dist = line.distance_to_point(*p);
        sum_sq += dist * dist;
    }

    (sum_sq / points.len() as f32).sqrt()
}

/// Find the point with maximum distance from a line.
///
/// Returns (index, distance) of the farthest point.
/// Returns None if points is empty.
pub fn max_distance_point(points: &[Point2D], line: &Line2D) -> Option<(usize, f32)> {
    if points.is_empty() {
        return None;
    }

    let mut max_idx = 0;
    let mut max_dist: f32 = 0.0;

    for (i, p) in points.iter().enumerate() {
        let dist = line.distance_to_point(*p);
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }

    Some((max_idx, max_dist))
}

/// Fit a line segment to points with known start and end indices.
///
/// The endpoints are snapped to actual point positions for accurate
/// segment boundaries.
pub fn fit_line_segment(points: &[Point2D], start_idx: usize, end_idx: usize) -> Option<Line2D> {
    if start_idx >= points.len() || end_idx >= points.len() || start_idx >= end_idx {
        return None;
    }

    let segment_points = &points[start_idx..=end_idx];
    let mut line = fit_line(segment_points)?;

    // Snap endpoints to actual point projections
    // This ensures the segment covers exactly the input points
    let start_proj = line.project_point_onto_line(points[start_idx]);
    let end_proj = line.project_point_onto_line(points[end_idx]);

    line.start = start_proj;
    line.end = end_proj;

    Some(line)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};

    #[test]
    fn test_fit_horizontal_line() {
        let points = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
            Point2D::new(3.0, 0.0),
            Point2D::new(4.0, 0.0),
        ];

        let line = fit_line(&points).unwrap();

        // Line should be horizontal
        assert_relative_eq!(line.angle(), 0.0, epsilon = 0.01);

        // Should span from (0,0) to (4,0)
        assert_relative_eq!(line.start.x, 0.0, epsilon = 0.01);
        assert_relative_eq!(line.end.x, 4.0, epsilon = 0.01);
        assert_relative_eq!(line.start.y, 0.0, epsilon = 0.01);
        assert_relative_eq!(line.end.y, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_fit_vertical_line() {
        let points = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(0.0, 1.0),
            Point2D::new(0.0, 2.0),
            Point2D::new(0.0, 3.0),
        ];

        let line = fit_line(&points).unwrap();

        // Line should be vertical
        assert_relative_eq!(line.angle().abs(), FRAC_PI_2, epsilon = 0.01);
    }

    #[test]
    fn test_fit_diagonal_line() {
        let points = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
            Point2D::new(2.0, 2.0),
            Point2D::new(3.0, 3.0),
        ];

        let line = fit_line(&points).unwrap();

        // Line should be at 45 degrees
        assert_relative_eq!(line.angle().abs(), FRAC_PI_4, epsilon = 0.01);
    }

    #[test]
    fn test_fit_with_noise() {
        let points = vec![
            Point2D::new(0.0, 0.02),
            Point2D::new(1.0, -0.01),
            Point2D::new(2.0, 0.03),
            Point2D::new(3.0, -0.02),
            Point2D::new(4.0, 0.01),
        ];

        let line = fit_line(&points).unwrap();

        // Line should still be approximately horizontal
        assert!(line.angle().abs() < 0.1);

        // Fitting error should be small
        let error = fitting_error(&points, &line);
        assert!(error < 0.03);
    }

    #[test]
    fn test_fit_two_points() {
        let points = vec![Point2D::new(0.0, 0.0), Point2D::new(5.0, 3.0)];

        let line = fit_line(&points).unwrap();

        assert_relative_eq!(line.start.distance(points[0]), 0.0, epsilon = 0.01);
        assert_relative_eq!(line.end.distance(points[1]), 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_fit_one_point() {
        let points = vec![Point2D::new(0.0, 0.0)];
        assert!(fit_line(&points).is_none());
    }

    #[test]
    fn test_fit_empty() {
        let points: Vec<Point2D> = vec![];
        assert!(fit_line(&points).is_none());
    }

    #[test]
    fn test_fitting_error() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        // Points on the line
        let points_on = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(5.0, 0.0),
            Point2D::new(10.0, 0.0),
        ];
        assert_relative_eq!(fitting_error(&points_on, &line), 0.0, epsilon = 1e-6);

        // Points 1 unit away
        let points_off = vec![
            Point2D::new(0.0, 1.0),
            Point2D::new(5.0, 1.0),
            Point2D::new(10.0, 1.0),
        ];
        assert_relative_eq!(fitting_error(&points_off, &line), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_max_distance_point() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        let points = vec![
            Point2D::new(2.0, 1.0),
            Point2D::new(5.0, 3.0), // Farthest
            Point2D::new(8.0, 2.0),
        ];

        let (idx, dist) = max_distance_point(&points, &line).unwrap();

        assert_eq!(idx, 1);
        assert_relative_eq!(dist, 3.0, epsilon = 1e-6);
    }

    #[test]
    fn test_fit_line_segment() {
        let points = vec![
            Point2D::new(0.0, 0.0), // idx 0
            Point2D::new(1.0, 0.0), // idx 1
            Point2D::new(2.0, 0.0), // idx 2
            Point2D::new(3.0, 0.0), // idx 3
            Point2D::new(4.0, 0.0), // idx 4
        ];

        let line = fit_line_segment(&points, 1, 3).unwrap();

        // Should span from point 1 to point 3
        assert_relative_eq!(line.start.x, 1.0, epsilon = 0.01);
        assert_relative_eq!(line.end.x, 3.0, epsilon = 0.01);
    }

    // ========================================
    // Weighted TLS Tests
    // ========================================

    #[test]
    fn test_compute_range_weights() {
        let sensor_pos = Point2D::new(0.0, 0.0);
        let points = vec![
            Point2D::new(1.0, 0.0), // 1m away
            Point2D::new(2.0, 0.0), // 2m away
            Point2D::new(5.0, 0.0), // 5m away
        ];

        let weights = compute_range_weights(&points, sensor_pos, None);

        assert_eq!(weights.len(), 3);
        // Closer points should have higher weights
        assert!(
            weights[0] > weights[1],
            "1m point should have higher weight than 2m"
        );
        assert!(
            weights[1] > weights[2],
            "2m point should have higher weight than 5m"
        );
    }

    #[test]
    fn test_compute_range_weights_custom_model() {
        let sensor_pos = Point2D::new(0.0, 0.0);
        let points = vec![Point2D::new(1.0, 0.0)];

        let custom_model = LidarNoiseModel::new(0.02, 0.002);
        let weights_custom = compute_range_weights(&points, sensor_pos, Some(&custom_model));
        let weights_default = compute_range_weights(&points, sensor_pos, None);

        // Custom model has higher base sigma, so weight should be lower
        assert!(weights_custom[0] < weights_default[0]);
    }

    #[test]
    fn test_fit_line_weighted_uniform() {
        // With uniform weights, should match unweighted fit
        let points = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
            Point2D::new(3.0, 0.0),
        ];
        let weights = vec![1.0; 4];

        let weighted_line = fit_line_weighted(&points, &weights).unwrap();
        let unweighted_line = fit_line(&points).unwrap();

        assert_relative_eq!(
            weighted_line.angle(),
            unweighted_line.angle(),
            epsilon = 0.01
        );
        assert_relative_eq!(
            weighted_line.start.x,
            unweighted_line.start.x,
            epsilon = 0.01
        );
        assert_relative_eq!(weighted_line.end.x, unweighted_line.end.x, epsilon = 0.01);
    }

    #[test]
    fn test_fit_line_weighted_pulls_toward_high_weight() {
        // Line with an outlier - weighting should affect result
        let points = vec![
            Point2D::new(0.0, 0.0), // Close point, on y=0
            Point2D::new(1.0, 0.0), // Close point, on y=0
            Point2D::new(2.0, 0.0), // Close point, on y=0
            Point2D::new(5.0, 0.5), // Far point with noise (above y=0)
        ];

        // High weights for close points, low weight for far point
        let weighted_weights = vec![10.0, 10.0, 10.0, 0.1];
        // Equal weights
        let uniform_weights = vec![1.0, 1.0, 1.0, 1.0];

        let weighted_line = fit_line_weighted(&points, &weighted_weights).unwrap();
        let uniform_line = fit_line_weighted(&points, &uniform_weights).unwrap();

        // Weighted line should be closer to horizontal (less affected by outlier)
        assert!(
            weighted_line.angle().abs() < uniform_line.angle().abs(),
            "Weighted line should be more horizontal: weighted={:.4}, uniform={:.4}",
            weighted_line.angle(),
            uniform_line.angle()
        );
    }

    #[test]
    fn test_fit_line_from_sensor_horizontal() {
        let sensor_pos = Point2D::new(0.0, 0.0);
        let points = vec![
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
            Point2D::new(3.0, 0.0),
            Point2D::new(4.0, 0.0),
        ];

        let line = fit_line_from_sensor(&points, sensor_pos, None).unwrap();

        // Should be horizontal
        assert_relative_eq!(line.angle(), 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_fit_line_from_sensor_with_noise() {
        let sensor_pos = Point2D::new(0.0, 0.0);
        // Points with noise - closer points are on line, far point has noise
        let points = vec![
            Point2D::new(1.0, 0.0), // 1m, on line
            Point2D::new(2.0, 0.0), // 2m, on line
            Point2D::new(5.0, 0.5), // 5m, noisy
        ];

        let line = fit_line_from_sensor(&points, sensor_pos, None).unwrap();
        let unweighted_line = fit_line(&points).unwrap();

        // Weighted line should be more horizontal because far point has less weight
        // Both should be near-horizontal, but weighted should be closer to 0
        assert!(
            line.angle().abs() <= unweighted_line.angle().abs() + 0.01,
            "Weighted angle {} should be <= unweighted angle {}",
            line.angle(),
            unweighted_line.angle()
        );
    }

    #[test]
    fn test_fit_line_weighted_empty() {
        let points: Vec<Point2D> = vec![];
        let weights: Vec<f32> = vec![];
        assert!(fit_line_weighted(&points, &weights).is_none());
    }

    #[test]
    fn test_fit_line_weighted_mismatched_lengths() {
        let points = vec![Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)];
        let weights = vec![1.0]; // Wrong length
        assert!(fit_line_weighted(&points, &weights).is_none());
    }

    #[test]
    fn test_fitting_error_weighted() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        // Points at different distances from line
        let points = vec![
            Point2D::new(2.0, 1.0), // 1m from line
            Point2D::new(5.0, 2.0), // 2m from line
        ];

        // Give high weight to the 1m point, low weight to 2m point
        let weights = vec![10.0, 0.1];

        let weighted_error = fitting_error_weighted(&points, &line, &weights);

        // Unweighted RMS would be sqrt((1 + 4) / 2) = sqrt(2.5) ≈ 1.58
        // Weighted should be much closer to 1.0 because first point dominates
        assert!(
            weighted_error < 1.2,
            "Weighted error {} should be < 1.2",
            weighted_error
        );
    }
}
