//! RANSAC-based robust pose estimation.
//!
//! Uses Random Sample Consensus to find the best pose transformation
//! from a set of point-to-line correspondences, robust to outliers.

use rand::Rng;

use crate::core::{Point2D, Pose2D};
use crate::features::Line2D;

use super::correspondence::{Correspondence, CorrespondenceSet};

/// Configuration for RANSAC pose estimation.
#[derive(Clone, Debug)]
pub struct RansacConfig {
    /// Maximum number of iterations.
    /// Default: 100
    pub max_iterations: usize,

    /// Inlier distance threshold (meters).
    /// Correspondences with distance below this are considered inliers.
    /// Default: 0.1m
    pub inlier_threshold: f32,

    /// Minimum inlier ratio required for a valid model.
    /// If the best model has fewer inliers, the estimation fails.
    /// Default: 0.3 (30%)
    pub min_inlier_ratio: f32,

    /// Early termination if inlier ratio exceeds this.
    /// Default: 0.9 (90%)
    pub early_termination_ratio: f32,

    /// Minimum number of correspondences for estimation.
    /// Default: 3
    pub min_correspondences: usize,
}

impl Default for RansacConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            inlier_threshold: 0.1,
            min_inlier_ratio: 0.3,
            early_termination_ratio: 0.9,
            min_correspondences: 3,
        }
    }
}

impl RansacConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for maximum iterations.
    pub fn with_max_iterations(mut self, iterations: usize) -> Self {
        self.max_iterations = iterations;
        self
    }

    /// Builder-style setter for inlier threshold.
    pub fn with_inlier_threshold(mut self, meters: f32) -> Self {
        self.inlier_threshold = meters;
        self
    }

    /// Builder-style setter for minimum inlier ratio.
    pub fn with_min_inlier_ratio(mut self, ratio: f32) -> Self {
        self.min_inlier_ratio = ratio;
        self
    }

    /// Builder-style setter for minimum correspondences.
    pub fn with_min_correspondences(mut self, count: usize) -> Self {
        self.min_correspondences = count;
        self
    }

    /// Builder-style setter for early termination ratio.
    pub fn with_early_termination_ratio(mut self, ratio: f32) -> Self {
        self.early_termination_ratio = ratio;
        self
    }
}

/// Result of RANSAC pose estimation.
#[derive(Clone, Debug)]
pub struct RansacResult {
    /// Estimated pose transformation.
    pub pose: Pose2D,
    /// Inlier correspondences.
    pub inliers: CorrespondenceSet,
    /// Inlier ratio (inliers / total correspondences).
    pub inlier_ratio: f32,
    /// Number of iterations performed.
    pub iterations: usize,
}

/// Estimate pose transformation using RANSAC.
///
/// Samples random subsets of correspondences, computes pose hypotheses,
/// and finds the pose with the most inliers.
///
/// # Arguments
/// * `correspondences` - Point-to-line correspondences
/// * `lines` - Map lines
/// * `config` - RANSAC configuration
///
/// # Returns
/// RANSAC result if successful, None if no valid model found.
pub fn estimate_pose_ransac(
    correspondences: &CorrespondenceSet,
    lines: &[Line2D],
    config: &RansacConfig,
) -> Option<RansacResult> {
    let n = correspondences.len();
    if n < config.min_correspondences {
        return None;
    }

    let mut rng = rand::rng();
    let mut best_inliers = CorrespondenceSet::new();
    let mut best_pose = Pose2D::identity();
    let mut best_count = 0;

    for iteration in 0..config.max_iterations {
        // Sample 2 random correspondences for minimal set
        let idx1 = rng.random_range(0..n);
        let mut idx2 = rng.random_range(0..n);
        while idx2 == idx1 {
            idx2 = rng.random_range(0..n);
        }

        let corr1 = &correspondences.correspondences[idx1];
        let corr2 = &correspondences.correspondences[idx2];

        // Get the corresponding lines
        let line1 = &lines[corr1.line_idx];
        let line2 = &lines[corr2.line_idx];

        // Compute pose hypothesis from 2 correspondences
        let Some(pose) = estimate_pose_from_two_correspondences(corr1, corr2, line1, line2) else {
            continue;
        };

        // Count inliers
        let (inlier_count, inliers) =
            count_inliers(correspondences, lines, &pose, config.inlier_threshold);

        if inlier_count > best_count {
            best_count = inlier_count;
            best_inliers = inliers;
            best_pose = pose;

            // Early termination check
            let ratio = inlier_count as f32 / n as f32;
            if ratio >= config.early_termination_ratio {
                return Some(RansacResult {
                    pose: best_pose,
                    inliers: best_inliers,
                    inlier_ratio: ratio,
                    iterations: iteration + 1,
                });
            }
        }
    }

    // Check minimum inlier ratio
    let inlier_ratio = best_count as f32 / n as f32;
    if inlier_ratio < config.min_inlier_ratio {
        return None;
    }

    Some(RansacResult {
        pose: best_pose,
        inliers: best_inliers,
        inlier_ratio,
        iterations: config.max_iterations,
    })
}

/// Estimate pose from two point-to-line correspondences.
///
/// Uses the constraint that each point must lie on its corresponding line
/// after transformation.
fn estimate_pose_from_two_correspondences(
    corr1: &Correspondence,
    corr2: &Correspondence,
    line1: &Line2D,
    line2: &Line2D,
) -> Option<Pose2D> {
    let p1 = corr1.point;
    let p2 = corr2.point;

    // Get projection points on lines (where points should map to)
    let target1 = line1.point_at(corr1.projection_t);
    let target2 = line2.point_at(corr2.projection_t);

    // Compute the transformation that maps p1->target1 and p2->target2
    // This is a rigid body transformation (rotation + translation)

    // Vector between source points
    let src_vec = p2 - p1;
    let src_len = src_vec.length();

    if src_len < 1e-6 {
        // Points too close together
        return None;
    }

    // Vector between target points
    let dst_vec = target2 - target1;
    let dst_len = dst_vec.length();

    if dst_len < 1e-6 {
        // Targets too close together
        return None;
    }

    // Check that lengths are approximately equal (rigid transformation)
    let length_ratio = dst_len / src_len;
    if (length_ratio - 1.0).abs() > 0.3 {
        // Too much scale difference - invalid correspondences
        return None;
    }

    // Compute rotation angle
    let src_angle = src_vec.y.atan2(src_vec.x);
    let dst_angle = dst_vec.y.atan2(dst_vec.x);
    let theta = dst_angle - src_angle;

    // Normalize angle
    let theta = crate::core::math::normalize_angle(theta);

    // Compute translation using centroid method
    let src_centroid = (p1 + p2) * 0.5;
    let dst_centroid = (target1 + target2) * 0.5;

    // Rotate source centroid
    let (sin, cos) = theta.sin_cos();
    let rotated_centroid = Point2D::new(
        src_centroid.x * cos - src_centroid.y * sin,
        src_centroid.x * sin + src_centroid.y * cos,
    );

    // Translation is the difference
    let tx = dst_centroid.x - rotated_centroid.x;
    let ty = dst_centroid.y - rotated_centroid.y;

    Some(Pose2D::new(tx, ty, theta))
}

/// Count inliers for a given pose hypothesis.
fn count_inliers(
    correspondences: &CorrespondenceSet,
    lines: &[Line2D],
    pose: &Pose2D,
    threshold: f32,
) -> (usize, CorrespondenceSet) {
    let mut inliers = CorrespondenceSet::with_capacity(correspondences.len());
    let (sin, cos) = pose.theta.sin_cos();

    for corr in correspondences.iter() {
        // Transform the point
        let p = corr.point;
        let transformed = Point2D::new(
            p.x * cos - p.y * sin + pose.x,
            p.x * sin + p.y * cos + pose.y,
        );

        // Compute distance to corresponding line
        let line = &lines[corr.line_idx];
        let distance = line.distance_to_point(transformed);

        if distance <= threshold {
            // Create updated correspondence with new position and distance
            inliers.push(Correspondence::new(
                corr.point_idx,
                corr.line_idx,
                transformed,
                distance,
                line.project_point(transformed),
            ));
        }
    }

    (inliers.len(), inliers)
}

/// Refine pose estimate using all inliers (least squares).
///
/// After RANSAC finds a good initial pose, this function refines it
/// using all inliers for better accuracy.
pub fn refine_pose_with_inliers(
    inliers: &CorrespondenceSet,
    lines: &[Line2D],
    initial_pose: Pose2D,
    iterations: usize,
) -> Pose2D {
    if inliers.is_empty() {
        return initial_pose;
    }

    let mut pose = initial_pose;

    for _ in 0..iterations {
        // Accumulate gradients
        let mut sum_dx = 0.0;
        let mut sum_dy = 0.0;
        let mut sum_dtheta = 0.0;
        let mut count = 0.0;

        let (sin, cos) = pose.theta.sin_cos();

        for corr in inliers.iter() {
            let p = corr.point;

            // Transform point by current pose
            let tx = p.x * cos - p.y * sin + pose.x;
            let ty = p.x * sin + p.y * cos + pose.y;
            let transformed = Point2D::new(tx, ty);

            let line = &lines[corr.line_idx];

            // Get line normal (perpendicular to line direction)
            let normal = line.normal();

            // Signed distance (positive = left of line)
            let signed_dist = line.signed_distance_to_point(transformed);

            // Gradient of distance w.r.t. pose parameters
            // d(dist)/d(tx) = normal.x
            // d(dist)/d(ty) = normal.y
            // d(dist)/d(theta) ≈ cross(p_rotated, normal)
            let p_rotated = Point2D::new(tx - pose.x, ty - pose.y);

            sum_dx += signed_dist * normal.x;
            sum_dy += signed_dist * normal.y;
            sum_dtheta += signed_dist * (p_rotated.x * normal.y - p_rotated.y * normal.x);
            count += 1.0;
        }

        if count < 1.0 {
            break;
        }

        // Apply gradient descent step
        let step = 0.5;
        pose.x -= step * sum_dx / count;
        pose.y -= step * sum_dy / count;
        pose.theta -= step * sum_dtheta / count;
        pose.theta = crate::core::math::normalize_angle(pose.theta);
    }

    pose
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point2D;
    use approx::assert_relative_eq;

    fn make_test_lines() -> Vec<Line2D> {
        vec![
            // Horizontal line at y=0
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            // Vertical line at x=5
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
            // Horizontal line at y=5
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(0.0, 5.0)),
            // Vertical line at x=0
            Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(0.0, 0.0)),
        ]
    }

    fn make_test_correspondences(offset: Point2D) -> CorrespondenceSet {
        let mut corrs = CorrespondenceSet::new();

        // Points that should match to the square walls
        // After applying translation, these should land on the lines
        corrs.push(Correspondence::new(
            0,
            0,
            Point2D::new(1.0, 0.0) - offset,
            0.0,
            0.2,
        ));
        corrs.push(Correspondence::new(
            1,
            0,
            Point2D::new(2.0, 0.0) - offset,
            0.0,
            0.4,
        ));
        corrs.push(Correspondence::new(
            2,
            1,
            Point2D::new(5.0, 1.0) - offset,
            0.0,
            0.2,
        ));
        corrs.push(Correspondence::new(
            3,
            1,
            Point2D::new(5.0, 2.0) - offset,
            0.0,
            0.4,
        ));
        corrs.push(Correspondence::new(
            4,
            2,
            Point2D::new(4.0, 5.0) - offset,
            0.0,
            0.2,
        ));
        corrs.push(Correspondence::new(
            5,
            3,
            Point2D::new(0.0, 3.0) - offset,
            0.0,
            0.6,
        ));

        corrs
    }

    #[test]
    fn test_ransac_translation() {
        let lines = make_test_lines();
        let offset = Point2D::new(0.5, 0.3);
        let corrs = make_test_correspondences(offset);

        let config = RansacConfig::default()
            .with_max_iterations(200)
            .with_inlier_threshold(0.2);

        let result = estimate_pose_ransac(&corrs, &lines, &config);

        assert!(result.is_some(), "RANSAC should find a solution");
        let result = result.unwrap();

        // The pose should approximately recover the offset
        assert_relative_eq!(result.pose.x, offset.x, epsilon = 0.3);
        assert_relative_eq!(result.pose.y, offset.y, epsilon = 0.3);
        assert!(result.inlier_ratio >= 0.3);
    }

    #[test]
    fn test_ransac_insufficient_correspondences() {
        let lines = make_test_lines();
        let mut corrs = CorrespondenceSet::new();
        corrs.push(Correspondence::new(0, 0, Point2D::new(1.0, 0.0), 0.0, 0.2));

        let config = RansacConfig::default().with_min_correspondences(3);
        let result = estimate_pose_ransac(&corrs, &lines, &config);

        assert!(result.is_none());
    }

    #[test]
    fn test_pose_from_two_correspondences() {
        // Two points on a horizontal line
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));
        let line2 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        // Points that are offset from where they should be
        let corr1 = Correspondence::new(0, 0, Point2D::new(1.0, 1.0), 1.0, 0.1);
        let corr2 = Correspondence::new(1, 0, Point2D::new(5.0, 1.0), 1.0, 0.5);

        let pose = estimate_pose_from_two_correspondences(&corr1, &corr2, &line1, &line2);

        assert!(pose.is_some());
        let pose = pose.unwrap();

        // Should translate down by approximately 1
        assert_relative_eq!(pose.y, -1.0, epsilon = 0.1);
        assert_relative_eq!(pose.theta.abs(), 0.0, epsilon = 0.1);
    }

    #[test]
    fn test_count_inliers() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0))];

        let mut corrs = CorrespondenceSet::new();
        // Point at y=0.05 - should be inlier
        corrs.push(Correspondence::new(
            0,
            0,
            Point2D::new(1.0, 0.05),
            0.05,
            0.1,
        ));
        // Point at y=0.5 - should be outlier with threshold 0.1
        corrs.push(Correspondence::new(1, 0, Point2D::new(2.0, 0.5), 0.5, 0.2));

        let pose = Pose2D::identity();
        let (count, inliers) = count_inliers(&corrs, &lines, &pose, 0.1);

        assert_eq!(count, 1);
        assert_eq!(inliers.len(), 1);
    }

    #[test]
    fn test_refine_pose() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 10.0)),
        ];

        let mut corrs = CorrespondenceSet::new();
        // Points slightly off the lines
        corrs.push(Correspondence::new(0, 0, Point2D::new(1.0, 0.1), 0.1, 0.1));
        corrs.push(Correspondence::new(1, 0, Point2D::new(5.0, 0.1), 0.1, 0.5));
        corrs.push(Correspondence::new(2, 1, Point2D::new(0.1, 3.0), 0.1, 0.3));

        let initial = Pose2D::identity();
        let refined = refine_pose_with_inliers(&corrs, &lines, initial, 10);

        // Pose should shift to better align points with lines
        // Points have positive y offset, so pose.y should be negative to compensate
        assert!(refined.y < 0.0 || refined.x < 0.0);
    }

    // === Additional edge case tests ===

    #[test]
    fn test_ransac_high_outlier_ratio() {
        // Test with >50% outliers - RANSAC should still find a valid solution
        // if the inlier threshold is set appropriately
        let lines = make_test_lines();
        let offset = Point2D::new(0.3, 0.2);

        let mut corrs = CorrespondenceSet::new();

        // Add 3 inliers (points that will land on lines after translation)
        corrs.push(Correspondence::new(
            0,
            0,
            Point2D::new(1.0, 0.0) - offset,
            0.0,
            0.2,
        ));
        corrs.push(Correspondence::new(
            1,
            0,
            Point2D::new(3.0, 0.0) - offset,
            0.0,
            0.6,
        ));
        corrs.push(Correspondence::new(
            2,
            1,
            Point2D::new(5.0, 2.0) - offset,
            0.0,
            0.4,
        ));

        // Add 7 outliers (points far from any line)
        for i in 0..7 {
            corrs.push(Correspondence::new(
                3 + i,
                0,
                Point2D::new(2.5 + i as f32 * 0.1, 2.5), // Center of square, far from walls
                2.5,
                0.5,
            ));
        }

        let config = RansacConfig::default()
            .with_max_iterations(500)
            .with_inlier_threshold(0.3)
            .with_min_inlier_ratio(0.2); // Lower threshold since we have many outliers

        let result = estimate_pose_ransac(&corrs, &lines, &config);

        // Should find a solution despite high outlier ratio
        assert!(
            result.is_some(),
            "RANSAC should find solution with 70% outliers"
        );
        let result = result.unwrap();
        assert!(result.inlier_ratio >= 0.2);
    }

    #[test]
    fn test_ransac_all_outliers() {
        // Test with all outliers - RANSAC should return None
        let lines = make_test_lines();

        let mut corrs = CorrespondenceSet::new();

        // All points are far from any line (center of the square)
        for i in 0..10 {
            corrs.push(Correspondence::new(
                i,
                0,
                Point2D::new(2.5 + i as f32 * 0.05, 2.5 + i as f32 * 0.05),
                2.5,
                0.5,
            ));
        }

        let config = RansacConfig::default()
            .with_max_iterations(100)
            .with_inlier_threshold(0.1)
            .with_min_inlier_ratio(0.5);

        let result = estimate_pose_ransac(&corrs, &lines, &config);

        // Should fail - no valid model with >50% inliers
        assert!(result.is_none(), "RANSAC should fail with all outliers");
    }

    #[test]
    fn test_ransac_early_termination() {
        // Test that early termination works when we have very good inliers
        let lines = make_test_lines();
        let offset = Point2D::new(0.2, 0.1);

        // Create correspondences where all points are inliers
        let mut corrs = CorrespondenceSet::new();
        corrs.push(Correspondence::new(
            0,
            0,
            Point2D::new(1.0, 0.0) - offset,
            0.0,
            0.2,
        ));
        corrs.push(Correspondence::new(
            1,
            0,
            Point2D::new(2.0, 0.0) - offset,
            0.0,
            0.4,
        ));
        corrs.push(Correspondence::new(
            2,
            0,
            Point2D::new(3.0, 0.0) - offset,
            0.0,
            0.6,
        ));
        corrs.push(Correspondence::new(
            3,
            0,
            Point2D::new(4.0, 0.0) - offset,
            0.0,
            0.8,
        ));

        let config = RansacConfig::default()
            .with_max_iterations(1000) // Many iterations
            .with_inlier_threshold(0.3)
            .with_early_termination_ratio(0.8); // Should terminate early

        let result = estimate_pose_ransac(&corrs, &lines, &config);

        assert!(result.is_some());
        let result = result.unwrap();

        // Should have terminated early (before max_iterations)
        assert!(
            result.iterations < 1000,
            "Should terminate early, but used {} iterations",
            result.iterations
        );
        assert!(result.inlier_ratio >= 0.8);
    }

    #[test]
    fn test_ransac_degenerate_geometry_collinear() {
        // Test with degenerate case: all correspondences point to same line
        // This should still work as long as we can compute a valid transformation
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0))];

        let offset = Point2D::new(0.0, 0.5); // Only Y offset

        let mut corrs = CorrespondenceSet::new();
        // All points on same line but with Y offset
        corrs.push(Correspondence::new(
            0,
            0,
            Point2D::new(1.0, 0.0) - offset,
            0.5,
            0.1,
        ));
        corrs.push(Correspondence::new(
            1,
            0,
            Point2D::new(3.0, 0.0) - offset,
            0.5,
            0.3,
        ));
        corrs.push(Correspondence::new(
            2,
            0,
            Point2D::new(5.0, 0.0) - offset,
            0.5,
            0.5,
        ));
        corrs.push(Correspondence::new(
            3,
            0,
            Point2D::new(7.0, 0.0) - offset,
            0.5,
            0.7,
        ));

        let config = RansacConfig::default()
            .with_max_iterations(200)
            .with_inlier_threshold(0.3);

        let result = estimate_pose_ransac(&corrs, &lines, &config);

        // Should find a solution (pure translation along Y)
        assert!(result.is_some());
        let result = result.unwrap();
        assert_relative_eq!(result.pose.y, offset.y, epsilon = 0.3);
    }

    #[test]
    fn test_ransac_points_too_close() {
        // Test with points that are too close together
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));
        let line2 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        // Two points that are extremely close (< 1e-6 apart)
        let corr1 = Correspondence::new(0, 0, Point2D::new(1.0, 0.5), 0.5, 0.1);
        let corr2 = Correspondence::new(1, 0, Point2D::new(1.0 + 1e-7, 0.5), 0.5, 0.1);

        let pose = estimate_pose_from_two_correspondences(&corr1, &corr2, &line1, &line2);

        // Should return None because points are too close
        assert!(pose.is_none(), "Should fail with points too close together");
    }

    #[test]
    fn test_ransac_config_builders() {
        // Test all config builder methods
        let config = RansacConfig::new()
            .with_max_iterations(50)
            .with_inlier_threshold(0.2)
            .with_min_inlier_ratio(0.4)
            .with_min_correspondences(5);

        assert_eq!(config.max_iterations, 50);
        assert_eq!(config.inlier_threshold, 0.2);
        assert_eq!(config.min_inlier_ratio, 0.4);
        assert_eq!(config.min_correspondences, 5);
    }

    #[test]
    fn test_ransac_empty_correspondences() {
        let lines = make_test_lines();
        let corrs = CorrespondenceSet::new();

        let config = RansacConfig::default();
        let result = estimate_pose_ransac(&corrs, &lines, &config);

        assert!(result.is_none());
    }

    #[test]
    fn test_refine_pose_empty_inliers() {
        let lines = make_test_lines();
        let corrs = CorrespondenceSet::new();
        let initial = Pose2D::new(1.0, 2.0, 0.5);

        let refined = refine_pose_with_inliers(&corrs, &lines, initial, 10);

        // Should return initial pose unchanged
        assert_relative_eq!(refined.x, initial.x);
        assert_relative_eq!(refined.y, initial.y);
        assert_relative_eq!(refined.theta, initial.theta);
    }
}
