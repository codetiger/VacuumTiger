//! Point-to-Line Iterative Closest Point (ICP) algorithm.
//!
//! This module implements the full scan-to-map matching pipeline:
//!
//! 1. **Correspondence Search**: Find nearest map lines for each scan point
//! 2. **Outlier Rejection**: Use RANSAC or distance threshold
//! 3. **Pose Optimization**: Gauss-Newton solver for point-to-line error
//! 4. **Iteration**: Repeat until convergence
//!
//! # Point-to-Line vs Point-to-Point
//!
//! Point-to-Line ICP converges faster than Point-to-Point because:
//! - Points can slide along lines in the tangent direction
//! - Only the perpendicular distance matters
//! - Better handles structured environments (walls, edges)

use crate::core::{Point2D, Pose2D};
use crate::features::{Line2D, LineCollection};

use super::correspondence::{CorrespondenceSet, MatchResult};
use super::gauss_newton::{GaussNewtonConfig, optimize_pose, optimize_pose_fast};
use super::nearest_neighbor::{
    NearestNeighborConfig, find_correspondences, find_correspondences_batch,
};
use super::ransac::{RansacConfig, estimate_pose_ransac};
use super::scratch::IcpScratchSpace;

/// Configuration for Point-to-Line ICP.
#[derive(Clone, Debug)]
pub struct IcpConfig {
    /// Maximum number of ICP iterations.
    /// Default: 30
    pub max_iterations: usize,

    /// Convergence threshold for pose change.
    /// If ‖Δpose‖ < threshold, ICP stops.
    /// Default: 1e-4
    pub convergence_threshold: f32,

    /// Maximum correspondence distance (meters).
    /// Points farther than this from all lines are rejected.
    /// Default: 0.5m
    pub max_correspondence_distance: f32,

    /// Outlier rejection method.
    /// Default: DistanceThreshold
    pub outlier_rejection: OutlierRejection,

    /// Minimum number of correspondences required.
    /// Default: 10
    pub min_correspondences: usize,

    /// Minimum match confidence to consider successful.
    /// Default: 0.3
    pub min_confidence: f32,

    /// Whether to use RANSAC for initial pose estimate.
    /// Default: false
    pub use_ransac_init: bool,

    /// Whether to use batch (SIMD) correspondence search.
    /// Default: true
    pub use_batch_search: bool,
}

/// Outlier rejection method.
#[derive(Clone, Debug)]
pub enum OutlierRejection {
    /// Reject by distance threshold (fast).
    DistanceThreshold(f32),
    /// Use RANSAC for robust rejection (slower but more robust).
    Ransac(RansacConfig),
    /// No outlier rejection.
    None,
}

impl Default for IcpConfig {
    fn default() -> Self {
        Self {
            max_iterations: 30,
            convergence_threshold: 1e-4,
            max_correspondence_distance: 0.5,
            outlier_rejection: OutlierRejection::DistanceThreshold(0.15),
            min_correspondences: 10,
            min_confidence: 0.3,
            use_ransac_init: false,
            use_batch_search: true,
        }
    }
}

impl IcpConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for maximum iterations.
    pub fn with_max_iterations(mut self, iterations: usize) -> Self {
        self.max_iterations = iterations;
        self
    }

    /// Builder-style setter for convergence threshold.
    pub fn with_convergence_threshold(mut self, threshold: f32) -> Self {
        self.convergence_threshold = threshold;
        self
    }

    /// Builder-style setter for maximum correspondence distance.
    pub fn with_max_correspondence_distance(mut self, meters: f32) -> Self {
        self.max_correspondence_distance = meters;
        self
    }

    /// Builder-style setter for outlier rejection method.
    pub fn with_outlier_rejection(mut self, method: OutlierRejection) -> Self {
        self.outlier_rejection = method;
        self
    }

    /// Builder-style setter for minimum correspondences.
    pub fn with_min_correspondences(mut self, count: usize) -> Self {
        self.min_correspondences = count;
        self
    }

    /// Builder-style setter for RANSAC initialization.
    pub fn with_ransac_init(mut self, enabled: bool) -> Self {
        self.use_ransac_init = enabled;
        self
    }
}

/// Point-to-Line ICP matcher.
///
/// Holds configuration and provides scan matching functionality.
pub struct PointToLineIcp {
    /// ICP configuration.
    config: IcpConfig,
    /// Nearest neighbor configuration.
    nn_config: NearestNeighborConfig,
    /// Gauss-Newton configuration.
    gn_config: GaussNewtonConfig,
}

impl PointToLineIcp {
    /// Create a new ICP matcher with default configuration.
    pub fn new() -> Self {
        Self::with_config(IcpConfig::default())
    }

    /// Create a new ICP matcher with custom configuration.
    pub fn with_config(config: IcpConfig) -> Self {
        let nn_config = NearestNeighborConfig::default()
            .with_max_distance(config.max_correspondence_distance)
            .with_unique_per_point(true);

        let gn_config = GaussNewtonConfig::default()
            .with_max_iterations(10)
            .with_convergence_threshold(config.convergence_threshold / 10.0);

        Self {
            config,
            nn_config,
            gn_config,
        }
    }

    /// Match scan points against map lines.
    ///
    /// # Arguments
    /// * `points` - Scan points in world frame (after initial transform)
    /// * `lines` - Map lines
    /// * `initial_pose` - Initial pose estimate (typically from odometry)
    ///
    /// # Returns
    /// Match result with optimized pose and confidence.
    pub fn match_scan(
        &self,
        points: &[Point2D],
        lines: &[Line2D],
        initial_pose: Pose2D,
    ) -> MatchResult {
        if points.is_empty() || lines.is_empty() {
            return MatchResult::new(initial_pose, CorrespondenceSet::new(), 0, false);
        }

        // Convert lines to collection for batch operations
        let line_collection = if self.config.use_batch_search {
            Some(LineCollection::from_lines(lines))
        } else {
            None
        };

        let mut pose = initial_pose;
        let mut prev_pose = pose;
        let mut final_correspondences = CorrespondenceSet::new();
        let mut converged = false;

        // Optional RANSAC initialization
        if self.config.use_ransac_init {
            let initial_corrs =
                self.find_correspondences_for_pose(points, lines, line_collection.as_ref(), &pose);

            if initial_corrs.len() >= self.config.min_correspondences {
                let ransac_config = RansacConfig::default()
                    .with_max_iterations(50)
                    .with_inlier_threshold(0.15);

                if let Some(ransac_result) =
                    estimate_pose_ransac(&initial_corrs, lines, &ransac_config)
                {
                    pose = pose.compose(ransac_result.pose);
                }
            }
        }

        // Main ICP loop
        for iteration in 0..self.config.max_iterations {
            // 1. Find correspondences at current pose
            let correspondences =
                self.find_correspondences_for_pose(points, lines, line_collection.as_ref(), &pose);

            if correspondences.len() < self.config.min_correspondences {
                // Not enough correspondences - return with low confidence
                return MatchResult::new(pose, correspondences, iteration + 1, false);
            }

            // 2. Reject outliers
            let filtered = self.reject_outliers(&correspondences, lines);

            if filtered.len() < self.config.min_correspondences {
                return MatchResult::new(pose, filtered, iteration + 1, false);
            }

            // 3. Optimize pose using Gauss-Newton
            let gn_result = optimize_pose(&filtered, lines, Pose2D::identity(), &self.gn_config);

            // 4. Update pose (compose with delta)
            pose = pose.compose(gn_result.pose);

            // 5. Check convergence
            let delta_trans =
                ((pose.x - prev_pose.x).powi(2) + (pose.y - prev_pose.y).powi(2)).sqrt();
            let delta_rot = crate::core::math::angle_diff(pose.theta, prev_pose.theta).abs();

            if delta_trans < self.config.convergence_threshold
                && delta_rot < self.config.convergence_threshold
            {
                converged = true;
                final_correspondences = filtered;
                break;
            }

            prev_pose = pose;
            final_correspondences = filtered;
        }

        MatchResult::new(
            pose,
            final_correspondences,
            self.config.max_iterations,
            converged,
        )
    }

    /// Match scan against line collection (optimized for large maps).
    pub fn match_scan_batch(
        &self,
        points: &[Point2D],
        lines: &LineCollection,
        initial_pose: Pose2D,
    ) -> MatchResult {
        let lines_vec = lines.to_lines();
        self.match_scan(points, &lines_vec, initial_pose)
    }

    /// Match scan using pre-allocated scratch space (zero-allocation hot path).
    ///
    /// This is the most efficient matching method, eliminating per-iteration
    /// allocations by reusing buffers from the scratch space.
    ///
    /// # Arguments
    /// * `points` - Scan points in robot frame
    /// * `lines` - Map lines as LineCollection (for SIMD operations)
    /// * `initial_pose` - Initial pose estimate (typically from odometry)
    /// * `scratch` - Pre-allocated scratch space for buffer reuse
    ///
    /// # Returns
    /// Match result with optimized pose and confidence.
    ///
    /// # Example
    /// ```rust,ignore
    /// let mut scratch = IcpScratchSpace::default_capacity();
    /// let icp = PointToLineIcp::new();
    ///
    /// // Reuse scratch across multiple calls
    /// for (scan, odom_pose) in scans.iter().zip(odometry.iter()) {
    ///     let result = icp.match_scan_with_scratch(&scan, &map_lines, *odom_pose, &mut scratch);
    ///     if result.converged {
    ///         // Use result.pose
    ///     }
    /// }
    /// ```
    pub fn match_scan_with_scratch(
        &self,
        points: &[Point2D],
        lines: &LineCollection,
        initial_pose: Pose2D,
        scratch: &mut IcpScratchSpace,
    ) -> MatchResult {
        if points.is_empty() || lines.is_empty() {
            return MatchResult::new(initial_pose, CorrespondenceSet::new(), 0, false);
        }

        // Convert to Line2D for outlier rejection (uses Line2D methods)
        let lines_vec = lines.to_lines();
        let mut pose = initial_pose;
        let mut prev_pose = pose;
        let mut final_correspondences = CorrespondenceSet::new();
        let mut converged = false;

        // Main ICP loop with scratch space
        for iteration in 0..self.config.max_iterations {
            // 1. Transform points and find correspondences using scratch space
            let (sin, cos) = pose.theta.sin_cos();
            scratch.transform_points(points, sin, cos, pose.x, pose.y);
            scratch.find_correspondences(lines, &self.nn_config);

            if scratch.num_correspondences() < self.config.min_correspondences {
                // Not enough correspondences - return with low confidence
                return MatchResult::new(
                    pose,
                    scratch.take_correspondences(),
                    iteration + 1,
                    false,
                );
            }

            // 2. Get correspondences and reject outliers
            let correspondences = scratch.correspondences();
            let filtered = self.reject_outliers(correspondences, &lines_vec);

            if filtered.len() < self.config.min_correspondences {
                return MatchResult::new(pose, filtered, iteration + 1, false);
            }

            // 3. Optimize pose using fast Gauss-Newton (uses pre-computed normals)
            let gn_result =
                optimize_pose_fast(&filtered, lines, Pose2D::identity(), &self.gn_config);

            // 4. Update pose (compose with delta)
            pose = pose.compose(gn_result.pose);

            // 5. Check convergence
            let delta_trans =
                ((pose.x - prev_pose.x).powi(2) + (pose.y - prev_pose.y).powi(2)).sqrt();
            let delta_rot = crate::core::math::angle_diff(pose.theta, prev_pose.theta).abs();

            if delta_trans < self.config.convergence_threshold
                && delta_rot < self.config.convergence_threshold
            {
                converged = true;
                final_correspondences = filtered;
                break;
            }

            prev_pose = pose;
            final_correspondences = filtered;
        }

        MatchResult::new(
            pose,
            final_correspondences,
            self.config.max_iterations,
            converged,
        )
    }

    /// Find correspondences for points at a given pose.
    fn find_correspondences_for_pose(
        &self,
        points: &[Point2D],
        lines: &[Line2D],
        line_collection: Option<&LineCollection>,
        pose: &Pose2D,
    ) -> CorrespondenceSet {
        // Transform points by pose
        let (sin, cos) = pose.theta.sin_cos();
        let transformed: Vec<Point2D> = points
            .iter()
            .map(|p| {
                Point2D::new(
                    p.x * cos - p.y * sin + pose.x,
                    p.x * sin + p.y * cos + pose.y,
                )
            })
            .collect();

        // Find correspondences
        if let Some(collection) = line_collection {
            find_correspondences_batch(&transformed, collection, &self.nn_config)
        } else {
            find_correspondences(&transformed, lines, &self.nn_config)
        }
    }

    /// Reject outliers from correspondences.
    fn reject_outliers(
        &self,
        correspondences: &CorrespondenceSet,
        lines: &[Line2D],
    ) -> CorrespondenceSet {
        match &self.config.outlier_rejection {
            OutlierRejection::None => correspondences.clone(),
            OutlierRejection::DistanceThreshold(threshold) => {
                correspondences.filter_by_distance(*threshold)
            }
            OutlierRejection::Ransac(config) => {
                match estimate_pose_ransac(correspondences, lines, config) {
                    Some(result) => result.inliers,
                    None => correspondences.filter_by_distance(0.2),
                }
            }
        }
    }
}

impl Default for PointToLineIcp {
    fn default() -> Self {
        Self::new()
    }
}

/// Convenience function for single-shot scan matching.
///
/// # Arguments
/// * `points` - Scan points in robot frame
/// * `lines` - Map lines in world frame
/// * `initial_pose` - Initial robot pose in world frame
///
/// # Returns
/// Match result with optimized pose.
pub fn match_scan(points: &[Point2D], lines: &[Line2D], initial_pose: Pose2D) -> MatchResult {
    let icp = PointToLineIcp::new();
    icp.match_scan(points, lines, initial_pose)
}

/// Match scan with custom configuration.
pub fn match_scan_with_config(
    points: &[Point2D],
    lines: &[Line2D],
    initial_pose: Pose2D,
    config: IcpConfig,
) -> MatchResult {
    let icp = PointToLineIcp::with_config(config);
    icp.match_scan(points, lines, initial_pose)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point2D;
    use approx::assert_relative_eq;

    fn make_room_lines() -> Vec<Line2D> {
        // 4m x 4m room centered at origin
        vec![
            Line2D::new(Point2D::new(-2.0, -2.0), Point2D::new(2.0, -2.0)), // Bottom
            Line2D::new(Point2D::new(2.0, -2.0), Point2D::new(2.0, 2.0)),   // Right
            Line2D::new(Point2D::new(2.0, 2.0), Point2D::new(-2.0, 2.0)),   // Top
            Line2D::new(Point2D::new(-2.0, 2.0), Point2D::new(-2.0, -2.0)), // Left
        ]
    }

    fn make_scan_points_at_origin() -> Vec<Point2D> {
        // Points as if robot is at origin, looking at walls
        let mut points = Vec::new();

        // Points seeing the four walls
        for i in -5..=5 {
            let x = i as f32 * 0.3;
            // Bottom wall (y = -2)
            points.push(Point2D::new(x, -2.0));
            // Top wall (y = 2)
            points.push(Point2D::new(x, 2.0));
        }
        for i in -5..=5 {
            let y = i as f32 * 0.3;
            // Left wall (x = -2)
            points.push(Point2D::new(-2.0, y));
            // Right wall (x = 2)
            points.push(Point2D::new(2.0, y));
        }

        points
    }

    #[test]
    fn test_icp_identity() {
        let lines = make_room_lines();
        let points = make_scan_points_at_origin();

        let icp = PointToLineIcp::new();
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        assert!(result.converged);
        assert_relative_eq!(result.pose.x, 0.0, epsilon = 0.1);
        assert_relative_eq!(result.pose.y, 0.0, epsilon = 0.1);
        assert_relative_eq!(result.pose.theta.abs(), 0.0, epsilon = 0.1);
    }

    #[test]
    fn test_icp_with_offset() {
        let lines = make_room_lines();

        // Create scan points slightly offset from walls (simulating sensor noise)
        let mut points = Vec::new();
        for i in -5..=5 {
            let x = i as f32 * 0.3;
            // Points near bottom wall with small offset
            points.push(Point2D::new(x, -1.95));
            // Points near top wall with small offset
            points.push(Point2D::new(x, 1.95));
        }
        for i in -5..=5 {
            let y = i as f32 * 0.3;
            // Points near left wall with small offset
            points.push(Point2D::new(-1.95, y));
            // Points near right wall with small offset
            points.push(Point2D::new(1.95, y));
        }

        let config = IcpConfig::default()
            .with_max_iterations(50)
            .with_max_correspondence_distance(0.5);

        let icp = PointToLineIcp::with_config(config);
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        // Should find correspondences
        assert!(!result.correspondences.is_empty());
        // RMS error should be small (points are close to lines)
        assert!(
            result.rms_error < 0.2,
            "RMS error too high: {}",
            result.rms_error
        );
    }

    #[test]
    fn test_icp_insufficient_points() {
        let lines = make_room_lines();
        let points = vec![Point2D::new(0.0, -2.0)]; // Only one point

        let config = IcpConfig::default().with_min_correspondences(5);
        let icp = PointToLineIcp::with_config(config);
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        assert!(!result.converged);
    }

    #[test]
    fn test_icp_no_lines() {
        let lines: Vec<Line2D> = vec![];
        let points = make_scan_points_at_origin();

        let icp = PointToLineIcp::new();
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        assert!(!result.converged);
        assert!(result.correspondences.is_empty());
    }

    #[test]
    fn test_icp_with_ransac() {
        let lines = make_room_lines();
        let mut points = make_scan_points_at_origin();

        // Add some outliers
        points.push(Point2D::new(10.0, 10.0));
        points.push(Point2D::new(-10.0, -10.0));
        points.push(Point2D::new(5.0, -5.0));

        let config = IcpConfig::default()
            .with_outlier_rejection(OutlierRejection::Ransac(RansacConfig::default()));

        let icp = PointToLineIcp::with_config(config);
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        // Should still converge despite outliers
        assert!(result.converged);
    }

    #[test]
    fn test_icp_distance_threshold() {
        let lines = make_room_lines();
        let mut points = make_scan_points_at_origin();

        // Add outliers far from lines
        points.push(Point2D::new(0.0, 0.0)); // Center - far from all walls

        let config =
            IcpConfig::default().with_outlier_rejection(OutlierRejection::DistanceThreshold(0.5));

        let icp = PointToLineIcp::with_config(config);
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        assert!(result.converged);
        // Center point should be filtered out
        assert!(result.correspondences.len() < points.len());
    }

    #[test]
    fn test_match_result_confidence() {
        let lines = make_room_lines();
        let points = make_scan_points_at_origin();

        let result = match_scan(&points, &lines, Pose2D::identity());

        assert!(result.converged);
        assert!(result.confidence > 0.0);
        assert!(result.is_good_match(0.1));
    }

    #[test]
    fn test_convenience_function() {
        let lines = make_room_lines();
        let points = make_scan_points_at_origin();

        let result = match_scan(&points, &lines, Pose2D::identity());

        assert!(result.converged);
    }
}
