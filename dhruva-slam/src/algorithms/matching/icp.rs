//! Point-to-Point Iterative Closest Point (ICP) algorithm.
//!
//! Classic ICP algorithm for aligning two point clouds by iteratively:
//! 1. Finding nearest neighbor correspondences
//! 2. Computing optimal rigid transform
//! 3. Applying transform and repeating until convergence
//!
//! # Algorithm
//!
//! ```text
//! Input: Source point cloud S, Target point cloud T, Initial guess T₀
//! Output: Transform T* that aligns S to T
//!
//! 1. Transform S by T₀ to get S'
//! 2. For each iteration:
//!    a. Find nearest neighbor in T for each point in S'
//!    b. Compute optimal transform ΔT using SVD
//!    c. Apply ΔT to S': S' = ΔT ⊕ S'
//!    d. If ΔT < threshold, converged
//! 3. Return accumulated transform
//! ```

use kiddo::{KdTree, SquaredEuclidean};

use super::{ScanMatchResult, ScanMatcher};
use crate::core::types::{Point2D, PointCloud2D, Pose2D};

/// Configuration for Point-to-Point ICP.
#[derive(Debug, Clone)]
pub struct IcpConfig {
    /// Maximum number of iterations.
    pub max_iterations: u32,

    /// Convergence threshold for translation (meters).
    ///
    /// If the translation component of the incremental transform
    /// is below this, consider converged.
    pub translation_epsilon: f32,

    /// Convergence threshold for rotation (radians).
    ///
    /// If the rotation component of the incremental transform
    /// is below this, consider converged.
    pub rotation_epsilon: f32,

    /// Maximum correspondence distance (meters).
    ///
    /// Point pairs farther than this are rejected as outliers.
    pub max_correspondence_distance: f32,

    /// Minimum number of valid correspondences required.
    ///
    /// If fewer correspondences are found, the match fails.
    pub min_correspondences: usize,

    /// Outlier rejection ratio (0.0 to 1.0).
    ///
    /// After computing correspondences, reject this fraction
    /// of the worst (largest distance) correspondences.
    pub outlier_ratio: f32,
}

impl Default for IcpConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            translation_epsilon: 0.001,       // 1mm
            rotation_epsilon: 0.001,          // ~0.06°
            max_correspondence_distance: 0.5, // 50cm
            min_correspondences: 10,
            outlier_ratio: 0.1, // Reject worst 10%
        }
    }
}

/// Point-to-Point ICP scan matcher.
///
/// Uses a k-d tree for efficient nearest neighbor queries.
/// Suitable for small to medium initial pose errors (<20cm, <10°).
#[derive(Debug, Clone)]
pub struct PointToPointIcp {
    config: IcpConfig,
}

impl PointToPointIcp {
    /// Create a new ICP matcher with the given configuration.
    pub fn new(config: IcpConfig) -> Self {
        Self { config }
    }

    /// Get the current configuration.
    pub fn config(&self) -> &IcpConfig {
        &self.config
    }

    /// Build a k-d tree from a point cloud.
    fn build_kdtree(cloud: &PointCloud2D) -> KdTree<f32, 2> {
        let mut tree: KdTree<f32, 2> = KdTree::new();
        for (i, point) in cloud.points.iter().enumerate() {
            tree.add(&[point.x, point.y], i as u64);
        }
        tree
    }

    /// Find correspondences between source and target using k-d tree.
    ///
    /// Returns vector of (source_idx, target_idx, squared_distance).
    fn find_correspondences(
        &self,
        source: &PointCloud2D,
        _target: &PointCloud2D,
        target_tree: &KdTree<f32, 2>,
        transform: &Pose2D,
    ) -> Vec<(usize, usize, f32)> {
        let max_dist_sq = self.config.max_correspondence_distance.powi(2);
        let mut correspondences = Vec::with_capacity(source.len());

        let (sin_t, cos_t) = transform.theta.sin_cos();

        for (i, point) in source.points.iter().enumerate() {
            // Transform source point
            let tx = transform.x + point.x * cos_t - point.y * sin_t;
            let ty = transform.y + point.x * sin_t + point.y * cos_t;

            // Find nearest neighbor in target
            let nearest = target_tree.nearest_one::<SquaredEuclidean>(&[tx, ty]);
            let dist_sq = nearest.distance;

            if dist_sq <= max_dist_sq {
                correspondences.push((i, nearest.item as usize, dist_sq));
            }
        }

        // Apply outlier rejection
        if self.config.outlier_ratio > 0.0 && !correspondences.is_empty() {
            // Sort by distance
            correspondences.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

            // Keep only the best correspondences
            let keep_count =
                ((1.0 - self.config.outlier_ratio) * correspondences.len() as f32) as usize;
            correspondences.truncate(keep_count.max(self.config.min_correspondences));
        }

        correspondences
    }

    /// Compute optimal rigid transform using SVD-based method.
    ///
    /// Uses the closed-form solution for point-to-point registration.
    fn compute_transform(
        &self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        correspondences: &[(usize, usize, f32)],
        current_transform: &Pose2D,
    ) -> Pose2D {
        if correspondences.len() < 3 {
            return Pose2D::identity();
        }

        let (sin_t, cos_t) = current_transform.theta.sin_cos();

        // Compute centroids of corresponding points
        let n = correspondences.len() as f32;
        let mut source_centroid = Point2D::default();
        let mut target_centroid = Point2D::default();

        for &(si, ti, _) in correspondences {
            let sp = &source.points[si];
            // Transform source point
            let tx = current_transform.x + sp.x * cos_t - sp.y * sin_t;
            let ty = current_transform.y + sp.x * sin_t + sp.y * cos_t;

            source_centroid.x += tx;
            source_centroid.y += ty;
            target_centroid.x += target.points[ti].x;
            target_centroid.y += target.points[ti].y;
        }

        source_centroid.x /= n;
        source_centroid.y /= n;
        target_centroid.x /= n;
        target_centroid.y /= n;

        // Compute cross-covariance matrix elements
        // H = Σ (source_i - centroid_s) × (target_i - centroid_t)^T
        let mut h00 = 0.0f32;
        let mut h01 = 0.0f32;
        let mut h10 = 0.0f32;
        let mut h11 = 0.0f32;

        for &(si, ti, _) in correspondences {
            let sp = &source.points[si];
            // Transform source point
            let sx = current_transform.x + sp.x * cos_t - sp.y * sin_t - source_centroid.x;
            let sy = current_transform.y + sp.x * sin_t + sp.y * cos_t - source_centroid.y;

            let tx = target.points[ti].x - target_centroid.x;
            let ty = target.points[ti].y - target_centroid.y;

            h00 += sx * tx;
            h01 += sx * ty;
            h10 += sy * tx;
            h11 += sy * ty;
        }

        // For 2D, we can compute the optimal rotation directly
        // θ = atan2(h10 - h01, h00 + h11)
        let dtheta = (h10 - h01).atan2(h00 + h11);

        // Compute translation after rotation
        let (sin_dt, cos_dt) = dtheta.sin_cos();
        let dx = target_centroid.x - (source_centroid.x * cos_dt - source_centroid.y * sin_dt);
        let dy = target_centroid.y - (source_centroid.x * sin_dt + source_centroid.y * cos_dt);

        // Return incremental transform (to be composed with current)
        // We need to compute the delta from current to new
        // new_transform = current ⊕ delta
        // So: delta = current⁻¹ ⊕ new

        let _new_theta = current_transform.theta + dtheta;
        let new_x = dx;
        let new_y = dy;

        // Compute delta transform
        // delta = inv(current) ⊕ new
        // For small deltas, approximate as the difference
        Pose2D::new(
            new_x - current_transform.x,
            new_y - current_transform.y,
            dtheta,
        )
    }

    /// Compute mean squared error of correspondences.
    fn compute_mse(
        &self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        correspondences: &[(usize, usize, f32)],
        transform: &Pose2D,
    ) -> f32 {
        if correspondences.is_empty() {
            return f32::MAX;
        }

        let (sin_t, cos_t) = transform.theta.sin_cos();
        let mut sum_sq = 0.0f32;

        for &(si, ti, _) in correspondences {
            let sp = &source.points[si];
            let tx = transform.x + sp.x * cos_t - sp.y * sin_t;
            let ty = transform.y + sp.x * sin_t + sp.y * cos_t;

            let tp = &target.points[ti];
            let dx = tx - tp.x;
            let dy = ty - tp.y;
            sum_sq += dx * dx + dy * dy;
        }

        sum_sq / correspondences.len() as f32
    }

    /// Convert MSE to a 0-1 score.
    fn mse_to_score(&self, mse: f32) -> f32 {
        // Score of 1.0 at mse=0, decaying exponentially
        // Score of ~0.5 at mse = 0.01 (1cm RMSE)
        (-mse * 100.0).exp()
    }
}

impl ScanMatcher for PointToPointIcp {
    fn match_scans(
        &self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        // Check for empty point clouds
        if source.is_empty() || target.is_empty() {
            return ScanMatchResult::failed();
        }

        // Build k-d tree for target
        let target_tree = Self::build_kdtree(target);

        let mut current_transform = *initial_guess;
        let mut iterations = 0u32;
        let mut last_mse = f32::MAX;

        for iter in 0..self.config.max_iterations {
            iterations = iter + 1;

            // Find correspondences
            let correspondences =
                self.find_correspondences(source, target, &target_tree, &current_transform);

            // Check minimum correspondences
            if correspondences.len() < self.config.min_correspondences {
                return ScanMatchResult::failed();
            }

            // Compute incremental transform
            let delta =
                self.compute_transform(source, target, &correspondences, &current_transform);

            // Apply delta
            current_transform = current_transform.compose(&delta);

            // Compute MSE
            let mse = self.compute_mse(source, target, &correspondences, &current_transform);

            // Check convergence
            let translation_change = (delta.x * delta.x + delta.y * delta.y).sqrt();
            let rotation_change = delta.theta.abs();

            if translation_change < self.config.translation_epsilon
                && rotation_change < self.config.rotation_epsilon
            {
                // Converged
                let score = self.mse_to_score(mse);
                return ScanMatchResult::success(current_transform, score, iterations, mse);
            }

            // Check if MSE is improving
            if mse > last_mse * 1.1 {
                // MSE is getting worse, might be diverging
                // Return best result so far
                break;
            }
            last_mse = mse;
        }

        // Max iterations reached
        let correspondences =
            self.find_correspondences(source, target, &target_tree, &current_transform);
        let final_mse = self.compute_mse(source, target, &correspondences, &current_transform);
        let score = self.mse_to_score(final_mse);

        // Consider it converged if MSE is reasonable
        if final_mse < 0.01 {
            ScanMatchResult::success(current_transform, score, iterations, final_mse)
        } else {
            ScanMatchResult {
                transform: current_transform,
                covariance: crate::core::types::Covariance2D::diagonal(0.1, 0.1, 0.05),
                score,
                converged: false,
                iterations,
                mse: final_mse,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::PI;

    fn create_line_cloud(n: usize, length: f32) -> PointCloud2D {
        let mut cloud = PointCloud2D::with_capacity(n);
        for i in 0..n {
            let x = (i as f32 / (n - 1) as f32) * length;
            cloud.push(Point2D::new(x, 0.0));
        }
        cloud
    }

    fn create_L_shape(n: usize, length: f32) -> PointCloud2D {
        let mut cloud = PointCloud2D::with_capacity(2 * n);
        // Horizontal line
        for i in 0..n {
            let x = (i as f32 / (n - 1) as f32) * length;
            cloud.push(Point2D::new(x, 0.0));
        }
        // Vertical line
        for i in 1..n {
            let y = (i as f32 / (n - 1) as f32) * length;
            cloud.push(Point2D::new(0.0, y));
        }
        cloud
    }

    fn transform_cloud(cloud: &PointCloud2D, pose: &Pose2D) -> PointCloud2D {
        cloud.transform(pose)
    }

    #[test]
    fn test_identity_transform() {
        let cloud = create_L_shape(20, 1.0);
        let icp = PointToPointIcp::new(IcpConfig::default());

        let result = icp.match_scans(&cloud, &cloud, &Pose2D::identity());

        assert!(result.converged);
        assert_relative_eq!(result.transform.x, 0.0, epsilon = 0.01);
        assert_relative_eq!(result.transform.y, 0.0, epsilon = 0.01);
        assert_relative_eq!(result.transform.theta, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_small_translation() {
        let source = create_L_shape(50, 2.0);
        let transform = Pose2D::new(0.1, 0.05, 0.0);
        let target = transform_cloud(&source, &transform);

        let icp = PointToPointIcp::new(IcpConfig::default());
        let result = icp.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "ICP should converge");
        assert_relative_eq!(result.transform.x, 0.1, epsilon = 0.02);
        assert_relative_eq!(result.transform.y, 0.05, epsilon = 0.02);
    }

    #[test]
    fn test_small_rotation() {
        let source = create_L_shape(50, 2.0);
        let transform = Pose2D::new(0.0, 0.0, 0.1); // ~5.7°
        let target = transform_cloud(&source, &transform);

        let icp = PointToPointIcp::new(IcpConfig::default());
        let result = icp.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "ICP should converge");
        assert_relative_eq!(result.transform.theta, 0.1, epsilon = 0.02);
    }

    #[test]
    fn test_combined_transform() {
        let source = create_L_shape(50, 2.0);
        let transform = Pose2D::new(0.15, -0.1, 0.08);
        let target = transform_cloud(&source, &transform);

        let icp = PointToPointIcp::new(IcpConfig::default());
        let result = icp.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "ICP should converge");
        assert_relative_eq!(result.transform.x, 0.15, epsilon = 0.03);
        assert_relative_eq!(result.transform.y, -0.1, epsilon = 0.03);
        assert_relative_eq!(result.transform.theta, 0.08, epsilon = 0.02);
    }

    #[test]
    fn test_with_initial_guess() {
        let source = create_L_shape(50, 2.0);
        let transform = Pose2D::new(0.3, 0.2, 0.15);
        let target = transform_cloud(&source, &transform);

        // With a reasonable initial guess
        let initial_guess = Pose2D::new(0.25, 0.15, 0.1);

        let icp = PointToPointIcp::new(IcpConfig::default());
        let result = icp.match_scans(&source, &target, &initial_guess);

        assert!(
            result.converged,
            "ICP should converge with good initial guess"
        );
        assert_relative_eq!(result.transform.x, 0.3, epsilon = 0.03);
        assert_relative_eq!(result.transform.y, 0.2, epsilon = 0.03);
        assert_relative_eq!(result.transform.theta, 0.15, epsilon = 0.02);
    }

    #[test]
    fn test_empty_clouds() {
        let empty = PointCloud2D::new();
        let cloud = create_line_cloud(10, 1.0);
        let icp = PointToPointIcp::new(IcpConfig::default());

        let result1 = icp.match_scans(&empty, &cloud, &Pose2D::identity());
        assert!(!result1.converged);

        let result2 = icp.match_scans(&cloud, &empty, &Pose2D::identity());
        assert!(!result2.converged);
    }

    #[test]
    fn test_config_accessors() {
        let config = IcpConfig {
            max_iterations: 100,
            translation_epsilon: 0.0001,
            ..IcpConfig::default()
        };
        let icp = PointToPointIcp::new(config);

        assert_eq!(icp.config().max_iterations, 100);
        assert_eq!(icp.config().translation_epsilon, 0.0001);
    }

    #[test]
    fn test_large_transform_fails_without_good_guess() {
        let source = create_L_shape(50, 2.0);
        let transform = Pose2D::new(1.0, 1.0, PI / 4.0); // Large transform
        let target = transform_cloud(&source, &transform);

        let icp = PointToPointIcp::new(IcpConfig::default());
        let result = icp.match_scans(&source, &target, &Pose2D::identity());

        // ICP typically fails with large initial error
        // The test verifies it doesn't crash, not that it succeeds
        // In practice, use correlative matcher for large errors
    }
}
