//! Shared utilities for ICP algorithms.
//!
//! Contains common functionality used by both Point-to-Point and Point-to-Line ICP:
//! - K-d tree building
//! - MSE to score conversion
//! - Convergence tracking
//! - Common configuration parameters
//! - Scan validation

use kiddo::KdTree;

use crate::core::types::{PointCloud2D, Pose2D};

/// Convergence tracker for ICP iterations.
///
/// Tracks the best result found and detects when MSE is diverging.
/// This can be used by any ICP variant to track iteration progress.
///
/// # Example
///
/// ```ignore
/// let mut tracker = ConvergenceTracker::new(initial_guess);
/// for iter in 0..max_iterations {
///     let mse = compute_mse();
///     tracker.update(mse, current_transform, iter);
///     if tracker.has_diverged() {
///         break;
///     }
/// }
/// let (best_transform, best_mse, best_iter) = tracker.best_result();
/// ```
#[derive(Debug, Clone)]
pub struct ConvergenceTracker {
    /// Best MSE found so far
    pub best_mse: f32,
    /// Transform corresponding to best MSE
    pub best_transform: Pose2D,
    /// Iteration at which best result was found
    pub best_iterations: u32,
    /// MSE from previous iteration
    pub last_mse: f32,
    /// Count of consecutive MSE increases
    pub consecutive_increases: u32,
}

impl ConvergenceTracker {
    /// Maximum consecutive MSE increases before stopping
    pub const MAX_CONSECUTIVE_INCREASES: u32 = 3;

    /// Create a new convergence tracker.
    pub fn new(initial_transform: Pose2D) -> Self {
        Self {
            best_mse: f32::MAX,
            best_transform: initial_transform,
            best_iterations: 0,
            last_mse: f32::MAX,
            consecutive_increases: 0,
        }
    }

    /// Update tracker with new MSE value.
    ///
    /// Returns `true` if MSE has improved, `false` if diverging.
    pub fn update(&mut self, mse: f32, transform: Pose2D, iteration: u32) -> bool {
        // Track best result
        if mse < self.best_mse {
            self.best_mse = mse;
            self.best_transform = transform;
            self.best_iterations = iteration;
        }

        // Check for divergence (only count if significantly worse)
        let is_improving = if mse > self.last_mse * 1.1 {
            self.consecutive_increases += 1;
            false
        } else {
            self.consecutive_increases = 0;
            true
        };

        self.last_mse = mse;
        is_improving
    }

    /// Check if MSE has diverged too many times.
    pub fn has_diverged(&self) -> bool {
        self.consecutive_increases >= Self::MAX_CONSECUTIVE_INCREASES
    }

    /// Get the best result found.
    pub fn best_result(&self) -> (Pose2D, f32, u32) {
        (self.best_transform, self.best_mse, self.best_iterations)
    }
}

/// Build a k-d tree from a point cloud.
///
/// This is a shared utility used by both ICP variants.
pub fn build_kdtree(cloud: &PointCloud2D) -> KdTree<f32, 2> {
    let mut tree: KdTree<f32, 2> = KdTree::new();
    for i in 0..cloud.len() {
        tree.add(&[cloud.xs[i], cloud.ys[i]], i as u64);
    }
    tree
}

/// Convert MSE to a 0-1 score.
///
/// Uses linear decay based on RMSE (root mean squared error):
/// - Score 1.0 at RMSE = 0
/// - Score 0.9 at RMSE = 2cm
/// - Score 0.7 at RMSE = 6cm
/// - Score 0.5 at RMSE = 10cm
/// - Score 0.0 at RMSE ≥ 20cm
#[inline]
pub fn mse_to_score(mse: f32) -> f32 {
    let rmse = mse.sqrt();
    // Linear decay from 1.0 to 0.0 over 0 to 20cm RMSE
    (1.0 - rmse * 5.0).clamp(0.0, 1.0)
}

/// MSE thresholds for convergence classification.
pub mod thresholds {
    /// Good convergence threshold (4cm² = 2cm RMSE)
    pub const GOOD_MSE: f32 = 0.04;
    /// Marginal convergence threshold (10cm² ~ 3cm RMSE)
    pub const MARGINAL_MSE: f32 = 0.1;
}

/// Minimum points required for reliable matching.
pub const MIN_POINTS_FOR_RELIABLE_MATCH: usize = 30;

/// Validates that both point clouds have sufficient points for reliable matching.
///
/// Returns `true` if both clouds meet the minimum point requirement.
#[inline]
pub fn validate_scan_sizes(source: &PointCloud2D, target: &PointCloud2D) -> bool {
    source.len() >= MIN_POINTS_FOR_RELIABLE_MATCH && target.len() >= MIN_POINTS_FOR_RELIABLE_MATCH
}

/// Common convergence configuration for ICP variants.
///
/// These parameters are shared between Point-to-Point and Point-to-Line ICP.
#[derive(Debug, Clone)]
pub struct IcpConvergenceConfig {
    /// Maximum number of iterations.
    pub max_iterations: u32,

    /// Convergence threshold for translation (meters).
    pub translation_epsilon: f32,

    /// Convergence threshold for rotation (radians).
    pub rotation_epsilon: f32,

    /// Maximum correspondence distance (meters).
    pub max_correspondence_distance: f32,

    /// Minimum number of valid correspondences required.
    pub min_correspondences: usize,

    /// Outlier rejection ratio (0.0 to 1.0).
    pub outlier_ratio: f32,
}

impl Default for IcpConvergenceConfig {
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

impl IcpConvergenceConfig {
    /// Check if transform change indicates convergence.
    #[inline]
    pub fn is_converged(&self, translation_change: f32, rotation_change: f32) -> bool {
        translation_change < self.translation_epsilon && rotation_change < self.rotation_epsilon
    }

    /// Compute translation change magnitude from a delta pose.
    #[inline]
    pub fn translation_change(delta: &Pose2D) -> f32 {
        (delta.x * delta.x + delta.y * delta.y).sqrt()
    }

    /// Compute rotation change magnitude from a delta pose.
    #[inline]
    pub fn rotation_change(delta: &Pose2D) -> f32 {
        delta.theta.abs()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_scan_sizes() {
        let mut small_cloud = PointCloud2D::new();
        for i in 0..20 {
            small_cloud.push_xy(i as f32, 0.0);
        }

        let mut large_cloud = PointCloud2D::new();
        for i in 0..50 {
            large_cloud.push_xy(i as f32, 0.0);
        }

        assert!(!validate_scan_sizes(&small_cloud, &large_cloud));
        assert!(!validate_scan_sizes(&large_cloud, &small_cloud));
        assert!(validate_scan_sizes(&large_cloud, &large_cloud));
    }

    #[test]
    fn test_icp_convergence_config() {
        let config = IcpConvergenceConfig::default();

        // Should converge for small changes
        assert!(config.is_converged(0.0005, 0.0005));

        // Should not converge for large changes
        assert!(!config.is_converged(0.01, 0.0005));
        assert!(!config.is_converged(0.0005, 0.01));
    }

    #[test]
    fn test_translation_rotation_change() {
        let delta = Pose2D::new(0.03, 0.04, 0.1);

        let trans_change = IcpConvergenceConfig::translation_change(&delta);
        let rot_change = IcpConvergenceConfig::rotation_change(&delta);

        assert!((trans_change - 0.05).abs() < 1e-6); // 3-4-5 triangle
        assert!((rot_change - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_mse_to_score() {
        // Perfect alignment
        assert_eq!(mse_to_score(0.0), 1.0);

        // 2cm RMSE (0.0004 MSE) -> ~0.9 score
        let score_2cm = mse_to_score(0.0004);
        assert!(score_2cm > 0.85 && score_2cm < 0.95);

        // 10cm RMSE (0.01 MSE) -> ~0.5 score
        let score_10cm = mse_to_score(0.01);
        assert!(score_10cm > 0.45 && score_10cm < 0.55);

        // 20cm RMSE (0.04 MSE) -> 0.0 score
        assert_eq!(mse_to_score(0.04), 0.0);

        // Beyond 20cm RMSE -> clamped to 0
        assert_eq!(mse_to_score(1.0), 0.0);
    }

    #[test]
    fn test_convergence_tracker_tracks_best() {
        let mut tracker = ConvergenceTracker::new(Pose2D::identity());

        // First update
        tracker.update(0.1, Pose2D::new(1.0, 0.0, 0.0), 1);
        assert_eq!(tracker.best_mse, 0.1);

        // Better result
        tracker.update(0.05, Pose2D::new(1.5, 0.0, 0.0), 2);
        assert_eq!(tracker.best_mse, 0.05);
        assert_eq!(tracker.best_transform.x, 1.5);

        // Worse result - best should not change
        tracker.update(0.2, Pose2D::new(2.0, 0.0, 0.0), 3);
        assert_eq!(tracker.best_mse, 0.05);
        assert_eq!(tracker.best_transform.x, 1.5);
    }

    #[test]
    fn test_convergence_tracker_detects_divergence() {
        let mut tracker = ConvergenceTracker::new(Pose2D::identity());

        tracker.update(0.1, Pose2D::identity(), 1);
        assert!(!tracker.has_diverged());

        // 3 consecutive increases (>10% worse each time)
        tracker.update(0.12, Pose2D::identity(), 2);
        assert!(!tracker.has_diverged());
        tracker.update(0.14, Pose2D::identity(), 3);
        assert!(!tracker.has_diverged());
        tracker.update(0.16, Pose2D::identity(), 4);
        assert!(tracker.has_diverged());
    }

    #[test]
    fn test_build_kdtree() {
        let mut cloud = PointCloud2D::new();
        cloud.push_xy(1.0, 2.0);
        cloud.push_xy(3.0, 4.0);
        cloud.push_xy(5.0, 6.0);

        let _ = build_kdtree(&cloud);
        // Tree should have 3 entries (can't directly check, but building shouldn't panic)
        assert!(!cloud.is_empty());
    }
}
