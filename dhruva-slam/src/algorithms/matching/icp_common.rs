//! Shared utilities for ICP algorithms.
//!
//! Contains common functionality used by both Point-to-Point and Point-to-Line ICP:
//! - K-d tree building
//! - MSE to score conversion
//!
//! Note: Some utilities are defined for future ICP variants.

use kiddo::KdTree;

use crate::core::types::{PointCloud2D, Pose2D};

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

/// Configuration for identity snapping in ICP.
///
/// Identity snapping prevents drift in static/slow-motion scenarios where the
/// scan matcher might introduce small errors even when the robot hasn't moved.
#[derive(Debug, Clone, Copy)]
pub struct IdentitySnapConfig {
    /// Minimum match score to consider snapping (0.0-1.0).
    pub min_score: f32,
    /// Maximum translation to consider "near identity" (meters).
    pub max_translation: f32,
    /// Maximum rotation to consider "near identity" (radians).
    pub max_rotation: f32,
}

impl Default for IdentitySnapConfig {
    fn default() -> Self {
        Self {
            min_score: 0.95,
            max_translation: 0.03, // 3cm - typical encoder noise
            max_rotation: 0.05,    // ~3° - typical gyro/P2L drift
        }
    }
}

/// Potentially snap a transform to the initial guess if both are near identity.
///
/// This prevents quantization drift in static/slow-motion scenarios where
/// the scan matcher might introduce small rotation errors (aperture problem).
///
/// # Arguments
/// * `score` - Match quality score (0.0-1.0)
/// * `initial_guess` - The initial transform estimate (e.g., from odometry)
/// * `result_transform` - The computed transform from scan matching
/// * `config` - Identity snap configuration
///
/// # Returns
/// Either the original `result_transform` or `initial_guess` if snapping is triggered.
#[inline]
pub fn maybe_snap_to_identity(
    score: f32,
    initial_guess: &Pose2D,
    result_transform: &Pose2D,
    config: &IdentitySnapConfig,
) -> Pose2D {
    if score < config.min_score {
        return *result_transform;
    }

    let init_trans = (initial_guess.x.powi(2) + initial_guess.y.powi(2)).sqrt();
    let init_rot = initial_guess.theta.abs();
    let result_trans = (result_transform.x.powi(2) + result_transform.y.powi(2)).sqrt();
    let result_rot = result_transform.theta.abs();

    // Snap to initial guess if both transforms are near-identity
    if init_trans < config.max_translation
        && init_rot < config.max_rotation
        && result_trans < config.max_translation
        && result_rot < config.max_rotation
    {
        *initial_guess
    } else {
        *result_transform
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
