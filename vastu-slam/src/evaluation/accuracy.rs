//! Absolute and Relative Trajectory Error metrics.
//!
//! This module provides ATE (Absolute Trajectory Error) and RPE (Relative Pose Error)
//! metrics that are compatible with the [evo](https://github.com/MichaelGrupp/evo)
//! trajectory evaluation tool.
//!
//! ## When to Use
//!
//! - **Pose Relations** (primary): Use for self-evaluation without external ground truth
//! - **ATE/RPE** (secondary): Use for cross-validation when external ground truth is available
//!
//! ## Metrics
//!
//! - **ATE**: Global accuracy after SE(2) alignment. Measures overall trajectory quality.
//! - **RPE**: Local drift over fixed distances. Measures odometry consistency.

use crate::Pose2D;
use serde::{Deserialize, Serialize};

/// Container for all accuracy metrics.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AccuracyMetrics {
    /// Absolute Trajectory Error
    pub ate: AbsoluteTrajectoryError,

    /// Relative Pose Error
    pub rpe: RelativePoseError,
}

impl AccuracyMetrics {
    /// Compute all accuracy metrics.
    ///
    /// # Arguments
    ///
    /// * `estimated` - Estimated trajectory from SLAM
    /// * `ground_truth` - Ground truth trajectory (from simulation or external sensor)
    /// * `delta_m` - Distance delta for RPE computation (meters)
    pub fn compute(estimated: &[Pose2D], ground_truth: &[Pose2D], delta_m: f32) -> Self {
        Self {
            ate: AbsoluteTrajectoryError::compute(estimated, ground_truth),
            rpe: RelativePoseError::compute(estimated, ground_truth, delta_m),
        }
    }

    /// Print all metrics.
    pub fn print(&self) {
        self.ate.print();
        println!();
        self.rpe.print();
    }
}

/// Error statistics for trajectory comparison.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TrajectoryError {
    /// Root mean square error
    pub rmse: f32,

    /// Mean error
    pub mean: f32,

    /// Standard deviation
    pub std: f32,

    /// Minimum error
    pub min: f32,

    /// Maximum error
    pub max: f32,

    /// Median error
    pub median: f32,

    /// Number of samples
    pub count: usize,
}

impl TrajectoryError {
    /// Compute statistics from a list of errors.
    pub fn from_errors(errors: &[f32]) -> Self {
        if errors.is_empty() {
            return Self::default();
        }

        let count = errors.len();
        let n = count as f32;

        let sum: f32 = errors.iter().sum();
        let mean = sum / n;

        let sum_sq: f32 = errors.iter().map(|e| e * e).sum();
        let rmse = (sum_sq / n).sqrt();

        let variance = errors.iter().map(|e| (e - mean).powi(2)).sum::<f32>() / n;
        let std = variance.sqrt();

        let min = errors.iter().cloned().fold(f32::INFINITY, f32::min);
        let max = errors.iter().cloned().fold(f32::NEG_INFINITY, f32::max);

        let mut sorted = errors.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = if count.is_multiple_of(2) {
            (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0
        } else {
            sorted[count / 2]
        };

        Self {
            rmse,
            mean,
            std,
            min,
            max,
            median,
            count,
        }
    }

    /// Format as a single-line summary.
    pub fn summary(&self) -> String {
        format!(
            "rmse: {:.4}, mean: {:.4}, std: {:.4}, min: {:.4}, max: {:.4}",
            self.rmse, self.mean, self.std, self.min, self.max
        )
    }
}

/// Absolute Trajectory Error (ATE).
///
/// Measures the global accuracy of a trajectory by computing the RMSE of position
/// errors after SE(2) alignment (translation + rotation).
///
/// This is the standard metric used by evo's `evo_ape` command.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AbsoluteTrajectoryError {
    /// Translational error statistics (meters)
    pub translation: TrajectoryError,

    /// Rotational error statistics (radians)
    pub rotation: TrajectoryError,

    /// Whether SE(2) alignment was applied
    pub aligned: bool,
}

impl AbsoluteTrajectoryError {
    /// Compute ATE with SE(2) alignment.
    ///
    /// # Arguments
    ///
    /// * `estimated` - Estimated trajectory from SLAM
    /// * `ground_truth` - Ground truth trajectory
    ///
    /// # Returns
    ///
    /// ATE metrics after aligning trajectories.
    pub fn compute(estimated: &[Pose2D], ground_truth: &[Pose2D]) -> Self {
        Self::compute_with_options(estimated, ground_truth, true)
    }

    /// Compute ATE with optional alignment.
    pub fn compute_with_options(
        estimated: &[Pose2D],
        ground_truth: &[Pose2D],
        align: bool,
    ) -> Self {
        let n = estimated.len().min(ground_truth.len());
        if n == 0 {
            return Self::default();
        }

        // Optionally align trajectories
        let aligned_estimated = if align {
            align_trajectory_se2(estimated, ground_truth)
        } else {
            estimated.to_vec()
        };

        // Compute per-pose errors
        let mut trans_errors = Vec::with_capacity(n);
        let mut rot_errors = Vec::with_capacity(n);

        for i in 0..n {
            let est = &aligned_estimated[i];
            let gt = &ground_truth[i];

            let trans_error = est.distance(gt);
            let rot_error = (est.theta - gt.theta).abs();

            trans_errors.push(trans_error);
            rot_errors.push(rot_error);
        }

        Self {
            translation: TrajectoryError::from_errors(&trans_errors),
            rotation: TrajectoryError::from_errors(&rot_errors),
            aligned: align,
        }
    }

    /// Print ATE metrics.
    pub fn print(&self) {
        println!("=== Absolute Trajectory Error (ATE) ===");
        println!("Alignment: {}", if self.aligned { "SE(2)" } else { "None" });
        println!(
            "Translation RMSE: {:.6} m (mean: {:.6}, std: {:.6})",
            self.translation.rmse, self.translation.mean, self.translation.std
        );
        println!(
            "Rotation RMSE:    {:.6} rad (mean: {:.6}, std: {:.6})",
            self.rotation.rmse, self.rotation.mean, self.rotation.std
        );
    }
}

/// Relative Pose Error (RPE).
///
/// Measures local drift by comparing relative poses over fixed distances.
/// This is useful for evaluating odometry quality independently of global drift.
///
/// This is the standard metric used by evo's `evo_rpe` command.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RelativePoseError {
    /// Translational error statistics (meters)
    pub translation: TrajectoryError,

    /// Rotational error statistics (radians)
    pub rotation: TrajectoryError,

    /// Distance delta used for computation (meters)
    pub delta_m: f32,

    /// Number of pairs evaluated
    pub pairs_evaluated: usize,
}

impl RelativePoseError {
    /// Compute RPE with a fixed distance delta.
    ///
    /// # Arguments
    ///
    /// * `estimated` - Estimated trajectory from SLAM
    /// * `ground_truth` - Ground truth trajectory
    /// * `delta_m` - Distance between pose pairs (meters)
    ///
    /// # Returns
    ///
    /// RPE metrics.
    pub fn compute(estimated: &[Pose2D], ground_truth: &[Pose2D], delta_m: f32) -> Self {
        let n = estimated.len().min(ground_truth.len());
        if n < 2 || delta_m <= 0.0 {
            return Self {
                delta_m,
                ..Default::default()
            };
        }

        // Compute cumulative distances in ground truth
        let mut cumulative_dist = vec![0.0f32; n];
        for i in 1..n {
            cumulative_dist[i] =
                cumulative_dist[i - 1] + ground_truth[i].distance(&ground_truth[i - 1]);
        }

        let mut trans_errors = Vec::new();
        let mut rot_errors = Vec::new();

        // For each pose, find the next pose that's delta_m away
        for i in 0..n {
            // Binary search for pose at distance delta_m
            let target_dist = cumulative_dist[i] + delta_m;

            // Find j such that cumulative_dist[j] >= target_dist
            let j = match cumulative_dist[i..].binary_search_by(|d| {
                d.partial_cmp(&target_dist)
                    .unwrap_or(std::cmp::Ordering::Less)
            }) {
                Ok(idx) => i + idx,
                Err(idx) => {
                    if i + idx >= n {
                        continue;
                    }
                    i + idx
                }
            };

            if j >= n {
                continue;
            }

            // Compute relative poses
            let est_rel = estimated[i].inverse().compose(&estimated[j]);
            let gt_rel = ground_truth[i].inverse().compose(&ground_truth[j]);

            // Error in relative pose
            let error_rel = gt_rel.inverse().compose(&est_rel);

            trans_errors.push(error_rel.position().length());
            rot_errors.push(error_rel.theta.abs());
        }

        Self {
            translation: TrajectoryError::from_errors(&trans_errors),
            rotation: TrajectoryError::from_errors(&rot_errors),
            delta_m,
            pairs_evaluated: trans_errors.len(),
        }
    }

    /// Print RPE metrics.
    pub fn print(&self) {
        println!("=== Relative Pose Error (RPE) ===");
        println!("Delta: {} m, Pairs: {}", self.delta_m, self.pairs_evaluated);
        println!(
            "Translation RMSE: {:.6} m (mean: {:.6}, std: {:.6})",
            self.translation.rmse, self.translation.mean, self.translation.std
        );
        println!(
            "Rotation RMSE:    {:.6} rad (mean: {:.6}, std: {:.6})",
            self.rotation.rmse, self.rotation.mean, self.rotation.std
        );
    }
}

/// Align a trajectory to a reference using SE(2) transformation.
///
/// Finds the optimal translation and rotation to minimize position error.
/// Uses Horn's method for 2D point cloud registration.
///
/// # Arguments
///
/// * `estimated` - Trajectory to align
/// * `reference` - Reference trajectory (ground truth)
///
/// # Returns
///
/// Aligned trajectory.
pub fn align_trajectory_se2(estimated: &[Pose2D], reference: &[Pose2D]) -> Vec<Pose2D> {
    let n = estimated.len().min(reference.len());
    if n == 0 {
        return Vec::new();
    }

    // Compute centroids
    let est_centroid = compute_centroid(estimated);
    let ref_centroid = compute_centroid(reference);

    // Center the point clouds
    let est_centered: Vec<(f32, f32)> = estimated
        .iter()
        .take(n)
        .map(|p| (p.x - est_centroid.0, p.y - est_centroid.1))
        .collect();
    let ref_centered: Vec<(f32, f32)> = reference
        .iter()
        .take(n)
        .map(|p| (p.x - ref_centroid.0, p.y - ref_centroid.1))
        .collect();

    // Compute optimal rotation using SVD-like approach for 2D
    // For 2D, we can compute this directly without full SVD
    let mut sxx = 0.0f32;
    let mut sxy = 0.0f32;
    let mut syx = 0.0f32;
    let mut syy = 0.0f32;

    for i in 0..n {
        sxx += est_centered[i].0 * ref_centered[i].0;
        sxy += est_centered[i].0 * ref_centered[i].1;
        syx += est_centered[i].1 * ref_centered[i].0;
        syy += est_centered[i].1 * ref_centered[i].1;
    }

    // Optimal rotation angle
    let theta = (sxy - syx).atan2(sxx + syy);
    let cos_t = theta.cos();
    let sin_t = theta.sin();

    // Compute translation
    let tx = ref_centroid.0 - (cos_t * est_centroid.0 - sin_t * est_centroid.1);
    let ty = ref_centroid.1 - (sin_t * est_centroid.0 + cos_t * est_centroid.1);

    // Apply transformation
    estimated
        .iter()
        .map(|p| {
            Pose2D::new(
                cos_t * p.x - sin_t * p.y + tx,
                sin_t * p.x + cos_t * p.y + ty,
                p.theta + theta,
            )
        })
        .collect()
}

fn compute_centroid(poses: &[Pose2D]) -> (f32, f32) {
    if poses.is_empty() {
        return (0.0, 0.0);
    }
    let n = poses.len() as f32;
    let sum_x: f32 = poses.iter().map(|p| p.x).sum();
    let sum_y: f32 = poses.iter().map(|p| p.y).sum();
    (sum_x / n, sum_y / n)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_4;

    fn create_straight_trajectory(n: usize, spacing: f32) -> Vec<Pose2D> {
        (0..n)
            .map(|i| Pose2D::new(i as f32 * spacing, 0.0, 0.0))
            .collect()
    }

    #[test]
    fn test_trajectory_error_stats() {
        let errors = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let stats = TrajectoryError::from_errors(&errors);

        assert_eq!(stats.count, 5);
        assert!((stats.mean - 3.0).abs() < 1e-5);
        assert!((stats.min - 1.0).abs() < 1e-5);
        assert!((stats.max - 5.0).abs() < 1e-5);
        assert!((stats.median - 3.0).abs() < 1e-5);
    }

    #[test]
    fn test_ate_perfect_trajectory() {
        let trajectory = create_straight_trajectory(10, 0.5);

        let ate = AbsoluteTrajectoryError::compute(&trajectory, &trajectory);

        assert!(ate.translation.rmse < 1e-5);
        assert!(ate.rotation.rmse < 1e-5);
    }

    #[test]
    fn test_ate_with_offset() {
        let ground_truth = create_straight_trajectory(10, 0.5);

        // Estimated with constant offset
        let estimated: Vec<Pose2D> = ground_truth
            .iter()
            .map(|p| Pose2D::new(p.x + 0.1, p.y + 0.1, p.theta))
            .collect();

        // Without alignment
        let ate_no_align =
            AbsoluteTrajectoryError::compute_with_options(&estimated, &ground_truth, false);
        assert!((ate_no_align.translation.rmse - 0.1414).abs() < 0.01);

        // With alignment, error should be near zero
        let ate_aligned = AbsoluteTrajectoryError::compute(&estimated, &ground_truth);
        assert!(ate_aligned.translation.rmse < 0.01);
    }

    #[test]
    fn test_rpe_perfect_trajectory() {
        let trajectory = create_straight_trajectory(10, 0.5);

        let rpe = RelativePoseError::compute(&trajectory, &trajectory, 1.0);

        assert!(rpe.translation.rmse < 1e-5);
        assert!(rpe.rotation.rmse < 1e-5);
        assert!(rpe.pairs_evaluated > 0);
    }

    #[test]
    fn test_align_trajectory() {
        let reference = create_straight_trajectory(5, 1.0);

        // Estimated with rotation and translation
        let estimated: Vec<Pose2D> = reference
            .iter()
            .map(|p| {
                let cos_t = FRAC_PI_4.cos();
                let sin_t = FRAC_PI_4.sin();
                Pose2D::new(
                    cos_t * p.x - sin_t * p.y + 1.0,
                    sin_t * p.x + cos_t * p.y + 2.0,
                    p.theta + FRAC_PI_4,
                )
            })
            .collect();

        let aligned = align_trajectory_se2(&estimated, &reference);

        // After alignment, should be close to reference
        for (a, r) in aligned.iter().zip(reference.iter()) {
            assert!(a.distance(r) < 0.01);
        }
    }

    #[test]
    fn test_accuracy_metrics_compute() {
        let trajectory = create_straight_trajectory(10, 0.5);
        let metrics = AccuracyMetrics::compute(&trajectory, &trajectory, 1.0);

        assert!(metrics.ate.translation.rmse < 1e-5);
        assert!(metrics.rpe.translation.rmse < 1e-5);
    }
}
