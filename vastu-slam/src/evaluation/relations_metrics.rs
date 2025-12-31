//! Pose relations metrics for Cartographer-style evaluation.
//!
//! This module provides the core accuracy evaluation following Google Cartographer's
//! `compute_relations_metrics` approach.
//!
//! ## Concept
//!
//! For each pose relation in the ground truth, we:
//! 1. Look up the corresponding poses in the test trajectory
//! 2. Compute the relative pose from the test trajectory
//! 3. Compare against the ground truth relative pose
//! 4. Aggregate errors into statistics
//!
//! This approach evaluates trajectory **consistency** rather than absolute accuracy,
//! which works without external ground truth.

use super::ground_truth::{GroundTruthRelations, PoseRelation};
use crate::Pose2D;
use crate::core::normalize_angle;
use serde::{Deserialize, Serialize};

/// Error for a single pose relation.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RelationError {
    /// Translational error squared (meters²).
    pub translational_squared: f32,

    /// Rotational error squared (radians²).
    pub rotational_squared: f32,

    /// Covered distance for this relation (for weighting).
    pub covered_distance: f32,
}

impl RelationError {
    /// Compute the error between a ground truth relation and test trajectory.
    pub fn compute(ground_truth: &PoseRelation, test_trajectory: &[Pose2D]) -> Option<Self> {
        let from_idx = ground_truth.from_node;
        let to_idx = ground_truth.to_node;

        if from_idx >= test_trajectory.len() || to_idx >= test_trajectory.len() {
            return None;
        }

        let test_from = &test_trajectory[from_idx];
        let test_to = &test_trajectory[to_idx];

        // Compute relative pose in test trajectory
        let test_relative = test_from.inverse().compose(test_to);

        // Compare with ground truth relative pose
        let trans_error_x = test_relative.x - ground_truth.relative_pose.x;
        let trans_error_y = test_relative.y - ground_truth.relative_pose.y;
        let rot_error = normalize_angle(test_relative.theta - ground_truth.relative_pose.theta);

        Some(Self {
            translational_squared: trans_error_x * trans_error_x + trans_error_y * trans_error_y,
            rotational_squared: rot_error * rot_error,
            covered_distance: ground_truth.covered_distance,
        })
    }

    /// Get translational error in meters.
    pub fn translational(&self) -> f32 {
        self.translational_squared.sqrt()
    }

    /// Get rotational error in radians.
    pub fn rotational(&self) -> f32 {
        self.rotational_squared.sqrt()
    }
}

/// Aggregated metrics from pose relations (Cartographer output format).
///
/// This struct contains the same metrics that Cartographer's
/// `compute_relations_metrics` outputs.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RelationsMetrics {
    // === Translational error ===
    /// Mean absolute translational error (meters).
    pub abs_trans_error: f32,

    /// Standard deviation of translational error (meters).
    pub abs_trans_std: f32,

    /// Mean squared translational error (m²).
    pub sqr_trans_error: f32,

    /// Standard deviation of squared translational error (m²).
    pub sqr_trans_std: f32,

    // === Rotational error ===
    /// Mean absolute rotational error (degrees).
    pub abs_rot_error: f32,

    /// Standard deviation of rotational error (degrees).
    pub abs_rot_std: f32,

    /// Mean squared rotational error (deg²).
    pub sqr_rot_error: f32,

    /// Standard deviation of squared rotational error (deg²).
    pub sqr_rot_std: f32,

    // === Summary ===
    /// Number of relations evaluated.
    pub num_relations: usize,

    /// Number of outliers (errors exceeding threshold).
    pub num_outliers: usize,

    /// Whether the metrics pass the target threshold.
    pub passed_threshold: bool,

    /// Threshold used for pass/fail (meters).
    pub threshold_meters: f32,
}

impl RelationsMetrics {
    /// Compute metrics by comparing test trajectory to ground truth relations.
    ///
    /// This is equivalent to Cartographer's `compute_relations_metrics`.
    ///
    /// # Arguments
    ///
    /// * `test_trajectory` - The trajectory to evaluate
    /// * `ground_truth` - Ground truth relations from an optimized trajectory
    ///
    /// # Returns
    ///
    /// Aggregated accuracy metrics.
    pub fn compute(test_trajectory: &[Pose2D], ground_truth: &GroundTruthRelations) -> Self {
        Self::compute_with_threshold(test_trajectory, ground_truth, 0.01) // 1cm default
    }

    /// Compute metrics with a custom pass/fail threshold.
    pub fn compute_with_threshold(
        test_trajectory: &[Pose2D],
        ground_truth: &GroundTruthRelations,
        threshold_meters: f32,
    ) -> Self {
        if ground_truth.is_empty() {
            return Self {
                threshold_meters,
                passed_threshold: true,
                ..Default::default()
            };
        }

        // Compute errors for all relations
        let errors: Vec<RelationError> = ground_truth
            .relations
            .iter()
            .filter_map(|rel| RelationError::compute(rel, test_trajectory))
            .collect();

        if errors.is_empty() {
            return Self {
                threshold_meters,
                passed_threshold: true,
                ..Default::default()
            };
        }

        let n = errors.len() as f32;

        // Compute translational statistics
        let trans_errors: Vec<f32> = errors.iter().map(|e| e.translational()).collect();
        let sqr_trans: Vec<f32> = errors.iter().map(|e| e.translational_squared).collect();

        let abs_trans_error = trans_errors.iter().sum::<f32>() / n;
        let sqr_trans_error = sqr_trans.iter().sum::<f32>() / n;

        let abs_trans_std = std_dev(&trans_errors, abs_trans_error);
        let sqr_trans_std = std_dev(&sqr_trans, sqr_trans_error);

        // Compute rotational statistics (convert to degrees)
        let rot_errors: Vec<f32> = errors.iter().map(|e| e.rotational().to_degrees()).collect();
        let sqr_rot: Vec<f32> = errors
            .iter()
            .map(|e| e.rotational_squared.to_degrees().powi(2))
            .collect();

        let abs_rot_error = rot_errors.iter().sum::<f32>() / n;
        let sqr_rot_error = sqr_rot.iter().sum::<f32>() / n;

        let abs_rot_std = std_dev(&rot_errors, abs_rot_error);
        let sqr_rot_std = std_dev(&sqr_rot, sqr_rot_error);

        // Count outliers
        let num_outliers = trans_errors
            .iter()
            .filter(|&&e| e > ground_truth.config.outlier_threshold_meters)
            .count();

        Self {
            abs_trans_error,
            abs_trans_std,
            sqr_trans_error,
            sqr_trans_std,
            abs_rot_error,
            abs_rot_std,
            sqr_rot_error,
            sqr_rot_std,
            num_relations: errors.len(),
            num_outliers,
            passed_threshold: abs_trans_error < threshold_meters,
            threshold_meters,
        }
    }

    /// Pretty print metrics in Cartographer format.
    pub fn print(&self) {
        println!("=== Pose Relations Metrics ===");
        println!(
            "Abs translational error {:.5} +/- {:.5} m",
            self.abs_trans_error, self.abs_trans_std
        );
        println!(
            "Sqr translational error {:.5} +/- {:.5} m²",
            self.sqr_trans_error, self.sqr_trans_std
        );
        println!(
            "Abs rotational error    {:.5} +/- {:.5} deg",
            self.abs_rot_error, self.abs_rot_std
        );
        println!(
            "Sqr rotational error    {:.5} +/- {:.5} deg²",
            self.sqr_rot_error, self.sqr_rot_std
        );
        println!("Relations evaluated: {}", self.num_relations);
        println!("Outliers: {}", self.num_outliers);
        println!(
            "Status: {} (threshold: {}m)",
            if self.passed_threshold {
                "PASSED"
            } else {
                "FAILED"
            },
            self.threshold_meters
        );
    }

    /// Format as a single-line summary.
    pub fn summary(&self) -> String {
        format!(
            "trans: {:.4}±{:.4}m, rot: {:.2}±{:.2}°, {} relations, {}",
            self.abs_trans_error,
            self.abs_trans_std,
            self.abs_rot_error,
            self.abs_rot_std,
            self.num_relations,
            if self.passed_threshold {
                "PASS"
            } else {
                "FAIL"
            }
        )
    }
}

/// Compute standard deviation given values and their mean.
fn std_dev(values: &[f32], mean: f32) -> f32 {
    if values.len() < 2 {
        return 0.0;
    }

    let variance =
        values.iter().map(|v| (v - mean).powi(2)).sum::<f32>() / (values.len() - 1) as f32;
    variance.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::evaluation::ground_truth::GroundTruthConfig;

    fn create_straight_trajectory(n: usize, spacing: f32) -> Vec<Pose2D> {
        (0..n)
            .map(|i| Pose2D::new(i as f32 * spacing, 0.0, 0.0))
            .collect()
    }

    #[test]
    fn test_perfect_trajectory() {
        let trajectory = create_straight_trajectory(10, 0.5);

        let config = GroundTruthConfig {
            min_covered_distance: 0.5,
            ..Default::default()
        };
        let ground_truth = GroundTruthRelations::generate(&trajectory, &[], config);

        // Same trajectory should have zero error
        let metrics = RelationsMetrics::compute(&trajectory, &ground_truth);

        assert!(metrics.abs_trans_error < 1e-5);
        assert!(metrics.abs_rot_error < 1e-5);
        assert!(metrics.passed_threshold);
    }

    #[test]
    fn test_trajectory_with_drift() {
        let ground_trajectory = create_straight_trajectory(10, 0.5);

        let config = GroundTruthConfig {
            min_covered_distance: 0.5,
            ..Default::default()
        };
        let ground_truth = GroundTruthRelations::generate(&ground_trajectory, &[], config);

        // Test trajectory with small drift
        let test_trajectory: Vec<Pose2D> = ground_trajectory
            .iter()
            .enumerate()
            .map(|(i, p)| Pose2D::new(p.x + 0.001 * i as f32, p.y, p.theta))
            .collect();

        let metrics = RelationsMetrics::compute(&test_trajectory, &ground_truth);

        // Should have small but non-zero error
        assert!(metrics.abs_trans_error > 0.0);
        assert!(metrics.abs_trans_error < 0.01); // Less than 1cm
    }

    #[test]
    fn test_relation_error_computation() {
        let from_pose = Pose2D::new(0.0, 0.0, 0.0);
        let to_pose = Pose2D::new(1.0, 0.0, 0.0);

        let relation = PoseRelation::from_poses(0, 1, &from_pose, &to_pose, 1.0);

        // Perfect match
        let test_trajectory = vec![from_pose, to_pose];
        let error = RelationError::compute(&relation, &test_trajectory).unwrap();

        assert!(error.translational() < 1e-5);
        assert!(error.rotational() < 1e-5);
    }

    #[test]
    fn test_std_dev() {
        let values = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let mean = 3.0;
        let std = std_dev(&values, mean);

        // Standard deviation should be sqrt(2.5) ≈ 1.58
        assert!((std - 1.5811).abs() < 0.01);
    }

    #[test]
    fn test_empty_ground_truth() {
        let trajectory = create_straight_trajectory(10, 0.5);

        let config = GroundTruthConfig {
            min_covered_distance: 100.0, // Very high, so no relations
            ..Default::default()
        };
        let ground_truth = GroundTruthRelations::generate(&trajectory, &[], config);

        let metrics = RelationsMetrics::compute(&trajectory, &ground_truth);

        assert_eq!(metrics.num_relations, 0);
        assert!(metrics.passed_threshold);
    }
}
