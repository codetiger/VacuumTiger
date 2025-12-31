//! Accuracy integration tests for VastuSLAM.
//!
//! These tests verify SLAM accuracy using the Cartographer-style pose relations metric.

mod common;

use vastu_slam::Pose2D;
use vastu_slam::evaluation::{
    AbsoluteTrajectoryError, AccuracyMetrics, GroundTruthConfig, GroundTruthRelations,
    RelationsMetrics, RelativePoseError,
};

/// Target accuracy: < 1cm translational error.
const TARGET_TRANS_ERROR_M: f32 = 0.01;

/// Target accuracy: < 0.5 degrees rotational error.
const TARGET_ROT_ERROR_DEG: f32 = 0.5;

// ============================================================================
// Pose Relations Metric Tests (Cartographer-style)
// ============================================================================

#[test]
fn test_relations_metric_perfect_trajectory() {
    // Ground truth: straight trajectory
    let trajectory = common::straight_trajectory(20, 0.5);

    let config = GroundTruthConfig {
        min_covered_distance: 0.5,
        node_sampling_interval: 1,
        ..Default::default()
    };

    let ground_truth = GroundTruthRelations::generate(&trajectory, &[], config);

    // Test same trajectory - should have zero error
    let metrics = RelationsMetrics::compute(&trajectory, &ground_truth);

    assert!(
        metrics.abs_trans_error < 1e-5,
        "Perfect trajectory should have zero translational error, got: {}",
        metrics.abs_trans_error
    );
    assert!(
        metrics.abs_rot_error < 1e-5,
        "Perfect trajectory should have zero rotational error, got: {}",
        metrics.abs_rot_error
    );
    assert!(metrics.passed_threshold);
}

#[test]
fn test_relations_metric_with_noise() {
    let ground_trajectory = common::straight_trajectory(50, 0.2);

    let config = GroundTruthConfig {
        min_covered_distance: 0.5,
        ..Default::default()
    };

    let ground_truth = GroundTruthRelations::generate(&ground_trajectory, &[], config);

    // Add small noise (< 1mm)
    let noisy_trajectory = common::add_noise(&ground_trajectory, 0.0005, 0.001, 42);
    let metrics = RelationsMetrics::compute(&noisy_trajectory, &ground_truth);

    println!("Noisy trajectory metrics: {}", metrics.summary());

    // Should still be under target with small noise
    assert!(
        metrics.abs_trans_error < TARGET_TRANS_ERROR_M,
        "Trans error {} should be < {}m",
        metrics.abs_trans_error,
        TARGET_TRANS_ERROR_M
    );
}

#[test]
fn test_relations_metric_with_drift() {
    let ground_trajectory = common::straight_trajectory(50, 0.2);

    let config = GroundTruthConfig {
        min_covered_distance: 0.5,
        ..Default::default()
    };

    let ground_truth = GroundTruthRelations::generate(&ground_trajectory, &[], config);

    // Add drift (1mm per meter traveled)
    let drifted_trajectory = common::add_drift(&ground_trajectory, 0.001);
    let metrics = RelationsMetrics::compute(&drifted_trajectory, &ground_truth);

    println!("Drifted trajectory metrics: {}", metrics.summary());

    // With 1mm/m drift over 10m, expect about 5mm average error
    assert!(
        metrics.abs_trans_error < 0.01,
        "Trans error {} with small drift should be < 1cm",
        metrics.abs_trans_error
    );
}

#[test]
fn test_relations_metric_square_loop() {
    // Square trajectory with loop closure
    let trajectory = common::square_trajectory(4.0, 10);

    // Mark loop closure between first and last poses
    let loop_closures = vec![(0, trajectory.len() - 1)];

    let config = GroundTruthConfig {
        min_covered_distance: 0.5,
        ..Default::default()
    };

    let ground_truth = GroundTruthRelations::generate(&trajectory, &loop_closures, config);

    // Verify ground truth contains loop closure relation
    assert!(
        ground_truth
            .relations
            .iter()
            .any(|r| r.covered_distance > 10.0),
        "Should have at least one long-distance relation from loop closure"
    );

    // Test with perfect trajectory
    let metrics = RelationsMetrics::compute(&trajectory, &ground_truth);
    assert!(metrics.passed_threshold);
}

// ============================================================================
// ATE/RPE Tests (evo-compatible)
// ============================================================================

#[test]
fn test_ate_perfect_trajectory() {
    let trajectory = common::straight_trajectory(20, 0.5);

    let ate = AbsoluteTrajectoryError::compute(&trajectory, &trajectory);

    assert!(
        ate.translation.rmse < 1e-5,
        "Perfect trajectory should have zero ATE"
    );
}

#[test]
fn test_ate_with_constant_offset() {
    let ground_truth = common::straight_trajectory(20, 0.5);

    // Add constant offset
    let estimated: Vec<Pose2D> = ground_truth
        .iter()
        .map(|p| Pose2D::new(p.x + 0.5, p.y + 0.3, p.theta))
        .collect();

    // Without alignment, should have significant error
    let ate_no_align =
        AbsoluteTrajectoryError::compute_with_options(&estimated, &ground_truth, false);
    assert!(
        ate_no_align.translation.rmse > 0.5,
        "Without alignment, should have large error"
    );

    // With alignment, should be near zero
    let ate_aligned = AbsoluteTrajectoryError::compute(&estimated, &ground_truth);
    assert!(
        ate_aligned.translation.rmse < 0.01,
        "With SE(2) alignment, constant offset should be corrected"
    );
}

#[test]
fn test_rpe_with_local_errors() {
    let ground_truth = common::straight_trajectory(50, 0.2);

    // Add noise to simulate local errors
    let estimated = common::add_noise(&ground_truth, 0.002, 0.005, 123);

    let rpe = RelativePoseError::compute(&estimated, &ground_truth, 1.0);

    println!(
        "RPE with noise: trans={:.4}m, rot={:.4}rad, pairs={}",
        rpe.translation.rmse, rpe.rotation.rmse, rpe.pairs_evaluated
    );

    // RPE should detect local errors
    assert!(rpe.pairs_evaluated > 0, "Should evaluate some pairs");
    assert!(
        rpe.translation.rmse < 0.01,
        "With 2mm noise, RPE should be small"
    );
}

#[test]
fn test_accuracy_metrics_all() {
    let ground_truth = common::figure_eight_trajectory(2.0, 40);
    let estimated = common::add_noise(&ground_truth, 0.001, 0.002, 456);

    let metrics = AccuracyMetrics::compute(&estimated, &ground_truth, 1.0);

    println!("=== Full Accuracy Report ===");
    metrics.print();

    assert!(
        metrics.ate.translation.rmse < 0.02,
        "ATE should be small with minor noise"
    );
}

// ============================================================================
// Regression Tests
// ============================================================================

#[test]
fn test_accuracy_regression_straight_line() {
    // This test ensures accuracy doesn't regress over time
    let ground_truth = common::straight_trajectory(100, 0.1);

    let config = GroundTruthConfig::default();
    let gt_relations = GroundTruthRelations::generate(&ground_truth, &[], config);

    // Simulate a reasonable SLAM output (small noise + small drift)
    let estimated = common::add_drift(
        &common::add_noise(&ground_truth, 0.001, 0.001, 789),
        0.0005, // 0.5mm/m drift
    );

    let metrics =
        RelationsMetrics::compute_with_threshold(&estimated, &gt_relations, TARGET_TRANS_ERROR_M);

    println!(
        "Regression test - Trans error: {:.4}m (target: <{}m)",
        metrics.abs_trans_error, TARGET_TRANS_ERROR_M
    );
    println!(
        "Regression test - Rot error: {:.4}deg (target: <{}deg)",
        metrics.abs_rot_error, TARGET_ROT_ERROR_DEG
    );

    assert!(
        metrics.passed_threshold,
        "REGRESSION: Trans error {} exceeds target {}m",
        metrics.abs_trans_error, TARGET_TRANS_ERROR_M
    );
}

#[test]
fn test_accuracy_regression_square_loop() {
    let ground_truth = common::square_trajectory(5.0, 20);
    let loop_closures = vec![(0, ground_truth.len() - 1)];

    let config = GroundTruthConfig::default();
    let gt_relations = GroundTruthRelations::generate(&ground_truth, &loop_closures, config);

    // Simulate SLAM with small errors (1mm position noise)
    let estimated = common::add_noise(&ground_truth, 0.001, 0.002, 101);

    let metrics =
        RelationsMetrics::compute_with_threshold(&estimated, &gt_relations, TARGET_TRANS_ERROR_M);

    println!(
        "Square loop - Trans error: {:.4}m, Rot error: {:.4}deg",
        metrics.abs_trans_error, metrics.abs_rot_error
    );

    assert!(
        metrics.passed_threshold,
        "REGRESSION: Square loop trans error {} exceeds target",
        metrics.abs_trans_error
    );
}
