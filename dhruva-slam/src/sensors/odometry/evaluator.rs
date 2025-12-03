//! Odometry Evaluator for drift metrics.
//!
//! Computes odometry accuracy metrics by comparing estimated poses
//! against ground truth or by analyzing return-to-origin error.
//!
//! # Metrics
//!
//! - **Relative Position Error (RPE)**: Error per meter traveled
//! - **Relative Heading Error (RHE)**: Heading error per radian turned
//! - **Closure Error**: Error when returning to start position
//! - **Drift Rate**: Accumulated error over distance
//!
//! # Use Cases
//!
//! - Calibration: Find optimal encoder/gyro parameters
//! - Validation: Verify odometry accuracy meets requirements
//! - Comparison: Compare different fusion algorithms

use crate::core::types::{Pose2D, Timestamped};

/// Statistics for a set of measurements.
#[derive(Debug, Clone, Copy, Default)]
pub struct Stats {
    /// Number of samples
    pub count: usize,
    /// Mean value
    pub mean: f32,
    /// Standard deviation
    pub std_dev: f32,
    /// Minimum value
    pub min: f32,
    /// Maximum value
    pub max: f32,
}

impl Stats {
    /// Compute statistics from a slice of values.
    pub fn from_slice(values: &[f32]) -> Self {
        if values.is_empty() {
            return Self::default();
        }

        let count = values.len();
        let sum: f32 = values.iter().sum();
        let mean = sum / count as f32;

        let variance: f32 = values.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / count as f32;
        let std_dev = variance.sqrt();

        let min = values.iter().cloned().fold(f32::MAX, f32::min);
        let max = values.iter().cloned().fold(f32::MIN, f32::max);

        Self {
            count,
            mean,
            std_dev,
            min,
            max,
        }
    }
}

/// Results from odometry evaluation.
#[derive(Debug, Clone, Default)]
pub struct EvaluationResult {
    /// Total distance traveled (meters)
    pub total_distance: f32,
    /// Total rotation (radians, absolute sum)
    pub total_rotation: f32,
    /// Position error statistics (meters)
    pub position_error: Stats,
    /// Heading error statistics (radians)
    pub heading_error: Stats,
    /// Position drift rate (meters error per meter traveled)
    pub position_drift_rate: f32,
    /// Heading drift rate (radians error per radian turned)
    pub heading_drift_rate: f32,
    /// Closure error at end (meters)
    pub closure_position_error: f32,
    /// Closure heading error at end (radians)
    pub closure_heading_error: f32,
    /// Number of samples evaluated
    pub sample_count: usize,
}

impl EvaluationResult {
    /// Check if position error meets a threshold (meters per meter traveled).
    pub fn position_error_ok(&self, max_drift_rate: f32) -> bool {
        self.position_drift_rate <= max_drift_rate
    }

    /// Check if heading error meets a threshold (radians per radian turned).
    pub fn heading_error_ok(&self, max_drift_rate: f32) -> bool {
        self.heading_drift_rate <= max_drift_rate
    }

    /// Check if closure error meets thresholds.
    pub fn closure_error_ok(&self, max_position: f32, max_heading: f32) -> bool {
        self.closure_position_error <= max_position && self.closure_heading_error <= max_heading
    }
}

/// Odometry evaluator for computing drift metrics.
///
/// Collects pose estimates and optionally ground truth, then computes
/// various accuracy metrics.
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::odometry::{OdometryEvaluator, EvaluationResult};
/// use dhruva_slam::types::Pose2D;
///
/// let mut evaluator = OdometryEvaluator::new();
///
/// // Record estimated poses during robot motion
/// evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 0);
/// evaluator.add_estimate(Pose2D::new(1.0, 0.0, 0.0), 100_000);
/// evaluator.add_estimate(Pose2D::new(2.0, 0.0, 0.0), 200_000);
/// // ... robot returns to start ...
/// evaluator.add_estimate(Pose2D::new(0.05, 0.02, 0.01), 1_000_000);
///
/// // Compute closure error (return-to-origin test)
/// let result = evaluator.evaluate_closure();
/// println!("Closure error: {}m", result.closure_position_error);
/// ```
#[derive(Debug, Default)]
pub struct OdometryEvaluator {
    /// Estimated poses with timestamps
    estimates: Vec<Timestamped<Pose2D>>,
    /// Ground truth poses (optional)
    ground_truth: Vec<Timestamped<Pose2D>>,
    /// Origin pose (for closure evaluation)
    origin: Option<Pose2D>,
}

impl OdometryEvaluator {
    /// Create a new evaluator.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset all recorded data.
    pub fn clear(&mut self) {
        self.estimates.clear();
        self.ground_truth.clear();
        self.origin = None;
    }

    /// Add an estimated pose.
    pub fn add_estimate(&mut self, pose: Pose2D, timestamp_us: u64) {
        if self.origin.is_none() {
            self.origin = Some(pose);
        }
        self.estimates.push(Timestamped::new(pose, timestamp_us));
    }

    /// Add a ground truth pose.
    pub fn add_ground_truth(&mut self, pose: Pose2D, timestamp_us: u64) {
        self.ground_truth.push(Timestamped::new(pose, timestamp_us));
    }

    /// Get the number of recorded estimates.
    pub fn estimate_count(&self) -> usize {
        self.estimates.len()
    }

    /// Get the number of recorded ground truth poses.
    pub fn ground_truth_count(&self) -> usize {
        self.ground_truth.len()
    }

    /// Compute total distance traveled from estimates.
    pub fn total_distance(&self) -> f32 {
        if self.estimates.len() < 2 {
            return 0.0;
        }

        let mut total = 0.0f32;
        for i in 1..self.estimates.len() {
            let prev = &self.estimates[i - 1].data;
            let curr = &self.estimates[i].data;
            let dx = curr.x - prev.x;
            let dy = curr.y - prev.y;
            total += (dx * dx + dy * dy).sqrt();
        }
        total
    }

    /// Compute total absolute rotation from estimates.
    pub fn total_rotation(&self) -> f32 {
        if self.estimates.len() < 2 {
            return 0.0;
        }

        let mut total = 0.0f32;
        for i in 1..self.estimates.len() {
            let prev = &self.estimates[i - 1].data;
            let curr = &self.estimates[i].data;
            let dtheta = crate::core::math::angle_diff(prev.theta, curr.theta);
            total += dtheta.abs();
        }
        total
    }

    /// Evaluate closure error (return-to-origin test).
    ///
    /// Assumes the robot started at origin and returned to the same
    /// physical location. Measures the error in the final pose.
    pub fn evaluate_closure(&self) -> EvaluationResult {
        if self.estimates.is_empty() {
            return EvaluationResult::default();
        }

        let origin = self.origin.unwrap_or_default();
        let final_pose = self.estimates.last().map(|t| t.data).unwrap_or(origin);

        let total_distance = self.total_distance();
        let total_rotation = self.total_rotation();

        // Closure error
        let dx = final_pose.x - origin.x;
        let dy = final_pose.y - origin.y;
        let closure_position_error = (dx * dx + dy * dy).sqrt();
        let closure_heading_error =
            crate::core::math::angle_diff(origin.theta, final_pose.theta).abs();

        // Drift rates
        let position_drift_rate = if total_distance > 0.0 {
            closure_position_error / total_distance
        } else {
            0.0
        };

        let heading_drift_rate = if total_rotation > 0.0 {
            closure_heading_error / total_rotation
        } else {
            0.0
        };

        EvaluationResult {
            total_distance,
            total_rotation,
            position_error: Stats::from_slice(&[closure_position_error]),
            heading_error: Stats::from_slice(&[closure_heading_error]),
            position_drift_rate,
            heading_drift_rate,
            closure_position_error,
            closure_heading_error,
            sample_count: self.estimates.len(),
        }
    }

    /// Evaluate against ground truth.
    ///
    /// Computes error metrics by comparing estimates to ground truth.
    /// Ground truth poses are interpolated to match estimate timestamps.
    pub fn evaluate_with_ground_truth(&self) -> EvaluationResult {
        if self.estimates.is_empty() || self.ground_truth.len() < 2 {
            return self.evaluate_closure();
        }

        let mut position_errors = Vec::with_capacity(self.estimates.len());
        let mut heading_errors = Vec::with_capacity(self.estimates.len());

        for estimate in &self.estimates {
            // Find ground truth at this timestamp
            if let Some(gt) = self.interpolate_ground_truth(estimate.timestamp_us) {
                let dx = estimate.data.x - gt.x;
                let dy = estimate.data.y - gt.y;
                let pos_error = (dx * dx + dy * dy).sqrt();
                let heading_error =
                    crate::core::math::angle_diff(estimate.data.theta, gt.theta).abs();

                position_errors.push(pos_error);
                heading_errors.push(heading_error);
            }
        }

        let total_distance = self.total_distance();
        let total_rotation = self.total_rotation();

        let position_error = Stats::from_slice(&position_errors);
        let heading_error = Stats::from_slice(&heading_errors);

        // Use final errors for closure
        let closure_position_error = position_errors.last().cloned().unwrap_or(0.0);
        let closure_heading_error = heading_errors.last().cloned().unwrap_or(0.0);

        let position_drift_rate = if total_distance > 0.0 {
            position_error.mean / (total_distance / position_errors.len() as f32)
        } else {
            0.0
        };

        let heading_drift_rate = if total_rotation > 0.0 {
            heading_error.mean / (total_rotation / heading_errors.len() as f32)
        } else {
            0.0
        };

        EvaluationResult {
            total_distance,
            total_rotation,
            position_error,
            heading_error,
            position_drift_rate,
            heading_drift_rate,
            closure_position_error,
            closure_heading_error,
            sample_count: position_errors.len(),
        }
    }

    /// Interpolate ground truth at a specific timestamp.
    fn interpolate_ground_truth(&self, timestamp_us: u64) -> Option<Pose2D> {
        if self.ground_truth.is_empty() {
            return None;
        }

        // Find surrounding ground truth poses
        let mut before_idx = None;
        let mut after_idx = None;

        for (i, gt) in self.ground_truth.iter().enumerate() {
            if gt.timestamp_us <= timestamp_us {
                before_idx = Some(i);
            } else if after_idx.is_none() {
                after_idx = Some(i);
                break;
            }
        }

        match (before_idx, after_idx) {
            (Some(bi), Some(ai)) => {
                // Interpolate between before and after
                let before = &self.ground_truth[bi];
                let after = &self.ground_truth[ai];
                Pose2D::interpolate(before, after, timestamp_us)
            }
            (Some(bi), None) => {
                // Use last available
                Some(self.ground_truth[bi].data)
            }
            (None, Some(ai)) => {
                // Use first available
                Some(self.ground_truth[ai].data)
            }
            (None, None) => None,
        }
    }

    /// Evaluate a specific path segment.
    ///
    /// Returns error metrics for poses between start_idx and end_idx.
    pub fn evaluate_segment(&self, start_idx: usize, end_idx: usize) -> EvaluationResult {
        if start_idx >= end_idx || end_idx > self.estimates.len() {
            return EvaluationResult::default();
        }

        let segment = &self.estimates[start_idx..end_idx];
        let origin = segment.first().map(|t| t.data).unwrap_or_default();
        let final_pose = segment.last().map(|t| t.data).unwrap_or(origin);

        // Compute distance in segment
        let mut total_distance = 0.0f32;
        let mut total_rotation = 0.0f32;

        for i in 1..segment.len() {
            let prev = &segment[i - 1].data;
            let curr = &segment[i].data;
            let dx = curr.x - prev.x;
            let dy = curr.y - prev.y;
            total_distance += (dx * dx + dy * dy).sqrt();
            total_rotation += crate::core::math::angle_diff(prev.theta, curr.theta).abs();
        }

        // Error from start to end of segment
        let dx = final_pose.x - origin.x;
        let dy = final_pose.y - origin.y;
        let position_error = (dx * dx + dy * dy).sqrt();
        let heading_error = crate::core::math::angle_diff(origin.theta, final_pose.theta).abs();

        EvaluationResult {
            total_distance,
            total_rotation,
            position_error: Stats::from_slice(&[position_error]),
            heading_error: Stats::from_slice(&[heading_error]),
            position_drift_rate: if total_distance > 0.0 {
                position_error / total_distance
            } else {
                0.0
            },
            heading_drift_rate: if total_rotation > 0.0 {
                heading_error / total_rotation
            } else {
                0.0
            },
            closure_position_error: position_error,
            closure_heading_error: heading_error,
            sample_count: segment.len(),
        }
    }

    /// Generate test scenarios for odometry validation.
    ///
    /// Returns expected error bounds for common test scenarios.
    pub fn scenario_bounds(scenario: TestScenario) -> ScenarioBounds {
        match scenario {
            TestScenario::StraightLine2m => ScenarioBounds {
                max_position_error: 0.05,      // 5cm for 2m travel
                max_heading_error: 0.035,      // ~2°
                max_position_drift_rate: 0.05, // 5%
            },
            TestScenario::Rotation360 => ScenarioBounds {
                max_position_error: 0.03, // 3cm (should stay in place)
                max_heading_error: 0.035, // ~2°
                max_position_drift_rate: 0.1,
            },
            TestScenario::SquarePath1m => ScenarioBounds {
                max_position_error: 0.10, // 10cm closure error
                max_heading_error: 0.05,  // ~3°
                max_position_drift_rate: 0.05,
            },
            TestScenario::FigureEight => ScenarioBounds {
                max_position_error: 0.15, // 15cm for complex path
                max_heading_error: 0.07,  // ~4°
                max_position_drift_rate: 0.07,
            },
        }
    }
}

/// Common test scenarios for odometry validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestScenario {
    /// Drive 2m forward, then 2m backward to start
    StraightLine2m,
    /// Rotate 360° clockwise, then counter-clockwise
    Rotation360,
    /// Drive 1m × 4 sides square, return to start
    SquarePath1m,
    /// Complex figure-8 path with varying velocities
    FigureEight,
}

/// Expected error bounds for a test scenario.
#[derive(Debug, Clone, Copy)]
pub struct ScenarioBounds {
    /// Maximum acceptable position error (meters)
    pub max_position_error: f32,
    /// Maximum acceptable heading error (radians)
    pub max_heading_error: f32,
    /// Maximum acceptable position drift rate (m/m)
    pub max_position_drift_rate: f32,
}

impl ScenarioBounds {
    /// Check if evaluation results meet these bounds.
    pub fn check(&self, result: &EvaluationResult) -> bool {
        result.closure_position_error <= self.max_position_error
            && result.closure_heading_error <= self.max_heading_error
            && result.position_drift_rate <= self.max_position_drift_rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::PI;

    #[test]
    fn test_stats_from_slice() {
        let values = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let stats = Stats::from_slice(&values);

        assert_eq!(stats.count, 5);
        assert_relative_eq!(stats.mean, 3.0, epsilon = 1e-6);
        assert_relative_eq!(stats.min, 1.0, epsilon = 1e-6);
        assert_relative_eq!(stats.max, 5.0, epsilon = 1e-6);
    }

    #[test]
    fn test_stats_empty() {
        let stats = Stats::from_slice(&[]);
        assert_eq!(stats.count, 0);
    }

    #[test]
    fn test_evaluator_empty() {
        let evaluator = OdometryEvaluator::new();
        let result = evaluator.evaluate_closure();

        assert_eq!(result.sample_count, 0);
        assert_eq!(result.total_distance, 0.0);
    }

    #[test]
    fn test_total_distance() {
        let mut evaluator = OdometryEvaluator::new();

        evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 0);
        evaluator.add_estimate(Pose2D::new(1.0, 0.0, 0.0), 100_000);
        evaluator.add_estimate(Pose2D::new(1.0, 1.0, 0.0), 200_000);
        evaluator.add_estimate(Pose2D::new(0.0, 1.0, 0.0), 300_000);
        evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 400_000);

        let distance = evaluator.total_distance();
        assert_relative_eq!(distance, 4.0, epsilon = 0.01);
    }

    #[test]
    fn test_total_rotation() {
        let mut evaluator = OdometryEvaluator::new();

        evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 0);
        evaluator.add_estimate(Pose2D::new(0.0, 0.0, PI / 2.0), 100_000);
        evaluator.add_estimate(Pose2D::new(0.0, 0.0, PI), 200_000);
        evaluator.add_estimate(Pose2D::new(0.0, 0.0, PI / 2.0), 300_000);
        evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 400_000);

        let rotation = evaluator.total_rotation();
        assert_relative_eq!(rotation, 2.0 * PI, epsilon = 0.01);
    }

    #[test]
    fn test_closure_error_perfect() {
        let mut evaluator = OdometryEvaluator::new();

        // Perfect return to origin
        evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 0);
        evaluator.add_estimate(Pose2D::new(1.0, 0.0, 0.0), 100_000);
        evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 200_000);

        let result = evaluator.evaluate_closure();

        assert_relative_eq!(result.closure_position_error, 0.0, epsilon = 0.001);
        assert_relative_eq!(result.closure_heading_error, 0.0, epsilon = 0.001);
    }

    #[test]
    fn test_closure_error_with_drift() {
        let mut evaluator = OdometryEvaluator::new();

        // Return with some error
        evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 0);
        evaluator.add_estimate(Pose2D::new(1.0, 0.0, 0.0), 100_000);
        evaluator.add_estimate(Pose2D::new(0.1, 0.05, 0.02), 200_000); // Drift

        let result = evaluator.evaluate_closure();

        let expected_pos_error = (0.1f32.powi(2) + 0.05f32.powi(2)).sqrt();
        assert_relative_eq!(
            result.closure_position_error,
            expected_pos_error,
            epsilon = 0.001
        );
        assert_relative_eq!(result.closure_heading_error, 0.02, epsilon = 0.001);
    }

    #[test]
    fn test_drift_rate() {
        let mut evaluator = OdometryEvaluator::new();

        // 10m forward with 1m error
        evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 0);
        evaluator.add_estimate(Pose2D::new(10.0, 0.0, 0.0), 100_000);
        evaluator.add_estimate(Pose2D::new(1.0, 0.0, 0.0), 200_000); // 1m off

        let result = evaluator.evaluate_closure();

        // Total distance ~20m (10 forward + 9 back), error ~1m
        // Drift rate ~1/20 = 0.05
        assert!(result.position_drift_rate > 0.0);
        assert!(result.position_drift_rate < 0.1);
    }

    #[test]
    fn test_evaluate_segment() {
        let mut evaluator = OdometryEvaluator::new();

        evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 0);
        evaluator.add_estimate(Pose2D::new(1.0, 0.0, 0.0), 100_000);
        evaluator.add_estimate(Pose2D::new(2.0, 0.0, 0.0), 200_000);
        evaluator.add_estimate(Pose2D::new(3.0, 0.0, 0.0), 300_000);

        // Segment [1..3] includes indices 1 and 2, distance from (1,0) to (2,0) = 1m
        let result = evaluator.evaluate_segment(1, 3);

        assert_eq!(result.sample_count, 2);
        assert_relative_eq!(result.total_distance, 1.0, epsilon = 0.01);
    }

    #[test]
    fn test_scenario_bounds() {
        let bounds = OdometryEvaluator::scenario_bounds(TestScenario::StraightLine2m);

        assert!(bounds.max_position_error > 0.0);
        assert!(bounds.max_heading_error > 0.0);
    }

    #[test]
    fn test_scenario_check() {
        let bounds = ScenarioBounds {
            max_position_error: 0.1,
            max_heading_error: 0.05,
            max_position_drift_rate: 0.1,
        };

        let good_result = EvaluationResult {
            closure_position_error: 0.05,
            closure_heading_error: 0.02,
            position_drift_rate: 0.05,
            ..Default::default()
        };

        let bad_result = EvaluationResult {
            closure_position_error: 0.15, // Too high
            closure_heading_error: 0.02,
            position_drift_rate: 0.05,
            ..Default::default()
        };

        assert!(bounds.check(&good_result));
        assert!(!bounds.check(&bad_result));
    }

    #[test]
    fn test_ground_truth_evaluation() {
        let mut evaluator = OdometryEvaluator::new();

        // Add estimates
        evaluator.add_estimate(Pose2D::new(0.0, 0.0, 0.0), 0);
        evaluator.add_estimate(Pose2D::new(1.05, 0.02, 0.01), 100_000); // Slight drift

        // Add ground truth
        evaluator.add_ground_truth(Pose2D::new(0.0, 0.0, 0.0), 0);
        evaluator.add_ground_truth(Pose2D::new(1.0, 0.0, 0.0), 100_000);

        let result = evaluator.evaluate_with_ground_truth();

        assert!(result.position_error.mean > 0.0);
        assert!(result.sample_count > 0);
    }

    #[test]
    fn test_clear() {
        let mut evaluator = OdometryEvaluator::new();

        evaluator.add_estimate(Pose2D::new(1.0, 0.0, 0.0), 0);
        evaluator.add_ground_truth(Pose2D::new(1.0, 0.0, 0.0), 0);

        assert_eq!(evaluator.estimate_count(), 1);
        assert_eq!(evaluator.ground_truth_count(), 1);

        evaluator.clear();

        assert_eq!(evaluator.estimate_count(), 0);
        assert_eq!(evaluator.ground_truth_count(), 0);
    }

    #[test]
    fn test_evaluation_result_checks() {
        let result = EvaluationResult {
            position_drift_rate: 0.03,
            heading_drift_rate: 0.02,
            closure_position_error: 0.05,
            closure_heading_error: 0.01,
            ..Default::default()
        };

        assert!(result.position_error_ok(0.05));
        assert!(!result.position_error_ok(0.02));
        assert!(result.heading_error_ok(0.05));
        assert!(result.closure_error_ok(0.1, 0.05));
        assert!(!result.closure_error_ok(0.01, 0.05));
    }
}
