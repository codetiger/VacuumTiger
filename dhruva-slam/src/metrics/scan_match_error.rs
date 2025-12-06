//! Transform error computation for scan matching evaluation.
//!
//! Provides unified error metrics for comparing estimated transforms
//! against ground truth (e.g., odometry).

use crate::core::types::Pose2D;

/// Error between an estimated transform and expected (ground truth) transform.
#[derive(Debug, Clone, Copy)]
pub struct TransformError {
    /// Translation error in meters (Euclidean distance)
    pub translation_error: f32,
    /// Rotation error in radians (absolute difference, normalized to [-π, π])
    pub rotation_error: f32,
}

impl TransformError {
    /// Check if error is within convergence thresholds.
    ///
    /// # Arguments
    /// * `trans_threshold` - Maximum acceptable translation error in meters
    /// * `rot_threshold` - Maximum acceptable rotation error in radians
    pub fn converged(&self, trans_threshold: f32, rot_threshold: f32) -> bool {
        self.translation_error <= trans_threshold && self.rotation_error <= rot_threshold
    }
}

/// Normalize an angle to the range [-π, π].
fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle;
    while a > std::f32::consts::PI {
        a -= 2.0 * std::f32::consts::PI;
    }
    while a < -std::f32::consts::PI {
        a += 2.0 * std::f32::consts::PI;
    }
    a
}

/// Compute the error between an estimated pose and expected pose.
///
/// # Arguments
/// * `estimated` - The pose estimated by scan matching
/// * `expected` - The ground truth pose (e.g., from odometry)
///
/// # Returns
/// `TransformError` containing translation and rotation errors
pub fn compute_transform_error(estimated: &Pose2D, expected: &Pose2D) -> TransformError {
    let dx = estimated.x - expected.x;
    let dy = estimated.y - expected.y;
    let translation_error = (dx * dx + dy * dy).sqrt();

    let dtheta = normalize_angle(estimated.theta - expected.theta);
    let rotation_error = dtheta.abs();

    TransformError {
        translation_error,
        rotation_error,
    }
}

/// Statistics for transform errors across multiple test pairs.
#[derive(Debug, Clone)]
pub struct TransformErrorStats {
    /// All translation errors in meters
    pub translation_errors: Vec<f32>,
    /// All rotation errors in radians
    pub rotation_errors: Vec<f32>,
    /// Offsets (scan separation) for each test pair
    pub offsets: Vec<usize>,
    /// Number of pairs that converged within threshold
    pub converged_count: usize,
}

impl TransformErrorStats {
    /// Create new empty stats.
    pub fn new() -> Self {
        Self {
            translation_errors: Vec::new(),
            rotation_errors: Vec::new(),
            offsets: Vec::new(),
            converged_count: 0,
        }
    }

    /// Record a transform error result.
    pub fn record(&mut self, error: TransformError, offset: usize, converged: bool) {
        self.translation_errors.push(error.translation_error);
        self.rotation_errors.push(error.rotation_error);
        self.offsets.push(offset);
        if converged {
            self.converged_count += 1;
        }
    }

    /// Get total number of test pairs.
    pub fn total_pairs(&self) -> usize {
        self.translation_errors.len()
    }

    /// Convergence rate as percentage.
    pub fn convergence_rate(&self) -> f32 {
        if self.total_pairs() == 0 {
            0.0
        } else {
            100.0 * self.converged_count as f32 / self.total_pairs() as f32
        }
    }

    /// Mean translation error in meters.
    pub fn mean_translation_error(&self) -> f32 {
        if self.translation_errors.is_empty() {
            f32::NAN
        } else {
            self.translation_errors.iter().sum::<f32>() / self.translation_errors.len() as f32
        }
    }

    /// Median translation error in meters.
    pub fn median_translation_error(&self) -> f32 {
        if self.translation_errors.is_empty() {
            return f32::NAN;
        }
        let mut sorted = self.translation_errors.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        sorted[sorted.len() / 2]
    }

    /// Mean rotation error in radians.
    pub fn mean_rotation_error(&self) -> f32 {
        if self.rotation_errors.is_empty() {
            f32::NAN
        } else {
            self.rotation_errors.iter().sum::<f32>() / self.rotation_errors.len() as f32
        }
    }

    /// Median rotation error in radians.
    pub fn median_rotation_error(&self) -> f32 {
        if self.rotation_errors.is_empty() {
            return f32::NAN;
        }
        let mut sorted = self.rotation_errors.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        sorted[sorted.len() / 2]
    }

    /// Get errors grouped by offset buckets.
    ///
    /// Returns (bucket_range, mean_trans_err, mean_rot_err, count) for each bucket.
    pub fn errors_by_offset_bucket(
        &self,
        bucket_size: usize,
    ) -> Vec<(std::ops::Range<usize>, f32, f32, usize)> {
        if self.offsets.is_empty() {
            return Vec::new();
        }

        let max_offset = *self.offsets.iter().max().unwrap_or(&0);
        let mut buckets: Vec<(std::ops::Range<usize>, Vec<f32>, Vec<f32>)> = Vec::new();

        let mut start = 1;
        while start <= max_offset {
            let end = start + bucket_size;
            buckets.push((start..end, Vec::new(), Vec::new()));
            start = end;
        }

        for i in 0..self.offsets.len() {
            let offset = self.offsets[i];
            for (range, trans_errs, rot_errs) in &mut buckets {
                if range.contains(&offset) {
                    trans_errs.push(self.translation_errors[i]);
                    rot_errs.push(self.rotation_errors[i]);
                    break;
                }
            }
        }

        buckets
            .into_iter()
            .filter(|(_, t, _)| !t.is_empty())
            .map(|(range, trans_errs, rot_errs)| {
                let count = trans_errs.len();
                let mean_trans = trans_errs.iter().sum::<f32>() / count as f32;
                let mean_rot = rot_errs.iter().sum::<f32>() / count as f32;
                (range, mean_trans, mean_rot, count)
            })
            .collect()
    }
}

impl Default for TransformErrorStats {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_transform_error_zero() {
        let p1 = Pose2D::new(0.0, 0.0, 0.0);
        let p2 = Pose2D::new(0.0, 0.0, 0.0);
        let err = compute_transform_error(&p1, &p2);
        assert!((err.translation_error - 0.0).abs() < 1e-6);
        assert!((err.rotation_error - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_transform_error_translation() {
        let p1 = Pose2D::new(3.0, 4.0, 0.0);
        let p2 = Pose2D::new(0.0, 0.0, 0.0);
        let err = compute_transform_error(&p1, &p2);
        assert!((err.translation_error - 5.0).abs() < 1e-6);
        assert!((err.rotation_error - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_transform_error_rotation() {
        let p1 = Pose2D::new(0.0, 0.0, 0.5);
        let p2 = Pose2D::new(0.0, 0.0, 0.0);
        let err = compute_transform_error(&p1, &p2);
        assert!((err.translation_error - 0.0).abs() < 1e-6);
        assert!((err.rotation_error - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-6);
        assert!((normalize_angle(std::f32::consts::PI) - std::f32::consts::PI).abs() < 1e-6);
        assert!((normalize_angle(2.0 * std::f32::consts::PI) - 0.0).abs() < 1e-6);
        assert!((normalize_angle(-2.0 * std::f32::consts::PI) - 0.0).abs() < 1e-6);
    }
}
