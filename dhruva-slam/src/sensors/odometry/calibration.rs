//! IMU calibration utilities.
//!
//! Provides tools for estimating gyroscope bias and other calibration parameters.
//!
//! # Hardware Constants
//!
//! This module defines hardware-specific calibration constants:
//! - [`CRL200S_GYRO_SCALE`]: Gyroscope scale factor for CRL-200S robot
//!
//! # Example
//!
//! ```
//! use dhruva_slam::sensors::odometry::GyroBiasEstimator;
//!
//! let mut estimator = GyroBiasEstimator::new(100);
//!
//! // Collect samples while robot is stationary
//! let gyro_readings: Vec<i16> = vec![10, 12, 11, 9, 10]; // Example raw gyro values
//! for gyro_z_raw in gyro_readings {
//!     if estimator.add_sample(gyro_z_raw) {
//!         // Calibration complete
//!         let bias = estimator.compute_bias();
//!         println!("Estimated gyro bias: {}", bias);
//!         break;
//!     }
//! }
//! ```

use std::f32::consts::PI;

// ============================================================================
// Hardware-specific calibration constants
// ============================================================================

/// Gyroscope scale factor for CRL-200S robot.
///
/// Converts raw gyro readings to radians/second.
///
/// # Derivation
///
/// The GD32F103 MCU reports gyro data at 110Hz with raw units of 0.1 deg/s.
/// After empirical calibration, the actual scale is 1.025 deg/s per raw unit
/// (5% higher than nominal due to sensor-specific variance).
///
/// Formula: 0.01025 * (π / 180) = ~0.000179 rad/s per raw unit
///
/// # Usage
///
/// ```ignore
/// let gyro_raw = sensor_reading as f32;
/// let gyro_rad_per_sec = (gyro_raw - bias) * CRL200S_GYRO_SCALE;
/// ```
pub const CRL200S_GYRO_SCALE: f32 = 0.01025 * (PI / 180.0);

/// Estimates gyroscope bias from stationary samples.
///
/// Collects raw gyroscope readings and computes the mean bias.
/// The robot should be stationary during bias estimation.
///
/// # Usage
///
/// 1. Create an estimator with desired sample count
/// 2. Feed samples using `add_sample()` until it returns `true`
/// 3. Call `compute_bias()` to get the estimated bias
///
/// Typical sample counts:
/// - 100 samples (~1s at 100Hz): Quick calibration
/// - 330 samples (~3s at 110Hz): Standard calibration
/// - 1000+ samples: High-precision calibration
#[derive(Debug, Clone)]
pub struct GyroBiasEstimator {
    samples: Vec<i16>,
    required_samples: usize,
}

impl GyroBiasEstimator {
    /// Create a new estimator that collects the specified number of samples.
    ///
    /// # Arguments
    ///
    /// * `required_samples` - Number of samples to collect before bias can be computed
    pub fn new(required_samples: usize) -> Self {
        Self {
            samples: Vec::with_capacity(required_samples),
            required_samples,
        }
    }

    /// Add a gyroscope sample.
    ///
    /// Returns `true` when enough samples have been collected.
    ///
    /// # Arguments
    ///
    /// * `gyro_z` - Raw gyroscope reading (typically Z-axis for yaw)
    pub fn add_sample(&mut self, gyro_z: i16) -> bool {
        if self.samples.len() < self.required_samples {
            self.samples.push(gyro_z);
        }
        self.is_ready()
    }

    /// Check if enough samples have been collected.
    pub fn is_ready(&self) -> bool {
        self.samples.len() >= self.required_samples
    }

    /// Get the number of samples collected so far.
    pub fn sample_count(&self) -> usize {
        self.samples.len()
    }

    /// Get the required number of samples.
    pub fn required_samples(&self) -> usize {
        self.required_samples
    }

    /// Get calibration progress as a fraction (0.0 to 1.0).
    pub fn progress(&self) -> f32 {
        if self.required_samples == 0 {
            1.0
        } else {
            self.samples.len() as f32 / self.required_samples as f32
        }
    }

    /// Compute the estimated bias as the mean of collected samples.
    ///
    /// Returns 0.0 if no samples have been collected.
    pub fn compute_bias(&self) -> f32 {
        if self.samples.is_empty() {
            return 0.0;
        }
        self.samples.iter().map(|&g| g as f32).sum::<f32>() / self.samples.len() as f32
    }

    /// Compute the standard deviation of collected samples.
    ///
    /// Useful for assessing gyro noise quality.
    /// Returns 0.0 if fewer than 2 samples.
    pub fn compute_std_dev(&self) -> f32 {
        if self.samples.len() < 2 {
            return 0.0;
        }

        let mean = self.compute_bias();
        let variance = self
            .samples
            .iter()
            .map(|&g| {
                let diff = g as f32 - mean;
                diff * diff
            })
            .sum::<f32>()
            / (self.samples.len() - 1) as f32;

        variance.sqrt()
    }

    /// Reset the estimator to collect new samples.
    pub fn reset(&mut self) {
        self.samples.clear();
    }

    /// Reset with a new required sample count.
    pub fn reset_with_count(&mut self, required_samples: usize) {
        self.samples.clear();
        self.samples.reserve(required_samples);
        self.required_samples = required_samples;
    }
}

impl Default for GyroBiasEstimator {
    /// Creates an estimator with 330 samples (3 seconds at 110Hz).
    fn default() -> Self {
        Self::new(330)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_gyro_bias_estimator_new() {
        let estimator = GyroBiasEstimator::new(100);
        assert_eq!(estimator.required_samples(), 100);
        assert_eq!(estimator.sample_count(), 0);
        assert!(!estimator.is_ready());
    }

    #[test]
    fn test_gyro_bias_estimator_default() {
        let estimator = GyroBiasEstimator::default();
        assert_eq!(estimator.required_samples(), 330);
    }

    #[test]
    fn test_gyro_bias_estimator_add_samples() {
        let mut estimator = GyroBiasEstimator::new(5);

        assert!(!estimator.add_sample(10));
        assert!(!estimator.add_sample(12));
        assert!(!estimator.add_sample(8));
        assert!(!estimator.add_sample(11));
        assert!(estimator.add_sample(9)); // 5th sample, now ready

        assert!(estimator.is_ready());
        assert_eq!(estimator.sample_count(), 5);
    }

    #[test]
    fn test_gyro_bias_estimator_compute_bias() {
        let mut estimator = GyroBiasEstimator::new(5);

        // Add samples with mean of 10
        estimator.add_sample(8);
        estimator.add_sample(10);
        estimator.add_sample(12);
        estimator.add_sample(9);
        estimator.add_sample(11);

        let bias = estimator.compute_bias();
        assert_relative_eq!(bias, 10.0, epsilon = 1e-5);
    }

    #[test]
    fn test_gyro_bias_estimator_compute_bias_empty() {
        let estimator = GyroBiasEstimator::new(100);
        assert_relative_eq!(estimator.compute_bias(), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_gyro_bias_estimator_std_dev() {
        let mut estimator = GyroBiasEstimator::new(5);

        // Add samples: 2, 4, 6, 8, 10 (mean=6, std_dev=sqrt(10)≈3.16)
        estimator.add_sample(2);
        estimator.add_sample(4);
        estimator.add_sample(6);
        estimator.add_sample(8);
        estimator.add_sample(10);

        let std_dev = estimator.compute_std_dev();
        // Sample std dev of [2,4,6,8,10] = sqrt(40/4) = sqrt(10) ≈ 3.162
        assert_relative_eq!(std_dev, 3.162, epsilon = 0.01);
    }

    #[test]
    fn test_gyro_bias_estimator_progress() {
        let mut estimator = GyroBiasEstimator::new(10);

        assert_relative_eq!(estimator.progress(), 0.0, epsilon = 1e-6);

        for _ in 0..5 {
            estimator.add_sample(0);
        }
        assert_relative_eq!(estimator.progress(), 0.5, epsilon = 1e-6);

        for _ in 0..5 {
            estimator.add_sample(0);
        }
        assert_relative_eq!(estimator.progress(), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_gyro_bias_estimator_reset() {
        let mut estimator = GyroBiasEstimator::new(5);

        for _ in 0..5 {
            estimator.add_sample(10);
        }
        assert!(estimator.is_ready());

        estimator.reset();
        assert!(!estimator.is_ready());
        assert_eq!(estimator.sample_count(), 0);
        assert_eq!(estimator.required_samples(), 5);
    }

    #[test]
    fn test_gyro_bias_estimator_reset_with_count() {
        let mut estimator = GyroBiasEstimator::new(5);

        for _ in 0..5 {
            estimator.add_sample(10);
        }

        estimator.reset_with_count(100);
        assert!(!estimator.is_ready());
        assert_eq!(estimator.sample_count(), 0);
        assert_eq!(estimator.required_samples(), 100);
    }

    #[test]
    fn test_gyro_bias_estimator_negative_values() {
        let mut estimator = GyroBiasEstimator::new(3);

        estimator.add_sample(-10);
        estimator.add_sample(-5);
        estimator.add_sample(-15);

        let bias = estimator.compute_bias();
        assert_relative_eq!(bias, -10.0, epsilon = 1e-5);
    }

    #[test]
    fn test_gyro_bias_estimator_does_not_exceed_required() {
        let mut estimator = GyroBiasEstimator::new(3);

        estimator.add_sample(10);
        estimator.add_sample(10);
        estimator.add_sample(10);
        estimator.add_sample(100); // Should not be added
        estimator.add_sample(100); // Should not be added

        assert_eq!(estimator.sample_count(), 3);
        assert_relative_eq!(estimator.compute_bias(), 10.0, epsilon = 1e-6);
    }
}
