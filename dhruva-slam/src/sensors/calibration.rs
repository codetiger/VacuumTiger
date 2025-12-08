//! Sensor calibration utilities.
//!
//! Provides automatic calibration routines for gyroscope bias estimation.
//!
//! # Usage
//!
//! ```ignore
//! use dhruva_slam::sensors::calibration::GyroBiasEstimator;
//!
//! let mut estimator = GyroBiasEstimator::new(100); // Collect 100 samples
//!
//! // During startup while robot is stationary:
//! while !estimator.is_ready() {
//!     estimator.add_sample(gyro_z_raw);
//! }
//!
//! let bias = estimator.bias();
//! complementary_config.gyro_bias_z = bias;
//! ```

/// Estimates gyroscope bias from stationary samples.
///
/// Collects samples while the robot is stationary and computes the mean
/// as the bias estimate. The variance is also computed to assess quality.
#[derive(Debug)]
pub struct GyroBiasEstimator {
    /// Target number of samples to collect
    target_samples: usize,
    /// Sum of all samples
    sum: f64,
    /// Sum of squared samples (for variance)
    sum_sq: f64,
    /// Number of samples collected
    count: usize,
    /// Computed bias (mean of samples)
    bias: f32,
    /// Computed variance
    variance: f32,
    /// Whether calibration is complete
    ready: bool,
}

impl GyroBiasEstimator {
    /// Create a new bias estimator.
    ///
    /// # Arguments
    ///
    /// * `target_samples` - Number of samples to collect before computing bias.
    ///   More samples = more accurate but longer calibration time.
    ///   Typical values: 10-45 samples at 110Hz = 100-400ms
    pub fn new(target_samples: usize) -> Self {
        Self {
            target_samples: target_samples.max(2), // Minimum 2 samples for variance calculation
            sum: 0.0,
            sum_sq: 0.0,
            count: 0,
            bias: 0.0,
            variance: 0.0,
            ready: false,
        }
    }

    /// Create with default settings (100 samples).
    pub fn default_samples() -> Self {
        Self::new(100)
    }

    /// Add a gyroscope sample.
    ///
    /// Call this repeatedly while the robot is stationary during startup.
    /// Once enough samples are collected, the bias will be automatically computed.
    ///
    /// # Arguments
    ///
    /// * `gyro_z_raw` - Raw gyroscope Z reading
    pub fn add_sample(&mut self, gyro_z_raw: i16) {
        if self.ready {
            return; // Already calibrated
        }

        let value = gyro_z_raw as f64;
        self.sum += value;
        self.sum_sq += value * value;
        self.count += 1;

        if self.count >= self.target_samples {
            self.finalize();
        }
    }

    /// Manually finalize calibration with current samples.
    ///
    /// Useful if you want to complete calibration early (e.g., after detecting
    /// that the robot has started moving).
    pub fn finalize(&mut self) {
        if self.count < 2 {
            return;
        }

        let n = self.count as f64;
        let mean = self.sum / n;
        let variance = (self.sum_sq / n) - (mean * mean);

        self.bias = mean as f32;
        self.variance = variance.max(0.0) as f32; // Ensure non-negative
        self.ready = true;
    }

    /// Check if calibration is complete.
    pub fn is_ready(&self) -> bool {
        self.ready
    }

    /// Get the estimated bias.
    ///
    /// Returns 0.0 if calibration is not complete.
    pub fn bias(&self) -> f32 {
        self.bias
    }

    /// Get the variance of the samples.
    ///
    /// Lower variance indicates more stable readings and higher quality calibration.
    /// Returns 0.0 if calibration is not complete.
    pub fn variance(&self) -> f32 {
        self.variance
    }

    /// Get the standard deviation of the samples.
    pub fn std_dev(&self) -> f32 {
        self.variance.sqrt()
    }

    /// Get the number of samples collected.
    pub fn sample_count(&self) -> usize {
        self.count
    }

    /// Reset the estimator to collect new samples.
    pub fn reset(&mut self) {
        self.sum = 0.0;
        self.sum_sq = 0.0;
        self.count = 0;
        self.bias = 0.0;
        self.variance = 0.0;
        self.ready = false;
    }

    /// Check if the calibration quality is acceptable.
    ///
    /// Returns true if variance is below the threshold, indicating stable readings.
    /// A high variance might indicate the robot was moving during calibration.
    ///
    /// # Arguments
    ///
    /// * `max_variance` - Maximum acceptable variance. Typical value: 100.0 for raw units.
    pub fn is_quality_acceptable(&self, max_variance: f32) -> bool {
        self.ready && self.variance < max_variance
    }
}

impl Default for GyroBiasEstimator {
    fn default() -> Self {
        Self::default_samples()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_calibration() {
        let mut estimator = GyroBiasEstimator::new(10);

        // Add samples with known bias of 50
        for _ in 0..10 {
            estimator.add_sample(50);
        }

        assert!(estimator.is_ready());
        assert!((estimator.bias() - 50.0).abs() < 0.01);
        assert!(estimator.variance() < 0.01); // Perfect samples = zero variance
    }

    #[test]
    fn test_variance_calculation() {
        let mut estimator = GyroBiasEstimator::new(4);

        // Add samples: 10, 20, 30, 40 -> mean = 25, var = 125
        estimator.add_sample(10);
        estimator.add_sample(20);
        estimator.add_sample(30);
        estimator.add_sample(40);

        assert!(estimator.is_ready());
        assert!((estimator.bias() - 25.0).abs() < 0.01);
        assert!((estimator.variance() - 125.0).abs() < 0.1);
    }

    #[test]
    fn test_not_ready_before_samples() {
        let mut estimator = GyroBiasEstimator::new(10);

        estimator.add_sample(50);
        estimator.add_sample(50);

        assert!(!estimator.is_ready());
        assert_eq!(estimator.bias(), 0.0);
    }

    #[test]
    fn test_manual_finalize() {
        let mut estimator = GyroBiasEstimator::new(100);

        // Add only 5 samples, then finalize early
        for _ in 0..5 {
            estimator.add_sample(100);
        }

        assert!(!estimator.is_ready());

        estimator.finalize();

        assert!(estimator.is_ready());
        assert!((estimator.bias() - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_reset() {
        let mut estimator = GyroBiasEstimator::new(5);

        for _ in 0..5 {
            estimator.add_sample(50);
        }
        assert!(estimator.is_ready());

        estimator.reset();

        assert!(!estimator.is_ready());
        assert_eq!(estimator.sample_count(), 0);
        assert_eq!(estimator.bias(), 0.0);
    }

    #[test]
    fn test_ignores_samples_after_ready() {
        let mut estimator = GyroBiasEstimator::new(5);

        for _ in 0..5 {
            estimator.add_sample(50);
        }

        let bias_before = estimator.bias();

        // These should be ignored
        for _ in 0..5 {
            estimator.add_sample(1000);
        }

        assert_eq!(estimator.bias(), bias_before);
        assert_eq!(estimator.sample_count(), 5);
    }

    #[test]
    fn test_quality_check() {
        let mut estimator = GyroBiasEstimator::new(4);

        // High variance samples
        estimator.add_sample(0);
        estimator.add_sample(100);
        estimator.add_sample(0);
        estimator.add_sample(100);

        assert!(estimator.is_ready());
        assert!(!estimator.is_quality_acceptable(100.0)); // Variance is 2500
        assert!(estimator.is_quality_acceptable(3000.0));
    }

    #[test]
    fn test_negative_values() {
        let mut estimator = GyroBiasEstimator::new(4);

        estimator.add_sample(-10);
        estimator.add_sample(-20);
        estimator.add_sample(-30);
        estimator.add_sample(-40);

        assert!(estimator.is_ready());
        assert!((estimator.bias() - (-25.0)).abs() < 0.01);
    }
}
