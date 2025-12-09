//! IMU analysis utilities for diagnostics and quality assessment.
//!
//! Provides tools for analyzing gyroscope and accelerometer data quality,
//! computing drift rates, and assessing sensor health.
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::sensors::odometry::{ImuAnalyzer, ImuQuality};
//!
//! let mut analyzer = ImuAnalyzer::new();
//!
//! // Collect samples while robot operates
//! for (gyro_raw, accel_raw, timestamp_us) in sensor_data {
//!     analyzer.add_sample(gyro_raw, accel_raw, timestamp_us);
//! }
//!
//! // Analyze results
//! let result = analyzer.analyze();
//! println!("Gyro bias: {:?}", result.gyro_bias);
//! println!("Quality: {:?}", result.quality);
//! ```

/// Quality assessment for IMU sensor data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuQuality {
    /// Excellent: Low noise, minimal bias, sensors agree well
    Excellent,
    /// Good: Acceptable noise levels, small bias
    Good,
    /// Moderate: Higher noise or noticeable bias
    Moderate,
    /// Poor: High noise or significant bias - check calibration
    Poor,
}

impl std::fmt::Display for ImuQuality {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ImuQuality::Excellent => write!(f, "Excellent"),
            ImuQuality::Good => write!(f, "Good"),
            ImuQuality::Moderate => write!(f, "Moderate"),
            ImuQuality::Poor => write!(f, "Poor"),
        }
    }
}

/// Results from IMU analysis.
#[derive(Debug, Clone)]
pub struct ImuAnalysisResult {
    /// Estimated gyro bias for each axis [x, y, z] in raw units.
    pub gyro_bias: [f32; 3],
    /// Standard deviation of gyro for each axis [x, y, z].
    pub gyro_std_dev: [f32; 3],
    /// Estimated accel bias for each axis [x, y, z] in raw units.
    pub accel_bias: [f32; 3],
    /// Standard deviation of accel for each axis [x, y, z].
    pub accel_std_dev: [f32; 3],
    /// Drift rate in degrees per second (computed from gyro Z bias).
    pub drift_rate_deg_s: f32,
    /// Recording duration in seconds.
    pub duration_s: f32,
    /// Total number of samples analyzed.
    pub sample_count: usize,
    /// Overall quality assessment.
    pub quality: ImuQuality,
}

/// Analyzes IMU data for quality assessment and diagnostics.
///
/// Collects gyroscope and accelerometer samples and computes statistics
/// useful for calibration and sensor health monitoring.
#[derive(Debug, Clone)]
pub struct ImuAnalyzer {
    /// Gyro samples for each axis.
    gyro_samples: [Vec<i16>; 3],
    /// Accel samples for each axis.
    accel_samples: [Vec<i16>; 3],
    /// First timestamp in microseconds.
    first_timestamp_us: Option<u64>,
    /// Last timestamp in microseconds.
    last_timestamp_us: Option<u64>,
    /// Gyro scale factor (raw units to rad/s).
    gyro_scale: f32,
}

impl ImuAnalyzer {
    /// Create a new IMU analyzer with default gyro scale.
    ///
    /// Default scale is 0.01 deg/s per raw unit (common for MPU-6050 at ±2000 dps).
    pub fn new() -> Self {
        Self::with_gyro_scale(0.01 * std::f32::consts::PI / 180.0)
    }

    /// Create with a specific gyro scale factor.
    ///
    /// # Arguments
    ///
    /// * `gyro_scale` - Conversion factor from raw units to rad/s
    pub fn with_gyro_scale(gyro_scale: f32) -> Self {
        Self {
            gyro_samples: [Vec::new(), Vec::new(), Vec::new()],
            accel_samples: [Vec::new(), Vec::new(), Vec::new()],
            first_timestamp_us: None,
            last_timestamp_us: None,
            gyro_scale,
        }
    }

    /// Add a sample from the IMU.
    ///
    /// # Arguments
    ///
    /// * `gyro_raw` - Raw gyroscope readings [x, y, z]
    /// * `accel_raw` - Raw accelerometer readings [x, y, z]
    /// * `timestamp_us` - Timestamp in microseconds
    pub fn add_sample(&mut self, gyro_raw: [i16; 3], accel_raw: [i16; 3], timestamp_us: u64) {
        for i in 0..3 {
            self.gyro_samples[i].push(gyro_raw[i]);
            self.accel_samples[i].push(accel_raw[i]);
        }

        if self.first_timestamp_us.is_none() {
            self.first_timestamp_us = Some(timestamp_us);
        }
        self.last_timestamp_us = Some(timestamp_us);
    }

    /// Get the number of samples collected.
    pub fn sample_count(&self) -> usize {
        self.gyro_samples[0].len()
    }

    /// Get the recording duration in seconds.
    pub fn duration_s(&self) -> f32 {
        match (self.first_timestamp_us, self.last_timestamp_us) {
            (Some(first), Some(last)) if last > first => (last - first) as f32 / 1_000_000.0,
            _ => 0.0,
        }
    }

    /// Analyze the collected IMU data.
    ///
    /// Returns statistics and quality assessment.
    pub fn analyze(&self) -> ImuAnalysisResult {
        let sample_count = self.sample_count();

        // Compute gyro statistics
        let gyro_bias = [
            compute_mean(&self.gyro_samples[0]),
            compute_mean(&self.gyro_samples[1]),
            compute_mean(&self.gyro_samples[2]),
        ];

        let gyro_std_dev = [
            compute_std_dev(&self.gyro_samples[0]),
            compute_std_dev(&self.gyro_samples[1]),
            compute_std_dev(&self.gyro_samples[2]),
        ];

        // Compute accel statistics
        let accel_bias = [
            compute_mean(&self.accel_samples[0]),
            compute_mean(&self.accel_samples[1]),
            compute_mean(&self.accel_samples[2]),
        ];

        let accel_std_dev = [
            compute_std_dev(&self.accel_samples[0]),
            compute_std_dev(&self.accel_samples[1]),
            compute_std_dev(&self.accel_samples[2]),
        ];

        // Compute drift rate from gyro Z bias (yaw axis per ROS REP-103)
        // drift_rate_deg_s = bias_raw * scale * rad2deg
        let drift_rate_deg_s = gyro_bias[2] * self.gyro_scale * 180.0 / std::f32::consts::PI;

        let duration_s = self.duration_s();

        // Assess quality based on gyro Z (yaw) statistics
        // These thresholds are for typical consumer-grade MEMS IMUs
        let quality = self.assess_quality(gyro_std_dev[2], gyro_bias[2].abs());

        ImuAnalysisResult {
            gyro_bias,
            gyro_std_dev,
            accel_bias,
            accel_std_dev,
            drift_rate_deg_s,
            duration_s,
            sample_count,
            quality,
        }
    }

    /// Compute integrated rotation from gyro Z.
    ///
    /// # Arguments
    ///
    /// * `encoder_theta_rad` - Optional encoder-derived rotation for comparison
    ///
    /// # Returns
    ///
    /// Tuple of (raw_integrated, bias_corrected, agreement_percent if encoder provided)
    pub fn compute_gyro_rotation(&self, encoder_theta_rad: Option<f64>) -> GyroRotationResult {
        let duration_s = self.duration_s() as f64;
        let sample_count = self.sample_count();

        if sample_count < 2 || duration_s <= 0.0 {
            return GyroRotationResult {
                raw_integrated_rad: 0.0,
                bias_corrected_rad: 0.0,
                encoder_theta_rad,
                agreement_percent: None,
            };
        }

        // Compute dt per sample (assume uniform sampling)
        let dt_s = duration_s / (sample_count - 1) as f64;

        // Integrate gyro Z
        let gyro_scale = self.gyro_scale as f64;
        let raw_integrated: f64 = self.gyro_samples[2]
            .iter()
            .map(|&g| g as f64 * gyro_scale * dt_s)
            .sum();

        // Bias correct
        let bias = compute_mean(&self.gyro_samples[2]) as f64;
        let bias_correction = bias * gyro_scale * duration_s;
        let bias_corrected = raw_integrated - bias_correction;

        // Compute agreement with encoder if provided
        let agreement_percent = encoder_theta_rad.map(|enc| {
            if enc.abs() > 0.01 {
                100.0 * (1.0 - (raw_integrated - enc).abs() / enc.abs())
            } else {
                // Static case - use raw value comparison
                if raw_integrated.abs() < 0.01 {
                    100.0
                } else {
                    0.0
                }
            }
        });

        GyroRotationResult {
            raw_integrated_rad: raw_integrated,
            bias_corrected_rad: bias_corrected,
            encoder_theta_rad,
            agreement_percent,
        }
    }

    /// Assess IMU quality based on noise and bias levels.
    fn assess_quality(&self, gyro_z_std: f32, gyro_z_bias_abs: f32) -> ImuQuality {
        // Thresholds based on typical consumer MEMS IMU characteristics
        // std_dev in raw units, bias_abs in raw units

        if gyro_z_std < 3.0 && gyro_z_bias_abs < 5.0 {
            ImuQuality::Excellent
        } else if gyro_z_std < 5.0 && gyro_z_bias_abs < 15.0 {
            ImuQuality::Good
        } else if gyro_z_std < 10.0 && gyro_z_bias_abs < 30.0 {
            ImuQuality::Moderate
        } else {
            ImuQuality::Poor
        }
    }

    /// Reset the analyzer to collect new samples.
    pub fn reset(&mut self) {
        for i in 0..3 {
            self.gyro_samples[i].clear();
            self.accel_samples[i].clear();
        }
        self.first_timestamp_us = None;
        self.last_timestamp_us = None;
    }
}

impl Default for ImuAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

/// Results from gyro rotation computation.
#[derive(Debug, Clone)]
pub struct GyroRotationResult {
    /// Raw integrated gyro rotation in radians.
    pub raw_integrated_rad: f64,
    /// Bias-corrected integrated rotation in radians.
    pub bias_corrected_rad: f64,
    /// Encoder-derived rotation for comparison (if provided).
    pub encoder_theta_rad: Option<f64>,
    /// Agreement percentage between gyro and encoder (if encoder provided).
    pub agreement_percent: Option<f64>,
}

// Helper functions

fn compute_mean(samples: &[i16]) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }
    samples.iter().map(|&v| v as f32).sum::<f32>() / samples.len() as f32
}

fn compute_std_dev(samples: &[i16]) -> f32 {
    if samples.len() < 2 {
        return 0.0;
    }

    let mean = compute_mean(samples);
    let variance = samples
        .iter()
        .map(|&v| {
            let diff = v as f32 - mean;
            diff * diff
        })
        .sum::<f32>()
        / (samples.len() - 1) as f32;

    variance.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_imu_analyzer_new() {
        let analyzer = ImuAnalyzer::new();
        assert_eq!(analyzer.sample_count(), 0);
        assert_relative_eq!(analyzer.duration_s(), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_imu_analyzer_add_samples() {
        let mut analyzer = ImuAnalyzer::new();

        analyzer.add_sample([0, 0, 10], [0, 0, 1000], 0);
        analyzer.add_sample([0, 0, 12], [0, 0, 1000], 10_000);
        analyzer.add_sample([0, 0, 8], [0, 0, 1000], 20_000);

        assert_eq!(analyzer.sample_count(), 3);
        assert_relative_eq!(analyzer.duration_s(), 0.02, epsilon = 1e-6);
    }

    #[test]
    fn test_imu_analyzer_analyze_gyro_bias() {
        let mut analyzer = ImuAnalyzer::new();

        // Add samples with mean gyro_z = 10
        for i in 0..100 {
            analyzer.add_sample([0, 0, 10], [0, 0, 1000], i * 10_000);
        }

        let result = analyzer.analyze();
        assert_relative_eq!(result.gyro_bias[2], 10.0, epsilon = 1e-5);
        assert_eq!(result.sample_count, 100);
    }

    #[test]
    fn test_imu_analyzer_analyze_gyro_std_dev() {
        let mut analyzer = ImuAnalyzer::new();

        // Add samples: 0, 2, 4, 6, 8, 10 (std_dev ≈ 3.74)
        let values = [0i16, 2, 4, 6, 8, 10];
        for (i, &v) in values.iter().enumerate() {
            analyzer.add_sample([0, 0, v], [0, 0, 0], (i * 10_000) as u64);
        }

        let result = analyzer.analyze();
        // Sample std dev
        assert!(result.gyro_std_dev[2] > 3.0 && result.gyro_std_dev[2] < 4.0);
    }

    #[test]
    fn test_imu_analyzer_quality_excellent() {
        let mut analyzer = ImuAnalyzer::new();

        // Low noise, low bias
        for i in 0..100 {
            analyzer.add_sample([0, 0, 1], [0, 0, 0], i * 10_000);
        }

        let result = analyzer.analyze();
        assert_eq!(result.quality, ImuQuality::Excellent);
    }

    #[test]
    fn test_imu_analyzer_quality_poor() {
        let mut analyzer = ImuAnalyzer::new();

        // High noise and bias
        for i in 0..100 {
            let noise = if i % 2 == 0 { 50 } else { -50 };
            analyzer.add_sample([0, 0, 100 + noise], [0, 0, 0], (i * 10_000) as u64);
        }

        let result = analyzer.analyze();
        assert_eq!(result.quality, ImuQuality::Poor);
    }

    #[test]
    fn test_imu_analyzer_gyro_rotation() {
        let mut analyzer = ImuAnalyzer::with_gyro_scale(0.01_f32.to_radians()); // 0.01 deg/s per raw unit

        // Simulate 1 second of data at 100Hz with constant gyro Z = 100 raw
        // That's 1.0 deg/s, so after 1s we should have 1 degree = 0.0175 rad
        for i in 0..100 {
            analyzer.add_sample([0, 0, 100], [0, 0, 0], i * 10_000);
        }

        let result = analyzer.compute_gyro_rotation(None);
        // Expected: 100 raw * 0.01 deg/s * (π/180) * 1s ≈ 0.0175 rad
        assert!(
            result.raw_integrated_rad > 0.01 && result.raw_integrated_rad < 0.025,
            "Got: {}",
            result.raw_integrated_rad
        );
    }

    #[test]
    fn test_imu_analyzer_gyro_rotation_with_encoder() {
        let mut analyzer = ImuAnalyzer::with_gyro_scale(0.01_f32.to_radians());

        // Simulate rotation matching encoder
        for i in 0..100 {
            analyzer.add_sample([0, 0, 100], [0, 0, 0], i * 10_000);
        }

        // If encoder shows similar rotation, agreement should be high
        let expected_rad = 0.0175; // ~1 degree
        let result = analyzer.compute_gyro_rotation(Some(expected_rad));

        assert!(result.agreement_percent.is_some());
        let agreement = result.agreement_percent.unwrap();
        assert!(
            agreement > 50.0, // Should be reasonably close
            "Agreement: {}%",
            agreement
        );
    }

    #[test]
    fn test_imu_analyzer_reset() {
        let mut analyzer = ImuAnalyzer::new();

        for i in 0..10 {
            analyzer.add_sample([0, 0, 10], [0, 0, 0], i * 10_000);
        }
        assert_eq!(analyzer.sample_count(), 10);

        analyzer.reset();
        assert_eq!(analyzer.sample_count(), 0);
        assert_relative_eq!(analyzer.duration_s(), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_imu_quality_display() {
        assert_eq!(format!("{}", ImuQuality::Excellent), "Excellent");
        assert_eq!(format!("{}", ImuQuality::Good), "Good");
        assert_eq!(format!("{}", ImuQuality::Moderate), "Moderate");
        assert_eq!(format!("{}", ImuQuality::Poor), "Poor");
    }

    #[test]
    fn test_compute_mean() {
        assert_relative_eq!(compute_mean(&[1, 2, 3, 4, 5]), 3.0, epsilon = 1e-5);
        assert_relative_eq!(compute_mean(&[]), 0.0, epsilon = 1e-6);
        assert_relative_eq!(compute_mean(&[-10, 10]), 0.0, epsilon = 1e-5);
    }

    #[test]
    fn test_compute_std_dev() {
        // std_dev of [2, 4, 4, 4, 5, 5, 7, 9] = 2.0
        let samples: Vec<i16> = vec![2, 4, 4, 4, 5, 5, 7, 9];
        let std = compute_std_dev(&samples);
        assert!(std > 1.9 && std < 2.4, "Got: {}", std);

        // Empty/single should return 0
        assert_relative_eq!(compute_std_dev(&[]), 0.0, epsilon = 1e-6);
        assert_relative_eq!(compute_std_dev(&[5]), 0.0, epsilon = 1e-6);
    }
}
