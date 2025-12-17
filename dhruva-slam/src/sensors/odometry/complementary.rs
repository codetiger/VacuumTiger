//! Complementary filter for encoder + gyroscope fusion.
//!
//! Combines wheel encoder heading estimates with gyroscope measurements
//! to produce more accurate heading estimates.
//!
//! # Why Complementary Filter?
//!
//! | Source     | Strength                        | Weakness                    |
//! |------------|--------------------------------|-----------------------------|
//! | Encoders   | Accurate distance, no drift    | Slip causes heading error   |
//! | Gyroscope  | Good short-term rotation       | Integrates bias → drift     |
//!
//! The complementary filter acts as a weighted combination:
//! - Low frequency (long term): Trust encoder heading
//! - High frequency (short term): Trust gyroscope
//!
//! This is achieved through the alpha parameter:
//! ```text
//! θ_fused = α × θ_gyro + (1 - α) × θ_encoder
//! ```
//!
//! Note: Some utility methods are defined for future use.

use super::calibration::CRL200S_GYRO_SCALE;
use crate::core::types::Pose2D;

/// Configuration for the complementary filter.
#[derive(Debug, Clone, Copy)]
pub struct ComplementaryConfig {
    /// Gyroscope weight (0.0 to 1.0).
    ///
    /// Higher values trust the gyroscope more.
    /// - 0.0 = pure encoder (ignores gyro)
    /// - 1.0 = pure gyro (ignores encoder heading)
    /// - 0.98 = good default for most applications
    pub alpha: f32,

    /// Gyroscope scale factor.
    ///
    /// Multiply raw gyro reading by this to get rad/s.
    /// For CRL-200S: raw units are 0.1 deg/s → use `CRL200S_GYRO_SCALE` (0.001745)
    pub gyro_scale: f32,

    /// Gyroscope yaw (Z axis) bias in raw units (before scaling).
    ///
    /// This is subtracted from the raw gyro reading before scaling.
    /// Should be calibrated at startup when robot is stationary.
    ///
    /// Note: For ROS REP-103 compliance, pass `gyro_raw[2]` (Z axis) as the
    /// yaw rate to `update()`. See `utils::constants::GYRO_YAW_INDEX`.
    pub gyro_bias_z: f32,

    /// Gyroscope sign multiplier.
    ///
    /// Use -1.0 if gyro reports opposite sign to encoder rotation.
    /// With SangamIO frame transforms, gyro sign should match encoder convention.
    pub gyro_sign: f32,
}

impl Default for ComplementaryConfig {
    fn default() -> Self {
        Self {
            // Reduced from 0.98 to 0.90 to trust encoders more.
            // With alpha=0.98, even small gyro bias causes significant drift.
            // With alpha=0.90, the filter is more balanced and robust to
            // uncalibrated gyro bias while still benefiting from gyro smoothness.
            alpha: 0.90,
            gyro_scale: CRL200S_GYRO_SCALE, // CRL-200S: raw units are 0.1 deg/s
            gyro_bias_z: 0.0,
            gyro_sign: 1.0, // Sign correction now done in SangamIO frame_transforms
        }
    }
}

/// Complementary filter for fusing encoder and gyroscope data.
///
/// Maintains a pose estimate by applying encoder-derived position changes
/// while fusing encoder and gyroscope heading estimates.
///
/// # Usage
///
/// ```ignore
/// use dhruva_slam::odometry::{ComplementaryFilter, ComplementaryConfig};
/// use dhruva_slam::types::Pose2D;
///
/// let config = ComplementaryConfig::default();
/// let mut filter = ComplementaryFilter::new(config);
///
/// // In your sensor processing loop:
/// let encoder_delta = Pose2D::new(0.01, 0.0, 0.05); // from WheelOdometry
/// let gyro_z_raw = 100i16; // raw gyro value
/// let timestamp_us = 1000000u64;
///
/// let pose = filter.update(encoder_delta, gyro_z_raw, timestamp_us);
/// println!("Current pose: {:?}", pose);
/// ```
#[derive(Debug)]
pub struct ComplementaryFilter {
    config: ComplementaryConfig,
    pose: Pose2D,
    last_timestamp_us: Option<u64>,
}

impl ComplementaryFilter {
    /// Create a new complementary filter at the origin.
    pub fn new(config: ComplementaryConfig) -> Self {
        Self {
            config,
            pose: Pose2D::identity(),
            last_timestamp_us: None,
        }
    }

    /// Update the filter with new sensor data.
    ///
    /// # Arguments
    ///
    /// * `encoder_delta` - Pose delta from wheel odometry (in robot frame)
    /// * `gyro_z_raw` - Raw gyroscope yaw (Z axis) reading before calibration.
    ///   For ROS REP-103: use `gyro_raw[2]` from the sensor status.
    /// * `timestamp_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// The updated global pose estimate.
    pub fn update(&mut self, encoder_delta: Pose2D, gyro_z_raw: i16, timestamp_us: u64) -> Pose2D {
        // Compute dt from timestamps
        let dt = match self.last_timestamp_us {
            Some(last) => {
                if timestamp_us <= last {
                    // Handle timestamp wraparound or out-of-order data
                    0.0
                } else {
                    (timestamp_us - last) as f32 / 1_000_000.0 // Convert us to seconds
                }
            }
            None => 0.0, // First update, no time delta
        };
        self.last_timestamp_us = Some(timestamp_us);

        // Apply calibration and sign correction to gyro and compute angular change
        let gyro_z_calibrated = (gyro_z_raw as f32 - self.config.gyro_bias_z)
            * self.config.gyro_scale
            * self.config.gyro_sign;
        let dtheta_gyro = gyro_z_calibrated * dt;

        // Get encoder heading change
        let dtheta_encoder = encoder_delta.theta;

        // Fuse heading estimates
        // When robot is stationary (no encoder motion), trust encoders completely to avoid gyro drift
        let dtheta_fused = if dt > 0.0 {
            let is_stationary = encoder_delta.x.abs() < 1e-6 && encoder_delta.theta.abs() < 1e-6;
            if is_stationary {
                // When stationary, ignore gyro completely - encoder says no motion
                // This prevents drift from residual gyro bias error
                dtheta_encoder
            } else {
                // Normal fusion when robot is moving
                self.config.alpha * dtheta_gyro + (1.0 - self.config.alpha) * dtheta_encoder
            }
        } else {
            // On first update or zero dt, just use encoder
            dtheta_encoder
        };

        // Create fused delta pose (position from encoder, heading fused)
        let fused_delta = Pose2D::new(encoder_delta.x, encoder_delta.y, dtheta_fused);

        // Apply delta to current pose
        self.pose = self.pose.compose(&fused_delta);

        self.pose
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn test_config() -> ComplementaryConfig {
        ComplementaryConfig {
            alpha: 0.98,
            gyro_scale: 0.001, // 1 raw unit = 0.001 rad/s
            gyro_bias_z: 0.0,
            gyro_sign: 1.0, // Tests use synthetic data with same sign convention
        }
    }

    #[test]
    fn test_initial_pose() {
        let mut filter = ComplementaryFilter::new(test_config());
        // Initialize
        filter.update(Pose2D::identity(), 0, 0);
        // After first update, pose should still be identity
        let pose = filter.update(Pose2D::identity(), 0, 10_000);
        assert_relative_eq!(pose.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_pure_encoder_motion() {
        let config = ComplementaryConfig {
            alpha: 0.0, // Trust encoder only
            gyro_scale: 1.0,
            gyro_bias_z: 0.0,
            gyro_sign: 1.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        // First update initializes timestamp
        let delta = Pose2D::new(1.0, 0.0, 0.1);
        let _pose = filter.update(delta, 0, 0);

        // Second update with encoder motion
        let delta = Pose2D::new(0.5, 0.0, 0.05);
        let pose = filter.update(delta, 1000, 100_000); // 100ms later

        // With alpha=0, should follow encoder exactly
        // First: (1,0,0.1), Second compose: should add roughly 0.5 forward
        assert!(pose.x > 1.0);
    }

    #[test]
    fn test_gyro_fusion() {
        let config = ComplementaryConfig {
            alpha: 0.98,
            gyro_scale: 1.0, // 1 unit = 1 rad/s
            gyro_bias_z: 0.0,
            gyro_sign: 1.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        // Initialize
        filter.update(Pose2D::identity(), 0, 0);

        // Encoder says no rotation, gyro says rotating at 1 rad/s
        // dt = 0.1s, so gyro contribution = 0.98 * 1.0 * 0.1 = 0.098 rad
        let delta = Pose2D::new(0.1, 0.0, 0.0); // Encoder: forward, no rotation
        let gyro_z = 1; // 1 rad/s (with scale=1)
        let pose = filter.update(delta, gyro_z, 100_000); // 100ms = 0.1s

        // Should have moved forward and rotated (mostly from gyro)
        assert!(pose.x > 0.0);
        assert!(pose.theta > 0.08); // Mostly gyro contribution
    }

    #[test]
    fn test_gyro_bias_correction() {
        let config = ComplementaryConfig {
            alpha: 1.0, // Trust gyro only for heading
            gyro_scale: 0.001,
            gyro_bias_z: 100.0, // Bias of 100 raw units
            gyro_sign: 1.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        // Initialize
        filter.update(Pose2D::identity(), 100, 0);

        // Gyro reading equals bias, so should be zero rotation
        let delta = Pose2D::new(0.1, 0.0, 0.0);
        let pose = filter.update(delta, 100, 100_000);

        // With gyro at bias level, net rotation should be ~0
        assert_relative_eq!(pose.theta, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_multiple_updates() {
        let config = ComplementaryConfig {
            alpha: 0.5, // Equal weight
            gyro_scale: 1.0,
            gyro_bias_z: 0.0,
            gyro_sign: 1.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        // Simulate a series of updates
        let dt_us = 10_000u64; // 10ms between updates

        filter.update(Pose2D::identity(), 0, 0);

        let mut final_pose = Pose2D::identity();
        for i in 1..=100 {
            let encoder_delta = Pose2D::new(0.001, 0.0, 0.001); // Small forward motion + turn
            let gyro = 1; // 1 rad/s
            final_pose = filter.update(encoder_delta, gyro, i * dt_us);
        }

        // Should have moved forward ~0.1m
        assert!(final_pose.x > 0.05);
        // Should have rotated
        assert!(final_pose.theta.abs() > 0.0);
    }

    // ========================================================================
    // Edge Case Tests
    // ========================================================================

    #[test]
    fn test_timestamp_going_backwards() {
        let config = test_config();
        let mut filter = ComplementaryFilter::new(config);

        // Initialize at timestamp 1000
        filter.update(Pose2D::identity(), 0, 1000);

        // Update at timestamp 500 (backwards!) - should handle gracefully
        let delta = Pose2D::new(0.1, 0.0, 0.1);
        let pose = filter.update(delta, 100, 500);

        // Should still produce valid pose (dt = 0 case)
        assert!(pose.x.is_finite());
        assert!(pose.theta.is_finite());
    }

    #[test]
    fn test_very_large_timestamp_gap() {
        let config = ComplementaryConfig {
            alpha: 0.98,
            gyro_scale: 1.0,
            gyro_bias_z: 0.0,
            gyro_sign: 1.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        filter.update(Pose2D::identity(), 0, 0);

        // 10 seconds later with high gyro reading
        let delta = Pose2D::new(0.1, 0.0, 0.0);
        let gyro = 1; // 1 rad/s for 10 seconds
        let pose = filter.update(delta, gyro, 10_000_000); // 10 seconds in µs

        // Should have accumulated ~10 radians of rotation from gyro
        // (normalized to [-π, π])
        assert!(pose.theta.is_finite());
    }

    #[test]
    fn test_zero_dt_first_update() {
        let config = test_config();
        let mut filter = ComplementaryFilter::new(config);

        // First update has no previous timestamp, so dt = 0
        let delta = Pose2D::new(0.1, 0.0, 0.1);
        let pose = filter.update(delta, 1000, 0);

        // Should use encoder-only for first update
        assert_relative_eq!(pose.x, 0.1, epsilon = 0.01);
        // Theta should be from encoder (no gyro contribution with dt=0)
        assert_relative_eq!(pose.theta, 0.1, epsilon = 0.01);
    }

    #[test]
    fn test_alpha_zero_pure_encoder() {
        let config = ComplementaryConfig {
            alpha: 0.0, // Pure encoder
            gyro_scale: 1.0,
            gyro_bias_z: 0.0,
            gyro_sign: 1.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        filter.update(Pose2D::identity(), 0, 0);

        // Encoder says 0.1 rad rotation, gyro says 100 rad/s (very different!)
        let delta = Pose2D::new(0.0, 0.0, 0.1);
        let pose = filter.update(delta, 100, 100_000); // 100ms

        // With alpha=0, should trust encoder completely
        assert_relative_eq!(pose.theta, 0.1, epsilon = 0.01);
    }

    #[test]
    fn test_alpha_one_pure_gyro() {
        let config = ComplementaryConfig {
            alpha: 1.0, // Pure gyro
            gyro_scale: 1.0,
            gyro_bias_z: 0.0,
            gyro_sign: 1.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        filter.update(Pose2D::identity(), 0, 0);

        // Encoder says 1.0 rad rotation, gyro says 0.5 rad/s
        let delta = Pose2D::new(0.1, 0.0, 1.0);
        let pose = filter.update(delta, 0, 100_000); // 100ms, gyro = 0

        // With alpha=1, should trust gyro completely
        // dt=0.1s, gyro=0 rad/s -> dtheta_gyro = 0
        assert_relative_eq!(pose.theta, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_same_timestamp_consecutive() {
        let config = test_config();
        let mut filter = ComplementaryFilter::new(config);

        filter.update(Pose2D::identity(), 0, 1000);

        // Same timestamp twice (dt = 0)
        let delta = Pose2D::new(0.1, 0.0, 0.1);
        let pose = filter.update(delta, 100, 1000);

        // Should fall back to encoder-only
        assert!(pose.x.is_finite());
    }

    #[test]
    fn test_large_gyro_bias_correction() {
        let config = ComplementaryConfig {
            alpha: 1.0,
            gyro_scale: 0.001,
            gyro_bias_z: 1000.0, // Large bias
            gyro_sign: 1.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        filter.update(Pose2D::identity(), 0, 0);

        // Raw gyro at bias level = zero rotation
        let delta = Pose2D::new(0.1, 0.0, 0.0);
        let pose = filter.update(delta, 1000, 100_000);

        // Gyro contribution should be (1000 - 1000) * 0.001 * 0.1 = 0
        assert_relative_eq!(pose.theta, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_negative_gyro_values() {
        let config = ComplementaryConfig {
            alpha: 0.98,
            gyro_scale: 0.01, // 0.01 rad/s per unit
            gyro_bias_z: 0.0,
            gyro_sign: 1.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        filter.update(Pose2D::identity(), 0, 0);

        // Negative gyro (clockwise rotation)
        // -10 raw * 0.01 scale = -0.1 rad/s
        // Over 0.1s: -0.01 rad (small, stays in bounds)
        let delta = Pose2D::new(0.1, 0.0, 0.0);
        let pose = filter.update(delta, -10, 100_000);

        // Should produce negative theta (CW rotation)
        // dtheta_fused = 0.98 * (-0.01) + 0.02 * 0 ≈ -0.0098
        assert!(
            pose.theta < 0.0,
            "Expected negative theta, got {}",
            pose.theta
        );
    }
}
