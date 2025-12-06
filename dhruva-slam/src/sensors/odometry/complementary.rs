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

use crate::core::types::Pose2D;

/// Gyroscope scale factor for CRL-200S IMU.
///
/// Raw gyro values are in units of 0.1 deg/s.
/// Convert to rad/s: 0.1 * (π / 180) ≈ 0.001745329
pub const CRL200S_GYRO_SCALE: f32 = 0.1 * (std::f32::consts::PI / 180.0);

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

    /// Gyroscope Z bias in raw units (before scaling).
    ///
    /// This is subtracted from the raw gyro reading before scaling.
    /// Should be calibrated at startup when robot is stationary.
    pub gyro_bias_z: f32,
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

    /// Create a new complementary filter at a specific initial pose.
    pub fn new_at(config: ComplementaryConfig, initial_pose: Pose2D) -> Self {
        Self {
            config,
            pose: initial_pose,
            last_timestamp_us: None,
        }
    }

    /// Get the current pose estimate.
    pub fn pose(&self) -> Pose2D {
        self.pose
    }

    /// Get the current configuration.
    pub fn config(&self) -> &ComplementaryConfig {
        &self.config
    }

    /// Reset to the origin.
    pub fn reset(&mut self) {
        self.pose = Pose2D::identity();
        self.last_timestamp_us = None;
    }

    /// Reset to a specific pose.
    pub fn reset_to(&mut self, pose: Pose2D) {
        self.pose = pose;
        self.last_timestamp_us = None;
    }

    /// Update the filter with new sensor data.
    ///
    /// # Arguments
    ///
    /// * `encoder_delta` - Pose delta from wheel odometry (in robot frame)
    /// * `gyro_z_raw` - Raw gyroscope Z reading (before calibration)
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

        // Apply calibration to gyro and compute angular change
        let gyro_z_calibrated =
            (gyro_z_raw as f32 - self.config.gyro_bias_z) * self.config.gyro_scale;
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

    /// Update with pre-calibrated gyro value in rad/s.
    ///
    /// Use this if you've already applied calibration to the gyro reading.
    ///
    /// # Arguments
    ///
    /// * `encoder_delta` - Pose delta from wheel odometry (in robot frame)
    /// * `gyro_z_rad_s` - Calibrated gyroscope Z reading in rad/s
    /// * `timestamp_us` - Current timestamp in microseconds
    pub fn update_calibrated(
        &mut self,
        encoder_delta: Pose2D,
        gyro_z_rad_s: f32,
        timestamp_us: u64,
    ) -> Pose2D {
        // Compute dt from timestamps
        let dt = match self.last_timestamp_us {
            Some(last) => {
                if timestamp_us <= last {
                    0.0
                } else {
                    (timestamp_us - last) as f32 / 1_000_000.0
                }
            }
            None => 0.0,
        };
        self.last_timestamp_us = Some(timestamp_us);

        let dtheta_gyro = gyro_z_rad_s * dt;
        let dtheta_encoder = encoder_delta.theta;

        let dtheta_fused = if dt > 0.0 {
            self.config.alpha * dtheta_gyro + (1.0 - self.config.alpha) * dtheta_encoder
        } else {
            dtheta_encoder
        };

        let fused_delta = Pose2D::new(encoder_delta.x, encoder_delta.y, dtheta_fused);
        self.pose = self.pose.compose(&fused_delta);

        self.pose
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::FRAC_PI_2;

    fn test_config() -> ComplementaryConfig {
        ComplementaryConfig {
            alpha: 0.98,
            gyro_scale: 0.001, // 1 raw unit = 0.001 rad/s
            gyro_bias_z: 0.0,
        }
    }

    #[test]
    fn test_initial_pose() {
        let filter = ComplementaryFilter::new(test_config());
        let pose = filter.pose();
        assert_eq!(pose.x, 0.0);
        assert_eq!(pose.y, 0.0);
        assert_eq!(pose.theta, 0.0);
    }

    #[test]
    fn test_initial_pose_custom() {
        let initial = Pose2D::new(1.0, 2.0, 0.5);
        let filter = ComplementaryFilter::new_at(test_config(), initial);
        let pose = filter.pose();
        assert_eq!(pose.x, 1.0);
        assert_eq!(pose.y, 2.0);
        assert_relative_eq!(pose.theta, 0.5, epsilon = 1e-6);
    }

    #[test]
    fn test_pure_encoder_motion() {
        let config = ComplementaryConfig {
            alpha: 0.0, // Trust encoder only
            gyro_scale: 1.0,
            gyro_bias_z: 0.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        // First update initializes timestamp
        let delta = Pose2D::new(1.0, 0.0, 0.1);
        filter.update(delta, 0, 0);

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
    fn test_reset() {
        let mut filter = ComplementaryFilter::new(test_config());

        // Move around
        filter.update(Pose2D::new(1.0, 0.0, 0.5), 0, 0);
        filter.update(Pose2D::new(0.5, 0.0, 0.1), 0, 100_000);

        assert!(filter.pose().x > 0.5);

        // Reset
        filter.reset();

        let pose = filter.pose();
        assert_eq!(pose.x, 0.0);
        assert_eq!(pose.y, 0.0);
        assert_eq!(pose.theta, 0.0);
    }

    #[test]
    fn test_reset_to() {
        let mut filter = ComplementaryFilter::new(test_config());

        filter.update(Pose2D::new(1.0, 0.0, 0.5), 0, 0);

        let new_pose = Pose2D::new(5.0, 3.0, FRAC_PI_2);
        filter.reset_to(new_pose);

        let pose = filter.pose();
        assert_relative_eq!(pose.x, 5.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 3.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, FRAC_PI_2, epsilon = 1e-6);
    }

    #[test]
    fn test_update_calibrated() {
        let config = ComplementaryConfig {
            alpha: 0.98,
            gyro_scale: 1.0, // ignored for calibrated
            gyro_bias_z: 0.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        // Initialize
        filter.update_calibrated(Pose2D::identity(), 0.0, 0);

        // Update with calibrated gyro (0.5 rad/s)
        let delta = Pose2D::new(0.1, 0.0, 0.0);
        let pose = filter.update_calibrated(delta, 0.5, 100_000);

        // dt = 0.1s, gyro contribution = 0.98 * 0.5 * 0.1 = 0.049 rad
        assert!(pose.theta > 0.04);
        assert!(pose.theta < 0.06);
    }

    #[test]
    fn test_multiple_updates() {
        let config = ComplementaryConfig {
            alpha: 0.5, // Equal weight
            gyro_scale: 1.0,
            gyro_bias_z: 0.0,
        };
        let mut filter = ComplementaryFilter::new(config);

        // Simulate a series of updates
        let dt_us = 10_000u64; // 10ms between updates

        filter.update(Pose2D::identity(), 0, 0);

        for i in 1..=100 {
            let encoder_delta = Pose2D::new(0.001, 0.0, 0.001); // Small forward motion + turn
            let gyro = 1; // 1 rad/s
            filter.update(encoder_delta, gyro, i * dt_us);
        }

        let final_pose = filter.pose();

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

    #[test]
    fn test_config_accessor() {
        let config = test_config();
        let filter = ComplementaryFilter::new(config);

        assert_eq!(filter.config().alpha, 0.98);
        assert_eq!(filter.config().gyro_scale, 0.001);
    }
}
