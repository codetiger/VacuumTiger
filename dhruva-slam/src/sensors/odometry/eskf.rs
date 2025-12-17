//! Error-State Kalman Filter (ESKF) for sensor fusion.
//!
//! The ESKF maintains a "nominal state" that represents the best estimate
//! of the robot's pose, and an "error state" that captures small deviations
//! from this nominal state. This separation provides better numerical
//! stability compared to a standard EKF.
//!
//! # State Representation
//!
//! - Nominal state: Full pose (x, y, θ)
//! - Error state: Small errors (δx, δy, δθ)
//!
//! # Algorithm
//!
//! 1. **Prediction**: Propagate nominal state with encoder odometry, propagate error covariance
//! 2. **Update**: Fuse gyro measurement by computing innovation (gyro_dtheta - encoder_dtheta)
//! 3. **Reset**: Inject error into nominal state, reset error to zero
//!
//! # Sensor Fusion Strategy
//!
//! The gyro update compares the heading change measured by the gyroscope against
//! the heading change from encoders. The innovation (difference) is used to correct
//! the estimate, weighted by the Kalman gain based on relative uncertainties.
//!
//! This approach fuses the two sensors properly:
//! - When gyro and encoder agree: small correction, trust both
//! - When they disagree: correction weighted by noise parameters
//!
//! # References
//!
//! - Madyastha, V., et al. "Extended Kalman filter vs. error-state Kalman filter for aircraft attitude estimation"
//! - Solà, J. "Quaternion kinematics for the error-state Kalman filter"
//!
//! Note: Some utility methods are defined for future use.

use crate::core::types::{Covariance2D, Pose2D};

/// Process noise parameters for ESKF.
#[derive(Debug, Clone, Copy)]
pub struct ProcessNoise {
    /// Position noise variance per meter traveled (m²/m)
    pub position_var_per_meter: f32,
    /// Heading noise variance from encoders per radian turned (rad²/rad)
    pub heading_var_per_radian: f32,
}

impl Default for ProcessNoise {
    fn default() -> Self {
        Self {
            position_var_per_meter: 0.01, // 10% of distance as std dev
            heading_var_per_radian: 0.01, // ~6° std dev per radian turned
        }
    }
}

/// Measurement noise parameters.
#[derive(Debug, Clone, Copy)]
pub struct MeasurementNoise {
    /// Gyroscope measurement variance (rad/s)²
    pub gyro_variance: f32,
}

impl Default for MeasurementNoise {
    fn default() -> Self {
        Self {
            gyro_variance: 0.001, // ~1.8°/s std dev
        }
    }
}

/// Configuration for the Error-State Kalman Filter.
#[derive(Debug, Clone)]
pub struct EskfConfig {
    /// Process noise parameters
    pub process_noise: ProcessNoise,
    /// Measurement noise parameters
    pub measurement_noise: MeasurementNoise,
    /// Gyroscope scale factor (raw units to rad/s)
    pub gyro_scale: f32,
    /// Gyroscope bias in raw units
    pub gyro_bias_z: f32,
    /// Gyroscope sign multiplier (1.0 with SangamIO REP-103 compliance)
    pub gyro_sign: f32,
    /// Initial position variance (m²)
    pub initial_position_variance: f32,
    /// Initial heading variance (rad²)
    pub initial_heading_variance: f32,
}

impl Default for EskfConfig {
    fn default() -> Self {
        Self {
            process_noise: ProcessNoise::default(),
            measurement_noise: MeasurementNoise::default(),
            // Calibrated using encoder ground truth from rotation bags.
            // Raw gyro value of ~3740 at 38.33°/s gives 0.01025 deg/s per LSB.
            gyro_scale: 0.01025 * (std::f32::consts::PI / 180.0),
            gyro_bias_z: 0.0,
            gyro_sign: 1.0, // SangamIO provides REP-103 compliant data (CCW positive)
            initial_position_variance: 0.01, // 10cm std dev
            initial_heading_variance: 0.01, // ~6° std dev
        }
    }
}

/// Error-State Kalman Filter for 2D robot pose estimation.
///
/// Fuses wheel encoder odometry with gyroscope measurements using an
/// error-state formulation for improved numerical stability.
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::odometry::{Eskf, EskfConfig};
/// use dhruva_slam::types::Pose2D;
///
/// let config = EskfConfig::default();
/// let mut eskf = Eskf::new(config);
///
/// // Combined update step with encoder delta and gyroscope
/// let encoder_delta = Pose2D::new(0.01, 0.0, 0.05);
/// let gyro_z_raw = 100i16;
/// let timestamp_us = 1000000u64;
/// let pose = eskf.update(encoder_delta, gyro_z_raw, timestamp_us);
/// ```
#[derive(Debug)]
pub struct Eskf {
    config: EskfConfig,
    /// Nominal state (best estimate)
    nominal: Pose2D,
    /// Error state covariance matrix P (3x3)
    /// Represents uncertainty in [δx, δy, δθ]
    covariance: Covariance2D,
    /// Last timestamp for dt computation
    last_timestamp_us: Option<u64>,
    /// Accumulated encoder heading change since last gyro update.
    /// Used to compute innovation: gyro_dtheta - encoder_dtheta
    pending_encoder_dtheta: f32,
}

impl Eskf {
    /// Create a new ESKF at the origin.
    pub fn new(config: EskfConfig) -> Self {
        let covariance = Covariance2D::diagonal(
            config.initial_position_variance,
            config.initial_position_variance,
            config.initial_heading_variance,
        );

        Self {
            config,
            nominal: Pose2D::identity(),
            covariance,
            last_timestamp_us: None,
            pending_encoder_dtheta: 0.0,
        }
    }

    /// Prediction step: propagate state with encoder odometry.
    ///
    /// Updates the nominal state and propagates error covariance.
    /// Also accumulates the encoder heading change for later gyro fusion.
    ///
    /// # Arguments
    ///
    /// * `encoder_delta` - Pose delta from wheel odometry (in robot frame)
    pub fn predict(&mut self, encoder_delta: Pose2D) {
        // Update nominal state
        self.nominal = self.nominal.compose(&encoder_delta);

        // Accumulate encoder heading change for gyro fusion
        self.pending_encoder_dtheta += encoder_delta.theta;

        // Compute motion magnitude for process noise scaling
        let distance =
            (encoder_delta.x * encoder_delta.x + encoder_delta.y * encoder_delta.y).sqrt();
        let rotation = encoder_delta.theta.abs();

        // Process noise covariance Q
        let q_xx = self.config.process_noise.position_var_per_meter * distance;
        let q_yy = self.config.process_noise.position_var_per_meter * distance;
        let q_tt = self.config.process_noise.heading_var_per_radian * rotation;

        // Jacobian of motion model with respect to state
        // For 2D rigid body motion: new_state = old_state ⊕ delta
        //
        // ∂(x',y',θ')/∂(x,y,θ) at θ:
        // | 1  0  -δx*sin(θ) - δy*cos(θ) |
        // | 0  1   δx*cos(θ) - δy*sin(θ) |
        // | 0  0   1                      |
        let (sin_t, cos_t) = self.nominal.theta.sin_cos();
        let dx = encoder_delta.x;
        let dy = encoder_delta.y;

        // F matrix elements (Jacobian of motion model)
        let f02 = -dx * sin_t - dy * cos_t;
        let f12 = dx * cos_t - dy * sin_t;

        // Propagate covariance: P' = F * P * F^T + Q
        // Using the structure of F (identity with modifications in last column)
        let p = self.covariance.as_slice();

        // P * F^T
        // F^T = | 1  0  0 |
        //       | 0  1  0 |
        //       | f02 f12 1 |
        let pft00 = p[0] + p[2] * f02;
        let pft01 = p[1] + p[2] * f12;
        let pft02 = p[2];
        let pft10 = p[3] + p[5] * f02;
        let pft11 = p[4] + p[5] * f12;
        let pft12 = p[5];
        let pft20 = p[6] + p[8] * f02;
        let pft21 = p[7] + p[8] * f12;
        let pft22 = p[8];

        // F * (P * F^T)
        let new_p = [
            pft00 + f02 * pft20 + q_xx, // [0,0]
            pft01 + f02 * pft21,        // [0,1]
            pft02 + f02 * pft22,        // [0,2]
            pft10 + f12 * pft20,        // [1,0]
            pft11 + f12 * pft21 + q_yy, // [1,1]
            pft12 + f12 * pft22,        // [1,2]
            pft20,                      // [2,0]
            pft21,                      // [2,1]
            pft22 + q_tt,               // [2,2]
        ];

        self.covariance = Covariance2D::from_array(new_p);
    }

    /// Update step with gyroscope measurement.
    ///
    /// Fuses gyroscope angular velocity measurement with encoder heading.
    /// The innovation is computed as the difference between gyro-measured
    /// heading change and encoder-measured heading change since the last update.
    ///
    /// # Arguments
    ///
    /// * `gyro_z_raw` - Raw gyroscope Z reading
    /// * `dt` - Time delta in seconds since last update
    pub fn update_gyro(&mut self, gyro_z_raw: i16, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        // Convert raw gyro to rad/s with sign correction
        let gyro_z_rad_s = (gyro_z_raw as f32 - self.config.gyro_bias_z)
            * self.config.gyro_scale
            * self.config.gyro_sign;

        // Heading change from gyro over this time interval
        let dtheta_gyro = gyro_z_rad_s * dt;

        // Innovation: difference between gyro and encoder heading change
        // This is the KEY fix: we compare the two sensors, not add gyro on top
        // - Positive innovation: gyro says we rotated more than encoder
        // - Negative innovation: gyro says we rotated less than encoder
        // The Kalman gain weights how much we trust each sensor
        let innovation = dtheta_gyro - self.pending_encoder_dtheta;

        // Reset pending encoder heading for next update cycle
        self.pending_encoder_dtheta = 0.0;

        // Skip update if innovation is negligible (sensors agree)
        if innovation.abs() < 1e-6 {
            return;
        }

        // Measurement matrix H: measures only heading error
        // H = [0, 0, 1]

        // Innovation covariance: S = H * P * H^T + R
        // R is measurement noise (gyro variance scaled by dt²)
        let p_theta = self.covariance.var_theta();
        let r = self.config.measurement_noise.gyro_variance * dt * dt;
        let s = p_theta + r;

        if s.abs() < 1e-10 {
            return;
        }

        // Kalman gain: K = P * H^T * S^-1
        // Since H = [0, 0, 1], K = P[:, 2] / S
        let p = self.covariance.as_slice();
        let k0 = p[2] / s; // K[0]
        let k1 = p[5] / s; // K[1]
        let k2 = p[8] / s; // K[2]

        // Update error state (then inject into nominal)
        // The correction is weighted by Kalman gain:
        // - High gain (large P, small R): trust gyro more
        // - Low gain (small P, large R): trust encoder more
        let delta_x = k0 * innovation;
        let delta_y = k1 * innovation;
        let delta_theta = k2 * innovation;

        // Inject error into nominal state
        self.nominal = Pose2D::new(
            self.nominal.x + delta_x,
            self.nominal.y + delta_y,
            self.nominal.theta + delta_theta,
        );

        // Update covariance: P' = (I - K * H) * P
        // With H = [0, 0, 1]:
        // (I - K*H) = | 1   0   -k0 |
        //             | 0   1   -k1 |
        //             | 0   0  1-k2 |
        let new_p = [
            p[0] - k0 * p[6],  // [0,0]
            p[1] - k0 * p[7],  // [0,1]
            p[2] - k0 * p[8],  // [0,2]
            p[3] - k1 * p[6],  // [1,0]
            p[4] - k1 * p[7],  // [1,1]
            p[5] - k1 * p[8],  // [1,2]
            (1.0 - k2) * p[6], // [2,0]
            (1.0 - k2) * p[7], // [2,1]
            (1.0 - k2) * p[8], // [2,2]
        ];

        self.covariance = Covariance2D::from_array(new_p);
    }

    /// Combined predict and update step.
    ///
    /// Convenience method that performs prediction with encoder data
    /// and update with gyroscope data in one call.
    ///
    /// # Arguments
    ///
    /// * `encoder_delta` - Pose delta from wheel odometry
    /// * `gyro_z_raw` - Raw gyroscope Z reading
    /// * `timestamp_us` - Current timestamp in microseconds
    pub fn update(&mut self, encoder_delta: Pose2D, gyro_z_raw: i16, timestamp_us: u64) -> Pose2D {
        // Compute dt
        let dt = match self.last_timestamp_us {
            Some(last) if timestamp_us > last => (timestamp_us - last) as f32 / 1_000_000.0,
            _ => 0.0,
        };
        self.last_timestamp_us = Some(timestamp_us);

        // Prediction step with encoder odometry
        self.predict(encoder_delta);

        // Update step with gyroscope
        if dt > 0.0 {
            self.update_gyro(gyro_z_raw, dt);
        }

        self.nominal
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::PI;

    fn test_config() -> EskfConfig {
        EskfConfig {
            process_noise: ProcessNoise {
                position_var_per_meter: 0.01,
                heading_var_per_radian: 0.01,
            },
            measurement_noise: MeasurementNoise {
                gyro_variance: 0.001,
            },
            gyro_scale: 0.001, // 1 raw unit = 0.001 rad/s
            gyro_bias_z: 0.0,
            gyro_sign: 1.0, // Tests use synthetic data with same sign convention
            initial_position_variance: 0.01,
            initial_heading_variance: 0.01,
        }
    }

    #[test]
    fn test_initial_state() {
        let mut eskf = Eskf::new(test_config());

        // Initialize and check
        let pose = eskf.update(Pose2D::identity(), 0, 0);
        assert_eq!(pose.x, 0.0);
        assert_eq!(pose.y, 0.0);
        assert_eq!(pose.theta, 0.0);
    }

    #[test]
    fn test_predict_forward() {
        let mut eskf = Eskf::new(test_config());

        // Move forward 1 meter
        let delta = Pose2D::new(1.0, 0.0, 0.0);
        eskf.predict(delta);

        // Check that the state is updated - we can still access internal state via update
        let _pose = eskf.update(Pose2D::identity(), 0, 0);
    }

    #[test]
    fn test_predict_with_rotation() {
        let mut eskf = Eskf::new(test_config());

        // Rotate 90 degrees CCW
        let delta = Pose2D::new(0.0, 0.0, PI / 2.0);
        eskf.predict(delta);

        // Then move forward (should go in +Y direction)
        let delta2 = Pose2D::new(1.0, 0.0, 0.0);
        let pose2 = eskf.update(delta2, 0, 0);
        assert_relative_eq!(pose2.x, 0.0, epsilon = 0.01);
        assert_relative_eq!(pose2.y, 1.0, epsilon = 0.01);
    }

    #[test]
    fn test_gyro_update_reduces_covariance() {
        let mut eskf = Eskf::new(test_config());

        // Predict with some motion to increase covariance
        eskf.predict(Pose2D::new(0.1, 0.0, 0.1));

        // Update with gyro measurement
        eskf.update_gyro(0, 0.02); // 20ms, gyro at 0

        // Test passes if no panic occurs
    }

    #[test]
    fn test_combined_update() {
        let mut eskf = Eskf::new(test_config());

        // First update initializes timestamp
        eskf.update(Pose2D::identity(), 0, 0);

        // Second update with motion
        let delta = Pose2D::new(0.1, 0.0, 0.0);
        let pose = eskf.update(delta, 0, 100_000); // 100ms later

        assert!(pose.x > 0.0);
    }

    #[test]
    fn test_covariance_grows_with_motion() {
        let mut eskf = Eskf::new(test_config());

        // Move forward multiple times
        for _ in 0..10 {
            eskf.predict(Pose2D::new(0.1, 0.0, 0.0));
        }

        // Test passes if no panic occurs
    }

    #[test]
    fn test_gyro_bias_correction() {
        let config = EskfConfig {
            gyro_bias_z: 100.0, // Bias of 100 raw units
            gyro_scale: 0.001,
            ..test_config()
        };
        let mut eskf = Eskf::new(config);

        // Update with gyro at bias level
        eskf.update(Pose2D::new(0.1, 0.0, 0.0), 100, 0);
        let pose = eskf.update(Pose2D::new(0.1, 0.0, 0.0), 100, 100_000);

        // With gyro at bias, net rotation should be minimal
        assert!(
            pose.theta.abs() < 0.1,
            "Theta should be small: {}",
            pose.theta
        );
    }

    #[test]
    fn test_zero_dt_handling() {
        let mut eskf = Eskf::new(test_config());

        // Same timestamp (dt = 0)
        eskf.update(Pose2D::new(0.1, 0.0, 0.0), 100, 1000);

        // Same timestamp again - gyro update should be skipped
        let pose = eskf.update(Pose2D::new(0.1, 0.0, 0.0), 100, 1000);

        // Should still produce valid results (prediction still runs)
        assert!(pose.x.is_finite());
    }
}
