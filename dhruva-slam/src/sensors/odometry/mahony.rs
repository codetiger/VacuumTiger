//! Mahony AHRS filter with automatic gyro bias calibration.
//!
//! This module provides a robust orientation estimator that:
//! - Automatically calibrates gyro bias during an initial stationary period
//! - Fuses gyroscope and accelerometer/gravity data using the Mahony algorithm
//! - Outputs Euler angles (roll, pitch, yaw) or quaternion orientation
//!
//! # Why Mahony AHRS?
//!
//! The Mahony filter is computationally efficient and well-suited for embedded systems.
//! It uses proportional-integral feedback to correct gyro integration drift using
//! gravity vector measurements.
//!
//! # Axis Convention (ROS REP-103)
//!
//! SangamIO provides IMU data in ROS REP-103 standard frame:
//! - `gyro_raw[0]`: Roll rate (X axis rotation, left/right tilt)
//! - `gyro_raw[1]`: Pitch rate (Y axis rotation, nose up/down)
//! - `gyro_raw[2]`: **Yaw rate (Z axis rotation, heading change, CCW positive)**
//!
//! For 2D navigation, `gyro_raw[2]` is the primary axis of interest.
//! See `utils::constants::GYRO_YAW_INDEX`.
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::sensors::odometry::{MahonyAhrs, MahonyConfig, RawImuData};
//!
//! let config = MahonyConfig::default();
//! let mut ahrs = MahonyAhrs::new(config);
//!
//! // Process IMU data at sensor rate (~110Hz)
//! loop {
//!     let imu = RawImuData::new(gyro_raw, tilt_raw);
//!     let (roll, pitch, yaw) = ahrs.update(&imu, timestamp_us);
//!
//!     if ahrs.is_calibrated() {
//!         // Use orientation for odometry fusion
//!     }
//! }
//! ```

use std::f32::consts::PI;

/// Gyroscope scale factor for CRL-200S IMU.
///
/// Calibrated using encoder ground truth from rotation bags.
/// Raw gyro value of ~3740 at 38.33°/s encoder-measured rotation gives:
/// 38.33 / 3740 = 0.01025 deg/s per LSB = 0.000179 rad/s per LSB
pub const CRL200S_GYRO_SCALE: f32 = 0.01025 * (PI / 180.0);

/// Gravity vector scale for LP-filtered tilt data.
pub const GRAVITY_SCALE: f32 = 1.0 / 1000.0;

/// Configuration for the Mahony AHRS filter.
#[derive(Debug, Clone, Copy)]
pub struct MahonyConfig {
    /// Proportional gain for error correction.
    /// Higher values = faster correction but more noise sensitivity.
    /// Typical range: 0.5 - 10.0
    pub kp: f32,

    /// Integral gain for bias estimation.
    /// Higher values = faster bias adaptation.
    /// Typical range: 0.0 - 0.5
    pub ki: f32,

    /// Gyroscope scale factor (raw units to rad/s).
    pub gyro_scale: f32,

    /// Gravity vector scale factor.
    pub gravity_scale: f32,

    /// Number of samples for initial bias calibration.
    /// At 110Hz, 330 samples = 3 seconds.
    pub calibration_samples: u32,

    /// Sample rate in Hz (used for fixed dt if timestamps unavailable).
    pub sample_rate_hz: f32,
}

impl Default for MahonyConfig {
    fn default() -> Self {
        Self {
            kp: 5.0, // Proportional gain (responsive but stable)
            ki: 0.1, // Integral gain (slow bias adaptation)
            gyro_scale: CRL200S_GYRO_SCALE,
            gravity_scale: GRAVITY_SCALE,
            calibration_samples: 330, // ~3 seconds at 110Hz
            sample_rate_hz: 110.0,
        }
    }
}

/// Quaternion representation [w, x, y, z].
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::identity()
    }
}

impl Quaternion {
    /// Create identity quaternion (no rotation).
    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Normalize the quaternion to unit length.
    pub fn normalize(&mut self) {
        let norm = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if norm > 1e-10 {
            self.w /= norm;
            self.x /= norm;
            self.y /= norm;
            self.z /= norm;
        }
    }

    /// Convert quaternion to Euler angles (ZYX convention).
    /// Returns (roll, pitch, yaw) in radians.
    pub fn to_euler(&self) -> (f32, f32, f32) {
        // Roll (X axis rotation)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (Y axis rotation)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp.abs() >= 1.0 {
            (PI / 2.0).copysign(sinp) // Use 90 degrees if out of range
        } else {
            sinp.asin()
        };

        // Yaw (Z axis rotation)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }
}

/// Euler angles in radians.
#[derive(Debug, Clone, Copy, Default)]
pub struct EulerAngles {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl EulerAngles {
    /// Convert to degrees.
    pub fn to_degrees(&self) -> (f32, f32, f32) {
        (
            self.roll * 180.0 / PI,
            self.pitch * 180.0 / PI,
            self.yaw * 180.0 / PI,
        )
    }
}

/// Raw IMU data from sensor (before calibration/scaling).
#[derive(Debug, Clone, Copy, Default)]
pub struct RawImuData {
    /// Raw gyroscope X (roll rate) in sensor units
    pub gyro_x: i16,
    /// Raw gyroscope Y (pitch rate) in sensor units
    pub gyro_y: i16,
    /// Raw gyroscope Z (yaw rate) in sensor units
    pub gyro_z: i16,
    /// LP-filtered gravity/tilt X
    pub tilt_x: i16,
    /// LP-filtered gravity/tilt Y
    pub tilt_y: i16,
    /// LP-filtered gravity/tilt Z
    pub tilt_z: i16,
}

impl RawImuData {
    /// Create new raw IMU data.
    pub fn new(gyro: [i16; 3], tilt: [i16; 3]) -> Self {
        Self {
            gyro_x: gyro[0],
            gyro_y: gyro[1],
            gyro_z: gyro[2],
            tilt_x: tilt[0],
            tilt_y: tilt[1],
            tilt_z: tilt[2],
        }
    }
}

/// Calibrated IMU data in physical units.
#[derive(Debug, Clone, Copy, Default)]
pub struct CalibratedImuData {
    /// Gyroscope X (roll rate) in rad/s
    pub gx: f32,
    /// Gyroscope Y (pitch rate) in rad/s
    pub gy: f32,
    /// Gyroscope Z (yaw rate) in rad/s
    pub gz: f32,
    /// Accelerometer/gravity X (normalized or m/s²)
    pub ax: f32,
    /// Accelerometer/gravity Y (normalized or m/s²)
    pub ay: f32,
    /// Accelerometer/gravity Z (normalized or m/s²)
    pub az: f32,
}

impl CalibratedImuData {
    /// Create new calibrated IMU data.
    pub fn new(gyro: [f32; 3], accel: [f32; 3]) -> Self {
        Self {
            gx: gyro[0],
            gy: gyro[1],
            gz: gyro[2],
            ax: accel[0],
            ay: accel[1],
            az: accel[2],
        }
    }
}

/// Gyro bias calibration state.
#[derive(Debug)]
struct BiasCalibration {
    samples: Vec<(i16, i16, i16)>,
    target_samples: u32,
    bias: [f32; 3],
    calibrated: bool,
}

impl BiasCalibration {
    fn new(target_samples: u32) -> Self {
        Self {
            samples: Vec::with_capacity(target_samples as usize),
            target_samples,
            bias: [0.0; 3],
            calibrated: false,
        }
    }

    /// Add a sample and return true if calibration is complete.
    fn add_sample(&mut self, gx: i16, gy: i16, gz: i16) -> bool {
        if self.calibrated {
            return true;
        }

        self.samples.push((gx, gy, gz));

        if self.samples.len() >= self.target_samples as usize {
            // Calculate average bias
            let n = self.samples.len() as f32;
            let sum: (f64, f64, f64) = self.samples.iter().fold((0.0, 0.0, 0.0), |acc, s| {
                (acc.0 + s.0 as f64, acc.1 + s.1 as f64, acc.2 + s.2 as f64)
            });
            self.bias[0] = (sum.0 / n as f64) as f32;
            self.bias[1] = (sum.1 / n as f64) as f32;
            self.bias[2] = (sum.2 / n as f64) as f32;
            self.calibrated = true;
            self.samples.clear(); // Free memory
            self.samples.shrink_to_fit();
            true
        } else {
            false
        }
    }

    fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    fn get_bias(&self) -> [f32; 3] {
        self.bias
    }

    /// Manually set bias (e.g., from a pre-recorded static bag).
    fn set_bias(&mut self, bias: [f32; 3]) {
        self.bias = bias;
        self.calibrated = true;
        self.samples.clear();
        self.samples.shrink_to_fit();
    }

    fn reset(&mut self) {
        self.samples.clear();
        self.bias = [0.0; 3];
        self.calibrated = false;
    }
}

/// Mahony AHRS (Attitude and Heading Reference System) filter.
///
/// Provides robust orientation estimation by fusing gyroscope and
/// accelerometer/gravity data with automatic bias calibration.
#[derive(Debug)]
pub struct MahonyAhrs {
    config: MahonyConfig,

    /// Quaternion orientation [w, x, y, z].
    q: Quaternion,

    /// Integral error for PI controller.
    integral_error: [f32; 3],

    /// Gyro bias calibration.
    bias_cal: BiasCalibration,

    /// Last timestamp for dt calculation.
    last_timestamp_us: Option<u64>,

    /// Fixed dt based on sample rate (fallback).
    fixed_dt: f32,
}

impl MahonyAhrs {
    /// Create a new Mahony AHRS filter.
    pub fn new(config: MahonyConfig) -> Self {
        let fixed_dt = 1.0 / config.sample_rate_hz;
        Self {
            bias_cal: BiasCalibration::new(config.calibration_samples),
            config,
            q: Quaternion::identity(),
            integral_error: [0.0; 3],
            last_timestamp_us: None,
            fixed_dt,
        }
    }

    /// Check if bias calibration is complete.
    pub fn is_calibrated(&self) -> bool {
        self.bias_cal.is_calibrated()
    }

    /// Get the current gyro bias values.
    pub fn gyro_bias(&self) -> [f32; 3] {
        self.bias_cal.get_bias()
    }

    /// Manually set gyro bias (skip calibration phase).
    pub fn set_gyro_bias(&mut self, bias: [f32; 3]) {
        self.bias_cal.set_bias(bias);
    }

    /// Get the current quaternion orientation.
    pub fn quaternion(&self) -> Quaternion {
        self.q
    }

    /// Get the current Euler angles.
    pub fn euler(&self) -> EulerAngles {
        let (roll, pitch, yaw) = self.q.to_euler();
        EulerAngles { roll, pitch, yaw }
    }

    /// Get yaw angle in radians.
    pub fn yaw(&self) -> f32 {
        self.euler().yaw
    }

    /// Reset orientation to identity.
    pub fn reset(&mut self) {
        self.q = Quaternion::identity();
        self.integral_error = [0.0; 3];
        self.last_timestamp_us = None;
    }

    /// Reset and restart bias calibration.
    pub fn recalibrate(&mut self) {
        self.reset();
        self.bias_cal.reset();
    }

    /// Update the filter with new raw IMU data.
    ///
    /// # Arguments
    ///
    /// * `imu` - Raw IMU data (gyro and tilt in sensor units, ROS REP-103 frame)
    /// * `timestamp_us` - Timestamp in microseconds
    ///
    /// # Returns
    ///
    /// Euler angles (roll, pitch, yaw) in radians, or (0, 0, 0) during calibration.
    pub fn update(&mut self, imu: &RawImuData, timestamp_us: u64) -> (f32, f32, f32) {
        // Calibration phase - collect bias samples
        if !self.bias_cal.is_calibrated() {
            self.bias_cal.add_sample(imu.gyro_x, imu.gyro_y, imu.gyro_z);
            self.last_timestamp_us = Some(timestamp_us);
            return (0.0, 0.0, 0.0);
        }

        // Calculate dt
        let dt = if let Some(last_ts) = self.last_timestamp_us {
            if timestamp_us > last_ts {
                (timestamp_us - last_ts) as f32 / 1_000_000.0
            } else {
                self.fixed_dt // Fallback for timestamp issues
            }
        } else {
            self.fixed_dt
        };
        self.last_timestamp_us = Some(timestamp_us);

        // Skip update if dt is too large (data gap) or zero
        if dt <= 0.0 || dt > 0.1 {
            return self.q.to_euler();
        }

        let bias = self.bias_cal.get_bias();

        // Remove bias and convert to rad/s
        // SangamIO provides gyro data already in ROS REP-103 frame:
        // gyro_x = Roll rate (X axis rotation)
        // gyro_y = Pitch rate (Y axis rotation)
        // gyro_z = Yaw rate (Z axis rotation)
        let gx = (imu.gyro_x as f32 - bias[0]) * self.config.gyro_scale; // Roll
        let gy = (imu.gyro_y as f32 - bias[1]) * self.config.gyro_scale; // Pitch
        let gz = (imu.gyro_z as f32 - bias[2]) * self.config.gyro_scale; // Yaw

        // Get current quaternion
        let q0 = self.q.w;
        let q1 = self.q.x;
        let q2 = self.q.y;
        let q3 = self.q.z;

        // Normalize gravity vector
        let mut ax = imu.tilt_x as f32 * self.config.gravity_scale;
        let mut ay = imu.tilt_y as f32 * self.config.gravity_scale;
        let mut az = imu.tilt_z as f32 * self.config.gravity_scale;

        let accel_norm = (ax * ax + ay * ay + az * az).sqrt();

        // Apply Mahony correction if we have valid gravity data
        let (mut gx_corrected, mut gy_corrected, mut gz_corrected) = (gx, gy, gz);

        if accel_norm > 0.01 {
            ax /= accel_norm;
            ay /= accel_norm;
            az /= accel_norm;

            // Estimated gravity direction from quaternion
            // v = [2(q1q3 - q0q2), 2(q0q1 + q2q3), q0² - q1² - q2² + q3²]
            let vx = 2.0 * (q1 * q3 - q0 * q2);
            let vy = 2.0 * (q0 * q1 + q2 * q3);
            let vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

            // Error is cross product between estimated and measured gravity
            let ex = ay * vz - az * vy;
            let ey = az * vx - ax * vz;
            let ez = ax * vy - ay * vx;

            // Integral feedback (for bias estimation)
            if self.config.ki > 0.0 {
                self.integral_error[0] += ex * dt;
                self.integral_error[1] += ey * dt;
                self.integral_error[2] += ez * dt;
            }

            // Apply proportional and integral feedback
            gx_corrected += self.config.kp * ex + self.config.ki * self.integral_error[0];
            gy_corrected += self.config.kp * ey + self.config.ki * self.integral_error[1];
            gz_corrected += self.config.kp * ez + self.config.ki * self.integral_error[2];
        }

        // Quaternion derivative (integration of angular velocity)
        let dq0 = 0.5 * (-q1 * gx_corrected - q2 * gy_corrected - q3 * gz_corrected);
        let dq1 = 0.5 * (q0 * gx_corrected + q2 * gz_corrected - q3 * gy_corrected);
        let dq2 = 0.5 * (q0 * gy_corrected - q1 * gz_corrected + q3 * gx_corrected);
        let dq3 = 0.5 * (q0 * gz_corrected + q1 * gy_corrected - q2 * gx_corrected);

        // Integrate
        self.q.w += dq0 * dt;
        self.q.x += dq1 * dt;
        self.q.y += dq2 * dt;
        self.q.z += dq3 * dt;

        // Normalize quaternion
        self.q.normalize();

        self.q.to_euler()
    }

    /// Update with pre-calibrated IMU data in physical units.
    ///
    /// Use this if you've already applied bias correction and scaling.
    ///
    /// # Arguments
    ///
    /// * `imu` - Calibrated IMU data (gyro in rad/s, accel normalized or in m/s²)
    /// * `dt` - Time delta in seconds
    pub fn update_calibrated(&mut self, imu: &CalibratedImuData, dt: f32) -> (f32, f32, f32) {
        if dt <= 0.0 || dt > 0.1 {
            return self.q.to_euler();
        }

        let q0 = self.q.w;
        let q1 = self.q.x;
        let q2 = self.q.y;
        let q3 = self.q.z;

        let (mut gx, mut gy, mut gz) = (imu.gx, imu.gy, imu.gz);

        // Normalize accelerometer
        let accel_norm = (imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az).sqrt();
        if accel_norm > 0.01 {
            let ax_n = imu.ax / accel_norm;
            let ay_n = imu.ay / accel_norm;
            let az_n = imu.az / accel_norm;

            // Estimated gravity from quaternion
            let vx = 2.0 * (q1 * q3 - q0 * q2);
            let vy = 2.0 * (q0 * q1 + q2 * q3);
            let vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

            // Error (cross product)
            let ex = ay_n * vz - az_n * vy;
            let ey = az_n * vx - ax_n * vz;
            let ez = ax_n * vy - ay_n * vx;

            if self.config.ki > 0.0 {
                self.integral_error[0] += ex * dt;
                self.integral_error[1] += ey * dt;
                self.integral_error[2] += ez * dt;
            }

            gx += self.config.kp * ex + self.config.ki * self.integral_error[0];
            gy += self.config.kp * ey + self.config.ki * self.integral_error[1];
            gz += self.config.kp * ez + self.config.ki * self.integral_error[2];
        }

        // Quaternion derivative
        let dq0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        let dq1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        let dq2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        let dq3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        self.q.w += dq0 * dt;
        self.q.x += dq1 * dt;
        self.q.y += dq2 * dt;
        self.q.z += dq3 * dt;

        self.q.normalize();
        self.q.to_euler()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_quaternion_identity() {
        let q = Quaternion::identity();
        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 0.0);
        assert_eq!(q.y, 0.0);
        assert_eq!(q.z, 0.0);
    }

    #[test]
    fn test_quaternion_to_euler_identity() {
        let q = Quaternion::identity();
        let (roll, pitch, yaw) = q.to_euler();
        assert_relative_eq!(roll, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pitch, 0.0, epsilon = 1e-6);
        assert_relative_eq!(yaw, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_bias_calibration() {
        let mut cal = BiasCalibration::new(10);

        // Add samples with known bias
        for _ in 0..9 {
            assert!(!cal.add_sample(100, 50, -30));
            assert!(!cal.is_calibrated());
        }

        // Last sample completes calibration
        assert!(cal.add_sample(100, 50, -30));
        assert!(cal.is_calibrated());

        let bias = cal.get_bias();
        assert_relative_eq!(bias[0], 100.0, epsilon = 0.1);
        assert_relative_eq!(bias[1], 50.0, epsilon = 0.1);
        assert_relative_eq!(bias[2], -30.0, epsilon = 0.1);
    }

    #[test]
    fn test_mahony_calibration_phase() {
        let config = MahonyConfig {
            calibration_samples: 10,
            ..Default::default()
        };
        let mut ahrs = MahonyAhrs::new(config);

        // During calibration, should return zeros
        let imu = RawImuData::new([100, 50, -30], [0, 0, 1000]);
        for i in 0..10 {
            let (r, p, y) = ahrs.update(&imu, i * 2000);
            if i < 9 {
                assert!(!ahrs.is_calibrated());
            }
            assert_eq!(r, 0.0);
            assert_eq!(p, 0.0);
            assert_eq!(y, 0.0);
        }

        // After calibration
        assert!(ahrs.is_calibrated());
        let bias = ahrs.gyro_bias();
        assert_relative_eq!(bias[0], 100.0, epsilon = 0.1);
    }

    #[test]
    fn test_mahony_stationary() {
        let config = MahonyConfig {
            calibration_samples: 5,
            ..Default::default()
        };
        let mut ahrs = MahonyAhrs::new(config);

        // Calibration phase
        let imu = RawImuData::new([0, 0, 0], [0, 0, 1000]);
        for i in 0..5 {
            ahrs.update(&imu, i * 2000);
        }

        // Stationary updates (gyro at bias = 0)
        for i in 5..20 {
            let (roll, pitch, yaw) = ahrs.update(&imu, i * 2000);
            // Should stay near zero when stationary
            assert!(roll.abs() < 0.1);
            assert!(pitch.abs() < 0.1);
            assert!(yaw.abs() < 0.1);
        }
    }

    #[test]
    fn test_set_gyro_bias() {
        let mut ahrs = MahonyAhrs::new(MahonyConfig::default());

        // Set bias manually
        ahrs.set_gyro_bias([160.0, -10.0, 20.0]);

        assert!(ahrs.is_calibrated());
        let bias = ahrs.gyro_bias();
        assert_eq!(bias[0], 160.0);
        assert_eq!(bias[1], -10.0);
        assert_eq!(bias[2], 20.0);
    }

    #[test]
    fn test_euler_to_degrees() {
        let euler = EulerAngles {
            roll: PI / 2.0,
            pitch: PI / 4.0,
            yaw: PI,
        };
        let (r, p, y) = euler.to_degrees();
        assert_relative_eq!(r, 90.0, epsilon = 0.01);
        assert_relative_eq!(p, 45.0, epsilon = 0.01);
        assert_relative_eq!(y, 180.0, epsilon = 0.01);
    }

    #[test]
    fn test_recalibrate() {
        let config = MahonyConfig {
            calibration_samples: 5,
            ..Default::default()
        };
        let mut ahrs = MahonyAhrs::new(config);

        // Complete calibration
        let imu1 = RawImuData::new([100, 0, 0], [0, 0, 1000]);
        for i in 0..5 {
            ahrs.update(&imu1, i * 2000);
        }
        assert!(ahrs.is_calibrated());
        assert_relative_eq!(ahrs.gyro_bias()[0], 100.0, epsilon = 0.1);

        // Recalibrate
        ahrs.recalibrate();
        assert!(!ahrs.is_calibrated());

        // New calibration with different bias
        let imu2 = RawImuData::new([50, 0, 0], [0, 0, 1000]);
        for i in 0..5 {
            ahrs.update(&imu2, (i + 10) * 2000);
        }
        assert!(ahrs.is_calibrated());
        assert_relative_eq!(ahrs.gyro_bias()[0], 50.0, epsilon = 0.1);
    }
}
