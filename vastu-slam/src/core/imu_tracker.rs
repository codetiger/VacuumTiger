//! IMU tracking with gravity estimation.
//!
//! This module implements Cartographer-style IMU tracking using an
//! exponential filter for gravity estimation and gyroscope integration
//! for orientation tracking.
//!
//! # Algorithm
//!
//! The ImuTracker uses two main components:
//!
//! 1. **Gravity Estimation**: Exponential smoothing filter on accelerometer
//!    readings to estimate the gravity direction. This allows detecting
//!    when the robot is tilted.
//!
//! 2. **Orientation Integration**: Gyroscope readings are integrated over
//!    time to track the robot's yaw angle.
//!
//! # Cartographer Approach
//!
//! This follows Google Cartographer's approach:
//! - Uses exponential filter instead of Kalman filter (simpler, works well)
//! - Time constant balances noise rejection vs responsiveness
//! - For 2D SLAM, primarily tracks yaw (rotation around gravity axis)
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_slam::core::{ImuTracker, ImuTrackerConfig, ImuMeasurement};
//!
//! let config = ImuTrackerConfig::default();
//! let mut tracker = ImuTracker::new(config);
//!
//! // Feed IMU measurements at ~110 Hz
//! tracker.add_imu(&imu_measurement);
//!
//! // Get current yaw estimate
//! let yaw = tracker.yaw();
//! ```

use serde::{Deserialize, Serialize};

use super::ImuMeasurement;

/// Configuration for the IMU tracker.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ImuTrackerConfig {
    /// Time constant for gravity estimation filter (seconds).
    ///
    /// Higher values make the filter slower to respond but more stable.
    /// Lower values make it faster but noisier.
    ///
    /// Default: 10.0 seconds (Cartographer default)
    pub gravity_time_constant: f32,

    /// Initial gravity vector assumption [x, y, z] in m/s².
    ///
    /// For a level robot: [0, 0, 9.81]
    /// Default: [0.0, 0.0, 9.81]
    pub initial_gravity: [f32; 3],
}

impl Default for ImuTrackerConfig {
    fn default() -> Self {
        Self {
            gravity_time_constant: 10.0,
            initial_gravity: [0.0, 0.0, 9.81],
        }
    }
}

/// IMU tracker for gravity estimation and orientation tracking.
///
/// Uses Cartographer-style exponential smoothing for gravity and
/// gyroscope integration for orientation.
#[derive(Clone, Debug)]
pub struct ImuTracker {
    /// Configuration
    config: ImuTrackerConfig,

    /// Estimated gravity vector (not normalized, in m/s²)
    gravity: [f32; 3],

    /// Current yaw angle (radians, CCW positive from initial heading)
    yaw: f32,

    /// Integrated pitch from gyroscope (for tilt compensation)
    pitch: f32,

    /// Integrated roll from gyroscope (for tilt compensation)
    roll: f32,

    /// Last IMU timestamp for delta calculation
    last_timestamp_us: Option<u64>,

    /// Number of IMU measurements processed
    measurement_count: u64,
}

impl ImuTracker {
    /// Create a new IMU tracker with the given configuration.
    pub fn new(config: ImuTrackerConfig) -> Self {
        let gravity = config.initial_gravity;
        Self {
            config,
            gravity,
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
            last_timestamp_us: None,
            measurement_count: 0,
        }
    }

    /// Process an IMU measurement.
    ///
    /// Updates the gravity estimate using exponential smoothing and
    /// integrates the gyroscope for orientation tracking.
    pub fn add_imu(&mut self, imu: &ImuMeasurement) {
        // Calculate time delta
        let dt = if let Some(last_ts) = self.last_timestamp_us {
            if imu.timestamp_us <= last_ts {
                // Non-monotonic timestamp, skip
                return;
            }
            (imu.timestamp_us - last_ts) as f32 / 1_000_000.0 // microseconds to seconds
        } else {
            // First measurement, just initialize
            self.last_timestamp_us = Some(imu.timestamp_us);
            self.gravity = imu.accel();
            self.measurement_count += 1;
            return;
        };

        // Sanity check on dt (reject unreasonably large gaps)
        if dt > 1.0 {
            // Gap too large, reset tracking
            self.last_timestamp_us = Some(imu.timestamp_us);
            self.gravity = imu.accel();
            self.measurement_count += 1;
            return;
        }

        // Update gravity using exponential filter
        // alpha = exp(-dt / time_constant)
        // gravity = alpha * gravity + (1 - alpha) * accel
        let alpha = (-dt / self.config.gravity_time_constant).exp();
        let accel = imu.accel();
        self.gravity[0] = alpha * self.gravity[0] + (1.0 - alpha) * accel[0];
        self.gravity[1] = alpha * self.gravity[1] + (1.0 - alpha) * accel[1];
        self.gravity[2] = alpha * self.gravity[2] + (1.0 - alpha) * accel[2];

        // Integrate gyroscope for orientation
        // Simple Euler integration (sufficient for small dt)
        self.roll += imu.gyro_x * dt;
        self.pitch += imu.gyro_y * dt;
        self.yaw += imu.gyro_z * dt;

        // Normalize yaw to [-PI, PI]
        self.yaw = normalize_angle(self.yaw);

        self.last_timestamp_us = Some(imu.timestamp_us);
        self.measurement_count += 1;
    }

    /// Get the current yaw angle estimate (radians, CCW positive).
    ///
    /// This is the primary output for 2D SLAM - the rotation around
    /// the gravity axis.
    #[inline]
    pub fn yaw(&self) -> f32 {
        self.yaw
    }

    /// Get the current pitch angle estimate (radians).
    #[inline]
    pub fn pitch(&self) -> f32 {
        self.pitch
    }

    /// Get the current roll angle estimate (radians).
    #[inline]
    pub fn roll(&self) -> f32 {
        self.roll
    }

    /// Get the estimated gravity vector [x, y, z] in m/s².
    ///
    /// For a level robot, this should be approximately [0, 0, 9.81].
    #[inline]
    pub fn gravity(&self) -> [f32; 3] {
        self.gravity
    }

    /// Get the normalized gravity direction vector.
    ///
    /// Returns a unit vector pointing in the gravity direction.
    pub fn gravity_direction(&self) -> [f32; 3] {
        let mag = (self.gravity[0] * self.gravity[0]
            + self.gravity[1] * self.gravity[1]
            + self.gravity[2] * self.gravity[2])
            .sqrt();

        if mag > 0.001 {
            [
                self.gravity[0] / mag,
                self.gravity[1] / mag,
                self.gravity[2] / mag,
            ]
        } else {
            [0.0, 0.0, 1.0] // Default to vertical if magnitude is too small
        }
    }

    /// Get the tilt angle (radians) - angle between gravity and vertical.
    ///
    /// Returns 0.0 when the robot is level, increases as it tilts.
    pub fn tilt_angle(&self) -> f32 {
        let dir = self.gravity_direction();
        // Tilt is the angle between gravity direction and Z axis
        // cos(theta) = dot(gravity_dir, [0,0,1]) = gravity_dir[2]
        dir[2].clamp(-1.0, 1.0).acos()
    }

    /// Check if the robot is approximately level.
    ///
    /// Returns true if tilt angle is less than the given threshold (radians).
    pub fn is_level(&self, threshold_radians: f32) -> bool {
        self.tilt_angle() < threshold_radians
    }

    /// Get the number of IMU measurements processed.
    #[inline]
    pub fn measurement_count(&self) -> u64 {
        self.measurement_count
    }

    /// Reset the tracker to initial state.
    pub fn reset(&mut self) {
        self.gravity = self.config.initial_gravity;
        self.yaw = 0.0;
        self.pitch = 0.0;
        self.roll = 0.0;
        self.last_timestamp_us = None;
        self.measurement_count = 0;
    }

    /// Reset only the yaw angle (useful when aligning with odometry).
    pub fn reset_yaw(&mut self, new_yaw: f32) {
        self.yaw = normalize_angle(new_yaw);
    }

    /// Advance the orientation estimate without a new IMU measurement.
    ///
    /// Uses the last known angular velocity to extrapolate forward.
    /// Useful for predicting pose between IMU samples.
    pub fn advance(&mut self, dt_seconds: f32, angular_velocity_z: f32) {
        self.yaw += angular_velocity_z * dt_seconds;
        self.yaw = normalize_angle(self.yaw);
    }

    /// Get the last timestamp processed.
    pub fn last_timestamp_us(&self) -> Option<u64> {
        self.last_timestamp_us
    }
}

/// Normalize angle to [-PI, PI] range.
#[inline]
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_tracker_creation() {
        let tracker = ImuTracker::new(ImuTrackerConfig::default());
        assert_eq!(tracker.yaw(), 0.0);
        assert_eq!(tracker.measurement_count(), 0);
    }

    #[test]
    fn test_gravity_estimation_level() {
        let mut tracker = ImuTracker::new(ImuTrackerConfig::default());

        // Feed level readings (gravity in Z)
        for i in 0..100 {
            let imu = ImuMeasurement::new(
                i * 10000, // 10ms intervals
                [0.0, 0.0, 9.81],
                [0.0, 0.0, 0.0],
            );
            tracker.add_imu(&imu);
        }

        let gravity = tracker.gravity();
        assert!((gravity[0]).abs() < 0.1);
        assert!((gravity[1]).abs() < 0.1);
        assert!((gravity[2] - 9.81).abs() < 0.1);
        assert!(tracker.is_level(0.1)); // Less than 0.1 rad tilt
    }

    #[test]
    fn test_yaw_integration() {
        let mut tracker = ImuTracker::new(ImuTrackerConfig::default());

        // Rotate at 1 rad/s for 1 second (100 samples at 10ms)
        for i in 0..100 {
            let imu = ImuMeasurement::new(
                i * 10000, // 10ms intervals
                [0.0, 0.0, 9.81],
                [0.0, 0.0, 1.0], // 1 rad/s CCW
            );
            tracker.add_imu(&imu);
        }

        // Should have rotated about 1 radian (minus first sample)
        let expected_yaw = 0.99; // 99 * 0.01s * 1 rad/s
        assert!(
            (tracker.yaw() - expected_yaw).abs() < 0.1,
            "yaw = {}, expected ~{}",
            tracker.yaw(),
            expected_yaw
        );
    }

    #[test]
    fn test_yaw_integration_negative() {
        let mut tracker = ImuTracker::new(ImuTrackerConfig::default());

        // Rotate at -0.5 rad/s (CW) for 1 second
        for i in 0..100 {
            let imu = ImuMeasurement::new(
                i * 10000,
                [0.0, 0.0, 9.81],
                [0.0, 0.0, -0.5], // -0.5 rad/s CW
            );
            tracker.add_imu(&imu);
        }

        let expected_yaw = -0.495; // 99 * 0.01s * -0.5 rad/s
        assert!(
            (tracker.yaw() - expected_yaw).abs() < 0.1,
            "yaw = {}, expected ~{}",
            tracker.yaw(),
            expected_yaw
        );
    }

    #[test]
    fn test_reset_yaw() {
        let mut tracker = ImuTracker::new(ImuTrackerConfig::default());

        // Integrate some yaw
        for i in 0..50 {
            let imu = ImuMeasurement::new(i * 10000, [0.0, 0.0, 9.81], [0.0, 0.0, 1.0]);
            tracker.add_imu(&imu);
        }

        assert!(tracker.yaw().abs() > 0.1);

        // Reset yaw
        tracker.reset_yaw(0.0);
        assert_eq!(tracker.yaw(), 0.0);
    }

    #[test]
    fn test_advance() {
        let mut tracker = ImuTracker::new(ImuTrackerConfig::default());

        // Initialize with one sample
        let imu = ImuMeasurement::new(0, [0.0, 0.0, 9.81], [0.0, 0.0, 0.0]);
        tracker.add_imu(&imu);

        // Advance by 0.5 seconds at 1 rad/s
        tracker.advance(0.5, 1.0);

        assert!((tracker.yaw() - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_tilt_detection() {
        let mut tracker = ImuTracker::new(ImuTrackerConfig {
            gravity_time_constant: 0.1, // Fast response for test
            initial_gravity: [0.0, 0.0, 9.81],
        });

        // Feed tilted readings (gravity in Y direction = 90 degree tilt)
        for i in 0..100 {
            let imu = ImuMeasurement::new(i * 10000, [0.0, 9.81, 0.0], [0.0, 0.0, 0.0]);
            tracker.add_imu(&imu);
        }

        // Should detect significant tilt
        assert!(tracker.tilt_angle() > 1.0); // More than 1 radian tilt
        assert!(!tracker.is_level(0.5));
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-6);
        assert!((normalize_angle(std::f32::consts::PI) - std::f32::consts::PI).abs() < 1e-6);
        assert!(
            (normalize_angle(2.0 * std::f32::consts::PI) - 0.0).abs() < 1e-6,
            "2PI should normalize to 0"
        );
        assert!(
            (normalize_angle(-2.0 * std::f32::consts::PI) - 0.0).abs() < 1e-6,
            "-2PI should normalize to 0"
        );
        assert!(
            (normalize_angle(3.0 * std::f32::consts::PI) - std::f32::consts::PI).abs() < 1e-6,
            "3PI should normalize to PI"
        );
    }

    #[test]
    fn test_non_monotonic_timestamp() {
        let mut tracker = ImuTracker::new(ImuTrackerConfig::default());

        tracker.add_imu(&ImuMeasurement::new(
            100000,
            [0.0, 0.0, 9.81],
            [0.0, 0.0, 1.0],
        ));
        tracker.add_imu(&ImuMeasurement::new(
            200000,
            [0.0, 0.0, 9.81],
            [0.0, 0.0, 1.0],
        ));

        let yaw_before = tracker.yaw();
        let count_before = tracker.measurement_count();

        // Non-monotonic timestamp should be ignored
        tracker.add_imu(&ImuMeasurement::new(
            150000,
            [0.0, 0.0, 9.81],
            [0.0, 0.0, 10.0],
        ));

        assert_eq!(tracker.yaw(), yaw_before);
        assert_eq!(tracker.measurement_count(), count_before);
    }
}
