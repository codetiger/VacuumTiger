//! IMU calibration constants.
//!
//! Hardware-specific calibration values for the CRL-200S robot.

use std::f32::consts::PI;

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
