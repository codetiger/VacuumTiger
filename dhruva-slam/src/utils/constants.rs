//! Robot constants for CRL-200S vacuum cleaner.
//!
//! Centralizes hardware-specific parameters to avoid duplication.

use std::f32::consts::PI;

/// Wheel encoder ticks per meter of travel.
/// Calibrated for CRL-200S wheel diameter and encoder resolution.
pub const WHEEL_TICKS_PER_METER: f32 = 4464.0;

/// Distance between wheel centers in meters.
/// Used for differential drive kinematics.
pub const WHEEL_BASE: f32 = 0.233;

/// Gyroscope scale factor: raw units are 0.01 deg/s.
/// Convert to rad/s: 0.01 * (PI / 180) = 0.0001745329
pub const GYRO_SCALE: f32 = 0.01 * (PI / 180.0);

/// Default lidar PWM duty cycle (60%).
pub const DEFAULT_LIDAR_PWM: u32 = 60;

/// Default rotation angular velocity in rad/s (~20 deg/s).
pub const DEFAULT_ROTATION_ANGULAR_VEL: f32 = 0.35;

/// Default forward linear velocity in m/s (~0.1 m/s).
pub const DEFAULT_LINEAR_VEL: f32 = 0.1;
