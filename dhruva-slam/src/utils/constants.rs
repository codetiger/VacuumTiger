//! Robot constants for CRL-200S vacuum cleaner.
//!
//! Centralizes hardware-specific parameters to avoid duplication.
//!
//! # IMU Axis Convention (ROS REP-103)
//!
//! SangamIO provides IMU data in ROS REP-103 standard frame:
//! - **X axis**: Forward (out the front of the robot)
//! - **Y axis**: Left (out the left side)
//! - **Z axis**: Up (out the top)
//!
//! For gyroscope data (`gyro_raw[3]`):
//! - `gyro_raw[0]`: Roll rate (rotation about X, tilting left/right)
//! - `gyro_raw[1]`: Pitch rate (rotation about Y, tilting forward/back)
//! - `gyro_raw[2]`: **Yaw rate (rotation about Z, heading change)** - this is the
//!   primary axis for 2D navigation. CCW (counter-clockwise) is positive.
//!
//! Verified empirically: In `rotate_3min.bag`, gyro_raw[2] shows mean of -1960
//! during continuous rotation, matching encoder-derived rotation of -3614°.

use std::f32::consts::PI;

/// Wheel encoder ticks per meter of travel.
/// Calibrated for CRL-200S wheel diameter and encoder resolution.
pub const WHEEL_TICKS_PER_METER: f32 = 4464.0;

/// Distance between wheel centers in meters.
/// Used for differential drive kinematics.
pub const WHEEL_BASE: f32 = 0.233;

/// Index of the yaw (heading) axis in the raw gyro array.
/// For ROS REP-103: Z axis (index 2) is yaw.
pub const GYRO_YAW_INDEX: usize = 2;

/// Gyroscope scale factor for CRL-200S.
///
/// Calibrated using encoder ground truth from rotation bags.
/// Raw gyro value of ~3740 at 38.33°/s encoder-measured rotation gives:
/// 38.33 / 3740 = 0.01025 deg/s per LSB = 0.000179 rad/s per LSB
///
/// Note: This is ~10x smaller than the originally assumed 0.1 deg/s per LSB.
pub const GYRO_SCALE: f32 = 0.01025 * (PI / 180.0);

/// Gyroscope sign for CRL-200S: 1.0 (no negation needed).
/// SangamIO provides gyro data in ROS REP-103 frame where CCW is positive,
/// matching the encoder convention.
pub const GYRO_SIGN: f32 = 1.0;

/// Default rotation angular velocity in rad/s (~20 deg/s).
pub const DEFAULT_ROTATION_ANGULAR_VEL: f32 = 0.35;

/// Default forward linear velocity in m/s (~0.1 m/s).
pub const DEFAULT_LINEAR_VEL: f32 = 0.1;
