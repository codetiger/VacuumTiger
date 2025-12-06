//! Sensor processing layer.
//!
//! This layer handles raw sensor data processing and fusion.
//!
//! # Contents
//!
//! - [`calibration`]: Gyroscope bias estimation and sensor calibration
//! - [`odometry`]: Wheel odometry, IMU fusion (complementary filter, ESKF)
//! - [`preprocessing`]: LiDAR scan filtering and conversion

pub mod calibration;
pub mod odometry;
pub mod preprocessing;
