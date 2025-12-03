//! Sensor processing layer.
//!
//! This layer handles raw sensor data processing and fusion.
//!
//! # Contents
//!
//! - [`odometry`]: Wheel odometry, IMU fusion (complementary filter, ESKF)
//! - [`preprocessing`]: LiDAR scan filtering and conversion

pub mod odometry;
pub mod preprocessing;
