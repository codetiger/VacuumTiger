//! I/O and infrastructure layer.
//!
//! This layer handles external communication and data persistence.
//!
//! # Contents
//!
//! - [`sangam_client`]: SangamIO TCP client for sensor data
//! - [`bag`]: Data recording and playback
//! - [`streaming`]: TCP publishing for odometry and SLAM results

pub mod bag;
pub mod sangam_client;
pub mod streaming;

// Re-export common types
pub use sangam_client::{LidarPoint, LidarScan, SangamClient};
