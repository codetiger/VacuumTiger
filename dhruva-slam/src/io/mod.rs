//! I/O and infrastructure layer.
//!
//! This layer handles external communication and data persistence.
//!
//! # Contents
//!
//! - [`sangam_client`]: SangamIO TCP client for commands
//! - [`sangam_udp_receiver`]: SangamIO UDP receiver for sensor data
//! - [`bag`]: Data recording and playback
//! - [`streaming`]: TCP/UDP publishing for odometry and SLAM results
//! - [`map_manager`]: Map storage and persistence
//! - [`motion_controller`]: Velocity command interface to SangamIO

pub mod bag;
pub mod map_manager;
pub mod motion_controller;
pub mod sangam_client;
pub mod sangam_udp_receiver;
pub mod streaming;

// Re-export LidarScan used by core/types and slam_thread
pub use sangam_client::LidarScan;
