//! Place recognition descriptors for loop closure detection.
//!
//! This module provides global descriptors for comparing LiDAR scans
//! without requiring initial pose estimates.
//!
//! # Contents
//!
//! - [`LidarIris`]: Binary descriptor based on LiDAR-IRIS for fast matching
//! - [`LidarIrisConfig`]: Configuration for IRIS descriptor generation

mod lidar_iris;

pub use lidar_iris::{LidarIris, LidarIrisConfig};
