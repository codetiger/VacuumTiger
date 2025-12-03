//! Core foundation layer.
//!
//! This is the bottom layer of the SLAM stack with no internal dependencies.
//! All other layers depend on core.
//!
//! # Contents
//!
//! - [`types`]: Core data types (poses, scans, odometry)
//! - [`math`]: Mathematical primitives (angle normalization, interpolation)

pub mod types;
pub mod math;
