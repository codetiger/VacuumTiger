//! Core types for vastu-map.
//!
//! This module provides fundamental geometric types:
//! - [`Point2D`]: 2D point/vector
//! - [`Pose2D`]: 2D position and orientation
//! - [`PolarScan`]: Lidar scan in polar coordinates
//! - [`PointCloud2D`]: Point cloud with SoA layout
//! - [`Bounds`]: Axis-aligned bounding box
//!
//! All coordinates use the ROS REP-103 convention:
//! - X-forward, Y-left, Z-up (right-handed)
//! - Counter-clockwise positive rotation

pub mod bounds;
pub mod math;
pub mod point;
pub mod pose;
pub mod scan;

pub use bounds::Bounds;
pub use point::Point2D;
pub use pose::Pose2D;
pub use scan::{PointCloud2D, PolarScan};
