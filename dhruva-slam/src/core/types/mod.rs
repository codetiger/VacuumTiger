//! Core data types for SLAM operations.
//!
//! Phase 1 types:
//! - [`Point2D`]: 2D point in meters
//! - [`Pose2D`]: Robot pose (x, y, theta) in meters and radians
//! - [`Timestamped<T>`]: Generic timestamp wrapper
//!
//! Phase 2 types (Odometry):
//! - [`Covariance2D`]: 3x3 covariance matrix for pose uncertainty
//!
//! Tracking types:
//! - [`PoseTracker`]: Pose accumulation and delta computation
//!
//! Phase 3 types (Scan Processing):
//! - [`LaserScan`]: Raw LiDAR scan in polar coordinates
//! - [`PointCloud2D`]: Collection of 2D points in Cartesian coordinates

mod odometry;
mod pose;
mod pose_tracker;
mod scan;
mod timestamped;

// Phase 1 types
pub use pose::{Point2D, Pose2D};
pub use timestamped::Timestamped;

// Phase 2 types
pub use odometry::Covariance2D;

// Tracking types
pub use pose_tracker::PoseTracker;

// Phase 3 types
pub use scan::{LaserScan, PointCloud2D};
