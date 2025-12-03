//! Core data types for SLAM operations.
//!
//! Phase 1 types:
//! - [`Point2D`]: 2D point in meters
//! - [`Pose2D`]: Robot pose (x, y, theta) in meters and radians
//! - [`Timestamped<T>`]: Generic timestamp wrapper
//!
//! Phase 2 types (Odometry):
//! - [`Twist2D`]: 2D velocity (linear and angular)
//! - [`ImuReading`]: Raw IMU sensor data
//! - [`Covariance2D`]: 3x3 covariance matrix for pose uncertainty
//!
//! Phase 3 types (Scan Processing):
//! - [`LaserScan`]: Raw LiDAR scan in polar coordinates
//! - [`PointCloud2D`]: Collection of 2D points in Cartesian coordinates

mod pose;
mod timestamped;
mod odometry;
mod scan;

// Phase 1 types
pub use pose::{Point2D, Pose2D};
pub use timestamped::Timestamped;

// Phase 2 types
pub use odometry::{Twist2D, ImuReading, Covariance2D};

// Phase 3 types
pub use scan::{LaserScan, PointCloud2D};
