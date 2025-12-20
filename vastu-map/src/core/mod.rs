//! Core types for vastu-map.
//!
//! This module provides fundamental geometric types for 2D SLAM:
//!
//! - [`Point2D`]: 2D point/vector with full geometric operations
//! - [`Pose2D`]: 2D position and orientation with frame transforms
//! - [`PolarScan`]: Lidar scan in polar coordinates (raw sensor data)
//! - [`PointCloud2D`]: Point cloud with SoA layout for SIMD efficiency
//! - [`Bounds`]: Axis-aligned bounding box for spatial queries
//!
//! # Coordinate Frame
//!
//! All coordinates follow the ROS REP-103 convention:
//! - **X-forward**: Positive X is ahead of the robot
//! - **Y-left**: Positive Y is to the robot's left
//! - **Z-up**: Positive Z is upward (unused in 2D)
//! - **Rotation**: Counter-clockwise positive (right-hand rule)
//!
//! # Usage Examples
//!
//! ## Working with Points
//!
//! ```rust
//! use vastu_map::core::Point2D;
//!
//! let p = Point2D::new(3.0, 4.0);
//! assert_eq!(p.length(), 5.0);  // Distance from origin
//!
//! let direction = p.normalized();  // Unit vector
//! let rotated = p.rotate(std::f32::consts::FRAC_PI_2);  // Rotate 90°
//! ```
//!
//! ## Working with Poses
//!
//! ```rust
//! use vastu_map::core::{Pose2D, Point2D};
//!
//! // Robot at (1, 0) facing left (90° CCW from forward)
//! let pose = Pose2D::new(1.0, 0.0, std::f32::consts::FRAC_PI_2);
//!
//! // Transform a point from robot frame to world frame
//! let local_point = Point2D::new(1.0, 0.0);  // 1m ahead in robot frame
//! let world_point = pose.transform_point(local_point);
//! // Result: (1.0, 1.0) - 1m to the left of robot position
//! ```
//!
//! ## Processing Lidar Scans
//!
//! ```rust
//! use vastu_map::core::{PolarScan, PointCloud2D, Pose2D};
//!
//! // Raw lidar data
//! let mut scan = PolarScan::new();
//! scan.push(0.0, 2.0, 100);      // 2m ahead, good quality
//! scan.push(1.57, 1.5, 100);     // 1.5m to the left
//!
//! // Convert to Cartesian, filtering by quality and range
//! let cloud = scan.to_cartesian(50, 0.1, 10.0);
//!
//! // Transform to world frame
//! let robot_pose = Pose2D::new(0.0, 0.0, 0.0);
//! let world_cloud = cloud.transform(&robot_pose);
//! ```
//!
//! # Data Layout
//!
//! [`PointCloud2D`] uses Struct-of-Arrays (SoA) layout for SIMD efficiency:
//! - `xs: Vec<f32>` and `ys: Vec<f32>` stored separately
//! - Enables 4-wide SIMD operations via auto-vectorization
//! - Cache-friendly for sequential access patterns

pub mod bounds;
pub mod math;
pub mod point;
pub mod pose;
pub mod scan;

pub use bounds::Bounds;
pub use point::Point2D;
pub use pose::Pose2D;
pub use scan::{PointCloud2D, PolarScan};
