//! Core types for the VastuSLAM library.
//!
//! This module provides the fundamental types used throughout the SLAM pipeline.
//! All types follow the ROS REP-103 coordinate convention:
//! - **X-axis**: Forward (positive ahead of robot)
//! - **Y-axis**: Left (positive to robot's left)
//! - **Theta**: Counter-clockwise rotation from +X axis (radians)
//!
//! ## Type Categories
//!
//! ### Coordinates
//! - [`GridCoord`]: Integer cell indices for occupancy grid access
//! - [`WorldPoint`]: Floating-point world coordinates in meters
//!
//! ### Robot State
//! - [`Pose2D`]: Robot position (x, y) and orientation (theta)
//! - [`MotionModel`]: Probabilistic odometry uncertainty model
//! - [`Odometry`]: Wheel encoder readings with differential drive kinematics
//!
//! ### Grid Cells
//! - [`CellType`]: Semantic classification (Unknown, Floor, Wall, Cliff, Bump)
//! - [`Cell`]: Grid cell with type, confidence, and observation count
//!
//! ### Sensors
//! - [`LidarScan`]: 360° range measurements in polar coordinates
//! - [`CliffSensors`]: Four IR floor drop-off detectors
//! - [`BumperSensors`]: Two mechanical collision sensors
//! - [`SensorObservation`]: Combined sensor snapshot at a timestamp
//!
//! ### SIMD Operations
//! - [`PointCloud`]: Structure-of-Arrays point storage for vectorized operations
//! - [`RotationMatrix4`]: Pre-computed rotation for batch transforms
//! - [`GridCoordBatch`]: Batch of grid coordinates for SIMD processing
//!
//! ## Example
//!
//! ```rust,ignore
//! use vastu_slam::core::{Pose2D, WorldPoint, LidarScan};
//! use vastu_slam::core::simd::{PointCloud, transform_points};
//!
//! // Robot pose: 1m forward, 2m left, facing 90° CCW
//! let pose = Pose2D::new(1.0, 2.0, std::f32::consts::FRAC_PI_2);
//!
//! // Transform sensor point to world frame
//! let sensor_point = WorldPoint::new(1.0, 0.0);  // 1m ahead in robot frame
//! let world_point = pose.transform_point(sensor_point);  // (1.0, 3.0) in world
//!
//! // Batch transform lidar scan using SIMD
//! let scan = LidarScan::new(ranges, angles, 0.15, 8.0);
//! let points = PointCloud::from_scan(&scan);
//! let world_points = transform_points(&points, &pose);
//! ```

mod cell;
mod motion;
mod point;
mod pose;
mod sensors;

pub mod simd;

pub use cell::{Cell, CellType};
pub use motion::{MotionModel, Odometry};
pub use point::{GridCoord, WorldPoint};
pub use pose::{Pose2D, interpolate_pose, normalize_angle};
pub use sensors::{BumperSensors, CliffSensors, LidarScan, SensorObservation};
pub use simd::{GridCoordBatch, PointCloud, RotationMatrix4};
