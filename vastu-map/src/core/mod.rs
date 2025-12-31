//! Core types for the vastu-map occupancy grid library.
//!
//! This module provides the fundamental types used throughout the library:
//! - [`Cell`] and [`CellType`]: Grid cell representation with semantic types
//! - [`GridCoord`] and [`WorldPoint`]: Coordinate types
//! - [`Pose2D`]: Robot pose (position + orientation)
//! - [`PointCloud`]: SIMD-optimized 2D point cloud (SoA layout)
//! - Sensor types: [`LidarScan`], [`CliffSensors`], [`BumperSensors`]

mod cell;
mod point;
mod pose;
mod sensors;

pub mod simd;

pub use cell::{Cell, CellType};
pub use point::{GridCoord, WorldPoint};
pub use pose::{Pose2D, interpolate_pose, normalize_angle};
pub use sensors::{BumperSensors, CliffSensors, LidarScan, SensorObservation};
pub use simd::{GridCoordBatch, PointCloud, RotationMatrix4};
