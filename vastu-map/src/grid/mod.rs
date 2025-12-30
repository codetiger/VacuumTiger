//! Occupancy grid implementation.
//!
//! This module provides the core occupancy grid storage and update mechanisms:
//!
//! - [`GridStorage`]: Core grid storage with coordinate conversion
//! - [`raycaster`]: Bresenham ray casting for lidar updates
//! - [`lidar_update`]: Process lidar scans to mark Floor/Wall cells
//! - [`cliff_update`]: Process cliff sensors to mark Cliff cells
//! - [`bumper_update`]: Process bumper collisions to mark Bump cells

pub mod bumper_update;
pub mod cliff_update;
mod config;
pub mod lidar_update;
pub mod raycaster;
mod storage;

pub use config::{ConfigError, GridConfig, MapConfig, SensorConfig};
pub use storage::{CellCounts, GridStorage};
