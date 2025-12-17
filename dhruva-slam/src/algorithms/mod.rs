//! Core SLAM algorithms layer.
//!
//! This layer contains the algorithmic building blocks for SLAM.
//!
//! # Contents
//!
//! - [`matching`]: Scan matching algorithms (ICP, correlative, multi-resolution)
//! - [`mapping`]: Occupancy grid mapping and ray tracing
//! - [`descriptors`]: Place recognition descriptors (LiDAR-IRIS)
//! - [`planning`]: A* path planning for navigation

pub mod descriptors;
pub mod mapping;
pub mod matching;
pub mod planning;
