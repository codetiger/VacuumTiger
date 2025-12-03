//! Core SLAM algorithms layer.
//!
//! This layer contains the algorithmic building blocks for SLAM.
//!
//! # Contents
//!
//! - [`matching`]: Scan matching algorithms (ICP, correlative, multi-resolution)
//! - [`mapping`]: Occupancy grid mapping and ray tracing
//! - [`localization`]: Particle filter localization (Monte Carlo Localization)

pub mod localization;
pub mod mapping;
pub mod matching;
