//! Test harness for the exploration example
//!
//! This module provides a simulation environment for testing
//! vastu-map's SLAM algorithms using sangam-io's mock device.
//!
//! # Features
//!
//! - Synchronous simulation with physics, lidar, and SLAM
//! - Ground truth tracking for accuracy metrics
//! - SVG visualization output

mod adapters;
mod harness;
mod metrics;
mod visualization;

// Re-exports
pub use harness::{HarnessConfig, TestHarness};
