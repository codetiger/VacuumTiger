//! Loop closure detection and pose graph optimization.
//!
//! This module provides:
//! - **LiDAR-IRIS**: Compact binary descriptor for place recognition
//! - **LoopClosureDetector**: Detects when robot revisits a location
//! - **PoseGraph**: Stores poses and constraints, with optimization

mod descriptor;
mod detector;
mod graph;

pub use descriptor::LidarIris;
pub use detector::{LoopClosure, LoopClosureConfig, LoopClosureDetector};
pub use graph::{LoopEdge, OdometryEdge, PoseGraph, PoseGraphConfig};
