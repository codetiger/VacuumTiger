//! Loop closure detection and pose graph optimization.
//!
//! This module provides:
//! - **LiDAR-IRIS**: Compact binary descriptor for place recognition
//! - **LoopClosureDetector**: Detects when robot revisits a location
//! - **LoopValidator**: Geometric verification of loop closure candidates
//! - **PoseGraph**: Stores poses and constraints, with optimization

mod descriptor;
mod detector;
mod graph;
mod validator;

pub use descriptor::LidarIris;
pub use detector::{LoopClosure, LoopClosureConfig, LoopClosureDetector};
pub use graph::{LoopEdge, OdometryEdge, PoseGraph, PoseGraphConfig};
pub use validator::{LoopValidator, LoopValidatorConfig, RejectionReason, ValidationResult};
