//! SLAM orchestration layer.
//!
//! This layer coordinates the SLAM algorithms into a unified system.
//!
//! # Contents
//!
//! - [`slam`]: Online SLAM engine, keyframe management, submap handling
//! - [`graph`]: Pose graph optimization and loop closure detection

pub mod slam;
pub mod graph;
