//! Loop Closure Detection for VectorMap SLAM.
//!
//! This module provides keyframe-based loop closure detection using
//! shape context descriptors. When the robot revisits a previously
//! mapped area, loop closures are detected and can be used to
//! correct accumulated drift.
//!
//! # Overview
//!
//! The loop closure pipeline:
//! 1. Store keyframes at regular intervals (every `keyframe_interval` meters)
//! 2. Compute scan descriptors for each keyframe
//! 3. When a new keyframe is added, compare against candidates
//! 4. Verify candidates with ICP matching
//! 5. Return loop closures for pose graph optimization
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_map::loop_closure::{LoopClosureDetector, LoopClosureConfig};
//! use vastu_map::core::Pose2D;
//!
//! let config = LoopClosureConfig::default();
//! let mut detector = LoopClosureDetector::new(config);
//!
//! // Process observations
//! if let Some(closure) = detector.process(current_pose, &lines, &corners, &points) {
//!     println!("Loop closure detected: {} -> {}", closure.from_keyframe, closure.to_keyframe);
//! }
//! ```
//!
//! # Module Structure
//!
//! - [`config`]: Configuration for loop closure detection
//! - [`types`]: Core types (Keyframe, LoopClosure)
//! - [`detector`]: LoopClosureDetector implementation
//! - [`traits`]: LoopDetector trait for custom implementations

mod config;
mod detector;
mod traits;
mod types;

// Re-export public API
pub use config::LoopClosureConfig;
pub use detector::LoopClosureDetector;
pub use traits::LoopDetector;
pub use types::{Keyframe, LoopClosure};
