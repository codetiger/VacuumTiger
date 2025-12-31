//! SLAM (Simultaneous Localization and Mapping) module.
//!
//! This module provides scan-to-map matching and loop closure capabilities
//! for correcting encoder drift during autonomous exploration.
//!
//! # Components
//!
//! - **Correlative Matcher**: Brute-force search for best scan alignment
//! - **Loop Closure**: Detects revisited locations using LiDAR-IRIS descriptors
//! - **Pose Graph**: Stores and optimizes pose constraints
//!
//! # Usage
//!
//! ```ignore
//! use vastu_map::slam::{CorrelativeMatcher, CorrelativeMatcherConfig};
//!
//! let config = CorrelativeMatcherConfig::default();
//! let matcher = CorrelativeMatcher::new(config);
//!
//! // Match scan against current map
//! let result = matcher.match_scan(&scan, encoder_pose, &storage);
//! if result.converged {
//!     // Use refined pose
//!     let corrected_pose = result.pose;
//! }
//! ```

mod background_optimizer;
mod branch_bound;
mod config;
mod matcher;
mod types;

pub mod loop_closure;

pub use background_optimizer::{
    BackgroundOptimizer, BackgroundOptimizerConfig, OptimizationResult, OptimizerTrigger,
    PoseCorrection,
};
pub use branch_bound::{
    BranchBoundConfig, BranchBoundResult, NUM_LEVELS, PrecomputedGrids, branch_and_bound_match,
    branch_and_bound_match_scan, branch_and_bound_match_simd,
};
pub use config::CorrelativeMatcherConfig;
pub use loop_closure::{LoopClosureConfig, LoopClosureDetector, PoseGraphConfig};
pub use matcher::{CorrelativeMatcher, ScratchBuffers};
pub use types::{MatchQuality, ScanMatchResult};
