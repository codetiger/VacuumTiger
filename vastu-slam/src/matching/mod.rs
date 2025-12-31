//! Scan matching and loop closure module.
//!
//! This module provides scan-to-map matching and loop closure capabilities
//! for correcting encoder drift during mapping.
//!
//! ## Architecture
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────────────┐
//! │                     SCAN MATCHING PIPELINE                        │
//! │                                                                   │
//! │  LidarScan + Encoder Pose                                        │
//! │       │                                                           │
//! │       ▼                                                           │
//! │  ┌───────────────┐    ┌───────────────┐    ┌──────────────────┐  │
//! │  │ Coarse Search │ ──▶│  Fine Search  │ ──▶│ Gauss-Newton     │  │
//! │  │ (4cm, 2.3°)   │    │ (2cm, 1.1°)   │    │ Refinement       │  │
//! │  └───────────────┘    └───────────────┘    └──────────────────┘  │
//! │                                                     │             │
//! │                                                     ▼             │
//! │                                            ScanMatchResult        │
//! └──────────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Components
//!
//! | Component | Purpose |
//! |-----------|---------|
//! | [`CorrelativeMatcher`] | Brute-force multi-resolution search |
//! | [`branch_and_bound_match`] | Efficient large-window search |
//! | [`LoopClosureDetector`] | LiDAR-IRIS place recognition |
//! | [`BackgroundOptimizer`] | Async pose graph optimization |
//!
//! ## Matching Methods
//!
//! ### Correlative Search (Default)
//!
//! Best for typical operation with good odometry:
//! - Search window: ±30cm × ±8.6°
//! - Multi-resolution: coarse (4cm) → fine (2cm) → Gauss-Newton
//! - Gaussian scoring using distance field
//!
//! ### Branch-and-Bound
//!
//! Best for large search windows or recovery:
//! - Uses precomputed multi-resolution probability grids
//! - Prunes search space using upper bounds
//! - More efficient for windows > 1m
//!
//! ## Example
//!
//! ```rust,ignore
//! use vastu_slam::matching::{CorrelativeMatcher, CorrelativeMatcherConfig, ScanMatcher};
//!
//! // Create matcher
//! let config = CorrelativeMatcherConfig::default();
//! let matcher = CorrelativeMatcher::new(config);
//!
//! // Match scan against current map
//! let result = matcher.match_scan(&lidar_scan, encoder_pose, &grid_storage);
//!
//! if result.converged {
//!     println!("Corrected pose: {:?}", result.pose);
//!     println!("Score: {:.2} ({})", result.score, result.quality);
//! } else {
//!     // Fall back to encoder pose
//! }
//! ```
//!
//! ## Performance Modes
//!
//! ```rust,ignore
//! // Fast: Reduced search for real-time with accurate odometry
//! let config = CorrelativeMatcherConfig::fast();
//!
//! // Thorough: Large search for poor odometry or relocalization
//! let config = CorrelativeMatcherConfig::thorough();
//!
//! // SIMD-optimized (reuse scratch buffers)
//! let mut scratch = ScratchBuffers::new(scan.ranges.len());
//! let result = matcher.match_scan_simd(&scan, pose, &storage, &mut scratch);
//! ```

mod background_optimizer;
mod branch_bound;
mod config;
mod matcher;
mod robust;
mod traits;
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
pub use loop_closure::{
    LoopClosureConfig, LoopClosureDetector, LoopValidator, LoopValidatorConfig, PoseGraphConfig,
};
pub use matcher::{CorrelativeMatcher, ScratchBuffers};
pub use robust::{RobustKernel, cauchy, huber};
pub use traits::ScanMatcher;
pub use types::{MatchQuality, ScanMatchResult};
