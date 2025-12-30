//! Submap-based SLAM architecture for reversible map updates.
//!
//! This module implements a submap architecture inspired by Google Cartographer,
//! enabling proper loop closure correction by storing raw scans and regenerating
//! the global map when submap origins are adjusted.
//!
//! ## Key Concepts
//!
//! - **Submap**: A locally-consistent map fragment with an adjustable origin
//! - **StoredScan**: Raw lidar scan stored for later regeneration
//! - **SubmapManager**: Orchestrates submap lifecycle and global grid composition
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                         World Frame                              │
//! │                                                                  │
//! │    Submap 0              Submap 1              Submap 2          │
//! │   ┌─────────┐           ┌─────────┐           ┌─────────┐       │
//! │   │ Local   │  ──T01──▶ │ Local   │  ──T12──▶ │ Local   │       │
//! │   │ Grid    │           │ Grid    │           │ Grid    │       │
//! │   └────┬────┘           └────┬────┘           └────┬────┘       │
//! │        │                     │                     │             │
//! │        ▼                     ▼                     ▼             │
//! │   origin_0 (fixed)      origin_1 (opt)        origin_2 (opt)    │
//! │                                                                  │
//! └─────────────────────────────────────────────────────────────────┘
//! ```
//!
//! After pose graph optimization, only submap origins change - local grids
//! are regenerated from stored scans at new positions.

mod config;
mod graph;
mod manager;
mod matching;
mod types;

pub use config::SubmapConfig;
pub use graph::{
    SubmapGraphConfig, SubmapLoopClosure, SubmapLoopEdge, SubmapNode, SubmapOdometryEdge,
    SubmapPoseGraph,
};
pub use manager::{CachedGlobalGrid, InsertResult, SubmapCorrection, SubmapManager};
pub use matching::{MultiSubmapMatchConfig, MultiSubmapMatchResult, MultiSubmapMatcher};
pub use types::{StoredScan, Submap, SubmapId, SubmapState};
