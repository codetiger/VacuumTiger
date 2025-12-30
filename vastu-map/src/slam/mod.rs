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

mod config;
mod matcher;
mod types;

pub mod loop_closure;

pub use config::CorrelativeMatcherConfig;
pub use matcher::CorrelativeMatcher;
pub use types::{MatchQuality, ScanMatchResult};
