//! Autonomous exploration for mapping.
//!
//! This module provides algorithms for autonomous exploration during map building:
//!
//! - [`strategy`]: Exploration strategy trait for extensibility
//! - [`frontier`]: Frontier-based exploration implementation
//!
//! # Architecture
//!
//! The exploration system runs as a separate thread that:
//! 1. Reads the current map and pose from SharedState
//! 2. Computes the next target using the active strategy
//! 3. Sends velocity commands to the motion controller
//! 4. Handles bumper events for obstacle avoidance
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::exploration::{ExplorationStrategy, FrontierExploration, FrontierConfig};
//!
//! let config = FrontierConfig::default();
//! let mut strategy = FrontierExploration::new(config);
//!
//! // In exploration loop:
//! let action = strategy.next_action(&map, &pose, bumper_triggered);
//! match action {
//!     ExplorationAction::MoveTo { target, .. } => { /* navigate to target */ }
//!     ExplorationAction::Complete => { /* mapping finished */ }
//!     // ...
//! }
//! ```

mod frontier;
mod strategy;

pub use frontier::{FrontierConfig, FrontierExploration};
pub use strategy::{ExplorationAction, ExplorationStrategy, HazardEvent, HazardType};
