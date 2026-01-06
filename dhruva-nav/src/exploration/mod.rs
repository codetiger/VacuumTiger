//! Autonomous exploration module.
//!
//! This module provides frontier-based exploration capabilities:
//! - Frontier detection and clustering
//! - Path following with pure pursuit
//! - Exploration state machine

mod explorer;
mod follower;
mod frontier;

pub use explorer::{ExplorationState, ExplorationStep, Explorer, ExplorerConfig};
pub use follower::FollowerConfig;
pub use frontier::FrontierConfig;
