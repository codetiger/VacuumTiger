//! High-level autonomous exploration API.
//!
//! This module provides a simple, unified API for autonomous map exploration.
//! The `VastuExplorer` handles all the complexity of SLAM, scan matching,
//! loop closure, and frontier-based exploration internally.
//!
//! # Example
//!
//! ```ignore
//! use vastu_map::explore::{VastuExplorer, SensorSource, ExplorerConfig};
//!
//! // Implement SensorSource for your robot
//! struct MyRobot { /* ... */ }
//! impl SensorSource for MyRobot { /* ... */ }
//!
//! // Create explorer and run
//! let config = ExplorerConfig::default();
//! let mut explorer = VastuExplorer::new(config);
//! let mut robot = MyRobot::new();
//!
//! match explorer.explore(&mut robot) {
//!     ExploreResult::Complete(map) => {
//!         // Save the completed map
//!         vastu_map::io::save_vastu(map.storage(), "map.vastu").unwrap();
//!     }
//!     ExploreResult::Failed { map, reason } => {
//!         println!("Exploration failed: {}", reason);
//!     }
//! }
//! ```

mod config;
pub mod controller;
mod error;
mod explorer;
mod motion_filter;
mod source;
mod submap_explorer;
mod velocity;

pub use config::ExplorerConfig;
pub use error::ExplorationError;
pub use explorer::{ExploreResult, ExploreStatus, VastuExplorer};
pub use motion_filter::{MotionFilter, MotionFilterConfig};
pub use source::SensorSource;
pub use submap_explorer::{SubmapExplorerConfig, VastuSubmapExplorer};
pub use velocity::{
    VelocityConfig, compute_angular_velocity_to_heading, compute_velocity_to_target,
};

// Re-export controller types for unified API
pub use controller::{
    ExplorationCommand, ExplorationConfig, ExplorationController, ExplorationEvent,
    ExplorationProgress, ExplorationState, RecoveryAction,
};
