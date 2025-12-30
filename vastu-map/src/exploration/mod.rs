//! Autonomous exploration controller.
//!
//! This module provides a state machine for frontier-based exploration:
//!
//! - **Frontier detection**: Find unexplored boundaries
//! - **Path planning**: Navigate to frontiers
//! - **Recovery**: Handle obstacles and stuck conditions
//!
//! ## Usage
//!
//! ```rust,ignore
//! use vastu_map::exploration::{ExplorationController, ExplorationConfig};
//!
//! let config = ExplorationConfig::default();
//! let mut controller = ExplorationController::new(config);
//!
//! controller.start();
//!
//! loop {
//!     let cmd = controller.update(robot_pose, map.storage());
//!     match cmd {
//!         ExplorationCommand::MoveTo { target, max_speed } => {
//!             // Send motion command to robot
//!         }
//!         ExplorationCommand::ExplorationComplete => {
//!             break;
//!         }
//!         _ => {}
//!     }
//! }
//! ```
//!
//! ## State Machine
//!
//! The controller cycles through these states:
//!
//! 1. **Idle** - Waiting to start
//! 2. **SearchingFrontiers** - Looking for unexplored areas
//! 3. **Planning** - Computing path to selected frontier
//! 4. **Navigating** - Following path to frontier
//! 5. **Scanning** - At frontier, allowing sensors to update map
//! 6. **Recovering** - Handling obstacles or stuck conditions
//! 7. **Complete** - All frontiers explored
//! 8. **Failed** - Too many errors

pub mod config;
pub mod controller;
pub mod state;

pub use config::ExplorationConfig;
pub use controller::{ExplorationController, ExplorationProgress};
pub use state::{ExplorationCommand, ExplorationEvent, ExplorationState, RecoveryAction};
