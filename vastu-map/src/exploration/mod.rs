//! Autonomous exploration module.
//!
//! Provides frontier-based exploration with collision recovery for
//! complete map coverage. This module is designed to be used by
//! DhruvaSLAM for real robot exploration.
//!
//! # Primary Goals
//!
//! 1. **Complete map coverage** - Explore every reachable area until no frontiers remain
//! 2. **Avoid obstacles** - Navigate safely using path planning with robot radius clearance
//!
//! # Architecture
//!
//! The exploration module is sensor-agnostic. It provides planning and control logic,
//! while the caller (DhruvaSLAM) handles:
//! - Sensor reading (bumpers, cliffs, lidar)
//! - SLAM map updates
//! - Velocity command execution
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_map::exploration::{
//!     ExplorationController, ExplorationConfig,
//!     CollisionEvent, CollisionType,
//! };
//! use vastu_map::{VectorMap, Pose2D};
//!
//! // Create controller
//! let config = ExplorationConfig::default();
//! let mut controller = ExplorationController::new(config);
//! controller.start();
//!
//! // Main loop
//! loop {
//!     // Get current state from SLAM
//!     let map = get_slam_map();
//!     let pose = get_current_pose();
//!
//!     // Check sensors
//!     let collision = check_bumpers_and_cliffs();
//!
//!     // Update exploration
//!     let step = controller.update(&map, pose, collision);
//!
//!     // Handle virtual wall (for mirror/glass obstacles)
//!     if let Some(wall) = step.virtual_wall {
//!         map.add_line(wall.line);
//!     }
//!
//!     // Execute velocity command
//!     if let Some(vel) = step.velocity {
//!         send_velocity(vel.linear, vel.angular);
//!     }
//!
//!     // Check completion
//!     if controller.is_complete() {
//!         println!("Exploration complete!");
//!         break;
//!     }
//! }
//! ```

mod collision;
mod config;
mod controller;
mod path_follower;

// Re-export public types
pub use collision::{CollisionEvent, CollisionType, VirtualWall};
pub use config::ExplorationConfig;
pub use controller::{ExplorationController, ExplorationState, ExplorationStep};
pub use path_follower::{FollowResult, PathFollower, VelocityCommand};
