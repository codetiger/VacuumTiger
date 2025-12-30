//! Map query operations.
//!
//! This module provides query operations on the occupancy grid:
//!
//! - **Frontier detection**: Find boundaries between known and unknown areas
//! - **Traversability**: Check if positions and paths are safe for the robot
//!
//! ## Frontier Detection
//!
//! Frontiers are floor cells adjacent to unknown cells. They represent the
//! boundary of explored space and are used as targets for exploration.
//!
//! ```rust,ignore
//! use vastu_map::query::{FrontierDetector, Frontier};
//!
//! let detector = FrontierDetector::new();
//! let frontiers = detector.detect_frontiers(map.storage());
//!
//! for frontier in frontiers {
//!     println!("Frontier at {:?} with {} cells",
//!         frontier.centroid_world, frontier.size);
//! }
//! ```
//!
//! ## Traversability
//!
//! Check if positions and paths are safe for robot navigation.
//!
//! ```rust,ignore
//! use vastu_map::query::{TraversabilityChecker, RobotFootprint};
//!
//! let footprint = RobotFootprint::new(0.17, 0.05);
//! let checker = TraversabilityChecker::new(map.storage(), footprint);
//!
//! if checker.is_position_safe(target_position) {
//!     // Safe to navigate to
//! }
//! ```

pub mod frontier;
pub mod traversability;

pub use frontier::{Frontier, FrontierCell, FrontierDetector, count_frontier_cells, has_frontiers};

pub use traversability::{
    PathCheckResult, RobotFootprint, TraversabilityChecker, is_safe, is_traversable,
};
