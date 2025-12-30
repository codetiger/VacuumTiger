//! Path planning algorithms.
//!
//! This module provides path planning on the occupancy grid:
//!
//! - **A* Search**: Find shortest collision-free paths
//! - **Path Smoothing**: Make paths more natural for robot motion
//!
//! ## A* Pathfinding
//!
//! ```rust,ignore
//! use vastu_map::pathfinding::{AStarPlanner, AStarConfig};
//!
//! let config = AStarConfig::default();
//! let planner = AStarPlanner::new(map.storage(), config);
//!
//! let result = planner.find_path_world(start, goal);
//! if result.success {
//!     println!("Path found with {} waypoints", result.path_world.len());
//! }
//! ```
//!
//! ## Path Smoothing
//!
//! ```rust,ignore
//! use vastu_map::pathfinding::{PathSmoother, SmoothingConfig};
//!
//! let smoother = PathSmoother::with_defaults(map.storage());
//! let smooth_path = smoother.smooth(&result.path_world);
//! ```

pub mod astar;
pub mod smoothing;

pub use astar::{
    AStarConfig, AStarPlanner, CachedAStarPlanner, PathFailure, PathResult, find_path, path_exists,
};

pub use smoothing::{PathSmoother, SmoothingConfig, path_length, resample_path, simplify_path};
