//! Path planning algorithms.
//!
//! This module provides the A* path planner used for navigation.
//!
//! # Architecture
//!
//! The planner operates on an occupancy grid and produces paths that:
//! - Avoid obstacles (occupied cells)
//! - Maintain clearance from walls (obstacle inflation by robot radius)
//! - Navigate around unknown areas (treated as obstacles)
//!
//! # Usage
//!
//! ```ignore
//! use dhruva_slam::algorithms::planning::{AStarPlanner, AStarConfig};
//!
//! let config = AStarConfig::default();
//! let mut planner = AStarPlanner::new(config);
//!
//! // Plan from robot position to goal
//! let path = planner.plan(&map_data, (robot_x, robot_y), (goal_x, goal_y));
//! ```

mod astar;

pub use astar::{AStarConfig, AStarPlanner, PlanningError};
