//! Path planning module for autonomous navigation.
//!
//! This module provides:
//! - Cost map with obstacle inflation and wall distance penalties
//! - Theta* path planner for any-angle pathfinding
//! - Path smoother for differential drive kinematics

mod cost_map;
mod smoother;
mod theta_star;

pub use cost_map::CostMap;
pub use smoother::{PathSegment, PathSmoother, SmoothedPath};
pub use theta_star::ThetaStarPlanner;
