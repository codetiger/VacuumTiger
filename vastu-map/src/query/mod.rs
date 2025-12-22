//! Map query operations for VectorMap.
//!
//! Provides spatial queries against the map:
//! - **Raycast**: Find distance to obstacles in a direction
//! - **Occupancy**: Query whether a point is free, occupied, or unknown
//! - **Frontier**: Detect unexplored boundaries for exploration
//! - **Path Planning**: Find paths using visibility graph
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_map::query::{raycast, query_occupancy, detect_frontiers, PathPlanner};
//! use vastu_map::core::Point2D;
//!
//! // Raycast to find wall distance
//! let origin = Point2D::new(0.0, 0.0);
//! let direction = Point2D::new(1.0, 0.0);
//! let distance = raycast(origin, direction, 10.0, &map_lines);
//!
//! // Check if a point is free
//! let occupancy = query_occupancy(Point2D::new(1.0, 1.0), &map_lines, None, &config);
//!
//! // Find exploration frontiers
//! let frontiers = detect_frontiers(&map_lines, &frontier_config);
//!
//! // Plan a path
//! let planner = PathPlanner::new(Default::default());
//! let path = planner.plan(start, goal, &lines, None, &occupancy_config);
//! ```

pub mod frontier;
pub mod occupancy;
pub mod path_planning;
pub mod raycast;

// Re-export main types and functions
pub use frontier::{
    FrontierConfig, FrontierDetector, cluster_centroid, cluster_frontiers, detect_frontiers,
    detect_frontiers_from_robot, get_best_frontier, rank_frontiers,
};
pub use occupancy::{
    OccupancyConfig, is_path_clear, is_region_clear, query_occupancy, query_occupancy_batch,
    query_occupancy_indexed,
};
pub use path_planning::{PathPlanner, PathPlanningConfig, VisibilityGraph, is_straight_path_clear};
pub use raycast::{
    RaycastResult, raycast, raycast_360, raycast_batch, raycast_detailed, raycast_indexed,
    raycast_sweep,
};
