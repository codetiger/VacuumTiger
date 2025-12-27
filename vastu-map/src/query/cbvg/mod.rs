//! Clearance-Based Visibility Graph (CBVG) path planning.
//!
//! This module implements a path planning algorithm that places graph nodes
//! along the **medial axis** (centerline) of free space, ensuring the robot
//! navigates through safe, traversable positions rather than wall corners.
//!
//! # Key Features
//!
//! - **Safe waypoints**: All nodes are at minimum distance from obstacles
//! - **Medial axis**: Nodes follow the centerline of corridors
//! - **Adaptive clearance**: Reduced clearance in narrow passages
//! - **Incremental updates**: Graph updates as map grows
//! - **SVG visualization**: Export graph for debugging
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_map::query::cbvg::{ClearanceVisibilityGraph, CBVGConfig};
//! use vastu_map::core::Point2D;
//!
//! // Create and configure graph
//! let config = CBVGConfig::default()
//!     .with_min_clearance(0.3)
//!     .with_min_clearance_narrow(0.18);
//!
//! let mut graph = ClearanceVisibilityGraph::new(config);
//!
//! // Build from wall segments
//! graph.build(&walls, Some(&bounds));
//!
//! // Find path
//! let start = Point2D::new(1.0, 1.0);
//! let goal = Point2D::new(5.0, 3.0);
//!
//! if let Some(path) = graph.find_path(start, goal, &walls) {
//!     println!("Path found: {} waypoints, {:.2}m", path.points.len(), path.length);
//! }
//!
//! // Export for visualization
//! let svg = graph.to_svg(&walls, Some(&path));
//! std::fs::write("path.svg", svg).unwrap();
//! ```
//!
//! # Algorithm Overview
//!
//! 1. **Node generation**: Sample points along the medial axis (Voronoi-based)
//! 2. **Edge construction**: Connect nodes with clearance-checked edges
//! 3. **Path planning**: Dijkstra's algorithm on the graph
//! 4. **Final approach**: Direct navigation from last node to goal
//!
//! # Configuration
//!
//! Key parameters in [`CBVGConfig`]:
//!
//! - `min_clearance`: Minimum distance from walls (default: 0.3m)
//! - `min_clearance_narrow`: Reduced clearance for tight passages (default: 0.18m)
//! - `max_edge_length`: Maximum distance between connected nodes (default: 2.0m)
//! - `voronoi_sample_step`: Density of medial axis sampling (default: 0.1m)

pub mod clearance;
pub mod config;
pub mod dijkstra;
pub mod graph;
pub mod node;
pub mod visibility;
pub mod voronoi;

// Re-export main types
pub use clearance::{
    closest_point_on_walls, is_edge_safe, is_path_clear_with_clearance, min_distance_to_walls,
    nearest_wall, sample_line,
};
pub use config::CBVGConfig;
pub use dijkstra::{
    DijkstraResult, DijkstraState, dijkstra, dijkstra_distances, dijkstra_with_limit,
};
pub use graph::ClearanceVisibilityGraph;
pub use node::{CBVGNode, NodeType};
pub use visibility::VisibilityRegion;
pub use voronoi::{
    VoronoiEdge, compute_voronoi_edges, generate_grid_nodes, sample_medial_axis_nodes,
    sample_wall_points,
};
