//! Vector-based path planning using Visibility Graph.
//!
//! Implements optimal shortest-path planning using a visibility graph
//! constructed from map line endpoints. This approach is natural for
//! VastuMap's line-based representation and provides optimal paths.
//!
//! # Algorithm
//!
//! 1. Collect nodes: all line endpoints + start + goal
//! 2. Build edges: connect mutually visible node pairs
//! 3. Search: Dijkstra from start to goal
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_map::query::path_planning::{PathPlanner, PathPlanningConfig};
//! use vastu_map::core::Point2D;
//!
//! let config = PathPlanningConfig::default();
//! let planner = PathPlanner::new(config);
//!
//! let path = planner.plan(
//!     Point2D::new(0.0, 0.0),
//!     Point2D::new(5.0, 5.0),
//!     &map_lines,
//!     Some(&bounds),
//!     &occupancy_config,
//! );
//! ```

use serde::{Deserialize, Serialize};

use crate::core::{Bounds, Point2D};
use crate::features::Line2D;
use crate::query::occupancy::OccupancyConfig;
use crate::{Occupancy, Path};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Configuration for path planning.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PathPlanningConfig {
    /// Robot radius for obstacle inflation (meters).
    /// Paths will avoid coming within this distance of obstacles.
    /// Default: 0.15m
    pub robot_radius: f32,

    /// Maximum number of nodes in the visibility graph.
    /// Limits computation for very complex maps.
    /// Default: 1000
    pub max_nodes: usize,

    /// Only plan through known free space.
    /// When true, edges that pass through unknown areas are rejected.
    /// Default: true
    pub conservative: bool,
}

impl Default for PathPlanningConfig {
    fn default() -> Self {
        Self {
            robot_radius: 0.15,
            max_nodes: 1000,
            conservative: true,
        }
    }
}

impl PathPlanningConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for robot radius.
    pub fn with_robot_radius(mut self, radius: f32) -> Self {
        self.robot_radius = radius;
        self
    }

    /// Builder-style setter for max nodes.
    pub fn with_max_nodes(mut self, max: usize) -> Self {
        self.max_nodes = max;
        self
    }

    /// Builder-style setter for conservative mode.
    pub fn with_conservative(mut self, conservative: bool) -> Self {
        self.conservative = conservative;
        self
    }
}

/// Visibility graph for path planning.
///
/// Nodes are points (line endpoints + start/goal), edges connect
/// mutually visible points with distance as cost.
#[derive(Clone, Debug)]
pub struct VisibilityGraph {
    /// Node positions.
    nodes: Vec<Point2D>,

    /// Adjacency list: for each node, list of (neighbor_idx, distance).
    edges: Vec<Vec<(usize, f32)>>,

    /// Index of start node.
    start_idx: usize,

    /// Index of goal node.
    goal_idx: usize,
}

impl VisibilityGraph {
    /// Build a visibility graph from line endpoints plus start and goal.
    ///
    /// # Arguments
    ///
    /// * `lines` - Map lines (obstacles)
    /// * `start` - Start position
    /// * `goal` - Goal position
    /// * `robot_radius` - Robot radius for collision checking
    /// * `max_nodes` - Maximum number of nodes
    ///
    /// # Returns
    ///
    /// A visibility graph ready for path search.
    pub fn build(
        lines: &[Line2D],
        start: Point2D,
        goal: Point2D,
        robot_radius: f32,
        max_nodes: usize,
    ) -> Self {
        // Collect nodes: line endpoints + start + goal
        let mut nodes = Vec::with_capacity(lines.len() * 2 + 2);

        // Add line endpoints (offset by robot radius perpendicular to line)
        for line in lines {
            // Add both endpoints
            nodes.push(line.start);
            nodes.push(line.end);

            if nodes.len() >= max_nodes.saturating_sub(2) {
                break;
            }
        }

        // Add start and goal
        let start_idx = nodes.len();
        nodes.push(start);
        let goal_idx = nodes.len();
        nodes.push(goal);

        // Build edges using visibility checks
        let n = nodes.len();
        let mut edges: Vec<Vec<(usize, f32)>> = vec![Vec::new(); n];

        // For each pair of nodes, check visibility
        for i in 0..n {
            for j in (i + 1)..n {
                let p1 = nodes[i];
                let p2 = nodes[j];

                // Create a test segment
                let test_line = Line2D::new(p1, p2);
                let test_length = test_line.length();

                // Skip very short edges
                if test_length < 0.001 {
                    continue;
                }

                // Check if this edge intersects any obstacle line
                let mut blocked = false;
                for line in lines {
                    // Skip lines that share an endpoint with the test edge
                    // (these are valid connections to wall corners)
                    let shares_endpoint = p1.distance(line.start) < 0.01
                        || p1.distance(line.end) < 0.01
                        || p2.distance(line.start) < 0.01
                        || p2.distance(line.end) < 0.01;

                    // Check for intersection with the obstacle line
                    if let Some(intersection) = test_line.intersection(line) {
                        // Make sure the intersection isn't at the endpoints
                        // (we allow touching at corners)
                        let dist_to_p1 = intersection.distance(p1);
                        let dist_to_p2 = intersection.distance(p2);

                        if dist_to_p1 > 0.01 && dist_to_p2 > 0.01 {
                            blocked = true;
                            break;
                        }
                    }

                    // Also check minimum distance to line segment (for robot radius)
                    // Skip this check for lines sharing an endpoint
                    if robot_radius > 0.0 && !shares_endpoint {
                        // Sample points along the test edge
                        let num_samples = (test_length / robot_radius).ceil() as usize;
                        let num_samples = num_samples.max(3);

                        for k in 1..num_samples {
                            let t = k as f32 / num_samples as f32;
                            let sample = test_line.point_at(t);
                            // Use distance_to_point_segment for accurate segment distance
                            let dist = line.distance_to_point_segment(sample);

                            if dist < robot_radius {
                                blocked = true;
                                break;
                            }
                        }

                        if blocked {
                            break;
                        }
                    }
                }

                if !blocked {
                    let dist = test_length;
                    edges[i].push((j, dist));
                    edges[j].push((i, dist));
                }
            }
        }

        Self {
            nodes,
            edges,
            start_idx,
            goal_idx,
        }
    }

    /// Find shortest path using Dijkstra's algorithm.
    ///
    /// # Returns
    ///
    /// Sequence of node indices from start to goal, or None if no path exists.
    pub fn find_path(&self) -> Option<Vec<usize>> {
        let n = self.nodes.len();
        if n == 0 {
            return None;
        }

        // Handle same start and goal
        if self.start_idx == self.goal_idx {
            return Some(vec![self.start_idx]);
        }

        // Distance to each node
        let mut dist: Vec<f32> = vec![f32::INFINITY; n];
        let mut prev: Vec<Option<usize>> = vec![None; n];
        dist[self.start_idx] = 0.0;

        // Priority queue: (negative distance for max-heap to min-heap, node_idx)
        let mut heap = BinaryHeap::new();
        heap.push(DijkstraState {
            cost: 0.0,
            node: self.start_idx,
        });

        while let Some(DijkstraState { cost, node }) = heap.pop() {
            // Skip if we've found a better path
            if cost > dist[node] {
                continue;
            }

            // Found goal
            if node == self.goal_idx {
                break;
            }

            // Explore neighbors
            for &(neighbor, edge_dist) in &self.edges[node] {
                let new_dist = dist[node] + edge_dist;
                if new_dist < dist[neighbor] {
                    dist[neighbor] = new_dist;
                    prev[neighbor] = Some(node);
                    heap.push(DijkstraState {
                        cost: new_dist,
                        node: neighbor,
                    });
                }
            }
        }

        // Reconstruct path
        prev[self.goal_idx]?;

        let mut path = Vec::new();
        let mut current = self.goal_idx;
        while current != self.start_idx {
            path.push(current);
            current = prev[current]?;
        }
        path.push(self.start_idx);
        path.reverse();

        Some(path)
    }

    /// Get node position by index.
    pub fn node(&self, idx: usize) -> Option<&Point2D> {
        self.nodes.get(idx)
    }

    /// Get all nodes.
    pub fn nodes(&self) -> &[Point2D] {
        &self.nodes
    }

    /// Get start node index.
    pub fn start_idx(&self) -> usize {
        self.start_idx
    }

    /// Get goal node index.
    pub fn goal_idx(&self) -> usize {
        self.goal_idx
    }

    /// Get number of nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Get number of edges.
    pub fn edge_count(&self) -> usize {
        self.edges.iter().map(|e| e.len()).sum::<usize>() / 2
    }

    /// Get the adjacency list of edges.
    ///
    /// Returns a slice of edge lists, where each inner list contains
    /// (neighbor_index, distance) pairs for the corresponding node.
    pub fn edges(&self) -> &[Vec<(usize, f32)>] {
        &self.edges
    }

    /// Get all unique edges as (from_node, to_node, distance) tuples.
    ///
    /// Each edge is returned only once (not duplicated for both directions).
    pub fn edge_list(&self) -> Vec<(usize, usize, f32)> {
        let mut edges = Vec::new();
        for (from_idx, neighbors) in self.edges.iter().enumerate() {
            for &(to_idx, dist) in neighbors {
                // Only include each edge once (from < to)
                if from_idx < to_idx {
                    edges.push((from_idx, to_idx, dist));
                }
            }
        }
        edges
    }
}

/// State for Dijkstra's algorithm priority queue.
#[derive(Clone, Copy, Debug)]
struct DijkstraState {
    cost: f32,
    node: usize,
}

impl PartialEq for DijkstraState {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for DijkstraState {}

impl Ord for DijkstraState {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for DijkstraState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// High-level path planner using visibility graph.
#[derive(Clone, Debug)]
pub struct PathPlanner {
    config: PathPlanningConfig,
}

impl PathPlanner {
    /// Create a new path planner with the given configuration.
    pub fn new(config: PathPlanningConfig) -> Self {
        Self { config }
    }

    /// Plan a path from start to goal.
    ///
    /// # Arguments
    ///
    /// * `from` - Start position
    /// * `to` - Goal position
    /// * `lines` - Map lines (obstacles)
    /// * `bounds` - Map bounds (for conservative mode)
    /// * `occupancy_config` - Occupancy configuration (for conservative mode)
    ///
    /// # Returns
    ///
    /// Path if one exists, None otherwise.
    pub fn plan(
        &self,
        from: Point2D,
        to: Point2D,
        lines: &[Line2D],
        bounds: Option<&Bounds>,
        occupancy_config: &OccupancyConfig,
    ) -> Option<Path> {
        // Handle same start and goal - trivial path
        if from.distance(to) < 0.001 {
            return Some(Path {
                points: vec![from],
                length: 0.0,
            });
        }

        // Check if start or goal is occupied
        if self.config.conservative {
            let start_occ =
                crate::query::occupancy::query_occupancy(from, lines, bounds, occupancy_config);
            let goal_occ =
                crate::query::occupancy::query_occupancy(to, lines, bounds, occupancy_config);

            // In conservative mode, reject if either is not Free
            if start_occ != Occupancy::Free || goal_occ != Occupancy::Free {
                return None;
            }
        }

        // Try direct path first - much faster than building full visibility graph
        if is_straight_path_clear(from, to, lines, self.config.robot_radius) {
            return Some(Path {
                points: vec![from, to],
                length: from.distance(to),
            });
        }

        // Build visibility graph
        let graph = VisibilityGraph::build(
            lines,
            from,
            to,
            self.config.robot_radius,
            self.config.max_nodes,
        );

        // Find path
        let indices = graph.find_path()?;

        // Convert to Path
        let mut points = Vec::with_capacity(indices.len());
        let mut length = 0.0;

        for (i, &idx) in indices.iter().enumerate() {
            let point = *graph.node(idx)?;
            points.push(point);

            if i > 0 {
                let prev = points[i - 1];
                length += prev.distance(point);
            }
        }

        Some(Path { points, length })
    }

    /// Get the configuration.
    pub fn config(&self) -> &PathPlanningConfig {
        &self.config
    }

    /// Build a visibility graph for visualization.
    ///
    /// This builds the full visibility graph without planning a path,
    /// useful for debugging and visualization purposes.
    ///
    /// # Arguments
    ///
    /// * `from` - Start position
    /// * `to` - Goal position
    /// * `lines` - Map lines (obstacles)
    ///
    /// # Returns
    ///
    /// The visibility graph with all nodes and edges.
    pub fn build_graph(&self, from: Point2D, to: Point2D, lines: &[Line2D]) -> VisibilityGraph {
        VisibilityGraph::build(
            lines,
            from,
            to,
            self.config.robot_radius,
            self.config.max_nodes,
        )
    }
}

/// Check if a straight-line path is clear of obstacles.
///
/// This is a simpler check than full path planning - just tests if
/// the direct line between two points intersects any obstacles.
///
/// # Arguments
///
/// * `from` - Start position
/// * `to` - End position
/// * `lines` - Map lines (obstacles)
/// * `robot_radius` - Robot radius for clearance
///
/// # Returns
///
/// True if the straight path is clear.
pub fn is_straight_path_clear(
    from: Point2D,
    to: Point2D,
    lines: &[Line2D],
    robot_radius: f32,
) -> bool {
    let test_line = Line2D::new(from, to);
    let test_length = test_line.length();

    if test_length < 0.001 {
        return true; // Zero-length path is always clear
    }

    for line in lines {
        // Check for direct intersection
        if test_line.intersection(line).is_some() {
            return false;
        }

        // Check clearance along the path (use segment distance, not infinite line)
        if robot_radius > 0.0 {
            let num_samples = (test_length / robot_radius).ceil() as usize;
            let num_samples = num_samples.max(3);

            for k in 0..=num_samples {
                let t = k as f32 / num_samples as f32;
                let sample = test_line.point_at(t);
                // Use distance_to_point_segment for accurate segment distance
                let dist = line.distance_to_point_segment(sample);

                if dist < robot_radius {
                    return false;
                }
            }
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_room() -> Vec<Line2D> {
        // Simple 10x10 room centered at origin
        vec![
            Line2D::new(Point2D::new(-5.0, -5.0), Point2D::new(5.0, -5.0)), // Bottom
            Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0)),   // Right
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(-5.0, 5.0)),   // Top
            Line2D::new(Point2D::new(-5.0, 5.0), Point2D::new(-5.0, -5.0)), // Left
        ]
    }

    fn make_room_with_obstacle() -> Vec<Line2D> {
        let mut lines = make_room();
        // Add a vertical wall in the middle
        lines.push(Line2D::new(Point2D::new(0.0, -3.0), Point2D::new(0.0, 3.0)));
        lines
    }

    #[test]
    fn test_config_default() {
        let config = PathPlanningConfig::default();
        assert_eq!(config.robot_radius, 0.15);
        assert_eq!(config.max_nodes, 1000);
        assert!(config.conservative);
    }

    #[test]
    fn test_config_builder() {
        let config = PathPlanningConfig::new()
            .with_robot_radius(0.2)
            .with_max_nodes(500)
            .with_conservative(false);

        assert_eq!(config.robot_radius, 0.2);
        assert_eq!(config.max_nodes, 500);
        assert!(!config.conservative);
    }

    #[test]
    fn test_visibility_graph_empty() {
        let lines: Vec<Line2D> = vec![];
        let graph = VisibilityGraph::build(
            &lines,
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
            0.0,
            100,
        );

        // Should have just start and goal
        assert_eq!(graph.node_count(), 2);
        assert_eq!(graph.start_idx(), 0);
        assert_eq!(graph.goal_idx(), 1);
    }

    #[test]
    fn test_visibility_graph_simple() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0))];

        let graph = VisibilityGraph::build(
            &lines,
            Point2D::new(-1.0, 0.0),
            Point2D::new(2.0, 0.0),
            0.0,
            100,
        );

        // 2 line endpoints + start + goal = 4 nodes
        assert_eq!(graph.node_count(), 4);
    }

    #[test]
    fn test_find_path_direct() {
        let lines: Vec<Line2D> = vec![];
        let graph = VisibilityGraph::build(
            &lines,
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
            0.0,
            100,
        );

        let path = graph.find_path();
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(path.len(), 2); // Direct path: start -> goal
    }

    #[test]
    fn test_find_path_around_obstacle() {
        let lines = make_room_with_obstacle();
        let graph = VisibilityGraph::build(
            &lines,
            Point2D::new(-3.0, 0.0), // Left of center wall
            Point2D::new(3.0, 0.0),  // Right of center wall
            0.1,
            100,
        );

        let path = graph.find_path();
        assert!(path.is_some());

        let indices = path.unwrap();
        // Should go around the obstacle (more than 2 nodes)
        assert!(indices.len() >= 2);
    }

    #[test]
    fn test_planner_direct_path() {
        let lines = make_room();
        let config = PathPlanningConfig::new()
            .with_robot_radius(0.1)
            .with_conservative(false);
        let planner = PathPlanner::new(config);
        let occupancy_config = OccupancyConfig::default();

        let path = planner.plan(
            Point2D::new(-2.0, 0.0),
            Point2D::new(2.0, 0.0),
            &lines,
            None,
            &occupancy_config,
        );

        assert!(path.is_some());
        let path = path.unwrap();
        assert!(!path.is_empty());
        assert!(path.length > 0.0);
    }

    #[test]
    fn test_planner_around_obstacle() {
        let lines = make_room_with_obstacle();
        let config = PathPlanningConfig::new()
            .with_robot_radius(0.1)
            .with_conservative(false);
        let planner = PathPlanner::new(config);
        let occupancy_config = OccupancyConfig::default();

        let path = planner.plan(
            Point2D::new(-2.0, 0.0),
            Point2D::new(2.0, 0.0),
            &lines,
            None,
            &occupancy_config,
        );

        assert!(path.is_some());
        let path = path.unwrap();
        // Path should go around the obstacle
        assert!(path.length > 4.0); // Longer than direct 4m path
    }

    #[test]
    fn test_is_straight_path_clear_empty() {
        let lines: Vec<Line2D> = vec![];

        assert!(is_straight_path_clear(
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
            &lines,
            0.0,
        ));
    }

    #[test]
    fn test_is_straight_path_clear_blocked() {
        let lines = vec![Line2D::new(Point2D::new(0.5, -1.0), Point2D::new(0.5, 1.0))];

        assert!(!is_straight_path_clear(
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            &lines,
            0.0,
        ));
    }

    #[test]
    fn test_is_straight_path_clear_with_radius() {
        let lines = vec![Line2D::new(Point2D::new(0.5, -1.0), Point2D::new(0.5, 1.0))];

        // Path at y=0.5, parallel to obstacle at x=0.5
        // Should be blocked due to robot radius
        assert!(!is_straight_path_clear(
            Point2D::new(0.0, 0.5),
            Point2D::new(1.0, 0.5),
            &lines,
            0.2, // Robot would clip the wall
        ));
    }

    #[test]
    fn test_is_straight_path_clear_with_clearance() {
        let lines = vec![Line2D::new(Point2D::new(0.5, -1.0), Point2D::new(0.5, 1.0))];

        // Path at y=2.0, far from obstacle
        assert!(is_straight_path_clear(
            Point2D::new(0.0, 2.0),
            Point2D::new(1.0, 2.0),
            &lines,
            0.2,
        ));
    }

    #[test]
    fn test_same_start_goal() {
        let lines = make_room();
        let config = PathPlanningConfig::new().with_conservative(false);
        let planner = PathPlanner::new(config);
        let occupancy_config = OccupancyConfig::default();

        let path = planner.plan(
            Point2D::new(0.0, 0.0),
            Point2D::new(0.0, 0.0),
            &lines,
            None,
            &occupancy_config,
        );

        // Should return a path with just one point
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(path.points.len(), 1);
        assert_eq!(path.length, 0.0);
    }
}
