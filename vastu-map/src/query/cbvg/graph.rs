//! Clearance-Based Visibility Graph implementation.
//!
//! The main graph structure that maintains nodes along the medial axis
//! and edges connecting them for path planning.

use crate::Path;
use crate::core::{Bounds, Point2D};
use crate::features::Line2D;

use super::clearance::{is_edge_safe, is_path_clear_with_clearance, min_distance_to_walls};
use super::config::CBVGConfig;
use super::dijkstra::{DijkstraResult, dijkstra};
use super::node::CBVGNode;
use super::visibility::VisibilityRegion;
use super::voronoi::{generate_medial_axis_nodes, generate_nodes_from_visibility};

/// Clearance-Based Visibility Graph for safe robot navigation.
///
/// The CBVG places nodes along the medial axis of free space, ensuring
/// all waypoints are at a safe distance from obstacles. This provides
/// safer paths compared to traditional visibility graphs that place
/// nodes at obstacle corners.
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::query::cbvg::{ClearanceVisibilityGraph, CBVGConfig};
/// use vastu_map::core::Point2D;
///
/// let config = CBVGConfig::default();
/// let mut graph = ClearanceVisibilityGraph::new(config);
///
/// // Build graph from walls
/// graph.build(&walls, Some(&bounds));
///
/// // Find path
/// if let Some(path) = graph.find_path(start, goal, &walls) {
///     println!("Path found: {} waypoints", path.points.len());
/// }
/// ```
#[derive(Clone, Debug)]
pub struct ClearanceVisibilityGraph {
    /// All nodes in the graph.
    nodes: Vec<CBVGNode>,

    /// Adjacency list: edges[i] = [(neighbor_idx, distance), ...]
    edges: Vec<Vec<(usize, f32)>>,

    /// Configuration.
    config: CBVGConfig,
}

impl ClearanceVisibilityGraph {
    /// Create a new empty graph with the given configuration.
    pub fn new(config: CBVGConfig) -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
            config,
        }
    }

    /// Build the graph from wall segments.
    ///
    /// This clears any existing nodes and rebuilds the entire graph
    /// by computing the medial axis of free space.
    ///
    /// # Arguments
    /// * `lines` - Wall segments
    /// * `bounds` - Optional map bounds (computed from lines if not provided)
    pub fn build(&mut self, lines: &[Line2D], bounds: Option<&Bounds>) {
        self.nodes.clear();
        self.edges.clear();

        if lines.is_empty() {
            return;
        }

        // Generate nodes along the medial axis
        self.nodes = generate_medial_axis_nodes(lines, bounds, &self.config);

        // Build edges between nodes
        self.build_edges(lines);
    }

    /// Build the graph from a visibility region (lidar-scanned areas).
    ///
    /// This only creates nodes in areas that have been observed by lidar,
    /// ensuring the robot doesn't plan paths through unexplored territory.
    ///
    /// # Arguments
    /// * `visibility` - Visibility region tracking scanned areas
    /// * `lines` - Wall segments for clearance computation
    pub fn build_from_visibility(&mut self, visibility: &VisibilityRegion, lines: &[Line2D]) {
        self.nodes.clear();
        self.edges.clear();

        if visibility.is_empty() || lines.is_empty() {
            return;
        }

        // Generate nodes only in visible (scanned) areas
        self.nodes = generate_nodes_from_visibility(visibility, lines, &self.config);

        // Build edges between nodes
        self.build_edges(lines);
    }

    /// Incrementally update the graph with new visibility data.
    ///
    /// This adds nodes in newly visible areas without regenerating
    /// the entire graph.
    ///
    /// # Arguments
    /// * `visibility` - Updated visibility region
    /// * `lines` - All wall segments
    pub fn update_from_visibility(&mut self, visibility: &VisibilityRegion, lines: &[Line2D]) {
        if visibility.is_empty() {
            return;
        }

        // 1. Invalidate nodes that are no longer valid (too close to walls)
        let mut invalid_indices: Vec<usize> = Vec::new();
        for (idx, node) in self.nodes.iter().enumerate() {
            let clearance = min_distance_to_walls(node.position, lines);
            if clearance < self.config.min_clearance_narrow {
                invalid_indices.push(idx);
            }
        }

        // Remove invalid nodes (in reverse order)
        for idx in invalid_indices.into_iter().rev() {
            self.remove_node(idx);
        }

        // 2. Generate new nodes from visibility
        let new_nodes = generate_nodes_from_visibility(visibility, lines, &self.config);

        // 3. Add new nodes that don't already exist
        for node in new_nodes {
            if !self.has_nearby_node(node.position) {
                self.add_node(node, lines);
            }
        }
    }

    /// Build edges between all nodes that can be safely connected.
    fn build_edges(&mut self, lines: &[Line2D]) {
        let n = self.nodes.len();
        self.edges = vec![Vec::new(); n];

        for i in 0..n {
            for j in (i + 1)..n {
                let dist = self.nodes[i].position.distance(self.nodes[j].position);

                // Skip if too far
                if dist > self.config.max_edge_length {
                    continue;
                }

                // Check if edge is safe
                if is_edge_safe(
                    self.nodes[i].position,
                    self.nodes[j].position,
                    lines,
                    self.config.min_clearance,
                    self.config.min_clearance_narrow,
                    self.config.edge_clearance_step,
                ) {
                    self.edges[i].push((j, dist));
                    self.edges[j].push((i, dist));
                }
            }
        }
    }

    /// Update the graph after new walls are added.
    ///
    /// This performs an incremental update:
    /// 1. Invalidates nodes too close to new walls
    /// 2. Generates new nodes in the updated space
    /// 3. Rebuilds affected edges
    ///
    /// # Arguments
    /// * `all_lines` - All wall segments in the map
    /// * `new_lines` - Only the newly added wall segments
    pub fn update(&mut self, all_lines: &[Line2D], new_lines: &[Line2D]) {
        if new_lines.is_empty() {
            return;
        }

        // 1. Invalidate nodes too close to new walls
        let mut invalid_indices: Vec<usize> = Vec::new();
        for (idx, node) in self.nodes.iter().enumerate() {
            for line in new_lines {
                let dist = line.distance_to_point_segment(node.position);
                if dist < self.config.min_clearance_narrow {
                    invalid_indices.push(idx);
                    break;
                }
            }
        }

        // Remove invalid nodes (in reverse order to preserve indices)
        for idx in invalid_indices.into_iter().rev() {
            self.remove_node(idx);
        }

        // 2. Generate new nodes
        let new_nodes = generate_medial_axis_nodes(all_lines, None, &self.config);

        // 3. Add new nodes, avoiding duplicates
        for node in new_nodes {
            if !self.has_nearby_node(node.position) {
                self.add_node(node, all_lines);
            }
        }
    }

    /// Remove a node from the graph.
    fn remove_node(&mut self, idx: usize) {
        if idx >= self.nodes.len() {
            return;
        }

        // Remove from nodes
        self.nodes.remove(idx);

        // Remove from edges and update indices
        self.edges.remove(idx);
        for neighbors in &mut self.edges {
            neighbors.retain(|(neighbor_idx, _)| *neighbor_idx != idx);
            for (neighbor_idx, _) in neighbors.iter_mut() {
                if *neighbor_idx > idx {
                    *neighbor_idx -= 1;
                }
            }
        }
    }

    /// Add a node to the graph and connect it to nearby nodes.
    fn add_node(&mut self, node: CBVGNode, lines: &[Line2D]) {
        let new_idx = self.nodes.len();
        self.nodes.push(node);
        self.edges.push(Vec::new());

        // Connect to existing nodes
        for i in 0..new_idx {
            let dist = self.nodes[i]
                .position
                .distance(self.nodes[new_idx].position);

            if dist <= self.config.max_edge_length
                && is_edge_safe(
                    self.nodes[i].position,
                    self.nodes[new_idx].position,
                    lines,
                    self.config.min_clearance,
                    self.config.min_clearance_narrow,
                    self.config.edge_clearance_step,
                )
            {
                self.edges[i].push((new_idx, dist));
                self.edges[new_idx].push((i, dist));
            }
        }
    }

    /// Check if there's a node near the given position.
    fn has_nearby_node(&self, position: Point2D) -> bool {
        self.nodes
            .iter()
            .any(|n| n.position.distance(position) < self.config.node_merge_distance)
    }

    /// Find a path from start to goal.
    ///
    /// Always routes through CBVG nodes for safe navigation with proper
    /// wall clearance. This ensures predictable paths that follow the
    /// medial axis of free space.
    ///
    /// # Arguments
    /// * `from` - Start position
    /// * `to` - Goal position
    /// * `lines` - Wall segments (for clearance checks)
    ///
    /// # Returns
    /// Path if one exists, None otherwise.
    pub fn find_path(&self, from: Point2D, to: Point2D, lines: &[Line2D]) -> Option<Path> {
        // Always route through CBVG nodes for safe navigation

        // 1. Find nearest reachable node to start
        let start_node = self.find_nearest_reachable(from, lines)?;

        // 2. Find nearest reachable node to goal
        let goal_node = self.find_nearest_reachable(to, lines)?;

        // 3. If same node, just go from -> node -> to
        if start_node == goal_node {
            let node_pos = self.nodes[start_node].position;
            let len1 = from.distance(node_pos);
            let len2 = node_pos.distance(to);
            return Some(Path {
                points: vec![from, node_pos, to],
                length: len1 + len2,
            });
        }

        // 4. Dijkstra search through CBVG nodes
        let result = dijkstra(&self.edges, start_node, goal_node)?;

        // 5. Build full path
        self.build_path(from, to, &result)
    }

    /// Find the nearest node reachable from a point.
    fn find_nearest_reachable(&self, point: Point2D, lines: &[Line2D]) -> Option<usize> {
        self.nodes
            .iter()
            .enumerate()
            .filter(|(_, node)| {
                is_path_clear_with_clearance(
                    point,
                    node.position,
                    lines,
                    self.config.min_clearance_narrow,
                    self.config.edge_clearance_step,
                )
            })
            .min_by(|(_, a), (_, b)| {
                point
                    .distance(a.position)
                    .partial_cmp(&point.distance(b.position))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(idx, _)| idx)
    }

    /// Build the final path from Dijkstra result.
    fn build_path(&self, from: Point2D, to: Point2D, result: &DijkstraResult) -> Option<Path> {
        let mut points = Vec::with_capacity(result.path.len() + 2);
        let mut length = 0.0;

        points.push(from);

        for (i, &idx) in result.path.iter().enumerate() {
            let node_pos = self.nodes[idx].position;

            if i == 0 {
                length += from.distance(node_pos);
            } else {
                length += points.last()?.distance(node_pos);
            }

            points.push(node_pos);
        }

        length += points.last()?.distance(to);
        points.push(to);

        Some(Path { points, length })
    }

    /// Get all nodes in the graph.
    pub fn nodes(&self) -> &[CBVGNode] {
        &self.nodes
    }

    /// Get the number of nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Get the number of edges.
    pub fn edge_count(&self) -> usize {
        self.edges.iter().map(|e| e.len()).sum::<usize>() / 2
    }

    /// Get the adjacency list.
    pub fn edges(&self) -> &[Vec<(usize, f32)>] {
        &self.edges
    }

    /// Get the configuration.
    pub fn config(&self) -> &CBVGConfig {
        &self.config
    }

    /// Check if the graph is empty.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Find the nearest node to a target point.
    ///
    /// Returns the node position if found within max_distance.
    /// This is useful for exploration to find reachable navigation targets.
    ///
    /// # Arguments
    /// * `target` - The point to find the nearest node to
    /// * `max_distance` - Maximum distance to search (nodes farther are ignored)
    ///
    /// # Returns
    /// The position of the nearest node, or None if no node is within range.
    pub fn nearest_node(&self, target: Point2D, max_distance: f32) -> Option<Point2D> {
        self.nodes
            .iter()
            .filter(|n| n.position.distance(target) <= max_distance)
            .min_by(|a, b| {
                a.position
                    .distance(target)
                    .partial_cmp(&b.position.distance(target))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|n| n.position)
    }

    /// Clear all nodes and edges.
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.edges.clear();
    }

    /// Export graph to SVG for visualization.
    ///
    /// # Arguments
    /// * `lines` - Wall segments to draw
    /// * `path` - Optional path to highlight
    ///
    /// # Returns
    /// SVG string that can be written to a file.
    pub fn to_svg(&self, lines: &[Line2D], path: Option<&Path>) -> String {
        // Calculate bounds
        let (min_x, min_y, max_x, max_y) = self.calculate_svg_bounds(lines);

        let width = max_x - min_x;
        let height = max_y - min_y;
        let margin = 0.5;

        let view_min_x = min_x - margin;
        let view_min_y = min_y - margin;
        let view_width = width + 2.0 * margin;
        let view_height = height + 2.0 * margin;

        let mut svg = format!(
            r#"<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="{} {} {} {}" width="800" height="600">
  <rect x="{}" y="{}" width="{}" height="{}" fill="white"/>
  <g transform="scale(1, -1) translate(0, -{})">
"#,
            view_min_x,
            -view_min_y - view_height,
            view_width,
            view_height,
            view_min_x,
            -view_min_y - view_height,
            view_width,
            view_height,
            2.0 * view_min_y + view_height
        );

        // Draw walls (black)
        svg.push_str("    <!-- Walls -->\n");
        for line in lines {
            svg.push_str(&format!(
                r#"    <line x1="{:.3}" y1="{:.3}" x2="{:.3}" y2="{:.3}" stroke="black" stroke-width="0.03"/>"#,
                line.start.x, line.start.y, line.end.x, line.end.y
            ));
            svg.push('\n');
        }

        // Draw edges (light gray)
        svg.push_str("    <!-- Graph Edges -->\n");
        for (i, neighbors) in self.edges.iter().enumerate() {
            for (j, _) in neighbors {
                if i < *j {
                    let p1 = self.nodes[i].position;
                    let p2 = self.nodes[*j].position;
                    svg.push_str(&format!(
                        "    <line x1=\"{:.3}\" y1=\"{:.3}\" x2=\"{:.3}\" y2=\"{:.3}\" stroke=\"#cccccc\" stroke-width=\"0.01\"/>\n",
                        p1.x, p1.y, p2.x, p2.y
                    ));
                }
            }
        }

        // Draw path if provided (thick green)
        if let Some(path) = path {
            svg.push_str("    <!-- Path -->\n");
            for i in 0..path.points.len().saturating_sub(1) {
                let p1 = path.points[i];
                let p2 = path.points[i + 1];
                svg.push_str(&format!(
                    r#"    <line x1="{:.3}" y1="{:.3}" x2="{:.3}" y2="{:.3}" stroke="lime" stroke-width="0.05"/>"#,
                    p1.x, p1.y, p2.x, p2.y
                ));
                svg.push('\n');
            }
        }

        // Draw nodes
        svg.push_str("    <!-- Graph Nodes -->\n");
        for node in &self.nodes {
            let color = node.svg_color();
            let radius = if node.is_temporary() { 0.08 } else { 0.05 };
            svg.push_str(&format!(
                r#"    <circle cx="{:.3}" cy="{:.3}" r="{}" fill="{}"/>"#,
                node.position.x, node.position.y, radius, color
            ));
            svg.push('\n');
        }

        svg.push_str("  </g>\n</svg>\n");
        svg
    }

    /// Calculate bounds for SVG export.
    fn calculate_svg_bounds(&self, lines: &[Line2D]) -> (f32, f32, f32, f32) {
        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;

        for line in lines {
            min_x = min_x.min(line.start.x).min(line.end.x);
            min_y = min_y.min(line.start.y).min(line.end.y);
            max_x = max_x.max(line.start.x).max(line.end.x);
            max_y = max_y.max(line.start.y).max(line.end.y);
        }

        for node in &self.nodes {
            min_x = min_x.min(node.position.x);
            min_y = min_y.min(node.position.y);
            max_x = max_x.max(node.position.x);
            max_y = max_y.max(node.position.y);
        }

        if min_x.is_infinite() {
            (0.0, 0.0, 1.0, 1.0)
        } else {
            (min_x, min_y, max_x, max_y)
        }
    }

    /// Add a temporary node for the robot's current position.
    ///
    /// Returns the index of the added node.
    pub fn add_robot_node(&mut self, position: Point2D, lines: &[Line2D]) -> usize {
        let clearance = min_distance_to_walls(position, lines);
        let node = CBVGNode::robot(position, clearance);
        self.add_node(node, lines);
        self.nodes.len() - 1
    }

    /// Add a temporary node for a goal position.
    ///
    /// Returns the index of the added node.
    pub fn add_goal_node(&mut self, position: Point2D, lines: &[Line2D]) -> usize {
        let clearance = min_distance_to_walls(position, lines);
        let node = CBVGNode::goal(position, clearance);
        self.add_node(node, lines);
        self.nodes.len() - 1
    }

    /// Remove all temporary nodes (robot and goal nodes).
    pub fn remove_temporary_nodes(&mut self) {
        let mut indices_to_remove: Vec<usize> = self
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.is_temporary())
            .map(|(i, _)| i)
            .collect();

        // Remove in reverse order
        indices_to_remove.sort_by(|a, b| b.cmp(a));
        for idx in indices_to_remove {
            self.remove_node(idx);
        }
    }
}

impl Default for ClearanceVisibilityGraph {
    fn default() -> Self {
        Self::new(CBVGConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_room() -> Vec<Line2D> {
        vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(4.0, 0.0)), // Bottom
            Line2D::new(Point2D::new(4.0, 0.0), Point2D::new(4.0, 3.0)), // Right
            Line2D::new(Point2D::new(4.0, 3.0), Point2D::new(0.0, 3.0)), // Top
            Line2D::new(Point2D::new(0.0, 3.0), Point2D::new(0.0, 0.0)), // Left
        ]
    }

    fn make_room_with_obstacle() -> Vec<Line2D> {
        let mut lines = make_room();
        // Add a vertical wall in the middle
        lines.push(Line2D::new(Point2D::new(2.0, 0.0), Point2D::new(2.0, 2.0)));
        lines
    }

    #[test]
    fn test_build_empty() {
        let mut graph = ClearanceVisibilityGraph::default();
        graph.build(&[], None);
        assert!(graph.is_empty());
    }

    #[test]
    fn test_build_simple_room() {
        let walls = make_room();
        let config = CBVGConfig::default()
            .with_voronoi_sample_step(0.2)
            .with_min_clearance(0.3)
            .with_min_clearance_narrow(0.18);

        let mut graph = ClearanceVisibilityGraph::new(config);
        graph.build(&walls, None);

        // Should have some nodes
        assert!(!graph.is_empty());

        // All nodes should have sufficient clearance
        for node in graph.nodes() {
            assert!(node.clearance >= 0.18);
        }
    }

    #[test]
    fn test_find_path_direct() {
        let walls = make_room();
        let config = CBVGConfig::default()
            .with_voronoi_sample_step(0.2)
            .with_min_clearance(0.2)
            .with_min_clearance_narrow(0.1);

        let mut graph = ClearanceVisibilityGraph::new(config);
        graph.build(&walls, None);

        // Direct path in open space
        let path = graph.find_path(Point2D::new(1.0, 1.5), Point2D::new(3.0, 1.5), &walls);

        assert!(path.is_some());
        let path = path.unwrap();
        assert!(!path.is_empty());
        assert!(path.length > 0.0);
    }

    #[test]
    fn test_find_path_around_obstacle() {
        let walls = make_room_with_obstacle();
        let config = CBVGConfig::default()
            .with_voronoi_sample_step(0.15)
            .with_min_clearance(0.2)
            .with_min_clearance_narrow(0.1);

        let mut graph = ClearanceVisibilityGraph::new(config);
        graph.build(&walls, None);

        // Path that must go around obstacle
        let path = graph.find_path(Point2D::new(1.0, 1.0), Point2D::new(3.0, 1.0), &walls);

        // Path should exist and go around
        if let Some(path) = path {
            assert!(path.points.len() >= 2);
            // Path should be longer than direct
            assert!(path.length > 2.0);
        }
    }

    #[test]
    fn test_add_remove_temporary_nodes() {
        let walls = make_room();
        let mut graph = ClearanceVisibilityGraph::default();
        graph.build(&walls, None);

        let initial_count = graph.node_count();

        // Add robot node
        let robot_idx = graph.add_robot_node(Point2D::new(1.0, 1.0), &walls);
        assert_eq!(graph.node_count(), initial_count + 1);
        assert!(graph.nodes()[robot_idx].is_temporary());

        // Add goal node
        graph.add_goal_node(Point2D::new(3.0, 2.0), &walls);
        assert_eq!(graph.node_count(), initial_count + 2);

        // Remove temporary nodes
        graph.remove_temporary_nodes();
        assert_eq!(graph.node_count(), initial_count);
    }

    #[test]
    fn test_svg_export() {
        let walls = make_room();
        let mut graph = ClearanceVisibilityGraph::default();
        graph.build(&walls, None);

        let svg = graph.to_svg(&walls, None);
        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));
        assert!(svg.contains("circle")); // Nodes
        assert!(svg.contains("line")); // Walls/edges
    }

    #[test]
    fn test_update_incremental() {
        let walls = make_room();
        let config = CBVGConfig::default().with_voronoi_sample_step(0.2);

        let mut graph = ClearanceVisibilityGraph::new(config);
        graph.build(&walls, None);

        let _initial_count = graph.node_count();

        // Add a new wall
        let new_wall = vec![Line2D::new(Point2D::new(2.0, 0.0), Point2D::new(2.0, 1.5))];

        let mut all_walls = walls.clone();
        all_walls.extend(new_wall.iter().cloned());

        graph.update(&all_walls, &new_wall);

        // Graph should be updated (nodes may change)
        // Just verify it doesn't crash and has nodes
        assert!(graph.node_count() > 0);
    }

    #[test]
    fn test_build_from_visibility() {
        use crate::core::Pose2D;

        let walls = make_room();
        let config = CBVGConfig::default()
            .with_coverage_node_interval(0.3)
            .with_min_clearance(0.3)
            .with_min_clearance_narrow(0.18);

        // Create visibility region with scan from center of room
        let mut visibility = VisibilityRegion::new(0.1);
        let robot_pose = Pose2D::new(2.0, 1.5, 0.0);

        // Simulate a circular scan
        let scan_points: Vec<Point2D> = (0..36)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::PI / 18.0;
                let dist = 1.5;
                robot_pose.transform_point(Point2D::from_polar(angle, dist))
            })
            .collect();

        visibility.update_from_scan(robot_pose, &scan_points);

        let mut graph = ClearanceVisibilityGraph::new(config);
        graph.build_from_visibility(&visibility, &walls);

        // Should have nodes
        assert!(!graph.is_empty());

        // All nodes should be in visible area
        for node in graph.nodes() {
            assert!(
                visibility.is_visible(node.position),
                "Node at {:?} is not in visible area",
                node.position
            );
        }
    }

    #[test]
    fn test_update_from_visibility() {
        use crate::core::Pose2D;

        let walls = make_room();
        let config = CBVGConfig::default()
            .with_coverage_node_interval(0.3)
            .with_min_clearance(0.3);

        // Initial visibility from one corner
        let mut visibility = VisibilityRegion::new(0.1);
        let pose1 = Pose2D::new(1.0, 1.0, 0.0);
        let scan1: Vec<Point2D> = (0..36)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::PI / 18.0;
                pose1.transform_point(Point2D::from_polar(angle, 1.0))
            })
            .collect();
        visibility.update_from_scan(pose1, &scan1);

        let mut graph = ClearanceVisibilityGraph::new(config);
        graph.build_from_visibility(&visibility, &walls);
        let initial_count = graph.node_count();

        // Add second scan from different location
        let pose2 = Pose2D::new(3.0, 2.0, 0.0);
        let scan2: Vec<Point2D> = (0..36)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::PI / 18.0;
                pose2.transform_point(Point2D::from_polar(angle, 1.0))
            })
            .collect();
        visibility.update_from_scan(pose2, &scan2);

        // Update graph with new visibility
        graph.update_from_visibility(&visibility, &walls);

        // Should have more nodes now (or at least the same)
        assert!(graph.node_count() >= initial_count);
    }
}
