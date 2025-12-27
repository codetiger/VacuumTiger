//! Clearance-Based Visibility Graph implementation.
//!
//! The main graph structure that maintains nodes along the medial axis
//! and edges connecting them for path planning.

use std::collections::HashSet;
use std::f32::consts::PI;

use crate::core::{Bounds, Point2D, Pose2D};
use crate::features::Line2D;
use crate::{Frontier, Path};

use super::clearance::{is_edge_safe, is_path_clear_with_clearance, min_distance_to_walls};
use super::config::CBVGConfig;
use super::dijkstra::{DijkstraResult, dijkstra};
use super::node::CBVGNode;
use super::visibility::VisibilityRegion;
use super::voronoi::generate_grid_nodes;

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

    /// Counter for periodic clique optimization (currently disabled).
    #[allow(dead_code)]
    scans_since_optimize: usize,

    /// Visibility region tracking (currently unused - using ray-based approach).
    #[allow(dead_code)]
    visibility: VisibilityRegion,
}

impl ClearanceVisibilityGraph {
    /// Create a new empty graph with the given configuration.
    pub fn new(config: CBVGConfig) -> Self {
        // Use coverage_node_interval as visibility grid resolution so each cell
        // corresponds to one potential node position
        let visibility = VisibilityRegion::new(config.coverage_node_interval);
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
            config,
            scans_since_optimize: 0,
            visibility,
        }
    }

    /// Build the graph from wall segments.
    ///
    /// This clears any existing nodes and rebuilds the entire graph
    /// by computing the medial axis of free space. Automatically optimizes
    /// the graph by merging visibility cliques into centroids.
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

        // Generate nodes on a uniform grid with clearance checks
        self.nodes = generate_grid_nodes(lines, bounds, &self.config);

        // Build edges between nodes
        self.build_edges(lines);

        // Optimize by merging visibility cliques into centroids
        self.optimize_nodes(lines);
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

        // 1. Remove nodes too close to ANY wall (using min_clearance, not min_clearance_narrow)
        // This catches nodes that were valid when created but are now too close
        // due to newly discovered walls OR were created with looser constraints
        let original_count = self.nodes.len();
        self.nodes.retain(|node| {
            min_distance_to_walls(node.position, all_lines) >= self.config.min_clearance
        });

        let removed = original_count - self.nodes.len();
        if removed > 0 {
            log::debug!("update: removed {} nodes too close to walls", removed);
        }

        // 2. Generate new nodes
        let new_nodes = generate_grid_nodes(all_lines, None, &self.config);

        // 3. Add new nodes, avoiding duplicates
        for node in new_nodes {
            if !self.has_nearby_node(node.position) {
                self.add_node(node, all_lines);
            }
        }
    }

    /// Add nodes from a single lidar scan using visibility-filtered grid sampling.
    ///
    /// This method provides uniform node distribution in visible areas:
    /// 1. Updates the visibility region with the new scan
    /// 2. Gets newly visible cells from the visibility grid
    /// 3. Places nodes at cell centers with deterministic jitter
    /// 4. Runs clique optimization periodically
    ///
    /// The visibility grid resolution matches `coverage_node_interval`, so each
    /// visible cell can generate at most one node, ensuring uniform spacing.
    ///
    /// # Arguments
    /// * `robot_pose` - Current robot pose (position and heading)
    /// * `scan_points` - Lidar scan endpoints in world coordinates
    /// * `lines` - Wall segments for clearance computation
    pub fn add_nodes_from_scan(
        &mut self,
        robot_pose: Pose2D,
        scan_points: &[Point2D],
        lines: &[Line2D],
    ) {
        if scan_points.is_empty() {
            return;
        }

        let robot_pos = robot_pose.position();

        // First, add a node at the robot position if it has good clearance.
        // This ensures the robot's current position is connected to the graph.
        if !self.has_nearby_node(robot_pos) && self.nodes.len() < self.config.max_nodes {
            // Check clearance from existing wall lines
            let clearance_from_lines = if lines.is_empty() {
                f32::INFINITY
            } else {
                min_distance_to_walls(robot_pos, lines)
            };

            // Check clearance from ALL scan points (current scan's wall positions)
            let clearance_from_scan = scan_points
                .iter()
                .map(|p| robot_pos.distance(*p))
                .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap_or(f32::INFINITY);

            // Use the minimum clearance from both sources
            let clearance = clearance_from_lines.min(clearance_from_scan);

            // Node must be at least min_clearance from any wall
            if clearance >= self.config.min_clearance {
                let node = CBVGNode::medial_axis(robot_pos, clearance, false);
                self.add_node(node, lines);
            }
        }

        // Add nodes along rays from robot to each scan point.
        // This ensures nodes are only placed in FREE SPACE (before the wall),
        // never at or beyond the scan point which represents the wall location.
        // We pass all scan_points so we can check distance to ALL walls (scan points),
        // not just the one we're tracing along.
        for scan_point in scan_points {
            self.add_nodes_along_ray(robot_pos, *scan_point, lines, scan_points);
        }
    }

    /// Add nodes along a ray from robot position toward scan endpoint.
    ///
    /// The scan endpoint represents where the lidar hit a wall, so nodes
    /// are only placed in the FREE SPACE before the wall, maintaining
    /// minimum clearance from ALL walls (both existing lines and scan points).
    ///
    /// # Arguments
    /// * `robot_pos` - Current robot position
    /// * `scan_point` - Lidar scan endpoint (wall location) in world coordinates
    /// * `lines` - Existing wall segments for clearance computation
    /// * `all_scan_points` - All scan points from current scan (represent walls not yet in lines)
    fn add_nodes_along_ray(
        &mut self,
        robot_pos: Point2D,
        scan_point: Point2D,
        lines: &[Line2D],
        all_scan_points: &[Point2D],
    ) {
        // Cap ray length at max_lidar_range for node creation
        // (wall detection still uses full scan range)
        let actual_distance = robot_pos.distance(scan_point);
        let ray_length = actual_distance.min(self.config.max_lidar_range);

        // Calculate unit direction for node placement
        let dir_x = (scan_point.x - robot_pos.x) / actual_distance;
        let dir_y = (scan_point.y - robot_pos.y) / actual_distance;

        // Only maintain clearance from wall if wall is within our node range
        let safe_length = if actual_distance <= self.config.max_lidar_range {
            ray_length - self.config.min_clearance // Stop before wall
        } else {
            ray_length // No wall in range, use full node range
        };

        if safe_length < self.config.coverage_node_interval * 0.5 {
            return; // Not enough room for any nodes
        }

        // Limit total nodes
        if self.nodes.len() >= self.config.max_nodes {
            return;
        }

        // Sample points along the ray at coverage_node_interval spacing
        // Only sample up to safe_length
        let num_samples = (safe_length / self.config.coverage_node_interval).floor() as usize;
        if num_samples == 0 {
            return;
        }

        for i in 1..=num_samples {
            let distance = i as f32 * self.config.coverage_node_interval;

            // Safety check: never place nodes within min_clearance of a nearby wall
            if actual_distance <= self.config.max_lidar_range {
                let dist_to_wall = actual_distance - distance;
                if dist_to_wall < self.config.min_clearance {
                    break;
                }
            }

            // Place node at distance along ray direction
            let candidate = Point2D::new(
                robot_pos.x + distance * dir_x,
                robot_pos.y + distance * dir_y,
            );

            // Check for duplicates first (fast check)
            if self.has_nearby_node(candidate) {
                continue;
            }

            // Check clearance from existing wall lines
            let clearance_from_lines = if lines.is_empty() {
                f32::INFINITY
            } else {
                min_distance_to_walls(candidate, lines)
            };

            // Check clearance from ALL scan points (current scan's wall positions)
            let clearance_from_scan = all_scan_points
                .iter()
                .map(|p| candidate.distance(*p))
                .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap_or(f32::INFINITY);

            // Use the minimum clearance from both sources
            let clearance = clearance_from_lines.min(clearance_from_scan);

            if clearance < self.config.min_clearance {
                continue; // Too close to wall
            }

            // Add node (all nodes now have good clearance, not narrow)
            let node = CBVGNode::medial_axis(candidate, clearance, false);
            self.add_node(node, lines);

            // Limit total nodes
            if self.nodes.len() >= self.config.max_nodes {
                break;
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
        let start_node = match self.find_nearest_reachable(from, lines) {
            Some(idx) => idx,
            None => {
                log::debug!(
                    "CBVG: No reachable node from start ({:.2}, {:.2}). Graph has {} nodes, {} edges",
                    from.x,
                    from.y,
                    self.nodes.len(),
                    self.edge_count()
                );
                return None;
            }
        };

        // 2. Find nearest reachable node to goal
        let goal_node = match self.find_nearest_reachable(to, lines) {
            Some(idx) => idx,
            None => {
                log::debug!(
                    "CBVG: No reachable node from goal ({:.2}, {:.2}). Graph has {} nodes, {} edges",
                    to.x,
                    to.y,
                    self.nodes.len(),
                    self.edge_count()
                );
                return None;
            }
        };

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
        let result = match dijkstra(&self.edges, start_node, goal_node) {
            Some(r) => r,
            None => {
                log::debug!(
                    "CBVG: Dijkstra failed from node {} to node {}. Graph may be disconnected ({} nodes, {} edges)",
                    start_node,
                    goal_node,
                    self.nodes.len(),
                    self.edge_count()
                );
                return None;
            }
        };

        // 5. Build full path
        self.build_path(from, to, &result)
    }

    /// Find the nearest node reachable from a point.
    fn find_nearest_reachable(&self, point: Point2D, lines: &[Line2D]) -> Option<usize> {
        self.find_nearest_reachable_excluding(point, lines, &[])
    }

    /// Find the nearest node reachable from a point, excluding specific node indices.
    /// Used to find alternative approach angles after collision.
    fn find_nearest_reachable_excluding(
        &self,
        point: Point2D,
        lines: &[Line2D],
        excluded: &[usize],
    ) -> Option<usize> {
        self.nodes
            .iter()
            .enumerate()
            .filter(|(idx, _)| !excluded.contains(idx))
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

    /// Find a path from start to goal, excluding specific goal node indices.
    ///
    /// This is used to find alternative approach angles after a collision.
    /// When a path to a frontier causes a collision, the goal node that led
    /// to the collision can be excluded, forcing the planner to find an
    /// alternative node (and thus a different approach angle).
    ///
    /// # Arguments
    /// * `from` - Start position
    /// * `to` - Goal position
    /// * `lines` - Wall segments (for clearance checks)
    /// * `excluded_goal_nodes` - Node indices to exclude when finding the goal node
    ///
    /// # Returns
    /// Tuple of (Path, goal_node_index) if path exists, None otherwise.
    pub fn find_path_excluding_nodes(
        &self,
        from: Point2D,
        to: Point2D,
        lines: &[Line2D],
        excluded_goal_nodes: &[usize],
    ) -> Option<(Path, usize)> {
        // 1. Find nearest reachable node to start (not excluded - we need to start somewhere)
        let start_node = match self.find_nearest_reachable(from, lines) {
            Some(idx) => idx,
            None => {
                log::debug!(
                    "CBVG: No reachable node from start ({:.2}, {:.2})",
                    from.x,
                    from.y
                );
                return None;
            }
        };

        // 2. Find nearest reachable node to goal, EXCLUDING specified nodes
        let goal_node = match self.find_nearest_reachable_excluding(to, lines, excluded_goal_nodes)
        {
            Some(idx) => idx,
            None => {
                log::debug!(
                    "CBVG: No reachable goal node from ({:.2}, {:.2}) after excluding {} nodes",
                    to.x,
                    to.y,
                    excluded_goal_nodes.len()
                );
                return None;
            }
        };

        // 3. If same node, just go from -> node -> to
        if start_node == goal_node {
            let node_pos = self.nodes[start_node].position;
            let len1 = from.distance(node_pos);
            let len2 = node_pos.distance(to);
            return Some((
                Path {
                    points: vec![from, node_pos, to],
                    length: len1 + len2,
                },
                goal_node,
            ));
        }

        // 4. Dijkstra search through CBVG nodes
        let result = match dijkstra(&self.edges, start_node, goal_node) {
            Some(r) => r,
            None => {
                log::debug!(
                    "CBVG: Dijkstra failed from node {} to node {}",
                    start_node,
                    goal_node
                );
                return None;
            }
        };

        // 5. Build full path and return with goal node index
        self.build_path(from, to, &result)
            .map(|path| (path, goal_node))
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

    // ─────────────────────────────────────────────────────────────────────────
    // Frontier Detection
    // ─────────────────────────────────────────────────────────────────────────

    /// Detect frontier nodes - nodes with uncovered angular gaps.
    ///
    /// A node is a frontier if it has a gap of at least `min_frontier_angle` degrees
    /// where there is neither a neighboring node nor a wall.
    ///
    /// # Arguments
    /// * `lines` - Wall segments for checking wall coverage
    /// * `min_frontier_angle` - Minimum uncovered angle to be a frontier (radians)
    /// * `max_wall_distance` - Maximum distance to check for walls (meters)
    ///
    /// # Returns
    /// Tuple of (frontiers, frontier_node_indices) for visualization
    pub fn detect_frontiers(
        &self,
        lines: &[Line2D],
        min_frontier_angle: f32,
        max_wall_distance: f32,
    ) -> (Vec<Frontier>, HashSet<usize>) {
        let mut frontiers = Vec::new();
        let mut frontier_indices = HashSet::new();

        for (node_idx, node) in self.nodes.iter().enumerate() {
            // Skip temporary nodes (robot/goal)
            if node.is_temporary() {
                continue;
            }

            if let Some((frontier, _gap_angle)) =
                self.check_node_frontier(node_idx, lines, min_frontier_angle, max_wall_distance)
            {
                frontiers.push(frontier);
                frontier_indices.insert(node_idx);
            }
        }

        (frontiers, frontier_indices)
    }

    /// Check if a single node is a frontier.
    ///
    /// Returns the frontier and the gap angle if this node is a frontier.
    fn check_node_frontier(
        &self,
        node_idx: usize,
        lines: &[Line2D],
        min_frontier_angle: f32,
        max_wall_distance: f32,
    ) -> Option<(Frontier, f32)> {
        let node = &self.nodes[node_idx];
        let mut covered_arcs: Vec<(f32, f32)> = Vec::new();

        // Add arcs covered by neighboring nodes
        for &(neighbor_idx, distance) in &self.edges[node_idx] {
            let neighbor = &self.nodes[neighbor_idx];
            let dx = neighbor.position.x - node.position.x;
            let dy = neighbor.position.y - node.position.y;
            let angle = dy.atan2(dx);

            // Each neighbor covers a cone - wider for closer neighbors
            // half_cone = atan(node_spacing / 2 / distance)
            let half_cone = (self.config.node_merge_distance / 2.0 / distance)
                .atan()
                .max(0.15); // At least ~8 degrees
            covered_arcs.push((angle - half_cone, angle + half_cone));
        }

        // Add arcs covered by nearby walls
        for line in lines {
            let distance = line.distance_to_point(node.position);

            if distance < max_wall_distance {
                // Compute angles to line endpoints
                let dx1 = line.start.x - node.position.x;
                let dy1 = line.start.y - node.position.y;
                let dx2 = line.end.x - node.position.x;
                let dy2 = line.end.y - node.position.y;

                let angle1 = dy1.atan2(dx1);
                let angle2 = dy2.atan2(dx2);

                // Handle angle wrapping - find the shorter arc
                let (start, end) = normalize_arc(angle1, angle2);
                covered_arcs.push((start, end));
            }
        }

        // Find the largest uncovered gap
        if let Some((gap_start, gap_end, gap_size)) = find_largest_gap(&covered_arcs)
            && gap_size >= min_frontier_angle
        {
            // This node is a frontier!
            let frontier_angle = normalize_angle((gap_start + gap_end) / 2.0);
            let look_direction = Point2D::new(frontier_angle.cos(), frontier_angle.sin());

            return Some((
                Frontier {
                    viewpoint: node.position,
                    look_direction,
                    endpoint: node.position,
                    line_idx: 0,
                    estimated_area: gap_size * 2.0, // Rough estimate
                },
                gap_size,
            ));
        }

        None
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

    /// Export graph to SVG with frontier nodes highlighted.
    ///
    /// Frontier nodes are shown in magenta with direction arrows.
    ///
    /// # Arguments
    /// * `lines` - Wall segments to draw
    /// * `path` - Optional path to highlight
    /// * `frontiers` - Frontier data with directions
    /// * `frontier_indices` - Set of node indices that are frontiers
    pub fn to_svg_with_frontiers(
        &self,
        lines: &[Line2D],
        path: Option<&Path>,
        frontiers: &[Frontier],
        frontier_indices: &HashSet<usize>,
    ) -> String {
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

        // Draw regular nodes (blue/orange)
        svg.push_str("    <!-- Graph Nodes -->\n");
        for (idx, node) in self.nodes.iter().enumerate() {
            if frontier_indices.contains(&idx) {
                continue; // Draw frontiers separately
            }
            let color = node.svg_color();
            let radius = if node.is_temporary() { 0.08 } else { 0.05 };
            svg.push_str(&format!(
                r#"    <circle cx="{:.3}" cy="{:.3}" r="{}" fill="{}"/>"#,
                node.position.x, node.position.y, radius, color
            ));
            svg.push('\n');
        }

        // Draw frontier nodes (magenta) with direction arrows
        svg.push_str("    <!-- Frontier Nodes -->\n");
        for frontier in frontiers {
            let pos = frontier.viewpoint;
            let dir = frontier.look_direction;

            // Draw node circle (magenta)
            svg.push_str(&format!(
                r#"    <circle cx="{:.3}" cy="{:.3}" r="0.08" fill="magenta" stroke="black" stroke-width="0.01"/>"#,
                pos.x, pos.y
            ));
            svg.push('\n');

            // Draw direction arrow
            let arrow_len = 0.3;
            let arrow_end_x = pos.x + dir.x * arrow_len;
            let arrow_end_y = pos.y + dir.y * arrow_len;
            svg.push_str(&format!(
                r#"    <line x1="{:.3}" y1="{:.3}" x2="{:.3}" y2="{:.3}" stroke="magenta" stroke-width="0.02" marker-end="url(#arrow)"/>"#,
                pos.x, pos.y, arrow_end_x, arrow_end_y
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

    /// Remove nodes that are too close to scan endpoints.
    ///
    /// This catches nodes that were valid when created but are now
    /// too close due to newly discovered obstacles from subsequent scans.
    /// Unlike wall-line-based checks, this uses raw scan points which are
    /// available immediately before line fitting occurs.
    ///
    /// # Arguments
    /// * `scan_points` - World-frame scan endpoints to check against
    /// * `lines` - Wall segments for rebuilding edges after cleanup
    pub fn cleanup_nodes_near_scan_points(&mut self, scan_points: &[Point2D], lines: &[Line2D]) {
        if scan_points.is_empty() {
            return;
        }

        let original_count = self.nodes.len();
        self.nodes.retain(|node| {
            // Find minimum distance to any scan point
            let min_dist = scan_points
                .iter()
                .map(|p| node.position.distance(*p))
                .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap_or(f32::INFINITY);

            min_dist >= self.config.min_clearance
        });

        let removed = original_count - self.nodes.len();
        if removed > 0 {
            log::debug!(
                "cleanup_nodes_near_scan_points: removed {} nodes too close to scan points",
                removed
            );
            // Rebuild edges after removing nodes (indices are invalidated)
            self.build_edges(lines);
        }
    }

    /// Remove edges that pass too close to scan endpoints.
    ///
    /// An edge between two nodes may pass through or near an obstacle even if
    /// both nodes have sufficient clearance. This method removes edges where
    /// any scan point has a perpendicular distance to the edge less than
    /// `min_clearance`.
    ///
    /// # Arguments
    /// * `scan_points` - World-frame scan endpoints to check against
    pub fn cleanup_edges_near_scan_points(&mut self, scan_points: &[Point2D]) {
        if scan_points.is_empty() || self.nodes.is_empty() {
            return;
        }

        let mut removed_count = 0;

        // Process each node's edge list
        for i in 0..self.edges.len() {
            let node_i_pos = self.nodes[i].position;

            // Filter edges that are too close to scan points
            self.edges[i].retain(|(j, _dist)| {
                let node_j_pos = self.nodes[*j].position;

                // Create a line segment representing the edge
                let edge_line = Line2D::new(node_i_pos, node_j_pos);

                // Find minimum perpendicular distance from any scan point to the edge
                let min_dist_to_edge = scan_points
                    .iter()
                    .map(|p| edge_line.distance_to_point_segment(*p))
                    .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                    .unwrap_or(f32::INFINITY);

                if min_dist_to_edge < self.config.min_clearance {
                    removed_count += 1;
                    false // Remove this edge
                } else {
                    true // Keep this edge
                }
            });
        }

        // Note: removed_count counts removals from one direction only (i -> j)
        // The j -> i edge will be removed when we process node j's edges
        if removed_count > 0 {
            log::debug!(
                "cleanup_edges_near_scan_points: removed {} edge references too close to scan points",
                removed_count
            );
        }
    }

    /// Optimize the graph by merging clusters of mutually visible nodes.
    ///
    /// This finds cliques (fully connected subgraphs where all nodes can see each other)
    /// and merges them into single centroid nodes. This reduces redundant path options
    /// and simplifies the graph.
    ///
    /// # Arguments
    /// * `lines` - Wall segments for visibility checks
    pub fn optimize_nodes(&mut self, lines: &[Line2D]) {
        // Step 0: Remove nodes that are too close to walls
        // This catches nodes that were valid when created but are now too close
        // due to newly discovered walls from subsequent scans
        let original_count = self.nodes.len();
        self.nodes.retain(|node| {
            let clearance = min_distance_to_walls(node.position, lines);
            clearance >= self.config.min_clearance
        });
        let removed = original_count - self.nodes.len();
        if removed > 0 {
            log::debug!(
                "optimize_nodes: removed {} nodes too close to walls",
                removed
            );
        }

        let n = self.nodes.len();
        if n < self.config.min_clique_size_for_merge {
            // Rebuild edges after removal even if we don't do clique merging
            if removed > 0 {
                self.build_edges(lines);
            }
            return; // Not enough nodes to form a clique worth merging
        }

        // Step 1: Build visibility matrix (which nodes can see each other)
        let visible = self.build_visibility_matrix(lines);

        // Step 2: Find greedy maximal cliques
        let cliques = Self::find_greedy_cliques(&visible);

        // Step 3: Merge cliques into centroids
        let mut new_nodes: Vec<CBVGNode> = Vec::new();

        for clique in cliques {
            if clique.len() >= self.config.min_clique_size_for_merge {
                // Merge clique to centroid
                let centroid_pos = Self::compute_centroid(&clique, &self.nodes);
                let centroid_clearance = min_distance_to_walls(centroid_pos, lines);

                if centroid_clearance >= self.config.min_clearance {
                    // Valid centroid - create merged node
                    new_nodes.push(CBVGNode::medial_axis(
                        centroid_pos,
                        centroid_clearance,
                        false,
                    ));
                } else {
                    // Centroid too close to wall - keep original nodes
                    for &idx in &clique {
                        new_nodes.push(self.nodes[idx].clone());
                    }
                }
            } else {
                // Clique too small to merge - keep original nodes
                for &idx in &clique {
                    new_nodes.push(self.nodes[idx].clone());
                }
            }
        }

        // Step 4: Replace nodes and rebuild edges
        self.nodes = new_nodes;
        self.build_edges(lines);
    }

    /// Build a visibility matrix showing which nodes can see each other.
    /// Uses the same constraints as build_edges() to ensure consistency.
    fn build_visibility_matrix(&self, lines: &[Line2D]) -> Vec<Vec<bool>> {
        let n = self.nodes.len();
        let mut visible = vec![vec![false; n]; n];

        for i in 0..n {
            for j in (i + 1)..n {
                let dist = self.nodes[i].position.distance(self.nodes[j].position);

                // Must be within max_edge_length (same as build_edges)
                if dist > self.config.max_edge_length {
                    continue;
                }

                // Check if nodes can see each other with safe clearance
                if is_edge_safe(
                    self.nodes[i].position,
                    self.nodes[j].position,
                    lines,
                    self.config.min_clearance,
                    self.config.min_clearance_narrow,
                    self.config.edge_clearance_step,
                ) {
                    visible[i][j] = true;
                    visible[j][i] = true;
                }
            }
        }

        visible
    }

    /// Find greedy maximal cliques from a visibility matrix.
    ///
    /// Uses a greedy algorithm that picks the node with most visible neighbors
    /// as seed, then greedily adds nodes that can see all clique members.
    fn find_greedy_cliques(visible: &[Vec<bool>]) -> Vec<Vec<usize>> {
        // Safety limit: max iterations = 2x the number of nodes (each iteration removes at least 1 node)
        const MAX_ITERATIONS: usize = 10000;

        let n = visible.len();
        let mut unassigned: std::collections::HashSet<usize> = (0..n).collect();
        let mut cliques: Vec<Vec<usize>> = Vec::new();
        let mut iterations = 0;

        while !unassigned.is_empty() {
            iterations += 1;
            if iterations > MAX_ITERATIONS {
                log::warn!(
                    "find_greedy_cliques: exceeded {} iterations with {} nodes remaining, assigning as singletons",
                    MAX_ITERATIONS,
                    unassigned.len()
                );
                // Assign remaining nodes as singleton cliques
                for node in unassigned {
                    cliques.push(vec![node]);
                }
                break;
            }

            // Find seed with most visible neighbors among unassigned nodes
            let seed = *unassigned
                .iter()
                .max_by_key(|&&i| unassigned.iter().filter(|&&j| visible[i][j]).count())
                .unwrap();

            let mut clique = vec![seed];

            // Get candidates: unassigned nodes visible to seed, with their visibility counts
            let mut candidates: Vec<(usize, usize)> = unassigned
                .iter()
                .filter(|&&j| j != seed && visible[seed][j])
                .map(|&j| {
                    // Count how many other candidates this node can see
                    let vis_count = unassigned.iter().filter(|&&k| visible[j][k]).count();
                    (j, vis_count)
                })
                .collect();

            // Sort by visibility count descending (greedy heuristic - prefer well-connected nodes)
            candidates.sort_by_key(|&(_, count)| std::cmp::Reverse(count));

            // Greedily add nodes that can see ALL current clique members
            for (candidate, _) in candidates {
                if clique.iter().all(|&m| visible[candidate][m]) {
                    clique.push(candidate);
                }
            }

            // Remove clique nodes from unassigned
            for &node in &clique {
                unassigned.remove(&node);
            }

            cliques.push(clique);
        }

        cliques
    }

    /// Compute the centroid (average position) of a set of nodes.
    fn compute_centroid(indices: &[usize], nodes: &[CBVGNode]) -> Point2D {
        let sum_x: f32 = indices.iter().map(|&i| nodes[i].position.x).sum();
        let sum_y: f32 = indices.iter().map(|&i| nodes[i].position.y).sum();
        let count = indices.len() as f32;
        Point2D::new(sum_x / count, sum_y / count)
    }
}

impl Default for ClearanceVisibilityGraph {
    fn default() -> Self {
        Self::new(CBVGConfig::default())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper functions for angular gap detection
// ─────────────────────────────────────────────────────────────────────────────

/// Normalize an angle to the range [-π, π].
/// Uses modular arithmetic to avoid potential infinite loops with extreme values.
#[inline]
fn normalize_angle(angle: f32) -> f32 {
    if !angle.is_finite() {
        return 0.0;
    }
    let mut a = angle % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Normalize an arc to ensure start < end and handle wrapping.
/// Returns (start, end) where the arc goes from start to end counter-clockwise.
fn normalize_arc(angle1: f32, angle2: f32) -> (f32, f32) {
    let a1 = normalize_angle(angle1);
    let a2 = normalize_angle(angle2);

    // Find the shorter arc between the two angles
    let diff = normalize_angle(a2 - a1);

    if diff >= 0.0 {
        (a1, a1 + diff)
    } else {
        (a2, a2 - diff)
    }
}

/// Find the largest uncovered gap in a set of angular arcs.
///
/// # Arguments
/// * `arcs` - List of (start_angle, end_angle) arcs that are "covered"
///
/// # Returns
/// The largest uncovered gap as (start_angle, end_angle, size_in_radians),
/// or None if the entire circle is covered.
fn find_largest_gap(arcs: &[(f32, f32)]) -> Option<(f32, f32, f32)> {
    if arcs.is_empty() {
        // No coverage at all - entire circle is a gap
        return Some((-PI, PI, 2.0 * PI));
    }

    // Convert all arcs to a common representation and collect edge events
    // Each arc contributes a "start coverage" and "end coverage" event
    let mut events: Vec<(f32, i32)> = Vec::with_capacity(arcs.len() * 2);

    for &(start, end) in arcs {
        let s = normalize_angle(start);
        let e = normalize_angle(end);

        if s <= e {
            // Normal arc
            events.push((s, 1)); // Start coverage
            events.push((e, -1)); // End coverage
        } else {
            // Arc wraps around -π/π boundary
            events.push((s, 1));
            events.push((PI, -1));
            events.push((-PI, 1));
            events.push((e, -1));
        }
    }

    // Sort events by angle
    events.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Sweep through events to find uncovered regions
    let mut coverage = 0;
    let mut largest_gap: Option<(f32, f32, f32)> = None;
    let mut gap_start: Option<f32> = None;

    // Check initial state at -π
    for &(angle, delta) in &events {
        if angle > -PI + 0.001 {
            break;
        }
        coverage += delta;
    }

    if coverage == 0 {
        gap_start = Some(-PI);
    }

    // Process all events
    for &(angle, delta) in &events {
        let was_covered = coverage > 0;
        coverage += delta;
        let now_covered = coverage > 0;

        if was_covered && !now_covered {
            // Gap starts here
            gap_start = Some(angle);
        } else if !was_covered && now_covered {
            // Gap ends here
            if let Some(start) = gap_start {
                let size = normalize_angle(angle - start);
                let size = if size < 0.0 { size + 2.0 * PI } else { size };

                if largest_gap.is_none() || size > largest_gap.unwrap().2 {
                    largest_gap = Some((start, angle, size));
                }
                gap_start = None;
            }
        }
    }

    // Check if there's a gap that wraps around
    if let Some(start) = gap_start {
        // Gap extends to +π and possibly wraps to -π
        let end = if coverage == 0 {
            // Find first coverage after -π
            events
                .iter()
                .find(|&&(_, d)| d > 0)
                .map(|&(a, _)| a)
                .unwrap_or(start)
        } else {
            PI
        };

        let size = if end > start {
            end - start
        } else {
            (PI - start) + (end + PI)
        };

        if largest_gap.is_none() || size > largest_gap.unwrap().2 {
            largest_gap = Some((start, end, size));
        }
    }

    largest_gap
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
    fn test_optimize_nodes_merges_cliques() {
        // Create a large open room where many nodes will be mutually visible
        let walls = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)), // Bottom
            Line2D::new(Point2D::new(10.0, 0.0), Point2D::new(10.0, 10.0)), // Right
            Line2D::new(Point2D::new(10.0, 10.0), Point2D::new(0.0, 10.0)), // Top
            Line2D::new(Point2D::new(0.0, 10.0), Point2D::new(0.0, 0.0)), // Left
        ];

        // Use config that generates many nodes
        let config = CBVGConfig::default()
            .with_coverage_node_interval(0.5)
            .with_max_edge_length(2.0) // Increase edge length for larger room
            .with_min_clearance(0.3)
            .with_min_clearance_narrow(0.18)
            .with_min_clique_size_for_merge(3);

        let mut graph = ClearanceVisibilityGraph::new(config);

        // First build without optimization to count original nodes
        graph.nodes = generate_grid_nodes(&walls, None, &graph.config);
        graph.build_edges(&walls);
        let nodes_before = graph.node_count();

        // Now optimize
        graph.optimize_nodes(&walls);
        let nodes_after = graph.node_count();

        // Optimization should reduce node count in open room with clusters
        // or at minimum keep the same count (if no cliques >= 3)
        assert!(
            nodes_after <= nodes_before,
            "Optimization should not increase node count: before={}, after={}",
            nodes_before,
            nodes_after
        );

        // Graph should still be valid (has nodes and can find paths)
        assert!(!graph.is_empty());

        // Verify path finding still works after optimization
        let path = graph.find_path(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0), &walls);
        assert!(
            path.is_some(),
            "Path should still be findable after optimization"
        );
    }

    #[test]
    fn test_optimize_nodes_preserves_linear_chain() {
        // Create a narrow corridor where nodes form a linear chain (no cliques)
        let walls = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)), // Bottom
            Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(10.0, 1.0)), // Top
        ];

        let config = CBVGConfig::default()
            .with_voronoi_sample_step(0.2)
            .with_min_clearance(0.3)
            .with_min_clearance_narrow(0.18)
            .with_min_clique_size_for_merge(3);

        let mut graph = ClearanceVisibilityGraph::new(config);

        // Build and let optimization run
        graph.build(&walls, None);

        // In a narrow corridor, most nodes should be preserved as they form
        // a linear chain (each node only sees neighbors, not a full clique)
        // Graph should still be valid
        assert!(!graph.is_empty());
    }

    #[test]
    fn test_find_greedy_cliques() {
        // Create a simple visibility matrix for testing
        // 4 nodes: 0-1-2 form a clique, 3 is isolated
        let visible = vec![
            vec![false, true, true, false],   // Node 0 sees 1, 2
            vec![true, false, true, false],   // Node 1 sees 0, 2
            vec![true, true, false, false],   // Node 2 sees 0, 1
            vec![false, false, false, false], // Node 3 sees nobody
        ];

        let cliques = ClearanceVisibilityGraph::find_greedy_cliques(&visible);

        // Should have 2 cliques: one with {0,1,2} and one with {3}
        assert_eq!(cliques.len(), 2);

        // One clique should have 3 nodes
        let has_large_clique = cliques.iter().any(|c| c.len() == 3);
        assert!(has_large_clique, "Should find the 3-node clique");

        // One clique should have 1 node (isolated)
        let has_singleton = cliques.iter().any(|c| c.len() == 1);
        assert!(
            has_singleton,
            "Should have isolated node as singleton clique"
        );
    }

    #[test]
    fn test_compute_centroid() {
        let nodes = vec![
            CBVGNode::medial_axis(Point2D::new(0.0, 0.0), 1.0, false),
            CBVGNode::medial_axis(Point2D::new(2.0, 0.0), 1.0, false),
            CBVGNode::medial_axis(Point2D::new(1.0, 2.0), 1.0, false),
        ];

        let indices = vec![0, 1, 2];
        let centroid = ClearanceVisibilityGraph::compute_centroid(&indices, &nodes);

        // Centroid of triangle (0,0), (2,0), (1,2) is (1, 0.667)
        assert!((centroid.x - 1.0).abs() < 0.01);
        assert!((centroid.y - 0.666).abs() < 0.01);
    }

    #[test]
    fn test_add_nodes_from_scan_basic() {
        let walls = make_room();
        let config = CBVGConfig::default()
            .with_coverage_node_interval(0.5)
            .with_min_clearance(0.3)
            .with_min_clearance_narrow(0.18);

        let mut graph = ClearanceVisibilityGraph::new(config);

        // Simulate a scan from the center of the room
        let robot_pose = Pose2D::new(2.0, 1.5, 0.0);
        let robot_pos = robot_pose.position();
        let scan_points: Vec<Point2D> = (0..36)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::PI / 18.0;
                let dist = 1.5;
                Point2D::new(
                    robot_pos.x + dist * angle.cos(),
                    robot_pos.y + dist * angle.sin(),
                )
            })
            .collect();

        // Add nodes from the scan
        graph.add_nodes_from_scan(robot_pose, &scan_points, &walls);

        // Should have some nodes
        assert!(!graph.is_empty(), "Graph should have nodes after scan");

        // All nodes should have sufficient clearance
        for node in graph.nodes() {
            assert!(
                node.clearance >= 0.18,
                "Node clearance {} is below minimum",
                node.clearance
            );
        }
    }

    #[test]
    fn test_add_nodes_from_scan_incremental() {
        let walls = make_room();
        let config = CBVGConfig::default()
            .with_coverage_node_interval(0.5)
            .with_min_clearance(0.3)
            .with_min_clearance_narrow(0.18)
            .with_optimize_interval(100); // High value so optimization doesn't trigger

        let mut graph = ClearanceVisibilityGraph::new(config);

        // First scan from one corner
        let pose1 = Pose2D::new(1.0, 1.0, 0.0);
        let pos1 = pose1.position();
        let scan1: Vec<Point2D> = (0..36)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::PI / 18.0;
                Point2D::new(pos1.x + 1.0 * angle.cos(), pos1.y + 1.0 * angle.sin())
            })
            .collect();

        graph.add_nodes_from_scan(pose1, &scan1, &walls);
        let count1 = graph.node_count();

        // Second scan from another corner
        let pose2 = Pose2D::new(3.0, 2.0, 0.0);
        let pos2 = pose2.position();
        let scan2: Vec<Point2D> = (0..36)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::PI / 18.0;
                Point2D::new(pos2.x + 1.0 * angle.cos(), pos2.y + 1.0 * angle.sin())
            })
            .collect();

        graph.add_nodes_from_scan(pose2, &scan2, &walls);
        let count2 = graph.node_count();

        // Should have more nodes after second scan
        assert!(
            count2 >= count1,
            "Node count should not decrease: {} -> {}",
            count1,
            count2
        );
    }

    #[test]
    fn test_add_nodes_from_scan_no_duplicates() {
        let walls = make_room();
        let config = CBVGConfig::default()
            .with_coverage_node_interval(0.5)
            .with_min_clearance(0.3)
            .with_node_merge_distance(0.15)
            .with_optimize_interval(100);

        let mut graph = ClearanceVisibilityGraph::new(config);

        // Same scan position twice
        let robot_pose = Pose2D::new(2.0, 1.5, 0.0);
        let robot_pos = robot_pose.position();
        let scan_points: Vec<Point2D> = (0..36)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::PI / 18.0;
                Point2D::new(
                    robot_pos.x + 1.5 * angle.cos(),
                    robot_pos.y + 1.5 * angle.sin(),
                )
            })
            .collect();

        graph.add_nodes_from_scan(robot_pose, &scan_points, &walls);
        let count1 = graph.node_count();

        // Add same scan again
        graph.add_nodes_from_scan(robot_pose, &scan_points, &walls);
        let count2 = graph.node_count();

        // Should not add duplicate nodes
        assert_eq!(
            count1, count2,
            "Adding same scan twice should not add duplicates: {} vs {}",
            count1, count2
        );
    }

    #[test]
    fn test_add_nodes_from_scan_optimization_trigger() {
        let walls = make_room();
        let config = CBVGConfig::default()
            .with_coverage_node_interval(0.3)
            .with_min_clearance(0.3)
            .with_min_clearance_narrow(0.18)
            .with_optimize_interval(2) // Low value to trigger optimization
            .with_min_clique_size_for_merge(3);

        let mut graph = ClearanceVisibilityGraph::new(config);

        // Add multiple scans to trigger optimization
        for i in 0..5 {
            let x = 1.0 + i as f32 * 0.5;
            let pose = Pose2D::new(x, 1.5, 0.0);
            let pos = pose.position();
            let scan: Vec<Point2D> = (0..36)
                .map(|j| {
                    let angle = j as f32 * std::f32::consts::PI / 18.0;
                    Point2D::new(pos.x + 1.0 * angle.cos(), pos.y + 1.0 * angle.sin())
                })
                .collect();

            graph.add_nodes_from_scan(pose, &scan, &walls);
        }

        // Should have nodes and path finding should work
        assert!(!graph.is_empty());

        let path = graph.find_path(Point2D::new(1.0, 1.0), Point2D::new(3.0, 2.0), &walls);
        // Path may or may not exist depending on node placement, but it shouldn't crash
        let _ = path;
    }
}
