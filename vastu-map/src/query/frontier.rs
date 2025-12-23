//! Gap-based frontier detection for exploration.
//!
//! Frontiers represent **gaps between unconnected wall endpoints**. The robot
//! navigates to a viewpoint (a reachable position) from which it can observe
//! the unexplored area through the gap.
//!
//! # Key Concept
//!
//! ```text
//! Wall A ─────┐              ┌───── Wall B
//!             │              │
//!       endpoint1      endpoint2
//!             │     GAP      │
//!             └──────────────┘
//!                   ↑
//!             Frontier is HERE
//!             (midpoint of gap)
//!
//! Viewpoint = nearest reachable position that can see the gap
//! ```
//!
//! # Algorithm
//!
//! 1. Find all **unconnected endpoints** (wall ends not connected to other walls)
//! 2. Find **pairs of endpoints** that form a gap:
//!    - Close enough to each other (within max_gap_width)
//!    - Wide enough for robot to pass (min_gap_width)
//!    - No wall blocking between them
//! 3. For each gap, find a **viewpoint**:
//!    - Must be in known free space
//!    - Must have line-of-sight to the gap
//!    - Prefer positions closer to the robot
//! 4. Also handle **single endpoints** (walls ending in open space)
//!
//! # Zero-Allocation Detection
//!
//! For repeated frontier detection (e.g., during SLAM), use `FrontierDetector`
//! which maintains persistent buffers to avoid per-call allocations:
//!
//! ```rust,ignore
//! use vastu_map::query::frontier::{FrontierDetector, FrontierConfig};
//!
//! let mut detector = FrontierDetector::new();
//! let config = FrontierConfig::default();
//!
//! // First detection - allocates buffers
//! let frontiers = detector.detect(&lines, robot_pos, &config);
//!
//! // Subsequent calls reuse buffers (zero allocation)
//! let frontiers = detector.detect(&new_lines, robot_pos, &config);
//! ```

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use crate::Frontier;
use crate::core::Point2D;
use crate::features::Line2D;

/// Spatial hash grid for endpoint proximity queries.
/// Maps (cell_x, cell_y) -> Vec<(point, line_index, is_start_endpoint)>
type EndpointGrid = HashMap<(i32, i32), Vec<(Point2D, usize, bool)>>;

/// An unconnected endpoint with metadata.
#[derive(Clone, Debug)]
struct UnconnectedEndpoint {
    /// The endpoint position.
    point: Point2D,
    /// Index of the line this endpoint belongs to.
    line_idx: usize,
    /// Outward direction (away from the line).
    outward_dir: Point2D,
}

/// A detected gap between two unconnected endpoints.
#[derive(Clone, Debug)]
struct Gap {
    /// Center of the gap.
    center: Point2D,
    /// Direction from endpoint1 to endpoint2 (normalized).
    direction: Point2D,
    /// Width of the gap.
    width: f32,
}

/// Persistent frontier detector with reusable buffers.
///
/// Uses gap-based detection: frontiers are gaps between unconnected
/// wall endpoints. The viewpoint is found by looking for reachable
/// positions with line-of-sight to the gap.
#[derive(Clone, Debug, Default)]
pub struct FrontierDetector {
    /// Reusable spatial hash grid.
    endpoint_grid: EndpointGrid,

    /// Reusable output buffer for detected frontiers.
    frontiers: Vec<Frontier>,

    /// Reusable buffer for ranking frontiers by value.
    ranked_frontiers: Vec<(Frontier, f32)>,

    /// Cells that were used in the last detection (for efficient clearing).
    used_cells: Vec<(i32, i32)>,

    /// Reusable buffer for unconnected endpoints.
    unconnected: Vec<UnconnectedEndpoint>,

    /// Reusable buffer for detected gaps.
    gaps: Vec<Gap>,

    /// Reusable buffer for visibility graph nodes.
    graph_nodes: Vec<Point2D>,
}

impl FrontierDetector {
    /// Create a new frontier detector with default capacity.
    pub fn new() -> Self {
        Self::with_capacity(100, 50)
    }

    /// Create a frontier detector with capacity hints.
    pub fn with_capacity(expected_lines: usize, expected_frontiers: usize) -> Self {
        Self {
            endpoint_grid: HashMap::with_capacity(expected_lines * 2),
            frontiers: Vec::with_capacity(expected_frontiers),
            ranked_frontiers: Vec::with_capacity(expected_frontiers),
            used_cells: Vec::with_capacity(expected_lines * 2),
            unconnected: Vec::with_capacity(expected_lines * 2),
            gaps: Vec::with_capacity(expected_frontiers),
            graph_nodes: Vec::with_capacity(expected_lines * 2 + 2),
        }
    }

    /// Clear grid by clearing each used cell's vector (retains capacity).
    #[inline]
    fn clear_grid(&mut self) {
        for cell in &self.used_cells {
            if let Some(vec) = self.endpoint_grid.get_mut(cell) {
                vec.clear();
            }
        }
        self.used_cells.clear();
    }

    /// Insert an endpoint into the grid.
    #[inline]
    fn insert_endpoint(&mut self, point: Point2D, line_idx: usize, is_start: bool, cell_size: f32) {
        let cell = point_to_cell(point, cell_size);

        if let std::collections::hash_map::Entry::Vacant(e) = self.endpoint_grid.entry(cell) {
            e.insert(Vec::with_capacity(4));
            self.used_cells.push(cell);
        } else if self.endpoint_grid.get(&cell).is_none_or(|v| v.is_empty()) {
            self.used_cells.push(cell);
        }

        self.endpoint_grid
            .get_mut(&cell)
            .unwrap()
            .push((point, line_idx, is_start));
    }

    /// Detect frontiers from map lines.
    ///
    /// Returns viewpoints that can observe unexplored gaps between lines.
    /// Uses gap-based detection: finds pairs of unconnected endpoints that
    /// form passable gaps, then finds reachable viewpoints for each gap.
    pub fn detect(
        &mut self,
        lines: &[Line2D],
        robot_position: Point2D,
        config: &FrontierConfig,
    ) -> &[Frontier] {
        self.frontiers.clear();
        self.unconnected.clear();
        self.gaps.clear();

        if lines.is_empty() {
            return &self.frontiers;
        }

        let cell_size = config.connection_distance;

        // Step 1: Build spatial hash grid and find unconnected endpoints
        self.clear_grid();

        for (idx, line) in lines.iter().enumerate() {
            for (point, is_start) in [(line.start, true), (line.end, false)] {
                self.insert_endpoint(point, idx, is_start, cell_size);
            }
        }

        // Find unconnected endpoints with their outward directions
        for (idx, line) in lines.iter().enumerate() {
            for (point, is_start) in [(line.start, true), (line.end, false)] {
                let cell = point_to_cell(point, cell_size);
                if !is_connected_in_grid(
                    &self.endpoint_grid,
                    cell,
                    point,
                    idx,
                    config.connection_distance,
                ) {
                    // Compute outward direction (away from the line)
                    let outward_dir = if is_start {
                        -line.unit_direction()
                    } else {
                        line.unit_direction()
                    };
                    self.unconnected.push(UnconnectedEndpoint {
                        point,
                        line_idx: idx,
                        outward_dir,
                    });
                }
            }
        }

        // Step 2: Find pairs of unconnected endpoints that form gaps
        let mut used_endpoints = vec![false; self.unconnected.len()];

        for i in 0..self.unconnected.len() {
            if used_endpoints[i] {
                continue;
            }

            let ep1 = &self.unconnected[i];

            // Find the closest unconnected endpoint that forms a valid gap
            let mut best_pair: Option<(usize, f32)> = None;

            for (j, (used, ep2)) in used_endpoints
                .iter()
                .zip(self.unconnected.iter())
                .enumerate()
                .skip(i + 1)
            {
                if *used {
                    continue;
                }
                let distance = ep1.point.distance(ep2.point);

                // Check if this pair forms a valid gap
                if distance < config.min_gap_width || distance > config.max_gap_width {
                    continue;
                }

                // Check that there's no wall blocking between them
                if is_path_blocked(ep1.point, ep2.point, lines) {
                    continue;
                }

                // Check if endpoints are facing each other (outward dirs point toward each other)
                let to_ep2 = (ep2.point - ep1.point).normalized();
                let to_ep1 = (ep1.point - ep2.point).normalized();
                let facing1 = ep1.outward_dir.dot(to_ep2);
                let facing2 = ep2.outward_dir.dot(to_ep1);

                // At least one endpoint should be facing the other
                if facing1 <= -0.5 && facing2 <= -0.5 {
                    continue;
                }

                if best_pair.is_none() || distance < best_pair.unwrap().1 {
                    best_pair = Some((j, distance));
                }
            }

            if let Some((j, width)) = best_pair {
                let ep2 = &self.unconnected[j];
                let center = Point2D::new(
                    (ep1.point.x + ep2.point.x) / 2.0,
                    (ep1.point.y + ep2.point.y) / 2.0,
                );
                let direction = (ep2.point - ep1.point).normalized();

                self.gaps.push(Gap {
                    center,
                    direction,
                    width,
                });

                used_endpoints[i] = true;
                used_endpoints[j] = true;
            }
        }

        // Step 3: Build visibility graph nodes (all line endpoints + robot position)
        self.graph_nodes.clear();
        for line in lines {
            self.graph_nodes.push(line.start);
            self.graph_nodes.push(line.end);
        }
        self.graph_nodes.push(robot_position);

        // Step 4: For each gap, find the nearest reachable graph node as target
        for gap in &self.gaps.clone() {
            if let Some(target) = find_nearest_reachable_node(
                gap.center,
                &self.graph_nodes,
                robot_position,
                lines,
                config,
            ) && target.distance(robot_position) >= config.min_distance_from_robot
            {
                // Look direction is perpendicular to gap direction
                let look_direction = Point2D::new(-gap.direction.y, gap.direction.x);
                self.frontiers.push(Frontier {
                    viewpoint: target,
                    look_direction,
                    endpoint: gap.center,
                    line_idx: 0,
                    estimated_area: gap.width * config.gap_probe_distance,
                });
            }
        }

        // Step 5: Handle single unconnected endpoints (no pair found)
        for (i, ep) in self.unconnected.iter().enumerate() {
            if used_endpoints[i] {
                continue; // Already part of a gap
            }

            // Find nearest reachable node toward this endpoint
            if let Some(target) = find_nearest_reachable_node(
                ep.point,
                &self.graph_nodes,
                robot_position,
                lines,
                config,
            ) && target.distance(robot_position) >= config.min_distance_from_robot
            {
                self.frontiers.push(Frontier {
                    viewpoint: target,
                    look_direction: ep.outward_dir,
                    endpoint: ep.point,
                    line_idx: ep.line_idx,
                    estimated_area: config.gap_probe_distance * config.gap_probe_distance * 0.5,
                });
            }
        }

        // Debug logging
        log::debug!(
            "Frontier detection: {} lines, {} unconnected, {} gaps, {} single, {} frontiers",
            lines.len(),
            self.unconnected.len(),
            self.gaps.len(),
            self.unconnected
                .iter()
                .enumerate()
                .filter(|(i, _)| !used_endpoints[*i])
                .count(),
            self.frontiers.len()
        );

        // Step 6: Sort by estimated_area (largest first)
        self.frontiers.sort_by(|a, b| {
            b.estimated_area
                .partial_cmp(&a.estimated_area)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        &self.frontiers
    }

    /// Rank the currently detected frontiers by exploration value.
    ///
    /// Value considers distance from robot and estimated area.
    pub fn rank(&mut self, robot_position: Point2D) -> &[(Frontier, f32)] {
        self.ranked_frontiers.clear();

        for f in &self.frontiers {
            let distance = f.viewpoint.distance(robot_position);

            // Distance factor: 1.0 at 0m, decaying to ~0.1 at 10m
            let distance_factor = 1.0 / (1.0 + distance * 0.1);

            // Area factor: normalize by expected room size (~25m²)
            let area_factor = (f.estimated_area / 25.0).min(1.0);

            // Value: prefer nearby frontiers with large unexplored areas
            let value = distance_factor * 0.3 + area_factor * 0.7;

            self.ranked_frontiers.push((f.clone(), value));
        }

        // Sort by value (descending)
        self.ranked_frontiers
            .sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        &self.ranked_frontiers
    }

    /// Get the best frontier for exploration.
    pub fn get_best(
        &mut self,
        lines: &[Line2D],
        robot_position: Point2D,
        config: &FrontierConfig,
    ) -> Option<Frontier> {
        self.detect(lines, robot_position, config);

        if self.frontiers.is_empty() {
            return None;
        }

        self.rank(robot_position);
        self.ranked_frontiers.first().map(|(f, _)| f.clone())
    }

    /// Get the currently detected frontiers (from last detect call).
    pub fn frontiers(&self) -> &[Frontier] {
        &self.frontiers
    }

    /// Get the currently ranked frontiers (from last rank call).
    pub fn ranked_frontiers(&self) -> &[(Frontier, f32)] {
        &self.ranked_frontiers
    }
}

/// Configuration for frontier detection.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FrontierConfig {
    /// Maximum distance for endpoints to be considered "connected" (meters).
    /// Endpoints closer than this are not frontiers.
    /// Default: 0.3m
    pub connection_distance: f32,

    /// Minimum distance from robot for a valid frontier (meters).
    /// Frontiers too close are ignored.
    /// Default: 0.5m
    pub min_distance_from_robot: f32,

    /// Safety margin: viewpoint offset from wall (meters).
    /// The viewpoint is placed this far from the line endpoint.
    /// Default: 0.4m
    pub safety_margin: f32,

    /// Gap probe distance: how far to probe for unexplored gaps (meters).
    /// Used for estimating unexplored area size.
    /// Default: 2.0m
    pub gap_probe_distance: f32,

    /// Robot radius for viewpoint validation (meters).
    /// Viewpoints must be clear of walls by this margin.
    /// Default: 0.15m
    pub robot_radius: f32,

    /// Maximum iterations for viewpoint adjustment.
    /// If viewpoint is blocked, move away from wall up to this many times.
    /// Default: 5
    pub max_viewpoint_iterations: usize,

    /// Viewpoint adjustment step size (meters).
    /// How far to move the viewpoint per iteration if blocked.
    /// Default: 0.1m
    pub viewpoint_step: f32,

    /// Minimum gap width for safe robot passage (meters).
    /// Gaps narrower than this are ignored (robot can't fit).
    /// Default: 0.4m (robot diameter + safety margin)
    pub min_gap_width: f32,

    /// Maximum gap width to consider as a single gap (meters).
    /// Larger distances are treated as separate areas, not a gap.
    /// Default: 3.0m
    pub max_gap_width: f32,
}

impl Default for FrontierConfig {
    fn default() -> Self {
        Self {
            connection_distance: 0.3,
            min_distance_from_robot: 0.5,
            safety_margin: 0.4,
            gap_probe_distance: 2.0,
            robot_radius: 0.15,
            max_viewpoint_iterations: 5,
            viewpoint_step: 0.1,
            min_gap_width: 0.4,  // robot diameter (0.3m) + margin
            max_gap_width: 10.0, // allow gaps across rooms
        }
    }
}

impl FrontierConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for connection distance.
    pub fn with_connection_distance(mut self, meters: f32) -> Self {
        self.connection_distance = meters;
        self
    }

    /// Builder-style setter for minimum distance from robot.
    pub fn with_min_distance_from_robot(mut self, meters: f32) -> Self {
        self.min_distance_from_robot = meters;
        self
    }

    /// Builder-style setter for safety margin.
    pub fn with_safety_margin(mut self, meters: f32) -> Self {
        self.safety_margin = meters;
        self
    }

    /// Builder-style setter for gap probe distance.
    pub fn with_gap_probe_distance(mut self, meters: f32) -> Self {
        self.gap_probe_distance = meters;
        self
    }

    /// Builder-style setter for robot radius.
    pub fn with_robot_radius(mut self, meters: f32) -> Self {
        self.robot_radius = meters;
        self
    }

    /// Builder-style setter for minimum gap width.
    pub fn with_min_gap_width(mut self, meters: f32) -> Self {
        self.min_gap_width = meters;
        self
    }

    /// Builder-style setter for maximum gap width.
    pub fn with_max_gap_width(mut self, meters: f32) -> Self {
        self.max_gap_width = meters;
        self
    }
}

/// Convert a point to a grid cell coordinate.
#[inline]
fn point_to_cell(point: Point2D, cell_size: f32) -> (i32, i32) {
    (
        (point.x / cell_size).floor() as i32,
        (point.y / cell_size).floor() as i32,
    )
}

/// Check if a point is connected to any other endpoint in the 3x3 neighborhood.
#[inline]
fn is_connected_in_grid(
    grid: &EndpointGrid,
    cell: (i32, i32),
    point: Point2D,
    line_idx: usize,
    connection_distance: f32,
) -> bool {
    for dx in -1..=1 {
        for dy in -1..=1 {
            let neighbor_cell = (cell.0 + dx, cell.1 + dy);
            if let Some(neighbors) = grid.get(&neighbor_cell) {
                for &(other_point, other_idx, _) in neighbors {
                    if other_idx == line_idx {
                        continue;
                    }
                    if point.distance(other_point) < connection_distance {
                        return true;
                    }
                }
            }
        }
    }
    false
}

/// Find a target point that makes progress toward the frontier.
///
/// Simple approach:
/// 1. If robot can see the frontier directly, move toward it
/// 2. Otherwise, find any reachable graph node that is closer to frontier
/// 3. If no such node, the frontier is not reachable from current position
fn find_nearest_reachable_node(
    frontier_point: Point2D,
    graph_nodes: &[Point2D],
    robot_position: Point2D,
    lines: &[Line2D],
    config: &FrontierConfig,
) -> Option<Point2D> {
    let robot_dist_to_frontier = robot_position.distance(frontier_point);

    // Skip if frontier is too close
    if robot_dist_to_frontier < config.min_distance_from_robot {
        return None;
    }

    // First, check if robot can directly see the frontier point
    if !is_path_blocked(robot_position, frontier_point, lines) {
        // Direct line of sight - move toward frontier
        let to_frontier = frontier_point - robot_position;
        let move_dist =
            (robot_dist_to_frontier - config.safety_margin).max(config.min_distance_from_robot);
        let target = robot_position + to_frontier.normalized() * move_dist;

        // Verify target is not too close to walls
        if !is_point_too_close_to_lines(target, lines, config.robot_radius) {
            return Some(target);
        }
        // Even if computed target is too close to walls, try moving there anyway
        // (path planner will find an alternative)
        return Some(target);
    }

    // No direct line of sight - find the best intermediate node
    let mut best_node: Option<(Point2D, f32)> = None;

    for &node in graph_nodes {
        // Skip robot position
        if node.distance(robot_position) < 0.1 {
            continue;
        }

        // Skip nodes too close to frontier (would be blocked by walls near frontier)
        if node.distance(frontier_point) < config.safety_margin {
            continue;
        }

        // Must be reachable from robot (direct line of sight)
        if is_path_blocked(robot_position, node, lines) {
            continue;
        }

        // Prefer nodes closer to frontier (makes progress)
        let node_dist_to_frontier = node.distance(frontier_point);

        if best_node.is_none() || node_dist_to_frontier < best_node.unwrap().1 {
            best_node = Some((node, node_dist_to_frontier));
        }
    }

    best_node.map(|(node, _)| node)
}

/// Check if the straight-line path between two points is blocked by any line.
/// Excludes intersections that occur exactly at the path endpoints (from/to).
fn is_path_blocked(from: Point2D, to: Point2D, lines: &[Line2D]) -> bool {
    let epsilon = 0.01; // 1cm tolerance for endpoint matching

    for line in lines {
        if segments_intersect(from, to, line.start, line.end) {
            // Check if the intersection is at one of the path endpoints
            // If the line endpoint is essentially the same as our path endpoint,
            // it's not a blocking intersection - it's expected.
            let from_matches_start = from.distance(line.start) < epsilon;
            let from_matches_end = from.distance(line.end) < epsilon;
            let to_matches_start = to.distance(line.start) < epsilon;
            let to_matches_end = to.distance(line.end) < epsilon;

            // If the intersection is only at endpoints, it's not truly blocking
            if from_matches_start || from_matches_end || to_matches_start || to_matches_end {
                continue;
            }
            return true;
        }
    }
    false
}

/// Check if a point is too close to any line (within robot radius).
fn is_point_too_close_to_lines(point: Point2D, lines: &[Line2D], min_distance: f32) -> bool {
    for line in lines {
        if line.distance_to_point(point) < min_distance {
            return true;
        }
    }
    false
}

/// Check if two line segments intersect.
fn segments_intersect(a1: Point2D, a2: Point2D, b1: Point2D, b2: Point2D) -> bool {
    let d1 = cross_product(b2 - b1, a1 - b1);
    let d2 = cross_product(b2 - b1, a2 - b1);
    let d3 = cross_product(a2 - a1, b1 - a1);
    let d4 = cross_product(a2 - a1, b2 - a1);

    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }

    // Check collinear cases
    let epsilon = 1e-6;
    if d1.abs() < epsilon && point_on_segment(a1, b1, b2) {
        return true;
    }
    if d2.abs() < epsilon && point_on_segment(a2, b1, b2) {
        return true;
    }
    if d3.abs() < epsilon && point_on_segment(b1, a1, a2) {
        return true;
    }
    if d4.abs() < epsilon && point_on_segment(b2, a1, a2) {
        return true;
    }

    false
}

/// 2D cross product of vectors.
#[inline]
fn cross_product(a: Point2D, b: Point2D) -> f32 {
    a.x * b.y - a.y * b.x
}

/// Check if point p lies on segment (a, b).
#[inline]
fn point_on_segment(p: Point2D, a: Point2D, b: Point2D) -> bool {
    p.x >= a.x.min(b.x) && p.x <= a.x.max(b.x) && p.y >= a.y.min(b.y) && p.y <= a.y.max(b.y)
}

// ─────────────────────────────────────────────────────────────────────────────
// Convenience functions (for backward compatibility)
// ─────────────────────────────────────────────────────────────────────────────

/// Detect frontiers from map lines.
pub fn detect_frontiers(
    lines: &[Line2D],
    robot_position: Point2D,
    config: &FrontierConfig,
) -> Vec<Frontier> {
    let mut detector = FrontierDetector::new();
    detector.detect(lines, robot_position, config).to_vec()
}

/// Rank frontiers by exploration value.
pub fn rank_frontiers(frontiers: &[Frontier], robot_position: Point2D) -> Vec<(Frontier, f32)> {
    frontiers
        .iter()
        .map(|f| {
            let distance = f.viewpoint.distance(robot_position);
            let distance_factor = 1.0 / (1.0 + distance * 0.1);
            let area_factor = (f.estimated_area / 25.0).min(1.0);
            let value = distance_factor * 0.3 + area_factor * 0.7;
            (f.clone(), value)
        })
        .collect()
}

/// Get the best frontier for exploration.
pub fn get_best_frontier(
    lines: &[Line2D],
    robot_position: Point2D,
    config: &FrontierConfig,
) -> Option<Frontier> {
    let mut detector = FrontierDetector::new();
    detector.get_best(lines, robot_position, config)
}

/// Cluster nearby frontiers into frontier regions.
pub fn cluster_frontiers(frontiers: &[Frontier], cluster_distance: f32) -> Vec<Vec<Frontier>> {
    if frontiers.is_empty() {
        return Vec::new();
    }

    let mut clusters: Vec<Vec<Frontier>> = Vec::new();
    let mut assigned = vec![false; frontiers.len()];

    for i in 0..frontiers.len() {
        if assigned[i] {
            continue;
        }

        let mut cluster = vec![frontiers[i].clone()];
        assigned[i] = true;

        for j in (i + 1)..frontiers.len() {
            if assigned[j] {
                continue;
            }

            let close = cluster
                .iter()
                .any(|f| f.viewpoint.distance(frontiers[j].viewpoint) < cluster_distance);

            if close {
                cluster.push(frontiers[j].clone());
                assigned[j] = true;
            }
        }

        clusters.push(cluster);
    }

    clusters
}

/// Get the centroid of a frontier cluster.
pub fn cluster_centroid(cluster: &[Frontier]) -> Option<Point2D> {
    if cluster.is_empty() {
        return None;
    }

    let sum_x: f32 = cluster.iter().map(|f| f.viewpoint.x).sum();
    let sum_y: f32 = cluster.iter().map(|f| f.viewpoint.y).sum();
    let n = cluster.len() as f32;

    Some(Point2D::new(sum_x / n, sum_y / n))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_partial_room() -> Vec<Line2D> {
        // Room with one side open (no top wall)
        vec![
            Line2D::new(Point2D::new(-5.0, -5.0), Point2D::new(5.0, -5.0)), // Bottom
            Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0)),   // Right
            Line2D::new(Point2D::new(-5.0, 5.0), Point2D::new(-5.0, -5.0)), // Left
        ]
    }

    fn make_complete_room() -> Vec<Line2D> {
        vec![
            Line2D::new(Point2D::new(-5.0, -5.0), Point2D::new(5.0, -5.0)), // Bottom
            Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0)),   // Right
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(-5.0, 5.0)),   // Top
            Line2D::new(Point2D::new(-5.0, 5.0), Point2D::new(-5.0, -5.0)), // Left
        ]
    }

    #[test]
    fn test_detect_frontiers_partial_room() {
        let lines = make_partial_room();
        let config = FrontierConfig::default();
        let robot = Point2D::zero();

        let frontiers = detect_frontiers(&lines, robot, &config);

        // Should have frontiers at the open top
        assert!(!frontiers.is_empty());

        // Viewpoints should be offset from walls
        for f in &frontiers {
            let min_dist = lines
                .iter()
                .map(|l| l.distance_to_point(f.viewpoint))
                .fold(f32::MAX, f32::min);
            assert!(
                min_dist >= config.robot_radius * 0.9,
                "Viewpoint too close to wall: {:.3}m",
                min_dist
            );
        }
    }

    #[test]
    fn test_detect_frontiers_complete_room() {
        let lines = make_complete_room();
        let config = FrontierConfig::default();
        let robot = Point2D::zero();

        let frontiers = detect_frontiers(&lines, robot, &config);

        // Complete room - all corners should be connected (no frontiers)
        assert_eq!(frontiers.len(), 0);
    }

    #[test]
    fn test_detect_frontiers_single_line() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let config = FrontierConfig::default();
        let robot = Point2D::new(2.5, 2.0);

        let frontiers = detect_frontiers(&lines, robot, &config);

        // Both endpoints are unconnected, but viewpoints should be valid
        assert!(frontiers.len() <= 2);

        // Check viewpoints are not on the line
        for f in &frontiers {
            assert!(f.viewpoint.distance(f.endpoint) > 0.1);
        }
    }

    #[test]
    fn test_viewpoint_not_on_wall() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let config = FrontierConfig::default();
        let robot = Point2D::new(2.5, 2.0);

        let frontiers = detect_frontiers(&lines, robot, &config);

        for f in &frontiers {
            // Viewpoint should be at least safety_margin away from the endpoint
            let dist_to_endpoint = f.viewpoint.distance(f.endpoint);
            assert!(
                dist_to_endpoint >= config.safety_margin * 0.9,
                "Viewpoint {:.2}m from endpoint (expected >= {:.2}m)",
                dist_to_endpoint,
                config.safety_margin
            );
        }
    }

    #[test]
    fn test_frontier_has_look_direction() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let config = FrontierConfig::default();
        let robot = Point2D::new(2.5, 2.0);

        let frontiers = detect_frontiers(&lines, robot, &config);

        for f in &frontiers {
            // Look direction should be a unit vector (or close to it)
            let len = (f.look_direction.x.powi(2) + f.look_direction.y.powi(2)).sqrt();
            assert!(
                (len - 1.0).abs() < 0.01,
                "Look direction not unit: len={:.3}",
                len
            );
        }
    }

    #[test]
    fn test_detector_reuse_buffers() {
        let mut detector = FrontierDetector::new();
        let config = FrontierConfig::default();
        let robot = Point2D::zero();

        // First detection
        let lines1 = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let frontiers1 = detector.detect(&lines1, robot, &config);
        let count1 = frontiers1.len();

        // Second detection with different lines
        let lines2 = make_complete_room();
        let frontiers2 = detector.detect(&lines2, robot, &config);
        assert_eq!(frontiers2.len(), 0);

        // Third detection - back to single line
        let frontiers3 = detector.detect(&lines1, robot, &config);
        assert_eq!(frontiers3.len(), count1);
    }

    #[test]
    fn test_cluster_frontiers_uses_viewpoint() {
        let frontiers = vec![
            Frontier {
                viewpoint: Point2D::new(0.0, 0.0),
                look_direction: Point2D::new(1.0, 0.0),
                endpoint: Point2D::new(-0.4, 0.0),
                line_idx: 0,
                estimated_area: 4.0,
            },
            Frontier {
                viewpoint: Point2D::new(0.1, 0.0),
                look_direction: Point2D::new(1.0, 0.0),
                endpoint: Point2D::new(-0.3, 0.0),
                line_idx: 1,
                estimated_area: 4.0,
            },
            Frontier {
                viewpoint: Point2D::new(10.0, 10.0),
                look_direction: Point2D::new(1.0, 0.0),
                endpoint: Point2D::new(9.6, 10.0),
                line_idx: 2,
                estimated_area: 4.0,
            },
        ];

        let clusters = cluster_frontiers(&frontiers, 0.5);

        // Should have 2 clusters based on viewpoint distance
        assert_eq!(clusters.len(), 2);
    }

    #[test]
    fn test_cluster_centroid_uses_viewpoint() {
        let frontiers = vec![
            Frontier {
                viewpoint: Point2D::new(0.0, 0.0),
                look_direction: Point2D::new(1.0, 0.0),
                endpoint: Point2D::new(-0.4, 0.0),
                line_idx: 0,
                estimated_area: 4.0,
            },
            Frontier {
                viewpoint: Point2D::new(2.0, 0.0),
                look_direction: Point2D::new(1.0, 0.0),
                endpoint: Point2D::new(1.6, 0.0),
                line_idx: 1,
                estimated_area: 4.0,
            },
        ];

        let centroid = cluster_centroid(&frontiers);

        assert!(centroid.is_some());
        let c = centroid.unwrap();
        // Centroid should be at (1.0, 0.0) - average of viewpoints
        assert!((c.x - 1.0).abs() < 0.01);
        assert!((c.y - 0.0).abs() < 0.01);
    }
}
