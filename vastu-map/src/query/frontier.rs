//! Frontier detection for exploration.
//!
//! Frontiers are boundaries between known and unknown space.
//! In a VectorMap, frontiers are typically unconnected line endpoints
//! that indicate areas that haven't been fully explored.
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
//! let frontiers = detector.detect(&lines, &config);
//!
//! // Subsequent calls reuse buffers (zero allocation)
//! let frontiers = detector.detect(&new_lines, &config);
//! ```

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use crate::Frontier;
use crate::core::Point2D;
use crate::features::{Line2D, LineCollection};

/// Spatial hash grid for endpoint proximity queries.
/// Maps (cell_x, cell_y) -> Vec<(point, line_index, is_start_endpoint)>
type EndpointGrid = HashMap<(i32, i32), Vec<(Point2D, usize, bool)>>;

/// Persistent frontier detector with reusable buffers.
///
/// Avoids per-call allocations by maintaining:
/// - Spatial hash grid with pre-allocated cell vectors
/// - Output frontier buffer
/// - Ranking distance buffer
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::query::frontier::{FrontierDetector, FrontierConfig};
///
/// let mut detector = FrontierDetector::new();
/// let config = FrontierConfig::default();
///
/// // Reuses internal buffers across calls
/// let frontiers = detector.detect(&lines, &config);
/// let ranked = detector.rank(robot_position, &lines);
/// ```
#[derive(Clone, Debug, Default)]
pub struct FrontierDetector {
    /// Reusable spatial hash grid.
    /// Cell vectors retain capacity across calls.
    endpoint_grid: EndpointGrid,

    /// Reusable output buffer for detected frontiers.
    frontiers: Vec<Frontier>,

    /// Reusable buffer for ranking frontiers by value.
    ranked_frontiers: Vec<(Frontier, f32)>,

    /// Reusable distance buffer for openness computation.
    distance_buffer: Vec<f32>,

    /// Cells that were used in the last detection (for efficient clearing).
    used_cells: Vec<(i32, i32)>,
}

impl FrontierDetector {
    /// Create a new frontier detector with default capacity.
    pub fn new() -> Self {
        Self::with_capacity(100, 50)
    }

    /// Create a frontier detector with capacity hints.
    ///
    /// # Arguments
    /// * `expected_lines` - Expected number of lines per detection
    /// * `expected_frontiers` - Expected number of frontiers per detection
    pub fn with_capacity(expected_lines: usize, expected_frontiers: usize) -> Self {
        Self {
            endpoint_grid: HashMap::with_capacity(expected_lines * 2),
            frontiers: Vec::with_capacity(expected_frontiers),
            ranked_frontiers: Vec::with_capacity(expected_frontiers),
            distance_buffer: Vec::with_capacity(expected_lines),
            used_cells: Vec::with_capacity(expected_lines * 2),
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

        // Track this cell for efficient clearing
        if let std::collections::hash_map::Entry::Vacant(e) = self.endpoint_grid.entry(cell) {
            e.insert(Vec::with_capacity(4));
            self.used_cells.push(cell);
        } else if self.endpoint_grid.get(&cell).is_none_or(|v| v.is_empty()) {
            // Cell exists but was cleared - track it again
            self.used_cells.push(cell);
        }

        self.endpoint_grid
            .get_mut(&cell)
            .unwrap()
            .push((point, line_idx, is_start));
    }

    /// Detect frontiers from map lines.
    ///
    /// Returns a slice of detected frontiers. The slice is valid until
    /// the next call to `detect` or `detect_from_robot`.
    ///
    /// # Arguments
    /// * `lines` - Map lines
    /// * `config` - Frontier detection configuration
    pub fn detect(&mut self, lines: &[Line2D], config: &FrontierConfig) -> &[Frontier] {
        self.frontiers.clear();

        if lines.is_empty() {
            return &self.frontiers;
        }

        let cell_size = config.connection_distance;

        // Clear and rebuild grid
        self.clear_grid();

        for (idx, line) in lines.iter().enumerate() {
            for (point, is_start) in [(line.start, true), (line.end, false)] {
                self.insert_endpoint(point, idx, is_start, cell_size);
            }
        }

        // Find unconnected endpoints
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
                    self.frontiers.push(Frontier {
                        point,
                        line_idx: idx,
                        is_start,
                    });
                }
            }
        }

        &self.frontiers
    }

    /// Detect frontiers filtered by distance from robot.
    ///
    /// Returns a slice of detected frontiers. The slice is valid until
    /// the next call to `detect` or `detect_from_robot`.
    pub fn detect_from_robot(
        &mut self,
        lines: &[Line2D],
        robot_position: Point2D,
        config: &FrontierConfig,
    ) -> &[Frontier] {
        // First detect all frontiers
        self.detect(lines, config);

        // Filter in place by distance
        self.frontiers
            .retain(|f| f.point.distance(robot_position) >= config.min_distance_from_robot);

        &self.frontiers
    }

    /// Rank the currently detected frontiers by exploration value.
    ///
    /// Must be called after `detect` or `detect_from_robot`.
    /// Returns a slice of (frontier, value) pairs sorted by value (best first).
    pub fn rank(&mut self, robot_position: Point2D, lines: &[Line2D]) -> &[(Frontier, f32)] {
        self.ranked_frontiers.clear();

        if lines.is_empty() {
            for f in &self.frontiers {
                self.ranked_frontiers.push((f.clone(), 0.0));
            }
            return &self.ranked_frontiers;
        }

        // Create LineCollection for SIMD-optimized distance computation
        let line_collection = LineCollection::from_lines(lines);

        for f in &self.frontiers {
            let distance = f.point.distance(robot_position);
            let openness =
                compute_openness_simd(f, lines, &line_collection, &mut self.distance_buffer);

            let distance_factor = 1.0 / (1.0 + distance * 0.1);
            let value = distance_factor * 0.3 + openness * 0.7;

            self.ranked_frontiers.push((f.clone(), value));
        }

        // Sort by value (descending)
        self.ranked_frontiers
            .sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        &self.ranked_frontiers
    }

    /// Get the best frontier for exploration.
    ///
    /// Convenience method that detects, filters, and ranks frontiers.
    pub fn get_best(
        &mut self,
        lines: &[Line2D],
        robot_position: Point2D,
        config: &FrontierConfig,
    ) -> Option<Frontier> {
        self.detect_from_robot(lines, robot_position, config);

        if self.frontiers.is_empty() {
            return None;
        }

        self.rank(robot_position, lines);
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

    /// Minimum "openness" score for a valid frontier.
    /// Higher values require more open space in the frontier direction.
    /// Default: 0.3
    pub min_openness: f32,
}

impl Default for FrontierConfig {
    fn default() -> Self {
        Self {
            connection_distance: 0.3,
            min_distance_from_robot: 0.5,
            min_openness: 0.3,
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
    // Check 3x3 neighborhood (covers all points within connection_distance)
    for dx in -1..=1 {
        for dy in -1..=1 {
            let neighbor_cell = (cell.0 + dx, cell.1 + dy);
            if let Some(neighbors) = grid.get(&neighbor_cell) {
                for &(other_point, other_idx, _) in neighbors {
                    // Don't compare with self (same line)
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

/// Detect frontiers from map lines.
///
/// Frontiers are line endpoints that don't connect to other lines,
/// indicating unexplored areas.
///
/// Uses spatial hashing for O(n) average complexity instead of O(n²).
///
/// # Arguments
/// * `lines` - Map lines
/// * `config` - Frontier detection configuration
///
/// # Returns
/// Vector of detected frontiers.
pub fn detect_frontiers(lines: &[Line2D], config: &FrontierConfig) -> Vec<Frontier> {
    if lines.is_empty() {
        return Vec::new();
    }

    let cell_size = config.connection_distance;

    // Build spatial hash grid - O(n)
    let mut grid: EndpointGrid = HashMap::with_capacity(lines.len() * 2);

    for (idx, line) in lines.iter().enumerate() {
        for (point, is_start) in [(line.start, true), (line.end, false)] {
            let cell = point_to_cell(point, cell_size);
            grid.entry(cell).or_default().push((point, idx, is_start));
        }
    }

    // Find unconnected endpoints - O(n) average case
    let mut frontiers = Vec::new();

    for (idx, line) in lines.iter().enumerate() {
        for (point, is_start) in [(line.start, true), (line.end, false)] {
            let cell = point_to_cell(point, cell_size);
            if !is_connected_in_grid(&grid, cell, point, idx, config.connection_distance) {
                frontiers.push(Frontier {
                    point,
                    line_idx: idx,
                    is_start,
                });
            }
        }
    }

    frontiers
}

/// Detect frontiers filtered by distance from robot.
///
/// Only returns frontiers that are at least `min_distance` from the robot.
pub fn detect_frontiers_from_robot(
    lines: &[Line2D],
    robot_position: Point2D,
    config: &FrontierConfig,
) -> Vec<Frontier> {
    detect_frontiers(lines, config)
        .into_iter()
        .filter(|f| f.point.distance(robot_position) >= config.min_distance_from_robot)
        .collect()
}

/// Rank frontiers by exploration value.
///
/// Returns frontiers sorted by their exploration value (higher is better).
/// Value considers:
/// - Distance from robot (closer is slightly better)
/// - "Openness" (frontiers facing into unknown space are better)
///
/// Uses SIMD-optimized LineCollection for efficient batch distance computation.
///
/// # Arguments
/// * `frontiers` - Detected frontiers
/// * `lines` - Map lines (for openness calculation)
/// * `robot_position` - Current robot position
///
/// # Returns
/// Frontiers sorted by value (best first).
pub fn rank_frontiers(
    frontiers: &[Frontier],
    lines: &[Line2D],
    robot_position: Point2D,
) -> Vec<(Frontier, f32)> {
    if lines.is_empty() {
        return frontiers.iter().map(|f| (f.clone(), 0.0)).collect();
    }

    // Create LineCollection once for SIMD-optimized distance computation
    let line_collection = LineCollection::from_lines(lines);

    // Reusable buffer for distance computation (avoids allocation per frontier)
    let mut distance_buffer = Vec::with_capacity(lines.len());

    let mut ranked: Vec<(Frontier, f32)> = frontiers
        .iter()
        .map(|f| {
            let distance = f.point.distance(robot_position);
            let openness = compute_openness_simd(f, lines, &line_collection, &mut distance_buffer);

            // Value function: prefer nearby, open frontiers
            // Distance factor: 1.0 at 0m, decaying to ~0.1 at 10m
            let distance_factor = 1.0 / (1.0 + distance * 0.1);

            // Openness is already 0-1
            let value = distance_factor * 0.3 + openness * 0.7;

            (f.clone(), value)
        })
        .collect();

    // Sort by value (descending)
    ranked.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    ranked
}

/// Compute "openness" score for a frontier using SIMD-optimized distance computation.
///
/// # Geometric Concept: Openness Score
///
/// Openness measures how much unknown/unexplored space exists in the directions
/// radiating from a frontier point. A frontier at a dead-end hallway has low openness,
/// while a frontier at an open room entrance has high openness.
///
/// ## Algorithm
///
/// 1. **Determine outward direction**: For a frontier at the start of a line, the
///    outward direction points opposite to the line direction. At the end of a line,
///    it points along the line direction.
///
/// 2. **Cast probe points**: Three probe points are placed 2 meters from the frontier:
///    - One in the outward direction (away from the line)
///    - Two perpendicular to the line (left and right)
///
/// ```text
///                    probe_left
///                        •
///                        |
///     line endpoint ─────●─────• probe_outward
///     (frontier)         |
///                        •
///                   probe_right
/// ```
///
/// 3. **Check distances**: For each probe, compute the minimum distance to any map line.
///    If the distance exceeds 1 meter, that direction is considered "open" (unexplored).
///
/// 4. **Score**: openness = (number of open probes) / 3.0
///    - 0.0: All directions blocked by walls (dead end)
///    - 0.33: One direction open
///    - 0.67: Two directions open
///    - 1.0: All directions open (wide open space)
///
/// ## Why This Matters
///
/// Frontiers with high openness are better exploration targets because:
/// - They likely lead to larger unexplored areas
/// - The robot has more maneuvering room
/// - They're less likely to be false frontiers from noise
fn compute_openness_simd(
    frontier: &Frontier,
    lines: &[Line2D],
    line_collection: &LineCollection,
    distance_buffer: &mut Vec<f32>,
) -> f32 {
    if frontier.line_idx >= lines.len() {
        return 0.0;
    }

    let line = &lines[frontier.line_idx];

    // Get direction pointing away from the line
    let outward_dir = if frontier.is_start {
        // At start, outward is opposite to line direction
        -line.unit_direction()
    } else {
        // At end, outward is along line direction
        line.unit_direction()
    };

    // Also consider the perpendicular direction
    let perp = Point2D::new(-outward_dir.y, outward_dir.x);

    // Check distances in outward and perpendicular directions
    let check_distance = 2.0; // meters
    let probe_outward = frontier.point + outward_dir * check_distance;
    let probe_left = frontier.point + perp * check_distance;
    let probe_right = frontier.point - perp * check_distance;

    // Count how many probe points are far from any line
    // Use SIMD-optimized batch distance computation with buffer reuse
    let mut open_count = 0;

    for probe in [probe_outward, probe_left, probe_right] {
        if let Some((_, min_dist)) =
            line_collection.nearest_line_with_buffer(probe, distance_buffer)
        {
            if min_dist > 1.0 {
                open_count += 1;
            }
        } else {
            // No lines - consider open
            open_count += 1;
        }
    }

    open_count as f32 / 3.0
}

/// Get the best frontier for exploration.
///
/// Convenience function that detects, filters, and ranks frontiers,
/// returning the best one.
pub fn get_best_frontier(
    lines: &[Line2D],
    robot_position: Point2D,
    config: &FrontierConfig,
) -> Option<Frontier> {
    let frontiers = detect_frontiers_from_robot(lines, robot_position, config);

    if frontiers.is_empty() {
        return None;
    }

    let ranked = rank_frontiers(&frontiers, lines, robot_position);

    ranked.into_iter().next().map(|(f, _)| f)
}

/// Cluster nearby frontiers into frontier regions.
///
/// Groups frontiers that are close together, useful for exploration
/// planning where you want to visit a region rather than individual points.
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

        // Find all frontiers close to this one
        for j in (i + 1)..frontiers.len() {
            if assigned[j] {
                continue;
            }

            // Check if close to any frontier in the cluster
            let close = cluster
                .iter()
                .any(|f| f.point.distance(frontiers[j].point) < cluster_distance);

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

    let sum_x: f32 = cluster.iter().map(|f| f.point.x).sum();
    let sum_y: f32 = cluster.iter().map(|f| f.point.y).sum();
    let n = cluster.len() as f32;

    Some(Point2D::new(sum_x / n, sum_y / n))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Compute "openness" score for a frontier.
    fn compute_openness(frontier: &Frontier, lines: &[Line2D]) -> f32 {
        if frontier.line_idx >= lines.len() {
            return 0.0;
        }

        let line = &lines[frontier.line_idx];

        // Get direction pointing away from the line
        let outward_dir = if frontier.is_start {
            -line.unit_direction()
        } else {
            line.unit_direction()
        };

        // Also consider the perpendicular direction
        let perp = Point2D::new(-outward_dir.y, outward_dir.x);

        // Check distances in outward and perpendicular directions
        let check_distance = 2.0;
        let probe_outward = frontier.point + outward_dir * check_distance;
        let probe_left = frontier.point + perp * check_distance;
        let probe_right = frontier.point - perp * check_distance;

        // Count how many probe points are far from any line
        let mut open_count = 0;

        for probe in [probe_outward, probe_left, probe_right] {
            let min_dist = lines
                .iter()
                .map(|l| l.distance_to_point(probe))
                .fold(f32::MAX, f32::min);

            if min_dist > 1.0 {
                open_count += 1;
            }
        }

        open_count as f32 / 3.0
    }

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

        let frontiers = detect_frontiers(&lines, &config);

        // Should have 2 frontiers (the open endpoints of the top)
        // Right wall top (5, 5) and Left wall top (-5, 5)
        assert!(frontiers.len() >= 2);
    }

    #[test]
    fn test_detect_frontiers_complete_room() {
        let lines = make_complete_room();
        let config = FrontierConfig::default();

        let frontiers = detect_frontiers(&lines, &config);

        // Complete room - all corners should be connected
        // Depending on connection_distance, might have 0 or some frontiers
        // With default 0.3m, corners at distance 0 should connect
        assert_eq!(frontiers.len(), 0);
    }

    #[test]
    fn test_detect_frontiers_single_line() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let config = FrontierConfig::default();

        let frontiers = detect_frontiers(&lines, &config);

        // Both endpoints are unconnected
        assert_eq!(frontiers.len(), 2);
    }

    #[test]
    fn test_detect_frontiers_from_robot() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let robot = Point2D::new(0.0, 0.0);
        let config = FrontierConfig::default().with_min_distance_from_robot(1.0);

        let frontiers = detect_frontiers_from_robot(&lines, robot, &config);

        // Start endpoint is at robot position, should be filtered
        // End endpoint is 5m away, should be included
        assert_eq!(frontiers.len(), 1);
        assert_eq!(frontiers[0].is_start, false); // End endpoint
    }

    #[test]
    fn test_rank_frontiers() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 10.0), Point2D::new(5.0, 10.0)),
        ];
        let robot = Point2D::new(0.0, 0.0);
        let config = FrontierConfig::default();

        let frontiers = detect_frontiers(&lines, &config);
        let ranked = rank_frontiers(&frontiers, &lines, robot);

        // Frontiers closer to robot should rank higher
        assert!(!ranked.is_empty());
    }

    #[test]
    fn test_get_best_frontier() {
        let lines = make_partial_room();
        let robot = Point2D::zero();
        let config = FrontierConfig::default();

        let best = get_best_frontier(&lines, robot, &config);

        assert!(best.is_some());
    }

    #[test]
    fn test_get_best_frontier_no_frontiers() {
        let lines = make_complete_room();
        let robot = Point2D::zero();
        let config = FrontierConfig::default();

        let best = get_best_frontier(&lines, robot, &config);

        // Complete room has no frontiers
        assert!(best.is_none());
    }

    #[test]
    fn test_cluster_frontiers() {
        let frontiers = vec![
            Frontier {
                point: Point2D::new(0.0, 0.0),
                line_idx: 0,
                is_start: true,
            },
            Frontier {
                point: Point2D::new(0.1, 0.0),
                line_idx: 1,
                is_start: true,
            }, // Close to first
            Frontier {
                point: Point2D::new(10.0, 10.0),
                line_idx: 2,
                is_start: true,
            }, // Far
        ];

        let clusters = cluster_frontiers(&frontiers, 0.5);

        // Should have 2 clusters: one with 2 close frontiers, one with the far one
        assert_eq!(clusters.len(), 2);
    }

    #[test]
    fn test_cluster_centroid() {
        let frontiers = vec![
            Frontier {
                point: Point2D::new(0.0, 0.0),
                line_idx: 0,
                is_start: true,
            },
            Frontier {
                point: Point2D::new(2.0, 0.0),
                line_idx: 1,
                is_start: true,
            },
        ];

        let centroid = cluster_centroid(&frontiers);

        assert!(centroid.is_some());
        let c = centroid.unwrap();
        assert!((c.x - 1.0).abs() < 0.01);
        assert!((c.y - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_compute_openness() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];

        let frontier = Frontier {
            point: Point2D::new(5.0, 0.0),
            line_idx: 0,
            is_start: false,
        };

        let openness = compute_openness(&frontier, &lines);

        // End of single line should have high openness
        assert!(openness > 0.0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // FrontierDetector tests
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_detector_new() {
        let detector = FrontierDetector::new();
        assert!(detector.frontiers().is_empty());
        assert!(detector.ranked_frontiers().is_empty());
    }

    #[test]
    fn test_detector_detect_single_line() {
        let mut detector = FrontierDetector::new();
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let config = FrontierConfig::default();

        let frontiers = detector.detect(&lines, &config);

        // Both endpoints are unconnected
        assert_eq!(frontiers.len(), 2);
    }

    #[test]
    fn test_detector_detect_complete_room() {
        let mut detector = FrontierDetector::new();
        let lines = make_complete_room();
        let config = FrontierConfig::default();

        let frontiers = detector.detect(&lines, &config);

        // Complete room - all corners connected
        assert_eq!(frontiers.len(), 0);
    }

    #[test]
    fn test_detector_detect_from_robot() {
        let mut detector = FrontierDetector::new();
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let robot = Point2D::new(0.0, 0.0);
        let config = FrontierConfig::default().with_min_distance_from_robot(1.0);

        let frontiers = detector.detect_from_robot(&lines, robot, &config);

        // Start endpoint at robot filtered, end endpoint included
        assert_eq!(frontiers.len(), 1);
        assert!(!frontiers[0].is_start);
    }

    #[test]
    fn test_detector_rank() {
        let mut detector = FrontierDetector::new();
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 10.0), Point2D::new(5.0, 10.0)),
        ];
        let robot = Point2D::new(0.0, 0.0);
        let config = FrontierConfig::default();

        detector.detect(&lines, &config);
        let ranked = detector.rank(robot, &lines);

        assert!(!ranked.is_empty());
        // Values should be sorted descending
        for i in 1..ranked.len() {
            assert!(ranked[i - 1].1 >= ranked[i].1);
        }
    }

    #[test]
    fn test_detector_get_best() {
        let mut detector = FrontierDetector::new();
        let lines = make_partial_room();
        let robot = Point2D::zero();
        let config = FrontierConfig::default();

        let best = detector.get_best(&lines, robot, &config);

        assert!(best.is_some());
    }

    #[test]
    fn test_detector_reuse_buffers() {
        let mut detector = FrontierDetector::new();
        let config = FrontierConfig::default();

        // First detection
        let lines1 = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let frontiers1 = detector.detect(&lines1, &config);
        assert_eq!(frontiers1.len(), 2);

        // Second detection with different lines - should reuse buffers
        let lines2 = make_complete_room();
        let frontiers2 = detector.detect(&lines2, &config);
        assert_eq!(frontiers2.len(), 0);

        // Third detection - back to single line
        let frontiers3 = detector.detect(&lines1, &config);
        assert_eq!(frontiers3.len(), 2);
    }

    #[test]
    fn test_detector_equivalence_to_standalone() {
        let mut detector = FrontierDetector::new();
        let lines = make_partial_room();
        let config = FrontierConfig::default();

        // Compare detector results to standalone function results
        let detector_frontiers = detector.detect(&lines, &config);
        let standalone_frontiers = detect_frontiers(&lines, &config);

        assert_eq!(detector_frontiers.len(), standalone_frontiers.len());

        // Check same frontiers detected (may be in different order)
        for f in standalone_frontiers.iter() {
            assert!(
                detector_frontiers
                    .iter()
                    .any(|df| { df.line_idx == f.line_idx && df.is_start == f.is_start })
            );
        }
    }
}
