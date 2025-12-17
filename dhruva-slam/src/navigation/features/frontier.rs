//! Frontier detection for autonomous exploration.
//!
//! A frontier is a boundary between explored (free) and unexplored (unknown) space.
//! This module finds frontier cells, clusters them, and ranks candidates for navigation.
//!
//! Note: Some types are defined for planned exploration features.

use crate::core::types::Point2D;
use crate::state::CurrentMapData;
use std::collections::VecDeque;

/// Minimum frontier cluster size to consider (cells).
const DEFAULT_MIN_FRONTIER_SIZE: usize = 3;

/// Distance to consider frontiers as same cluster (meters).
const DEFAULT_CLUSTER_DISTANCE: f32 = 0.30;

/// Cell states for frontier detection.
const CELL_FREE: u8 = 0;
const CELL_OCCUPIED: u8 = 100;
const CELL_UNKNOWN: u8 = 255;

/// A frontier candidate representing a cluster of frontier cells.
#[derive(Debug, Clone)]
pub struct FrontierCandidate {
    /// Center of frontier cluster (world coordinates).
    pub centroid: Point2D,
    /// Number of frontier cells in cluster.
    pub size: usize,
    /// Distance from robot to centroid.
    pub distance: f32,
    /// Individual frontier cell positions (for visualization).
    pub cells: Vec<Point2D>,
}

impl FrontierCandidate {
    /// Create a new frontier candidate from cells.
    pub fn from_cells(cells: Vec<Point2D>, robot_pos: &Point2D) -> Self {
        let size = cells.len();

        // Calculate centroid
        let (sum_x, sum_y) = cells
            .iter()
            .fold((0.0, 0.0), |(sx, sy), p| (sx + p.x, sy + p.y));
        let centroid = Point2D::new(sum_x / size as f32, sum_y / size as f32);

        let distance = centroid.distance(robot_pos);

        Self {
            centroid,
            size,
            distance,
            cells,
        }
    }
}

/// Find all frontier candidates in the map.
///
/// A frontier cell is a free cell adjacent to at least one unknown cell.
/// Adjacent frontier cells are clustered together.
///
/// # Arguments
/// - `map`: Current occupancy grid map
/// - `robot_pos`: Current robot position for distance calculation
/// - `min_frontier_size`: Minimum cells to form a valid frontier cluster
/// - `cluster_distance`: Maximum distance between cells to be in same cluster
///
/// # Returns
/// Vector of frontier candidates, sorted by distance (closest first).
pub fn find_frontiers(
    map: &CurrentMapData,
    robot_pos: &Point2D,
    min_frontier_size: usize,
    _cluster_distance: f32,
) -> Vec<FrontierCandidate> {
    if map.cells.is_empty() || map.width == 0 || map.height == 0 {
        return Vec::new();
    }

    let width = map.width as usize;
    let height = map.height as usize;

    // Find all frontier cells
    let mut frontier_cells = Vec::new();
    let mut visited = vec![false; width * height];

    for y in 1..(height - 1) {
        for x in 1..(width - 1) {
            let idx = y * width + x;

            // Skip if not free
            if map.cells[idx] != CELL_FREE {
                continue;
            }

            // Check if adjacent to unknown (8-connected)
            let is_frontier = is_adjacent_to_unknown(map, x, y, width, height);

            if is_frontier {
                let world_x = map.origin_x + (x as f32 + 0.5) * map.resolution;
                let world_y = map.origin_y + (y as f32 + 0.5) * map.resolution;
                frontier_cells.push((x, y, Point2D::new(world_x, world_y)));
            }
        }
    }

    // Cluster frontier cells using flood fill
    let mut candidates = Vec::new();

    for (fx, fy, _world_pos) in &frontier_cells {
        let idx = fy * width + fx;
        if visited[idx] {
            continue;
        }

        // Flood fill to find connected frontier cells
        let cluster = flood_fill_frontier(map, *fx, *fy, width, height, &mut visited);

        if cluster.len() >= min_frontier_size {
            candidates.push(FrontierCandidate::from_cells(cluster, robot_pos));
        }
    }

    // Sort by distance (closest first)
    candidates.sort_by(|a, b| {
        a.distance
            .partial_cmp(&b.distance)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    candidates
}

/// Check if a cell is adjacent to unknown space (8-connected).
fn is_adjacent_to_unknown(
    map: &CurrentMapData,
    x: usize,
    y: usize,
    width: usize,
    height: usize,
) -> bool {
    let neighbors = [
        (x.wrapping_sub(1), y.wrapping_sub(1)),
        (x, y.wrapping_sub(1)),
        (x + 1, y.wrapping_sub(1)),
        (x.wrapping_sub(1), y),
        (x + 1, y),
        (x.wrapping_sub(1), y + 1),
        (x, y + 1),
        (x + 1, y + 1),
    ];

    for (nx, ny) in neighbors {
        if nx < width && ny < height {
            let idx = ny * width + nx;
            if map.cells[idx] == CELL_UNKNOWN {
                return true;
            }
        }
    }

    false
}

/// Flood fill to find connected frontier cells.
fn flood_fill_frontier(
    map: &CurrentMapData,
    start_x: usize,
    start_y: usize,
    width: usize,
    height: usize,
    visited: &mut [bool],
) -> Vec<Point2D> {
    let mut cluster = Vec::new();
    let mut queue = VecDeque::new();

    queue.push_back((start_x, start_y));
    visited[start_y * width + start_x] = true;

    while let Some((x, y)) = queue.pop_front() {
        // Check if this is a frontier cell
        if map.cells[y * width + x] == CELL_FREE && is_adjacent_to_unknown(map, x, y, width, height)
        {
            let world_x = map.origin_x + (x as f32 + 0.5) * map.resolution;
            let world_y = map.origin_y + (y as f32 + 0.5) * map.resolution;
            cluster.push(Point2D::new(world_x, world_y));

            // Check 4-connected neighbors for more frontier cells
            let neighbors = [
                (x.wrapping_sub(1), y),
                (x + 1, y),
                (x, y.wrapping_sub(1)),
                (x, y + 1),
            ];

            for (nx, ny) in neighbors {
                if nx < width && ny < height {
                    let idx = ny * width + nx;
                    if !visited[idx] && map.cells[idx] == CELL_FREE {
                        visited[idx] = true;
                        queue.push_back((nx, ny));
                    }
                }
            }
        }
    }

    cluster
}

/// Find the best frontier to navigate to.
///
/// Filters out blocked frontiers and selects the closest valid one.
pub fn select_best_frontier<'a>(
    candidates: &'a [FrontierCandidate],
    blocked_positions: &[Point2D],
    min_frontier_distance: f32,
    min_frontier_size: usize,
    block_radius: f32,
) -> Option<&'a FrontierCandidate> {
    candidates
        .iter()
        .filter(|f| f.distance > min_frontier_distance)
        .filter(|f| f.size >= min_frontier_size)
        .find(|f| !is_blocked(f, blocked_positions, block_radius)) // Already sorted by distance, so first is closest
}

/// Check if frontier is in blocked list.
fn is_blocked(frontier: &FrontierCandidate, blocked: &[Point2D], radius: f32) -> bool {
    blocked
        .iter()
        .any(|blocked_pos| frontier.centroid.distance(blocked_pos) < radius)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_map(width: u32, height: u32, cells: Vec<u8>) -> CurrentMapData {
        CurrentMapData {
            map_id: "test".to_string(),
            name: "Test Map".to_string(),
            resolution: 0.05,
            width,
            height,
            origin_x: 0.0,
            origin_y: 0.0,
            cells,
            explored_area_m2: 0.0,
        }
    }

    #[test]
    fn test_find_frontiers_empty_map() {
        let map = create_test_map(10, 10, vec![CELL_UNKNOWN; 100]);
        let robot_pos = Point2D::new(0.25, 0.25);
        let frontiers = find_frontiers(
            &map,
            &robot_pos,
            DEFAULT_MIN_FRONTIER_SIZE,
            DEFAULT_CLUSTER_DISTANCE,
        );
        assert!(frontiers.is_empty());
    }

    #[test]
    fn test_find_frontiers_all_explored() {
        let map = create_test_map(10, 10, vec![CELL_FREE; 100]);
        let robot_pos = Point2D::new(0.25, 0.25);
        let frontiers = find_frontiers(
            &map,
            &robot_pos,
            DEFAULT_MIN_FRONTIER_SIZE,
            DEFAULT_CLUSTER_DISTANCE,
        );
        assert!(frontiers.is_empty());
    }

    #[test]
    fn test_find_frontiers_with_boundary() {
        // Create map with free center and unknown edges
        let mut cells = vec![CELL_UNKNOWN; 100];
        // Make center 6x6 free
        for y in 2..8 {
            for x in 2..8 {
                cells[y * 10 + x] = CELL_FREE;
            }
        }

        let map = create_test_map(10, 10, cells);
        let robot_pos = Point2D::new(0.25, 0.25);
        let frontiers = find_frontiers(&map, &robot_pos, 1, DEFAULT_CLUSTER_DISTANCE);

        // Should find frontier cells at boundary of free/unknown
        assert!(!frontiers.is_empty());
    }

    #[test]
    fn test_frontier_candidate_distance() {
        let cells = vec![
            Point2D::new(1.0, 0.0),
            Point2D::new(1.1, 0.0),
            Point2D::new(1.2, 0.0),
        ];
        let robot_pos = Point2D::new(0.0, 0.0);
        let candidate = FrontierCandidate::from_cells(cells, &robot_pos);

        assert_eq!(candidate.size, 3);
        assert!((candidate.centroid.x - 1.1).abs() < 0.01);
        assert!((candidate.distance - 1.1).abs() < 0.1);
    }

    #[test]
    fn test_select_best_frontier_filters_blocked() {
        let candidates = vec![
            FrontierCandidate {
                centroid: Point2D::new(1.0, 0.0),
                size: 5,
                distance: 1.0,
                cells: vec![],
            },
            FrontierCandidate {
                centroid: Point2D::new(2.0, 0.0),
                size: 5,
                distance: 2.0,
                cells: vec![],
            },
        ];

        let blocked = vec![Point2D::new(1.0, 0.0)];
        let best = select_best_frontier(&candidates, &blocked, 0.1, 3, 0.3);

        assert!(best.is_some());
        assert!((best.unwrap().distance - 2.0).abs() < 0.01);
    }
}
