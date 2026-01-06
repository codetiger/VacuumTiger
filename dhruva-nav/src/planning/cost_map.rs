//! Cost map with obstacle inflation and wall distance penalties.
//!
//! Provides efficient path planning by pre-computing traversability costs
//! and distance-to-obstacle fields using SIMD-optimized algorithms.

use std::collections::{HashSet, VecDeque};
use std::simd::{cmp::SimdPartialOrd, f32x4};
use vastu_slam::{CellType, GridCoord, GridStorage, WorldPoint};

/// Number of SIMD lanes for f32 operations
const LANES: usize = 4;

/// Cost thresholds for path planning
pub mod costs {
    /// Safe to traverse, no penalty
    pub const FREE: u8 = 0;
    /// Close to wall, penalized but traversable
    pub const NEAR_OBSTACLE: u8 = 50;
    /// Within robot radius, should avoid
    pub const INSCRIBED: u8 = 254;
    /// Obstacle or unknown, blocked
    pub const LETHAL: u8 = 255;
}

/// Cost map for path planning with obstacle inflation and wall distance penalty.
///
/// The cost map pre-computes:
/// 1. Traversability costs (lethal, inscribed, inflated, free)
/// 2. Distance field (distance to nearest obstacle in cells)
#[derive(Clone, Debug)]
pub struct CostMap {
    /// Grid dimensions
    width: usize,
    height: usize,
    /// Resolution in meters per cell
    resolution: f32,
    /// Grid origin in world coordinates
    origin: WorldPoint,
    /// Cost values: 0 = free, 255 = lethal obstacle
    costs: Vec<u8>,
    /// Distance to nearest obstacle (in cells) for each cell
    distance_field: Vec<f32>,
    /// Wall penalty distance in cells
    wall_penalty_cells: usize,
}

impl CostMap {
    /// Build cost map from occupancy grid.
    ///
    /// # Arguments
    /// * `grid` - Source occupancy grid
    /// * `robot_radius` - Robot radius in meters
    /// * `safety_margin` - Extra clearance beyond robot radius in meters
    /// * `wall_penalty_dist` - Distance within which to penalize wall proximity in meters
    pub fn from_grid(
        grid: &GridStorage,
        robot_radius: f32,
        safety_margin: f32,
        wall_penalty_dist: f32,
    ) -> Self {
        let width = grid.width();
        let height = grid.height();
        let resolution = grid.resolution();
        let (origin, _) = grid.bounds();

        // Convert distances to cells
        let robot_radius_cells = (robot_radius / resolution).ceil() as usize;
        let safety_margin_cells = (safety_margin / resolution).ceil() as usize;
        let wall_penalty_cells = (wall_penalty_dist / resolution).ceil() as usize;

        let total_cells = width * height;
        let mut costs = vec![costs::FREE; total_cells];
        let mut distance_field = vec![f32::MAX; total_cells];

        // Initialize: mark obstacles and build initial distance field
        let mut obstacle_queue = VecDeque::with_capacity(total_cells / 10);

        for y in 0..height as i32 {
            for x in 0..width as i32 {
                let coord = GridCoord::new(x, y);
                let cell_type = grid.get_type(coord);
                let idx = (y as usize) * width + (x as usize);

                if cell_type.is_obstacle() || cell_type == CellType::Unknown {
                    costs[idx] = costs::LETHAL;
                    distance_field[idx] = 0.0;
                    obstacle_queue.push_back((x, y));
                }
            }
        }

        // Compute distance field using Brushfire algorithm
        Self::compute_distance_field_brushfire(
            &mut distance_field,
            width,
            height,
            &mut obstacle_queue,
        );

        // Apply cost layers based on distance field
        Self::apply_cost_layers(
            &mut costs,
            &distance_field,
            width,
            height,
            robot_radius_cells,
            safety_margin_cells,
            wall_penalty_cells,
        );

        Self {
            width,
            height,
            resolution,
            origin,
            costs,
            distance_field,
            wall_penalty_cells,
        }
    }

    /// Compute distance field using Brushfire (BFS) algorithm.
    fn compute_distance_field_brushfire(
        distance_field: &mut [f32],
        width: usize,
        height: usize,
        queue: &mut VecDeque<(i32, i32)>,
    ) {
        let sqrt2 = 1.414f32;

        // 8-connected neighbors with distances
        let neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, sqrt2),
            (1, -1, sqrt2),
            (-1, 1, sqrt2),
            (1, 1, sqrt2),
        ];

        while let Some((x, y)) = queue.pop_front() {
            let current_idx = (y as usize) * width + (x as usize);
            let current_dist = distance_field[current_idx];

            for &(dx, dy, cost) in &neighbors {
                let nx = x + dx;
                let ny = y + dy;

                // Bounds check
                if nx < 0 || ny < 0 || nx >= width as i32 || ny >= height as i32 {
                    continue;
                }

                let neighbor_idx = (ny as usize) * width + (nx as usize);
                let new_dist = current_dist + cost;

                if new_dist < distance_field[neighbor_idx] {
                    distance_field[neighbor_idx] = new_dist;
                    queue.push_back((nx, ny));
                }
            }
        }
    }

    /// Apply cost layers based on distance field.
    fn apply_cost_layers(
        costs: &mut [u8],
        distance_field: &[f32],
        width: usize,
        height: usize,
        robot_radius_cells: usize,
        safety_margin_cells: usize,
        wall_penalty_cells: usize,
    ) {
        let inscribed_dist = robot_radius_cells as f32;
        let inflation_dist = (robot_radius_cells + safety_margin_cells) as f32;
        let penalty_dist = wall_penalty_cells as f32;

        // SIMD-optimized cost computation
        let total_cells = width * height;
        let inscribed_vec = f32x4::splat(inscribed_dist);
        let inflation_vec = f32x4::splat(inflation_dist);
        let penalty_vec = f32x4::splat(penalty_dist);

        // Process 4 cells at a time
        let aligned_len = (total_cells / LANES) * LANES;

        for i in (0..aligned_len).step_by(LANES) {
            // Skip if already lethal
            if costs[i] == costs::LETHAL
                || costs[i + 1] == costs::LETHAL
                || costs[i + 2] == costs::LETHAL
                || costs[i + 3] == costs::LETHAL
            {
                // Process individually for mixed lethal/non-lethal
                for j in 0..LANES {
                    if costs[i + j] != costs::LETHAL {
                        costs[i + j] = Self::compute_cell_cost(
                            distance_field[i + j],
                            inscribed_dist,
                            inflation_dist,
                            penalty_dist,
                        );
                    }
                }
                continue;
            }

            let dist = f32x4::from_slice(&distance_field[i..]);

            // Check inscribed radius
            let is_inscribed = dist.simd_le(inscribed_vec);

            // Check inflation radius
            let is_inflated = dist.simd_le(inflation_vec);

            // Check wall penalty distance
            let is_near_wall = dist.simd_le(penalty_vec);

            // Convert masks to costs
            for j in 0..LANES {
                if is_inscribed.test(j) {
                    costs[i + j] = costs::INSCRIBED;
                } else if is_inflated.test(j) {
                    // Exponential decay in inflation zone
                    let d = distance_field[i + j];
                    let ratio = (d - inscribed_dist) / (inflation_dist - inscribed_dist);
                    let cost = costs::INSCRIBED
                        - ((costs::INSCRIBED - costs::NEAR_OBSTACLE) as f32 * ratio) as u8;
                    costs[i + j] = cost.max(costs::NEAR_OBSTACLE);
                } else if is_near_wall.test(j) {
                    // Linear penalty in wall proximity zone
                    let d = distance_field[i + j];
                    let ratio = 1.0 - (d - inflation_dist) / (penalty_dist - inflation_dist);
                    let cost = (costs::NEAR_OBSTACLE as f32 * ratio.max(0.0)) as u8;
                    costs[i + j] = cost.max(1);
                }
                // Otherwise stays FREE (0)
            }
        }

        // Handle remaining cells
        for i in aligned_len..total_cells {
            if costs[i] != costs::LETHAL {
                costs[i] = Self::compute_cell_cost(
                    distance_field[i],
                    inscribed_dist,
                    inflation_dist,
                    penalty_dist,
                );
            }
        }
    }

    /// Compute cost for a single cell based on distance to obstacle.
    #[inline]
    fn compute_cell_cost(
        distance: f32,
        inscribed_dist: f32,
        inflation_dist: f32,
        penalty_dist: f32,
    ) -> u8 {
        if distance <= inscribed_dist {
            costs::INSCRIBED
        } else if distance <= inflation_dist {
            let ratio = (distance - inscribed_dist) / (inflation_dist - inscribed_dist);
            let cost =
                costs::INSCRIBED - ((costs::INSCRIBED - costs::NEAR_OBSTACLE) as f32 * ratio) as u8;
            cost.max(costs::NEAR_OBSTACLE)
        } else if distance <= penalty_dist {
            let ratio = 1.0 - (distance - inflation_dist) / (penalty_dist - inflation_dist);
            let cost = (costs::NEAR_OBSTACLE as f32 * ratio.max(0.0)) as u8;
            cost.max(1)
        } else {
            costs::FREE
        }
    }

    /// Get cost at grid coordinate.
    #[inline]
    pub fn cost(&self, coord: GridCoord) -> u8 {
        if coord.x < 0 || coord.y < 0 {
            return costs::LETHAL;
        }
        let x = coord.x as usize;
        let y = coord.y as usize;
        if x >= self.width || y >= self.height {
            return costs::LETHAL;
        }
        self.costs[y * self.width + x]
    }

    /// Get distance to nearest obstacle at grid coordinate (in cells).
    #[inline]
    pub fn obstacle_distance(&self, coord: GridCoord) -> f32 {
        if coord.x < 0 || coord.y < 0 {
            return 0.0;
        }
        let x = coord.x as usize;
        let y = coord.y as usize;
        if x >= self.width || y >= self.height {
            return 0.0;
        }
        self.distance_field[y * self.width + x]
    }

    /// Check if a cell is traversable (cost < INSCRIBED).
    #[inline]
    pub fn is_traversable(&self, coord: GridCoord) -> bool {
        self.cost(coord) < costs::INSCRIBED
    }

    /// Find the closest traversable cell to a target using BFS.
    ///
    /// Returns `None` if no traversable cell is found within `max_radius` cells.
    ///
    /// # Arguments
    /// * `target` - The target coordinate to find a reachable point near
    /// * `max_radius` - Maximum search radius in cells
    pub fn find_closest_reachable_to(
        &self,
        target: GridCoord,
        max_radius: i32,
    ) -> Option<GridCoord> {
        // Check if target itself is traversable
        if self.is_traversable(target) {
            return Some(target);
        }

        // BFS to find closest traversable cell
        let mut queue = VecDeque::new();
        let mut visited = HashSet::new();

        queue.push_back(target);
        visited.insert((target.x, target.y));

        // 8-connected neighbors for search
        let neighbors = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (1, -1),
            (-1, 1),
            (1, 1),
        ];

        while let Some(current) = queue.pop_front() {
            // Check distance from target (use Chebyshev distance for rectangular search)
            let dx = (current.x - target.x).abs();
            let dy = (current.y - target.y).abs();
            if dx > max_radius || dy > max_radius {
                continue; // Outside search radius
            }

            // Check if this cell is traversable
            if self.is_traversable(current) {
                return Some(current);
            }

            // Add unvisited neighbors to queue
            for (ndx, ndy) in &neighbors {
                let nx = current.x + ndx;
                let ny = current.y + ndy;

                // Bounds check
                if nx < 0 || ny < 0 || nx >= self.width as i32 || ny >= self.height as i32 {
                    continue;
                }

                if !visited.contains(&(nx, ny)) {
                    visited.insert((nx, ny));
                    queue.push_back(GridCoord::new(nx, ny));
                }
            }
        }

        None
    }

    /// Find the closest traversable cell and return as world coordinates.
    ///
    /// # Arguments
    /// * `target` - Target point in world coordinates
    /// * `max_radius_meters` - Maximum search radius in meters
    pub fn find_closest_reachable_world(
        &self,
        target: WorldPoint,
        max_radius_meters: f32,
    ) -> Option<WorldPoint> {
        let target_coord = self.world_to_grid(target);
        let max_radius_cells = (max_radius_meters / self.resolution).ceil() as i32;

        self.find_closest_reachable_to(target_coord, max_radius_cells)
            .map(|coord| self.grid_to_world(coord))
    }

    /// Check if line-of-sight is clear between two grid coordinates.
    ///
    /// Uses Bresenham's line algorithm to check all cells along the line.
    pub fn line_of_sight(&self, from: GridCoord, to: GridCoord) -> bool {
        let mut x0 = from.x;
        let mut y0 = from.y;
        let x1 = to.x;
        let y1 = to.y;

        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx - dy;

        loop {
            // Check current cell
            if self.cost(GridCoord::new(x0, y0)) >= costs::INSCRIBED {
                return false;
            }

            if x0 == x1 && y0 == y1 {
                break;
            }

            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x0 += sx;
            }
            if e2 < dx {
                err += dx;
                y0 += sy;
            }
        }

        true
    }

    /// Compute wall penalty along a line segment.
    ///
    /// Returns the integrated penalty based on distance to walls.
    pub fn integrate_wall_penalty(&self, from: GridCoord, to: GridCoord) -> f32 {
        let mut penalty = 0.0;
        let mut count = 0;

        let mut x0 = from.x;
        let mut y0 = from.y;
        let x1 = to.x;
        let y1 = to.y;

        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx - dy;

        let penalty_dist = self.wall_penalty_cells as f32;

        loop {
            let coord = GridCoord::new(x0, y0);
            let dist = self.obstacle_distance(coord);

            if dist < penalty_dist {
                penalty += (penalty_dist - dist) / penalty_dist;
            }
            count += 1;

            if x0 == x1 && y0 == y1 {
                break;
            }

            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x0 += sx;
            }
            if e2 < dx {
                err += dx;
                y0 += sy;
            }
        }

        // Normalize by path length
        if count > 0 {
            penalty / count as f32
        } else {
            0.0
        }
    }

    /// Convert world coordinates to grid coordinates.
    #[inline]
    pub fn world_to_grid(&self, point: WorldPoint) -> GridCoord {
        let x = ((point.x - self.origin.x) / self.resolution).floor() as i32;
        let y = ((point.y - self.origin.y) / self.resolution).floor() as i32;
        GridCoord::new(x, y)
    }

    /// Convert grid coordinates to world coordinates (cell center).
    #[inline]
    pub fn grid_to_world(&self, coord: GridCoord) -> WorldPoint {
        WorldPoint::new(
            self.origin.x + (coord.x as f32 + 0.5) * self.resolution,
            self.origin.y + (coord.y as f32 + 0.5) * self.resolution,
        )
    }

    /// Get grid resolution in meters.
    #[inline]
    pub fn resolution(&self) -> f32 {
        self.resolution
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_grid() -> GridStorage {
        let mut grid = GridStorage::new(20, 20, 0.05, WorldPoint::ZERO);

        // Add some walls
        for x in 5..15 {
            grid.set_type(GridCoord::new(x, 10), CellType::Wall);
        }

        // Mark rest as floor
        for y in 0..20 {
            for x in 0..20 {
                let coord = GridCoord::new(x, y);
                if grid.get_type(coord) == CellType::Unknown {
                    grid.set_type(coord, CellType::Floor);
                }
            }
        }

        grid
    }

    #[test]
    fn test_cost_map_creation() {
        let grid = create_test_grid();
        let cost_map = CostMap::from_grid(&grid, 0.10, 0.05, 0.20);

        // Wall cells should be lethal
        assert_eq!(cost_map.cost(GridCoord::new(10, 10)), costs::LETHAL);

        // Cells far from walls should be free or low cost
        let far_cost = cost_map.cost(GridCoord::new(10, 0));
        assert!(far_cost < costs::INSCRIBED);
    }

    #[test]
    fn test_distance_field() {
        let grid = create_test_grid();
        let cost_map = CostMap::from_grid(&grid, 0.10, 0.05, 0.20);

        // Wall cells have distance 0
        assert_eq!(cost_map.obstacle_distance(GridCoord::new(10, 10)), 0.0);

        // Adjacent cells have distance ~1
        let adj_dist = cost_map.obstacle_distance(GridCoord::new(10, 9));
        assert!(adj_dist > 0.9 && adj_dist < 1.1);

        // Distant cells have larger distance
        let far_dist = cost_map.obstacle_distance(GridCoord::new(10, 0));
        assert!(far_dist > 5.0);
    }

    #[test]
    fn test_line_of_sight() {
        let grid = create_test_grid();
        let cost_map = CostMap::from_grid(&grid, 0.10, 0.05, 0.20);

        // Line that doesn't cross wall
        assert!(cost_map.line_of_sight(GridCoord::new(0, 0), GridCoord::new(5, 5)));

        // Line that crosses wall
        assert!(!cost_map.line_of_sight(GridCoord::new(10, 5), GridCoord::new(10, 15)));
    }

    #[test]
    fn test_traversability() {
        let grid = create_test_grid();
        let cost_map = CostMap::from_grid(&grid, 0.10, 0.05, 0.20);

        // Wall is not traversable
        assert!(!cost_map.is_traversable(GridCoord::new(10, 10)));

        // Open space is traversable
        assert!(cost_map.is_traversable(GridCoord::new(10, 0)));
    }
}
