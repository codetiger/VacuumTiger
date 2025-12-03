//! Bresenham ray tracing for marking free space in occupancy grids.
//!
//! When a laser scan detects an obstacle at distance D, all cells between
//! the robot and distance D must be free space. This module provides
//! efficient ray tracing to mark those cells.
//!
//! # Algorithm
//!
//! Uses Bresenham's line algorithm for efficient integer-only traversal
//! of grid cells along a ray from robot position to scan endpoint.

use super::OccupancyGrid;

/// Ray tracer for marking free space in occupancy grids.
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::mapping::{OccupancyGrid, RayTracer};
///
/// let mut grid = OccupancyGrid::new(config);
/// let ray_tracer = RayTracer::default();
///
/// // Mark ray from robot at origin to endpoint at (5.0, 3.0)
/// let endpoint = (5.0, 3.0);
/// ray_tracer.trace_ray(&mut grid, 0.0, 0.0, endpoint.0, endpoint.1, true);
/// ```
#[derive(Debug, Clone)]
pub struct RayTracer {
    /// Maximum ray length in cells (for safety).
    max_ray_length: usize,
}

impl Default for RayTracer {
    fn default() -> Self {
        Self {
            max_ray_length: 1000, // ~50m at 5cm resolution
        }
    }
}

impl RayTracer {
    /// Create a ray tracer with custom max length.
    pub fn new(max_ray_length: usize) -> Self {
        Self { max_ray_length }
    }

    /// Trace a ray from start to end, marking cells.
    ///
    /// All cells along the ray (except the endpoint) are marked as free.
    /// If `mark_endpoint` is true, the endpoint is marked as occupied.
    ///
    /// # Arguments
    ///
    /// * `grid` - The occupancy grid to update
    /// * `start_x`, `start_y` - Ray start point in world coordinates (robot position)
    /// * `end_x`, `end_y` - Ray end point in world coordinates (obstacle position)
    /// * `mark_endpoint` - If true, mark endpoint as occupied
    pub fn trace_ray(
        &self,
        grid: &mut OccupancyGrid,
        start_x: f32,
        start_y: f32,
        end_x: f32,
        end_y: f32,
        mark_endpoint: bool,
    ) {
        // Convert to cell coordinates
        let (sx, sy) = grid.world_to_cell_signed(start_x, start_y);
        let (ex, ey) = grid.world_to_cell_signed(end_x, end_y);

        // Use Bresenham's algorithm to traverse cells
        self.bresenham(grid, sx, sy, ex, ey, mark_endpoint);
    }

    /// Bresenham line algorithm for grid traversal.
    ///
    /// Marks all cells along the line as free, optionally marking
    /// the endpoint as occupied.
    fn bresenham(
        &self,
        grid: &mut OccupancyGrid,
        x0: i32,
        y0: i32,
        x1: i32,
        y1: i32,
        mark_endpoint: bool,
    ) {
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();

        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };

        let mut x = x0;
        let mut y = y0;
        let mut err = dx - dy;

        let mut steps = 0;

        loop {
            // Check if we've reached the endpoint
            let is_endpoint = x == x1 && y == y1;

            if is_endpoint {
                if mark_endpoint {
                    // Mark as occupied
                    grid.update_cell_signed(x, y, true);
                }
                break;
            }

            // Mark intermediate cells as free
            grid.update_cell_signed(x, y, false);

            steps += 1;
            if steps >= self.max_ray_length {
                break;
            }

            let e2 = 2 * err;

            if e2 > -dy {
                err -= dy;
                x += sx;
            }

            if e2 < dx {
                err += dx;
                y += sy;
            }
        }
    }

    /// Trace a ray from start in a given direction for a given distance.
    ///
    /// # Arguments
    ///
    /// * `grid` - The occupancy grid to update
    /// * `start_x`, `start_y` - Ray start point in world coordinates
    /// * `angle` - Ray direction in radians
    /// * `distance` - Ray length in meters
    /// * `mark_endpoint` - If true, mark endpoint as occupied
    pub fn trace_ray_polar(
        &self,
        grid: &mut OccupancyGrid,
        start_x: f32,
        start_y: f32,
        angle: f32,
        distance: f32,
        mark_endpoint: bool,
    ) {
        let (sin_a, cos_a) = angle.sin_cos();
        let end_x = start_x + distance * cos_a;
        let end_y = start_y + distance * sin_a;

        self.trace_ray(grid, start_x, start_y, end_x, end_y, mark_endpoint);
    }
}

/// Iterator over cells along a ray (without modifying grid).
///
/// Useful for checking line of sight or counting obstacles.
#[cfg(test)]
pub struct RayCellIterator {
    x: i32,
    y: i32,
    x1: i32,
    y1: i32,
    dx: i32,
    dy: i32,
    sx: i32,
    sy: i32,
    err: i32,
    finished: bool,
}

#[cfg(test)]
impl RayCellIterator {
    /// Create a new ray cell iterator.
    pub fn new(x0: i32, y0: i32, x1: i32, y1: i32) -> Self {
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };

        Self {
            x: x0,
            y: y0,
            x1,
            y1,
            dx,
            dy,
            sx,
            sy,
            err: dx - dy,
            finished: false,
        }
    }
}

#[cfg(test)]
impl Iterator for RayCellIterator {
    type Item = (i32, i32);

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        let result = (self.x, self.y);

        if self.x == self.x1 && self.y == self.y1 {
            self.finished = true;
            return Some(result);
        }

        let e2 = 2 * self.err;

        if e2 > -self.dy {
            self.err -= self.dy;
            self.x += self.sx;
        }

        if e2 < self.dx {
            self.err += self.dx;
            self.y += self.sy;
        }

        Some(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::mapping::OccupancyGridConfig;

    fn create_test_grid() -> OccupancyGrid {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 10.0,
            initial_height: 10.0,
            ..Default::default()
        };
        OccupancyGrid::new(config)
    }

    #[test]
    fn test_ray_cell_iterator_horizontal() {
        let iter = RayCellIterator::new(0, 0, 5, 0);
        let cells: Vec<_> = iter.collect();

        assert_eq!(cells.len(), 6); // 0 to 5 inclusive
        assert_eq!(cells[0], (0, 0));
        assert_eq!(cells[5], (5, 0));

        // All should have y = 0
        for (_, y) in &cells {
            assert_eq!(*y, 0);
        }
    }

    #[test]
    fn test_ray_cell_iterator_vertical() {
        let iter = RayCellIterator::new(0, 0, 0, 5);
        let cells: Vec<_> = iter.collect();

        assert_eq!(cells.len(), 6);
        assert_eq!(cells[0], (0, 0));
        assert_eq!(cells[5], (0, 5));
    }

    #[test]
    fn test_ray_cell_iterator_diagonal() {
        let iter = RayCellIterator::new(0, 0, 5, 5);
        let cells: Vec<_> = iter.collect();

        // Diagonal should hit each cell
        assert!(cells.len() >= 6);
        assert_eq!(cells[0], (0, 0));
        assert_eq!(*cells.last().unwrap(), (5, 5));
    }

    #[test]
    fn test_ray_cell_iterator_negative_direction() {
        let iter = RayCellIterator::new(5, 5, 0, 0);
        let cells: Vec<_> = iter.collect();

        assert_eq!(cells[0], (5, 5));
        assert_eq!(*cells.last().unwrap(), (0, 0));
    }

    #[test]
    fn test_trace_ray_marks_free_space() {
        let mut grid = create_test_grid();
        let tracer = RayTracer::default();

        // Trace ray from origin toward (2, 0)
        tracer.trace_ray(&mut grid, 0.0, 0.0, 2.0, 0.0, true);

        // Start cell should be marked free
        let (sx, sy) = grid.world_to_cell(0.0, 0.0).unwrap();
        assert!(grid.get_log_odds(sx, sy) < 0.0, "Start should be free");

        // Middle cells should be free
        let (mx, my) = grid.world_to_cell(1.0, 0.0).unwrap();
        assert!(grid.get_log_odds(mx, my) < 0.0, "Middle should be free");

        // Endpoint should be occupied
        let (ex, ey) = grid.world_to_cell(2.0, 0.0).unwrap();
        assert!(
            grid.get_log_odds(ex, ey) > 0.0,
            "Endpoint should be occupied"
        );
    }

    #[test]
    fn test_trace_ray_without_endpoint() {
        let mut grid = create_test_grid();
        let tracer = RayTracer::default();

        // Trace ray without marking endpoint (for max range returns)
        tracer.trace_ray(&mut grid, 0.0, 0.0, 2.0, 0.0, false);

        // Endpoint should NOT be marked
        let (ex, ey) = grid.world_to_cell(2.0, 0.0).unwrap();
        assert!(grid.get_log_odds(ex, ey) == 0.0 || grid.get_log_odds(ex, ey) < 0.0);
    }

    #[test]
    fn test_trace_ray_polar() {
        let mut grid = create_test_grid();
        let tracer = RayTracer::default();

        use std::f32::consts::FRAC_PI_4;

        // Trace ray at 45 degrees for 1.5m
        tracer.trace_ray_polar(&mut grid, 0.0, 0.0, FRAC_PI_4, 1.5, true);

        // Should have marked some cells as free
        let (free, _, occupied) = grid.count_cells();
        assert!(free > 0, "Should have marked free cells");
        assert!(occupied > 0, "Should have marked endpoint");
    }

    #[test]
    fn test_trace_multiple_rays() {
        let mut grid = create_test_grid();
        let tracer = RayTracer::default();

        // Trace several rays from origin
        for i in 0..8 {
            let angle = (i as f32) * std::f32::consts::FRAC_PI_4;
            tracer.trace_ray_polar(&mut grid, 0.0, 0.0, angle, 2.0, true);
        }

        // Should have marked a pattern of free and occupied cells
        let (free, _, occupied) = grid.count_cells();
        assert!(free > 10, "Should have many free cells");
        assert!(occupied >= 8, "Should have at least 8 occupied endpoints");
    }

    #[test]
    fn test_max_ray_length() {
        let mut grid = create_test_grid();
        let tracer = RayTracer::new(10); // Very short max length

        // Try to trace a long ray
        tracer.trace_ray(&mut grid, 0.0, 0.0, 100.0, 0.0, true);

        // Should not crash, and should have limited effect
        // (grid would need to resize for the full ray)
    }
}
