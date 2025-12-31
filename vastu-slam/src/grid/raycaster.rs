//! Bresenham ray casting for occupancy grid updates.
//!
//! This module provides efficient line drawing algorithms for:
//! - Marking cells along lidar rays as free (Floor)
//! - Marking endpoint cells as occupied (Wall)
//!
//! ## Algorithms
//!
//! ### Bresenham Line Algorithm
//!
//! Integer-only algorithm for drawing lines between grid cells.
//! Optimal for ray tracing in occupancy grids:
//!
//! ```text
//! From (0,0) to (7,3):
//!
//!     3 │        ●
//!     2 │     ●●
//!     1 │  ●●
//!     0 ●●
//!       └──────────
//!        0 1 2 3 4 5 6 7
//! ```
//!
//! **Advantages:**
//! - No floating-point operations
//! - Deterministic cell coverage
//! - No gaps in traced line
//!
//! ### Continuous Ray Casting
//!
//! Floating-point ray stepping for distance-aware queries.
//! Useful when distance information is needed:
//!
//! ```text
//! start ●────●────●────●────● end
//!       d=0  d=0.1 d=0.2 ...
//! ```
//!
//! ## Usage
//!
//! ```rust,ignore
//! use vastu_slam::grid::raycaster::{BresenhamLine, cells_along_ray};
//!
//! // Iterate cells along a ray
//! for cell in BresenhamLine::new(start, end) {
//!     // Apply log-odds update
//! }
//!
//! // Collect all cells (excluding endpoint for miss updates)
//! let cells = cells_along_ray_excluding_end(start, end);
//! ```

use crate::core::{GridCoord, WorldPoint};

/// Bresenham's line algorithm iterator.
///
/// Generates all grid cells along a line from start to end.
/// Uses the standard Bresenham algorithm for efficient integer arithmetic.
pub struct BresenhamLine {
    x: i32,
    y: i32,
    dx: i32,
    dy: i32,
    x_inc: i32,
    y_inc: i32,
    error: i32,
    steep: bool,
    end_x: i32,
    end_y: i32,
    done: bool,
}

impl BresenhamLine {
    /// Create a new Bresenham line iterator from start to end coordinates.
    pub fn new(start: GridCoord, end: GridCoord) -> Self {
        let dx = (end.x - start.x).abs();
        let dy = (end.y - start.y).abs();
        let steep = dy > dx;

        let (x, y, end_x, end_y, dx, dy) = if steep {
            (start.y, start.x, end.y, end.x, dy, dx)
        } else {
            (start.x, start.y, end.x, end.y, dx, dy)
        };

        let x_inc = if end_x > x { 1 } else { -1 };
        let y_inc = if end_y > y { 1 } else { -1 };

        Self {
            x,
            y,
            dx,
            dy,
            x_inc,
            y_inc,
            error: dx / 2,
            steep,
            end_x,
            end_y,
            done: false,
        }
    }

    /// Create a line iterator from world points given a resolution.
    pub fn from_world(
        start: WorldPoint,
        end: WorldPoint,
        origin: WorldPoint,
        resolution: f32,
    ) -> Self {
        let start_coord = GridCoord::new(
            ((start.x - origin.x) / resolution).floor() as i32,
            ((start.y - origin.y) / resolution).floor() as i32,
        );
        let end_coord = GridCoord::new(
            ((end.x - origin.x) / resolution).floor() as i32,
            ((end.y - origin.y) / resolution).floor() as i32,
        );
        Self::new(start_coord, end_coord)
    }
}

impl Iterator for BresenhamLine {
    type Item = GridCoord;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }

        let result = if self.steep {
            GridCoord::new(self.y, self.x)
        } else {
            GridCoord::new(self.x, self.y)
        };

        // Check if we've reached the end
        if self.x == self.end_x && self.y == self.end_y {
            self.done = true;
            return Some(result);
        }

        // Advance to next cell
        self.error -= self.dy;
        if self.error < 0 {
            self.y += self.y_inc;
            self.error += self.dx;
        }
        self.x += self.x_inc;

        Some(result)
    }
}

/// Ray casting result for a single ray.
#[derive(Clone, Copy, Debug)]
pub struct RaycastResult {
    /// Distance to hit (or max_range if no hit)
    pub distance: f32,
    /// Grid coordinate of hit cell (if any)
    pub hit_coord: Option<GridCoord>,
    /// Did the ray hit an obstacle?
    pub hit: bool,
}

/// Cast a ray through the grid and return cells along the path.
///
/// This is a more flexible version that allows custom handling of each cell.
/// Returns an iterator of (GridCoord, distance_from_origin).
pub struct RayCast {
    /// Current position along the ray (world coordinates)
    current: WorldPoint,
    /// Direction unit vector
    direction: WorldPoint,
    /// Step size (typically resolution / 2 for accuracy)
    step_size: f32,
    /// Maximum range
    max_range: f32,
    /// Current distance traveled
    distance: f32,
    /// Grid origin
    origin: WorldPoint,
    /// Grid resolution
    resolution: f32,
    /// Last grid coordinate (to avoid duplicates)
    last_coord: Option<GridCoord>,
}

impl RayCast {
    /// Create a new ray cast from origin in a given direction.
    pub fn new(
        start: WorldPoint,
        angle: f32,
        max_range: f32,
        grid_origin: WorldPoint,
        resolution: f32,
    ) -> Self {
        let direction = WorldPoint::new(angle.cos(), angle.sin());
        // Step size is half the resolution for accuracy
        let step_size = resolution * 0.5;

        Self {
            current: start,
            direction,
            step_size,
            max_range,
            distance: 0.0,
            origin: grid_origin,
            resolution,
            last_coord: None,
        }
    }

    /// Get the current world position
    pub fn current_position(&self) -> WorldPoint {
        self.current
    }

    /// Get the current distance traveled
    pub fn current_distance(&self) -> f32 {
        self.distance
    }
}

impl Iterator for RayCast {
    type Item = (GridCoord, f32);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.distance > self.max_range {
                return None;
            }

            // Calculate current grid coordinate
            let coord = GridCoord::new(
                ((self.current.x - self.origin.x) / self.resolution).floor() as i32,
                ((self.current.y - self.origin.y) / self.resolution).floor() as i32,
            );

            // Advance position
            self.current.x += self.direction.x * self.step_size;
            self.current.y += self.direction.y * self.step_size;
            self.distance += self.step_size;

            // Only yield if this is a new cell
            if self.last_coord != Some(coord) {
                self.last_coord = Some(coord);
                return Some((coord, self.distance - self.step_size));
            }
        }
    }
}

/// Collect all cells along a ray from start to end.
///
/// Uses Bresenham's algorithm which is faster than stepping but
/// doesn't provide distance information.
pub fn cells_along_ray(start: GridCoord, end: GridCoord) -> Vec<GridCoord> {
    BresenhamLine::new(start, end).collect()
}

/// Collect all cells along a ray, excluding the endpoint.
pub fn cells_along_ray_excluding_end(start: GridCoord, end: GridCoord) -> Vec<GridCoord> {
    let mut cells: Vec<_> = BresenhamLine::new(start, end).collect();
    if !cells.is_empty() {
        cells.pop(); // Remove endpoint
    }
    cells
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bresenham_horizontal() {
        let start = GridCoord::new(0, 0);
        let end = GridCoord::new(5, 0);
        let cells: Vec<_> = BresenhamLine::new(start, end).collect();

        assert_eq!(cells.len(), 6);
        assert_eq!(cells[0], GridCoord::new(0, 0));
        assert_eq!(cells[5], GridCoord::new(5, 0));
    }

    #[test]
    fn test_bresenham_vertical() {
        let start = GridCoord::new(0, 0);
        let end = GridCoord::new(0, 5);
        let cells: Vec<_> = BresenhamLine::new(start, end).collect();

        assert_eq!(cells.len(), 6);
        assert_eq!(cells[0], GridCoord::new(0, 0));
        assert_eq!(cells[5], GridCoord::new(0, 5));
    }

    #[test]
    fn test_bresenham_diagonal() {
        let start = GridCoord::new(0, 0);
        let end = GridCoord::new(5, 5);
        let cells: Vec<_> = BresenhamLine::new(start, end).collect();

        assert_eq!(cells.len(), 6);
        assert_eq!(cells[0], GridCoord::new(0, 0));
        assert_eq!(cells[5], GridCoord::new(5, 5));
    }

    #[test]
    fn test_bresenham_negative() {
        let start = GridCoord::new(5, 5);
        let end = GridCoord::new(0, 0);
        let cells: Vec<_> = BresenhamLine::new(start, end).collect();

        assert_eq!(cells.len(), 6);
        assert_eq!(cells[0], GridCoord::new(5, 5));
        assert_eq!(cells[5], GridCoord::new(0, 0));
    }

    #[test]
    fn test_bresenham_steep() {
        let start = GridCoord::new(0, 0);
        let end = GridCoord::new(2, 5);
        let cells: Vec<_> = BresenhamLine::new(start, end).collect();

        // Should have 6 cells (max of dx, dy + 1)
        assert_eq!(cells.len(), 6);
        assert_eq!(cells[0], GridCoord::new(0, 0));
        assert_eq!(cells[5], GridCoord::new(2, 5));
    }

    #[test]
    fn test_raycast_cells() {
        let origin = WorldPoint::new(0.0, 0.0);
        let resolution = 0.1;

        // Cast ray from (0.05, 0.05) at 0 degrees for 0.5m
        let start = WorldPoint::new(0.05, 0.05);
        let ray = RayCast::new(start, 0.0, 0.5, origin, resolution);
        let cells: Vec<_> = ray.collect();

        // Should hit multiple cells along X axis
        assert!(!cells.is_empty());
        assert_eq!(cells[0].0, GridCoord::new(0, 0));
    }

    #[test]
    fn test_cells_excluding_end() {
        let start = GridCoord::new(0, 0);
        let end = GridCoord::new(5, 0);
        let cells = cells_along_ray_excluding_end(start, end);

        assert_eq!(cells.len(), 5);
        assert!(!cells.contains(&end));
    }
}
