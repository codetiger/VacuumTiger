//! Visibility region tracking for scan-based node generation.
//!
//! This module tracks areas that have been observed by the lidar scanner,
//! ensuring CBVG nodes are only placed in regions the robot has actually seen.
//!
//! # Algorithm
//!
//! For each lidar scan:
//! 1. For each ray from robot position to scan point
//! 2. Mark all grid cells along the ray as visible
//! 3. The endpoint (scan point) marks the boundary
//!
//! This creates a conservative estimate of free space - areas the lidar
//! has actually "seen through".

use crate::core::{Bounds, Point2D, Pose2D};

/// Grid-based visibility tracking.
///
/// Tracks which areas of the map have been observed by lidar scans.
/// Uses a grid representation for efficient lookup.
#[derive(Clone, Debug)]
pub struct VisibilityRegion {
    /// 2D grid of visibility: true = visible, false = unknown.
    /// Indexed as grid[y_cell][x_cell].
    grid: Vec<Vec<bool>>,

    /// Resolution of the visibility grid (meters per cell).
    resolution: f32,

    /// Origin of the grid in world coordinates (min corner).
    origin: Point2D,

    /// Grid dimensions.
    width: usize,
    height: usize,

    /// Current bounds of visible area.
    bounds: Bounds,
}

impl VisibilityRegion {
    /// Create a new visibility region with the given resolution.
    ///
    /// # Arguments
    /// * `resolution` - Grid cell size in meters (default: 0.1)
    pub fn new(resolution: f32) -> Self {
        Self {
            grid: Vec::new(),
            resolution,
            origin: Point2D::zero(),
            width: 0,
            height: 0,
            bounds: Bounds::empty(),
        }
    }

    /// Create a visibility region with default resolution (0.1m).
    pub fn default_resolution() -> Self {
        Self::new(0.1)
    }

    /// Get the current bounds of the visible region.
    pub fn visible_bounds(&self) -> Bounds {
        self.bounds
    }

    /// Check if the visibility region is empty (no observations yet).
    pub fn is_empty(&self) -> bool {
        self.grid.is_empty()
    }

    /// Check if a point is in a visible (scanned) area.
    ///
    /// Returns true if the point has been observed by lidar.
    pub fn is_visible(&self, point: Point2D) -> bool {
        if self.grid.is_empty() {
            return false;
        }

        // Convert to grid coordinates
        let (x_cell, y_cell) = self.world_to_grid(point);

        // Check bounds
        if x_cell < 0 || y_cell < 0 {
            return false;
        }
        let x_cell = x_cell as usize;
        let y_cell = y_cell as usize;

        if x_cell >= self.width || y_cell >= self.height {
            return false;
        }

        self.grid[y_cell][x_cell]
    }

    /// Update visibility from a lidar scan.
    ///
    /// Marks all cells along rays from robot to scan points as visible.
    ///
    /// # Arguments
    /// * `robot_pose` - Current robot pose (position and heading)
    /// * `scan_points` - Lidar scan points in world coordinates
    pub fn update_from_scan(&mut self, robot_pose: Pose2D, scan_points: &[Point2D]) {
        if scan_points.is_empty() {
            return;
        }

        let robot_pos = robot_pose.position();

        // First, ensure grid covers all scan points plus robot position
        self.ensure_coverage(robot_pos, scan_points);

        // Mark rays from robot to each scan point
        for &scan_point in scan_points {
            self.mark_ray_visible(robot_pos, scan_point);
        }
    }

    /// Update visibility from scan points already in robot-local frame.
    ///
    /// # Arguments
    /// * `robot_pose` - Current robot pose for transforming points
    /// * `local_points` - Points in robot-local frame
    pub fn update_from_local_scan(&mut self, robot_pose: Pose2D, local_points: &[Point2D]) {
        if local_points.is_empty() {
            return;
        }

        // Transform points to world frame
        let world_points: Vec<Point2D> = local_points
            .iter()
            .map(|&p| robot_pose.transform_point(p))
            .collect();

        self.update_from_scan(robot_pose, &world_points);
    }

    /// Ensure the grid covers all given points.
    fn ensure_coverage(&mut self, robot_pos: Point2D, scan_points: &[Point2D]) {
        // Calculate required bounds
        let mut required_bounds = Bounds::from_point(robot_pos);
        for &point in scan_points {
            required_bounds.expand_to_include(point);
        }

        // Add margin (one cell)
        required_bounds = required_bounds.expand(self.resolution);

        if self.grid.is_empty() {
            // Initialize grid
            self.initialize_grid(required_bounds);
        } else if !self.bounds_contain(&required_bounds) {
            // Expand grid
            self.expand_grid(required_bounds);
        }

        // Update visible bounds
        self.bounds = self.bounds.union(&required_bounds);
    }

    /// Check if current grid bounds contain the required bounds.
    fn bounds_contain(&self, required: &Bounds) -> bool {
        let grid_min = self.origin;
        let grid_max = Point2D::new(
            self.origin.x + self.width as f32 * self.resolution,
            self.origin.y + self.height as f32 * self.resolution,
        );

        required.min.x >= grid_min.x
            && required.min.y >= grid_min.y
            && required.max.x <= grid_max.x
            && required.max.y <= grid_max.y
    }

    /// Initialize the grid with the given bounds.
    fn initialize_grid(&mut self, bounds: Bounds) {
        self.origin = bounds.min;
        self.width = ((bounds.width() / self.resolution).ceil() as usize).max(1);
        self.height = ((bounds.height() / self.resolution).ceil() as usize).max(1);

        // Create grid filled with false (not visible)
        self.grid = vec![vec![false; self.width]; self.height];
    }

    /// Expand the grid to cover new bounds while preserving existing data.
    fn expand_grid(&mut self, new_bounds: Bounds) {
        // Calculate new grid bounds (union of current and required)
        let current_max = Point2D::new(
            self.origin.x + self.width as f32 * self.resolution,
            self.origin.y + self.height as f32 * self.resolution,
        );
        let current_bounds = Bounds::new(self.origin, current_max);
        let expanded = current_bounds.union(&new_bounds);

        let new_origin = expanded.min;
        let new_width = ((expanded.width() / self.resolution).ceil() as usize).max(1);
        let new_height = ((expanded.height() / self.resolution).ceil() as usize).max(1);

        // Create new grid
        let mut new_grid = vec![vec![false; new_width]; new_height];

        // Copy existing data
        let offset_x = ((self.origin.x - new_origin.x) / self.resolution).round() as isize;
        let offset_y = ((self.origin.y - new_origin.y) / self.resolution).round() as isize;

        for (old_y, row) in self.grid.iter().enumerate() {
            let new_y = old_y as isize + offset_y;
            if new_y >= 0 && (new_y as usize) < new_height {
                for (old_x, &visible) in row.iter().enumerate() {
                    let new_x = old_x as isize + offset_x;
                    if new_x >= 0 && (new_x as usize) < new_width && visible {
                        new_grid[new_y as usize][new_x as usize] = true;
                    }
                }
            }
        }

        // Update state
        self.origin = new_origin;
        self.width = new_width;
        self.height = new_height;
        self.grid = new_grid;
    }

    /// Mark all cells along a ray from start to end as visible.
    fn mark_ray_visible(&mut self, start: Point2D, end: Point2D) {
        // Use Bresenham-like line drawing
        let (x0, y0) = self.world_to_grid(start);
        let (x1, y1) = self.world_to_grid(end);

        // Simple DDA line algorithm
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let steps = dx.max(dy);

        if steps == 0 {
            self.mark_cell_visible(x0, y0);
            return;
        }

        let x_step = (x1 - x0) as f32 / steps as f32;
        let y_step = (y1 - y0) as f32 / steps as f32;

        let mut x = x0 as f32;
        let mut y = y0 as f32;

        for _ in 0..=steps {
            self.mark_cell_visible(x.round() as i32, y.round() as i32);
            x += x_step;
            y += y_step;
        }
    }

    /// Mark a single cell as visible.
    fn mark_cell_visible(&mut self, x_cell: i32, y_cell: i32) {
        if x_cell < 0 || y_cell < 0 {
            return;
        }
        let x_cell = x_cell as usize;
        let y_cell = y_cell as usize;

        if x_cell < self.width && y_cell < self.height {
            self.grid[y_cell][x_cell] = true;
        }
    }

    /// Convert world coordinates to grid cell indices.
    fn world_to_grid(&self, point: Point2D) -> (i32, i32) {
        let x = ((point.x - self.origin.x) / self.resolution).floor() as i32;
        let y = ((point.y - self.origin.y) / self.resolution).floor() as i32;
        (x, y)
    }

    /// Get the grid resolution in meters.
    pub fn resolution(&self) -> f32 {
        self.resolution
    }

    /// Get grid dimensions (width, height) in cells.
    pub fn grid_size(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// Count the number of visible cells.
    pub fn visible_cell_count(&self) -> usize {
        self.grid
            .iter()
            .map(|row| row.iter().filter(|&&v| v).count())
            .sum()
    }

    /// Get total cell count.
    pub fn total_cell_count(&self) -> usize {
        self.width * self.height
    }

    /// Get the visibility ratio (visible cells / total cells).
    pub fn visibility_ratio(&self) -> f32 {
        if self.total_cell_count() == 0 {
            return 0.0;
        }
        self.visible_cell_count() as f32 / self.total_cell_count() as f32
    }

    /// Iterator over all visible cell center points.
    pub fn visible_points(&self) -> impl Iterator<Item = Point2D> + '_ {
        self.grid.iter().enumerate().flat_map(move |(y, row)| {
            row.iter()
                .enumerate()
                .filter(|&(_, visible)| *visible)
                .map(move |(x, _)| self.grid_to_world_center(x, y))
        })
    }

    /// Convert grid cell to world coordinates (cell center).
    fn grid_to_world_center(&self, x_cell: usize, y_cell: usize) -> Point2D {
        Point2D::new(
            self.origin.x + (x_cell as f32 + 0.5) * self.resolution,
            self.origin.y + (y_cell as f32 + 0.5) * self.resolution,
        )
    }
}

impl Default for VisibilityRegion {
    fn default() -> Self {
        Self::default_resolution()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_new_visibility_region() {
        let vis = VisibilityRegion::new(0.1);
        assert!(vis.is_empty());
        assert_eq!(vis.resolution(), 0.1);
        assert!(vis.visible_bounds().is_empty());
    }

    #[test]
    fn test_empty_scan() {
        let mut vis = VisibilityRegion::new(0.1);
        let pose = Pose2D::identity();
        vis.update_from_scan(pose, &[]);
        assert!(vis.is_empty());
    }

    #[test]
    fn test_single_point_scan() {
        let mut vis = VisibilityRegion::new(0.1);
        let pose = Pose2D::identity();
        let points = vec![Point2D::new(1.0, 0.0)];

        vis.update_from_scan(pose, &points);

        // Robot position should be visible
        assert!(vis.is_visible(Point2D::new(0.0, 0.0)));
        // Point along ray should be visible
        assert!(vis.is_visible(Point2D::new(0.5, 0.0)));
        // Scan point should be visible
        assert!(vis.is_visible(Point2D::new(1.0, 0.0)));
        // Point beyond scan should not be visible
        assert!(!vis.is_visible(Point2D::new(2.0, 0.0)));
        // Point off to the side should not be visible
        assert!(!vis.is_visible(Point2D::new(0.5, 1.0)));
    }

    #[test]
    fn test_circular_scan() {
        let mut vis = VisibilityRegion::new(0.1);
        let pose = Pose2D::identity();

        // Create a circular scan (8 points at 1m radius)
        let points: Vec<Point2D> = (0..8)
            .map(|i| {
                let angle = i as f32 * PI / 4.0;
                Point2D::from_polar(angle, 1.0)
            })
            .collect();

        vis.update_from_scan(pose, &points);

        // Center should be visible
        assert!(vis.is_visible(Point2D::zero()));
        // Points halfway should be visible
        assert!(vis.is_visible(Point2D::new(0.5, 0.0)));
        assert!(vis.is_visible(Point2D::new(0.0, 0.5)));
    }

    #[test]
    fn test_incremental_scans() {
        let mut vis = VisibilityRegion::new(0.1);

        // First scan from origin
        let pose1 = Pose2D::identity();
        let points1 = vec![Point2D::new(1.0, 0.0)];
        vis.update_from_scan(pose1, &points1);

        // Second scan from different position
        let pose2 = Pose2D::new(1.0, 0.0, PI / 2.0);
        let points2 = vec![Point2D::new(1.0, 1.0)];
        vis.update_from_scan(pose2, &points2);

        // Both scan areas should be visible
        assert!(vis.is_visible(Point2D::new(0.5, 0.0)));
        assert!(vis.is_visible(Point2D::new(1.0, 0.5)));
    }

    #[test]
    fn test_visibility_bounds() {
        let mut vis = VisibilityRegion::new(0.1);
        let pose = Pose2D::identity();
        let points = vec![
            Point2D::new(1.0, 0.0),
            Point2D::new(-1.0, 0.0),
            Point2D::new(0.0, 1.0),
            Point2D::new(0.0, -1.0),
        ];

        vis.update_from_scan(pose, &points);

        let bounds = vis.visible_bounds();
        assert!(!bounds.is_empty());
        assert!(bounds.contains(Point2D::zero()));
    }

    #[test]
    fn test_grid_expansion() {
        let mut vis = VisibilityRegion::new(0.1);

        // First small scan
        let pose1 = Pose2D::identity();
        let points1 = vec![Point2D::new(0.5, 0.0)];
        vis.update_from_scan(pose1, &points1);
        let initial_size = vis.grid_size();

        // Larger scan requiring expansion
        let pose2 = Pose2D::identity();
        let points2 = vec![Point2D::new(5.0, 5.0)];
        vis.update_from_scan(pose2, &points2);
        let expanded_size = vis.grid_size();

        // Grid should have grown
        assert!(expanded_size.0 > initial_size.0);
        assert!(expanded_size.1 > initial_size.1);

        // Original visibility should be preserved
        assert!(vis.is_visible(Point2D::new(0.25, 0.0)));
    }

    #[test]
    fn test_visibility_statistics() {
        let mut vis = VisibilityRegion::new(0.1);
        let pose = Pose2D::identity();
        let points = vec![Point2D::new(1.0, 0.0)];

        vis.update_from_scan(pose, &points);

        assert!(vis.visible_cell_count() > 0);
        assert!(vis.total_cell_count() >= vis.visible_cell_count());
        let ratio = vis.visibility_ratio();
        assert!(ratio > 0.0 && ratio <= 1.0);
    }
}
