//! Grid-based spatial index for fast line proximity queries.
//!
//! This module provides O(1) average-case spatial lookup for line segments,
//! reducing ICP correspondence search from O(n×m) to O(n×k) where k is the
//! average number of lines per grid cell.
//!
//! # Algorithm
//!
//! 1. Divide 2D space into uniform grid cells of size `cell_size`
//! 2. Each line is stored in all cells its bounding box overlaps
//! 3. Point queries return line indices from the cell containing the point
//!    plus neighboring cells (to handle lines near cell boundaries)
//!
//! # Performance
//!
//! - Build: O(m) where m = number of lines
//! - Query: O(k) where k = average lines per cell (~10-20 for typical indoor maps)
//! - Memory: O(m × overlap_factor) where overlap_factor ≈ 2-4 for typical line lengths
//!
//! # When to Use
//!
//! Spatial indexing is only beneficial for larger maps. For small rooms (< 4m),
//! the overhead of building and querying the index exceeds brute-force search.
//! Use `should_use_spatial_index()` to check if indexing is worthwhile.

use std::collections::HashMap;

use super::LineCollection;
use crate::core::Point2D;

/// Minimum cell size in meters.
/// For typical indoor SLAM with ~4m rooms, cells smaller than this create
/// unnecessary overhead without improving query performance.
pub const MIN_CELL_SIZE: f32 = 4.0;

/// Minimum number of lines before spatial indexing is beneficial.
/// Below this threshold, brute-force O(n×m) is faster than index overhead.
pub const MIN_LINES_FOR_INDEX: usize = 30;

/// Check if spatial indexing should be used for a given line collection.
///
/// Returns `true` if the map is large enough to benefit from spatial indexing.
/// For small maps (< 30 lines or < 8m extent), brute-force is faster.
#[inline]
pub fn should_use_spatial_index(lines: &LineCollection) -> bool {
    // Too few lines - brute force is faster
    if lines.len() < MIN_LINES_FOR_INDEX {
        return false;
    }

    // Check if map extent is large enough to benefit from spatial indexing
    // For a 4x4m room with 4m cells, we'd have just 1 cell - not worth it
    let (min_x, min_y, max_x, max_y) = lines_bounds(lines);
    let extent_x = max_x - min_x;
    let extent_y = max_y - min_y;

    // Need at least 2 cells in one dimension to benefit
    extent_x > MIN_CELL_SIZE * 2.0 || extent_y > MIN_CELL_SIZE * 2.0
}

/// Compute bounds of a line collection.
fn lines_bounds(lines: &LineCollection) -> (f32, f32, f32, f32) {
    if lines.is_empty() {
        return (0.0, 0.0, 0.0, 0.0);
    }

    let mut min_x = f32::MAX;
    let mut min_y = f32::MAX;
    let mut max_x = f32::MIN;
    let mut max_y = f32::MIN;

    for i in 0..lines.len() {
        min_x = min_x.min(lines.start_xs[i]).min(lines.end_xs[i]);
        min_y = min_y.min(lines.start_ys[i]).min(lines.end_ys[i]);
        max_x = max_x.max(lines.start_xs[i]).max(lines.end_xs[i]);
        max_y = max_y.max(lines.start_ys[i]).max(lines.end_ys[i]);
    }

    (min_x, min_y, max_x, max_y)
}

/// Grid cell coordinate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct CellCoord {
    x: i32,
    y: i32,
}

impl CellCoord {
    #[inline]
    fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// Get all neighboring cells (including self) for a query radius.
    #[inline]
    fn neighbors(&self, radius: i32) -> impl Iterator<Item = CellCoord> + '_ {
        let r = radius;
        (-r..=r)
            .flat_map(move |dx| (-r..=r).map(move |dy| CellCoord::new(self.x + dx, self.y + dy)))
    }
}

/// Grid-based spatial index for line segments.
///
/// Provides fast spatial queries for finding lines near a point.
/// Used to accelerate ICP correspondence search.
///
/// # Example
/// ```rust,ignore
/// use vastu_map::features::{LineCollection, LineSpatialIndex};
///
/// let lines = LineCollection::from_lines(&[...]);
/// let index = LineSpatialIndex::build(&lines, 0.5);  // 0.5m cell size
///
/// // Query for lines near a point (returns indices)
/// let candidates = index.query_radius(Point2D::new(1.0, 2.0), 0.3);
/// ```
#[derive(Clone, Debug)]
pub struct LineSpatialIndex {
    /// Grid cell size in meters.
    cell_size: f32,
    /// Inverse cell size for fast coordinate conversion.
    inv_cell_size: f32,
    /// Map from cell coordinate to line indices in that cell.
    grid: HashMap<CellCoord, Vec<usize>>,
    /// Total number of lines indexed.
    num_lines: usize,
    /// Bounds of indexed area.
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
}

impl LineSpatialIndex {
    /// Build a spatial index from a line collection.
    ///
    /// # Arguments
    /// * `lines` - Line collection to index
    /// * `cell_size` - Grid cell size in meters (will be clamped to MIN_CELL_SIZE minimum)
    ///
    /// # Performance
    /// Larger cell sizes = fewer cells but more lines per cell.
    /// Smaller cell sizes = more cells but fewer lines per cell.
    /// For small rooms (~4m), use MIN_CELL_SIZE to avoid overhead.
    pub fn build(lines: &LineCollection, cell_size: f32) -> Self {
        // Enforce minimum cell size to avoid overhead for small maps
        let cell_size = cell_size.max(MIN_CELL_SIZE);
        let inv_cell_size = 1.0 / cell_size;
        let num_lines = lines.len();

        let mut grid: HashMap<CellCoord, Vec<usize>> = HashMap::new();
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for i in 0..num_lines {
            let sx = lines.start_xs[i];
            let sy = lines.start_ys[i];
            let ex = lines.end_xs[i];
            let ey = lines.end_ys[i];

            // Update bounds
            min_x = min_x.min(sx).min(ex);
            min_y = min_y.min(sy).min(ey);
            max_x = max_x.max(sx).max(ex);
            max_y = max_y.max(sy).max(ey);

            // Compute bounding box cells
            let cell_min_x = (sx.min(ex) * inv_cell_size).floor() as i32;
            let cell_min_y = (sy.min(ey) * inv_cell_size).floor() as i32;
            let cell_max_x = (sx.max(ex) * inv_cell_size).floor() as i32;
            let cell_max_y = (sy.max(ey) * inv_cell_size).floor() as i32;

            // Insert line into all overlapping cells
            for cx in cell_min_x..=cell_max_x {
                for cy in cell_min_y..=cell_max_y {
                    let coord = CellCoord::new(cx, cy);
                    grid.entry(coord).or_default().push(i);
                }
            }
        }

        Self {
            cell_size,
            inv_cell_size,
            grid,
            num_lines,
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    /// Build with automatic cell size based on query radius.
    ///
    /// Uses cell_size = 2 × query_radius for optimal performance.
    #[inline]
    pub fn build_for_radius(lines: &LineCollection, query_radius: f32) -> Self {
        Self::build(lines, query_radius * 2.0)
    }

    /// Query lines near a point within a radius.
    ///
    /// Returns indices of lines whose bounding boxes are within `radius` of the point.
    /// The actual distances should still be computed for precise filtering.
    ///
    /// # Arguments
    /// * `point` - Query point
    /// * `radius` - Search radius in meters
    ///
    /// # Returns
    /// Vector of line indices that may be within radius (sorted and deduplicated).
    #[inline]
    pub fn query_radius(&self, point: Point2D, radius: f32) -> Vec<usize> {
        let mut result = Vec::new();
        self.query_radius_into(point, radius, &mut result);
        result
    }

    /// Query lines near a point, returning results in a reusable buffer.
    ///
    /// This is the zero-allocation version for hot paths.
    /// The buffer is cleared and filled with candidate line indices.
    ///
    /// # Arguments
    /// * `point` - Query point
    /// * `radius` - Search radius in meters
    /// * `buffer` - Reusable buffer for results (will be cleared and sorted)
    ///
    /// # Note
    /// Results are sorted and deduplicated to handle lines spanning multiple cells.
    pub fn query_radius_into(&self, point: Point2D, radius: f32, buffer: &mut Vec<usize>) {
        buffer.clear();

        let cell = self.point_to_cell(point);
        let cell_radius = (radius * self.inv_cell_size).ceil() as i32;

        // Collect all candidates
        for neighbor in cell.neighbors(cell_radius) {
            if let Some(indices) = self.grid.get(&neighbor) {
                buffer.extend(indices.iter().copied());
            }
        }

        // Sort and deduplicate (lines can appear in multiple cells)
        if buffer.len() > 1 {
            buffer.sort_unstable();
            buffer.dedup();
        }
    }

    /// Convert a point to its grid cell coordinate.
    #[inline]
    fn point_to_cell(&self, point: Point2D) -> CellCoord {
        CellCoord::new(
            (point.x * self.inv_cell_size).floor() as i32,
            (point.y * self.inv_cell_size).floor() as i32,
        )
    }

    /// Get the number of lines in the index.
    #[inline]
    pub fn len(&self) -> usize {
        self.num_lines
    }

    /// Check if the index is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.num_lines == 0
    }

    /// Get the cell size used by this index.
    #[inline]
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }

    /// Get the number of non-empty cells.
    #[inline]
    pub fn num_cells(&self) -> usize {
        self.grid.len()
    }

    /// Get the average number of lines per cell.
    pub fn avg_lines_per_cell(&self) -> f32 {
        if self.grid.is_empty() {
            return 0.0;
        }
        let total: usize = self.grid.values().map(|v| v.len()).sum();
        total as f32 / self.grid.len() as f32
    }

    /// Get the bounds of the indexed area.
    #[inline]
    pub fn bounds(&self) -> (f32, f32, f32, f32) {
        (self.min_x, self.min_y, self.max_x, self.max_y)
    }
}

impl Default for LineSpatialIndex {
    fn default() -> Self {
        Self {
            cell_size: 0.5,
            inv_cell_size: 2.0,
            grid: HashMap::new(),
            num_lines: 0,
            min_x: 0.0,
            min_y: 0.0,
            max_x: 0.0,
            max_y: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::features::Line2D;

    fn make_grid_lines() -> LineCollection {
        // Create a 3x3 grid of horizontal lines
        let mut lines = Vec::new();
        for y in 0..3 {
            lines.push(Line2D::new(
                Point2D::new(0.0, y as f32),
                Point2D::new(3.0, y as f32),
            ));
        }
        // Add some vertical lines
        for x in 0..3 {
            lines.push(Line2D::new(
                Point2D::new(x as f32, 0.0),
                Point2D::new(x as f32, 3.0),
            ));
        }
        LineCollection::from_lines(&lines)
    }

    #[test]
    fn test_build_empty() {
        let lines = LineCollection::new();
        let index = LineSpatialIndex::build(&lines, 0.5);

        assert!(index.is_empty());
        assert_eq!(index.num_cells(), 0);
    }

    #[test]
    fn test_build_single_line() {
        let lines = LineCollection::from_lines(&[Line2D::new(
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
        )]);

        let index = LineSpatialIndex::build(&lines, 0.5);

        assert_eq!(index.len(), 1);
        assert!(!index.is_empty());
    }

    #[test]
    fn test_query_finds_nearby_lines() {
        let lines = make_grid_lines();
        let index = LineSpatialIndex::build(&lines, 0.5);

        // Query near origin - should find lines at y=0 and x=0
        let candidates = index.query_radius(Point2D::new(0.1, 0.1), 0.5);

        assert!(!candidates.is_empty());
        // Should contain at least the horizontal line at y=0 and vertical line at x=0
    }

    #[test]
    fn test_query_radius_into() {
        let lines = make_grid_lines();
        let index = LineSpatialIndex::build(&lines, 0.5);

        let mut buffer = Vec::new();
        index.query_radius_into(Point2D::new(0.1, 0.1), 0.5, &mut buffer);

        assert!(!buffer.is_empty());
        // Results should be sorted and unique
        for i in 1..buffer.len() {
            assert!(
                buffer[i] > buffer[i - 1],
                "Buffer should be sorted and unique"
            );
        }
    }

    #[test]
    fn test_query_no_results_far_away() {
        let lines = LineCollection::from_lines(&[Line2D::new(
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
        )]);

        let index = LineSpatialIndex::build(&lines, 0.5);

        // Query far from the line
        let candidates = index.query_radius(Point2D::new(100.0, 100.0), 0.5);

        assert!(candidates.is_empty());
    }

    #[test]
    fn test_line_in_multiple_cells() {
        // Long line spanning multiple cells
        let lines = LineCollection::from_lines(&[Line2D::new(
            Point2D::new(0.0, 0.0),
            Point2D::new(5.0, 0.0),
        )]);

        let index = LineSpatialIndex::build(&lines, 0.5);

        // Line should be found from any cell along its length
        for x in 0..5 {
            let candidates = index.query_radius(Point2D::new(x as f32 + 0.25, 0.0), 0.3);
            assert!(candidates.contains(&0), "Line should be found at x={}", x);
        }
    }

    #[test]
    fn test_build_for_radius() {
        let lines = make_grid_lines();
        let index = LineSpatialIndex::build_for_radius(&lines, 0.25);

        // Cell size should be at least MIN_CELL_SIZE (4.0m)
        // Even though 2 × 0.25 = 0.5, it's clamped to MIN_CELL_SIZE
        assert!((index.cell_size() - MIN_CELL_SIZE).abs() < 0.01);
    }

    #[test]
    fn test_should_use_spatial_index_small_map() {
        // Small map with few lines - should NOT use spatial index
        let lines = make_grid_lines(); // 6 lines in 3x3m area
        assert!(!should_use_spatial_index(&lines));
    }

    #[test]
    fn test_should_use_spatial_index_large_map() {
        // Large map with many lines - should use spatial index
        let mut lines = LineCollection::new();
        for i in 0..50 {
            let y = i as f32 * 0.5;
            lines.push_full(0.0, y, 20.0, y, 1, 10); // 20m wide lines
        }
        assert!(should_use_spatial_index(&lines));
    }

    #[test]
    fn test_avg_lines_per_cell() {
        let lines = make_grid_lines();
        let index = LineSpatialIndex::build(&lines, 0.5);

        let avg = index.avg_lines_per_cell();
        assert!(avg > 0.0);
        // With 6 lines (3 horizontal + 3 vertical) over a 3x3 area,
        // average should be reasonable
    }

    #[test]
    fn test_bounds() {
        let lines = LineCollection::from_lines(&[Line2D::new(
            Point2D::new(-1.0, -2.0),
            Point2D::new(3.0, 4.0),
        )]);

        let index = LineSpatialIndex::build(&lines, 0.5);
        let (min_x, min_y, max_x, max_y) = index.bounds();

        assert!(min_x <= -1.0);
        assert!(min_y <= -2.0);
        assert!(max_x >= 3.0);
        assert!(max_y >= 4.0);
    }
}
