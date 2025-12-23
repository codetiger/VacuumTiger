//! Exploration history tracking.
//!
//! Tracks visited areas using a spatial grid to avoid revisiting
//! recently explored regions.

use std::collections::{HashMap, VecDeque};

use crate::core::Point2D;

/// Tracks exploration history using a spatial grid.
///
/// Used to penalize revisiting recently explored areas during
/// region selection and frontier scoring.
#[derive(Clone, Debug)]
pub struct ExplorationHistory {
    /// Grid of visited cells mapping (cell_x, cell_y) -> visit count.
    visited_grid: HashMap<(i32, i32), u8>,

    /// Cell size for the grid (meters).
    cell_size: f32,

    /// Recently visited positions (ring buffer).
    recent_positions: VecDeque<Point2D>,

    /// Maximum number of recent positions to track.
    max_recent: usize,
}

impl ExplorationHistory {
    /// Create a new exploration history.
    ///
    /// # Arguments
    /// * `cell_size` - Size of each grid cell in meters (default: 0.5)
    /// * `max_recent` - Maximum number of recent positions to track
    pub fn new(cell_size: f32, max_recent: usize) -> Self {
        Self {
            visited_grid: HashMap::with_capacity(1000),
            cell_size,
            recent_positions: VecDeque::with_capacity(max_recent),
            max_recent,
        }
    }

    /// Create with default parameters.
    pub fn with_defaults() -> Self {
        Self::new(0.5, 100)
    }

    /// Mark a position as visited.
    pub fn mark_visited(&mut self, position: Point2D) {
        // Update grid cell
        let cell = self.point_to_cell(position);
        let count = self.visited_grid.entry(cell).or_insert(0);
        *count = count.saturating_add(1);

        // Update recent positions
        self.recent_positions.push_back(position);
        if self.recent_positions.len() > self.max_recent {
            self.recent_positions.pop_front();
        }
    }

    /// Check if a position is in a recently visited area.
    ///
    /// Returns true if the position is within the same grid cell
    /// as any recent position.
    pub fn is_recently_visited(&self, position: Point2D) -> bool {
        let cell = self.point_to_cell(position);
        self.visited_grid.get(&cell).copied().unwrap_or(0) > 0
    }

    /// Check if an area around a position has been recently visited.
    ///
    /// Checks a 3x3 neighborhood of cells around the position.
    pub fn is_area_recently_visited(&self, position: Point2D) -> bool {
        let (cx, cy) = self.point_to_cell(position);

        for dx in -1..=1 {
            for dy in -1..=1 {
                let cell = (cx + dx, cy + dy);
                if self.visited_grid.get(&cell).copied().unwrap_or(0) > 0 {
                    return true;
                }
            }
        }

        false
    }

    /// Get the visit count for a position.
    pub fn get_visit_count(&self, position: Point2D) -> u8 {
        let cell = self.point_to_cell(position);
        self.visited_grid.get(&cell).copied().unwrap_or(0)
    }

    /// Get the total number of cells visited.
    pub fn cells_visited(&self) -> usize {
        self.visited_grid.len()
    }

    /// Get the number of recent positions tracked.
    pub fn recent_count(&self) -> usize {
        self.recent_positions.len()
    }

    /// Get the distance to the nearest recent position.
    ///
    /// Returns None if there are no recent positions.
    pub fn distance_to_nearest_recent(&self, position: Point2D) -> Option<f32> {
        self.recent_positions
            .iter()
            .map(|&p| position.distance(p))
            .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
    }

    /// Clear all history.
    pub fn clear(&mut self) {
        self.visited_grid.clear();
        self.recent_positions.clear();
    }

    /// Convert a point to a grid cell coordinate.
    #[inline]
    fn point_to_cell(&self, point: Point2D) -> (i32, i32) {
        (
            (point.x / self.cell_size).floor() as i32,
            (point.y / self.cell_size).floor() as i32,
        )
    }

    /// Get estimated memory usage in bytes.
    pub fn memory_usage(&self) -> usize {
        // HashMap overhead + entries
        let grid_size = self.visited_grid.len()
            * (std::mem::size_of::<(i32, i32)>() + std::mem::size_of::<u8>() + 16);
        // VecDeque + Point2D entries
        let recent_size = self.recent_positions.len() * std::mem::size_of::<Point2D>();

        grid_size + recent_size
    }
}

impl Default for ExplorationHistory {
    fn default() -> Self {
        Self::with_defaults()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_history() {
        let history = ExplorationHistory::new(0.5, 100);
        assert_eq!(history.cells_visited(), 0);
        assert_eq!(history.recent_count(), 0);
        assert!(!history.is_recently_visited(Point2D::new(0.0, 0.0)));
    }

    #[test]
    fn test_mark_visited() {
        let mut history = ExplorationHistory::new(0.5, 100);

        history.mark_visited(Point2D::new(1.0, 1.0));

        assert_eq!(history.cells_visited(), 1);
        assert_eq!(history.recent_count(), 1);
        assert!(history.is_recently_visited(Point2D::new(1.0, 1.0)));
        assert!(history.is_recently_visited(Point2D::new(1.2, 1.2))); // Same cell
        assert!(!history.is_recently_visited(Point2D::new(5.0, 5.0))); // Different cell
    }

    #[test]
    fn test_visit_count() {
        let mut history = ExplorationHistory::new(0.5, 100);

        // Visit same cell multiple times
        history.mark_visited(Point2D::new(1.0, 1.0));
        history.mark_visited(Point2D::new(1.1, 1.1));
        history.mark_visited(Point2D::new(1.2, 1.2));

        assert_eq!(history.get_visit_count(Point2D::new(1.0, 1.0)), 3);
        assert_eq!(history.cells_visited(), 1);
    }

    #[test]
    fn test_recent_positions_limit() {
        let mut history = ExplorationHistory::new(0.5, 3);

        history.mark_visited(Point2D::new(0.0, 0.0));
        history.mark_visited(Point2D::new(1.0, 0.0));
        history.mark_visited(Point2D::new(2.0, 0.0));
        history.mark_visited(Point2D::new(3.0, 0.0));

        // Should only have last 3 positions
        assert_eq!(history.recent_count(), 3);

        // But all cells should still be marked
        assert_eq!(history.cells_visited(), 4);
    }

    #[test]
    fn test_area_recently_visited() {
        let mut history = ExplorationHistory::new(1.0, 100);

        history.mark_visited(Point2D::new(5.0, 5.0));

        // Position in same cell
        assert!(history.is_area_recently_visited(Point2D::new(5.5, 5.5)));

        // Position in adjacent cell (within 3x3 neighborhood)
        assert!(history.is_area_recently_visited(Point2D::new(6.5, 5.5)));

        // Position far away
        assert!(!history.is_area_recently_visited(Point2D::new(10.0, 10.0)));
    }

    #[test]
    fn test_distance_to_nearest_recent() {
        let mut history = ExplorationHistory::new(0.5, 100);

        assert!(
            history
                .distance_to_nearest_recent(Point2D::new(0.0, 0.0))
                .is_none()
        );

        history.mark_visited(Point2D::new(0.0, 0.0));
        history.mark_visited(Point2D::new(3.0, 0.0));

        let dist = history.distance_to_nearest_recent(Point2D::new(1.0, 0.0));
        assert!(dist.is_some());
        assert!((dist.unwrap() - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_clear() {
        let mut history = ExplorationHistory::new(0.5, 100);

        history.mark_visited(Point2D::new(1.0, 1.0));
        history.mark_visited(Point2D::new(2.0, 2.0));

        assert_eq!(history.cells_visited(), 2);

        history.clear();

        assert_eq!(history.cells_visited(), 0);
        assert_eq!(history.recent_count(), 0);
    }

    #[test]
    fn test_cell_boundaries() {
        let mut history = ExplorationHistory::new(1.0, 100);

        // Points at cell boundaries
        history.mark_visited(Point2D::new(0.99, 0.99)); // Cell (0, 0)
        history.mark_visited(Point2D::new(1.01, 1.01)); // Cell (1, 1)

        assert_eq!(history.cells_visited(), 2);
        assert_eq!(history.get_visit_count(Point2D::new(0.5, 0.5)), 1);
        assert_eq!(history.get_visit_count(Point2D::new(1.5, 1.5)), 1);
    }
}
