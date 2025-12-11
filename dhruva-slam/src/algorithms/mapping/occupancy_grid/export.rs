//! Export functionality for occupancy grids.

use super::OccupancyGrid;
use super::config::CellState;
use crate::algorithms::mapping::MapRegion;
use crate::core::types::Point2D;

impl OccupancyGrid {
    /// Export map as grayscale image data.
    ///
    /// Returns (width, height, pixels) where pixels are 0-255 grayscale values.
    /// 0 = occupied, 128 = unknown, 255 = free
    pub fn to_grayscale(&self) -> (usize, usize, Vec<u8>) {
        let mut pixels = Vec::with_capacity(self.width() * self.height());

        for y in 0..self.height() {
            for x in 0..self.width() {
                let state = self.get_state(x, y);
                let value = match state {
                    CellState::Free => 255u8,
                    CellState::Unknown => 128u8,
                    CellState::Occupied => 0u8,
                };
                pixels.push(value);
            }
        }

        (self.width(), self.height(), pixels)
    }

    /// Count cells by state.
    pub fn count_cells(&self) -> (usize, usize, usize) {
        let mut free = 0;
        let mut unknown = 0;
        let mut occupied = 0;

        let config = self.config();
        for &log_odds in self.cells() {
            if log_odds >= config.occupied_threshold {
                occupied += 1;
            } else if log_odds <= config.free_threshold {
                free += 1;
            } else {
                unknown += 1;
            }
        }

        (free, unknown, occupied)
    }

    /// Get memory usage in bytes.
    pub fn memory_usage(&self) -> usize {
        std::mem::size_of_val(self.cells())
    }

    /// Get all occupied cell centers as world-coordinate points.
    ///
    /// Useful for feature extraction algorithms.
    pub fn occupied_points(&self) -> Vec<Point2D> {
        let mut points = Vec::new();

        for cy in 0..self.height() {
            for cx in 0..self.width() {
                if self.get_state(cx, cy) == CellState::Occupied {
                    let (x, y) = self.cell_to_world(cx, cy);
                    points.push(Point2D::new(x, y));
                }
            }
        }

        points
    }

    /// Get occupied cell centers within a specific region.
    pub fn occupied_points_in_region(&self, region: &MapRegion) -> Vec<Point2D> {
        let mut points = Vec::new();

        let min_x = region.min_x.max(0) as usize;
        let max_x = (region.max_x as usize).min(self.width().saturating_sub(1));
        let min_y = region.min_y.max(0) as usize;
        let max_y = (region.max_y as usize).min(self.height().saturating_sub(1));

        for cy in min_y..=max_y {
            for cx in min_x..=max_x {
                if self.get_state(cx, cy) == CellState::Occupied {
                    let (x, y) = self.cell_to_world(cx, cy);
                    points.push(Point2D::new(x, y));
                }
            }
        }

        points
    }
}
