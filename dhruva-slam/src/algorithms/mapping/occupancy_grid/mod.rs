//! Occupancy grid map with log-odds probabilities.
//!
//! Uses log-odds representation for efficient Bayesian updates.
//!
//! # Log-Odds Representation
//!
//! ```text
//! P(occupied) = 1 / (1 + exp(-log_odds))
//!
//! log_odds = log(P(occupied) / P(free))
//!
//! Update: log_odds_new = log_odds_old + log_odds_observation
//! ```
//!
//! Benefits:
//! - Simple addition for updates (no multiplication)
//! - Avoids numerical issues at probability extremes
//! - Easy to clamp values

mod config;
mod export;
mod serialization;

pub use config::{CellState, OccupancyGridConfig};

use std::path::Path;

use super::{MapMetadata, MapRegion};

/// 2D occupancy grid map.
///
/// Stores log-odds values for each cell. Supports dynamic growth
/// as the robot explores new areas.
#[derive(Debug)]
pub struct OccupancyGrid {
    config: OccupancyGridConfig,

    /// Grid cells (log-odds values).
    ///
    /// Row-major storage: index = y * width + x
    cells: Vec<f32>,

    /// Grid width in cells.
    width: usize,

    /// Grid height in cells.
    height: usize,

    /// World X coordinate of cell (0, 0).
    origin_x: f32,

    /// World Y coordinate of cell (0, 0).
    origin_y: f32,

    /// Region of cells that have been updated.
    updated_region: Option<MapRegion>,
}

impl OccupancyGrid {
    /// Create a new occupancy grid.
    pub fn new(config: OccupancyGridConfig) -> Self {
        let width = (config.initial_width / config.resolution).ceil() as usize;
        let height = (config.initial_height / config.resolution).ceil() as usize;

        // Center the grid around origin
        let origin_x = -config.initial_width / 2.0;
        let origin_y = -config.initial_height / 2.0;

        Self {
            config,
            cells: vec![0.0; width * height], // 0.0 = unknown
            width,
            height,
            origin_x,
            origin_y,
            updated_region: None,
        }
    }

    /// Create from raw data (used by serialization).
    pub(crate) fn from_raw(
        config: OccupancyGridConfig,
        cells: Vec<f32>,
        width: usize,
        height: usize,
        origin_x: f32,
        origin_y: f32,
    ) -> Self {
        Self {
            config,
            cells,
            width,
            height,
            origin_x,
            origin_y,
            updated_region: None,
        }
    }

    /// Get the configuration.
    pub fn config(&self) -> &OccupancyGridConfig {
        &self.config
    }

    /// Get the raw cells (for serialization).
    pub(crate) fn cells(&self) -> &[f32] {
        &self.cells
    }

    /// Get grid width in cells.
    pub fn width(&self) -> usize {
        self.width
    }

    /// Get grid height in cells.
    pub fn height(&self) -> usize {
        self.height
    }

    /// Get grid dimensions.
    pub fn dimensions(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// Get the resolution in meters per cell.
    pub fn resolution(&self) -> f32 {
        self.config.resolution
    }

    /// Get grid origin in world coordinates.
    pub fn origin(&self) -> (f32, f32) {
        (self.origin_x, self.origin_y)
    }

    /// Convert world coordinates to cell indices.
    ///
    /// Returns `None` if outside grid bounds.
    #[inline]
    pub fn world_to_cell(&self, x: f32, y: f32) -> Option<(usize, usize)> {
        let cx = ((x - self.origin_x) / self.config.resolution).floor();
        let cy = ((y - self.origin_y) / self.config.resolution).floor();

        if cx >= 0.0 && cy >= 0.0 {
            let cx = cx as usize;
            let cy = cy as usize;
            if cx < self.width && cy < self.height {
                return Some((cx, cy));
            }
        }
        None
    }

    /// Convert world coordinates to cell indices, signed.
    ///
    /// Used for ray tracing where we need to handle cells outside bounds.
    #[inline]
    pub fn world_to_cell_signed(&self, x: f32, y: f32) -> (i32, i32) {
        let cx = ((x - self.origin_x) / self.config.resolution).floor() as i32;
        let cy = ((y - self.origin_y) / self.config.resolution).floor() as i32;
        (cx, cy)
    }

    /// Convert cell indices to world coordinates (center of cell).
    #[inline]
    pub fn cell_to_world(&self, cx: usize, cy: usize) -> (f32, f32) {
        let x = self.origin_x + (cx as f32 + 0.5) * self.config.resolution;
        let y = self.origin_y + (cy as f32 + 0.5) * self.config.resolution;
        (x, y)
    }

    /// Check if cell indices are valid.
    #[inline]
    pub fn is_valid_cell(&self, cx: i32, cy: i32) -> bool {
        cx >= 0 && cy >= 0 && (cx as usize) < self.width && (cy as usize) < self.height
    }

    /// Get the cell index for array access.
    #[inline]
    fn cell_index(&self, cx: usize, cy: usize) -> usize {
        cy * self.width + cx
    }

    /// Get log-odds value at cell.
    ///
    /// Returns 0.0 (unknown) for out-of-bounds cells.
    #[inline]
    pub fn get_log_odds(&self, cx: usize, cy: usize) -> f32 {
        if cx < self.width && cy < self.height {
            self.cells[self.cell_index(cx, cy)]
        } else {
            0.0
        }
    }

    /// Get log-odds value at signed cell indices.
    #[inline]
    pub fn get_log_odds_signed(&self, cx: i32, cy: i32) -> f32 {
        if self.is_valid_cell(cx, cy) {
            self.cells[self.cell_index(cx as usize, cy as usize)]
        } else {
            0.0
        }
    }

    /// Get cell state (for visualization).
    pub fn get_state(&self, cx: usize, cy: usize) -> CellState {
        let log_odds = self.get_log_odds(cx, cy);

        if log_odds >= self.config.occupied_threshold {
            CellState::Occupied
        } else if log_odds <= self.config.free_threshold {
            CellState::Free
        } else {
            CellState::Unknown
        }
    }

    /// Get occupancy probability (0.0 to 1.0).
    pub fn get_probability(&self, cx: usize, cy: usize) -> f32 {
        let log_odds = self.get_log_odds(cx, cy);
        1.0 / (1.0 + (-log_odds).exp())
    }

    /// Update a cell with an observation.
    ///
    /// If `occupied` is true, adds `log_odds_occupied`.
    /// If false, adds `log_odds_free`.
    #[inline]
    pub fn update_cell(&mut self, cx: usize, cy: usize, occupied: bool) {
        if cx >= self.width || cy >= self.height {
            return;
        }

        let idx = self.cell_index(cx, cy);
        let delta = if occupied {
            self.config.log_odds_occupied
        } else {
            self.config.log_odds_free
        };

        self.cells[idx] =
            (self.cells[idx] + delta).clamp(self.config.log_odds_min, self.config.log_odds_max);

        // Track updated region
        let cx_i = cx as i32;
        let cy_i = cy as i32;
        match &mut self.updated_region {
            Some(region) => region.expand_to_include(cx_i, cy_i),
            None => self.updated_region = Some(MapRegion::new(cx_i, cy_i, cx_i, cy_i)),
        }
    }

    /// Update a cell at signed indices.
    #[inline]
    pub fn update_cell_signed(&mut self, cx: i32, cy: i32, occupied: bool) {
        if self.is_valid_cell(cx, cy) {
            self.update_cell(cx as usize, cy as usize, occupied);
        }
    }

    /// Get and clear the updated region since last call.
    pub fn take_updated_region(&mut self) -> Option<MapRegion> {
        self.updated_region.take()
    }

    /// Ensure the grid can contain the given world point.
    ///
    /// Grows the grid if necessary, preserving existing data.
    pub fn ensure_contains(&mut self, x: f32, y: f32) {
        let (cx, cy) = self.world_to_cell_signed(x, y);

        let mut needs_resize = false;
        let mut new_origin_x = self.origin_x;
        let mut new_origin_y = self.origin_y;
        let mut new_width = self.width;
        let mut new_height = self.height;

        // Check if we need to expand in negative X direction
        if cx < 0 {
            let expand = (-cx) as usize + 1;
            new_origin_x -= expand as f32 * self.config.resolution;
            new_width += expand;
            needs_resize = true;
        }

        // Check if we need to expand in positive X direction
        if cx >= self.width as i32 {
            new_width = (cx as usize) + 1;
            needs_resize = true;
        }

        // Check if we need to expand in negative Y direction
        if cy < 0 {
            let expand = (-cy) as usize + 1;
            new_origin_y -= expand as f32 * self.config.resolution;
            new_height += expand;
            needs_resize = true;
        }

        // Check if we need to expand in positive Y direction
        if cy >= self.height as i32 {
            new_height = (cy as usize) + 1;
            needs_resize = true;
        }

        if needs_resize {
            self.resize(new_width, new_height, new_origin_x, new_origin_y);
        }
    }

    /// Resize the grid, preserving existing data.
    fn resize(
        &mut self,
        new_width: usize,
        new_height: usize,
        new_origin_x: f32,
        new_origin_y: f32,
    ) {
        let mut new_cells = vec![0.0f32; new_width * new_height];

        // Calculate offset in cell coordinates
        let dx = ((self.origin_x - new_origin_x) / self.config.resolution).round() as i32;
        let dy = ((self.origin_y - new_origin_y) / self.config.resolution).round() as i32;

        // Copy old data to new grid
        for old_y in 0..self.height {
            for old_x in 0..self.width {
                let new_x = old_x as i32 + dx;
                let new_y = old_y as i32 + dy;

                if new_x >= 0
                    && new_y >= 0
                    && (new_x as usize) < new_width
                    && (new_y as usize) < new_height
                {
                    let old_idx = old_y * self.width + old_x;
                    let new_idx = (new_y as usize) * new_width + (new_x as usize);
                    new_cells[new_idx] = self.cells[old_idx];
                }
            }
        }

        self.cells = new_cells;
        self.width = new_width;
        self.height = new_height;
        self.origin_x = new_origin_x;
        self.origin_y = new_origin_y;
    }

    /// Clear the map (reset all cells to unknown).
    pub fn clear(&mut self) {
        self.cells.fill(0.0);
        self.updated_region = None;
    }

    /// Get metadata for serialization.
    pub fn metadata(&self) -> MapMetadata {
        MapMetadata {
            resolution: self.config.resolution,
            width: self.width,
            height: self.height,
            origin_x: self.origin_x,
            origin_y: self.origin_y,
        }
    }

    /// Save map to a binary file.
    pub fn save<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        serialization::save(self, path)
    }

    /// Load map from a binary file.
    pub fn load<P: AsRef<Path>>(path: P, config: OccupancyGridConfig) -> std::io::Result<Self> {
        serialization::load(path, config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_world_to_cell_conversion() {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 10.0,
            initial_height: 10.0,
            ..Default::default()
        };
        let grid = OccupancyGrid::new(config);

        // Origin is at center, so (0, 0) should map to center of grid
        let (cx, cy) = grid.world_to_cell(0.0, 0.0).unwrap();
        assert_eq!(cx, 50); // 5m / 0.1m = 50
        assert_eq!(cy, 50);

        // Test conversion back
        let (wx, wy) = grid.cell_to_world(cx, cy);
        assert_relative_eq!(wx, 0.05, epsilon = 0.01); // Center of cell
        assert_relative_eq!(wy, 0.05, epsilon = 0.01);
    }

    #[test]
    fn test_update_cell() {
        let config = OccupancyGridConfig::default();
        let mut grid = OccupancyGrid::new(config);

        let (cx, cy) = grid.world_to_cell(0.0, 0.0).unwrap();

        // Initially unknown
        assert_eq!(grid.get_log_odds(cx, cy), 0.0);
        assert_eq!(grid.get_state(cx, cy), CellState::Unknown);

        // Update as occupied multiple times
        for _ in 0..5 {
            grid.update_cell(cx, cy, true);
        }

        assert!(grid.get_log_odds(cx, cy) > 0.0);
        assert_eq!(grid.get_state(cx, cy), CellState::Occupied);
    }

    #[test]
    fn test_update_cell_free() {
        let config = OccupancyGridConfig::default();
        let mut grid = OccupancyGrid::new(config);

        let (cx, cy) = grid.world_to_cell(0.0, 0.0).unwrap();

        // Update as free multiple times
        for _ in 0..5 {
            grid.update_cell(cx, cy, false);
        }

        assert!(grid.get_log_odds(cx, cy) < 0.0);
        assert_eq!(grid.get_state(cx, cy), CellState::Free);
    }

    #[test]
    fn test_log_odds_clamping() {
        let config = OccupancyGridConfig {
            log_odds_max: 10.0,
            log_odds_min: -10.0,
            log_odds_occupied: 5.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        let (cx, cy) = grid.world_to_cell(0.0, 0.0).unwrap();

        // Update many times - should clamp at max
        for _ in 0..100 {
            grid.update_cell(cx, cy, true);
        }

        assert_eq!(grid.get_log_odds(cx, cy), 10.0);
    }

    #[test]
    fn test_probability_conversion() {
        let config = OccupancyGridConfig::default();
        let mut grid = OccupancyGrid::new(config);

        let (cx, cy) = grid.world_to_cell(0.0, 0.0).unwrap();

        // Unknown = 50% probability
        assert_relative_eq!(grid.get_probability(cx, cy), 0.5, epsilon = 0.01);

        // Make occupied
        for _ in 0..10 {
            grid.update_cell(cx, cy, true);
        }
        assert!(grid.get_probability(cx, cy) > 0.9);
    }

    #[test]
    fn test_grid_resize() {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 2.0,
            initial_height: 2.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        // Mark a cell as occupied
        let (cx, cy) = grid.world_to_cell(0.0, 0.0).unwrap();
        for _ in 0..5 {
            grid.update_cell(cx, cy, true);
        }
        let original_value = grid.get_log_odds(cx, cy);

        // Try to access a point outside the grid
        grid.ensure_contains(5.0, 5.0);

        // Original cell should still have same value
        let (new_cx, new_cy) = grid.world_to_cell(0.0, 0.0).unwrap();
        assert_relative_eq!(
            grid.get_log_odds(new_cx, new_cy),
            original_value,
            epsilon = 0.001
        );

        // New point should be accessible
        assert!(grid.world_to_cell(5.0, 5.0).is_some());
    }

    #[test]
    fn test_clear() {
        let config = OccupancyGridConfig::default();
        let mut grid = OccupancyGrid::new(config);

        // Update some cells
        grid.update_cell(10, 10, true);
        grid.update_cell(20, 20, false);

        assert!(grid.get_log_odds(10, 10) != 0.0);

        grid.clear();

        assert_eq!(grid.get_log_odds(10, 10), 0.0);
        assert_eq!(grid.get_log_odds(20, 20), 0.0);
    }

    #[test]
    fn test_updated_region() {
        let config = OccupancyGridConfig::default();
        let mut grid = OccupancyGrid::new(config);

        grid.update_cell(10, 15, true);
        grid.update_cell(20, 25, true);
        grid.update_cell(5, 8, false);

        let region = grid.take_updated_region().unwrap();

        assert_eq!(region.min_x, 5);
        assert_eq!(region.min_y, 8);
        assert_eq!(region.max_x, 20);
        assert_eq!(region.max_y, 25);

        // Should be cleared
        assert!(grid.take_updated_region().is_none());
    }

    #[test]
    fn test_to_grayscale() {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 1.0,
            initial_height: 1.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        // Make one cell occupied, one free
        grid.update_cell(0, 0, true);
        grid.update_cell(0, 0, true);
        grid.update_cell(0, 0, true);

        grid.update_cell(1, 1, false);
        grid.update_cell(1, 1, false);
        grid.update_cell(1, 1, false);

        let (w, h, pixels) = grid.to_grayscale();
        assert_eq!(w, 10);
        assert_eq!(h, 10);
        assert_eq!(pixels.len(), 100);

        // Occupied cell should be dark (0)
        assert_eq!(pixels[0], 0);

        // Free cell should be white (255)
        assert_eq!(pixels[11], 255);
    }

    #[test]
    fn test_count_cells() {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 1.0,
            initial_height: 1.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        // Initially all unknown
        let (free, unknown, occupied) = grid.count_cells();
        assert_eq!(free, 0);
        assert_eq!(occupied, 0);
        assert_eq!(unknown, 100);

        // Mark some cells
        for _ in 0..5 {
            grid.update_cell(0, 0, true);
            grid.update_cell(1, 0, false);
        }

        let (free, unknown, occupied) = grid.count_cells();
        assert_eq!(occupied, 1);
        assert_eq!(free, 1);
        assert_eq!(unknown, 98);
    }

    #[test]
    fn test_save_and_load() {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 2.0,
            initial_height: 2.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config.clone());

        // Add some data
        for _ in 0..5 {
            grid.update_cell(5, 5, true);
            grid.update_cell(10, 10, false);
        }

        let original_occupied = grid.get_log_odds(5, 5);
        let original_free = grid.get_log_odds(10, 10);

        // Save to temp file
        let temp_path = std::env::temp_dir().join("test_map.dsm");
        grid.save(&temp_path).unwrap();

        // Load
        let loaded = OccupancyGrid::load(&temp_path, config).unwrap();

        assert_eq!(loaded.dimensions(), grid.dimensions());
        assert_relative_eq!(
            loaded.get_log_odds(5, 5),
            original_occupied,
            epsilon = 0.001
        );
        assert_relative_eq!(loaded.get_log_odds(10, 10), original_free, epsilon = 0.001);

        // Clean up
        std::fs::remove_file(temp_path).ok();
    }

    #[test]
    fn test_known_wall_at_correct_position() {
        let config = OccupancyGridConfig {
            resolution: 0.05, // 5cm cells
            initial_width: 10.0,
            initial_height: 10.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        let wall_x = 2.0;
        let wall_y = 1.5;

        if let Some((cx, cy)) = grid.world_to_cell(wall_x, wall_y) {
            for _ in 0..5 {
                grid.update_cell(cx, cy, true);
            }

            let (recovered_x, recovered_y) = grid.cell_to_world(cx, cy);

            let error_x = (recovered_x - wall_x).abs();
            let error_y = (recovered_y - wall_y).abs();

            assert!(error_x < grid.resolution());
            assert!(error_y < grid.resolution());
            assert_eq!(grid.get_state(cx, cy), CellState::Occupied);
        }
    }

    #[test]
    fn test_probability_always_bounded() {
        let config = OccupancyGridConfig {
            log_odds_max: 100.0,
            log_odds_min: -100.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        let (cx, cy) = grid.world_to_cell(0.0, 0.0).unwrap();

        for _ in 0..1000 {
            grid.update_cell(cx, cy, true);
        }
        let prob_occupied = grid.get_probability(cx, cy);
        assert!(prob_occupied >= 0.0 && prob_occupied <= 1.0);
        assert!(prob_occupied > 0.99);

        grid.clear();
        for _ in 0..1000 {
            grid.update_cell(cx, cy, false);
        }
        let prob_free = grid.get_probability(cx, cy);
        assert!(prob_free >= 0.0 && prob_free <= 1.0);
        assert!(prob_free < 0.01);
    }

    #[test]
    fn test_log_odds_numerical_stability() {
        let config = OccupancyGridConfig::default();
        let mut grid = OccupancyGrid::new(config);

        let (cx, cy) = grid.world_to_cell(0.0, 0.0).unwrap();

        for i in 0..10000 {
            grid.update_cell(cx, cy, i % 2 == 0);
        }

        let log_odds = grid.get_log_odds(cx, cy);
        let prob = grid.get_probability(cx, cy);

        assert!(log_odds.is_finite());
        assert!(prob.is_finite());
        assert!(!log_odds.is_nan());
        assert!(!prob.is_nan());
    }

    #[test]
    fn test_grid_resize_preserves_accuracy() {
        let config = OccupancyGridConfig {
            resolution: 0.05,
            initial_width: 2.0,
            initial_height: 2.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        let reference_x = 0.5;
        let reference_y = 0.5;
        let (orig_cx, orig_cy) = grid.world_to_cell(reference_x, reference_y).unwrap();

        for _ in 0..10 {
            grid.update_cell(orig_cx, orig_cy, true);
        }
        let original_log_odds = grid.get_log_odds(orig_cx, orig_cy);
        assert!(original_log_odds > 1.0);

        grid.ensure_contains(3.0, 3.0);

        let (new_cx, new_cy) = grid.world_to_cell(reference_x, reference_y).unwrap();
        let new_log_odds = grid.get_log_odds(new_cx, new_cy);

        assert_relative_eq!(new_log_odds, original_log_odds, epsilon = 0.001);

        let (recovered_x, recovered_y) = grid.cell_to_world(new_cx, new_cy);
        let error_x = (recovered_x - reference_x).abs();
        let error_y = (recovered_y - reference_y).abs();

        assert!(error_x < grid.resolution());
        assert!(error_y < grid.resolution());
    }

    #[test]
    fn test_cell_state_thresholds() {
        let config = OccupancyGridConfig {
            occupied_threshold: 0.5,
            free_threshold: -0.5,
            log_odds_occupied: 0.2,
            log_odds_free: -0.2,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        let (cx, cy) = grid.world_to_cell(0.0, 0.0).unwrap();

        assert_eq!(grid.get_state(cx, cy), CellState::Unknown);

        grid.update_cell(cx, cy, true);
        grid.update_cell(cx, cy, true);
        grid.update_cell(cx, cy, true);
        assert_eq!(grid.get_state(cx, cy), CellState::Occupied);

        grid.clear();
        grid.update_cell(cx, cy, false);
        grid.update_cell(cx, cy, false);
        grid.update_cell(cx, cy, false);
        assert_eq!(grid.get_state(cx, cy), CellState::Free);
    }

    #[test]
    fn test_coordinate_conversion_round_trip() {
        let config = OccupancyGridConfig {
            resolution: 0.05,
            initial_width: 20.0,
            initial_height: 20.0,
            ..Default::default()
        };
        let grid = OccupancyGrid::new(config);

        let test_points = [
            (0.0, 0.0),
            (1.234, 2.567),
            (-3.14, 1.59),
            (5.0, -5.0),
            (-7.5, 7.5),
        ];

        for (wx, wy) in test_points {
            if let Some((cx, cy)) = grid.world_to_cell(wx, wy) {
                let (rx, ry) = grid.cell_to_world(cx, cy);

                let error_x = (rx - wx).abs();
                let error_y = (ry - wy).abs();

                assert!(error_x < grid.resolution());
                assert!(error_y < grid.resolution());
            }
        }
    }
}
