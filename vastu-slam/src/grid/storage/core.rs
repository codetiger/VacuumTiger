//! Grid storage core implementation.
//!
//! Uses Structure-of-Arrays (SoA) layout for SIMD optimization.

use std::simd::{cmp::SimdPartialEq, u8x16};

use crate::core::{Cell, CellType, GridCoord, WorldPoint};
use crate::grid::LogOddsConfig;

use super::types::{CellCounts, CellMut};

/// Grid storage using Structure-of-Arrays (SoA) layout.
///
/// The grid uses a coordinate system where:
/// - (0, 0) is at `origin` in world coordinates
/// - Positive X is to the right
/// - Positive Y is up
/// - Cell (x, y) covers the area from (origin + x*resolution) to (origin + (x+1)*resolution)
///
/// ## Memory Layout
///
/// Each cell field is stored in a separate contiguous array:
/// ```text
/// cell_types:         [T T T T T T T T T T T T T T T T|...]
/// confidences:        [C C C C C C C C C C C C C C C C|...]
/// observation_counts: [O O O O O O O O O O O O O O O O|...]
/// swept_flags:        [S S S S S S S S S S S S S S S S|...]
/// log_odds:           [L L L L L L L L L L L L L L L L|...]
/// ```
///
/// This enables SIMD operations to process 16 cells at a time (u8x16).
#[derive(Clone, Debug)]
pub struct GridStorage {
    // === SoA Data Arrays ===
    pub(super) cell_types: Vec<u8>,
    pub(super) confidences: Vec<u8>,
    pub(super) observation_counts: Vec<u8>,
    pub(super) swept_flags: Vec<u8>,
    pub(super) log_odds: Vec<i16>,
    pub(super) distance_field: Vec<f32>,

    // === Grid Metadata ===
    pub(super) width: usize,
    pub(super) height: usize,
    pub(super) resolution: f32,
    /// Pre-computed 1.0 / resolution for faster world-to-grid conversion.
    pub(super) inv_resolution: f32,
    pub(super) origin: WorldPoint,
    pub(super) max_distance: f32,
    pub(super) log_odds_config: LogOddsConfig,
}

impl GridStorage {
    /// Default max distance for distance field propagation (meters).
    pub(super) const DEFAULT_MAX_DISTANCE: f32 = 1.0;

    // === Deprecated Log-Odds Constants ===

    /// Log-odds for a "hit" observation (legacy constant, use config instead).
    #[deprecated(note = "Use log_odds_config().l_hit instead")]
    pub const L_HIT: i16 = 70;

    /// Log-odds for a "miss" observation (legacy constant, use config instead).
    #[deprecated(note = "Use log_odds_config().l_miss instead")]
    pub const L_MISS: i16 = -28;

    /// Minimum log-odds (legacy constant, use config instead).
    #[deprecated(note = "Use log_odds_config().l_min instead")]
    pub const L_MIN: i16 = -200;

    /// Maximum log-odds (legacy constant, use config instead).
    #[deprecated(note = "Use log_odds_config().l_max instead")]
    pub const L_MAX: i16 = 200;

    /// Log-odds threshold for occupied (legacy constant, use config instead).
    #[deprecated(note = "Use log_odds_config().l_occupied_threshold instead")]
    pub const L_OCCUPIED_THRESHOLD: i16 = 50;

    /// Log-odds threshold for free (legacy constant, use config instead).
    #[deprecated(note = "Use log_odds_config().l_free_threshold instead")]
    pub const L_FREE_THRESHOLD: i16 = -50;

    // === Constructors ===

    /// Create a new grid with the given dimensions and default log-odds config.
    pub fn new(width: usize, height: usize, resolution: f32, origin: WorldPoint) -> Self {
        Self::with_config(width, height, resolution, origin, LogOddsConfig::default())
    }

    /// Create a new grid with the given dimensions and custom log-odds config.
    pub fn with_config(
        width: usize,
        height: usize,
        resolution: f32,
        origin: WorldPoint,
        log_odds_config: LogOddsConfig,
    ) -> Self {
        let size = width * height;
        Self {
            cell_types: vec![CellType::Unknown as u8; size],
            confidences: vec![0; size],
            observation_counts: vec![0; size],
            swept_flags: vec![0; size],
            log_odds: vec![0; size],
            distance_field: vec![f32::MAX; size],
            width,
            height,
            resolution,
            inv_resolution: 1.0 / resolution,
            origin,
            max_distance: Self::DEFAULT_MAX_DISTANCE,
            log_odds_config,
        }
    }

    /// Create a grid centered at the origin with default config.
    pub fn centered(width: usize, height: usize, resolution: f32) -> Self {
        Self::centered_with_config(width, height, resolution, LogOddsConfig::default())
    }

    /// Create a grid centered at the origin with custom log-odds config.
    pub fn centered_with_config(
        width: usize,
        height: usize,
        resolution: f32,
        log_odds_config: LogOddsConfig,
    ) -> Self {
        let half_width = (width as f32 * resolution) / 2.0;
        let half_height = (height as f32 * resolution) / 2.0;
        let origin = WorldPoint::new(-half_width, -half_height);
        Self::with_config(width, height, resolution, origin, log_odds_config)
    }

    // === Basic Properties ===

    /// Get the current log-odds configuration.
    #[inline]
    pub fn log_odds_config(&self) -> &LogOddsConfig {
        &self.log_odds_config
    }

    /// Set the log-odds configuration.
    pub fn set_log_odds_config(&mut self, config: LogOddsConfig) {
        self.log_odds_config = config;
    }

    /// Grid width in cells.
    #[inline]
    pub fn width(&self) -> usize {
        self.width
    }

    /// Grid height in cells.
    #[inline]
    pub fn height(&self) -> usize {
        self.height
    }

    /// Resolution in meters per cell.
    #[inline]
    pub fn resolution(&self) -> f32 {
        self.resolution
    }

    /// World coordinates of cell (0, 0).
    #[inline]
    pub fn origin(&self) -> WorldPoint {
        self.origin
    }

    /// Total number of cells.
    #[inline]
    pub fn cell_count(&self) -> usize {
        self.width * self.height
    }

    /// World bounds: (min_point, max_point).
    pub fn bounds(&self) -> (WorldPoint, WorldPoint) {
        let min = self.origin;
        let max = WorldPoint::new(
            self.origin.x + self.width as f32 * self.resolution,
            self.origin.y + self.height as f32 * self.resolution,
        );
        (min, max)
    }

    // === Coordinate Conversion ===

    /// Convert world coordinates to grid coordinates.
    /// Uses pre-computed inverse resolution for faster computation.
    #[inline]
    pub fn world_to_grid(&self, point: WorldPoint) -> GridCoord {
        let x = ((point.x - self.origin.x) * self.inv_resolution).floor() as i32;
        let y = ((point.y - self.origin.y) * self.inv_resolution).floor() as i32;
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

    /// Check if grid coordinates are within bounds.
    #[inline]
    pub fn is_valid_coord(&self, coord: GridCoord) -> bool {
        coord.x >= 0
            && coord.y >= 0
            && (coord.x as usize) < self.width
            && (coord.y as usize) < self.height
    }

    /// Check if world point is within grid bounds.
    #[inline]
    pub fn contains_point(&self, point: WorldPoint) -> bool {
        let (min, max) = self.bounds();
        point.x >= min.x && point.x < max.x && point.y >= min.y && point.y < max.y
    }

    /// Convert grid coordinates to flat array index.
    #[inline]
    pub fn coord_to_index(&self, coord: GridCoord) -> Option<usize> {
        if self.is_valid_coord(coord) {
            Some(coord.y as usize * self.width + coord.x as usize)
        } else {
            None
        }
    }

    /// Convert flat array index to grid coordinates.
    #[inline]
    pub fn index_to_coord(&self, index: usize) -> GridCoord {
        GridCoord::new((index % self.width) as i32, (index / self.width) as i32)
    }

    // === Cell Access ===

    /// Get cell at grid coordinates (reconstructs Cell from SoA data).
    #[inline]
    pub fn get(&self, coord: GridCoord) -> Option<Cell> {
        self.coord_to_index(coord).map(|i| Cell {
            cell_type: CellType::from_u8(self.cell_types[i]),
            confidence: self.confidences[i],
            observation_count: self.observation_counts[i],
            swept: self.swept_flags[i] != 0,
        })
    }

    /// Get mutable cell at grid coordinates.
    #[inline]
    pub fn get_mut(&mut self, coord: GridCoord) -> Option<CellMut<'_>> {
        self.coord_to_index(coord).map(move |i| CellMut {
            cell_type: &mut self.cell_types[i],
            confidence: &mut self.confidences[i],
            observation_count: &mut self.observation_counts[i],
            swept: &mut self.swept_flags[i],
        })
    }

    /// Get cell at world coordinates.
    #[inline]
    pub fn get_world(&self, point: WorldPoint) -> Option<Cell> {
        self.get(self.world_to_grid(point))
    }

    /// Get mutable cell at world coordinates.
    #[inline]
    pub fn get_world_mut(&mut self, point: WorldPoint) -> Option<CellMut<'_>> {
        let coord = self.world_to_grid(point);
        self.get_mut(coord)
    }

    /// Get cell type at grid coordinates (returns Unknown if out of bounds).
    #[inline]
    pub fn get_type(&self, coord: GridCoord) -> CellType {
        self.coord_to_index(coord)
            .map(|i| CellType::from_u8(self.cell_types[i]))
            .unwrap_or(CellType::Unknown)
    }

    /// Get cell type at world coordinates.
    #[inline]
    pub fn get_type_world(&self, point: WorldPoint) -> CellType {
        self.get_type(self.world_to_grid(point))
    }

    /// Set cell type at grid coordinates.
    #[inline]
    pub fn set_type(&mut self, coord: GridCoord, cell_type: CellType) -> bool {
        if let Some(i) = self.coord_to_index(coord) {
            self.observe_at_index(i, cell_type)
        } else {
            false
        }
    }

    /// Set cell type with priority rules.
    #[inline]
    pub fn set_type_with_priority(&mut self, coord: GridCoord, cell_type: CellType) -> bool {
        if let Some(i) = self.coord_to_index(coord) {
            self.observe_with_priority_at_index(i, cell_type)
        } else {
            false
        }
    }

    /// Observe a cell type at a given index (internal helper).
    #[inline]
    pub(super) fn observe_at_index(&mut self, i: usize, cell_type: CellType) -> bool {
        let old_type = self.cell_types[i];
        let new_type = cell_type as u8;
        let changed = old_type != new_type;

        if changed {
            self.cell_types[i] = new_type;
            self.confidences[i] = 1;
            self.observation_counts[i] = 1;
        } else {
            self.confidences[i] = self.confidences[i].saturating_add(1);
            self.observation_counts[i] = self.observation_counts[i].saturating_add(1);
        }

        changed
    }

    /// Observe a cell type with priority at a given index (internal helper).
    #[inline]
    pub(super) fn observe_with_priority_at_index(&mut self, i: usize, cell_type: CellType) -> bool {
        let old_type = CellType::from_u8(self.cell_types[i]);
        let should_update = match (old_type, cell_type) {
            (CellType::Unknown, _) => true,
            (CellType::Bump, CellType::Bump) => true,
            (CellType::Bump, _) => false,
            (CellType::Cliff, CellType::Bump) => true,
            (CellType::Cliff, CellType::Cliff) => true,
            (CellType::Cliff, _) => false,
            (CellType::Wall, CellType::Bump) => true,
            (CellType::Wall, CellType::Cliff) => true,
            (CellType::Wall, CellType::Wall) => true,
            (CellType::Wall, _) => false,
            (CellType::Floor, _) => true,
        };

        if should_update {
            self.observe_at_index(i, cell_type)
        } else {
            self.observation_counts[i] = self.observation_counts[i].saturating_add(1);
            false
        }
    }

    /// Clear all cells to Unknown.
    pub fn clear(&mut self) {
        self.cell_types.fill(CellType::Unknown as u8);
        self.confidences.fill(0);
        self.observation_counts.fill(0);
        self.swept_flags.fill(0);
        self.log_odds.fill(0);
        self.distance_field.fill(f32::MAX);
    }

    // === Iterators ===

    /// Iterate over all cells with their coordinates.
    pub fn iter(&self) -> impl Iterator<Item = (GridCoord, Cell)> + '_ {
        (0..self.cell_types.len()).map(move |i| {
            let x = (i % self.width) as i32;
            let y = (i / self.width) as i32;
            (
                GridCoord::new(x, y),
                Cell {
                    cell_type: CellType::from_u8(self.cell_types[i]),
                    confidence: self.confidences[i],
                    observation_count: self.observation_counts[i],
                    swept: self.swept_flags[i] != 0,
                },
            )
        })
    }

    /// Iterate over all cells mutably with their coordinates.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (GridCoord, CellMut<'_>)> {
        let width = self.width;
        let types_ptr = self.cell_types.as_mut_ptr();
        let conf_ptr = self.confidences.as_mut_ptr();
        let obs_ptr = self.observation_counts.as_mut_ptr();
        let swept_ptr = self.swept_flags.as_mut_ptr();
        let len = self.cell_types.len();

        (0..len).map(move |i| {
            let x = (i % width) as i32;
            let y = (i / width) as i32;
            // SAFETY: We iterate exactly once over each index in range
            unsafe {
                (
                    GridCoord::new(x, y),
                    CellMut {
                        cell_type: &mut *types_ptr.add(i),
                        confidence: &mut *conf_ptr.add(i),
                        observation_count: &mut *obs_ptr.add(i),
                        swept: &mut *swept_ptr.add(i),
                    },
                )
            }
        })
    }

    // === Raw Slice Access ===

    /// Raw access to cell types array (for SIMD operations).
    #[inline]
    pub fn cell_types_raw(&self) -> &[u8] {
        &self.cell_types
    }

    /// Get raw cell types slice.
    #[inline]
    pub fn cell_types(&self) -> &[u8] {
        &self.cell_types
    }

    /// Get mutable raw cell types slice.
    #[inline]
    pub fn cell_types_mut(&mut self) -> &mut [u8] {
        &mut self.cell_types
    }

    /// Get raw confidences slice.
    #[inline]
    pub fn confidences(&self) -> &[u8] {
        &self.confidences
    }

    /// Get raw observation counts slice.
    #[inline]
    pub fn observation_counts(&self) -> &[u8] {
        &self.observation_counts
    }

    /// Get raw swept flags slice.
    #[inline]
    pub fn swept_flags(&self) -> &[u8] {
        &self.swept_flags
    }

    /// Get mutable swept flags slice.
    #[inline]
    pub fn swept_flags_mut(&mut self) -> &mut [u8] {
        &mut self.swept_flags
    }

    /// Get raw cells slice (for serialization compatibility).
    pub fn cells(&self) -> Vec<Cell> {
        (0..self.cell_types.len())
            .map(|i| Cell {
                cell_type: CellType::from_u8(self.cell_types[i]),
                confidence: self.confidences[i],
                observation_count: self.observation_counts[i],
                swept: self.swept_flags[i] != 0,
            })
            .collect()
    }

    // === Log-Odds Methods ===

    /// Get raw log-odds slice.
    #[inline]
    pub fn log_odds(&self) -> &[i16] {
        &self.log_odds
    }

    /// Get mutable log-odds slice.
    #[inline]
    pub fn log_odds_mut(&mut self) -> &mut [i16] {
        &mut self.log_odds
    }

    /// Get log-odds value at grid coordinates.
    #[inline]
    pub fn get_log_odds(&self, coord: GridCoord) -> i16 {
        self.coord_to_index(coord)
            .map(|i| self.log_odds[i])
            .unwrap_or(0)
    }

    /// Get occupancy probability at grid coordinates.
    #[inline]
    pub fn get_probability(&self, coord: GridCoord) -> f32 {
        let log_odds = self.get_log_odds(coord);
        Self::log_odds_to_probability(log_odds)
    }

    /// Convert log-odds (fixed-point i16) to probability.
    #[inline]
    pub fn log_odds_to_probability(log_odds: i16) -> f32 {
        let l = log_odds as f32 / 100.0;
        1.0 / (1.0 + (-l).exp())
    }

    /// Convert probability to log-odds (fixed-point i16).
    #[inline]
    pub fn probability_to_log_odds(p: f32) -> i16 {
        let p = p.clamp(0.01, 0.99);
        let l = (p / (1.0 - p)).ln();
        (l * 100.0).round() as i16
    }

    /// Check if a cell is considered occupied based on log-odds.
    #[inline]
    pub fn is_occupied(&self, coord: GridCoord) -> bool {
        self.get_log_odds(coord) > self.log_odds_config.l_occupied_threshold
    }

    /// Check if a cell is considered free based on log-odds.
    #[inline]
    pub fn is_free(&self, coord: GridCoord) -> bool {
        self.get_log_odds(coord) < self.log_odds_config.l_free_threshold
    }

    /// Apply a "hit" observation at grid coordinates.
    pub fn apply_hit(&mut self, coord: GridCoord) -> bool {
        if let Some(i) = self.coord_to_index(coord) {
            let cfg = &self.log_odds_config;
            let old = self.log_odds[i];
            let new = old.saturating_add(cfg.l_hit).clamp(cfg.l_min, cfg.l_max);
            self.log_odds[i] = new;
            self.observation_counts[i] = self.observation_counts[i].saturating_add(1);

            let became_occupied = old <= cfg.l_occupied_threshold && new > cfg.l_occupied_threshold;
            if new > cfg.l_occupied_threshold {
                self.cell_types[i] = CellType::Wall as u8;
                self.confidences[i] = ((new - cfg.l_occupied_threshold) / 2).min(255) as u8;
            }

            became_occupied
        } else {
            false
        }
    }

    /// Apply a "miss" observation at grid coordinates.
    pub fn apply_miss(&mut self, coord: GridCoord) -> bool {
        if let Some(i) = self.coord_to_index(coord) {
            let cfg = &self.log_odds_config;
            let old = self.log_odds[i];
            let new = old.saturating_add(cfg.l_miss).clamp(cfg.l_min, cfg.l_max);
            self.log_odds[i] = new;
            self.observation_counts[i] = self.observation_counts[i].saturating_add(1);

            let old_type = CellType::from_u8(self.cell_types[i]);
            let became_free = old >= cfg.l_free_threshold && new < cfg.l_free_threshold;

            if new < cfg.l_free_threshold && matches!(old_type, CellType::Unknown | CellType::Wall)
            {
                self.cell_types[i] = CellType::Floor as u8;
                self.confidences[i] = ((cfg.l_free_threshold - new).abs() / 2).min(255) as u8;
            }

            became_free
        } else {
            false
        }
    }

    /// Apply a hit observation with custom log-odds value.
    pub fn apply_hit_custom(&mut self, coord: GridCoord, l_hit: i16) -> bool {
        if let Some(i) = self.coord_to_index(coord) {
            let cfg = &self.log_odds_config;
            let old = self.log_odds[i];
            let new = old.saturating_add(l_hit).clamp(cfg.l_min, cfg.l_max);
            self.log_odds[i] = new;
            self.observation_counts[i] = self.observation_counts[i].saturating_add(1);

            let became_occupied = old <= cfg.l_occupied_threshold && new > cfg.l_occupied_threshold;
            if new > cfg.l_occupied_threshold {
                self.cell_types[i] = CellType::Wall as u8;
            }

            became_occupied
        } else {
            false
        }
    }

    /// Apply a miss observation with custom log-odds value.
    pub fn apply_miss_custom(&mut self, coord: GridCoord, l_miss: i16) -> bool {
        if let Some(i) = self.coord_to_index(coord) {
            let cfg = &self.log_odds_config;
            let old = self.log_odds[i];
            let new = old.saturating_add(l_miss).clamp(cfg.l_min, cfg.l_max);
            self.log_odds[i] = new;
            self.observation_counts[i] = self.observation_counts[i].saturating_add(1);

            let old_type = CellType::from_u8(self.cell_types[i]);
            let became_free = old >= cfg.l_free_threshold && new < cfg.l_free_threshold;

            if new < cfg.l_free_threshold && matches!(old_type, CellType::Unknown | CellType::Wall)
            {
                self.cell_types[i] = CellType::Floor as u8;
            }

            became_free
        } else {
            false
        }
    }

    // === Distance Field Methods ===

    /// Get distance to nearest wall at grid coordinates (in meters).
    #[inline]
    pub fn get_distance(&self, coord: GridCoord) -> f32 {
        self.coord_to_index(coord)
            .map(|i| self.distance_field[i])
            .unwrap_or(f32::MAX)
    }

    /// Get distance to nearest wall at world coordinates (in meters).
    #[inline]
    pub fn get_distance_world(&self, point: WorldPoint) -> f32 {
        self.get_distance(self.world_to_grid(point))
    }

    /// Get raw distance field slice.
    #[inline]
    pub fn distance_field(&self) -> &[f32] {
        &self.distance_field
    }

    /// Get distance to nearest wall with bilinear interpolation.
    #[inline]
    pub fn get_distance_interpolated(&self, point: WorldPoint) -> (f32, f32, f32) {
        let gx = (point.x - self.origin.x) / self.resolution;
        let gy = (point.y - self.origin.y) / self.resolution;

        let x0 = gx.floor() as i32;
        let y0 = gy.floor() as i32;

        let fx = gx - x0 as f32;
        let fy = gy - y0 as f32;

        let d00 = self.get_distance_safe(x0, y0);
        let d10 = self.get_distance_safe(x0 + 1, y0);
        let d01 = self.get_distance_safe(x0, y0 + 1);
        let d11 = self.get_distance_safe(x0 + 1, y0 + 1);

        let d0 = d00 * (1.0 - fx) + d10 * fx;
        let d1 = d01 * (1.0 - fx) + d11 * fx;
        let distance = d0 * (1.0 - fy) + d1 * fy;

        let grad_x = ((d10 - d00) * (1.0 - fy) + (d11 - d01) * fy) / self.resolution;
        let grad_y = ((d01 - d00) * (1.0 - fx) + (d11 - d10) * fx) / self.resolution;

        (distance, grad_x, grad_y)
    }

    /// Helper to get distance with safe bounds checking.
    #[inline]
    fn get_distance_safe(&self, x: i32, y: i32) -> f32 {
        if x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height {
            let idx = y as usize * self.width + x as usize;
            let d = self.distance_field[idx];
            if d >= f32::MAX / 2.0 {
                self.max_distance
            } else {
                d
            }
        } else {
            self.max_distance
        }
    }

    /// Update distance field around a newly added wall cell.
    pub fn update_distance_field_from_wall(&mut self, wall_coord: GridCoord) {
        use std::collections::VecDeque;

        if let Some(wall_idx) = self.coord_to_index(wall_coord) {
            self.distance_field[wall_idx] = 0.0;
        } else {
            return;
        }

        let mut queue = VecDeque::new();
        queue.push_back((wall_coord, 0.0f32));

        let neighbors_4 = [(0, 1), (1, 0), (0, -1), (-1, 0)];
        let neighbors_diag = [(1, 1), (1, -1), (-1, 1), (-1, -1)];
        let diag_dist = self.resolution * std::f32::consts::SQRT_2;

        while let Some((coord, dist)) = queue.pop_front() {
            for (dx, dy) in neighbors_4 {
                let neighbor = GridCoord::new(coord.x + dx, coord.y + dy);
                let new_dist = dist + self.resolution;

                if new_dist > self.max_distance {
                    continue;
                }

                if let Some(idx) = self.coord_to_index(neighbor)
                    && new_dist < self.distance_field[idx]
                {
                    self.distance_field[idx] = new_dist;
                    queue.push_back((neighbor, new_dist));
                }
            }

            for (dx, dy) in neighbors_diag {
                let neighbor = GridCoord::new(coord.x + dx, coord.y + dy);
                let new_dist = dist + diag_dist;

                if new_dist > self.max_distance {
                    continue;
                }

                if let Some(idx) = self.coord_to_index(neighbor)
                    && new_dist < self.distance_field[idx]
                {
                    self.distance_field[idx] = new_dist;
                    queue.push_back((neighbor, new_dist));
                }
            }
        }
    }

    /// Batch update distance field for multiple wall cells.
    pub fn update_distance_field_batch(&mut self, wall_coords: &[GridCoord]) {
        use std::collections::VecDeque;

        if wall_coords.is_empty() {
            return;
        }

        let mut queue = VecDeque::with_capacity(wall_coords.len() * 4);

        for &coord in wall_coords {
            if let Some(idx) = self.coord_to_index(coord) {
                self.distance_field[idx] = 0.0;
                queue.push_back((coord, 0.0f32));
            }
        }

        let neighbors_4 = [(0, 1), (1, 0), (0, -1), (-1, 0)];
        let neighbors_diag = [(1, 1), (1, -1), (-1, 1), (-1, -1)];
        let diag_dist = self.resolution * std::f32::consts::SQRT_2;

        while let Some((coord, dist)) = queue.pop_front() {
            for (dx, dy) in neighbors_4 {
                let neighbor = GridCoord::new(coord.x + dx, coord.y + dy);
                let new_dist = dist + self.resolution;

                if new_dist > self.max_distance {
                    continue;
                }

                if let Some(idx) = self.coord_to_index(neighbor)
                    && new_dist < self.distance_field[idx]
                {
                    self.distance_field[idx] = new_dist;
                    queue.push_back((neighbor, new_dist));
                }
            }

            for (dx, dy) in neighbors_diag {
                let neighbor = GridCoord::new(coord.x + dx, coord.y + dy);
                let new_dist = dist + diag_dist;

                if new_dist > self.max_distance {
                    continue;
                }

                if let Some(idx) = self.coord_to_index(neighbor)
                    && new_dist < self.distance_field[idx]
                {
                    self.distance_field[idx] = new_dist;
                    queue.push_back((neighbor, new_dist));
                }
            }
        }
    }

    /// Recompute entire distance field from scratch.
    pub fn recompute_distance_field(&mut self) {
        self.distance_field.fill(f32::MAX);

        let wall_coords: Vec<GridCoord> = (0..self.cell_types.len())
            .filter(|&i| self.cell_types[i] == CellType::Wall as u8)
            .map(|i| self.index_to_coord(i))
            .collect();

        self.update_distance_field_batch(&wall_coords);
    }

    // === SIMD Cell Counting ===

    /// Count cells by type (SIMD-optimized implementation).
    pub fn count_by_type(&self) -> CellCounts {
        self.count_by_type_simd()
    }

    /// Scalar implementation of cell counting (for testing).
    #[cfg(test)]
    pub fn count_by_type_scalar(&self) -> CellCounts {
        let mut counts = CellCounts::default();
        for &cell_type in &self.cell_types {
            match cell_type {
                0 => counts.unknown += 1,
                1 => counts.floor += 1,
                2 => counts.wall += 1,
                3 => counts.cliff += 1,
                4 => counts.bump += 1,
                _ => counts.unknown += 1,
            }
        }
        counts
    }

    /// SIMD-optimized cell counting.
    fn count_by_type_simd(&self) -> CellCounts {
        let mut counts = CellCounts::default();

        let unknown_vec = u8x16::splat(CellType::Unknown as u8);
        let floor_vec = u8x16::splat(CellType::Floor as u8);
        let wall_vec = u8x16::splat(CellType::Wall as u8);
        let cliff_vec = u8x16::splat(CellType::Cliff as u8);
        let bump_vec = u8x16::splat(CellType::Bump as u8);

        let chunks = self.cell_types.chunks_exact(16);
        let remainder = chunks.remainder();

        for chunk in chunks {
            let data = u8x16::from_slice(chunk);
            counts.unknown += data.simd_eq(unknown_vec).to_bitmask().count_ones() as usize;
            counts.floor += data.simd_eq(floor_vec).to_bitmask().count_ones() as usize;
            counts.wall += data.simd_eq(wall_vec).to_bitmask().count_ones() as usize;
            counts.cliff += data.simd_eq(cliff_vec).to_bitmask().count_ones() as usize;
            counts.bump += data.simd_eq(bump_vec).to_bitmask().count_ones() as usize;
        }

        for &cell_type in remainder {
            match cell_type {
                0 => counts.unknown += 1,
                1 => counts.floor += 1,
                2 => counts.wall += 1,
                3 => counts.cliff += 1,
                4 => counts.bump += 1,
                _ => counts.unknown += 1,
            }
        }

        counts
    }

    /// Check if all cells in a rectangular region are Floor (SIMD-optimized).
    pub fn is_rect_all_floor(
        &self,
        start_x: usize,
        start_y: usize,
        rect_width: usize,
        rect_height: usize,
    ) -> bool {
        let floor_vec = u8x16::splat(CellType::Floor as u8);

        for row in 0..rect_height {
            let y = start_y + row;
            if y >= self.height {
                return false;
            }

            let row_start = y * self.width + start_x;
            let row_end = (row_start + rect_width).min(y * self.width + self.width);

            if row_end > self.cell_types.len() {
                return false;
            }

            let row_slice = &self.cell_types[row_start..row_end];

            let chunks = row_slice.chunks_exact(16);
            let remainder = chunks.remainder();

            for chunk in chunks {
                let data = u8x16::from_slice(chunk);
                let is_floor = data.simd_eq(floor_vec);
                if is_floor.to_bitmask() != 0xFFFF {
                    return false;
                }
            }

            for &cell_type in remainder {
                if cell_type != CellType::Floor as u8 {
                    return false;
                }
            }
        }

        true
    }

    // === Grid Expansion ===

    /// Expand the grid to include a world point.
    pub fn expand_to_include(
        &mut self,
        point: WorldPoint,
        max_width: usize,
        max_height: usize,
    ) -> bool {
        let coord = self.world_to_grid(point);

        let expand_left = if coord.x < 0 { (-coord.x) as usize } else { 0 };
        let expand_right = if coord.x >= self.width as i32 {
            coord.x as usize - self.width + 1
        } else {
            0
        };
        let expand_down = if coord.y < 0 { (-coord.y) as usize } else { 0 };
        let expand_up = if coord.y >= self.height as i32 {
            coord.y as usize - self.height + 1
        } else {
            0
        };

        if expand_left == 0 && expand_right == 0 && expand_down == 0 && expand_up == 0 {
            return false;
        }

        let new_width = self.width + expand_left + expand_right;
        let new_height = self.height + expand_down + expand_up;

        if new_width > max_width || new_height > max_height {
            return false;
        }

        let new_size = new_width * new_height;

        let mut new_cell_types = vec![CellType::Unknown as u8; new_size];
        let mut new_confidences = vec![0u8; new_size];
        let mut new_observation_counts = vec![0u8; new_size];
        let mut new_swept_flags = vec![0u8; new_size];
        let mut new_log_odds = vec![0i16; new_size];
        let mut new_distance_field = vec![f32::MAX; new_size];

        for y in 0..self.height {
            let old_row_start = y * self.width;
            let new_row_start = (y + expand_down) * new_width + expand_left;

            new_cell_types[new_row_start..new_row_start + self.width]
                .copy_from_slice(&self.cell_types[old_row_start..old_row_start + self.width]);
            new_confidences[new_row_start..new_row_start + self.width]
                .copy_from_slice(&self.confidences[old_row_start..old_row_start + self.width]);
            new_observation_counts[new_row_start..new_row_start + self.width].copy_from_slice(
                &self.observation_counts[old_row_start..old_row_start + self.width],
            );
            new_swept_flags[new_row_start..new_row_start + self.width]
                .copy_from_slice(&self.swept_flags[old_row_start..old_row_start + self.width]);
            new_log_odds[new_row_start..new_row_start + self.width]
                .copy_from_slice(&self.log_odds[old_row_start..old_row_start + self.width]);
            new_distance_field[new_row_start..new_row_start + self.width]
                .copy_from_slice(&self.distance_field[old_row_start..old_row_start + self.width]);
        }

        self.cell_types = new_cell_types;
        self.confidences = new_confidences;
        self.observation_counts = new_observation_counts;
        self.swept_flags = new_swept_flags;
        self.log_odds = new_log_odds;
        self.distance_field = new_distance_field;
        self.width = new_width;
        self.height = new_height;
        self.origin = WorldPoint::new(
            self.origin.x - expand_left as f32 * self.resolution,
            self.origin.y - expand_down as f32 * self.resolution,
        );

        true
    }
}
