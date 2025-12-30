//! Grid storage implementation for the occupancy grid.
//!
//! Uses Structure-of-Arrays (SoA) layout for SIMD optimization.
//! Cell data is stored in separate arrays for each field, enabling
//! efficient vectorized operations on ARM NEON and x86 SSE/AVX.

use crate::core::{Cell, CellType, GridCoord, WorldPoint};
use std::simd::{cmp::SimdPartialEq, u8x16};

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
/// ```
///
/// This enables SIMD operations to process 16 cells at a time (u8x16).
#[derive(Clone, Debug)]
pub struct GridStorage {
    // === SoA Data Arrays ===
    /// Cell types (CellType as u8: Unknown=0, Floor=1, Wall=2, Cliff=3, Bump=4)
    cell_types: Vec<u8>,
    /// Confidence levels (0-255)
    confidences: Vec<u8>,
    /// Observation counts (0-255, saturating)
    observation_counts: Vec<u8>,
    /// Swept/cleaned flags (0 = not swept, 1 = swept)
    swept_flags: Vec<u8>,
    /// Distance to nearest wall in meters (for Gaussian scoring in scan matching)
    /// f32::MAX indicates no wall nearby (unknown areas)
    distance_field: Vec<f32>,

    // === Grid Metadata ===
    /// Grid width in cells
    width: usize,
    /// Grid height in cells
    height: usize,
    /// Resolution in meters per cell
    resolution: f32,
    /// World coordinates of cell (0, 0) center
    origin: WorldPoint,
    /// Maximum distance to propagate in distance field (meters)
    max_distance: f32,
}

impl GridStorage {
    /// Default max distance for distance field propagation (meters)
    const DEFAULT_MAX_DISTANCE: f32 = 1.0;

    /// Create a new grid with the given dimensions
    pub fn new(width: usize, height: usize, resolution: f32, origin: WorldPoint) -> Self {
        let size = width * height;
        Self {
            cell_types: vec![CellType::Unknown as u8; size],
            confidences: vec![0; size],
            observation_counts: vec![0; size],
            swept_flags: vec![0; size],
            distance_field: vec![f32::MAX; size],
            width,
            height,
            resolution,
            origin,
            max_distance: Self::DEFAULT_MAX_DISTANCE,
        }
    }

    /// Create a grid centered at the origin
    pub fn centered(width: usize, height: usize, resolution: f32) -> Self {
        let half_width = (width as f32 * resolution) / 2.0;
        let half_height = (height as f32 * resolution) / 2.0;
        let origin = WorldPoint::new(-half_width, -half_height);
        Self::new(width, height, resolution, origin)
    }

    /// Grid width in cells
    #[inline]
    pub fn width(&self) -> usize {
        self.width
    }

    /// Grid height in cells
    #[inline]
    pub fn height(&self) -> usize {
        self.height
    }

    /// Resolution in meters per cell
    #[inline]
    pub fn resolution(&self) -> f32 {
        self.resolution
    }

    /// World coordinates of cell (0, 0)
    #[inline]
    pub fn origin(&self) -> WorldPoint {
        self.origin
    }

    /// Total number of cells
    #[inline]
    pub fn cell_count(&self) -> usize {
        self.width * self.height
    }

    /// Raw access to cell types array (for SIMD operations)
    #[inline]
    pub fn cell_types_raw(&self) -> &[u8] {
        &self.cell_types
    }

    /// World bounds: (min_point, max_point)
    pub fn bounds(&self) -> (WorldPoint, WorldPoint) {
        let min = self.origin;
        let max = WorldPoint::new(
            self.origin.x + self.width as f32 * self.resolution,
            self.origin.y + self.height as f32 * self.resolution,
        );
        (min, max)
    }

    /// Convert world coordinates to grid coordinates
    #[inline]
    pub fn world_to_grid(&self, point: WorldPoint) -> GridCoord {
        let x = ((point.x - self.origin.x) / self.resolution).floor() as i32;
        let y = ((point.y - self.origin.y) / self.resolution).floor() as i32;
        GridCoord::new(x, y)
    }

    /// Convert grid coordinates to world coordinates (cell center)
    #[inline]
    pub fn grid_to_world(&self, coord: GridCoord) -> WorldPoint {
        WorldPoint::new(
            self.origin.x + (coord.x as f32 + 0.5) * self.resolution,
            self.origin.y + (coord.y as f32 + 0.5) * self.resolution,
        )
    }

    /// Check if grid coordinates are within bounds
    #[inline]
    pub fn is_valid_coord(&self, coord: GridCoord) -> bool {
        coord.x >= 0
            && coord.y >= 0
            && (coord.x as usize) < self.width
            && (coord.y as usize) < self.height
    }

    /// Check if world point is within grid bounds
    #[inline]
    pub fn contains_point(&self, point: WorldPoint) -> bool {
        let (min, max) = self.bounds();
        point.x >= min.x && point.x < max.x && point.y >= min.y && point.y < max.y
    }

    /// Convert grid coordinates to flat array index
    #[inline]
    pub fn coord_to_index(&self, coord: GridCoord) -> Option<usize> {
        if self.is_valid_coord(coord) {
            Some(coord.y as usize * self.width + coord.x as usize)
        } else {
            None
        }
    }

    /// Convert flat array index to grid coordinates
    #[inline]
    pub fn index_to_coord(&self, index: usize) -> GridCoord {
        GridCoord::new((index % self.width) as i32, (index / self.width) as i32)
    }

    /// Get cell at grid coordinates (reconstructs Cell from SoA data)
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
    ///
    /// Returns a `CellMut` that allows modifying individual fields.
    #[inline]
    pub fn get_mut(&mut self, coord: GridCoord) -> Option<CellMut<'_>> {
        self.coord_to_index(coord).map(move |i| CellMut {
            cell_type: &mut self.cell_types[i],
            confidence: &mut self.confidences[i],
            observation_count: &mut self.observation_counts[i],
            swept: &mut self.swept_flags[i],
        })
    }

    /// Get cell at world coordinates
    #[inline]
    pub fn get_world(&self, point: WorldPoint) -> Option<Cell> {
        self.get(self.world_to_grid(point))
    }

    /// Get mutable cell at world coordinates
    #[inline]
    pub fn get_world_mut(&mut self, point: WorldPoint) -> Option<CellMut<'_>> {
        let coord = self.world_to_grid(point);
        self.get_mut(coord)
    }

    /// Get cell type at grid coordinates (returns Unknown if out of bounds)
    #[inline]
    pub fn get_type(&self, coord: GridCoord) -> CellType {
        self.coord_to_index(coord)
            .map(|i| CellType::from_u8(self.cell_types[i]))
            .unwrap_or(CellType::Unknown)
    }

    /// Get cell type at world coordinates
    #[inline]
    pub fn get_type_world(&self, point: WorldPoint) -> CellType {
        self.get_type(self.world_to_grid(point))
    }

    /// Set cell type at grid coordinates
    /// Returns true if the cell was updated
    #[inline]
    pub fn set_type(&mut self, coord: GridCoord, cell_type: CellType) -> bool {
        if let Some(i) = self.coord_to_index(coord) {
            self.observe_at_index(i, cell_type)
        } else {
            false
        }
    }

    /// Set cell type with priority rules
    /// Returns true if the cell type changed
    #[inline]
    pub fn set_type_with_priority(&mut self, coord: GridCoord, cell_type: CellType) -> bool {
        if let Some(i) = self.coord_to_index(coord) {
            self.observe_with_priority_at_index(i, cell_type)
        } else {
            false
        }
    }

    /// Observe a cell type at a given index (internal helper)
    #[inline]
    fn observe_at_index(&mut self, i: usize, cell_type: CellType) -> bool {
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

    /// Observe a cell type with priority at a given index (internal helper)
    #[inline]
    fn observe_with_priority_at_index(&mut self, i: usize, cell_type: CellType) -> bool {
        let old_type = CellType::from_u8(self.cell_types[i]);
        let should_update = match (old_type, cell_type) {
            // Unknown can be updated by anything
            (CellType::Unknown, _) => true,
            // Bump is highest priority - only update with another Bump
            (CellType::Bump, CellType::Bump) => true,
            (CellType::Bump, _) => false,
            // Cliff can be updated by Bump or Cliff
            (CellType::Cliff, CellType::Bump) => true,
            (CellType::Cliff, CellType::Cliff) => true,
            (CellType::Cliff, _) => false,
            // Wall can be updated by Bump, Cliff, or Wall
            (CellType::Wall, CellType::Bump) => true,
            (CellType::Wall, CellType::Cliff) => true,
            (CellType::Wall, CellType::Wall) => true,
            (CellType::Wall, _) => false,
            // Floor can be updated by anything
            (CellType::Floor, _) => true,
        };

        if should_update {
            self.observe_at_index(i, cell_type)
        } else {
            // Still count the observation even if type didn't change
            self.observation_counts[i] = self.observation_counts[i].saturating_add(1);
            false
        }
    }

    /// Clear all cells to Unknown
    pub fn clear(&mut self) {
        self.cell_types.fill(CellType::Unknown as u8);
        self.confidences.fill(0);
        self.observation_counts.fill(0);
        self.swept_flags.fill(0);
        self.distance_field.fill(f32::MAX);
    }

    /// Iterate over all cells with their coordinates
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

    /// Iterate over all cells mutably with their coordinates
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

    /// Get raw cell types slice (for SIMD operations and serialization)
    #[inline]
    pub fn cell_types(&self) -> &[u8] {
        &self.cell_types
    }

    /// Get mutable raw cell types slice
    #[inline]
    pub fn cell_types_mut(&mut self) -> &mut [u8] {
        &mut self.cell_types
    }

    /// Get raw confidences slice
    #[inline]
    pub fn confidences(&self) -> &[u8] {
        &self.confidences
    }

    /// Get raw observation counts slice
    #[inline]
    pub fn observation_counts(&self) -> &[u8] {
        &self.observation_counts
    }

    /// Get raw swept flags slice
    #[inline]
    pub fn swept_flags(&self) -> &[u8] {
        &self.swept_flags
    }

    /// Get mutable swept flags slice
    #[inline]
    pub fn swept_flags_mut(&mut self) -> &mut [u8] {
        &mut self.swept_flags
    }

    /// Get raw cells slice (for serialization compatibility)
    ///
    /// Note: This reconstructs Cell structs from SoA data.
    /// For performance-critical code, use the individual field slices directly.
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

    /// Count cells by type (SIMD-optimized implementation)
    pub fn count_by_type(&self) -> CellCounts {
        self.count_by_type_simd()
    }

    /// Scalar implementation of cell counting (always available for testing)
    #[allow(dead_code)]
    pub fn count_by_type_scalar(&self) -> CellCounts {
        let mut counts = CellCounts::default();
        for &cell_type in &self.cell_types {
            match cell_type {
                0 => counts.unknown += 1, // Unknown
                1 => counts.floor += 1,   // Floor
                2 => counts.wall += 1,    // Wall
                3 => counts.cliff += 1,   // Cliff
                4 => counts.bump += 1,    // Bump
                _ => counts.unknown += 1, // Invalid -> Unknown
            }
        }
        counts
    }

    /// SIMD-optimized cell counting (processes 16 cells at a time)
    fn count_by_type_simd(&self) -> CellCounts {
        let mut counts = CellCounts::default();

        // SIMD constants for comparison
        let unknown_vec = u8x16::splat(CellType::Unknown as u8);
        let floor_vec = u8x16::splat(CellType::Floor as u8);
        let wall_vec = u8x16::splat(CellType::Wall as u8);
        let cliff_vec = u8x16::splat(CellType::Cliff as u8);
        let bump_vec = u8x16::splat(CellType::Bump as u8);

        // Process 16 cells at a time
        let chunks = self.cell_types.chunks_exact(16);
        let remainder = chunks.remainder();

        for chunk in chunks {
            let data = u8x16::from_slice(chunk);

            // Compare and count matches using bitmask
            counts.unknown += data.simd_eq(unknown_vec).to_bitmask().count_ones() as usize;
            counts.floor += data.simd_eq(floor_vec).to_bitmask().count_ones() as usize;
            counts.wall += data.simd_eq(wall_vec).to_bitmask().count_ones() as usize;
            counts.cliff += data.simd_eq(cliff_vec).to_bitmask().count_ones() as usize;
            counts.bump += data.simd_eq(bump_vec).to_bitmask().count_ones() as usize;
        }

        // Handle remainder with scalar fallback
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

    /// Check if all cells in a rectangular region are Floor (SIMD-optimized)
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

            // Process 16 cells at a time
            let chunks = row_slice.chunks_exact(16);
            let remainder = chunks.remainder();

            for chunk in chunks {
                let data = u8x16::from_slice(chunk);
                let is_floor = data.simd_eq(floor_vec);
                if is_floor.to_bitmask() != 0xFFFF {
                    return false; // At least one cell is not Floor
                }
            }

            // Check remainder
            for &cell_type in remainder {
                if cell_type != CellType::Floor as u8 {
                    return false;
                }
            }
        }

        true
    }

    /// Expand the grid to include a world point
    /// Returns true if the grid was expanded
    pub fn expand_to_include(
        &mut self,
        point: WorldPoint,
        max_width: usize,
        max_height: usize,
    ) -> bool {
        let coord = self.world_to_grid(point);

        // Calculate required expansion
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
            return false; // No expansion needed
        }

        let new_width = self.width + expand_left + expand_right;
        let new_height = self.height + expand_down + expand_up;

        // Check max size
        if new_width > max_width || new_height > max_height {
            return false; // Would exceed max size
        }

        let new_size = new_width * new_height;

        // Allocate new arrays
        let mut new_cell_types = vec![CellType::Unknown as u8; new_size];
        let mut new_confidences = vec![0u8; new_size];
        let mut new_observation_counts = vec![0u8; new_size];
        let mut new_swept_flags = vec![0u8; new_size];
        let mut new_distance_field = vec![f32::MAX; new_size];

        // Copy old data row by row
        for y in 0..self.height {
            let old_row_start = y * self.width;
            let new_row_start = (y + expand_down) * new_width + expand_left;

            // Copy each array
            new_cell_types[new_row_start..new_row_start + self.width]
                .copy_from_slice(&self.cell_types[old_row_start..old_row_start + self.width]);
            new_confidences[new_row_start..new_row_start + self.width]
                .copy_from_slice(&self.confidences[old_row_start..old_row_start + self.width]);
            new_observation_counts[new_row_start..new_row_start + self.width].copy_from_slice(
                &self.observation_counts[old_row_start..old_row_start + self.width],
            );
            new_swept_flags[new_row_start..new_row_start + self.width]
                .copy_from_slice(&self.swept_flags[old_row_start..old_row_start + self.width]);
            new_distance_field[new_row_start..new_row_start + self.width]
                .copy_from_slice(&self.distance_field[old_row_start..old_row_start + self.width]);
        }

        // Update state
        self.cell_types = new_cell_types;
        self.confidences = new_confidences;
        self.observation_counts = new_observation_counts;
        self.swept_flags = new_swept_flags;
        self.distance_field = new_distance_field;
        self.width = new_width;
        self.height = new_height;
        self.origin = WorldPoint::new(
            self.origin.x - expand_left as f32 * self.resolution,
            self.origin.y - expand_down as f32 * self.resolution,
        );

        true
    }

    // === Distance Field Methods ===

    /// Get distance to nearest wall at grid coordinates (in meters).
    /// Returns f32::MAX if no wall is nearby or if coord is out of bounds.
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

    /// Get raw distance field slice (for debugging/visualization)
    #[inline]
    pub fn distance_field(&self) -> &[f32] {
        &self.distance_field
    }

    /// Get distance to nearest wall with bilinear interpolation.
    ///
    /// Unlike `get_distance_world` which uses nearest-cell lookup,
    /// this method provides smooth gradients for sub-pixel accuracy
    /// in Gauss-Newton optimization during scan matching.
    ///
    /// Returns (distance, gradient_x, gradient_y) where gradients are
    /// in meters per meter (dimensionless slope).
    #[inline]
    pub fn get_distance_interpolated(&self, point: WorldPoint) -> (f32, f32, f32) {
        // Convert to continuous grid coordinates
        let gx = (point.x - self.origin.x) / self.resolution;
        let gy = (point.y - self.origin.y) / self.resolution;

        // Get integer cell coordinates (floor)
        let x0 = gx.floor() as i32;
        let y0 = gy.floor() as i32;

        // Fractional part within cell [0, 1)
        let fx = gx - x0 as f32;
        let fy = gy - y0 as f32;

        // Sample 2x2 neighborhood
        let d00 = self.get_distance_safe(x0, y0);
        let d10 = self.get_distance_safe(x0 + 1, y0);
        let d01 = self.get_distance_safe(x0, y0 + 1);
        let d11 = self.get_distance_safe(x0 + 1, y0 + 1);

        // Bilinear interpolation for distance
        let d0 = d00 * (1.0 - fx) + d10 * fx; // Bottom edge
        let d1 = d01 * (1.0 - fx) + d11 * fx; // Top edge
        let distance = d0 * (1.0 - fy) + d1 * fy;

        // Gradient computation via finite differences
        // ∂d/∂x ≈ (d(x+h) - d(x-h)) / (2h) where h = resolution
        // Using the bilinear coefficients:
        let grad_x = ((d10 - d00) * (1.0 - fy) + (d11 - d01) * fy) / self.resolution;
        let grad_y = ((d01 - d00) * (1.0 - fx) + (d11 - d10) * fx) / self.resolution;

        (distance, grad_x, grad_y)
    }

    /// Helper to get distance with safe bounds checking.
    /// Returns max_distance for out-of-bounds or unknown areas.
    #[inline]
    fn get_distance_safe(&self, x: i32, y: i32) -> f32 {
        if x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height {
            let idx = y as usize * self.width + x as usize;
            let d = self.distance_field[idx];
            // Clamp MAX values to max_distance for smoother interpolation
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
    /// Uses brushfire (BFS) algorithm to propagate distances outward.
    pub fn update_distance_field_from_wall(&mut self, wall_coord: GridCoord) {
        use std::collections::VecDeque;

        // Wall cell has distance 0
        if let Some(wall_idx) = self.coord_to_index(wall_coord) {
            self.distance_field[wall_idx] = 0.0;
        } else {
            return;
        }

        // BFS queue: (coord, distance)
        let mut queue = VecDeque::new();
        queue.push_back((wall_coord, 0.0f32));

        // 8-connected neighbors (including diagonals for smoother distance)
        let neighbors_4 = [(0, 1), (1, 0), (0, -1), (-1, 0)];
        let neighbors_diag = [(1, 1), (1, -1), (-1, 1), (-1, -1)];

        let diag_dist = self.resolution * std::f32::consts::SQRT_2;

        while let Some((coord, dist)) = queue.pop_front() {
            // Process 4-connected neighbors (orthogonal)
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

            // Process diagonal neighbors
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
    /// More efficient than calling update_distance_field_from_wall multiple times.
    pub fn update_distance_field_batch(&mut self, wall_coords: &[GridCoord]) {
        use std::collections::VecDeque;

        if wall_coords.is_empty() {
            return;
        }

        // BFS queue: (coord, distance)
        let mut queue = VecDeque::with_capacity(wall_coords.len() * 4);

        // Initialize all wall cells with distance 0
        for &coord in wall_coords {
            if let Some(idx) = self.coord_to_index(coord) {
                self.distance_field[idx] = 0.0;
                queue.push_back((coord, 0.0f32));
            }
        }

        // 8-connected neighbors
        let neighbors_4 = [(0, 1), (1, 0), (0, -1), (-1, 0)];
        let neighbors_diag = [(1, 1), (1, -1), (-1, 1), (-1, -1)];

        let diag_dist = self.resolution * std::f32::consts::SQRT_2;

        while let Some((coord, dist)) = queue.pop_front() {
            // Process 4-connected neighbors
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

            // Process diagonal neighbors
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
    /// Call this after major changes to wall configuration.
    pub fn recompute_distance_field(&mut self) {
        // Reset all distances to MAX
        self.distance_field.fill(f32::MAX);

        // Collect all wall cells
        let wall_coords: Vec<GridCoord> = (0..self.cell_types.len())
            .filter(|&i| self.cell_types[i] == CellType::Wall as u8)
            .map(|i| self.index_to_coord(i))
            .collect();

        // Batch update from all walls
        self.update_distance_field_batch(&wall_coords);
    }
}

/// Mutable reference to a single cell's fields in the SoA storage.
///
/// This allows modifying individual cell properties without reconstructing
/// the entire Cell struct.
pub struct CellMut<'a> {
    /// Cell type as raw u8
    pub cell_type: &'a mut u8,
    /// Confidence level
    pub confidence: &'a mut u8,
    /// Observation count
    pub observation_count: &'a mut u8,
    /// Swept flag (0 = not swept, 1 = swept)
    pub swept: &'a mut u8,
}

impl<'a> CellMut<'a> {
    /// Get the cell type
    #[inline]
    pub fn get_type(&self) -> CellType {
        CellType::from_u8(*self.cell_type)
    }

    /// Set the cell type
    #[inline]
    pub fn set_type(&mut self, cell_type: CellType) {
        *self.cell_type = cell_type as u8;
    }

    /// Check if traversable
    #[inline]
    pub fn is_traversable(&self) -> bool {
        *self.cell_type == CellType::Floor as u8
    }

    /// Check if obstacle
    #[inline]
    pub fn is_obstacle(&self) -> bool {
        let t = *self.cell_type;
        t == CellType::Wall as u8 || t == CellType::Cliff as u8 || t == CellType::Bump as u8
    }

    /// Update with a new observation
    pub fn observe(&mut self, observed_type: CellType) -> bool {
        let new_type = observed_type as u8;
        let changed = *self.cell_type != new_type;

        if changed {
            *self.cell_type = new_type;
            *self.confidence = 1;
            *self.observation_count = 1;
        } else {
            *self.confidence = self.confidence.saturating_add(1);
            *self.observation_count = self.observation_count.saturating_add(1);
        }

        changed
    }

    /// Update with priority rules
    pub fn observe_with_priority(&mut self, observed_type: CellType) -> bool {
        let old_type = CellType::from_u8(*self.cell_type);
        let should_update = match (old_type, observed_type) {
            (CellType::Unknown, _) => true,
            (CellType::Bump, CellType::Bump) => true,
            (CellType::Bump, _) => false,
            (CellType::Cliff, CellType::Bump | CellType::Cliff) => true,
            (CellType::Cliff, _) => false,
            (CellType::Wall, CellType::Bump | CellType::Cliff | CellType::Wall) => true,
            (CellType::Wall, _) => false,
            (CellType::Floor, _) => true,
        };

        if should_update {
            self.observe(observed_type)
        } else {
            *self.observation_count = self.observation_count.saturating_add(1);
            false
        }
    }
}

/// Cell counts by type
#[derive(Clone, Copy, Debug, Default)]
pub struct CellCounts {
    /// Unknown cells (not yet observed)
    pub unknown: usize,
    /// Floor cells (traversable)
    pub floor: usize,
    /// Wall cells (lidar-detected obstacles)
    pub wall: usize,
    /// Cliff cells (floor drop-offs)
    pub cliff: usize,
    /// Bump cells (invisible obstacles)
    pub bump: usize,
}

impl CellCounts {
    /// Total known cells
    pub fn known(&self) -> usize {
        self.floor + self.wall + self.cliff + self.bump
    }

    /// Total cells
    pub fn total(&self) -> usize {
        self.unknown + self.known()
    }

    /// Total obstacle cells
    pub fn obstacles(&self) -> usize {
        self.wall + self.cliff + self.bump
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grid_creation() {
        let grid = GridStorage::new(100, 100, 0.05, WorldPoint::ZERO);
        assert_eq!(grid.width(), 100);
        assert_eq!(grid.height(), 100);
        assert_eq!(grid.resolution(), 0.05);
        assert_eq!(grid.cell_count(), 10000);
    }

    #[test]
    fn test_centered_grid() {
        let grid = GridStorage::centered(100, 100, 0.05);
        let (min, max) = grid.bounds();

        // Should be centered around origin
        assert!((min.x + 2.5).abs() < 1e-6);
        assert!((min.y + 2.5).abs() < 1e-6);
        assert!((max.x - 2.5).abs() < 1e-6);
        assert!((max.y - 2.5).abs() < 1e-6);
    }

    #[test]
    fn test_world_to_grid_conversion() {
        let grid = GridStorage::new(100, 100, 0.05, WorldPoint::ZERO);

        // Origin should map to (0, 0)
        let coord = grid.world_to_grid(WorldPoint::new(0.0, 0.0));
        assert_eq!(coord, GridCoord::new(0, 0));

        // 1 meter should be 20 cells at 0.05 resolution
        let coord = grid.world_to_grid(WorldPoint::new(1.0, 1.0));
        assert_eq!(coord, GridCoord::new(20, 20));
    }

    #[test]
    fn test_grid_to_world_conversion() {
        let grid = GridStorage::new(100, 100, 0.05, WorldPoint::ZERO);

        // Cell (0, 0) center should be at (0.025, 0.025)
        let point = grid.grid_to_world(GridCoord::new(0, 0));
        assert!((point.x - 0.025).abs() < 1e-6);
        assert!((point.y - 0.025).abs() < 1e-6);
    }

    #[test]
    fn test_get_set_cell() {
        let mut grid = GridStorage::new(10, 10, 0.1, WorldPoint::ZERO);

        // Initially unknown
        assert_eq!(grid.get_type(GridCoord::new(5, 5)), CellType::Unknown);

        // Set to floor
        grid.set_type(GridCoord::new(5, 5), CellType::Floor);
        assert_eq!(grid.get_type(GridCoord::new(5, 5)), CellType::Floor);

        // Out of bounds returns Unknown
        assert_eq!(grid.get_type(GridCoord::new(100, 100)), CellType::Unknown);
    }

    #[test]
    fn test_grid_expansion() {
        let mut grid = GridStorage::centered(10, 10, 0.1);

        // Initial size
        assert_eq!(grid.width(), 10);
        assert_eq!(grid.height(), 10);

        // Expand to include a point outside
        let far_point = WorldPoint::new(2.0, 2.0);
        assert!(grid.expand_to_include(far_point, 100, 100));

        // Grid should be larger now
        assert!(grid.width() > 10);
        assert!(grid.height() > 10);

        // Point should now be within bounds
        assert!(grid.contains_point(far_point));
    }

    #[test]
    fn test_cell_counts() {
        let mut grid = GridStorage::new(10, 10, 0.1, WorldPoint::ZERO);

        grid.set_type(GridCoord::new(0, 0), CellType::Floor);
        grid.set_type(GridCoord::new(1, 0), CellType::Wall);
        grid.set_type(GridCoord::new(2, 0), CellType::Cliff);
        grid.set_type(GridCoord::new(3, 0), CellType::Bump);

        let counts = grid.count_by_type();
        assert_eq!(counts.floor, 1);
        assert_eq!(counts.wall, 1);
        assert_eq!(counts.cliff, 1);
        assert_eq!(counts.bump, 1);
        assert_eq!(counts.unknown, 96);
        assert_eq!(counts.known(), 4);
    }

    #[test]
    fn test_soa_iter() {
        let mut grid = GridStorage::new(10, 10, 0.1, WorldPoint::ZERO);
        grid.set_type(GridCoord::new(5, 5), CellType::Wall);

        let mut found_wall = false;
        for (coord, cell) in grid.iter() {
            if coord == GridCoord::new(5, 5) {
                assert_eq!(cell.cell_type, CellType::Wall);
                found_wall = true;
            }
        }
        assert!(found_wall);
    }

    #[test]
    fn test_cell_priority() {
        let mut grid = GridStorage::new(10, 10, 0.1, WorldPoint::ZERO);

        // Set to Floor first
        grid.set_type(GridCoord::new(0, 0), CellType::Floor);
        assert_eq!(grid.get_type(GridCoord::new(0, 0)), CellType::Floor);

        // Wall overrides Floor
        grid.set_type_with_priority(GridCoord::new(0, 0), CellType::Wall);
        assert_eq!(grid.get_type(GridCoord::new(0, 0)), CellType::Wall);

        // Floor does NOT override Wall
        grid.set_type_with_priority(GridCoord::new(0, 0), CellType::Floor);
        assert_eq!(grid.get_type(GridCoord::new(0, 0)), CellType::Wall);

        // Bump overrides Wall
        grid.set_type_with_priority(GridCoord::new(0, 0), CellType::Bump);
        assert_eq!(grid.get_type(GridCoord::new(0, 0)), CellType::Bump);
    }

    #[test]
    fn test_simd_count_matches_scalar() {
        let mut grid = GridStorage::new(100, 100, 0.05, WorldPoint::ZERO);

        // Set various cell types
        for i in 0..100 {
            grid.set_type(GridCoord::new(i, 0), CellType::Floor);
            grid.set_type(GridCoord::new(i, 1), CellType::Wall);
        }
        for i in 0..50 {
            grid.set_type(GridCoord::new(i, 2), CellType::Cliff);
            grid.set_type(GridCoord::new(i, 3), CellType::Bump);
        }

        let simd_counts = grid.count_by_type_simd();
        let scalar_counts = grid.count_by_type_scalar();

        assert_eq!(simd_counts.unknown, scalar_counts.unknown);
        assert_eq!(simd_counts.floor, scalar_counts.floor);
        assert_eq!(simd_counts.wall, scalar_counts.wall);
        assert_eq!(simd_counts.cliff, scalar_counts.cliff);
        assert_eq!(simd_counts.bump, scalar_counts.bump);
    }
}
