//! Traversability checker implementation.

use std::simd::{cmp::SimdPartialOrd, u8x16};

use crate::core::{CellType, GridCoord, WorldPoint};
use crate::grid::GridStorage;

use super::footprint::RobotFootprint;
use super::path::PathCheckResult;

/// Traversability checker for path planning and navigation.
pub struct TraversabilityChecker<'a> {
    storage: &'a GridStorage,
    footprint: RobotFootprint,
}

impl<'a> TraversabilityChecker<'a> {
    /// Create a new traversability checker.
    pub fn new(storage: &'a GridStorage, footprint: RobotFootprint) -> Self {
        Self { storage, footprint }
    }

    /// Create with default robot footprint.
    pub fn with_default_footprint(storage: &'a GridStorage) -> Self {
        Self::new(storage, RobotFootprint::default())
    }

    /// Get the robot footprint.
    pub fn footprint(&self) -> &RobotFootprint {
        &self.footprint
    }

    /// Get the storage.
    pub fn storage(&self) -> &GridStorage {
        self.storage
    }

    // =========================================================================
    // Single cell traversability
    // =========================================================================

    /// Check if a single cell is traversable.
    pub fn is_cell_traversable(&self, coord: GridCoord) -> bool {
        if !self.storage.is_valid_coord(coord) {
            return false;
        }
        self.storage.get_type(coord).is_traversable()
    }

    /// Check if a world point is traversable (single cell).
    pub fn is_point_traversable(&self, point: WorldPoint) -> bool {
        let coord = self.storage.world_to_grid(point);
        self.is_cell_traversable(coord)
    }

    // =========================================================================
    // Position safety (considering footprint)
    // =========================================================================

    /// Check if a position is safe for the robot (considering footprint).
    pub fn is_position_safe(&self, position: WorldPoint) -> bool {
        self.is_position_safe_mode(position, false)
    }

    /// Check if a position is safe, with option to allow Unknown cells (SIMD-optimized).
    ///
    /// In exploration mode, Unknown cells are allowed in the footprint
    /// (they're assumed to be likely free). Only actual obstacles block.
    ///
    /// Uses SIMD to quickly check the bounding box for obstacles before
    /// performing the circular footprint check.
    pub fn is_position_safe_mode(&self, position: WorldPoint, allow_unknown: bool) -> bool {
        let center = self.storage.world_to_grid(position);
        let resolution = self.storage.resolution();
        let cells_radius = (self.footprint.total_radius() / resolution).ceil() as i32;

        // Fast path: SIMD check for obstacles in bounding box
        if !self.is_rect_obstacle_free(center, cells_radius, cells_radius) {
            return false;
        }

        // If allow_unknown is true, we're done (no obstacles found via SIMD)
        if allow_unknown {
            // Still need to check bounds validity
            return self.is_footprint_in_bounds(center, cells_radius, position);
        }

        // Strict mode: Need to check for Unknown cells in circular footprint
        self.is_footprint_traversable(center, cells_radius, position)
    }

    /// Check if a position is known (all cells within footprint are known).
    pub fn is_position_known(&self, position: WorldPoint) -> bool {
        let center = self.storage.world_to_grid(position);
        let resolution = self.storage.resolution();
        let cells_radius = (self.footprint.total_radius() / resolution).ceil() as i32;

        for dy in -cells_radius..=cells_radius {
            for dx in -cells_radius..=cells_radius {
                let coord = center + GridCoord::new(dx, dy);

                if !self.storage.is_valid_coord(coord) {
                    return false;
                }

                let cell_world = self.storage.grid_to_world(coord);
                let dist = cell_world.distance(&position);

                if dist <= self.footprint.total_radius() && !self.storage.get_type(coord).is_known()
                {
                    return false;
                }
            }
        }

        true
    }

    // =========================================================================
    // SIMD-optimized obstacle checking
    // =========================================================================

    /// Check if a rectangular region contains any obstacle cells (SIMD-optimized).
    ///
    /// Obstacles are Wall, Cliff, or Bump (cell_type >= 2).
    /// Returns true if no obstacles found.
    #[inline]
    pub(super) fn is_rect_obstacle_free(
        &self,
        center: GridCoord,
        half_width: i32,
        half_height: i32,
    ) -> bool {
        let width = self.storage.width() as i32;
        let height = self.storage.height() as i32;

        let min_x = (center.x - half_width).max(0) as usize;
        let max_x = ((center.x + half_width).min(width - 1) as usize).min(self.storage.width() - 1);
        let min_y = (center.y - half_height).max(0) as usize;
        let max_y =
            ((center.y + half_height).min(height - 1) as usize).min(self.storage.height() - 1);

        if min_x > max_x || min_y > max_y {
            return false; // Out of bounds
        }

        let cell_types = self.storage.cell_types_raw();
        let storage_width = self.storage.width();

        // Wall, Cliff, Bump are >= 2 (Floor=1, Unknown=0)
        let wall_val = CellType::Wall as u8;
        let wall_vec = u8x16::splat(wall_val);

        for y in min_y..=max_y {
            let row_start = y * storage_width + min_x;
            let row_end = y * storage_width + max_x + 1;
            let row_slice = &cell_types[row_start..row_end];

            // Process 16 cells at a time
            let chunks = row_slice.chunks_exact(16);
            let remainder = chunks.remainder();

            for chunk in chunks {
                let data = u8x16::from_slice(chunk);
                // Check if any cell >= Wall (2)
                let is_obstacle = data.simd_ge(wall_vec);
                if is_obstacle.to_bitmask() != 0 {
                    return false;
                }
            }

            // Scalar remainder
            for &cell_type in remainder {
                if cell_type >= wall_val {
                    return false;
                }
            }
        }

        true
    }

    // =========================================================================
    // Footprint checking helpers
    // =========================================================================

    /// Check if all cells in the circular footprint are within bounds.
    #[inline]
    fn is_footprint_in_bounds(
        &self,
        center: GridCoord,
        cells_radius: i32,
        position: WorldPoint,
    ) -> bool {
        for dy in -cells_radius..=cells_radius {
            for dx in -cells_radius..=cells_radius {
                let coord = center + GridCoord::new(dx, dy);

                if !self.storage.is_valid_coord(coord) {
                    // Check if this cell is within the circular footprint
                    let cell_world = self.storage.grid_to_world(coord);
                    let dist = cell_world.distance(&position);
                    if dist <= self.footprint.total_radius() {
                        return false; // Out of bounds within footprint
                    }
                }
            }
        }
        true
    }

    /// Check if all cells in the circular footprint are traversable (Floor).
    #[inline]
    fn is_footprint_traversable(
        &self,
        center: GridCoord,
        cells_radius: i32,
        position: WorldPoint,
    ) -> bool {
        for dy in -cells_radius..=cells_radius {
            for dx in -cells_radius..=cells_radius {
                let coord = center + GridCoord::new(dx, dy);

                if !self.storage.is_valid_coord(coord) {
                    // Check if this cell is within the circular footprint
                    let cell_world = self.storage.grid_to_world(coord);
                    let dist = cell_world.distance(&position);
                    if dist <= self.footprint.total_radius() {
                        return false; // Out of bounds within footprint
                    }
                    continue;
                }

                // Check if within circular footprint
                let cell_world = self.storage.grid_to_world(coord);
                let dist = cell_world.distance(&position);

                if dist <= self.footprint.total_radius() {
                    let cell_type = self.storage.get_type(coord);
                    if !cell_type.is_traversable() {
                        return false;
                    }
                }
            }
        }
        true
    }

    // =========================================================================
    // Path checking
    // =========================================================================

    /// Check if a straight-line path is traversable.
    /// Returns the furthest traversable point and whether the full path is clear.
    pub fn check_path(
        &self,
        start: WorldPoint,
        end: WorldPoint,
        step_size: Option<f32>,
    ) -> PathCheckResult {
        let step = step_size.unwrap_or(self.storage.resolution() * 0.5);
        let total_dist = start.distance(&end);

        if total_dist < step {
            // Very short path - just check endpoints
            let start_safe = self.is_position_safe(start);
            let end_safe = self.is_position_safe(end);

            return PathCheckResult {
                is_clear: start_safe && end_safe,
                blocked_at: if !start_safe {
                    Some(start)
                } else if !end_safe {
                    Some(end)
                } else {
                    None
                },
                furthest_safe: if start_safe { Some(end) } else { None },
                distance_traversable: if start_safe && end_safe {
                    total_dist
                } else {
                    0.0
                },
            };
        }

        let dir_x = (end.x - start.x) / total_dist;
        let dir_y = (end.y - start.y) / total_dist;

        let mut furthest_safe = start;
        let mut distance_traversable = 0.0;

        // Check start position first
        if !self.is_position_safe(start) {
            return PathCheckResult {
                is_clear: false,
                blocked_at: Some(start),
                furthest_safe: None,
                distance_traversable: 0.0,
            };
        }

        let mut dist = step;
        while dist < total_dist {
            let point = WorldPoint::new(start.x + dir_x * dist, start.y + dir_y * dist);

            if self.is_position_safe(point) {
                furthest_safe = point;
                distance_traversable = dist;
            } else {
                return PathCheckResult {
                    is_clear: false,
                    blocked_at: Some(point),
                    furthest_safe: Some(furthest_safe),
                    distance_traversable,
                };
            }

            dist += step;
        }

        // Check endpoint
        if self.is_position_safe(end) {
            PathCheckResult {
                is_clear: true,
                blocked_at: None,
                furthest_safe: Some(end),
                distance_traversable: total_dist,
            }
        } else {
            PathCheckResult {
                is_clear: false,
                blocked_at: Some(end),
                furthest_safe: Some(furthest_safe),
                distance_traversable,
            }
        }
    }

    /// Check if movement from current position in a direction is safe.
    pub fn check_direction(
        &self,
        position: WorldPoint,
        angle: f32,
        distance: f32,
    ) -> PathCheckResult {
        let end = WorldPoint::new(
            position.x + distance * angle.cos(),
            position.y + distance * angle.sin(),
        );
        self.check_path(position, end, None)
    }

    // =========================================================================
    // Obstacle queries
    // =========================================================================

    /// Find the nearest obstacle from a position.
    pub fn nearest_obstacle(&self, position: WorldPoint, max_distance: f32) -> Option<WorldPoint> {
        let center = self.storage.world_to_grid(position);
        let resolution = self.storage.resolution();
        let max_cells = (max_distance / resolution).ceil() as i32;

        let mut nearest: Option<(WorldPoint, f32)> = None;

        for dy in -max_cells..=max_cells {
            for dx in -max_cells..=max_cells {
                let coord = center + GridCoord::new(dx, dy);

                if !self.storage.is_valid_coord(coord) {
                    continue;
                }

                let cell_type = self.storage.get_type(coord);
                if cell_type.is_obstacle() {
                    let obstacle_pos = self.storage.grid_to_world(coord);
                    let dist = position.distance(&obstacle_pos);

                    if dist <= max_distance {
                        match nearest {
                            None => nearest = Some((obstacle_pos, dist)),
                            Some((_, d)) if dist < d => nearest = Some((obstacle_pos, dist)),
                            _ => {}
                        }
                    }
                }
            }
        }

        nearest.map(|(p, _)| p)
    }

    /// Get clearance (distance to nearest obstacle) at a position.
    pub fn clearance(&self, position: WorldPoint, max_distance: f32) -> f32 {
        match self.nearest_obstacle(position, max_distance) {
            Some(obstacle) => position.distance(&obstacle),
            None => max_distance,
        }
    }
}
