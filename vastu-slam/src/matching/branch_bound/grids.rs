//! Precomputed multi-resolution grid hierarchy.

use crate::core::simd::{PointCloud, RotationMatrix4, transform_points_simd4};
use crate::core::{GridCoord, Pose2D, WorldPoint};
use crate::grid::GridStorage;

use super::config::NUM_LEVELS;

/// Precomputed multi-resolution grid hierarchy for branch-and-bound matching.
///
/// Each level stores probabilities (or distance-to-wall scores) where coarser
/// levels contain the maximum of all finer cells within each coarse cell.
pub struct PrecomputedGrids {
    /// Resolution levels from finest (0) to coarsest (NUM_LEVELS-1).
    /// Level i has cells 4^i times larger than level 0.
    pub(super) levels: [Vec<f32>; NUM_LEVELS],
    /// Width of each level in cells.
    pub(super) widths: [usize; NUM_LEVELS],
    /// Height of each level in cells.
    pub(super) heights: [usize; NUM_LEVELS],
    /// Resolution of level 0 (same as source grid).
    pub(super) resolution: f32,
    /// Origin of level 0 (same as source grid).
    pub(super) origin: WorldPoint,
}

impl PrecomputedGrids {
    /// Build precomputed grids from a grid storage.
    ///
    /// Level 0 is the original resolution, each subsequent level
    /// is 4x coarser (2x in each dimension).
    pub fn from_storage(storage: &GridStorage) -> Self {
        let width = storage.width();
        let height = storage.height();
        let resolution = storage.resolution();
        let origin = storage.origin();

        // Initialize level 0 from storage
        let mut levels: [Vec<f32>; NUM_LEVELS] = Default::default();
        let mut widths = [0usize; NUM_LEVELS];
        let mut heights = [0usize; NUM_LEVELS];

        widths[0] = width;
        heights[0] = height;
        levels[0] = Vec::with_capacity(width * height);

        // Convert grid to probability/score values
        // Use Gaussian scoring based on distance field for smooth values
        let sigma = resolution * 2.0;
        let two_sigma_sq = 2.0 * sigma * sigma;

        for y in 0..height {
            for x in 0..width {
                let coord = GridCoord::new(x as i32, y as i32);
                let distance = storage.get_distance(coord);

                // Gaussian score: high near walls, low far from walls
                let score = if distance < f32::MAX / 2.0 {
                    (-distance * distance / two_sigma_sq).exp()
                } else {
                    // Unknown area - small positive score for exploration
                    0.1
                };

                levels[0].push(score);
            }
        }

        // Build coarser levels by taking max of 2x2 regions
        for level in 1..NUM_LEVELS {
            let prev_width = widths[level - 1];
            let prev_height = heights[level - 1];

            // Round up to ensure we cover all cells
            let new_width = prev_width.div_ceil(2);
            let new_height = prev_height.div_ceil(2);

            widths[level] = new_width;
            heights[level] = new_height;

            let mut new_grid = vec![0.0f32; new_width * new_height];

            for y in 0..new_height {
                for x in 0..new_width {
                    // Find max of 2x2 region in previous level
                    let mut max_val = 0.0f32;

                    for dy in 0..2 {
                        for dx in 0..2 {
                            let px = x * 2 + dx;
                            let py = y * 2 + dy;

                            if px < prev_width && py < prev_height {
                                let idx = py * prev_width + px;
                                max_val = max_val.max(levels[level - 1][idx]);
                            }
                        }
                    }

                    new_grid[y * new_width + x] = max_val;
                }
            }

            levels[level] = new_grid;
        }

        Self {
            levels,
            widths,
            heights,
            resolution,
            origin,
        }
    }

    /// Get the score at a specific level and grid coordinate.
    #[inline]
    pub(super) fn get_score(&self, level: usize, x: i32, y: i32) -> f32 {
        if x < 0 || y < 0 {
            return 0.0;
        }
        let x = x as usize;
        let y = y as usize;
        if x >= self.widths[level] || y >= self.heights[level] {
            return 0.0;
        }
        self.levels[level][y * self.widths[level] + x]
    }

    /// Get the resolution at a specific level.
    #[inline]
    pub(super) fn level_resolution(&self, level: usize) -> f32 {
        self.resolution * (1 << level) as f32
    }

    /// Convert world point to grid coordinates at a specific level.
    #[inline]
    pub(super) fn world_to_level_grid(&self, point: WorldPoint, level: usize) -> GridCoord {
        let res = self.level_resolution(level);
        let x = ((point.x - self.origin.x) / res).floor() as i32;
        let y = ((point.y - self.origin.y) / res).floor() as i32;
        GridCoord::new(x, y)
    }

    /// Score a scan at a given pose and level (upper bound for coarser levels).
    pub(super) fn score_scan_at_level(
        &self,
        points: &[(f32, f32)],
        pose: Pose2D,
        level: usize,
        sensor_offset: (f32, f32),
    ) -> f32 {
        let cos_theta = pose.theta.cos();
        let sin_theta = pose.theta.sin();

        // Sensor position in world frame
        let (offset_x, offset_y) = sensor_offset;
        let sensor_x = pose.x + offset_x * cos_theta - offset_y * sin_theta;
        let sensor_y = pose.y + offset_x * sin_theta + offset_y * cos_theta;

        let mut score = 0.0f32;

        for &(px, py) in points {
            // Transform point from sensor frame to world frame
            let world_x = sensor_x + px * cos_theta - py * sin_theta;
            let world_y = sensor_y + px * sin_theta + py * cos_theta;

            let coord = self.world_to_level_grid(WorldPoint::new(world_x, world_y), level);
            score += self.get_score(level, coord.x, coord.y);
        }

        score
    }

    // =========================================================================
    // SIMD-OPTIMIZED METHODS
    // =========================================================================

    /// Score a scan at a given pose and level using SIMD-optimized transforms.
    ///
    /// Uses PointCloud (SoA format) and SIMD point transformation for efficiency.
    /// The final scoring is scalar (random grid access is not SIMD-friendly).
    pub fn score_scan_simd(
        &self,
        points: &PointCloud,
        pose: Pose2D,
        level: usize,
        sensor_offset: (f32, f32),
    ) -> f32 {
        let n = points.len();
        if n == 0 {
            return 0.0;
        }

        // Compute sensor pose in world frame
        let cos_t = pose.theta.cos();
        let sin_t = pose.theta.sin();
        let sensor_x = pose.x + sensor_offset.0 * cos_t - sensor_offset.1 * sin_t;
        let sensor_y = pose.y + sensor_offset.0 * sin_t + sensor_offset.1 * cos_t;
        let sensor_pose = Pose2D::new(sensor_x, sensor_y, pose.theta);

        // Transform all points to world frame using SIMD
        let rot = RotationMatrix4::from_pose(&sensor_pose);
        let mut world_xs = vec![0.0f32; n];
        let mut world_ys = vec![0.0f32; n];
        transform_points_simd4(points, &rot, &mut world_xs, &mut world_ys);

        // Convert to grid coordinates and sum scores
        let level_res = self.level_resolution(level);
        let inv_res = 1.0 / level_res;
        let width = self.widths[level] as i32;
        let height = self.heights[level] as i32;

        let mut score = 0.0f32;
        for i in 0..n {
            let gx = ((world_xs[i] - self.origin.x) * inv_res).floor() as i32;
            let gy = ((world_ys[i] - self.origin.y) * inv_res).floor() as i32;

            if gx >= 0 && gy >= 0 && gx < width && gy < height {
                score += self.levels[level][(gy as usize) * (width as usize) + (gx as usize)];
            }
        }

        score
    }

    /// Score scan with reusable scratch buffers to avoid allocations.
    ///
    /// This is the most efficient version for repeated calls.
    pub fn score_scan_simd_with_scratch(
        &self,
        points: &PointCloud,
        pose: Pose2D,
        level: usize,
        sensor_offset: (f32, f32),
        world_xs: &mut Vec<f32>,
        world_ys: &mut Vec<f32>,
    ) -> f32 {
        let n = points.len();
        if n == 0 {
            return 0.0;
        }

        // Ensure scratch buffers are large enough
        world_xs.resize(n, 0.0);
        world_ys.resize(n, 0.0);

        // Compute sensor pose in world frame
        let cos_t = pose.theta.cos();
        let sin_t = pose.theta.sin();
        let sensor_x = pose.x + sensor_offset.0 * cos_t - sensor_offset.1 * sin_t;
        let sensor_y = pose.y + sensor_offset.0 * sin_t + sensor_offset.1 * cos_t;
        let sensor_pose = Pose2D::new(sensor_x, sensor_y, pose.theta);

        // Transform all points using SIMD
        let rot = RotationMatrix4::from_pose(&sensor_pose);
        transform_points_simd4(points, &rot, world_xs, world_ys);

        // Convert to grid coordinates and sum scores
        let level_res = self.level_resolution(level);
        let inv_res = 1.0 / level_res;
        let width = self.widths[level] as i32;
        let height = self.heights[level] as i32;

        let mut score = 0.0f32;
        for i in 0..n {
            let gx = ((world_xs[i] - self.origin.x) * inv_res).floor() as i32;
            let gy = ((world_ys[i] - self.origin.y) * inv_res).floor() as i32;

            if gx >= 0 && gy >= 0 && gx < width && gy < height {
                score += self.levels[level][(gy as usize) * (width as usize) + (gx as usize)];
            }
        }

        score
    }

    /// Update level 0 from storage (for incremental updates).
    /// Call this when the storage has changed and you need fresh grids.
    pub fn update_from_storage(&mut self, storage: &GridStorage) {
        let width = storage.width();
        let height = storage.height();

        // Only update if dimensions match
        if width != self.widths[0] || height != self.heights[0] {
            // Full rebuild needed
            *self = Self::from_storage(storage);
            return;
        }

        // Update level 0
        let sigma = self.resolution * 2.0;
        let two_sigma_sq = 2.0 * sigma * sigma;

        self.levels[0].clear();
        for y in 0..height {
            for x in 0..width {
                let coord = GridCoord::new(x as i32, y as i32);
                let distance = storage.get_distance(coord);

                let score = if distance < f32::MAX / 2.0 {
                    (-distance * distance / two_sigma_sq).exp()
                } else {
                    0.1
                };

                self.levels[0].push(score);
            }
        }

        // Rebuild coarser levels
        for level in 1..NUM_LEVELS {
            let prev_width = self.widths[level - 1];
            let prev_height = self.heights[level - 1];
            let new_width = self.widths[level];
            let new_height = self.heights[level];

            for y in 0..new_height {
                for x in 0..new_width {
                    let mut max_val = 0.0f32;

                    for dy in 0..2 {
                        for dx in 0..2 {
                            let px = x * 2 + dx;
                            let py = y * 2 + dy;

                            if px < prev_width && py < prev_height {
                                let idx = py * prev_width + px;
                                max_val = max_val.max(self.levels[level - 1][idx]);
                            }
                        }
                    }

                    self.levels[level][y * new_width + x] = max_val;
                }
            }
        }
    }

    /// Get the dimensions at level 0.
    pub fn dimensions(&self) -> (usize, usize) {
        (self.widths[0], self.heights[0])
    }

    /// Get the resolution at level 0.
    pub fn resolution(&self) -> f32 {
        self.resolution
    }

    /// Get the origin.
    pub fn origin(&self) -> WorldPoint {
        self.origin
    }
}
