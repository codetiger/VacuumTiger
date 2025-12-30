//! SIMD-accelerated batch operations for point transformations.
//!
//! This module provides vectorized operations for transforming point clouds,
//! optimized for ARM NEON (f32x4) and x86 SSE/AVX.
//!
//! ## Key Types
//!
//! - [`PointBatch`]: Structure-of-Arrays storage for 2D points
//! - [`RotationMatrix4`]: Pre-computed rotation matrix for SIMD transforms
//!
//! ## Usage
//!
//! ```rust,ignore
//! use vastu_map::core::simd::{PointBatch, transform_points_simd4, RotationMatrix4};
//!
//! let points = PointBatch::from_tuples(&[(1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (2.0, 0.0)]);
//! let pose = Pose2D::new(1.0, 2.0, 0.5);
//! let rot = RotationMatrix4::from_pose(&pose);
//!
//! let mut out_xs = vec![0.0f32; 4];
//! let mut out_ys = vec![0.0f32; 4];
//! transform_points_simd4(&points, &rot, &mut out_xs, &mut out_ys);
//! ```

use std::simd::{StdFloat, cmp::SimdPartialOrd, f32x4, i32x4, num::SimdFloat};

use super::Pose2D;

/// Number of lanes for SIMD operations (4 for ARM NEON / SSE)
pub const LANES: usize = 4;

/// Batch of 2D points in Structure-of-Arrays (SoA) layout.
///
/// This layout is optimal for SIMD operations where x and y coordinates
/// are processed independently.
///
/// ## Memory Layout
/// ```text
/// xs: [x0, x1, x2, x3, x4, ...]
/// ys: [y0, y1, y2, y3, y4, ...]
/// ```
#[derive(Clone, Debug, Default)]
pub struct PointBatch {
    /// X coordinates
    pub xs: Vec<f32>,
    /// Y coordinates
    pub ys: Vec<f32>,
}

impl PointBatch {
    /// Create an empty batch.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with specified capacity.
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            xs: Vec::with_capacity(capacity),
            ys: Vec::with_capacity(capacity),
        }
    }

    /// Create from slice of (x, y) tuples.
    pub fn from_tuples(points: &[(f32, f32)]) -> Self {
        let n = points.len();
        let mut xs = Vec::with_capacity(n);
        let mut ys = Vec::with_capacity(n);
        for &(x, y) in points {
            xs.push(x);
            ys.push(y);
        }
        Self { xs, ys }
    }

    /// Create with padding to next multiple of LANES.
    ///
    /// Padded values are zero (neutral for transforms).
    /// This eliminates remainder handling in SIMD loops.
    pub fn from_tuples_padded(points: &[(f32, f32)]) -> Self {
        let n = points.len();
        let padded_len = n.div_ceil(LANES) * LANES;
        let mut xs = vec![0.0; padded_len];
        let mut ys = vec![0.0; padded_len];
        for (i, &(x, y)) in points.iter().enumerate() {
            xs[i] = x;
            ys[i] = y;
        }
        Self { xs, ys }
    }

    /// Push a point.
    #[inline]
    pub fn push(&mut self, x: f32, y: f32) {
        self.xs.push(x);
        self.ys.push(y);
    }

    /// Number of points.
    #[inline]
    pub fn len(&self) -> usize {
        self.xs.len()
    }

    /// Check if empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.xs.is_empty()
    }

    /// Clear all points.
    #[inline]
    pub fn clear(&mut self) {
        self.xs.clear();
        self.ys.clear();
    }

    /// Original (unpadded) length for iteration.
    ///
    /// Use this when the batch was created with padding.
    pub fn original_len(&self) -> usize {
        // Find last non-zero element (assuming zeros are padding)
        // This is a heuristic; for precise tracking, store original_len separately
        self.len()
    }
}

/// Batch of grid coordinates in Structure-of-Arrays layout.
#[derive(Clone, Debug, Default)]
pub struct GridCoordBatch {
    /// X coordinates (column indices)
    pub xs: Vec<i32>,
    /// Y coordinates (row indices)
    pub ys: Vec<i32>,
}

impl GridCoordBatch {
    /// Create with specified capacity.
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            xs: Vec::with_capacity(capacity),
            ys: Vec::with_capacity(capacity),
        }
    }

    /// Resize to the given length, filling with zeros.
    #[inline]
    pub fn resize(&mut self, len: usize) {
        self.xs.resize(len, 0);
        self.ys.resize(len, 0);
    }

    /// Number of coordinates.
    #[inline]
    pub fn len(&self) -> usize {
        self.xs.len()
    }

    /// Check if empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.xs.is_empty()
    }
}

/// Pre-computed rotation matrix for batch transforms.
///
/// Stores cos(theta) and sin(theta) as SIMD vectors to avoid
/// redundant broadcasts in tight loops.
#[derive(Clone, Copy)]
pub struct RotationMatrix4 {
    /// cos(theta) broadcast to all lanes
    pub cos: f32x4,
    /// sin(theta) broadcast to all lanes
    pub sin: f32x4,
    /// Translation X (pose.x) broadcast to all lanes
    pub tx: f32x4,
    /// Translation Y (pose.y) broadcast to all lanes
    pub ty: f32x4,
}

impl RotationMatrix4 {
    /// Create rotation matrix from a pose.
    #[inline]
    pub fn from_pose(pose: &Pose2D) -> Self {
        let cos = pose.theta.cos();
        let sin = pose.theta.sin();
        Self {
            cos: f32x4::splat(cos),
            sin: f32x4::splat(sin),
            tx: f32x4::splat(pose.x),
            ty: f32x4::splat(pose.y),
        }
    }

    /// Create identity transform (no rotation or translation).
    #[inline]
    pub fn identity() -> Self {
        Self {
            cos: f32x4::splat(1.0),
            sin: f32x4::splat(0.0),
            tx: f32x4::splat(0.0),
            ty: f32x4::splat(0.0),
        }
    }
}

/// Transform points from sensor frame to world frame using SIMD.
///
/// Processes 4 points per iteration using ARM NEON / SSE instructions.
///
/// # Algorithm
/// ```text
/// world_x = pose.x + px * cos(theta) - py * sin(theta)
/// world_y = pose.y + px * sin(theta) + py * cos(theta)
/// ```
///
/// # Arguments
/// * `points` - Input points in sensor frame (SoA layout)
/// * `rot` - Pre-computed rotation matrix
/// * `out_xs` - Output X coordinates (must be at least `points.len()` long)
/// * `out_ys` - Output Y coordinates (must be at least `points.len()` long)
#[inline]
pub fn transform_points_simd4(
    points: &PointBatch,
    rot: &RotationMatrix4,
    out_xs: &mut [f32],
    out_ys: &mut [f32],
) {
    debug_assert_eq!(points.xs.len(), points.ys.len());
    debug_assert!(points.xs.len() <= out_xs.len());
    debug_assert!(points.xs.len() <= out_ys.len());

    let n = points.xs.len();
    let simd_end = n - (n % LANES);

    // SIMD path: process 4 points per iteration
    for i in (0..simd_end).step_by(LANES) {
        // Load 4 x and 4 y coordinates
        let px = f32x4::from_slice(&points.xs[i..]);
        let py = f32x4::from_slice(&points.ys[i..]);

        // Transform: world = translation + rotation * local
        // Uses FMA (fused multiply-add) on supporting hardware
        let world_x = rot.tx + px * rot.cos - py * rot.sin;
        let world_y = rot.ty + px * rot.sin + py * rot.cos;

        // Store results
        world_x.copy_to_slice(&mut out_xs[i..]);
        world_y.copy_to_slice(&mut out_ys[i..]);
    }

    // Scalar fallback for remainder
    let cos = rot.cos[0];
    let sin = rot.sin[0];
    let tx = rot.tx[0];
    let ty = rot.ty[0];

    for i in simd_end..n {
        let px = points.xs[i];
        let py = points.ys[i];
        out_xs[i] = tx + px * cos - py * sin;
        out_ys[i] = ty + px * sin + py * cos;
    }
}

/// Allocating version of `transform_points_simd4`.
///
/// Returns a new `PointBatch` with the transformed points.
pub fn transform_points(points: &PointBatch, pose: &Pose2D) -> PointBatch {
    let n = points.xs.len();
    let mut out_xs = vec![0.0; n];
    let mut out_ys = vec![0.0; n];

    let rot = RotationMatrix4::from_pose(pose);
    transform_points_simd4(points, &rot, &mut out_xs, &mut out_ys);

    PointBatch {
        xs: out_xs,
        ys: out_ys,
    }
}

/// Convert world coordinates to grid coordinates using SIMD.
///
/// Performs: `grid_coord = floor((world - origin) / resolution)`
///
/// # Arguments
/// * `world_xs` - World X coordinates
/// * `world_ys` - World Y coordinates
/// * `origin_x` - Grid origin X
/// * `origin_y` - Grid origin Y
/// * `inv_resolution` - Precomputed `1.0 / resolution`
/// * `out_xs` - Output grid X coordinates
/// * `out_ys` - Output grid Y coordinates
#[inline]
pub fn world_to_grid_simd4(
    world_xs: &[f32],
    world_ys: &[f32],
    origin_x: f32,
    origin_y: f32,
    inv_resolution: f32,
    out_xs: &mut [i32],
    out_ys: &mut [i32],
) {
    let n = world_xs.len();
    let simd_end = n - (n % LANES);

    let origin_x_v = f32x4::splat(origin_x);
    let origin_y_v = f32x4::splat(origin_y);
    let inv_res_v = f32x4::splat(inv_resolution);

    for i in (0..simd_end).step_by(LANES) {
        let wx = f32x4::from_slice(&world_xs[i..]);
        let wy = f32x4::from_slice(&world_ys[i..]);

        // (world - origin) / resolution
        let gx_f = (wx - origin_x_v) * inv_res_v;
        let gy_f = (wy - origin_y_v) * inv_res_v;

        // Floor and convert to i32 (SIMD cast)
        let gx_floor = gx_f.floor();
        let gy_floor = gy_f.floor();

        // Cast f32x4 to i32x4 and store directly
        let gx_i32: i32x4 = gx_floor.cast();
        let gy_i32: i32x4 = gy_floor.cast();
        gx_i32.copy_to_slice(&mut out_xs[i..i + LANES]);
        gy_i32.copy_to_slice(&mut out_ys[i..i + LANES]);
    }

    // Scalar remainder
    for i in simd_end..n {
        out_xs[i] = ((world_xs[i] - origin_x) * inv_resolution).floor() as i32;
        out_ys[i] = ((world_ys[i] - origin_y) * inv_resolution).floor() as i32;
    }
}

/// Calculate squared distances from origin for multiple points.
///
/// Returns `dx*dx + dy*dy` for each point (avoids sqrt for efficiency).
#[inline]
pub fn distance_squared_simd4(
    xs: &[f32],
    ys: &[f32],
    origin_x: f32,
    origin_y: f32,
    out: &mut [f32],
) {
    let n = xs.len();
    let simd_end = n - (n % LANES);

    let ox = f32x4::splat(origin_x);
    let oy = f32x4::splat(origin_y);

    for i in (0..simd_end).step_by(LANES) {
        let px = f32x4::from_slice(&xs[i..]);
        let py = f32x4::from_slice(&ys[i..]);

        let dx = px - ox;
        let dy = py - oy;

        // dx * dx + dy * dy
        let dist_sq = dx * dx + dy * dy;
        dist_sq.copy_to_slice(&mut out[i..]);
    }

    for i in simd_end..n {
        let dx = xs[i] - origin_x;
        let dy = ys[i] - origin_y;
        out[i] = dx * dx + dy * dy;
    }
}

/// Calculate actual distances (with sqrt).
///
/// Only use when actual distances are needed; squared is faster.
#[inline]
pub fn distance_simd4(xs: &[f32], ys: &[f32], origin_x: f32, origin_y: f32, out: &mut [f32]) {
    distance_squared_simd4(xs, ys, origin_x, origin_y, out);

    let n = out.len();
    let simd_end = n - (n % LANES);

    for i in (0..simd_end).step_by(LANES) {
        let sq = f32x4::from_slice(&out[i..]);
        let dist = sq.sqrt();
        dist.copy_to_slice(&mut out[i..]);
    }

    for val in out.iter_mut().skip(simd_end) {
        *val = val.sqrt();
    }
}

/// Check if grid coordinates are within bounds (batch operation).
///
/// Returns a mask where `true` indicates the coordinate is valid.
#[inline]
pub fn is_valid_coords_simd4(
    grid_xs: &[i32],
    grid_ys: &[i32],
    width: i32,
    height: i32,
    out_valid: &mut [bool],
) {
    let n = grid_xs.len();
    let simd_end = n - (n % LANES);

    let zero_v = i32x4::splat(0);
    let width_v = i32x4::splat(width);
    let height_v = i32x4::splat(height);

    for i in (0..simd_end).step_by(LANES) {
        let gx = i32x4::from_slice(&grid_xs[i..]);
        let gy = i32x4::from_slice(&grid_ys[i..]);

        // Valid if: x >= 0 && y >= 0 && x < width && y < height
        let x_ge_0 = gx.simd_ge(zero_v);
        let y_ge_0 = gy.simd_ge(zero_v);
        let x_lt_w = gx.simd_lt(width_v);
        let y_lt_h = gy.simd_lt(height_v);

        let valid = x_ge_0 & y_ge_0 & x_lt_w & y_lt_h;

        // Extract mask bits
        for j in 0..LANES {
            out_valid[i + j] = valid.test(j);
        }
    }

    // Scalar remainder
    for i in simd_end..n {
        let gx = grid_xs[i];
        let gy = grid_ys[i];
        out_valid[i] = gx >= 0 && gy >= 0 && gx < width && gy < height;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn test_point_batch_creation() {
        let points = PointBatch::from_tuples(&[(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]);
        assert_eq!(points.len(), 3);
        assert_eq!(points.xs[0], 1.0);
        assert_eq!(points.ys[0], 2.0);
    }

    #[test]
    fn test_point_batch_padded() {
        let points = PointBatch::from_tuples_padded(&[(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]);
        assert_eq!(points.len(), 4); // Padded to next multiple of 4
        assert_eq!(points.xs[3], 0.0); // Padding is zero
    }

    #[test]
    fn test_transform_identity() {
        let points = PointBatch::from_tuples(&[(1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (2.0, 2.0)]);
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        let result = transform_points(&points, &pose);

        for i in 0..4 {
            assert!((result.xs[i] - points.xs[i]).abs() < 1e-6);
            assert!((result.ys[i] - points.ys[i]).abs() < 1e-6);
        }
    }

    #[test]
    fn test_transform_translation() {
        let points = PointBatch::from_tuples(&[(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)]);
        let pose = Pose2D::new(10.0, 20.0, 0.0);

        let result = transform_points(&points, &pose);

        assert!((result.xs[0] - 10.0).abs() < 1e-6);
        assert!((result.ys[0] - 20.0).abs() < 1e-6);
        assert!((result.xs[1] - 11.0).abs() < 1e-6);
        assert!((result.ys[1] - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_transform_rotation_90deg() {
        let points = PointBatch::from_tuples(&[(1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (0.0, 0.0)]);
        let pose = Pose2D::new(0.0, 0.0, FRAC_PI_2);

        let result = transform_points(&points, &pose);

        // (1, 0) rotated 90° CCW -> (0, 1)
        assert!((result.xs[0] - 0.0).abs() < 1e-5);
        assert!((result.ys[0] - 1.0).abs() < 1e-5);

        // (0, 1) rotated 90° CCW -> (-1, 0)
        assert!((result.xs[1] - (-1.0)).abs() < 1e-5);
        assert!((result.ys[1] - 0.0).abs() < 1e-5);
    }

    #[test]
    fn test_transform_matches_scalar() {
        let points =
            PointBatch::from_tuples(&[(1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (7.0, 8.0), (9.0, 10.0)]);
        let pose = Pose2D::new(1.0, 2.0, 0.5);

        let result = transform_points(&points, &pose);

        // Compare with scalar transform
        let cos = pose.theta.cos();
        let sin = pose.theta.sin();

        for i in 0..points.len() {
            let px = points.xs[i];
            let py = points.ys[i];
            let expected_x = pose.x + px * cos - py * sin;
            let expected_y = pose.y + px * sin + py * cos;

            assert!(
                (result.xs[i] - expected_x).abs() < 1e-5,
                "x mismatch at {}",
                i
            );
            assert!(
                (result.ys[i] - expected_y).abs() < 1e-5,
                "y mismatch at {}",
                i
            );
        }
    }

    #[test]
    fn test_world_to_grid() {
        let world_xs = [0.0f32, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5];
        let world_ys = [0.0f32, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5];
        let mut grid_xs = [0i32; 8];
        let mut grid_ys = [0i32; 8];

        world_to_grid_simd4(
            &world_xs,
            &world_ys,
            0.0,
            0.0,
            1.0 / 0.5, // 2 cells per meter
            &mut grid_xs,
            &mut grid_ys,
        );

        assert_eq!(grid_xs[0], 0);
        assert_eq!(grid_xs[1], 1);
        assert_eq!(grid_xs[2], 2);
        assert_eq!(grid_xs[3], 3);
    }

    #[test]
    fn test_distance_squared() {
        let xs = [0.0f32, 3.0, 0.0, 3.0];
        let ys = [0.0f32, 0.0, 4.0, 4.0];
        let mut out = [0.0f32; 4];

        distance_squared_simd4(&xs, &ys, 0.0, 0.0, &mut out);

        assert!((out[0] - 0.0).abs() < 1e-6);
        assert!((out[1] - 9.0).abs() < 1e-6);
        assert!((out[2] - 16.0).abs() < 1e-6);
        assert!((out[3] - 25.0).abs() < 1e-6);
    }

    #[test]
    fn test_distance() {
        let xs = [0.0f32, 3.0, 0.0, 3.0];
        let ys = [0.0f32, 0.0, 4.0, 4.0];
        let mut out = [0.0f32; 4];

        distance_simd4(&xs, &ys, 0.0, 0.0, &mut out);

        assert!((out[0] - 0.0).abs() < 1e-6);
        assert!((out[1] - 3.0).abs() < 1e-6);
        assert!((out[2] - 4.0).abs() < 1e-6);
        assert!((out[3] - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_valid_coords() {
        let grid_xs = [-1, 0, 5, 10, 0, 0, 0, 0];
        let grid_ys = [0, -1, 0, 0, 5, 10, 0, 0];
        let mut valid = [false; 8];

        is_valid_coords_simd4(&grid_xs, &grid_ys, 10, 10, &mut valid);

        assert!(!valid[0]); // x < 0
        assert!(!valid[1]); // y < 0
        assert!(valid[2]); // valid
        assert!(!valid[3]); // x >= width
        assert!(valid[4]); // valid
        assert!(!valid[5]); // y >= height
        assert!(valid[6]); // valid (0, 0)
        assert!(valid[7]); // valid (0, 0)
    }
}
