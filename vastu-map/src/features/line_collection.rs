//! Line collection with SoA (Struct of Arrays) layout for SIMD operations.
//!
//! This layout enables efficient batch operations via SIMD:
//! - 4 floats per line = clean SIMD alignment
//! - Batch distance computations
//! - Parallel transforms
//!
//! # Performance Optimizations
//!
//! Pre-computed cached values avoid redundant calculations in hot paths:
//! - `normal_xs`, `normal_ys`: Unit normal vectors (avoids sqrt per distance calc)
//! - `inv_lengths`: Reciprocal of line lengths (avoids division in inner loops)
//!
//! # SIMD Implementation
//!
//! Uses `std::simd::f32x4` for explicit 4-wide SIMD operations via the
//! `portable_simd` feature.

use std::simd::{f32x4, num::SimdFloat};

use super::line::Line2D;
use crate::core::{Point2D, Pose2D};

/// Collection of line segments with SoA layout for SIMD operations.
///
/// Each line is represented by 4 floats: (start_x, start_y, end_x, end_y).
/// This layout enables efficient batch operations using 4-wide SIMD.
///
/// # Pre-computed Values
///
/// For performance, the collection caches:
/// - **Normals**: Unit normal vectors for each line
/// - **Inverse lengths**: `1.0 / length` to avoid division in inner loops
///
/// These are automatically computed when lines are added.
///
/// # Example
/// ```
/// use vastu_map::features::{Line2D, LineCollection};
/// use vastu_map::core::Point2D;
///
/// let lines = vec![
///     Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
///     Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0)),
/// ];
///
/// let collection = LineCollection::from_lines(&lines);
/// assert_eq!(collection.len(), 2);
/// ```
#[derive(Clone, Debug, Default)]
pub struct LineCollection {
    /// X coordinates of line start points.
    pub start_xs: Vec<f32>,
    /// Y coordinates of line start points.
    pub start_ys: Vec<f32>,
    /// X coordinates of line end points.
    pub end_xs: Vec<f32>,
    /// Y coordinates of line end points.
    pub end_ys: Vec<f32>,
    /// Observation counts for each line.
    pub observation_counts: Vec<u32>,

    // === Pre-computed cached values for performance ===
    /// X components of unit normal vectors.
    normal_xs: Vec<f32>,
    /// Y components of unit normal vectors.
    normal_ys: Vec<f32>,
    /// Reciprocal of line lengths (1.0 / length).
    /// Used to avoid division in distance calculations.
    inv_lengths: Vec<f32>,
}

impl LineCollection {
    /// Create a new empty line collection.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a line collection with capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            start_xs: Vec::with_capacity(capacity),
            start_ys: Vec::with_capacity(capacity),
            end_xs: Vec::with_capacity(capacity),
            end_ys: Vec::with_capacity(capacity),
            observation_counts: Vec::with_capacity(capacity),
            normal_xs: Vec::with_capacity(capacity),
            normal_ys: Vec::with_capacity(capacity),
            inv_lengths: Vec::with_capacity(capacity),
        }
    }

    /// Create from a slice of Line2D.
    pub fn from_lines(lines: &[Line2D]) -> Self {
        let mut collection = Self::with_capacity(lines.len());
        for line in lines {
            collection.push_line(line);
        }
        collection
    }

    /// Compute and cache normal and inverse length for a line.
    #[inline]
    fn compute_cached_values(
        start_x: f32,
        start_y: f32,
        end_x: f32,
        end_y: f32,
    ) -> (f32, f32, f32) {
        let dx = end_x - start_x;
        let dy = end_y - start_y;
        let len_sq = dx * dx + dy * dy;

        if len_sq < f32::EPSILON {
            // Degenerate line - use arbitrary normal and large inv_length
            (0.0, 1.0, 0.0)
        } else {
            let len = len_sq.sqrt();
            let inv_len = 1.0 / len;
            // Normal is perpendicular to direction: (-dy, dx) normalized
            let nx = -dy * inv_len;
            let ny = dx * inv_len;
            (nx, ny, inv_len)
        }
    }

    /// Add a line to the collection.
    #[inline]
    pub fn push(&mut self, start_x: f32, start_y: f32, end_x: f32, end_y: f32, count: u32) {
        let (nx, ny, inv_len) = Self::compute_cached_values(start_x, start_y, end_x, end_y);

        self.start_xs.push(start_x);
        self.start_ys.push(start_y);
        self.end_xs.push(end_x);
        self.end_ys.push(end_y);
        self.observation_counts.push(count);
        self.normal_xs.push(nx);
        self.normal_ys.push(ny);
        self.inv_lengths.push(inv_len);
    }

    /// Add a Line2D to the collection.
    #[inline]
    pub fn push_line(&mut self, line: &Line2D) {
        self.push(
            line.start.x,
            line.start.y,
            line.end.x,
            line.end.y,
            line.observation_count,
        );
    }

    /// Get the pre-computed normal vector for a line.
    #[inline]
    pub fn normal(&self, index: usize) -> Option<Point2D> {
        if index < self.len() {
            Some(Point2D::new(self.normal_xs[index], self.normal_ys[index]))
        } else {
            None
        }
    }

    /// Get the pre-computed inverse length for a line.
    #[inline]
    pub fn inv_length(&self, index: usize) -> Option<f32> {
        self.inv_lengths.get(index).copied()
    }

    /// Get all normals as slices (for batch operations).
    #[inline]
    pub fn normals(&self) -> (&[f32], &[f32]) {
        (&self.normal_xs, &self.normal_ys)
    }

    /// Get all inverse lengths (for batch operations).
    #[inline]
    pub fn inv_lengths(&self) -> &[f32] {
        &self.inv_lengths
    }

    /// Number of lines in the collection.
    #[inline]
    pub fn len(&self) -> usize {
        self.start_xs.len()
    }

    /// Check if the collection is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.start_xs.is_empty()
    }

    /// Get a line by index.
    #[inline]
    pub fn get(&self, index: usize) -> Option<Line2D> {
        if index < self.len() {
            Some(Line2D::with_observation_count(
                Point2D::new(self.start_xs[index], self.start_ys[index]),
                Point2D::new(self.end_xs[index], self.end_ys[index]),
                self.observation_counts[index],
            ))
        } else {
            None
        }
    }

    /// Convert to `Vec<Line2D>`.
    pub fn to_lines(&self) -> Vec<Line2D> {
        (0..self.len())
            .map(|i| {
                Line2D::with_observation_count(
                    Point2D::new(self.start_xs[i], self.start_ys[i]),
                    Point2D::new(self.end_xs[i], self.end_ys[i]),
                    self.observation_counts[i],
                )
            })
            .collect()
    }

    /// Compute perpendicular distances from a point to all lines.
    ///
    /// Uses SIMD-optimized batch computation with pre-computed inverse lengths.
    /// Returns a vector of distances (always non-negative).
    ///
    /// # Formula
    /// ```text
    /// distance = |cross(p - start, end - start)| * inv_length
    ///          = |((px - sx) * (ey - sy) - (py - sy) * (ex - sx))| * inv_length
    /// ```
    ///
    /// # Performance
    /// Uses pre-computed `inv_lengths` to avoid sqrt/division in the inner loop.
    pub fn distances_to_point(&self, point: Point2D) -> Vec<f32> {
        let n = self.len();
        let mut distances = vec![0.0; n];

        let px = f32x4::splat(point.x);
        let py = f32x4::splat(point.y);

        // Process 4 lines at a time
        let chunks = n / 4;
        for i in 0..chunks {
            let base = i * 4;

            // Load line data - contiguous SIMD loads
            let sx = f32x4::from_slice(&self.start_xs[base..]);
            let sy = f32x4::from_slice(&self.start_ys[base..]);
            let ex = f32x4::from_slice(&self.end_xs[base..]);
            let ey = f32x4::from_slice(&self.end_ys[base..]);

            // Load pre-computed inverse lengths
            let inv_len = f32x4::from_slice(&self.inv_lengths[base..]);

            // Compute direction vectors
            let dx = ex - sx;
            let dy = ey - sy;

            // Compute vectors from start to point
            let to_px = px - sx;
            let to_py = py - sy;

            // Cross product: (px - sx) * (ey - sy) - (py - sy) * (ex - sx)
            // = to_px * dy - to_py * dx
            let cross = to_px * dy - to_py * dx;

            // Distance = |cross| * inv_length (no sqrt needed!)
            let abs_cross = cross.abs();
            let dist_4 = abs_cross * inv_len;
            distances[base..base + 4].copy_from_slice(&dist_4.to_array());
        }

        // Handle remainder (scalar) using pre-computed values
        #[allow(clippy::needless_range_loop)]
        for i in (chunks * 4)..n {
            let sx = self.start_xs[i];
            let sy = self.start_ys[i];
            let ex = self.end_xs[i];
            let ey = self.end_ys[i];
            let inv_len = self.inv_lengths[i];

            let dx = ex - sx;
            let dy = ey - sy;
            let to_px = point.x - sx;
            let to_py = point.y - sy;
            let cross = to_px * dy - to_py * dx;

            distances[i] = cross.abs() * inv_len;
        }

        distances
    }

    /// Compute projection parameters for a point onto all lines.
    ///
    /// Returns a vector of projection parameters t, where:
    /// - t = 0 means the projection is at line start
    /// - t = 1 means the projection is at line end
    /// - t < 0 or t > 1 means the projection is outside the segment
    ///
    /// # Formula
    /// ```text
    /// t = dot(point - start, end - start) / |end - start|²
    /// ```
    pub fn project_point(&self, point: Point2D) -> Vec<f32> {
        let n = self.len();
        let mut projections = vec![0.0; n];

        let px = f32x4::splat(point.x);
        let py = f32x4::splat(point.y);

        // Process 4 lines at a time
        let chunks = n / 4;
        for i in 0..chunks {
            let base = i * 4;

            // Load line data - contiguous SIMD loads
            let sx = f32x4::from_slice(&self.start_xs[base..]);
            let sy = f32x4::from_slice(&self.start_ys[base..]);
            let ex = f32x4::from_slice(&self.end_xs[base..]);
            let ey = f32x4::from_slice(&self.end_ys[base..]);

            // Compute direction vectors
            let dx = ex - sx;
            let dy = ey - sy;

            // Compute vectors from start to point
            let to_px = px - sx;
            let to_py = py - sy;

            // Dot product: (px - sx) * dx + (py - sy) * dy
            let dot = to_px * dx + to_py * dy;

            // Length squared: dx*dx + dy*dy
            let len_sq = dx * dx + dy * dy;

            // Compute projection t = dot / len_sq
            let dot_arr = dot.to_array();
            let len_sq_arr = len_sq.to_array();

            for j in 0..4 {
                projections[base + j] = if len_sq_arr[j] > f32::EPSILON {
                    dot_arr[j] / len_sq_arr[j]
                } else {
                    // Degenerate line - return 0.5 (midpoint)
                    0.5
                };
            }
        }

        // Handle remainder (scalar)
        for (i, proj) in projections.iter_mut().enumerate().skip(chunks * 4) {
            let line = self.get(i).unwrap();
            *proj = line.project_point(point);
        }

        projections
    }

    /// Find the nearest line to a point.
    ///
    /// Returns (index, distance) of the nearest line, or None if collection is empty.
    pub fn nearest_line(&self, point: Point2D) -> Option<(usize, f32)> {
        if self.is_empty() {
            return None;
        }

        let distances = self.distances_to_point(point);

        let mut min_idx = 0;
        let mut min_dist = distances[0];

        for (i, &d) in distances.iter().enumerate().skip(1) {
            if d < min_dist {
                min_dist = d;
                min_idx = i;
            }
        }

        Some((min_idx, min_dist))
    }

    /// Compute perpendicular distances from a point to all lines into a pre-allocated buffer.
    ///
    /// Zero-allocation variant of `distances_to_point()` for hot paths.
    /// Uses SIMD-optimized batch computation with pre-computed inverse lengths.
    ///
    /// # Panics
    /// Panics if `buffer.len() < self.len()`.
    pub fn distances_to_point_into(&self, point: Point2D, buffer: &mut [f32]) {
        let n = self.len();
        debug_assert!(
            buffer.len() >= n,
            "Buffer too small: {} < {}",
            buffer.len(),
            n
        );

        let px = f32x4::splat(point.x);
        let py = f32x4::splat(point.y);

        // Process 4 lines at a time
        let chunks = n / 4;
        for i in 0..chunks {
            let base = i * 4;

            // Load line data - contiguous SIMD loads
            let sx = f32x4::from_slice(&self.start_xs[base..]);
            let sy = f32x4::from_slice(&self.start_ys[base..]);
            let ex = f32x4::from_slice(&self.end_xs[base..]);
            let ey = f32x4::from_slice(&self.end_ys[base..]);

            // Load pre-computed inverse lengths
            let inv_len = f32x4::from_slice(&self.inv_lengths[base..]);

            // Compute direction vectors
            let dx = ex - sx;
            let dy = ey - sy;

            // Compute vectors from start to point
            let to_px = px - sx;
            let to_py = py - sy;

            // Cross product: (px - sx) * (ey - sy) - (py - sy) * (ex - sx)
            // = to_px * dy - to_py * dx
            let cross = to_px * dy - to_py * dx;

            // Distance = |cross| * inv_length (no sqrt needed!)
            let abs_cross = cross.abs();
            let dist_4 = abs_cross * inv_len;
            buffer[base..base + 4].copy_from_slice(&dist_4.to_array());
        }

        // Handle remainder (scalar) using pre-computed values
        #[allow(clippy::needless_range_loop)]
        for i in (chunks * 4)..n {
            let sx = self.start_xs[i];
            let sy = self.start_ys[i];
            let ex = self.end_xs[i];
            let ey = self.end_ys[i];
            let inv_len = self.inv_lengths[i];

            let dx = ex - sx;
            let dy = ey - sy;
            let to_px = point.x - sx;
            let to_py = point.y - sy;
            let cross = to_px * dy - to_py * dx;

            buffer[i] = cross.abs() * inv_len;
        }
    }

    /// Find the nearest line to a point using a reusable buffer.
    ///
    /// Zero-allocation variant of `nearest_line()` for hot paths like ICP.
    /// The buffer will be resized if needed.
    ///
    /// Returns (index, distance) of the nearest line, or None if collection is empty.
    pub fn nearest_line_with_buffer(
        &self,
        point: Point2D,
        buffer: &mut Vec<f32>,
    ) -> Option<(usize, f32)> {
        if self.is_empty() {
            return None;
        }

        let n = self.len();
        buffer.resize(n, 0.0);
        self.distances_to_point_into(point, buffer);

        let mut min_idx = 0;
        let mut min_dist = buffer[0];

        for (i, &d) in buffer.iter().enumerate().skip(1).take(n - 1) {
            if d < min_dist {
                min_dist = d;
                min_idx = i;
            }
        }

        Some((min_idx, min_dist))
    }

    /// Transform all lines by a pose.
    ///
    /// SIMD-optimized: processes 4 points per iteration.
    /// Normals are rotated (not recomputed) since rotation preserves angles.
    /// Inverse lengths are preserved since rotation+translation preserves length.
    pub fn transform(&self, pose: &Pose2D) -> Self {
        let n = self.len();
        let mut result = Self::with_capacity(n);
        result.start_xs.resize(n, 0.0);
        result.start_ys.resize(n, 0.0);
        result.end_xs.resize(n, 0.0);
        result.end_ys.resize(n, 0.0);
        result.normal_xs.resize(n, 0.0);
        result.normal_ys.resize(n, 0.0);
        result.observation_counts = self.observation_counts.clone();
        // Inverse lengths are preserved by rotation+translation
        result.inv_lengths = self.inv_lengths.clone();

        let (sin, cos) = pose.theta.sin_cos();
        let cos4 = f32x4::splat(cos);
        let sin4 = f32x4::splat(sin);
        let tx4 = f32x4::splat(pose.x);
        let ty4 = f32x4::splat(pose.y);

        // Process 4 start points at a time
        let chunks = n / 4;
        for i in 0..chunks {
            let base = i * 4;

            // Transform start points - contiguous SIMD loads
            let sx = f32x4::from_slice(&self.start_xs[base..]);
            let sy = f32x4::from_slice(&self.start_ys[base..]);

            // new_sx = sx*cos - sy*sin + tx
            // new_sy = sx*sin + sy*cos + ty
            let new_sx = sx * cos4 - sy * sin4 + tx4;
            let new_sy = sx * sin4 + sy * cos4 + ty4;

            result.start_xs[base..base + 4].copy_from_slice(&new_sx.to_array());
            result.start_ys[base..base + 4].copy_from_slice(&new_sy.to_array());

            // Transform end points
            let ex = f32x4::from_slice(&self.end_xs[base..]);
            let ey = f32x4::from_slice(&self.end_ys[base..]);

            let new_ex = ex * cos4 - ey * sin4 + tx4;
            let new_ey = ex * sin4 + ey * cos4 + ty4;

            result.end_xs[base..base + 4].copy_from_slice(&new_ex.to_array());
            result.end_ys[base..base + 4].copy_from_slice(&new_ey.to_array());

            // Rotate normals (no translation needed for unit vectors)
            let nx = f32x4::from_slice(&self.normal_xs[base..]);
            let ny = f32x4::from_slice(&self.normal_ys[base..]);

            // Rotate normal: n' = R * n (no translation)
            let new_nx = nx * cos4 - ny * sin4;
            let new_ny = nx * sin4 + ny * cos4;

            result.normal_xs[base..base + 4].copy_from_slice(&new_nx.to_array());
            result.normal_ys[base..base + 4].copy_from_slice(&new_ny.to_array());
        }

        // Handle remainder (scalar)
        for i in (chunks * 4)..n {
            let sx = self.start_xs[i];
            let sy = self.start_ys[i];
            let ex = self.end_xs[i];
            let ey = self.end_ys[i];
            let nx = self.normal_xs[i];
            let ny = self.normal_ys[i];

            result.start_xs[i] = sx * cos - sy * sin + pose.x;
            result.start_ys[i] = sx * sin + sy * cos + pose.y;
            result.end_xs[i] = ex * cos - ey * sin + pose.x;
            result.end_ys[i] = ex * sin + ey * cos + pose.y;
            result.normal_xs[i] = nx * cos - ny * sin;
            result.normal_ys[i] = nx * sin + ny * cos;
        }

        result
    }

    /// Clear all lines.
    pub fn clear(&mut self) {
        self.start_xs.clear();
        self.start_ys.clear();
        self.end_xs.clear();
        self.end_ys.clear();
        self.observation_counts.clear();
        self.normal_xs.clear();
        self.normal_ys.clear();
        self.inv_lengths.clear();
    }

    /// Remove a line by index (swap-remove for efficiency).
    pub fn swap_remove(&mut self, index: usize) {
        self.start_xs.swap_remove(index);
        self.start_ys.swap_remove(index);
        self.end_xs.swap_remove(index);
        self.end_ys.swap_remove(index);
        self.observation_counts.swap_remove(index);
        self.normal_xs.swap_remove(index);
        self.normal_ys.swap_remove(index);
        self.inv_lengths.swap_remove(index);
    }

    /// Iterate over lines.
    pub fn iter(&self) -> impl Iterator<Item = Line2D> + '_ {
        (0..self.len()).map(move |i| {
            Line2D::with_observation_count(
                Point2D::new(self.start_xs[i], self.start_ys[i]),
                Point2D::new(self.end_xs[i], self.end_ys[i]),
                self.observation_counts[i],
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn test_from_lines() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0)),
        ];

        let collection = LineCollection::from_lines(&lines);

        assert_eq!(collection.len(), 2);
        assert_eq!(collection.start_xs, vec![0.0, 0.0]);
        assert_eq!(collection.start_ys, vec![0.0, 1.0]);
        assert_eq!(collection.end_xs, vec![1.0, 1.0]);
        assert_eq!(collection.end_ys, vec![0.0, 1.0]);
    }

    #[test]
    fn test_to_lines() {
        let mut collection = LineCollection::new();
        collection.push(0.0, 0.0, 1.0, 0.0, 1);
        collection.push(0.0, 1.0, 1.0, 1.0, 2);

        let lines = collection.to_lines();

        assert_eq!(lines.len(), 2);
        assert_eq!(lines[0].start, Point2D::new(0.0, 0.0));
        assert_eq!(lines[0].end, Point2D::new(1.0, 0.0));
        assert_eq!(lines[0].observation_count, 1);
        assert_eq!(lines[1].observation_count, 2);
    }

    #[test]
    fn test_distances_to_point() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)), // Horizontal at y=0
            Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(10.0, 5.0)), // Horizontal at y=5
        ];

        let collection = LineCollection::from_lines(&lines);
        let point = Point2D::new(5.0, 2.0);

        let distances = collection.distances_to_point(point);

        assert_eq!(distances.len(), 2);
        assert_relative_eq!(distances[0], 2.0, epsilon = 1e-6); // 2 units above first line
        assert_relative_eq!(distances[1], 3.0, epsilon = 1e-6); // 3 units below second line
    }

    #[test]
    fn test_distances_to_point_simd() {
        // Test with more than 4 lines to exercise SIMD path
        let mut lines = Vec::new();
        for i in 0..8 {
            lines.push(Line2D::new(
                Point2D::new(0.0, i as f32),
                Point2D::new(10.0, i as f32),
            ));
        }

        let collection = LineCollection::from_lines(&lines);
        let point = Point2D::new(5.0, 3.5);

        let distances = collection.distances_to_point(point);

        assert_eq!(distances.len(), 8);
        assert_relative_eq!(distances[3], 0.5, epsilon = 1e-6); // Closest to y=3 line
        assert_relative_eq!(distances[4], 0.5, epsilon = 1e-6); // Equal distance to y=4 line
    }

    #[test]
    fn test_nearest_line() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(10.0, 5.0)),
            Line2D::new(Point2D::new(0.0, 2.0), Point2D::new(10.0, 2.0)),
        ];

        let collection = LineCollection::from_lines(&lines);
        let point = Point2D::new(5.0, 1.8);

        let (idx, dist) = collection.nearest_line(point).unwrap();

        assert_eq!(idx, 2); // y=2 line is nearest
        assert_relative_eq!(dist, 0.2, epsilon = 1e-6);
    }

    #[test]
    fn test_transform() {
        let lines = vec![Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(2.0, 0.0))];

        let collection = LineCollection::from_lines(&lines);

        // Rotate 90 degrees
        let pose = Pose2D::new(0.0, 0.0, FRAC_PI_2);
        let transformed = collection.transform(&pose);

        assert_relative_eq!(transformed.start_xs[0], 0.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.start_ys[0], 1.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.end_xs[0], 0.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.end_ys[0], 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_transform_simd() {
        // Test with more than 4 lines
        let mut lines = Vec::new();
        for i in 0..8 {
            lines.push(Line2D::new(
                Point2D::new(i as f32, 0.0),
                Point2D::new(i as f32 + 1.0, 0.0),
            ));
        }

        let collection = LineCollection::from_lines(&lines);
        let pose = Pose2D::new(10.0, 20.0, 0.0); // Just translation

        let transformed = collection.transform(&pose);

        for i in 0..8 {
            assert_relative_eq!(transformed.start_xs[i], i as f32 + 10.0, epsilon = 1e-5);
            assert_relative_eq!(transformed.start_ys[i], 20.0, epsilon = 1e-5);
        }
    }

    #[test]
    fn test_swap_remove() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0)),
            Line2D::new(Point2D::new(0.0, 2.0), Point2D::new(1.0, 2.0)),
        ];

        let mut collection = LineCollection::from_lines(&lines);
        collection.swap_remove(0);

        assert_eq!(collection.len(), 2);
        // First element is now the last one (swap-remove)
        assert_eq!(collection.start_ys[0], 2.0);
        assert_eq!(collection.start_ys[1], 1.0);
    }

    #[test]
    fn test_iter() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0)),
        ];

        let collection = LineCollection::from_lines(&lines);
        let collected: Vec<_> = collection.iter().collect();

        assert_eq!(collected.len(), 2);
        assert_eq!(collected[0].start, Point2D::new(0.0, 0.0));
        assert_eq!(collected[1].start, Point2D::new(0.0, 1.0));
    }

    #[test]
    fn test_distances_to_point_into() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)), // Horizontal at y=0
            Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(10.0, 5.0)), // Horizontal at y=5
        ];

        let collection = LineCollection::from_lines(&lines);
        let point = Point2D::new(5.0, 2.0);

        // Test buffer reuse variant
        let mut buffer = vec![0.0; 10]; // Oversized buffer
        collection.distances_to_point_into(point, &mut buffer);

        assert_relative_eq!(buffer[0], 2.0, epsilon = 1e-6);
        assert_relative_eq!(buffer[1], 3.0, epsilon = 1e-6);

        // Verify matches allocating version
        let distances = collection.distances_to_point(point);
        assert_relative_eq!(buffer[0], distances[0], epsilon = 1e-6);
        assert_relative_eq!(buffer[1], distances[1], epsilon = 1e-6);
    }

    #[test]
    fn test_nearest_line_with_buffer() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(10.0, 5.0)),
            Line2D::new(Point2D::new(0.0, 2.0), Point2D::new(10.0, 2.0)),
        ];

        let collection = LineCollection::from_lines(&lines);
        let point = Point2D::new(5.0, 1.8);

        // Test buffer reuse variant
        let mut buffer = Vec::new();
        let (idx, dist) = collection
            .nearest_line_with_buffer(point, &mut buffer)
            .unwrap();

        assert_eq!(idx, 2); // y=2 line is nearest
        assert_relative_eq!(dist, 0.2, epsilon = 1e-6);

        // Buffer should be reused on subsequent calls
        let (idx2, dist2) = collection
            .nearest_line_with_buffer(point, &mut buffer)
            .unwrap();
        assert_eq!(idx, idx2);
        assert_relative_eq!(dist, dist2, epsilon = 1e-6);

        // Verify matches allocating version
        let (idx_alloc, dist_alloc) = collection.nearest_line(point).unwrap();
        assert_eq!(idx, idx_alloc);
        assert_relative_eq!(dist, dist_alloc, epsilon = 1e-6);
    }

    #[test]
    fn test_distances_to_point_into_simd() {
        // Test with more than 4 lines to exercise SIMD path
        let mut lines = Vec::new();
        for i in 0..8 {
            lines.push(Line2D::new(
                Point2D::new(0.0, i as f32),
                Point2D::new(10.0, i as f32),
            ));
        }

        let collection = LineCollection::from_lines(&lines);
        let point = Point2D::new(5.0, 3.5);

        let mut buffer = vec![0.0; 8];
        collection.distances_to_point_into(point, &mut buffer);

        // Verify matches allocating version
        let distances = collection.distances_to_point(point);
        for i in 0..8 {
            assert_relative_eq!(buffer[i], distances[i], epsilon = 1e-6);
        }
    }
}
