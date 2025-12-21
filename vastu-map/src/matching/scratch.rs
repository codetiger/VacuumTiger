//! Pre-allocated scratch space for ICP hot paths.
//!
//! This module provides buffer reuse to eliminate allocations during the
//! ICP inner loop, significantly improving performance on embedded systems.
//!
//! # Performance Impact
//!
//! Without scratch space, each ICP iteration allocates:
//! - Transformed points buffer (~360 points × 8 bytes = ~3KB)
//! - Distance buffer (~100 lines × 4 bytes × 360 points = ~140KB)
//! - Projection buffer (same as distance)
//! - Correspondence set (~360 × 24 bytes = ~9KB)
//!
//! With scratch space, these allocations happen once and are reused.

use crate::config::LidarNoiseModel;
use crate::core::Point2D;
use crate::features::LineCollection;

use super::correspondence::{Correspondence, CorrespondenceSet, CorrespondenceSoA};
use super::nearest_neighbor::NearestNeighborConfig;

/// Pre-allocated scratch space for ICP operations.
///
/// Reuse this struct across multiple `match_scan` calls to avoid
/// per-iteration allocations. Create once, use many times.
///
/// # Example
/// ```rust,ignore
/// use vastu_map::matching::{IcpScratchSpace, PointToLineIcp};
///
/// // Create scratch space sized for typical scans
/// let mut scratch = IcpScratchSpace::new(400, 200);
///
/// let icp = PointToLineIcp::new();
/// for scan in scans {
///     let result = icp.match_scan_with_scratch(&scan, &map, pose, &mut scratch);
/// }
/// ```
#[derive(Clone, Debug)]
pub struct IcpScratchSpace {
    /// Buffer for transformed points.
    pub(crate) transformed_xs: Vec<f32>,
    pub(crate) transformed_ys: Vec<f32>,

    /// Buffer for distances to all lines (per point).
    pub(crate) distances: Vec<f32>,

    /// Buffer for projection parameters (per point).
    pub(crate) projections: Vec<f32>,

    /// Reusable correspondence set (AoS layout).
    pub(crate) correspondences: CorrespondenceSet,

    /// Reusable correspondence set (SoA layout for SIMD).
    pub(crate) correspondences_soa: CorrespondenceSoA,

    /// Buffer for ranges (distances from sensor).
    pub(crate) ranges: Vec<f32>,

    // =========================================================================
    // Multi-resolution / Coarse search buffers
    // =========================================================================
    /// Buffer for subsampled point X coordinates.
    /// Used by multi-resolution ICP and coarse search to avoid allocations.
    pub(crate) subsampled_xs: Vec<f32>,

    /// Buffer for subsampled point Y coordinates.
    pub(crate) subsampled_ys: Vec<f32>,
}

impl IcpScratchSpace {
    /// Create new scratch space with capacity hints.
    ///
    /// # Arguments
    /// * `max_points` - Expected maximum scan points (e.g., 360 for a typical lidar)
    /// * `max_lines` - Expected maximum map lines (e.g., 100-200 for indoor)
    pub fn new(max_points: usize, max_lines: usize) -> Self {
        Self {
            transformed_xs: Vec::with_capacity(max_points),
            transformed_ys: Vec::with_capacity(max_points),
            distances: Vec::with_capacity(max_lines),
            projections: Vec::with_capacity(max_lines),
            correspondences: CorrespondenceSet::with_capacity(max_points),
            correspondences_soa: CorrespondenceSoA::with_capacity(max_points),
            ranges: Vec::with_capacity(max_points),
            subsampled_xs: Vec::with_capacity(max_points),
            subsampled_ys: Vec::with_capacity(max_points),
        }
    }

    /// Create with default capacity (360 points, 200 lines).
    pub fn default_capacity() -> Self {
        Self::new(360, 200)
    }

    /// Clear buffers for next iteration (keeps capacity).
    #[inline]
    pub fn clear(&mut self) {
        self.transformed_xs.clear();
        self.transformed_ys.clear();
        self.distances.clear();
        self.projections.clear();
        self.correspondences.clear();
        self.correspondences_soa.clear();
        self.ranges.clear();
        self.subsampled_xs.clear();
        self.subsampled_ys.clear();
    }

    // =========================================================================
    // Subsampling methods for multi-resolution ICP
    // =========================================================================

    /// Subsample points into internal buffer using step_by.
    ///
    /// This is the zero-allocation version of subsampling for multi-resolution ICP.
    /// After calling this, use `subsampled_point()` or `num_subsampled()` to access.
    ///
    /// # Arguments
    /// * `points` - Source points to subsample
    /// * `step` - Step size (1 = no subsampling, 2 = every other point, etc.)
    #[inline]
    pub fn subsample_points(&mut self, points: &[Point2D], step: usize) {
        self.subsampled_xs.clear();
        self.subsampled_ys.clear();

        let step = step.max(1);
        let expected_len = points.len().div_ceil(step);
        self.subsampled_xs.reserve(expected_len);
        self.subsampled_ys.reserve(expected_len);

        for p in points.iter().step_by(step) {
            self.subsampled_xs.push(p.x);
            self.subsampled_ys.push(p.y);
        }
    }

    /// Subsample from SoA arrays into internal buffer.
    ///
    /// Useful when points are already in SoA format (e.g., from PointCloud2D).
    #[inline]
    pub fn subsample_from_soa(&mut self, xs: &[f32], ys: &[f32], step: usize) {
        self.subsampled_xs.clear();
        self.subsampled_ys.clear();

        let step = step.max(1);
        let len = xs.len().min(ys.len());
        let expected_len = len.div_ceil(step);
        self.subsampled_xs.reserve(expected_len);
        self.subsampled_ys.reserve(expected_len);

        for i in (0..len).step_by(step) {
            self.subsampled_xs.push(xs[i]);
            self.subsampled_ys.push(ys[i]);
        }
    }

    /// Get a subsampled point by index.
    #[inline]
    pub fn subsampled_point(&self, index: usize) -> Point2D {
        Point2D::new(self.subsampled_xs[index], self.subsampled_ys[index])
    }

    /// Get number of subsampled points.
    #[inline]
    pub fn num_subsampled(&self) -> usize {
        self.subsampled_xs.len()
    }

    /// Get slice of subsampled X coordinates.
    #[inline]
    pub fn subsampled_xs(&self) -> &[f32] {
        &self.subsampled_xs
    }

    /// Get slice of subsampled Y coordinates.
    #[inline]
    pub fn subsampled_ys(&self) -> &[f32] {
        &self.subsampled_ys
    }

    /// Transform points by pose into internal buffer.
    ///
    /// After calling this, `transformed_point()` can be used to access results.
    #[inline]
    pub fn transform_points(&mut self, points: &[Point2D], sin: f32, cos: f32, tx: f32, ty: f32) {
        self.transformed_xs.clear();
        self.transformed_ys.clear();

        // Pre-extend to avoid reallocations during push
        self.transformed_xs.reserve(points.len());
        self.transformed_ys.reserve(points.len());

        for p in points {
            let new_x = p.x * cos - p.y * sin + tx;
            let new_y = p.x * sin + p.y * cos + ty;
            self.transformed_xs.push(new_x);
            self.transformed_ys.push(new_y);
        }
    }

    /// Get a transformed point by index.
    #[inline]
    pub fn transformed_point(&self, index: usize) -> Point2D {
        Point2D::new(self.transformed_xs[index], self.transformed_ys[index])
    }

    /// Get number of transformed points.
    #[inline]
    pub fn num_transformed(&self) -> usize {
        self.transformed_xs.len()
    }

    /// Find correspondences using pre-allocated buffers.
    ///
    /// This is the zero-allocation hot path for ICP iterations.
    pub fn find_correspondences(&mut self, lines: &LineCollection, config: &NearestNeighborConfig) {
        self.correspondences.clear();

        if self.transformed_xs.is_empty() || lines.is_empty() {
            return;
        }

        let min_t = -config.max_projection_extension;
        let max_t = 1.0 + config.max_projection_extension;
        let n_lines = lines.len();
        let n_points = self.transformed_xs.len();

        // Ensure buffers are large enough
        self.distances.resize(n_lines, 0.0);
        self.projections.resize(n_lines, 0.0);

        for point_idx in 0..n_points {
            let point = Point2D::new(
                self.transformed_xs[point_idx],
                self.transformed_ys[point_idx],
            );

            // Compute distances and projections into buffers
            lines.distances_to_point_into(point, &mut self.distances);
            self.compute_projections_into(point, lines);

            let mut best_distance = config.max_distance;
            let mut best_corr: Option<Correspondence> = None;

            for line_idx in 0..n_lines {
                let t = self.projections[line_idx];
                let distance = self.distances[line_idx];

                // Check projection bounds
                if t < min_t || t > max_t {
                    continue;
                }

                // Check distance threshold
                if distance > config.max_distance {
                    continue;
                }

                if config.unique_per_point {
                    if distance < best_distance {
                        best_distance = distance;
                        best_corr =
                            Some(Correspondence::new(point_idx, line_idx, point, distance, t));
                    }
                } else {
                    self.correspondences
                        .push(Correspondence::new(point_idx, line_idx, point, distance, t));
                }
            }

            if config.unique_per_point
                && let Some(corr) = best_corr
            {
                self.correspondences.push(corr);
            }
        }
    }

    /// Compute projection parameters into the projections buffer.
    ///
    /// Formula: t = dot(point - start, end - start) / |end - start|²
    fn compute_projections_into(&mut self, point: Point2D, lines: &LineCollection) {
        let n = lines.len();

        for i in 0..n {
            let sx = lines.start_xs[i];
            let sy = lines.start_ys[i];
            let ex = lines.end_xs[i];
            let ey = lines.end_ys[i];

            let dx = ex - sx;
            let dy = ey - sy;
            let len_sq = dx * dx + dy * dy;

            if len_sq < f32::EPSILON {
                self.projections[i] = 0.5; // Degenerate line
            } else {
                let to_px = point.x - sx;
                let to_py = point.y - sy;
                self.projections[i] = (to_px * dx + to_py * dy) / len_sq;
            }
        }
    }

    /// Take the correspondences out of the scratch space.
    ///
    /// This moves the correspondence set, leaving an empty one in the scratch space.
    /// Use `clear()` to reset for the next iteration instead if you don't need ownership.
    #[inline]
    pub fn take_correspondences(&mut self) -> CorrespondenceSet {
        std::mem::take(&mut self.correspondences)
    }

    /// Get a reference to the current correspondences.
    #[inline]
    pub fn correspondences(&self) -> &CorrespondenceSet {
        &self.correspondences
    }

    /// Get the number of correspondences found.
    #[inline]
    pub fn num_correspondences(&self) -> usize {
        self.correspondences.len()
    }

    // =========================================================================
    // SIMD-optimized SoA correspondence finding
    // =========================================================================

    /// Store ranges for later use in SoA correspondence finding.
    #[inline]
    pub fn store_ranges(&mut self, points: &[Point2D]) {
        self.ranges.clear();
        self.ranges.reserve(points.len());
        for p in points {
            self.ranges.push((p.x * p.x + p.y * p.y).sqrt());
        }
    }

    /// Find correspondences using pre-allocated buffers, producing SoA layout.
    ///
    /// This is the SIMD-optimized hot path for ICP iterations.
    /// The result is stored in `self.correspondences_soa` and is ready for
    /// use with `optimize_pose_simd()`.
    ///
    /// # Arguments
    /// * `lines` - Map lines in SoA format
    /// * `noise_model` - Lidar noise model for weight computation
    /// * `config` - Matching configuration
    pub fn find_correspondences_soa(
        &mut self,
        lines: &LineCollection,
        noise_model: &LidarNoiseModel,
        config: &NearestNeighborConfig,
    ) {
        self.correspondences_soa.clear();

        if self.transformed_xs.is_empty() || lines.is_empty() {
            return;
        }

        let min_t = -config.max_projection_extension;
        let max_t = 1.0 + config.max_projection_extension;
        let n_lines = lines.len();
        let n_points = self.transformed_xs.len();

        // Ensure buffers are large enough
        self.distances.resize(n_lines, 0.0);
        self.projections.resize(n_lines, 0.0);

        for point_idx in 0..n_points {
            let point = Point2D::new(
                self.transformed_xs[point_idx],
                self.transformed_ys[point_idx],
            );

            // Compute distances and projections into buffers
            lines.distances_to_point_into(point, &mut self.distances);
            self.compute_projections_into(point, lines);

            // Compute weight for this point based on its range
            let range = if point_idx < self.ranges.len() {
                self.ranges[point_idx]
            } else {
                1.0 // Default weight factor
            };
            let weight = noise_model.weight(range);

            let mut best_distance = config.max_distance;
            let mut best_line_idx: Option<usize> = None;

            for line_idx in 0..n_lines {
                let t = self.projections[line_idx];
                let distance = self.distances[line_idx];

                // Check projection bounds
                if t < min_t || t > max_t {
                    continue;
                }

                // Check distance threshold
                if distance > config.max_distance {
                    continue;
                }

                if config.unique_per_point {
                    if distance < best_distance {
                        best_distance = distance;
                        best_line_idx = Some(line_idx);
                    }
                } else {
                    self.correspondences_soa
                        .push(point_idx, line_idx, point, distance, weight, lines);
                }
            }

            if config.unique_per_point
                && let Some(line_idx) = best_line_idx
            {
                self.correspondences_soa.push(
                    point_idx,
                    line_idx,
                    point,
                    best_distance,
                    weight,
                    lines,
                );
            }
        }

        // Finalize with SIMD padding
        self.correspondences_soa.finalize();
    }

    /// Get a reference to the current SoA correspondences.
    #[inline]
    pub fn correspondences_soa(&self) -> &CorrespondenceSoA {
        &self.correspondences_soa
    }

    /// Get the number of SoA correspondences found.
    #[inline]
    pub fn num_correspondences_soa(&self) -> usize {
        self.correspondences_soa.len()
    }

    /// Take the SoA correspondences out of the scratch space.
    #[inline]
    pub fn take_correspondences_soa(&mut self) -> CorrespondenceSoA {
        std::mem::take(&mut self.correspondences_soa)
    }
}

impl Default for IcpScratchSpace {
    fn default() -> Self {
        Self::default_capacity()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::features::Line2D;

    fn make_room_lines() -> LineCollection {
        let lines = vec![
            Line2D::new(Point2D::new(-2.0, -2.0), Point2D::new(2.0, -2.0)), // Bottom
            Line2D::new(Point2D::new(2.0, -2.0), Point2D::new(2.0, 2.0)),   // Right
            Line2D::new(Point2D::new(2.0, 2.0), Point2D::new(-2.0, 2.0)),   // Top
            Line2D::new(Point2D::new(-2.0, 2.0), Point2D::new(-2.0, -2.0)), // Left
        ];
        LineCollection::from_lines(&lines)
    }

    #[test]
    fn test_scratch_space_creation() {
        let scratch = IcpScratchSpace::new(100, 50);
        // Verify buffers have the expected capacity
        assert!(scratch.transformed_xs.capacity() >= 100);
        assert!(scratch.distances.capacity() >= 50);
    }

    #[test]
    fn test_transform_points() {
        let mut scratch = IcpScratchSpace::new(10, 10);
        let points = vec![Point2D::new(1.0, 0.0), Point2D::new(0.0, 1.0)];

        // Identity transform
        scratch.transform_points(&points, 0.0, 1.0, 0.0, 0.0);

        assert_eq!(scratch.num_transformed(), 2);
        let p0 = scratch.transformed_point(0);
        let p1 = scratch.transformed_point(1);

        assert!((p0.x - 1.0).abs() < 1e-6);
        assert!((p0.y - 0.0).abs() < 1e-6);
        assert!((p1.x - 0.0).abs() < 1e-6);
        assert!((p1.y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_transform_with_rotation() {
        let mut scratch = IcpScratchSpace::new(10, 10);
        let points = vec![Point2D::new(1.0, 0.0)];

        // 90 degree rotation
        let (sin, cos) = std::f32::consts::FRAC_PI_2.sin_cos();
        scratch.transform_points(&points, sin, cos, 0.0, 0.0);

        let p = scratch.transformed_point(0);
        assert!((p.x - 0.0).abs() < 1e-6);
        assert!((p.y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_find_correspondences_basic() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let lines = make_room_lines();

        // Points near the walls
        let points = vec![
            Point2D::new(0.0, -1.95), // Near bottom wall
            Point2D::new(1.95, 0.0),  // Near right wall
            Point2D::new(0.0, 1.95),  // Near top wall
            Point2D::new(-1.95, 0.0), // Near left wall
        ];

        // Identity transform
        scratch.transform_points(&points, 0.0, 1.0, 0.0, 0.0);

        let config = NearestNeighborConfig::default();
        scratch.find_correspondences(&lines, &config);

        assert_eq!(scratch.num_correspondences(), 4);
    }

    #[test]
    fn test_find_correspondences_with_threshold() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let lines = make_room_lines();

        // One point near wall, one far from walls
        let points = vec![
            Point2D::new(0.0, -1.95), // Near bottom wall (0.05m away)
            Point2D::new(0.0, 0.0),   // Center (2.0m from all walls)
        ];

        scratch.transform_points(&points, 0.0, 1.0, 0.0, 0.0);

        let config = NearestNeighborConfig::default().with_max_distance(0.5);
        scratch.find_correspondences(&lines, &config);

        // Only the first point should match
        assert_eq!(scratch.num_correspondences(), 1);
    }

    #[test]
    fn test_clear_reuse() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let lines = make_room_lines();
        let points = vec![Point2D::new(0.0, -1.95)];

        // First iteration
        scratch.transform_points(&points, 0.0, 1.0, 0.0, 0.0);
        let config = NearestNeighborConfig::default();
        scratch.find_correspondences(&lines, &config);
        assert_eq!(scratch.num_correspondences(), 1);

        // Clear and reuse
        scratch.clear();
        assert_eq!(scratch.num_correspondences(), 0);
        assert_eq!(scratch.num_transformed(), 0);

        // Second iteration
        scratch.transform_points(&points, 0.0, 1.0, 0.0, 0.0);
        scratch.find_correspondences(&lines, &config);
        assert_eq!(scratch.num_correspondences(), 1);
    }

    #[test]
    fn test_take_correspondences() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let lines = make_room_lines();
        let points = vec![Point2D::new(0.0, -1.95)];

        scratch.transform_points(&points, 0.0, 1.0, 0.0, 0.0);
        let config = NearestNeighborConfig::default();
        scratch.find_correspondences(&lines, &config);

        let corrs = scratch.take_correspondences();
        assert_eq!(corrs.len(), 1);
        assert_eq!(scratch.num_correspondences(), 0); // Should be empty after take
    }

    #[test]
    fn test_empty_inputs() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let lines = LineCollection::new();
        let config = NearestNeighborConfig::default();

        // Empty points
        scratch.transform_points(&[], 0.0, 1.0, 0.0, 0.0);
        scratch.find_correspondences(&lines, &config);
        assert_eq!(scratch.num_correspondences(), 0);

        // Empty lines
        let points = vec![Point2D::new(0.0, 0.0)];
        scratch.transform_points(&points, 0.0, 1.0, 0.0, 0.0);
        scratch.find_correspondences(&lines, &config);
        assert_eq!(scratch.num_correspondences(), 0);
    }

    // =========================================================================
    // Subsample tests
    // =========================================================================

    #[test]
    fn test_subsample_no_subsampling() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let points: Vec<Point2D> = (0..10).map(|i| Point2D::new(i as f32, 0.0)).collect();

        scratch.subsample_points(&points, 1);

        assert_eq!(scratch.num_subsampled(), 10);
        for i in 0..10 {
            let p = scratch.subsampled_point(i);
            assert!((p.x - i as f32).abs() < 1e-6);
            assert!((p.y - 0.0).abs() < 1e-6);
        }
    }

    #[test]
    fn test_subsample_every_other() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let points: Vec<Point2D> = (0..10).map(|i| Point2D::new(i as f32, 0.0)).collect();

        scratch.subsample_points(&points, 2);

        assert_eq!(scratch.num_subsampled(), 5);
        // Should have 0, 2, 4, 6, 8
        assert!((scratch.subsampled_point(0).x - 0.0).abs() < 1e-6);
        assert!((scratch.subsampled_point(1).x - 2.0).abs() < 1e-6);
        assert!((scratch.subsampled_point(2).x - 4.0).abs() < 1e-6);
        assert!((scratch.subsampled_point(3).x - 6.0).abs() < 1e-6);
        assert!((scratch.subsampled_point(4).x - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_subsample_step_3() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let points: Vec<Point2D> = (0..12).map(|i| Point2D::new(i as f32, 0.0)).collect();

        scratch.subsample_points(&points, 3);

        assert_eq!(scratch.num_subsampled(), 4);
        // Should have 0, 3, 6, 9
        assert!((scratch.subsampled_point(0).x - 0.0).abs() < 1e-6);
        assert!((scratch.subsampled_point(1).x - 3.0).abs() < 1e-6);
        assert!((scratch.subsampled_point(2).x - 6.0).abs() < 1e-6);
        assert!((scratch.subsampled_point(3).x - 9.0).abs() < 1e-6);
    }

    #[test]
    fn test_subsample_soa() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let xs: Vec<f32> = (0..10).map(|i| i as f32).collect();
        let ys: Vec<f32> = (0..10).map(|i| (i as f32) * 2.0).collect();

        scratch.subsample_from_soa(&xs, &ys, 2);

        assert_eq!(scratch.num_subsampled(), 5);
        assert!((scratch.subsampled_xs()[0] - 0.0).abs() < 1e-6);
        assert!((scratch.subsampled_xs()[1] - 2.0).abs() < 1e-6);
        assert!((scratch.subsampled_ys()[0] - 0.0).abs() < 1e-6);
        assert!((scratch.subsampled_ys()[1] - 4.0).abs() < 1e-6);
    }

    #[test]
    fn test_subsample_reuse() {
        let mut scratch = IcpScratchSpace::new(100, 10);

        // First subsample
        let points1: Vec<Point2D> = (0..10).map(|i| Point2D::new(i as f32, 0.0)).collect();
        scratch.subsample_points(&points1, 2);
        assert_eq!(scratch.num_subsampled(), 5);

        // Second subsample (different data, reusing scratch)
        let points2: Vec<Point2D> = (0..6).map(|i| Point2D::new(i as f32 * 10.0, 0.0)).collect();
        scratch.subsample_points(&points2, 3);
        assert_eq!(scratch.num_subsampled(), 2);
        // Should have 0.0, 30.0
        assert!((scratch.subsampled_point(0).x - 0.0).abs() < 1e-6);
        assert!((scratch.subsampled_point(1).x - 30.0).abs() < 1e-6);
    }

    #[test]
    fn test_subsample_empty() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let points: Vec<Point2D> = vec![];

        scratch.subsample_points(&points, 2);

        assert_eq!(scratch.num_subsampled(), 0);
    }

    #[test]
    fn test_subsample_step_zero_treated_as_one() {
        let mut scratch = IcpScratchSpace::new(100, 10);
        let points: Vec<Point2D> = (0..5).map(|i| Point2D::new(i as f32, 0.0)).collect();

        // Step of 0 should be treated as 1 (no subsampling)
        scratch.subsample_points(&points, 0);

        assert_eq!(scratch.num_subsampled(), 5);
    }
}
