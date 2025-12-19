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

use crate::core::Point2D;
use crate::features::LineCollection;

use super::correspondence::{Correspondence, CorrespondenceSet};
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

    /// Reusable correspondence set.
    pub(crate) correspondences: CorrespondenceSet,
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
}
