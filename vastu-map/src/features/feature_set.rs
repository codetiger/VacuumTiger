//! Feature set container holding lines and corners.
//!
//! A FeatureSet represents all features extracted from a single scan
//! or the accumulated features in a map.

use super::corner::{Corner2D, CornerCollection};
use super::line::Line2D;
use super::line_collection::LineCollection;
use crate::core::{Bounds, Point2D, Pose2D};

/// A set of geometric features (lines and corners).
///
/// Used to represent:
/// - Features extracted from a single lidar scan
/// - Accumulated features in a map
///
/// Provides both AoS (Array of Structs) and SoA (Struct of Arrays) access
/// for flexibility between ease of use and SIMD performance.
#[derive(Clone, Debug, Default)]
pub struct FeatureSet {
    /// Lines stored in SoA layout for SIMD operations.
    lines: LineCollection,
    /// Corners stored in SoA layout.
    corners: CornerCollection,
}

impl FeatureSet {
    /// Create a new empty feature set.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a feature set with capacity hints.
    pub fn with_capacity(line_capacity: usize, corner_capacity: usize) -> Self {
        Self {
            lines: LineCollection::with_capacity(line_capacity),
            corners: CornerCollection::with_capacity(corner_capacity),
        }
    }

    /// Create from lines and corners.
    pub fn from_features(lines: &[Line2D], corners: &[Corner2D]) -> Self {
        Self {
            lines: LineCollection::from_lines(lines),
            corners: CornerCollection::from_corners(corners),
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Line operations
    // ─────────────────────────────────────────────────────────────────────────

    /// Add a line to the feature set.
    #[inline]
    pub fn add_line(&mut self, line: Line2D) {
        self.lines.push_line(&line);
    }

    /// Add a line from components.
    #[inline]
    pub fn add_line_from_points(&mut self, start: Point2D, end: Point2D) {
        self.lines.push(start.x, start.y, end.x, end.y, 1);
    }

    /// Number of lines.
    #[inline]
    pub fn line_count(&self) -> usize {
        self.lines.len()
    }

    /// Get a line by index.
    #[inline]
    pub fn get_line(&self, index: usize) -> Option<Line2D> {
        self.lines.get(index)
    }

    /// Get all lines as a vector.
    pub fn lines(&self) -> Vec<Line2D> {
        self.lines.to_lines()
    }

    /// Get reference to the line collection (SoA layout).
    #[inline]
    pub fn line_collection(&self) -> &LineCollection {
        &self.lines
    }

    /// Get mutable reference to the line collection.
    #[inline]
    pub fn line_collection_mut(&mut self) -> &mut LineCollection {
        &mut self.lines
    }

    /// Iterate over lines.
    pub fn iter_lines(&self) -> impl Iterator<Item = Line2D> + '_ {
        self.lines.iter()
    }

    /// Remove a line by index (swap-remove).
    pub fn remove_line(&mut self, index: usize) {
        self.lines.swap_remove(index);
        // Note: This invalidates corner line indices!
        // Caller should update or clear corners after removing lines.
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Corner operations
    // ─────────────────────────────────────────────────────────────────────────

    /// Add a corner to the feature set.
    #[inline]
    pub fn add_corner(&mut self, corner: Corner2D) {
        self.corners.push_corner(&corner);
    }

    /// Number of corners.
    #[inline]
    pub fn corner_count(&self) -> usize {
        self.corners.len()
    }

    /// Get a corner by index.
    #[inline]
    pub fn get_corner(&self, index: usize) -> Option<Corner2D> {
        self.corners.get(index)
    }

    /// Get all corners as a vector.
    pub fn corners(&self) -> Vec<Corner2D> {
        self.corners.to_corners()
    }

    /// Get reference to the corner collection.
    #[inline]
    pub fn corner_collection(&self) -> &CornerCollection {
        &self.corners
    }

    /// Get mutable reference to the corner collection.
    #[inline]
    pub fn corner_collection_mut(&mut self) -> &mut CornerCollection {
        &mut self.corners
    }

    /// Iterate over corners.
    pub fn iter_corners(&self) -> impl Iterator<Item = Corner2D> + '_ {
        self.corners.iter()
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Combined operations
    // ─────────────────────────────────────────────────────────────────────────

    /// Total number of features (lines + corners).
    #[inline]
    pub fn total_count(&self) -> usize {
        self.line_count() + self.corner_count()
    }

    /// Check if the feature set is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.lines.is_empty() && self.corners.is_empty()
    }

    /// Transform all features by a pose.
    pub fn transform(&self, pose: &Pose2D) -> Self {
        Self {
            lines: self.lines.transform(pose),
            corners: {
                let mut transformed = CornerCollection::with_capacity(self.corners.len());
                for corner in self.corners.iter() {
                    transformed.push_corner(&corner.transform(pose));
                }
                transformed
            },
        }
    }

    /// Compute bounding box of all features.
    pub fn bounds(&self) -> Bounds {
        let mut bounds = Bounds::empty();

        // Include all line endpoints
        for i in 0..self.lines.len() {
            bounds.expand_to_include(Point2D::new(self.lines.start_xs[i], self.lines.start_ys[i]));
            bounds.expand_to_include(Point2D::new(self.lines.end_xs[i], self.lines.end_ys[i]));
        }

        // Include all corner positions
        for i in 0..self.corners.len() {
            bounds.expand_to_include(Point2D::new(self.corners.xs[i], self.corners.ys[i]));
        }

        bounds
    }

    /// Clear all features.
    pub fn clear(&mut self) {
        self.lines.clear();
        self.corners.clear();
    }

    /// Clear only corners (useful after modifying lines).
    pub fn clear_corners(&mut self) {
        self.corners.clear();
    }

    /// Find the nearest line to a point.
    ///
    /// Returns (index, distance) or None if no lines.
    #[inline]
    pub fn nearest_line(&self, point: Point2D) -> Option<(usize, f32)> {
        self.lines.nearest_line(point)
    }

    /// Find the nearest corner to a point.
    ///
    /// Returns (index, distance) or None if no corners.
    #[inline]
    pub fn nearest_corner(&self, point: Point2D) -> Option<(usize, f32)> {
        self.corners.nearest_corner(point)
    }

    /// Compute distances from a point to all lines.
    #[inline]
    pub fn distances_to_lines(&self, point: Point2D) -> Vec<f32> {
        self.lines.distances_to_point(point)
    }

    /// Merge another feature set into this one.
    ///
    /// Note: This is a simple append. For intelligent merging with
    /// association and deduplication, use the integration module.
    pub fn append(&mut self, other: &FeatureSet) {
        for line in other.lines.iter() {
            self.add_line(line);
        }
        for corner in other.corners.iter() {
            self.add_corner(corner);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn test_new_empty() {
        let fs = FeatureSet::new();
        assert!(fs.is_empty());
        assert_eq!(fs.line_count(), 0);
        assert_eq!(fs.corner_count(), 0);
        assert_eq!(fs.total_count(), 0);
    }

    #[test]
    fn test_add_line() {
        let mut fs = FeatureSet::new();
        fs.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)));
        fs.add_line_from_points(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0));

        assert_eq!(fs.line_count(), 2);
        assert!(!fs.is_empty());

        let line = fs.get_line(0).unwrap();
        assert_eq!(line.start, Point2D::new(0.0, 0.0));
    }

    #[test]
    fn test_add_corner() {
        let mut fs = FeatureSet::new();
        fs.add_corner(Corner2D::new(Point2D::new(5.0, 5.0), 0, 1, FRAC_PI_2));

        assert_eq!(fs.corner_count(), 1);

        let corner = fs.get_corner(0).unwrap();
        assert_eq!(corner.position, Point2D::new(5.0, 5.0));
    }

    #[test]
    fn test_from_features() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0)),
        ];
        let corners = vec![Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2)];

        let fs = FeatureSet::from_features(&lines, &corners);

        assert_eq!(fs.line_count(), 2);
        assert_eq!(fs.corner_count(), 1);
        assert_eq!(fs.total_count(), 3);
    }

    #[test]
    fn test_transform() {
        let mut fs = FeatureSet::new();
        fs.add_line(Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(2.0, 0.0)));
        fs.add_corner(Corner2D::new(Point2D::new(1.0, 0.0), 0, 0, FRAC_PI_2));

        let pose = Pose2D::new(0.0, 0.0, FRAC_PI_2);
        let transformed = fs.transform(&pose);

        let line = transformed.get_line(0).unwrap();
        assert_relative_eq!(line.start.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(line.start.y, 1.0, epsilon = 1e-6);

        let corner = transformed.get_corner(0).unwrap();
        assert_relative_eq!(corner.position.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(corner.position.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_bounds() {
        let mut fs = FeatureSet::new();
        fs.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)));
        fs.add_line(Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0)));

        let bounds = fs.bounds();

        assert_eq!(bounds.min, Point2D::new(0.0, -5.0));
        assert_eq!(bounds.max, Point2D::new(10.0, 5.0));
    }

    #[test]
    fn test_bounds_empty() {
        let fs = FeatureSet::new();
        let bounds = fs.bounds();
        assert!(bounds.is_empty());
    }

    #[test]
    fn test_nearest_line() {
        let mut fs = FeatureSet::new();
        fs.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)));
        fs.add_line(Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(10.0, 5.0)));

        let point = Point2D::new(5.0, 2.0);
        let (idx, dist) = fs.nearest_line(point).unwrap();

        assert_eq!(idx, 0);
        assert_relative_eq!(dist, 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_iter_lines() {
        let mut fs = FeatureSet::new();
        fs.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)));
        fs.add_line(Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0)));

        let lines: Vec<_> = fs.iter_lines().collect();
        assert_eq!(lines.len(), 2);
    }

    #[test]
    fn test_append() {
        let mut fs1 = FeatureSet::new();
        fs1.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)));

        let mut fs2 = FeatureSet::new();
        fs2.add_line(Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0)));
        fs2.add_corner(Corner2D::new(Point2D::new(0.0, 0.0), 0, 0, FRAC_PI_2));

        fs1.append(&fs2);

        assert_eq!(fs1.line_count(), 2);
        assert_eq!(fs1.corner_count(), 1);
    }

    #[test]
    fn test_clear() {
        let mut fs = FeatureSet::new();
        fs.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)));
        fs.add_corner(Corner2D::new(Point2D::new(0.0, 0.0), 0, 0, FRAC_PI_2));

        fs.clear();

        assert!(fs.is_empty());
        assert_eq!(fs.line_count(), 0);
        assert_eq!(fs.corner_count(), 0);
    }
}
