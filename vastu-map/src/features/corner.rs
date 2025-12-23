//! Corner feature type representing line intersections.
//!
//! Corners are detected where two lines meet at a significant angle.

use crate::core::{Point2D, Pose2D};

/// A corner (vertex) feature detected at a line intersection.
///
/// Corners are stable features useful for localization because they're
/// viewpoint-invariant and easy to detect consistently.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Corner2D {
    /// Position of the corner in world coordinates.
    pub position: Point2D,
    /// Index of the first line forming this corner.
    pub line1_idx: usize,
    /// Index of the second line forming this corner.
    pub line2_idx: usize,
    /// Angle between the two lines (in radians, 0 to π).
    pub angle: f32,
    /// Number of times this corner has been observed.
    pub observation_count: u32,
}

impl Corner2D {
    /// Create a new corner.
    #[inline]
    pub fn new(position: Point2D, line1_idx: usize, line2_idx: usize, angle: f32) -> Self {
        Self {
            position,
            line1_idx,
            line2_idx,
            angle,
            observation_count: 1,
        }
    }

    /// Create a corner with a specific observation count.
    #[inline]
    pub fn with_observation_count(
        position: Point2D,
        line1_idx: usize,
        line2_idx: usize,
        angle: f32,
        count: u32,
    ) -> Self {
        Self {
            position,
            line1_idx,
            line2_idx,
            angle,
            observation_count: count,
        }
    }

    /// Transform corner by a pose.
    #[inline]
    pub fn transform(&self, pose: &Pose2D) -> Self {
        Self {
            position: pose.transform_point(self.position),
            line1_idx: self.line1_idx,
            line2_idx: self.line2_idx,
            angle: self.angle,
            observation_count: self.observation_count,
        }
    }

    /// Distance to another corner.
    #[inline]
    pub fn distance(&self, other: &Corner2D) -> f32 {
        self.position.distance(other.position)
    }

    /// Check if this corner is approximately equal to another.
    ///
    /// Compares position and angle, ignoring line indices.
    #[inline]
    pub fn approx_eq(&self, other: &Corner2D, pos_epsilon: f32, angle_epsilon: f32) -> bool {
        self.position.approx_eq(other.position, pos_epsilon)
            && (self.angle - other.angle).abs() <= angle_epsilon
    }
}

impl Default for Corner2D {
    fn default() -> Self {
        Self {
            position: Point2D::zero(),
            line1_idx: 0,
            line2_idx: 0,
            angle: 0.0,
            observation_count: 0,
        }
    }
}

/// Collection of corners with efficient storage.
#[derive(Clone, Debug, Default)]
pub struct CornerCollection {
    /// Positions of all corners (SoA: x coordinates).
    pub xs: Vec<f32>,
    /// Positions of all corners (SoA: y coordinates).
    pub ys: Vec<f32>,
    /// First line index for each corner.
    pub line1_indices: Vec<usize>,
    /// Second line index for each corner.
    pub line2_indices: Vec<usize>,
    /// Angles at each corner.
    pub angles: Vec<f32>,
    /// Observation counts.
    pub observation_counts: Vec<u32>,
}

impl CornerCollection {
    /// Create a new empty corner collection.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            xs: Vec::with_capacity(capacity),
            ys: Vec::with_capacity(capacity),
            line1_indices: Vec::with_capacity(capacity),
            line2_indices: Vec::with_capacity(capacity),
            angles: Vec::with_capacity(capacity),
            observation_counts: Vec::with_capacity(capacity),
        }
    }

    /// Create from a slice of Corner2D.
    pub fn from_corners(corners: &[Corner2D]) -> Self {
        let mut collection = Self::with_capacity(corners.len());
        for corner in corners {
            collection.push_corner(corner);
        }
        collection
    }

    /// Add a corner to the collection.
    #[inline]
    pub fn push_corner(&mut self, corner: &Corner2D) {
        self.xs.push(corner.position.x);
        self.ys.push(corner.position.y);
        self.line1_indices.push(corner.line1_idx);
        self.line2_indices.push(corner.line2_idx);
        self.angles.push(corner.angle);
        self.observation_counts.push(corner.observation_count);
    }

    /// Number of corners in the collection.
    #[inline]
    pub fn len(&self) -> usize {
        self.xs.len()
    }

    /// Check if the collection is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.xs.is_empty()
    }

    /// Get a corner by index.
    #[inline]
    pub fn get(&self, index: usize) -> Option<Corner2D> {
        if index < self.len() {
            Some(Corner2D::with_observation_count(
                Point2D::new(self.xs[index], self.ys[index]),
                self.line1_indices[index],
                self.line2_indices[index],
                self.angles[index],
                self.observation_counts[index],
            ))
        } else {
            None
        }
    }

    /// Convert to `Vec<Corner2D>`.
    pub fn to_corners(&self) -> Vec<Corner2D> {
        (0..self.len())
            .map(|i| {
                Corner2D::with_observation_count(
                    Point2D::new(self.xs[i], self.ys[i]),
                    self.line1_indices[i],
                    self.line2_indices[i],
                    self.angles[i],
                    self.observation_counts[i],
                )
            })
            .collect()
    }

    /// Find the nearest corner to a point.
    ///
    /// Returns (index, distance) of the nearest corner, or None if collection is empty.
    pub fn nearest_corner(&self, point: Point2D) -> Option<(usize, f32)> {
        if self.is_empty() {
            return None;
        }

        let mut min_idx = 0;
        let mut min_dist_sq = f32::MAX;

        for i in 0..self.len() {
            let dx = point.x - self.xs[i];
            let dy = point.y - self.ys[i];
            let dist_sq = dx * dx + dy * dy;

            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                min_idx = i;
            }
        }

        Some((min_idx, min_dist_sq.sqrt()))
    }

    /// Clear all corners.
    pub fn clear(&mut self) {
        self.xs.clear();
        self.ys.clear();
        self.line1_indices.clear();
        self.line2_indices.clear();
        self.angles.clear();
        self.observation_counts.clear();
    }

    /// Iterate over corners.
    pub fn iter(&self) -> impl Iterator<Item = Corner2D> + '_ {
        (0..self.len()).map(move |i| {
            Corner2D::with_observation_count(
                Point2D::new(self.xs[i], self.ys[i]),
                self.line1_indices[i],
                self.line2_indices[i],
                self.angles[i],
                self.observation_counts[i],
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};

    #[test]
    fn test_corner_new() {
        let corner = Corner2D::new(Point2D::new(1.0, 2.0), 0, 1, FRAC_PI_2);

        assert_eq!(corner.position, Point2D::new(1.0, 2.0));
        assert_eq!(corner.line1_idx, 0);
        assert_eq!(corner.line2_idx, 1);
        assert_relative_eq!(corner.angle, FRAC_PI_2, epsilon = 1e-6);
        assert_eq!(corner.observation_count, 1);
    }

    #[test]
    fn test_corner_transform() {
        let corner = Corner2D::new(Point2D::new(1.0, 0.0), 0, 1, FRAC_PI_2);
        let pose = Pose2D::new(0.0, 0.0, FRAC_PI_2);

        let transformed = corner.transform(&pose);

        assert_relative_eq!(transformed.position.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.position.y, 1.0, epsilon = 1e-6);
        // Angle stays the same (corner angle, not orientation)
        assert_relative_eq!(transformed.angle, FRAC_PI_2, epsilon = 1e-6);
    }

    #[test]
    fn test_corner_distance() {
        let c1 = Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2);
        let c2 = Corner2D::new(Point2D::new(3.0, 4.0), 2, 3, FRAC_PI_2);

        assert_eq!(c1.distance(&c2), 5.0);
    }

    #[test]
    fn test_corner_collection_from_corners() {
        let corners = vec![
            Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2),
            Corner2D::new(Point2D::new(1.0, 1.0), 1, 2, FRAC_PI_4),
        ];

        let collection = CornerCollection::from_corners(&corners);

        assert_eq!(collection.len(), 2);
        assert_eq!(collection.xs, vec![0.0, 1.0]);
        assert_eq!(collection.ys, vec![0.0, 1.0]);
    }

    #[test]
    fn test_corner_collection_to_corners() {
        let corners = vec![
            Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2),
            Corner2D::new(Point2D::new(1.0, 1.0), 1, 2, FRAC_PI_4),
        ];

        let collection = CornerCollection::from_corners(&corners);
        let recovered = collection.to_corners();

        assert_eq!(recovered.len(), 2);
        assert_eq!(recovered[0].position, corners[0].position);
        assert_eq!(recovered[1].position, corners[1].position);
    }

    #[test]
    fn test_corner_collection_nearest() {
        let corners = vec![
            Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2),
            Corner2D::new(Point2D::new(10.0, 0.0), 1, 2, FRAC_PI_2),
            Corner2D::new(Point2D::new(5.0, 5.0), 2, 3, FRAC_PI_2),
        ];

        let collection = CornerCollection::from_corners(&corners);
        let point = Point2D::new(4.0, 4.0);

        let (idx, dist) = collection.nearest_corner(point).unwrap();

        assert_eq!(idx, 2); // (5, 5) is nearest
        assert_relative_eq!(dist, 2.0_f32.sqrt(), epsilon = 1e-6);
    }

    #[test]
    fn test_corner_collection_iter() {
        let corners = vec![
            Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2),
            Corner2D::new(Point2D::new(1.0, 1.0), 1, 2, FRAC_PI_4),
        ];

        let collection = CornerCollection::from_corners(&corners);
        let collected: Vec<_> = collection.iter().collect();

        assert_eq!(collected.len(), 2);
        assert_eq!(collected[0].position, Point2D::new(0.0, 0.0));
        assert_eq!(collected[1].position, Point2D::new(1.0, 1.0));
    }
}
