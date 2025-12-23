//! Axis-aligned bounding box for spatial operations.
//!
//! [`Bounds`] represents a rectangular region in 2D space, commonly used for:
//! - Map extent tracking (how large is the explored area)
//! - Spatial queries (is a point inside a region)
//! - Collision detection (do two regions overlap)
//! - Viewport clipping (what's visible on screen)
//!
//! # Usage
//!
//! ```rust
//! use vastu_map::core::{Bounds, Point2D};
//!
//! // Create bounds from corner points
//! let bounds = Bounds::new(
//!     Point2D::new(0.0, 0.0),   // min corner
//!     Point2D::new(10.0, 8.0),  // max corner
//! );
//!
//! // Query properties
//! assert_eq!(bounds.width(), 10.0);
//! assert_eq!(bounds.height(), 8.0);
//! assert!(bounds.contains(Point2D::new(5.0, 4.0)));
//!
//! // Expand dynamically as robot explores
//! let mut map_bounds = Bounds::empty();
//! map_bounds.expand_to_include(Point2D::new(1.0, 1.0));
//! map_bounds.expand_to_include(Point2D::new(-2.0, 3.0));
//! // Now bounds: min=(-2, 1), max=(1, 3)
//! ```

use super::point::Point2D;

/// Axis-aligned bounding box.
///
/// Represents a rectangular region in 2D space.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Bounds {
    /// Minimum corner (smallest x and y values).
    pub min: Point2D,
    /// Maximum corner (largest x and y values).
    pub max: Point2D,
}

impl Bounds {
    /// Create a new bounding box from min and max corners.
    #[inline]
    pub const fn new(min: Point2D, max: Point2D) -> Self {
        Self { min, max }
    }

    /// Create an empty (invalid) bounding box.
    ///
    /// The empty bounds has min > max, so it will expand to fit any point.
    #[inline]
    pub fn empty() -> Self {
        Self {
            min: Point2D::new(f32::INFINITY, f32::INFINITY),
            max: Point2D::new(f32::NEG_INFINITY, f32::NEG_INFINITY),
        }
    }

    /// Check if the bounds are empty (invalid).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y
    }

    /// Create bounds from a single point.
    #[inline]
    pub fn from_point(point: Point2D) -> Self {
        Self {
            min: point,
            max: point,
        }
    }

    /// Width of the bounding box (x extent).
    #[inline]
    pub fn width(&self) -> f32 {
        self.max.x - self.min.x
    }

    /// Height of the bounding box (y extent).
    #[inline]
    pub fn height(&self) -> f32 {
        self.max.y - self.min.y
    }

    /// Size of the bounding box as a Point2D (width, height).
    #[inline]
    pub fn size(&self) -> Point2D {
        Point2D::new(self.width(), self.height())
    }

    /// Center of the bounding box.
    #[inline]
    pub fn center(&self) -> Point2D {
        Point2D::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
        )
    }

    /// Check if a point is inside the bounding box.
    #[inline]
    pub fn contains(&self, point: Point2D) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
    }

    /// Check if this bounds intersects with another.
    #[inline]
    pub fn intersects(&self, other: &Bounds) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    /// Compute the intersection of two bounds.
    ///
    /// Returns empty bounds if they don't intersect.
    #[inline]
    pub fn intersection(&self, other: &Bounds) -> Self {
        Self {
            min: self.min.max(other.min),
            max: self.max.min(other.max),
        }
    }

    /// Compute the union of two bounds (smallest box containing both).
    #[inline]
    pub fn union(&self, other: &Bounds) -> Self {
        if self.is_empty() {
            return *other;
        }
        if other.is_empty() {
            return *self;
        }
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Expand bounds to include a point.
    #[inline]
    pub fn expand_to_include(&mut self, point: Point2D) {
        self.min = self.min.min(point);
        self.max = self.max.max(point);
    }

    /// Expand bounds by a margin on all sides.
    #[inline]
    pub fn expand(&self, margin: f32) -> Self {
        Self {
            min: Point2D::new(self.min.x - margin, self.min.y - margin),
            max: Point2D::new(self.max.x + margin, self.max.y + margin),
        }
    }

    /// Clamp a point to be inside the bounds.
    #[inline]
    pub fn clamp(&self, point: Point2D) -> Point2D {
        Point2D::new(
            point.x.max(self.min.x).min(self.max.x),
            point.y.max(self.min.y).min(self.max.y),
        )
    }

    /// Get corners of the bounding box.
    ///
    /// Returns [min, (max.x, min.y), max, (min.x, max.y)] in CCW order.
    #[inline]
    pub fn corners(&self) -> [Point2D; 4] {
        [
            self.min,
            Point2D::new(self.max.x, self.min.y),
            self.max,
            Point2D::new(self.min.x, self.max.y),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let bounds = Bounds::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 20.0));

        assert_eq!(bounds.min, Point2D::new(0.0, 0.0));
        assert_eq!(bounds.max, Point2D::new(10.0, 20.0));
    }

    #[test]
    fn test_empty() {
        let bounds = Bounds::empty();
        assert!(bounds.is_empty());

        let valid = Bounds::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 1.0));
        assert!(!valid.is_empty());
    }

    #[test]
    fn test_from_point() {
        let bounds = Bounds::from_point(Point2D::new(5.0, 10.0));

        assert_eq!(bounds.min, Point2D::new(5.0, 10.0));
        assert_eq!(bounds.max, Point2D::new(5.0, 10.0));
        assert_eq!(bounds.width(), 0.0);
        assert_eq!(bounds.height(), 0.0);
    }

    #[test]
    fn test_dimensions() {
        let bounds = Bounds::new(Point2D::new(1.0, 2.0), Point2D::new(5.0, 8.0));

        assert_eq!(bounds.width(), 4.0);
        assert_eq!(bounds.height(), 6.0);
        assert_eq!(bounds.size(), Point2D::new(4.0, 6.0));
        assert_eq!(bounds.center(), Point2D::new(3.0, 5.0));
    }

    #[test]
    fn test_contains() {
        let bounds = Bounds::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0));

        assert!(bounds.contains(Point2D::new(5.0, 5.0)));
        assert!(bounds.contains(Point2D::new(0.0, 0.0))); // Edge
        assert!(bounds.contains(Point2D::new(10.0, 10.0))); // Edge
        assert!(!bounds.contains(Point2D::new(-1.0, 5.0)));
        assert!(!bounds.contains(Point2D::new(5.0, 11.0)));
    }

    #[test]
    fn test_intersects() {
        let a = Bounds::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0));
        let b = Bounds::new(Point2D::new(5.0, 5.0), Point2D::new(15.0, 15.0));
        let c = Bounds::new(Point2D::new(20.0, 20.0), Point2D::new(30.0, 30.0));

        assert!(a.intersects(&b));
        assert!(b.intersects(&a));
        assert!(!a.intersects(&c));
        assert!(!c.intersects(&a));
    }

    #[test]
    fn test_intersection() {
        let a = Bounds::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0));
        let b = Bounds::new(Point2D::new(5.0, 5.0), Point2D::new(15.0, 15.0));

        let inter = a.intersection(&b);

        assert_eq!(inter.min, Point2D::new(5.0, 5.0));
        assert_eq!(inter.max, Point2D::new(10.0, 10.0));
    }

    #[test]
    fn test_union() {
        let a = Bounds::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0));
        let b = Bounds::new(Point2D::new(5.0, 5.0), Point2D::new(15.0, 15.0));

        let u = a.union(&b);

        assert_eq!(u.min, Point2D::new(0.0, 0.0));
        assert_eq!(u.max, Point2D::new(15.0, 15.0));
    }

    #[test]
    fn test_union_with_empty() {
        let a = Bounds::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0));
        let empty = Bounds::empty();

        assert_eq!(a.union(&empty), a);
        assert_eq!(empty.union(&a), a);
    }

    #[test]
    fn test_expand_to_include() {
        let mut bounds = Bounds::empty();

        bounds.expand_to_include(Point2D::new(5.0, 5.0));
        assert_eq!(bounds.min, Point2D::new(5.0, 5.0));
        assert_eq!(bounds.max, Point2D::new(5.0, 5.0));

        bounds.expand_to_include(Point2D::new(0.0, 10.0));
        assert_eq!(bounds.min, Point2D::new(0.0, 5.0));
        assert_eq!(bounds.max, Point2D::new(5.0, 10.0));
    }

    #[test]
    fn test_expand() {
        let bounds = Bounds::new(Point2D::new(5.0, 5.0), Point2D::new(10.0, 10.0));
        let expanded = bounds.expand(2.0);

        assert_eq!(expanded.min, Point2D::new(3.0, 3.0));
        assert_eq!(expanded.max, Point2D::new(12.0, 12.0));
    }

    #[test]
    fn test_clamp() {
        let bounds = Bounds::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0));

        assert_eq!(bounds.clamp(Point2D::new(5.0, 5.0)), Point2D::new(5.0, 5.0));
        assert_eq!(
            bounds.clamp(Point2D::new(-5.0, 5.0)),
            Point2D::new(0.0, 5.0)
        );
        assert_eq!(
            bounds.clamp(Point2D::new(15.0, 15.0)),
            Point2D::new(10.0, 10.0)
        );
    }

    #[test]
    fn test_corners() {
        let bounds = Bounds::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 20.0));
        let corners = bounds.corners();

        assert_eq!(corners[0], Point2D::new(0.0, 0.0));
        assert_eq!(corners[1], Point2D::new(10.0, 0.0));
        assert_eq!(corners[2], Point2D::new(10.0, 20.0));
        assert_eq!(corners[3], Point2D::new(0.0, 20.0));
    }
}
