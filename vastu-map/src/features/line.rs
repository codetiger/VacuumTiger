//! Line segment type using endpoint representation.
//!
//! Lines are represented by their start and end points only (no parametric form).
//! This design choice simplifies transforms and enables better SIMD compatibility.

use crate::core::{Point2D, Pose2D};

/// A 2D line segment defined by its endpoints.
///
/// # Design Decision: Endpoint-Only Representation
///
/// We use endpoints instead of parametric (ρ,θ) form because:
/// - **Simpler transforms**: Just rotate+translate two points
/// - **Better SIMD layout**: 4 homogeneous floats per line
/// - **No angle wrapping**: No discontinuity at ±π
/// - **Built-in bounds**: Segment extent is implicit
///
/// # Point-to-Line Distance
///
/// Uses cross product formula (no trig functions):
/// ```text
/// distance = |cross(p - start, direction)| / |direction|
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Line2D {
    /// Start point of the line segment.
    pub start: Point2D,
    /// End point of the line segment.
    pub end: Point2D,
    /// Number of times this line has been observed (for merging).
    pub observation_count: u32,
    /// Number of scan points that formed this line (for dominance).
    pub point_count: u32,
}

impl Line2D {
    /// Create a new line segment from two points.
    #[inline]
    pub fn new(start: Point2D, end: Point2D) -> Self {
        Self {
            start,
            end,
            observation_count: 1,
            point_count: 0,
        }
    }

    /// Create a line segment with a specific observation count.
    #[inline]
    pub fn with_observation_count(start: Point2D, end: Point2D, count: u32) -> Self {
        Self {
            start,
            end,
            observation_count: count,
            point_count: 0,
        }
    }

    /// Create a line segment with a specific point count.
    ///
    /// Used when extracting lines from scan points where point_count
    /// indicates how many scan points formed this line.
    #[inline]
    pub fn with_point_count(start: Point2D, end: Point2D, point_count: u32) -> Self {
        Self {
            start,
            end,
            observation_count: 1,
            point_count,
        }
    }

    /// Create a line segment with all fields specified.
    #[inline]
    pub fn full(start: Point2D, end: Point2D, observation_count: u32, point_count: u32) -> Self {
        Self {
            start,
            end,
            observation_count,
            point_count,
        }
    }

    /// Direction vector from start to end (not normalized).
    #[inline]
    pub fn direction(&self) -> Point2D {
        self.end - self.start
    }

    /// Unit direction vector from start to end.
    #[inline]
    pub fn unit_direction(&self) -> Point2D {
        self.direction().normalized()
    }

    /// Normal vector (perpendicular to direction, pointing left).
    ///
    /// If the line goes from left to right, the normal points up.
    #[inline]
    pub fn normal(&self) -> Point2D {
        self.unit_direction().perpendicular()
    }

    /// Length of the line segment.
    #[inline]
    pub fn length(&self) -> f32 {
        self.direction().length()
    }

    /// Squared length of the line segment (avoids sqrt).
    #[inline]
    pub fn length_squared(&self) -> f32 {
        self.direction().length_squared()
    }

    /// Midpoint of the line segment.
    #[inline]
    pub fn midpoint(&self) -> Point2D {
        Point2D::new(
            (self.start.x + self.end.x) * 0.5,
            (self.start.y + self.end.y) * 0.5,
        )
    }

    /// Get a point along the line at parameter t.
    ///
    /// - `t = 0`: returns start point
    /// - `t = 1`: returns end point
    /// - `t = 0.5`: returns midpoint
    #[inline]
    pub fn point_at(&self, t: f32) -> Point2D {
        Point2D::new(
            self.start.x + t * (self.end.x - self.start.x),
            self.start.y + t * (self.end.y - self.start.y),
        )
    }

    /// Get a perpendicular line at parameter t along this line.
    ///
    /// Returns a unit-length line perpendicular to this line,
    /// centered at `point_at(t)`.
    ///
    /// # Arguments
    /// * `t` - Parameter along line (0=start, 1=end, 0.5=midpoint)
    /// * `length` - Length of the perpendicular line (centered on the point)
    ///
    /// # Example
    /// ```rust,ignore
    /// let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(4.0, 0.0));
    /// let perp = line.perpendicular_at(0.5, 2.0);
    /// // perp goes from (2, -1) to (2, 1) - perpendicular at midpoint
    /// ```
    #[inline]
    pub fn perpendicular_at(&self, t: f32, length: f32) -> Line2D {
        let center = self.point_at(t);
        let normal = self.normal();
        let half_len = length * 0.5;

        Line2D::new(
            Point2D::new(
                center.x - normal.x * half_len,
                center.y - normal.y * half_len,
            ),
            Point2D::new(
                center.x + normal.x * half_len,
                center.y + normal.y * half_len,
            ),
        )
    }

    /// Extend this line by a factor in both directions.
    ///
    /// # Arguments
    /// * `factor` - Extension factor (1.0 = no change, 2.0 = double length)
    ///
    /// The line is extended symmetrically from its midpoint.
    ///
    /// # Example
    /// ```rust,ignore
    /// let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(2.0, 0.0));
    /// let extended = line.extended(2.0);
    /// // extended goes from (-1, 0) to (3, 0) - doubled in length
    /// ```
    #[inline]
    pub fn extended(&self, factor: f32) -> Line2D {
        let mid = self.midpoint();
        let half_dir = Point2D::new(
            (self.end.x - self.start.x) * 0.5 * factor,
            (self.end.y - self.start.y) * 0.5 * factor,
        );

        Line2D::full(
            Point2D::new(mid.x - half_dir.x, mid.y - half_dir.y),
            Point2D::new(mid.x + half_dir.x, mid.y + half_dir.y),
            self.observation_count,
            self.point_count,
        )
    }

    /// Angle of the line direction from X-axis (in radians, [-π, π]).
    #[inline]
    pub fn angle(&self) -> f32 {
        self.direction().angle()
    }

    /// Perpendicular distance from a point to the infinite line.
    ///
    /// Uses the cross product formula:
    /// ```text
    /// distance = |cross(p - start, end - start)| / |end - start|
    /// ```
    ///
    /// This is always non-negative.
    #[inline]
    pub fn distance_to_point(&self, point: Point2D) -> f32 {
        let dir = self.direction();
        let len_sq = dir.length_squared();

        if len_sq < f32::EPSILON {
            // Degenerate line (zero length)
            return point.distance(self.start);
        }

        let to_point = point - self.start;
        (to_point.cross(dir)).abs() / len_sq.sqrt()
    }

    /// Signed perpendicular distance from a point to the infinite line.
    ///
    /// Positive if the point is to the left of the line (in direction of normal),
    /// negative if to the right.
    #[inline]
    pub fn signed_distance_to_point(&self, point: Point2D) -> f32 {
        let dir = self.direction();
        let len_sq = dir.length_squared();

        if len_sq < f32::EPSILON {
            return point.distance(self.start);
        }

        let to_point = point - self.start;
        // Cross product: dir × to_point gives positive when point is to the left
        dir.cross(to_point) / len_sq.sqrt()
    }

    /// Project a point onto the infinite line, returning parameter t.
    ///
    /// The projection point is: `start + t * (end - start)`
    /// - `t = 0`: projection is at start
    /// - `t = 1`: projection is at end
    /// - `0 < t < 1`: projection is between endpoints
    /// - `t < 0` or `t > 1`: projection is outside segment
    #[inline]
    pub fn project_point(&self, point: Point2D) -> f32 {
        let dir = self.direction();
        let len_sq = dir.length_squared();

        if len_sq < f32::EPSILON {
            return 0.0;
        }

        let to_point = point - self.start;
        to_point.dot(dir) / len_sq
    }

    /// Get the projected point on the infinite line.
    #[inline]
    pub fn project_point_onto_line(&self, point: Point2D) -> Point2D {
        let t = self.project_point(point);
        self.point_at(t)
    }

    /// Check if a projection parameter t falls within the segment [0, 1].
    #[inline]
    pub fn contains_projection(&self, t: f32) -> bool {
        (0.0..=1.0).contains(&t)
    }

    /// Check if a projection parameter t falls within the segment with extension tolerance.
    ///
    /// Allows points to project slightly beyond line endpoints by `extension` amount.
    /// The valid range becomes `[-extension, 1 + extension]`.
    ///
    /// # Arguments
    /// * `t` - Projection parameter (0 = start, 1 = end)
    /// * `extension` - Tolerance for projection beyond endpoints (e.g., 0.2 = 20% extension)
    ///
    /// # Example
    /// ```rust,ignore
    /// let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));
    /// let t = line.project_point(Point2D::new(-1.0, 0.5)); // t ≈ -0.1
    ///
    /// assert!(!line.contains_projection(t));              // Outside [0, 1]
    /// assert!(line.contains_projection_extended(t, 0.2)); // Inside [-0.2, 1.2]
    /// ```
    #[inline]
    pub fn contains_projection_extended(&self, t: f32, extension: f32) -> bool {
        t >= -extension && t <= 1.0 + extension
    }

    /// Distance from a point to the line segment (not infinite line).
    ///
    /// Returns the minimum distance to any point on the segment.
    #[inline]
    pub fn distance_to_point_segment(&self, point: Point2D) -> f32 {
        let t = self.project_point(point).clamp(0.0, 1.0);
        point.distance(self.point_at(t))
    }

    /// Transform line by a pose (rotate and translate both endpoints).
    #[inline]
    pub fn transform(&self, pose: &Pose2D) -> Self {
        Self {
            start: pose.transform_point(self.start),
            end: pose.transform_point(self.end),
            observation_count: self.observation_count,
            point_count: self.point_count,
        }
    }

    /// Compute the overlap between two line segments.
    ///
    /// Returns the length of the overlap when projected onto this line.
    /// Lines must be roughly parallel for meaningful overlap.
    ///
    /// Returns 0 if lines don't overlap or aren't parallel.
    pub fn overlap_length(&self, other: &Line2D, angle_tolerance: f32) -> f32 {
        use crate::core::math::angle_diff;

        // Check if lines are roughly parallel
        let angle_diff = angle_diff(self.angle(), other.angle()).abs();
        if angle_diff > angle_tolerance && angle_diff < std::f32::consts::PI - angle_tolerance {
            return 0.0;
        }

        // Project other line's endpoints onto this line
        let t1 = self.project_point(other.start);
        let t2 = self.project_point(other.end);

        // Find overlap in parameter space
        let (t_min, t_max) = if t1 < t2 { (t1, t2) } else { (t2, t1) };

        // Clamp to [0, 1] (this line's extent)
        let overlap_start = t_min.max(0.0);
        let overlap_end = t_max.min(1.0);

        if overlap_end <= overlap_start {
            return 0.0;
        }

        (overlap_end - overlap_start) * self.length()
    }

    /// Check if a ray intersects this line segment.
    ///
    /// Returns the distance along the ray to the intersection point,
    /// or None if no intersection.
    ///
    /// # Arguments
    /// * `ray_origin` - Origin of the ray
    /// * `ray_direction` - Direction of the ray (will be normalized internally)
    pub fn ray_intersection(&self, ray_origin: Point2D, ray_direction: Point2D) -> Option<f32> {
        let dir = ray_direction.normalized();
        let seg = self.direction();

        // Use parametric form: ray_origin + t*dir = start + s*seg
        // Solve using cross products
        let cross_dir_seg = dir.cross(seg);

        if cross_dir_seg.abs() < f32::EPSILON {
            // Parallel lines
            return None;
        }

        let origin_to_start = self.start - ray_origin;
        let t = origin_to_start.cross(seg) / cross_dir_seg;
        let s = origin_to_start.cross(dir) / cross_dir_seg;

        // t must be positive (forward along ray)
        // s must be in [0, 1] (within segment)
        if t >= 0.0 && (0.0..=1.0).contains(&s) {
            Some(t)
        } else {
            None
        }
    }

    /// Find intersection point with another line segment.
    ///
    /// Returns the intersection point if the segments intersect,
    /// None otherwise.
    pub fn intersection(&self, other: &Line2D) -> Option<Point2D> {
        let d1 = self.direction();
        let d2 = other.direction();

        let cross = d1.cross(d2);
        if cross.abs() < f32::EPSILON {
            // Parallel lines
            return None;
        }

        let origin_diff = other.start - self.start;
        let t = origin_diff.cross(d2) / cross;
        let s = origin_diff.cross(d1) / cross;

        if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&s) {
            Some(self.point_at(t))
        } else {
            None
        }
    }

    /// Check if this line approximately equals another.
    #[inline]
    pub fn approx_eq(&self, other: &Line2D, epsilon: f32) -> bool {
        self.start.approx_eq(other.start, epsilon) && self.end.approx_eq(other.end, epsilon)
    }
}

impl Default for Line2D {
    fn default() -> Self {
        Self {
            start: Point2D::zero(),
            end: Point2D::zero(),
            observation_count: 0,
            point_count: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};

    #[test]
    fn test_new() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0));
        assert_eq!(line.start, Point2D::new(0.0, 0.0));
        assert_eq!(line.end, Point2D::new(1.0, 0.0));
        assert_eq!(line.observation_count, 1);
    }

    #[test]
    fn test_direction_and_length() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(3.0, 4.0));

        assert_eq!(line.direction(), Point2D::new(3.0, 4.0));
        assert_eq!(line.length(), 5.0);
        assert_eq!(line.length_squared(), 25.0);

        let unit = line.unit_direction();
        assert_relative_eq!(unit.x, 0.6, epsilon = 1e-6);
        assert_relative_eq!(unit.y, 0.8, epsilon = 1e-6);
    }

    #[test]
    fn test_midpoint() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(4.0, 6.0));
        let mid = line.midpoint();
        assert_eq!(mid.x, 2.0);
        assert_eq!(mid.y, 3.0);
    }

    #[test]
    fn test_angle() {
        let horizontal = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0));
        assert_relative_eq!(horizontal.angle(), 0.0, epsilon = 1e-6);

        let vertical = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 1.0));
        assert_relative_eq!(vertical.angle(), FRAC_PI_2, epsilon = 1e-6);

        let diagonal = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 1.0));
        assert_relative_eq!(diagonal.angle(), FRAC_PI_4, epsilon = 1e-6);
    }

    #[test]
    fn test_distance_to_point() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        // Point above the line
        let dist = line.distance_to_point(Point2D::new(5.0, 3.0));
        assert_relative_eq!(dist, 3.0, epsilon = 1e-6);

        // Point below the line
        let dist = line.distance_to_point(Point2D::new(5.0, -4.0));
        assert_relative_eq!(dist, 4.0, epsilon = 1e-6);

        // Point on the line
        let dist = line.distance_to_point(Point2D::new(5.0, 0.0));
        assert_relative_eq!(dist, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_signed_distance() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        // Point above (left of line direction) - positive
        let dist = line.signed_distance_to_point(Point2D::new(5.0, 3.0));
        assert_relative_eq!(dist, 3.0, epsilon = 1e-6);

        // Point below (right of line direction) - negative
        let dist = line.signed_distance_to_point(Point2D::new(5.0, -3.0));
        assert_relative_eq!(dist, -3.0, epsilon = 1e-6);
    }

    #[test]
    fn test_project_point() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        // Project point to midpoint
        let t = line.project_point(Point2D::new(5.0, 3.0));
        assert_relative_eq!(t, 0.5, epsilon = 1e-6);

        // Project point at start
        let t = line.project_point(Point2D::new(0.0, 5.0));
        assert_relative_eq!(t, 0.0, epsilon = 1e-6);

        // Project point beyond end
        let t = line.project_point(Point2D::new(15.0, 0.0));
        assert_relative_eq!(t, 1.5, epsilon = 1e-6);

        // Project point before start
        let t = line.project_point(Point2D::new(-5.0, 0.0));
        assert_relative_eq!(t, -0.5, epsilon = 1e-6);
    }

    #[test]
    fn test_contains_projection() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        assert!(line.contains_projection(0.0));
        assert!(line.contains_projection(0.5));
        assert!(line.contains_projection(1.0));
        assert!(!line.contains_projection(-0.1));
        assert!(!line.contains_projection(1.1));
    }

    #[test]
    fn test_contains_projection_extended() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        // With 0.2 extension, valid range is [-0.2, 1.2]
        assert!(line.contains_projection_extended(0.0, 0.2));
        assert!(line.contains_projection_extended(0.5, 0.2));
        assert!(line.contains_projection_extended(1.0, 0.2));
        assert!(line.contains_projection_extended(-0.1, 0.2)); // Within extension
        assert!(line.contains_projection_extended(1.1, 0.2)); // Within extension
        assert!(!line.contains_projection_extended(-0.3, 0.2)); // Beyond extension
        assert!(!line.contains_projection_extended(1.3, 0.2)); // Beyond extension

        // Zero extension = strict [0, 1]
        assert!(!line.contains_projection_extended(-0.1, 0.0));
        assert!(!line.contains_projection_extended(1.1, 0.0));
    }

    #[test]
    fn test_point_at() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        assert_eq!(line.point_at(0.0), line.start);
        assert_eq!(line.point_at(1.0), line.end);
        assert_eq!(line.point_at(0.5), Point2D::new(5.0, 0.0));
    }

    #[test]
    fn test_perpendicular_at() {
        // Horizontal line
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(4.0, 0.0));

        // Perpendicular at midpoint with length 2
        let perp = line.perpendicular_at(0.5, 2.0);

        // Center should be at (2, 0)
        assert_relative_eq!(perp.midpoint().x, 2.0, epsilon = 1e-6);
        assert_relative_eq!(perp.midpoint().y, 0.0, epsilon = 1e-6);

        // Should be vertical (length 2)
        assert_relative_eq!(perp.length(), 2.0, epsilon = 1e-6);

        // Endpoints should be at (2, -1) and (2, 1)
        assert_relative_eq!(perp.start.x, 2.0, epsilon = 1e-6);
        assert_relative_eq!(perp.end.x, 2.0, epsilon = 1e-6);
        assert_relative_eq!((perp.start.y - perp.end.y).abs(), 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_extended() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(2.0, 0.0));

        // Double the length
        let ext = line.extended(2.0);

        // Midpoint should be same
        assert_relative_eq!(ext.midpoint().x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(ext.midpoint().y, 0.0, epsilon = 1e-6);

        // Length should be doubled
        assert_relative_eq!(ext.length(), 4.0, epsilon = 1e-6);

        // Endpoints should be at (-1, 0) and (3, 0)
        assert_relative_eq!(ext.start.x, -1.0, epsilon = 1e-6);
        assert_relative_eq!(ext.end.x, 3.0, epsilon = 1e-6);

        // No change when factor is 1.0
        let same = line.extended(1.0);
        assert_relative_eq!(same.length(), 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_distance_to_point_segment() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        // Point above segment (closest to segment)
        let dist = line.distance_to_point_segment(Point2D::new(5.0, 3.0));
        assert_relative_eq!(dist, 3.0, epsilon = 1e-6);

        // Point beyond end (closest to endpoint)
        let dist = line.distance_to_point_segment(Point2D::new(13.0, 4.0));
        assert_relative_eq!(dist, 5.0, epsilon = 1e-6); // Distance to (10, 0)
    }

    #[test]
    fn test_transform() {
        let line = Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(2.0, 0.0));

        // Rotate 90 degrees at origin
        let pose = Pose2D::new(0.0, 0.0, FRAC_PI_2);
        let transformed = line.transform(&pose);

        assert_relative_eq!(transformed.start.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.start.y, 1.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.end.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.end.y, 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_ray_intersection() {
        let line = Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0));

        // Ray from origin pointing right
        let dist = line.ray_intersection(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0));
        assert_relative_eq!(dist.unwrap(), 5.0, epsilon = 1e-6);

        // Ray pointing away from line
        let dist = line.ray_intersection(Point2D::new(0.0, 0.0), Point2D::new(-1.0, 0.0));
        assert!(dist.is_none());

        // Ray parallel to line
        let dist = line.ray_intersection(Point2D::new(0.0, 0.0), Point2D::new(0.0, 1.0));
        assert!(dist.is_none());
    }

    #[test]
    fn test_segment_intersection() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));
        let line2 = Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0));

        let intersection = line1.intersection(&line2);
        assert!(intersection.is_some());
        let p = intersection.unwrap();
        assert_relative_eq!(p.x, 5.0, epsilon = 1e-6);
        assert_relative_eq!(p.y, 0.0, epsilon = 1e-6);

        // Non-intersecting segments
        let line3 = Line2D::new(Point2D::new(20.0, 0.0), Point2D::new(30.0, 0.0));
        assert!(line1.intersection(&line3).is_none());
    }

    #[test]
    fn test_overlap_length() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));
        let line2 = Line2D::new(Point2D::new(5.0, 0.1), Point2D::new(15.0, 0.1)); // Slightly offset, parallel

        let overlap = line1.overlap_length(&line2, 0.1);
        assert_relative_eq!(overlap, 5.0, epsilon = 0.1);

        // Non-overlapping
        let line3 = Line2D::new(Point2D::new(15.0, 0.0), Point2D::new(25.0, 0.0));
        let overlap = line1.overlap_length(&line3, 0.1);
        assert_eq!(overlap, 0.0);

        // Perpendicular lines (not parallel)
        let line4 = Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0));
        let overlap = line1.overlap_length(&line4, 0.1);
        assert_eq!(overlap, 0.0);
    }

    #[test]
    fn test_normal() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0));
        let normal = line.normal();

        // For a line pointing right, normal points up
        assert_relative_eq!(normal.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(normal.y, 1.0, epsilon = 1e-6);
    }
}
