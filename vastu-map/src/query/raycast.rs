//! Ray casting for obstacle detection.
//!
//! Computes the distance from a point in a given direction to the
//! first obstacle (line segment) in the map.

use crate::core::Point2D;
use crate::features::Line2D;

/// Result of a raycast operation.
#[derive(Clone, Debug)]
pub struct RaycastResult {
    /// Distance to the hit point, or max_range if no hit.
    pub distance: f32,
    /// Index of the line that was hit, if any.
    pub line_idx: Option<usize>,
    /// The hit point in world coordinates.
    pub hit_point: Option<Point2D>,
    /// Whether a hit occurred.
    pub hit: bool,
}

impl RaycastResult {
    /// Create a result indicating no hit.
    pub fn miss(max_range: f32) -> Self {
        Self {
            distance: max_range,
            line_idx: None,
            hit_point: None,
            hit: false,
        }
    }

    /// Create a result indicating a hit.
    pub fn hit(distance: f32, line_idx: usize, hit_point: Point2D) -> Self {
        Self {
            distance,
            line_idx: Some(line_idx),
            hit_point: Some(hit_point),
            hit: true,
        }
    }
}

/// Cast a ray and find the first intersection with map lines.
///
/// # Arguments
/// * `origin` - Ray origin in world frame
/// * `direction` - Ray direction (will be normalized)
/// * `max_range` - Maximum ray distance
/// * `lines` - Map line segments
///
/// # Returns
/// Distance to first intersection, or max_range if none.
pub fn raycast(origin: Point2D, direction: Point2D, max_range: f32, lines: &[Line2D]) -> f32 {
    raycast_detailed(origin, direction, max_range, lines).distance
}

/// Cast a ray with detailed hit information.
///
/// # Arguments
/// * `origin` - Ray origin in world frame
/// * `direction` - Ray direction (will be normalized)
/// * `max_range` - Maximum ray distance
/// * `lines` - Map line segments
///
/// # Returns
/// Detailed raycast result.
pub fn raycast_detailed(
    origin: Point2D,
    direction: Point2D,
    max_range: f32,
    lines: &[Line2D],
) -> RaycastResult {
    if lines.is_empty() {
        return RaycastResult::miss(max_range);
    }

    let dir = direction.normalized();
    let mut closest_dist = max_range;
    let mut closest_line: Option<usize> = None;
    let mut closest_point: Option<Point2D> = None;

    for (idx, line) in lines.iter().enumerate() {
        if let Some(t) = line.ray_intersection(origin, dir)
            && t > 0.0
            && t < closest_dist
        {
            closest_dist = t;
            closest_line = Some(idx);
            closest_point = Some(origin + dir * t);
        }
    }

    match closest_line {
        Some(idx) => RaycastResult::hit(closest_dist, idx, closest_point.unwrap()),
        None => RaycastResult::miss(max_range),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point2D;
    use approx::assert_relative_eq;

    fn make_room() -> Vec<Line2D> {
        vec![
            Line2D::new(Point2D::new(-5.0, -5.0), Point2D::new(5.0, -5.0)), // Bottom
            Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0)),   // Right
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(-5.0, 5.0)),   // Top
            Line2D::new(Point2D::new(-5.0, 5.0), Point2D::new(-5.0, -5.0)), // Left
        ]
    }

    #[test]
    fn test_raycast_hit() {
        let lines = make_room();
        let origin = Point2D::zero();
        let direction = Point2D::new(1.0, 0.0); // East

        let dist = raycast(origin, direction, 100.0, &lines);

        // Should hit right wall at x=5
        assert_relative_eq!(dist, 5.0, epsilon = 0.01);
    }

    #[test]
    fn test_raycast_miss() {
        let lines = vec![Line2D::new(
            Point2D::new(10.0, 0.0),
            Point2D::new(10.0, 10.0),
        )];
        let origin = Point2D::zero();
        let direction = Point2D::new(-1.0, 0.0); // West (away from line)

        let dist = raycast(origin, direction, 100.0, &lines);

        assert_eq!(dist, 100.0); // Max range
    }

    #[test]
    fn test_raycast_detailed() {
        let lines = make_room();
        let origin = Point2D::zero();
        let direction = Point2D::new(0.0, -1.0); // South

        let result = raycast_detailed(origin, direction, 100.0, &lines);

        assert!(result.hit);
        assert_relative_eq!(result.distance, 5.0, epsilon = 0.01);
        assert_eq!(result.line_idx, Some(0)); // Bottom wall
        assert!(result.hit_point.is_some());
        assert_relative_eq!(result.hit_point.unwrap().y, -5.0, epsilon = 0.01);
    }

    #[test]
    fn test_raycast_diagonal() {
        let lines = make_room();
        let origin = Point2D::zero();
        let direction = Point2D::new(1.0, 1.0).normalized(); // Northeast

        let dist = raycast(origin, direction, 100.0, &lines);

        // Should hit corner at (5, 5), distance = sqrt(50) ≈ 7.07
        assert_relative_eq!(dist, 5.0 * 2.0_f32.sqrt(), epsilon = 0.1);
    }

    #[test]
    fn test_raycast_empty_lines() {
        let lines: Vec<Line2D> = vec![];
        let origin = Point2D::zero();
        let direction = Point2D::new(1.0, 0.0);

        let dist = raycast(origin, direction, 10.0, &lines);

        assert_eq!(dist, 10.0);
    }

    #[test]
    fn test_raycast_from_edge() {
        let lines = make_room();
        // Origin near left wall
        let origin = Point2D::new(-4.9, 0.0);
        let direction = Point2D::new(-1.0, 0.0); // Toward wall

        let dist = raycast(origin, direction, 100.0, &lines);

        // Should hit wall very close
        assert_relative_eq!(dist, 0.1, epsilon = 0.01);
    }

    #[test]
    fn test_raycast_parallel_to_wall() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(10.0, 5.0))];
        let origin = Point2D::zero();
        let direction = Point2D::new(1.0, 0.0); // Parallel to line

        let dist = raycast(origin, direction, 100.0, &lines);

        // Should not hit (parallel)
        assert_eq!(dist, 100.0);
    }

    #[test]
    fn test_raycast_behind_origin() {
        let lines = vec![Line2D::new(
            Point2D::new(-5.0, -1.0),
            Point2D::new(-5.0, 1.0),
        )];
        let origin = Point2D::zero();
        let direction = Point2D::new(1.0, 0.0); // Away from line

        let dist = raycast(origin, direction, 100.0, &lines);

        // Line is behind, should not hit
        assert_eq!(dist, 100.0);
    }
}
