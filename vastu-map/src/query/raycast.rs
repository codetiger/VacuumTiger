//! Ray casting for obstacle detection.
//!
//! Computes the distance from a point in a given direction to the
//! first obstacle (line segment) in the map.

use crate::core::Point2D;
use crate::features::{Line2D, LineCollection};
use crate::integration::SpatialIndex;

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

/// Cast a ray using spatial index for efficiency.
///
/// Uses the spatial index to only test lines in the ray's path.
/// This is O(log n + k) where k is the number of candidate lines,
/// compared to O(n) for the brute-force version.
pub fn raycast_indexed(
    origin: Point2D,
    direction: Point2D,
    max_range: f32,
    lines: &[Line2D],
    index: &SpatialIndex,
) -> RaycastResult {
    use crate::core::Bounds;

    if lines.is_empty() {
        return RaycastResult::miss(max_range);
    }

    let dir = direction.normalized();
    let end_point = origin + dir * max_range;

    // Compute the bounding box of the ray
    let min_x = origin.x.min(end_point.x);
    let max_x = origin.x.max(end_point.x);
    let min_y = origin.y.min(end_point.y);
    let max_y = origin.y.max(end_point.y);

    // Query the spatial index for candidate lines
    let bounds = Bounds::new(Point2D::new(min_x, min_y), Point2D::new(max_x, max_y));
    let candidates = index.lines_in_bounds(&bounds);

    if candidates.is_empty() {
        return RaycastResult::miss(max_range);
    }

    // Test only candidate lines
    let mut closest_dist = max_range;
    let mut closest_line: Option<usize> = None;
    let mut closest_point: Option<Point2D> = None;

    for &idx in &candidates {
        if let Some(line) = lines.get(idx)
            && let Some(t) = line.ray_intersection(origin, dir)
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

/// Cast multiple rays (e.g., for simulating a lidar).
///
/// # Arguments
/// * `origin` - Ray origin
/// * `start_angle` - Starting angle (radians)
/// * `end_angle` - Ending angle (radians)
/// * `num_rays` - Number of rays to cast
/// * `max_range` - Maximum range
/// * `lines` - Map lines
///
/// # Returns
/// Vector of distances for each ray.
pub fn raycast_sweep(
    origin: Point2D,
    start_angle: f32,
    end_angle: f32,
    num_rays: usize,
    max_range: f32,
    lines: &[Line2D],
) -> Vec<f32> {
    if num_rays == 0 {
        return Vec::new();
    }

    let angle_step = if num_rays > 1 {
        (end_angle - start_angle) / (num_rays - 1) as f32
    } else {
        0.0
    };

    (0..num_rays)
        .map(|i| {
            let angle = start_angle + i as f32 * angle_step;
            let dir = Point2D::from_polar(1.0, angle);
            raycast(origin, dir, max_range, lines)
        })
        .collect()
}

/// Cast rays in a full 360° sweep.
pub fn raycast_360(origin: Point2D, num_rays: usize, max_range: f32, lines: &[Line2D]) -> Vec<f32> {
    use std::f32::consts::PI;
    raycast_sweep(origin, -PI, PI, num_rays, max_range, lines)
}

/// Batch raycast using LineCollection (optimized for SIMD).
///
/// Note: Currently falls back to per-line iteration.
/// Future optimization could use SIMD for parallel intersection tests.
pub fn raycast_batch(
    origin: Point2D,
    direction: Point2D,
    max_range: f32,
    lines: &LineCollection,
) -> f32 {
    let lines_vec = lines.to_lines();
    raycast(origin, direction, max_range, &lines_vec)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point2D;
    use approx::assert_relative_eq;
    use std::f32::consts::PI;

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
    fn test_raycast_sweep() {
        let lines = make_room();
        let origin = Point2D::zero();

        // Cast 4 rays: N, E, S, W
        let distances = raycast_sweep(origin, -PI, PI, 4, 100.0, &lines);

        // Note: with 4 rays from -π to π:
        // Ray 0: angle = -π (West) → should hit left wall at distance 5
        // Ray 1: angle = -π/3 ≈ -60° (roughly SW)
        // etc.
        assert_eq!(distances.len(), 4);
        // All should be around 5m (hitting walls)
        for dist in &distances {
            assert!(*dist <= 10.0); // Should hit some wall
        }
    }

    #[test]
    fn test_raycast_360() {
        let lines = make_room();
        let origin = Point2D::zero();

        let distances = raycast_360(origin, 8, 100.0, &lines);

        assert_eq!(distances.len(), 8);
        // All rays from center should hit a wall
        for dist in &distances {
            assert!(*dist < 100.0);
        }
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

    #[test]
    fn test_raycast_indexed_hit() {
        let lines = make_room();
        let index = SpatialIndex::new(&lines);
        let origin = Point2D::zero();
        let direction = Point2D::new(1.0, 0.0); // East

        let result = raycast_indexed(origin, direction, 100.0, &lines, &index);

        // Should hit right wall at x=5
        assert!(result.hit);
        assert_relative_eq!(result.distance, 5.0, epsilon = 0.01);
        assert_eq!(result.line_idx, Some(1)); // Right wall
    }

    #[test]
    fn test_raycast_indexed_miss() {
        let lines = vec![Line2D::new(
            Point2D::new(10.0, 0.0),
            Point2D::new(10.0, 10.0),
        )];
        let index = SpatialIndex::new(&lines);
        let origin = Point2D::zero();
        let direction = Point2D::new(-1.0, 0.0); // West (away from line)

        let result = raycast_indexed(origin, direction, 5.0, &lines, &index);

        // Should miss (line is behind)
        assert!(!result.hit);
        assert_eq!(result.distance, 5.0);
    }

    #[test]
    fn test_raycast_indexed_matches_brute_force() {
        let lines = make_room();
        let index = SpatialIndex::new(&lines);

        // Test several directions
        let origin = Point2D::zero();
        let directions = [
            Point2D::new(1.0, 0.0),              // East
            Point2D::new(-1.0, 0.0),             // West
            Point2D::new(0.0, 1.0),              // North
            Point2D::new(0.0, -1.0),             // South
            Point2D::new(1.0, 1.0).normalized(), // NE
        ];

        for dir in &directions {
            let brute = raycast_detailed(origin, *dir, 100.0, &lines);
            let indexed = raycast_indexed(origin, *dir, 100.0, &lines, &index);

            assert_eq!(brute.hit, indexed.hit);
            assert_relative_eq!(brute.distance, indexed.distance, epsilon = 0.01);
        }
    }
}
