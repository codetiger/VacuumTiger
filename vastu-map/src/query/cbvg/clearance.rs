//! Clearance computation utilities for CBVG.
//!
//! Provides functions to compute distances from points to walls
//! and check path clearance.

use crate::core::Point2D;
use crate::features::Line2D;

/// Compute minimum distance from a point to any wall segment.
///
/// # Arguments
/// * `point` - The query point
/// * `lines` - Wall segments in the map
///
/// # Returns
/// Minimum distance to any wall, or `f32::INFINITY` if no walls exist.
#[inline]
pub fn min_distance_to_walls(point: Point2D, lines: &[Line2D]) -> f32 {
    lines
        .iter()
        .map(|line| line.distance_to_point_segment(point))
        .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or(f32::INFINITY)
}

/// Find the nearest wall to a point.
///
/// # Arguments
/// * `point` - The query point
/// * `lines` - Wall segments in the map
///
/// # Returns
/// Tuple of (distance, wall_index), or None if no walls exist.
pub fn nearest_wall(point: Point2D, lines: &[Line2D]) -> Option<(f32, usize)> {
    lines
        .iter()
        .enumerate()
        .map(|(idx, line)| (line.distance_to_point_segment(point), idx))
        .min_by(|(a, _), (b, _)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
}

/// Check if a straight path between two points has sufficient clearance.
///
/// Samples points along the path and verifies each has minimum clearance.
///
/// # Arguments
/// * `from` - Start position
/// * `to` - End position
/// * `lines` - Wall segments
/// * `min_clearance` - Required minimum distance from walls
/// * `sample_step` - Distance between sample points
///
/// # Returns
/// True if the entire path has sufficient clearance.
pub fn is_path_clear_with_clearance(
    from: Point2D,
    to: Point2D,
    lines: &[Line2D],
    min_clearance: f32,
    sample_step: f32,
) -> bool {
    let segment = Line2D::new(from, to);
    let length = segment.length();

    if length < 0.001 {
        // Zero-length path - just check the point
        return min_distance_to_walls(from, lines) >= min_clearance;
    }

    // Sample at regular intervals
    let step = sample_step.min(min_clearance * 0.5);
    let num_samples = (length / step).ceil() as usize;
    let num_samples = num_samples.max(2); // At least check start and end

    for k in 0..=num_samples {
        let t = k as f32 / num_samples as f32;
        let sample = segment.point_at(t);

        if min_distance_to_walls(sample, lines) < min_clearance {
            return false;
        }
    }

    true
}

/// Check if an edge between two nodes is safe for robot traversal.
///
/// Uses adaptive clearance: full clearance in open areas,
/// reduced clearance in narrow passages.
///
/// # Arguments
/// * `from` - Start position
/// * `to` - End position
/// * `lines` - Wall segments
/// * `min_clearance` - Full clearance for open areas
/// * `min_clearance_narrow` - Reduced clearance for narrow passages
/// * `sample_step` - Distance between sample points
///
/// # Returns
/// True if the edge is safe for traversal.
pub fn is_edge_safe(
    from: Point2D,
    to: Point2D,
    lines: &[Line2D],
    min_clearance: f32,
    min_clearance_narrow: f32,
    sample_step: f32,
) -> bool {
    let segment = Line2D::new(from, to);
    let length = segment.length();

    if length < 0.001 {
        return min_distance_to_walls(from, lines) >= min_clearance_narrow;
    }

    let num_samples = (length / sample_step).ceil() as usize;
    let num_samples = num_samples.max(2);

    for k in 0..=num_samples {
        let t = k as f32 / num_samples as f32;
        let sample = segment.point_at(t);

        let clearance = min_distance_to_walls(sample, lines);

        // Adaptive clearance: use narrow clearance if we're in a tight spot
        let required = if clearance < min_clearance {
            min_clearance_narrow
        } else {
            min_clearance
        };

        if clearance < required {
            return false;
        }
    }

    true
}

/// Sample points along a line segment at regular intervals.
///
/// # Arguments
/// * `line` - The line segment to sample
/// * `step` - Distance between samples
///
/// # Returns
/// Vector of sampled points including start and end.
pub fn sample_line(line: &Line2D, step: f32) -> Vec<Point2D> {
    let length = line.length();

    if length < step {
        return vec![line.start, line.end];
    }

    let num_samples = (length / step).ceil() as usize;
    let mut points = Vec::with_capacity(num_samples + 1);

    for i in 0..=num_samples {
        let t = i as f32 / num_samples as f32;
        points.push(line.point_at(t));
    }

    points
}

/// Find the closest point on any wall to the given point.
///
/// # Arguments
/// * `point` - The query point
/// * `lines` - Wall segments
///
/// # Returns
/// The closest point on any wall, or the input point if no walls exist.
pub fn closest_point_on_walls(point: Point2D, lines: &[Line2D]) -> Point2D {
    lines
        .iter()
        .map(|line| {
            let t = line.project_point(point).clamp(0.0, 1.0);
            let projected = line.point_at(t);
            let dist = point.distance(projected);
            (projected, dist)
        })
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(p, _)| p)
        .unwrap_or(point)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_walls() -> Vec<Line2D> {
        vec![
            // Horizontal wall at y=2
            Line2D::new(Point2D::new(-5.0, 2.0), Point2D::new(5.0, 2.0)),
            // Vertical wall at x=3
            Line2D::new(Point2D::new(3.0, -2.0), Point2D::new(3.0, 2.0)),
        ]
    }

    #[test]
    fn test_min_distance_to_walls() {
        let walls = make_test_walls();

        // Point at origin
        let dist = min_distance_to_walls(Point2D::new(0.0, 0.0), &walls);
        assert!((dist - 2.0).abs() < 0.01); // 2m from horizontal wall

        // Point near vertical wall
        let dist = min_distance_to_walls(Point2D::new(2.5, 0.0), &walls);
        assert!((dist - 0.5).abs() < 0.01); // 0.5m from vertical wall

        // Empty walls
        let dist = min_distance_to_walls(Point2D::new(0.0, 0.0), &[]);
        assert!(dist.is_infinite());
    }

    #[test]
    fn test_nearest_wall() {
        let walls = make_test_walls();

        let result = nearest_wall(Point2D::new(0.0, 0.0), &walls);
        assert!(result.is_some());
        let (dist, idx) = result.unwrap();
        assert!((dist - 2.0).abs() < 0.01);
        assert_eq!(idx, 0); // Horizontal wall is closer

        let result = nearest_wall(Point2D::new(0.0, 0.0), &[]);
        assert!(result.is_none());
    }

    #[test]
    fn test_is_path_clear_with_clearance() {
        let walls = make_test_walls();

        // Path in open area (far from walls)
        assert!(is_path_clear_with_clearance(
            Point2D::new(-2.0, 0.0),
            Point2D::new(0.0, 0.0),
            &walls,
            0.3,
            0.1
        ));

        // Path too close to wall
        assert!(!is_path_clear_with_clearance(
            Point2D::new(-2.0, 1.9),
            Point2D::new(2.0, 1.9),
            &walls,
            0.3,
            0.1
        ));
    }

    #[test]
    fn test_is_edge_safe() {
        let walls = make_test_walls();

        // Safe edge in open area
        assert!(is_edge_safe(
            Point2D::new(-2.0, 0.0),
            Point2D::new(0.0, 0.0),
            &walls,
            0.3,
            0.18,
            0.05
        ));

        // Edge too close to wall
        assert!(!is_edge_safe(
            Point2D::new(-2.0, 1.95),
            Point2D::new(2.0, 1.95),
            &walls,
            0.3,
            0.18,
            0.05
        ));
    }

    #[test]
    fn test_sample_line() {
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0));

        let samples = sample_line(&line, 0.25);
        assert_eq!(samples.len(), 5); // 0, 0.25, 0.5, 0.75, 1.0

        // Short line
        let short = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.1, 0.0));
        let samples = sample_line(&short, 0.5);
        assert_eq!(samples.len(), 2); // Just start and end
    }

    #[test]
    fn test_closest_point_on_walls() {
        let walls = make_test_walls();

        let closest = closest_point_on_walls(Point2D::new(0.0, 3.0), &walls);
        assert!((closest.x - 0.0).abs() < 0.01);
        assert!((closest.y - 2.0).abs() < 0.01); // Closest to horizontal wall
    }
}
