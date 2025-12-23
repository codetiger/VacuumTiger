//! Path clearance checking utilities.
//!
//! Provides functions to check if a straight-line path is clear of obstacles.

use crate::core::Point2D;
use crate::features::Line2D;

/// Check if a straight-line path between two points is clear of obstacles.
///
/// This performs collision checking by:
/// 1. Checking for direct line-line intersections
/// 2. Sampling points along the path and checking clearance
///
/// # Arguments
/// * `from` - Start position
/// * `to` - End position
/// * `lines` - Map lines (obstacles)
/// * `robot_radius` - Robot radius for clearance
///
/// # Returns
/// True if the straight path is clear (no obstacles within robot_radius).
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::query::is_straight_path_clear;
/// use vastu_map::core::Point2D;
///
/// let clear = is_straight_path_clear(
///     Point2D::new(0.0, 0.0),
///     Point2D::new(1.0, 1.0),
///     &walls,
///     0.15, // robot radius
/// );
/// ```
pub fn is_straight_path_clear(
    from: Point2D,
    to: Point2D,
    lines: &[Line2D],
    robot_radius: f32,
) -> bool {
    let test_line = Line2D::new(from, to);
    let test_length = test_line.length();

    if test_length < 0.001 {
        return true; // Zero-length path is always clear
    }

    for line in lines {
        // Check for direct intersection
        if test_line.intersection(line).is_some() {
            return false;
        }

        // Check clearance along the path (use segment distance, not infinite line)
        if robot_radius > 0.0 {
            let num_samples = (test_length / robot_radius).ceil() as usize;
            let num_samples = num_samples.max(3);

            for k in 0..=num_samples {
                let t = k as f32 / num_samples as f32;
                let sample = test_line.point_at(t);
                // Use distance_to_point_segment for accurate segment distance
                let dist = line.distance_to_point_segment(sample);

                if dist < robot_radius {
                    return false;
                }
            }
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_straight_path_clear_empty() {
        let lines: Vec<Line2D> = vec![];
        assert!(is_straight_path_clear(
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
            &lines,
            0.1
        ));
    }

    #[test]
    fn test_is_straight_path_clear_blocked() {
        let lines = vec![Line2D::new(Point2D::new(0.5, -1.0), Point2D::new(0.5, 1.0))];
        assert!(!is_straight_path_clear(
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            &lines,
            0.0
        ));
    }

    #[test]
    fn test_is_straight_path_clear_with_radius() {
        // Path goes close to wall but doesn't intersect
        let lines = vec![Line2D::new(Point2D::new(0.5, 0.15), Point2D::new(0.5, 1.0))];
        // With radius 0.2, should be blocked (too close)
        assert!(!is_straight_path_clear(
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            &lines,
            0.2
        ));
    }

    #[test]
    fn test_is_straight_path_clear_with_clearance() {
        // Wall is far enough away
        let lines = vec![Line2D::new(Point2D::new(0.5, 0.5), Point2D::new(0.5, 1.0))];
        assert!(is_straight_path_clear(
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            &lines,
            0.1
        ));
    }

    #[test]
    fn test_zero_length_path() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0))];
        // Zero-length path is always clear
        assert!(is_straight_path_clear(
            Point2D::new(0.0, 0.0),
            Point2D::new(0.0, 0.0),
            &lines,
            0.1
        ));
    }
}
