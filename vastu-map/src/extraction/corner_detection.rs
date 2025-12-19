//! Corner detection from line segments.
//!
//! Corners are detected at intersections of adjacent line segments
//! that meet certain angle and distance criteria.

use crate::core::Point2D;
use crate::features::{Corner2D, Line2D};
use std::f32::consts::PI;

/// Configuration for corner detection.
#[derive(Clone, Debug)]
pub struct CornerConfig {
    /// Minimum angle between lines to form a corner (radians).
    /// Lines that are nearly parallel (angle below this) don't form corners.
    /// Default: π/6 (30°)
    pub min_angle: f32,

    /// Maximum angle between lines to form a corner (radians).
    /// Lines that are nearly collinear (angle above this) don't form corners.
    /// Default: 5π/6 (150°)
    pub max_angle: f32,

    /// Maximum distance between line endpoints to form a corner.
    /// If the nearest endpoints are farther apart, no corner is detected.
    /// Default: 0.05m (5cm)
    pub max_endpoint_distance: f32,
}

impl Default for CornerConfig {
    fn default() -> Self {
        Self {
            min_angle: PI / 6.0,         // 30°
            max_angle: 5.0 * PI / 6.0,   // 150°
            max_endpoint_distance: 0.05, // 5cm
        }
    }
}

impl CornerConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for minimum angle.
    pub fn with_min_angle(mut self, radians: f32) -> Self {
        self.min_angle = radians;
        self
    }

    /// Builder-style setter for maximum angle.
    pub fn with_max_angle(mut self, radians: f32) -> Self {
        self.max_angle = radians;
        self
    }

    /// Builder-style setter for maximum endpoint distance.
    pub fn with_max_endpoint_distance(mut self, meters: f32) -> Self {
        self.max_endpoint_distance = meters;
        self
    }
}

/// Detect corners from adjacent line segments.
///
/// Adjacent segments are checked for corner formation based on:
/// - Endpoint proximity
/// - Angle between lines
///
/// # Arguments
/// * `lines` - Line segments (assumed to be in order, e.g., from split-merge)
/// * `config` - Corner detection configuration
///
/// # Returns
/// Vector of detected corners with line indices and angles.
pub fn detect_corners(lines: &[Line2D], config: &CornerConfig) -> Vec<Corner2D> {
    if lines.len() < 2 {
        return Vec::new();
    }

    let mut corners = Vec::new();

    for i in 0..lines.len() - 1 {
        if let Some(corner) = detect_corner_between(&lines[i], &lines[i + 1], i, i + 1, config) {
            corners.push(corner);
        }
    }

    corners
}

/// Detect corners from all pairs of lines (not just adjacent).
///
/// This is useful for finding corners in maps where lines may not be
/// in sequential order.
///
/// # Note
/// This has O(n²) complexity, so use sparingly for large line sets.
pub fn detect_all_corners(lines: &[Line2D], config: &CornerConfig) -> Vec<Corner2D> {
    if lines.len() < 2 {
        return Vec::new();
    }

    let mut corners = Vec::new();

    for i in 0..lines.len() {
        for j in i + 1..lines.len() {
            if let Some(corner) = detect_corner_between(&lines[i], &lines[j], i, j, config) {
                corners.push(corner);
            }
        }
    }

    corners
}

/// Detect corner between two specific lines.
fn detect_corner_between(
    line1: &Line2D,
    line2: &Line2D,
    idx1: usize,
    idx2: usize,
    config: &CornerConfig,
) -> Option<Corner2D> {
    // Find nearest endpoints
    let endpoints1 = [line1.start, line1.end];
    let endpoints2 = [line2.start, line2.end];

    let mut min_dist = f32::MAX;
    let mut corner_point = Point2D::zero();
    let mut _ep1_idx = 0;
    let mut _ep2_idx = 0;

    for (i, &ep1) in endpoints1.iter().enumerate() {
        for (j, &ep2) in endpoints2.iter().enumerate() {
            let dist = ep1.distance(ep2);
            if dist < min_dist {
                min_dist = dist;
                corner_point = Point2D::new((ep1.x + ep2.x) / 2.0, (ep1.y + ep2.y) / 2.0);
                _ep1_idx = i;
                _ep2_idx = j;
            }
        }
    }

    // Check distance threshold
    if min_dist > config.max_endpoint_distance {
        return None;
    }

    // Compute angle between lines
    let angle = angle_between_lines(line1, line2);

    // Check angle thresholds
    if angle < config.min_angle || angle > config.max_angle {
        return None;
    }

    // Alternatively, use the intersection point if lines actually intersect
    if let Some(intersection) = line_intersection_point(line1, line2) {
        // Use intersection if it's close to the endpoints
        if intersection.distance(corner_point) < config.max_endpoint_distance * 2.0 {
            corner_point = intersection;
        }
    }

    Some(Corner2D::new(corner_point, idx1, idx2, angle))
}

/// Compute the angle between two lines (0 to π).
///
/// Returns the acute or obtuse angle at their intersection.
fn angle_between_lines(line1: &Line2D, line2: &Line2D) -> f32 {
    let dir1 = line1.unit_direction();
    let dir2 = line2.unit_direction();

    // Use absolute dot product to get angle regardless of direction
    let dot = dir1.dot(dir2).abs();

    // Clamp to handle floating-point errors
    let dot = dot.clamp(-1.0, 1.0);

    // acos gives angle between directions
    // We want the supplementary angle if > 90°
    dot.acos()
}

/// Find intersection point of two infinite lines.
fn line_intersection_point(line1: &Line2D, line2: &Line2D) -> Option<Point2D> {
    let d1 = line1.direction();
    let d2 = line2.direction();

    let cross = d1.cross(d2);
    if cross.abs() < f32::EPSILON {
        // Parallel lines
        return None;
    }

    let diff = line2.start - line1.start;
    let t = diff.cross(d2) / cross;

    Some(line1.point_at(t))
}

/// Filter corners by removing duplicates within a tolerance.
///
/// Corners at approximately the same position are merged,
/// keeping the one with highest observation count.
pub fn deduplicate_corners(corners: &mut Vec<Corner2D>, distance_tolerance: f32) {
    if corners.len() < 2 {
        return;
    }

    // Sort by x coordinate for spatial coherence
    corners.sort_by(|a, b| a.position.x.partial_cmp(&b.position.x).unwrap());

    let mut keep = vec![true; corners.len()];

    for i in 0..corners.len() {
        if !keep[i] {
            continue;
        }

        for j in i + 1..corners.len() {
            if !keep[j] {
                continue;
            }

            // Early exit if too far in x
            if corners[j].position.x - corners[i].position.x > distance_tolerance {
                break;
            }

            if corners[i].distance(&corners[j]) < distance_tolerance {
                // Keep the one with higher observation count
                if corners[j].observation_count > corners[i].observation_count {
                    keep[i] = false;
                } else {
                    keep[j] = false;
                }
            }
        }
    }

    let mut idx = 0;
    corners.retain(|_| {
        let result = keep[idx];
        idx += 1;
        result
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};

    #[test]
    fn test_detect_corner_perpendicular() {
        // Two perpendicular lines meeting at origin
        let line1 = Line2D::new(Point2D::new(-1.0, 0.0), Point2D::new(0.0, 0.0));
        let line2 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 1.0));

        let config = CornerConfig::default();
        let corners = detect_corners(&[line1, line2], &config);

        assert_eq!(corners.len(), 1);
        assert_relative_eq!(corners[0].angle, FRAC_PI_2, epsilon = 0.1);
        assert!(corners[0].position.distance(Point2D::zero()) < 0.1);
    }

    #[test]
    fn test_detect_corner_45_degrees() {
        // Two lines meeting at 45 degrees
        let line1 = Line2D::new(Point2D::new(-1.0, 0.0), Point2D::new(0.0, 0.0));
        let line2 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 1.0));

        let config = CornerConfig::default();
        let corners = detect_corners(&[line1, line2], &config);

        assert_eq!(corners.len(), 1);
        assert_relative_eq!(corners[0].angle, FRAC_PI_4, epsilon = 0.1);
    }

    #[test]
    fn test_no_corner_parallel_lines() {
        // Parallel lines don't form corners
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0));
        let line2 = Line2D::new(Point2D::new(1.01, 0.0), Point2D::new(2.0, 0.0));

        let config = CornerConfig::default();
        let corners = detect_corners(&[line1, line2], &config);

        // Should be empty - lines are collinear
        assert!(corners.is_empty());
    }

    #[test]
    fn test_no_corner_too_far() {
        // Lines too far apart
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0));
        let line2 = Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(0.0, 2.0));

        let config = CornerConfig::default().with_max_endpoint_distance(0.05);
        let corners = detect_corners(&[line1, line2], &config);

        assert!(corners.is_empty());
    }

    #[test]
    fn test_no_corner_angle_too_small() {
        // Angle below threshold (nearly parallel)
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0));
        let line2 = Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(2.0, 0.1)); // ~5.7° deviation

        let config = CornerConfig::default().with_min_angle(PI / 6.0); // 30°
        let corners = detect_corners(&[line1, line2], &config);

        assert!(corners.is_empty());
    }

    #[test]
    fn test_detect_all_corners() {
        // Three lines forming two corners
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(1.0, 1.0)),
            Line2D::new(Point2D::new(1.0, 1.0), Point2D::new(0.0, 1.0)),
        ];

        let config = CornerConfig::default();
        let corners = detect_corners(&lines, &config);

        assert_eq!(corners.len(), 2);
    }

    #[test]
    fn test_angle_between_lines() {
        // Perpendicular lines
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0));
        let line2 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 1.0));

        let angle = angle_between_lines(&line1, &line2);
        assert_relative_eq!(angle, FRAC_PI_2, epsilon = 0.01);

        // Parallel lines
        let line3 = Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0));
        let angle2 = angle_between_lines(&line1, &line3);
        assert_relative_eq!(angle2, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_deduplicate_corners() {
        let mut corners = vec![
            Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2),
            Corner2D::new(Point2D::new(0.01, 0.01), 0, 1, FRAC_PI_2), // Duplicate
            Corner2D::new(Point2D::new(5.0, 5.0), 2, 3, FRAC_PI_2),
        ];

        deduplicate_corners(&mut corners, 0.05);

        assert_eq!(corners.len(), 2);
    }

    #[test]
    fn test_line_intersection() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(2.0, 2.0));
        let line2 = Line2D::new(Point2D::new(0.0, 2.0), Point2D::new(2.0, 0.0));

        let intersection = line_intersection_point(&line1, &line2).unwrap();

        assert_relative_eq!(intersection.x, 1.0, epsilon = 0.01);
        assert_relative_eq!(intersection.y, 1.0, epsilon = 0.01);
    }
}
