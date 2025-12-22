//! Corner detection from line segments and point curvature.
//!
//! This module provides two approaches to corner detection:
//!
//! 1. **Line Intersection**: Corners at intersections of adjacent line segments
//!    that meet angle and distance criteria.
//!
//! 2. **Curvature-Based**: Corners at points where the local curvature exceeds
//!    a threshold (sharp direction changes in the point sequence).
//!
//! The hybrid approach combines both methods for robust detection:
//! - Curvature catches corners even when line extraction fails
//! - Line intersection provides precise positioning
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_map::extraction::{CornerConfig, detect_hybrid_corners, extract_lines};
//! use vastu_map::core::Point2D;
//!
//! let points = vec![/* lidar hits */];
//! let lines = extract_lines(&points, &config);
//! let corners = detect_hybrid_corners(&points, &lines, &corner_config);
//! ```

use serde::{Deserialize, Serialize};

use crate::core::Point2D;
use crate::features::{Corner2D, Line2D};
use std::f32::consts::PI;

/// Configuration for corner detection.
#[derive(Clone, Debug, Serialize, Deserialize)]
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

    // ─────────────────────────────────────────────────────────────────────────
    // Curvature Detection Parameters
    // ─────────────────────────────────────────────────────────────────────────
    /// Window size for curvature computation (number of points on each side).
    /// Larger windows are more robust to noise but may miss sharp corners.
    /// Default: 3 points (uses 7-point window: 3 before + center + 3 after)
    pub curvature_window_size: usize,

    /// Minimum curvature angle (radians) to be considered a corner candidate.
    /// This is the angle change between the before-segment and after-segment.
    /// Default: π/4 (45°) - same as min_angle for consistency
    pub curvature_threshold: f32,

    /// Distance for merging corners from different detection methods.
    /// Corners within this distance are considered duplicates.
    /// Default: 0.10m (10cm)
    pub merge_distance: f32,

    /// Non-maximum suppression radius for curvature corners.
    /// Only the highest curvature point within this radius is kept.
    /// Default: 0.15m (15cm)
    pub nms_radius: f32,
}

impl Default for CornerConfig {
    fn default() -> Self {
        Self {
            min_angle: PI / 6.0,         // 30°
            max_angle: 5.0 * PI / 6.0,   // 150°
            max_endpoint_distance: 0.05, // 5cm

            // Curvature detection
            curvature_window_size: 3,      // 7-point window
            curvature_threshold: PI / 4.0, // 45°
            merge_distance: 0.10,          // 10cm
            nms_radius: 0.15,              // 15cm
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

    /// Builder-style setter for curvature window size.
    pub fn with_curvature_window_size(mut self, points: usize) -> Self {
        self.curvature_window_size = points.max(1);
        self
    }

    /// Builder-style setter for curvature threshold.
    pub fn with_curvature_threshold(mut self, radians: f32) -> Self {
        self.curvature_threshold = radians;
        self
    }

    /// Builder-style setter for merge distance.
    pub fn with_merge_distance(mut self, meters: f32) -> Self {
        self.merge_distance = meters;
        self
    }

    /// Builder-style setter for NMS radius.
    pub fn with_nms_radius(mut self, meters: f32) -> Self {
        self.nms_radius = meters;
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

// =============================================================================
// Curvature-Based Corner Detection
// =============================================================================

/// Curvature candidate before non-maximum suppression.
#[derive(Clone, Debug)]
struct CurvatureCandidate {
    position: Point2D,
    point_idx: usize,
    curvature: f32, // Angle change in radians
}

/// Detect corners using curvature analysis on raw point cloud.
///
/// This method finds sharp direction changes in the point sequence,
/// independent of line extraction. It's useful for detecting corners
/// that line extraction might miss.
///
/// # Algorithm
/// 1. For each point, compute direction vectors to neighbors within window
/// 2. Compute angle between before-segment and after-segment directions
/// 3. Keep points where angle exceeds threshold
/// 4. Apply non-maximum suppression
///
/// # Arguments
/// * `points` - Ordered point sequence (e.g., from lidar scan)
/// * `config` - Corner detection configuration
pub fn detect_curvature_corners(points: &[Point2D], config: &CornerConfig) -> Vec<Corner2D> {
    let window = config.curvature_window_size;
    let min_points = 2 * window + 1;

    if points.len() < min_points {
        return Vec::new();
    }

    // Step 1: Compute curvature at each point
    let mut candidates = Vec::new();

    for i in window..points.len() - window {
        let curvature = compute_curvature_at(points, i, window);

        if curvature >= config.curvature_threshold {
            candidates.push(CurvatureCandidate {
                position: points[i],
                point_idx: i,
                curvature,
            });
        }
    }

    // Step 2: Non-maximum suppression
    let filtered = non_maximum_suppression(&candidates, config.nms_radius);

    // Step 3: Convert to Corner2D
    filtered
        .into_iter()
        .map(|c| {
            // Use point index as both line indices (no line association)
            Corner2D::new(c.position, c.point_idx, c.point_idx, c.curvature)
        })
        .collect()
}

/// Compute curvature (angle change) at a specific point.
///
/// Uses points before and after within the window to compute direction vectors,
/// then calculates the angle between them.
fn compute_curvature_at(points: &[Point2D], center: usize, window: usize) -> f32 {
    // Get points for direction computation
    let before_start = center.saturating_sub(window);
    let before_end = center;
    let after_start = center;
    let after_end = (center + window).min(points.len() - 1);

    // Compute direction vectors using regression through window points
    let dir_before = compute_direction(&points[before_start..=before_end]);
    let dir_after = compute_direction(&points[after_start..=after_end]);

    // Compute angle between directions
    let dot = dir_before.dot(dir_after);
    let cross = dir_before.cross(dir_after);

    // Use atan2 for signed angle, then take absolute value
    cross.atan2(dot).abs()
}

/// Compute average direction through a set of points.
fn compute_direction(points: &[Point2D]) -> Point2D {
    if points.len() < 2 {
        return Point2D::new(1.0, 0.0);
    }

    // Use first and last points for direction
    let dir = points[points.len() - 1] - points[0];
    let len = (dir.x * dir.x + dir.y * dir.y).sqrt();

    if len < f32::EPSILON {
        Point2D::new(1.0, 0.0)
    } else {
        Point2D::new(dir.x / len, dir.y / len)
    }
}

/// Apply non-maximum suppression to curvature candidates.
///
/// Within each neighborhood, only keep the candidate with highest curvature.
fn non_maximum_suppression(
    candidates: &[CurvatureCandidate],
    radius: f32,
) -> Vec<CurvatureCandidate> {
    if candidates.is_empty() {
        return Vec::new();
    }

    let mut keep = vec![true; candidates.len()];
    let radius_sq = radius * radius;

    for i in 0..candidates.len() {
        if !keep[i] {
            continue;
        }

        for j in i + 1..candidates.len() {
            if !keep[j] {
                continue;
            }

            let dx = candidates[i].position.x - candidates[j].position.x;
            let dy = candidates[i].position.y - candidates[j].position.y;
            let dist_sq = dx * dx + dy * dy;

            if dist_sq < radius_sq {
                // Keep the one with higher curvature
                if candidates[j].curvature > candidates[i].curvature {
                    keep[i] = false;
                    break;
                } else {
                    keep[j] = false;
                }
            }
        }
    }

    candidates
        .iter()
        .enumerate()
        .filter(|(i, _)| keep[*i])
        .map(|(_, c)| c.clone())
        .collect()
}

// =============================================================================
// Hybrid Corner Detection
// =============================================================================

/// Detect corners using both curvature and line intersection methods.
///
/// This hybrid approach combines:
/// 1. Curvature-based detection on raw points (catches corners even without line extraction)
/// 2. Line intersection detection (provides precise positioning)
///
/// Corners from both methods are merged and deduplicated.
///
/// # Arguments
/// * `points` - Ordered point sequence (e.g., from lidar scan)
/// * `lines` - Extracted line segments
/// * `config` - Corner detection configuration
///
/// # Example
/// ```
/// use vastu_map::extraction::{CornerConfig, detect_hybrid_corners, extract_lines, SplitMergeConfig};
/// use vastu_map::core::Point2D;
///
/// // L-shaped points
/// let points = vec![
///     Point2D::new(0.0, 0.0),
///     Point2D::new(0.5, 0.0),
///     Point2D::new(1.0, 0.0),
///     Point2D::new(1.0, 0.5),
///     Point2D::new(1.0, 1.0),
/// ];
///
/// let split_config = SplitMergeConfig::default();
/// let lines = extract_lines(&points, &split_config);
/// let corner_config = CornerConfig::default();
/// let corners = detect_hybrid_corners(&points, &lines, &corner_config);
/// ```
pub fn detect_hybrid_corners(
    points: &[Point2D],
    lines: &[Line2D],
    config: &CornerConfig,
) -> Vec<Corner2D> {
    // Step 1: Detect curvature-based corners
    let mut curvature_corners = detect_curvature_corners(points, config);

    // Step 2: Detect line intersection corners
    let intersection_corners = detect_corners(lines, config);

    // Step 3: Merge and deduplicate
    // Intersection corners are generally more precise, so prefer them
    curvature_corners.extend(intersection_corners);

    deduplicate_corners(&mut curvature_corners, config.merge_distance);

    curvature_corners
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

    // ========================================
    // Curvature Detection Tests
    // ========================================

    #[test]
    fn test_curvature_l_shape() {
        // L-shaped points with sharp corner
        let mut points = Vec::new();

        // Horizontal segment
        for i in 0..10 {
            points.push(Point2D::new(i as f32 * 0.1, 0.0));
        }

        // Vertical segment
        for i in 1..10 {
            points.push(Point2D::new(0.9, i as f32 * 0.1));
        }

        let config = CornerConfig::default()
            .with_curvature_window_size(2)
            .with_curvature_threshold(PI / 4.0); // 45°

        let corners = detect_curvature_corners(&points, &config);

        // Should detect the corner at the L junction
        assert!(
            corners.len() >= 1,
            "Expected at least 1 corner, got {}",
            corners.len()
        );

        // Corner should be near (0.9, 0.0)
        let has_corner_at_junction = corners
            .iter()
            .any(|c| c.position.distance(Point2D::new(0.9, 0.0)) < 0.2);

        assert!(
            has_corner_at_junction,
            "Expected corner near (0.9, 0.0), got {:?}",
            corners.iter().map(|c| c.position).collect::<Vec<_>>()
        );
    }

    #[test]
    fn test_curvature_no_corner_straight_line() {
        // Straight line should have no curvature corners
        let points: Vec<_> = (0..20).map(|i| Point2D::new(i as f32 * 0.1, 0.0)).collect();

        let config = CornerConfig::default();
        let corners = detect_curvature_corners(&points, &config);

        assert!(
            corners.is_empty(),
            "Expected no corners on straight line, got {}",
            corners.len()
        );
    }

    #[test]
    fn test_curvature_too_few_points() {
        // Not enough points for the window
        let points = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(1.0, 1.0),
        ];

        let config = CornerConfig::default().with_curvature_window_size(3); // needs 7 points

        let corners = detect_curvature_corners(&points, &config);

        assert!(corners.is_empty());
    }

    #[test]
    fn test_hybrid_detection() {
        // L-shaped points
        let mut points = Vec::new();

        // Horizontal segment (enough points for lines)
        for i in 0..15 {
            points.push(Point2D::new(i as f32 * 0.1, 0.0));
        }

        // Vertical segment
        for i in 1..15 {
            points.push(Point2D::new(1.4, i as f32 * 0.1));
        }

        // Extract lines
        let split_config = crate::extraction::SplitMergeConfig::default()
            .with_min_points(5)
            .with_min_length(0.3);
        let lines = crate::extraction::extract_lines(&points, &split_config);

        let config = CornerConfig::default()
            .with_curvature_window_size(2)
            .with_merge_distance(0.2);

        let corners = detect_hybrid_corners(&points, &lines, &config);

        // Should detect corner(s) at the junction
        assert!(
            !corners.is_empty(),
            "Expected at least 1 corner from hybrid detection"
        );
    }

    #[test]
    fn test_non_maximum_suppression() {
        let candidates = vec![
            CurvatureCandidate {
                position: Point2D::new(0.0, 0.0),
                point_idx: 0,
                curvature: 1.0,
            },
            CurvatureCandidate {
                position: Point2D::new(0.05, 0.0), // Too close to first
                point_idx: 1,
                curvature: 0.5,
            },
            CurvatureCandidate {
                position: Point2D::new(1.0, 0.0), // Far enough
                point_idx: 10,
                curvature: 0.8,
            },
        ];

        let filtered = non_maximum_suppression(&candidates, 0.1);

        // Should keep first (higher curvature) and third (far enough)
        assert_eq!(filtered.len(), 2);
        assert_eq!(filtered[0].point_idx, 0);
        assert_eq!(filtered[1].point_idx, 10);
    }

    #[test]
    fn test_config_curvature_builders() {
        let config = CornerConfig::default()
            .with_curvature_window_size(5)
            .with_curvature_threshold(PI / 3.0)
            .with_merge_distance(0.15)
            .with_nms_radius(0.2);

        assert_eq!(config.curvature_window_size, 5);
        assert_relative_eq!(config.curvature_threshold, PI / 3.0, epsilon = 0.01);
        assert_relative_eq!(config.merge_distance, 0.15, epsilon = 0.01);
        assert_relative_eq!(config.nms_radius, 0.2, epsilon = 0.01);
    }
}
