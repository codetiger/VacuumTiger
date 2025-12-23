//! Nearest neighbor matching for scan-to-map correspondence.
//!
//! Finds correspondences between scan points and map lines using
//! distance-based matching with geometric constraints.
//!
//! Note: For ICP hot paths, use `IcpScratchSpace::find_correspondences()`
//! which is zero-allocation. This module's `find_correspondences()` function
//! is provided for testing and simple use cases.

use crate::core::Point2D;
use crate::features::Line2D;

use super::correspondence::{Correspondence, CorrespondenceSet};

// ============================================================================
// Correspondence Validation Helper
// ============================================================================

/// Result of validating a potential correspondence.
///
/// Contains the projection parameter and perpendicular distance needed
/// to create a `Correspondence` if the validation passes.
#[derive(Clone, Copy, Debug)]
pub struct CorrespondenceCandidate {
    /// Projection parameter along the line (0 = start, 1 = end).
    pub t: f32,
    /// Perpendicular distance from point to line.
    pub distance: f32,
}

/// Validate a potential correspondence between a point and a line.
///
/// Checks:
/// 1. Projection bounds: point projects within the line segment (with extension tolerance)
/// 2. Distance threshold: perpendicular distance is within the maximum allowed
///
/// # Arguments
/// * `point` - The scan point to validate
/// * `line` - The map line to match against
/// * `max_distance` - Maximum allowed perpendicular distance (meters)
/// * `max_projection_extension` - Tolerance for projection beyond line endpoints
///
/// # Returns
/// `Some(CorrespondenceCandidate)` if valid, `None` if rejected.
///
/// # Example
/// ```rust,ignore
/// let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
/// let point = Point2D::new(2.5, 0.1); // Near middle of line
///
/// if let Some(candidate) = validate_correspondence_candidate(point, &line, 0.5, 0.2) {
///     // Point is valid: distance=0.1, t=0.5
///     let corr = Correspondence::new(point_idx, line_idx, point, candidate.distance, candidate.t);
/// }
/// ```
#[inline]
pub fn validate_correspondence_candidate(
    point: Point2D,
    line: &Line2D,
    max_distance: f32,
    max_projection_extension: f32,
) -> Option<CorrespondenceCandidate> {
    // Compute projection parameter
    let t = line.project_point(point);

    // Check projection bounds using Line2D helper
    if !line.contains_projection_extended(t, max_projection_extension) {
        return None;
    }

    // Compute perpendicular distance
    let distance = line.distance_to_point(point);

    // Check distance threshold
    if distance > max_distance {
        return None;
    }

    Some(CorrespondenceCandidate { t, distance })
}

/// Configuration for nearest neighbor matching.
///
/// # Geometric Concepts
///
/// ## Projection Parameter (t)
///
/// When a point P is matched against a line segment from A to B, we compute
/// the projection parameter `t` where `t=0` means P projects onto A, and
/// `t=1` means P projects onto B:
///
/// ```text
///        P (point)
///        |
///        v  (perpendicular distance)
///   A----+----B  (line segment)
///   t=0  t=0.5  t=1
/// ```
///
/// ## Projection Extension
///
/// The `max_projection_extension` parameter allows matching points that
/// project slightly beyond the line endpoints. With extension=0.2:
///
/// ```text
///   Valid projection range: [-0.2, 1.2]
///
///      |<-- 0.2 -->|<---- line ---->|<-- 0.2 -->|
///   ---+===========A================B===========+---
///     t=-0.2      t=0              t=1        t=1.2
/// ```
///
/// This is useful because lidar points near corners may project just past
/// the extracted line segment.
#[derive(Clone, Debug)]
pub struct NearestNeighborConfig {
    /// Maximum perpendicular distance for a valid correspondence (meters).
    /// Points farther than this from all lines are rejected.
    /// Default: 0.5m
    pub max_distance: f32,

    /// Maximum projection extension beyond line endpoints.
    ///
    /// The projection parameter `t` normally ranges from 0 (at line start)
    /// to 1 (at line end). This setting allows points to project up to
    /// `extension` units beyond the endpoints:
    /// - Valid range becomes `[-extension, 1+extension]`
    /// - Value of 0.2 allows 20% extension beyond endpoints
    /// - Value of 0.0 requires projection to fall exactly on the segment
    ///
    /// Default: 0.2
    pub max_projection_extension: f32,

    /// Whether to only keep the best correspondence per point.
    /// If false, a point may correspond to multiple lines.
    /// Default: true
    pub unique_per_point: bool,
}

impl Default for NearestNeighborConfig {
    fn default() -> Self {
        Self {
            max_distance: 0.5,
            max_projection_extension: 0.2,
            unique_per_point: true,
        }
    }
}

impl NearestNeighborConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for maximum distance.
    pub fn with_max_distance(mut self, meters: f32) -> Self {
        self.max_distance = meters;
        self
    }

    /// Builder-style setter for maximum projection extension.
    pub fn with_max_projection_extension(mut self, extension: f32) -> Self {
        self.max_projection_extension = extension;
        self
    }

    /// Builder-style setter for unique per point.
    pub fn with_unique_per_point(mut self, unique: bool) -> Self {
        self.unique_per_point = unique;
        self
    }
}

/// Find correspondences between scan points and map lines.
///
/// For each point, finds the nearest line within the distance threshold.
/// Uses geometric constraints (projection bounds) to filter invalid matches.
///
/// # Arguments
/// * `points` - Scan points in world frame
/// * `lines` - Map lines to match against
/// * `config` - Matching configuration
///
/// # Returns
/// Set of valid correspondences.
pub fn find_correspondences(
    points: &[Point2D],
    lines: &[Line2D],
    config: &NearestNeighborConfig,
) -> CorrespondenceSet {
    if points.is_empty() || lines.is_empty() {
        return CorrespondenceSet::new();
    }

    let mut result = CorrespondenceSet::with_capacity(points.len());

    for (point_idx, &point) in points.iter().enumerate() {
        let mut best_distance = config.max_distance;
        let mut best_corr: Option<Correspondence> = None;

        for (line_idx, line) in lines.iter().enumerate() {
            // Use validation helper to check projection and distance constraints
            let Some(candidate) = validate_correspondence_candidate(
                point,
                line,
                config.max_distance,
                config.max_projection_extension,
            ) else {
                continue;
            };

            if config.unique_per_point {
                // Keep only the best match
                if candidate.distance < best_distance {
                    best_distance = candidate.distance;
                    best_corr = Some(Correspondence::new(
                        point_idx,
                        line_idx,
                        point,
                        candidate.distance,
                        candidate.t,
                    ));
                }
            } else {
                // Add all valid matches
                result.push(Correspondence::new(
                    point_idx,
                    line_idx,
                    point,
                    candidate.distance,
                    candidate.t,
                ));
            }
        }

        if config.unique_per_point
            && let Some(corr) = best_corr
        {
            result.push(corr);
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_horizontal_line() -> Line2D {
        Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(2.0, 0.0))
    }

    fn make_vertical_line() -> Line2D {
        Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 2.0))
    }

    // === Tests for validate_correspondence_candidate helper ===

    #[test]
    fn test_validate_correspondence_candidate_valid() {
        let line = make_horizontal_line(); // (0,0) to (2,0)
        let point = Point2D::new(1.0, 0.1); // Above middle of line

        let result = validate_correspondence_candidate(point, &line, 0.5, 0.2);

        assert!(result.is_some());
        let candidate = result.unwrap();
        assert!((candidate.t - 0.5).abs() < 0.01); // Projects to middle
        assert!((candidate.distance - 0.1).abs() < 0.01); // 0.1m perpendicular
    }

    #[test]
    fn test_validate_correspondence_candidate_too_far() {
        let line = make_horizontal_line();
        let point = Point2D::new(1.0, 1.0); // 1m away from line

        let result = validate_correspondence_candidate(point, &line, 0.5, 0.2);

        assert!(result.is_none()); // Rejected: distance > 0.5
    }

    #[test]
    fn test_validate_correspondence_candidate_outside_projection() {
        let line = make_horizontal_line(); // (0,0) to (2,0)
        let point = Point2D::new(-1.0, 0.1); // Projects before line start

        // Strict: no extension
        let result_strict = validate_correspondence_candidate(point, &line, 0.5, 0.0);
        assert!(result_strict.is_none());

        // Relaxed: 0.5 extension (projects at t=-0.5, within [-0.5, 1.5])
        let result_relaxed = validate_correspondence_candidate(point, &line, 0.5, 0.6);
        assert!(result_relaxed.is_some());
    }

    #[test]
    fn test_validate_correspondence_candidate_at_endpoints() {
        let line = make_horizontal_line(); // (0,0) to (2,0)

        // At start
        let at_start = validate_correspondence_candidate(Point2D::new(0.0, 0.1), &line, 0.5, 0.0);
        assert!(at_start.is_some());
        assert!((at_start.unwrap().t - 0.0).abs() < 0.01);

        // At end
        let at_end = validate_correspondence_candidate(Point2D::new(2.0, 0.1), &line, 0.5, 0.0);
        assert!(at_end.is_some());
        assert!((at_end.unwrap().t - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_find_correspondences_single_point() {
        let lines = vec![make_horizontal_line()];
        let points = vec![Point2D::new(1.0, 0.1)]; // Close to middle of line

        let config = NearestNeighborConfig::default();
        let corrs = find_correspondences(&points, &lines, &config);

        assert_eq!(corrs.len(), 1);
        assert_eq!(corrs.correspondences[0].point_idx, 0);
        assert_eq!(corrs.correspondences[0].line_idx, 0);
        assert!((corrs.correspondences[0].distance - 0.1).abs() < 0.01);
    }

    #[test]
    fn test_find_correspondences_multiple_lines() {
        let lines = vec![make_horizontal_line(), make_vertical_line()];
        let points = vec![Point2D::new(0.05, 0.1)]; // Closer to vertical line

        let config = NearestNeighborConfig::default();
        let corrs = find_correspondences(&points, &lines, &config);

        assert_eq!(corrs.len(), 1);
        // Should match vertical line (distance 0.05) not horizontal (distance 0.1)
        assert_eq!(corrs.correspondences[0].line_idx, 1);
        assert!((corrs.correspondences[0].distance - 0.05).abs() < 0.01);
    }

    #[test]
    fn test_find_correspondences_no_match_distance() {
        let lines = vec![make_horizontal_line()];
        let points = vec![Point2D::new(1.0, 1.0)]; // Too far from line

        let config = NearestNeighborConfig::default().with_max_distance(0.5);
        let corrs = find_correspondences(&points, &lines, &config);

        assert!(corrs.is_empty());
    }

    #[test]
    fn test_find_correspondences_no_match_projection() {
        let lines = vec![make_horizontal_line()]; // From (0,0) to (2,0)
        let points = vec![Point2D::new(-1.0, 0.1)]; // Projects before line start

        let config = NearestNeighborConfig::default().with_max_projection_extension(0.0);
        let corrs = find_correspondences(&points, &lines, &config);

        assert!(corrs.is_empty());
    }

    #[test]
    fn test_find_correspondences_projection_extension() {
        let lines = vec![make_horizontal_line()]; // From (0,0) to (2,0)
        let points = vec![Point2D::new(-0.2, 0.1)]; // Projects slightly before start

        // Without extension: no match
        let config_strict = NearestNeighborConfig::default().with_max_projection_extension(0.0);
        let corrs_strict = find_correspondences(&points, &lines, &config_strict);
        assert!(corrs_strict.is_empty());

        // With extension: match
        let config_relaxed = NearestNeighborConfig::default().with_max_projection_extension(0.2);
        let corrs_relaxed = find_correspondences(&points, &lines, &config_relaxed);
        assert_eq!(corrs_relaxed.len(), 1);
    }

    #[test]
    fn test_find_correspondences_multiple_per_point() {
        let lines = vec![
            make_horizontal_line(),
            Line2D::new(Point2D::new(0.0, 0.2), Point2D::new(2.0, 0.2)), // Parallel line
        ];
        let points = vec![Point2D::new(1.0, 0.1)]; // Between the two lines

        // Unique: only best match
        let config_unique = NearestNeighborConfig::default().with_unique_per_point(true);
        let corrs_unique = find_correspondences(&points, &lines, &config_unique);
        assert_eq!(corrs_unique.len(), 1);

        // Non-unique: both matches
        let config_multi = NearestNeighborConfig::default().with_unique_per_point(false);
        let corrs_multi = find_correspondences(&points, &lines, &config_multi);
        assert_eq!(corrs_multi.len(), 2);
    }

    #[test]
    fn test_find_correspondences_empty_inputs() {
        let config = NearestNeighborConfig::default();

        // Empty points
        let corrs1 = find_correspondences(&[], &[make_horizontal_line()], &config);
        assert!(corrs1.is_empty());

        // Empty lines
        let corrs2 = find_correspondences(&[Point2D::new(0.0, 0.0)], &[], &config);
        assert!(corrs2.is_empty());
    }

    #[test]
    fn test_projection_accuracy() {
        let line = make_horizontal_line(); // From (0,0) to (2,0)
        let points = vec![
            Point2D::new(0.0, 0.1), // At start, t ≈ 0
            Point2D::new(1.0, 0.1), // At middle, t ≈ 0.5
            Point2D::new(2.0, 0.1), // At end, t ≈ 1
        ];

        let config = NearestNeighborConfig::default();
        let corrs = find_correspondences(&points, &[line], &config);

        assert_eq!(corrs.len(), 3);
        assert!((corrs.correspondences[0].projection_t - 0.0).abs() < 0.01);
        assert!((corrs.correspondences[1].projection_t - 0.5).abs() < 0.01);
        assert!((corrs.correspondences[2].projection_t - 1.0).abs() < 0.01);
    }

    // === Additional edge case tests ===

    #[test]
    fn test_very_close_parallel_lines() {
        // Two parallel lines very close together - test tie-breaking
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 0.02), Point2D::new(5.0, 0.02)), // 2cm apart
        ];

        // Point exactly between the two lines
        let points = vec![Point2D::new(2.5, 0.01)];

        let config = NearestNeighborConfig::default().with_unique_per_point(true);
        let corrs = find_correspondences(&points, &lines, &config);

        // Should match one line (the closest one)
        assert_eq!(corrs.len(), 1);
        // Distance should be ~0.01m to either line
        assert!(corrs.correspondences[0].distance < 0.015);
    }

    #[test]
    fn test_large_line_collection() {
        // Test with many lines to verify performance doesn't degrade
        let mut lines = Vec::new();

        // Create a grid of lines (100 lines)
        for i in 0..10 {
            // Horizontal lines
            lines.push(Line2D::new(
                Point2D::new(0.0, i as f32 * 0.5),
                Point2D::new(5.0, i as f32 * 0.5),
            ));
            // Vertical lines
            lines.push(Line2D::new(
                Point2D::new(i as f32 * 0.5, 0.0),
                Point2D::new(i as f32 * 0.5, 5.0),
            ));
        }

        // Many points
        let points: Vec<Point2D> = (0..50)
            .map(|i| Point2D::new((i % 10) as f32 * 0.5 + 0.1, (i / 10) as f32 * 0.5 + 0.1))
            .collect();

        let config = NearestNeighborConfig::default()
            .with_max_distance(0.5)
            .with_unique_per_point(true);

        let corrs = find_correspondences(&points, &lines, &config);

        // Should find correspondences for most points
        assert!(
            corrs.len() > 30,
            "Expected >30 matches, got {}",
            corrs.len()
        );
    }

    #[test]
    fn test_config_default_values() {
        let config = NearestNeighborConfig::default();

        assert_eq!(config.max_distance, 0.5);
        assert_eq!(config.max_projection_extension, 0.2);
        assert!(config.unique_per_point);
    }

    #[test]
    fn test_multiple_points_same_line() {
        // Multiple points all matching the same line
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0))];

        let points = vec![
            Point2D::new(1.0, 0.05),
            Point2D::new(3.0, 0.08),
            Point2D::new(5.0, 0.03),
            Point2D::new(7.0, 0.06),
            Point2D::new(9.0, 0.04),
        ];

        let config = NearestNeighborConfig::default();
        let corrs = find_correspondences(&points, &lines, &config);

        // All points should match the same line
        assert_eq!(corrs.len(), 5);
        for corr in corrs.iter() {
            assert_eq!(corr.line_idx, 0);
        }
    }

    #[test]
    fn test_non_unique_mode_with_many_lines() {
        // Test non-unique mode returns all valid matches
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 0.1), Point2D::new(5.0, 0.1)),
            Line2D::new(Point2D::new(0.0, 0.2), Point2D::new(5.0, 0.2)),
        ];

        // Point between all three lines
        let points = vec![Point2D::new(2.5, 0.1)];

        let config = NearestNeighborConfig::default()
            .with_max_distance(0.2)
            .with_unique_per_point(false);

        let corrs = find_correspondences(&points, &lines, &config);

        // Should match all three lines (distances: 0.1, 0.0, 0.1)
        assert_eq!(corrs.len(), 3, "Should match all 3 parallel lines");
    }
}
