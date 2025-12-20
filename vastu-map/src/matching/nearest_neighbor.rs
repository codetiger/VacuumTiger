//! Nearest neighbor matching for scan-to-map correspondence.
//!
//! Finds correspondences between scan points and map lines using
//! distance-based matching with geometric constraints.
//!
//! # Performance
//!
//! Three variants are provided:
//! - `find_correspondences`: Basic O(n×m) brute force
//! - `find_correspondences_batch`: SIMD-optimized O(n×m) using LineCollection
//! - `find_correspondences_spatial`: O(n×log(m)) using R-tree spatial index
//!
//! Use `find_correspondences_spatial` for large maps (>50 lines).
//!
//! # Weighted Correspondences
//!
//! For uncertainty-aware matching, use `find_correspondences_weighted` which
//! computes weights based on range using a LidarNoiseModel.

use crate::config::LidarNoiseModel;
use crate::core::Point2D;
use crate::features::{Corner2D, Line2D, LineCollection};
use crate::integration::SpatialIndex;

use super::correspondence::{
    CornerCorrespondence, CornerCorrespondenceSet, Correspondence, CorrespondenceSet,
};

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
    let min_t = -config.max_projection_extension;
    let max_t = 1.0 + config.max_projection_extension;

    for (point_idx, &point) in points.iter().enumerate() {
        let mut best_distance = config.max_distance;
        let mut best_corr: Option<Correspondence> = None;

        for (line_idx, line) in lines.iter().enumerate() {
            // Compute projection parameter
            let t = line.project_point(point);

            // Check projection bounds
            if t < min_t || t > max_t {
                continue;
            }

            // Compute perpendicular distance
            let distance = line.distance_to_point(point);

            // Check distance threshold
            if distance > config.max_distance {
                continue;
            }

            if config.unique_per_point {
                // Keep only the best match
                if distance < best_distance {
                    best_distance = distance;
                    best_corr = Some(Correspondence::new(point_idx, line_idx, point, distance, t));
                }
            } else {
                // Add all valid matches
                result.push(Correspondence::new(point_idx, line_idx, point, distance, t));
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

/// Find correspondences using SoA LineCollection (SIMD-optimized).
///
/// This version uses batch distance computation for better performance
/// on large line sets.
///
/// # Arguments
/// * `points` - Scan points in world frame
/// * `lines` - Map lines in SoA format
/// * `config` - Matching configuration
///
/// # Returns
/// Set of valid correspondences.
pub fn find_correspondences_batch(
    points: &[Point2D],
    lines: &LineCollection,
    config: &NearestNeighborConfig,
) -> CorrespondenceSet {
    if points.is_empty() || lines.is_empty() {
        return CorrespondenceSet::new();
    }

    let mut result = CorrespondenceSet::with_capacity(points.len());
    let min_t = -config.max_projection_extension;
    let max_t = 1.0 + config.max_projection_extension;
    let n_lines = lines.len();

    for (point_idx, &point) in points.iter().enumerate() {
        // Compute distances to all lines at once (SIMD-friendly)
        let distances = lines.distances_to_point(point);
        let projections = lines.project_point(point);

        let mut best_distance = config.max_distance;
        let mut best_corr: Option<Correspondence> = None;

        for line_idx in 0..n_lines {
            let t = projections[line_idx];
            let distance = distances[line_idx];

            // Check projection bounds
            if t < min_t || t > max_t {
                continue;
            }

            // Check distance threshold
            if distance > config.max_distance {
                continue;
            }

            if config.unique_per_point {
                if distance < best_distance {
                    best_distance = distance;
                    best_corr = Some(Correspondence::new(point_idx, line_idx, point, distance, t));
                }
            } else {
                result.push(Correspondence::new(point_idx, line_idx, point, distance, t));
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

/// Find correspondences with range-based weights using LineCollection.
///
/// Computes weights for each correspondence based on the range from sensor
/// to the point, using the provided noise model. Closer points are weighted
/// higher as they have less measurement uncertainty.
///
/// # Arguments
/// * `points` - Scan points in world frame
/// * `ranges` - Range (distance from sensor) for each point
/// * `lines` - Map lines in SoA format
/// * `noise_model` - Lidar noise model for weight computation
/// * `config` - Matching configuration
///
/// # Returns
/// Set of weighted correspondences.
pub fn find_correspondences_weighted(
    points: &[Point2D],
    ranges: &[f32],
    lines: &LineCollection,
    noise_model: &LidarNoiseModel,
    config: &NearestNeighborConfig,
) -> CorrespondenceSet {
    if points.is_empty() || lines.is_empty() {
        return CorrespondenceSet::new();
    }

    debug_assert_eq!(
        points.len(),
        ranges.len(),
        "points and ranges must have same length"
    );

    let mut result = CorrespondenceSet::with_capacity(points.len());
    let min_t = -config.max_projection_extension;
    let max_t = 1.0 + config.max_projection_extension;
    let n_lines = lines.len();

    for (point_idx, (&point, &range)) in points.iter().zip(ranges.iter()).enumerate() {
        // Compute distances to all lines at once (SIMD-friendly)
        let distances = lines.distances_to_point(point);
        let projections = lines.project_point(point);

        // Compute weight for this point based on its range
        let weight = noise_model.weight(range);

        let mut best_distance = config.max_distance;
        let mut best_corr: Option<Correspondence> = None;

        for line_idx in 0..n_lines {
            let t = projections[line_idx];
            let distance = distances[line_idx];

            // Check projection bounds
            if t < min_t || t > max_t {
                continue;
            }

            // Check distance threshold
            if distance > config.max_distance {
                continue;
            }

            if config.unique_per_point {
                if distance < best_distance {
                    best_distance = distance;
                    best_corr = Some(Correspondence::with_weight(
                        point_idx, line_idx, point, distance, t, weight, range,
                    ));
                }
            } else {
                result.push(Correspondence::with_weight(
                    point_idx, line_idx, point, distance, t, weight, range,
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

/// Find correspondences using R-tree spatial index (O(n log m) complexity).
///
/// This version uses a spatial index for fast nearest-neighbor queries,
/// making it much faster than brute-force for large maps (>50 lines).
///
/// # Arguments
/// * `points` - Scan points in world frame
/// * `lines` - Map lines to match against
/// * `index` - Pre-built spatial index for the lines
/// * `config` - Matching configuration
///
/// # Performance
///
/// - For small maps (<50 lines): Use `find_correspondences_batch` instead
/// - For large maps (>50 lines): This version is 5-10x faster
///
/// # Example
/// ```rust,ignore
/// use vastu_map::matching::{find_correspondences_spatial, NearestNeighborConfig};
/// use vastu_map::integration::SpatialIndex;
///
/// let lines = vec![/* map lines */];
/// let index = SpatialIndex::new(&lines);
///
/// for scan in scans {
///     // Reuse index across scans
///     let corrs = find_correspondences_spatial(&scan, &lines, &index, &config);
/// }
/// ```
pub fn find_correspondences_spatial(
    points: &[Point2D],
    lines: &[Line2D],
    index: &SpatialIndex,
    config: &NearestNeighborConfig,
) -> CorrespondenceSet {
    if points.is_empty() || lines.is_empty() {
        return CorrespondenceSet::new();
    }

    let mut result = CorrespondenceSet::with_capacity(points.len());
    let min_t = -config.max_projection_extension;
    let max_t = 1.0 + config.max_projection_extension;

    for (point_idx, &point) in points.iter().enumerate() {
        // Use spatial index to find candidate lines within distance threshold
        // This is O(log n) instead of O(n)
        let candidates = index.lines_within_distance(point, config.max_distance);

        if candidates.is_empty() {
            continue;
        }

        let mut best_distance = config.max_distance;
        let mut best_corr: Option<Correspondence> = None;

        for (line_idx, distance) in candidates {
            let line = &lines[line_idx];

            // Compute projection parameter
            let t = line.project_point(point);

            // Check projection bounds
            if t < min_t || t > max_t {
                continue;
            }

            if config.unique_per_point {
                if distance < best_distance {
                    best_distance = distance;
                    best_corr = Some(Correspondence::new(point_idx, line_idx, point, distance, t));
                }
            } else {
                result.push(Correspondence::new(point_idx, line_idx, point, distance, t));
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

/// Find correspondences with angle constraint.
///
/// Adds an additional constraint that the point's bearing from the sensor
/// should be approximately perpendicular to the line normal.
///
/// # Arguments
/// * `points` - Scan points in world frame
/// * `lines` - Map lines to match against
/// * `sensor_position` - Position of the sensor in world frame
/// * `max_angle_deviation` - Maximum deviation from perpendicular (radians)
/// * `config` - Matching configuration
pub fn find_correspondences_with_angle(
    points: &[Point2D],
    lines: &[Line2D],
    sensor_position: Point2D,
    max_angle_deviation: f32,
    config: &NearestNeighborConfig,
) -> CorrespondenceSet {
    if points.is_empty() || lines.is_empty() {
        return CorrespondenceSet::new();
    }

    let mut result = CorrespondenceSet::with_capacity(points.len());
    let min_t = -config.max_projection_extension;
    let max_t = 1.0 + config.max_projection_extension;
    let cos_threshold = max_angle_deviation.cos();

    for (point_idx, &point) in points.iter().enumerate() {
        // Compute bearing from sensor to point
        let bearing = (point - sensor_position).normalized();

        let mut best_distance = config.max_distance;
        let mut best_corr: Option<Correspondence> = None;

        for (line_idx, line) in lines.iter().enumerate() {
            // Check angle constraint
            let line_normal = line.normal();
            let dot = bearing.dot(line_normal).abs();

            // bearing should be roughly parallel to normal (hitting line head-on)
            if dot < cos_threshold {
                continue;
            }

            // Compute projection parameter
            let t = line.project_point(point);

            // Check projection bounds
            if t < min_t || t > max_t {
                continue;
            }

            // Compute perpendicular distance
            let distance = line.distance_to_point(point);

            // Check distance threshold
            if distance > config.max_distance {
                continue;
            }

            if config.unique_per_point {
                if distance < best_distance {
                    best_distance = distance;
                    best_corr = Some(Correspondence::new(point_idx, line_idx, point, distance, t));
                }
            } else {
                result.push(Correspondence::new(point_idx, line_idx, point, distance, t));
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

/// Configuration for corner correspondence matching.
#[derive(Clone, Debug)]
pub struct CornerMatchConfig {
    /// Maximum distance from point to corner for a valid correspondence (meters).
    /// Default: 0.15m (15cm)
    pub max_distance: f32,

    /// Whether to only keep the best correspondence per point.
    /// Default: true
    pub unique_per_point: bool,
}

impl Default for CornerMatchConfig {
    fn default() -> Self {
        Self {
            max_distance: 0.15,
            unique_per_point: true,
        }
    }
}

impl CornerMatchConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for maximum distance.
    pub fn with_max_distance(mut self, max_distance: f32) -> Self {
        self.max_distance = max_distance;
        self
    }

    /// Builder-style setter for unique per point.
    pub fn with_unique_per_point(mut self, unique: bool) -> Self {
        self.unique_per_point = unique;
        self
    }
}

/// Find correspondences between scan points and map corners.
///
/// Uses simple nearest-neighbor matching to associate scan points with
/// map corners within a distance threshold.
///
/// # Arguments
/// * `points` - Scan points in world frame (after initial pose transform)
/// * `corners` - Map corners to match against
/// * `config` - Matching configuration
///
/// # Returns
/// Set of corner correspondences, where each point is matched to its
/// nearest corner within the distance threshold.
///
/// # Example
/// ```rust,ignore
/// use vastu_map::matching::{find_corner_correspondences, CornerMatchConfig};
///
/// let corners = vec![/* map corners */];
/// let points = vec![/* scan points in world frame */];
/// let config = CornerMatchConfig::default();
///
/// let corrs = find_corner_correspondences(&points, &corners, &config);
/// println!("Found {} corner matches", corrs.len());
/// ```
pub fn find_corner_correspondences(
    points: &[Point2D],
    corners: &[Corner2D],
    config: &CornerMatchConfig,
) -> CornerCorrespondenceSet {
    if points.is_empty() || corners.is_empty() {
        return CornerCorrespondenceSet::new();
    }

    let mut result = CornerCorrespondenceSet::with_capacity(corners.len().min(points.len()));
    let max_dist_sq = config.max_distance * config.max_distance;

    for (point_idx, &point) in points.iter().enumerate() {
        let mut best_distance_sq = max_dist_sq;
        let mut best_corner_idx: Option<usize> = None;

        for (corner_idx, corner) in corners.iter().enumerate() {
            let dx = point.x - corner.position.x;
            let dy = point.y - corner.position.y;
            let distance_sq = dx * dx + dy * dy;

            if distance_sq < best_distance_sq {
                best_distance_sq = distance_sq;
                best_corner_idx = Some(corner_idx);

                // Early exit if we find a very close match
                if distance_sq < 0.001 {
                    break;
                }
            }
        }

        if let Some(corner_idx) = best_corner_idx {
            let distance = best_distance_sq.sqrt();
            let corner = &corners[corner_idx];
            result.push(CornerCorrespondence::new(
                point_idx,
                corner_idx,
                point,
                corner.position,
                distance,
            ));
        }
    }

    result
}

/// Find corner correspondences with weighting based on range.
///
/// Similar to `find_corner_correspondences`, but applies weights based on
/// the range from the sensor to each point using a lidar noise model.
///
/// # Arguments
/// * `points` - Scan points in world frame
/// * `robot_frame_points` - Original scan points in robot frame (for range computation)
/// * `corners` - Map corners to match against
/// * `config` - Matching configuration
/// * `noise_model` - Lidar noise model for weight computation
pub fn find_corner_correspondences_weighted(
    points: &[Point2D],
    robot_frame_points: &[Point2D],
    corners: &[Corner2D],
    config: &CornerMatchConfig,
    noise_model: &LidarNoiseModel,
) -> CornerCorrespondenceSet {
    let mut result = find_corner_correspondences(points, corners, config);

    // Apply weights based on range
    for corr in result.iter_mut() {
        if corr.point_idx < robot_frame_points.len() {
            let p = robot_frame_points[corr.point_idx];
            let range = (p.x * p.x + p.y * p.y).sqrt();
            corr.weight = noise_model.weight(range);
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
    fn test_find_correspondences_batch() {
        let mut collection = LineCollection::new();
        collection.push_line(&make_horizontal_line());
        collection.push_line(&make_vertical_line());

        let points = vec![
            Point2D::new(1.0, 0.1), // Near horizontal
            Point2D::new(0.1, 1.0), // Near vertical
        ];

        let config = NearestNeighborConfig::default();
        let corrs = find_correspondences_batch(&points, &collection, &config);

        assert_eq!(corrs.len(), 2);
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
    fn test_batch_vs_nonbatch_consistency() {
        // Results from batch and non-batch should be identical
        let lines_vec = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(0.0, 5.0)),
            Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(0.0, 0.0)),
        ];

        let mut collection = LineCollection::new();
        for line in &lines_vec {
            collection.push_line(line);
        }

        let points = vec![
            Point2D::new(2.5, 0.1),
            Point2D::new(4.9, 2.5),
            Point2D::new(2.5, 4.9),
            Point2D::new(0.1, 2.5),
        ];

        let config = NearestNeighborConfig::default().with_unique_per_point(true);

        let corrs_regular = find_correspondences(&points, &lines_vec, &config);
        let corrs_batch = find_correspondences_batch(&points, &collection, &config);

        // Same number of correspondences
        assert_eq!(corrs_regular.len(), corrs_batch.len());

        // Same line indices matched
        for i in 0..corrs_regular.len() {
            assert_eq!(
                corrs_regular.correspondences[i].line_idx, corrs_batch.correspondences[i].line_idx,
                "Line index mismatch at position {}",
                i
            );
            assert!(
                (corrs_regular.correspondences[i].distance
                    - corrs_batch.correspondences[i].distance)
                    .abs()
                    < 0.001,
                "Distance mismatch at position {}",
                i
            );
        }
    }

    #[test]
    fn test_find_correspondences_with_angle_basic() {
        // Test angle-constrained matching
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)), // Horizontal
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 5.0)), // Vertical
        ];

        // Sensor at center, point on horizontal line
        let sensor_pos = Point2D::new(2.5, 2.5);
        let points = vec![Point2D::new(2.5, 0.1)]; // Point below sensor, near horizontal line

        // Bearing from sensor to point is straight down (perpendicular to horizontal line normal)
        let config = NearestNeighborConfig::default();
        let max_angle_deviation = 0.5; // ~29 degrees

        let corrs = find_correspondences_with_angle(
            &points,
            &lines,
            sensor_pos,
            max_angle_deviation,
            &config,
        );

        // Should match horizontal line (bearing is perpendicular to line, parallel to normal)
        assert_eq!(corrs.len(), 1);
        assert_eq!(corrs.correspondences[0].line_idx, 0);
    }

    #[test]
    fn test_find_correspondences_with_angle_rejection() {
        // Test that angle constraint rejects bad matches
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)), // Horizontal
        ];

        // Sensor to the left, point to the right of sensor
        let sensor_pos = Point2D::new(0.0, 0.5);
        let points = vec![Point2D::new(2.5, 0.5)]; // Point to the right, bearing is horizontal

        // Bearing is parallel to line (perpendicular to normal) - should be rejected
        let config = NearestNeighborConfig::default();
        let max_angle_deviation = 0.3; // ~17 degrees - strict

        let corrs = find_correspondences_with_angle(
            &points,
            &lines,
            sensor_pos,
            max_angle_deviation,
            &config,
        );

        // Should reject because bearing is parallel to line, not hitting it head-on
        assert!(corrs.is_empty(), "Should reject grazing angle match");
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
    fn test_batch_empty_inputs() {
        let config = NearestNeighborConfig::default();

        // Empty points
        let collection = LineCollection::new();
        let corrs1 = find_correspondences_batch(&[], &collection, &config);
        assert!(corrs1.is_empty());

        // Empty lines
        let mut collection2 = LineCollection::new();
        collection2.push_line(&make_horizontal_line());
        let corrs2 = find_correspondences_batch(&[], &collection2, &config);
        assert!(corrs2.is_empty());
    }

    #[test]
    fn test_angle_with_empty_inputs() {
        let config = NearestNeighborConfig::default();
        let sensor_pos = Point2D::new(0.0, 0.0);

        // Empty points
        let corrs1 = find_correspondences_with_angle(
            &[],
            &[make_horizontal_line()],
            sensor_pos,
            0.5,
            &config,
        );
        assert!(corrs1.is_empty());

        // Empty lines
        let corrs2 = find_correspondences_with_angle(
            &[Point2D::new(1.0, 0.1)],
            &[],
            sensor_pos,
            0.5,
            &config,
        );
        assert!(corrs2.is_empty());
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

    // === Corner correspondence tests ===

    fn make_corner(x: f32, y: f32) -> Corner2D {
        Corner2D::new(Point2D::new(x, y), 0, 1, std::f32::consts::FRAC_PI_2)
    }

    #[test]
    fn test_find_corner_correspondences_basic() {
        let corners = vec![
            make_corner(0.0, 0.0),
            make_corner(5.0, 0.0),
            make_corner(5.0, 5.0),
            make_corner(0.0, 5.0),
        ];

        // Point near corner 0
        let points = vec![Point2D::new(0.05, 0.02)];

        let config = CornerMatchConfig::default();
        let corrs = find_corner_correspondences(&points, &corners, &config);

        assert_eq!(corrs.len(), 1);
        assert_eq!(corrs.correspondences[0].corner_idx, 0);
        // Distance should be sqrt(0.05^2 + 0.02^2) ≈ 0.054
        assert!(corrs.correspondences[0].distance < 0.06);
    }

    #[test]
    fn test_find_corner_correspondences_multiple_points() {
        let corners = vec![
            make_corner(0.0, 0.0),
            make_corner(5.0, 0.0),
            make_corner(5.0, 5.0),
            make_corner(0.0, 5.0),
        ];

        // Points near each corner
        let points = vec![
            Point2D::new(0.05, 0.02), // Near corner 0
            Point2D::new(4.98, 0.01), // Near corner 1
            Point2D::new(5.02, 4.97), // Near corner 2
            Point2D::new(0.01, 5.03), // Near corner 3
        ];

        let config = CornerMatchConfig::default();
        let corrs = find_corner_correspondences(&points, &corners, &config);

        assert_eq!(corrs.len(), 4);
        // Check that each point matched the expected corner
        assert_eq!(corrs.correspondences[0].corner_idx, 0);
        assert_eq!(corrs.correspondences[1].corner_idx, 1);
        assert_eq!(corrs.correspondences[2].corner_idx, 2);
        assert_eq!(corrs.correspondences[3].corner_idx, 3);
    }

    #[test]
    fn test_find_corner_correspondences_no_match() {
        let corners = vec![make_corner(0.0, 0.0)];
        let points = vec![Point2D::new(1.0, 1.0)]; // Too far from corner

        let config = CornerMatchConfig::default().with_max_distance(0.1);
        let corrs = find_corner_correspondences(&points, &corners, &config);

        assert!(corrs.is_empty());
    }

    #[test]
    fn test_find_corner_correspondences_empty_inputs() {
        let config = CornerMatchConfig::default();

        // Empty points
        let corrs1 = find_corner_correspondences(&[], &[make_corner(0.0, 0.0)], &config);
        assert!(corrs1.is_empty());

        // Empty corners
        let corrs2 = find_corner_correspondences(&[Point2D::new(0.0, 0.0)], &[], &config);
        assert!(corrs2.is_empty());
    }

    #[test]
    fn test_corner_match_config_builder() {
        let config = CornerMatchConfig::new()
            .with_max_distance(0.2)
            .with_unique_per_point(false);

        assert_eq!(config.max_distance, 0.2);
        assert!(!config.unique_per_point);
    }
}
