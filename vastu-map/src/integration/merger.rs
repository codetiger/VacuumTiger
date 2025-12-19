//! Feature merging for map integration.
//!
//! Merges associated lines into updated map lines, extending
//! endpoints and updating parameters based on observations.

use crate::core::Point2D;
use crate::features::Line2D;

use super::association::Association;

/// Configuration for line merging.
#[derive(Clone, Debug)]
pub struct MergerConfig {
    /// Weight given to existing map line (vs. scan line).
    /// Higher values make the map more stable but slower to adapt.
    /// Default: 0.8 (map line has 80% weight)
    pub map_weight: f32,

    /// Whether to extend line endpoints based on observations.
    /// Default: true
    pub extend_endpoints: bool,

    /// Maximum endpoint extension per observation (meters).
    /// Default: 0.5m
    pub max_extension: f32,

    /// Minimum extension required to actually extend (meters).
    /// Small extensions are ignored to avoid noise accumulation.
    /// Default: 0.05m
    pub min_extension: f32,
}

impl Default for MergerConfig {
    fn default() -> Self {
        Self {
            map_weight: 0.8,
            extend_endpoints: true,
            max_extension: 0.5,
            min_extension: 0.05,
        }
    }
}

impl MergerConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for map weight.
    pub fn with_map_weight(mut self, weight: f32) -> Self {
        self.map_weight = weight;
        self
    }

    /// Builder-style setter for endpoint extension.
    pub fn with_extend_endpoints(mut self, extend: bool) -> Self {
        self.extend_endpoints = extend;
        self
    }

    /// Builder-style setter for maximum extension.
    pub fn with_max_extension(mut self, meters: f32) -> Self {
        self.max_extension = meters;
        self
    }
}

/// Result of merging a line.
#[derive(Clone, Debug)]
pub struct MergeResult {
    /// The merged line.
    pub line: Line2D,
    /// Whether the line was extended at the start.
    pub extended_start: bool,
    /// Whether the line was extended at the end.
    pub extended_end: bool,
    /// Amount extended at start (meters, positive = extended).
    pub start_extension: f32,
    /// Amount extended at end (meters, positive = extended).
    pub end_extension: f32,
}

impl MergeResult {
    /// Create a new merge result with no extension.
    pub fn new(line: Line2D) -> Self {
        Self {
            line,
            extended_start: false,
            extended_end: false,
            start_extension: 0.0,
            end_extension: 0.0,
        }
    }
}

/// Merge a scan line with a map line based on their association.
///
/// # Arguments
/// * `scan_line` - The new observation
/// * `map_line` - The existing map line
/// * `association` - Association between the lines
/// * `config` - Merger configuration
///
/// # Returns
/// Merge result with the updated line.
pub fn merge_lines(
    scan_line: &Line2D,
    map_line: &Line2D,
    _association: &Association,
    config: &MergerConfig,
) -> MergeResult {
    let scan_weight = 1.0 - config.map_weight;

    // Merge direction using weighted average
    let map_dir = map_line.unit_direction();
    let scan_dir = scan_line.unit_direction();

    // Ensure directions are aligned (not pointing opposite ways)
    let dot = map_dir.dot(scan_dir);
    let aligned_scan_dir = if dot < 0.0 { -scan_dir } else { scan_dir };

    // Weighted average of directions
    let merged_dir = Point2D::new(
        map_dir.x * config.map_weight + aligned_scan_dir.x * scan_weight,
        map_dir.y * config.map_weight + aligned_scan_dir.y * scan_weight,
    )
    .normalized();

    // Get all four endpoints
    let scan_start = scan_line.start;
    let scan_end = scan_line.end;
    let map_start = map_line.start;
    let map_end = map_line.end;

    // Project all endpoints onto the merged direction line
    // Use the map line's midpoint as reference
    let reference = map_line.midpoint();

    let project = |p: Point2D| -> f32 { (p - reference).dot(merged_dir) };

    let scan_start_t = project(scan_start);
    let scan_end_t = project(scan_end);
    let map_start_t = project(map_start);
    let map_end_t = project(map_end);

    // Find the extent of the merged line
    let mut min_t = map_start_t.min(map_end_t);
    let mut max_t = map_start_t.max(map_end_t);

    let mut extended_start = false;
    let mut extended_end = false;
    let mut start_extension = 0.0f32;
    let mut end_extension = 0.0f32;

    if config.extend_endpoints {
        // Check if scan extends beyond map
        let scan_min_t = scan_start_t.min(scan_end_t);
        let scan_max_t = scan_start_t.max(scan_end_t);

        if scan_min_t < min_t {
            let extension = (min_t - scan_min_t).min(config.max_extension);
            if extension >= config.min_extension {
                min_t -= extension;
                extended_start = true;
                start_extension = extension;
            }
        }

        if scan_max_t > max_t {
            let extension = (scan_max_t - max_t).min(config.max_extension);
            if extension >= config.min_extension {
                max_t += extension;
                extended_end = true;
                end_extension = extension;
            }
        }
    }

    // Compute new endpoints
    let new_start = reference + merged_dir * min_t;
    let new_end = reference + merged_dir * max_t;

    // Average perpendicular offset (to preserve line position)
    let map_perp = map_line.normal();
    let map_offset = (map_start - reference).dot(map_perp);
    let scan_perp_offset = (scan_line.midpoint() - reference).dot(map_perp);
    let merged_offset = map_offset * config.map_weight + scan_perp_offset * scan_weight;

    let perp = Point2D::new(-merged_dir.y, merged_dir.x);
    let final_start = new_start + perp * merged_offset;
    let final_end = new_end + perp * merged_offset;

    // Create merged line with incremented observation count
    let merged_line = Line2D::with_observation_count(
        final_start,
        final_end,
        map_line.observation_count.saturating_add(1),
    );

    MergeResult {
        line: merged_line,
        extended_start,
        extended_end,
        start_extension,
        end_extension,
    }
}

/// Batch merge multiple associations.
///
/// # Arguments
/// * `scan_lines` - All scan lines
/// * `map_lines` - All map lines (mutable for in-place update)
/// * `associations` - Associations between scan and map lines
/// * `config` - Merger configuration
///
/// # Returns
/// Vector of (map_line_idx, MergeResult) for each merge performed.
pub fn batch_merge(
    scan_lines: &[Line2D],
    map_lines: &mut [Line2D],
    associations: &[Association],
    config: &MergerConfig,
) -> Vec<(usize, MergeResult)> {
    let mut results = Vec::with_capacity(associations.len());

    for assoc in associations {
        let scan_line = &scan_lines[assoc.scan_line_idx];
        let map_line = &map_lines[assoc.map_line_idx];

        let result = merge_lines(scan_line, map_line, assoc, config);

        // Update map line in place
        map_lines[assoc.map_line_idx] = result.line;

        results.push((assoc.map_line_idx, result));
    }

    results
}

/// Create a new line from a scan observation (for unmatched scan lines).
///
/// This is used when adding new features to the map.
pub fn create_new_line(scan_line: &Line2D) -> Line2D {
    Line2D::with_observation_count(scan_line.start, scan_line.end, 1)
}

/// Merge two line segments that represent the same physical feature.
///
/// This is a simpler merge that just extends the endpoints.
pub fn merge_collinear_lines(line1: &Line2D, line2: &Line2D) -> Option<Line2D> {
    // Check if lines are approximately collinear
    let angle_diff = crate::core::math::angle_diff(line1.angle(), line2.angle()).abs();
    let angle_diff = angle_diff.min(std::f32::consts::PI - angle_diff);

    if angle_diff > 0.1 {
        // ~5.7 degrees
        return None;
    }

    // Check perpendicular distance
    let dist1 = line1.distance_to_point(line2.start);
    let dist2 = line1.distance_to_point(line2.end);
    let max_dist = dist1.max(dist2);

    if max_dist > 0.1 {
        return None;
    }

    // Merge by finding the extended endpoints
    let dir = line1.unit_direction();
    let reference = line1.midpoint();

    let project = |p: Point2D| -> f32 { (p - reference).dot(dir) };

    let t1 = project(line1.start);
    let t2 = project(line1.end);
    let t3 = project(line2.start);
    let t4 = project(line2.end);

    let min_t = t1.min(t2).min(t3).min(t4);
    let max_t = t1.max(t2).max(t3).max(t4);

    let new_start = reference + dir * min_t;
    let new_end = reference + dir * max_t;

    let count = line1
        .observation_count
        .saturating_add(line2.observation_count);

    Some(Line2D::with_observation_count(new_start, new_end, count))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_merge_identical_lines() {
        let scan = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let map = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let assoc = Association::new(0, 0, 0.0, 0.0, 5.0, 1.0);

        let config = MergerConfig::default();
        let result = merge_lines(&scan, &map, &assoc, &config);

        assert_relative_eq!(result.line.length(), 5.0, epsilon = 0.01);
        assert!(!result.extended_start);
        assert!(!result.extended_end);
    }

    #[test]
    fn test_merge_extends_line() {
        let scan = Line2D::new(Point2D::new(-1.0, 0.0), Point2D::new(6.0, 0.0));
        let map = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let assoc = Association::new(0, 0, 0.0, 0.0, 5.0, 1.0);

        let config = MergerConfig::default();
        let result = merge_lines(&scan, &map, &assoc, &config);

        assert!(result.line.length() > 5.0);
        assert!(result.extended_start || result.extended_end);
    }

    #[test]
    fn test_merge_respects_max_extension() {
        let scan = Line2D::new(Point2D::new(-5.0, 0.0), Point2D::new(10.0, 0.0));
        let map = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let assoc = Association::new(0, 0, 0.0, 0.0, 5.0, 1.0);

        let config = MergerConfig::default().with_max_extension(0.5);
        let result = merge_lines(&scan, &map, &assoc, &config);

        // Should not extend by more than 0.5m on each side
        assert!(result.line.length() <= 6.0); // Original 5.0 + max 0.5 + 0.5
    }

    #[test]
    fn test_merge_weighted_average() {
        // Scan line slightly offset from map line
        let scan = Line2D::new(Point2D::new(0.0, 0.1), Point2D::new(5.0, 0.1));
        let map = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let assoc = Association::new(0, 0, 0.1, 0.0, 5.0, 1.0);

        let config = MergerConfig::default().with_map_weight(0.8);
        let result = merge_lines(&scan, &map, &assoc, &config);

        // Result should be closer to map line (weighted 80%)
        let mid_y = result.line.midpoint().y;
        assert!(mid_y < 0.05); // Closer to map's y=0 than scan's y=0.1
    }

    #[test]
    fn test_batch_merge() {
        let scan_lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
        ];
        let mut map_lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
        ];
        let associations = vec![
            Association::new(0, 0, 0.0, 0.0, 5.0, 1.0),
            Association::new(1, 1, 0.0, 0.0, 5.0, 1.0),
        ];

        let config = MergerConfig::default();
        let results = batch_merge(&scan_lines, &mut map_lines, &associations, &config);

        assert_eq!(results.len(), 2);
        // Observation counts should be incremented
        assert!(map_lines[0].observation_count > 1);
        assert!(map_lines[1].observation_count > 1);
    }

    #[test]
    fn test_create_new_line() {
        let scan = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let new_line = create_new_line(&scan);

        assert_eq!(new_line.start, scan.start);
        assert_eq!(new_line.end, scan.end);
        assert_eq!(new_line.observation_count, 1);
    }

    #[test]
    fn test_merge_collinear_lines() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(3.0, 0.0));
        let line2 = Line2D::new(Point2D::new(2.0, 0.0), Point2D::new(5.0, 0.0));

        let merged = merge_collinear_lines(&line1, &line2);

        assert!(merged.is_some());
        let merged = merged.unwrap();
        assert_relative_eq!(merged.length(), 5.0, epsilon = 0.01);
    }

    #[test]
    fn test_merge_collinear_rejects_perpendicular() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let line2 = Line2D::new(Point2D::new(2.5, -2.0), Point2D::new(2.5, 2.0));

        let merged = merge_collinear_lines(&line1, &line2);

        assert!(merged.is_none());
    }

    #[test]
    fn test_merge_collinear_rejects_far() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let line2 = Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(5.0, 1.0));

        let merged = merge_collinear_lines(&line1, &line2);

        assert!(merged.is_none());
    }
}
