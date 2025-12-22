//! Feature merging for map integration.
//!
//! Merges associated lines into updated map lines, extending
//! endpoints and updating parameters based on observations.

use serde::{Deserialize, Serialize};

use crate::core::Point2D;
use crate::features::Line2D;

use super::association::Association;

/// Configuration for line merging.
#[derive(Clone, Debug, Serialize, Deserialize)]
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

/// Configuration for coplanar line merging optimization.
///
/// Used by [`optimize_coplanar_lines`] to merge multiple lines
/// that lie on the same infinite line (within tolerances).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CoplanarMergeConfig {
    /// Maximum angle difference for lines to be considered coplanar (radians).
    /// Default: 0.052 rad (~3°)
    pub max_angle_diff: f32,

    /// Maximum perpendicular distance between lines (meters).
    /// Default: 0.05m (5cm)
    pub max_perpendicular_dist: f32,

    /// Maximum gap between line endpoints to allow merging (meters).
    /// Lines that are coplanar but separated by more than this gap
    /// will not be merged.
    /// Default: 0.3m (30cm)
    pub max_gap: f32,

    /// Minimum point count for a line to become the dominant line.
    /// Lines with fewer scan points may be absorbed but won't be chosen
    /// as the base direction for merging.
    /// Default: 10
    pub min_dominant_point_count: u32,
}

impl Default for CoplanarMergeConfig {
    fn default() -> Self {
        Self {
            max_angle_diff: 0.052,        // ~3 degrees
            max_perpendicular_dist: 0.05, // 5cm
            max_gap: 0.3,                 // 30cm
            min_dominant_point_count: 10,
        }
    }
}

impl CoplanarMergeConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for maximum angle difference.
    pub fn with_max_angle_diff(mut self, radians: f32) -> Self {
        self.max_angle_diff = radians;
        self
    }

    /// Builder-style setter for maximum perpendicular distance.
    pub fn with_max_perpendicular_dist(mut self, meters: f32) -> Self {
        self.max_perpendicular_dist = meters;
        self
    }

    /// Builder-style setter for maximum gap.
    pub fn with_max_gap(mut self, meters: f32) -> Self {
        self.max_gap = meters;
        self
    }

    /// Builder-style setter for minimum dominant point count.
    pub fn with_min_dominant_point_count(mut self, count: u32) -> Self {
        self.min_dominant_point_count = count;
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

    // Create merged line with incremented observation count and summed point count
    let merged_line = Line2D::full(
        final_start,
        final_end,
        map_line.observation_count.saturating_add(1),
        map_line.point_count.saturating_add(scan_line.point_count),
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
/// Preserves the scan line's point_count for dominance tracking.
pub fn create_new_line(scan_line: &Line2D) -> Line2D {
    Line2D::full(scan_line.start, scan_line.end, 1, scan_line.point_count)
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

    let obs_count = line1
        .observation_count
        .saturating_add(line2.observation_count);
    let pt_count = line1.point_count.saturating_add(line2.point_count);

    Some(Line2D::full(new_start, new_end, obs_count, pt_count))
}

/// Merge two line segments using configurable thresholds.
///
/// This is like [`merge_collinear_lines`] but uses explicit configuration
/// instead of hardcoded values.
pub fn merge_collinear_lines_with_config(
    line1: &Line2D,
    line2: &Line2D,
    config: &CoplanarMergeConfig,
) -> Option<Line2D> {
    if !are_lines_coplanar(line1, line2, config) {
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

    let obs_count = line1
        .observation_count
        .saturating_add(line2.observation_count);
    let pt_count = line1.point_count.saturating_add(line2.point_count);

    Some(Line2D::full(new_start, new_end, obs_count, pt_count))
}

/// Check if two lines are coplanar (on the same infinite line within tolerances).
///
/// Two lines are considered coplanar if:
/// 1. Their angles differ by less than `max_angle_diff` (handling 180° ambiguity)
/// 2. The perpendicular distance between them is less than `max_perpendicular_dist`
fn are_lines_coplanar(line1: &Line2D, line2: &Line2D, config: &CoplanarMergeConfig) -> bool {
    // Check angular compatibility
    let angle_diff = crate::core::math::angle_diff(line1.angle(), line2.angle()).abs();
    let angle_diff = angle_diff.min(std::f32::consts::PI - angle_diff);

    if angle_diff > config.max_angle_diff {
        return false;
    }

    // Check perpendicular distance using midpoint of line2
    let dist = line1.distance_to_point(line2.midpoint());
    dist <= config.max_perpendicular_dist
}

/// Check if two coplanar lines can be merged (overlap or small gap).
///
/// Lines can be merged if they are coplanar AND either:
/// - They overlap, OR
/// - The gap between their closest endpoints is less than `max_gap`
fn lines_can_merge(line1: &Line2D, line2: &Line2D, config: &CoplanarMergeConfig) -> bool {
    if !are_lines_coplanar(line1, line2, config) {
        return false;
    }

    // Project line2's endpoints onto line1's direction to check overlap/gap
    let dir = line1.unit_direction();
    let reference = line1.midpoint();

    let project = |p: Point2D| -> f32 { (p - reference).dot(dir) };

    let t1_min = project(line1.start).min(project(line1.end));
    let t1_max = project(line1.start).max(project(line1.end));
    let t2_min = project(line2.start).min(project(line2.end));
    let t2_max = project(line2.start).max(project(line2.end));

    // Check for overlap
    if t1_max >= t2_min && t2_max >= t1_min {
        return true;
    }

    // Check gap
    let gap = if t1_max < t2_min {
        t2_min - t1_max
    } else {
        t1_min - t2_max
    };

    gap <= config.max_gap
}

/// Merge a group of coplanar lines into a single extended line.
///
/// Uses the line with most scan points as the base direction (for robust dominance).
/// Falls back to observation_count if no line has point_count set.
/// Both observation_count and point_count are summed.
///
/// # Panics
/// Panics if `lines` is empty.
fn merge_coplanar_group(lines: &[&Line2D]) -> Line2D {
    assert!(!lines.is_empty(), "Cannot merge empty group");

    if lines.len() == 1 {
        return *lines[0];
    }

    // Find the dominant line (most scan points, fallback to observation count)
    let dominant = lines
        .iter()
        .max_by_key(|l| {
            // Use point_count if available, otherwise fall back to observation_count
            if l.point_count > 0 {
                l.point_count
            } else {
                l.observation_count
            }
        })
        .unwrap();

    let dir = dominant.unit_direction();
    let reference = dominant.midpoint();

    let project = |p: Point2D| -> f32 { (p - reference).dot(dir) };

    // Find extent across all lines and sum counts
    let mut min_t = f32::MAX;
    let mut max_t = f32::MIN;
    let mut total_obs_count: u32 = 0;
    let mut total_pt_count: u32 = 0;

    for line in lines {
        let t_start = project(line.start);
        let t_end = project(line.end);
        min_t = min_t.min(t_start).min(t_end);
        max_t = max_t.max(t_start).max(t_end);
        total_obs_count = total_obs_count.saturating_add(line.observation_count);
        total_pt_count = total_pt_count.saturating_add(line.point_count);
    }

    let new_start = reference + dir * min_t;
    let new_end = reference + dir * max_t;

    Line2D::full(new_start, new_end, total_obs_count, total_pt_count)
}

/// Optimize map lines by merging coplanar segments.
///
/// Groups lines by coplanarity (similar angle + perpendicular offset) and merges
/// each group into a single extended line using the most-observed line as the base.
///
/// # Arguments
/// * `lines` - Mutable vector of lines to optimize (modified in place)
/// * `config` - Coplanar merge configuration
///
/// # Returns
/// Number of lines merged (removed from the vector)
///
/// # Example
/// ```rust,ignore
/// use vastu_map::integration::{optimize_coplanar_lines, CoplanarMergeConfig};
///
/// let mut lines = vec![...]; // Multiple lines on the same wall
/// let config = CoplanarMergeConfig::default();
/// let merged_count = optimize_coplanar_lines(&mut lines, &config);
/// println!("Merged {} lines", merged_count);
/// ```
pub fn optimize_coplanar_lines(lines: &mut Vec<Line2D>, config: &CoplanarMergeConfig) -> usize {
    if lines.len() < 2 {
        return 0;
    }

    let n = lines.len();

    // Union-Find for grouping coplanar lines
    let mut parent: Vec<usize> = (0..n).collect();
    let mut rank: Vec<usize> = vec![0; n];

    fn find(parent: &mut [usize], i: usize) -> usize {
        if parent[i] != i {
            parent[i] = find(parent, parent[i]);
        }
        parent[i]
    }

    fn union(parent: &mut [usize], rank: &mut [usize], i: usize, j: usize) {
        let pi = find(parent, i);
        let pj = find(parent, j);
        if pi != pj {
            if rank[pi] < rank[pj] {
                parent[pi] = pj;
            } else if rank[pi] > rank[pj] {
                parent[pj] = pi;
            } else {
                parent[pj] = pi;
                rank[pi] += 1;
            }
        }
    }

    // Group coplanar lines that can be merged
    for i in 0..n {
        for j in (i + 1)..n {
            if lines_can_merge(&lines[i], &lines[j], config) {
                union(&mut parent, &mut rank, i, j);
            }
        }
    }

    // Collect groups
    let mut groups: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
    for i in 0..n {
        let root = find(&mut parent, i);
        groups.entry(root).or_default().push(i);
    }

    // Process each group and build result
    let mut result: Vec<Line2D> = Vec::with_capacity(n);
    let mut merged_count = 0;

    for (_root, indices) in groups {
        if indices.len() == 1 {
            // Single line, keep as-is
            result.push(lines[indices[0]]);
        } else {
            // Check if any line in the group has enough scan points to be dominant
            let has_dominant = indices.iter().any(|&i| {
                let line = &lines[i];
                // Use point_count if available, fall back to observation_count
                let confidence = if line.point_count > 0 {
                    line.point_count
                } else {
                    line.observation_count
                };
                confidence >= config.min_dominant_point_count
            });

            if !has_dominant {
                // No dominant line, keep all lines as-is
                for &idx in &indices {
                    result.push(lines[idx]);
                }
            } else {
                // Merge the group
                let group_lines: Vec<&Line2D> = indices.iter().map(|&i| &lines[i]).collect();
                let merged = merge_coplanar_group(&group_lines);
                result.push(merged);
                merged_count += indices.len() - 1;
            }
        }
    }

    *lines = result;
    merged_count
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

    // ===== Coplanar merge config tests =====

    #[test]
    fn test_coplanar_merge_config_default() {
        let config = CoplanarMergeConfig::default();

        assert_relative_eq!(config.max_angle_diff, 0.052, epsilon = 0.001);
        assert_relative_eq!(config.max_perpendicular_dist, 0.05, epsilon = 0.001);
        assert_relative_eq!(config.max_gap, 0.3, epsilon = 0.001);
        assert_eq!(config.min_dominant_point_count, 10);
    }

    #[test]
    fn test_coplanar_merge_config_builders() {
        let config = CoplanarMergeConfig::new()
            .with_max_angle_diff(0.2)
            .with_max_perpendicular_dist(0.08)
            .with_max_gap(1.0)
            .with_min_dominant_point_count(5);

        assert_relative_eq!(config.max_angle_diff, 0.2, epsilon = 0.001);
        assert_relative_eq!(config.max_perpendicular_dist, 0.08, epsilon = 0.001);
        assert_relative_eq!(config.max_gap, 1.0, epsilon = 0.001);
        assert_eq!(config.min_dominant_point_count, 5);
    }

    // ===== are_lines_coplanar tests =====

    #[test]
    fn test_are_lines_coplanar_same_line() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let line2 = Line2D::new(Point2D::new(3.0, 0.0), Point2D::new(8.0, 0.0));
        let config = CoplanarMergeConfig::default();

        assert!(are_lines_coplanar(&line1, &line2, &config));
    }

    #[test]
    fn test_are_lines_coplanar_parallel_close() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let line2 = Line2D::new(Point2D::new(0.0, 0.05), Point2D::new(5.0, 0.05)); // 5cm offset
        let config = CoplanarMergeConfig::default();

        assert!(are_lines_coplanar(&line1, &line2, &config));
    }

    #[test]
    fn test_are_lines_coplanar_rejects_angle() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let line2 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 1.0)); // ~11° angle
        let config = CoplanarMergeConfig::default();

        assert!(!are_lines_coplanar(&line1, &line2, &config));
    }

    #[test]
    fn test_are_lines_coplanar_rejects_distance() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let line2 = Line2D::new(Point2D::new(0.0, 0.5), Point2D::new(5.0, 0.5)); // 50cm offset
        let config = CoplanarMergeConfig::default();

        assert!(!are_lines_coplanar(&line1, &line2, &config));
    }

    #[test]
    fn test_are_lines_coplanar_opposite_direction() {
        // Lines pointing in opposite directions should still be coplanar
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let line2 = Line2D::new(Point2D::new(10.0, 0.0), Point2D::new(6.0, 0.0)); // Opposite direction
        let config = CoplanarMergeConfig::default();

        assert!(are_lines_coplanar(&line1, &line2, &config));
    }

    // ===== lines_can_merge tests =====

    #[test]
    fn test_lines_can_merge_overlapping() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));
        let line2 = Line2D::new(Point2D::new(3.0, 0.0), Point2D::new(8.0, 0.0));
        let config = CoplanarMergeConfig::default();

        assert!(lines_can_merge(&line1, &line2, &config));
    }

    #[test]
    fn test_lines_can_merge_with_small_gap() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(3.0, 0.0));
        let line2 = Line2D::new(Point2D::new(3.3, 0.0), Point2D::new(6.0, 0.0)); // 0.3m gap
        let config = CoplanarMergeConfig::default();

        assert!(lines_can_merge(&line1, &line2, &config));
    }

    #[test]
    fn test_lines_can_merge_rejects_large_gap() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(3.0, 0.0));
        let line2 = Line2D::new(Point2D::new(4.0, 0.0), Point2D::new(7.0, 0.0)); // 1.0m gap
        let config = CoplanarMergeConfig::default();

        assert!(!lines_can_merge(&line1, &line2, &config));
    }

    // ===== optimize_coplanar_lines tests =====

    #[test]
    fn test_optimize_coplanar_lines_merges_overlapping() {
        let mut lines = vec![
            Line2D::with_point_count(
                Point2D::new(0.0, 0.0),
                Point2D::new(3.0, 0.0),
                15, // Above threshold
            ),
            Line2D::with_point_count(Point2D::new(2.0, 0.0), Point2D::new(5.0, 0.0), 12),
        ];
        let config = CoplanarMergeConfig::default();

        let merged_count = optimize_coplanar_lines(&mut lines, &config);

        assert_eq!(merged_count, 1);
        assert_eq!(lines.len(), 1);
        assert_relative_eq!(lines[0].length(), 5.0, epsilon = 0.01);
        assert_eq!(lines[0].point_count, 27); // 15 + 12
    }

    #[test]
    fn test_optimize_coplanar_lines_merges_with_gap() {
        let mut lines = vec![
            Line2D::with_point_count(Point2D::new(0.0, 0.0), Point2D::new(2.0, 0.0), 12),
            Line2D::with_point_count(
                Point2D::new(2.2, 0.0), // 0.2m gap, within default 0.3m
                Point2D::new(5.0, 0.0),
                15,
            ),
        ];
        let config = CoplanarMergeConfig::default();

        let merged_count = optimize_coplanar_lines(&mut lines, &config);

        assert_eq!(merged_count, 1);
        assert_eq!(lines.len(), 1);
        assert_relative_eq!(lines[0].length(), 5.0, epsilon = 0.01);
    }

    #[test]
    fn test_optimize_coplanar_lines_uses_dominant() {
        // The line with most points should determine the direction
        let mut lines = vec![
            Line2D::with_point_count(
                Point2D::new(0.0, 0.0),
                Point2D::new(2.0, 0.0),
                25, // Dominant (most points)
            ),
            Line2D::with_point_count(
                Point2D::new(2.0, 0.0),
                Point2D::new(4.0, 0.01), // Slightly off
                10,
            ),
        ];
        let config = CoplanarMergeConfig::default();

        let merged_count = optimize_coplanar_lines(&mut lines, &config);

        assert_eq!(merged_count, 1);
        assert_eq!(lines.len(), 1);
        // The merged line should use the dominant line's direction
        // So it should be close to horizontal
        assert!(lines[0].angle().abs() < 0.02);
    }

    #[test]
    fn test_optimize_coplanar_lines_respects_max_gap() {
        let mut lines = vec![
            Line2D::with_point_count(Point2D::new(0.0, 0.0), Point2D::new(2.0, 0.0), 15),
            Line2D::with_point_count(
                Point2D::new(4.0, 0.0), // 2.0m gap, above default 0.3m
                Point2D::new(6.0, 0.0),
                15,
            ),
        ];
        let config = CoplanarMergeConfig::default(); // max_gap = 0.3m

        let merged_count = optimize_coplanar_lines(&mut lines, &config);

        assert_eq!(merged_count, 0);
        assert_eq!(lines.len(), 2);
    }

    #[test]
    fn test_optimize_coplanar_lines_keeps_non_dominant() {
        // When no line has enough points, don't merge
        let mut lines = vec![
            Line2D::with_point_count(
                Point2D::new(0.0, 0.0),
                Point2D::new(2.0, 0.0),
                5, // Below threshold of 10
            ),
            Line2D::with_point_count(
                Point2D::new(2.0, 0.0),
                Point2D::new(4.0, 0.0),
                5, // Below threshold of 10
            ),
        ];
        let config = CoplanarMergeConfig::default(); // min_dominant_point_count = 10

        let merged_count = optimize_coplanar_lines(&mut lines, &config);

        assert_eq!(merged_count, 0);
        assert_eq!(lines.len(), 2);
    }

    #[test]
    fn test_optimize_coplanar_lines_multiple_groups() {
        // Two separate walls (horizontal and vertical)
        let mut lines = vec![
            // Horizontal wall (will merge)
            Line2D::with_point_count(Point2D::new(0.0, 0.0), Point2D::new(2.0, 0.0), 15),
            Line2D::with_point_count(Point2D::new(2.0, 0.0), Point2D::new(4.0, 0.0), 12),
            // Vertical wall (will merge)
            Line2D::with_point_count(Point2D::new(5.0, 0.0), Point2D::new(5.0, 2.0), 15),
            Line2D::with_point_count(Point2D::new(5.0, 2.0), Point2D::new(5.0, 4.0), 12),
        ];
        let config = CoplanarMergeConfig::default();

        let merged_count = optimize_coplanar_lines(&mut lines, &config);

        assert_eq!(merged_count, 2); // Merged 2 lines (one from each group)
        assert_eq!(lines.len(), 2); // Two merged lines remain
    }

    #[test]
    fn test_optimize_coplanar_lines_empty() {
        let mut lines: Vec<Line2D> = vec![];
        let config = CoplanarMergeConfig::default();

        let merged_count = optimize_coplanar_lines(&mut lines, &config);

        assert_eq!(merged_count, 0);
        assert_eq!(lines.len(), 0);
    }

    #[test]
    fn test_optimize_coplanar_lines_single() {
        let mut lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let config = CoplanarMergeConfig::default();

        let merged_count = optimize_coplanar_lines(&mut lines, &config);

        assert_eq!(merged_count, 0);
        assert_eq!(lines.len(), 1);
    }

    #[test]
    fn test_merge_collinear_lines_with_config() {
        let line1 = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(3.0, 0.0));
        let line2 = Line2D::new(Point2D::new(2.0, 0.0), Point2D::new(5.0, 0.0));
        let config = CoplanarMergeConfig::default();

        let merged = merge_collinear_lines_with_config(&line1, &line2, &config);

        assert!(merged.is_some());
        let merged = merged.unwrap();
        assert_relative_eq!(merged.length(), 5.0, epsilon = 0.01);
    }
}
