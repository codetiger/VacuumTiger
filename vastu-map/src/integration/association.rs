//! Line association for map integration.
//!
//! Associates new scan lines with existing map lines based on:
//! - Spatial proximity
//! - Angular similarity
//! - Overlap extent

use crate::core::math::angle_diff;
use crate::features::Line2D;

use super::spatial_index::SpatialIndex;

/// Configuration for line association.
#[derive(Clone, Debug)]
pub struct AssociationConfig {
    /// Maximum perpendicular distance for association (meters).
    /// Default: 0.15m
    pub max_perpendicular_distance: f32,

    /// Maximum angular difference for association (radians).
    /// Default: 0.2 (~11°)
    pub max_angle_difference: f32,

    /// Minimum overlap ratio for valid association.
    /// Overlap is measured as a fraction of the shorter line's length.
    /// Default: 0.3 (30%)
    pub min_overlap_ratio: f32,

    /// Maximum gap between lines for association (meters).
    /// Lines that don't overlap but are close enough may still associate.
    /// Default: 0.3m
    pub max_endpoint_gap: f32,
}

impl Default for AssociationConfig {
    fn default() -> Self {
        Self {
            max_perpendicular_distance: 0.15,
            max_angle_difference: 0.2,
            min_overlap_ratio: 0.3,
            max_endpoint_gap: 0.3,
        }
    }
}

impl AssociationConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for maximum perpendicular distance.
    pub fn with_max_perpendicular_distance(mut self, meters: f32) -> Self {
        self.max_perpendicular_distance = meters;
        self
    }

    /// Builder-style setter for maximum angle difference.
    pub fn with_max_angle_difference(mut self, radians: f32) -> Self {
        self.max_angle_difference = radians;
        self
    }

    /// Builder-style setter for minimum overlap ratio.
    pub fn with_min_overlap_ratio(mut self, ratio: f32) -> Self {
        self.min_overlap_ratio = ratio;
        self
    }
}

/// Result of line association.
#[derive(Clone, Debug)]
pub struct Association {
    /// Index of the scan line.
    pub scan_line_idx: usize,
    /// Index of the associated map line.
    pub map_line_idx: usize,
    /// Perpendicular distance between lines.
    pub distance: f32,
    /// Angular difference between lines (radians).
    pub angle_diff: f32,
    /// Overlap length (meters).
    pub overlap_length: f32,
    /// Overlap ratio (0 to 1).
    pub overlap_ratio: f32,
}

impl Association {
    /// Create a new association.
    pub fn new(
        scan_line_idx: usize,
        map_line_idx: usize,
        distance: f32,
        angle_diff: f32,
        overlap_length: f32,
        overlap_ratio: f32,
    ) -> Self {
        Self {
            scan_line_idx,
            map_line_idx,
            distance,
            angle_diff,
            overlap_length,
            overlap_ratio,
        }
    }

    /// Compute a quality score for the association (higher is better).
    pub fn quality_score(&self) -> f32 {
        // Weighted combination of factors
        let dist_score = 1.0 / (1.0 + self.distance * 10.0);
        let angle_score = 1.0 / (1.0 + self.angle_diff * 5.0);
        let overlap_score = self.overlap_ratio;

        dist_score * angle_score * overlap_score
    }
}

/// Find associations between scan lines and map lines.
///
/// # Arguments
/// * `scan_lines` - Lines extracted from the current scan
/// * `map_lines` - Existing map lines
/// * `index` - Spatial index of map lines (optional, for efficiency)
/// * `config` - Association configuration
///
/// # Returns
/// Vector of associations, one per scan line that found a match.
/// Scan lines without matches are not included.
pub fn find_associations(
    scan_lines: &[Line2D],
    map_lines: &[Line2D],
    index: Option<&SpatialIndex>,
    config: &AssociationConfig,
) -> Vec<Association> {
    if scan_lines.is_empty() || map_lines.is_empty() {
        return Vec::new();
    }

    let mut associations = Vec::with_capacity(scan_lines.len());

    for (scan_idx, scan_line) in scan_lines.iter().enumerate() {
        // Find candidate map lines using spatial index if available
        let candidates: Vec<usize> = if let Some(idx) = index {
            idx.potential_matches(scan_line, config.max_perpendicular_distance * 2.0)
        } else {
            (0..map_lines.len()).collect()
        };

        let mut best_association: Option<Association> = None;
        let mut best_score = 0.0;

        for &map_idx in &candidates {
            let map_line = &map_lines[map_idx];

            if let Some(assoc) = try_associate(scan_idx, scan_line, map_idx, map_line, config) {
                let score = assoc.quality_score();
                if score > best_score {
                    best_score = score;
                    best_association = Some(assoc);
                }
            }
        }

        if let Some(assoc) = best_association {
            associations.push(assoc);
        }
    }

    associations
}

/// Try to associate two lines.
///
/// Returns Some(Association) if lines are compatible, None otherwise.
fn try_associate(
    scan_idx: usize,
    scan_line: &Line2D,
    map_idx: usize,
    map_line: &Line2D,
    config: &AssociationConfig,
) -> Option<Association> {
    // Check angular compatibility
    let scan_angle = scan_line.angle();
    let map_angle = map_line.angle();
    let angle_difference = angle_diff(scan_angle, map_angle).abs();

    // Lines can be oriented either way, so check both directions
    let angle_difference = angle_difference.min(std::f32::consts::PI - angle_difference);

    if angle_difference > config.max_angle_difference {
        return None;
    }

    // Compute perpendicular distance (average of endpoint distances)
    let dist1 = map_line.distance_to_point(scan_line.start);
    let dist2 = map_line.distance_to_point(scan_line.end);
    let avg_distance = (dist1 + dist2) / 2.0;

    if avg_distance > config.max_perpendicular_distance {
        return None;
    }

    // Compute overlap
    let overlap = compute_overlap(scan_line, map_line);
    let scan_len = scan_line.length();
    let map_len = map_line.length();
    let shorter_len = scan_len.min(map_len);

    // Handle case where lines don't overlap but are close
    if overlap <= 0.0 {
        let gap = compute_endpoint_gap(scan_line, map_line);
        if gap > config.max_endpoint_gap {
            return None;
        }
        // Allow association with zero overlap if endpoints are close
        return Some(Association::new(
            scan_idx,
            map_idx,
            avg_distance,
            angle_difference,
            0.0,
            0.0,
        ));
    }

    let overlap_ratio = if shorter_len > 0.0 {
        overlap / shorter_len
    } else {
        0.0
    };

    if overlap_ratio < config.min_overlap_ratio {
        return None;
    }

    Some(Association::new(
        scan_idx,
        map_idx,
        avg_distance,
        angle_difference,
        overlap,
        overlap_ratio,
    ))
}

/// Compute overlap length between two lines.
///
/// Projects both lines onto a common axis and computes intersection.
fn compute_overlap(line1: &Line2D, line2: &Line2D) -> f32 {
    // Use Line2D's overlap_length method with default angle tolerance
    line1.overlap_length(line2, 0.3)
}

/// Compute minimum gap between line endpoints.
fn compute_endpoint_gap(line1: &Line2D, line2: &Line2D) -> f32 {
    let d1 = line1.start.distance(line2.start);
    let d2 = line1.start.distance(line2.end);
    let d3 = line1.end.distance(line2.start);
    let d4 = line1.end.distance(line2.end);

    d1.min(d2).min(d3).min(d4)
}

/// Associate scan lines to map lines, ensuring each map line
/// has at most one association (best match wins).
///
/// This is useful when updating the map to avoid duplicate associations.
pub fn find_unique_associations(
    scan_lines: &[Line2D],
    map_lines: &[Line2D],
    index: Option<&SpatialIndex>,
    config: &AssociationConfig,
) -> Vec<Association> {
    let all_associations = find_associations(scan_lines, map_lines, index, config);

    // Group by map line and keep best match
    let mut best_per_map: std::collections::HashMap<usize, Association> =
        std::collections::HashMap::new();

    for assoc in all_associations {
        let entry = best_per_map.entry(assoc.map_line_idx);
        entry
            .and_modify(|existing| {
                if assoc.quality_score() > existing.quality_score() {
                    *existing = assoc.clone();
                }
            })
            .or_insert(assoc);
    }

    best_per_map.into_values().collect()
}

/// Find scan lines that don't match any map lines.
///
/// These are candidates for new features to add to the map.
pub fn find_unmatched_scan_lines(
    scan_lines: &[Line2D],
    map_lines: &[Line2D],
    index: Option<&SpatialIndex>,
    config: &AssociationConfig,
) -> Vec<usize> {
    let associations = find_associations(scan_lines, map_lines, index, config);
    let matched: std::collections::HashSet<usize> =
        associations.iter().map(|a| a.scan_line_idx).collect();

    (0..scan_lines.len())
        .filter(|i| !matched.contains(i))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point2D;
    use approx::assert_relative_eq;

    #[test]
    fn test_association_quality_score() {
        let good = Association::new(0, 0, 0.01, 0.01, 1.0, 0.9);
        let bad = Association::new(0, 0, 0.5, 0.5, 0.1, 0.1);

        assert!(good.quality_score() > bad.quality_score());
    }

    #[test]
    fn test_find_associations_identical_lines() {
        let scan_lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
        let map_lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];

        let config = AssociationConfig::default();
        let assocs = find_associations(&scan_lines, &map_lines, None, &config);

        assert_eq!(assocs.len(), 1);
        assert_eq!(assocs[0].scan_line_idx, 0);
        assert_eq!(assocs[0].map_line_idx, 0);
        assert!(assocs[0].distance < 0.01);
        assert!(assocs[0].overlap_ratio > 0.99);
    }

    #[test]
    fn test_find_associations_parallel_lines() {
        // Parallel lines with small offset
        let scan_lines = vec![Line2D::new(Point2D::new(0.0, 0.1), Point2D::new(5.0, 0.1))];
        let map_lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];

        let config = AssociationConfig::default();
        let assocs = find_associations(&scan_lines, &map_lines, None, &config);

        assert_eq!(assocs.len(), 1);
        assert_relative_eq!(assocs[0].distance, 0.1, epsilon = 0.01);
    }

    #[test]
    fn test_find_associations_no_match_angle() {
        // Perpendicular lines
        let scan_lines = vec![Line2D::new(Point2D::new(2.5, -1.0), Point2D::new(2.5, 1.0))];
        let map_lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];

        let config = AssociationConfig::default();
        let assocs = find_associations(&scan_lines, &map_lines, None, &config);

        // Should not match due to angle difference
        assert!(assocs.is_empty());
    }

    #[test]
    fn test_find_associations_no_match_distance() {
        // Parallel but far apart
        let scan_lines = vec![Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(5.0, 1.0))];
        let map_lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];

        let config = AssociationConfig::default().with_max_perpendicular_distance(0.5);
        let assocs = find_associations(&scan_lines, &map_lines, None, &config);

        // Should not match due to distance
        assert!(assocs.is_empty());
    }

    #[test]
    fn test_find_associations_partial_overlap() {
        // Overlapping lines
        let scan_lines = vec![Line2D::new(Point2D::new(2.0, 0.0), Point2D::new(7.0, 0.0))];
        let map_lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];

        let config = AssociationConfig::default();
        let assocs = find_associations(&scan_lines, &map_lines, None, &config);

        assert_eq!(assocs.len(), 1);
        assert!(assocs[0].overlap_length > 2.0); // Should overlap by 3 meters
    }

    #[test]
    fn test_find_associations_multiple() {
        let scan_lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
        ];
        let map_lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
        ];

        let config = AssociationConfig::default();
        let assocs = find_associations(&scan_lines, &map_lines, None, &config);

        assert_eq!(assocs.len(), 2);
    }

    #[test]
    fn test_find_unique_associations() {
        // Two scan lines that could match the same map line
        let scan_lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(2.5, 0.0)),
            Line2D::new(Point2D::new(2.5, 0.0), Point2D::new(5.0, 0.0)),
        ];
        let map_lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];

        let config = AssociationConfig::default();
        let assocs = find_unique_associations(&scan_lines, &map_lines, None, &config);

        // Should only have one association (best match)
        assert_eq!(assocs.len(), 1);
    }

    #[test]
    fn test_find_unmatched() {
        let scan_lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 10.0), Point2D::new(5.0, 10.0)), // No match
        ];
        let map_lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];

        let config = AssociationConfig::default();
        let unmatched = find_unmatched_scan_lines(&scan_lines, &map_lines, None, &config);

        assert_eq!(unmatched.len(), 1);
        assert_eq!(unmatched[0], 1); // Second scan line is unmatched
    }

    #[test]
    fn test_with_spatial_index() {
        let scan_lines = vec![Line2D::new(Point2D::new(0.0, 0.1), Point2D::new(5.0, 0.1))];
        let map_lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 10.0), Point2D::new(5.0, 10.0)),
        ];

        let index = SpatialIndex::new(&map_lines);
        let config = AssociationConfig::default();

        let assocs = find_associations(&scan_lines, &map_lines, Some(&index), &config);

        assert_eq!(assocs.len(), 1);
        assert_eq!(assocs[0].map_line_idx, 0);
    }
}
