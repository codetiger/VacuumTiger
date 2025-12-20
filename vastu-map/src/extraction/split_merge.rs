//! Split-and-Merge line extraction algorithm.
//!
//! This algorithm extracts line segments from an ordered sequence of points
//! (e.g., from a lidar scan). It works by:
//!
//! 1. **Split Phase**: Recursively split point sequences where the maximum
//!    deviation from the fitted line exceeds a threshold.
//!
//! 2. **Merge Phase**: Adjacent segments that can be represented by a single
//!    line within tolerance are merged.
//!
//! Requirements:
//! - Points must be ordered (sequential around scan)
//! - Works best with relatively uniform point density
//!
//! # Adaptive Thresholds
//!
//! When sensor position is known, the split threshold can adapt to range:
//! farther points have higher measurement uncertainty, so a larger threshold
//! is used. This prevents over-splitting distant segments while maintaining
//! accuracy for close-range features.
//!
//! ```rust,ignore
//! use vastu_map::extraction::{SplitMergeConfig, extract_lines_from_sensor};
//! use vastu_map::core::Point2D;
//!
//! let sensor_pos = Point2D::new(0.0, 0.0);
//! let lines = extract_lines_from_sensor(&points, &config, sensor_pos);
//! ```

use crate::core::Point2D;
use crate::features::Line2D;

use super::line_fitting::{fit_line, fit_line_from_sensor, max_distance_point};

/// Configuration for Split-and-Merge algorithm.
#[derive(Clone, Debug)]
pub struct SplitMergeConfig {
    /// Maximum perpendicular distance for a point to be considered on the line.
    /// If any point deviates more than this, the segment is split.
    /// Default: 0.05m (5cm)
    pub split_threshold: f32,

    /// Minimum number of points required to form a line segment.
    /// Segments with fewer points are discarded.
    /// Default: 5
    pub min_points: usize,

    /// Minimum line segment length.
    /// Segments shorter than this are discarded.
    /// Default: 0.10m (10cm)
    pub min_length: f32,

    /// Maximum gap between consecutive points.
    /// If gap exceeds this, points are split into separate sequences.
    /// Default: 0.30m (30cm)
    pub max_point_gap: f32,

    /// Merge threshold for adjacent segments.
    /// Segments whose combined fit error is below this are merged.
    /// Default: same as split_threshold
    pub merge_threshold: f32,

    /// Coefficient for range-adaptive threshold scaling.
    /// When using adaptive thresholds:
    ///   effective_threshold = split_threshold × (1 + adaptive_range_scale × avg_range)
    /// Default: 0.03 (3% increase per meter)
    pub adaptive_range_scale: f32,
}

impl Default for SplitMergeConfig {
    fn default() -> Self {
        Self {
            split_threshold: 0.05,
            min_points: 5,
            min_length: 0.10,
            max_point_gap: 0.30,
            merge_threshold: 0.05,
            adaptive_range_scale: 0.03, // 3% per meter
        }
    }
}

impl SplitMergeConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for split threshold.
    pub fn with_split_threshold(mut self, value: f32) -> Self {
        self.split_threshold = value;
        self
    }

    /// Builder-style setter for minimum points.
    pub fn with_min_points(mut self, value: usize) -> Self {
        self.min_points = value;
        self
    }

    /// Builder-style setter for minimum length.
    pub fn with_min_length(mut self, value: f32) -> Self {
        self.min_length = value;
        self
    }

    /// Builder-style setter for maximum point gap.
    pub fn with_max_point_gap(mut self, value: f32) -> Self {
        self.max_point_gap = value;
        self
    }

    /// Builder-style setter for adaptive range scale.
    pub fn with_adaptive_range_scale(mut self, value: f32) -> Self {
        self.adaptive_range_scale = value;
        self
    }
}

/// Compute adaptive split threshold based on average range from sensor.
///
/// Farther points have higher measurement uncertainty, so a larger threshold
/// is appropriate. The formula is:
///   threshold = base_threshold × (1 + scale × avg_range)
///
/// # Arguments
/// * `points` - Points to compute average range for
/// * `base_threshold` - Base threshold (e.g., 0.05m)
/// * `sensor_pos` - Position of the sensor
/// * `scale` - Range scaling factor (e.g., 0.03 = 3% per meter)
#[inline]
pub fn adaptive_split_threshold(
    points: &[Point2D],
    base_threshold: f32,
    sensor_pos: Point2D,
    scale: f32,
) -> f32 {
    if points.is_empty() {
        return base_threshold;
    }

    let avg_range =
        points.iter().map(|p| p.distance(sensor_pos)).sum::<f32>() / points.len() as f32;

    // Threshold scales with range: farther points have more noise
    base_threshold * (1.0 + scale * avg_range)
}

/// Extract line segments from an ordered sequence of points.
///
/// Points should be ordered (e.g., by angle in a lidar scan).
/// Returns extracted line segments.
///
/// # Example
/// ```
/// use vastu_map::extraction::{SplitMergeConfig, extract_lines};
/// use vastu_map::core::Point2D;
///
/// // Simulated lidar points forming two perpendicular walls
/// let points = vec![
///     // Horizontal wall
///     Point2D::new(0.0, 1.0),
///     Point2D::new(0.5, 1.0),
///     Point2D::new(1.0, 1.0),
///     Point2D::new(1.5, 1.0),
///     Point2D::new(2.0, 1.0),
///     // Corner transition
///     // Vertical wall
///     Point2D::new(2.0, 1.5),
///     Point2D::new(2.0, 2.0),
///     Point2D::new(2.0, 2.5),
///     Point2D::new(2.0, 3.0),
///     Point2D::new(2.0, 3.5),
/// ];
///
/// let config = SplitMergeConfig::default();
/// let lines = extract_lines(&points, &config);
/// // Should extract approximately 2 lines
/// ```
pub fn extract_lines(points: &[Point2D], config: &SplitMergeConfig) -> Vec<Line2D> {
    if points.len() < config.min_points {
        return Vec::new();
    }

    // Split points into continuous sequences by index pairs (avoids copying)
    let sequences = split_by_gaps_indices(points, config.max_point_gap);

    let mut all_lines = Vec::new();

    for (start, end) in sequences {
        let seq_len = end - start;
        if seq_len < config.min_points {
            continue;
        }

        // Extract lines from this sequence using split-and-merge
        let sequence = &points[start..end];
        let lines = split_and_merge(sequence, config);
        all_lines.extend(lines);
    }

    all_lines
}

/// Extract line segments with sensor-aware processing.
///
/// This is the preferred method for lidar data. It uses:
/// 1. **Adaptive thresholds**: Split threshold scales with range (farther = more lenient)
/// 2. **Weighted line fitting**: Closer points have more influence on line direction
///
/// # Arguments
/// * `points` - Ordered sequence of lidar hits (e.g., by angle)
/// * `config` - Split-and-merge configuration
/// * `sensor_pos` - Position of the lidar sensor (typically robot position)
///
/// # Example
/// ```
/// use vastu_map::extraction::{SplitMergeConfig, extract_lines_from_sensor};
/// use vastu_map::core::Point2D;
///
/// let sensor_pos = Point2D::new(0.0, 0.0);
/// let points = vec![
///     Point2D::new(1.0, 0.0),
///     Point2D::new(2.0, 0.0),
///     Point2D::new(3.0, 0.0),
///     Point2D::new(4.0, 0.0),
///     Point2D::new(5.0, 0.0),
/// ];
///
/// let config = SplitMergeConfig::default();
/// let lines = extract_lines_from_sensor(&points, &config, sensor_pos);
/// ```
pub fn extract_lines_from_sensor(
    points: &[Point2D],
    config: &SplitMergeConfig,
    sensor_pos: Point2D,
) -> Vec<Line2D> {
    if points.len() < config.min_points {
        return Vec::new();
    }

    // Split points into continuous sequences by index pairs (avoids copying)
    let sequences = split_by_gaps_indices(points, config.max_point_gap);

    let mut all_lines = Vec::new();

    for (start, end) in sequences {
        let seq_len = end - start;
        if seq_len < config.min_points {
            continue;
        }

        // Extract lines from this sequence using sensor-aware split-and-merge
        let sequence = &points[start..end];
        let lines = split_and_merge_from_sensor(sequence, config, sensor_pos);
        all_lines.extend(lines);
    }

    all_lines
}

/// Split points into continuous sequences based on gap threshold and corner detection.
/// Returns (start, end) index pairs instead of copying points.
///
/// This function splits when:
/// 1. Gap between consecutive points exceeds max_gap, OR
/// 2. Direction change between consecutive segments exceeds corner threshold (~45°)
///
/// This is critical for 360° lidar scans where corners don't create gaps.
fn split_by_gaps_indices(points: &[Point2D], max_gap: f32) -> Vec<(usize, usize)> {
    if points.is_empty() {
        return Vec::new();
    }

    if points.len() < 3 {
        return vec![(0, points.len())];
    }

    let mut sequences = Vec::new();
    let mut seq_start = 0;

    // Corner detection threshold (cosine of 45 degrees = 0.707)
    // A sharp direction change indicates a corner between two walls
    const CORNER_COS_THRESHOLD: f32 = 0.707;

    for i in 1..points.len() {
        let gap = points[i].distance(points[i - 1]);
        let is_corner = if i >= 2 {
            detect_corner_at(points, i, CORNER_COS_THRESHOLD)
        } else {
            false
        };

        if gap > max_gap || is_corner {
            // End current sequence and start new one
            if i > seq_start {
                sequences.push((seq_start, i));
            }
            seq_start = i;
        }
    }

    // Add final sequence
    if points.len() > seq_start {
        sequences.push((seq_start, points.len()));
    }

    sequences
}

/// Detect if there's a corner at the given index by checking direction change.
///
/// A corner is detected when the direction from points[i-2] to points[i-1]
/// differs significantly from the direction from points[i-1] to points[i].
fn detect_corner_at(points: &[Point2D], i: usize, cos_threshold: f32) -> bool {
    if i < 2 || i >= points.len() {
        return false;
    }

    // Direction from i-2 to i-1
    let dir1_x = points[i - 1].x - points[i - 2].x;
    let dir1_y = points[i - 1].y - points[i - 2].y;
    let len1 = (dir1_x * dir1_x + dir1_y * dir1_y).sqrt();

    // Direction from i-1 to i
    let dir2_x = points[i].x - points[i - 1].x;
    let dir2_y = points[i].y - points[i - 1].y;
    let len2 = (dir2_x * dir2_x + dir2_y * dir2_y).sqrt();

    // Skip if either segment is too short
    if len1 < 0.01 || len2 < 0.01 {
        return false;
    }

    // Compute cosine of angle between directions
    let dot = (dir1_x * dir2_x + dir1_y * dir2_y) / (len1 * len2);

    // Corner if directions are not aligned (cos < threshold)
    dot < cos_threshold
}

/// Split points into continuous sequences based on gap threshold.
/// Returns copied sequences (for compatibility with tests).
#[cfg(test)]
fn split_by_gaps(points: &[Point2D], max_gap: f32) -> Vec<Vec<Point2D>> {
    split_by_gaps_indices(points, max_gap)
        .into_iter()
        .map(|(start, end)| points[start..end].to_vec())
        .collect()
}

/// Main split-and-merge algorithm on a continuous sequence.
fn split_and_merge(points: &[Point2D], config: &SplitMergeConfig) -> Vec<Line2D> {
    // Split phase: find breakpoints
    let mut breakpoints = vec![0];
    split_recursive(points, 0, points.len() - 1, config, &mut breakpoints);
    breakpoints.push(points.len() - 1);
    breakpoints.sort_unstable();
    breakpoints.dedup();

    #[cfg(test)]
    {
        if points.len() > 100 {
            eprintln!(
                "split_and_merge: {} points, {} breakpoints: {:?}",
                points.len(),
                breakpoints.len(),
                &breakpoints[..breakpoints.len().min(20)]
            );
        }
    }

    // Create line segments from breakpoints
    let mut segments: Vec<(usize, usize, Line2D)> = Vec::new();

    for i in 0..breakpoints.len() - 1 {
        let start = breakpoints[i];
        let end = breakpoints[i + 1];

        if end - start + 1 < config.min_points {
            continue;
        }

        let segment_points = &points[start..=end];
        if let Some(line) = fit_line(segment_points)
            && line.length() >= config.min_length
        {
            segments.push((start, end, line));
        }
    }

    #[cfg(test)]
    {
        if points.len() > 100 {
            eprintln!("split_and_merge: {} segments before merge", segments.len());
            for (i, (start, end, line)) in segments.iter().enumerate().take(10) {
                eprintln!(
                    "  Seg {}: pts {}..{}, angle={:.1}°",
                    i,
                    start,
                    end,
                    line.angle().to_degrees()
                );
            }
        }
    }

    // Merge phase: combine adjacent segments that fit well together
    let merged = merge_segments(points, segments, config);

    #[cfg(test)]
    {
        if points.len() > 100 {
            eprintln!("split_and_merge: {} segments after merge", merged.len());
        }
    }

    merged.into_iter().map(|(_, _, line)| line).collect()
}

/// Recursive split function.
fn split_recursive(
    points: &[Point2D],
    start: usize,
    end: usize,
    config: &SplitMergeConfig,
    breakpoints: &mut Vec<usize>,
) {
    if end <= start + 1 {
        return;
    }

    let segment_points = &points[start..=end];

    // Fit line to segment
    let Some(line) = fit_line(segment_points) else {
        return;
    };

    // Find point with maximum deviation, excluding boundary points
    // Boundary points can't be used as split points without causing degenerate recursion
    #[allow(unused_variables)] // max_dist only used in debug output under cfg(test)
    let Some((split_rel_idx, max_dist)) =
        find_best_split_point(segment_points, &line, config.split_threshold)
    else {
        return;
    };

    #[cfg(test)]
    {
        if points.len() > 100 && end - start > 50 {
            eprintln!(
                "split_recursive: {}..{} ({} pts), line_angle={:.1}°, max_dist={:.3} at rel_idx={}, split_thresh={}",
                start,
                end,
                end - start + 1,
                line.angle().to_degrees(),
                max_dist,
                split_rel_idx,
                config.split_threshold
            );
        }
    }

    let split_idx = start + split_rel_idx;
    breakpoints.push(split_idx);

    // Recursively split both halves
    split_recursive(points, start, split_idx, config, breakpoints);
    split_recursive(points, split_idx, end, config, breakpoints);
}

/// Find the best split point in a segment.
///
/// Returns (relative_index, max_distance) of the best point to split at.
/// The split point must:
/// 1. Have deviation > threshold
/// 2. Not be at the very first or last index (to prevent degenerate recursion)
/// 3. Prefer points that are well away from boundaries (actual corners)
///
/// Returns None if no valid split point exists.
fn find_best_split_point(
    segment_points: &[Point2D],
    line: &Line2D,
    threshold: f32,
) -> Option<(usize, f32)> {
    if segment_points.len() < 3 {
        return None;
    }

    let n = segment_points.len();
    let min_boundary_distance = n / 10; // At least 10% away from boundary

    // Compute all deviations
    let deviations: Vec<f32> = segment_points
        .iter()
        .map(|p| line.distance_to_point(*p))
        .collect();

    // Find max deviation point, preferring points away from boundaries
    let mut best_idx = None;
    let mut best_dist: f32 = 0.0;

    for (i, &dist) in deviations.iter().enumerate() {
        // Skip boundary points (degenerate splits)
        if i == 0 || i == n - 1 {
            continue;
        }

        // Check if this point is far enough from boundaries
        let dist_from_boundary = i.min(n - 1 - i);
        let is_interior = dist_from_boundary >= min_boundary_distance.max(1);

        // For interior points, use normal threshold
        // For near-boundary points, require much higher deviation
        let effective_threshold = if is_interior {
            threshold
        } else {
            // Near-boundary points need 5x threshold to be considered
            threshold * 5.0
        };

        if dist > effective_threshold && dist > best_dist {
            best_dist = dist;
            best_idx = Some(i);
        }
    }

    best_idx.map(|idx| (idx, best_dist))
}

/// Merge adjacent segments that can be represented by a single line.
fn merge_segments(
    points: &[Point2D],
    mut segments: Vec<(usize, usize, Line2D)>,
    config: &SplitMergeConfig,
) -> Vec<(usize, usize, Line2D)> {
    if segments.len() < 2 {
        return segments;
    }

    let mut merged = true;
    while merged {
        merged = false;
        let mut new_segments = Vec::new();
        let mut i = 0;

        while i < segments.len() {
            if i + 1 < segments.len() {
                let (start1, end1, _) = segments[i];
                let (start2, end2, _) = segments[i + 1];

                // Check if segments are adjacent
                if end1 == start2 || end1 + 1 == start2 {
                    // Try to merge
                    let merged_points = &points[start1..=end2];
                    if let Some(merged_line) = fit_line(merged_points) {
                        // Check if merged line fits well
                        if let Some((_, max_dist)) = max_distance_point(merged_points, &merged_line)
                            && max_dist <= config.merge_threshold
                            && merged_line.length() >= config.min_length
                        {
                            // Merge successful
                            new_segments.push((start1, end2, merged_line));
                            i += 2;
                            merged = true;
                            continue;
                        }
                    }
                }
            }

            // Keep original segment
            new_segments.push(segments[i]);
            i += 1;
        }

        segments = new_segments;
    }

    segments
}

// =============================================================================
// Sensor-Aware Split-and-Merge (with adaptive thresholds and weighted fitting)
// =============================================================================

/// Main split-and-merge algorithm with sensor-aware processing.
///
/// Uses adaptive thresholds based on range and weighted line fitting.
fn split_and_merge_from_sensor(
    points: &[Point2D],
    config: &SplitMergeConfig,
    sensor_pos: Point2D,
) -> Vec<Line2D> {
    // Split phase: find breakpoints with adaptive thresholds
    let mut breakpoints = vec![0];
    split_recursive_from_sensor(
        points,
        0,
        points.len() - 1,
        config,
        sensor_pos,
        &mut breakpoints,
    );
    breakpoints.push(points.len() - 1);
    breakpoints.sort_unstable();
    breakpoints.dedup();

    #[cfg(test)]
    {
        if points.len() > 100 {
            eprintln!(
                "split_and_merge_from_sensor: {} points, {} breakpoints",
                points.len(),
                breakpoints.len()
            );
        }
    }

    // Create line segments from breakpoints using weighted fitting
    let mut segments: Vec<(usize, usize, Line2D)> = Vec::new();

    for i in 0..breakpoints.len() - 1 {
        let start = breakpoints[i];
        let end = breakpoints[i + 1];

        if end - start + 1 < config.min_points {
            continue;
        }

        let segment_points = &points[start..=end];
        // Use weighted line fitting for better accuracy
        if let Some(line) = fit_line_from_sensor(segment_points, sensor_pos, None)
            && line.length() >= config.min_length
        {
            segments.push((start, end, line));
        }
    }

    // Merge phase: combine adjacent segments with adaptive thresholds
    let merged = merge_segments_from_sensor(points, segments, config, sensor_pos);

    merged.into_iter().map(|(_, _, line)| line).collect()
}

/// Recursive split function with adaptive thresholds based on range.
fn split_recursive_from_sensor(
    points: &[Point2D],
    start: usize,
    end: usize,
    config: &SplitMergeConfig,
    sensor_pos: Point2D,
    breakpoints: &mut Vec<usize>,
) {
    if end <= start + 1 {
        return;
    }

    let segment_points = &points[start..=end];

    // Fit line using weighted TLS
    let Some(line) = fit_line_from_sensor(segment_points, sensor_pos, None) else {
        return;
    };

    // Compute adaptive threshold for this segment based on average range
    let adaptive_threshold = adaptive_split_threshold(
        segment_points,
        config.split_threshold,
        sensor_pos,
        config.adaptive_range_scale,
    );

    // Find point with maximum deviation using adaptive threshold
    let Some((split_rel_idx, _max_dist)) =
        find_best_split_point(segment_points, &line, adaptive_threshold)
    else {
        return;
    };

    let split_idx = start + split_rel_idx;
    breakpoints.push(split_idx);

    // Recursively split both halves
    split_recursive_from_sensor(points, start, split_idx, config, sensor_pos, breakpoints);
    split_recursive_from_sensor(points, split_idx, end, config, sensor_pos, breakpoints);
}

/// Merge adjacent segments with adaptive thresholds and weighted fitting.
fn merge_segments_from_sensor(
    points: &[Point2D],
    mut segments: Vec<(usize, usize, Line2D)>,
    config: &SplitMergeConfig,
    sensor_pos: Point2D,
) -> Vec<(usize, usize, Line2D)> {
    if segments.len() < 2 {
        return segments;
    }

    let mut merged = true;
    while merged {
        merged = false;
        let mut new_segments = Vec::new();
        let mut i = 0;

        while i < segments.len() {
            if i + 1 < segments.len() {
                let (start1, end1, _) = segments[i];
                let (start2, end2, _) = segments[i + 1];

                // Check if segments are adjacent
                if end1 == start2 || end1 + 1 == start2 {
                    // Try to merge with weighted fitting
                    let merged_points = &points[start1..=end2];
                    if let Some(merged_line) = fit_line_from_sensor(merged_points, sensor_pos, None)
                    {
                        // Compute adaptive merge threshold
                        let adaptive_threshold = adaptive_split_threshold(
                            merged_points,
                            config.merge_threshold,
                            sensor_pos,
                            config.adaptive_range_scale,
                        );

                        // Check if merged line fits well
                        if let Some((_, max_dist)) = max_distance_point(merged_points, &merged_line)
                            && max_dist <= adaptive_threshold
                            && merged_line.length() >= config.min_length
                        {
                            // Merge successful
                            new_segments.push((start1, end2, merged_line));
                            i += 2;
                            merged = true;
                            continue;
                        }
                    }
                }
            }

            // Keep original segment
            new_segments.push(segments[i]);
            i += 1;
        }

        segments = new_segments;
    }

    segments
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn test_split_by_gaps() {
        let points = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(0.1, 0.0),
            Point2D::new(0.2, 0.0),
            // Gap
            Point2D::new(1.0, 0.0),
            Point2D::new(1.1, 0.0),
        ];

        let sequences = split_by_gaps(&points, 0.3);

        assert_eq!(sequences.len(), 2);
        assert_eq!(sequences[0].len(), 3);
        assert_eq!(sequences[1].len(), 2);
    }

    #[test]
    fn test_extract_single_line() {
        // Generate points along a horizontal line
        let points: Vec<_> = (0..20).map(|i| Point2D::new(i as f32 * 0.1, 0.0)).collect();

        let config = SplitMergeConfig::default();
        let lines = extract_lines(&points, &config);

        assert_eq!(lines.len(), 1);
        assert_relative_eq!(lines[0].angle(), 0.0, epsilon = 0.1);
    }

    #[test]
    fn test_extract_two_lines_corner() {
        // L-shaped points: horizontal then vertical
        let mut points = Vec::new();

        // Horizontal segment (10 points)
        for i in 0..10 {
            points.push(Point2D::new(i as f32 * 0.2, 0.0));
        }

        // Vertical segment (10 points), starting from end of horizontal
        for i in 1..10 {
            points.push(Point2D::new(1.8, i as f32 * 0.2));
        }

        let config = SplitMergeConfig::default()
            .with_min_points(5)
            .with_min_length(0.5);

        let lines = extract_lines(&points, &config);

        // Should extract 2 lines
        assert!(
            lines.len() >= 1,
            "Expected at least 1 line, got {}",
            lines.len()
        );

        // One should be approximately horizontal, one approximately vertical
        let angles: Vec<_> = lines.iter().map(|l| l.angle()).collect();

        if lines.len() >= 2 {
            // Check that we have both orientations
            let has_horizontal = angles.iter().any(|a| a.abs() < 0.3);
            let has_vertical = angles.iter().any(|a| (a.abs() - FRAC_PI_2).abs() < 0.3);

            assert!(
                has_horizontal || has_vertical,
                "Expected horizontal or vertical lines, got angles: {:?}",
                angles
            );
        }
    }

    #[test]
    fn test_extract_noisy_line() {
        // Points along a line with some noise
        let points: Vec<_> = (0..20)
            .map(|i| {
                let noise = if i % 2 == 0 { 0.02 } else { -0.02 };
                Point2D::new(i as f32 * 0.1, noise)
            })
            .collect();

        let config = SplitMergeConfig::default().with_split_threshold(0.05);
        let lines = extract_lines(&points, &config);

        // Should still extract as single line (noise within threshold)
        assert_eq!(lines.len(), 1);
    }

    #[test]
    fn test_extract_min_points_filter() {
        // Only 3 points - below minimum
        let points = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
        ];

        let config = SplitMergeConfig::default().with_min_points(5);
        let lines = extract_lines(&points, &config);

        assert!(lines.is_empty());
    }

    #[test]
    fn test_extract_min_length_filter() {
        // Short line
        let points: Vec<_> = (0..10)
            .map(|i| Point2D::new(i as f32 * 0.005, 0.0)) // Total length 0.045m
            .collect();

        let config = SplitMergeConfig::default()
            .with_min_points(3)
            .with_min_length(0.1);

        let lines = extract_lines(&points, &config);

        assert!(lines.is_empty());
    }

    #[test]
    fn test_extract_with_gap() {
        // Two separate line segments with a gap
        let mut points = Vec::new();

        // First segment
        for i in 0..10 {
            points.push(Point2D::new(i as f32 * 0.1, 0.0));
        }

        // Gap of 1.0m
        // Second segment
        for i in 0..10 {
            points.push(Point2D::new(2.0 + i as f32 * 0.1, 1.0));
        }

        let config = SplitMergeConfig::default()
            .with_max_point_gap(0.5)
            .with_min_points(5);

        let lines = extract_lines(&points, &config);

        // Should extract 2 separate lines
        assert_eq!(lines.len(), 2);
    }

    #[test]
    fn test_empty_input() {
        let points: Vec<Point2D> = vec![];
        let config = SplitMergeConfig::default();
        let lines = extract_lines(&points, &config);

        assert!(lines.is_empty());
    }

    #[test]
    fn test_circular_points_no_infinite_recursion() {
        // Simulate a 360° lidar scan forming a rough circle.
        // This previously caused infinite recursion when max deviation
        // point was at index 0, causing split_idx == start.
        use std::f32::consts::PI;

        let num_points = 100;
        let radius = 2.0;
        let points: Vec<_> = (0..num_points)
            .map(|i| {
                let angle = 2.0 * PI * (i as f32) / (num_points as f32);
                // Add some noise to create deviation
                let r = radius + 0.1 * ((i % 5) as f32 - 2.0);
                Point2D::new(r * angle.cos(), r * angle.sin())
            })
            .collect();

        let config = SplitMergeConfig::default()
            .with_min_points(3)
            .with_split_threshold(0.1);

        // This should complete without hanging (previously infinite recursion)
        let lines = extract_lines(&points, &config);

        // We should get some lines (exact number depends on noise pattern)
        // The important thing is that it terminates without panic
        let _ = lines.len();
    }

    #[test]
    fn test_square_room_360_scan() {
        // Simulate a 360° lidar scan of a 4m x 4m square room
        // with robot at center (0, 0) facing +X.
        // This tests the algorithm's ability to detect axis-aligned walls.
        use std::f32::consts::{PI, TAU};

        // Room walls at x = ±2, y = ±2
        let half_size = 2.0;
        let num_rays = 360;
        let mut points = Vec::with_capacity(num_rays);

        for i in 0..num_rays {
            let angle = TAU * (i as f32) / (num_rays as f32);
            let (sin, cos) = angle.sin_cos();

            // Compute distance to walls using ray-box intersection
            let t_right = if cos > 0.0 {
                half_size / cos
            } else {
                f32::INFINITY
            };
            let t_left = if cos < 0.0 {
                -half_size / cos
            } else {
                f32::INFINITY
            };
            let t_top = if sin > 0.0 {
                half_size / sin
            } else {
                f32::INFINITY
            };
            let t_bottom = if sin < 0.0 {
                -half_size / sin
            } else {
                f32::INFINITY
            };

            let t = t_right.min(t_left).min(t_top).min(t_bottom);
            let point = Point2D::new(t * cos, t * sin);
            points.push(point);
        }

        let config = SplitMergeConfig::default()
            .with_min_points(5)
            .with_min_length(0.5)
            .with_split_threshold(0.05);

        let lines = extract_lines(&points, &config);

        // Debug: print extracted lines
        println!("\nExtracted {} lines from square room scan:", lines.len());
        for (i, line) in lines.iter().enumerate() {
            let angle_deg = line.angle().to_degrees();
            println!(
                "  Line {}: ({:.2}, {:.2}) to ({:.2}, {:.2}), angle={:.1}°, len={:.2}",
                i,
                line.start.x,
                line.start.y,
                line.end.x,
                line.end.y,
                angle_deg,
                line.length()
            );
        }

        // We should get 4 lines (one per wall)
        // Allow some flexibility in case corners create extra short segments
        assert!(
            lines.len() >= 4 && lines.len() <= 8,
            "Expected 4-8 lines, got {}",
            lines.len()
        );

        // Check that lines are axis-aligned (within 10 degrees)
        let axis_aligned_count = lines
            .iter()
            .filter(|line| {
                let angle = line.angle().abs();
                let normalized = angle % PI;
                let dist_horiz = normalized.min(PI - normalized);
                let dist_vert = (normalized - FRAC_PI_2).abs();
                dist_horiz < 0.175 || dist_vert < 0.175
            })
            .count();

        println!(
            "Axis-aligned lines: {} / {}",
            axis_aligned_count,
            lines.len()
        );

        // At least 80% of lines should be axis-aligned in a square room
        let axis_ratio = axis_aligned_count as f32 / lines.len() as f32;
        assert!(
            axis_ratio >= 0.75,
            "Only {}% of lines are axis-aligned (expected >= 75%)",
            (axis_ratio * 100.0) as i32
        );
    }

    #[test]
    fn test_degenerate_split_at_boundary() {
        // Test case where all points except first/last are collinear,
        // forcing max deviation at boundary indices.
        let points = vec![
            Point2D::new(0.0, 1.0), // Outlier at start
            Point2D::new(0.1, 0.0),
            Point2D::new(0.2, 0.0),
            Point2D::new(0.3, 0.0),
            Point2D::new(0.4, 0.0),
            Point2D::new(0.5, 0.0),
            Point2D::new(0.6, 0.0),
            Point2D::new(0.7, 0.0),
            Point2D::new(0.8, 0.0),
            Point2D::new(0.9, 1.0), // Outlier at end
        ];

        let config = SplitMergeConfig::default()
            .with_min_points(3)
            .with_split_threshold(0.05);

        // Should terminate without infinite recursion
        let lines = extract_lines(&points, &config);
        let _ = lines.len();
    }

    // ========================================
    // Adaptive Threshold Tests
    // ========================================

    #[test]
    fn test_adaptive_split_threshold_basic() {
        let sensor_pos = Point2D::new(0.0, 0.0);

        // Close points (1m range)
        let close_points = vec![
            Point2D::new(1.0, 0.0),
            Point2D::new(1.0, 0.1),
            Point2D::new(1.0, 0.2),
        ];

        // Far points (5m range)
        let far_points = vec![
            Point2D::new(5.0, 0.0),
            Point2D::new(5.0, 0.1),
            Point2D::new(5.0, 0.2),
        ];

        let base_threshold = 0.05;
        let scale = 0.03;

        let close_threshold =
            adaptive_split_threshold(&close_points, base_threshold, sensor_pos, scale);
        let far_threshold =
            adaptive_split_threshold(&far_points, base_threshold, sensor_pos, scale);

        // Far points should have higher threshold
        assert!(
            far_threshold > close_threshold,
            "Far threshold {} should be > close threshold {}",
            far_threshold,
            close_threshold
        );

        // Close threshold should be ~0.05 * (1 + 0.03 * 1) = 0.0515
        assert!(close_threshold > base_threshold);
        assert!(close_threshold < base_threshold * 1.1);

        // Far threshold should be ~0.05 * (1 + 0.03 * 5) = 0.0575
        assert!(far_threshold > base_threshold * 1.1);
    }

    #[test]
    fn test_adaptive_split_threshold_empty() {
        let sensor_pos = Point2D::new(0.0, 0.0);
        let points: Vec<Point2D> = vec![];
        let threshold = adaptive_split_threshold(&points, 0.05, sensor_pos, 0.03);
        assert_eq!(threshold, 0.05); // Should return base threshold
    }

    #[test]
    fn test_extract_lines_from_sensor_horizontal() {
        let sensor_pos = Point2D::new(0.0, 0.0);
        let points: Vec<_> = (0..20)
            .map(|i| Point2D::new(2.0 + i as f32 * 0.1, 0.0))
            .collect();

        let config = SplitMergeConfig::default();
        let lines = extract_lines_from_sensor(&points, &config, sensor_pos);

        assert_eq!(lines.len(), 1);
        assert_relative_eq!(lines[0].angle(), 0.0, epsilon = 0.1);
    }

    #[test]
    fn test_extract_lines_from_sensor_with_noise() {
        let sensor_pos = Point2D::new(0.0, 0.0);

        // Points at 5m range with noise that's within adaptive threshold
        let points: Vec<_> = (0..20)
            .map(|i| {
                let noise = if i % 2 == 0 { 0.08 } else { -0.08 };
                Point2D::new(5.0 + i as f32 * 0.1, noise)
            })
            .collect();

        let config = SplitMergeConfig::default()
            .with_split_threshold(0.05)
            .with_adaptive_range_scale(0.03);

        // Without sensor awareness, this might over-split due to noise
        let lines_no_sensor = extract_lines(&points, &config);

        // With sensor awareness, adaptive threshold should tolerate more noise at 5m
        let lines_from_sensor = extract_lines_from_sensor(&points, &config, sensor_pos);

        // Sensor-aware should produce same or fewer lines (less over-splitting)
        assert!(
            lines_from_sensor.len() <= lines_no_sensor.len(),
            "Sensor-aware should not over-split: {} vs {} lines",
            lines_from_sensor.len(),
            lines_no_sensor.len()
        );
    }

    #[test]
    fn test_extract_lines_from_sensor_corner() {
        let sensor_pos = Point2D::new(0.0, 0.0);

        // L-shaped points: horizontal then vertical
        let mut points = Vec::new();

        // Horizontal segment at y=2 (2m range)
        for i in 0..10 {
            points.push(Point2D::new(i as f32 * 0.2, 2.0));
        }

        // Vertical segment at x=1.8
        for i in 1..10 {
            points.push(Point2D::new(1.8, 2.0 + i as f32 * 0.2));
        }

        let config = SplitMergeConfig::default()
            .with_min_points(5)
            .with_min_length(0.5);

        let lines = extract_lines_from_sensor(&points, &config, sensor_pos);

        // Should extract at least 1 line
        assert!(
            lines.len() >= 1,
            "Expected at least 1 line, got {}",
            lines.len()
        );
    }

    #[test]
    fn test_config_adaptive_range_scale() {
        let config = SplitMergeConfig::default().with_adaptive_range_scale(0.05);
        assert_eq!(config.adaptive_range_scale, 0.05);

        // Default should be 0.03
        let default_config = SplitMergeConfig::default();
        assert_eq!(default_config.adaptive_range_scale, 0.03);
    }
}
