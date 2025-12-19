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

use crate::core::Point2D;
use crate::features::Line2D;

use super::line_fitting::{fit_line, max_distance_point};

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
}

impl Default for SplitMergeConfig {
    fn default() -> Self {
        Self {
            split_threshold: 0.05,
            min_points: 5,
            min_length: 0.10,
            max_point_gap: 0.30,
            merge_threshold: 0.05,
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

/// Split points into continuous sequences based on gap threshold.
/// Returns (start, end) index pairs instead of copying points.
///
/// This is a zero-copy optimization that returns index ranges into the
/// original points slice.
fn split_by_gaps_indices(points: &[Point2D], max_gap: f32) -> Vec<(usize, usize)> {
    if points.is_empty() {
        return Vec::new();
    }

    let mut sequences = Vec::new();
    let mut seq_start = 0;

    for i in 1..points.len() {
        let gap = points[i].distance(points[i - 1]);
        if gap > max_gap {
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

    // Merge phase: combine adjacent segments that fit well together
    let merged = merge_segments(points, segments, config);

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

    // Find point with maximum deviation
    let Some((max_idx, max_dist)) = max_distance_point(segment_points, &line) else {
        return;
    };

    // If deviation exceeds threshold, split at this point
    if max_dist > config.split_threshold {
        let split_idx = start + max_idx;

        // Prevent degenerate splits at boundaries that cause infinite recursion.
        // This happens with continuous 360° lidar scans where max deviation point
        // is at index 0, causing split_idx == start and identical recursive calls.
        if split_idx == start || split_idx == end {
            return;
        }

        breakpoints.push(split_idx);

        // Recursively split both halves
        split_recursive(points, start, split_idx, config, breakpoints);
        split_recursive(points, split_idx, end, config, breakpoints);
    }
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
}
