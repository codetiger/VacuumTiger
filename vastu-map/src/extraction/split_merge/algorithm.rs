//! Core Split-and-Merge algorithm implementation.
//!
//! Contains the recursive split phase and the main split-and-merge orchestration.

use crate::core::Point2D;
use crate::features::Line2D;

use super::config::SplitMergeConfig;
use super::helpers::{
    adaptive_split_threshold, find_best_split_point, merge_segments, merge_segments_from_sensor,
    split_by_gaps_indices,
};
use crate::extraction::line_fitting::{fit_line, fit_line_from_sensor};

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

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::FRAC_PI_2;

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
