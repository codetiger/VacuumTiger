//! Helper functions for Split-and-Merge algorithm.
//!
//! Contains gap detection, corner detection, segment merging, and adaptive threshold computation.

use std::simd::{f32x4, num::SimdFloat};

use crate::core::Point2D;
use crate::features::Line2D;

use super::config::SplitMergeConfig;
use crate::extraction::line_fitting::{fit_line, fit_line_from_sensor, max_distance_point};

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

/// Split points into continuous sequences based on gap threshold and corner detection.
/// Returns (start, end) index pairs instead of copying points.
///
/// This function splits when:
/// 1. Gap between consecutive points exceeds max_gap, OR
/// 2. Direction change between consecutive segments exceeds corner threshold (~45°)
///
/// This is critical for 360° lidar scans where corners don't create gaps.
pub(crate) fn split_by_gaps_indices(points: &[Point2D], max_gap: f32) -> Vec<(usize, usize)> {
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
pub(crate) fn split_by_gaps(points: &[Point2D], max_gap: f32) -> Vec<Vec<Point2D>> {
    split_by_gaps_indices(points, max_gap)
        .into_iter()
        .map(|(start, end)| points[start..end].to_vec())
        .collect()
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
/// SIMD-optimized for batches of 4+ points.
pub(crate) fn find_best_split_point(
    segment_points: &[Point2D],
    line: &Line2D,
    threshold: f32,
) -> Option<(usize, f32)> {
    if segment_points.len() < 3 {
        return None;
    }

    let n = segment_points.len();
    let min_boundary_distance = n / 10; // At least 10% away from boundary

    // Pre-compute line parameters for SIMD distance calculation
    let dx = line.end.x - line.start.x;
    let dy = line.end.y - line.start.y;
    let length_sq = dx * dx + dy * dy;

    if length_sq < f32::EPSILON {
        // Degenerate line - use scalar fallback
        return find_best_split_point_scalar(segment_points, line, threshold);
    }

    let inv_length = 1.0 / length_sq.sqrt();

    // Compute all deviations using SIMD
    let chunks = n / 4;
    let mut deviations = Vec::with_capacity(n);

    if chunks > 0 {
        let dx4 = f32x4::splat(dx);
        let dy4 = f32x4::splat(dy);
        let sx4 = f32x4::splat(line.start.x);
        let sy4 = f32x4::splat(line.start.y);
        let inv_len4 = f32x4::splat(inv_length);

        for i in 0..chunks {
            let base = i * 4;

            // Load 4 points
            let px = f32x4::from_array([
                segment_points[base].x,
                segment_points[base + 1].x,
                segment_points[base + 2].x,
                segment_points[base + 3].x,
            ]);
            let py = f32x4::from_array([
                segment_points[base].y,
                segment_points[base + 1].y,
                segment_points[base + 2].y,
                segment_points[base + 3].y,
            ]);

            // Compute cross product and distance
            let to_px = px - sx4;
            let to_py = py - sy4;
            let cross = dx4 * to_py - dy4 * to_px;
            let dist = cross.abs() * inv_len4;

            deviations.extend_from_slice(&dist.to_array());
        }
    }

    // Handle remainder
    for p in segment_points.iter().take(n).skip(chunks * 4) {
        let to_px = p.x - line.start.x;
        let to_py = p.y - line.start.y;
        let cross = dx * to_py - dy * to_px;
        let dist = cross.abs() * inv_length;
        deviations.push(dist);
    }

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

/// Scalar fallback for degenerate lines.
fn find_best_split_point_scalar(
    segment_points: &[Point2D],
    line: &Line2D,
    threshold: f32,
) -> Option<(usize, f32)> {
    let n = segment_points.len();
    let min_boundary_distance = n / 10;

    let deviations: Vec<f32> = segment_points
        .iter()
        .map(|p| line.distance_to_point(*p))
        .collect();

    let mut best_idx = None;
    let mut best_dist: f32 = 0.0;

    for (i, &dist) in deviations.iter().enumerate() {
        if i == 0 || i == n - 1 {
            continue;
        }

        let dist_from_boundary = i.min(n - 1 - i);
        let is_interior = dist_from_boundary >= min_boundary_distance.max(1);

        let effective_threshold = if is_interior {
            threshold
        } else {
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
pub(crate) fn merge_segments(
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

/// Merge adjacent segments with adaptive thresholds and weighted fitting.
pub(crate) fn merge_segments_from_sensor(
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
}
