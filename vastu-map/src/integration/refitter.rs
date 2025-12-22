//! Line re-fitting from accumulated point clouds.
//!
//! Re-fits existing map lines using points accumulated in ScanStore,
//! eliminating first-scan bias in line geometry.

use std::collections::HashMap;

use crate::config::LidarNoiseModel;
use crate::core::{Point2D, Pose2D};
use crate::extraction::fit_line_weighted;
use crate::features::Line2D;
use crate::integration::point_association::AssociatedPoint;

/// Configuration for line re-fitting.
#[derive(Clone, Debug)]
pub struct RefitConfig {
    /// Minimum points required to re-fit a line.
    /// Lines with fewer points keep their original geometry.
    /// Default: 10
    pub min_points: usize,

    /// Gap threshold for splitting lines (meters).
    /// If consecutive points (sorted by projection) have a gap larger than this,
    /// the line is split into multiple segments.
    /// Default: 0.30m (30cm)
    pub gap_threshold: f32,

    /// Noise model for range-based weighting.
    /// Closer points get higher weights during fitting.
    pub noise_model: LidarNoiseModel,
}

impl Default for RefitConfig {
    fn default() -> Self {
        Self {
            min_points: 10,
            gap_threshold: 0.30,
            noise_model: LidarNoiseModel::default(),
        }
    }
}

impl RefitConfig {
    /// Create a new configuration with defaults.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set minimum points for re-fitting.
    pub fn with_min_points(mut self, min: usize) -> Self {
        self.min_points = min;
        self
    }

    /// Set gap threshold for line splitting.
    pub fn with_gap_threshold(mut self, gap: f32) -> Self {
        self.gap_threshold = gap;
        self
    }

    /// Set the noise model for weighting.
    pub fn with_noise_model(mut self, model: LidarNoiseModel) -> Self {
        self.noise_model = model;
        self
    }
}

/// Result of re-fitting a line.
#[derive(Clone, Debug)]
pub enum RefitResult {
    /// Single line fitted (no gaps detected).
    Single(Line2D),
    /// Line was split due to gaps in point cloud.
    Split(Vec<Line2D>),
    /// Not enough points to re-fit (keeps original geometry).
    Insufficient,
}

impl RefitResult {
    /// Get all lines from the result.
    pub fn lines(&self) -> Vec<Line2D> {
        match self {
            RefitResult::Single(line) => vec![*line],
            RefitResult::Split(lines) => lines.clone(),
            RefitResult::Insufficient => vec![],
        }
    }

    /// Get number of lines produced.
    pub fn line_count(&self) -> usize {
        match self {
            RefitResult::Single(_) => 1,
            RefitResult::Split(lines) => lines.len(),
            RefitResult::Insufficient => 0,
        }
    }

    /// Check if re-fitting was successful (not insufficient).
    pub fn is_success(&self) -> bool {
        !matches!(self, RefitResult::Insufficient)
    }
}

/// Statistics from a batch re-fitting operation.
#[derive(Clone, Debug, Default)]
pub struct RefitStats {
    /// Number of lines re-fitted as single segments.
    pub lines_refitted_single: usize,
    /// Number of lines that were split due to gaps.
    pub lines_split: usize,
    /// Number of lines with insufficient points (kept original).
    pub lines_insufficient: usize,
    /// Total number of new line segments created (from splits).
    pub total_segments_created: usize,
}

impl RefitStats {
    /// Total lines processed.
    pub fn total_processed(&self) -> usize {
        self.lines_refitted_single + self.lines_split + self.lines_insufficient
    }

    /// Success rate (refitted / total).
    pub fn success_rate(&self) -> f32 {
        let total = self.total_processed();
        if total == 0 {
            0.0
        } else {
            (self.lines_refitted_single + self.lines_split) as f32 / total as f32
        }
    }
}

/// Re-fit a line from associated points with gap detection.
///
/// # Arguments
/// * `points` - Points associated with this line (from point_association)
/// * `original` - Original line geometry (for fallback and observation_count)
/// * `robot_poses` - Map of scan_id -> robot pose (for computing weights)
/// * `config` - Re-fitting configuration
///
/// # Returns
/// - `RefitResult::Single(line)` if fitted as a single line
/// - `RefitResult::Split(lines)` if gaps were detected and line was split
/// - `RefitResult::Insufficient` if not enough points
pub fn refit_line(
    points: &[AssociatedPoint],
    original: &Line2D,
    robot_poses: &HashMap<u32, Pose2D>,
    config: &RefitConfig,
) -> RefitResult {
    // Check minimum points
    if points.len() < config.min_points {
        return RefitResult::Insufficient;
    }

    // Sort points by projection onto original line direction
    let mut sorted_points: Vec<&AssociatedPoint> = points.iter().collect();
    sorted_points.sort_by(|a, b| {
        a.projection_t
            .partial_cmp(&b.projection_t)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Detect gaps and find split indices
    let split_indices = find_gap_splits(&sorted_points, config.gap_threshold, original);

    if split_indices.is_empty() {
        // No gaps - fit as single line
        let line = fit_segment(
            &sorted_points,
            robot_poses,
            original.observation_count,
            &config.noise_model,
        );
        match line {
            Some(l) => RefitResult::Single(l),
            None => RefitResult::Insufficient,
        }
    } else {
        // Gaps detected - split into segments
        let segments = split_and_fit(
            &sorted_points,
            &split_indices,
            robot_poses,
            original.observation_count,
            config,
        );
        if segments.is_empty() {
            RefitResult::Insufficient
        } else if segments.len() == 1 {
            RefitResult::Single(segments.into_iter().next().unwrap())
        } else {
            RefitResult::Split(segments)
        }
    }
}

/// Find indices where the point cloud should be split due to gaps.
///
/// Returns a list of indices where splits should occur.
/// Each index represents the start of a new segment.
fn find_gap_splits(
    sorted_points: &[&AssociatedPoint],
    gap_threshold: f32,
    original: &Line2D,
) -> Vec<usize> {
    if sorted_points.len() < 2 {
        return vec![];
    }

    let line_length = original.length();
    if line_length < f32::EPSILON {
        return vec![];
    }

    let mut splits = Vec::new();

    for i in 1..sorted_points.len() {
        let prev = sorted_points[i - 1];
        let curr = sorted_points[i];

        // Gap in projection space (as fraction of line length)
        let t_gap = (curr.projection_t - prev.projection_t).abs();

        // Convert to world distance
        let world_gap = t_gap * line_length;

        if world_gap > gap_threshold {
            splits.push(i);
        }
    }

    splits
}

/// Split points at gap indices and fit each segment.
fn split_and_fit(
    sorted_points: &[&AssociatedPoint],
    split_indices: &[usize],
    robot_poses: &HashMap<u32, Pose2D>,
    observation_count: u32,
    config: &RefitConfig,
) -> Vec<Line2D> {
    let mut segments = Vec::new();

    // Create segment ranges
    let mut ranges: Vec<(usize, usize)> = Vec::new();
    let mut start = 0;

    for &split_idx in split_indices {
        if split_idx > start {
            ranges.push((start, split_idx));
        }
        start = split_idx;
    }

    // Add final segment
    if start < sorted_points.len() {
        ranges.push((start, sorted_points.len()));
    }

    // Fit each segment
    for (range_start, range_end) in ranges {
        let segment_points: Vec<&AssociatedPoint> = sorted_points[range_start..range_end].to_vec();

        if segment_points.len() < config.min_points {
            continue;
        }

        if let Some(line) = fit_segment(
            &segment_points,
            robot_poses,
            observation_count,
            &config.noise_model,
        ) {
            segments.push(line);
        }
    }

    segments
}

/// Fit a line segment from points using weighted TLS.
fn fit_segment(
    points: &[&AssociatedPoint],
    robot_poses: &HashMap<u32, Pose2D>,
    observation_count: u32,
    noise_model: &LidarNoiseModel,
) -> Option<Line2D> {
    if points.len() < 2 {
        return None;
    }

    // Extract Point2D and compute weights
    let mut point_vec: Vec<Point2D> = Vec::with_capacity(points.len());
    let mut weights: Vec<f32> = Vec::with_capacity(points.len());

    for ap in points {
        point_vec.push(ap.point);

        // Compute weight based on range from robot
        let weight = if let Some(pose) = robot_poses.get(&ap.scan_id) {
            let robot_pos = Point2D::new(pose.x, pose.y);
            let range = ap.point.distance(robot_pos);
            noise_model.weight(range)
        } else {
            // Fallback: use default weight
            1.0
        };
        weights.push(weight);
    }

    // Fit using weighted TLS
    let mut line = fit_line_weighted(&point_vec, &weights)?;

    // Preserve observation count from original
    line.observation_count = observation_count;
    line.point_count = points.len() as u32;

    Some(line)
}

/// Re-fit a line from points using simple (unweighted) fitting.
///
/// Use this when robot poses are not available.
pub fn refit_line_simple(
    points: &[AssociatedPoint],
    original: &Line2D,
    config: &RefitConfig,
) -> RefitResult {
    // Use empty pose map for simple fitting
    let empty_poses: HashMap<u32, Pose2D> = HashMap::new();
    refit_line(points, original, &empty_poses, config)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_horizontal_line(y: f32, x_start: f32, x_end: f32) -> Line2D {
        Line2D::with_observation_count(Point2D::new(x_start, y), Point2D::new(x_end, y), 5)
    }

    fn make_associated_point(x: f32, y: f32, scan_id: u32, t: f32) -> AssociatedPoint {
        AssociatedPoint {
            point: Point2D::new(x, y),
            scan_id,
            distance: 0.0,
            projection_t: t,
        }
    }

    fn make_points_along_line(
        x_start: f32,
        x_end: f32,
        y: f32,
        count: usize,
    ) -> Vec<AssociatedPoint> {
        let line_length = (x_end - x_start).abs();
        (0..count)
            .map(|i| {
                let t = i as f32 / (count - 1).max(1) as f32;
                let x = x_start + t * (x_end - x_start);
                // projection_t is the fraction along the line
                make_associated_point(x, y, 0, x / line_length)
            })
            .collect()
    }

    /// Create a dense point cloud (no gaps > threshold).
    fn make_dense_points(
        x_start: f32,
        x_end: f32,
        y: f32,
        gap_threshold: f32,
    ) -> Vec<AssociatedPoint> {
        let line_length = (x_end - x_start).abs();
        // Create points with spacing less than gap_threshold
        let spacing = gap_threshold * 0.5;
        let count = ((line_length / spacing) as usize).max(2);
        (0..count)
            .map(|i| {
                let x = x_start + i as f32 * spacing;
                let t = x / line_length;
                make_associated_point(x.min(x_end), y, 0, t.min(1.0))
            })
            .collect()
    }

    #[test]
    fn test_config_defaults() {
        let config = RefitConfig::default();
        assert_eq!(config.min_points, 10);
        assert!((config.gap_threshold - 0.30).abs() < 0.001);
    }

    #[test]
    fn test_config_builder() {
        let config = RefitConfig::new()
            .with_min_points(5)
            .with_gap_threshold(0.5);
        assert_eq!(config.min_points, 5);
        assert!((config.gap_threshold - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_refit_insufficient_points() {
        let original = make_horizontal_line(0.0, 0.0, 10.0);
        let points = make_points_along_line(0.0, 10.0, 0.0, 5); // Only 5 points
        let config = RefitConfig::default().with_min_points(10);
        let poses = HashMap::new();

        let result = refit_line(&points, &original, &poses, &config);

        assert!(matches!(result, RefitResult::Insufficient));
        assert!(!result.is_success());
        assert_eq!(result.line_count(), 0);
    }

    #[test]
    fn test_refit_single_line() {
        let original = make_horizontal_line(0.0, 0.0, 10.0);
        // Create dense points along the line with small noise (spacing < gap_threshold)
        let gap_threshold = 0.30;
        let spacing = gap_threshold * 0.5; // 0.15m spacing
        let count = (10.0 / spacing) as usize; // ~67 points for 10m line

        let mut points: Vec<AssociatedPoint> = Vec::new();
        for i in 0..count {
            let x = i as f32 * spacing;
            let t = x / 10.0;
            let y = if i % 2 == 0 { 0.01 } else { -0.01 }; // Small noise
            points.push(make_associated_point(x, y, 0, t));
        }

        let config = RefitConfig::default()
            .with_min_points(10)
            .with_gap_threshold(gap_threshold);
        let poses = HashMap::new();

        let result = refit_line(&points, &original, &poses, &config);

        assert!(result.is_success());
        assert!(matches!(result, RefitResult::Single(_)));

        let lines = result.lines();
        assert_eq!(lines.len(), 1);

        // Verify line is approximately horizontal
        let line = &lines[0];
        assert!(line.angle().abs() < 0.1);
        // Verify point count is set
        assert_eq!(line.point_count as usize, count);
    }

    #[test]
    fn test_refit_with_gap_split() {
        let original = make_horizontal_line(0.0, 0.0, 10.0);
        let gap_threshold = 0.5;
        let spacing = 0.1; // Dense points within each segment

        // Create points with a gap in the middle
        let mut points: Vec<AssociatedPoint> = Vec::new();

        // First segment: 0-3m (dense points)
        let mut x = 0.0;
        while x <= 3.0 {
            let t = x / 10.0;
            points.push(make_associated_point(x, 0.0, 0, t));
            x += spacing;
        }

        // GAP: 3m to 7m (4m gap > 0.5m threshold)

        // Second segment: 7-10m (dense points)
        x = 7.0;
        while x <= 10.0 {
            let t = x / 10.0;
            points.push(make_associated_point(x, 0.0, 1, t));
            x += spacing;
        }

        let config = RefitConfig::default()
            .with_min_points(5)
            .with_gap_threshold(gap_threshold);
        let poses = HashMap::new();

        let result = refit_line(&points, &original, &poses, &config);

        assert!(result.is_success());
        assert!(
            matches!(result, RefitResult::Split(_)),
            "Expected Split, got {:?}",
            result
        );

        let lines = result.lines();
        assert_eq!(lines.len(), 2);

        // First line should cover ~0-3m
        assert!(lines[0].start.x < 1.0);
        assert!(lines[0].end.x < 4.0);

        // Second line should cover ~7-10m
        assert!(lines[1].start.x > 6.0);
        assert!(lines[1].end.x > 9.0);
    }

    #[test]
    fn test_refit_with_poses_weighting() {
        let original = make_horizontal_line(0.0, 0.0, 10.0);
        let gap_threshold = 0.5;
        let spacing = 0.1; // Dense points

        // Create dense points with noise
        let mut points: Vec<AssociatedPoint> = Vec::new();

        // Points close to robot (scan 0 at origin) - on y=0
        let mut x = 0.0;
        while x <= 5.0 {
            let t = x / 10.0;
            points.push(make_associated_point(x, 0.0, 0, t));
            x += spacing;
        }

        // Points far from robot (scan 1 at x=10) - with noise above y=0
        x = 5.0 + spacing;
        while x <= 10.0 {
            let t = x / 10.0;
            points.push(make_associated_point(x, 0.5, 1, t));
            x += spacing;
        }

        let mut poses = HashMap::new();
        poses.insert(0, Pose2D::new(0.0, 0.0, 0.0)); // Robot at origin for scan 0
        poses.insert(1, Pose2D::new(10.0, 0.0, 0.0)); // Robot at x=10 for scan 1

        let config = RefitConfig::default()
            .with_min_points(10)
            .with_gap_threshold(gap_threshold);

        let result = refit_line(&points, &original, &poses, &config);

        assert!(result.is_success());
        let lines = result.lines();
        assert_eq!(lines.len(), 1);

        // Line should be closer to y=0 due to close points having higher weight
        let line = &lines[0];
        let midpoint = line.midpoint();
        assert!(
            midpoint.y < 0.3,
            "Midpoint y={} should be < 0.3 (closer to close points)",
            midpoint.y
        );
    }

    #[test]
    fn test_refit_preserves_observation_count() {
        let mut original = make_horizontal_line(0.0, 0.0, 10.0);
        original.observation_count = 42;

        // Use dense points to avoid gap splitting
        let points = make_dense_points(0.0, 10.0, 0.0, 0.3);
        let config = RefitConfig::default()
            .with_min_points(10)
            .with_gap_threshold(0.3);
        let poses = HashMap::new();

        let result = refit_line(&points, &original, &poses, &config);

        let lines = result.lines();
        assert_eq!(lines.len(), 1, "Expected 1 line, got {}", lines.len());
        assert_eq!(lines[0].observation_count, 42);
    }

    #[test]
    fn test_refit_simple() {
        let original = make_horizontal_line(0.0, 0.0, 10.0);
        // Use dense points to avoid gap splitting
        let points = make_dense_points(0.0, 10.0, 0.05, 0.3);
        let config = RefitConfig::default()
            .with_min_points(10)
            .with_gap_threshold(0.3);

        let result = refit_line_simple(&points, &original, &config);

        assert!(result.is_success());
    }

    #[test]
    fn test_refit_stats() {
        let stats = RefitStats {
            lines_refitted_single: 5,
            lines_split: 2,
            lines_insufficient: 3,
            total_segments_created: 5, // 2 splits creating ~2.5 segments each
        };

        assert_eq!(stats.total_processed(), 10);
        assert!((stats.success_rate() - 0.7).abs() < 0.001);
    }

    #[test]
    fn test_find_gap_splits_no_gaps() {
        let original = make_horizontal_line(0.0, 0.0, 10.0);
        let gap_threshold = 0.5;
        let spacing = 0.2; // Smaller than gap_threshold

        // Create dense points with no gaps
        let points: Vec<AssociatedPoint> = (0..50)
            .map(|i| {
                let x = i as f32 * spacing;
                let t = x / 10.0;
                make_associated_point(x.min(10.0), 0.0, 0, t.min(1.0))
            })
            .collect();

        let sorted: Vec<&AssociatedPoint> = points.iter().collect();
        let splits = find_gap_splits(&sorted, gap_threshold, &original);

        assert!(splits.is_empty(), "Expected no splits, got {:?}", splits);
    }

    #[test]
    fn test_find_gap_splits_with_gap() {
        let original = make_horizontal_line(0.0, 0.0, 10.0);
        let gap_threshold = 0.5;

        // Create points with a clear gap
        let points = vec![
            make_associated_point(0.0, 0.0, 0, 0.0),
            make_associated_point(0.2, 0.0, 0, 0.02),
            make_associated_point(0.4, 0.0, 0, 0.04),
            // Gap here: 0.4 to 5.0 = 4.6m > 0.5m threshold
            make_associated_point(5.0, 0.0, 0, 0.5),
            make_associated_point(5.2, 0.0, 0, 0.52),
        ];

        let sorted: Vec<&AssociatedPoint> = points.iter().collect();
        let splits = find_gap_splits(&sorted, gap_threshold, &original);

        assert_eq!(splits.len(), 1, "Expected 1 split, got {:?}", splits);
        assert_eq!(splits[0], 3); // Split before the 4th point (index 3)
    }

    #[test]
    fn test_refitresult_lines() {
        let line = make_horizontal_line(0.0, 0.0, 10.0);

        let single = RefitResult::Single(line.clone());
        assert_eq!(single.lines().len(), 1);
        assert_eq!(single.line_count(), 1);
        assert!(single.is_success());

        let split = RefitResult::Split(vec![line.clone(), line.clone()]);
        assert_eq!(split.lines().len(), 2);
        assert_eq!(split.line_count(), 2);
        assert!(split.is_success());

        let insufficient = RefitResult::Insufficient;
        assert!(insufficient.lines().is_empty());
        assert_eq!(insufficient.line_count(), 0);
        assert!(!insufficient.is_success());
    }

    #[test]
    fn test_multiple_gaps_creates_multiple_segments() {
        let original = make_horizontal_line(0.0, 0.0, 10.0);

        // Create three clusters of points with gaps between
        let mut points: Vec<AssociatedPoint> = Vec::new();

        // Cluster 1: t = 0.0 to 0.2 (10 points)
        for i in 0..10 {
            let t = i as f32 / 50.0;
            points.push(make_associated_point(t * 10.0, 0.0, 0, t));
        }

        // Cluster 2: t = 0.4 to 0.6 (10 points) - gap before
        for i in 0..10 {
            let t = 0.4 + i as f32 / 50.0;
            points.push(make_associated_point(t * 10.0, 0.0, 0, t));
        }

        // Cluster 3: t = 0.8 to 1.0 (10 points) - gap before
        for i in 0..10 {
            let t = 0.8 + i as f32 / 50.0;
            points.push(make_associated_point(t * 10.0, 0.0, 0, t));
        }

        let config = RefitConfig::default()
            .with_min_points(5)
            .with_gap_threshold(0.3); // 0.3m < 2m gaps

        let poses = HashMap::new();
        let result = refit_line(&points, &original, &poses, &config);

        assert!(result.is_success());
        assert!(matches!(result, RefitResult::Split(_)));

        let lines = result.lines();
        assert_eq!(lines.len(), 3);
    }
}
