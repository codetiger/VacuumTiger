//! Traits for line extraction algorithms.
//!
//! This module defines the `LineExtractor` trait which allows different
//! line extraction algorithms to be used interchangeably.

use crate::core::Point2D;
use crate::features::Line2D;

/// Trait for line extraction algorithms.
///
/// Implementations extract line segments from ordered point sequences
/// (typically from a lidar scan).
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::extraction::{LineExtractor, SplitMergeExtractor, SplitMergeConfig};
/// use vastu_map::core::Point2D;
///
/// let extractor = SplitMergeExtractor::new(SplitMergeConfig::default());
/// let points = vec![
///     Point2D::new(0.0, 1.0),
///     Point2D::new(0.5, 1.0),
///     Point2D::new(1.0, 1.0),
/// ];
/// let lines = extractor.extract(&points);
/// ```
pub trait LineExtractor: Send + Sync {
    /// Extract line segments from an ordered sequence of points.
    ///
    /// Points should be ordered (e.g., by angle in a lidar scan).
    fn extract(&self, points: &[Point2D]) -> Vec<Line2D>;

    /// Extract line segments with sensor position for adaptive processing.
    ///
    /// Uses sensor position for:
    /// - Adaptive thresholds (farther points get more lenient thresholds)
    /// - Weighted line fitting (closer points have more influence)
    ///
    /// Default implementation ignores sensor position and calls `extract()`.
    fn extract_from_sensor(&self, points: &[Point2D], _sensor_pos: Point2D) -> Vec<Line2D> {
        self.extract(points)
    }
}

/// Split-and-Merge line extractor.
///
/// Wraps the split-and-merge algorithm in a `LineExtractor` trait implementation.
#[derive(Clone, Debug)]
pub struct SplitMergeExtractor {
    config: super::SplitMergeConfig,
}

impl SplitMergeExtractor {
    /// Create a new extractor with the given configuration.
    pub fn new(config: super::SplitMergeConfig) -> Self {
        Self { config }
    }

    /// Get the configuration.
    pub fn config(&self) -> &super::SplitMergeConfig {
        &self.config
    }
}

impl Default for SplitMergeExtractor {
    fn default() -> Self {
        Self::new(super::SplitMergeConfig::default())
    }
}

impl LineExtractor for SplitMergeExtractor {
    fn extract(&self, points: &[Point2D]) -> Vec<Line2D> {
        super::extract_lines(points, &self.config)
    }

    fn extract_from_sensor(&self, points: &[Point2D], sensor_pos: Point2D) -> Vec<Line2D> {
        super::extract_lines_from_sensor(points, &self.config, sensor_pos)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_split_merge_extractor() {
        let extractor = SplitMergeExtractor::default();

        // Generate points along a horizontal line
        let points: Vec<Point2D> = (0..20).map(|i| Point2D::new(i as f32 * 0.1, 0.0)).collect();

        let lines = extractor.extract(&points);

        assert_eq!(lines.len(), 1);
    }

    #[test]
    fn test_extractor_from_sensor() {
        let extractor = SplitMergeExtractor::default();
        let sensor_pos = Point2D::new(0.0, 0.0);

        let points: Vec<Point2D> = (0..20)
            .map(|i| Point2D::new(2.0 + i as f32 * 0.1, 0.0))
            .collect();

        let lines = extractor.extract_from_sensor(&points, sensor_pos);

        assert_eq!(lines.len(), 1);
    }

    #[test]
    fn test_trait_object() {
        // Verify trait can be used as a trait object
        let extractor: Box<dyn LineExtractor> = Box::new(SplitMergeExtractor::default());

        let points: Vec<Point2D> = (0..20).map(|i| Point2D::new(i as f32 * 0.1, 0.0)).collect();

        let lines = extractor.extract(&points);
        assert_eq!(lines.len(), 1);
    }
}
