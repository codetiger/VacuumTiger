//! Configuration for Split-and-Merge line extraction.

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
