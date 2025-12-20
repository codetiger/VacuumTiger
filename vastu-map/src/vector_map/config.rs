//! Configuration for VectorMap.

use crate::extraction::{CornerConfig, RansacLineConfig, SplitMergeConfig};
use crate::integration::{AssociationConfig, MergerConfig};
use crate::loop_closure::LoopClosureConfig;
use crate::matching::IcpConfig;
use crate::query::{FrontierConfig, OccupancyConfig};

/// Configuration for VectorMap.
#[derive(Clone, Debug)]
pub struct VectorMapConfig {
    /// Configuration for line extraction (split-merge).
    pub extraction: SplitMergeConfig,

    /// Configuration for RANSAC line extraction.
    /// Used when `use_hybrid_extraction` is true.
    pub ransac_extraction: RansacLineConfig,

    /// Whether to use hybrid RANSAC + split-merge extraction.
    /// When true, RANSAC extracts dominant lines first, then split-merge
    /// processes remaining points. This is more robust to outliers.
    /// Default: true
    pub use_hybrid_extraction: bool,

    /// Configuration for corner detection.
    pub corner: CornerConfig,

    /// Configuration for scan matching (ICP).
    pub matching: IcpConfig,

    /// Configuration for line association.
    pub association: AssociationConfig,

    /// Configuration for line merging.
    pub merger: MergerConfig,

    /// Configuration for frontier detection.
    pub frontier: FrontierConfig,

    /// Configuration for occupancy queries.
    pub occupancy: OccupancyConfig,

    /// Configuration for loop closure detection.
    pub loop_closure: LoopClosureConfig,

    /// Minimum match confidence to use scan matching result.
    /// Below this, odometry is used instead.
    /// Default: 0.3
    pub min_match_confidence: f32,

    /// Whether to update the map with new observations.
    /// Set to false for localization-only mode.
    /// Default: true
    pub mapping_enabled: bool,

    /// Whether loop closure detection is enabled.
    /// Default: true
    pub loop_closure_enabled: bool,
}

impl Default for VectorMapConfig {
    fn default() -> Self {
        Self {
            extraction: SplitMergeConfig::default(),
            ransac_extraction: RansacLineConfig::default(),
            use_hybrid_extraction: true,
            corner: CornerConfig::default(),
            matching: IcpConfig::default(),
            association: AssociationConfig::default(),
            merger: MergerConfig::default(),
            frontier: FrontierConfig::default(),
            occupancy: OccupancyConfig::default(),
            loop_closure: LoopClosureConfig::default(),
            min_match_confidence: 0.3,
            mapping_enabled: true,
            loop_closure_enabled: true,
        }
    }
}

impl VectorMapConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    // ===== Preset Configurations =====

    /// Preset for fast but less accurate operation.
    ///
    /// Use this when CPU is limited or real-time is critical:
    /// - Fewer ICP iterations (15 vs 30)
    /// - Larger convergence threshold (5e-4 vs 1e-4)
    /// - Coarse search disabled
    /// - Loop closure disabled
    ///
    /// # Example
    /// ```
    /// use vastu_map::VectorMapConfig;
    /// let config = VectorMapConfig::fast();
    /// assert!(!config.loop_closure_enabled);
    /// ```
    pub fn fast() -> Self {
        let mut config = Self::default();
        config.matching.max_iterations = 15;
        config.matching.convergence_threshold = 5e-4;
        config.matching.use_coarse_search = false;
        config.loop_closure_enabled = false;
        config
    }

    /// Preset for high accuracy at the cost of speed.
    ///
    /// Use this when accuracy is more important than real-time:
    /// - More ICP iterations (50 vs 30)
    /// - Tighter convergence (5e-5 vs 1e-4)
    /// - Stricter line association thresholds
    /// - Coarse search enabled
    ///
    /// # Example
    /// ```
    /// use vastu_map::VectorMapConfig;
    /// let config = VectorMapConfig::accurate();
    /// assert_eq!(config.matching.max_iterations, 50);
    /// ```
    pub fn accurate() -> Self {
        let mut config = Self::default();
        config.matching.max_iterations = 50;
        config.matching.convergence_threshold = 5e-5;
        config.matching.use_coarse_search = true;
        config.matching.coarse_search_confidence_threshold = 0.6;
        config.association.max_perpendicular_distance = 0.10; // Stricter association
        config.association.max_angle_difference = 0.15;
        config
    }

    /// Preset for localization-only mode (no map updates).
    ///
    /// Use this when operating with a pre-built map:
    /// - Mapping disabled
    /// - Loop closure disabled
    /// - Lower min correspondences (map may be sparse)
    ///
    /// # Example
    /// ```
    /// use vastu_map::VectorMapConfig;
    /// let config = VectorMapConfig::localization_only();
    /// assert!(!config.mapping_enabled);
    /// ```
    pub fn localization_only() -> Self {
        Self {
            mapping_enabled: false,
            loop_closure_enabled: false,
            matching: crate::matching::IcpConfig {
                min_correspondences: 5,
                ..crate::matching::IcpConfig::default()
            },
            ..Self::default()
        }
    }

    // ===== Common Nested Config Shortcuts =====

    /// Set the maximum ICP iterations.
    ///
    /// Shortcut for modifying `config.matching.max_iterations`.
    pub fn with_max_icp_iterations(mut self, iterations: usize) -> Self {
        self.matching.max_iterations = iterations;
        self
    }

    /// Set the correspondence distance threshold.
    ///
    /// Shortcut for modifying `config.matching.max_correspondence_distance`.
    pub fn with_correspondence_distance(mut self, meters: f32) -> Self {
        self.matching.max_correspondence_distance = meters;
        self
    }

    /// Set the line association distance threshold.
    ///
    /// Shortcut for modifying `config.association.max_perpendicular_distance`.
    pub fn with_association_distance(mut self, meters: f32) -> Self {
        self.association.max_perpendicular_distance = meters;
        self
    }

    // ===== Builder Methods =====

    /// Builder-style setter for minimum match confidence.
    pub fn with_min_match_confidence(mut self, confidence: f32) -> Self {
        self.min_match_confidence = confidence;
        self
    }

    /// Builder-style setter for mapping enabled.
    pub fn with_mapping_enabled(mut self, enabled: bool) -> Self {
        self.mapping_enabled = enabled;
        self
    }

    /// Builder-style setter for loop closure enabled.
    pub fn with_loop_closure_enabled(mut self, enabled: bool) -> Self {
        self.loop_closure_enabled = enabled;
        self
    }

    /// Builder-style setter for hybrid RANSAC + split-merge extraction.
    ///
    /// When enabled, line extraction uses RANSAC to find dominant lines first,
    /// then split-merge to process remaining points. This is more robust to
    /// outliers and cluttered environments.
    pub fn with_hybrid_extraction(mut self, enabled: bool) -> Self {
        self.use_hybrid_extraction = enabled;
        self
    }

    /// Builder-style setter for multi-resolution ICP.
    ///
    /// When enabled, scan matching uses coarse-to-fine matching with
    /// subsampled point clouds for faster convergence from poor initial guesses.
    pub fn with_multi_resolution_icp(mut self, enabled: bool) -> Self {
        self.matching.multi_resolution.enabled = enabled;
        self
    }
}
