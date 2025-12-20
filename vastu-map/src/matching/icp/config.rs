//! Configuration for Point-to-Line ICP algorithm.

use crate::config::LidarNoiseModel;

use super::super::robust::RobustCostFunction;

/// Configuration for Point-to-Line ICP.
#[derive(Clone, Debug)]
pub struct IcpConfig {
    /// Maximum number of ICP iterations.
    /// Default: 30
    pub max_iterations: usize,

    /// Convergence threshold for pose change.
    /// If ‖Δpose‖ < threshold, ICP stops.
    /// Default: 1e-4
    pub convergence_threshold: f32,

    /// Pre-computed squared convergence threshold for efficient comparison.
    /// Avoids sqrt() in hot loop by comparing squared distances.
    pub(crate) convergence_threshold_sq: f32,

    /// Maximum correspondence distance (meters).
    /// Points farther than this from all lines are rejected.
    /// Default: 0.5m
    pub max_correspondence_distance: f32,

    /// Robust cost function for outlier handling.
    /// Uses M-estimators to down-weight outliers based on residual magnitude.
    /// Default: Huber with delta = 0.03 (3cm)
    pub robust_cost: RobustCostFunction,

    /// Enable Iteratively Reweighted Least Squares (IRLS).
    /// When true, correspondence weights are updated each iteration
    /// based on residuals and the robust cost function.
    /// Default: true
    pub use_irls: bool,

    /// Minimum number of correspondences required.
    /// Default: 10
    pub min_correspondences: usize,

    /// Minimum match confidence to consider successful.
    /// Default: 0.3
    pub min_confidence: f32,

    /// Whether to use RANSAC for initial pose estimate.
    /// Default: true
    pub use_ransac_init: bool,

    /// Whether to use batch (SIMD) correspondence search.
    /// Default: true
    pub use_batch_search: bool,

    /// Lidar noise model for correspondence weighting.
    /// Used to weight measurements by their estimated uncertainty.
    pub noise_model: LidarNoiseModel,

    /// Whether to use coarse search when odometry is unreliable.
    /// Default: false
    pub use_coarse_search: bool,

    /// Confidence threshold below which coarse search is triggered.
    /// Only used if use_coarse_search is true.
    /// Default: 0.5
    pub coarse_search_confidence_threshold: f32,

    /// Multi-resolution ICP configuration (coarse-to-fine matching).
    /// When enabled, matching starts with subsampled points and refines
    /// with progressively more points for better convergence.
    pub multi_resolution: MultiResolutionConfig,
}

impl Default for IcpConfig {
    fn default() -> Self {
        let convergence_threshold = 1e-4;
        Self {
            max_iterations: 30,
            convergence_threshold,
            convergence_threshold_sq: convergence_threshold * convergence_threshold,
            max_correspondence_distance: 0.5,
            robust_cost: RobustCostFunction::default(), // Huber with delta = 0.03
            use_irls: true,
            min_correspondences: 10,
            min_confidence: 0.3,
            use_ransac_init: true,
            use_batch_search: true,
            noise_model: LidarNoiseModel::default(),
            use_coarse_search: true, // Enabled: triggers when previous match confidence < threshold
            coarse_search_confidence_threshold: 0.5,
            multi_resolution: MultiResolutionConfig::default(),
        }
    }
}

impl IcpConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for maximum iterations.
    pub fn with_max_iterations(mut self, iterations: usize) -> Self {
        self.max_iterations = iterations;
        self
    }

    /// Builder-style setter for convergence threshold.
    pub fn with_convergence_threshold(mut self, threshold: f32) -> Self {
        self.convergence_threshold = threshold;
        self.convergence_threshold_sq = threshold * threshold;
        self
    }

    /// Builder-style setter for maximum correspondence distance.
    pub fn with_max_correspondence_distance(mut self, meters: f32) -> Self {
        self.max_correspondence_distance = meters;
        self
    }

    /// Builder-style setter for robust cost function.
    pub fn with_robust_cost(mut self, cost: RobustCostFunction) -> Self {
        self.robust_cost = cost;
        self
    }

    /// Builder-style setter for IRLS (Iteratively Reweighted Least Squares).
    pub fn with_irls(mut self, enabled: bool) -> Self {
        self.use_irls = enabled;
        self
    }

    /// Builder-style setter for batch search.
    pub fn with_batch_search(mut self, enabled: bool) -> Self {
        self.use_batch_search = enabled;
        self
    }

    /// Builder-style setter for minimum correspondences.
    pub fn with_min_correspondences(mut self, count: usize) -> Self {
        self.min_correspondences = count;
        self
    }

    /// Builder-style setter for RANSAC initialization.
    pub fn with_ransac_init(mut self, enabled: bool) -> Self {
        self.use_ransac_init = enabled;
        self
    }

    /// Builder-style setter for lidar noise model.
    pub fn with_noise_model(mut self, model: LidarNoiseModel) -> Self {
        self.noise_model = model;
        self
    }

    /// Builder-style setter for coarse search.
    pub fn with_coarse_search(mut self, enabled: bool) -> Self {
        self.use_coarse_search = enabled;
        self
    }

    /// Builder-style setter for coarse search confidence threshold.
    pub fn with_coarse_search_threshold(mut self, threshold: f32) -> Self {
        self.coarse_search_confidence_threshold = threshold;
        self
    }

    /// Builder-style setter for multi-resolution configuration.
    pub fn with_multi_resolution(mut self, config: MultiResolutionConfig) -> Self {
        self.multi_resolution = config;
        self
    }

    /// Disable multi-resolution matching.
    pub fn without_multi_resolution(mut self) -> Self {
        self.multi_resolution.enabled = false;
        self
    }
}

/// Configuration for coarse search initialization.
#[derive(Clone, Debug)]
pub struct CoarseSearchConfig {
    /// Maximum translation search range (meters).
    /// Search from -range to +range in both x and y.
    /// Default: 0.3m
    pub translation_range: f32,

    /// Translation step size (meters).
    /// Default: 0.05m (5cm)
    pub translation_step: f32,

    /// Maximum rotation search range (radians).
    /// Search from -range to +range.
    /// Default: 0.262 rad (~15°)
    pub rotation_range: f32,

    /// Rotation step size (radians).
    /// Default: 0.052 rad (~3°)
    pub rotation_step: f32,

    /// Point subsampling rate for fast scoring.
    /// Only use every Nth point for initial evaluation.
    /// Default: 4
    pub subsample_rate: usize,
}

impl Default for CoarseSearchConfig {
    fn default() -> Self {
        Self {
            translation_range: 0.3,
            translation_step: 0.05,
            rotation_range: 0.262, // ~15°
            rotation_step: 0.052,  // ~3°
            subsample_rate: 4,
        }
    }
}

/// Configuration for multi-resolution ICP (coarse-to-fine matching).
///
/// Multi-resolution ICP improves convergence by starting with subsampled
/// points (coarse) and progressively refining with more points (fine).
/// This helps escape local minima when the initial guess is poor.
#[derive(Clone, Debug)]
pub struct MultiResolutionConfig {
    /// Enable multi-resolution matching.
    /// Default: true
    pub enabled: bool,

    /// Number of pyramid levels.
    /// Level 0 is coarsest, level (levels-1) is finest (full resolution).
    /// Default: 3
    pub levels: usize,

    /// Subsampling factor between levels.
    /// Points at level i use every (subsample_factor^(levels-1-i)) points.
    /// Example with levels=3, factor=2: coarse=1/4, medium=1/2, fine=full
    /// Default: 2
    pub subsample_factor: usize,

    /// ICP iterations at each level (from coarsest to finest).
    /// Length should match `levels`. If shorter, last value is repeated.
    /// Default: [3, 5, 10]
    pub iterations_per_level: Vec<usize>,
}

impl Default for MultiResolutionConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            levels: 3,
            subsample_factor: 2,
            iterations_per_level: vec![3, 5, 10],
        }
    }
}

impl MultiResolutionConfig {
    /// Create a new multi-resolution config with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create an enabled config with default pyramid settings.
    pub fn enabled() -> Self {
        Self {
            enabled: true,
            ..Self::default()
        }
    }

    /// Builder-style setter for number of levels.
    pub fn with_levels(mut self, levels: usize) -> Self {
        self.levels = levels.max(1);
        self
    }

    /// Builder-style setter for subsample factor.
    pub fn with_subsample_factor(mut self, factor: usize) -> Self {
        self.subsample_factor = factor.max(1);
        self
    }

    /// Builder-style setter for iterations per level.
    pub fn with_iterations(mut self, iterations: Vec<usize>) -> Self {
        self.iterations_per_level = iterations;
        self
    }

    /// Get iterations for a given level (0 = coarsest).
    pub fn iterations_for_level(&self, level: usize) -> usize {
        if level < self.iterations_per_level.len() {
            self.iterations_per_level[level]
        } else {
            // Use last value if not enough entries
            *self.iterations_per_level.last().unwrap_or(&10)
        }
    }

    /// Get subsample rate for a given level (0 = coarsest).
    ///
    /// Returns the step size for point sampling at this level.
    pub fn subsample_for_level(&self, level: usize) -> usize {
        if level >= self.levels {
            return 1;
        }
        self.subsample_factor.pow((self.levels - 1 - level) as u32)
    }
}
