//! Configuration for Point-to-Line ICP algorithm.

use crate::config::LidarNoiseModel;

use super::super::ransac::RansacConfig;

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

    /// Maximum correspondence distance (meters).
    /// Points farther than this from all lines are rejected.
    /// Default: 0.5m
    pub max_correspondence_distance: f32,

    /// Outlier rejection method.
    /// Default: DistanceThreshold
    pub outlier_rejection: OutlierRejection,

    /// Minimum number of correspondences required.
    /// Default: 10
    pub min_correspondences: usize,

    /// Minimum match confidence to consider successful.
    /// Default: 0.3
    pub min_confidence: f32,

    /// Whether to use RANSAC for initial pose estimate.
    /// Default: false
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
}

/// Outlier rejection method.
#[derive(Clone, Debug)]
pub enum OutlierRejection {
    /// Reject by distance threshold (fast).
    DistanceThreshold(f32),
    /// Use RANSAC for robust rejection (slower but more robust).
    Ransac(RansacConfig),
    /// No outlier rejection.
    None,
}

impl Default for IcpConfig {
    fn default() -> Self {
        Self {
            max_iterations: 30,
            convergence_threshold: 1e-4,
            max_correspondence_distance: 0.5,
            outlier_rejection: OutlierRejection::DistanceThreshold(0.15),
            min_correspondences: 10,
            min_confidence: 0.3,
            use_ransac_init: false,
            use_batch_search: true,
            noise_model: LidarNoiseModel::default(),
            use_coarse_search: true, // Enabled: triggers when previous match confidence < threshold
            coarse_search_confidence_threshold: 0.5,
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
        self
    }

    /// Builder-style setter for maximum correspondence distance.
    pub fn with_max_correspondence_distance(mut self, meters: f32) -> Self {
        self.max_correspondence_distance = meters;
        self
    }

    /// Builder-style setter for outlier rejection method.
    pub fn with_outlier_rejection(mut self, method: OutlierRejection) -> Self {
        self.outlier_rejection = method;
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
