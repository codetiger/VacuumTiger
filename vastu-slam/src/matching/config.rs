//! SLAM configuration types.

use serde::{Deserialize, Serialize};

/// Configuration for the correlative scan matcher.
///
/// The correlative matcher performs a brute-force search over a 3D space
/// (x, y, theta) to find the best alignment of a lidar scan to the current map.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CorrelativeMatcherConfig {
    /// Search window in X direction (meters).
    /// The matcher will search from -search_x to +search_x relative to prior pose.
    #[serde(default = "default_search_xy")]
    pub search_x: f32,

    /// Search window in Y direction (meters).
    /// The matcher will search from -search_y to +search_y relative to prior pose.
    #[serde(default = "default_search_xy")]
    pub search_y: f32,

    /// Search window in theta (radians).
    /// The matcher will search from -search_theta to +search_theta relative to prior pose.
    #[serde(default = "default_search_theta")]
    pub search_theta: f32,

    /// Linear search resolution (meters).
    /// Step size for X and Y search dimensions.
    #[serde(default = "default_linear_resolution")]
    pub linear_resolution: f32,

    /// Angular search resolution (radians).
    /// Step size for theta search dimension.
    #[serde(default = "default_angular_resolution")]
    pub angular_resolution: f32,

    /// Minimum acceptable match score (0.0 to 1.0).
    /// Matches below this threshold will be rejected.
    #[serde(default = "default_min_score")]
    pub min_score: f32,

    /// Whether to use multi-resolution search.
    /// First does coarse search, then refines around best candidate.
    #[serde(default = "default_multi_resolution")]
    pub multi_resolution: bool,

    /// Coarse resolution multiplier for multi-resolution search.
    /// The coarse search uses resolution * coarse_factor.
    #[serde(default = "default_coarse_factor")]
    pub coarse_factor: f32,

    /// Maximum number of scan points to use.
    /// Subsamples if scan has more points. 0 = use all.
    #[serde(default)]
    pub max_points: usize,

    /// Whether matching is enabled.
    /// If false, matcher returns prior pose unchanged.
    #[serde(default = "default_true")]
    pub enabled: bool,

    /// Sensor offset from robot center in robot frame (x, y).
    /// This offset is applied when transforming scan points to world frame.
    /// Typically the lidar is mounted behind the robot center (negative x).
    #[serde(default)]
    pub sensor_offset: (f32, f32),

    /// Whether to use Gaussian scoring based on distance field.
    /// If true, uses continuous distance-to-wall scoring instead of binary cell type.
    /// This provides better gradients for small pose changes.
    #[serde(default = "default_true")]
    pub use_gaussian_scoring: bool,

    /// Standard deviation for Gaussian scoring (meters).
    /// Controls how quickly score drops off with distance from wall.
    /// - sigma=0.05 (5cm): Very tight, requires near-exact alignment
    /// - sigma=0.10 (10cm): Good balance for 5cm grid resolution
    /// - sigma=0.15 (15cm): More forgiving, smoother gradients
    #[serde(default = "default_gaussian_sigma")]
    pub gaussian_sigma: f32,

    // === Gauss-Newton Refinement ===
    /// Maximum iterations for Gauss-Newton refinement.
    /// Typically converges in 3-5 iterations.
    #[serde(default = "default_gn_max_iterations")]
    pub gn_max_iterations: usize,

    /// Convergence threshold for Gauss-Newton (meters).
    /// Stop when pose update magnitude is below this.
    #[serde(default = "default_gn_convergence")]
    pub gn_convergence_threshold: f32,

    /// Levenberg-Marquardt damping factor for Gauss-Newton.
    /// Higher values make optimization more stable but slower.
    /// 0.0 = pure Gauss-Newton, 0.001-0.01 = typical LM damping.
    #[serde(default = "default_gn_damping")]
    pub gn_damping: f32,

    /// Translation weight for Gauss-Newton prior constraint.
    /// Penalizes deviation from the correlative search result.
    /// Higher = trust prior more, lower = trust scan matching more.
    /// Cartographer uses ~1e5 for indoor environments.
    #[serde(default = "default_gn_translation_weight")]
    pub gn_translation_weight: f32,

    /// Rotation weight for Gauss-Newton prior constraint.
    /// Penalizes angular deviation from the correlative search result.
    #[serde(default = "default_gn_rotation_weight")]
    pub gn_rotation_weight: f32,

    /// Whether to use parallel search (rayon).
    /// Parallelizes the pose search loop across CPU cores.
    /// Recommended for multi-core systems (4+ cores).
    #[serde(default)]
    pub use_parallel: bool,
}

fn default_search_xy() -> f32 {
    0.3 // 30cm search window
}

fn default_search_theta() -> f32 {
    0.15 // ~8.6 degrees - appropriate for inter-scan corrections
}

fn default_linear_resolution() -> f32 {
    0.02 // 2cm steps
}

fn default_angular_resolution() -> f32 {
    0.02 // ~1.1 degrees
}

fn default_min_score() -> f32 {
    0.5
}

fn default_true() -> bool {
    true
}

fn default_multi_resolution() -> bool {
    true // Multi-resolution for efficiency
}

fn default_coarse_factor() -> f32 {
    2.0 // Reduced from 4.0 to improve accuracy (0.04m coarse steps instead of 0.08m)
}

fn default_gaussian_sigma() -> f32 {
    0.10 // 10cm - good balance for 5cm grid resolution
}

fn default_gn_max_iterations() -> usize {
    10 // Typically converges in 3-5 iterations
}

fn default_gn_convergence() -> f32 {
    1e-5 // Sub-millimeter convergence
}

fn default_gn_damping() -> f32 {
    1e-3 // Light LM damping for stability
}

fn default_gn_translation_weight() -> f32 {
    500.0 // Very strong translation prior - encoders are typically reliable for distance
}

fn default_gn_rotation_weight() -> f32 {
    20.0 // Lower rotation weight - allow scan matching to correct heading drift
}

impl Default for CorrelativeMatcherConfig {
    fn default() -> Self {
        Self {
            search_x: default_search_xy(),
            search_y: default_search_xy(),
            search_theta: default_search_theta(),
            linear_resolution: default_linear_resolution(),
            angular_resolution: default_angular_resolution(),
            min_score: default_min_score(),
            multi_resolution: default_multi_resolution(),
            coarse_factor: default_coarse_factor(),
            max_points: 0,
            enabled: true,
            sensor_offset: (0.0, 0.0),
            use_gaussian_scoring: true,
            gaussian_sigma: default_gaussian_sigma(),
            gn_max_iterations: default_gn_max_iterations(),
            gn_convergence_threshold: default_gn_convergence(),
            gn_damping: default_gn_damping(),
            gn_translation_weight: default_gn_translation_weight(),
            gn_rotation_weight: default_gn_rotation_weight(),
            use_parallel: false,
        }
    }
}

impl CorrelativeMatcherConfig {
    /// Create a fast configuration with reduced search space.
    /// Good for real-time applications with accurate odometry.
    pub fn fast() -> Self {
        Self {
            search_x: 0.15,
            search_y: 0.15,
            search_theta: 0.25,
            linear_resolution: 0.03,
            angular_resolution: 0.03,
            max_points: 180,
            ..Default::default()
        }
    }

    /// Create a thorough configuration with larger search space.
    /// Good for situations with poor odometry or after long gaps.
    pub fn thorough() -> Self {
        Self {
            search_x: 0.5,
            search_y: 0.5,
            search_theta: 0.8,
            linear_resolution: 0.01,
            angular_resolution: 0.01,
            ..Default::default()
        }
    }

    /// Create a disabled matcher that just passes through poses.
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..Default::default()
        }
    }

    /// Calculate the number of search poses to evaluate.
    pub fn search_space_size(&self) -> usize {
        let x_steps = (2.0 * self.search_x / self.linear_resolution) as usize + 1;
        let y_steps = (2.0 * self.search_y / self.linear_resolution) as usize + 1;
        let theta_steps = (2.0 * self.search_theta / self.angular_resolution) as usize + 1;
        x_steps * y_steps * theta_steps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = CorrelativeMatcherConfig::default();
        assert!((config.search_x - 0.3).abs() < 0.001);
        assert!(config.enabled);
        assert!(config.multi_resolution);
    }

    #[test]
    fn test_search_space_size() {
        let config = CorrelativeMatcherConfig::default();
        let size = config.search_space_size();
        // With defaults: 31 x 31 x 51 = 49,011 poses
        assert!(size > 10000);
        assert!(size < 100000);
    }

    #[test]
    fn test_fast_config() {
        let config = CorrelativeMatcherConfig::fast();
        let default_config = CorrelativeMatcherConfig::default();
        assert!(config.search_space_size() < default_config.search_space_size());
    }
}
