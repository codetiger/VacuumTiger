//! Submap configuration.

use serde::{Deserialize, Serialize};

/// Configuration for submap management.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SubmapConfig {
    /// Number of scans before transitioning to Filling state.
    /// Default: 50
    pub scans_per_submap: usize,

    /// Additional scans during Filling state (overlap period).
    /// These scans are added to both the filling submap and the new active submap.
    /// Default: 10
    pub overlap_scans: usize,

    /// Local grid size in cells (square).
    /// Default: 200 (5m × 5m at 2.5cm resolution)
    pub local_grid_size: usize,

    /// Grid resolution in meters per cell.
    /// Should match the global grid resolution.
    /// Default: 0.025 (2.5cm)
    pub resolution: f32,

    /// Distance traveled threshold for creating new submap (meters).
    /// If robot travels more than this within a submap, trigger new submap.
    /// Default: 3.0
    pub max_distance_per_submap: f32,

    /// Maximum number of submaps to keep in memory.
    /// Oldest finalized submaps may be evicted if exceeded.
    /// Default: 100 (sufficient for ~100m² home)
    pub max_submaps: usize,

    /// Minimum gap (in submap IDs) before considering loop closure.
    /// Prevents false positives from recent submaps.
    /// Default: 3
    pub min_loop_closure_gap: usize,

    /// Maximum distance (meters) for loop closure candidates.
    /// Submaps further than this are not considered for loop closure.
    /// Default: 10.0
    pub max_loop_closure_distance: f32,
}

impl Default for SubmapConfig {
    fn default() -> Self {
        Self {
            scans_per_submap: 50,
            overlap_scans: 10,
            local_grid_size: 200,
            resolution: 0.025,
            max_distance_per_submap: 3.0,
            max_submaps: 100,
            min_loop_closure_gap: 3,
            max_loop_closure_distance: 10.0,
        }
    }
}

impl SubmapConfig {
    /// Create a config optimized for small rooms (< 20m²).
    pub fn small_room() -> Self {
        Self {
            scans_per_submap: 30,
            overlap_scans: 5,
            local_grid_size: 160, // 4m × 4m
            max_distance_per_submap: 2.0,
            max_submaps: 50,
            ..Default::default()
        }
    }

    /// Create a config optimized for large spaces (> 50m²).
    pub fn large_space() -> Self {
        Self {
            scans_per_submap: 80,
            overlap_scans: 15,
            local_grid_size: 300, // 7.5m × 7.5m
            max_distance_per_submap: 5.0,
            max_submaps: 200,
            ..Default::default()
        }
    }

    /// Validate the configuration.
    pub fn validate(&self) -> Result<(), ConfigValidationError> {
        if self.scans_per_submap == 0 {
            return Err(ConfigValidationError::InvalidValue(
                "scans_per_submap must be > 0".to_string(),
            ));
        }

        if self.local_grid_size < 50 {
            return Err(ConfigValidationError::InvalidValue(
                "local_grid_size must be >= 50".to_string(),
            ));
        }

        if self.resolution <= 0.0 {
            return Err(ConfigValidationError::InvalidValue(
                "resolution must be > 0".to_string(),
            ));
        }

        if self.max_distance_per_submap <= 0.0 {
            return Err(ConfigValidationError::InvalidValue(
                "max_distance_per_submap must be > 0".to_string(),
            ));
        }

        // Check that local grid can fit the distance
        let grid_extent = self.local_grid_size as f32 * self.resolution;
        if self.max_distance_per_submap > grid_extent {
            return Err(ConfigValidationError::InvalidValue(format!(
                "max_distance_per_submap ({:.1}m) exceeds local grid extent ({:.1}m)",
                self.max_distance_per_submap, grid_extent
            )));
        }

        Ok(())
    }

    /// Get the local grid extent in meters.
    #[inline]
    pub fn local_grid_extent(&self) -> f32 {
        self.local_grid_size as f32 * self.resolution
    }

    /// Get approximate memory per submap in bytes.
    pub fn estimated_memory_per_submap(&self) -> usize {
        // Grid: cells × ~6 bytes per cell (type, confidence, etc.)
        let grid_bytes = self.local_grid_size * self.local_grid_size * 6;

        // Scans: ~1.4KB per scan (360 rays × 4 bytes each for ranges + angles)
        let scans_bytes = (self.scans_per_submap + self.overlap_scans) * 1400;

        grid_bytes + scans_bytes
    }

    /// Get total estimated memory for max submaps.
    pub fn estimated_total_memory(&self) -> usize {
        self.estimated_memory_per_submap() * self.max_submaps
    }
}

/// Error during configuration validation.
#[derive(Clone, Debug)]
pub enum ConfigValidationError {
    /// Invalid configuration value.
    InvalidValue(String),
}

impl std::fmt::Display for ConfigValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigValidationError::InvalidValue(msg) => write!(f, "Invalid config: {}", msg),
        }
    }
}

impl std::error::Error for ConfigValidationError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = SubmapConfig::default();
        assert_eq!(config.scans_per_submap, 50);
        assert_eq!(config.overlap_scans, 10);
        assert_eq!(config.local_grid_size, 200);
        assert!((config.resolution - 0.025).abs() < 1e-6);
    }

    #[test]
    fn test_config_validation() {
        let config = SubmapConfig::default();
        assert!(config.validate().is_ok());

        let mut bad_config = SubmapConfig::default();
        bad_config.scans_per_submap = 0;
        assert!(bad_config.validate().is_err());

        let mut bad_config = SubmapConfig::default();
        bad_config.resolution = -1.0;
        assert!(bad_config.validate().is_err());
    }

    #[test]
    fn test_local_grid_extent() {
        let config = SubmapConfig::default();
        let extent = config.local_grid_extent();
        // 200 cells × 0.025m = 5.0m
        assert!((extent - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_memory_estimation() {
        let config = SubmapConfig::default();
        let per_submap = config.estimated_memory_per_submap();

        // Grid: 200 × 200 × 6 = 240KB
        // Scans: 60 × 1.4KB = 84KB
        // Total: ~324KB
        assert!(per_submap > 300_000);
        assert!(per_submap < 400_000);

        // Total for 100 submaps: ~32MB
        let total = config.estimated_total_memory();
        assert!(total > 30_000_000);
        assert!(total < 40_000_000);
    }
}
