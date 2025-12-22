//! YAML configuration file loading for VectorMap.
//!
//! Load algorithm configuration from YAML files for reproducible experiments
//! and easy parameter tuning.
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_map::VectorMapConfig;
//!
//! // Load from file
//! let config = VectorMapConfig::from_yaml_file("config.yaml")?;
//!
//! // Validate before use
//! config.validate()?;
//!
//! let map = VectorMap::new(config);
//! ```

use std::io;
use std::path::Path;

use crate::vector_map::config::{ConfigError, VectorMapConfig};

/// Errors that can occur when loading YAML configuration.
#[derive(Debug)]
pub enum YamlConfigError {
    /// IO error reading the file.
    Io(io::Error),
    /// YAML parsing error.
    Parse(serde_yaml::Error),
    /// Configuration validation failed.
    Validation(ConfigError),
}

impl std::fmt::Display for YamlConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            YamlConfigError::Io(e) => write!(f, "IO error: {}", e),
            YamlConfigError::Parse(e) => write!(f, "YAML parse error: {}", e),
            YamlConfigError::Validation(e) => write!(f, "Config validation error: {}", e),
        }
    }
}

impl std::error::Error for YamlConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            YamlConfigError::Io(e) => Some(e),
            YamlConfigError::Parse(e) => Some(e),
            YamlConfigError::Validation(e) => Some(e),
        }
    }
}

impl From<io::Error> for YamlConfigError {
    fn from(err: io::Error) -> Self {
        YamlConfigError::Io(err)
    }
}

impl From<serde_yaml::Error> for YamlConfigError {
    fn from(err: serde_yaml::Error) -> Self {
        YamlConfigError::Parse(err)
    }
}

impl From<ConfigError> for YamlConfigError {
    fn from(err: ConfigError) -> Self {
        YamlConfigError::Validation(err)
    }
}

impl VectorMapConfig {
    /// Load configuration from a YAML file.
    ///
    /// Parses the file and validates the configuration.
    ///
    /// # Arguments
    /// * `path` - Path to the YAML configuration file
    ///
    /// # Errors
    /// Returns `YamlConfigError` if the file cannot be read, parsed, or validated.
    ///
    /// # Example
    /// ```rust,ignore
    /// let config = VectorMapConfig::from_yaml_file("config.yaml")?;
    /// let map = VectorMap::new(config);
    /// ```
    pub fn from_yaml_file<P: AsRef<Path>>(path: P) -> Result<Self, YamlConfigError> {
        let contents = std::fs::read_to_string(path)?;
        Self::from_yaml_str(&contents)
    }

    /// Load configuration from a YAML string.
    ///
    /// Parses the string and validates the configuration.
    ///
    /// # Example
    /// ```rust,ignore
    /// let yaml = r#"
    /// extraction:
    ///   split_threshold: 0.05
    /// "#;
    /// let config = VectorMapConfig::from_yaml_str(yaml)?;
    /// ```
    pub fn from_yaml_str(yaml: &str) -> Result<Self, YamlConfigError> {
        let mut config: VectorMapConfig = serde_yaml::from_str(yaml)?;

        // Finalize derived fields (like convergence_threshold_sq)
        config.finalize();

        // Validate configuration
        config.validate()?;

        Ok(config)
    }

    /// Load configuration from a YAML file without validation.
    ///
    /// Use this if you want to manually validate or modify the config after loading.
    ///
    /// # Example
    /// ```rust,ignore
    /// let mut config = VectorMapConfig::from_yaml_file_unchecked("config.yaml")?;
    /// config.matching.max_iterations = 50;  // Modify
    /// config.validate()?;  // Manually validate
    /// ```
    pub fn from_yaml_file_unchecked<P: AsRef<Path>>(path: P) -> Result<Self, YamlConfigError> {
        let contents = std::fs::read_to_string(path)?;
        Self::from_yaml_str_unchecked(&contents)
    }

    /// Load configuration from a YAML string without validation.
    pub fn from_yaml_str_unchecked(yaml: &str) -> Result<Self, YamlConfigError> {
        let mut config: VectorMapConfig = serde_yaml::from_str(yaml)?;
        config.finalize();
        Ok(config)
    }

    /// Serialize configuration to a YAML string.
    ///
    /// Useful for saving configurations or generating templates.
    ///
    /// # Example
    /// ```rust,ignore
    /// let config = VectorMapConfig::default();
    /// let yaml = config.to_yaml_string()?;
    /// println!("{}", yaml);
    /// ```
    pub fn to_yaml_string(&self) -> Result<String, serde_yaml::Error> {
        serde_yaml::to_string(self)
    }

    /// Save configuration to a YAML file.
    ///
    /// # Example
    /// ```rust,ignore
    /// let config = VectorMapConfig::default();
    /// config.to_yaml_file("reference.yaml")?;
    /// ```
    pub fn to_yaml_file<P: AsRef<Path>>(&self, path: P) -> Result<(), YamlConfigError> {
        let yaml = self.to_yaml_string()?;
        std::fs::write(path, yaml)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_roundtrip_default_config() {
        let original = VectorMapConfig::default();
        let yaml = original.to_yaml_string().expect("Failed to serialize");
        let loaded = VectorMapConfig::from_yaml_str(&yaml).expect("Failed to deserialize");

        // Key fields should match
        assert_eq!(
            original.matching.max_iterations,
            loaded.matching.max_iterations
        );
        assert_eq!(original.use_hybrid_extraction, loaded.use_hybrid_extraction);
        assert_eq!(original.mapping_enabled, loaded.mapping_enabled);
    }

    #[test]
    fn test_validation_called_on_load() {
        // Create invalid YAML with zero iterations
        let yaml = r#"
extraction:
  split_threshold: 0.05
  min_points: 5
  min_length: 0.1
  max_point_gap: 0.3
  merge_threshold: 0.05
  adaptive_range_scale: 0.03
ransac_extraction:
  iterations: 0
  inlier_threshold: 0.02
  min_inliers: 5
  min_line_length: 0.1
  max_point_gap: 0.1
  max_lines: 0
  seed: 0
use_hybrid_extraction: true
corner:
  min_angle: 0.524
  max_angle: 2.618
  max_endpoint_distance: 0.05
  curvature_window_size: 3
  curvature_threshold: 0.785
  merge_distance: 0.1
  nms_radius: 0.15
matching:
  max_iterations: 30
  convergence_threshold: 0.0001
  max_correspondence_distance: 0.5
  robust_cost:
    type: Huber
    delta: 0.03
  use_irls: true
  min_correspondences: 10
  min_confidence: 0.3
  use_ransac_init: true
  use_batch_search: true
  noise_model:
    sigma_base: 0.01
    sigma_range: 0.001
  use_coarse_search: true
  coarse_search_confidence_threshold: 0.5
  multi_resolution:
    enabled: true
    levels: 3
    subsample_factor: 2
    iterations_per_level: [3, 5, 10]
association:
  max_perpendicular_distance: 0.15
  max_angle_difference: 0.2
  min_overlap_ratio: 0.3
  max_endpoint_gap: 0.3
  distance_weight: 10.0
  angle_weight: 5.0
merger:
  map_weight: 0.8
  extend_endpoints: true
  max_extension: 0.5
  min_extension: 0.05
frontier:
  connection_distance: 0.3
  min_distance_from_robot: 0.5
  min_openness: 0.3
occupancy:
  obstacle_distance: 0.1
  known_distance: 2.0
  use_winding_number: false
path_planning:
  robot_radius: 0.15
  max_nodes: 1000
  conservative: true
loop_closure:
  min_travel_distance: 5.0
  max_descriptor_distance: 0.5
  min_verification_confidence: 0.6
  keyframe_interval: 2.0
  descriptor_radius: 2.0
  max_candidates_to_verify: 3
  min_lines_for_keyframe: 3
  min_corners_for_keyframe: 2
  icp_config:
    max_iterations: 30
    convergence_threshold: 0.0001
    max_correspondence_distance: 0.5
    robust_cost:
      type: Huber
      delta: 0.03
    use_irls: true
    min_correspondences: 10
    min_confidence: 0.3
    use_ransac_init: true
    use_batch_search: true
    noise_model:
      sigma_base: 0.01
      sigma_range: 0.001
    use_coarse_search: true
    coarse_search_confidence_threshold: 0.5
    multi_resolution:
      enabled: true
      levels: 3
      subsample_factor: 2
      iterations_per_level: [3, 5, 10]
min_match_confidence: 0.3
mapping_enabled: true
loop_closure_enabled: true
"#;

        let result = VectorMapConfig::from_yaml_str(yaml);
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            YamlConfigError::Validation(_)
        ));
    }

    #[test]
    fn test_unchecked_load_skips_validation() {
        // Same invalid YAML but using unchecked method
        let yaml = r#"
extraction:
  split_threshold: 0.05
  min_points: 5
  min_length: 0.1
  max_point_gap: 0.3
  merge_threshold: 0.05
  adaptive_range_scale: 0.03
ransac_extraction:
  iterations: 0
  inlier_threshold: 0.02
  min_inliers: 5
  min_line_length: 0.1
  max_point_gap: 0.1
  max_lines: 0
  seed: 0
use_hybrid_extraction: true
corner:
  min_angle: 0.524
  max_angle: 2.618
  max_endpoint_distance: 0.05
  curvature_window_size: 3
  curvature_threshold: 0.785
  merge_distance: 0.1
  nms_radius: 0.15
matching:
  max_iterations: 30
  convergence_threshold: 0.0001
  max_correspondence_distance: 0.5
  robust_cost:
    type: Huber
    delta: 0.03
  use_irls: true
  min_correspondences: 10
  min_confidence: 0.3
  use_ransac_init: true
  use_batch_search: true
  noise_model:
    sigma_base: 0.01
    sigma_range: 0.001
  use_coarse_search: true
  coarse_search_confidence_threshold: 0.5
  multi_resolution:
    enabled: true
    levels: 3
    subsample_factor: 2
    iterations_per_level: [3, 5, 10]
association:
  max_perpendicular_distance: 0.15
  max_angle_difference: 0.2
  min_overlap_ratio: 0.3
  max_endpoint_gap: 0.3
  distance_weight: 10.0
  angle_weight: 5.0
merger:
  map_weight: 0.8
  extend_endpoints: true
  max_extension: 0.5
  min_extension: 0.05
frontier:
  connection_distance: 0.3
  min_distance_from_robot: 0.5
  min_openness: 0.3
occupancy:
  obstacle_distance: 0.1
  known_distance: 2.0
  use_winding_number: false
path_planning:
  robot_radius: 0.15
  max_nodes: 1000
  conservative: true
loop_closure:
  min_travel_distance: 5.0
  max_descriptor_distance: 0.5
  min_verification_confidence: 0.6
  keyframe_interval: 2.0
  descriptor_radius: 2.0
  max_candidates_to_verify: 3
  min_lines_for_keyframe: 3
  min_corners_for_keyframe: 2
  icp_config:
    max_iterations: 30
    convergence_threshold: 0.0001
    max_correspondence_distance: 0.5
    robust_cost:
      type: Huber
      delta: 0.03
    use_irls: true
    min_correspondences: 10
    min_confidence: 0.3
    use_ransac_init: true
    use_batch_search: true
    noise_model:
      sigma_base: 0.01
      sigma_range: 0.001
    use_coarse_search: true
    coarse_search_confidence_threshold: 0.5
    multi_resolution:
      enabled: true
      levels: 3
      subsample_factor: 2
      iterations_per_level: [3, 5, 10]
min_match_confidence: 0.3
mapping_enabled: true
loop_closure_enabled: true
"#;

        // Should succeed with unchecked method
        let result = VectorMapConfig::from_yaml_str_unchecked(yaml);
        assert!(result.is_ok());

        // But validation should fail
        let config = result.unwrap();
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_empty_yaml_fails() {
        let result = VectorMapConfig::from_yaml_str("");
        assert!(result.is_err());
    }

    #[test]
    fn test_io_error_conversion() {
        let io_err = io::Error::new(io::ErrorKind::NotFound, "file not found");
        let yaml_err: YamlConfigError = io_err.into();
        assert!(matches!(yaml_err, YamlConfigError::Io(_)));
    }

    #[test]
    fn test_load_default_yaml_file() {
        // Load the reference config file from configs/default.yaml
        let config_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("configs")
            .join("default.yaml");

        let config = VectorMapConfig::from_yaml_file(&config_path)
            .expect("Failed to load configs/default.yaml");

        // Verify key values match expected defaults
        assert_eq!(config.matching.max_iterations, 30);
        assert!(config.use_hybrid_extraction);
        assert!(config.mapping_enabled);
        assert!(config.loop_closure_enabled);
        assert!(config.validate().is_ok());
    }
}
