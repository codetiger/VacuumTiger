//! Configuration for loop closure detection.

use serde::{Deserialize, Serialize};

use crate::matching::IcpConfig;

/// Configuration for loop closure detection.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LoopClosureConfig {
    /// Minimum travel distance before considering loop closure (meters).
    /// Prevents detecting loops immediately after keyframe creation.
    /// Default: 5.0m
    pub min_travel_distance: f32,

    /// Maximum descriptor distance for candidate retrieval.
    /// Lower values are more selective but may miss valid loops.
    /// Default: 0.5
    pub max_descriptor_distance: f32,

    /// Minimum ICP confidence to accept a loop closure.
    /// Default: 0.6
    pub min_verification_confidence: f32,

    /// Distance between keyframes (meters).
    /// Default: 2.0m
    pub keyframe_interval: f32,

    /// Radius for computing corner descriptors (meters).
    /// Default: 2.0m
    pub descriptor_radius: f32,

    /// Maximum number of candidates to verify with ICP per keyframe.
    /// Default: 3
    pub max_candidates_to_verify: usize,

    /// Minimum number of lines required for a valid keyframe.
    /// Default: 3
    pub min_lines_for_keyframe: usize,

    /// Minimum number of corners required for a valid keyframe.
    /// Default: 2
    pub min_corners_for_keyframe: usize,

    /// ICP configuration for verification.
    pub icp_config: IcpConfig,
}

impl Default for LoopClosureConfig {
    fn default() -> Self {
        Self {
            min_travel_distance: 5.0,
            max_descriptor_distance: 0.5,
            min_verification_confidence: 0.6,
            keyframe_interval: 2.0,
            descriptor_radius: 2.0,
            max_candidates_to_verify: 3,
            min_lines_for_keyframe: 3,
            min_corners_for_keyframe: 2,
            icp_config: IcpConfig::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_default() {
        let config = LoopClosureConfig::default();
        assert_eq!(config.min_travel_distance, 5.0);
        assert_eq!(config.max_descriptor_distance, 0.5);
        assert_eq!(config.min_verification_confidence, 0.6);
        assert_eq!(config.keyframe_interval, 2.0);
    }
}
