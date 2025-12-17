//! Dynamic matcher selection for runtime algorithm switching.
//!
//! Provides runtime-selectable scan matchers through the [`DynMatcher`] enum,
//! allowing applications to switch between matching algorithms based on configuration.
//!
//! # Matcher Types
//!
//! - **Icp**: Point-to-Point ICP (fast, needs good initial guess)
//! - **P2l**: Point-to-Line ICP (better for structured environments)
//! - **Correlative**: Exhaustive search (handles large initial errors)
//! - **MultiRes**: Multi-resolution correlative (speed/robustness balance)
//! - **HybridIcp**: Correlative + Point-to-Point ICP
//! - **HybridP2l**: Correlative + Point-to-Line ICP (recommended)
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::algorithms::matching::{DynMatcher, DynMatcherConfig, MatcherType, ScanMatcher};
//!
//! // Create matcher with runtime-selected type
//! let config = DynMatcherConfig::default();
//! let mut matcher = DynMatcher::new(MatcherType::HybridP2l, config);
//!
//! // Match scans
//! let result = matcher.match_scans(&source, &target, &initial_guess);
//! if result.converged {
//!     println!("Transform: {:?}", result.transform);
//! }
//! ```

use clap::ValueEnum;
use serde::{Deserialize, Serialize};

use crate::algorithms::matching::{
    CorrelativeConfig, CorrelativeMatcher, HybridIcpMatcher, HybridIcpMatcherConfig,
    HybridP2LMatcher, HybridP2LMatcherConfig, IcpConfig, MultiResolutionConfig,
    MultiResolutionMatcher, PointToLineIcp, PointToLineIcpConfig, PointToPointIcp, ScanMatchResult,
    ScanMatcher,
};
use crate::core::types::{PointCloud2D, Pose2D};

/// Available scan matcher algorithm types.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, ValueEnum, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum MatcherType {
    /// Point-to-Point ICP.
    ///
    /// Classic Iterative Closest Point algorithm.
    /// Fast but needs a good initial guess to converge.
    Icp,

    /// Point-to-Line ICP.
    ///
    /// Point-to-line correspondence with line extraction.
    /// Better for structured environments with walls.
    P2l,

    /// Correlative matcher.
    ///
    /// Exhaustive search over discretized pose space.
    /// Handles large initial errors but slower.
    Correlative,

    /// Multi-resolution correlative matcher.
    ///
    /// Hierarchical correlative search with multiple resolution levels.
    /// Balances speed and robustness.
    MultiRes,

    /// Hybrid Correlative + Point-to-Point ICP.
    ///
    /// Uses correlative search for coarse alignment, then P2P ICP for refinement.
    HybridIcp,

    /// Hybrid Correlative + Point-to-Line ICP.
    ///
    /// Uses correlative search for coarse alignment, then P2L ICP for refinement.
    /// Best accuracy for structured environments. **Recommended.**
    #[default]
    HybridP2l,
}

impl std::fmt::Display for MatcherType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MatcherType::Icp => write!(f, "ICP"),
            MatcherType::P2l => write!(f, "P2L"),
            MatcherType::Correlative => write!(f, "Correlative"),
            MatcherType::MultiRes => write!(f, "MultiRes"),
            MatcherType::HybridIcp => write!(f, "HybridICP"),
            MatcherType::HybridP2l => write!(f, "HybridP2L"),
        }
    }
}

/// Configuration for all matcher types.
///
/// Each field configures its respective matcher algorithm.
/// Only the relevant configs are used based on the selected [`MatcherType`].
#[derive(Debug, Clone)]
pub struct DynMatcherConfig {
    /// Configuration for correlative matcher (used by Correlative, HybridIcp, HybridP2l).
    pub correlative: CorrelativeConfig,

    /// Configuration for Point-to-Point ICP (used by Icp, HybridIcp).
    pub icp: IcpConfig,

    /// Configuration for Point-to-Line ICP (used by P2l, HybridP2l).
    pub p2l: PointToLineIcpConfig,

    /// Configuration for multi-resolution matcher.
    pub multi_res: MultiResolutionConfig,

    /// If true, always run correlative/coarse search first for hybrid matchers.
    /// If false, only run on ICP failure.
    pub always_correlative: bool,

    /// Encoder weight for initial guess (0.0 = identity, 1.0 = full odometry).
    pub encoder_weight: f32,
}

impl Default for DynMatcherConfig {
    fn default() -> Self {
        Self {
            correlative: CorrelativeConfig {
                search_window_x: 0.3,
                search_window_y: 0.3,
                search_window_theta: 0.5,
                linear_resolution: 0.03,
                angular_resolution: 0.03,
                grid_resolution: 0.05,
                min_score: 0.4,
            },
            icp: IcpConfig::default(),
            p2l: PointToLineIcpConfig::default(),
            multi_res: MultiResolutionConfig::default(),
            always_correlative: true,
            encoder_weight: 1.0,
        }
    }
}

// ============================================================================
// Dynamic Matcher Enum
// ============================================================================

/// Runtime-selectable scan matcher implementation.
///
/// Wraps all available scan matching algorithms behind a unified interface,
/// enabling runtime algorithm selection based on configuration.
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::algorithms::matching::{DynMatcher, DynMatcherConfig, MatcherType, ScanMatcher};
///
/// // Create with default config
/// let mut matcher = DynMatcher::new(MatcherType::HybridP2l, DynMatcherConfig::default());
///
/// // Match scans
/// let result = matcher.match_scans(&source, &target, &initial_guess);
/// println!("Score: {:.3}, Converged: {}", result.score, result.converged);
/// ```
#[derive(Debug)]
#[non_exhaustive]
pub enum DynMatcher {
    /// Point-to-Point ICP matcher.
    Icp(Box<PointToPointIcp>),
    /// Point-to-Line ICP matcher.
    P2l(Box<PointToLineIcp>),
    /// Correlative matcher.
    Correlative(Box<CorrelativeMatcher>),
    /// Multi-resolution correlative matcher.
    MultiRes(Box<MultiResolutionMatcher>),
    /// Hybrid Correlative + P2P ICP matcher.
    HybridIcp(Box<HybridIcpMatcher>),
    /// Hybrid Correlative + P2L ICP matcher.
    HybridP2l(Box<HybridP2LMatcher>),
}

impl DynMatcher {
    /// Create a new dynamic matcher instance.
    ///
    /// # Arguments
    ///
    /// * `matcher_type` - The type of matching algorithm to use
    /// * `config` - Configuration containing settings for all algorithm types
    pub fn new(matcher_type: MatcherType, config: DynMatcherConfig) -> Self {
        match matcher_type {
            MatcherType::Icp => DynMatcher::Icp(Box::new(PointToPointIcp::new(config.icp))),
            MatcherType::P2l => DynMatcher::P2l(Box::new(PointToLineIcp::new(config.p2l))),
            MatcherType::Correlative => {
                DynMatcher::Correlative(Box::new(CorrelativeMatcher::new(config.correlative)))
            }
            MatcherType::MultiRes => {
                DynMatcher::MultiRes(Box::new(MultiResolutionMatcher::new(config.multi_res)))
            }
            MatcherType::HybridIcp => {
                let hybrid_config = HybridIcpMatcherConfig {
                    correlative: config.correlative,
                    icp: config.icp,
                    always_correlative: config.always_correlative,
                };
                DynMatcher::HybridIcp(Box::new(HybridIcpMatcher::from_config(hybrid_config)))
            }
            MatcherType::HybridP2l => {
                let hybrid_config = HybridP2LMatcherConfig {
                    correlative: config.correlative,
                    p2l_icp: config.p2l,
                    always_correlative: config.always_correlative,
                    encoder_weight: config.encoder_weight,
                };
                DynMatcher::HybridP2l(Box::new(HybridP2LMatcher::from_config(hybrid_config)))
            }
        }
    }
}

impl ScanMatcher for DynMatcher {
    fn match_scans(
        &mut self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        match self {
            DynMatcher::Icp(m) => m.match_scans(source, target, initial_guess),
            DynMatcher::P2l(m) => m.match_scans(source, target, initial_guess),
            DynMatcher::Correlative(m) => m.match_scans(source, target, initial_guess),
            DynMatcher::MultiRes(m) => m.match_scans(source, target, initial_guess),
            DynMatcher::HybridIcp(m) => m.match_scans(source, target, initial_guess),
            DynMatcher::HybridP2l(m) => m.match_scans(source, target, initial_guess),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::matching::test_utils::create_room;
    use approx::assert_relative_eq;

    #[test]
    fn test_matcher_type_display() {
        assert_eq!(format!("{}", MatcherType::Icp), "ICP");
        assert_eq!(format!("{}", MatcherType::P2l), "P2L");
        assert_eq!(format!("{}", MatcherType::Correlative), "Correlative");
        assert_eq!(format!("{}", MatcherType::MultiRes), "MultiRes");
        assert_eq!(format!("{}", MatcherType::HybridIcp), "HybridICP");
        assert_eq!(format!("{}", MatcherType::HybridP2l), "HybridP2L");
    }

    #[test]
    fn test_matcher_type_default() {
        assert_eq!(MatcherType::default(), MatcherType::HybridP2l);
    }

    #[test]
    fn test_dyn_matcher_icp() {
        let config = DynMatcherConfig::default();
        let mut matcher = DynMatcher::new(MatcherType::Icp, config);

        // Test with identical clouds
        let cloud = create_room(100, 4.0, 3.0);
        let result = matcher.match_scans(&cloud, &cloud, &Pose2D::identity());

        assert!(result.converged);
        assert_relative_eq!(result.transform.x, 0.0, epsilon = 0.05);
    }

    #[test]
    fn test_dyn_matcher_p2l() {
        let config = DynMatcherConfig::default();
        let mut matcher = DynMatcher::new(MatcherType::P2l, config);

        let cloud = create_room(100, 4.0, 3.0);
        let result = matcher.match_scans(&cloud, &cloud, &Pose2D::identity());

        assert!(result.converged);
    }

    #[test]
    fn test_dyn_matcher_correlative() {
        let config = DynMatcherConfig::default();
        let mut matcher = DynMatcher::new(MatcherType::Correlative, config);

        let source = create_room(100, 4.0, 3.0);
        let transform = Pose2D::new(0.15, 0.1, 0.2);
        let target = source.transform(&transform);

        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged);
        assert_relative_eq!(result.transform.x, 0.15, epsilon = 0.06);
    }

    #[test]
    fn test_dyn_matcher_multi_res() {
        let config = DynMatcherConfig::default();
        let mut matcher = DynMatcher::new(MatcherType::MultiRes, config);

        let source = create_room(100, 4.0, 3.0);
        let transform = Pose2D::new(0.1, 0.08, 0.1);
        let target = source.transform(&transform);

        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged);
    }

    #[test]
    fn test_dyn_matcher_hybrid_icp() {
        let config = DynMatcherConfig::default();
        let mut matcher = DynMatcher::new(MatcherType::HybridIcp, config);

        // Test with identical scans - HybridIcp should converge to identity
        let cloud = create_room(100, 4.0, 3.0);
        let result = matcher.match_scans(&cloud, &cloud, &Pose2D::identity());

        assert!(result.converged);
        // Should converge close to identity for identical scans
        assert_relative_eq!(result.transform.x, 0.0, epsilon = 0.05);
        assert_relative_eq!(result.transform.y, 0.0, epsilon = 0.05);
    }

    #[test]
    fn test_dyn_matcher_hybrid_p2l() {
        let config = DynMatcherConfig::default();
        let mut matcher = DynMatcher::new(MatcherType::HybridP2l, config);

        let source = create_room(100, 4.0, 3.0);
        let transform = Pose2D::new(0.2, 0.15, 0.3);
        let target = source.transform(&transform);

        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged);
        assert_relative_eq!(result.transform.x, 0.2, epsilon = 0.08);
        assert_relative_eq!(result.transform.y, 0.15, epsilon = 0.08);
    }
}
