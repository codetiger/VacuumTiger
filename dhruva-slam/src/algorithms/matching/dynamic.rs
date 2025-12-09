//! Dynamic matcher selection for runtime algorithm switching.
//!
//! Provides runtime-selectable scan matchers through the [`DynMatcher`] enum,
//! allowing applications to switch between matching algorithms based on configuration.
//!
//! # Matcher Types
//!
//! - **Icp**: Point-to-Point ICP (fast, good for small initial errors)
//! - **P2l**: Point-to-Line ICP (better for structured environments)
//! - **Correlative**: Exhaustive search (handles large initial errors)
//! - **MultiRes**: Multi-resolution correlative (efficient + robust)
//! - **HybridIcp**: Correlative + P2P ICP (robust + accurate)
//! - **HybridP2l**: Correlative + P2L ICP (recommended for production)
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::algorithms::matching::{DynMatcher, MatcherType, ScanMatcher};
//!
//! // Create matcher with runtime-selected type
//! let mut matcher = DynMatcher::new(MatcherType::HybridP2l);
//!
//! // Use like any other ScanMatcher
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

/// Available scan matching algorithm types.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, ValueEnum, Serialize, Deserialize)]
pub enum MatcherType {
    /// Point-to-Point ICP.
    ///
    /// Classic Iterative Closest Point algorithm.
    /// Fast but requires good initial guess.
    Icp,

    /// Point-to-Line ICP.
    ///
    /// Better accuracy for structured environments (walls, corridors).
    /// Slightly slower than P2P but more robust to sampling patterns.
    P2l,

    /// Correlative scan matcher.
    ///
    /// Exhaustive search over translation and rotation.
    /// Handles large initial errors (30°+) but slower.
    Correlative,

    /// Multi-resolution correlative matcher.
    ///
    /// Hierarchical search from coarse to fine resolution.
    /// Good balance of speed and robustness.
    MultiRes,

    /// Hybrid: Correlative + Point-to-Point ICP.
    ///
    /// Uses correlative for initial alignment, ICP for refinement.
    /// Robust to large errors with good final accuracy.
    HybridIcp,

    /// Hybrid: Correlative + Point-to-Line ICP.
    ///
    /// **Recommended for production SLAM.**
    /// Best combination of robustness and accuracy for indoor environments.
    HybridP2l,
}

impl std::fmt::Display for MatcherType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MatcherType::Icp => write!(f, "P2P ICP"),
            MatcherType::P2l => write!(f, "P2L ICP"),
            MatcherType::Correlative => write!(f, "Correlative"),
            MatcherType::MultiRes => write!(f, "Multi-Res"),
            MatcherType::HybridIcp => write!(f, "Hybrid ICP"),
            MatcherType::HybridP2l => write!(f, "Hybrid P2L"),
        }
    }
}

impl MatcherType {
    /// Get a short description of this matcher type.
    pub fn description(&self) -> &'static str {
        match self {
            MatcherType::Icp => "Point-to-Point ICP only",
            MatcherType::P2l => "Point-to-Line ICP only",
            MatcherType::Correlative => "Correlative matcher only",
            MatcherType::MultiRes => "Multi-resolution correlative",
            MatcherType::HybridIcp => "Correlative + P2P ICP",
            MatcherType::HybridP2l => "Correlative + P2L ICP (recommended)",
        }
    }

    /// Returns all available matcher types.
    pub fn all() -> &'static [MatcherType] {
        &[
            MatcherType::Icp,
            MatcherType::P2l,
            MatcherType::Correlative,
            MatcherType::MultiRes,
            MatcherType::HybridIcp,
            MatcherType::HybridP2l,
        ]
    }
}

/// Configuration for all matcher types.
///
/// Each field configures its respective matching algorithm.
/// Only the relevant config is used based on the selected [`MatcherType`].
#[derive(Debug, Clone, Default)]
pub struct DynMatcherConfig {
    /// Configuration for Point-to-Point ICP.
    pub icp: IcpConfig,

    /// Configuration for Point-to-Line ICP.
    pub p2l: PointToLineIcpConfig,

    /// Configuration for correlative matcher.
    pub correlative: CorrelativeConfig,

    /// Configuration for multi-resolution matcher.
    pub multi_res: MultiResolutionConfig,

    /// Configuration for hybrid ICP matcher.
    pub hybrid_icp: HybridIcpMatcherConfig,

    /// Configuration for hybrid P2L matcher.
    pub hybrid_p2l: HybridP2LMatcherConfig,
}

/// Runtime-selectable scan matcher implementation.
///
/// Wraps all available matching algorithms behind a unified interface,
/// enabling runtime algorithm selection based on configuration.
///
/// Implements the [`ScanMatcher`] trait for interoperability with
/// existing SLAM infrastructure.
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::algorithms::matching::{DynMatcher, DynMatcherConfig, MatcherType, ScanMatcher};
///
/// // Create with default config
/// let mut matcher = DynMatcher::new(MatcherType::HybridP2l);
///
/// // Or with custom config
/// let config = DynMatcherConfig::default();
/// let mut matcher = DynMatcher::new_with_config(MatcherType::Icp, config);
///
/// // Use like any ScanMatcher
/// let result = matcher.match_scans(&source, &target, &guess);
/// ```
#[derive(Debug)]
pub enum DynMatcher {
    /// Point-to-Point ICP matcher.
    Icp(PointToPointIcp),
    /// Point-to-Line ICP matcher.
    P2l(PointToLineIcp),
    /// Correlative scan matcher.
    Correlative(CorrelativeMatcher),
    /// Multi-resolution correlative matcher.
    MultiRes(MultiResolutionMatcher),
    /// Hybrid Correlative + P2P ICP matcher.
    HybridIcp(HybridIcpMatcher),
    /// Hybrid Correlative + P2L ICP matcher.
    HybridP2l(HybridP2LMatcher),
}

impl DynMatcher {
    /// Create a new dynamic matcher with default configuration.
    ///
    /// # Arguments
    ///
    /// * `matcher_type` - The type of matching algorithm to use
    pub fn new(matcher_type: MatcherType) -> Self {
        Self::new_with_config(matcher_type, DynMatcherConfig::default())
    }

    /// Create a new dynamic matcher with custom configuration.
    ///
    /// # Arguments
    ///
    /// * `matcher_type` - The type of matching algorithm to use
    /// * `config` - Configuration containing settings for all algorithm types
    pub fn new_with_config(matcher_type: MatcherType, config: DynMatcherConfig) -> Self {
        match matcher_type {
            MatcherType::Icp => DynMatcher::Icp(PointToPointIcp::new(config.icp)),
            MatcherType::P2l => DynMatcher::P2l(PointToLineIcp::new(config.p2l)),
            MatcherType::Correlative => {
                DynMatcher::Correlative(CorrelativeMatcher::new(config.correlative))
            }
            MatcherType::MultiRes => {
                DynMatcher::MultiRes(MultiResolutionMatcher::new(config.multi_res))
            }
            MatcherType::HybridIcp => {
                DynMatcher::HybridIcp(HybridIcpMatcher::from_config(config.hybrid_icp))
            }
            MatcherType::HybridP2l => {
                DynMatcher::HybridP2l(HybridP2LMatcher::from_config(config.hybrid_p2l))
            }
        }
    }

    /// Get the matcher type.
    pub fn matcher_type(&self) -> MatcherType {
        match self {
            DynMatcher::Icp(_) => MatcherType::Icp,
            DynMatcher::P2l(_) => MatcherType::P2l,
            DynMatcher::Correlative(_) => MatcherType::Correlative,
            DynMatcher::MultiRes(_) => MatcherType::MultiRes,
            DynMatcher::HybridIcp(_) => MatcherType::HybridIcp,
            DynMatcher::HybridP2l(_) => MatcherType::HybridP2l,
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
    use crate::core::types::Point2D;
    use approx::assert_relative_eq;

    fn create_room(n: usize, width: f32, height: f32) -> PointCloud2D {
        let mut cloud = PointCloud2D::new();
        let points_per_wall = n / 4;

        // Bottom wall
        for i in 0..points_per_wall {
            let x = (i as f32 / points_per_wall as f32) * width;
            cloud.push(Point2D::new(x, 0.0));
        }
        // Right wall
        for i in 0..points_per_wall {
            let y = (i as f32 / points_per_wall as f32) * height;
            cloud.push(Point2D::new(width, y));
        }
        // Top wall
        for i in 0..points_per_wall {
            let x = width - (i as f32 / points_per_wall as f32) * width;
            cloud.push(Point2D::new(x, height));
        }
        // Left wall
        for i in 0..points_per_wall {
            let y = height - (i as f32 / points_per_wall as f32) * height;
            cloud.push(Point2D::new(0.0, y));
        }
        cloud
    }

    #[test]
    fn test_matcher_type_display() {
        assert_eq!(format!("{}", MatcherType::Icp), "P2P ICP");
        assert_eq!(format!("{}", MatcherType::P2l), "P2L ICP");
        assert_eq!(format!("{}", MatcherType::Correlative), "Correlative");
        assert_eq!(format!("{}", MatcherType::MultiRes), "Multi-Res");
        assert_eq!(format!("{}", MatcherType::HybridIcp), "Hybrid ICP");
        assert_eq!(format!("{}", MatcherType::HybridP2l), "Hybrid P2L");
    }

    #[test]
    fn test_matcher_type_all() {
        let all = MatcherType::all();
        assert_eq!(all.len(), 6);
        assert!(all.contains(&MatcherType::HybridP2l));
    }

    #[test]
    fn test_matcher_type_description() {
        assert!(!MatcherType::Icp.description().is_empty());
        assert!(MatcherType::HybridP2l.description().contains("recommended"));
    }

    #[test]
    fn test_dyn_matcher_icp() {
        let mut matcher = DynMatcher::new(MatcherType::Icp);
        assert_eq!(matcher.matcher_type(), MatcherType::Icp);

        // Use room shape - lines are degenerate for ICP (can slide along)
        let source = create_room(100, 2.0, 1.5);
        // Small transform that ICP can handle without initial guess
        let transform = Pose2D::new(0.02, 0.01, 0.01);
        let target = source.transform(&transform);

        let result = matcher.match_scans(&source, &target, &Pose2D::identity());
        assert!(result.converged);
        // Just verify it found something reasonable
        assert!(result.transform.x.abs() < 0.1);
    }

    #[test]
    fn test_dyn_matcher_p2l() {
        let mut matcher = DynMatcher::new(MatcherType::P2l);
        assert_eq!(matcher.matcher_type(), MatcherType::P2l);

        let source = create_room(100, 2.0, 1.5);
        let transform = Pose2D::new(0.05, 0.02, 0.02);
        let target = source.transform(&transform);

        let result = matcher.match_scans(&source, &target, &Pose2D::identity());
        assert!(result.converged);
    }

    #[test]
    fn test_dyn_matcher_correlative() {
        let mut matcher = DynMatcher::new(MatcherType::Correlative);
        assert_eq!(matcher.matcher_type(), MatcherType::Correlative);

        let source = create_room(100, 2.0, 1.5);
        let transform = Pose2D::new(0.1, 0.05, 0.1);
        let target = source.transform(&transform);

        let result = matcher.match_scans(&source, &target, &Pose2D::identity());
        assert!(result.converged);
    }

    #[test]
    fn test_dyn_matcher_multi_res() {
        let mut matcher = DynMatcher::new(MatcherType::MultiRes);
        assert_eq!(matcher.matcher_type(), MatcherType::MultiRes);

        let source = create_room(100, 2.0, 1.5);
        let transform = Pose2D::new(0.05, 0.02, 0.05);
        let target = source.transform(&transform);

        let result = matcher.match_scans(&source, &target, &Pose2D::identity());
        assert!(result.converged);
    }

    #[test]
    fn test_dyn_matcher_hybrid_icp() {
        let mut matcher = DynMatcher::new(MatcherType::HybridIcp);
        assert_eq!(matcher.matcher_type(), MatcherType::HybridIcp);

        let source = create_room(100, 2.0, 1.5);
        let transform = Pose2D::new(0.1, 0.05, 0.1);
        let target = source.transform(&transform);

        let result = matcher.match_scans(&source, &target, &Pose2D::identity());
        assert!(result.converged);
    }

    #[test]
    fn test_dyn_matcher_hybrid_p2l() {
        let mut matcher = DynMatcher::new(MatcherType::HybridP2l);
        assert_eq!(matcher.matcher_type(), MatcherType::HybridP2l);

        let source = create_room(100, 2.0, 1.5);
        let transform = Pose2D::new(0.1, 0.05, 0.2);
        let target = source.transform(&transform);

        let result = matcher.match_scans(&source, &target, &Pose2D::identity());
        assert!(result.converged);
        assert_relative_eq!(result.transform.x, 0.1, epsilon = 0.05);
        assert_relative_eq!(result.transform.y, 0.05, epsilon = 0.05);
    }

    #[test]
    fn test_dyn_matcher_with_custom_config() {
        let mut config = DynMatcherConfig::default();
        config.icp.max_iterations = 50;

        let mut matcher = DynMatcher::new_with_config(MatcherType::Icp, config);
        assert_eq!(matcher.matcher_type(), MatcherType::Icp);

        let source = create_room(80, 2.0, 1.5);
        let result = matcher.match_scans(&source, &source, &Pose2D::identity());
        assert!(result.converged);
    }

    #[test]
    fn test_dyn_matcher_implements_scan_matcher() {
        // This test verifies that DynMatcher can be used wherever ScanMatcher is expected
        fn use_scan_matcher(matcher: &mut impl ScanMatcher) -> ScanMatchResult {
            let source = create_room(80, 2.0, 1.5);
            matcher.match_scans(&source, &source, &Pose2D::identity())
        }

        let mut matcher = DynMatcher::new(MatcherType::Icp);
        let result = use_scan_matcher(&mut matcher);
        assert!(result.converged);
    }
}
