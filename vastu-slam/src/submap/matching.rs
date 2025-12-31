//! Multi-submap scan matching.
//!
//! Provides scan matching against the active submap and overlapping finalized submaps.
//! This enables robust matching when the robot returns to previously mapped areas.

use crate::core::{LidarScan, Pose2D};
use crate::matching::{CorrelativeMatcher, CorrelativeMatcherConfig, ScanMatchResult};

use super::manager::SubmapManager;
use super::types::{Submap, SubmapId};

/// Result of matching against multiple submaps.
#[derive(Clone, Debug)]
pub struct MultiSubmapMatchResult {
    /// The best match result across all submaps.
    pub best_match: ScanMatchResult,

    /// The submap that produced the best match.
    pub matched_submap_id: SubmapId,

    /// The local pose within the matched submap.
    pub local_pose: Pose2D,

    /// Number of submaps that were searched.
    pub submaps_searched: usize,

    /// All match results (for debugging/visualization).
    pub all_results: Vec<(SubmapId, ScanMatchResult)>,
}

impl MultiSubmapMatchResult {
    /// Create a failed result.
    pub fn failed(prior_pose: Pose2D) -> Self {
        Self {
            best_match: ScanMatchResult::failed(prior_pose),
            matched_submap_id: SubmapId::new(0),
            local_pose: Pose2D::default(),
            submaps_searched: 0,
            all_results: Vec::new(),
        }
    }

    /// Did matching converge?
    pub fn converged(&self) -> bool {
        self.best_match.converged
    }

    /// Get the matched world pose.
    pub fn world_pose(&self) -> Pose2D {
        self.best_match.pose
    }

    /// Get the match score.
    pub fn score(&self) -> f32 {
        self.best_match.score
    }
}

/// Configuration for multi-submap matching.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MultiSubmapMatchConfig {
    /// Maximum distance to consider overlapping submaps.
    #[serde(default = "default_max_overlap_distance")]
    pub max_overlap_distance: f32,

    /// Include active submap in search.
    #[serde(default = "default_include_active")]
    pub include_active: bool,

    /// Include finalized submaps in search.
    #[serde(default = "default_include_finalized")]
    pub include_finalized: bool,

    /// Maximum number of submaps to search.
    #[serde(default = "default_max_submaps")]
    pub max_submaps: usize,

    /// Minimum score improvement to prefer a different submap.
    /// Prevents jumping between submaps with similar scores.
    #[serde(default = "default_score_hysteresis")]
    pub score_hysteresis: f32,
}

fn default_max_overlap_distance() -> f32 {
    3.0
}

fn default_include_active() -> bool {
    true
}

fn default_include_finalized() -> bool {
    true
}

fn default_max_submaps() -> usize {
    3
}

fn default_score_hysteresis() -> f32 {
    0.05
}

impl Default for MultiSubmapMatchConfig {
    fn default() -> Self {
        Self {
            max_overlap_distance: default_max_overlap_distance(),
            include_active: default_include_active(),
            include_finalized: default_include_finalized(),
            max_submaps: default_max_submaps(),
            score_hysteresis: default_score_hysteresis(),
        }
    }
}

/// Matcher that can search across multiple submaps.
pub struct MultiSubmapMatcher {
    /// Underlying correlative matcher.
    matcher: CorrelativeMatcher,

    /// Configuration.
    config: MultiSubmapMatchConfig,
}

impl MultiSubmapMatcher {
    /// Create a new multi-submap matcher.
    pub fn new(
        matcher_config: CorrelativeMatcherConfig,
        multi_config: MultiSubmapMatchConfig,
    ) -> Self {
        Self {
            matcher: CorrelativeMatcher::new(matcher_config),
            config: multi_config,
        }
    }

    /// Create with default configurations.
    pub fn with_defaults() -> Self {
        Self::new(
            CorrelativeMatcherConfig::default(),
            MultiSubmapMatchConfig::default(),
        )
    }

    /// Match a scan against the active submap only.
    ///
    /// This is the fast path for normal operation.
    pub fn match_active_only(
        &self,
        scan: &LidarScan,
        prior_world_pose: Pose2D,
        manager: &SubmapManager,
    ) -> MultiSubmapMatchResult {
        let Some(active) = manager.active_submap() else {
            return MultiSubmapMatchResult::failed(prior_world_pose);
        };

        let result = self.match_against_submap(scan, prior_world_pose, active);

        MultiSubmapMatchResult {
            best_match: result.clone(),
            matched_submap_id: active.id,
            local_pose: active.origin.inverse().compose(&result.pose),
            submaps_searched: 1,
            all_results: vec![(active.id, result)],
        }
    }

    /// Match a scan against multiple submaps.
    ///
    /// Searches the active submap and overlapping finalized submaps,
    /// returning the best match.
    pub fn match_multi(
        &self,
        scan: &LidarScan,
        prior_world_pose: Pose2D,
        manager: &SubmapManager,
    ) -> MultiSubmapMatchResult {
        let mut candidates: Vec<&Submap> = Vec::new();

        // Add active submap
        if self.config.include_active
            && let Some(active) = manager.active_submap()
        {
            candidates.push(active);
        }

        // Add overlapping finalized submaps
        if self.config.include_finalized {
            let finalized: Vec<_> = manager
                .submaps()
                .iter()
                .filter(|s| s.is_finalized())
                .filter(|s| {
                    s.origin.position().distance(&prior_world_pose.position())
                        <= self.config.max_overlap_distance
                })
                .collect();

            candidates.extend(finalized);
        }

        // Limit candidates
        if candidates.len() > self.config.max_submaps {
            // Sort by distance to prior pose, take closest
            candidates.sort_by(|a, b| {
                let dist_a = a.origin.position().distance(&prior_world_pose.position());
                let dist_b = b.origin.position().distance(&prior_world_pose.position());
                dist_a.partial_cmp(&dist_b).unwrap()
            });
            candidates.truncate(self.config.max_submaps);
        }

        if candidates.is_empty() {
            return MultiSubmapMatchResult::failed(prior_world_pose);
        }

        // Match against each candidate
        let mut all_results: Vec<(SubmapId, ScanMatchResult)> = Vec::new();
        let mut best_score = f32::NEG_INFINITY;
        let mut best_idx = 0;

        for (i, submap) in candidates.iter().enumerate() {
            let result = self.match_against_submap(scan, prior_world_pose, submap);
            all_results.push((submap.id, result.clone()));

            // Use hysteresis: prefer current best unless new is significantly better
            let adjusted_score = if i == 0 {
                result.score
            } else {
                result.score - self.config.score_hysteresis
            };

            if adjusted_score > best_score && result.converged {
                best_score = adjusted_score;
                best_idx = i;
            }
        }

        let (best_submap_id, best_match) = all_results[best_idx].clone();
        let best_submap = candidates[best_idx];

        MultiSubmapMatchResult {
            local_pose: best_submap.origin.inverse().compose(&best_match.pose),
            best_match,
            matched_submap_id: best_submap_id,
            submaps_searched: all_results.len(),
            all_results,
        }
    }

    /// Match a scan against a single submap.
    fn match_against_submap(
        &self,
        scan: &LidarScan,
        prior_world_pose: Pose2D,
        submap: &Submap,
    ) -> ScanMatchResult {
        // Convert world pose to local pose for matching
        let prior_local_pose = submap.origin.inverse().compose(&prior_world_pose);

        // Match against the submap's local grid
        let local_result = self
            .matcher
            .match_scan(scan, prior_local_pose, &submap.grid);

        // Convert result back to world frame
        let world_pose = submap.origin.compose(&local_result.pose);

        ScanMatchResult::new(
            world_pose,
            local_result.score,
            local_result.hits,
            local_result.total_points,
        )
    }
}

impl SubmapManager {
    /// Match a scan against the active submap.
    ///
    /// This is the fast path for normal SLAM operation.
    pub fn match_scan(
        &self,
        scan: &LidarScan,
        prior_world_pose: Pose2D,
        matcher_config: &CorrelativeMatcherConfig,
    ) -> MultiSubmapMatchResult {
        let matcher = MultiSubmapMatcher::new(
            matcher_config.clone(),
            MultiSubmapMatchConfig {
                include_finalized: false,
                ..Default::default()
            },
        );
        matcher.match_active_only(scan, prior_world_pose, self)
    }

    /// Match a scan against multiple submaps (active + overlapping).
    ///
    /// Use this when the robot may have returned to a previously mapped area.
    pub fn match_scan_multi(
        &self,
        scan: &LidarScan,
        prior_world_pose: Pose2D,
        matcher_config: &CorrelativeMatcherConfig,
        multi_config: &MultiSubmapMatchConfig,
    ) -> MultiSubmapMatchResult {
        let matcher = MultiSubmapMatcher::new(matcher_config.clone(), multi_config.clone());
        matcher.match_multi(scan, prior_world_pose, self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::MapConfig;
    use crate::submap::SubmapConfig;

    fn make_test_scan() -> LidarScan {
        let angles: Vec<f32> = (0..36).map(|i| (i as f32) * 0.1745).collect();
        let ranges: Vec<f32> = angles.iter().map(|_| 2.0).collect();
        LidarScan::new(ranges, angles, 0.15, 8.0)
    }

    #[test]
    fn test_match_active_only() {
        let config = SubmapConfig::default();
        let map_config = MapConfig::default();
        let mut manager = SubmapManager::new(config, &map_config);

        let scan = make_test_scan();
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        // Insert a scan to create a submap
        manager.insert_scan(&scan, pose, 0);

        // Match against active
        let matcher_config = CorrelativeMatcherConfig::fast();
        let result = manager.match_scan(&scan, pose, &matcher_config);

        assert_eq!(result.submaps_searched, 1);
        assert!(result.world_pose().x.is_finite());
    }

    #[test]
    fn test_match_multi() {
        let mut submap_config = SubmapConfig::default();
        submap_config.scans_per_submap = 2;
        submap_config.overlap_scans = 1;

        let map_config = MapConfig::default();
        let mut manager = SubmapManager::new(submap_config, &map_config);

        let scan = make_test_scan();

        // Insert several scans to create multiple submaps
        for i in 0..6 {
            let pose = Pose2D::new(i as f32 * 0.2, 0.0, 0.0);
            manager.insert_scan(&scan, pose, i as u64);
        }

        // Match with multi-submap search
        let matcher_config = CorrelativeMatcherConfig::fast();
        let multi_config = MultiSubmapMatchConfig {
            max_overlap_distance: 5.0,
            ..Default::default()
        };

        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let result = manager.match_scan_multi(&scan, pose, &matcher_config, &multi_config);

        // Should have searched multiple submaps
        assert!(result.submaps_searched >= 1);
        assert!(result.world_pose().x.is_finite());
    }
}
