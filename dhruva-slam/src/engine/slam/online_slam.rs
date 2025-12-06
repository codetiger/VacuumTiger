//! Online SLAM implementation.
//!
//! Combines scan matching, submap management, and keyframe selection into
//! a unified real-time SLAM system.

use std::time::Instant;

use crate::algorithms::mapping::{OccupancyGrid, OccupancyGridConfig};
use crate::algorithms::matching::{
    HybridP2LMatcher, HybridP2LMatcherConfig, ScanMatchResult, ScanMatcher,
};
use crate::core::types::{Covariance2D, PointCloud2D, Pose2D};
use crate::io::streaming::{
    LoopClosureStats, MappingStats, ScanMatchStats, SlamDiagnosticsMessage, TimingBreakdown,
};

use super::SlamEngine;
use super::keyframe::{KeyframeManager, KeyframeManagerConfig};
use super::submap::{SubmapManager, SubmapManagerConfig};

/// SLAM operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlamMode {
    /// Building a new map.
    Mapping,
    /// Localizing in a known map.
    Localization,
    /// Idle (not processing).
    Idle,
}

/// SLAM status information.
#[derive(Debug, Clone)]
pub struct SlamStatus {
    /// Current operating mode.
    pub mode: SlamMode,

    /// Number of scans processed.
    pub num_scans: u64,

    /// Number of keyframes created.
    pub num_keyframes: usize,

    /// Number of submaps.
    pub num_submaps: usize,

    /// Number of finished submaps.
    pub num_finished_submaps: usize,

    /// Last scan match quality score (0-1).
    pub last_match_score: f32,

    /// Whether the robot is considered "lost" (poor localization).
    pub is_lost: bool,

    /// Memory usage in bytes.
    pub memory_usage: usize,
}

impl Default for SlamStatus {
    fn default() -> Self {
        Self {
            mode: SlamMode::Idle,
            num_scans: 0,
            num_keyframes: 0,
            num_submaps: 0,
            num_finished_submaps: 0,
            last_match_score: 0.0,
            is_lost: false,
            memory_usage: 0,
        }
    }
}

/// Result of processing a scan.
#[derive(Debug, Clone)]
pub struct SlamResult {
    /// Corrected robot pose after scan matching.
    pub pose: Pose2D,

    /// Pose covariance estimate.
    pub covariance: Covariance2D,

    /// Whether a new keyframe was created.
    pub keyframe_created: bool,

    /// Whether a new submap was started.
    pub new_submap: bool,

    /// Scan match quality (0-1).
    pub match_score: f32,

    /// Whether the pose was successfully refined.
    pub pose_refined: bool,
}

impl Default for SlamResult {
    fn default() -> Self {
        Self {
            pose: Pose2D::identity(),
            covariance: Covariance2D::diagonal(1.0, 1.0, 0.5),
            keyframe_created: false,
            new_submap: false,
            match_score: 0.0,
            pose_refined: false,
        }
    }
}

/// Configuration for online SLAM.
#[derive(Debug, Clone)]
pub struct OnlineSlamConfig {
    /// Keyframe manager configuration.
    pub keyframe: KeyframeManagerConfig,

    /// Submap manager configuration.
    pub submap: SubmapManagerConfig,

    /// Scan-to-scan matcher configuration (Correlative + P2L ICP).
    /// Same as slam_benchmark for consistent results.
    pub matcher: HybridP2LMatcherConfig,

    /// Submap matcher configuration (Correlative + P2L ICP for scan-to-submap).
    pub submap_matcher: HybridP2LMatcherConfig,

    /// Global map configuration.
    pub global_map: OccupancyGridConfig,

    /// Minimum match score to accept pose refinement.
    pub min_match_score: f32,

    /// Score threshold below which robot is considered "lost".
    pub lost_threshold: f32,

    /// Whether to use scan-to-submap matching (vs scan-to-scan).
    pub use_submap_matching: bool,

    /// Minimum points in scan to process.
    pub min_scan_points: usize,

    /// Encoder weight for initial guess (0.0 = identity, 1.0 = full odometry).
    /// Higher values trust encoder more for initial guess.
    pub encoder_weight: f32,
}

impl Default for OnlineSlamConfig {
    fn default() -> Self {
        Self {
            keyframe: KeyframeManagerConfig::default(),
            submap: SubmapManagerConfig::default(),
            // Use HybridP2L (same as slam_benchmark) for consistent results
            matcher: HybridP2LMatcherConfig::default(),
            submap_matcher: HybridP2LMatcherConfig::default(),
            global_map: OccupancyGridConfig::default(),
            min_match_score: 0.3,
            lost_threshold: 0.1,
            // Use scan-to-scan matching by default - submap matching has semantic
            // issues where match_to_submap returns absolute pose but apply_scan_match
            // expects relative transform
            use_submap_matching: false,
            min_scan_points: 50,
            encoder_weight: 0.8, // Trust encoder 80% for initial guess
        }
    }
}

/// Online SLAM engine.
///
/// Provides real-time simultaneous localization and mapping by combining:
/// - Scan matching for pose refinement
/// - Submap-based local mapping
/// - Keyframe selection for loop closure
pub struct OnlineSlam {
    config: OnlineSlamConfig,

    /// Current robot pose estimate.
    current_pose: Pose2D,

    /// Pose covariance.
    covariance: Covariance2D,

    /// Keyframe manager.
    keyframes: KeyframeManager,

    /// Submap manager.
    submaps: SubmapManager,

    /// Scan-to-scan matcher (Correlative + Point-to-Line ICP).
    /// Same as slam_benchmark for consistent results.
    matcher: HybridP2LMatcher,

    /// Scan-to-submap matcher (Correlative + Point-to-Line ICP for rotation handling).
    submap_matcher: HybridP2LMatcher,

    /// Global map (built on demand).
    global_map: Option<OccupancyGrid>,

    /// Double-buffer for previous scan to avoid cloning.
    /// scan_buffers[current_scan_idx] is the current scan,
    /// scan_buffers[1 - current_scan_idx] is the previous scan.
    scan_buffers: [PointCloud2D; 2],

    /// Index of the current scan buffer (0 or 1).
    current_scan_idx: usize,

    /// Whether we have a valid previous scan.
    has_previous_scan: bool,

    /// Operating mode.
    mode: SlamMode,

    /// Total scans processed.
    num_scans: u64,

    /// Is the robot currently "lost"?
    is_lost: bool,

    /// Last match score.
    last_match_score: f32,

    /// Number of finished submaps when global map was last built.
    /// Used to avoid unnecessary map rebuilds.
    last_finished_submaps: usize,

    // ========================================================================
    // Diagnostics tracking
    // ========================================================================
    /// Last timing breakdown.
    last_timing: TimingBreakdown,

    /// Last scan match stats.
    last_scan_match_stats: ScanMatchStats,

    /// Running average of total cycle time.
    avg_total_time_us: f32,

    /// Number of samples for averaging.
    timing_sample_count: u64,
}

impl OnlineSlam {
    /// Typical scan size for pre-allocation (360° lidar at 1° resolution).
    const TYPICAL_SCAN_POINTS: usize = 360;

    /// Create a new online SLAM instance.
    pub fn new(config: OnlineSlamConfig) -> Self {
        // Use Correlative + Point-to-Line ICP for scan-to-scan matching:
        // - Same as slam_benchmark for consistent results
        // - Correlative handles large initial errors (especially rotation)
        // - Point-to-Line ICP provides sub-cm accuracy for structured environments
        let matcher = HybridP2LMatcher::new(config.matcher.clone());

        // Use Correlative + Point-to-Line ICP for submap matching:
        // - Correlative handles large initial errors (especially rotation)
        // - Point-to-Line ICP provides sub-cm accuracy for structured environments
        let submap_matcher = HybridP2LMatcher::new(config.submap_matcher.clone());

        // Pre-allocate scan buffers to avoid reallocation during hot path
        Self {
            keyframes: KeyframeManager::new(config.keyframe.clone()),
            submaps: SubmapManager::new(config.submap.clone()),
            matcher,
            submap_matcher,
            global_map: None,
            scan_buffers: [
                PointCloud2D::with_capacity(Self::TYPICAL_SCAN_POINTS),
                PointCloud2D::with_capacity(Self::TYPICAL_SCAN_POINTS),
            ],
            current_scan_idx: 0,
            has_previous_scan: false,
            current_pose: Pose2D::identity(),
            covariance: Covariance2D::diagonal(0.01, 0.01, 0.01),
            mode: SlamMode::Mapping,
            num_scans: 0,
            is_lost: false,
            last_match_score: 1.0,
            last_finished_submaps: 0,
            config,
            // Diagnostics fields
            last_timing: TimingBreakdown::default(),
            last_scan_match_stats: ScanMatchStats::default(),
            avg_total_time_us: 0.0,
            timing_sample_count: 0,
        }
    }

    /// Get the current mode.
    pub fn mode(&self) -> SlamMode {
        self.mode
    }

    /// Set the operating mode.
    pub fn set_mode(&mut self, mode: SlamMode) {
        self.mode = mode;
    }

    /// Get keyframe manager.
    pub fn keyframes(&self) -> &KeyframeManager {
        &self.keyframes
    }

    /// Get submap manager.
    pub fn submaps(&self) -> &SubmapManager {
        &self.submaps
    }

    /// Build and return the global map.
    ///
    /// This merges all submaps into a single occupancy grid.
    pub fn global_map(&mut self) -> &OccupancyGrid {
        if self.global_map.is_none() || self.needs_map_rebuild() {
            self.rebuild_global_map();
        }
        self.global_map.as_ref().unwrap()
    }

    /// Check if global map needs rebuilding.
    fn needs_map_rebuild(&self) -> bool {
        // Only rebuild if we have new finished submaps since last build
        let current_finished = self.submaps.finished_submaps().count();
        current_finished > self.last_finished_submaps
    }

    /// Rebuild the global map from submaps.
    fn rebuild_global_map(&mut self) {
        self.last_finished_submaps = self.submaps.finished_submaps().count();
        self.global_map = Some(self.submaps.global_map(&self.config.global_map));
    }

    /// Match scan to current submap.
    fn match_to_submap(
        &mut self,
        scan: &PointCloud2D,
        initial_pose: &Pose2D,
    ) -> Option<ScanMatchResult> {
        // Get submap origin first (immutable borrow)
        let submap = self.submaps.active_submap()?;
        let origin = submap.origin;

        // Convert scan to submap frame
        let local_pose = origin.inverse().compose(initial_pose);

        // Get cached pointcloud from submap (mutable borrow)
        let submap_mut = self.submaps.active_submap_mut()?;
        let target = submap_mut.as_pointcloud();

        if target.len() < self.config.min_scan_points {
            return None;
        }

        let result = self.submap_matcher.match_scans(scan, target, &local_pose);

        if result.converged {
            // Convert result back to global frame
            let global_pose = origin.compose(&result.transform);
            Some(ScanMatchResult {
                transform: global_pose,
                ..result
            })
        } else {
            None
        }
    }

    /// Match scan to previous scan (scan-to-scan matching).
    ///
    /// The initial guess for scan matching should be the transform from source to target.
    /// Since source=current scan and target=previous scan, and the robot moved by odom_delta,
    /// the initial guess should be odom_delta.inverse() (to undo the robot motion).
    fn match_to_previous_scan(
        &mut self,
        scan: &PointCloud2D,
        odom_delta: &Pose2D,
    ) -> Option<ScanMatchResult> {
        if !self.has_previous_scan {
            return None;
        }

        let prev_idx = 1 - self.current_scan_idx;
        let prev = &self.scan_buffers[prev_idx];

        if prev.len() < self.config.min_scan_points {
            return None;
        }

        // Initial guess: inverse of odometry delta (transform from current scan to previous scan)
        let initial_guess = odom_delta.inverse();
        let result = self.matcher.match_scans(scan, prev, &initial_guess);

        if result.converged && result.score >= self.config.min_match_score {
            Some(result)
        } else {
            None
        }
    }

    /// Process pose update from scan matching.
    ///
    /// The scan match result contains a transform from current scan to previous scan.
    /// To compute the corrected pose:
    /// - previous_pose is where the previous scan was taken
    /// - match_result.transform maps current scan → previous scan frame
    /// - So: corrected_pose = previous_pose.compose(match_result.transform.inverse())
    ///
    /// Since we don't store previous_pose explicitly, we compute it as:
    ///   previous_pose = current_pose (before the odom_delta was applied)
    ///
    /// Uses a threshold-based approach to blend between scan match and odometry
    /// to avoid erratic pose jumps from small score variations.
    fn apply_scan_match(&mut self, match_result: &ScanMatchResult, odom_delta: &Pose2D) -> Pose2D {
        let odom_pose = self.current_pose.compose(odom_delta);

        // Compute scan-match-corrected pose:
        // match_result.transform is the transform from current scan to previous scan
        // The inverse gives us the relative motion from previous scan to current scan
        // So: corrected_pose = current_pose.compose(match_result.transform.inverse())
        let scan_match_pose = self.current_pose.compose(&match_result.transform.inverse());

        if match_result.score >= 0.6 {
            // High confidence: trust scan match fully
            scan_match_pose
        } else if match_result.score >= 0.3 {
            // Medium confidence: blend with bias toward scan match
            // alpha goes from 0.0 (at score=0.3) to 1.0 (at score=0.6)
            let alpha = (match_result.score - 0.3) / 0.3;
            // blend_factor goes from 0.5 to 1.0
            let blend_factor = 0.5 + 0.5 * alpha;

            let x = odom_pose.x * (1.0 - blend_factor) + scan_match_pose.x * blend_factor;
            let y = odom_pose.y * (1.0 - blend_factor) + scan_match_pose.y * blend_factor;
            let theta =
                crate::core::math::angle_lerp(odom_pose.theta, scan_match_pose.theta, blend_factor);
            Pose2D::new(x, y, theta)
        } else {
            // Low confidence: use odometry only
            odom_pose
        }
    }

    /// Get the configuration.
    pub fn config(&self) -> &OnlineSlamConfig {
        &self.config
    }

    /// Get current diagnostics for visualization/debugging.
    ///
    /// Returns a snapshot of timing breakdowns, scan matching stats,
    /// mapping stats, and loop closure stats.
    pub fn diagnostics(&self, timestamp_us: u64) -> SlamDiagnosticsMessage {
        // Get mapping stats from submap manager
        let mapping = MappingStats {
            cells_updated: 0, // TODO: Track this in submaps
            map_size_bytes: self.submaps.memory_usage(),
            occupied_cells: 0, // Could compute from global map
            free_cells: 0,
            active_submap_id: self.submaps.active_submap().map(|s| s.id),
            num_submaps: self.submaps.len(),
        };

        // Loop closure stats (placeholder for now - will be populated when loop detector is added)
        let loop_closure = LoopClosureStats {
            candidates_evaluated: 0,
            closures_accepted: 0,
            last_closure_score: 0.0,
            pose_graph_nodes: self.keyframes.len(),
            pose_graph_edges: 0, // Will be populated when pose graph is integrated
        };

        SlamDiagnosticsMessage {
            timestamp_us,
            timing: self.last_timing.clone(),
            scan_match: self.last_scan_match_stats.clone(),
            particle_filter: None, // Will be populated when MCL is active
            mapping,
            loop_closure,
        }
    }
}

impl SlamEngine for OnlineSlam {
    fn process_scan(
        &mut self,
        scan: &PointCloud2D,
        odom_delta: &Pose2D,
        timestamp_us: u64,
    ) -> SlamResult {
        let cycle_start = Instant::now();
        let mut result = SlamResult::default();

        // Initialize timing breakdown
        let mut timing = TimingBreakdown::default();

        if self.mode == SlamMode::Idle {
            return result;
        }

        // Skip if scan is too sparse
        if scan.len() < self.config.min_scan_points {
            log::debug!("Skipping sparse scan with {} points", scan.len());
            result.pose = self.current_pose.compose(odom_delta);
            return result;
        }

        self.num_scans += 1;

        // Predict pose from odometry
        let predicted_pose = self.current_pose.compose(odom_delta);

        // =========== Scan Matching (timed) ===========
        let match_start = Instant::now();
        let match_result =
            if self.config.use_submap_matching && self.submaps.active_submap().is_some() {
                self.match_to_submap(scan, &predicted_pose)
            } else {
                self.match_to_previous_scan(scan, odom_delta)
            };
        timing.scan_matching_us = match_start.elapsed().as_micros() as u64;

        // Update scan match stats for diagnostics
        // Use static strings to avoid allocation in hot path
        const METHOD_SUBMAP_P2L: &str = "submap_p2l";
        const METHOD_MULTI_RES: &str = "multi_res_icp";
        const METHOD_NONE: &str = "none";

        if let Some(ref mr) = match_result {
            self.last_scan_match_stats = ScanMatchStats {
                method: if self.config.use_submap_matching {
                    METHOD_SUBMAP_P2L
                } else {
                    METHOD_MULTI_RES
                }
                .into(),
                iterations: mr.iterations,
                score: mr.score,
                mse: mr.mse,
                converged: mr.converged,
                correspondences: 0, // TODO: Track this in matcher
                inlier_ratio: 0.0,  // TODO: Track this in matcher
            };
        } else {
            self.last_scan_match_stats = ScanMatchStats {
                method: METHOD_NONE.into(),
                ..Default::default()
            };
        }

        // Apply match result or use odometry
        let refined_pose = if let Some(ref mr) = match_result {
            result.pose_refined = true;
            result.match_score = mr.score;
            self.last_match_score = mr.score;
            self.is_lost = mr.score < self.config.lost_threshold;

            if mr.score >= self.config.min_match_score {
                self.apply_scan_match(mr, odom_delta)
            } else {
                predicted_pose
            }
        } else {
            // No match, use odometry only
            self.last_match_score = 0.0;
            predicted_pose
        };

        self.current_pose = refined_pose;
        result.pose = refined_pose;

        // Update covariance (simplified)
        let position_var = 0.01 + (1.0 - result.match_score) * 0.1;
        let theta_var = 0.005 + (1.0 - result.match_score) * 0.05;
        self.covariance = Covariance2D::diagonal(position_var, position_var, theta_var);
        result.covariance = self.covariance;

        // =========== Map Update (timed) ===========
        let map_start = Instant::now();
        if self.mode == SlamMode::Mapping {
            let old_submap_count = self.submaps.len();
            let submap_id = self.submaps.process_scan(scan, &refined_pose, timestamp_us);
            result.new_submap = self.submaps.len() > old_submap_count;

            // =========== Keyframe Check (timed) ===========
            let keyframe_start = Instant::now();
            if self
                .keyframes
                .should_create_keyframe(&refined_pose, timestamp_us)
            {
                self.keyframes
                    .create_keyframe(refined_pose, scan.clone(), timestamp_us, submap_id);
                self.submaps
                    .add_keyframe_to_active(self.keyframes.len() as u64 - 1);
                result.keyframe_created = true;
            }
            timing.keyframe_check_us = keyframe_start.elapsed().as_micros() as u64;

            // Invalidate cached global map
            if result.keyframe_created || result.new_submap {
                self.global_map = None;
            }
        }
        timing.map_update_us = map_start.elapsed().as_micros() as u64;

        // Store scan for next iteration using double-buffer swap
        // Copy into the current buffer slot, then swap indices
        // Reserve capacity first to avoid reallocation during extend
        let buffer = &mut self.scan_buffers[self.current_scan_idx];
        buffer.xs.clear();
        buffer.ys.clear();
        if buffer.xs.capacity() < scan.len() {
            buffer.xs.reserve(scan.len() - buffer.xs.capacity());
            buffer.ys.reserve(scan.len() - buffer.ys.capacity());
        }
        buffer.xs.extend_from_slice(&scan.xs);
        buffer.ys.extend_from_slice(&scan.ys);
        buffer.intensities = scan.intensities.clone();
        self.current_scan_idx = 1 - self.current_scan_idx;
        self.has_previous_scan = true;

        // =========== Finalize Timing ===========
        timing.total_us = cycle_start.elapsed().as_micros() as u64;

        // Update running average
        self.timing_sample_count += 1;
        let alpha = 0.1; // Exponential moving average factor
        self.avg_total_time_us = if self.timing_sample_count == 1 {
            timing.total_us as f32
        } else {
            self.avg_total_time_us * (1.0 - alpha) + timing.total_us as f32 * alpha
        };
        timing.avg_total_us = self.avg_total_time_us;

        // Store timing for diagnostics query
        self.last_timing = timing;

        result
    }

    fn current_pose(&self) -> Pose2D {
        self.current_pose
    }

    fn map(&self) -> &OccupancyGrid {
        // Return empty map if global map not built
        // (caller should use global_map() method instead)
        static EMPTY_MAP: std::sync::OnceLock<OccupancyGrid> = std::sync::OnceLock::new();
        self.global_map.as_ref().unwrap_or_else(|| {
            EMPTY_MAP.get_or_init(|| OccupancyGrid::new(OccupancyGridConfig::default()))
        })
    }

    fn status(&self) -> SlamStatus {
        SlamStatus {
            mode: self.mode,
            num_scans: self.num_scans,
            num_keyframes: self.keyframes.len(),
            num_submaps: self.submaps.len(),
            num_finished_submaps: self.submaps.finished_submaps().count(),
            last_match_score: self.last_match_score,
            is_lost: self.is_lost,
            memory_usage: self.submaps.memory_usage(),
        }
    }

    fn reset(&mut self) {
        self.current_pose = Pose2D::identity();
        self.covariance = Covariance2D::diagonal(0.01, 0.01, 0.01);
        self.keyframes.clear();
        self.submaps.clear();
        self.global_map = None;
        self.scan_buffers[0].clear();
        self.scan_buffers[1].clear();
        self.current_scan_idx = 0;
        self.has_previous_scan = false;
        self.num_scans = 0;
        self.is_lost = false;
        self.last_match_score = 1.0;
        self.last_finished_submaps = 0;
        // Reset diagnostics
        self.last_timing = TimingBreakdown::default();
        self.last_scan_match_stats = ScanMatchStats::default();
        self.avg_total_time_us = 0.0;
        self.timing_sample_count = 0;
    }

    fn reset_to(&mut self, pose: Pose2D) {
        self.reset();
        self.current_pose = pose;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;

    fn create_test_scan() -> PointCloud2D {
        let mut scan = PointCloud2D::new();
        // Create a wall-like pattern with tiny noise to avoid k-d tree bucket issues
        for i in 0..100 {
            let x = i as f32 * 0.05;
            let y_noise = (i as f32) * 0.0001;
            scan.push(Point2D::new(x, 2.0 + y_noise)); // Wall at y≈2
        }
        // Add side walls with tiny noise
        for i in 0..40 {
            let y = i as f32 * 0.05;
            let x_noise = (i as f32) * 0.0001;
            scan.push(Point2D::new(0.0 + x_noise, y));
            scan.push(Point2D::new(5.0 + x_noise, y));
        }
        scan
    }

    #[test]
    fn test_online_slam_creation() {
        let config = OnlineSlamConfig::default();
        let slam = OnlineSlam::new(config);

        assert_eq!(slam.mode(), SlamMode::Mapping);
        assert_eq!(slam.current_pose().x, 0.0);
    }

    #[test]
    fn test_online_slam_process_scan() {
        let config = OnlineSlamConfig::default();
        let mut slam = OnlineSlam::new(config);

        let scan = create_test_scan();
        let odom_delta = Pose2D::new(0.1, 0.0, 0.0);

        let result = slam.process_scan(&scan, &odom_delta, 0);

        assert!(result.pose.x > 0.0);
        assert_eq!(slam.status().num_scans, 1);
    }

    #[test]
    fn test_online_slam_creates_submaps() {
        let config = OnlineSlamConfig {
            submap: SubmapManagerConfig {
                scans_per_submap: 5,
                overlap_scans: 0,
                ..Default::default()
            },
            ..Default::default()
        };
        let mut slam = OnlineSlam::new(config);

        let scan = create_test_scan();

        // Process enough scans to create multiple submaps
        for i in 0..12 {
            let odom_delta = Pose2D::new(0.1, 0.0, 0.0);
            slam.process_scan(&scan, &odom_delta, i * 1000);
        }

        assert!(slam.submaps().len() >= 2);
    }

    /// Create a transformed test scan at a given offset
    fn create_test_scan_at(offset_x: f32, offset_y: f32) -> PointCloud2D {
        let mut scan = PointCloud2D::new();
        // Create a wall-like pattern with tiny noise to avoid k-d tree bucket issues
        for i in 0..100 {
            let x = i as f32 * 0.05 + offset_x;
            let y_noise = (i as f32) * 0.0001;
            scan.push(Point2D::new(x, 2.0 + offset_y + y_noise));
        }
        // Add side walls with tiny noise
        for i in 0..40 {
            let y = i as f32 * 0.05 + offset_y;
            let x_noise = (i as f32) * 0.0001;
            scan.push(Point2D::new(offset_x + x_noise, y));
            scan.push(Point2D::new(5.0 + offset_x + x_noise, y));
        }
        scan
    }

    #[test]
    fn test_online_slam_creates_keyframes() {
        // This test verifies keyframe creation based on motion thresholds.
        // We use scans at different offsets to simulate actual robot motion.
        let config = OnlineSlamConfig {
            keyframe: KeyframeManagerConfig {
                min_translation: 0.3,
                min_rotation: 0.3,
                min_interval_us: 0,
                ..Default::default()
            },
            ..Default::default()
        };
        let mut slam = OnlineSlam::new(config);

        // First scan at origin
        let scan1 = create_test_scan_at(0.0, 0.0);
        slam.process_scan(&scan1, &Pose2D::identity(), 0);
        assert_eq!(slam.keyframes().len(), 1);

        // Small motion with scan at same position - no new keyframe
        let scan2 = create_test_scan_at(0.1, 0.0);
        slam.process_scan(&scan2, &Pose2D::new(0.1, 0.0, 0.0), 1000);
        assert_eq!(slam.keyframes().len(), 1);

        // Large motion with scan at new position - new keyframe
        let scan3 = create_test_scan_at(0.5, 0.0);
        slam.process_scan(&scan3, &Pose2D::new(0.4, 0.0, 0.0), 2000);
        assert!(
            slam.keyframes().len() >= 2,
            "Expected at least 2 keyframes, got {}",
            slam.keyframes().len()
        );
    }

    #[test]
    fn test_online_slam_mode_switching() {
        let config = OnlineSlamConfig::default();
        let mut slam = OnlineSlam::new(config);

        assert_eq!(slam.mode(), SlamMode::Mapping);

        slam.set_mode(SlamMode::Localization);
        assert_eq!(slam.mode(), SlamMode::Localization);

        slam.set_mode(SlamMode::Idle);
        assert_eq!(slam.mode(), SlamMode::Idle);
    }

    #[test]
    fn test_online_slam_reset() {
        let config = OnlineSlamConfig::default();
        let mut slam = OnlineSlam::new(config);

        let scan = create_test_scan();
        slam.process_scan(&scan, &Pose2D::new(1.0, 0.0, 0.0), 0);
        slam.process_scan(&scan, &Pose2D::new(1.0, 0.0, 0.0), 1000);

        assert!(slam.current_pose().x > 0.5);
        assert!(slam.status().num_scans > 0);

        slam.reset();

        assert_eq!(slam.current_pose().x, 0.0);
        assert_eq!(slam.status().num_scans, 0);
        assert_eq!(slam.submaps().len(), 0);
    }

    #[test]
    fn test_online_slam_idle_mode() {
        let config = OnlineSlamConfig::default();
        let mut slam = OnlineSlam::new(config);

        slam.set_mode(SlamMode::Idle);

        let scan = create_test_scan();
        let result = slam.process_scan(&scan, &Pose2D::new(1.0, 0.0, 0.0), 0);

        // In idle mode, pose should stay at origin
        assert_eq!(result.pose.x, 0.0);
        assert_eq!(slam.status().num_scans, 0);
    }

    #[test]
    fn test_online_slam_status() {
        let config = OnlineSlamConfig::default();
        let slam = OnlineSlam::new(config);

        let status = slam.status();

        assert_eq!(status.mode, SlamMode::Mapping);
        assert_eq!(status.num_scans, 0);
        assert!(!status.is_lost);
    }
}
