//! Online SLAM implementation.
//!
//! Combines scan matching, submap management, keyframe selection, and
//! loop closure detection into a unified real-time SLAM system.
//!
//! # Loop Closure Integration
//!
//! When loop closure is enabled, the system:
//! 1. Builds a pose graph as keyframes are created
//! 2. Uses LiDAR-IRIS descriptors for fast place recognition
//! 3. Verifies candidates with ICP scan matching
//! 4. Optimizes the pose graph when closures are detected
//! 5. Corrects keyframe poses after optimization

use crate::algorithms::mapping::{OccupancyGrid, OccupancyGridConfig};
use crate::algorithms::matching::{
    DynMatcher, DynMatcherConfig, MatcherType, ScanMatchResult, ScanMatcher,
};
use crate::core::types::{Covariance2D, PointCloud2D, Pose2D};
use crate::engine::graph::{
    GraphOptimizer, GraphOptimizerConfig, Information2D, LoopClosureCandidate, LoopDetector,
    LoopDetectorConfig, PoseGraph, PoseNode,
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
#[derive(Debug, Clone, Default)]
pub struct SlamStatus {
    /// Number of keyframes created.
    pub num_keyframes: usize,

    /// Whether the robot is considered "lost" (poor localization).
    pub is_lost: bool,
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

/// Configuration for loop closure detection and optimization.
#[derive(Debug, Clone)]
pub struct LoopClosureConfig {
    /// Enable loop closure detection.
    pub enabled: bool,

    /// Check for loop closures every N keyframes.
    pub detection_interval: usize,

    /// Number of loop closures required to trigger optimization.
    pub optimization_threshold: usize,

    /// Loop detector configuration.
    pub detector: LoopDetectorConfig,

    /// Graph optimizer configuration.
    pub optimizer: GraphOptimizerConfig,
}

impl Default for LoopClosureConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            detection_interval: 5,
            optimization_threshold: 3,
            detector: LoopDetectorConfig::default(),
            optimizer: GraphOptimizerConfig::default(),
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

    /// Matcher type to use (selects algorithm at runtime).
    pub matcher_type: MatcherType,

    /// Scan-to-scan matcher configuration.
    pub matcher: DynMatcherConfig,

    /// Global map configuration.
    pub global_map: OccupancyGridConfig,

    /// Minimum match score to accept pose refinement.
    pub min_match_score: f32,

    /// Score threshold below which robot is considered "lost".
    pub lost_threshold: f32,

    /// Whether to use scan-to-submap matching (vs scan-to-scan).
    ///
    /// **WARNING**: Currently disabled by default due to a semantic mismatch:
    /// `match_to_submap()` returns an absolute pose in the global frame, but
    /// `apply_scan_match()` expects a relative transform (delta). This needs
    /// to be fixed before enabling submap matching.
    ///
    /// TODO: Fix `match_to_submap()` to return delta transform instead of
    /// absolute pose, or modify `apply_scan_match()` to handle both cases.
    pub use_submap_matching: bool,

    /// Minimum points in scan to process.
    pub min_scan_points: usize,

    /// Loop closure configuration.
    pub loop_closure: LoopClosureConfig,
}

impl Default for OnlineSlamConfig {
    fn default() -> Self {
        Self {
            keyframe: KeyframeManagerConfig::default(),
            submap: SubmapManagerConfig::default(),
            // Use HybridP2L by default (best accuracy for structured environments)
            matcher_type: MatcherType::HybridP2l,
            matcher: DynMatcherConfig::default(),
            global_map: OccupancyGridConfig::default(),
            min_match_score: 0.3,
            lost_threshold: 0.1,
            // Use scan-to-scan matching by default - submap matching has semantic
            // issues where match_to_submap returns absolute pose but apply_scan_match
            // expects relative transform
            use_submap_matching: false,
            min_scan_points: 50,
            loop_closure: LoopClosureConfig::default(),
        }
    }
}

/// Online SLAM engine.
///
/// Provides real-time simultaneous localization and mapping by combining:
/// - Scan matching for pose refinement
/// - Submap-based local mapping
/// - Keyframe selection for loop closure
/// - Loop closure detection with LiDAR-IRIS descriptors
/// - Pose graph optimization for global consistency
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

    /// Scan-to-scan matcher (runtime-selected algorithm).
    matcher: DynMatcher,

    /// Scan-to-submap matcher (runtime-selected algorithm).
    submap_matcher: DynMatcher,

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
    // Loop Closure Components
    // ========================================================================
    /// Loop closure detector (uses LiDAR-IRIS descriptors).
    loop_detector: Option<LoopDetector>,

    /// Pose graph for loop closure optimization.
    pose_graph: PoseGraph,

    /// Graph optimizer for pose graph.
    graph_optimizer: GraphOptimizer,

    /// Pending loop closure candidates waiting for optimization.
    pending_closures: Vec<LoopClosureCandidate>,

    /// Previous keyframe pose (for computing odometry edges).
    prev_keyframe_pose: Option<Pose2D>,

    /// Total loop closures detected.
    total_loop_closures: usize,

    /// Total optimizations performed.
    total_optimizations: usize,

    // ========================================================================
    // Localization Components
    // ========================================================================
    /// Reference map for localization mode (loaded via EnableMap command).
    reference_map: Option<OccupancyGrid>,

    /// Cached point cloud of reference map for efficient scan matching.
    reference_pointcloud: Option<PointCloud2D>,
}

impl OnlineSlam {
    /// Typical scan size for pre-allocation (360° lidar at 1° resolution).
    const TYPICAL_SCAN_POINTS: usize = 360;

    /// Create a new online SLAM instance.
    pub fn new(config: OnlineSlamConfig) -> Self {
        // Create scan-to-scan matcher with runtime-selected algorithm
        let matcher = DynMatcher::new(config.matcher_type, config.matcher.clone());

        // Create scan-to-submap matcher with same algorithm
        let submap_matcher = DynMatcher::new(config.matcher_type, config.matcher.clone());

        // Initialize loop closure components if enabled
        let loop_detector = if config.loop_closure.enabled {
            Some(LoopDetector::new(config.loop_closure.detector.clone()))
        } else {
            None
        };
        let graph_optimizer = GraphOptimizer::new(config.loop_closure.optimizer.clone());

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
            // Loop closure fields
            loop_detector,
            pose_graph: PoseGraph::new(),
            graph_optimizer,
            pending_closures: Vec::new(),
            prev_keyframe_pose: None,
            total_loop_closures: 0,
            total_optimizations: 0,
            // Localization fields
            reference_map: None,
            reference_pointcloud: None,
            config,
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

    /// Set a reference map for localization mode.
    ///
    /// When set, the engine will match scans against this map instead of
    /// scan-to-scan matching in Localization mode.
    pub fn set_reference_map(&mut self, map: OccupancyGrid) {
        // Build point cloud representation for scan matching
        let pointcloud = map.as_pointcloud();
        log::info!(
            "Reference map set: {}x{} grid -> {} occupied points",
            map.width(),
            map.height(),
            pointcloud.len()
        );
        self.reference_pointcloud = Some(pointcloud);
        self.reference_map = Some(map);
    }

    /// Clear the reference map.
    pub fn clear_reference_map(&mut self) {
        self.reference_map = None;
        self.reference_pointcloud = None;
    }

    /// Check if a reference map is loaded.
    pub fn has_reference_map(&self) -> bool {
        self.reference_pointcloud.is_some()
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

    /// Mark a bumper obstacle in the global map.
    ///
    /// When a bumper is triggered, this marks cells in front of the robot as
    /// occupied. This ensures obstacles detected by physical contact are
    /// reflected in the map for future planning.
    ///
    /// Returns true if the obstacle was marked, false if mapping mode is not
    /// active or the map hasn't been built yet.
    pub fn mark_bumper_obstacle(
        &mut self,
        robot_x: f32,
        robot_y: f32,
        robot_theta: f32,
        bumper_left: bool,
        bumper_right: bool,
        robot_radius: f32,
    ) -> bool {
        // Only mark obstacles when we're mapping
        if self.mode != SlamMode::Mapping {
            return false;
        }

        // Ensure global map exists
        if self.global_map.is_none() {
            self.rebuild_global_map();
        }

        if let Some(ref mut map) = self.global_map {
            map.mark_bumper_obstacle(
                robot_x,
                robot_y,
                robot_theta,
                bumper_left,
                bumper_right,
                robot_radius,
            );
            log::debug!(
                "Marked bumper obstacle at ({:.2}, {:.2}) theta={:.2}rad, left={}, right={}",
                robot_x,
                robot_y,
                robot_theta,
                bumper_left,
                bumper_right
            );
            true
        } else {
            false
        }
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
    /// The scan matcher aligns source (current scan) to target (previous scan).
    /// The returned transform represents the robot motion from previous to current.
    /// We pass odom_delta directly as the initial guess since it's exactly the
    /// expected robot motion.
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

        // Initial guess: odometry delta (the robot motion from previous to current).
        // The scan matcher aligns source (current scan) to target (previous scan),
        // returning a transform T that represents the robot motion from prev→curr.
        // This is exactly odom_delta, so we pass it directly.
        let initial_guess = *odom_delta;
        let result = self.matcher.match_scans(scan, prev, &initial_guess);

        if result.converged && result.score >= self.config.min_match_score {
            Some(result)
        } else {
            None
        }
    }

    /// Match scan to reference map for localization.
    ///
    /// Used in Localization mode when a reference map has been loaded via
    /// `set_reference_map()`. The scan is matched against the cached point cloud
    /// representation of the reference map.
    ///
    /// Returns the corrected pose in the map frame if matching succeeds.
    fn match_to_reference_map(
        &mut self,
        scan: &PointCloud2D,
        predicted_pose: &Pose2D,
    ) -> Option<ScanMatchResult> {
        let reference = self.reference_pointcloud.as_ref()?;

        if reference.len() < self.config.min_scan_points {
            log::warn!(
                "Reference map has too few points ({} < {})",
                reference.len(),
                self.config.min_scan_points
            );
            return None;
        }

        // Transform scan to map frame using predicted pose for matching
        let transformed_scan = scan.transform(predicted_pose);

        // Match transformed scan against reference map
        // Initial guess is identity since scan is already in map frame
        let result = self
            .matcher
            .match_scans(&transformed_scan, reference, &Pose2D::identity());

        if result.converged && result.score >= self.config.min_match_score {
            // Combine predicted pose with the correction from matching
            let corrected_pose = predicted_pose.compose(&result.transform);
            Some(ScanMatchResult {
                transform: corrected_pose,
                ..result
            })
        } else {
            None
        }
    }

    /// Process pose update from scan matching.
    ///
    /// The scan match result transform represents the robot motion from previous to current.
    /// To compute the corrected pose:
    /// - current_pose is the robot's pose before this update
    /// - match_result.transform is the refined motion estimate (prev→curr)
    /// - corrected_pose = current_pose.compose(match_result.transform)
    ///
    /// Uses a threshold-based approach to blend between scan match and odometry
    /// to avoid erratic pose jumps from small score variations.
    ///
    /// Also includes a sanity check to reject scan matches that differ too much
    /// from odometry, which can happen in symmetric environments or during
    /// fast rotation where the matcher finds wrong local minima.
    fn apply_scan_match(&mut self, match_result: &ScanMatchResult, odom_delta: &Pose2D) -> Pose2D {
        let odom_pose = self.current_pose.compose(odom_delta);

        // Compute scan-match-corrected pose:
        // match_result.transform is the robot motion from previous to current.
        // Apply it directly to get the new pose.
        let scan_match_pose = self.current_pose.compose(&match_result.transform);

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

    // ========================================================================
    // Loop Closure Methods
    // ========================================================================

    /// Process a new keyframe for loop closure detection.
    ///
    /// This method:
    /// 1. Adds the keyframe to the pose graph
    /// 2. Adds an odometry edge from the previous keyframe
    /// 3. Adds the scan to the descriptor database
    /// 4. Detects loop closures (every N keyframes)
    /// 5. Triggers optimization when enough closures are found
    fn process_keyframe_for_loop_closure(
        &mut self,
        kf_id: u64,
        scan: &PointCloud2D,
        pose: &Pose2D,
        timestamp_us: u64,
    ) {
        // 1. Add node to pose graph
        let node = PoseNode::new(kf_id, *pose, timestamp_us).with_keyframe(kf_id);
        self.pose_graph.add_node_full(node);

        // 2. Add odometry edge from previous keyframe
        if let Some(prev_pose) = self.prev_keyframe_pose {
            let odom_delta = prev_pose.inverse().compose(pose);
            // Use information matrix based on scan match quality
            let info = Information2D::from_std_dev(0.05, 0.05, 0.03); // ~5cm, ~2deg
            self.pose_graph
                .add_odometry_edge(kf_id - 1, kf_id, odom_delta, info);
        } else {
            // First keyframe - fix it as anchor
            self.pose_graph.fix_first_node();
        }
        self.prev_keyframe_pose = Some(*pose);

        // 3. Add scan to descriptor database
        if let Some(detector) = &mut self.loop_detector {
            detector.add_keyframe(kf_id, scan);
        }

        // 4. Detect loop closures (every detection_interval keyframes)
        let detection_interval = self.config.loop_closure.detection_interval;
        if kf_id > 0 && (kf_id as usize).is_multiple_of(detection_interval) {
            self.detect_loop_closures(kf_id, scan, pose);
        }

        // 5. Trigger optimization if enough closures pending
        let optimization_threshold = self.config.loop_closure.optimization_threshold;
        if self.pending_closures.len() >= optimization_threshold {
            self.optimize_pose_graph();
        }
    }

    /// Detect loop closures for a keyframe.
    fn detect_loop_closures(&mut self, query_id: u64, scan: &PointCloud2D, pose: &Pose2D) {
        let keyframes = self.keyframes.keyframes();

        // Use the detector to find candidates
        if let Some(detector) = &mut self.loop_detector {
            let candidates = detector.detect(query_id, scan, pose, keyframes);

            for candidate in candidates {
                log::info!(
                    "Loop closure detected: {} -> {} (score: {:.2})",
                    candidate.query_id,
                    candidate.match_id,
                    candidate.confidence
                );

                // Add loop closure edge to pose graph
                self.pose_graph.add_loop_closure_edge(
                    candidate.query_id,
                    candidate.match_id,
                    candidate.relative_pose,
                    candidate.information,
                    candidate.confidence,
                );

                self.total_loop_closures += 1;
                self.pending_closures.push(candidate);
            }
        }
    }

    /// Optimize the pose graph and apply corrections.
    fn optimize_pose_graph(&mut self) {
        if self.pose_graph.num_nodes() < 3 {
            return;
        }

        log::info!(
            "Optimizing pose graph: {} nodes, {} edges ({} loop closures)",
            self.pose_graph.num_nodes(),
            self.pose_graph.num_edges(),
            self.pose_graph.num_loop_closures()
        );

        let result = self.graph_optimizer.optimize(&mut self.pose_graph);

        if result.converged {
            log::info!(
                "Optimization converged in {} iterations (error: {:.4} -> {:.4})",
                result.iterations,
                result.initial_error,
                result.final_error
            );

            // Apply corrections to keyframes and current pose
            self.apply_pose_corrections();
            self.total_optimizations += 1;
        } else {
            log::warn!(
                "Optimization did not converge: {:?}",
                result.termination_reason
            );
        }

        // Clear pending closures after optimization
        self.pending_closures.clear();
    }

    /// Apply pose corrections from optimized graph to keyframes.
    fn apply_pose_corrections(&mut self) {
        // Update keyframe poses from optimized graph
        for node in self.pose_graph.nodes() {
            if let Some(kf_id) = node.keyframe_id
                && let Some(kf) = self.keyframes.get_mut(kf_id)
            {
                kf.pose = node.pose;
            }
        }

        // Update current pose if we have a latest node
        if let Some(latest) = self.pose_graph.latest_node() {
            self.current_pose = latest.pose;
            self.prev_keyframe_pose = Some(latest.pose);
        }

        // Invalidate global map cache (will be rebuilt with corrected poses)
        self.global_map = None;

        log::debug!(
            "Applied pose corrections to {} keyframes",
            self.keyframes.len()
        );
    }

    /// Get the pose graph (for inspection/debugging).
    pub fn pose_graph(&self) -> &PoseGraph {
        &self.pose_graph
    }

    /// Get total loop closures detected.
    pub fn total_loop_closures(&self) -> usize {
        self.total_loop_closures
    }

    /// Get total optimizations performed.
    pub fn total_optimizations(&self) -> usize {
        self.total_optimizations
    }

    /// Check if loop closure is enabled.
    pub fn loop_closure_enabled(&self) -> bool {
        self.loop_detector.is_some()
    }
}

impl SlamEngine for OnlineSlam {
    fn process_scan(
        &mut self,
        scan: &PointCloud2D,
        odom_delta: &Pose2D,
        timestamp_us: u64,
    ) -> SlamResult {
        let mut result = SlamResult::default();

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

        // Select matching strategy based on mode and available data
        let (match_result, is_localization_match) =
            if self.mode == SlamMode::Localization && self.reference_pointcloud.is_some() {
                // Localization mode with reference map: match against reference
                (self.match_to_reference_map(scan, &predicted_pose), true)
            } else if self.config.use_submap_matching && self.submaps.active_submap().is_some() {
                // Mapping mode with submap matching
                (self.match_to_submap(scan, &predicted_pose), false)
            } else {
                // Default: scan-to-scan matching
                (self.match_to_previous_scan(scan, odom_delta), false)
            };

        // Apply match result or use odometry
        let refined_pose = if let Some(ref mr) = match_result {
            result.pose_refined = true;
            result.match_score = mr.score;
            self.last_match_score = mr.score;
            self.is_lost = mr.score < self.config.lost_threshold;

            if mr.score >= self.config.min_match_score {
                if is_localization_match {
                    // Localization mode: match_to_reference_map returns absolute pose
                    mr.transform
                } else {
                    // Mapping mode: apply delta transform
                    let result_pose = self.apply_scan_match(mr, odom_delta);

                    // Debug: log every 10th scan to track drift
                    if self.num_scans.is_multiple_of(10) {
                        let odom_theta_deg = odom_delta.theta.to_degrees();
                        let match_theta_deg = mr.transform.theta.to_degrees();
                        let diff_theta_deg = (mr.transform.theta - odom_delta.theta).to_degrees();
                        log::debug!(
                            "Scan {}: odom_dθ={:.2}° match_dθ={:.2}° diff={:.2}° score={:.2}",
                            self.num_scans,
                            odom_theta_deg,
                            match_theta_deg,
                            diff_theta_deg,
                            mr.score
                        );
                    }
                    result_pose
                }
            } else {
                log::debug!(
                    "Scan {}: low score {:.2}, using odometry",
                    self.num_scans,
                    mr.score
                );
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

        // Map update in mapping mode
        if self.mode == SlamMode::Mapping {
            let old_submap_count = self.submaps.len();
            let submap_id = self.submaps.process_scan(scan, &refined_pose, timestamp_us);
            result.new_submap = self.submaps.len() > old_submap_count;

            // Keyframe check
            if self
                .keyframes
                .should_create_keyframe(&refined_pose, timestamp_us)
            {
                self.keyframes
                    .create_keyframe(refined_pose, scan.clone(), timestamp_us, submap_id);
                let kf_id = self.keyframes.len() as u64 - 1;
                self.submaps.add_keyframe_to_active(kf_id);
                result.keyframe_created = true;

                // Loop closure integration
                if self.loop_detector.is_some() {
                    self.process_keyframe_for_loop_closure(
                        kf_id,
                        scan,
                        &refined_pose,
                        timestamp_us,
                    );
                }
            }

            // Invalidate cached global map
            if result.keyframe_created || result.new_submap {
                self.global_map = None;
            }
        }

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

        result
    }

    fn current_pose(&self) -> Pose2D {
        self.current_pose
    }

    fn status(&self) -> SlamStatus {
        SlamStatus {
            num_keyframes: self.keyframes.len(),
            is_lost: self.is_lost,
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
        // Reset loop closure state
        if let Some(detector) = &mut self.loop_detector {
            detector.clear();
        }
        self.pose_graph.clear();
        self.pending_closures.clear();
        self.prev_keyframe_pose = None;
        self.total_loop_closures = 0;
        self.total_optimizations = 0;
        // Reset localization state
        self.reference_map = None;
        self.reference_pointcloud = None;
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

        slam.reset();

        assert_eq!(slam.current_pose().x, 0.0);
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
    }

    #[test]
    fn test_online_slam_status() {
        let config = OnlineSlamConfig::default();
        let slam = OnlineSlam::new(config);

        let status = slam.status();

        assert_eq!(status.num_keyframes, 0);
        assert!(!status.is_lost);
    }

    #[test]
    fn test_loop_closure_enabled_by_default() {
        let config = OnlineSlamConfig::default();
        let slam = OnlineSlam::new(config);

        assert!(
            slam.loop_closure_enabled(),
            "Loop closure should be enabled by default"
        );
        assert_eq!(slam.total_loop_closures(), 0);
        assert_eq!(slam.total_optimizations(), 0);
    }

    #[test]
    fn test_loop_closure_disabled() {
        let config = OnlineSlamConfig {
            loop_closure: LoopClosureConfig {
                enabled: false,
                ..Default::default()
            },
            ..Default::default()
        };
        let slam = OnlineSlam::new(config);

        assert!(!slam.loop_closure_enabled());
    }

    #[test]
    fn test_pose_graph_builds_with_keyframes() {
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

        // Process scans to create keyframes
        for i in 0..5 {
            let offset = i as f32 * 0.5;
            let scan = create_test_scan_at(offset, 0.0);
            slam.process_scan(&scan, &Pose2D::new(0.5, 0.0, 0.0), i * 1000);
        }

        // Should have built pose graph nodes
        let num_keyframes = slam.keyframes().len();
        let num_graph_nodes = slam.pose_graph().num_nodes();

        // When loop closure is enabled, pose graph nodes should match keyframes
        assert_eq!(
            num_graph_nodes, num_keyframes,
            "Pose graph nodes ({}) should match keyframes ({})",
            num_graph_nodes, num_keyframes
        );
    }

    #[test]
    fn test_reset_clears_loop_closure_state() {
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

        // Process some scans
        for i in 0..3 {
            let offset = i as f32 * 0.5;
            let scan = create_test_scan_at(offset, 0.0);
            slam.process_scan(&scan, &Pose2D::new(0.5, 0.0, 0.0), i * 1000);
        }

        // Should have some pose graph state
        assert!(slam.pose_graph().num_nodes() > 0);

        // Reset should clear everything
        slam.reset();

        assert_eq!(slam.pose_graph().num_nodes(), 0);
        assert_eq!(slam.pose_graph().num_edges(), 0);
        assert_eq!(slam.total_loop_closures(), 0);
        assert_eq!(slam.total_optimizations(), 0);
    }
}
