//! VectorMap: The main SLAM map implementation.

use std::time::Instant;

use crate::config::ExplorationConfig;
use crate::core::{Bounds, Point2D, PointCloud2D, Pose2D};
use crate::extraction::{detect_corners, extract_lines, extract_lines_hybrid};
use crate::features::{Corner2D, FeatureSet, Line2D};
use crate::integration::{
    PointAssociationConfig, RefitConfig, RefitResult, RefitStats, ScanStore, ScanStoreConfig,
    associate_points_to_lines, batch_merge, create_new_line, find_associations,
    find_unmatched_scan_lines, refit_line,
};
use crate::loop_closure::{LoopClosure, LoopClosureDetector};
use crate::matching::PointToLineIcp;
use crate::query::{
    PathPlanner, detect_frontiers, is_straight_path_clear, query_occupancy, raycast,
};
use crate::{Frontier, Map, ObserveResult, Occupancy, Path, TimingBreakdown};

use super::config::VectorMapConfig;
use super::line_store::LineStore;

// ============================================================================
// Scratch Space for Zero-Allocation Observe Pipeline
// ============================================================================

/// Pre-allocated scratch space for the observe pipeline.
///
/// Eliminates per-observation allocations by reusing buffers across calls.
/// This is critical for real-time SLAM at high observation rates (5-110 Hz).
///
/// # Performance Impact
///
/// Without scratch space, each `observe()` call allocates:
/// - `robot_frame_points`: O(n) points for ICP (done up to 3x per observation!)
/// - `lines_robot`: O(m) lines from extraction
/// - `lines_world`: O(m) lines after transform
/// - `scan_lines`: O(m) lines for integration
///
/// With scratch space, these allocations happen once and are reused.
#[derive(Clone, Debug, Default)]
pub struct ObserveScratchSpace {
    /// Buffer for robot-frame scan points.
    /// Reused for ICP matching and loop closure.
    robot_frame_points: Vec<Point2D>,

    /// Buffer for extracted lines in robot frame.
    lines_robot: Vec<Line2D>,

    /// Buffer for lines transformed to world frame.
    lines_world: Vec<Line2D>,
}

impl ObserveScratchSpace {
    /// Create new scratch space with capacity hints.
    ///
    /// # Arguments
    /// * `max_points` - Expected maximum scan points (e.g., 360 for typical lidar)
    /// * `max_lines` - Expected maximum lines per scan (e.g., 20-50)
    pub fn with_capacity(max_points: usize, max_lines: usize) -> Self {
        Self {
            robot_frame_points: Vec::with_capacity(max_points),
            lines_robot: Vec::with_capacity(max_lines),
            lines_world: Vec::with_capacity(max_lines),
        }
    }

    /// Create with default capacity (360 points, 50 lines).
    pub fn default_capacity() -> Self {
        Self::with_capacity(360, 50)
    }

    /// Fill robot_frame_points from a PointCloud2D.
    #[inline]
    pub fn fill_robot_frame_points(&mut self, scan: &PointCloud2D) {
        self.robot_frame_points.clear();
        self.robot_frame_points.reserve(scan.len());
        self.robot_frame_points.extend(scan.iter());
    }

    /// Get robot frame points slice.
    #[inline]
    pub fn robot_frame_points(&self) -> &[Point2D] {
        &self.robot_frame_points
    }

    /// Fill lines_robot from extracted lines.
    #[inline]
    pub fn fill_lines_robot(&mut self, lines: &[Line2D]) {
        self.lines_robot.clear();
        self.lines_robot.extend_from_slice(lines);
    }

    /// Transform robot-frame lines to world frame and store in lines_world.
    #[inline]
    pub fn transform_lines_to_world(&mut self, pose: &Pose2D) {
        self.lines_world.clear();
        self.lines_world.reserve(self.lines_robot.len());
        for line in &self.lines_robot {
            self.lines_world.push(line.transform(pose));
        }
    }

    /// Get world-frame lines slice.
    #[inline]
    pub fn lines_world(&self) -> &[Line2D] {
        &self.lines_world
    }
}

/// VectorMap: Feature-based 2D SLAM map.
///
/// Stores the environment as a collection of line and corner features.
/// Provides localization via scan matching and mapping via feature integration.
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::{VectorMap, VectorMapConfig, Map};
/// use vastu_map::core::{Pose2D, PointCloud2D, Point2D};
///
/// let config = VectorMapConfig::default();
/// let mut map = VectorMap::new(config);
///
/// // Process scans
/// let scan = PointCloud2D::from_points(&[
///     Point2D::new(1.0, 0.0),
///     Point2D::new(1.0, 0.1),
///     Point2D::new(1.0, 0.2),
/// ]);
/// let odometry = Pose2D::identity();
///
/// let result = map.observe(&scan, odometry);
/// println!("Pose: ({:.2}, {:.2}, {:.2})", result.pose.x, result.pose.y, result.pose.theta);
/// ```
pub struct VectorMap {
    /// Configuration.
    config: VectorMapConfig,

    /// Line storage with synchronized auxiliary structures.
    line_store: LineStore,

    /// Corner features.
    corners: Vec<Corner2D>,

    /// Current robot pose estimate.
    current_pose: Pose2D,

    /// Number of observations processed.
    observation_count: usize,

    /// Loop closure detector.
    loop_detector: LoopClosureDetector,

    /// Detected loop closures.
    loop_closures: Vec<LoopClosure>,

    /// Persistent ICP matcher (preserves state across scans for adaptive coarse search).
    icp_matcher: PointToLineIcp,

    /// Pre-allocated scratch space for observe pipeline (zero-allocation).
    observe_scratch: ObserveScratchSpace,

    /// Scan storage for exploration mode.
    /// When enabled, stores raw scan data for point-cloud-based line re-fitting.
    scan_store: Option<ScanStore>,

    /// Exploration configuration (None if exploration mode disabled).
    exploration_config: Option<ExplorationConfig>,
}

impl VectorMap {
    /// Create a new empty VectorMap.
    pub fn new(config: VectorMapConfig) -> Self {
        let loop_detector = LoopClosureDetector::new(config.loop_closure.clone());
        let icp_matcher = PointToLineIcp::with_config(config.matching.clone());
        Self {
            config,
            line_store: LineStore::new(),
            corners: Vec::new(),
            current_pose: Pose2D::identity(),
            observation_count: 0,
            loop_detector,
            loop_closures: Vec::new(),
            icp_matcher,
            observe_scratch: ObserveScratchSpace::default_capacity(),
            scan_store: None,
            exploration_config: None,
        }
    }

    /// Get the current configuration.
    pub fn config(&self) -> &VectorMapConfig {
        &self.config
    }

    /// Get all lines in the map.
    pub fn lines(&self) -> &[Line2D] {
        self.line_store.lines()
    }

    /// Get all corners in the map.
    pub fn corners(&self) -> &[Corner2D] {
        &self.corners
    }

    /// Get the current robot pose.
    pub fn current_pose(&self) -> Pose2D {
        self.current_pose
    }

    /// Get the number of observations processed.
    pub fn observation_count(&self) -> usize {
        self.observation_count
    }

    /// Get the feature set (lines and corners).
    pub fn features(&self) -> FeatureSet {
        FeatureSet::from_features(self.line_store.lines(), &self.corners)
    }

    /// Get detected loop closures.
    ///
    /// These can be used by an external pose graph optimizer to correct drift.
    pub fn get_loop_closures(&self) -> &[LoopClosure] {
        &self.loop_closures
    }

    /// Get the number of detected loop closures.
    pub fn loop_closure_count(&self) -> usize {
        self.loop_closures.len()
    }

    /// Get the loop closure detector (for advanced access).
    pub fn loop_detector(&self) -> &LoopClosureDetector {
        &self.loop_detector
    }

    /// Clear all loop closures.
    pub fn clear_loop_closures(&mut self) {
        self.loop_closures.clear();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Exploration Mode
    // ─────────────────────────────────────────────────────────────────────────

    /// Enable exploration mode with the given configuration.
    ///
    /// In exploration mode, VectorMap stores raw scan data and periodically
    /// re-fits lines from accumulated point clouds, eliminating first-scan bias.
    ///
    /// # Arguments
    /// * `config` - Exploration configuration
    ///
    /// # Example
    /// ```rust,ignore
    /// use vastu_map::config::ExplorationConfig;
    ///
    /// let config = ExplorationConfig::default()
    ///     .with_refit_interval(10)
    ///     .with_gap_threshold(0.3);
    ///
    /// map.enable_exploration_mode(config);
    /// ```
    pub fn enable_exploration_mode(&mut self, config: ExplorationConfig) {
        let scan_config = ScanStoreConfig::default();
        self.scan_store = Some(ScanStore::new(scan_config));
        self.exploration_config = Some(config);
    }

    /// Enable exploration mode with custom scan store configuration.
    ///
    /// Allows configuring max scans, distance/rotation filtering, etc.
    pub fn enable_exploration_mode_with_scan_config(
        &mut self,
        exploration_config: ExplorationConfig,
        scan_config: ScanStoreConfig,
    ) {
        self.scan_store = Some(ScanStore::new(scan_config));
        self.exploration_config = Some(exploration_config);
    }

    /// Disable exploration mode and return the accumulated scan store.
    ///
    /// Returns the ScanStore containing all accumulated scans, or None
    /// if exploration mode was not enabled.
    pub fn disable_exploration_mode(&mut self) -> Option<ScanStore> {
        self.exploration_config = None;
        self.scan_store.take()
    }

    /// Check if exploration mode is enabled.
    pub fn is_exploration_mode(&self) -> bool {
        self.exploration_config.is_some()
    }

    /// Get the exploration configuration (if exploration mode is enabled).
    pub fn exploration_config(&self) -> Option<&ExplorationConfig> {
        self.exploration_config.as_ref()
    }

    /// Get the scan store (if exploration mode is enabled).
    pub fn scan_store(&self) -> Option<&ScanStore> {
        self.scan_store.as_ref()
    }

    /// Get mutable access to the scan store (if exploration mode is enabled).
    pub fn scan_store_mut(&mut self) -> Option<&mut ScanStore> {
        self.scan_store.as_mut()
    }

    /// Add a line to the map.
    pub fn add_line(&mut self, line: Line2D) {
        self.line_store.add(line);
    }

    /// Add multiple lines to the map.
    pub fn add_lines(&mut self, lines: &[Line2D]) {
        self.line_store.add_many(lines);
    }

    /// Set the current pose directly.
    pub fn set_pose(&mut self, pose: Pose2D) {
        self.current_pose = pose;
    }

    /// Process a scan observation.
    ///
    /// This is the main SLAM function. It:
    /// 1. Extracts features from the scan
    /// 2. Matches against the map (localization)
    /// 3. Updates the map with new features (mapping)
    /// 4. Checks for loop closure (if enabled)
    fn process_scan(&mut self, scan: &PointCloud2D, odometry: Pose2D) -> ObserveResult {
        let total_start = Instant::now();
        let mut timing = TimingBreakdown::new();

        self.observation_count += 1;

        // If map is empty, use odometry and add all features
        if self.line_store.is_empty() {
            return self.initialize_from_scan(scan, odometry);
        }

        // Apply odometry to get predicted pose
        let predicted_pose = self.current_pose.compose(odometry);

        // Get robot-frame scan points for weighted ICP matching (reuse scratch buffer)
        self.observe_scratch.fill_robot_frame_points(scan);

        // ─── ICP Matching ───
        let icp_start = Instant::now();
        // Use cached LineCollection from LineStore (avoids rebuilding SoA structure)
        let match_result = self.icp_matcher.match_scan_robot_frame_collection(
            self.observe_scratch.robot_frame_points(),
            self.line_store.collection(),
            predicted_pose,
        );
        timing.icp_matching_us = icp_start.elapsed().as_micros() as u64;

        // Update ICP matcher's confidence state for adaptive coarse search triggering
        self.icp_matcher
            .update_last_confidence(match_result.confidence);

        // Determine final pose and capture ICP stats
        let icp_valid = if match_result.converged
            && match_result.confidence >= self.config.min_match_confidence
        {
            // Check for degenerate geometry
            if match_result.condition_number > 100.0 {
                log::debug!(
                    "Rejecting ICP match: degenerate geometry (cond={})",
                    match_result.condition_number
                );
                false
            } else {
                // Check if ICP pose is reasonable (not too far from prediction)
                let dx = match_result.pose.x - predicted_pose.x;
                let dy = match_result.pose.y - predicted_pose.y;
                let dist = (dx * dx + dy * dy).sqrt();
                let dtheta =
                    crate::core::math::angle_diff(match_result.pose.theta, predicted_pose.theta)
                        .abs();

                if dist > 0.2 || dtheta > 0.175 {
                    log::debug!(
                        "Rejecting ICP match: large deviation (dist={:.3}m, dtheta={:.1}°)",
                        dist,
                        dtheta.to_degrees()
                    );
                    false
                } else {
                    true
                }
            }
        } else {
            false
        };

        let (final_pose, confidence, icp_iterations, icp_converged) = if icp_valid {
            (
                match_result.pose,
                match_result.confidence,
                match_result.iterations,
                match_result.converged,
            )
        } else {
            (
                predicted_pose,
                0.0,
                match_result.iterations,
                match_result.converged,
            )
        };

        self.current_pose = final_pose;

        // ─── Scan Storage (Exploration Mode) ───
        if let Some(ref mut store) = self.scan_store {
            store.add_scan(
                odometry, final_pose, scan,
                0, // features_extracted - will be updated after extraction
                confidence,
            );
        }

        // ─── Feature Extraction ───
        // (robot_frame_points already filled in scratch above)
        let features = self.extract_features_timed(&final_pose, &mut timing);

        // ─── Feature Integration ───
        let (features_added, features_merged) = if self.config.mapping_enabled {
            self.integrate_features_timed(&mut timing)
        } else {
            (0, 0)
        };

        // ─── Periodic Line Re-fitting (Exploration Mode) ───
        if let Some(ref config) = self.exploration_config
            && config.refit_interval > 0
            && self.observation_count.is_multiple_of(config.refit_interval)
        {
            log::debug!(
                "Triggering line re-fit at observation {}",
                self.observation_count
            );
            // Trigger line re-fitting from accumulated scans
            let _ = self.refit_lines_from_scans();
        }

        // ─── Loop Closure ───
        let loop_closure_detected = if self.config.loop_closure_enabled {
            let lc_start = Instant::now();
            // Reuse robot_frame_points from scratch (already filled for ICP)
            let feature_lines = features.lines();
            let feature_corners = features.corners();

            let detected = if let Some(loop_closure) = self.loop_detector.process(
                final_pose,
                &feature_lines,
                &feature_corners,
                self.observe_scratch.robot_frame_points(),
            ) {
                self.loop_closures.push(loop_closure);
                true
            } else {
                false
            };
            timing.loop_closure_us = lc_start.elapsed().as_micros() as u64;
            detected
        } else {
            false
        };

        timing.total_us = total_start.elapsed().as_micros() as u64;

        ObserveResult {
            pose: final_pose,
            confidence,
            features_extracted: features.lines().len(),
            features_added,
            features_merged,
            icp_iterations,
            icp_converged,
            loop_closure_detected,
            timing,
        }
    }

    /// Initialize map from first scan.
    fn initialize_from_scan(&mut self, scan: &PointCloud2D, odometry: Pose2D) -> ObserveResult {
        let total_start = Instant::now();
        let mut timing = TimingBreakdown::new();

        self.current_pose = odometry;

        // Fill scratch with robot frame points (used by feature extraction and loop closure)
        self.observe_scratch.fill_robot_frame_points(scan);

        // ─── Scan Storage (Exploration Mode) ───
        if let Some(ref mut store) = self.scan_store {
            store.add_scan(
                odometry, odometry, // First scan: odometry = estimated pose
                scan, 0,   // features_extracted - unknown at this point
                0.0, // confidence - no ICP for first scan
            );
        }

        // Extract features from robot-frame scan, transform to world using odometry
        let features = self.extract_features_timed(&odometry, &mut timing);

        // Add all features to map
        let num_lines = features.lines().len();
        self.add_lines(&features.lines());

        for corner in features.corners() {
            self.corners.push(corner);
        }

        // Initialize first keyframe if loop closure is enabled
        if self.config.loop_closure_enabled {
            let lc_start = Instant::now();
            let feature_lines = features.lines();
            let feature_corners = features.corners();
            self.loop_detector.force_keyframe(
                odometry,
                &feature_lines,
                &feature_corners,
                self.observe_scratch.robot_frame_points(),
            );
            timing.loop_closure_us = lc_start.elapsed().as_micros() as u64;
        }

        timing.total_us = total_start.elapsed().as_micros() as u64;

        ObserveResult {
            pose: odometry,
            confidence: 0.0,
            features_extracted: num_lines,
            features_added: num_lines,
            features_merged: 0,
            icp_iterations: 0,
            icp_converged: false,
            loop_closure_detected: false,
            timing,
        }
    }

    /// Extract features from a scan with timing.
    ///
    /// NOTE: Caller must fill observe_scratch.robot_frame_points before calling this.
    fn extract_features_timed(
        &mut self,
        pose: &Pose2D,
        timing: &mut TimingBreakdown,
    ) -> FeatureSet {
        // Use robot_frame_points from scratch (already filled by caller)
        let points = self.observe_scratch.robot_frame_points();

        // ─── Line Extraction ───
        let line_start = Instant::now();
        let lines_robot = if self.config.use_hybrid_extraction {
            // Hybrid: RANSAC for dominant lines, then split-merge for remaining
            extract_lines_hybrid(
                points,
                &self.config.ransac_extraction,
                &self.config.extraction,
            )
        } else {
            // Default: split-merge (preserves angular ordering)
            extract_lines(points, &self.config.extraction)
        };

        // Transform lines to world frame using scratch buffer
        self.observe_scratch.fill_lines_robot(&lines_robot);
        self.observe_scratch.transform_lines_to_world(pose);
        timing.line_extraction_us = line_start.elapsed().as_micros() as u64;

        // ─── Corner Detection ───
        let corner_start = Instant::now();
        let corners = detect_corners(self.observe_scratch.lines_world(), &self.config.corner);
        timing.corner_detection_us = corner_start.elapsed().as_micros() as u64;

        FeatureSet::from_features(self.observe_scratch.lines_world(), &corners)
    }

    /// Integrate new features into the map with timing.
    ///
    /// NOTE: Uses lines_world from observe_scratch (filled by extract_features_timed).
    fn integrate_features_timed(&mut self, timing: &mut TimingBreakdown) -> (usize, usize) {
        // Use lines from scratch (already in world frame from extract_features_timed)
        let scan_lines = self.observe_scratch.lines_world();

        if scan_lines.is_empty() {
            return (0, 0);
        }

        // ─── Association ───
        let assoc_start = Instant::now();
        let associations = find_associations(
            scan_lines,
            self.line_store.lines(),
            Some(self.line_store.index()),
            &self.config.association,
        );
        timing.association_us = assoc_start.elapsed().as_micros() as u64;

        // ─── Merging ───
        let merge_start = Instant::now();
        let merged = batch_merge(
            scan_lines,
            self.line_store.lines_mut(),
            &associations,
            &self.config.merger,
        );
        let features_merged = merged.len();
        timing.merging_us = merge_start.elapsed().as_micros() as u64;

        // ─── New Features ───
        let new_start = Instant::now();
        let unmatched = find_unmatched_scan_lines(
            scan_lines,
            self.line_store.lines(),
            Some(self.line_store.index()),
            &self.config.association,
        );

        let mut features_added = 0;
        for idx in unmatched {
            let new_line = create_new_line(&scan_lines[idx]);
            self.line_store.lines_mut().push(new_line);
            features_added += 1;
        }

        // Rebuild auxiliary structures
        self.line_store.rebuild_all();

        // Update corners
        self.update_corners();
        timing.new_features_us = new_start.elapsed().as_micros() as u64;

        (features_added, features_merged)
    }

    /// Update corners based on current lines.
    fn update_corners(&mut self) {
        self.corners = detect_corners(self.line_store.lines(), &self.config.corner);
    }

    /// Optimize map by merging coplanar line segments.
    ///
    /// Call this periodically (e.g., after N observations) to clean up
    /// redundant line segments that represent the same wall.
    ///
    /// # Arguments
    /// * `config` - Configuration for coplanar merging thresholds
    ///
    /// # Returns
    /// Number of lines merged (removed from the map)
    ///
    /// # Example
    /// ```rust,ignore
    /// use vastu_map::integration::CoplanarMergeConfig;
    ///
    /// // After exploration...
    /// let config = CoplanarMergeConfig::default()
    ///     .with_max_angle_diff(0.0175)  // 1 degree
    ///     .with_max_perpendicular_dist(0.02);  // 2cm
    /// let merged = map.optimize_lines(&config);
    /// println!("Merged {} redundant lines", merged);
    /// ```
    pub fn optimize_lines(&mut self, config: &crate::integration::CoplanarMergeConfig) -> usize {
        let merged =
            crate::integration::optimize_coplanar_lines(self.line_store.lines_mut(), config);

        if merged > 0 {
            // Rebuild auxiliary structures
            self.line_store.rebuild_all();
            // Update corners based on new line set
            self.update_corners();
        }

        merged
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Line Re-fitting from Accumulated Scans
    // ─────────────────────────────────────────────────────────────────────────

    /// Re-fit all map lines from accumulated scan points.
    ///
    /// This method uses points from the ScanStore to re-fit line geometry,
    /// eliminating first-scan bias. Call this periodically during exploration.
    ///
    /// Requires exploration mode to be enabled.
    ///
    /// # Returns
    /// Statistics about the re-fitting operation, or None if exploration mode
    /// is not enabled.
    ///
    /// # Example
    /// ```rust,ignore
    /// if map.is_exploration_mode() && map.observation_count() % 10 == 0 {
    ///     if let Some(stats) = map.refit_lines_from_scans() {
    ///         println!("Refitted lines: {}/{} success rate",
    ///             stats.lines_refitted_single + stats.lines_split,
    ///             stats.total_processed());
    ///     }
    /// }
    /// ```
    pub fn refit_lines_from_scans(&mut self) -> Option<RefitStats> {
        // Get exploration config and scan store
        let exploration_config = self.exploration_config.as_ref()?;
        let scan_store = self.scan_store.as_ref()?;

        if self.line_store.is_empty() || scan_store.is_empty() {
            return Some(RefitStats::default());
        }

        // Build association config from exploration config
        let assoc_config = PointAssociationConfig::default()
            .with_max_distance(exploration_config.max_point_distance);

        // Associate all stored points with map lines
        let associations = associate_points_to_lines(
            scan_store,
            self.line_store.lines(),
            self.line_store.index(),
            &assoc_config,
        );

        log::debug!(
            "Refit: {} scans, {} lines, {} points associated ({:.1}% rate)",
            scan_store.len(),
            self.line_store.lines().len(),
            associations.total_associated,
            associations.association_rate() * 100.0
        );

        // Build robot pose map for weighted fitting
        let robot_poses: std::collections::HashMap<u32, Pose2D> = scan_store
            .iter()
            .map(|scan| (scan.scan_id, scan.estimated_pose))
            .collect();

        // Build refit config from exploration config
        let refit_config = RefitConfig::default()
            .with_min_points(exploration_config.min_points_per_line)
            .with_gap_threshold(exploration_config.gap_threshold)
            .with_noise_model(exploration_config.noise_model);

        // Re-fit each line
        let mut stats = RefitStats::default();
        let mut new_lines: Vec<Line2D> = Vec::new();

        for (line_idx, associated_points) in associations.line_points.iter().enumerate() {
            let original = &self.line_store.lines()[line_idx];

            let result = refit_line(associated_points, original, &robot_poses, &refit_config);

            match result {
                RefitResult::Single(line) => {
                    new_lines.push(line);
                    stats.lines_refitted_single += 1;
                }
                RefitResult::Split(lines) => {
                    stats.total_segments_created += lines.len();
                    new_lines.extend(lines);
                    stats.lines_split += 1;
                }
                RefitResult::Insufficient => {
                    // Keep original line
                    new_lines.push(*original);
                    stats.lines_insufficient += 1;
                }
            }
        }

        // Replace all lines and rebuild
        *self.line_store.lines_mut() = new_lines;
        self.line_store.rebuild_all();
        self.update_corners();

        log::debug!(
            "Refit complete: {} single, {} split, {} insufficient, {} new segments",
            stats.lines_refitted_single,
            stats.lines_split,
            stats.lines_insufficient,
            stats.total_segments_created
        );

        Some(stats)
    }
}

impl Map for VectorMap {
    fn observe(&mut self, scan: &PointCloud2D, odometry: Pose2D) -> ObserveResult {
        self.process_scan(scan, odometry)
    }

    fn raycast(&self, from: Point2D, direction: Point2D, max_range: f32) -> f32 {
        raycast(from, direction, max_range, self.line_store.lines())
    }

    fn query(&self, point: Point2D) -> Occupancy {
        query_occupancy(
            point,
            self.line_store.lines(),
            self.line_store.bounds(),
            &self.config.occupancy,
        )
    }

    fn frontiers(&self) -> Vec<Frontier> {
        // Use current robot pose for distance-based filtering
        detect_frontiers(
            self.line_store.lines(),
            self.current_pose.position(),
            &self.config.frontier,
        )
    }

    fn get_path(&self, from: Point2D, to: Point2D) -> Option<Path> {
        let planner = PathPlanner::new(self.config.path_planning.clone());
        planner.plan(
            from,
            to,
            self.line_store.lines(),
            self.line_store.bounds(),
            &self.config.occupancy,
        )
    }

    fn is_path_clear(&self, from: Point2D, to: Point2D) -> bool {
        is_straight_path_clear(
            from,
            to,
            self.line_store.lines(),
            self.config.path_planning.robot_radius,
        )
    }

    fn bounds(&self) -> Bounds {
        self.line_store
            .bounds()
            .cloned()
            .unwrap_or_else(Bounds::empty)
    }

    fn clear(&mut self) {
        self.line_store.clear();
        self.corners.clear();
        self.current_pose = Pose2D::identity();
        self.observation_count = 0;
        self.loop_detector.clear();
        self.loop_closures.clear();
        self.icp_matcher.update_last_confidence(1.0);
    }
}

impl Default for VectorMap {
    fn default() -> Self {
        Self::new(VectorMapConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_simple_scan() -> PointCloud2D {
        let mut xs = Vec::new();
        let mut ys = Vec::new();

        for i in -10..=10 {
            xs.push(2.0);
            ys.push(i as f32 * 0.1);
        }

        PointCloud2D { xs, ys }
    }

    fn make_room_scan() -> PointCloud2D {
        let mut xs = Vec::new();
        let mut ys = Vec::new();

        for i in -10..=10 {
            let y = i as f32 * 0.2;
            xs.push(-2.0);
            ys.push(y);
            xs.push(2.0);
            ys.push(y);
        }
        for i in -10..=10 {
            let x = i as f32 * 0.2;
            xs.push(x);
            ys.push(-2.0);
            xs.push(x);
            ys.push(2.0);
        }

        PointCloud2D { xs, ys }
    }

    #[test]
    fn test_new_map() {
        let config = VectorMapConfig::default();
        let map = VectorMap::new(config);

        assert!(map.lines().is_empty());
        assert!(map.corners.is_empty());
        assert_eq!(map.observation_count, 0);
    }

    #[test]
    fn test_add_line() {
        let mut map = VectorMap::default();
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));

        map.add_line(line);

        assert_eq!(map.lines().len(), 1);
        assert!(map.line_store.bounds().is_some());
    }

    #[test]
    fn test_first_observation() {
        let mut map = VectorMap::default();
        let scan = make_simple_scan();

        let result = map.observe(&scan, Pose2D::identity());

        assert_eq!(result.pose, Pose2D::identity());
        assert!(map.lines().len() > 0 || result.features_extracted > 0);
        assert_eq!(map.observation_count, 1);
    }

    #[test]
    fn test_raycast_through_map() {
        let mut map = VectorMap::default();
        map.add_line(Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0)));

        let dist = map.raycast(Point2D::zero(), Point2D::new(1.0, 0.0), 100.0);

        assert!((dist - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_query_occupancy() {
        let mut map = VectorMap::default();
        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)));

        let on_line = map.query(Point2D::new(5.0, 0.0));
        assert_eq!(on_line, Occupancy::Occupied);
    }

    #[test]
    fn test_bounds() {
        let mut map = VectorMap::default();

        let empty_bounds = map.bounds();
        assert!(empty_bounds.is_empty());

        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 5.0)));

        let bounds = map.bounds();
        assert!(!bounds.is_empty());
        assert!(bounds.contains(Point2D::new(5.0, 2.0)));
    }

    #[test]
    fn test_clear() {
        let mut map = VectorMap::default();
        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)));

        map.clear();

        assert!(map.lines().is_empty());
        assert!(map.corners.is_empty());
        assert!(map.line_store.bounds().is_none());
        assert_eq!(map.current_pose, Pose2D::identity());
    }

    #[test]
    fn test_frontiers_empty_map() {
        let map = VectorMap::default();
        let frontiers = map.frontiers();

        assert!(frontiers.is_empty());
    }

    #[test]
    fn test_frontiers_partial_room() {
        let mut map = VectorMap::default();

        map.add_lines(&[
            Line2D::new(Point2D::new(-2.0, -2.0), Point2D::new(2.0, -2.0)),
            Line2D::new(Point2D::new(2.0, -2.0), Point2D::new(2.0, 2.0)),
            Line2D::new(Point2D::new(-2.0, 2.0), Point2D::new(-2.0, -2.0)),
        ]);

        let frontiers = map.frontiers();
        assert!(!frontiers.is_empty());
    }

    #[test]
    fn test_localization_only_mode() {
        let config = VectorMapConfig::default().with_mapping_enabled(false);
        let mut map = VectorMap::new(config);

        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)));
        let initial_count = map.lines().len();

        let scan = make_simple_scan();
        map.observe(&scan, Pose2D::identity());

        assert_eq!(map.lines().len(), initial_count);
    }

    #[test]
    fn test_features() {
        let mut map = VectorMap::default();
        map.add_lines(&[
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
        ]);

        let features = map.features();

        assert_eq!(features.lines().len(), 2);
    }

    #[test]
    fn test_multi_scan_mapping_grows_map() {
        let mut map = VectorMap::default();

        let mut scan1 = PointCloud2D {
            xs: Vec::new(),
            ys: Vec::new(),
        };
        for i in 0..20 {
            scan1.xs.push(-2.0);
            scan1.ys.push(-2.0 + i as f32 * 0.2);
        }
        for i in 0..20 {
            scan1.xs.push(2.0);
            scan1.ys.push(-2.0 + i as f32 * 0.2);
        }

        let _ = map.observe(&scan1, Pose2D::identity());
        assert_eq!(map.observation_count(), 1);

        let odometry = Pose2D::new(0.3, 0.2, 0.05);
        let _ = map.observe(&scan1, odometry);

        assert_eq!(map.observation_count(), 2);
    }

    #[test]
    fn test_slam_cycle_extract_match_integrate() {
        let mut map = VectorMap::default();

        map.add_lines(&[
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(0.0, 5.0)),
            Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(0.0, 0.0)),
        ]);

        let initial_lines = map.lines().len();

        let mut scan = PointCloud2D {
            xs: Vec::new(),
            ys: Vec::new(),
        };
        for i in 0..10 {
            scan.xs.push(0.5 + i as f32 * 0.4);
            scan.ys.push(0.05);
        }
        for i in 0..10 {
            scan.xs.push(4.95);
            scan.ys.push(0.5 + i as f32 * 0.4);
        }

        map.set_pose(Pose2D::new(2.5, 2.5, 0.0));
        let _ = map.observe(&scan, Pose2D::identity());

        assert!(
            map.lines().len() <= initial_lines + 5,
            "Map should not grow excessively: had {}, now {}",
            initial_lines,
            map.lines().len()
        );
    }

    #[test]
    fn test_localization_maintains_pose() {
        let config = VectorMapConfig::default().with_min_match_confidence(0.1);
        let mut map = VectorMap::new(config);

        map.add_lines(&[
            Line2D::new(Point2D::new(-2.0, -2.0), Point2D::new(2.0, -2.0)),
            Line2D::new(Point2D::new(2.0, -2.0), Point2D::new(2.0, 2.0)),
            Line2D::new(Point2D::new(2.0, 2.0), Point2D::new(-2.0, 2.0)),
            Line2D::new(Point2D::new(-2.0, 2.0), Point2D::new(-2.0, -2.0)),
        ]);

        map.set_pose(Pose2D::identity());

        let scan = make_room_scan();
        let mut total_movement = 0.0;

        for _ in 0..5 {
            let odom = Pose2D::new(0.1, 0.05, 0.02);
            map.observe(&scan, odom);
            total_movement += (0.1f32.powi(2) + 0.05f32.powi(2)).sqrt();
        }

        let pose = map.current_pose();
        let pose_dist = (pose.x.powi(2) + pose.y.powi(2)).sqrt();

        assert!(
            pose_dist < total_movement * 2.0,
            "Pose drift too large: {} vs max {}",
            pose_dist,
            total_movement * 2.0
        );
    }

    #[test]
    fn test_map_bounds_grow_with_exploration() {
        let mut map = VectorMap::default();

        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)));
        let initial_bounds = map.bounds();

        map.add_line(Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(5.0, 0.0)));
        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 3.0)));

        let expanded_bounds = map.bounds();

        assert!(
            expanded_bounds.width() > initial_bounds.width(),
            "Bounds width should grow"
        );
        assert!(
            expanded_bounds.height() > initial_bounds.height(),
            "Bounds height should grow"
        );
    }

    #[test]
    fn test_feature_count_stability() {
        let mut map = VectorMap::default();
        let scan = make_room_scan();

        map.observe(&scan, Pose2D::identity());
        let count_after_1 = map.lines().len();

        for _ in 0..5 {
            let small_move = Pose2D::new(0.01, 0.01, 0.005);
            map.observe(&scan, small_move);
        }

        let count_after_6 = map.lines().len();

        assert!(
            count_after_6 <= count_after_1 * 3,
            "Feature count should stabilize: started with {}, ended with {}",
            count_after_1,
            count_after_6
        );
    }

    #[test]
    fn test_config_propagation() {
        let config = VectorMapConfig::default()
            .with_min_match_confidence(0.5)
            .with_mapping_enabled(false);

        let map = VectorMap::new(config);

        assert_eq!(map.config().min_match_confidence, 0.5);
        assert!(!map.config().mapping_enabled);
    }

    #[test]
    fn test_set_pose() {
        let mut map = VectorMap::default();

        let target_pose = Pose2D::new(1.5, 2.5, 0.7);
        map.set_pose(target_pose);

        assert_eq!(map.current_pose().x, target_pose.x);
        assert_eq!(map.current_pose().y, target_pose.y);
        assert_eq!(map.current_pose().theta, target_pose.theta);
    }

    #[test]
    fn test_observation_count_increments() {
        let mut map = VectorMap::default();
        let scan = make_simple_scan();

        assert_eq!(map.observation_count(), 0);

        map.observe(&scan, Pose2D::identity());
        assert_eq!(map.observation_count(), 1);

        map.observe(&scan, Pose2D::identity());
        assert_eq!(map.observation_count(), 2);

        map.observe(&scan, Pose2D::identity());
        assert_eq!(map.observation_count(), 3);
    }

    #[test]
    fn test_add_lines_batch() {
        let mut map = VectorMap::default();

        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(1.0, 1.0)),
            Line2D::new(Point2D::new(1.0, 1.0), Point2D::new(0.0, 1.0)),
        ];

        map.add_lines(&lines);

        assert_eq!(map.lines().len(), 3);
        assert!(map.bounds().contains(Point2D::new(0.5, 0.5)));
    }

    #[test]
    fn test_empty_scan_handling() {
        let mut map = VectorMap::default();
        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)));

        let empty_scan = PointCloud2D {
            xs: vec![],
            ys: vec![],
        };
        let result = map.observe(&empty_scan, Pose2D::identity());

        assert_eq!(result.features_extracted, 0);
        assert_eq!(map.observation_count(), 1);
    }
}
