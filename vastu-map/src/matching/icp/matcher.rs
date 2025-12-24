//! Point-to-Line ICP matcher implementation.

use crate::core::{Point2D, Pose2D};
use crate::features::{Line2D, LineCollection, LineSpatialIndex, should_use_spatial_index};

use super::super::correspondence::{
    CorrespondenceSet, IDENTITY_COVARIANCE, MatchResult, PoseCovariance,
};
use super::super::gauss_newton::{GaussNewtonConfig, optimize_pose, optimize_pose_fast};
use super::super::nearest_neighbor::NearestNeighborConfig;
use super::super::ransac::RansacConfig;
use super::super::ransac::estimate_pose_ransac;
use super::super::scratch::IcpScratchSpace;
use super::config::{CoarseSearchConfig, IcpConfig};

/// Point-to-Line ICP matcher.
///
/// Holds configuration and provides scan matching functionality.
/// Includes internal scratch space for zero-allocation matching.
pub struct PointToLineIcp {
    /// ICP configuration.
    config: IcpConfig,
    /// Nearest neighbor configuration.
    nn_config: NearestNeighborConfig,
    /// Gauss-Newton configuration.
    gn_config: GaussNewtonConfig,
    /// Coarse search configuration.
    coarse_config: CoarseSearchConfig,
    /// Last match confidence for adaptive coarse search.
    last_confidence: f32,
    /// Pre-allocated scratch space for zero-allocation matching.
    scratch: IcpScratchSpace,
}

impl PointToLineIcp {
    /// Create a new ICP matcher with default configuration.
    pub fn new() -> Self {
        Self::with_config(IcpConfig::default())
    }

    /// Create a new ICP matcher with custom configuration.
    pub fn with_config(config: IcpConfig) -> Self {
        let nn_config = NearestNeighborConfig::default()
            .with_max_distance(config.max_correspondence_distance)
            .with_unique_per_point(true);

        let gn_config = GaussNewtonConfig::default()
            .with_max_iterations(10)
            .with_convergence_threshold(config.convergence_threshold / 10.0);

        Self {
            config,
            nn_config,
            gn_config,
            coarse_config: CoarseSearchConfig::default(),
            last_confidence: 1.0, // Start with high confidence (no coarse search first time)
            scratch: IcpScratchSpace::default_capacity(), // 360 points, 200 lines
        }
    }

    /// Update the last match confidence (for adaptive coarse search).
    pub fn update_last_confidence(&mut self, confidence: f32) {
        self.last_confidence = confidence;
    }

    /// Check if coarse search should be used.
    fn should_use_coarse_search(&self) -> bool {
        self.config.use_coarse_search
            && self.last_confidence < self.config.coarse_search_confidence_threshold
    }

    /// Perform coarse search to find a better initial pose.
    ///
    /// Searches a grid of poses around the initial pose and returns
    /// the one with the lowest total point-to-line distance.
    fn coarse_search(
        &self,
        points: &[Point2D],
        lines: &LineCollection,
        initial_pose: Pose2D,
    ) -> Pose2D {
        if lines.is_empty() || points.is_empty() {
            return initial_pose;
        }

        let cfg = &self.coarse_config;

        // Compute number of steps in each dimension
        let t_steps = ((2.0 * cfg.translation_range / cfg.translation_step) as usize).max(1);
        let r_steps = ((2.0 * cfg.rotation_range / cfg.rotation_step) as usize).max(1);

        // Subsample points for fast evaluation
        let subsampled_points: Vec<&Point2D> =
            points.iter().step_by(cfg.subsample_rate.max(1)).collect();

        let mut best_pose = initial_pose;
        let mut best_score = f32::MAX;

        for tx_i in 0..=t_steps {
            let tx = initial_pose.x - cfg.translation_range + (tx_i as f32 * cfg.translation_step);

            for ty_i in 0..=t_steps {
                let ty =
                    initial_pose.y - cfg.translation_range + (ty_i as f32 * cfg.translation_step);

                for r_i in 0..=r_steps {
                    let theta =
                        initial_pose.theta - cfg.rotation_range + (r_i as f32 * cfg.rotation_step);

                    let test_pose = Pose2D::new(tx, ty, theta);
                    let score = self.score_pose(&subsampled_points, lines, &test_pose);

                    if score < best_score {
                        best_score = score;
                        best_pose = test_pose;
                    }
                }
            }
        }

        best_pose
    }

    /// Score a pose hypothesis by summing squared point-to-line distances.
    fn score_pose(&self, points: &[&Point2D], lines: &LineCollection, pose: &Pose2D) -> f32 {
        let (sin, cos) = pose.theta.sin_cos();
        let mut sum_sq = 0.0f32;

        for &point in points {
            // Transform point by pose
            let tx = point.x * cos - point.y * sin + pose.x;
            let ty = point.x * sin + point.y * cos + pose.y;

            // Find minimum distance to any line
            let min_dist_sq = lines
                .distances_to_point(Point2D::new(tx, ty))
                .into_iter()
                .map(|d| d * d)
                .fold(f32::MAX, f32::min);

            // Clamp to avoid outliers dominating
            sum_sq += min_dist_sq.min(1.0);
        }

        sum_sq
    }

    /// Match scan points against map lines.
    ///
    /// # Arguments
    /// * `points` - Scan points in world frame (after initial transform)
    /// * `lines` - Map lines
    /// * `initial_pose` - Initial pose estimate (typically from odometry)
    ///
    /// # Returns
    /// Match result with optimized pose, covariance, and confidence.
    pub fn match_scan(
        &mut self,
        points: &[Point2D],
        lines: &[Line2D],
        initial_pose: Pose2D,
    ) -> MatchResult {
        if points.is_empty() || lines.is_empty() {
            return MatchResult::new(initial_pose, CorrespondenceSet::new(), 0, false);
        }

        // Convert lines to collection for batch operations
        let line_collection = LineCollection::from_lines(lines);

        // Only use spatial index for large maps
        let use_spatial = should_use_spatial_index(&line_collection);
        let spatial_index = if use_spatial {
            Some(
                line_collection
                    .build_spatial_index_for_radius(self.config.max_correspondence_distance),
            )
        } else {
            None
        };

        let mut pose = initial_pose;
        let mut prev_pose = pose;
        let mut final_correspondences = CorrespondenceSet::new();
        let mut converged = false;
        let mut used_coarse_search = false;
        let mut final_covariance: PoseCovariance = IDENTITY_COVARIANCE;
        let mut final_condition_number = 1.0f32;

        // Optional coarse search when confidence is low
        if self.should_use_coarse_search() {
            pose = self.coarse_search(points, &line_collection, initial_pose);
            used_coarse_search = true;
        }

        // Optional RANSAC initialization (after coarse search if used)
        if self.config.use_ransac_init {
            let initial_corrs = self.find_correspondences_for_pose(
                points,
                &line_collection,
                spatial_index.as_ref(),
                &pose,
            );

            if initial_corrs.len() >= self.config.min_correspondences {
                let ransac_config = RansacConfig::default()
                    .with_max_iterations(50)
                    .with_inlier_threshold(0.15);

                if let Some(ransac_result) =
                    estimate_pose_ransac(&initial_corrs, lines, &ransac_config)
                {
                    pose = pose.compose(ransac_result.pose);
                }
            }
        }

        // Main ICP loop
        let mut final_iteration = 0;
        for iteration in 0..self.config.max_iterations {
            final_iteration = iteration + 1;

            // 1. Find correspondences at current pose
            let correspondences = self.find_correspondences_for_pose(
                points,
                &line_collection,
                spatial_index.as_ref(),
                &pose,
            );

            if correspondences.len() < self.config.min_correspondences {
                // Not enough correspondences - return with low confidence
                return MatchResult::with_covariance(
                    pose,
                    correspondences,
                    final_iteration,
                    false,
                    final_covariance,
                    final_condition_number,
                    used_coarse_search,
                );
            }

            // 2. Filter by max distance and apply IRLS weights
            let mut filtered = self.filter_by_max_distance(&correspondences);
            self.apply_irls_weights(&mut filtered);

            if filtered.len() < self.config.min_correspondences {
                return MatchResult::with_covariance(
                    pose,
                    filtered,
                    final_iteration,
                    false,
                    final_covariance,
                    final_condition_number,
                    used_coarse_search,
                );
            }

            // 3. Optimize pose using Gauss-Newton with weighted correspondences
            let gn_result = optimize_pose(&filtered, lines, Pose2D::identity(), &self.gn_config);
            final_covariance = gn_result.covariance;
            final_condition_number = gn_result.condition_number;

            // 4. Update pose (compose with delta)
            pose = pose.compose(gn_result.pose);

            // 5. Check convergence
            // Using squared distance comparison to avoid sqrt()
            let dx = pose.x - prev_pose.x;
            let dy = pose.y - prev_pose.y;
            let delta_trans_sq = dx * dx + dy * dy;
            let delta_rot = crate::core::math::angle_diff(pose.theta, prev_pose.theta).abs();

            if delta_trans_sq < self.config.convergence_threshold_sq
                && delta_rot < self.config.convergence_threshold
            {
                converged = true;
                final_correspondences = filtered;
                break;
            }

            prev_pose = pose;
            final_correspondences = filtered;
        }

        MatchResult::with_covariance(
            pose,
            final_correspondences,
            final_iteration,
            converged,
            final_covariance,
            final_condition_number,
            used_coarse_search,
        )
    }

    /// Match scan against line collection (optimized for large maps).
    pub fn match_scan_batch(
        &mut self,
        points: &[Point2D],
        lines: &LineCollection,
        initial_pose: Pose2D,
    ) -> MatchResult {
        let lines_vec = lines.to_lines();
        self.match_scan(points, &lines_vec, initial_pose)
    }

    /// Match scan with range-based weighted correspondences.
    ///
    /// This method takes points in robot frame and applies proper uncertainty-based
    /// weighting using the configured noise model. Close-range measurements get
    /// higher weight than far-range ones.
    ///
    /// # Arguments
    /// * `robot_frame_points` - Scan points in robot frame (for weight computation)
    /// * `lines` - Map lines in world frame
    /// * `initial_pose` - Initial pose estimate in world frame (from odometry)
    ///
    /// # Returns
    /// Match result with optimized pose, covariance, and confidence.
    pub fn match_scan_robot_frame(
        &mut self,
        robot_frame_points: &[Point2D],
        lines: &[Line2D],
        initial_pose: Pose2D,
    ) -> MatchResult {
        if robot_frame_points.is_empty() || lines.is_empty() {
            return MatchResult::new(initial_pose, CorrespondenceSet::new(), 0, false);
        }

        // Use multi-resolution ICP if enabled
        if self.config.multi_resolution.enabled {
            return match_scan_multiresolution(
                robot_frame_points,
                lines,
                initial_pose,
                &self.config.multi_resolution,
                &self.config,
            );
        }

        // Convert lines to collection for batch operations
        let line_collection = LineCollection::from_lines(lines);
        self.match_scan_robot_frame_impl(robot_frame_points, lines, &line_collection, initial_pose)
    }

    /// Match scan with range-based weighted correspondences using cached LineCollection.
    ///
    /// This is the preferred method when the LineCollection is already cached
    /// (e.g., in LineStore), avoiding re-computation of the SoA structure.
    ///
    /// # Arguments
    /// * `robot_frame_points` - Scan points in robot frame (for weight computation)
    /// * `line_collection` - Cached LineCollection from LineStore
    /// * `initial_pose` - Initial pose estimate in world frame (from odometry)
    ///
    /// # Returns
    /// Match result with optimized pose, covariance, and confidence.
    pub fn match_scan_robot_frame_collection(
        &mut self,
        robot_frame_points: &[Point2D],
        line_collection: &LineCollection,
        initial_pose: Pose2D,
    ) -> MatchResult {
        if robot_frame_points.is_empty() || line_collection.is_empty() {
            return MatchResult::new(initial_pose, CorrespondenceSet::new(), 0, false);
        }

        // Convert to lines for RANSAC/GN (they require line endpoints)
        let lines = line_collection.to_lines();

        // Use multi-resolution ICP if enabled
        if self.config.multi_resolution.enabled {
            return match_scan_multiresolution(
                robot_frame_points,
                &lines,
                initial_pose,
                &self.config.multi_resolution,
                &self.config,
            );
        }

        self.match_scan_robot_frame_impl(robot_frame_points, &lines, line_collection, initial_pose)
    }

    /// Internal implementation for robot-frame matching.
    fn match_scan_robot_frame_impl(
        &mut self,
        robot_frame_points: &[Point2D],
        lines: &[Line2D],
        line_collection: &LineCollection,
        initial_pose: Pose2D,
    ) -> MatchResult {
        // Only use spatial index for large maps
        let use_spatial = should_use_spatial_index(line_collection);
        let spatial_index = if use_spatial {
            Some(
                line_collection
                    .build_spatial_index_for_radius(self.config.max_correspondence_distance),
            )
        } else {
            None
        };

        let mut pose = initial_pose;
        let mut prev_pose = pose;
        let mut final_correspondences = CorrespondenceSet::new();
        let mut converged = false;
        let mut used_coarse_search = false;
        let mut final_covariance: PoseCovariance = IDENTITY_COVARIANCE;
        let mut final_condition_number = 1.0f32;

        // Keep points in robot frame - find_correspondences_for_pose will transform them
        // using the current pose estimate during each ICP iteration

        // Optional coarse search when confidence is low
        // Note: coarse_search expects robot-frame points and transforms them internally
        if self.should_use_coarse_search() {
            pose = self.coarse_search(robot_frame_points, line_collection, initial_pose);
            used_coarse_search = true;
        }

        // Optional RANSAC initialization
        if self.config.use_ransac_init {
            let mut initial_corrs = self.find_correspondences_for_pose(
                robot_frame_points,
                line_collection,
                spatial_index.as_ref(),
                &pose,
            );
            // Apply weights based on robot-frame ranges
            initial_corrs.apply_weights(robot_frame_points, &self.config.noise_model);

            if initial_corrs.len() >= self.config.min_correspondences {
                let ransac_config = RansacConfig::default()
                    .with_max_iterations(50)
                    .with_inlier_threshold(0.15);

                if let Some(ransac_result) =
                    estimate_pose_ransac(&initial_corrs, lines, &ransac_config)
                {
                    pose = pose.compose(ransac_result.pose);
                }
            }
        }

        // Main ICP loop
        let mut final_iteration = 0;
        for iteration in 0..self.config.max_iterations {
            final_iteration = iteration + 1;

            // 1. Find correspondences at current pose
            // Pass robot-frame points - find_correspondences_for_pose transforms them internally
            let mut correspondences = self.find_correspondences_for_pose(
                robot_frame_points,
                line_collection,
                spatial_index.as_ref(),
                &pose,
            );

            // Apply weights based on robot-frame ranges
            correspondences.apply_weights(robot_frame_points, &self.config.noise_model);

            if correspondences.len() < self.config.min_correspondences {
                return MatchResult::with_covariance(
                    pose,
                    correspondences,
                    final_iteration,
                    false,
                    final_covariance,
                    final_condition_number,
                    used_coarse_search,
                );
            }

            // 2. Filter by max distance and apply IRLS weights
            // Note: Range-based weights already applied above via apply_weights()
            let mut filtered = self.filter_by_max_distance(&correspondences);
            self.apply_irls_weights(&mut filtered);

            if filtered.len() < self.config.min_correspondences {
                return MatchResult::with_covariance(
                    pose,
                    filtered,
                    final_iteration,
                    false,
                    final_covariance,
                    final_condition_number,
                    used_coarse_search,
                );
            }

            // 3. Optimize pose using Gauss-Newton with weighted correspondences
            let gn_result = optimize_pose(&filtered, lines, Pose2D::identity(), &self.gn_config);
            final_covariance = gn_result.covariance;
            final_condition_number = gn_result.condition_number;

            // 4. Update pose (compose with delta)
            pose = pose.compose(gn_result.pose);

            // 5. Check convergence
            // Using squared distance comparison to avoid sqrt()
            let dx = pose.x - prev_pose.x;
            let dy = pose.y - prev_pose.y;
            let delta_trans_sq = dx * dx + dy * dy;
            let delta_rot = crate::core::math::angle_diff(pose.theta, prev_pose.theta).abs();

            if delta_trans_sq < self.config.convergence_threshold_sq
                && delta_rot < self.config.convergence_threshold
            {
                converged = true;
                final_correspondences = filtered;
                break;
            }

            prev_pose = pose;
            final_correspondences = filtered;
        }

        MatchResult::with_covariance(
            pose,
            final_correspondences,
            final_iteration,
            converged,
            final_covariance,
            final_condition_number,
            used_coarse_search,
        )
    }

    /// Match scan using pre-allocated scratch space (zero-allocation hot path).
    ///
    /// This method automatically uses spatial indexing for O(n×k) correspondence
    /// search instead of O(n×m) brute force, providing significant speedup for
    /// maps with many lines.
    ///
    /// # Arguments
    /// * `points` - Scan points in robot frame
    /// * `lines` - Map lines as LineCollection (for SIMD operations)
    /// * `initial_pose` - Initial pose estimate (typically from odometry)
    /// * `scratch` - Pre-allocated scratch space for buffer reuse
    ///
    /// # Returns
    /// Match result with optimized pose and confidence.
    ///
    /// # Example
    /// ```rust,ignore
    /// let mut scratch = IcpScratchSpace::default_capacity();
    /// let mut icp = PointToLineIcp::new();
    ///
    /// // Reuse scratch across multiple calls
    /// for (scan, odom_pose) in scans.iter().zip(odometry.iter()) {
    ///     let result = icp.match_scan_with_scratch(&scan, &map_lines, *odom_pose, &mut scratch);
    ///     if result.converged {
    ///         // Use result.pose
    ///     }
    /// }
    /// ```
    pub fn match_scan_with_scratch(
        &self,
        points: &[Point2D],
        lines: &LineCollection,
        initial_pose: Pose2D,
        scratch: &mut IcpScratchSpace,
    ) -> MatchResult {
        if points.is_empty() || lines.is_empty() {
            return MatchResult::new(initial_pose, CorrespondenceSet::new(), 0, false);
        }

        // Only use spatial index for large maps (> 30 lines and > 8m extent)
        // For small rooms, brute-force is faster due to lower overhead
        let use_spatial = should_use_spatial_index(lines);
        let spatial_index = if use_spatial {
            Some(lines.build_spatial_index_for_radius(self.config.max_correspondence_distance))
        } else {
            None
        };

        let mut pose = initial_pose;
        let mut prev_pose = pose;
        let mut final_correspondences = CorrespondenceSet::new();
        let mut converged = false;
        let mut used_coarse_search = false;
        let mut final_covariance: PoseCovariance = IDENTITY_COVARIANCE;
        let mut final_condition_number = 1.0f32;

        // Optional coarse search when confidence is low
        if self.should_use_coarse_search() {
            pose = self.coarse_search(points, lines, initial_pose);
            used_coarse_search = true;
        }

        // Main ICP loop
        let mut final_iteration = 0;
        for iteration in 0..self.config.max_iterations {
            final_iteration = iteration + 1;

            // 1. Transform points and find correspondences
            let (sin, cos) = pose.theta.sin_cos();
            scratch.transform_points(points, sin, cos, pose.x, pose.y);

            // Use spatial index for large maps, brute force for small maps
            if let Some(ref index) = spatial_index {
                scratch.find_correspondences_indexed(lines, index, &self.nn_config);
            } else {
                scratch.find_correspondences(lines, &self.nn_config);
            }

            if scratch.num_correspondences() < self.config.min_correspondences {
                // Not enough correspondences - return with low confidence
                return MatchResult::with_covariance(
                    pose,
                    scratch.take_correspondences(),
                    final_iteration,
                    false,
                    final_covariance,
                    final_condition_number,
                    used_coarse_search,
                );
            }

            // 2. Get correspondences, filter by distance, and apply IRLS weights
            let correspondences = scratch.correspondences();
            let mut filtered = self.filter_by_max_distance(correspondences);
            self.apply_irls_weights(&mut filtered);

            if filtered.len() < self.config.min_correspondences {
                return MatchResult::with_covariance(
                    pose,
                    filtered,
                    final_iteration,
                    false,
                    final_covariance,
                    final_condition_number,
                    used_coarse_search,
                );
            }

            // 3. Optimize pose using fast Gauss-Newton with weighted correspondences
            let gn_result =
                optimize_pose_fast(&filtered, lines, Pose2D::identity(), &self.gn_config);
            final_covariance = gn_result.covariance;
            final_condition_number = gn_result.condition_number;

            // 4. Update pose (compose with delta)
            pose = pose.compose(gn_result.pose);

            // 5. Check convergence
            // Using squared distance comparison to avoid sqrt()
            let dx = pose.x - prev_pose.x;
            let dy = pose.y - prev_pose.y;
            let delta_trans_sq = dx * dx + dy * dy;
            let delta_rot = crate::core::math::angle_diff(pose.theta, prev_pose.theta).abs();

            if delta_trans_sq < self.config.convergence_threshold_sq
                && delta_rot < self.config.convergence_threshold
            {
                converged = true;
                final_correspondences = filtered;
                break;
            }

            prev_pose = pose;
            final_correspondences = filtered;
        }

        MatchResult::with_covariance(
            pose,
            final_correspondences,
            final_iteration,
            converged,
            final_covariance,
            final_condition_number,
            used_coarse_search,
        )
    }

    /// Match scan using spatial index for O(n×k) correspondence search.
    ///
    /// This is the fastest matching method for large maps, using spatial hashing
    /// to reduce correspondence search from O(n×m) to O(n×k) where k << m.
    ///
    /// # Arguments
    /// * `points` - Scan points in robot frame
    /// * `lines` - Map lines as LineCollection (for SIMD operations)
    /// * `spatial_index` - Pre-built spatial index for fast line lookup
    /// * `initial_pose` - Initial pose estimate (typically from odometry)
    /// * `scratch` - Pre-allocated scratch space for buffer reuse
    ///
    /// # Performance
    /// For a map with 100 lines and 360 points:
    /// - Standard ICP: ~36,000 distance calculations per iteration
    /// - With spatial index: ~3,600 (assuming ~10 candidates per point)
    ///
    /// # Example
    /// ```rust,ignore
    /// let mut scratch = IcpScratchSpace::default_capacity();
    /// let mut icp = PointToLineIcp::new();
    ///
    /// // Build spatial index once when map changes
    /// let spatial_index = map_lines.build_spatial_index_for_radius(0.5);
    ///
    /// // Reuse across multiple scans
    /// for (scan, odom_pose) in scans.iter().zip(odometry.iter()) {
    ///     let result = icp.match_scan_indexed(
    ///         &scan, &map_lines, &spatial_index, *odom_pose, &mut scratch
    ///     );
    /// }
    /// ```
    pub fn match_scan_indexed(
        &self,
        points: &[Point2D],
        lines: &LineCollection,
        spatial_index: &LineSpatialIndex,
        initial_pose: Pose2D,
        scratch: &mut IcpScratchSpace,
    ) -> MatchResult {
        if points.is_empty() || lines.is_empty() {
            return MatchResult::new(initial_pose, CorrespondenceSet::new(), 0, false);
        }

        let mut pose = initial_pose;
        let mut prev_pose = pose;
        let mut final_correspondences = CorrespondenceSet::new();
        let mut converged = false;
        let used_coarse_search = false; // Coarse search not supported with spatial index yet
        let mut final_covariance: PoseCovariance = IDENTITY_COVARIANCE;
        let mut final_condition_number = 1.0f32;

        // Main ICP loop with spatial-indexed correspondence search
        let mut final_iteration = 0;
        for iteration in 0..self.config.max_iterations {
            final_iteration = iteration + 1;

            // 1. Transform points and find correspondences using spatial index
            let (sin, cos) = pose.theta.sin_cos();
            scratch.transform_points(points, sin, cos, pose.x, pose.y);
            scratch.find_correspondences_indexed(lines, spatial_index, &self.nn_config);

            if scratch.num_correspondences() < self.config.min_correspondences {
                return MatchResult::with_covariance(
                    pose,
                    scratch.take_correspondences(),
                    final_iteration,
                    false,
                    final_covariance,
                    final_condition_number,
                    used_coarse_search,
                );
            }

            // 2. Get correspondences, filter by distance, and apply IRLS weights
            let correspondences = scratch.correspondences();
            let mut filtered = self.filter_by_max_distance(correspondences);
            self.apply_irls_weights(&mut filtered);

            if filtered.len() < self.config.min_correspondences {
                return MatchResult::with_covariance(
                    pose,
                    filtered,
                    final_iteration,
                    false,
                    final_covariance,
                    final_condition_number,
                    used_coarse_search,
                );
            }

            // 3. Optimize pose using fast Gauss-Newton with weighted correspondences
            let gn_result =
                optimize_pose_fast(&filtered, lines, Pose2D::identity(), &self.gn_config);
            final_covariance = gn_result.covariance;
            final_condition_number = gn_result.condition_number;

            // 4. Update pose (compose with delta)
            pose = pose.compose(gn_result.pose);

            // 5. Check convergence
            let dx = pose.x - prev_pose.x;
            let dy = pose.y - prev_pose.y;
            let delta_trans_sq = dx * dx + dy * dy;
            let delta_rot = crate::core::math::angle_diff(pose.theta, prev_pose.theta).abs();

            if delta_trans_sq < self.config.convergence_threshold_sq
                && delta_rot < self.config.convergence_threshold
            {
                converged = true;
                final_correspondences = filtered;
                break;
            }

            prev_pose = pose;
            final_correspondences = filtered;
        }

        MatchResult::with_covariance(
            pose,
            final_correspondences,
            final_iteration,
            converged,
            final_covariance,
            final_condition_number,
            used_coarse_search,
        )
    }

    /// Find correspondences for points at a given pose using scratch space.
    ///
    /// Uses internal scratch space for zero-allocation transformation.
    /// Uses spatial indexing if provided, otherwise falls back to brute force.
    fn find_correspondences_for_pose(
        &mut self,
        points: &[Point2D],
        line_collection: &LineCollection,
        spatial_index: Option<&LineSpatialIndex>,
        pose: &Pose2D,
    ) -> CorrespondenceSet {
        // Transform points by pose into scratch space (zero-allocation)
        let (sin, cos) = pose.theta.sin_cos();
        self.scratch
            .transform_points(points, sin, cos, pose.x, pose.y);

        // Find correspondences - use spatial index if available, else brute force
        if let Some(index) = spatial_index {
            self.scratch
                .find_correspondences_indexed(line_collection, index, &self.nn_config);
        } else {
            self.scratch
                .find_correspondences(line_collection, &self.nn_config);
        }

        // Return a clone of the correspondences (scratch needs to be reusable)
        self.scratch.correspondences().clone()
    }

    /// Apply IRLS robust weights to correspondences.
    ///
    /// Multiplies each correspondence weight by a robust weight computed
    /// from its residual (distance). This down-weights outliers softly
    /// rather than removing them entirely.
    fn apply_irls_weights(&self, correspondences: &mut CorrespondenceSet) {
        if !self.config.use_irls || self.config.robust_cost.is_none() {
            return;
        }

        for corr in correspondences.iter_mut() {
            let robust_weight = self.config.robust_cost.weight(corr.distance);
            corr.weight *= robust_weight; // Multiply range-based weight with robust weight
        }
    }

    /// Filter correspondences by max correspondence distance.
    ///
    /// This is a hard filter applied before IRLS weighting to remove
    /// obviously bad correspondences (beyond max search distance).
    fn filter_by_max_distance(&self, correspondences: &CorrespondenceSet) -> CorrespondenceSet {
        correspondences.filter_by_distance(self.config.max_correspondence_distance)
    }
}

impl Default for PointToLineIcp {
    fn default() -> Self {
        Self::new()
    }
}

/// Convenience function for single-shot scan matching.
///
/// # Arguments
/// * `points` - Scan points in robot frame
/// * `lines` - Map lines in world frame
/// * `initial_pose` - Initial robot pose in world frame
///
/// # Returns
/// Match result with optimized pose.
pub fn match_scan(points: &[Point2D], lines: &[Line2D], initial_pose: Pose2D) -> MatchResult {
    let mut icp = PointToLineIcp::new();
    icp.match_scan(points, lines, initial_pose)
}

/// Match scan with custom configuration.
pub fn match_scan_with_config(
    points: &[Point2D],
    lines: &[Line2D],
    initial_pose: Pose2D,
    config: IcpConfig,
) -> MatchResult {
    let mut icp = PointToLineIcp::with_config(config);
    icp.match_scan(points, lines, initial_pose)
}

/// Match scan using multi-resolution (coarse-to-fine) ICP.
///
/// This method improves convergence by starting with subsampled points
/// (coarse level) and progressively refining with more points (fine level).
/// Useful when the initial pose guess may be poor.
///
/// # Arguments
/// * `points` - Scan points in robot frame
/// * `lines` - Map lines in world frame
/// * `initial_pose` - Initial pose estimate
/// * `multi_config` - Multi-resolution pyramid configuration
/// * `icp_config` - Base ICP configuration (iterations overridden per level)
///
/// # Returns
/// Match result with optimized pose from finest level.
///
/// # Example
/// ```rust,ignore
/// use vastu_map::matching::{match_scan_multiresolution, MultiResolutionConfig, IcpConfig};
///
/// let multi_config = MultiResolutionConfig::enabled()
///     .with_levels(3)
///     .with_iterations(vec![3, 5, 10]);
/// let icp_config = IcpConfig::default();
///
/// let result = match_scan_multiresolution(
///     &points, &lines, initial_pose, &multi_config, &icp_config
/// );
/// ```
pub fn match_scan_multiresolution(
    points: &[Point2D],
    lines: &[Line2D],
    initial_pose: Pose2D,
    multi_config: &super::config::MultiResolutionConfig,
    icp_config: &IcpConfig,
) -> MatchResult {
    if points.is_empty() || lines.is_empty() {
        return MatchResult::new(initial_pose, CorrespondenceSet::new(), 0, false);
    }

    // If multi-resolution disabled, fall back to standard ICP
    if !multi_config.enabled || multi_config.levels == 0 {
        return match_scan_with_config(points, lines, initial_pose, icp_config.clone());
    }

    let mut pose = initial_pose;
    let mut total_iterations = 0;
    let mut final_result = None;

    // Process from coarsest (level 0) to finest (level N-1)
    for level in 0..multi_config.levels {
        let subsample = multi_config.subsample_for_level(level);
        let level_iterations = multi_config.iterations_for_level(level);

        // Subsample points deterministically using step_by
        let subsampled: Vec<Point2D> = points.iter().step_by(subsample.max(1)).copied().collect();

        // Skip if too few points after subsampling
        if subsampled.len() < icp_config.min_correspondences {
            continue;
        }

        // Configure ICP for this level
        let mut level_config = icp_config.clone();
        level_config.max_iterations = level_iterations;

        // Run ICP at this level
        let mut icp = PointToLineIcp::with_config(level_config);
        let result = icp.match_scan(&subsampled, lines, pose);

        // Update pose for next level
        pose = result.pose;
        total_iterations += result.iterations;
        final_result = Some(result);
    }

    // Return result from finest level (or initial if no levels ran)
    match final_result {
        Some(mut result) => {
            result.iterations = total_iterations;
            result
        }
        None => MatchResult::new(pose, CorrespondenceSet::new(), 0, false),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point2D;
    use crate::matching::RobustCostFunction;
    use approx::assert_relative_eq;

    fn make_room_lines() -> Vec<Line2D> {
        // 4m x 4m room centered at origin
        vec![
            Line2D::new(Point2D::new(-2.0, -2.0), Point2D::new(2.0, -2.0)), // Bottom
            Line2D::new(Point2D::new(2.0, -2.0), Point2D::new(2.0, 2.0)),   // Right
            Line2D::new(Point2D::new(2.0, 2.0), Point2D::new(-2.0, 2.0)),   // Top
            Line2D::new(Point2D::new(-2.0, 2.0), Point2D::new(-2.0, -2.0)), // Left
        ]
    }

    fn make_scan_points_at_origin() -> Vec<Point2D> {
        // Points as if robot is at origin, looking at walls
        let mut points = Vec::new();

        // Points seeing the four walls
        for i in -5..=5 {
            let x = i as f32 * 0.3;
            // Bottom wall (y = -2)
            points.push(Point2D::new(x, -2.0));
            // Top wall (y = 2)
            points.push(Point2D::new(x, 2.0));
        }
        for i in -5..=5 {
            let y = i as f32 * 0.3;
            // Left wall (x = -2)
            points.push(Point2D::new(-2.0, y));
            // Right wall (x = 2)
            points.push(Point2D::new(2.0, y));
        }

        points
    }

    #[test]
    fn test_icp_identity() {
        let lines = make_room_lines();
        let points = make_scan_points_at_origin();

        let mut icp = PointToLineIcp::new();
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        assert!(result.converged);
        assert_relative_eq!(result.pose.x, 0.0, epsilon = 0.1);
        assert_relative_eq!(result.pose.y, 0.0, epsilon = 0.1);
        assert_relative_eq!(result.pose.theta.abs(), 0.0, epsilon = 0.1);
    }

    #[test]
    fn test_icp_with_offset() {
        let lines = make_room_lines();

        // Create scan points slightly offset from walls (simulating sensor noise)
        let mut points = Vec::new();
        for i in -5..=5 {
            let x = i as f32 * 0.3;
            // Points near bottom wall with small offset
            points.push(Point2D::new(x, -1.95));
            // Points near top wall with small offset
            points.push(Point2D::new(x, 1.95));
        }
        for i in -5..=5 {
            let y = i as f32 * 0.3;
            // Points near left wall with small offset
            points.push(Point2D::new(-1.95, y));
            // Points near right wall with small offset
            points.push(Point2D::new(1.95, y));
        }

        let config = IcpConfig::default()
            .with_max_iterations(50)
            .with_max_correspondence_distance(0.5);

        let mut icp = PointToLineIcp::with_config(config);
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        // Should find correspondences
        assert!(!result.correspondences.is_empty());
        // RMS error should be small (points are close to lines)
        assert!(
            result.rms_error < 0.2,
            "RMS error too high: {}",
            result.rms_error
        );
    }

    #[test]
    fn test_icp_insufficient_points() {
        let lines = make_room_lines();
        let points = vec![Point2D::new(0.0, -2.0)]; // Only one point

        let config = IcpConfig::default().with_min_correspondences(5);
        let mut icp = PointToLineIcp::with_config(config);
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        assert!(!result.converged);
    }

    #[test]
    fn test_icp_no_lines() {
        let lines: Vec<Line2D> = vec![];
        let points = make_scan_points_at_origin();

        let mut icp = PointToLineIcp::new();
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        assert!(!result.converged);
        assert!(result.correspondences.is_empty());
    }

    #[test]
    fn test_icp_with_tukey_robust_cost() {
        let lines = make_room_lines();
        let mut points = make_scan_points_at_origin();

        // Add some outliers
        points.push(Point2D::new(10.0, 10.0));
        points.push(Point2D::new(-10.0, -10.0));
        points.push(Point2D::new(5.0, -5.0));

        // Use Tukey for aggressive outlier rejection (zero weight beyond cutoff)
        let config = IcpConfig::default()
            .with_robust_cost(RobustCostFunction::Tukey { c: 0.1 })
            .with_irls(true);

        let mut icp = PointToLineIcp::with_config(config);
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        // Should still converge despite outliers (they get zero weight)
        assert!(result.converged);
    }

    #[test]
    fn test_icp_with_huber_robust_cost() {
        let lines = make_room_lines();
        let mut points = make_scan_points_at_origin();

        // Add outliers far from lines
        points.push(Point2D::new(0.0, 0.0)); // Center - far from all walls

        // Use Huber for soft outlier down-weighting
        let config = IcpConfig::default()
            .with_robust_cost(RobustCostFunction::Huber { delta: 0.03 })
            .with_irls(true);

        let mut icp = PointToLineIcp::with_config(config);
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        assert!(result.converged);
        // Outlier should have low weight but still be included
        assert!(!result.correspondences.is_empty());
    }

    #[test]
    fn test_match_result_confidence() {
        let lines = make_room_lines();
        let points = make_scan_points_at_origin();

        let result = match_scan(&points, &lines, Pose2D::identity());

        assert!(result.converged);
        assert!(result.confidence > 0.0);
        assert!(result.is_good_match(0.1));
    }

    #[test]
    fn test_convenience_function() {
        let lines = make_room_lines();
        let points = make_scan_points_at_origin();

        let result = match_scan(&points, &lines, Pose2D::identity());

        assert!(result.converged);
    }

    #[test]
    fn test_icp_collinear_points() {
        // Test with all points along a single line (degenerate case)
        // This can cause issues with the Jacobian being rank-deficient

        // Map has only a horizontal wall
        let lines = vec![Line2D::new(Point2D::new(-5.0, 0.0), Point2D::new(5.0, 0.0))];

        // All points on a line parallel to the wall
        let points: Vec<Point2D> = (-10..=10)
            .map(|i| Point2D::new(i as f32 * 0.5, 0.1))
            .collect();

        let config = IcpConfig::default().with_min_correspondences(3);
        let mut icp = PointToLineIcp::with_config(config);
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        // Should handle gracefully (may or may not converge, but shouldn't crash)
        // The key is no panics or NaN values
        assert!(!result.pose.x.is_nan());
        assert!(!result.pose.y.is_nan());
        assert!(!result.pose.theta.is_nan());
    }

    #[test]
    fn test_icp_max_iterations_reached() {
        // Create a scenario where ICP struggles to converge

        // Small room
        let lines = vec![
            Line2D::new(Point2D::new(-1.0, -1.0), Point2D::new(1.0, -1.0)),
            Line2D::new(Point2D::new(1.0, -1.0), Point2D::new(1.0, 1.0)),
        ];

        // Points with large initial offset
        let points: Vec<Point2D> = (-5..=5)
            .map(|i| Point2D::new(i as f32 * 0.1 + 2.0, -0.9))
            .collect();

        // Very low max iterations to force hitting the limit
        let config = IcpConfig::default()
            .with_max_iterations(2)
            .with_convergence_threshold(1e-10) // Very strict to prevent early convergence
            .with_min_correspondences(3);

        let mut icp = PointToLineIcp::with_config(config);
        let result = icp.match_scan(&points, &lines, Pose2D::identity());

        // Should hit max iterations (may not converge)
        // The important check is that iterations <= max_iterations
        assert!(result.iterations <= 2);
    }

    #[test]
    fn test_icp_iterations_tracked() {
        let lines = make_room_lines();
        let points = make_scan_points_at_origin();

        let result = match_scan(&points, &lines, Pose2D::identity());

        // If converged, should have used some iterations
        if result.converged {
            assert!(
                result.iterations > 0,
                "Converged result should track iterations"
            );
            assert!(
                result.iterations <= 30,
                "Iterations should not exceed max (30)"
            );
        }
    }

    #[test]
    fn test_icp_symmetric_room() {
        // Test with a symmetric room where correspondences might be ambiguous

        // Square room (all walls equal length)
        let lines = vec![
            Line2D::new(Point2D::new(-2.0, -2.0), Point2D::new(2.0, -2.0)), // Bottom
            Line2D::new(Point2D::new(2.0, -2.0), Point2D::new(2.0, 2.0)),   // Right
            Line2D::new(Point2D::new(2.0, 2.0), Point2D::new(-2.0, 2.0)),   // Top
            Line2D::new(Point2D::new(-2.0, 2.0), Point2D::new(-2.0, -2.0)), // Left
        ];

        // Symmetric scan from center
        let mut points = Vec::new();
        for i in -3..=3 {
            let offset = i as f32 * 0.5;
            points.push(Point2D::new(offset, -2.0)); // Bottom
            points.push(Point2D::new(offset, 2.0)); // Top
            points.push(Point2D::new(-2.0, offset)); // Left
            points.push(Point2D::new(2.0, offset)); // Right
        }

        let result = match_scan(&points, &lines, Pose2D::identity());

        // Should converge despite symmetry
        assert!(result.converged);
        // Pose should be near identity (we're at center looking at symmetric walls)
        assert!(result.pose.x.abs() < 0.5);
        assert!(result.pose.y.abs() < 0.5);
    }

    #[test]
    fn test_icp_indexed_matches_standard() {
        // Verify that indexed ICP produces same results as standard ICP
        let lines = make_room_lines();
        let points = make_scan_points_at_origin();

        // Standard ICP
        let line_collection = LineCollection::from_lines(&lines);
        let mut scratch_std = IcpScratchSpace::default_capacity();
        let icp = PointToLineIcp::new();
        let result_std = icp.match_scan_with_scratch(
            &points,
            &line_collection,
            Pose2D::identity(),
            &mut scratch_std,
        );

        // Indexed ICP
        let spatial_index = line_collection.build_spatial_index_for_radius(0.5);
        let mut scratch_idx = IcpScratchSpace::default_capacity();
        let result_idx = icp.match_scan_indexed(
            &points,
            &line_collection,
            &spatial_index,
            Pose2D::identity(),
            &mut scratch_idx,
        );

        // Both should converge
        assert!(result_std.converged, "Standard ICP should converge");
        assert!(result_idx.converged, "Indexed ICP should converge");

        // Poses should be very close
        assert_relative_eq!(result_std.pose.x, result_idx.pose.x, epsilon = 0.05);
        assert_relative_eq!(result_std.pose.y, result_idx.pose.y, epsilon = 0.05);
        assert_relative_eq!(result_std.pose.theta, result_idx.pose.theta, epsilon = 0.05);
    }

    #[test]
    fn test_icp_indexed_identity() {
        let lines = make_room_lines();
        let points = make_scan_points_at_origin();

        let line_collection = LineCollection::from_lines(&lines);
        let spatial_index = line_collection.build_spatial_index_for_radius(0.5);
        let mut scratch = IcpScratchSpace::default_capacity();

        let icp = PointToLineIcp::new();
        let result = icp.match_scan_indexed(
            &points,
            &line_collection,
            &spatial_index,
            Pose2D::identity(),
            &mut scratch,
        );

        assert!(result.converged);
        assert_relative_eq!(result.pose.x, 0.0, epsilon = 0.1);
        assert_relative_eq!(result.pose.y, 0.0, epsilon = 0.1);
        assert_relative_eq!(result.pose.theta.abs(), 0.0, epsilon = 0.1);
    }
}
