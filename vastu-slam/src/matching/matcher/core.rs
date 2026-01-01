//! Correlative scan-to-map matcher implementation.

use crate::core::{CellType, LidarScan, Pose2D, WorldPoint};
use crate::grid::GridStorage;

use crate::core::simd::{PointCloud, RotationMatrix4, transform_points_simd4, world_to_grid_simd4};

use super::helpers::{normalize_angle, solve_3x3};
use super::lm::AdaptiveLM;
use super::types::ScratchBuffers;
use crate::matching::config::CorrelativeMatcherConfig;
use crate::matching::types::ScanMatchResult;

/// Parameters for pose search space.
struct SearchParams {
    /// Center pose for the search grid.
    center_pose: Pose2D,
    /// Prior pose for tie-breaking penalty.
    prior_pose: Pose2D,
    /// Linear step size in meters.
    linear_step: f32,
    /// Angular step size in radians.
    angular_step: f32,
}

/// Pre-computed parameters for pose scoring.
/// Groups trig values, sensor offset, and Gaussian scoring constants
/// to avoid redundant computations in tight loops.
#[derive(Clone, Copy)]
struct ScoringParams {
    /// Pre-computed sin(theta)
    sin_theta: f32,
    /// Pre-computed cos(theta)
    cos_theta: f32,
    /// Sensor X offset
    offset_x: f32,
    /// Sensor Y offset
    offset_y: f32,
    /// Whether to use Gaussian scoring
    use_gaussian: bool,
    /// Pre-computed 1.0 / (2.0 * sigma^2)
    inv_two_sigma_sq: f32,
    /// Hit threshold (sigma)
    hit_threshold: f32,
}

/// Correlative scan-to-map matcher.
///
/// Uses a brute-force search with optional multi-resolution refinement
/// to find the best alignment of a lidar scan to the map.
pub struct CorrelativeMatcher {
    config: CorrelativeMatcherConfig,
}

impl CorrelativeMatcher {
    /// Create a new correlative matcher.
    pub fn new(config: CorrelativeMatcherConfig) -> Self {
        Self { config }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(CorrelativeMatcherConfig::default())
    }

    /// Get configuration.
    pub fn config(&self) -> &CorrelativeMatcherConfig {
        &self.config
    }

    /// Match a lidar scan against the occupancy grid map.
    ///
    /// # Arguments
    /// * `scan` - The lidar scan to match
    /// * `prior_pose` - Initial pose estimate (e.g., from encoders)
    /// * `storage` - The occupancy grid map
    ///
    /// # Returns
    /// The best matching pose and quality metrics.
    pub fn match_scan(
        &self,
        scan: &LidarScan,
        prior_pose: Pose2D,
        storage: &GridStorage,
    ) -> ScanMatchResult {
        if !self.config.enabled {
            return ScanMatchResult::new(prior_pose, 1.0, 0, 0);
        }

        // Extract valid scan points (in sensor frame)
        let points = self.extract_scan_points(scan);
        if points.is_empty() {
            return ScanMatchResult::failed(prior_pose);
        }

        if self.config.multi_resolution {
            self.match_multi_resolution(&points, prior_pose, storage)
        } else {
            self.match_single_resolution(&points, prior_pose, storage)
        }
    }

    /// Extract valid points from scan, optionally subsampling.
    fn extract_scan_points(&self, scan: &LidarScan) -> Vec<(f32, f32)> {
        let mut points: Vec<(f32, f32)> = scan
            .valid_points()
            .map(|(angle, range)| {
                let x = range * angle.cos();
                let y = range * angle.sin();
                (x, y)
            })
            .collect();

        // Subsample if needed
        if self.config.max_points > 0 && points.len() > self.config.max_points {
            let step = points.len() / self.config.max_points;
            points = points.into_iter().step_by(step).collect();
        }

        points
    }

    /// Single resolution search.
    fn match_single_resolution(
        &self,
        points: &[(f32, f32)],
        prior_pose: Pose2D,
        storage: &GridStorage,
    ) -> ScanMatchResult {
        self.search_poses(
            points,
            prior_pose,
            prior_pose, // center and prior are the same for single resolution
            storage,
            self.config.linear_resolution,
            self.config.angular_resolution,
        )
    }

    /// Multi-resolution search: coarse first, then refine with Gauss-Newton.
    fn match_multi_resolution(
        &self,
        points: &[(f32, f32)],
        prior_pose: Pose2D,
        storage: &GridStorage,
    ) -> ScanMatchResult {
        // Coarse search
        let coarse_linear = self.config.linear_resolution * self.config.coarse_factor;
        let coarse_angular = self.config.angular_resolution * self.config.coarse_factor;

        let coarse_result = self.search_poses(
            points,
            prior_pose,
            prior_pose,
            storage,
            coarse_linear,
            coarse_angular,
        );

        if !coarse_result.converged {
            return coarse_result;
        }

        // Fine search around coarse result (with tighter window)
        // NOTE: We still penalize based on original prior_pose, not coarse_result
        let fine_config = CorrelativeMatcherConfig {
            search_x: coarse_linear * 1.5,
            search_y: coarse_linear * 1.5,
            search_theta: coarse_angular * 1.5,
            ..self.config.clone()
        };

        let fine_matcher = CorrelativeMatcher::new(fine_config);
        let fine_result = fine_matcher.search_poses(
            points,
            coarse_result.pose, // center on coarse result
            prior_pose,         // but penalize based on original prior
            storage,
            self.config.linear_resolution,
            self.config.angular_resolution,
        );

        // Gauss-Newton refinement for sub-pixel accuracy
        self.refine_with_gauss_newton(points, fine_result, storage)
    }

    /// Refine a scan match result using Gauss-Newton optimization with adaptive LM.
    ///
    /// This provides sub-pixel/sub-degree accuracy by minimizing the sum of
    /// squared distances from scan points to the nearest walls.
    ///
    /// Uses **adaptive** Levenberg-Marquardt damping that adjusts based on step
    /// quality, providing faster convergence than fixed damping while maintaining
    /// stability. Also implements step acceptance/rejection logic.
    ///
    /// Prior constraints (like Cartographer's CeresScanMatcher) prevent over-correction.
    fn refine_with_gauss_newton(
        &self,
        points: &[(f32, f32)],
        initial_result: ScanMatchResult,
        storage: &GridStorage,
    ) -> ScanMatchResult {
        if points.is_empty() || self.config.gn_max_iterations == 0 {
            return initial_result;
        }

        let mut pose = initial_result.pose;
        let initial_pose = initial_result.pose; // Save for prior constraint
        let (offset_x, offset_y) = self.config.sensor_offset;

        // Prior constraint weights (Cartographer-style)
        let trans_weight = self.config.gn_translation_weight;
        let rot_weight = self.config.gn_rotation_weight;

        // Adaptive LM optimizer
        let mut lm = AdaptiveLM::new();

        // Compute initial cost
        let mut current_cost = self.compute_refinement_cost(
            points,
            pose,
            initial_pose,
            storage,
            trans_weight,
            rot_weight,
        );

        for _iter in 0..self.config.gn_max_iterations {
            // Build normal equations: H * delta = -g
            // where H ≈ J^T * J, g = J^T * r
            let mut h = [[0.0f32; 3]; 3]; // 3x3 Hessian approximation
            let mut g = [0.0f32; 3]; // Gradient

            let cos_theta = pose.theta.cos();
            let sin_theta = pose.theta.sin();

            // Sensor position in world frame
            let sensor_x = pose.x + offset_x * cos_theta - offset_y * sin_theta;
            let sensor_y = pose.y + offset_x * sin_theta + offset_y * cos_theta;

            for &(px, py) in points {
                // Transform point from sensor frame to world frame
                let world_x = sensor_x + px * cos_theta - py * sin_theta;
                let world_y = sensor_y + px * sin_theta + py * cos_theta;

                // Get interpolated distance and gradient
                let (dist, grad_x, grad_y) =
                    storage.get_distance_interpolated(WorldPoint::new(world_x, world_y));

                // Skip points with no gradient info (far from walls)
                if dist >= storage.resolution() * 20.0 {
                    continue;
                }

                // Jacobian row: [∂d/∂x, ∂d/∂y, ∂d/∂θ]
                let dworld_x_dtheta = -px * sin_theta - py * cos_theta;
                let dworld_y_dtheta = px * cos_theta - py * sin_theta;
                let j_theta = grad_x * dworld_x_dtheta + grad_y * dworld_y_dtheta;

                let j = [grad_x, grad_y, j_theta];

                // Accumulate H = J^T * J and g = J^T * r
                for i in 0..3 {
                    g[i] += j[i] * dist;
                    for k in 0..3 {
                        h[i][k] += j[i] * j[k];
                    }
                }
            }

            // Add prior constraint (Cartographer-style CeresScanMatcher)
            if trans_weight > 0.0 {
                h[0][0] += trans_weight;
                h[1][1] += trans_weight;
                g[0] += trans_weight * (pose.x - initial_pose.x);
                g[1] += trans_weight * (pose.y - initial_pose.y);
            }
            if rot_weight > 0.0 {
                h[2][2] += rot_weight;
                let angle_error = normalize_angle(pose.theta - initial_pose.theta);
                g[2] += rot_weight * angle_error;
            }

            // Store undamped Hessian for predicted reduction calculation
            let h_undamped = h;

            // Add adaptive Levenberg-Marquardt damping to diagonal
            let damping = lm.damping();
            for (i, row) in h.iter_mut().enumerate() {
                row[i] += damping * (1.0 + row[i]);
            }

            // Solve 3x3 system using Cramer's rule
            let delta = solve_3x3(&h, &[-g[0], -g[1], -g[2]]);

            // Check for numerical issues
            if delta.iter().any(|&d| !d.is_finite()) {
                break;
            }

            // Compute predicted cost reduction using linear model
            // predicted_reduction = -delta^T * g - 0.5 * delta^T * H * delta
            let htd = [
                h_undamped[0][0] * delta[0]
                    + h_undamped[0][1] * delta[1]
                    + h_undamped[0][2] * delta[2],
                h_undamped[1][0] * delta[0]
                    + h_undamped[1][1] * delta[1]
                    + h_undamped[1][2] * delta[2],
                h_undamped[2][0] * delta[0]
                    + h_undamped[2][1] * delta[1]
                    + h_undamped[2][2] * delta[2],
            ];
            let delta_dot_g = delta[0] * g[0] + delta[1] * g[1] + delta[2] * g[2];
            let delta_dot_htd = delta[0] * htd[0] + delta[1] * htd[1] + delta[2] * htd[2];
            let predicted_reduction = -delta_dot_g - 0.5 * delta_dot_htd;

            // Compute candidate pose
            let mut candidate = pose;
            candidate.x += delta[0];
            candidate.y += delta[1];
            candidate.theta = normalize_angle(candidate.theta + delta[2]);

            // Compute actual cost at candidate pose
            let candidate_cost = self.compute_refinement_cost(
                points,
                candidate,
                initial_pose,
                storage,
                trans_weight,
                rot_weight,
            );
            let actual_reduction = current_cost - candidate_cost;

            // Compute step quality ratio (rho)
            let rho = if predicted_reduction.abs() > 1e-10 {
                actual_reduction / predicted_reduction
            } else if actual_reduction > 0.0 {
                1.0 // Tiny predicted but positive actual = good
            } else {
                0.0
            };

            // Accept or reject step based on actual improvement
            if actual_reduction > 0.0 {
                // Step improved cost - accept it
                pose = candidate;
                current_cost = candidate_cost;
                lm.update(rho);

                // Check convergence
                let update_mag = (delta[0] * delta[0] + delta[1] * delta[1]).sqrt();
                if update_mag < self.config.gn_convergence_threshold {
                    break;
                }
            } else {
                // Step made things worse - reject and increase damping
                lm.reject_step();

                // If stuck at maximum damping, stop
                if lm.is_stuck() {
                    break;
                }
            }
        }

        // Recompute score at refined pose
        let (_raw_score, hits) = self.score_pose(points, pose, storage);
        let normalized_score = if points.is_empty() {
            0.0
        } else {
            (hits as f32 / points.len() as f32).min(1.0)
        };

        ScanMatchResult::new(pose, normalized_score, hits, points.len())
    }

    /// Compute the total cost for Gauss-Newton refinement.
    ///
    /// Cost = sum of squared distances + prior constraint penalties.
    fn compute_refinement_cost(
        &self,
        points: &[(f32, f32)],
        pose: Pose2D,
        initial_pose: Pose2D,
        storage: &GridStorage,
        trans_weight: f32,
        rot_weight: f32,
    ) -> f32 {
        let (offset_x, offset_y) = self.config.sensor_offset;
        let cos_theta = pose.theta.cos();
        let sin_theta = pose.theta.sin();

        // Sensor position in world frame
        let sensor_x = pose.x + offset_x * cos_theta - offset_y * sin_theta;
        let sensor_y = pose.y + offset_x * sin_theta + offset_y * cos_theta;

        let max_dist = storage.resolution() * 20.0;
        let mut cost = 0.0f32;

        // Sum of squared distances
        for &(px, py) in points {
            let world_x = sensor_x + px * cos_theta - py * sin_theta;
            let world_y = sensor_y + px * sin_theta + py * cos_theta;

            let (dist, _, _) = storage.get_distance_interpolated(WorldPoint::new(world_x, world_y));
            if dist < max_dist {
                cost += dist * dist;
            }
        }

        // Prior constraint cost
        if trans_weight > 0.0 {
            let dx = pose.x - initial_pose.x;
            let dy = pose.y - initial_pose.y;
            cost += trans_weight * (dx * dx + dy * dy);
        }
        if rot_weight > 0.0 {
            let dtheta = normalize_angle(pose.theta - initial_pose.theta);
            cost += rot_weight * dtheta * dtheta;
        }

        cost
    }

    /// Search over pose space to find best match.
    ///
    /// # Arguments
    /// * `center_pose` - The pose to center the search around
    /// * `prior_pose` - The original prior pose used for penalty calculation
    ///   (may differ from center_pose in multi-resolution search)
    fn search_poses(
        &self,
        points: &[(f32, f32)],
        center_pose: Pose2D,
        prior_pose: Pose2D,
        storage: &GridStorage,
        linear_step: f32,
        angular_step: f32,
    ) -> ScanMatchResult {
        let mut best_pose = center_pose;
        let mut best_score = f32::NEG_INFINITY;
        let mut best_hits = 0usize;

        // Generate search steps
        let x_steps = (self.config.search_x / linear_step).ceil() as i32;
        let y_steps = (self.config.search_y / linear_step).ceil() as i32;
        let theta_steps = (self.config.search_theta / angular_step).ceil() as i32;

        // Pre-compute Gaussian scoring constants (avoid recomputing per pose)
        let use_gaussian = self.config.use_gaussian_scoring;
        let sigma = self.config.gaussian_sigma;
        let inv_two_sigma_sq = 1.0 / (2.0 * sigma * sigma);
        let hit_threshold = sigma;

        // Pre-compute sensor offset
        let (offset_x, offset_y) = self.config.sensor_offset;

        // Original loop order: x → y → theta (innermost)
        for dx in -x_steps..=x_steps {
            let x = center_pose.x + dx as f32 * linear_step;

            for dy in -y_steps..=y_steps {
                let y = center_pose.y + dy as f32 * linear_step;

                for dtheta in -theta_steps..=theta_steps {
                    let theta = center_pose.theta + dtheta as f32 * angular_step;
                    let candidate = Pose2D::new(x, y, theta);

                    // Hardware sin/cos is faster than cache lookup on Apple Silicon
                    let sin_theta = theta.sin();
                    let cos_theta = theta.cos();

                    // Calculate sensor position in world frame
                    let sensor_x = x + offset_x * cos_theta - offset_y * sin_theta;
                    let sensor_y = y + offset_x * sin_theta + offset_y * cos_theta;

                    let mut hits = 0usize;
                    let mut score = 0.0f32;

                    for &(px, py) in points {
                        // Transform point from sensor frame to world frame
                        let world_x = sensor_x + px * cos_theta - py * sin_theta;
                        let world_y = sensor_y + px * sin_theta + py * cos_theta;

                        let world_point = WorldPoint::new(world_x, world_y);
                        let grid_coord = storage.world_to_grid(world_point);

                        // Check if point is within map bounds
                        if !storage.is_valid_coord(grid_coord) {
                            continue;
                        }

                        if use_gaussian {
                            let distance = storage.get_distance(grid_coord);
                            if distance < f32::MAX {
                                let point_score = (-distance * distance * inv_two_sigma_sq).exp();
                                score += point_score;
                                if distance <= hit_threshold {
                                    hits += 1;
                                }
                            } else {
                                score += 0.1;
                            }
                        } else {
                            match storage.get_type(grid_coord) {
                                CellType::Wall => {
                                    hits += 1;
                                    score += 1.0;
                                }
                                CellType::Floor => {
                                    score -= 1.0;
                                }
                                CellType::Unknown => {
                                    score += 0.1;
                                }
                                _ => {}
                            }
                        }
                    }

                    // Add tie-breaking penalty for distance from ORIGINAL prior pose
                    // This prefers poses closer to odometry when scores are similar
                    // Position: Higher weight since encoders are typically reliable for distance
                    // Angular: Lower weight to allow scan matching to correct heading drift
                    let dist_to_prior = ((candidate.x - prior_pose.x).powi(2)
                        + (candidate.y - prior_pose.y).powi(2))
                    .sqrt();
                    let angle_dist = (candidate.theta - prior_pose.theta).abs();
                    let adjusted_score = score - dist_to_prior * 300.0 - angle_dist * 30.0;

                    if adjusted_score > best_score {
                        best_score = adjusted_score;
                        best_pose = candidate;
                        best_hits = hits;
                    }
                }
            }
        }

        // Normalize score to 0-1 range
        let normalized_score = if points.is_empty() {
            0.0
        } else {
            (best_hits as f32 / points.len() as f32).min(1.0)
        };

        ScanMatchResult::new(best_pose, normalized_score, best_hits, points.len())
    }

    /// Score how well a pose aligns the scan points with the map.
    ///
    /// Returns (raw_score, hit_count).
    pub fn score_pose(
        &self,
        points: &[(f32, f32)],
        pose: Pose2D,
        storage: &GridStorage,
    ) -> (f32, usize) {
        let (offset_x, offset_y) = self.config.sensor_offset;
        let sigma = self.config.gaussian_sigma;

        let params = ScoringParams {
            sin_theta: pose.theta.sin(),
            cos_theta: pose.theta.cos(),
            offset_x,
            offset_y,
            use_gaussian: self.config.use_gaussian_scoring,
            inv_two_sigma_sq: 1.0 / (2.0 * sigma * sigma),
            hit_threshold: sigma,
        };

        self.score_pose_with_trig(points, pose, storage, &params)
    }

    /// Internal scoring method with pre-computed trigonometric and Gaussian values.
    ///
    /// This avoids redundant sin/cos and constant computations in tight loops.
    #[inline]
    fn score_pose_with_trig(
        &self,
        points: &[(f32, f32)],
        pose: Pose2D,
        storage: &GridStorage,
        params: &ScoringParams,
    ) -> (f32, usize) {
        // Calculate sensor position in world frame (applying sensor offset)
        let sensor_x =
            pose.x + params.offset_x * params.cos_theta - params.offset_y * params.sin_theta;
        let sensor_y =
            pose.y + params.offset_x * params.sin_theta + params.offset_y * params.cos_theta;

        let mut hits = 0usize;
        let mut score = 0.0f32;

        for &(px, py) in points {
            // Transform point from sensor frame to world frame
            let world_x = sensor_x + px * params.cos_theta - py * params.sin_theta;
            let world_y = sensor_y + px * params.sin_theta + py * params.cos_theta;

            let world_point = WorldPoint::new(world_x, world_y);
            let grid_coord = storage.world_to_grid(world_point);

            // Check if point is within map bounds
            if !storage.is_valid_coord(grid_coord) {
                continue;
            }

            if params.use_gaussian {
                // Gaussian scoring based on distance field
                let distance = storage.get_distance(grid_coord);

                if distance < f32::MAX {
                    // Gaussian score: exp(-d²/(2σ²))
                    // Distance 0 → score 1.0
                    // Distance sigma → score ~0.6
                    // Distance 2*sigma → score ~0.14
                    let point_score = (-distance * distance * params.inv_two_sigma_sq).exp();
                    score += point_score;

                    if distance <= params.hit_threshold {
                        hits += 1;
                    }
                } else {
                    // No wall nearby (unknown area) - small bonus for exploration
                    score += 0.1;
                }
            } else {
                // Binary scoring (original behavior)
                match storage.get_type(grid_coord) {
                    CellType::Wall => {
                        hits += 1;
                        score += 1.0;
                    }
                    CellType::Floor => {
                        score -= 1.0;
                    }
                    CellType::Unknown => {
                        score += 0.1;
                    }
                    _ => {}
                }
            }
        }

        (score, hits)
    }

    // =========================================================================
    // SIMD-OPTIMIZED METHODS
    // =========================================================================

    /// Match a lidar scan using SIMD-optimized batch operations.
    ///
    /// This method pre-converts scan points to SoA format and uses SIMD
    /// for point transformations. Reusable scratch buffers avoid allocations.
    pub fn match_scan_simd(
        &self,
        scan: &LidarScan,
        prior_pose: Pose2D,
        storage: &GridStorage,
        scratch: &mut ScratchBuffers,
    ) -> ScanMatchResult {
        if !self.config.enabled {
            return ScanMatchResult::new(prior_pose, 1.0, 0, 0);
        }

        // Extract valid scan points in SoA format
        let points = self.extract_scan_points_simd(scan);
        if points.is_empty() {
            return ScanMatchResult::failed(prior_pose);
        }

        // Ensure scratch buffers are large enough
        scratch.resize(points.len());

        if self.config.multi_resolution {
            self.match_multi_resolution_simd(&points, prior_pose, storage, scratch)
        } else {
            self.match_single_resolution_simd(&points, prior_pose, storage, scratch)
        }
    }

    /// Extract valid points from scan in SoA format for SIMD processing.
    fn extract_scan_points_simd(&self, scan: &LidarScan) -> PointCloud {
        // Use PointCloud::from_scan for simple case (handles padding automatically)
        if self.config.max_points == 0 || scan.ranges.len() <= self.config.max_points {
            return PointCloud::from_scan(scan);
        }

        // Manual extraction with subsampling
        let mut xs = Vec::with_capacity(scan.ranges.len());
        let mut ys = Vec::with_capacity(scan.ranges.len());

        for (angle, range) in scan.valid_points() {
            xs.push(range * angle.cos());
            ys.push(range * angle.sin());
        }

        // Subsample
        let step = xs.len() / self.config.max_points;
        xs = xs.into_iter().step_by(step).collect();
        ys = ys.into_iter().step_by(step).collect();

        let mut points = PointCloud { xs, ys };
        points.pad_to_lanes(); // Ensure SIMD alignment
        points
    }

    /// Single resolution search with SIMD.
    fn match_single_resolution_simd(
        &self,
        points: &PointCloud,
        prior_pose: Pose2D,
        storage: &GridStorage,
        scratch: &mut ScratchBuffers,
    ) -> ScanMatchResult {
        self.search_poses_simd(
            points,
            SearchParams {
                center_pose: prior_pose,
                prior_pose, // center and prior are the same for single resolution
                linear_step: self.config.linear_resolution,
                angular_step: self.config.angular_resolution,
            },
            storage,
            scratch,
        )
    }

    /// Multi-resolution search with SIMD: coarse first, then refine with Gauss-Newton.
    fn match_multi_resolution_simd(
        &self,
        points: &PointCloud,
        prior_pose: Pose2D,
        storage: &GridStorage,
        scratch: &mut ScratchBuffers,
    ) -> ScanMatchResult {
        // Coarse search
        let coarse_linear = self.config.linear_resolution * self.config.coarse_factor;
        let coarse_angular = self.config.angular_resolution * self.config.coarse_factor;

        let coarse_result = self.search_poses_simd(
            points,
            SearchParams {
                center_pose: prior_pose,
                prior_pose,
                linear_step: coarse_linear,
                angular_step: coarse_angular,
            },
            storage,
            scratch,
        );

        if !coarse_result.converged {
            return coarse_result;
        }

        // Fine search around coarse result (with tighter window)
        // NOTE: We still penalize based on original prior_pose, not coarse_result
        let fine_config = CorrelativeMatcherConfig {
            search_x: coarse_linear * 1.5,
            search_y: coarse_linear * 1.5,
            search_theta: coarse_angular * 1.5,
            ..self.config.clone()
        };

        let fine_matcher = CorrelativeMatcher::new(fine_config);
        let fine_result = fine_matcher.search_poses_simd(
            points,
            SearchParams {
                center_pose: coarse_result.pose, // center on coarse result
                prior_pose,                      // but penalize based on original prior
                linear_step: self.config.linear_resolution,
                angular_step: self.config.angular_resolution,
            },
            storage,
            scratch,
        );

        // Gauss-Newton refinement for sub-pixel accuracy
        // Convert PointCloud to Vec<(f32, f32)> for refinement
        let points_vec: Vec<(f32, f32)> = points
            .xs
            .iter()
            .zip(points.ys.iter())
            .map(|(&x, &y)| (x, y))
            .collect();
        self.refine_with_gauss_newton(&points_vec, fine_result, storage)
    }

    /// Search over pose space to find best match using SIMD.
    fn search_poses_simd(
        &self,
        points: &PointCloud,
        params: SearchParams,
        storage: &GridStorage,
        scratch: &mut ScratchBuffers,
    ) -> ScanMatchResult {
        let mut best_pose = params.center_pose;
        let mut best_score = f32::NEG_INFINITY;
        let mut best_hits = 0usize;

        // Generate search poses
        let x_steps = (self.config.search_x / params.linear_step).ceil() as i32;
        let y_steps = (self.config.search_y / params.linear_step).ceil() as i32;
        let theta_steps = (self.config.search_theta / params.angular_step).ceil() as i32;

        for dx in -x_steps..=x_steps {
            let x = params.center_pose.x + dx as f32 * params.linear_step;

            for dy in -y_steps..=y_steps {
                let y = params.center_pose.y + dy as f32 * params.linear_step;

                for dtheta in -theta_steps..=theta_steps {
                    let theta = params.center_pose.theta + dtheta as f32 * params.angular_step;
                    let candidate = Pose2D::new(x, y, theta);

                    let (score, hits) = self.score_pose_simd(points, candidate, storage, scratch);

                    // Add tie-breaking penalty for distance from ORIGINAL prior pose
                    // This prefers poses closer to odometry when scores are similar
                    // Position: Higher weight since encoders are typically reliable for distance
                    // Angular: Lower weight to allow scan matching to correct heading drift
                    let dist_to_prior = ((candidate.x - params.prior_pose.x).powi(2)
                        + (candidate.y - params.prior_pose.y).powi(2))
                    .sqrt();
                    let angle_dist = (candidate.theta - params.prior_pose.theta).abs();
                    let adjusted_score = score - dist_to_prior * 300.0 - angle_dist * 30.0;

                    if adjusted_score > best_score {
                        best_score = adjusted_score;
                        best_pose = candidate;
                        best_hits = hits;
                    }
                }
            }
        }

        // Normalize score to 0-1 range
        let normalized_score = if points.is_empty() {
            0.0
        } else {
            (best_hits as f32 / points.len() as f32).min(1.0)
        };

        ScanMatchResult::new(best_pose, normalized_score, best_hits, points.len())
    }

    /// Score how well a pose aligns the scan points with the map using SIMD.
    ///
    /// Uses SIMD for batch point transformation and world-to-grid conversion.
    /// The final scoring loop is scalar due to random memory access patterns.
    pub fn score_pose_simd(
        &self,
        points: &PointCloud,
        pose: Pose2D,
        storage: &GridStorage,
        scratch: &mut ScratchBuffers,
    ) -> (f32, usize) {
        let n = points.len();
        if n == 0 {
            return (0.0, 0);
        }

        // Calculate sensor position in world frame (applying sensor offset)
        let (offset_x, offset_y) = self.config.sensor_offset;
        let cos_theta = pose.theta.cos();
        let sin_theta = pose.theta.sin();
        let sensor_x = pose.x + offset_x * cos_theta - offset_y * sin_theta;
        let sensor_y = pose.y + offset_x * sin_theta + offset_y * cos_theta;

        // Create a pose with sensor position for SIMD transform
        let sensor_pose = Pose2D::new(sensor_x, sensor_y, pose.theta);

        // 1. Batch transform points from sensor frame to world frame (SIMD)
        let rot = RotationMatrix4::from_pose(&sensor_pose);
        transform_points_simd4(
            points,
            &rot,
            &mut scratch.world_xs[..n],
            &mut scratch.world_ys[..n],
        );

        // 2. Batch convert world to grid coordinates (SIMD)
        let inv_resolution = 1.0 / storage.resolution();
        world_to_grid_simd4(
            &scratch.world_xs[..n],
            &scratch.world_ys[..n],
            storage.origin().x,
            storage.origin().y,
            inv_resolution,
            &mut scratch.grid_xs[..n],
            &mut scratch.grid_ys[..n],
        );

        // 3. Score each point (scalar - random memory access)
        let mut hits = 0usize;
        let mut score = 0.0f32;

        let width = storage.width() as i32;
        let height = storage.height() as i32;

        // Precompute Gaussian constants
        let use_gaussian = self.config.use_gaussian_scoring;
        let sigma = self.config.gaussian_sigma;
        let two_sigma_sq = 2.0 * sigma * sigma;
        let hit_threshold = sigma;

        for i in 0..n {
            let gx = scratch.grid_xs[i];
            let gy = scratch.grid_ys[i];

            // Check if point is within map bounds
            if gx < 0 || gx >= width || gy < 0 || gy >= height {
                continue;
            }

            let coord = crate::core::GridCoord::new(gx, gy);

            if use_gaussian {
                // Gaussian scoring based on distance field
                let distance = storage.get_distance(coord);

                if distance < f32::MAX {
                    let point_score = (-distance * distance / two_sigma_sq).exp();
                    score += point_score;

                    if distance <= hit_threshold {
                        hits += 1;
                    }
                } else {
                    score += 0.1;
                }
            } else {
                // Binary scoring (original behavior)
                match storage.get_type(coord) {
                    CellType::Wall => {
                        hits += 1;
                        score += 1.0;
                    }
                    CellType::Floor => {
                        score -= 1.0;
                    }
                    CellType::Unknown => {
                        score += 0.1;
                    }
                    _ => {}
                }
            }
        }

        (score, hits)
    }
}
