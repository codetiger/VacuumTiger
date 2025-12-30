//! Correlative scan-to-map matcher.
//!
//! Performs brute-force search over a 3D pose space (x, y, theta) to find
//! the best alignment of a lidar scan to the current occupancy grid map.

use crate::core::{CellType, LidarScan, Pose2D, WorldPoint};
use crate::grid::GridStorage;

use crate::core::simd::{PointBatch, RotationMatrix4, transform_points_simd4, world_to_grid_simd4};

use super::config::CorrelativeMatcherConfig;
use super::types::ScanMatchResult;

/// Correlative scan-to-map matcher.
///
/// Uses a brute-force search with optional multi-resolution refinement
/// to find the best alignment of a lidar scan to the map.
pub struct CorrelativeMatcher {
    config: CorrelativeMatcherConfig,
}

/// Scratch buffers for SIMD operations to avoid per-call allocations.
pub struct ScratchBuffers {
    /// Transformed world X coordinates
    world_xs: Vec<f32>,
    /// Transformed world Y coordinates
    world_ys: Vec<f32>,
    /// Grid X coordinates
    grid_xs: Vec<i32>,
    /// Grid Y coordinates
    grid_ys: Vec<i32>,
}

impl ScratchBuffers {
    /// Create scratch buffers for a given number of points.
    pub fn new(capacity: usize) -> Self {
        Self {
            world_xs: vec![0.0; capacity],
            world_ys: vec![0.0; capacity],
            grid_xs: vec![0; capacity],
            grid_ys: vec![0; capacity],
        }
    }

    /// Resize buffers if needed.
    pub fn resize(&mut self, n: usize) {
        if self.world_xs.len() < n {
            self.world_xs.resize(n, 0.0);
            self.world_ys.resize(n, 0.0);
            self.grid_xs.resize(n, 0);
            self.grid_ys.resize(n, 0);
        }
    }
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

    /// Refine a scan match result using Gauss-Newton optimization.
    ///
    /// This provides sub-pixel/sub-degree accuracy by minimizing the sum of
    /// squared distances from scan points to the nearest walls.
    ///
    /// Uses Levenberg-Marquardt damping for stability and a prior constraint
    /// (like Cartographer's CeresScanMatcher) to prevent over-correction.
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
        // These penalize deviation from the correlative search result
        let trans_weight = self.config.gn_translation_weight;
        let rot_weight = self.config.gn_rotation_weight;

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
                // ∂d/∂x = grad_x (direct translation effect)
                // ∂d/∂y = grad_y (direct translation effect)
                // ∂d/∂θ = grad_x * ∂world_x/∂θ + grad_y * ∂world_y/∂θ
                //       = grad_x * (-px*sin - py*cos) + grad_y * (px*cos - py*sin)
                let dworld_x_dtheta = -px * sin_theta - py * cos_theta;
                let dworld_y_dtheta = px * cos_theta - py * sin_theta;
                let j_theta = grad_x * dworld_x_dtheta + grad_y * dworld_y_dtheta;

                let j = [grad_x, grad_y, j_theta];

                // Accumulate H = J^T * J and g = J^T * r
                // Using outer product: H += j * j^T, g += j * dist
                for i in 0..3 {
                    g[i] += j[i] * dist;
                    for k in 0..3 {
                        h[i][k] += j[i] * j[k];
                    }
                }
            }

            // Add prior constraint (Cartographer-style CeresScanMatcher)
            // This penalizes deviation from the correlative search result
            // E_prior = trans_weight * ||p - p_init||² + rot_weight * (θ - θ_init)²
            // Adds to H diagonal and g vector
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

            // Add Levenberg-Marquardt damping to diagonal
            let damping = self.config.gn_damping;
            for i in 0..3 {
                h[i][i] += damping * (1.0 + h[i][i]);
            }

            // Solve 3x3 system using Cramer's rule
            let delta = solve_3x3(&h, &[-g[0], -g[1], -g[2]]);

            // Check for numerical issues
            if delta.iter().any(|&d| !d.is_finite()) {
                break;
            }

            // Update pose
            pose.x += delta[0];
            pose.y += delta[1];
            pose.theta += delta[2];

            // Normalize theta to [-π, π]
            pose.theta = normalize_angle(pose.theta);

            // Check convergence
            let update_mag = (delta[0] * delta[0] + delta[1] * delta[1]).sqrt();
            if update_mag < self.config.gn_convergence_threshold {
                break;
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

        // Generate search poses
        let x_steps = (self.config.search_x / linear_step).ceil() as i32;
        let y_steps = (self.config.search_y / linear_step).ceil() as i32;
        let theta_steps = (self.config.search_theta / angular_step).ceil() as i32;

        for dx in -x_steps..=x_steps {
            let x = center_pose.x + dx as f32 * linear_step;

            for dy in -y_steps..=y_steps {
                let y = center_pose.y + dy as f32 * linear_step;

                for dtheta in -theta_steps..=theta_steps {
                    let theta = center_pose.theta + dtheta as f32 * angular_step;
                    let candidate = Pose2D::new(x, y, theta);

                    let (score, hits) = self.score_pose(points, candidate, storage);

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
    fn score_pose(
        &self,
        points: &[(f32, f32)],
        pose: Pose2D,
        storage: &GridStorage,
    ) -> (f32, usize) {
        let cos_theta = pose.theta.cos();
        let sin_theta = pose.theta.sin();

        // Calculate sensor position in world frame (applying sensor offset)
        let (offset_x, offset_y) = self.config.sensor_offset;
        let sensor_x = pose.x + offset_x * cos_theta - offset_y * sin_theta;
        let sensor_y = pose.y + offset_x * sin_theta + offset_y * cos_theta;

        let mut hits = 0usize;
        let mut score = 0.0f32;

        // Precompute Gaussian constants
        let use_gaussian = self.config.use_gaussian_scoring;
        let sigma = self.config.gaussian_sigma;
        let two_sigma_sq = 2.0 * sigma * sigma;
        // Hit threshold: within 1 sigma of wall
        let hit_threshold = sigma;

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
                // Gaussian scoring based on distance field
                let distance = storage.get_distance(grid_coord);

                if distance < f32::MAX {
                    // Gaussian score: exp(-d²/(2σ²))
                    // Distance 0 → score 1.0
                    // Distance sigma → score ~0.6
                    // Distance 2*sigma → score ~0.14
                    let point_score = (-distance * distance / two_sigma_sq).exp();
                    score += point_score;

                    if distance <= hit_threshold {
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
    fn extract_scan_points_simd(&self, scan: &LidarScan) -> PointBatch {
        let mut xs = Vec::with_capacity(scan.ranges.len());
        let mut ys = Vec::with_capacity(scan.ranges.len());

        for (angle, range) in scan.valid_points() {
            xs.push(range * angle.cos());
            ys.push(range * angle.sin());
        }

        // Subsample if needed
        if self.config.max_points > 0 && xs.len() > self.config.max_points {
            let step = xs.len() / self.config.max_points;
            xs = xs.into_iter().step_by(step).collect();
            ys = ys.into_iter().step_by(step).collect();
        }

        PointBatch { xs, ys }
    }

    /// Single resolution search with SIMD.
    fn match_single_resolution_simd(
        &self,
        points: &PointBatch,
        prior_pose: Pose2D,
        storage: &GridStorage,
        scratch: &mut ScratchBuffers,
    ) -> ScanMatchResult {
        self.search_poses_simd(
            points,
            prior_pose,
            prior_pose, // center and prior are the same for single resolution
            storage,
            self.config.linear_resolution,
            self.config.angular_resolution,
            scratch,
        )
    }

    /// Multi-resolution search with SIMD: coarse first, then refine with Gauss-Newton.
    fn match_multi_resolution_simd(
        &self,
        points: &PointBatch,
        prior_pose: Pose2D,
        storage: &GridStorage,
        scratch: &mut ScratchBuffers,
    ) -> ScanMatchResult {
        // Coarse search
        let coarse_linear = self.config.linear_resolution * self.config.coarse_factor;
        let coarse_angular = self.config.angular_resolution * self.config.coarse_factor;

        let coarse_result = self.search_poses_simd(
            points,
            prior_pose,
            prior_pose,
            storage,
            coarse_linear,
            coarse_angular,
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
            coarse_result.pose, // center on coarse result
            prior_pose,         // but penalize based on original prior
            storage,
            self.config.linear_resolution,
            self.config.angular_resolution,
            scratch,
        );

        // Gauss-Newton refinement for sub-pixel accuracy
        // Convert PointBatch to Vec<(f32, f32)> for refinement
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
        points: &PointBatch,
        center_pose: Pose2D,
        prior_pose: Pose2D,
        storage: &GridStorage,
        linear_step: f32,
        angular_step: f32,
        scratch: &mut ScratchBuffers,
    ) -> ScanMatchResult {
        let mut best_pose = center_pose;
        let mut best_score = f32::NEG_INFINITY;
        let mut best_hits = 0usize;

        // Generate search poses
        let x_steps = (self.config.search_x / linear_step).ceil() as i32;
        let y_steps = (self.config.search_y / linear_step).ceil() as i32;
        let theta_steps = (self.config.search_theta / angular_step).ceil() as i32;

        for dx in -x_steps..=x_steps {
            let x = center_pose.x + dx as f32 * linear_step;

            for dy in -y_steps..=y_steps {
                let y = center_pose.y + dy as f32 * linear_step;

                for dtheta in -theta_steps..=theta_steps {
                    let theta = center_pose.theta + dtheta as f32 * angular_step;
                    let candidate = Pose2D::new(x, y, theta);

                    let (score, hits) = self.score_pose_simd(points, candidate, storage, scratch);

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

    /// Score how well a pose aligns the scan points with the map using SIMD.
    ///
    /// Uses SIMD for batch point transformation and world-to-grid conversion.
    /// The final scoring loop is scalar due to random memory access patterns.
    fn score_pose_simd(
        &self,
        points: &PointBatch,
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

/// Solve a 3x3 linear system Ax = b using Cramer's rule.
///
/// Returns [0, 0, 0] if the matrix is singular.
fn solve_3x3(a: &[[f32; 3]; 3], b: &[f32; 3]) -> [f32; 3] {
    // Compute determinant of A
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if det.abs() < 1e-10 {
        return [0.0, 0.0, 0.0];
    }

    let inv_det = 1.0 / det;

    // Cramer's rule: x_i = det(A_i) / det(A)
    // where A_i is A with column i replaced by b
    let x0 = (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
        + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]))
        * inv_det;

    let x1 = (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
        - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]))
        * inv_det;

    let x2 = (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
        - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
        + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
        * inv_det;

    [x0, x1, x2]
}

/// Normalize angle to [-π, π).
fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle % std::f32::consts::TAU;
    if a > std::f32::consts::PI {
        a -= std::f32::consts::TAU;
    } else if a < -std::f32::consts::PI {
        a += std::f32::consts::TAU;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::GridCoord;

    fn create_test_storage() -> GridStorage {
        GridStorage::centered(100, 100, 0.05) // 5m x 5m at 5cm resolution
    }

    fn create_wall_in_front(storage: &mut GridStorage, pose: Pose2D, distance: f32) {
        // Create a wall in front of the pose
        let wall_x = pose.x + distance * pose.theta.cos();
        let wall_y = pose.y + distance * pose.theta.sin();

        // Create a line of wall cells
        for offset in -10..=10 {
            let perp_angle = pose.theta + std::f32::consts::FRAC_PI_2;
            let wx = wall_x + offset as f32 * 0.05 * perp_angle.cos();
            let wy = wall_y + offset as f32 * 0.05 * perp_angle.sin();
            let coord = storage.world_to_grid(WorldPoint::new(wx, wy));
            if storage.is_valid_coord(coord) {
                storage.set_type(coord, CellType::Wall);
            }
        }

        // Update distance field for Gaussian scoring
        storage.recompute_distance_field();
    }

    fn create_simple_scan(distance: f32, num_points: usize) -> LidarScan {
        // Create a scan with points at uniform angles, all at same distance
        let angle_min = -std::f32::consts::PI;
        let angle_max = std::f32::consts::PI;
        let angle_increment = (angle_max - angle_min) / num_points as f32;

        let ranges = vec![distance; num_points];
        let angles: Vec<f32> = (0..num_points)
            .map(|i| angle_min + i as f32 * angle_increment)
            .collect();

        LidarScan::new(ranges, angles, 0.1, 10.0)
    }

    #[test]
    fn test_matcher_creation() {
        let matcher = CorrelativeMatcher::with_defaults();
        assert!(matcher.config().enabled);
    }

    #[test]
    fn test_disabled_matcher() {
        let config = CorrelativeMatcherConfig::disabled();
        let matcher = CorrelativeMatcher::new(config);

        let storage = create_test_storage();
        let scan = create_simple_scan(2.0, 360);
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        let result = matcher.match_scan(&scan, pose, &storage);

        // Disabled matcher should return input pose unchanged
        assert!((result.pose.x - pose.x).abs() < 0.001);
        assert!((result.pose.y - pose.y).abs() < 0.001);
    }

    #[test]
    fn test_matching_returns_result() {
        let mut storage = create_test_storage();
        let true_pose = Pose2D::new(0.0, 0.0, 0.0);

        // Create a wall 2m in front
        create_wall_in_front(&mut storage, true_pose, 2.0);

        // Use fast config for speed
        let config = CorrelativeMatcherConfig::fast();
        let matcher = CorrelativeMatcher::new(config);

        // Test with prior very close to true pose
        let prior = Pose2D::new(0.01, 0.01, 0.01);

        // Create a scan that simulates hitting the wall from the prior pose
        let angles: Vec<f32> = (-10..=10).map(|i| i as f32 * 0.05).collect();
        let ranges: Vec<f32> = vec![2.0; angles.len()];
        let scan = LidarScan::new(ranges, angles, 0.1, 10.0);

        let result = matcher.match_scan(&scan, prior, &storage);

        // Matcher should return a result (may or may not converge depending on map state)
        // At minimum, it should return a valid pose
        assert!(result.pose.x.is_finite());
        assert!(result.pose.y.is_finite());
        assert!(result.pose.theta.is_finite());
    }

    #[test]
    fn test_score_increases_with_hits() {
        let mut storage = create_test_storage();

        // Fill some cells with walls
        for x in 40..60 {
            for y in 40..60 {
                storage.set_type(GridCoord::new(x, y), CellType::Wall);
            }
        }

        // Update distance field for Gaussian scoring
        storage.recompute_distance_field();

        let matcher = CorrelativeMatcher::with_defaults();
        let points: Vec<(f32, f32)> = vec![(0.0, 0.0), (0.05, 0.0), (0.1, 0.0)];

        // Pose in wall area should score higher
        let pose_in_wall = Pose2D::new(0.0, 0.0, 0.0);
        let pose_in_floor = Pose2D::new(-2.0, -2.0, 0.0);

        let (score_wall, hits_wall) = matcher.score_pose(&points, pose_in_wall, &storage);
        let (score_floor, hits_floor) = matcher.score_pose(&points, pose_in_floor, &storage);

        assert!(hits_wall > hits_floor);
        assert!(score_wall > score_floor);
    }

    #[test]
    fn test_simd_matches_scalar() {
        let mut storage = create_test_storage();
        let true_pose = Pose2D::new(0.0, 0.0, 0.0);

        // Create a wall 2m in front
        create_wall_in_front(&mut storage, true_pose, 2.0);

        // Use fast config for speed
        let config = CorrelativeMatcherConfig::fast();
        let matcher = CorrelativeMatcher::new(config);

        // Create a scan
        let angles: Vec<f32> = (-10..=10).map(|i| i as f32 * 0.05).collect();
        let ranges: Vec<f32> = vec![2.0; angles.len()];
        let scan = LidarScan::new(ranges, angles, 0.1, 10.0);

        let prior = Pose2D::new(0.01, 0.01, 0.01);

        // Get scalar result
        let scalar_result = matcher.match_scan(&scan, prior, &storage);

        // Get SIMD result
        let mut scratch = ScratchBuffers::new(scan.ranges.len());
        let simd_result = matcher.match_scan_simd(&scan, prior, &storage, &mut scratch);

        // Results should match
        assert!((scalar_result.pose.x - simd_result.pose.x).abs() < 0.001);
        assert!((scalar_result.pose.y - simd_result.pose.y).abs() < 0.001);
        assert!((scalar_result.pose.theta - simd_result.pose.theta).abs() < 0.001);
        assert!((scalar_result.score - simd_result.score).abs() < 0.001);
        assert_eq!(scalar_result.hits, simd_result.hits);
    }

    #[test]
    fn test_simd_score_matches_scalar() {
        let mut storage = create_test_storage();

        // Fill some cells with walls
        for x in 40..60 {
            for y in 40..60 {
                storage.set_type(GridCoord::new(x, y), CellType::Wall);
            }
        }

        // Update distance field for Gaussian scoring
        storage.recompute_distance_field();

        let matcher = CorrelativeMatcher::with_defaults();

        // Create points in AoS format for scalar
        let points: Vec<(f32, f32)> = vec![(0.0, 0.0), (0.05, 0.0), (0.1, 0.0), (0.15, 0.0)];

        // Convert to SoA format for SIMD
        let points_soa = PointBatch {
            xs: points.iter().map(|(x, _)| *x).collect(),
            ys: points.iter().map(|(_, y)| *y).collect(),
        };

        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let mut scratch = ScratchBuffers::new(points.len());

        // Get scalar score
        let (scalar_score, scalar_hits) = matcher.score_pose(&points, pose, &storage);

        // Get SIMD score
        let (simd_score, simd_hits) =
            matcher.score_pose_simd(&points_soa, pose, &storage, &mut scratch);

        // Results should match
        assert!(
            (scalar_score - simd_score).abs() < 0.001,
            "score mismatch: {} vs {}",
            scalar_score,
            simd_score
        );
        assert_eq!(
            scalar_hits, simd_hits,
            "hits mismatch: {} vs {}",
            scalar_hits, simd_hits
        );
    }
}
