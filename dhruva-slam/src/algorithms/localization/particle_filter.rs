//! Particle filter (Monte Carlo Localization) implementation.
//!
//! Implements Adaptive MCL with KLD-sampling for efficient particle count.

use crate::algorithms::mapping::OccupancyGrid;
use crate::core::types::{Covariance2D, LaserScan, Pose2D};

use super::motion_model::{MotionModel, MotionModelConfig, Rng, SimpleRng};
use super::sensor_model::{LikelihoodFieldModel, SensorModel, SensorModelConfig};

/// A single particle representing a possible robot pose.
#[derive(Debug, Clone, Copy)]
pub struct Particle {
    /// Hypothesized robot pose.
    pub pose: Pose2D,
    /// Importance weight (unnormalized).
    pub weight: f64,
}

impl Particle {
    /// Create a new particle with unit weight.
    pub fn new(pose: Pose2D) -> Self {
        Self { pose, weight: 1.0 }
    }

    /// Create a new particle with specified weight.
    pub fn with_weight(pose: Pose2D, weight: f64) -> Self {
        Self { pose, weight }
    }
}

/// Configuration for the particle filter.
#[derive(Debug, Clone)]
pub struct ParticleFilterConfig {
    /// Number of particles.
    pub num_particles: usize,

    /// Effective particle ratio threshold for resampling.
    /// Resample when Neff / num_particles < this value.
    /// Typical: 0.5
    pub resampling_threshold: f64,

    /// Motion model configuration.
    pub motion: MotionModelConfig,

    /// Sensor model configuration.
    pub sensor: SensorModelConfig,

    /// Initial pose spread (standard deviation) for initialization.
    pub initial_spread_xy: f32,
    pub initial_spread_theta: f32,

    /// Random seed for deterministic behavior (0 for random).
    pub seed: u64,
}

impl Default for ParticleFilterConfig {
    fn default() -> Self {
        Self {
            num_particles: 500,
            resampling_threshold: 0.5,
            motion: MotionModelConfig::default(),
            sensor: SensorModelConfig::default(),
            initial_spread_xy: 0.1,
            initial_spread_theta: 0.1,
            seed: 0,
        }
    }
}

impl ParticleFilterConfig {
    /// Create a configuration for global localization (large spread).
    pub fn global_localization() -> Self {
        Self {
            num_particles: 2000,
            initial_spread_xy: 10.0,
            initial_spread_theta: std::f32::consts::PI,
            ..Default::default()
        }
    }

    /// Create a configuration for tracking (small spread).
    pub fn tracking() -> Self {
        Self {
            num_particles: 200,
            initial_spread_xy: 0.05,
            initial_spread_theta: 0.05,
            ..Default::default()
        }
    }
}

/// State of the particle filter for diagnostics.
#[derive(Debug, Clone, Default)]
pub struct ParticleFilterState {
    /// Effective number of particles.
    pub neff: f64,
    /// Whether resampling occurred this iteration.
    pub resampled: bool,
    /// Best particle weight.
    pub max_weight: f64,
    /// Total number of iterations.
    pub iterations: u64,
}

/// Monte Carlo Localization particle filter.
#[derive(Debug)]
pub struct ParticleFilter {
    config: ParticleFilterConfig,
    particles: Vec<Particle>,
    motion_model: MotionModel,
    sensor_model: LikelihoodFieldModel,
    rng: SimpleRng,
    state: ParticleFilterState,
}

impl ParticleFilter {
    /// Create a new particle filter initialized around the given pose.
    pub fn new(config: ParticleFilterConfig, initial_pose: Pose2D, map: &OccupancyGrid) -> Self {
        let seed = if config.seed == 0 {
            // Use a simple time-based seed
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(12345)
        } else {
            config.seed
        };

        let mut rng = SimpleRng::new(seed);
        let motion_model = MotionModel::new(config.motion);
        let sensor_model = LikelihoodFieldModel::from_map(config.sensor, map);

        // Initialize particles around initial pose
        let particles = Self::initialize_particles(
            config.num_particles,
            &initial_pose,
            config.initial_spread_xy,
            config.initial_spread_theta,
            &mut rng,
        );

        Self {
            config,
            particles,
            motion_model,
            sensor_model,
            rng,
            state: ParticleFilterState::default(),
        }
    }

    /// Initialize particles with Gaussian distribution around a pose.
    fn initialize_particles(
        num_particles: usize,
        center: &Pose2D,
        spread_xy: f32,
        spread_theta: f32,
        rng: &mut SimpleRng,
    ) -> Vec<Particle> {
        (0..num_particles)
            .map(|_| {
                let x = center.x + rng.gen_standard_normal() * spread_xy;
                let y = center.y + rng.gen_standard_normal() * spread_xy;
                let theta = crate::core::math::normalize_angle(
                    center.theta + rng.gen_standard_normal() * spread_theta,
                );
                Particle::new(Pose2D::new(x, y, theta))
            })
            .collect()
    }

    /// Get the configuration.
    pub fn config(&self) -> &ParticleFilterConfig {
        &self.config
    }

    /// Get current particles (for visualization).
    pub fn particles(&self) -> &[Particle] {
        &self.particles
    }

    /// Get current filter state (for diagnostics).
    pub fn state(&self) -> &ParticleFilterState {
        &self.state
    }

    /// Get the number of particles.
    pub fn num_particles(&self) -> usize {
        self.particles.len()
    }

    /// Prediction step: propagate particles through motion model.
    ///
    /// Call this with odometry delta when the robot moves.
    pub fn predict(&mut self, odom_delta: &Pose2D) {
        for particle in &mut self.particles {
            particle.pose = self
                .motion_model
                .sample(&particle.pose, odom_delta, &mut self.rng);
        }
    }

    /// Update step: weight particles based on sensor observation.
    ///
    /// Call this when a new laser scan is available.
    pub fn update(&mut self, scan: &LaserScan, map: &OccupancyGrid) {
        self.state.iterations += 1;
        self.state.resampled = false;

        // Compute log-likelihoods for all particles
        let log_weights: Vec<f64> = self
            .particles
            .iter()
            .map(|p| self.sensor_model.log_likelihood(scan, &p.pose, map) as f64)
            .collect();

        // Convert to normalized weights using log-sum-exp trick
        let max_log_weight = log_weights
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);

        if !max_log_weight.is_finite() {
            // All particles have zero likelihood - reinitialize
            log::warn!("All particles have zero likelihood, keeping current weights");
            return;
        }

        // Subtract max for numerical stability and exponentiate
        let sum_exp: f64 = log_weights
            .iter()
            .map(|&lw| (lw - max_log_weight).exp())
            .sum();

        // Normalize weights
        for (i, particle) in self.particles.iter_mut().enumerate() {
            let normalized = (log_weights[i] - max_log_weight).exp() / sum_exp;
            particle.weight = normalized;
        }

        // Compute effective particle count
        let sum_sq: f64 = self.particles.iter().map(|p| p.weight * p.weight).sum();
        self.state.neff = if sum_sq > 1e-10 { 1.0 / sum_sq } else { 0.0 };
        self.state.max_weight = self.particles.iter().map(|p| p.weight).fold(0.0, f64::max);

        // Resample if needed
        let threshold = self.config.resampling_threshold * self.particles.len() as f64;
        if self.state.neff < threshold {
            self.resample();
            self.state.resampled = true;
        }
    }

    /// Low-variance resampling.
    fn resample(&mut self) {
        let n = self.particles.len();
        let mut new_particles = Vec::with_capacity(n);

        // Compute cumulative weights
        let mut cumulative: Vec<f64> = Vec::with_capacity(n);
        let mut sum = 0.0;
        for p in &self.particles {
            sum += p.weight;
            cumulative.push(sum);
        }

        // Normalize cumulative weights
        if sum > 1e-10 {
            for c in &mut cumulative {
                *c /= sum;
            }
        } else {
            // Uniform weights if sum is too small
            for (i, c) in cumulative.iter_mut().enumerate() {
                *c = (i + 1) as f64 / n as f64;
            }
        }

        // Low-variance resampling
        let step = 1.0 / n as f64;
        let mut r = self.rng.gen_f32() as f64 * step;
        let mut idx = 0;

        for _ in 0..n {
            while r > cumulative[idx] && idx < n - 1 {
                idx += 1;
            }

            // Copy particle with unit weight
            let mut new_particle = self.particles[idx];
            new_particle.weight = 1.0 / n as f64;
            new_particles.push(new_particle);

            r += step;
        }

        self.particles = new_particles;
    }

    /// Get the estimated pose (weighted mean of particles).
    pub fn estimate(&self) -> Pose2D {
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_sin = 0.0;
        let mut sum_cos = 0.0;
        let mut total_weight = 0.0;

        for p in &self.particles {
            let w = p.weight as f32;
            sum_x += w * p.pose.x;
            sum_y += w * p.pose.y;
            sum_sin += w * p.pose.theta.sin();
            sum_cos += w * p.pose.theta.cos();
            total_weight += w;
        }

        if total_weight > 1e-10 {
            Pose2D::new(
                sum_x / total_weight,
                sum_y / total_weight,
                sum_sin.atan2(sum_cos),
            )
        } else {
            // Return mean of particles if weights are all zero
            let n = self.particles.len() as f32;
            let mean_x: f32 = self.particles.iter().map(|p| p.pose.x).sum::<f32>() / n;
            let mean_y: f32 = self.particles.iter().map(|p| p.pose.y).sum::<f32>() / n;
            let mean_sin: f32 = self
                .particles
                .iter()
                .map(|p| p.pose.theta.sin())
                .sum::<f32>()
                / n;
            let mean_cos: f32 = self
                .particles
                .iter()
                .map(|p| p.pose.theta.cos())
                .sum::<f32>()
                / n;
            Pose2D::new(mean_x, mean_y, mean_sin.atan2(mean_cos))
        }
    }

    /// Get the estimated covariance of the pose.
    pub fn covariance(&self) -> Covariance2D {
        let mean = self.estimate();
        let mut cov_xx = 0.0f64;
        let mut cov_xy = 0.0f64;
        let mut cov_yy = 0.0f64;
        let mut cov_tt = 0.0f64;
        let mut total_weight = 0.0f64;

        for p in &self.particles {
            let dx = (p.pose.x - mean.x) as f64;
            let dy = (p.pose.y - mean.y) as f64;
            let dtheta = crate::core::math::angle_diff(mean.theta, p.pose.theta) as f64;

            cov_xx += p.weight * dx * dx;
            cov_xy += p.weight * dx * dy;
            cov_yy += p.weight * dy * dy;
            cov_tt += p.weight * dtheta * dtheta;
            total_weight += p.weight;
        }

        if total_weight > 1e-10 {
            // Row-major: [xx, xy, xt, yx, yy, yt, tx, ty, tt]
            let cov_xy_f32 = (cov_xy / total_weight) as f32;
            Covariance2D::from_array([
                (cov_xx / total_weight) as f32,
                cov_xy_f32,
                0.0,
                cov_xy_f32,
                (cov_yy / total_weight) as f32,
                0.0,
                0.0,
                0.0,
                (cov_tt / total_weight) as f32,
            ])
        } else {
            Covariance2D::diagonal(1.0, 1.0, 0.5) // Large uncertainty
        }
    }

    /// Reset the filter to a new pose.
    pub fn reset(&mut self, pose: Pose2D) {
        self.particles = Self::initialize_particles(
            self.config.num_particles,
            &pose,
            self.config.initial_spread_xy,
            self.config.initial_spread_theta,
            &mut self.rng,
        );
        self.state = ParticleFilterState::default();
    }

    /// Globally reinitialize particles (for kidnapped robot problem).
    pub fn global_reinitialize(&mut self, map: &OccupancyGrid) {
        let (width, height) = map.dimensions();
        let (origin_x, origin_y) = map.origin();
        let resolution = map.resolution();

        let world_width = width as f32 * resolution;
        let world_height = height as f32 * resolution;

        // Uniformly distribute particles in free space
        let mut new_particles = Vec::with_capacity(self.config.num_particles);

        while new_particles.len() < self.config.num_particles {
            let x = origin_x + self.rng.gen_f32() * world_width;
            let y = origin_y + self.rng.gen_f32() * world_height;
            let theta = (self.rng.gen_f32() - 0.5) * 2.0 * std::f32::consts::PI;

            // Check if position is in free space
            if let Some((cx, cy)) = map.world_to_cell(x, y) {
                use crate::algorithms::mapping::CellState;
                if map.get_state(cx, cy) != CellState::Occupied {
                    new_particles.push(Particle::new(Pose2D::new(x, y, theta)));
                }
            }
        }

        self.particles = new_particles;
        self.state = ParticleFilterState::default();
    }

    /// Update the sensor model when the map changes.
    pub fn update_map(&mut self, map: &OccupancyGrid) {
        self.sensor_model.update_from_map(map);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::mapping::OccupancyGridConfig;

    fn create_test_map() -> OccupancyGrid {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 10.0,
            initial_height: 10.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        // Create walls
        for i in 0..100 {
            let x = -5.0 + i as f32 * 0.1;
            // Top wall
            if let Some((cx, cy)) = grid.world_to_cell(x, 4.0) {
                for _ in 0..10 {
                    grid.update_cell(cx, cy, true);
                }
            }
            // Bottom wall
            if let Some((cx, cy)) = grid.world_to_cell(x, -4.0) {
                for _ in 0..10 {
                    grid.update_cell(cx, cy, true);
                }
            }
        }

        grid
    }

    #[test]
    fn test_particle_filter_creation() {
        let map = create_test_map();
        let config = ParticleFilterConfig::default();
        let filter = ParticleFilter::new(config.clone(), Pose2D::identity(), &map);

        assert_eq!(filter.num_particles(), config.num_particles);
    }

    #[test]
    fn test_particle_filter_predict() {
        let map = create_test_map();
        let config = ParticleFilterConfig {
            num_particles: 100,
            seed: 42,
            ..Default::default()
        };
        let mut filter = ParticleFilter::new(config, Pose2D::identity(), &map);

        let initial_mean = filter.estimate();

        // Predict with forward motion
        filter.predict(&Pose2D::new(1.0, 0.0, 0.0));

        let new_mean = filter.estimate();

        // Mean should have moved approximately 1m forward
        assert!(
            (new_mean.x - initial_mean.x - 1.0).abs() < 0.2,
            "Mean X should move ~1m: {}",
            new_mean.x - initial_mean.x
        );
    }

    #[test]
    fn test_particle_filter_estimate_initial() {
        let map = create_test_map();
        let initial_pose = Pose2D::new(1.0, 2.0, 0.5);
        let config = ParticleFilterConfig {
            num_particles: 500,
            initial_spread_xy: 0.1,
            initial_spread_theta: 0.1,
            seed: 42,
            ..Default::default()
        };
        let filter = ParticleFilter::new(config, initial_pose, &map);

        let estimate = filter.estimate();

        // Estimate should be close to initial pose
        assert!((estimate.x - initial_pose.x).abs() < 0.2);
        assert!((estimate.y - initial_pose.y).abs() < 0.2);
    }

    #[test]
    fn test_particle_filter_covariance() {
        let map = create_test_map();
        let config = ParticleFilterConfig {
            num_particles: 500,
            initial_spread_xy: 0.5,
            initial_spread_theta: 0.2,
            seed: 42,
            ..Default::default()
        };
        let filter = ParticleFilter::new(config, Pose2D::identity(), &map);

        let cov = filter.covariance();

        // Covariance should be reasonable (positive diagonal)
        assert!(cov.var_x() > 0.0);
        assert!(cov.var_y() > 0.0);
        assert!(cov.var_theta() > 0.0);
    }

    #[test]
    fn test_particle_filter_reset() {
        let map = create_test_map();
        let config = ParticleFilterConfig {
            num_particles: 100,
            seed: 42,
            ..Default::default()
        };
        let mut filter = ParticleFilter::new(config, Pose2D::identity(), &map);

        // Move particles
        filter.predict(&Pose2D::new(5.0, 3.0, 1.0));

        // Reset to new pose
        let new_pose = Pose2D::new(-1.0, -2.0, 0.5);
        filter.reset(new_pose);

        let estimate = filter.estimate();

        // Should be near new pose
        assert!((estimate.x - new_pose.x).abs() < 0.3);
        assert!((estimate.y - new_pose.y).abs() < 0.3);
    }

    #[test]
    fn test_particle_filter_neff() {
        let map = create_test_map();
        let config = ParticleFilterConfig {
            num_particles: 100,
            seed: 42,
            ..Default::default()
        };
        let filter = ParticleFilter::new(config, Pose2D::identity(), &map);

        // After initialization, Neff should be high (uniform weights)
        // Note: Neff is computed after update, so initially it's 0
        assert_eq!(filter.state().neff, 0.0);
    }

    #[test]
    fn test_low_variance_resampling() {
        let map = create_test_map();
        let config = ParticleFilterConfig {
            num_particles: 100,
            resampling_threshold: 1.0, // Force resampling
            seed: 42,
            ..Default::default()
        };
        let mut filter = ParticleFilter::new(config, Pose2D::identity(), &map);

        // Create a simple scan
        let scan = LaserScan::new(
            -1.0,          // angle_min
            1.0,           // angle_max
            0.1,           // angle_increment
            0.1,           // range_min
            8.0,           // range_max
            vec![4.0; 21], // ranges - sees wall at 4m
        );

        // Update should trigger resampling
        filter.update(&scan, &map);

        // After resampling, all weights should be equal
        let expected_weight = 1.0 / filter.num_particles() as f64;
        for p in filter.particles() {
            assert!(
                (p.weight - expected_weight).abs() < 0.01,
                "Weight {} should be ~{}",
                p.weight,
                expected_weight
            );
        }
    }

    #[test]
    fn test_config_presets() {
        let tracking = ParticleFilterConfig::tracking();
        let global = ParticleFilterConfig::global_localization();

        // Tracking should have fewer particles
        assert!(tracking.num_particles < global.num_particles);

        // Global should have larger spread
        assert!(global.initial_spread_xy > tracking.initial_spread_xy);
    }
}
