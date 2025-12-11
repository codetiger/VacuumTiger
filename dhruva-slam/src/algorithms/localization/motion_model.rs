//! Odometry-based motion model for particle filter.
//!
//! Implements the standard odometry motion model from Probabilistic Robotics
//! (Thrun et al.) which samples from a distribution of possible poses given
//! an odometry measurement.
//!
//! The model decomposes motion into:
//! 1. Initial rotation to face the target
//! 2. Translation toward the target
//! 3. Final rotation to match the target heading
//!
//! Each component has associated noise controlled by alpha parameters.

use crate::core::math::normalize_angle;
use crate::core::types::Pose2D;

/// Configuration for the odometry motion model.
///
/// The alpha parameters control noise proportional to motion:
/// - `alpha1`: Rotation noise from rotation (rad/rad)
/// - `alpha2`: Rotation noise from translation (rad/m)
/// - `alpha3`: Translation noise from translation (m/m)
/// - `alpha4`: Translation noise from rotation (m/rad)
#[derive(Debug, Clone, Copy)]
pub struct MotionModelConfig {
    /// Rotation noise from rotation (rad/rad).
    /// Typical: 0.1-0.2 for differential drive.
    pub alpha1: f32,

    /// Rotation noise from translation (rad/m).
    /// Typical: 0.05-0.1 for differential drive.
    pub alpha2: f32,

    /// Translation noise from translation (m/m).
    /// Typical: 0.1-0.2 for differential drive.
    pub alpha3: f32,

    /// Translation noise from rotation (m/rad).
    /// Typical: 0.05-0.1 for differential drive.
    pub alpha4: f32,
}

impl Default for MotionModelConfig {
    fn default() -> Self {
        // Conservative defaults for indoor differential drive robot
        Self {
            alpha1: 0.15,
            alpha2: 0.08,
            alpha3: 0.15,
            alpha4: 0.08,
        }
    }
}

impl MotionModelConfig {
    /// Create a low-noise configuration (high quality encoders).
    pub fn low_noise() -> Self {
        Self {
            alpha1: 0.05,
            alpha2: 0.02,
            alpha3: 0.05,
            alpha4: 0.02,
        }
    }

    /// Create a high-noise configuration (slippery floors, poor encoders).
    pub fn high_noise() -> Self {
        Self {
            alpha1: 0.3,
            alpha2: 0.15,
            alpha3: 0.3,
            alpha4: 0.15,
        }
    }
}

/// Odometry motion model for sampling particle poses.
///
/// Uses the "sample_motion_model_odometry" algorithm from Probabilistic Robotics.
#[derive(Debug, Clone)]
pub struct MotionModel {
    config: MotionModelConfig,
}

impl MotionModel {
    /// Create a new motion model with the given configuration.
    pub fn new(config: MotionModelConfig) -> Self {
        Self { config }
    }

    /// Get the configuration.
    pub fn config(&self) -> &MotionModelConfig {
        &self.config
    }

    /// Sample a new pose given current pose and odometry delta.
    ///
    /// This implements the odometry motion model algorithm:
    /// 1. Decompose delta into (rot1, trans, rot2)
    /// 2. Add noise proportional to motion magnitude
    /// 3. Apply noisy motion to current pose
    ///
    /// # Arguments
    /// * `pose` - Current particle pose
    /// * `odom_delta` - Measured odometry change (from wheel encoders)
    /// * `rng` - Random number generator for noise sampling
    ///
    /// # Returns
    /// New sampled pose with motion noise applied.
    pub fn sample<R: Rng>(&self, pose: &Pose2D, odom_delta: &Pose2D, rng: &mut R) -> Pose2D {
        // Extract motion components
        let delta_trans = (odom_delta.x * odom_delta.x + odom_delta.y * odom_delta.y).sqrt();

        // Handle near-zero motion (avoid division by zero)
        if delta_trans < 1e-6 && odom_delta.theta.abs() < 1e-6 {
            return *pose;
        }

        // Decompose into rotation-translation-rotation
        let delta_rot1 = if delta_trans > 1e-6 {
            normalize_angle(odom_delta.y.atan2(odom_delta.x))
        } else {
            0.0
        };
        let delta_rot2 = normalize_angle(odom_delta.theta - delta_rot1);

        // Compute noise standard deviations
        let rot1_abs = delta_rot1.abs();
        let rot2_abs = delta_rot2.abs();

        let sigma_rot1 = (self.config.alpha1 * rot1_abs + self.config.alpha2 * delta_trans).sqrt();
        let sigma_trans =
            (self.config.alpha3 * delta_trans + self.config.alpha4 * (rot1_abs + rot2_abs)).sqrt();
        let sigma_rot2 = (self.config.alpha1 * rot2_abs + self.config.alpha2 * delta_trans).sqrt();

        // Sample noisy motion
        let noisy_rot1 = delta_rot1 + sample_gaussian(rng, sigma_rot1);
        let noisy_trans = delta_trans + sample_gaussian(rng, sigma_trans);
        let noisy_rot2 = delta_rot2 + sample_gaussian(rng, sigma_rot2);

        // Apply noisy motion to pose
        let new_theta = normalize_angle(pose.theta + noisy_rot1);
        let new_x = pose.x + noisy_trans * new_theta.cos();
        let new_y = pose.y + noisy_trans * new_theta.sin();
        let final_theta = normalize_angle(new_theta + noisy_rot2);

        Pose2D::new(new_x, new_y, final_theta)
    }

    /// Compute the probability of reaching `pose_new` from `pose_old` given odometry.
    ///
    /// This is the probability density p(pose_new | pose_old, odom_delta).
    /// Useful for importance weighting when odometry is the proposal distribution.
    ///
    /// # Arguments
    /// * `pose_new` - Target pose
    /// * `pose_old` - Starting pose
    /// * `odom_delta` - Measured odometry change
    ///
    /// # Returns
    /// Log probability density (for numerical stability).
    pub fn log_probability(
        &self,
        pose_new: &Pose2D,
        pose_old: &Pose2D,
        odom_delta: &Pose2D,
    ) -> f32 {
        // Compute actual motion from pose_old to pose_new
        let actual_dx = pose_new.x - pose_old.x;
        let actual_dy = pose_new.y - pose_old.y;
        let actual_trans = (actual_dx * actual_dx + actual_dy * actual_dy).sqrt();

        // Decompose odometry delta
        let delta_trans = (odom_delta.x * odom_delta.x + odom_delta.y * odom_delta.y).sqrt();

        if delta_trans < 1e-6 && odom_delta.theta.abs() < 1e-6 {
            // No motion expected, check if pose changed
            if actual_trans < 1e-4 && normalize_angle(pose_new.theta - pose_old.theta).abs() < 1e-4
            {
                return 0.0; // log(1) for identity
            }
            return f32::NEG_INFINITY;
        }

        let delta_rot1 = if delta_trans > 1e-6 {
            normalize_angle(odom_delta.y.atan2(odom_delta.x))
        } else {
            0.0
        };
        let delta_rot2 = normalize_angle(odom_delta.theta - delta_rot1);

        // Compute actual motion components
        let actual_rot1 = if actual_trans > 1e-6 {
            normalize_angle(actual_dy.atan2(actual_dx) - pose_old.theta)
        } else {
            0.0
        };
        let actual_rot2 =
            normalize_angle(normalize_angle(pose_new.theta - pose_old.theta) - actual_rot1);

        // Compute noise variances
        let rot1_abs = delta_rot1.abs();
        let rot2_abs = delta_rot2.abs();

        let var_rot1 = self.config.alpha1 * rot1_abs + self.config.alpha2 * delta_trans;
        let var_trans =
            self.config.alpha3 * delta_trans + self.config.alpha4 * (rot1_abs + rot2_abs);
        let var_rot2 = self.config.alpha1 * rot2_abs + self.config.alpha2 * delta_trans;

        // Compute log probabilities (Gaussian)
        let log_p_rot1 = log_gaussian_probability(actual_rot1 - delta_rot1, var_rot1);
        let log_p_trans = log_gaussian_probability(actual_trans - delta_trans, var_trans);
        let log_p_rot2 = log_gaussian_probability(actual_rot2 - delta_rot2, var_rot2);

        log_p_rot1 + log_p_trans + log_p_rot2
    }
}

/// Trait for random number generation (abstracted for testing).
pub trait Rng {
    /// Generate a random f32 in [0, 1).
    fn gen_f32(&mut self) -> f32;

    /// Generate a random f32 from standard normal distribution.
    fn gen_standard_normal(&mut self) -> f32;
}

/// Sample from Gaussian distribution with zero mean.
fn sample_gaussian<R: Rng>(rng: &mut R, sigma: f32) -> f32 {
    if sigma < 1e-10 {
        return 0.0;
    }
    rng.gen_standard_normal() * sigma
}

/// Compute log probability of value under zero-mean Gaussian.
fn log_gaussian_probability(x: f32, variance: f32) -> f32 {
    if variance < 1e-10 {
        if x.abs() < 1e-10 {
            return 0.0;
        }
        return f32::NEG_INFINITY;
    }
    let _sigma = variance.sqrt();
    -0.5 * (x * x / variance + (2.0 * std::f32::consts::PI * variance).ln())
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Simple LCG-based RNG for deterministic testing.
    #[derive(Debug, Clone)]
    struct SimpleRng {
        state: u64,
    }

    impl SimpleRng {
        fn new(seed: u64) -> Self {
            Self { state: seed }
        }

        fn next_u64(&mut self) -> u64 {
            // LCG parameters from Numerical Recipes
            self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1);
            self.state
        }
    }

    impl Rng for SimpleRng {
        fn gen_f32(&mut self) -> f32 {
            (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
        }

        fn gen_standard_normal(&mut self) -> f32 {
            // Box-Muller transform
            let u1 = self.gen_f32().max(1e-10);
            let u2 = self.gen_f32();
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = 2.0 * std::f32::consts::PI * u2;
            r * theta.cos()
        }
    }

    #[test]
    fn test_motion_model_no_motion() {
        let model = MotionModel::new(MotionModelConfig::default());
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let delta = Pose2D::identity();
        let mut rng = SimpleRng::new(42);

        let new_pose = model.sample(&pose, &delta, &mut rng);

        // With no motion, pose should be unchanged
        assert_relative_eq!(new_pose.x, pose.x, epsilon = 1e-6);
        assert_relative_eq!(new_pose.y, pose.y, epsilon = 1e-6);
        assert_relative_eq!(new_pose.theta, pose.theta, epsilon = 1e-6);
    }

    #[test]
    fn test_motion_model_forward_motion() {
        let model = MotionModel::new(MotionModelConfig::low_noise());
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let delta = Pose2D::new(1.0, 0.0, 0.0); // 1m forward
        let mut rng = SimpleRng::new(42);

        // Sample multiple times and check mean is near expected
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let n = 1000;

        for _ in 0..n {
            let new_pose = model.sample(&pose, &delta, &mut rng);
            sum_x += new_pose.x;
            sum_y += new_pose.y;
        }

        let mean_x = sum_x / n as f32;
        let mean_y = sum_y / n as f32;

        // Mean should be approximately at (1, 0) with some noise
        assert!((mean_x - 1.0).abs() < 0.1, "Mean X: {}", mean_x);
        assert!(mean_y.abs() < 0.1, "Mean Y: {}", mean_y);
    }

    #[test]
    fn test_motion_model_rotation() {
        let model = MotionModel::new(MotionModelConfig::low_noise());
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let delta = Pose2D::new(0.0, 0.0, std::f32::consts::PI / 2.0); // 90° rotation
        let mut rng = SimpleRng::new(42);

        // Sample multiple times
        let mut sum_theta = 0.0;
        let n = 1000;

        for _ in 0..n {
            let new_pose = model.sample(&pose, &delta, &mut rng);
            sum_theta += new_pose.theta;
        }

        let mean_theta = sum_theta / n as f32;

        // Mean should be approximately PI/2
        assert!(
            (mean_theta - std::f32::consts::PI / 2.0).abs() < 0.2,
            "Mean theta: {}",
            mean_theta
        );
    }

    #[test]
    fn test_motion_model_spread_increases_with_noise() {
        let low_noise = MotionModel::new(MotionModelConfig::low_noise());
        let high_noise = MotionModel::new(MotionModelConfig::high_noise());
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let delta = Pose2D::new(1.0, 0.0, 0.5);
        let n = 500;

        // Compute variance for low noise
        let mut rng = SimpleRng::new(42);
        let mut sum_sq_low = 0.0;
        for _ in 0..n {
            let new_pose = low_noise.sample(&pose, &delta, &mut rng);
            let dx = new_pose.x - 1.0;
            let dy = new_pose.y;
            sum_sq_low += dx * dx + dy * dy;
        }

        // Compute variance for high noise
        let mut rng = SimpleRng::new(42);
        let mut sum_sq_high = 0.0;
        for _ in 0..n {
            let new_pose = high_noise.sample(&pose, &delta, &mut rng);
            let dx = new_pose.x - 1.0;
            let dy = new_pose.y;
            sum_sq_high += dx * dx + dy * dy;
        }

        // High noise should have larger spread
        assert!(
            sum_sq_high > sum_sq_low,
            "High noise spread {} should be > low noise spread {}",
            sum_sq_high,
            sum_sq_low
        );
    }

    #[test]
    fn test_log_probability_identity() {
        let model = MotionModel::new(MotionModelConfig::default());
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let delta = Pose2D::identity();

        // No motion expected, no motion occurred
        let log_p = model.log_probability(&pose, &pose, &delta);
        assert!(log_p.is_finite());
        assert!(log_p >= -1.0); // Should be high probability
    }

    #[test]
    fn test_log_probability_expected_motion() {
        let model = MotionModel::new(MotionModelConfig::default());
        let pose_old = Pose2D::new(0.0, 0.0, 0.0);
        let delta = Pose2D::new(1.0, 0.0, 0.0);
        let pose_new = Pose2D::new(1.0, 0.0, 0.0);

        // Motion exactly as expected
        let log_p = model.log_probability(&pose_new, &pose_old, &delta);
        assert!(log_p.is_finite());
    }

    #[test]
    fn test_log_probability_unexpected_motion() {
        let model = MotionModel::new(MotionModelConfig::low_noise());
        let pose_old = Pose2D::new(0.0, 0.0, 0.0);
        let delta = Pose2D::new(1.0, 0.0, 0.0); // Expected 1m forward
        let pose_new = Pose2D::new(0.0, 5.0, 0.0); // Actually 5m sideways

        // Very unexpected motion should have low probability
        let log_p = model.log_probability(&pose_new, &pose_old, &delta);
        assert!(
            log_p < -10.0,
            "Unexpected motion should have low prob: {}",
            log_p
        );
    }

    #[test]
    fn test_simple_rng_deterministic() {
        let mut rng1 = SimpleRng::new(42);
        let mut rng2 = SimpleRng::new(42);

        for _ in 0..100 {
            assert_eq!(rng1.gen_f32(), rng2.gen_f32());
        }
    }

    #[test]
    fn test_simple_rng_range() {
        let mut rng = SimpleRng::new(12345);

        for _ in 0..1000 {
            let v = rng.gen_f32();
            assert!((0.0..1.0).contains(&v), "Value out of range: {}", v);
        }
    }

    #[test]
    fn test_config_presets() {
        let default = MotionModelConfig::default();
        let low = MotionModelConfig::low_noise();
        let high = MotionModelConfig::high_noise();

        // Verify ordering
        assert!(low.alpha1 < default.alpha1);
        assert!(default.alpha1 < high.alpha1);
    }
}
