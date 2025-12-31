//! Odometry motion model for SLAM.
//!
//! Implements the probabilistic motion model for differential drive robots,
//! following the formulation from "Probabilistic Robotics" (Thrun et al.).

use serde::{Deserialize, Serialize};

use super::Pose2D;

/// Odometry motion model parameters.
///
/// The model uses four noise parameters (alpha1-4) that characterize
/// the uncertainty in robot motion:
///
/// - `alpha1`: Rotation noise from rotation
/// - `alpha2`: Rotation noise from translation
/// - `alpha3`: Translation noise from translation
/// - `alpha4`: Translation noise from rotation
///
/// # Example
///
/// ```
/// use vastu_slam::core::MotionModel;
///
/// // Default model for typical indoor robot
/// let model = MotionModel::default();
///
/// // Custom model for high-precision robot
/// let precise = MotionModel::new(0.01, 0.01, 0.02, 0.01);
/// ```
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MotionModel {
    /// Rotation noise from rotation (rad/rad)
    pub alpha1: f32,

    /// Rotation noise from translation (rad/m)
    pub alpha2: f32,

    /// Translation noise from translation (m/m)
    pub alpha3: f32,

    /// Translation noise from rotation (m/rad)
    pub alpha4: f32,
}

impl Default for MotionModel {
    fn default() -> Self {
        // Typical values for indoor mobile robot
        Self {
            alpha1: 0.05, // 5% rotation noise from rotation
            alpha2: 0.02, // 2% rotation noise from translation
            alpha3: 0.05, // 5% translation noise from translation
            alpha4: 0.02, // 2% translation noise from rotation
        }
    }
}

impl MotionModel {
    /// Create a new motion model with specified parameters
    pub fn new(alpha1: f32, alpha2: f32, alpha3: f32, alpha4: f32) -> Self {
        Self {
            alpha1,
            alpha2,
            alpha3,
            alpha4,
        }
    }

    /// Create a low-noise motion model for high-precision robots
    pub fn low_noise() -> Self {
        Self {
            alpha1: 0.01,
            alpha2: 0.005,
            alpha3: 0.01,
            alpha4: 0.005,
        }
    }

    /// Create a high-noise motion model for robots with poor odometry
    pub fn high_noise() -> Self {
        Self {
            alpha1: 0.15,
            alpha2: 0.05,
            alpha3: 0.15,
            alpha4: 0.05,
        }
    }

    /// Predict the next pose given current pose and odometry delta.
    ///
    /// This is the deterministic motion model (mean prediction).
    ///
    /// # Arguments
    /// * `pose` - Current robot pose
    /// * `odom_delta` - Odometry change (dx, dy, dtheta in robot frame)
    ///
    /// # Returns
    /// Predicted pose after motion
    pub fn predict(&self, pose: &Pose2D, odom_delta: &Pose2D) -> Pose2D {
        pose.compose(odom_delta)
    }

    /// Compute the expected standard deviation of the motion.
    ///
    /// Returns (linear_stddev, angular_stddev) based on the motion magnitude.
    ///
    /// # Arguments
    /// * `odom_delta` - Odometry change
    ///
    /// # Returns
    /// Tuple of (linear standard deviation, angular standard deviation)
    pub fn motion_uncertainty(&self, odom_delta: &Pose2D) -> (f32, f32) {
        let translation = (odom_delta.x * odom_delta.x + odom_delta.y * odom_delta.y).sqrt();
        let rotation = odom_delta.theta.abs();

        // Linear uncertainty
        let linear_var = self.alpha3 * translation + self.alpha4 * rotation;
        let linear_stddev = linear_var.sqrt().max(0.001); // Minimum 1mm

        // Angular uncertainty
        let angular_var = self.alpha1 * rotation + self.alpha2 * translation;
        let angular_stddev = angular_var.sqrt().max(0.001); // Minimum 0.06 degrees

        (linear_stddev, angular_stddev)
    }

    /// Compute the covariance matrix for the motion.
    ///
    /// Returns a 3x3 covariance matrix [xx, xy, xt, yx, yy, yt, tx, ty, tt]
    /// where x, y are position and t is theta.
    ///
    /// # Arguments
    /// * `odom_delta` - Odometry change
    ///
    /// # Returns
    /// 3x3 covariance matrix as a 9-element array (row-major)
    pub fn motion_covariance(&self, odom_delta: &Pose2D) -> [f32; 9] {
        let (linear_std, angular_std) = self.motion_uncertainty(odom_delta);
        let linear_var = linear_std * linear_std;
        let angular_var = angular_std * angular_std;

        // Simple diagonal covariance (assumes independent x, y, theta)
        // For a more accurate model, this should account for the heading
        [
            linear_var,
            0.0,
            0.0, // Row 1: xx, xy, xt
            0.0,
            linear_var,
            0.0, // Row 2: yx, yy, yt
            0.0,
            0.0,
            angular_var, // Row 3: tx, ty, tt
        ]
    }
}

/// Odometry reading from wheel encoders.
#[derive(Clone, Debug, Default)]
pub struct Odometry {
    /// Timestamp in seconds
    pub timestamp: f64,

    /// Left wheel distance traveled (meters)
    pub left_distance: f32,

    /// Right wheel distance traveled (meters)
    pub right_distance: f32,

    /// Wheel base (distance between wheels) in meters
    pub wheel_base: f32,
}

impl Odometry {
    /// Create new odometry reading
    pub fn new(timestamp: f64, left: f32, right: f32, wheel_base: f32) -> Self {
        Self {
            timestamp,
            left_distance: left,
            right_distance: right,
            wheel_base,
        }
    }

    /// Convert to pose delta (dx, dy, dtheta) in robot frame.
    ///
    /// Uses differential drive kinematics to compute the pose change.
    pub fn to_pose_delta(&self) -> Pose2D {
        let delta_left = self.left_distance;
        let delta_right = self.right_distance;

        // Differential drive kinematics
        let delta_trans = (delta_left + delta_right) / 2.0;
        let delta_rot = (delta_right - delta_left) / self.wheel_base;

        // In robot frame: forward motion along x, no lateral motion
        Pose2D::new(delta_trans, 0.0, delta_rot)
    }

    /// Compute pose delta between two odometry readings.
    pub fn delta_from(&self, previous: &Odometry) -> Pose2D {
        let delta_left = self.left_distance - previous.left_distance;
        let delta_right = self.right_distance - previous.right_distance;

        let delta_trans = (delta_left + delta_right) / 2.0;
        let delta_rot = (delta_right - delta_left) / self.wheel_base;

        Pose2D::new(delta_trans, 0.0, delta_rot)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motion_model_default() {
        let model = MotionModel::default();
        assert!(model.alpha1 > 0.0);
        assert!(model.alpha3 > 0.0);
    }

    #[test]
    fn test_predict_straight_motion() {
        let model = MotionModel::default();
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let delta = Pose2D::new(1.0, 0.0, 0.0); // Move 1m forward

        let predicted = model.predict(&pose, &delta);

        assert!((predicted.x - 1.0).abs() < 1e-6);
        assert!((predicted.y - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_motion_uncertainty() {
        let model = MotionModel::default();

        // Large motion should have large uncertainty
        let large_delta = Pose2D::new(1.0, 0.0, 0.5);
        let (lin_std, ang_std) = model.motion_uncertainty(&large_delta);
        assert!(lin_std > 0.0);
        assert!(ang_std > 0.0);

        // Small motion should have small uncertainty
        let small_delta = Pose2D::new(0.01, 0.0, 0.01);
        let (small_lin, small_ang) = model.motion_uncertainty(&small_delta);
        assert!(small_lin < lin_std);
        assert!(small_ang < ang_std);
    }

    #[test]
    fn test_odometry_to_delta() {
        let odom = Odometry::new(0.0, 0.5, 0.5, 0.3);
        let delta = odom.to_pose_delta();

        // Equal wheel distances = straight line motion
        assert!((delta.x - 0.5).abs() < 1e-6);
        assert!((delta.theta).abs() < 1e-6);
    }

    #[test]
    fn test_odometry_rotation() {
        let odom = Odometry::new(0.0, -0.15, 0.15, 0.3);
        let delta = odom.to_pose_delta();

        // Opposite wheel distances = pure rotation
        assert!((delta.x).abs() < 1e-6); // No forward motion
        assert!((delta.theta - 1.0).abs() < 1e-6); // 1 radian rotation
    }
}
