//! Wheel encoder-based odometry calculator
//!
//! Computes robot pose from differential drive wheel encoders.

use vastu_slam::Pose2D;

use crate::utils::normalize_angle;

/// Odometry calculator for differential drive robot
pub struct Odometry {
    /// Distance between wheels in meters
    wheel_base: f32,
    /// Encoder ticks per meter of travel
    ticks_per_meter: f32,
    /// Previous left wheel encoder value
    prev_left: u16,
    /// Previous right wheel encoder value
    prev_right: u16,
    /// Current estimated pose
    pose: Pose2D,
    /// Whether we have received the first encoder reading
    initialized: bool,
}

impl Odometry {
    /// Create a new odometry calculator
    ///
    /// # Arguments
    /// * `wheel_base` - Distance between wheels in meters (CRL-200S: 0.233m)
    /// * `ticks_per_meter` - Encoder ticks per meter (CRL-200S: 4464.0)
    /// * `start_pose` - Initial pose of the robot
    pub fn new(wheel_base: f32, ticks_per_meter: f32, start_pose: Pose2D) -> Self {
        Self {
            wheel_base,
            ticks_per_meter,
            prev_left: 0,
            prev_right: 0,
            pose: start_pose,
            initialized: false,
        }
    }

    /// Update odometry with new encoder readings
    ///
    /// # Arguments
    /// * `left` - Left wheel encoder tick count (u16, wrapping)
    /// * `right` - Right wheel encoder tick count (u16, wrapping)
    ///
    /// # Returns
    /// The updated pose
    pub fn update(&mut self, left: u16, right: u16) -> Pose2D {
        if !self.initialized {
            // First reading - just store the values
            self.prev_left = left;
            self.prev_right = right;
            self.initialized = true;
            return self.pose;
        }

        // Calculate tick deltas (handle u16 wraparound using wrapping_sub)
        let dl = left.wrapping_sub(self.prev_left) as i16;
        let dr = right.wrapping_sub(self.prev_right) as i16;

        // Update stored values
        self.prev_left = left;
        self.prev_right = right;

        // Skip if no movement
        if dl == 0 && dr == 0 {
            return self.pose;
        }

        // Convert ticks to meters
        let dist_left = dl as f32 / self.ticks_per_meter;
        let dist_right = dr as f32 / self.ticks_per_meter;

        // Differential drive kinematics
        let dist = (dist_left + dist_right) / 2.0;
        let dtheta = (dist_right - dist_left) / self.wheel_base;

        // Update pose using mid-point integration
        // For more accuracy with rotation, we use the average heading
        let mid_theta = self.pose.theta + dtheta / 2.0;
        self.pose.x += dist * mid_theta.cos();
        self.pose.y += dist * mid_theta.sin();
        self.pose.theta += dtheta;

        // Normalize theta to [-pi, pi]
        self.pose.theta = normalize_angle(self.pose.theta);

        self.pose
    }

    /// Get the current pose
    pub fn pose(&self) -> Pose2D {
        self.pose
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_straight_line() {
        let mut odom = Odometry::new(0.233, 4464.0, Pose2D::new(0.0, 0.0, 0.0));

        // Initialize
        odom.update(0, 0);

        // Move forward 1 meter (4464 ticks on each wheel)
        let pose = odom.update(4464, 4464);

        assert!((pose.x - 1.0).abs() < 0.01);
        assert!(pose.y.abs() < 0.01);
        assert!(pose.theta.abs() < 0.01);
    }

    #[test]
    fn test_rotation() {
        let mut odom = Odometry::new(0.233, 4464.0, Pose2D::new(0.0, 0.0, 0.0));

        // Initialize
        odom.update(0, 0);

        // Rotate in place: right wheel forward, left wheel backward
        // For a 90 degree turn: arc length = theta * radius = (pi/2) * (0.233/2)
        // ticks = arc_length * ticks_per_meter
        let arc_ticks = (std::f32::consts::FRAC_PI_2 * 0.233 / 2.0 * 4464.0) as u16;
        let pose = odom.update(0u16.wrapping_sub(arc_ticks), arc_ticks);

        // Should have rotated approximately 90 degrees (pi/2 radians)
        assert!(pose.x.abs() < 0.05); // Minimal translation
        assert!(pose.y.abs() < 0.05);
        assert!((pose.theta - std::f32::consts::FRAC_PI_2).abs() < 0.1);
    }
}
