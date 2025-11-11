//! Motion constraints and safety limits

use crate::config::SangamConfig;

/// Motion constraints for safe operation
#[derive(Debug, Clone)]
pub struct MotionConstraints {
    /// Maximum linear velocity (m/s)
    pub max_linear_velocity: f32,

    /// Maximum angular velocity (rad/s)
    pub max_angular_velocity: f32,

    /// Linear acceleration limit (m/s²)
    pub linear_acceleration: f32,

    /// Angular acceleration limit (rad/s²)
    pub angular_acceleration: f32,

    /// Emergency deceleration (m/s²)
    pub emergency_deceleration: f32,
}

impl MotionConstraints {
    /// Create from config
    pub fn from_config(config: &SangamConfig) -> Self {
        Self {
            max_linear_velocity: config.max_linear_velocity,
            max_angular_velocity: config.max_angular_velocity,
            linear_acceleration: config.linear_acceleration,
            angular_acceleration: config.angular_acceleration,
            emergency_deceleration: config.emergency_deceleration,
        }
    }

    /// Apply velocity constraints
    pub fn constrain_velocity(&self, linear: f32, angular: f32) -> (f32, f32) {
        let constrained_linear = linear
            .max(-self.max_linear_velocity)
            .min(self.max_linear_velocity);

        let constrained_angular = angular
            .max(-self.max_angular_velocity)
            .min(self.max_angular_velocity);

        (constrained_linear, constrained_angular)
    }

    /// Apply acceleration constraints
    /// Returns new velocity after applying acceleration limits
    pub fn apply_acceleration(
        &self,
        current_linear: f32,
        current_angular: f32,
        target_linear: f32,
        target_angular: f32,
        dt: f32,
        emergency: bool,
    ) -> (f32, f32) {
        // Use emergency deceleration if needed
        let linear_accel_limit = if emergency {
            self.emergency_deceleration
        } else {
            self.linear_acceleration
        };

        let angular_accel_limit = if emergency {
            self.emergency_deceleration * 2.0 // Angular can decelerate faster
        } else {
            self.angular_acceleration
        };

        // Calculate required acceleration
        let linear_accel = (target_linear - current_linear) / dt;
        let angular_accel = (target_angular - current_angular) / dt;

        // Apply limits
        let constrained_linear_accel = linear_accel
            .max(-linear_accel_limit)
            .min(linear_accel_limit);

        let constrained_angular_accel = angular_accel
            .max(-angular_accel_limit)
            .min(angular_accel_limit);

        // Calculate new velocities
        let new_linear = current_linear + constrained_linear_accel * dt;
        let new_angular = current_angular + constrained_angular_accel * dt;

        // Apply velocity constraints
        self.constrain_velocity(new_linear, new_angular)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_velocity_constraints() {
        let config = SangamConfig::default();
        let constraints = MotionConstraints::from_config(&config);

        // Test within limits
        let (linear, angular) = constraints.constrain_velocity(0.2, 0.5);
        assert_eq!(linear, 0.2);
        assert_eq!(angular, 0.5);

        // Test exceeding limits
        let (linear, angular) = constraints.constrain_velocity(1.0, 3.0);
        assert_eq!(linear, config.max_linear_velocity);
        assert_eq!(angular, config.max_angular_velocity);

        // Test negative limits
        let (linear, angular) = constraints.constrain_velocity(-1.0, -3.0);
        assert_eq!(linear, -config.max_linear_velocity);
        assert_eq!(angular, -config.max_angular_velocity);
    }

    #[test]
    fn test_acceleration_limits() {
        let config = SangamConfig::default();
        let constraints = MotionConstraints::from_config(&config);

        // Test normal acceleration
        let (new_linear, _) = constraints.apply_acceleration(
            0.0, 0.0, // current
            0.3, 0.0,   // target
            1.0,   // dt = 1 second
            false, // not emergency
        );

        // Should be limited by acceleration
        assert!(new_linear <= config.linear_acceleration);

        // Test emergency stop
        let (new_linear, _) = constraints.apply_acceleration(
            0.3, 0.0, // current
            0.0, 0.0,  // target (stop)
            0.1,  // dt = 0.1 second
            true, // emergency
        );

        // Should decelerate faster
        assert!(new_linear < 0.3);
        assert!(new_linear >= 0.0);
    }
}
