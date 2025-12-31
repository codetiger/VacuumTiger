//! Velocity computation utilities for exploration.
//!
//! Shared functions for computing linear and angular velocities
//! to navigate towards target positions.

use crate::core::{Pose2D, WorldPoint, normalize_angle};

/// Configuration for velocity computation.
#[derive(Clone, Debug)]
pub struct VelocityConfig {
    /// Angle threshold (radians) - turn in place if angle error exceeds this.
    /// Default: 0.3 rad (~17 degrees)
    pub turn_threshold: f32,

    /// Proportional gain for angular velocity.
    /// Default: 2.0
    pub angular_gain: f32,

    /// Maximum angular velocity (rad/s).
    /// Default: 2.0
    pub max_angular_velocity: f32,

    /// Distance threshold to consider target reached (meters).
    /// Default: 0.05m (5cm)
    pub reached_threshold: f32,

    /// Angle threshold to consider heading reached (radians).
    /// Default: 0.05 rad (~3 degrees)
    pub heading_reached_threshold: f32,
}

impl Default for VelocityConfig {
    fn default() -> Self {
        Self {
            turn_threshold: 0.3,
            angular_gain: 2.0,
            max_angular_velocity: 2.0,
            reached_threshold: 0.05,
            heading_reached_threshold: 0.05,
        }
    }
}

/// Compute velocity command to move towards a target position.
///
/// Returns (linear_velocity, angular_velocity) in (m/s, rad/s).
///
/// The behavior is:
/// 1. If angle to target exceeds `turn_threshold`, rotate in place first
/// 2. Otherwise, move forward with proportional angular correction
///
/// # Arguments
/// * `pose` - Current robot pose
/// * `target` - Target position in world coordinates
/// * `max_speed` - Maximum linear speed (m/s)
/// * `config` - Velocity computation configuration
pub fn compute_velocity_to_target(
    pose: Pose2D,
    target: WorldPoint,
    max_speed: f32,
    config: &VelocityConfig,
) -> (f32, f32) {
    let dx = target.x - pose.x;
    let dy = target.y - pose.y;
    let distance = (dx * dx + dy * dy).sqrt();

    // Target reached
    if distance < config.reached_threshold {
        return (0.0, 0.0);
    }

    let target_angle = dy.atan2(dx);
    let angle_error = normalize_angle(target_angle - pose.theta);

    // Turn towards target first if angle is large
    if angle_error.abs() > config.turn_threshold {
        let angular = angle_error.signum()
            * config
                .max_angular_velocity
                .min(angle_error.abs() * config.angular_gain);
        return (0.0, angular);
    }

    // Move forward with proportional angular correction
    let linear = max_speed.min(distance);
    let angular = (angle_error * config.angular_gain)
        .clamp(-config.max_angular_velocity, config.max_angular_velocity);

    (linear, angular)
}

/// Compute angular velocity to rotate to a target heading.
///
/// Returns angular velocity in rad/s.
///
/// # Arguments
/// * `current_heading` - Current heading (radians)
/// * `target_heading` - Target heading (radians)
/// * `max_speed` - Maximum angular speed (rad/s)
/// * `config` - Velocity computation configuration
pub fn compute_angular_velocity_to_heading(
    current_heading: f32,
    target_heading: f32,
    max_speed: f32,
    config: &VelocityConfig,
) -> f32 {
    let error = normalize_angle(target_heading - current_heading);

    if error.abs() < config.heading_reached_threshold {
        0.0
    } else {
        (error * config.angular_gain).clamp(-max_speed, max_speed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_target_reached() {
        let config = VelocityConfig::default();
        let pose = Pose2D::new(1.0, 1.0, 0.0);
        let target = WorldPoint::new(1.02, 1.02); // Within 5cm

        let (linear, angular) = compute_velocity_to_target(pose, target, 0.5, &config);
        assert_eq!(linear, 0.0);
        assert_eq!(angular, 0.0);
    }

    #[test]
    fn test_turn_first_when_angle_large() {
        let config = VelocityConfig::default();
        let pose = Pose2D::new(0.0, 0.0, 0.0); // Facing +X
        let target = WorldPoint::new(0.0, 1.0); // Target is +Y (90 degrees away)

        let (linear, angular) = compute_velocity_to_target(pose, target, 0.5, &config);
        assert_eq!(linear, 0.0); // Should not move forward
        assert!(angular > 0.0); // Should turn left (positive)
    }

    #[test]
    fn test_move_forward_when_aligned() {
        let config = VelocityConfig::default();
        let pose = Pose2D::new(0.0, 0.0, 0.0); // Facing +X
        let target = WorldPoint::new(1.0, 0.0); // Target straight ahead

        let (linear, angular) = compute_velocity_to_target(pose, target, 0.5, &config);
        assert!(linear > 0.0); // Should move forward
        assert!(angular.abs() < 0.01); // Minimal angular correction
    }

    #[test]
    fn test_heading_reached() {
        let config = VelocityConfig::default();
        let angular = compute_angular_velocity_to_heading(0.0, 0.02, 1.0, &config);
        assert_eq!(angular, 0.0); // Within threshold
    }

    #[test]
    fn test_rotate_to_heading() {
        let config = VelocityConfig::default();
        let angular = compute_angular_velocity_to_heading(0.0, PI / 2.0, 1.0, &config);
        assert!(angular > 0.0); // Should turn positive (counter-clockwise)
    }
}
