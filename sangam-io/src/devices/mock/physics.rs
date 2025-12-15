//! Physics engine for differential drive robot simulation
//!
//! Implements kinematics and collision handling.

use super::config::RobotConfig;
use super::map_loader::SimulationMap;
use std::f32::consts::{PI, TAU};

/// Collision handling mode
#[derive(Debug, Clone)]
pub enum CollisionMode {
    /// Stop on collision
    Stop,
    /// Slide along obstacle surface
    Slide { friction: f32 },
    /// Pass through obstacles (for debugging)
    Passthrough,
}

impl CollisionMode {
    /// Parse collision mode from config string
    pub fn from_config(mode: &str, friction: f32) -> Self {
        match mode {
            "slide" => Self::Slide { friction },
            "passthrough" => Self::Passthrough,
            _ => Self::Stop,
        }
    }
}

/// Physics state for the simulated robot
pub struct PhysicsState {
    /// X position in world frame (meters)
    x: f32,
    /// Y position in world frame (meters)
    y: f32,
    /// Orientation angle (radians, CCW from +X)
    theta: f32,
    /// Collision handling mode
    collision_mode: CollisionMode,
    /// Robot collision radius
    robot_radius: f32,
}

impl PhysicsState {
    /// Create new physics state at given pose
    pub fn new(x: f32, y: f32, theta: f32, config: &RobotConfig) -> Self {
        Self {
            x,
            y,
            theta: normalize_angle(theta),
            collision_mode: CollisionMode::from_config(
                &config.collision_mode,
                config.slide_friction,
            ),
            robot_radius: config.robot_radius,
        }
    }

    /// Get current X position
    #[inline]
    pub fn x(&self) -> f32 {
        self.x
    }

    /// Get current Y position
    #[inline]
    pub fn y(&self) -> f32 {
        self.y
    }

    /// Get current orientation
    #[inline]
    pub fn theta(&self) -> f32 {
        self.theta
    }

    /// Update physics state based on velocity commands
    ///
    /// Returns true if a collision occurred.
    pub fn update(
        &mut self,
        dt: f32,
        linear_vel: f32,
        angular_vel: f32,
        map: &SimulationMap,
        config: &RobotConfig,
    ) -> bool {
        // Clamp velocities to limits
        let linear_vel = linear_vel.clamp(-config.max_linear_speed, config.max_linear_speed);
        let angular_vel = angular_vel.clamp(-config.max_angular_speed, config.max_angular_speed);

        // Compute new pose using differential drive kinematics
        let (new_x, new_y, new_theta) = if angular_vel.abs() < 1e-6 {
            // Straight line motion
            let new_x = self.x + linear_vel * self.theta.cos() * dt;
            let new_y = self.y + linear_vel * self.theta.sin() * dt;
            (new_x, new_y, self.theta)
        } else {
            // Arc motion
            let r = linear_vel / angular_vel;
            let new_theta = self.theta + angular_vel * dt;
            let new_x = self.x + r * (new_theta.sin() - self.theta.sin());
            let new_y = self.y + r * (self.theta.cos() - new_theta.cos());
            (new_x, new_y, new_theta)
        };

        // Check collision at new position
        let would_collide = self.check_collision(new_x, new_y, map);

        match &self.collision_mode {
            CollisionMode::Stop => {
                if would_collide {
                    // Don't update position, only rotation
                    self.theta = normalize_angle(new_theta);
                    return true;
                }
                self.x = new_x;
                self.y = new_y;
                self.theta = normalize_angle(new_theta);
            }
            CollisionMode::Slide { friction } => {
                if would_collide {
                    // Try to slide along the obstacle
                    let (slide_x, slide_y) =
                        self.compute_slide(new_x, new_y, linear_vel, *friction, dt, map);
                    self.x = slide_x;
                    self.y = slide_y;
                    self.theta = normalize_angle(new_theta);
                    return true;
                }
                self.x = new_x;
                self.y = new_y;
                self.theta = normalize_angle(new_theta);
            }
            CollisionMode::Passthrough => {
                self.x = new_x;
                self.y = new_y;
                self.theta = normalize_angle(new_theta);
            }
        }

        false
    }

    /// Check if robot at given position would collide with obstacles
    fn check_collision(&self, x: f32, y: f32, map: &SimulationMap) -> bool {
        // Check multiple points around robot circumference
        let num_checks = 8;
        for i in 0..num_checks {
            let angle = (i as f32 / num_checks as f32) * TAU;
            let check_x = x + self.robot_radius * angle.cos();
            let check_y = y + self.robot_radius * angle.sin();
            if map.is_occupied(check_x, check_y) {
                return true;
            }
        }
        // Also check center
        map.is_occupied(x, y)
    }

    /// Compute slide motion when colliding
    fn compute_slide(
        &self,
        _target_x: f32,
        _target_y: f32,
        linear_vel: f32,
        friction: f32,
        dt: f32,
        map: &SimulationMap,
    ) -> (f32, f32) {
        // Simplified slide: try perpendicular directions
        let vel_mag = linear_vel.abs() * (1.0 - friction) * dt;

        // Try sliding in +/- 90 degrees from heading
        let perp_angles = [self.theta + PI / 2.0, self.theta - PI / 2.0];

        for perp_angle in perp_angles {
            let slide_x = self.x + vel_mag * perp_angle.cos();
            let slide_y = self.y + vel_mag * perp_angle.sin();

            if !self.check_collision(slide_x, slide_y, map) {
                return (slide_x, slide_y);
            }
        }

        // Can't slide, stay in place
        (self.x, self.y)
    }

    /// Calculate individual wheel velocities from (v, ω) commands
    ///
    /// Returns (left_vel, right_vel) in m/s
    pub fn wheel_velocities(
        &self,
        linear_vel: f32,
        angular_vel: f32,
        wheel_base: f32,
    ) -> (f32, f32) {
        // Differential drive kinematics:
        // v = (v_r + v_l) / 2
        // ω = (v_r - v_l) / wheel_base
        //
        // Solving for v_l and v_r:
        // v_l = v - ω * wheel_base / 2
        // v_r = v + ω * wheel_base / 2
        let half_base = wheel_base / 2.0;
        let left_vel = linear_vel - angular_vel * half_base;
        let right_vel = linear_vel + angular_vel * half_base;
        (left_vel, right_vel)
    }
}

/// Normalize angle to [-π, π)
fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle % TAU;
    if a >= PI {
        a -= TAU;
    } else if a < -PI {
        a += TAU;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> RobotConfig {
        RobotConfig::default()
    }

    #[test]
    fn test_wheel_velocities() {
        let config = default_config();
        let physics = PhysicsState::new(0.0, 0.0, 0.0, &config);

        // Pure forward motion
        let (left, right) = physics.wheel_velocities(0.3, 0.0, config.wheel_base);
        assert!((left - 0.3).abs() < 1e-6);
        assert!((right - 0.3).abs() < 1e-6);

        // Pure rotation
        let (left, right) = physics.wheel_velocities(0.0, 1.0, config.wheel_base);
        let half_base = config.wheel_base / 2.0;
        assert!((left + half_base).abs() < 1e-6); // Negative
        assert!((right - half_base).abs() < 1e-6); // Positive
    }

    #[test]
    fn test_normalize_angle() {
        // Basic cases
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-6);

        // PI is at the boundary, could normalize to PI or -PI
        // Both are equivalent, so just check the absolute value is near PI
        assert!((normalize_angle(PI).abs() - PI).abs() < 1e-6);
        assert!((normalize_angle(-PI).abs() - PI).abs() < 1e-6);

        // Full rotation should be near 0
        assert!(normalize_angle(TAU).abs() < 1e-6);

        // Arbitrary angles
        assert!((normalize_angle(PI / 4.0) - PI / 4.0).abs() < 1e-6);
        assert!((normalize_angle(-PI / 4.0) + PI / 4.0).abs() < 1e-6);
        assert!((normalize_angle(5.0 * PI / 4.0) + 3.0 * PI / 4.0).abs() < 1e-6);
    }
}
