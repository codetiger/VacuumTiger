//! Sensor source trait for robot abstraction.

use crate::core::{LidarScan, Pose2D};

/// Trait for providing sensor data and accepting commands.
///
/// Implement this trait to connect VastuExplorer to your robot hardware
/// or simulation environment.
///
/// # Example
///
/// ```ignore
/// struct MyRobot {
///     // Hardware connections
/// }
///
/// impl SensorSource for MyRobot {
///     fn get_pose(&self) -> Pose2D {
///         // Read from encoders/odometry
///         Pose2D::new(self.x, self.y, self.theta)
///     }
///
///     fn get_lidar_scan(&self) -> Option<LidarScan> {
///         // Read from lidar sensor
///         Some(self.latest_scan.clone())
///     }
///
///     fn send_velocity(&mut self, linear: f32, angular: f32) {
///         // Send to motor controller
///         self.motor.set_velocity(linear, angular);
///     }
///
///     fn is_connected(&self) -> bool {
///         self.connection.is_alive()
///     }
/// }
/// ```
pub trait SensorSource {
    /// Get current robot pose from odometry/encoders.
    ///
    /// This is the initial pose estimate before scan matching.
    /// The pose should be in the world frame (x, y in meters, theta in radians).
    fn get_pose(&self) -> Pose2D;

    /// Get the latest lidar scan.
    ///
    /// Returns None if no new scan is available since the last call.
    /// The scan should contain ranges in meters and angles in radians.
    fn get_lidar_scan(&self) -> Option<LidarScan>;

    /// Send velocity command to the robot.
    ///
    /// # Arguments
    /// * `linear` - Forward velocity in m/s (positive = forward)
    /// * `angular` - Angular velocity in rad/s (positive = counter-clockwise)
    fn send_velocity(&mut self, linear: f32, angular: f32);

    /// Check if the robot is still connected and operational.
    ///
    /// Returns false if the connection is lost or the robot is in an error state.
    /// The explorer will stop if this returns false.
    fn is_connected(&self) -> bool;

    /// Update sensor readings (optional polling).
    ///
    /// Called at the beginning of each exploration step.
    /// Use this to poll sensors if they don't push data automatically.
    /// Default implementation does nothing.
    fn poll(&mut self) {}

    /// Get cliff sensor state (optional).
    ///
    /// Returns true if any cliff sensor is triggered.
    /// Default returns false (no cliff sensors).
    fn is_cliff_detected(&self) -> bool {
        false
    }

    /// Get bumper state (optional).
    ///
    /// Returns true if any bumper is triggered.
    /// Default returns false (no bumpers).
    fn is_bumper_triggered(&self) -> bool {
        false
    }

    /// Stop the robot immediately (emergency or normal stop).
    ///
    /// Default implementation sends zero velocity.
    fn stop(&mut self) {
        self.send_velocity(0.0, 0.0);
    }

    /// Get current battery level as percentage (0-100).
    ///
    /// Returns None if battery monitoring is not available.
    /// Default returns None.
    fn battery_percent(&self) -> Option<u8> {
        None
    }
}
