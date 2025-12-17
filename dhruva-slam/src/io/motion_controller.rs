//! Motion controller for sending velocity commands to SangamIO.
//!
//! This module provides a high-level interface for controlling robot movement
//! during autonomous exploration and navigation.
//!
//! ## Communication Architecture
//!
//! With the UDP/TCP split in SangamIO:
//! - **UDP**: Sensor data (handled by SangamUdpReceiver in SLAM thread)
//! - **TCP**: Commands only (handled by this controller)
//!
//! ## Shared Access
//!
//! SangamIO only allows ONE TCP client connection at a time. To allow multiple
//! threads (navigation, exploration) to send commands, use `SharedMotionController`:
//!
//!
//! ```ignore
//! let shared = create_shared_motion_controller(config);
//! // Pass to navigation thread
//! // Pass to exploration thread
//! // They share the same TCP connection
//! ```
//!
//! ## Connection Management
//!
//! The controller automatically handles TCP reconnection when the connection
//! to SangamIO is lost. On connection errors (broken pipe, connection reset),
//! it will attempt to reconnect before retrying the failed operation.
//!
//! Note: Some config fields and utility methods are defined for future use.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::io::sangam_client::{ComponentActionType, SangamClient, SensorValue};

/// Thread-safe shared motion controller.
///
/// Use this when multiple threads (navigation, exploration) need to send
/// commands to SangamIO. SangamIO only allows one TCP client, so all threads
/// must share a single connection.
pub type SharedMotionController = Arc<Mutex<MotionController>>;

/// Create a shared motion controller for use by multiple threads.
pub fn create_shared_motion_controller(config: MotionConfig) -> SharedMotionController {
    Arc::new(Mutex::new(MotionController::new(config)))
}

/// Configuration for motion controller.
#[derive(Debug, Clone)]
pub struct MotionConfig {
    /// Maximum linear velocity (m/s).
    pub max_linear_vel: f32,
    /// Maximum angular velocity (rad/s).
    pub max_angular_vel: f32,
    /// Minimum safe distance from obstacles (m).
    pub obstacle_clearance: f32,
    /// Linear velocity for normal movement (m/s).
    pub linear_vel: f32,
    /// Angular velocity for in-place rotation (rad/s).
    pub angular_vel: f32,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            max_linear_vel: 0.3,
            max_angular_vel: 0.5,
            obstacle_clearance: 0.15,
            linear_vel: 0.2,
            angular_vel: 0.3,
        }
    }
}

/// Minimum time between reconnection attempts.
const RECONNECT_COOLDOWN: Duration = Duration::from_secs(2);

/// Motion controller for robot movement via SangamIO.
///
/// Provides velocity commands and motor control for autonomous exploration.
/// Automatically handles TCP reconnection on connection failures.
pub struct MotionController {
    /// SangamIO client for sending commands.
    client: Option<SangamClient>,
    /// Motion configuration.
    config: MotionConfig,
    /// Whether motors are enabled for navigation.
    is_enabled: bool,
    /// Whether lidar is enabled for scanning.
    is_lidar_enabled: bool,
    /// SangamIO address for reconnection.
    address: String,
    /// Last reconnection attempt time (to avoid reconnect spam).
    last_reconnect_attempt: Option<Instant>,
    /// Number of consecutive connection failures.
    consecutive_failures: u32,
}

impl MotionController {
    /// Create a new motion controller with the given configuration.
    pub fn new(config: MotionConfig) -> Self {
        Self {
            client: None,
            config,
            is_enabled: false,
            is_lidar_enabled: false,
            address: String::new(),
            last_reconnect_attempt: None,
            consecutive_failures: 0,
        }
    }

    /// Connect to SangamIO for sending commands.
    ///
    /// # Arguments
    /// * `address` - SangamIO address (e.g., "192.168.68.101:5555")
    pub fn connect(&mut self, address: &str) -> Result<(), String> {
        self.address = address.to_string();
        self.connect_internal()
    }

    /// Internal connection logic (used for initial connect and reconnect).
    fn connect_internal(&mut self) -> Result<(), String> {
        if self.address.is_empty() {
            return Err("No address configured".to_string());
        }

        // Drop existing client if any
        self.client = None;

        match SangamClient::connect(&self.address) {
            Ok(mut client) => {
                // Set a reasonable timeout for commands
                if let Err(e) = client.set_timeout(Some(Duration::from_secs(5))) {
                    return Err(format!("Failed to set timeout: {}", e));
                }
                self.client = Some(client);
                self.consecutive_failures = 0;
                log::info!("Motion controller connected to {}", self.address);
                Ok(())
            }
            Err(e) => {
                self.consecutive_failures += 1;
                Err(format!("Failed to connect to SangamIO: {}", e))
            }
        }
    }

    /// Attempt to reconnect if enough time has passed since last attempt.
    ///
    /// Returns true if reconnection was successful, false otherwise.
    fn try_reconnect(&mut self) -> bool {
        // Check cooldown to avoid reconnect spam
        if let Some(last_attempt) = self.last_reconnect_attempt
            && last_attempt.elapsed() < RECONNECT_COOLDOWN
        {
            return false;
        }

        self.last_reconnect_attempt = Some(Instant::now());
        log::info!(
            "Attempting to reconnect to SangamIO at {} (attempt #{})",
            self.address,
            self.consecutive_failures + 1
        );

        match self.connect_internal() {
            Ok(()) => {
                log::info!("Reconnected to SangamIO successfully");
                // Re-enable motors/lidar if they were enabled before disconnect
                if self.is_enabled {
                    self.is_enabled = false; // Reset flag so enable() will actually send command
                    if let Err(e) = self.enable() {
                        log::warn!("Failed to re-enable motors after reconnect: {}", e);
                    }
                }
                if self.is_lidar_enabled {
                    self.is_lidar_enabled = false;
                    if let Err(e) = self.enable_lidar() {
                        log::warn!("Failed to re-enable lidar after reconnect: {}", e);
                    }
                }
                true
            }
            Err(e) => {
                log::warn!("Reconnection failed: {}", e);
                false
            }
        }
    }

    /// Handle a connection error by attempting reconnection.
    ///
    /// Returns true if the operation should be retried.
    fn handle_connection_error(&mut self, error_msg: &str) -> bool {
        // Check if this looks like a connection error
        let is_connection_error = error_msg.contains("Broken pipe")
            || error_msg.contains("Connection reset")
            || error_msg.contains("Connection refused")
            || error_msg.contains("not connected")
            || error_msg.contains("os error 32")  // EPIPE
            || error_msg.contains("os error 104"); // ECONNRESET

        if is_connection_error {
            // Mark client as disconnected
            self.client = None;
            // Try to reconnect
            self.try_reconnect()
        } else {
            false
        }
    }

    /// Check if connected to SangamIO.
    ///
    /// Note: This only checks if a client exists. The connection may still
    /// be broken; actual connection health is verified on send operations.
    pub fn is_connected(&self) -> bool {
        self.client.is_some()
    }

    /// Check if we have an address configured (can attempt connection).
    pub fn has_address(&self) -> bool {
        !self.address.is_empty()
    }

    /// Enable motor navigation mode.
    ///
    /// Must be called before sending velocity commands.
    /// Automatically attempts reconnection if connection is lost.
    pub fn enable(&mut self) -> Result<(), String> {
        // Try to connect if we have an address but no client
        if self.client.is_none() && self.has_address() {
            self.try_reconnect();
        }

        let client = self.client.as_mut().ok_or("Not connected")?;

        match client.send_component_command("drive", ComponentActionType::Enable) {
            Ok(()) => {
                self.is_enabled = true;
                self.consecutive_failures = 0;
                log::info!("Motion controller enabled");
                Ok(())
            }
            Err(e) => {
                let err_msg = format!("Failed to enable drive: {}", e);
                if self.handle_connection_error(&err_msg) {
                    // Retry after reconnection
                    let client = self
                        .client
                        .as_mut()
                        .ok_or("Not connected after reconnect")?;
                    client
                        .send_component_command("drive", ComponentActionType::Enable)
                        .map_err(|e| format!("Failed to enable drive after reconnect: {}", e))?;
                    self.is_enabled = true;
                    log::info!("Motion controller enabled (after reconnect)");
                    Ok(())
                } else {
                    Err(err_msg)
                }
            }
        }
    }

    /// Enable lidar scanning.
    ///
    /// Must be called to start receiving lidar data from SangamIO.
    /// Automatically attempts reconnection if connection is lost.
    pub fn enable_lidar(&mut self) -> Result<(), String> {
        // Try to connect if we have an address but no client
        if self.client.is_none() && self.has_address() {
            self.try_reconnect();
        }

        let client = self.client.as_mut().ok_or("Not connected")?;

        match client.send_component_command("lidar", ComponentActionType::Enable) {
            Ok(()) => {
                self.is_lidar_enabled = true;
                self.consecutive_failures = 0;
                log::info!("Lidar enabled");
                Ok(())
            }
            Err(e) => {
                let err_msg = format!("Failed to enable lidar: {}", e);
                if self.handle_connection_error(&err_msg) {
                    // Retry after reconnection
                    let client = self
                        .client
                        .as_mut()
                        .ok_or("Not connected after reconnect")?;
                    client
                        .send_component_command("lidar", ComponentActionType::Enable)
                        .map_err(|e| format!("Failed to enable lidar after reconnect: {}", e))?;
                    self.is_lidar_enabled = true;
                    log::info!("Lidar enabled (after reconnect)");
                    Ok(())
                } else {
                    Err(err_msg)
                }
            }
        }
    }

    /// Disable lidar scanning.
    /// Silently succeeds if not connected (best-effort disable).
    pub fn disable_lidar(&mut self) -> Result<(), String> {
        if let Some(ref mut client) = self.client
            && let Err(e) = client.send_component_command("lidar", ComponentActionType::Disable)
        {
            let err_msg = format!("Failed to disable lidar: {}", e);
            // Don't reconnect for disable - just mark as disabled
            self.handle_connection_error(&err_msg);
            // Still mark as disabled locally
        }

        self.is_lidar_enabled = false;
        log::info!("Lidar disabled");
        Ok(())
    }

    /// Check if lidar is enabled.
    pub fn is_lidar_enabled(&self) -> bool {
        self.is_lidar_enabled
    }

    /// Disable motors and return to idle.
    /// Silently succeeds if not connected (best-effort disable).
    pub fn disable(&mut self) -> Result<(), String> {
        // First stop any motion
        self.emergency_stop().ok();

        if let Some(ref mut client) = self.client
            && let Err(e) = client.send_component_command("drive", ComponentActionType::Disable)
        {
            let err_msg = format!("Failed to disable drive: {}", e);
            // Don't reconnect for disable - just mark as disabled
            self.handle_connection_error(&err_msg);
            // Still mark as disabled locally
        }

        self.is_enabled = false;
        log::info!("Motion controller disabled");
        Ok(())
    }

    /// Set velocity (linear m/s, angular rad/s).
    ///
    /// Values are clamped to configured maximums.
    /// Automatically attempts reconnection if connection is lost.
    pub fn set_velocity(&mut self, linear: f32, angular: f32) -> Result<(), String> {
        if !self.is_enabled {
            return Err("Motors not enabled".to_string());
        }

        // Try to reconnect if disconnected
        if self.client.is_none() && self.has_address() && self.try_reconnect() {
            // Re-enable motors after reconnect
            self.is_enabled = false;
            if let Err(e) = self.enable() {
                return Err(format!("Failed to re-enable motors: {}", e));
            }
        }

        let client = self.client.as_mut().ok_or("Not connected")?;

        // Clamp to configured limits
        let linear = linear.clamp(-self.config.max_linear_vel, self.config.max_linear_vel);
        let angular = angular.clamp(-self.config.max_angular_vel, self.config.max_angular_vel);

        // Build velocity command
        let mut cmd_config = HashMap::new();
        cmd_config.insert("linear".to_string(), SensorValue::F32(linear));
        cmd_config.insert("angular".to_string(), SensorValue::F32(angular));

        match client.send_component_command_with_config(
            "drive",
            ComponentActionType::Configure,
            cmd_config.clone(),
        ) {
            Ok(()) => {
                self.consecutive_failures = 0;
                Ok(())
            }
            Err(e) => {
                let err_msg = format!("Failed to set velocity: {}", e);
                if self.handle_connection_error(&err_msg) {
                    // Re-enable motors after reconnect
                    self.is_enabled = false;
                    self.enable()?;

                    // Retry the velocity command
                    let client = self
                        .client
                        .as_mut()
                        .ok_or("Not connected after reconnect")?;
                    client
                        .send_component_command_with_config(
                            "drive",
                            ComponentActionType::Configure,
                            cmd_config,
                        )
                        .map_err(|e| format!("Failed to set velocity after reconnect: {}", e))?;
                    Ok(())
                } else {
                    Err(err_msg)
                }
            }
        }
    }

    /// Emergency stop - immediately halt all motion.
    /// Critical operation: tries to reconnect if needed since stopping is essential for safety.
    pub fn emergency_stop(&mut self) -> Result<(), String> {
        // Try to reconnect if not connected
        if self.client.is_none() && self.has_address() {
            self.try_reconnect();
        }

        let client = match self.client.as_mut() {
            Some(c) => c,
            None => {
                // No connection available - can't stop but not an error for emergency
                log::debug!("Emergency stop: no connection to SangamIO");
                return Ok(());
            }
        };

        let mut cmd_config = HashMap::new();
        cmd_config.insert("linear".to_string(), SensorValue::F32(0.0));
        cmd_config.insert("angular".to_string(), SensorValue::F32(0.0));

        match client.send_component_command_with_config(
            "drive",
            ComponentActionType::Configure,
            cmd_config.clone(),
        ) {
            Ok(()) => Ok(()),
            Err(e) => {
                let err_msg = format!("Failed to stop: {}", e);

                // Try to reconnect and retry if this looks like a connection error
                if self.handle_connection_error(&err_msg) {
                    // Retry the stop command after reconnection
                    let client = self
                        .client
                        .as_mut()
                        .ok_or("Not connected after reconnect")?;
                    client
                        .send_component_command_with_config(
                            "drive",
                            ComponentActionType::Configure,
                            cmd_config,
                        )
                        .map_err(|e| format!("Failed to stop after reconnect: {}", e))?;
                    log::info!("Emergency stop sent after reconnection");
                    Ok(())
                } else {
                    Err(err_msg)
                }
            }
        }
    }

    /// Move forward at configured linear velocity.
    pub fn move_forward(&mut self) -> Result<(), String> {
        self.set_velocity(self.config.linear_vel, 0.0)
    }

    /// Move backward at configured linear velocity.
    pub fn move_backward(&mut self) -> Result<(), String> {
        self.set_velocity(-self.config.linear_vel, 0.0)
    }

    /// Rotate left (CCW) at configured angular velocity.
    pub fn rotate_left(&mut self) -> Result<(), String> {
        self.set_velocity(0.0, self.config.angular_vel)
    }

    /// Rotate right (CW) at configured angular velocity.
    pub fn rotate_right(&mut self) -> Result<(), String> {
        self.set_velocity(0.0, -self.config.angular_vel)
    }

    /// Get the motion configuration.
    pub fn config(&self) -> &MotionConfig {
        &self.config
    }

    /// Check if motors are enabled.
    pub fn is_enabled(&self) -> bool {
        self.is_enabled
    }
}

impl Drop for MotionController {
    fn drop(&mut self) {
        // Ensure motors are stopped when controller is dropped
        self.disable().ok();
    }
}

/// Normalize angle to [-PI, PI] range.
pub fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle;
    while a > std::f32::consts::PI {
        a -= 2.0 * std::f32::consts::PI;
    }
    while a < -std::f32::consts::PI {
        a += 2.0 * std::f32::consts::PI;
    }
    a
}

/// Compute velocity commands to move toward a target pose.
///
/// Uses proportional control with rotation-first strategy:
/// - If angle error > threshold: rotate in place toward target
/// - Otherwise: move forward with proportional angular correction
///
/// Returns (linear_velocity, angular_velocity) in m/s and rad/s.
pub fn compute_velocity_to_target(
    current_x: f32,
    current_y: f32,
    current_theta: f32,
    target_x: f32,
    target_y: f32,
    config: &MotionConfig,
) -> (f32, f32) {
    let dx = target_x - current_x;
    let dy = target_y - current_y;
    let distance = (dx * dx + dy * dy).sqrt();

    // Target reached threshold
    if distance < 0.05 {
        return (0.0, 0.0);
    }

    let target_angle = dy.atan2(dx);
    let angle_error = normalize_angle(target_angle - current_theta);

    // Rotation-first strategy: if angle error is large, rotate in place
    let angle_threshold = 0.3; // ~17 degrees
    if angle_error.abs() > angle_threshold {
        let angular = angle_error.signum() * config.angular_vel;
        return (0.0, angular);
    }

    // Move forward with proportional angular correction
    let linear = (distance * 0.5).min(config.linear_vel);
    let angular = (angle_error * 0.5).clamp(-config.angular_vel, config.angular_vel);

    (linear, angular)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_angle() {
        use std::f32::consts::PI;

        assert!((normalize_angle(0.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(PI) - PI).abs() < 0.001);
        assert!((normalize_angle(-PI) - (-PI)).abs() < 0.001);
        assert!((normalize_angle(2.0 * PI) - 0.0).abs() < 0.001);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 0.001);
        assert!((normalize_angle(-3.0 * PI) - (-PI)).abs() < 0.001);
    }

    #[test]
    fn test_velocity_to_target_straight_ahead() {
        let config = MotionConfig::default();
        let (linear, angular) = compute_velocity_to_target(0.0, 0.0, 0.0, 1.0, 0.0, &config);

        // Should move forward with minimal angular correction
        assert!(linear > 0.0);
        assert!(angular.abs() < 0.1);
    }

    #[test]
    fn test_velocity_to_target_needs_rotation() {
        let config = MotionConfig::default();
        let (linear, angular) = compute_velocity_to_target(0.0, 0.0, 0.0, 0.0, 1.0, &config); // Target to the left

        // Should rotate in place (target is 90 degrees left)
        assert!(linear.abs() < 0.01); // Nearly zero forward velocity
        assert!(angular > 0.0); // Positive = CCW = left
    }

    #[test]
    fn test_velocity_to_target_reached() {
        let config = MotionConfig::default();
        let (linear, angular) = compute_velocity_to_target(1.0, 1.0, 0.0, 1.02, 1.02, &config);

        // Target reached - should stop
        assert!(linear.abs() < 0.01);
        assert!(angular.abs() < 0.01);
    }
}
