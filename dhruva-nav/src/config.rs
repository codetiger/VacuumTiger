//! Configuration loading for DhruvaNav

use crate::error::{DhruvaError, Result};
use serde::Deserialize;
use std::path::Path;

/// Main configuration structure
#[derive(Clone, Debug, Deserialize)]
pub struct DhruvaConfig {
    pub connection: ConnectionConfig,
    pub robot: RobotConfig,
    #[serde(default)]
    pub exploration: ExplorationConfig,
    pub output: OutputConfig,
}

/// Network connection settings
#[derive(Clone, Debug, Deserialize)]
pub struct ConnectionConfig {
    /// Robot IP address (default: 127.0.0.1 for local mock)
    #[serde(default = "default_robot_ip")]
    pub robot_ip: String,

    /// TCP port number (default: 5555)
    #[serde(default = "default_port")]
    pub port: u16,

    /// Connection timeout in milliseconds (default: 5000)
    #[serde(default = "default_timeout")]
    pub timeout_ms: u64,
}

/// Robot physical parameters
#[derive(Clone, Debug, Deserialize)]
pub struct RobotConfig {
    /// Distance between wheels in meters (default: 0.233)
    #[serde(default = "default_wheel_base")]
    pub wheel_base: f32,

    /// Encoder ticks per meter of travel (default: 4464.0)
    #[serde(default = "default_ticks_per_meter")]
    pub ticks_per_meter: f32,

    /// Maximum linear velocity in m/s (default: 0.2)
    #[serde(default = "default_max_linear")]
    pub max_linear_vel: f32,

    /// Maximum angular velocity in rad/s (default: 0.5)
    #[serde(default = "default_max_angular")]
    pub max_angular_vel: f32,
}

/// Output configuration
#[derive(Clone, Debug, Deserialize)]
pub struct OutputConfig {
    /// Path to save the map file
    #[serde(default = "default_map_path")]
    pub map_path: String,

    /// Path to save SVG visualization
    #[serde(default = "default_svg_path")]
    pub svg_path: String,
}

/// Exploration configuration
#[derive(Clone, Debug, Deserialize)]
pub struct ExplorationConfig {
    /// Robot radius for obstacle inflation (meters)
    #[serde(default = "default_robot_radius")]
    pub robot_radius: f32,

    /// Safety margin beyond robot radius (meters)
    #[serde(default = "default_safety_margin")]
    pub safety_margin: f32,

    /// Wall penalty distance for path planning (meters)
    #[serde(default = "default_wall_penalty_distance")]
    pub wall_penalty_distance: f32,

    /// Minimum frontier cluster size
    #[serde(default = "default_min_frontier_size")]
    pub min_frontier_size: usize,

    /// Frontier refresh interval (seconds)
    #[serde(default = "default_frontier_refresh")]
    pub frontier_refresh_interval: f32,

    /// Return to start position when complete
    #[serde(default = "default_return_to_start")]
    pub return_to_start: bool,

    /// Goal tolerance for waypoints (meters)
    #[serde(default = "default_goal_tolerance")]
    pub goal_tolerance: f32,

    // --- Recovery and stuck detection parameters ---
    /// Maximum replans allowed for a single frontier before blacklisting
    #[serde(default = "default_max_replans_per_frontier")]
    pub max_replans_per_frontier: usize,

    /// Timeout for making progress toward frontier (seconds)
    #[serde(default = "default_frontier_timeout_secs")]
    pub frontier_timeout_secs: f32,

    /// Minimum distance to consider progress made (meters)
    #[serde(default = "default_min_progress_distance")]
    pub min_progress_distance: f32,

    /// Cooldown before blacklisted frontier can be retried (seconds)
    #[serde(default = "default_blacklist_cooldown_secs")]
    pub blacklist_cooldown_secs: f32,

    /// Distance threshold to consider robot stuck (meters)
    #[serde(default = "default_stuck_distance_threshold")]
    pub stuck_distance_threshold: f32,

    /// Time window for stuck detection (seconds)
    #[serde(default = "default_stuck_time_window_secs")]
    pub stuck_time_window_secs: f32,

    /// Distance to back up during recovery (meters)
    #[serde(default = "default_recovery_backup_distance")]
    pub recovery_backup_distance: f32,

    /// Angle to rotate during recovery (radians)
    #[serde(default = "default_recovery_rotation_rad")]
    pub recovery_rotation_rad: f32,

    /// Maximum recovery attempts per frontier before blacklisting
    #[serde(default = "default_max_recovery_attempts")]
    pub max_recovery_attempts: usize,
}

impl Default for ExplorationConfig {
    fn default() -> Self {
        Self {
            robot_radius: default_robot_radius(),
            safety_margin: default_safety_margin(),
            wall_penalty_distance: default_wall_penalty_distance(),
            min_frontier_size: default_min_frontier_size(),
            frontier_refresh_interval: default_frontier_refresh(),
            return_to_start: default_return_to_start(),
            goal_tolerance: default_goal_tolerance(),
            // Recovery and stuck detection
            max_replans_per_frontier: default_max_replans_per_frontier(),
            frontier_timeout_secs: default_frontier_timeout_secs(),
            min_progress_distance: default_min_progress_distance(),
            blacklist_cooldown_secs: default_blacklist_cooldown_secs(),
            stuck_distance_threshold: default_stuck_distance_threshold(),
            stuck_time_window_secs: default_stuck_time_window_secs(),
            recovery_backup_distance: default_recovery_backup_distance(),
            recovery_rotation_rad: default_recovery_rotation_rad(),
            max_recovery_attempts: default_max_recovery_attempts(),
        }
    }
}

// Default value functions
fn default_robot_ip() -> String {
    "127.0.0.1".to_string()
}
fn default_port() -> u16 {
    5555
}
fn default_timeout() -> u64 {
    5000
}
fn default_wheel_base() -> f32 {
    0.233
}
fn default_ticks_per_meter() -> f32 {
    4464.0
}
fn default_max_linear() -> f32 {
    0.2
}
fn default_max_angular() -> f32 {
    0.5
}
fn default_map_path() -> String {
    "output/map.vastu".to_string()
}
fn default_svg_path() -> String {
    "output/map.svg".to_string()
}

// Exploration defaults
fn default_robot_radius() -> f32 {
    0.10
}
fn default_safety_margin() -> f32 {
    0.15
} // 25cm total clearance from robot center
fn default_wall_penalty_distance() -> f32 {
    0.30
} // Penalize paths within 30cm of walls
fn default_min_frontier_size() -> usize {
    5
}
fn default_frontier_refresh() -> f32 {
    1.5
}
fn default_return_to_start() -> bool {
    true
}
fn default_goal_tolerance() -> f32 {
    0.15
}

// Recovery and stuck detection defaults
fn default_max_replans_per_frontier() -> usize {
    3
}
fn default_frontier_timeout_secs() -> f32 {
    60.0
}
fn default_min_progress_distance() -> f32 {
    0.10
}
fn default_blacklist_cooldown_secs() -> f32 {
    120.0
}
fn default_stuck_distance_threshold() -> f32 {
    0.02
}
fn default_stuck_time_window_secs() -> f32 {
    3.0
}
fn default_recovery_backup_distance() -> f32 {
    0.10
}
fn default_recovery_rotation_rad() -> f32 {
    0.5
}
fn default_max_recovery_attempts() -> usize {
    2
}

impl Default for DhruvaConfig {
    fn default() -> Self {
        Self {
            connection: ConnectionConfig {
                robot_ip: default_robot_ip(),
                port: default_port(),
                timeout_ms: default_timeout(),
            },
            robot: RobotConfig {
                wheel_base: default_wheel_base(),
                ticks_per_meter: default_ticks_per_meter(),
                max_linear_vel: default_max_linear(),
                max_angular_vel: default_max_angular(),
            },
            exploration: ExplorationConfig::default(),
            output: OutputConfig {
                map_path: default_map_path(),
                svg_path: default_svg_path(),
            },
        }
    }
}

impl DhruvaConfig {
    /// Load configuration from a TOML file
    pub fn load(path: &Path) -> Result<Self> {
        let content = std::fs::read_to_string(path)
            .map_err(|e| DhruvaError::Config(format!("Failed to read config file: {}", e)))?;
        let config: DhruvaConfig = toml::from_str(&content)?;
        Ok(config)
    }

    /// Get the full address string for connection
    pub fn address(&self) -> String {
        format!("{}:{}", self.connection.robot_ip, self.connection.port)
    }
}
