//! Thread management for multi-threaded SLAM daemon.
//!
//! This module provides four main threads:
//! - `SlamThread`: Real-time sensor processing (blocks on SangamIO)
//! - `PublisherThread`: Data stream publishing and command handling (port 5557)
//! - `ExplorationThread`: Autonomous mapping (optional)
//! - `NavigationThread`: Goal-based path following (10Hz)

mod exploration_thread;
mod navigation_thread;
mod publisher_thread;
mod slam_thread;

pub use exploration_thread::{ExplorationConfig, ExplorationThread};
pub use navigation_thread::{
    NavCommand, NavCommandSender, NavigationThread, NavigationThreadConfig, create_nav_channel,
};
pub use publisher_thread::PublisherThread;
pub use slam_thread::{SlamThread, SlamThreadConfig};

/// Configuration for stream publishing rates.
#[derive(Debug, Clone)]
pub struct StreamConfig {
    /// Robot status publish rate (Hz).
    pub robot_status_hz: f32,
    /// Sensor status publish rate (Hz).
    pub sensor_status_hz: f32,
    /// Map publish rate (Hz).
    pub map_hz: f32,
    /// Navigation status publish rate (Hz).
    pub navigation_hz: f32,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            robot_status_hz: 10.0,
            sensor_status_hz: 10.0,
            map_hz: 1.0,
            navigation_hz: 5.0,
        }
    }
}
