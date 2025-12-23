//! Configuration for exploration behavior.

use crate::query::frontier::FrontierConfig;

use super::planner::PlannerConfig;

/// Configuration for exploration behavior.
#[derive(Clone, Debug)]
pub struct ExplorationConfig {
    /// Length of virtual walls added at collision points (meters).
    /// Default: 0.3
    pub virtual_wall_length: f32,

    /// Distance to back off after collision (meters).
    /// Default: 0.15
    pub backoff_distance: f32,

    /// Distance threshold to consider waypoint reached (meters).
    /// Default: 0.1
    pub waypoint_tolerance: f32,

    /// Maximum linear velocity (m/s).
    /// Default: 0.3
    pub max_linear_speed: f32,

    /// Maximum angular velocity (rad/s).
    /// Default: 1.0
    pub max_angular_speed: f32,

    /// Robot radius for path planning clearance (meters).
    /// Default: 0.15
    pub robot_radius: f32,

    /// Lookahead distance for pure pursuit path following (meters).
    /// Default: 0.3
    pub lookahead_distance: f32,

    /// Frontier detection configuration.
    /// Note: With shadow-based frontiers, viewpoints are already safe observation points.
    pub frontier_config: FrontierConfig,

    /// Region-based exploration planner configuration.
    pub planner_config: PlannerConfig,

    /// Cell size for visit history tracking (meters).
    /// Default: 0.5
    pub history_cell_size: f32,

    /// Maximum number of recent positions to track in history.
    /// Default: 100
    pub history_max_recent: usize,
}

impl Default for ExplorationConfig {
    fn default() -> Self {
        Self {
            virtual_wall_length: 0.3,
            backoff_distance: 0.15,
            waypoint_tolerance: 0.1,
            max_linear_speed: 0.3,
            max_angular_speed: 1.0,
            robot_radius: 0.15,
            lookahead_distance: 0.3,
            frontier_config: FrontierConfig::default(),
            planner_config: PlannerConfig::default(),
            history_cell_size: 0.5,
            history_max_recent: 100,
        }
    }
}

impl ExplorationConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for virtual wall length.
    pub fn with_virtual_wall_length(mut self, length: f32) -> Self {
        self.virtual_wall_length = length;
        self
    }

    /// Builder-style setter for backoff distance.
    pub fn with_backoff_distance(mut self, distance: f32) -> Self {
        self.backoff_distance = distance;
        self
    }

    /// Builder-style setter for waypoint tolerance.
    pub fn with_waypoint_tolerance(mut self, tolerance: f32) -> Self {
        self.waypoint_tolerance = tolerance;
        self
    }

    /// Builder-style setter for max linear speed.
    pub fn with_max_linear_speed(mut self, speed: f32) -> Self {
        self.max_linear_speed = speed;
        self
    }

    /// Builder-style setter for max angular speed.
    pub fn with_max_angular_speed(mut self, speed: f32) -> Self {
        self.max_angular_speed = speed;
        self
    }

    /// Builder-style setter for robot radius.
    pub fn with_robot_radius(mut self, radius: f32) -> Self {
        self.robot_radius = radius;
        self
    }

    /// Builder-style setter for lookahead distance.
    pub fn with_lookahead_distance(mut self, distance: f32) -> Self {
        self.lookahead_distance = distance;
        self
    }

    /// Builder-style setter for frontier config.
    pub fn with_frontier_config(mut self, config: FrontierConfig) -> Self {
        self.frontier_config = config;
        self
    }

    /// Builder-style setter for planner config.
    pub fn with_planner_config(mut self, config: PlannerConfig) -> Self {
        self.planner_config = config;
        self
    }

    /// Builder-style setter for history cell size.
    pub fn with_history_cell_size(mut self, size: f32) -> Self {
        self.history_cell_size = size;
        self
    }

    /// Builder-style setter for history max recent.
    pub fn with_history_max_recent(mut self, max: usize) -> Self {
        self.history_max_recent = max;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = ExplorationConfig::default();
        assert_eq!(config.virtual_wall_length, 0.3);
        assert_eq!(config.backoff_distance, 0.15);
        assert_eq!(config.waypoint_tolerance, 0.1);
        assert_eq!(config.max_linear_speed, 0.3);
        assert_eq!(config.max_angular_speed, 1.0);
        assert_eq!(config.robot_radius, 0.15);
    }

    #[test]
    fn test_builder_pattern() {
        let config = ExplorationConfig::new()
            .with_max_linear_speed(0.5)
            .with_robot_radius(0.2);

        assert_eq!(config.max_linear_speed, 0.5);
        assert_eq!(config.robot_radius, 0.2);
    }
}
