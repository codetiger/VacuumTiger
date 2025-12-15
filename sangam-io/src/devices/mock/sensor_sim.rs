//! Bumper, cliff, and actuator simulation
//!
//! Provides collision-based bumper detection and map-based cliff detection.

use super::config::{BumpersConfig, CliffsConfig};
use super::map_loader::SimulationMap;
use crate::core::types::ComponentAction;
use std::collections::HashMap;

/// Bumper simulator using angular zones
pub struct BumperSimulator {
    config: BumpersConfig,
}

impl BumperSimulator {
    /// Create new bumper simulator
    pub fn new(config: &BumpersConfig) -> Self {
        Self {
            config: config.clone(),
        }
    }

    /// Check bumper states based on map collisions
    ///
    /// # Arguments
    /// * `map` - Simulation map for collision checking
    /// * `x` - Robot X position (meters)
    /// * `y` - Robot Y position (meters)
    /// * `theta` - Robot orientation (radians)
    /// * `robot_radius` - Robot collision radius (meters)
    ///
    /// # Returns
    /// (left_triggered, right_triggered)
    pub fn check(
        &self,
        map: &SimulationMap,
        x: f32,
        y: f32,
        theta: f32,
        robot_radius: f32,
    ) -> (bool, bool) {
        let check_radius = robot_radius + self.config.trigger_distance;
        let num_samples = 16; // Samples per bumper zone

        let mut left_triggered = false;
        let mut right_triggered = false;

        // Check left bumper zone
        let left_range = self.config.left.end_angle - self.config.left.start_angle;
        for i in 0..num_samples {
            let local_angle =
                self.config.left.start_angle + (i as f32 / num_samples as f32) * left_range;
            let world_angle = theta + local_angle;
            let check_x = x + check_radius * world_angle.cos();
            let check_y = y + check_radius * world_angle.sin();

            if map.is_occupied(check_x, check_y) {
                left_triggered = true;
                break;
            }
        }

        // Check right bumper zone
        let right_range = self.config.right.end_angle - self.config.right.start_angle;
        for i in 0..num_samples {
            let local_angle =
                self.config.right.start_angle + (i as f32 / num_samples as f32) * right_range;
            let world_angle = theta + local_angle;
            let check_x = x + check_radius * world_angle.cos();
            let check_y = y + check_radius * world_angle.sin();

            if map.is_occupied(check_x, check_y) {
                right_triggered = true;
                break;
            }
        }

        (left_triggered, right_triggered)
    }
}

/// Cliff sensor simulator
pub struct CliffSimulator {
    config: CliffsConfig,
}

impl CliffSimulator {
    /// Create new cliff simulator
    pub fn new(config: &CliffsConfig) -> Self {
        Self {
            config: config.clone(),
        }
    }

    /// Check cliff states based on map
    ///
    /// # Arguments
    /// * `map` - Simulation map with cliff mask
    /// * `x` - Robot X position (meters)
    /// * `y` - Robot Y position (meters)
    /// * `theta` - Robot orientation (radians)
    ///
    /// # Returns
    /// HashMap with sensor names and their triggered states
    pub fn check(&self, map: &SimulationMap, x: f32, y: f32, theta: f32) -> HashMap<String, bool> {
        let mut states = HashMap::new();

        if !self.config.enabled {
            // Return all sensors as not triggered
            for sensor in &self.config.sensors {
                states.insert(sensor.name.clone(), false);
            }
            return states;
        }

        let cos_theta = theta.cos();
        let sin_theta = theta.sin();

        for sensor in &self.config.sensors {
            // Transform sensor position to world coordinates
            let world_x = x + sensor.x * cos_theta - sensor.y * sin_theta;
            let world_y = y + sensor.x * sin_theta + sensor.y * cos_theta;

            // Check if cliff at sensor position
            let triggered = map.is_cliff(world_x, world_y);
            states.insert(sensor.name.clone(), triggered);
        }

        states
    }
}

/// Actuator state tracker (for logging)
pub struct ActuatorState {
    /// Current states by actuator ID
    states: HashMap<String, String>,
}

impl ActuatorState {
    /// Create new actuator state tracker
    pub fn new() -> Self {
        Self {
            states: HashMap::new(),
        }
    }

    /// Handle an actuator command
    pub fn handle_command(&mut self, id: &str, action: &ComponentAction, log_changes: bool) {
        let state_str = match action {
            ComponentAction::Enable { config } => {
                if let Some(cfg) = config {
                    format!("enabled({:?})", cfg)
                } else {
                    "enabled".to_string()
                }
            }
            ComponentAction::Disable { .. } => "disabled".to_string(),
            ComponentAction::Reset { .. } => "reset".to_string(),
            ComponentAction::Configure { config } => {
                format!("configured({:?})", config)
            }
        };

        let old_state = self.states.get(id).cloned();
        self.states.insert(id.to_string(), state_str.clone());

        if log_changes {
            if let Some(old) = old_state {
                if old != state_str {
                    log::info!("Actuator '{}': {} -> {}", id, old, state_str);
                }
            } else {
                log::info!("Actuator '{}': -> {}", id, state_str);
            }
        }
    }
}

impl Default for ActuatorState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bumper_no_collision() {
        let config = BumpersConfig::default();
        let bumper = BumperSimulator::new(&config);

        // Create a simple free space map (all white)
        let img = image::GrayImage::from_fn(100, 100, |_, _| image::Luma([255u8]));
        let map = SimulationMap::from_test_image(img, 0.1, (0.0, 0.0));

        let (left, right) = bumper.check(&map, 5.0, 5.0, 0.0, 0.17);

        assert!(!left);
        assert!(!right);
    }

    #[test]
    fn test_cliff_simulator() {
        let config = CliffsConfig::default();
        let cliff = CliffSimulator::new(&config);

        // Create map with no cliff mask
        let img = image::GrayImage::from_fn(100, 100, |_, _| image::Luma([255u8]));
        let map = SimulationMap::from_test_image(img, 0.1, (0.0, 0.0));

        let states = cliff.check(&map, 5.0, 5.0, 0.0);

        // All sensors should report no cliff
        assert_eq!(states.len(), 4);
        assert!(!states["left_side"]);
        assert!(!states["left_front"]);
        assert!(!states["right_front"]);
        assert!(!states["right_side"]);
    }

    #[test]
    fn test_actuator_state() {
        let mut state = ActuatorState::new();

        state.handle_command(
            "main_brush",
            &ComponentAction::Enable { config: None },
            false,
        );

        assert_eq!(state.states.get("main_brush"), Some(&"enabled".to_string()));

        state.handle_command(
            "main_brush",
            &ComponentAction::Disable { config: None },
            false,
        );

        assert_eq!(
            state.states.get("main_brush"),
            Some(&"disabled".to_string())
        );
    }
}
