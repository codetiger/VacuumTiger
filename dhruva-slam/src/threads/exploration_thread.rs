//! Exploration thread for autonomous mapping.
//!
//! This thread coordinates the exploration process:
//! 1. Reads current map and pose from SharedState
//! 2. Uses exploration strategy to determine next action
//! 3. Sends velocity commands via MotionController
//! 4. Handles bumper events for obstacle avoidance

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::exploration::{
    ExplorationAction, ExplorationStrategy, FrontierConfig, FrontierExploration, HazardEvent,
    HazardType,
};
use crate::io::motion_controller::{
    MotionConfig, SharedMotionController, compute_velocity_to_target,
};
use crate::state::{CliffState, ExplorationMode, SharedStateHandle};

/// State for initial scanning behavior when no map exists yet.
#[derive(Debug, Clone, Copy, PartialEq)]
enum InitialScanState {
    /// Not yet started initial scan.
    NotStarted,
    /// Currently rotating to build initial map.
    Rotating { start_time: Instant },
    /// Initial scan complete, have map data.
    Complete,
}

/// Configuration for exploration thread.
#[derive(Debug, Clone)]
pub struct ExplorationConfig {
    /// Motion controller configuration (for velocity limits).
    pub motion: MotionConfig,
    /// Frontier exploration configuration.
    pub frontier: FrontierConfig,
    /// Loop rate in Hz.
    pub loop_rate_hz: f32,
}

impl Default for ExplorationConfig {
    fn default() -> Self {
        Self {
            motion: MotionConfig::default(),
            frontier: FrontierConfig::default(),
            loop_rate_hz: 10.0,
        }
    }
}

/// Handle to the exploration thread.
pub struct ExplorationThread {
    handle: JoinHandle<()>,
}

impl ExplorationThread {
    /// Spawn the exploration thread.
    ///
    /// # Arguments
    ///
    /// - `config`: Thread configuration.
    /// - `shared_state`: Handle to shared state.
    /// - `motion_controller`: Shared motion controller (shared with navigation thread).
    /// - `running`: Atomic flag for shutdown.
    pub fn spawn(
        config: ExplorationConfig,
        shared_state: SharedStateHandle,
        motion_controller: SharedMotionController,
        running: Arc<AtomicBool>,
    ) -> Self {
        let handle = thread::Builder::new()
            .name("exploration".into())
            .spawn(move || {
                run_exploration_loop(config, shared_state, motion_controller, running);
            })
            .expect("Failed to spawn exploration thread");

        Self { handle }
    }

    /// Wait for the thread to finish.
    pub fn join(self) {
        if let Err(e) = self.handle.join() {
            log::error!("Exploration thread panicked: {:?}", e);
        }
    }
}

/// Main exploration loop.
fn run_exploration_loop(
    config: ExplorationConfig,
    shared_state: SharedStateHandle,
    motion_controller: SharedMotionController,
    running: Arc<AtomicBool>,
) {
    log::info!("Exploration thread started");

    // Check if motion controller has an address configured
    let motion_available = {
        let mc = motion_controller.lock().unwrap();
        mc.has_address()
    };

    // Initialize exploration strategy
    let mut strategy: Box<dyn ExplorationStrategy> =
        Box::new(FrontierExploration::new(config.frontier.clone()));

    let loop_duration = Duration::from_secs_f32(1.0 / config.loop_rate_hz);
    let mut last_bumper_state = (false, false);
    let mut last_cliff_state = CliffState::default();
    let mut motion_enabled = false;
    let mut lidar_enabled = false;
    let mut initial_scan_state = InitialScanState::NotStarted;

    // Duration to rotate in place to build initial map (full 360° rotation)
    let initial_scan_duration =
        Duration::from_secs_f32((2.0 * std::f32::consts::PI) / config.motion.angular_vel);

    while running.load(Ordering::Relaxed) {
        let loop_start = Instant::now();

        // Read current state
        let (exploration_mode, current_pose, current_map, bumper_state, cliff_state) = {
            let state = match shared_state.read() {
                Ok(s) => s,
                Err(e) => {
                    log::error!("Failed to read shared state: {}", e);
                    thread::sleep(loop_duration);
                    continue;
                }
            };

            (
                state.exploration_state.mode,
                state.robot_status.pose,
                state.current_map.clone(),
                state.sensor_status.bumper.clone(),
                state.sensor_status.cliff.clone(),
            )
        };

        // Detect hazard events (rising edge detection for bumpers and cliff sensors)
        let hazard_event: Option<HazardEvent> = {
            // Check bumpers first (higher priority - physical collision)
            let bumper_left_triggered = bumper_state.left_pressed && !last_bumper_state.0;
            let bumper_right_triggered = bumper_state.right_pressed && !last_bumper_state.1;

            // Check cliff sensors (also high priority - fall hazard)
            let cliff_left_side_triggered = cliff_state.left_side && !last_cliff_state.left_side;
            let cliff_left_front_triggered = cliff_state.left_front && !last_cliff_state.left_front;
            let cliff_right_front_triggered =
                cliff_state.right_front && !last_cliff_state.right_front;
            let cliff_right_side_triggered = cliff_state.right_side && !last_cliff_state.right_side;

            // Determine hazard type (prioritize bumpers, then front cliffs, then side cliffs)
            if bumper_left_triggered && bumper_right_triggered {
                Some(HazardEvent::new(HazardType::BumperBoth, &current_pose))
            } else if bumper_left_triggered {
                Some(HazardEvent::new(HazardType::BumperLeft, &current_pose))
            } else if bumper_right_triggered {
                Some(HazardEvent::new(HazardType::BumperRight, &current_pose))
            } else if cliff_left_front_triggered {
                Some(HazardEvent::new(HazardType::CliffLeftFront, &current_pose))
            } else if cliff_right_front_triggered {
                Some(HazardEvent::new(HazardType::CliffRightFront, &current_pose))
            } else if cliff_left_side_triggered {
                Some(HazardEvent::new(HazardType::CliffLeftSide, &current_pose))
            } else if cliff_right_side_triggered {
                Some(HazardEvent::new(HazardType::CliffRightSide, &current_pose))
            } else {
                None
            }
        };

        // Update last states for edge detection
        last_bumper_state = (bumper_state.left_pressed, bumper_state.right_pressed);
        last_cliff_state = cliff_state.clone();

        // Handle hazard emergency stop (bumper or cliff)
        let any_hazard_active =
            bumper_state.left_pressed || bumper_state.right_pressed || cliff_state.any_triggered();

        if any_hazard_active {
            if let Err(e) = motion_controller.lock().unwrap().emergency_stop() {
                log::warn!("Emergency stop failed: {}", e);
            }

            // Update shared state with hazard detection
            if let Ok(mut state) = shared_state.write() {
                state.exploration_state.bumper_obstacle_detected = true;
            }

            // Don't wait - let the strategy handle the hazard event
            // but do a brief pause to let motors stop
            thread::sleep(Duration::from_millis(50));
        } else {
            // Clear hazard flag when all sensors are clear
            if let Ok(mut state) = shared_state.write()
                && state.exploration_state.bumper_obstacle_detected
            {
                state.exploration_state.bumper_obstacle_detected = false;
            }
        }

        // Check if navigation has active targets - yield to navigation
        let nav_has_targets = {
            match shared_state.read() {
                Ok(s) => s.navigation_state.has_targets(),
                Err(_) => false,
            }
        };

        // Only explore if in exploration mode AND navigation is not active
        if exploration_mode != ExplorationMode::Exploring || nav_has_targets {
            // Disable motors and lidar if we were exploring
            if motion_enabled || lidar_enabled {
                let mut mc = motion_controller.lock().unwrap();
                if motion_enabled {
                    if let Err(e) = mc.disable() {
                        log::warn!("Failed to disable motors: {}", e);
                    }
                    motion_enabled = false;
                }
                if lidar_enabled {
                    if let Err(e) = mc.disable_lidar() {
                        log::warn!("Failed to disable lidar: {}", e);
                    }
                    lidar_enabled = false;
                }
            }

            // Check if we need to reset strategy (mode changed)
            if exploration_mode == ExplorationMode::Disabled {
                strategy.reset();
                // Reset initial scan state for next exploration session
                initial_scan_state = InitialScanState::NotStarted;
            }

            // If yielding to navigation, update status
            if nav_has_targets
                && exploration_mode == ExplorationMode::Exploring
                && let Ok(mut state) = shared_state.write()
            {
                state.exploration_state.status = "Paused: Navigation active".to_string();
            }

            thread::sleep(loop_duration);
            continue;
        }

        // Enable lidar first (needed for SLAM to build map)
        if !lidar_enabled && motion_available {
            let result = motion_controller.lock().unwrap().enable_lidar();
            match result {
                Ok(()) => {
                    lidar_enabled = true;
                    log::info!(
                        "Lidar enabled for mapping, waiting 3 seconds for controller to settle..."
                    );
                    // Wait 3 seconds for lidar motor controller to settle
                    thread::sleep(Duration::from_secs(3));
                    log::info!("Lidar settled, continuing");
                }
                Err(e) => {
                    // Only log at debug level - MotionController will retry automatically
                    log::debug!("Lidar enable pending: {}", e);
                }
            }
        }

        // Enable motors if not already enabled
        if !motion_enabled && motion_available {
            let result = motion_controller.lock().unwrap().enable();
            match result {
                Ok(()) => {
                    motion_enabled = true;
                    log::info!("Motors enabled, waiting 3 seconds for controller to settle...");
                    // Wait 3 seconds for motor controller to settle
                    thread::sleep(Duration::from_secs(3));
                    log::info!("Motors settled, ready for exploration");
                }
                Err(e) => {
                    // Only log at debug level - MotionController will retry automatically
                    log::debug!("Motor enable pending: {}", e);
                }
            }
        }

        // Get map data for exploration
        let map = match current_map {
            Some(m) => {
                // We have map data, mark initial scan as complete
                if initial_scan_state != InitialScanState::Complete {
                    log::info!("Initial scan complete, map data available");
                    initial_scan_state = InitialScanState::Complete;
                }
                m
            }
            None => {
                // No map yet - need to rotate in place to build initial map
                match initial_scan_state {
                    InitialScanState::NotStarted => {
                        log::info!("No map data yet, starting initial 360° scan rotation");
                        initial_scan_state = InitialScanState::Rotating {
                            start_time: Instant::now(),
                        };

                        // Update status
                        if let Ok(mut state) = shared_state.write() {
                            state.exploration_state.status =
                                "Initial scan: rotating to build map".to_string();
                        }

                        // Start rotating
                        if let Err(e) = motion_controller
                            .lock()
                            .unwrap()
                            .set_velocity(0.0, config.motion.angular_vel)
                        {
                            log::warn!("Failed to start initial rotation: {}", e);
                        }
                    }
                    InitialScanState::Rotating { start_time } => {
                        // Check if we've completed the full rotation
                        if start_time.elapsed() >= initial_scan_duration {
                            log::info!("Initial rotation complete, waiting for map data");
                            // Stop rotating
                            if let Err(e) = motion_controller.lock().unwrap().emergency_stop() {
                                log::warn!("Failed to stop after initial rotation: {}", e);
                            }

                            // Update status - still waiting for map
                            if let Ok(mut state) = shared_state.write() {
                                state.exploration_state.status =
                                    "Waiting for map data after initial scan".to_string();
                            }

                            // Give a brief pause then continue checking
                            // Don't reset state - keep waiting for map
                        } else {
                            // Still rotating - update status with progress
                            let progress = start_time.elapsed().as_secs_f32()
                                / initial_scan_duration.as_secs_f32()
                                * 100.0;
                            if let Ok(mut state) = shared_state.write() {
                                state.exploration_state.status =
                                    format!("Initial scan: {:.0}% complete", progress);
                            }
                        }
                    }
                    InitialScanState::Complete => {
                        // Should have map but don't - continue waiting
                        if let Ok(mut state) = shared_state.write() {
                            state.exploration_state.status = "Waiting for map data".to_string();
                        }
                    }
                }
                thread::sleep(loop_duration);
                continue;
            }
        };

        // Get next action from strategy
        let action = strategy.next_action(&map, &current_pose, hazard_event.as_ref());

        // Update shared state with exploration progress
        if let Ok(mut state) = shared_state.write() {
            state.exploration_state.frontiers_remaining = strategy.frontier_count();
            state.exploration_state.completion_percent = strategy.completion_percent();
            state.exploration_state.explored_area_m2 = map.explored_area_m2;
        }

        // Execute action
        match action {
            ExplorationAction::MoveTo { target, reason } => {
                log::debug!("Moving to ({:.2}, {:.2}) - {}", target.x, target.y, reason);

                // Compute velocity to reach target
                let (linear, angular) = compute_velocity_to_target(
                    current_pose.x,
                    current_pose.y,
                    current_pose.theta,
                    target.x,
                    target.y,
                    &config.motion,
                );

                if let Err(e) = motion_controller
                    .lock()
                    .unwrap()
                    .set_velocity(linear, angular)
                {
                    log::warn!("Failed to set velocity: {}", e);
                }

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.exploration_state.current_target = Some(target);
                    state.exploration_state.status = format!("Exploring: {}", reason);
                }
            }

            ExplorationAction::RotateInPlace { angle_rad } => {
                log::debug!("Rotating in place: {:.2} rad", angle_rad);

                let angular = angle_rad.signum() * config.motion.angular_vel;
                if let Err(e) = motion_controller.lock().unwrap().set_velocity(0.0, angular) {
                    log::warn!("Failed to set rotation velocity: {}", e);
                }

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.exploration_state.status = "Scanning area".to_string();
                }
            }

            ExplorationAction::Complete => {
                log::info!("Exploration complete - all areas mapped!");

                if let Err(e) = motion_controller.lock().unwrap().disable() {
                    log::warn!("Failed to disable motors: {}", e);
                }
                motion_enabled = false;

                // Update exploration mode to complete
                if let Ok(mut state) = shared_state.write() {
                    state.exploration_state.mode = ExplorationMode::Complete;
                    state.exploration_state.status = "Exploration complete".to_string();
                    state.exploration_state.current_target = None;
                    state.exploration_state.completion_percent = 100.0;
                }
            }

            ExplorationAction::Stuck { reason } => {
                log::warn!("Exploration stuck: {}", reason);

                {
                    let mut mc = motion_controller.lock().unwrap();
                    if let Err(e) = mc.emergency_stop() {
                        log::warn!("Failed to stop: {}", e);
                    }
                }

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.exploration_state.status = format!("Stuck: {}", reason);
                }

                // Try rotating to find new path
                if let Err(e) = motion_controller
                    .lock()
                    .unwrap()
                    .set_velocity(0.0, config.motion.angular_vel)
                {
                    log::warn!("Failed to rotate: {}", e);
                }
            }
        }

        // Maintain loop rate
        let elapsed = loop_start.elapsed();
        if elapsed < loop_duration {
            thread::sleep(loop_duration - elapsed);
        }
    }

    // Cleanup: disable motors and lidar
    if motion_available {
        let mut mc = motion_controller.lock().unwrap();
        if let Err(e) = mc.disable() {
            log::warn!("Failed to disable motors on shutdown: {}", e);
        }
        if let Err(e) = mc.disable_lidar() {
            log::warn!("Failed to disable lidar on shutdown: {}", e);
        }
    }

    log::info!("Exploration thread stopped");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_exploration_config_default() {
        let config = ExplorationConfig::default();
        assert_eq!(config.loop_rate_hz, 10.0);
    }
}
