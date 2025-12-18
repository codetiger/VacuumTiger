//! Navigation Thread - Goal-based path following at 10Hz.
//!
//! This thread:
//! - Runs at 10Hz (100ms interval)
//! - Reads pose and map from SharedState
//! - Uses Navigator to compute velocity commands
//! - Sends velocity commands via shared MotionController
//! - Updates NavigationState in SharedState
//! - Manages MappingFeature for autonomous map building
//!
//! Commands (SetGoal, CancelGoal, StartMapping, StopMapping) are received via a channel.
//!
//! NOTE: Uses SharedMotionController to share TCP connection with ExplorationThread
//! since SangamIO only allows one TCP client.
//!
//! Note: Some methods are defined for planned features.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::core::types::LaserScan;
use crate::io::motion_controller::SharedMotionController;
use crate::navigation::features::{MappingConfig, MappingFeature, MappingState};
use crate::navigation::{NavState, NavTarget, NavTargetSource, Navigator, NavigatorConfig};
use crate::state::{ExplorationMode, SharedStateHandle};

/// Commands that can be sent to the navigation thread.
#[derive(Debug)]
pub enum NavCommand {
    /// Set a navigation goal (user click on map).
    SetGoal {
        /// Target X position (meters).
        x: f32,
        /// Target Y position (meters).
        y: f32,
        /// Optional target heading (radians).
        theta: Option<f32>,
        /// Optional description for UI.
        description: String,
    },

    /// Cancel active navigation.
    CancelGoal,

    /// Start autonomous mapping (used by tests).
    StartMapping,

    /// Stop autonomous mapping.
    StopMapping,
}

/// Sender type for navigation commands.
pub type NavCommandSender = mpsc::Sender<NavCommand>;

/// Receiver type for navigation commands.
pub type NavCommandReceiver = mpsc::Receiver<NavCommand>;

/// Create a new navigation command channel.
pub fn create_nav_channel() -> (NavCommandSender, NavCommandReceiver) {
    mpsc::channel()
}

/// Configuration for the navigation thread.
#[derive(Debug, Clone)]
pub struct NavigationThreadConfig {
    /// Navigator configuration.
    pub navigator: NavigatorConfig,

    /// Mapping feature configuration.
    pub mapping: MappingConfig,

    /// Update rate in Hz.
    pub update_rate_hz: f32,
}

impl Default for NavigationThreadConfig {
    fn default() -> Self {
        Self {
            navigator: NavigatorConfig::default(),
            mapping: MappingConfig::default(),
            update_rate_hz: 10.0,
        }
    }
}

/// Navigation thread handle.
pub struct NavigationThread {
    handle: JoinHandle<()>,
}

impl NavigationThread {
    /// Spawn the navigation thread.
    ///
    /// # Arguments
    ///
    /// - `config`: Thread configuration.
    /// - `shared_state`: Handle to shared state.
    /// - `motion_controller`: Shared motion controller (shared with exploration thread).
    /// - `nav_rx`: Receiver for navigation commands.
    /// - `running`: Atomic flag for shutdown.
    pub fn spawn(
        config: NavigationThreadConfig,
        shared_state: SharedStateHandle,
        motion_controller: SharedMotionController,
        nav_rx: NavCommandReceiver,
        running: Arc<AtomicBool>,
    ) -> Self {
        let handle = thread::Builder::new()
            .name("navigation".into())
            .spawn(move || {
                run_navigation_loop(config, shared_state, motion_controller, nav_rx, running);
            })
            .expect("Failed to spawn navigation thread");

        Self { handle }
    }

    /// Wait for thread to finish.
    pub fn join(self) -> thread::Result<()> {
        self.handle.join()
    }
}

/// Main navigation loop.
fn run_navigation_loop(
    config: NavigationThreadConfig,
    shared_state: SharedStateHandle,
    motion_controller: SharedMotionController,
    nav_rx: NavCommandReceiver,
    running: Arc<AtomicBool>,
) {
    log::info!("Navigation thread starting");

    // Create navigator
    let mut navigator = Navigator::new(config.navigator.clone());

    // Create mapping feature
    let mut mapping_feature = MappingFeature::with_config(config.mapping.clone());

    // Check if motion controller has an address configured
    let motion_available = {
        let mc = motion_controller.lock().unwrap();
        mc.has_address()
    };

    // Track enabled state - we enable on first target, disable when idle
    let mut lidar_enabled = false;
    let mut motors_enabled = false;
    let mut was_active = false;

    // Track previous target count for completion detection (reserved for future use)
    let mut _prev_target_count: usize = 0;
    let mut _prev_nav_state = NavState::Idle;

    // Time since navigation became inactive (for delayed disable)
    let mut inactive_since: Option<Instant> = None;
    const IDLE_DISABLE_DELAY: Duration = Duration::from_secs(10);

    // Calculate loop timing
    let loop_interval = Duration::from_secs_f32(1.0 / config.update_rate_hz);

    log::info!(
        "Navigation thread running at {}Hz ({}ms interval)",
        config.update_rate_hz,
        loop_interval.as_millis()
    );

    while running.load(Ordering::Relaxed) {
        let loop_start = Instant::now();

        // Process any pending commands (non-blocking)
        while let Ok(cmd) = nav_rx.try_recv() {
            process_nav_command(&cmd, &shared_state, &mut navigator, &mut mapping_feature);
        }

        // Read current state
        let (pose, map, hazard, nav_active, is_charging, lidar_scan, current_target, slam_mapping) = {
            match shared_state.read() {
                Ok(state) => {
                    let pose = state.robot_status.pose;
                    let map = state.current_map.clone();
                    let hazard = state.sensor_status.bumper.left_pressed
                        || state.sensor_status.bumper.right_pressed
                        || state.sensor_status.cliff.any_triggered();
                    let nav_active = state.navigation_state.is_active();
                    let is_charging = state.robot_status.is_charging;
                    // Check if SLAM is in mapping mode (set by StartMapping command)
                    let slam_mapping =
                        state.robot_status.state == crate::state::RobotState::Mapping;

                    // Create LaserScan from sensor status if available
                    let lidar_scan = if !state.sensor_status.lidar_ranges.is_empty() {
                        let n = state.sensor_status.lidar_ranges.len();
                        let angle_increment = std::f32::consts::TAU / n as f32;
                        Some(LaserScan::new(
                            0.0,
                            std::f32::consts::TAU - angle_increment,
                            angle_increment,
                            0.1,
                            10.0,
                            state.sensor_status.lidar_ranges.clone(),
                        ))
                    } else {
                        None
                    };

                    let current_target = state.navigation_state.current_target().cloned();
                    (
                        pose,
                        map,
                        hazard,
                        nav_active,
                        is_charging,
                        lidar_scan,
                        current_target,
                        slam_mapping,
                    )
                }
                Err(e) => {
                    log::warn!("Failed to read shared state: {}", e);
                    thread::sleep(loop_interval);
                    continue;
                }
            }
        };

        // Check if mapping feature is active
        let mapping_active = mapping_feature.is_active();

        // Handle motor/lidar enabling when navigation, mapping, or SLAM mapping becomes active
        // slam_mapping is true when SLAM thread received StartMapping command (even without MappingFeature)
        if (nav_active || mapping_active || slam_mapping) && motion_available {
            // Clear inactive timer
            inactive_since = None;

            // Enable lidar if not already (needed for SLAM map updates)
            if !lidar_enabled {
                let result = motion_controller.lock().unwrap().enable_lidar();
                match result {
                    Ok(()) => {
                        lidar_enabled = true;
                        log::debug!("Navigation: Lidar enabled");
                        // Brief pause for lidar to start
                        thread::sleep(Duration::from_millis(500));

                        // Notify mapping feature that hardware is ready
                        if mapping_active && let Ok(mut state) = shared_state.write() {
                            mapping_feature.on_hardware_ready(&mut state.navigation_state);
                        }
                    }
                    Err(e) => {
                        // Only log at debug level - MotionController will retry automatically
                        log::debug!("Navigation: Lidar enable pending: {}", e);
                    }
                }
            }

            // Enable motors if not already
            if !motors_enabled {
                let result = motion_controller.lock().unwrap().enable();
                match result {
                    Ok(()) => {
                        motors_enabled = true;
                        log::debug!("Navigation: Motors enabled");
                        // Brief pause for motors to initialize
                        thread::sleep(Duration::from_millis(500));
                    }
                    Err(e) => {
                        // Only log at debug level - MotionController will retry automatically
                        log::debug!("Navigation: Motor enable pending: {}", e);
                    }
                }
            }
        }

        // Handle disabling when navigation becomes inactive
        // IMPORTANT: slam_mapping keeps motors enabled during SLAM mapping even without active navigation
        if !nav_active && !mapping_active && !slam_mapping && was_active {
            // Start the inactive timer
            if inactive_since.is_none() {
                inactive_since = Some(Instant::now());
                log::debug!(
                    "Navigation became inactive, starting disable timer (nav={}, mapping={}, slam={})",
                    nav_active,
                    mapping_active,
                    slam_mapping
                );
            }
        } else if slam_mapping && inactive_since.is_some() {
            // Clear inactive timer if slam_mapping is active
            log::debug!("Clearing inactive timer - SLAM mapping is active");
            inactive_since = None;
        }

        // Disable motors/lidar after being inactive for a while
        if !nav_active
            && !mapping_active
            && !slam_mapping
            && motion_available
            && let Some(since) = inactive_since
            && since.elapsed() >= IDLE_DISABLE_DELAY
        {
            if motors_enabled {
                if let Err(e) = motion_controller.lock().unwrap().disable() {
                    log::warn!("Navigation: Failed to disable motors: {}", e);
                } else {
                    motors_enabled = false;
                    log::info!(
                        "Navigation: Motors disabled after idle timeout (nav={}, mapping={}, slam={})",
                        nav_active,
                        mapping_active,
                        slam_mapping
                    );
                }
            }
            // Note: Keep lidar enabled - SLAM needs it for localization
            inactive_since = None;
        }

        // Include slam_mapping in was_active to properly track activity during SLAM mapping
        was_active = nav_active || mapping_active || slam_mapping;

        // Process mapping feature state machine
        if mapping_active {
            // Feed lidar scan for clearance analysis when in appropriate states
            if let Some(ref scan) = lidar_scan {
                match mapping_feature.state() {
                    MappingState::AnalyzingClearance | MappingState::EscapingConfined { .. } => {
                        if let Ok(mut state) = shared_state.write() {
                            mapping_feature.on_lidar_scan(
                                &mut state.navigation_state,
                                scan,
                                &pose,
                                is_charging,
                            );
                        }
                    }
                    _ => {}
                }
            }

            // Periodic update
            if let Ok(mut state) = shared_state.write() {
                mapping_feature.update(&mut state.navigation_state, &pose);

                // Get explored area from current map
                let area_explored_m2 = state
                    .current_map
                    .as_ref()
                    .map(|m| m.explored_area_m2)
                    .unwrap_or(0.0);

                // Update mapping progress for UI streaming
                state.mapping_progress = mapping_feature.get_progress(area_explored_m2);

                // Sync mapping state to exploration state
                state.exploration_state.mode = match mapping_feature.state() {
                    MappingState::Idle => ExplorationMode::Disabled,
                    MappingState::Complete => ExplorationMode::Complete,
                    MappingState::Failed { .. } => ExplorationMode::Disabled,
                    _ => ExplorationMode::Exploring,
                };
                state.exploration_state.status = state.navigation_state.status_message.clone();
            }
        }

        // Only run navigation if there's an active target
        if nav_active {
            // Get current target count and nav state before update
            let (target_count_before, nav_state_before) = {
                match shared_state.read() {
                    Ok(state) => (
                        state.navigation_state.target_count(),
                        state.navigation_state.nav_state,
                    ),
                    Err(_) => (0, NavState::Idle),
                }
            };

            // Get mutable access to navigation state for update
            let nav_update = {
                match shared_state.write() {
                    Ok(mut state) => {
                        navigator.update(&mut state.navigation_state, &pose, map.as_ref(), hazard)
                    }
                    Err(e) => {
                        log::warn!("Failed to write shared state: {}", e);
                        thread::sleep(loop_interval);
                        continue;
                    }
                }
            };

            // Check for target completion (target count decreased)
            let (target_count_after, nav_state_after) = {
                match shared_state.read() {
                    Ok(state) => (
                        state.navigation_state.target_count(),
                        state.navigation_state.nav_state,
                    ),
                    Err(_) => (target_count_before, nav_state_before),
                }
            };

            // Detect target completion
            if target_count_after < target_count_before && mapping_active {
                // A target was completed - notify mapping feature
                if let Some(ref completed_target) = current_target
                    && completed_target.source == NavTargetSource::Mapping
                    && let Ok(mut state) = shared_state.write()
                {
                    mapping_feature.on_target_reached(
                        &mut state.navigation_state,
                        map.as_ref(),
                        &pose,
                        completed_target,
                    );
                }
            }

            // Detect target failure
            if nav_state_after == NavState::Failed
                && nav_state_before != NavState::Failed
                && mapping_active
                && let Some(ref failed_target) = current_target
                && failed_target.source == NavTargetSource::Mapping
            {
                let failure_reason = {
                    shared_state
                        .read()
                        .ok()
                        .and_then(|s| {
                            s.navigation_state
                                .failure_reason
                                .as_ref()
                                .map(|r| r.description())
                        })
                        .unwrap_or_else(|| "Unknown".to_string())
                };

                if let Ok(mut state) = shared_state.write() {
                    mapping_feature.on_target_failed(
                        &mut state.navigation_state,
                        map.as_ref(),
                        &pose,
                        failed_target,
                        &failure_reason,
                    );
                }
            }

            _prev_target_count = target_count_after;
            _prev_nav_state = nav_state_after;

            // Send velocity command if motors are enabled
            if motors_enabled {
                if let Err(e) = motion_controller
                    .lock()
                    .unwrap()
                    .set_velocity(nav_update.linear_vel, nav_update.angular_vel)
                {
                    log::warn!("Failed to set velocity: {}", e);
                }
            } else if motion_available {
                log::debug!("Skipping velocity command - motors not enabled yet");
            }

            // Log state changes
            if nav_update.state_changed
                && let Ok(state) = shared_state.read()
            {
                log::info!(
                    "Navigation state: {:?}, targets: {}",
                    state.navigation_state.nav_state,
                    state.navigation_state.target_count()
                );
            }
        }

        // Sleep for remainder of loop interval
        let elapsed = loop_start.elapsed();
        if elapsed < loop_interval {
            thread::sleep(loop_interval - elapsed);
        }
    }

    // Cleanup: disable motors and lidar on shutdown
    if motion_available {
        let mut mc = motion_controller.lock().unwrap();
        if motors_enabled && let Err(e) = mc.disable() {
            log::warn!("Failed to disable motors on shutdown: {}", e);
        }
        if lidar_enabled && let Err(e) = mc.disable_lidar() {
            log::warn!("Failed to disable lidar on shutdown: {}", e);
        }
        if let Err(e) = mc.emergency_stop() {
            log::warn!("Failed to stop robot on shutdown: {}", e);
        }
    }

    log::info!("Navigation thread shutting down");
}

/// Process a navigation command.
fn process_nav_command(
    cmd: &NavCommand,
    shared_state: &SharedStateHandle,
    navigator: &mut Navigator,
    mapping_feature: &mut MappingFeature,
) {
    match cmd {
        NavCommand::SetGoal {
            x,
            y,
            theta,
            description,
        } => {
            log::info!(
                "SetGoal received: ({:.2}, {:.2}), theta={:?}, desc={}",
                x,
                y,
                theta,
                description
            );

            // Stop mapping if active (user navigation takes priority)
            if mapping_feature.is_active()
                && let Ok(mut state) = shared_state.write()
            {
                mapping_feature.stop(&mut state.navigation_state);
                log::info!("Mapping stopped for user navigation");
            }

            // Create target and push to stack
            let target = NavTarget::user_waypoint(*x, *y, *theta, description.clone());
            let target_id = target.id;

            match shared_state.write() {
                Ok(mut state) => {
                    state.navigation_state.push_target(target);
                    state
                        .navigation_state
                        .set_status(format!("Planning to {}", description));
                    log::info!("Target {} added to navigation stack", target_id);
                }
                Err(e) => {
                    log::error!("Failed to add target: {}", e);
                }
            }

            // Reset navigator state for new goal
            navigator.reset();
        }

        NavCommand::CancelGoal => {
            log::info!("CancelGoal received");

            // Stop mapping if active
            if mapping_feature.is_active()
                && let Ok(mut state) = shared_state.write()
            {
                mapping_feature.stop(&mut state.navigation_state);
                log::info!("Mapping stopped");
            }

            match shared_state.write() {
                Ok(mut state) => {
                    state.navigation_state.clear_targets();
                    state.navigation_state.set_status("Navigation cancelled");
                    log::info!("Navigation cancelled");
                }
                Err(e) => {
                    log::error!("Failed to cancel navigation: {}", e);
                }
            }

            navigator.reset();
        }

        NavCommand::StartMapping => {
            log::info!("StartMapping received");

            // Get current pose
            let pose = match shared_state.read() {
                Ok(state) => state.robot_status.pose,
                Err(e) => {
                    log::error!("Failed to read pose for mapping: {}", e);
                    return;
                }
            };

            // Start mapping feature
            match shared_state.write() {
                Ok(mut state) => {
                    mapping_feature.start(&mut state.navigation_state, &pose);
                    state.exploration_state.mode = ExplorationMode::Exploring;
                    state.exploration_state.status = "Mapping: Initializing".to_string();
                    log::info!("Mapping started");
                }
                Err(e) => {
                    log::error!("Failed to start mapping: {}", e);
                }
            }

            navigator.reset();
        }

        NavCommand::StopMapping => {
            log::info!("StopMapping received");

            match shared_state.write() {
                Ok(mut state) => {
                    mapping_feature.stop(&mut state.navigation_state);
                    state.exploration_state.mode = ExplorationMode::Disabled;
                    state.exploration_state.status = "Mapping stopped".to_string();
                    log::info!("Mapping stopped");
                }
                Err(e) => {
                    log::error!("Failed to stop mapping: {}", e);
                }
            }

            navigator.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::create_shared_state;

    #[test]
    fn test_nav_command_channel() {
        let (tx, rx) = create_nav_channel();

        tx.send(NavCommand::SetGoal {
            x: 1.0,
            y: 2.0,
            theta: Some(0.5),
            description: "Test".to_string(),
        })
        .unwrap();

        let cmd = rx.recv().unwrap();
        match cmd {
            NavCommand::SetGoal {
                x,
                y,
                theta,
                description,
            } => {
                assert_eq!(x, 1.0);
                assert_eq!(y, 2.0);
                assert_eq!(theta, Some(0.5));
                assert_eq!(description, "Test");
            }
            _ => panic!("Expected SetGoal"),
        }
    }

    #[test]
    fn test_process_set_goal() {
        let shared_state = create_shared_state();
        let mut navigator = Navigator::new(NavigatorConfig::default());
        let mut mapping_feature = MappingFeature::new();

        let cmd = NavCommand::SetGoal {
            x: 3.0,
            y: 4.0,
            theta: None,
            description: "Kitchen".to_string(),
        };

        process_nav_command(&cmd, &shared_state, &mut navigator, &mut mapping_feature);

        let state = shared_state.read().unwrap();
        assert!(state.navigation_state.has_targets());
        assert_eq!(state.navigation_state.target_count(), 1);
        assert_eq!(state.navigation_state.nav_state, NavState::Planning);
    }

    #[test]
    fn test_process_cancel_goal() {
        let shared_state = create_shared_state();
        let mut navigator = Navigator::new(NavigatorConfig::default());
        let mut mapping_feature = MappingFeature::new();

        // First add a goal
        let cmd = NavCommand::SetGoal {
            x: 1.0,
            y: 1.0,
            theta: None,
            description: "Test".to_string(),
        };
        process_nav_command(&cmd, &shared_state, &mut navigator, &mut mapping_feature);

        // Then cancel
        let cmd = NavCommand::CancelGoal;
        process_nav_command(&cmd, &shared_state, &mut navigator, &mut mapping_feature);

        let state = shared_state.read().unwrap();
        assert!(!state.navigation_state.has_targets());
        assert_eq!(state.navigation_state.nav_state, NavState::Cancelled);
    }

    #[test]
    fn test_process_start_mapping() {
        let shared_state = create_shared_state();
        let mut navigator = Navigator::new(NavigatorConfig::default());
        let mut mapping_feature = MappingFeature::new();

        let cmd = NavCommand::StartMapping;
        process_nav_command(&cmd, &shared_state, &mut navigator, &mut mapping_feature);

        assert!(mapping_feature.is_active());
        let state = shared_state.read().unwrap();
        assert_eq!(state.exploration_state.mode, ExplorationMode::Exploring);
    }

    #[test]
    fn test_process_stop_mapping() {
        let shared_state = create_shared_state();
        let mut navigator = Navigator::new(NavigatorConfig::default());
        let mut mapping_feature = MappingFeature::new();

        // Start mapping first
        let cmd = NavCommand::StartMapping;
        process_nav_command(&cmd, &shared_state, &mut navigator, &mut mapping_feature);

        // Then stop
        let cmd = NavCommand::StopMapping;
        process_nav_command(&cmd, &shared_state, &mut navigator, &mut mapping_feature);

        assert!(!mapping_feature.is_active());
        let state = shared_state.read().unwrap();
        assert_eq!(state.exploration_state.mode, ExplorationMode::Disabled);
    }

    #[test]
    fn test_navigation_thread_config_default() {
        let config = NavigationThreadConfig::default();

        assert!(config.update_rate_hz > 0.0);
    }
}
