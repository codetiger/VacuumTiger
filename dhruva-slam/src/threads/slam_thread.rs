//! SLAM Thread - Real-time sensor processing.
//!
//! This thread:
//! - Receives sensor data via UDP from SangamIO (live mode)
//! - Processes odometry and lidar data through SLAM engine
//! - Updates SharedState with pose, map, and sensor status
//! - Handles commands from Command Thread via channel (non-blocking)
//!
//! # Communication Architecture
//!
//! **Live mode**: Event-driven via UDP
//! - Sensors arrive via crossbeam channels from SangamUdpReceiver
//! - Uses `crossbeam::select!` to wait on sensor_status + lidar channels
//! - Processes immediately on arrival (no fixed rate loop)
//!
//! Note: Some methods are defined for planned features.
//!
//! **Bag mode**: Same as before
//! - Reads from bag file with timing control
//!
//! CRITICAL: This thread NEVER blocks on commands or publishing.
//! Sensor data flows uninterrupted.

use crossbeam_channel::select;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Instant;

use crate::core::types::{LaserScan, Pose2D, PoseTracker};
use crate::engine::slam::{OnlineSlam, OnlineSlamConfig, SlamEngine, SlamMode};
use crate::io::bag::{BagMessage, BagPlayer, SensorStatusMsg};
use crate::io::sangam_client::Message as SangamMessage;
use crate::io::sangam_udp_receiver::{
    LidarData, ReceiverConfig, SangamUdpReceiver, SensorStatusData,
};
use crate::sensors::odometry::{DynOdometry, DynOdometryConfig, OdometryType};
use crate::sensors::preprocessing::{PreprocessorConfig, ScanPreprocessor};
use crate::state::{
    CommandReceiver, CommandResponse, CommandResult, RobotState, SharedStateHandle, SlamCommand,
};

/// Configuration for the SLAM thread.
#[derive(Debug, Clone)]
pub struct SlamThreadConfig {
    /// SangamIO address for TCP commands and UDP sensor data.
    /// Format: "host:port" (e.g., "192.168.68.101:5555").
    /// Both TCP and UDP use the same port.
    pub sangam_address: String,
    /// UDP port override for sensor streaming (optional).
    /// If specified, UDP receiver will bind to this port instead of the TCP port.
    /// Useful for localhost testing where TCP and UDP need different ports.
    pub udp_port: Option<u16>,
    /// SLAM engine configuration.
    pub slam_config: OnlineSlamConfig,
    /// Odometry algorithm type.
    pub odometry_type: OdometryType,
    /// Odometry configuration.
    pub odometry_config: DynOdometryConfig,
    /// Scan preprocessor configuration.
    pub preprocessor_config: PreprocessorConfig,
    /// Bag file path for offline playback (None for live mode).
    pub bag_file: Option<String>,
    /// Loop bag file playback.
    pub loop_bag: bool,
}

/// SLAM Thread handle.
pub struct SlamThread {
    handle: JoinHandle<()>,
}

impl SlamThread {
    /// Spawn the SLAM thread.
    pub fn spawn(
        config: SlamThreadConfig,
        shared_state: SharedStateHandle,
        command_rx: CommandReceiver,
        running: Arc<AtomicBool>,
    ) -> Self {
        let handle = thread::Builder::new()
            .name("slam".into())
            .spawn(move || {
                if let Some(bag_path) = config.bag_file.clone() {
                    run_bag_loop(config, bag_path, shared_state, command_rx, running);
                } else {
                    run_live_loop(config, shared_state, command_rx, running);
                }
            })
            .expect("Failed to spawn SLAM thread");

        Self { handle }
    }

    /// Wait for thread to finish.
    pub fn join(self) -> thread::Result<()> {
        self.handle.join()
    }
}

/// Run SLAM loop with live SangamIO UDP connection.
///
/// Uses event-driven architecture:
/// - UDP receiver runs in separate thread, sends data via crossbeam channels
/// - SLAM thread uses `select!` to wait on sensor_status + lidar channels
/// - Processes immediately on arrival (no fixed rate loop)
fn run_live_loop(
    config: SlamThreadConfig,
    shared_state: SharedStateHandle,
    command_rx: CommandReceiver,
    running: Arc<AtomicBool>,
) {
    log::info!("SLAM thread starting (live mode - UDP)");

    // Get UDP port: use explicit udp_port if set, otherwise extract from sangam_address
    let tcp_port = config
        .sangam_address
        .rsplit(':')
        .next()
        .and_then(|p| p.parse::<u16>().ok())
        .unwrap_or(5555);
    let udp_port = config.udp_port.unwrap_or(tcp_port);

    let receiver_config = ReceiverConfig {
        bind_addr: format!("0.0.0.0:{}", udp_port),
    };

    if udp_port == tcp_port {
        log::info!(
            "  Starting UDP receiver on port {} (same as TCP)...",
            udp_port
        );
    } else {
        log::info!(
            "  Starting UDP receiver on port {} (TCP on {})...",
            udp_port,
            tcp_port
        );
    }

    // Create UDP receiver
    let (receiver, sensor_rx, lidar_rx) =
        match SangamUdpReceiver::new(receiver_config, running.clone()) {
            Ok(r) => r,
            Err(e) => {
                log::error!("Failed to create UDP receiver: {}", e);
                return;
            }
        };

    // Spawn UDP receiver thread
    let _receiver_running = running.clone();
    let receiver_handle = thread::Builder::new()
        .name("udp-receiver".into())
        .spawn(move || receiver.run())
        .expect("Failed to spawn UDP receiver thread");

    log::info!("  UDP receiver started, waiting for sensor data...");

    // Initialize components
    let mut slam_context = SlamContext::new(&config);

    // Main event loop - process sensor data as it arrives
    while running.load(Ordering::Relaxed) {
        // Check for commands (non-blocking)
        slam_context.process_commands(&command_rx, &shared_state);

        // Wait for sensor data from UDP channels using select!
        // This is event-driven: we process immediately when data arrives
        select! {
            recv(sensor_rx) -> result => {
                if let Ok(sensor_data) = result {
                    slam_context.process_udp_sensor_status(&sensor_data, &shared_state);
                }
            }
            recv(lidar_rx) -> result => {
                if let Ok(lidar_data) = result {
                    slam_context.process_udp_lidar(&lidar_data, &shared_state);
                }
            }
            // Timeout to allow checking running flag and processing commands
            default(std::time::Duration::from_millis(10)) => {}
        }
    }

    log::info!("SLAM thread shutting down (waiting for UDP receiver)...");

    // Wait for UDP receiver to stop
    receiver_handle.join().ok();

    log::info!("SLAM thread shutdown complete");
}

/// Run SLAM loop with bag file playback.
fn run_bag_loop(
    config: SlamThreadConfig,
    bag_path: String,
    shared_state: SharedStateHandle,
    command_rx: CommandReceiver,
    running: Arc<AtomicBool>,
) {
    log::info!("SLAM thread starting (bag playback)");
    log::info!("  Bag file: {}", bag_path);

    loop {
        // Open bag file
        let mut player = match BagPlayer::open(&bag_path) {
            Ok(p) => p,
            Err(e) => {
                log::error!("Failed to open bag file: {}", e);
                return;
            }
        };
        player.set_speed(1.0); // Real-time playback

        // Initialize components
        let mut slam_context = SlamContext::new(&config);

        // Main loop
        while running.load(Ordering::Relaxed) {
            // 1. Check for commands (non-blocking)
            slam_context.process_commands(&command_rx, &shared_state);

            // 2. Get next message from bag
            let msg = match player.next() {
                Ok(Some(m)) => m,
                Ok(None) => {
                    log::info!("End of bag file reached");
                    break;
                }
                Err(e) => {
                    log::error!("Bag read error: {}", e);
                    break;
                }
            };

            match msg {
                BagMessage::SensorStatus(status) => {
                    slam_context.process_bag_sensor_status(&status, &shared_state);
                }
                BagMessage::Lidar(timestamped) => {
                    slam_context.process_lidar(
                        &timestamped.data,
                        timestamped.timestamp_us,
                        &shared_state,
                    );
                }
                BagMessage::Odometry(_) => {
                    // Skip raw odometry messages, we compute our own
                }
            }
        }

        // Check if we should loop
        if !config.loop_bag || !running.load(Ordering::Relaxed) {
            break;
        }
        log::info!("Looping bag file...");
    }

    log::info!("SLAM thread shutting down (bag playback)");
}

/// Robot radius used for bumper obstacle marking.
const ROBOT_RADIUS: f32 = 0.18;

/// Internal SLAM context holding all processing state.
struct SlamContext {
    /// SLAM engine.
    slam: OnlineSlam,
    /// Odometry estimator.
    odometry: DynOdometry,
    /// Scan preprocessor.
    preprocessor: ScanPreprocessor,
    /// Pose tracker for odometry.
    odom_tracker: PoseTracker,
    /// Last odometry pose (for delta computation).
    last_odom_pose: Pose2D,
    /// Whether SLAM is in mapping mode.
    is_mapping: bool,
    /// Whether SLAM is in localization mode.
    is_localizing: bool,
    /// Total distance traveled.
    distance_traveled: f32,
    /// Last pose for distance computation.
    last_pose_for_distance: Pose2D,
    /// Last map update time.
    last_map_update: Instant,
    /// Previous bumper state for edge detection (left, right).
    prev_bumper_state: (bool, bool),
}

impl SlamContext {
    fn new(config: &SlamThreadConfig) -> Self {
        let slam = OnlineSlam::new(config.slam_config.clone());
        log::info!(
            "Initializing odometry with algorithm: {:?}",
            config.odometry_type
        );
        let odometry = DynOdometry::new(config.odometry_type, config.odometry_config.clone());
        let preprocessor = ScanPreprocessor::new(config.preprocessor_config.clone());

        Self {
            slam,
            odometry,
            preprocessor,
            odom_tracker: PoseTracker::new(),
            last_odom_pose: Pose2D::identity(),
            is_mapping: false,
            is_localizing: false,
            distance_traveled: 0.0,
            last_pose_for_distance: Pose2D::identity(),
            last_map_update: Instant::now(),
            prev_bumper_state: (false, false),
        }
    }

    /// Process commands from command channel (non-blocking).
    fn process_commands(&mut self, command_rx: &CommandReceiver, shared_state: &SharedStateHandle) {
        while let Ok(cmd_with_response) = command_rx.try_recv() {
            let result = self.handle_command(&cmd_with_response.command, shared_state);
            // Send response back (ignore send errors - command thread may have timed out)
            cmd_with_response.response_tx.send(result).ok();
        }
    }

    /// Handle a single SLAM command.
    fn handle_command(
        &mut self,
        cmd: &SlamCommand,
        shared_state: &SharedStateHandle,
    ) -> CommandResult {
        match cmd {
            SlamCommand::StartMapping { map_id, name } => {
                log::info!("Starting mapping: {} ({})", name, map_id);
                self.slam.reset();
                self.slam.set_mode(SlamMode::Mapping);
                self.is_mapping = true;
                self.is_localizing = false;
                self.distance_traveled = 0.0;
                self.last_pose_for_distance = Pose2D::identity();

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.start_mapping(map_id.clone(), name.clone());
                    // Enable exploration so the robot starts moving autonomously
                    state.exploration_state.mode = crate::state::ExplorationMode::Exploring;
                    state.exploration_state.status = "Starting mapping exploration".to_string();
                }

                Ok(CommandResponse::MappingStarted {
                    map_id: map_id.clone(),
                })
            }

            SlamCommand::StopMapping => {
                log::info!("Stopping mapping");
                self.slam.set_mode(SlamMode::Idle);
                self.is_mapping = false;

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.stop_mapping();
                    // Disable exploration when mapping stops
                    state.exploration_state.mode = crate::state::ExplorationMode::Disabled;
                    state.exploration_state.status = "Mapping stopped".to_string();
                }

                Ok(CommandResponse::MappingStopped)
            }

            SlamCommand::ClearMap => {
                log::info!("Clearing map");
                self.slam.reset();
                self.distance_traveled = 0.0;
                self.last_pose_for_distance = Pose2D::identity();

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.current_map = None;
                    state.robot_status.keyframe_count = 0;
                    state.robot_status.loop_closures = 0;
                    state.robot_status.distance_traveled_m = 0.0;
                    state.robot_status.map_area_m2 = 0.0;
                }

                Ok(CommandResponse::MapCleared)
            }

            SlamCommand::EnableMap { map_id, map, name } => {
                log::info!("Enabling map for localization: {} ({})", name, map_id);

                // Set the reference map for localization matching
                self.slam.set_reference_map(map.clone());

                // Set localization mode
                self.slam.set_mode(SlamMode::Localization);
                self.is_mapping = false;
                self.is_localizing = true;

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.enable_localization(map_id.clone());
                    state.current_map_name = name.clone();
                    state.update_map(map);
                }

                Ok(CommandResponse::MapEnabled)
            }

            SlamCommand::EmergencyStop => {
                log::warn!("Emergency stop activated!");

                // Stop mapping/localization
                self.slam.set_mode(SlamMode::Idle);
                self.is_mapping = false;
                self.is_localizing = false;

                // Update shared state to stop exploration (which will stop motors and lidar)
                if let Ok(mut state) = shared_state.write() {
                    state.stop_mapping();
                    state.exploration_state.mode = crate::state::ExplorationMode::Disabled;
                    state.exploration_state.status = "Emergency stopped".to_string();
                }

                Ok(CommandResponse::EmergencyStopped)
            }
        }
    }

    /// Process sensor status from live SangamIO message.
    fn process_sensor_status(
        &mut self,
        msg: &SangamMessage,
        timestamp_us: u64,
        shared_state: &SharedStateHandle,
    ) {
        let (left, right) = match msg.encoder_ticks() {
            Some(ticks) => ticks,
            None => return,
        };
        let gyro_yaw = msg.gyro_yaw_raw().unwrap_or(0);

        // Extract bumper state
        let bumper_state = msg.bumper_state();

        // Extract cliff sensor state
        let cliff_state = msg.cliff_state();

        self.process_odometry(left, right, gyro_yaw, timestamp_us, shared_state);

        // Handle bumper obstacle marking (detect rising edge)
        if let Some((bumper_left, bumper_right)) = bumper_state {
            let bumper_left_rising = bumper_left && !self.prev_bumper_state.0;
            let bumper_right_rising = bumper_right && !self.prev_bumper_state.1;

            if (bumper_left_rising || bumper_right_rising) && self.is_mapping {
                let pose = self.slam.current_pose();
                log::info!(
                    "Bumper triggered (left={}, right={}) at pose ({:.2}, {:.2}, {:.2}°) - marking obstacle",
                    bumper_left,
                    bumper_right,
                    pose.x,
                    pose.y,
                    pose.theta.to_degrees()
                );

                self.slam.mark_bumper_obstacle(
                    pose.x,
                    pose.y,
                    pose.theta,
                    bumper_left,
                    bumper_right,
                    ROBOT_RADIUS,
                );
            }

            // Update previous bumper state
            self.prev_bumper_state = (bumper_left, bumper_right);
        }

        // Update bumper and cliff states in shared state
        if let Ok(mut state) = shared_state.write() {
            // Update bumper state
            if let Some((left_bumper, right_bumper)) = bumper_state {
                state.sensor_status.bumper.left_pressed = left_bumper;
                state.sensor_status.bumper.right_pressed = right_bumper;
                if left_bumper || right_bumper {
                    state.sensor_status.bumper.last_trigger_us = Some(timestamp_us);
                }
            }

            // Update cliff state
            if let Some((left_side, left_front, right_front, right_side)) = cliff_state {
                state.sensor_status.cliff.left_side = left_side;
                state.sensor_status.cliff.left_front = left_front;
                state.sensor_status.cliff.right_front = right_front;
                state.sensor_status.cliff.right_side = right_side;
                if left_side || left_front || right_front || right_side {
                    state.sensor_status.cliff.last_trigger_us = Some(timestamp_us);
                }
            }
        }
    }

    /// Process sensor status from bag file.
    fn process_bag_sensor_status(
        &mut self,
        status: &SensorStatusMsg,
        shared_state: &SharedStateHandle,
    ) {
        let timestamp_us = status.timestamp_us;
        self.process_odometry(
            status.encoder.left,
            status.encoder.right,
            status.gyro_raw[2],
            timestamp_us,
            shared_state,
        );
    }

    /// Common odometry processing.
    fn process_odometry(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
        shared_state: &SharedStateHandle,
    ) {
        // Update odometry
        if let Some(odom_pose) = self.odometry.update(left, right, gyro_yaw, timestamp_us) {
            self.odom_tracker.set(odom_pose);

            // Interpolate: slam_pose + delta since last SLAM update
            let odom_delta = self.last_odom_pose.inverse().compose(&odom_pose);
            let published_pose = self.slam.current_pose().compose(&odom_delta);

            // Update distance traveled
            let dx = published_pose.x - self.last_pose_for_distance.x;
            let dy = published_pose.y - self.last_pose_for_distance.y;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist > 0.01 {
                // Only count if moved more than 1cm
                self.distance_traveled += dist;
                self.last_pose_for_distance = published_pose;
            }

            // Update shared state (quick write)
            if let Ok(mut state) = shared_state.write() {
                state.robot_status.pose = published_pose;
                state.robot_status.timestamp_us = timestamp_us;
                state.robot_status.distance_traveled_m = self.distance_traveled;

                // Update sensor status
                state.sensor_status.left_encoder_ticks = left as i32;
                state.sensor_status.right_encoder_ticks = right as i32;
                state.sensor_status.gyro_z_radps = gyro_yaw as f32 * 0.001; // Rough conversion
                state.sensor_status.raw_odometry = odom_pose;
                state.sensor_status.timestamp_us = timestamp_us;
            }
        }
    }

    /// Process lidar scan.
    fn process_lidar(
        &mut self,
        lidar_data: &crate::io::LidarScan,
        timestamp_us: u64,
        shared_state: &SharedStateHandle,
    ) {
        // Preprocess scan
        let laser_scan = LaserScan::from_lidar_scan(lidar_data);
        let scan_cloud = self.preprocessor.process(&laser_scan);

        // Skip sparse scans
        if scan_cloud.len() < 50 {
            log::debug!("Skipping sparse scan: {} points", scan_cloud.len());
            return;
        }

        // Log scan processing decision
        if !self.is_mapping && !self.is_localizing {
            static SKIP_COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
            let count = SKIP_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if count < 5 || count.is_multiple_of(50) {
                log::info!(
                    "Skipping scan (not mapping/localizing): is_mapping={}, is_localizing={}, skip#{}",
                    self.is_mapping,
                    self.is_localizing,
                    count
                );
            }
        }

        // Compute odometry delta since last scan
        let current_odom = self.odom_tracker.pose();
        let odom_delta = self.last_odom_pose.inverse().compose(&current_odom);

        // Only process if mapping or localizing
        if self.is_mapping || self.is_localizing {
            // Process through OnlineSlam
            let result = self
                .slam
                .process_scan(&scan_cloud, &odom_delta, timestamp_us);

            // Log first few pose updates
            static POSE_LOG_COUNT: std::sync::atomic::AtomicU32 =
                std::sync::atomic::AtomicU32::new(0);
            let log_count = POSE_LOG_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if log_count < 5 || log_count.is_multiple_of(50) {
                log::info!(
                    "SLAM result: pose=({:.3}, {:.3}, {:.1}°), score={:.3}, odom_delta=({:.3}, {:.3}, {:.1}°)",
                    result.pose.x,
                    result.pose.y,
                    result.pose.theta.to_degrees(),
                    result.match_score,
                    odom_delta.x,
                    odom_delta.y,
                    odom_delta.theta.to_degrees()
                );
            }

            // Update shared state
            if let Ok(mut state) = shared_state.write() {
                let status = self.slam.status();
                state.update_from_slam(
                    result.pose,
                    if self.is_mapping {
                        RobotState::Mapping
                    } else if status.is_lost {
                        RobotState::Lost
                    } else {
                        RobotState::Localizing
                    },
                    status.num_keyframes as u32,
                    self.slam.total_loop_closures() as u32,
                    result.match_score,
                    timestamp_us,
                );

                // Update lidar data in sensor status
                // Apply lidar angle_offset to transform from lidar frame to robot frame
                let angle_offset = self.preprocessor.lidar_offset().total_theta();
                state.sensor_status.lidar_ranges = laser_scan.ranges.clone();
                state.sensor_status.lidar_angles = (0..laser_scan.ranges.len())
                    .map(|i| {
                        let raw_angle =
                            laser_scan.angle_min + i as f32 * laser_scan.angle_increment;
                        raw_angle + angle_offset // Apply lidar angular offset
                    })
                    .collect();
            }

            // Update map periodically (not every scan - expensive)
            if self.last_map_update.elapsed().as_secs_f32() > 0.5 {
                if let Ok(mut state) = shared_state.write() {
                    let map = self.slam.global_map();
                    let (w, h) = map.dimensions();
                    log::debug!("Updating map in shared state: {}x{}", w, h);
                    state.update_map(map);
                }
                self.last_map_update = Instant::now();
            }
        }

        // Update for next iteration
        self.last_odom_pose = current_odom;
        self.odom_tracker.take_snapshot();
    }

    /// Process sensor status from UDP receiver.
    fn process_udp_sensor_status(
        &mut self,
        data: &SensorStatusData,
        shared_state: &SharedStateHandle,
    ) {
        let timestamp_us = data.timestamp_us;

        // Process odometry
        self.process_odometry(
            data.left_encoder,
            data.right_encoder,
            data.gyro_yaw_raw,
            timestamp_us,
            shared_state,
        );

        // Detect bumper rising edge and mark obstacle in map
        // Only trigger on rising edge to avoid marking multiple times for same collision
        let bumper_left_rising = data.bumper_left && !self.prev_bumper_state.0;
        let bumper_right_rising = data.bumper_right && !self.prev_bumper_state.1;

        if (bumper_left_rising || bumper_right_rising) && self.is_mapping {
            // Get current pose for obstacle marking
            let pose = self.slam.current_pose();
            log::info!(
                "Bumper triggered (left={}, right={}) at pose ({:.2}, {:.2}, {:.2}°) - marking obstacle",
                data.bumper_left,
                data.bumper_right,
                pose.x,
                pose.y,
                pose.theta.to_degrees()
            );

            // Mark obstacle in map
            self.slam.mark_bumper_obstacle(
                pose.x,
                pose.y,
                pose.theta,
                data.bumper_left,
                data.bumper_right,
                ROBOT_RADIUS,
            );
        }

        // Update previous bumper state for next iteration
        self.prev_bumper_state = (data.bumper_left, data.bumper_right);

        // Update bumper and cliff states in shared state
        if let Ok(mut state) = shared_state.write() {
            // Update bumper state
            state.sensor_status.bumper.left_pressed = data.bumper_left;
            state.sensor_status.bumper.right_pressed = data.bumper_right;
            if data.bumper_left || data.bumper_right {
                state.sensor_status.bumper.last_trigger_us = Some(timestamp_us);
            }

            // Update cliff state
            state.sensor_status.cliff.left_side = data.cliff_left_side;
            state.sensor_status.cliff.left_front = data.cliff_left_front;
            state.sensor_status.cliff.right_front = data.cliff_right_front;
            state.sensor_status.cliff.right_side = data.cliff_right_side;
            if data.cliff_left_side
                || data.cliff_left_front
                || data.cliff_right_front
                || data.cliff_right_side
            {
                state.sensor_status.cliff.last_trigger_us = Some(timestamp_us);
            }
        }
    }

    /// Process lidar data from UDP receiver.
    fn process_udp_lidar(&mut self, data: &LidarData, shared_state: &SharedStateHandle) {
        let timestamp_us = data.timestamp_us;

        // Log first few scans to trace processing
        static SCAN_COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
        let count = SCAN_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if count < 5 || count.is_multiple_of(50) {
            log::info!(
                "Processing UDP lidar: {} points, is_mapping={}, is_localizing={}, scan#{}",
                data.points.len(),
                self.is_mapping,
                self.is_localizing,
                count
            );
        }

        // Convert UDP LidarData to LidarScan format expected by process_lidar
        let lidar_scan: crate::io::LidarScan = data
            .points
            .iter()
            .map(|(angle, dist, quality)| (*angle, *dist, *quality))
            .collect();

        self.process_lidar(&lidar_scan, timestamp_us, shared_state);
    }
}
