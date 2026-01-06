//! Sensor thread: High-frequency UDP reading, odometry, safety, and command sending.
//!
//! This thread runs at ~100Hz and handles:
//! - Reading UDP sensor data (non-blocking)
//! - Updating odometry from wheel encoders
//! - Checking safety sensors (bumpers, cliffs)
//! - Sending lidar scans to mapping thread
//! - Sending velocity commands to robot

use std::sync::Arc;
use std::sync::mpsc::SyncSender;
use std::time::{Duration, Instant};

use crate::client::SangamClient;
use crate::config::DhruvaConfig;
use crate::error::Result;
use crate::odometry::Odometry;
use crate::shared::{SharedState, messages::LidarScanMsg};

use vastu_slam::Pose2D;

/// Sensor thread state and logic.
pub struct SensorThread {
    config: DhruvaConfig,
    shared_state: Arc<SharedState>,
    lidar_tx: SyncSender<LidarScanMsg>,
    client: Option<SangamClient>,
    odometry: Odometry,
    /// Last encoder pose - cached for lidar synchronization.
    /// Encoder data arrives at ~110Hz, lidar at ~5Hz.
    /// When a lidar scan arrives, we use this cached pose (from the most recent
    /// encoder update) instead of the current pose to avoid synchronization drift.
    last_encoder_pose: Pose2D,
}

impl SensorThread {
    /// Create a new sensor thread.
    pub fn new(
        config: DhruvaConfig,
        shared_state: Arc<SharedState>,
        lidar_tx: SyncSender<LidarScanMsg>,
    ) -> Self {
        // Robot starts at origin - odometry accumulates from (0,0,0)
        let start_pose = Pose2D::new(0.0, 0.0, 0.0);

        let odometry = Odometry::new(
            config.robot.wheel_base,
            config.robot.ticks_per_meter,
            start_pose,
        );

        Self {
            config,
            shared_state,
            lidar_tx,
            client: None,
            odometry,
            last_encoder_pose: start_pose,
        }
    }

    /// Run the sensor thread main loop.
    pub fn run(&mut self) -> Result<()> {
        // Connect to SangamIO
        self.connect()?;

        let command_interval = Duration::from_millis(50); // 20Hz command rate
        let mut last_command_time = Instant::now();

        tracing::info!("Sensor thread started");

        // Main sensor loop
        loop {
            // Check for shutdown
            if self.shared_state.should_shutdown() {
                tracing::info!("Sensor thread shutting down");
                self.send_stop();
                break;
            }

            // Check for safety stop
            if self.shared_state.is_safety_stop() {
                self.send_stop();
                // Continue running to maintain connection but don't process
                std::thread::sleep(Duration::from_millis(100));
                continue;
            }

            // Drain UDP buffer
            self.process_udp_messages();

            // Send velocity command at fixed interval
            if last_command_time.elapsed() >= command_interval {
                self.send_velocity_command();
                last_command_time = Instant::now();
            }

            // Small sleep to avoid busy-waiting (target ~100Hz)
            std::thread::sleep(Duration::from_millis(5));
        }

        Ok(())
    }

    /// Connect to SangamIO daemon.
    fn connect(&mut self) -> Result<()> {
        let addr = self.config.address();
        tracing::info!("Sensor thread connecting to SangamIO at {}...", addr);

        let timeout = Duration::from_millis(self.config.connection.timeout_ms);
        let mut client = SangamClient::connect_timeout(&addr, timeout)?;

        // Set non-blocking for UDP reads
        client.set_timeout(Some(Duration::from_millis(10)))?;

        // Enable drive and lidar components
        tracing::info!("Enabling drive and lidar...");
        client.enable_drive()?;
        client.enable_lidar()?;

        // Wait for sensor data to start flowing
        std::thread::sleep(Duration::from_millis(500));

        self.client = Some(client);
        tracing::info!("Sensor thread connected to SangamIO");

        Ok(())
    }

    /// Process all available UDP messages.
    fn process_udp_messages(&mut self) {
        let client = match self.client.as_mut() {
            Some(c) => c,
            None => return,
        };

        let mut messages_processed = 0;
        const MAX_MESSAGES_PER_ITERATION: usize = 50;

        // Buffer for the latest lidar scan
        let mut pending_lidar: Option<LidarScanMsg> = None;

        loop {
            match client.recv_udp() {
                Ok(Some(msg)) => {
                    messages_processed += 1;

                    // Update odometry from encoders
                    if let Some((left, right)) = msg.encoder_ticks() {
                        self.odometry.update(left, right);
                        // Cache pose for lidar synchronization
                        self.last_encoder_pose = self.odometry.pose();
                        // Update shared odometry (mapping thread will apply corrections)
                        self.shared_state.set_odometry(self.last_encoder_pose);
                    }

                    // Check safety sensors - bumpers
                    if let Some((bump_l, bump_r)) = msg.bumper_state()
                        && (bump_l || bump_r)
                    {
                        self.shared_state.trigger_safety_stop(format!(
                            "Bumper triggered: left={}, right={}",
                            bump_l, bump_r
                        ));
                        return;
                    }

                    // Check safety sensors - cliffs
                    if let Some((cl_ls, cl_lf, cl_rf, cl_rs)) = msg.cliff_state()
                        && (cl_ls || cl_lf || cl_rf || cl_rs)
                    {
                        self.shared_state.trigger_safety_stop(format!(
                            "Cliff detected: ls={}, lf={}, rf={}, rs={}",
                            cl_ls, cl_lf, cl_rf, cl_rs
                        ));
                        return;
                    }

                    // Buffer lidar scan (keep only the latest)
                    // Use cached encoder pose for synchronization - encoder data arrives
                    // at ~110Hz, lidar at ~5Hz, so last_encoder_pose is at most ~9ms old
                    if let Some(points) = msg.as_lidar() {
                        pending_lidar = Some(LidarScanMsg {
                            points: points.clone(),
                            encoder_pose: self.last_encoder_pose,
                        });
                    }

                    // Limit messages per iteration
                    if messages_processed >= MAX_MESSAGES_PER_ITERATION {
                        break;
                    }
                }
                Ok(None) => {
                    // No more data available
                    break;
                }
                Err(e) => {
                    tracing::error!("UDP receive error: {}", e);
                    self.shared_state
                        .trigger_safety_stop(format!("Connection error: {}", e));
                    return;
                }
            }
        }

        // Send the latest lidar scan to mapping thread
        if let Some(scan_msg) = pending_lidar {
            // Use try_send to avoid blocking if channel is full
            if let Err(e) = self.lidar_tx.try_send(scan_msg) {
                tracing::warn!("Failed to send lidar scan to mapping thread: {}", e);
            }
        }
    }

    /// Send velocity command to robot.
    fn send_velocity_command(&mut self) {
        let client = match self.client.as_mut() {
            Some(c) => c,
            None => return,
        };

        let (linear, angular) = self.shared_state.velocity();

        if let Err(e) = client.send_drive_command(linear, angular) {
            tracing::error!("Failed to send drive command: {}", e);
            self.shared_state
                .trigger_safety_stop(format!("Command send failed: {}", e));
        }
    }

    /// Send stop command.
    fn send_stop(&mut self) {
        if let Some(client) = self.client.as_mut()
            && let Err(e) = client.send_stop()
        {
            tracing::error!("Failed to send stop command: {}", e);
        }
    }
}
