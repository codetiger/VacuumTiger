//! Publisher Thread - Unified client interface.
//!
//! This thread provides a single-port interface for clients:
//! - **TCP**: Bidirectional - commands (inbound) + maps/responses (outbound)
//! - **UDP**: Outbound only - high-frequency status streams
//!
//! # Architecture
//!
//! Single port (5557) following the SangamIO pattern:
//! - TCP connection is bidirectional (client sends commands, server sends maps + responses)
//! - UDP unicast for high-frequency data (RobotStatus, SensorStatus, NavigationStatus)
//! - When TCP client connects, they are automatically registered for UDP streaming
//!
//! # Wire Format
//!
//! Both transports use the same length-prefixed Protobuf format:
//! - 4 bytes: message length (big-endian u32)
//! - N bytes: Protobuf message (DhruvaStream, DhruvaCommand, or DhruvaResponse)

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use prost::Message;

use super::StreamConfig;
use crate::io::map_manager::MapManager;
use crate::io::streaming::{DhruvaUdpPublisher, create_client_registry};
use crate::navigation::NavTarget;
use crate::state::{
    CommandResponse, CommandSender, SharedStateHandle, SlamCommand, send_command_sync,
};
use crate::threads::NavCommandSender;

/// Publisher Thread handle.
pub struct PublisherThread {
    handle: JoinHandle<()>,
}

impl PublisherThread {
    /// Spawn the publisher thread.
    ///
    /// This thread handles both data streaming (outbound) and commands (inbound)
    /// on a single TCP port, plus UDP for high-frequency status.
    pub fn spawn(
        port: u16,
        shared_state: SharedStateHandle,
        config: StreamConfig,
        slam_command_tx: CommandSender,
        nav_command_tx: Option<NavCommandSender>,
        map_manager: Arc<Mutex<MapManager>>,
        running: Arc<AtomicBool>,
    ) -> Self {
        let handle = thread::Builder::new()
            .name("publisher".into())
            .spawn(move || {
                run_publisher_loop(
                    port,
                    shared_state,
                    config,
                    slam_command_tx,
                    nav_command_tx,
                    map_manager,
                    running,
                );
            })
            .expect("Failed to spawn publisher thread");

        Self { handle }
    }

    /// Wait for thread to finish.
    pub fn join(self) -> thread::Result<()> {
        self.handle.join()
    }
}

/// Connected client with bidirectional TCP communication.
struct Client {
    stream: TcpStream,
    addr: String,
    /// Socket address for UDP registration.
    socket_addr: SocketAddr,
    /// Whether we failed to write recently (for cleanup).
    failed: bool,
    /// Read buffer for partial message reads.
    read_buf: Vec<u8>,
    /// Expected message length (0 if not yet read).
    pending_len: usize,
}

impl Client {
    fn new(stream: TcpStream, addr: SocketAddr) -> Self {
        // Set non-blocking for both reads and writes
        stream.set_nonblocking(true).ok();
        // Set write timeout as fallback
        stream
            .set_write_timeout(Some(Duration::from_millis(100)))
            .ok();
        Self {
            stream,
            addr: addr.to_string(),
            socket_addr: addr,
            failed: false,
            read_buf: Vec::with_capacity(1024),
            pending_len: 0,
        }
    }

    /// Try to write data to client. Returns false if write fails.
    fn try_write(&mut self, data: &[u8]) -> bool {
        match self.stream.write_all(data) {
            Ok(()) => {
                self.stream.flush().ok();
                true
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                // Would block - client is slow, skip this message
                true
            }
            Err(_) => {
                self.failed = true;
                false
            }
        }
    }

    /// Try to read a command from the client (non-blocking).
    /// Returns Some(command) if a complete command was received.
    fn try_read_command(&mut self) -> Option<crate::io::streaming::DhruvaCommand> {
        // First, try to read length prefix if we don't have it
        if self.pending_len == 0 {
            let mut len_buf = [0u8; 4];
            match self.stream.read(&mut len_buf) {
                Ok(4) => {
                    self.pending_len = u32::from_be_bytes(len_buf) as usize;
                    // Sanity check
                    if self.pending_len > 1024 * 1024 {
                        log::warn!("Client {} sent oversized message", self.addr);
                        self.failed = true;
                        return None;
                    }
                    self.read_buf.clear();
                }
                Ok(0) => {
                    // Connection closed
                    self.failed = true;
                    return None;
                }
                Ok(_) => {
                    // Partial read of length - shouldn't happen with TCP
                    return None;
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No data available
                    return None;
                }
                Err(_) => {
                    self.failed = true;
                    return None;
                }
            }
        }

        // Now try to read the message body
        if self.pending_len > 0 {
            let remaining = self.pending_len - self.read_buf.len();
            let mut buf = vec![0u8; remaining.min(4096)];
            match self.stream.read(&mut buf) {
                Ok(0) => {
                    // Connection closed
                    self.failed = true;
                    return None;
                }
                Ok(n) => {
                    self.read_buf.extend_from_slice(&buf[..n]);
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No data available
                    return None;
                }
                Err(_) => {
                    self.failed = true;
                    return None;
                }
            }

            // Check if we have the complete message
            if self.read_buf.len() >= self.pending_len {
                let msg_buf = self.read_buf.drain(..self.pending_len).collect::<Vec<_>>();
                self.pending_len = 0;

                // Decode the command
                match crate::io::streaming::DhruvaCommand::decode(&msg_buf[..]) {
                    Ok(cmd) => return Some(cmd),
                    Err(e) => {
                        log::warn!("Failed to decode command from {}: {}", self.addr, e);
                        return None;
                    }
                }
            }
        }

        None
    }

    /// Send a response to the client.
    fn send_response(&mut self, response: &crate::io::streaming::DhruvaResponse) -> bool {
        let buf = response.encode_to_vec();
        let len = buf.len() as u32;
        let mut frame = Vec::with_capacity(4 + buf.len());
        frame.extend_from_slice(&len.to_be_bytes());
        frame.extend_from_slice(&buf);
        self.try_write(&frame)
    }
}

/// Main publisher loop.
fn run_publisher_loop(
    port: u16,
    shared_state: SharedStateHandle,
    config: StreamConfig,
    slam_command_tx: CommandSender,
    nav_command_tx: Option<NavCommandSender>,
    map_manager: Arc<Mutex<MapManager>>,
    running: Arc<AtomicBool>,
) {
    log::info!(
        "Publisher thread starting on port {} (TCP commands+data, UDP status)",
        port
    );

    // Create UDP client registry (shared with UDP publisher)
    let udp_registry = create_client_registry();

    // Create UDP publisher for high-frequency streams
    let mut udp_publisher =
        match DhruvaUdpPublisher::new(port, udp_registry.clone(), running.clone()) {
            Ok(p) => p,
            Err(e) => {
                log::error!("Failed to create UDP publisher on port {}: {}", port, e);
                return;
            }
        };

    // Bind TCP listener
    let listener = match TcpListener::bind(format!("0.0.0.0:{}", port)) {
        Ok(l) => l,
        Err(e) => {
            log::error!("Failed to bind TCP stream port {}: {}", port, e);
            return;
        }
    };

    // Set non-blocking
    listener
        .set_nonblocking(true)
        .expect("Failed to set non-blocking");

    log::info!(
        "Publisher thread listening on port {} (TCP for maps, UDP for status)",
        port
    );

    let mut clients: Vec<Client> = Vec::new();

    // Timing intervals
    let robot_interval = Duration::from_secs_f64(1.0 / config.robot_status_hz as f64);
    let sensor_interval = Duration::from_secs_f64(1.0 / config.sensor_status_hz as f64);
    let map_interval = Duration::from_secs_f64(1.0 / config.map_hz as f64);
    let navigation_interval = Duration::from_secs_f64(1.0 / config.navigation_hz as f64);
    let mapping_interval = Duration::from_secs_f64(1.0 / 2.0); // 2Hz per proto spec

    let mut last_robot = Instant::now();
    let mut last_sensor = Instant::now();
    let mut last_map = Instant::now();
    let mut last_navigation = Instant::now();
    let mut last_mapping = Instant::now();
    let mut last_map_list_dirty = false;

    while running.load(Ordering::Relaxed) {
        // Accept new TCP clients (non-blocking)
        while let Ok((stream, addr)) = listener.accept() {
            log::info!(
                "TCP client connected: {} - registering for UDP streaming",
                addr
            );

            // Register client for UDP streaming (single client model)
            // Note: New client replaces any existing UDP registration
            DhruvaUdpPublisher::register_client(&udp_registry, addr);

            clients.push(Client::new(stream, addr));
        }

        // --- Process incoming commands from TCP clients ---
        for client in clients.iter_mut() {
            if let Some(cmd) = client.try_read_command() {
                let response = process_command(
                    &cmd,
                    &shared_state,
                    &slam_command_tx,
                    &nav_command_tx,
                    &map_manager,
                );
                client.send_response(&response);
            }
        }

        let now = Instant::now();
        let timestamp_us = now_us();

        // Read shared state once per loop
        let (
            robot_status,
            sensor_status,
            current_map,
            map_list,
            map_list_dirty,
            navigation_state,
            mapping_progress,
        ) = {
            match shared_state.read() {
                Ok(state) => (
                    state.robot_status.clone(),
                    state.sensor_status.clone(),
                    state.current_map.clone(),
                    state.map_list.clone(),
                    state.map_list_dirty,
                    state.navigation_state.clone(),
                    state.mapping_progress.clone(),
                ),
                Err(_) => {
                    thread::sleep(Duration::from_millis(10));
                    continue;
                }
            }
        };

        // Clear dirty flag if it was set
        if map_list_dirty
            && !last_map_list_dirty
            && let Ok(mut state) = shared_state.write()
        {
            state.map_list_dirty = false;
        }
        last_map_list_dirty = map_list_dirty;

        // --- UDP: High-frequency streams ---

        // Publish robot status via UDP at configured rate
        if now.duration_since(last_robot) >= robot_interval {
            udp_publisher.publish_robot_status(&robot_status, timestamp_us);
            last_robot = now;
        }

        // Publish sensor status via UDP at configured rate
        if now.duration_since(last_sensor) >= sensor_interval {
            udp_publisher.publish_sensor_status(&sensor_status, timestamp_us);
            last_sensor = now;
        }

        // Publish navigation status via UDP at configured rate
        if now.duration_since(last_navigation) >= navigation_interval {
            udp_publisher.publish_navigation_status(
                &navigation_state,
                &robot_status.pose,
                timestamp_us,
            );
            last_navigation = now;
        }

        // Publish mapping progress via UDP at 2Hz
        if now.duration_since(last_mapping) >= mapping_interval {
            udp_publisher.publish_mapping_progress(&mapping_progress, timestamp_us);
            last_mapping = now;
        }

        // --- TCP: Large/critical data ---

        // Publish map via TCP at configured rate
        if now.duration_since(last_map) >= map_interval {
            if let Some(ref map_data) = current_map {
                let msg = build_current_map_message(map_data, timestamp_us);
                publish_to_clients(&mut clients, &msg);
                log::debug!(
                    "Published map via TCP: {}x{} @ {:.3}m, origin=({:.2}, {:.2})",
                    map_data.width,
                    map_data.height,
                    map_data.resolution,
                    map_data.origin_x,
                    map_data.origin_y
                );
            } else {
                log::debug!("No map data to publish");
            }
            last_map = now;
        }

        // Publish map list via TCP on change
        if map_list_dirty {
            let msg = build_map_list_message(&map_list, timestamp_us);
            publish_to_clients(&mut clients, &msg);
        }

        // Remove disconnected/failed clients and unregister from UDP
        let client_count_before = clients.len();
        let failed_clients: Vec<_> = clients
            .iter()
            .filter(|c| c.failed)
            .map(|c| c.socket_addr)
            .collect();

        clients.retain(|c| !c.failed);

        if clients.len() < client_count_before {
            // Unregister UDP for disconnected clients
            for _addr in &failed_clients {
                DhruvaUdpPublisher::unregister_client(&udp_registry);
            }
            log::info!(
                "Removed {} disconnected TCP clients, {} remaining (UDP unregistered)",
                client_count_before - clients.len(),
                clients.len()
            );
        }

        // Small sleep to prevent busy-waiting
        thread::sleep(Duration::from_millis(5));
    }

    log::info!("Publisher thread shutting down");
}

/// Publish message to all clients.
fn publish_to_clients(clients: &mut [Client], msg: &crate::io::streaming::DhruvaStream) {
    if clients.is_empty() {
        return;
    }

    // Encode message once
    let payload = msg.encode_to_vec();

    // Build length-prefixed frame
    let len = payload.len() as u32;
    let mut frame = Vec::with_capacity(4 + payload.len());
    frame.extend_from_slice(&len.to_be_bytes());
    frame.extend_from_slice(&payload);

    // Send to all clients
    for client in clients.iter_mut() {
        client.try_write(&frame);
    }
}

/// Build CurrentMap stream message.
fn build_current_map_message(
    map_data: &crate::state::CurrentMapData,
    timestamp_us: u64,
) -> crate::io::streaming::DhruvaStream {
    use crate::io::streaming::{CurrentMap, dhruva_stream};

    crate::io::streaming::DhruvaStream {
        timestamp_us,
        data: Some(dhruva_stream::Data::CurrentMap(CurrentMap {
            map_id: map_data.map_id.clone(),
            name: map_data.name.clone(),
            resolution: map_data.resolution,
            width: map_data.width,
            height: map_data.height,
            origin_x: map_data.origin_x,
            origin_y: map_data.origin_y,
            cells: map_data.cells.clone(),
            explored_area_m2: 0.0,
        })),
    }
}

/// Build MapList stream message.
fn build_map_list_message(
    map_list: &crate::state::MapList,
    timestamp_us: u64,
) -> crate::io::streaming::DhruvaStream {
    use crate::io::streaming::{MapList, MapSummary, dhruva_stream};

    let maps: Vec<MapSummary> = map_list
        .maps
        .iter()
        .map(|m| MapSummary {
            map_id: m.map_id.clone(),
            name: m.name.clone(),
            created_at_us: m.created_at_us,
            updated_at_us: m.updated_at_us,
            area_m2: m.area_m2,
            room_count: m.room_count,
            is_complete: m.is_complete,
        })
        .collect();

    crate::io::streaming::DhruvaStream {
        timestamp_us,
        data: Some(dhruva_stream::Data::MapList(MapList {
            maps,
            active_map_id: map_list.active_map_id.clone().unwrap_or_default(),
        })),
    }
}

/// Get current timestamp in microseconds.
fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

// ============================================================================
// Command Processing (merged from command_thread.rs)
// ============================================================================

/// Process a command and return response.
fn process_command(
    cmd: &crate::io::streaming::DhruvaCommand,
    shared_state: &SharedStateHandle,
    slam_command_tx: &CommandSender,
    nav_command_tx: &Option<NavCommandSender>,
    map_manager: &Arc<Mutex<MapManager>>,
) -> crate::io::streaming::DhruvaResponse {
    use crate::io::streaming::dhruva_command::Command;
    use crate::threads::NavCommand;

    let request_id = cmd.request_id.clone();

    // Log which command type was received
    let cmd_type = match &cmd.command {
        Some(Command::StartMapping(_)) => "StartMapping",
        Some(Command::StopMapping(_)) => "StopMapping",
        Some(Command::ClearMap(_)) => "ClearMap",
        Some(Command::RenameMap(_)) => "RenameMap",
        Some(Command::DeleteMap(_)) => "DeleteMap",
        Some(Command::EnableMap(_)) => "EnableMap",
        Some(Command::EmergencyStop(_)) => "EmergencyStop",
        Some(Command::SetGoal(_)) => "SetGoal",
        Some(Command::CancelGoal(_)) => "CancelGoal",
        None => "None",
    };
    log::info!("Received command: {} (request_id={})", cmd_type, request_id);

    let result: Result<Option<crate::io::streaming::dhruva_response::Data>, String> = match &cmd
        .command
    {
        Some(Command::StartMapping(c)) => {
            // Generate map ID
            let map_id = format!("map_{}", chrono_timestamp());
            let name = if c.map_name.is_empty() {
                "Untitled".to_string()
            } else {
                c.map_name.clone()
            };

            // Send to SLAM thread
            match send_command_sync(
                slam_command_tx,
                SlamCommand::StartMapping {
                    map_id: map_id.clone(),
                    name,
                },
                5000,
            ) {
                Ok(CommandResponse::MappingStarted { map_id }) => {
                    // Note: Don't notify navigation thread - let exploration thread handle autonomous mapping
                    // MappingFeature is designed for dock escape which interferes with simple exploration
                    Ok(Some(
                        crate::io::streaming::dhruva_response::Data::MappingStarted(
                            crate::io::streaming::MappingStartedResponse { map_id },
                        ),
                    ))
                }
                Ok(_) => Ok(None),
                Err(e) => Err(e),
            }
        }

        Some(Command::StopMapping(c)) => {
            // Also notify navigation thread to stop MappingFeature
            if let Some(tx) = nav_command_tx
                && let Err(e) = tx.send(NavCommand::StopMapping)
            {
                log::warn!("Failed to notify navigation of StopMapping: {}", e);
            }

            // Send to SLAM thread
            match send_command_sync(slam_command_tx, SlamCommand::StopMapping, 5000) {
                Ok(CommandResponse::MappingStopped) => {
                    // If save requested, save to map manager
                    if c.save
                        && let Ok(state) = shared_state.read()
                        && let Some(ref map_data) = state.current_map
                        && let Ok(mut mgr) = map_manager.lock()
                        && let Err(e) = mgr.save_current_map(
                            &state.current_map_id,
                            &state.current_map_name,
                            map_data,
                        )
                    {
                        log::error!("Failed to save map: {}", e);
                    }

                    Ok(Some(
                        crate::io::streaming::dhruva_response::Data::MappingStopped(
                            crate::io::streaming::MappingStoppedResponse {
                                saved: c.save,
                                area_m2: 0.0,
                            },
                        ),
                    ))
                }
                Ok(_) => Ok(None),
                Err(e) => Err(e),
            }
        }

        Some(Command::ClearMap(_)) => {
            match send_command_sync(slam_command_tx, SlamCommand::ClearMap, 5000) {
                Ok(_) => Ok(None),
                Err(e) => Err(e),
            }
        }

        Some(Command::RenameMap(c)) => {
            // Handle directly (no SLAM thread involvement)
            let mgr_result = map_manager.lock();
            match mgr_result {
                Ok(mut mgr) => match mgr.rename_map(&c.map_id, &c.new_name) {
                    Ok(()) => {
                        drop(mgr);
                        update_map_list(shared_state, map_manager);
                        Ok(None)
                    }
                    Err(e) => Err(e),
                },
                Err(_) => Err("Lock error".to_string()),
            }
        }

        Some(Command::DeleteMap(c)) => {
            // Handle directly (no SLAM thread involvement)
            let mgr_result = map_manager.lock();
            match mgr_result {
                Ok(mut mgr) => match mgr.delete_map(&c.map_id) {
                    Ok(()) => {
                        drop(mgr);
                        update_map_list(shared_state, map_manager);
                        Ok(None)
                    }
                    Err(e) => Err(e),
                },
                Err(_) => Err("Lock error".to_string()),
            }
        }

        Some(Command::EnableMap(c)) => {
            // Load map from storage
            let load_result = {
                let mgr_result = map_manager.lock();
                match mgr_result {
                    Ok(mgr) => match mgr.load_map(&c.map_id) {
                        Ok(map) => {
                            let name = mgr
                                .get_map_info(&c.map_id)
                                .map(|info| info.name.clone())
                                .unwrap_or_else(|| "Unknown".to_string());
                            Ok((map, name))
                        }
                        Err(e) => Err(e),
                    },
                    Err(_) => Err("Lock error".to_string()),
                }
            };

            match load_result {
                Ok((map, name)) => {
                    // Send to SLAM thread
                    match send_command_sync(
                        slam_command_tx,
                        SlamCommand::EnableMap {
                            map_id: c.map_id.clone(),
                            map,
                            name,
                        },
                        5000,
                    ) {
                        Ok(_) => Ok(None),
                        Err(e) => Err(e),
                    }
                }
                Err(e) => Err(e),
            }
        }

        Some(Command::EmergencyStop(_)) => {
            // Send emergency stop to SLAM thread
            match send_command_sync(slam_command_tx, SlamCommand::EmergencyStop, 5000) {
                Ok(CommandResponse::EmergencyStopped) => Ok(Some(
                    crate::io::streaming::dhruva_response::Data::EmergencyStop(
                        crate::io::streaming::EmergencyStopResponse {},
                    ),
                )),
                Ok(_) => Ok(None),
                Err(e) => Err(e),
            }
        }

        Some(Command::SetGoal(c)) => {
            // Send to navigation thread
            match nav_command_tx {
                Some(tx) => {
                    // Create target and get its ID
                    let target = NavTarget::user_waypoint(c.x, c.y, c.theta, c.description.clone());
                    let target_id = target.id;

                    // Send command to navigation thread
                    let nav_cmd = NavCommand::SetGoal {
                        x: c.x,
                        y: c.y,
                        theta: c.theta,
                        description: c.description.clone(),
                    };

                    match tx.send(nav_cmd) {
                        Ok(()) => {
                            log::info!(
                                "SetGoal sent to navigation thread: target_id={}",
                                target_id
                            );
                            Ok(Some(
                                crate::io::streaming::dhruva_response::Data::GoalAccepted(
                                    crate::io::streaming::GoalAcceptedResponse { target_id },
                                ),
                            ))
                        }
                        Err(e) => Err(format!("Failed to send goal to navigation thread: {}", e)),
                    }
                }
                None => Err("Navigation thread not available".to_string()),
            }
        }

        Some(Command::CancelGoal(_)) => {
            // Send to navigation thread
            match nav_command_tx {
                Some(tx) => match tx.send(NavCommand::CancelGoal) {
                    Ok(()) => {
                        log::info!("CancelGoal sent to navigation thread");
                        Ok(Some(
                            crate::io::streaming::dhruva_response::Data::GoalCancelled(
                                crate::io::streaming::GoalCancelledResponse {},
                            ),
                        ))
                    }
                    Err(e) => Err(format!("Failed to send cancel to navigation thread: {}", e)),
                },
                None => Err("Navigation thread not available".to_string()),
            }
        }

        None => Err("Empty command".to_string()),
    };

    match result {
        Ok(data) => crate::io::streaming::DhruvaResponse {
            request_id,
            success: true,
            error_message: String::new(),
            data,
        },
        Err(e) => crate::io::streaming::DhruvaResponse {
            request_id,
            success: false,
            error_message: e,
            data: None,
        },
    }
}

/// Update shared state with current map list.
fn update_map_list(shared_state: &SharedStateHandle, map_manager: &Arc<Mutex<MapManager>>) {
    if let Ok(mgr) = map_manager.lock()
        && let Ok(mut state) = shared_state.write()
    {
        state.map_list = mgr.as_map_list();
        state.map_list_dirty = true;
    }
}

/// Generate a simple timestamp string for map IDs.
fn chrono_timestamp() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}
