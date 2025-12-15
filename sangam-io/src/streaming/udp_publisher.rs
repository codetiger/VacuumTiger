//! UDP publisher thread - streams sensor data to registered clients
//!
//! Supports two modes for different data rates:
//! - **Streaming mode**: For high-rate sensors (GD32 @ 110Hz), consumes from channel
//! - **Polling mode**: For low-rate sensors (lidar @ 5Hz), polls shared mutex
//!
//! Uses UDP unicast to registered clients only (not broadcast).
//! Clients are registered when they connect via TCP for commands.

use crate::core::types::{SensorGroupData, StreamReceiver};
use crate::error::Result;
use crate::streaming::wire::Serializer;
use crossbeam_channel::TryRecvError;
use std::collections::HashMap;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Type alias for UDP client registry (single client at a time)
pub type UdpClientRegistry = Arc<Mutex<Option<SocketAddr>>>;

/// UDP publisher that streams sensor data to registered clients
pub struct UdpPublisher {
    socket: UdpSocket,
    serializer: Serializer,
    /// Mutex-based sensor data for low-rate polling (lidar, version)
    sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>,
    /// Channel-based receivers for high-rate streaming (sensor_status @ 110Hz)
    stream_receivers: HashMap<String, StreamReceiver>,
    /// Global running flag (daemon shutdown)
    running: Arc<AtomicBool>,
    /// Client registry - current registered client for UDP streaming
    client_registry: UdpClientRegistry,
}

impl UdpPublisher {
    /// Create a new UDP publisher
    pub fn new(
        socket: UdpSocket,
        serializer: Serializer,
        sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>,
        stream_receivers: HashMap<String, StreamReceiver>,
        running: Arc<AtomicBool>,
        client_registry: UdpClientRegistry,
    ) -> Self {
        Self {
            socket,
            serializer,
            sensor_data,
            stream_receivers,
            running,
            client_registry,
        }
    }

    /// Get current registered client address (if any)
    fn get_client(&self) -> Option<SocketAddr> {
        *self.client_registry
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    /// Run the publisher loop (unicast to registered client)
    pub fn run(&self) -> Result<()> {
        log::info!("UDP publisher started (unicast mode - waiting for client registration)");

        // Track last sent sequence numbers for polling-based groups
        // (excluding groups that have streaming channels)
        let mut last_seq: HashMap<String, u64> = HashMap::new();
        for group_id in self.sensor_data.keys() {
            if !self.stream_receivers.contains_key(group_id) {
                last_seq.insert(group_id.clone(), 0);
            }
        }

        let mut last_client: Option<SocketAddr> = None;

        while self.running.load(Ordering::Relaxed) {
            // Get current registered client
            let client_addr = self.get_client();

            // Log client registration changes
            if client_addr != last_client {
                match &client_addr {
                    Some(addr) => log::info!("UDP streaming to client: {}", addr),
                    None => log::info!("UDP streaming paused (no client registered)"),
                }
                last_client = client_addr;
            }

            // Skip sending if no client registered
            let Some(target_addr) = client_addr else {
                // No client - sleep longer to reduce CPU usage
                std::thread::sleep(Duration::from_millis(10));
                continue;
            };

            let mut sent_any = false;

            // Phase 1: Drain all streaming channels (high-rate sensors like GD32 @ 110Hz)
            for (group_id, rx) in &self.stream_receivers {
                loop {
                    match rx.try_recv() {
                        Ok(data) => {
                            if let Err(e) = self.send_sensor_group(&data, target_addr) {
                                // UDP send errors are not fatal - just log and continue
                                log::warn!("Failed to send {} stream data: {}", group_id, e);
                            } else {
                                log::trace!(
                                    "Sent {} (seq: {}, ts: {}) to {}",
                                    data.group_id,
                                    data.sequence_number,
                                    data.timestamp_us,
                                    target_addr
                                );
                            }
                            sent_any = true;
                        }
                        Err(TryRecvError::Empty) => break,
                        Err(TryRecvError::Disconnected) => {
                            log::warn!("Stream channel for {} disconnected", group_id);
                            break;
                        }
                    }
                }
            }

            // Phase 2: Poll mutex-based groups (low-rate sensors like lidar @ 5Hz)
            for (group_id, data_mutex) in &self.sensor_data {
                // Skip groups that have streaming channels
                if self.stream_receivers.contains_key(group_id) {
                    continue;
                }

                let data = match data_mutex.lock() {
                    Ok(d) => d,
                    Err(e) => {
                        log::error!("Failed to lock sensor data for {}: {}", group_id, e);
                        continue;
                    }
                };

                let last = last_seq.get(group_id).copied().unwrap_or(0);
                if data.sequence_number != last {
                    // Clone data while holding lock
                    let cloned = data.clone();
                    drop(data); // Release lock before serializing

                    if let Some(seq) = last_seq.get_mut(group_id) {
                        *seq = cloned.sequence_number;
                    }

                    if let Err(e) = self.send_sensor_group(&cloned, target_addr) {
                        // UDP send errors are not fatal - just log and continue
                        log::warn!("Failed to send {} poll data: {}", group_id, e);
                    } else {
                        log::trace!(
                            "Sent {} (seq: {}, ts: {}) to {} [polled]",
                            cloned.group_id,
                            cloned.sequence_number,
                            cloned.timestamp_us,
                            target_addr
                        );
                    }
                    sent_any = true;
                }
            }

            // Rate limiting: Always sleep briefly to yield CPU on single-core embedded systems.
            // At 110Hz sensor rate, we need ~9ms between messages. 500μs sleep is plenty of margin
            // while still allowing up to 2000 packets/sec if needed.
            if sent_any {
                std::thread::sleep(Duration::from_micros(500));
            } else {
                // No data - sleep longer to reduce CPU usage
                std::thread::sleep(Duration::from_millis(1));
            }
        }

        log::info!("UDP publisher stopped");
        Ok(())
    }

    /// Send a sensor group via UDP unicast to specific client
    ///
    /// Uses same wire format as TCP for client compatibility:
    /// [4-byte length prefix (big-endian)] + [protobuf payload]
    fn send_sensor_group(&self, data: &SensorGroupData, target: SocketAddr) -> Result<()> {
        let payload = self.serializer.serialize_sensor_group(data)?;
        let len = (payload.len() as u32).to_be_bytes();

        // Combine length prefix and payload into a single buffer
        // (same format as TCP for client compatibility)
        let mut buf = Vec::with_capacity(4 + payload.len());
        buf.extend_from_slice(&len);
        buf.extend_from_slice(&payload);

        self.socket.send_to(&buf, target)?;

        Ok(())
    }
}
