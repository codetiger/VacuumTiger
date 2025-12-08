//! TCP publisher thread - streams sensor data to client
//!
//! Supports two modes for different data rates:
//! - **Polling mode**: For low-rate sensors (lidar @ 5Hz), polls shared mutex
//! - **Streaming mode**: For high-rate sensors (GD32 @ 110Hz), consumes from channel

use crate::core::types::{SensorGroupData, StreamReceiver};
use crate::error::Result;
use crate::streaming::wire::Serializer;
use crossbeam_channel::TryRecvError;
use std::collections::HashMap;
use std::io::Write;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// TCP publisher that streams sensor data to connected client
pub struct TcpPublisher {
    serializer: Serializer,
    /// Mutex-based sensor data for low-rate polling (lidar, version)
    sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>,
    /// Channel-based receivers for high-rate streaming (sensor_status @ 110Hz)
    stream_receivers: HashMap<String, StreamReceiver>,
    running: Arc<AtomicBool>,
}

impl TcpPublisher {
    /// Create a new TCP publisher
    pub fn new(
        serializer: Serializer,
        sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>,
        stream_receivers: HashMap<String, StreamReceiver>,
        running: Arc<AtomicBool>,
    ) -> Self {
        Self {
            serializer,
            sensor_data,
            stream_receivers,
            running,
        }
    }

    /// Run the publisher loop for a connected client
    pub fn run(&self, mut stream: TcpStream) -> Result<()> {
        log::info!("TCP publisher started for client: {:?}", stream.peer_addr());

        // Set TCP_NODELAY for low latency
        if let Err(e) = stream.set_nodelay(true) {
            log::warn!("Failed to set TCP_NODELAY: {}", e);
        }

        // Set write timeout to avoid blocking forever
        if let Err(e) = stream.set_write_timeout(Some(Duration::from_secs(5))) {
            log::warn!("Failed to set write timeout: {}", e);
        }

        // Track last sent sequence numbers for polling-based groups
        // (excluding groups that have streaming channels)
        let mut last_seq: HashMap<String, u64> = HashMap::new();
        for group_id in self.sensor_data.keys() {
            if !self.stream_receivers.contains_key(group_id) {
                last_seq.insert(group_id.clone(), 0);
            }
        }

        while self.running.load(Ordering::Relaxed) {
            let mut sent_any = false;

            // Phase 1: Drain all streaming channels (high-rate sensors like GD32 @ 110Hz)
            for (group_id, rx) in &self.stream_receivers {
                loop {
                    match rx.try_recv() {
                        Ok(data) => {
                            if let Err(e) = self.send_sensor_group(&mut stream, &data) {
                                log::error!("Failed to send {} stream data: {}", group_id, e);
                                return Err(e);
                            }
                            log::trace!(
                                "Sent {} (seq: {}, ts: {}) [streamed]",
                                data.group_id,
                                data.sequence_number,
                                data.timestamp_us
                            );
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

                    if let Err(e) = self.send_sensor_group(&mut stream, &cloned) {
                        log::error!("Failed to send {} poll data: {}", group_id, e);
                        return Err(e);
                    }
                    log::trace!(
                        "Sent {} (seq: {}, ts: {}) [polled]",
                        cloned.group_id,
                        cloned.sequence_number,
                        cloned.timestamp_us
                    );
                    sent_any = true;
                }
            }

            // Brief sleep if no data to prevent busy loop
            if !sent_any {
                std::thread::sleep(Duration::from_micros(100));
            }
        }

        log::info!("TCP publisher stopped");
        Ok(())
    }

    /// Send a sensor group to the client
    fn send_sensor_group(&self, stream: &mut TcpStream, data: &SensorGroupData) -> Result<()> {
        let bytes = self.serializer.serialize_sensor_group(data)?;
        let len = (bytes.len() as u32).to_be_bytes();

        stream.write_all(&len)?;
        stream.write_all(&bytes)?;

        Ok(())
    }
}
