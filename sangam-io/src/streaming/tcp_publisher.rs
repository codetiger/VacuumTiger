//! TCP publisher thread - streams sensor data to client

use crate::core::types::SensorGroupData;
use crate::error::Result;
use crate::streaming::messages::Message;
use crate::streaming::wire::Serializer;
use std::collections::HashMap;
use std::io::Write;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// TCP publisher that streams sensor data to connected client
pub struct TcpPublisher {
    serializer: Serializer,
    sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>,
    running: Arc<AtomicBool>,
}

impl TcpPublisher {
    /// Create a new TCP publisher
    pub fn new(
        serializer: Serializer,
        sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>,
        running: Arc<AtomicBool>,
    ) -> Self {
        Self {
            serializer,
            sensor_data,
            running,
        }
    }

    /// Run the publisher loop for a connected client
    pub fn run(&self, mut stream: TcpStream) -> Result<()> {
        log::info!("TCP publisher started for client: {:?}", stream.peer_addr());

        // Set TCP_NODELAY for low latency
        stream.set_nodelay(true).ok();

        // Set write timeout to avoid blocking forever
        stream.set_write_timeout(Some(Duration::from_secs(5))).ok();

        // Track last sent timestamps for each group
        let mut last_sent: HashMap<String, u64> = HashMap::new();
        for group_id in self.sensor_data.keys() {
            last_sent.insert(group_id.clone(), 0);
        }

        while self.running.load(Ordering::Relaxed) {
            let mut sent_any = false;

            for (group_id, data) in &self.sensor_data {
                let data = match data.lock() {
                    Ok(d) => d,
                    Err(e) => {
                        log::error!("Failed to lock sensor data for {}: {}", group_id, e);
                        continue;
                    }
                };

                let last = last_sent.get(group_id).copied().unwrap_or(0);
                if data.timestamp_us > last {
                    // Create and send message
                    let msg = Message::sensor_group(&data);
                    if let Err(e) = self.send_message(&mut stream, &msg) {
                        log::error!("Failed to send sensor data: {}", e);
                        return Err(e);
                    }

                    log::trace!("Sent {} (ts: {} -> {})", group_id, last, data.timestamp_us);
                    last_sent.insert(group_id.clone(), data.timestamp_us);
                    sent_any = true;
                } else {
                    log::trace!("No new data for {} (ts: {} <= {})", group_id, data.timestamp_us, last);
                }
            }

            // Small sleep if nothing sent to prevent busy loop
            if !sent_any {
                std::thread::sleep(Duration::from_millis(1));
            }
        }

        log::info!("TCP publisher stopped");
        Ok(())
    }

    /// Send a message to the client
    fn send_message(&self, stream: &mut TcpStream, msg: &Message) -> Result<()> {
        let bytes = self.serializer.serialize(msg)?;
        let len = (bytes.len() as u32).to_be_bytes();

        stream.write_all(&len)?;
        stream.write_all(&bytes)?;

        Ok(())
    }
}
