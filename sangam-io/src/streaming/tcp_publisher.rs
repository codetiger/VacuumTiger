//! Telemetry and lidar data publisher using TCP sockets.
//!
//! Publishes sensor data and lidar scans to external consumers via TCP.
//!
//! Uses a lock-free queue architecture for sub-2ms hardware timing guarantees.
//! A dedicated publisher thread owns the TCP listener, and hardware threads push to queue.

use crate::error::Result;
use crate::streaming::messages::{LidarScan, TelemetryMessage};
use crossbeam_queue::ArrayQueue;
use log::{debug, error, info, warn};
use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// Telemetry publisher that publishes sensor data and lidar scans via TCP
///
/// Uses a lock-free queue for non-blocking hardware thread communication.
/// Hardware threads use try_push() which returns immediately, never blocking.
pub struct TcpPublisher {
    telemetry_queue: Arc<ArrayQueue<TelemetryMessage>>,
    lidar_queue: Arc<ArrayQueue<LidarScan>>,
    publisher_thread: Option<JoinHandle<()>>,
    shutdown: Arc<AtomicBool>,
}

impl TcpPublisher {
    /// Create a new TCP streaming publisher
    ///
    /// Spawns a dedicated publisher thread that owns the TCP listener.
    /// Hardware threads push data to lock-free queues using try_push().
    ///
    /// # Arguments
    /// - `bind_address`: TCP bind address (e.g., "0.0.0.0:5555")
    ///
    /// # Returns
    /// New TcpPublisher instance with lock-free queues
    pub fn new(bind_address: String) -> Result<Self> {
        // Create bounded lock-free queues
        // 1000 messages = ~2 seconds buffer at 500Hz (telemetry)
        // 100 scans = ~10 seconds buffer at 10Hz (lidar)
        let telemetry_queue = Arc::new(ArrayQueue::new(1000));
        let lidar_queue = Arc::new(ArrayQueue::new(100));

        let shutdown = Arc::new(AtomicBool::new(false));
        let shutdown_clone = Arc::clone(&shutdown);
        let bind_address_clone = bind_address.clone();
        let telemetry_queue_clone = Arc::clone(&telemetry_queue);
        let lidar_queue_clone = Arc::clone(&lidar_queue);

        // Spawn dedicated publisher thread
        let publisher_thread = thread::Builder::new()
            .name("tcp-publisher".to_string())
            .spawn(move || {
                if let Err(e) = Self::publisher_thread_loop(
                    &bind_address_clone,
                    telemetry_queue_clone,
                    lidar_queue_clone,
                    shutdown_clone,
                ) {
                    error!("Publisher thread error: {}", e);
                }
            })?;

        info!("TCP streaming publisher started on {}", bind_address);

        Ok(Self {
            telemetry_queue,
            lidar_queue,
            publisher_thread: Some(publisher_thread),
            shutdown,
        })
    }

    /// Publisher thread main loop - owns the TCP listener
    ///
    /// Uses lock-free queues to receive data from hardware threads without blocking.
    /// Batch processes messages for efficiency (up to 50 per iteration).
    fn publisher_thread_loop(
        bind_address: &str,
        telemetry_queue: Arc<ArrayQueue<TelemetryMessage>>,
        lidar_queue: Arc<ArrayQueue<LidarScan>>,
        shutdown: Arc<AtomicBool>,
    ) -> Result<()> {
        // Create TCP listener
        let listener = TcpListener::bind(bind_address)?;
        listener.set_nonblocking(true)?;

        info!("Publisher thread: TCP listener bound to {}", bind_address);

        let mut clients: Vec<TcpStream> = Vec::new();
        let mut telemetry_count = 0u64;
        let mut lidar_count = 0u64;
        let mut queue_full_warnings = 0u64;

        // Reusable buffer for message serialization (avoids allocations)
        let mut message_buffer = Vec::with_capacity(4096);

        // Process messages from queues
        while !shutdown.load(Ordering::Relaxed) {
            // Accept new client connections (non-blocking)
            match listener.accept() {
                Ok((stream, addr)) => {
                    if let Err(e) = stream.set_nonblocking(false) {
                        warn!("Failed to set blocking mode for client {}: {}", addr, e);
                    } else {
                        info!("New client connected: {}", addr);
                        clients.push(stream);
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No new connections, continue
                }
                Err(e) => {
                    error!("Error accepting client connection: {}", e);
                }
            }

            // Batch process telemetry messages (up to 50 at a time)
            let mut telemetry_batch = 0;
            while let Some(message) = telemetry_queue.pop() {
                if let Err(e) = Self::broadcast_to_clients(
                    &mut clients,
                    "telemetry",
                    &message,
                    &mut message_buffer,
                ) {
                    debug!("Failed to publish telemetry: {}", e);
                } else {
                    telemetry_count += 1;
                }

                telemetry_batch += 1;
                if telemetry_batch >= 50 {
                    break; // Limit batch size to prevent starvation
                }
            }

            // Batch process lidar scans (up to 10 at a time)
            let mut lidar_batch = 0;
            while let Some(scan) = lidar_queue.pop() {
                if let Err(e) =
                    Self::broadcast_to_clients(&mut clients, "lidar", &scan, &mut message_buffer)
                {
                    debug!("Failed to publish lidar scan: {}", e);
                } else {
                    lidar_count += 1;
                    if lidar_count % 100 == 0 {
                        debug!("Published {} lidar scans", lidar_count);
                    }
                }

                lidar_batch += 1;
                if lidar_batch >= 10 {
                    break; // Limit batch size
                }
            }

            // Monitor queue health
            let telemetry_len = telemetry_queue.len();
            let telemetry_cap = telemetry_queue.capacity();
            if telemetry_len > (telemetry_cap * 8 / 10) {
                queue_full_warnings += 1;
                if queue_full_warnings % 100 == 0 {
                    warn!(
                        "Telemetry queue near full: {}/{} ({:.1}%)",
                        telemetry_len,
                        telemetry_cap,
                        (telemetry_len as f32 / telemetry_cap as f32) * 100.0
                    );
                }
            }

            // Sleep briefly if queues are empty (reduce CPU usage)
            if telemetry_queue.is_empty() && lidar_queue.is_empty() {
                thread::sleep(Duration::from_millis(10));
            }
        }

        info!(
            "Publisher thread exiting ({} telemetry, {} lidar published)",
            telemetry_count, lidar_count
        );
        Ok(())
    }

    /// Broadcast a message to all connected TCP clients
    ///
    /// Message format: [4-byte length (big-endian)][topic (null-terminated)][MessagePack payload]
    fn broadcast_to_clients<T: serde::Serialize>(
        clients: &mut Vec<TcpStream>,
        topic: &str,
        message: &T,
        buffer: &mut Vec<u8>,
    ) -> Result<()> {
        // Serialize message
        let payload = rmp_serde::to_vec(message)?;

        // Reuse buffer for creating the message
        buffer.clear();
        buffer.reserve(4 + topic.len() + 1 + payload.len());

        // Calculate total length (topic + null + payload)
        let frame_length = (topic.len() + 1 + payload.len()) as u32;

        // Write length prefix (4 bytes big-endian)
        buffer.extend_from_slice(&frame_length.to_be_bytes());

        // Write topic
        buffer.extend_from_slice(topic.as_bytes());
        buffer.push(0); // Null terminator for topic

        // Write payload
        buffer.extend_from_slice(&payload);

        // Send to all clients, removing disconnected ones
        clients.retain_mut(|client| match client.write_all(buffer) {
            Ok(_) => true,
            Err(e) => {
                if let Ok(addr) = client.peer_addr() {
                    debug!("Client {} disconnected: {}", addr, e);
                }
                false
            }
        });

        Ok(())
    }

    /// Get the telemetry queue for direct access by hardware threads
    ///
    /// Hardware threads should use `queue.push()` to send telemetry.
    /// This is a non-blocking operation that returns immediately.
    ///
    /// # Returns
    /// Arc reference to the lock-free telemetry queue
    pub fn get_telemetry_queue(&self) -> Arc<ArrayQueue<TelemetryMessage>> {
        Arc::clone(&self.telemetry_queue)
    }

    /// Get the lidar queue for direct access by hardware threads
    ///
    /// Hardware threads should use `queue.push()` to send lidar scans.
    /// This is a non-blocking operation that returns immediately.
    ///
    /// # Returns
    /// Arc reference to the lock-free lidar queue
    pub fn get_lidar_queue(&self) -> Arc<ArrayQueue<LidarScan>> {
        Arc::clone(&self.lidar_queue)
    }

    /// Stop the publisher
    pub fn stop(&self) {
        self.shutdown.store(true, Ordering::SeqCst);
        info!("TCP streaming publisher shutdown requested");
    }
}

impl Drop for TcpPublisher {
    fn drop(&mut self) {
        self.stop();

        // Wait for publisher thread to finish
        if let Some(thread) = self.publisher_thread.take() {
            let _ = thread.join();
        }
    }
}
