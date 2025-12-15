//! TCP command receiver for client control
//!
//! This module handles incoming commands from connected clients over TCP.
//! Commands include robot motion control, actuator control, and component
//! enable/disable operations.
//!
//! # Purpose
//!
//! TCP is used for commands (not UDP) because:
//!
//! - **Reliability**: Commands must not be lost (e.g., "stop motors")
//! - **Ordering**: Commands must execute in sequence
//! - **Acknowledgment**: Sender knows the command was received
//! - **State sync**: TCP connection state tracks client presence
//!
//! # Wire Format
//!
//! Commands use length-prefixed Protobuf encoding:
//!
//! ```text
//! ┌──────────────────┬─────────────────────┐
//! │ Length (4 bytes) │ Protobuf Command    │
//! │ Big-endian u32   │ (variable size)     │
//! └──────────────────┴─────────────────────┘
//! ```
//!
//! # Command Types
//!
//! | Command | Description |
//! |---------|-------------|
//! | `ComponentControl::drive` | Set linear/angular velocity |
//! | `ComponentControl::lidar` | Enable/disable lidar motor |
//! | `ComponentControl::vacuum` | Control vacuum motor speed |
//! | `ComponentControl::main_brush` | Control main brush speed |
//! | `ComponentControl::side_brush` | Control side brush speed |
//! | `ComponentControl::water_pump` | Control water pump speed |
//! | `ComponentControl::led` | Set LED state |
//! | `Shutdown` | Graceful daemon shutdown |
//!
//! # Connection Lifecycle
//!
//! ```text
//! 1. Client connects to TCP port 5555
//! 2. Server spawns TcpReceiver thread for this client
//! 3. Client IP is registered for UDP streaming
//! 4. Receiver loop processes commands until disconnect
//! 5. On disconnect, UDP registration is cleared
//! ```
//!
//! # Safety Features
//!
//! - **Read timeout**: 500ms timeout allows periodic shutdown flag checks
//! - **Buffer limit**: Commands > 1MB are rejected (DoS protection)
//! - **Graceful shutdown**: Handles both global and per-connection flags

use crate::core::driver::DeviceDriver;
use crate::core::types::Command;
use crate::error::{Error, Result};
use crate::streaming::wire::Serializer;
use std::io::Read;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// TCP receiver that handles commands from connected client
pub struct TcpReceiver {
    serializer: Serializer,
    driver: Arc<Mutex<Box<dyn DeviceDriver>>>,
    /// Global running flag (daemon shutdown)
    running: Arc<AtomicBool>,
    /// Per-connection alive flag (connection health)
    conn_alive: Arc<AtomicBool>,
    /// Reusable buffer for reading command payloads (avoids allocation per command)
    read_buffer: Vec<u8>,
}

/// Initial capacity for command read buffer (typical command size)
const INITIAL_BUFFER_CAPACITY: usize = 256;

impl TcpReceiver {
    /// Create a new TCP receiver
    pub fn new(
        serializer: Serializer,
        driver: Arc<Mutex<Box<dyn DeviceDriver>>>,
        running: Arc<AtomicBool>,
        conn_alive: Arc<AtomicBool>,
    ) -> Self {
        Self {
            serializer,
            driver,
            running,
            conn_alive,
            // Pre-allocate buffer to avoid allocation on first command
            read_buffer: Vec::with_capacity(INITIAL_BUFFER_CAPACITY),
        }
    }

    /// Run the receiver loop for a connected client
    pub fn run(&mut self, mut stream: TcpStream) -> Result<()> {
        log::info!("TCP receiver started for client: {:?}", stream.peer_addr());

        // Set read timeout so we can check shutdown flag
        if let Err(e) = stream.set_read_timeout(Some(std::time::Duration::from_millis(500))) {
            log::warn!("Failed to set read timeout: {}", e);
        }

        log::debug!("Entering receiver loop");

        loop {
            // Check both global running flag and per-connection alive flag
            if !self.running.load(Ordering::Relaxed) {
                log::debug!("Running flag cleared, exiting");
                break;
            }
            if !self.conn_alive.load(Ordering::Relaxed) {
                log::debug!("Connection alive flag cleared, exiting");
                break;
            }

            match self.read_command(&mut stream) {
                Ok(Some(cmd)) => {
                    log::info!("Received command: {:?}", cmd);
                    if let Err(e) = self.handle_command(cmd) {
                        log::error!("Failed to handle command: {}", e);
                    }
                }
                Ok(None) => {
                    // Timeout or non-command message, continue loop
                }
                Err(e) => {
                    // Signal connection is dead and shutdown socket
                    self.conn_alive.store(false, Ordering::Relaxed);
                    let _ = stream.shutdown(std::net::Shutdown::Both);

                    // Check if it's a connection closed error
                    if let Error::Io(ref io_err) = e
                        && (io_err.kind() == std::io::ErrorKind::UnexpectedEof
                            || io_err.kind() == std::io::ErrorKind::ConnectionReset)
                    {
                        log::info!("Client disconnected");
                        return Ok(());
                    }
                    log::error!("Failed to read message: {}", e);
                    return Err(e);
                }
            }
        }

        // Clean shutdown: signal connection dead and close socket
        self.conn_alive.store(false, Ordering::Relaxed);
        let _ = stream.shutdown(std::net::Shutdown::Both);

        log::info!("TCP receiver stopped");
        Ok(())
    }

    /// Read a command from the client
    ///
    /// Uses a reusable internal buffer to avoid allocation per command.
    fn read_command(&mut self, stream: &mut TcpStream) -> Result<Option<Command>> {
        // Read length prefix
        let mut len_buf = [0u8; 4];
        match stream.read_exact(&mut len_buf) {
            Ok(_) => {
                log::debug!("Read length prefix: {:?}", len_buf);
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(None),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => return Ok(None),
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                log::debug!("EOF on length read");
                return Err(Error::Io(e));
            }
            Err(e) => {
                log::debug!("Error reading length: {:?}", e.kind());
                return Err(Error::Io(e));
            }
        }

        let len = u32::from_be_bytes(len_buf) as usize;

        // Sanity check on length
        if len > 1024 * 1024 {
            return Err(Error::Other(format!("Message too large: {} bytes", len)));
        }

        // Reuse buffer - resize only if needed (no allocation if capacity sufficient)
        self.read_buffer.clear();
        self.read_buffer.resize(len, 0);
        stream.read_exact(&mut self.read_buffer)?;

        // Deserialize command
        self.serializer.deserialize_command(&self.read_buffer)
    }

    /// Handle a command
    fn handle_command(&self, cmd: Command) -> Result<()> {
        log::debug!("Executing command: {:?}", cmd);
        let mut driver = self.driver.lock().map_err(|_| Error::ThreadPanic)?;
        let result = driver.send_command(cmd);
        if result.is_ok() {
            log::trace!("Command executed successfully");
        } else {
            log::error!("Command execution failed: {:?}", result);
        }
        result
    }
}
