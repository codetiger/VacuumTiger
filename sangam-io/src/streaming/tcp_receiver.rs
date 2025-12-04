//! TCP receiver thread - receives commands from client

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
    running: Arc<AtomicBool>,
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
    ) -> Self {
        Self {
            serializer,
            driver,
            running,
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
            if !self.running.load(Ordering::Relaxed) {
                log::debug!("Running flag cleared, exiting");
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
                    // Check if it's a connection closed error
                    if let Error::Io(ref io_err) = e {
                        if io_err.kind() == std::io::ErrorKind::UnexpectedEof
                            || io_err.kind() == std::io::ErrorKind::ConnectionReset
                        {
                            log::info!("Client disconnected");
                            return Ok(());
                        }
                    }
                    log::error!("Failed to read message: {}", e);
                    return Err(e);
                }
            }
        }

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
