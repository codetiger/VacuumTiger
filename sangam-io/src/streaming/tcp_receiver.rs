//! TCP receiver thread - receives commands from client

use crate::core::driver::DeviceDriver;
use crate::core::types::Command;
use crate::error::{Error, Result};
use crate::streaming::messages::{Message, Payload};
use crate::streaming::wire::Serializer;
use std::io::Read;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// TCP receiver that handles commands from connected client
pub struct TcpReceiver {
    serializer: Serializer,
    driver: Arc<Mutex<Box<dyn DeviceDriver>>>,
    shutdown: Arc<AtomicBool>,
}

impl TcpReceiver {
    /// Create a new TCP receiver
    pub fn new(
        serializer: Serializer,
        driver: Arc<Mutex<Box<dyn DeviceDriver>>>,
        shutdown: Arc<AtomicBool>,
    ) -> Self {
        Self {
            serializer,
            driver,
            shutdown,
        }
    }

    /// Run the receiver loop for a connected client
    pub fn run(&self, mut stream: TcpStream) -> Result<()> {
        log::info!("TCP receiver started for client: {:?}", stream.peer_addr());

        // Set read timeout so we can check shutdown flag
        stream
            .set_read_timeout(Some(std::time::Duration::from_millis(100)))
            .ok();

        while self.shutdown.load(Ordering::Relaxed) {
            match self.read_message(&mut stream) {
                Ok(Some(msg)) => {
                    if let Err(e) = self.handle_message(msg) {
                        log::error!("Failed to handle message: {}", e);
                    }
                }
                Ok(None) => {
                    // Timeout, continue loop
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

    /// Read a message from the client
    fn read_message(&self, stream: &mut TcpStream) -> Result<Option<Message>> {
        // Read length prefix
        let mut len_buf = [0u8; 4];
        match stream.read_exact(&mut len_buf) {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(None),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => return Ok(None),
            Err(e) => return Err(Error::Io(e)),
        }

        let len = u32::from_be_bytes(len_buf) as usize;

        // Sanity check on length
        if len > 1024 * 1024 {
            return Err(Error::Other(format!("Message too large: {} bytes", len)));
        }

        // Read payload
        let mut buf = vec![0u8; len];
        stream.read_exact(&mut buf)?;

        // Deserialize
        let msg: Message = self.serializer.deserialize(&buf)?;
        Ok(Some(msg))
    }

    /// Handle a received message
    fn handle_message(&self, msg: Message) -> Result<()> {
        match msg.payload {
            Payload::Command { command } => {
                log::debug!("Received command: {:?}", command);
                self.handle_command(command)
            }
            Payload::SensorGroup { .. } => {
                log::warn!("Received unexpected SensorGroup message from client");
                Ok(())
            }
        }
    }

    /// Handle a command
    fn handle_command(&self, cmd: Command) -> Result<()> {
        let mut driver = self.driver.lock().map_err(|_| Error::ThreadPanic)?;
        driver.send_command(cmd)
    }
}
