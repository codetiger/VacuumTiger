//! 3iRobotix Delta-2D Lidar driver

mod protocol;

use crate::drivers::LidarDriver;
use crate::error::Result;
use crate::transport::Transport;
use crate::types::LidarScan;

use parking_lot::Mutex;
use std::sync::Arc;

pub use protocol::CommandType;

/// 3iRobotix Delta-2D Lidar driver
///
/// Communicates via UART at 115200 baud. Power control is handled externally
/// via GD32 CMD=0x97 (Lidar Power command).
pub struct Delta2DDriver {
    /// Transport layer for serial I/O
    transport: Arc<Mutex<Box<dyn Transport>>>,
    /// Scanning state
    scanning: Arc<Mutex<bool>>,
}

impl Delta2DDriver {
    /// Create new Delta-2D lidar driver
    ///
    /// Note: The lidar motor must be powered on separately via GD32 driver
    /// using set_lidar_power(true).
    pub fn new<T: Transport + 'static>(transport: T) -> Result<Self> {
        let transport = Arc::new(Mutex::new(Box::new(transport) as Box<dyn Transport>));
        let scanning = Arc::new(Mutex::new(false));

        log::info!("Delta2D: Driver initialized");

        Ok(Delta2DDriver {
            transport,
            scanning,
        })
    }

    /// Try to read a scan packet
    ///
    /// Returns Ok(Some(scan)) if measurement data received,
    /// Ok(None) if health packet or no data available,
    /// Err(_) on read/parse error.
    fn try_read_scan(&self) -> Result<Option<LidarScan>> {
        let mut transport = self.transport.lock();

        // Check if enough data available for header
        let available = transport.available()?;
        if available < 8 {
            return Ok(None);
        }

        // Read header
        let mut header_buf = [0u8; 8];
        let read = transport.read(&mut header_buf)?;
        if read < 8 {
            return Ok(None);
        }

        let header = protocol::PacketHeader::parse(&header_buf)?;

        // Release transport lock before calling read_exact
        drop(transport);

        // Read payload
        let mut payload = vec![0u8; header.payload_length as usize];
        self.read_exact(&mut payload)?;

        // Read CRC (currently not validated)
        let mut crc_buf = [0u8; 2];
        self.read_exact(&mut crc_buf)?;
        let _payload_crc = u16::from_be_bytes(crc_buf);

        match header.command_type {
            CommandType::Measurement => {
                let scan = protocol::parse_measurement(&payload)?;
                Ok(Some(scan))
            }
            CommandType::Health => {
                // Health packets don't contain scan data
                log::debug!("Delta2D: Received health packet");
                Ok(None)
            }
        }
    }

    /// Read exact number of bytes from transport
    fn read_exact(&self, buf: &mut [u8]) -> Result<()> {
        let mut transport = self.transport.lock();
        let mut offset = 0;

        while offset < buf.len() {
            let read = transport.read(&mut buf[offset..])?;
            if read == 0 {
                return Err(crate::error::Error::Io(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "Unexpected EOF while reading",
                )));
            }
            offset += read;
        }

        Ok(())
    }
}

impl LidarDriver for Delta2DDriver {
    fn start(&mut self) -> Result<()> {
        log::info!("Delta2D: Starting scan");
        *self.scanning.lock() = true;
        Ok(())
    }

    fn get_scan(&mut self) -> Result<Option<LidarScan>> {
        if !*self.scanning.lock() {
            return Ok(None);
        }

        self.try_read_scan()
    }

    fn stop(&mut self) -> Result<()> {
        log::info!("Delta2D: Stopping scan");
        *self.scanning.lock() = false;
        Ok(())
    }

    fn is_scanning(&self) -> bool {
        *self.scanning.lock()
    }
}
