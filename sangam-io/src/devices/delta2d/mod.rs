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

        // Log availability periodically (every ~100 calls = ~1 second)
        use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
        static CALL_COUNT: AtomicU32 = AtomicU32::new(0);
        static MAX_BYTES_SEEN: AtomicUsize = AtomicUsize::new(0);

        let count = CALL_COUNT.fetch_add(1, Ordering::Relaxed);
        let max_seen = MAX_BYTES_SEEN.load(Ordering::Relaxed);

        if available > max_seen {
            MAX_BYTES_SEEN.store(available, Ordering::Relaxed);
            log::info!("Delta2D: NEW MAX bytes available: {}", available);
        }

        if count % 200 == 0 {
            log::info!("Delta2D: Bytes available on ttyS1: {} (max seen: {})", available, max_seen);

            // Try to read and log small amounts to see what's being received
            if available > 0 && available < 8 {
                let mut peek_buf = vec![0u8; available.min(16)];
                if let Ok(read) = transport.read(&mut peek_buf) {
                    log::warn!("Delta2D: Insufficient data - Read {} bytes (for inspection): {:02X?}", read, &peek_buf[..read]);
                }
            }
        }

        // Python uses "if ser.in_waiting > 8" - need MORE than 8 bytes
        // This ensures we have header + some payload data ready
        if available <= 8 {
            return Ok(None);
        }

        // Read header
        let mut header_buf = [0u8; 8];
        let read = transport.read(&mut header_buf)?;

        log::debug!("Delta2D: RX header {} bytes: {:02X?}", read, header_buf);

        if read < 8 {
            log::warn!("Delta2D: Header read incomplete: got {} bytes, expected 8", read);
            return Ok(None);
        }

        let header = match protocol::PacketHeader::parse(&header_buf) {
            Ok(h) => h,
            Err(e) => {
                log::warn!("Delta2D: Failed to parse header: {} (bytes: {:02X?})", e, header_buf);
                return Err(e);
            }
        };

        log::debug!("Delta2D: Parsed header - CMD=0x{:02X}, payload_len={}",
                   header.command_type as u8, header.payload_length);

        // CRITICAL FIX: Wait for payload + CRC to arrive
        // Python does: while ser.in_waiting < payload_length + 2: pass
        let required_bytes = header.payload_length as usize + 2; // +2 for CRC
        let wait_start = std::time::Instant::now();
        let wait_timeout = std::time::Duration::from_millis(100);

        loop {
            let available = transport.available()?;
            if available >= required_bytes {
                log::debug!("Delta2D: Payload ready - {} bytes available, {} required",
                          available, required_bytes);
                break;
            }

            if wait_start.elapsed() > wait_timeout {
                log::warn!("Delta2D: Timeout waiting for payload - got {} bytes, needed {}",
                          available, required_bytes);
                return Ok(None);
            }

            // Release lock briefly to let data arrive
            drop(transport);
            std::thread::sleep(std::time::Duration::from_millis(1));
            transport = self.transport.lock();
        }

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
                log::debug!("Delta2D: Parsing measurement packet (payload: {} bytes)", payload.len());
                let scan = protocol::parse_measurement(&payload)?;
                log::info!("Delta2D: Received scan with {} points", scan.points.len());
                Ok(Some(scan))
            }
            CommandType::Health => {
                // Health packets don't contain scan data
                log::info!("Delta2D: Received health packet (payload: {} bytes)", payload.len());
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
