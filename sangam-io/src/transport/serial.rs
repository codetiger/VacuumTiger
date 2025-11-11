//! Serial transport implementation

use super::Transport;
use crate::error::Result;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{Read, Write};
use std::time::Duration;

/// Serial transport for UART communication
pub struct SerialTransport {
    port: Box<dyn SerialPort>,
}

impl SerialTransport {
    /// Open a serial port
    ///
    /// # Arguments
    /// * `path` - Serial port path (e.g., "/dev/ttyS3")
    /// * `baud_rate` - Baud rate (e.g., 115200)
    pub fn open(path: &str, baud_rate: u32) -> Result<Self> {
        let port = serialport::new(path, baud_rate)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(Duration::from_micros(100)) // 100μs timeout for minimal blocking
            .open()?;

        log::info!("Opened serial port: {} at {} baud", path, baud_rate);

        // Apply low-level optimizations for minimal latency (matches AuxCtrl configuration)
        #[cfg(unix)]
        {
            Self::apply_low_level_config(&*port)?;
        }

        Ok(SerialTransport { port })
    }

    /// Apply low-level terminal configuration for minimal latency
    #[cfg(unix)]
    fn apply_low_level_config(_port: &dyn SerialPort) -> Result<()> {
        // Note: Low-level termios configuration is currently disabled due to
        // trait object limitations in the serialport crate. The serialport crate
        // already configures reasonable defaults (8N1, no flow control).
        //
        // Future improvement: Consider using platform-specific extensions or
        // accessing the underlying file descriptor through unsafe code if
        // performance testing shows this is necessary.

        log::debug!("Low-level serial optimizations skipped (using serialport defaults)");
        Ok(())
    }
}

impl Transport for SerialTransport {
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        match self.port.read(buffer) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(0),
            Err(e) => Err(e.into()),
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<usize> {
        Ok(self.port.write(data)?)
    }

    fn flush(&mut self) -> Result<()> {
        self.port.flush()?;
        Ok(())
    }

    fn available(&mut self) -> Result<usize> {
        Ok(self.port.bytes_to_read()? as usize)
    }
}
