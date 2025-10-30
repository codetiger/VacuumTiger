//! Transport layer for I/O abstraction

use crate::error::Result;

#[cfg(feature = "std")]
mod serial;
#[cfg(feature = "std")]
pub use serial::SerialTransport;

mod mock;
pub use mock::MockTransport;

/// Transport trait for device communication
pub trait Transport: Send {
    /// Read data into buffer, returns number of bytes read
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize>;

    /// Write data from buffer, returns number of bytes written
    fn write(&mut self, data: &[u8]) -> Result<usize>;

    /// Flush any pending writes
    fn flush(&mut self) -> Result<()>;

    /// Check if data is available to read
    fn available(&mut self) -> Result<usize> {
        Ok(0) // Default implementation
    }
}
