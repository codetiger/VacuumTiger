//! Transport layer for I/O abstraction

use crate::error::Result;

mod serial;
pub use serial::SerialTransport;

/// Transport trait for device communication
pub trait Transport: Send {
    /// Read data into buffer, returns number of bytes read
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize>;

    /// Write data from buffer, returns number of bytes written
    fn write(&mut self, data: &[u8]) -> Result<usize>;

    /// Flush any pending writes (blocking until complete)
    fn flush(&mut self) -> Result<()>;

    /// Try to flush pending writes without blocking (best-effort)
    ///
    /// This is a hint to the transport layer that writes should be sent,
    /// but unlike flush(), this does not block. The implementation may
    /// choose to do nothing if flushing would block.
    #[allow(dead_code)] // Part of trait API, may be used by future implementations
    fn try_flush(&mut self) -> Result<()> {
        // Default: no-op (fire and forget)
        Ok(())
    }

    /// Check if data is available to read
    fn available(&mut self) -> Result<usize> {
        Ok(0) // Default implementation
    }
}
