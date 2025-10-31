//! Error types for SangamIO

/// Result type alias
pub type Result<T> = std::result::Result<T, Error>;

/// SangamIO error types
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// Serial port error
    #[cfg(feature = "std")]
    #[error("Serial port error: {0}")]
    Serial(#[from] serialport::Error),

    /// I/O error
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Device not initialized
    #[error("Device not initialized")]
    NotInitialized,

    /// Device initialization failed
    #[error("Initialization failed: {0}")]
    InitializationFailed(String),

    /// Communication timeout
    #[error("Communication timeout")]
    Timeout,

    /// Invalid packet or response
    #[error("Invalid packet: {0}")]
    InvalidPacket(String),

    /// Checksum mismatch
    #[error("Checksum error: expected {expected:#04x}, got {actual:#04x}")]
    ChecksumError {
        /// Expected checksum value
        expected: u8,
        /// Actual checksum value
        actual: u8,
    },

    /// Device error code
    #[error("Device error: {0:#04x}")]
    DeviceError(u8),

    /// Component not available
    #[error("Component not available: {0}")]
    ComponentNotAvailable(&'static str),

    /// Invalid parameter
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    /// Operation not supported
    #[error("Operation not supported: {0}")]
    NotSupported(String),

    /// Generic error with message
    #[error("{0}")]
    Other(String),
}
