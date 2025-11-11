//! Error types for SangamIO

/// Result type alias
pub type Result<T> = std::result::Result<T, Error>;

/// SangamIO error types
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// Serial port error
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

    /// Invalid packet or response
    #[error("Invalid packet: {0}")]
    InvalidPacket(String),

    /// Component not available
    #[error("Component not available: {0}")]
    ComponentNotAvailable(&'static str),

    /// Invalid parameter
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    /// Operation not supported
    #[error("Operation not supported: {0}")]
    NotSupported(String),

    /// Thread panicked during execution
    #[error("Thread panicked")]
    ThreadPanic,

    /// Thread shutdown timeout exceeded
    #[error("Thread shutdown timeout exceeded")]
    ShutdownTimeout,

    /// Scanning already in progress
    #[error("Scanning already in progress")]
    AlreadyScanning,

    /// Generic error with message
    #[error("{0}")]
    Other(String),
}
