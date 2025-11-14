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

    /// TOML deserialization error
    #[error("TOML deserialization error: {0}")]
    TomlDe(#[from] toml::de::Error),

    /// TOML serialization error
    #[error("TOML serialization error: {0}")]
    TomlSer(#[from] toml::ser::Error),

    /// MessagePack encode error
    #[error("MessagePack encode error: {0}")]
    MsgPackEncode(#[from] rmp_serde::encode::Error),

    /// MessagePack decode error
    #[error("MessagePack decode error: {0}")]
    MsgPackDecode(#[from] rmp_serde::decode::Error),

    // TODO: Streaming error variant removed - not used in current implementation
    // If we need custom streaming errors in the future, uncomment:
    // /// Streaming error (TCP or serialization)
    // #[error("Streaming error: {0}")]
    // Streaming(String),
    /// Device initialization failed
    #[error("Initialization failed: {0}")]
    InitializationFailed(String),

    /// Invalid packet or response
    #[error("Invalid packet: {0}")]
    InvalidPacket(String),

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
