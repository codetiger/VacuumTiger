//! Error types for SangamIO
//!
//! # Error Recovery Strategies
//!
//! Different error types require different recovery approaches:
//!
//! ## Fatal Errors (Require Restart)
//!
//! - **`ThreadPanic`**: A thread panicked unexpectedly. The driver must be restarted.
//!   Application should log the error and attempt to re-initialize the device.
//!
//! - **`MutexPoisoned`**: A mutex was poisoned by a panicking thread. The affected
//!   component (heartbeat, reader, or command handler) will exit. For the heartbeat
//!   thread, this is **critical** as it means motors will stop. The application
//!   should restart the driver immediately.
//!
//! ## Transient Errors (Retry Recommended)
//!
//! - **`Serial`**: Serial port communication error. Often caused by:
//!   - Cable disconnection (unrecoverable without hardware intervention)
//!   - Temporary I/O congestion (retryable)
//!   - Device reset (wait for device to reinitialize)
//!
//! - **`Io`**: Generic I/O error. Usually retryable after a brief delay.
//!
//! ## Protocol Errors (Log and Continue)
//!
//! - **`Json`**: Message deserialization failed. Log the error and discard the
//!   malformed message. The connection remains usable for future messages.
//!
//! - **`Serialization`**: Failed to serialize outbound message. Log and skip
//!   this specific message.
//!
//! ## Configuration Errors (Fix and Restart)
//!
//! - **`Config`**: Configuration file is invalid. Fix the configuration and restart.
//! - **`UnknownDevice`**: Device type not recognized. Check hardware.json.
//!
//! ## Implementation Status
//!
//! - **`NotImplemented`**: Feature not yet implemented. This is a development-time
//!   error that should not appear in production if all command paths are tested.
//!
//! # Safety Note
//!
//! **The heartbeat thread never panics.** If it encounters a fatal error (mutex
//! poisoned), it logs and exits gracefully, which causes the GD32 watchdog to
//! stop the motors. This fail-safe behavior prevents runaway robot conditions.

use thiserror::Error;

/// Errors that can occur in SangamIO
///
/// See module-level documentation for recovery strategies.
#[derive(Error, Debug)]
pub enum Error {
    #[error("Serial port error: {0}")]
    Serial(#[from] serialport::Error),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("JSON error: {0}")]
    Json(#[from] serde_json::Error),

    #[error("Thread panic")]
    ThreadPanic,

    #[error("Mutex poisoned")]
    MutexPoisoned,

    #[error("Not implemented: {0}")]
    NotImplemented(String),

    #[error("Unknown device type: {0}")]
    UnknownDevice(String),

    #[error("Config error: {0}")]
    Config(String),

    #[error("Serialization error: {0}")]
    Serialization(String),

    #[error("{0}")]
    Other(String),
}

pub type Result<T> = std::result::Result<T, Error>;
