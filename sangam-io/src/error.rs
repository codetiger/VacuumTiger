//! Error types for SangamIO

use thiserror::Error;

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
