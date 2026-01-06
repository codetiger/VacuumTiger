//! Error types for DhruvaNav

use thiserror::Error;

/// DhruvaNav error type
#[derive(Error, Debug)]
pub enum DhruvaError {
    #[error("Connection failed: {0}")]
    Connection(#[from] std::io::Error),

    #[error("Protocol error: {0}")]
    Protocol(String),

    #[error("Configuration error: {0}")]
    Config(String),

    #[error("SLAM error: {0}")]
    Slam(String),
}

impl From<prost::DecodeError> for DhruvaError {
    fn from(e: prost::DecodeError) -> Self {
        DhruvaError::Protocol(e.to_string())
    }
}

impl From<toml::de::Error> for DhruvaError {
    fn from(e: toml::de::Error) -> Self {
        DhruvaError::Config(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, DhruvaError>;
