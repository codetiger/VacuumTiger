//! Configuration loading errors.

/// Config load error
#[derive(Debug, Clone)]
pub enum ConfigLoadError {
    /// I/O error
    Io(String),
    /// Parse error
    Parse(String),
}

impl std::fmt::Display for ConfigLoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigLoadError::Io(msg) => write!(f, "IO error: {}", msg),
            ConfigLoadError::Parse(msg) => write!(f, "Parse error: {}", msg),
        }
    }
}

impl std::error::Error for ConfigLoadError {}
