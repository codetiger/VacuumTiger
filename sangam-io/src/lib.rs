//! SangamIO - Hardware abstraction library for robot vacuum
//!
//! This library provides the core components for interacting with robot vacuum
//! hardware and simulation.
//!
//! ## Features
//!
//! - `mock`: Enable mock device simulation for hardware-free testing

pub mod config;
pub mod core;
pub mod devices;
pub mod error;
pub mod streaming;

// Re-export commonly used types
pub use config::Config;
pub use error::{Error, Result};
