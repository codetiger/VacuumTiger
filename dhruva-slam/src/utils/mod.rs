//! Utility functions for example programs and tools.
//!
//! This module provides:
//! - Robot constants (wheel odometry, gyroscope scale)
//! - Signal handling (Ctrl-C)
//! - Statistical utilities (std_dev_i16, I16Stats)

mod constants;
mod signal;
mod stats;

pub use constants::*;
pub use signal::setup_ctrl_c_handler;
pub use stats::{I16Stats, std_dev_i16};
