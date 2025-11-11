//! # SangamIO - Where Robotic Signals Meet 🤖🌊
//!
//! A hardware abstraction library for robotic vacuum cleaners running on embedded Linux.
//!
//! ## Overview
//!
//! - **Unified Hardware Interface**: Single `SangamIO` struct abstracts all hardware
//! - **Motion Control**: Velocity and position-based movement commands with constraints
//! - **SLAM Support**: Odometry deltas and lidar integration for mapping
//! - **Automatic Device Management**: Background threads, heartbeat, state synchronization
//! - **Type-Safe**: Compile-time guarantees with Rust's type system
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use sangam_io::SangamIO;
//!
//! # fn main() -> sangam_io::Result<()> {
//! // Initialize hardware (CRL-200S configuration)
//! let mut sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;
//!
//! // Set velocity (forward at 0.2 m/s, no rotation)
//! sangam.set_velocity(0.2, 0.0)?;
//!
//! // Move forward 0.5 meters
//! sangam.move_forward(0.5)?;
//!
//! // Get odometry for SLAM
//! let delta = sangam.get_odometry_delta()?;
//! # Ok(())
//! # }
//! ```

#![warn(missing_docs)]

// Internal modules (not exposed in public API)
mod config;
mod devices;
mod drivers;
mod motion;
mod odometry;
mod transport;

// Public modules (only error and types are exposed)
pub mod error;
pub mod types;

// SangamIO hardware abstraction
mod sangam;

// Public API exports - only SangamIO and essential types
pub use error::{Error, Result};
pub use odometry::OdometryDelta;
pub use sangam::SangamIO;
pub use types::{LidarPoint, LidarScan, Odometry, Pose2D};

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
