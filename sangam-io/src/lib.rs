//! # SangamIO - Where Robotic Signals Meet 🤖🌊
//!
//! A high-level robot API library for robotic vacuum cleaners running on embedded Linux.
//!
//! ## Features
//!
//! - **High-Level Robot API**: Simple interface - `robot.move_forward()`, `robot.scan()`
//! - **Automatic Device Management**: Background heartbeat threads, state synchronization
//! - **Modular Architecture**: Easy to add new devices and protocols
//! - **Type-Safe**: Compile-time guarantees with Rust's type system
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use sangam_io::devices::Gd32Driver;
//! use sangam_io::drivers::MotorDriver;
//! use sangam_io::transport::SerialTransport;
//!
//! # fn main() -> sangam_io::Result<()> {
//! // Initialize GD32 motor controller
//! let transport = SerialTransport::open("/dev/ttyS3", 115200)?;
//! let mut motor = Gd32Driver::new(transport)?;
//!
//! // Set velocity (forward at 0.2 m/s, no rotation)
//! motor.set_velocity(0.2, 0.0)?;
//!
//! // Control lidar power
//! motor.set_lidar_power(true)?;
//! # Ok(())
//! # }
//! ```

#![warn(missing_docs)]

pub mod devices;
pub mod drivers;
pub mod error;
pub mod transport;
pub mod types;

// Robot API (to be implemented)
// pub mod robot;
// pub mod builder;

/// Re-exports for convenient imports
pub mod prelude {
    pub use crate::drivers::{BatteryDriver, ImuDriver, LidarDriver, MotorDriver};
    pub use crate::error::{Error, Result};
    pub use crate::types::{
        BatteryStatus, ImuData, LidarPoint, LidarScan, Odometry, Pose2D, Velocity,
    };
    // pub use crate::robot::Robot;
    // pub use crate::builder::RobotBuilder;
}

pub use error::{Error, Result};
pub use types::*;

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
