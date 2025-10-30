//! Device driver traits

pub mod battery;
pub mod imu;
pub mod lidar;
pub mod motor;

pub use battery::BatteryDriver;
pub use imu::ImuDriver;
pub use lidar::LidarDriver;
pub use motor::MotorDriver;
