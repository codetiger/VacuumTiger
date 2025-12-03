//! Core data types for bag file format.

use serde::{Deserialize, Serialize};
use std::path::PathBuf;

use crate::io::sangam_client::LidarScan;
use crate::core::types::{Pose2D, Timestamped};

/// Magic bytes at start of bag file.
pub const BAG_MAGIC: [u8; 4] = *b"DBAG";

/// Current bag file format version.
pub const BAG_VERSION: u16 = 1;

/// Size of the bag file header in bytes.
pub const HEADER_SIZE: usize = 64;

/// Flag indicating the bag file is compressed.
pub const FLAG_COMPRESSED: u16 = 0x01;

/// Flag indicating the bag file has an index.
pub const FLAG_HAS_INDEX: u16 = 0x02;

/// Bag file header (64 bytes fixed size).
///
/// Stored at the beginning of every bag file. Contains metadata
/// about the recording and pointers to optional sections.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BagHeader {
    /// Magic bytes: "DBAG"
    pub magic: [u8; 4],
    /// File format version
    pub version: u16,
    /// Feature flags (compression, index)
    pub flags: u16,
    /// Timestamp of first message (microseconds since epoch)
    pub start_time_us: u64,
    /// Timestamp of last message (microseconds since epoch)
    pub end_time_us: u64,
    /// Total number of messages in the file
    pub message_count: u64,
    /// Byte offset to index section (0 if no index)
    pub index_offset: u64,
    /// Reserved for future use
    pub reserved: [u8; 24],
}

impl BagHeader {
    /// Create a new header with default values.
    pub fn new() -> Self {
        Self {
            magic: BAG_MAGIC,
            version: BAG_VERSION,
            flags: 0,
            start_time_us: 0,
            end_time_us: 0,
            message_count: 0,
            index_offset: 0,
            reserved: [0; 24],
        }
    }

    /// Check if magic bytes are valid.
    pub fn is_valid(&self) -> bool {
        self.magic == BAG_MAGIC
    }

    /// Get recording duration in microseconds.
    pub fn duration_us(&self) -> u64 {
        self.end_time_us.saturating_sub(self.start_time_us)
    }

    /// Get recording duration in seconds.
    pub fn duration_secs(&self) -> f64 {
        self.duration_us() as f64 / 1_000_000.0
    }
}

impl Default for BagHeader {
    fn default() -> Self {
        Self::new()
    }
}

/// Wheel encoder tick values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct EncoderTicks {
    /// Left wheel encoder ticks (raw 16-bit value)
    pub left: u16,
    /// Right wheel encoder ticks (raw 16-bit value)
    pub right: u16,
}

impl EncoderTicks {
    /// Create new encoder ticks.
    pub fn new(left: u16, right: u16) -> Self {
        Self { left, right }
    }
}

/// Sensor status message from SangamIO.
///
/// Contains wheel encoder and IMU data at 500Hz.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorStatusMsg {
    /// Timestamp in microseconds since epoch
    pub timestamp_us: u64,
    /// Wheel encoder ticks
    pub encoder: EncoderTicks,
    /// Raw gyroscope values [x, y, z] in 0.1 deg/s units
    pub gyro_raw: [i16; 3],
    /// Raw accelerometer values [x, y, z]
    pub accel_raw: [i16; 3],
}

impl SensorStatusMsg {
    /// Create a new sensor status message.
    pub fn new(
        timestamp_us: u64,
        encoder: EncoderTicks,
        gyro_raw: [i16; 3],
        accel_raw: [i16; 3],
    ) -> Self {
        Self {
            timestamp_us,
            encoder,
            gyro_raw,
            accel_raw,
        }
    }
}

/// Message types stored in bag files.
///
/// Each variant corresponds to a different sensor stream.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BagMessage {
    /// Sensor status (500 Hz): encoders + IMU
    SensorStatus(SensorStatusMsg),
    /// LiDAR scan (5 Hz): point cloud
    Lidar(Timestamped<LidarScan>),
    /// Computed odometry pose (for ground truth or computed values)
    Odometry(Timestamped<Pose2D>),
}

impl BagMessage {
    /// Get the timestamp of this message in microseconds.
    pub fn timestamp_us(&self) -> u64 {
        match self {
            BagMessage::SensorStatus(msg) => msg.timestamp_us,
            BagMessage::Lidar(msg) => msg.timestamp_us,
            BagMessage::Odometry(msg) => msg.timestamp_us,
        }
    }

    /// Check if this is a sensor status message.
    pub fn is_sensor_status(&self) -> bool {
        matches!(self, BagMessage::SensorStatus(_))
    }

    /// Check if this is a lidar message.
    pub fn is_lidar(&self) -> bool {
        matches!(self, BagMessage::Lidar(_))
    }

    /// Check if this is an odometry message.
    pub fn is_odometry(&self) -> bool {
        matches!(self, BagMessage::Odometry(_))
    }

    /// Get as sensor status if this is that type.
    pub fn as_sensor_status(&self) -> Option<&SensorStatusMsg> {
        match self {
            BagMessage::SensorStatus(msg) => Some(msg),
            _ => None,
        }
    }

    /// Get as lidar scan if this is that type.
    pub fn as_lidar(&self) -> Option<&Timestamped<LidarScan>> {
        match self {
            BagMessage::Lidar(msg) => Some(msg),
            _ => None,
        }
    }

    /// Get as odometry if this is that type.
    pub fn as_odometry(&self) -> Option<&Timestamped<Pose2D>> {
        match self {
            BagMessage::Odometry(msg) => Some(msg),
            _ => None,
        }
    }
}

/// Information about a bag file.
///
/// Returned after recording or when inspecting a bag file.
#[derive(Debug, Clone)]
pub struct BagInfo {
    /// Path to the bag file
    pub path: PathBuf,
    /// Recording duration in microseconds
    pub duration_us: u64,
    /// Total number of messages
    pub message_count: u64,
    /// File size in bytes
    pub file_size_bytes: u64,
    /// Number of sensor status messages
    pub sensor_count: u64,
    /// Number of lidar messages
    pub lidar_count: u64,
    /// Number of odometry messages
    pub odometry_count: u64,
}

impl BagInfo {
    /// Get recording duration in seconds.
    pub fn duration_secs(&self) -> f64 {
        self.duration_us as f64 / 1_000_000.0
    }

    /// Get file size in megabytes.
    pub fn file_size_mb(&self) -> f64 {
        self.file_size_bytes as f64 / 1_048_576.0
    }

    /// Get average message rate in Hz.
    pub fn message_rate_hz(&self) -> f64 {
        if self.duration_us == 0 {
            0.0
        } else {
            self.message_count as f64 / self.duration_secs()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bag_header_new() {
        let header = BagHeader::new();
        assert!(header.is_valid());
        assert_eq!(header.version, BAG_VERSION);
        assert_eq!(header.flags, 0);
        assert_eq!(header.message_count, 0);
    }

    #[test]
    fn test_bag_header_duration() {
        let mut header = BagHeader::new();
        header.start_time_us = 1_000_000;
        header.end_time_us = 6_000_000;

        assert_eq!(header.duration_us(), 5_000_000);
        assert!((header.duration_secs() - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_encoder_ticks() {
        let ticks = EncoderTicks::new(1000, 1001);
        assert_eq!(ticks.left, 1000);
        assert_eq!(ticks.right, 1001);
    }

    #[test]
    fn test_bag_message_timestamp() {
        let sensor_msg = BagMessage::SensorStatus(SensorStatusMsg {
            timestamp_us: 1000,
            encoder: EncoderTicks::new(0, 0),
            gyro_raw: [0, 0, 0],
            accel_raw: [0, 0, 0],
        });
        assert_eq!(sensor_msg.timestamp_us(), 1000);
        assert!(sensor_msg.is_sensor_status());
        assert!(!sensor_msg.is_lidar());

        let lidar_msg = BagMessage::Lidar(Timestamped::new(vec![], 2000));
        assert_eq!(lidar_msg.timestamp_us(), 2000);
        assert!(lidar_msg.is_lidar());
        assert!(!lidar_msg.is_sensor_status());

        let odom_msg = BagMessage::Odometry(Timestamped::new(Pose2D::identity(), 3000));
        assert_eq!(odom_msg.timestamp_us(), 3000);
        assert!(odom_msg.is_odometry());
    }

    #[test]
    fn test_bag_info_calculations() {
        let info = BagInfo {
            path: PathBuf::from("test.bag"),
            duration_us: 5_000_000,
            message_count: 2500,
            file_size_bytes: 1_048_576,
            sensor_count: 2400,
            lidar_count: 25,
            odometry_count: 75,
        };

        assert!((info.duration_secs() - 5.0).abs() < 0.001);
        assert!((info.file_size_mb() - 1.0).abs() < 0.001);
        assert!((info.message_rate_hz() - 500.0).abs() < 0.1);
    }
}
