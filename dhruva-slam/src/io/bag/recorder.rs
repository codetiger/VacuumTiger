//! Bag file recorder for capturing sensor data.

use std::fs::File;
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use super::types::{
    BAG_MAGIC, BAG_VERSION, BagHeader, BagInfo, BagMessage, HEADER_SIZE, SensorStatusMsg,
};
use crate::core::types::{Pose2D, Timestamped};
use crate::io::sangam_client::LidarScan;

/// Error type for bag recording operations.
#[derive(Debug)]
pub enum RecorderError {
    /// I/O error
    Io(std::io::Error),
    /// Serialization error
    Serialize(String),
}

impl std::fmt::Display for RecorderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RecorderError::Io(e) => write!(f, "I/O error: {}", e),
            RecorderError::Serialize(e) => write!(f, "Serialization error: {}", e),
        }
    }
}

impl std::error::Error for RecorderError {}

impl From<std::io::Error> for RecorderError {
    fn from(e: std::io::Error) -> Self {
        RecorderError::Io(e)
    }
}

impl From<postcard::Error> for RecorderError {
    fn from(e: postcard::Error) -> Self {
        RecorderError::Serialize(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, RecorderError>;

/// Bag file recorder.
///
/// Records sensor messages to a binary bag file using Postcard serialization.
/// The file format uses length-prefixed messages for efficient streaming.
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::bag::{BagRecorder, SensorStatusMsg, EncoderTicks};
///
/// let mut recorder = BagRecorder::create("recording.bag")?;
///
/// // Record sensor messages
/// recorder.record_sensor_status(&SensorStatusMsg {
///     timestamp_us: 1000000,
///     encoder: EncoderTicks::new(100, 100),
///     gyro_raw: [0, 0, 0],
///     accel_raw: [0, 0, 1638],
///     tilt_raw: [0, 0, 1000],
/// })?;
///
/// // Finalize and close
/// let info = recorder.finish()?;
/// println!("Recorded {} messages", info.message_count);
/// ```
pub struct BagRecorder {
    writer: BufWriter<File>,
    path: PathBuf,
    message_count: u64,
    sensor_count: u64,
    lidar_count: u64,
    odometry_count: u64,
    start_time_us: Option<u64>,
    end_time_us: u64,
}

impl BagRecorder {
    /// Create a new bag recorder writing to the specified path.
    ///
    /// The file is created immediately and the header space is reserved.
    /// Call `finish()` to write the final header and close the file.
    pub fn create(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref().to_path_buf();
        let file = File::create(&path)?;
        let mut writer = BufWriter::new(file);

        // Reserve space for header (64 bytes of zeros)
        writer.write_all(&[0u8; HEADER_SIZE])?;

        Ok(Self {
            writer,
            path,
            message_count: 0,
            sensor_count: 0,
            lidar_count: 0,
            odometry_count: 0,
            start_time_us: None,
            end_time_us: 0,
        })
    }

    /// Record a sensor status message.
    pub fn record_sensor_status(&mut self, msg: &SensorStatusMsg) -> Result<()> {
        self.sensor_count += 1;
        self.record_message(&BagMessage::SensorStatus(msg.clone()))
    }

    /// Record a LiDAR scan.
    pub fn record_lidar(&mut self, scan: &Timestamped<LidarScan>) -> Result<()> {
        self.lidar_count += 1;
        self.record_message(&BagMessage::Lidar(scan.clone()))
    }

    /// Record an odometry pose.
    pub fn record_odometry(&mut self, pose: &Timestamped<Pose2D>) -> Result<()> {
        self.odometry_count += 1;
        self.record_message(&BagMessage::Odometry(pose.clone()))
    }

    /// Record a generic bag message.
    pub fn record_message(&mut self, msg: &BagMessage) -> Result<()> {
        let timestamp = msg.timestamp_us();

        // Track time range
        if self.start_time_us.is_none() {
            self.start_time_us = Some(timestamp);
        }
        self.end_time_us = timestamp;

        // Serialize message using postcard
        let bytes = postcard::to_allocvec(msg)?;

        // Write length prefix (4 bytes, little-endian) + payload
        let len = bytes.len() as u32;
        self.writer.write_all(&len.to_le_bytes())?;
        self.writer.write_all(&bytes)?;

        self.message_count += 1;
        Ok(())
    }

    /// Get current message count.
    pub fn message_count(&self) -> u64 {
        self.message_count
    }

    /// Get current recording duration in microseconds.
    pub fn duration_us(&self) -> u64 {
        self.end_time_us
            .saturating_sub(self.start_time_us.unwrap_or(0))
    }

    /// Finalize and close the bag file.
    ///
    /// Writes the header with final statistics and returns file info.
    /// This method consumes the recorder.
    pub fn finish(mut self) -> Result<BagInfo> {
        // Flush buffered data
        self.writer.flush()?;

        // Get file size before seeking
        let file_size = self.writer.stream_position()?;

        // Seek back to beginning to write header
        self.writer.seek(SeekFrom::Start(0))?;

        // Create header
        let header = BagHeader {
            magic: BAG_MAGIC,
            version: BAG_VERSION,
            flags: 0,
            start_time_us: self.start_time_us.unwrap_or(0),
            end_time_us: self.end_time_us,
            message_count: self.message_count,
            index_offset: 0,
            reserved: [0; 24],
        };

        // Serialize header to fixed-size buffer
        let header_bytes = postcard::to_allocvec(&header)?;

        // Write header (pad to HEADER_SIZE if needed)
        let mut header_buffer = [0u8; HEADER_SIZE];
        let copy_len = header_bytes.len().min(HEADER_SIZE);
        header_buffer[..copy_len].copy_from_slice(&header_bytes[..copy_len]);
        self.writer.write_all(&header_buffer)?;

        // Final flush
        self.writer.flush()?;

        // Calculate duration before moving path
        let duration_us = self.duration_us();

        Ok(BagInfo {
            path: self.path,
            duration_us,
            message_count: self.message_count,
            file_size_bytes: file_size,
            sensor_count: self.sensor_count,
            lidar_count: self.lidar_count,
            odometry_count: self.odometry_count,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::bag::types::EncoderTicks;
    use std::fs;
    use tempfile::TempDir;

    #[test]
    fn test_recorder_create_and_finish() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");

        let recorder = BagRecorder::create(&bag_path).unwrap();
        let info = recorder.finish().unwrap();

        assert_eq!(info.message_count, 0);
        assert!(bag_path.exists());

        // File should have at least the header
        let metadata = fs::metadata(&bag_path).unwrap();
        assert!(metadata.len() >= HEADER_SIZE as u64);
    }

    #[test]
    fn test_recorder_sensor_status() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("sensors.bag");

        let mut recorder = BagRecorder::create(&bag_path).unwrap();

        for i in 0..100 {
            recorder
                .record_sensor_status(&SensorStatusMsg {
                    timestamp_us: i * 2000, // 500 Hz
                    encoder: EncoderTicks::new(i as u16, i as u16),
                    gyro_raw: [0, 0, 0],
                    accel_raw: [0, 0, 1638],
                    tilt_raw: [0, 0, 1000],
                })
                .unwrap();
        }

        let info = recorder.finish().unwrap();

        assert_eq!(info.message_count, 100);
        assert_eq!(info.sensor_count, 100);
        assert_eq!(info.lidar_count, 0);
        assert_eq!(info.duration_us, 99 * 2000);
    }

    #[test]
    fn test_recorder_mixed_messages() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("mixed.bag");

        let mut recorder = BagRecorder::create(&bag_path).unwrap();

        // Record some sensor messages
        recorder
            .record_sensor_status(&SensorStatusMsg {
                timestamp_us: 1000,
                encoder: EncoderTicks::new(0, 0),
                gyro_raw: [0, 0, 0],
                accel_raw: [0, 0, 0],
                tilt_raw: [0, 0, 1000],
            })
            .unwrap();

        // Record a lidar scan
        let scan = vec![(0.0f32, 1.0f32, 100u8), (0.1, 1.1, 100)];
        recorder
            .record_lidar(&Timestamped::new(scan, 2000))
            .unwrap();

        // Record an odometry pose
        recorder
            .record_odometry(&Timestamped::new(Pose2D::new(1.0, 2.0, 0.5), 3000))
            .unwrap();

        let info = recorder.finish().unwrap();

        assert_eq!(info.message_count, 3);
        assert_eq!(info.sensor_count, 1);
        assert_eq!(info.lidar_count, 1);
        assert_eq!(info.odometry_count, 1);
    }
}
