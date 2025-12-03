//! Bag file player for replaying recorded sensor data.

use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::Path;
use std::time::{Duration, Instant};

use super::types::{BagHeader, BagMessage, SensorStatusMsg, HEADER_SIZE};
use crate::io::sangam_client::LidarScan;
use crate::core::types::Timestamped;

/// Error type for bag playback operations.
#[derive(Debug)]
pub enum PlayerError {
    /// I/O error
    Io(std::io::Error),
    /// Deserialization error
    Deserialize(String),
    /// Invalid bag file format
    InvalidFormat(String),
}

impl std::fmt::Display for PlayerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlayerError::Io(e) => write!(f, "I/O error: {}", e),
            PlayerError::Deserialize(e) => write!(f, "Deserialization error: {}", e),
            PlayerError::InvalidFormat(e) => write!(f, "Invalid format: {}", e),
        }
    }
}

impl std::error::Error for PlayerError {}

impl From<std::io::Error> for PlayerError {
    fn from(e: std::io::Error) -> Self {
        PlayerError::Io(e)
    }
}

impl From<postcard::Error> for PlayerError {
    fn from(e: postcard::Error) -> Self {
        PlayerError::Deserialize(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, PlayerError>;

/// Bag file player for replaying recorded sensor data.
///
/// Supports both immediate playback (as fast as possible) and real-time
/// playback with configurable speed multiplier.
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::bag::BagPlayer;
///
/// let mut player = BagPlayer::open("recording.bag")?;
///
/// // Fast playback (no timing)
/// while let Some(msg) = player.next_immediate()? {
///     println!("Message at {} us", msg.timestamp_us());
/// }
///
/// // Real-time playback
/// player.rewind()?;
/// player.set_speed(1.0); // Real-time
/// while let Some(msg) = player.next()? {
///     // This will block to match original timing
/// }
/// ```
pub struct BagPlayer {
    reader: BufReader<File>,
    header: BagHeader,
    playback_start: Option<Instant>,
    first_msg_time_us: Option<u64>,
    playback_speed: f32,
    messages_read: u64,
}

impl BagPlayer {
    /// Open a bag file for playback.
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let file = File::open(path)?;
        let mut reader = BufReader::new(file);

        // Read header
        let mut header_buffer = [0u8; HEADER_SIZE];
        reader.read_exact(&mut header_buffer)?;

        // Deserialize header
        let header: BagHeader = postcard::from_bytes(&header_buffer).map_err(|e| {
            PlayerError::InvalidFormat(format!("Failed to parse header: {}", e))
        })?;

        // Verify magic
        if !header.is_valid() {
            return Err(PlayerError::InvalidFormat(
                "Invalid bag file magic bytes".to_string(),
            ));
        }

        Ok(Self {
            reader,
            header,
            playback_start: None,
            first_msg_time_us: None,
            playback_speed: 0.0, // Default: no timing (as fast as possible)
            messages_read: 0,
        })
    }

    /// Get the bag file header.
    pub fn header(&self) -> &BagHeader {
        &self.header
    }

    /// Get total duration in microseconds.
    pub fn duration_us(&self) -> u64 {
        self.header.duration_us()
    }

    /// Get total duration in seconds.
    pub fn duration_secs(&self) -> f64 {
        self.header.duration_secs()
    }

    /// Get total message count.
    pub fn message_count(&self) -> u64 {
        self.header.message_count
    }

    /// Get number of messages read so far.
    pub fn messages_read(&self) -> u64 {
        self.messages_read
    }

    /// Set playback speed.
    ///
    /// - 0.0 = as fast as possible (no timing)
    /// - 1.0 = real-time
    /// - 2.0 = 2x speed
    /// - 0.5 = half speed
    pub fn set_speed(&mut self, speed: f32) {
        self.playback_speed = speed;
    }

    /// Get current playback speed.
    pub fn speed(&self) -> f32 {
        self.playback_speed
    }

    /// Read next message without timing delay.
    ///
    /// Returns `None` when end of file is reached.
    pub fn next_immediate(&mut self) -> Result<Option<BagMessage>> {
        // Read length prefix (4 bytes, little-endian)
        let mut len_bytes = [0u8; 4];
        match self.reader.read_exact(&mut len_bytes) {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(e) => return Err(e.into()),
        }

        let len = u32::from_le_bytes(len_bytes) as usize;

        // Sanity check length
        if len > 10_000_000 {
            // 10 MB max message size
            return Err(PlayerError::InvalidFormat(format!(
                "Message too large: {} bytes",
                len
            )));
        }

        // Read payload
        let mut payload = vec![0u8; len];
        self.reader.read_exact(&mut payload)?;

        // Deserialize
        let msg: BagMessage = postcard::from_bytes(&payload)?;

        self.messages_read += 1;

        Ok(Some(msg))
    }

    /// Read next message with real-time timing.
    ///
    /// If `playback_speed` is > 0, this will block to match the original
    /// recording timing.
    pub fn next(&mut self) -> Result<Option<BagMessage>> {
        let msg = self.next_immediate()?;

        if let Some(ref msg) = msg {
            if self.playback_speed > 0.0 {
                self.wait_for_timing(msg.timestamp_us());
            }
        }

        Ok(msg)
    }

    /// Wait until the appropriate time to return this message.
    fn wait_for_timing(&mut self, msg_time_us: u64) {
        // Initialize playback timing on first message
        if self.playback_start.is_none() {
            self.playback_start = Some(Instant::now());
            self.first_msg_time_us = Some(msg_time_us);
            return;
        }

        let elapsed_real = self.playback_start.unwrap().elapsed();
        let msg_offset_us = msg_time_us.saturating_sub(self.first_msg_time_us.unwrap());
        let target_elapsed = Duration::from_micros(
            (msg_offset_us as f64 / self.playback_speed as f64) as u64,
        );

        if target_elapsed > elapsed_real {
            std::thread::sleep(target_elapsed - elapsed_real);
        }
    }

    /// Reset to beginning of file.
    pub fn rewind(&mut self) -> Result<()> {
        self.reader.seek(SeekFrom::Start(HEADER_SIZE as u64))?;
        self.playback_start = None;
        self.first_msg_time_us = None;
        self.messages_read = 0;
        Ok(())
    }

    /// Iterate over only sensor status messages.
    pub fn sensor_status_iter(&mut self) -> impl Iterator<Item = Result<SensorStatusMsg>> + '_ {
        std::iter::from_fn(move || loop {
            match self.next_immediate() {
                Ok(Some(BagMessage::SensorStatus(msg))) => return Some(Ok(msg)),
                Ok(Some(_)) => continue,
                Ok(None) => return None,
                Err(e) => return Some(Err(e)),
            }
        })
    }

    /// Iterate over only LiDAR scans.
    pub fn lidar_iter(&mut self) -> impl Iterator<Item = Result<Timestamped<LidarScan>>> + '_ {
        std::iter::from_fn(move || loop {
            match self.next_immediate() {
                Ok(Some(BagMessage::Lidar(scan))) => return Some(Ok(scan)),
                Ok(Some(_)) => continue,
                Ok(None) => return None,
                Err(e) => return Some(Err(e)),
            }
        })
    }
}

/// Iterator implementation for convenient `for msg in player { }` syntax.
impl Iterator for BagPlayer {
    type Item = Result<BagMessage>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.next_immediate() {
            Ok(Some(msg)) => Some(Ok(msg)),
            Ok(None) => None,
            Err(e) => Some(Err(e)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::bag::recorder::BagRecorder;
    use crate::io::bag::types::EncoderTicks;
    use crate::core::types::Pose2D;
    use tempfile::TempDir;

    fn create_test_bag(path: &Path, num_messages: u64) {
        let mut recorder = BagRecorder::create(path).unwrap();

        for i in 0..num_messages {
            recorder
                .record_sensor_status(&SensorStatusMsg {
                    timestamp_us: i * 2000,
                    encoder: EncoderTicks::new(i as u16, i as u16),
                    gyro_raw: [0, 0, 0],
                    accel_raw: [0, 0, 1638],
                })
                .unwrap();
        }

        recorder.finish().unwrap();
    }

    #[test]
    fn test_player_open_and_header() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");
        create_test_bag(&bag_path, 100);

        let player = BagPlayer::open(&bag_path).unwrap();

        assert!(player.header().is_valid());
        assert_eq!(player.message_count(), 100);
        assert_eq!(player.duration_us(), 99 * 2000);
    }

    #[test]
    fn test_player_read_all_messages() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");
        create_test_bag(&bag_path, 50);

        let mut player = BagPlayer::open(&bag_path).unwrap();
        let mut count = 0;

        while let Some(msg) = player.next_immediate().unwrap() {
            assert!(msg.is_sensor_status());
            count += 1;
        }

        assert_eq!(count, 50);
        assert_eq!(player.messages_read(), 50);
    }

    #[test]
    fn test_player_iterator() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");
        create_test_bag(&bag_path, 25);

        let player = BagPlayer::open(&bag_path).unwrap();
        let messages: Vec<_> = player.collect::<std::result::Result<Vec<_>, _>>().unwrap();

        assert_eq!(messages.len(), 25);
    }

    #[test]
    fn test_player_rewind() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");
        create_test_bag(&bag_path, 10);

        let mut player = BagPlayer::open(&bag_path).unwrap();

        // Read all messages
        while player.next_immediate().unwrap().is_some() {}
        assert_eq!(player.messages_read(), 10);

        // Rewind
        player.rewind().unwrap();
        assert_eq!(player.messages_read(), 0);

        // Read again
        while player.next_immediate().unwrap().is_some() {}
        assert_eq!(player.messages_read(), 10);
    }

    #[test]
    fn test_player_sensor_status_iter() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("mixed.bag");

        // Create bag with mixed messages
        let mut recorder = BagRecorder::create(&bag_path).unwrap();
        recorder
            .record_sensor_status(&SensorStatusMsg {
                timestamp_us: 1000,
                encoder: EncoderTicks::new(0, 0),
                gyro_raw: [0, 0, 0],
                accel_raw: [0, 0, 0],
            })
            .unwrap();
        recorder
            .record_lidar(&Timestamped::new(vec![], 2000))
            .unwrap();
        recorder
            .record_sensor_status(&SensorStatusMsg {
                timestamp_us: 3000,
                encoder: EncoderTicks::new(1, 1),
                gyro_raw: [0, 0, 0],
                accel_raw: [0, 0, 0],
            })
            .unwrap();
        recorder
            .record_odometry(&Timestamped::new(Pose2D::identity(), 4000))
            .unwrap();
        recorder.finish().unwrap();

        let mut player = BagPlayer::open(&bag_path).unwrap();
        let sensor_msgs: Vec<_> = player
            .sensor_status_iter()
            .collect::<std::result::Result<Vec<_>, _>>()
            .unwrap();

        assert_eq!(sensor_msgs.len(), 2);
        assert_eq!(sensor_msgs[0].timestamp_us, 1000);
        assert_eq!(sensor_msgs[1].timestamp_us, 3000);
    }

    #[test]
    fn test_player_invalid_file() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("invalid.bag");

        // Write invalid data
        std::fs::write(&bag_path, b"not a bag file").unwrap();

        let result = BagPlayer::open(&bag_path);
        assert!(result.is_err());
    }
}
