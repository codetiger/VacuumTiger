//! Simulated SangamIO client that reads from bag files.
//!
//! Provides an API-compatible replacement for `SangamClient` to enable
//! testing and development without hardware.

use std::collections::HashMap;
use std::path::Path;

use super::player::{BagPlayer, PlayerError};
use super::types::BagMessage;
use crate::io::sangam_client::{ClientError, Message, SensorValue};

/// A SangamClient replacement that reads from a bag file.
///
/// This provides the same API as `SangamClient`, allowing bag files
/// to be used as drop-in replacements for live sensor data during
/// testing and algorithm development.
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::bag::SimulatedClient;
///
/// // Create from bag file (max speed)
/// let mut client = SimulatedClient::from_bag("recording.bag")?;
///
/// // Use exactly like SangamClient
/// while let Ok(msg) = client.recv() {
///     if let Some(lidar) = msg.as_lidar() {
///         println!("Lidar: {} points", lidar.data.len());
///     }
///     if let Some((left, right)) = msg.encoder_ticks() {
///         println!("Encoders: {} / {}", left, right);
///     }
/// }
/// ```
pub struct SimulatedClient {
    player: BagPlayer,
    finished: bool,
}

impl SimulatedClient {
    /// Create a simulated client from a bag file.
    ///
    /// Plays back at maximum speed (no timing delays).
    pub fn from_bag(path: impl AsRef<Path>) -> Result<Self, PlayerError> {
        let player = BagPlayer::open(path)?;
        Ok(Self {
            player,
            finished: false,
        })
    }

    /// Create a simulated client with custom playback speed.
    ///
    /// - 0.0 = maximum speed (no timing)
    /// - 1.0 = real-time
    /// - 2.0 = 2x speed
    pub fn from_bag_with_speed(path: impl AsRef<Path>, speed: f32) -> Result<Self, PlayerError> {
        let mut player = BagPlayer::open(path)?;
        player.set_speed(speed);
        Ok(Self {
            player,
            finished: false,
        })
    }

    /// Check if playback is finished.
    pub fn is_finished(&self) -> bool {
        self.finished
    }

    /// Get number of messages received so far.
    pub fn messages_received(&self) -> u64 {
        self.player.messages_read()
    }

    /// Rewind to beginning of bag file.
    pub fn rewind(&mut self) -> Result<(), PlayerError> {
        self.player.rewind()?;
        self.finished = false;
        Ok(())
    }

    /// Receive next message (mimics `SangamClient::recv`).
    ///
    /// Returns `ClientError::Disconnected` when bag file is exhausted.
    pub fn recv(&mut self) -> Result<Message, ClientError> {
        if self.finished {
            return Err(ClientError::Disconnected);
        }

        match self.player.next() {
            Ok(Some(bag_msg)) => Ok(self.convert_to_message(bag_msg)),
            Ok(None) => {
                self.finished = true;
                Err(ClientError::Disconnected)
            }
            Err(e) => Err(ClientError::Deserialize(e.to_string())),
        }
    }

    /// Convert bag message to SangamIO message format.
    fn convert_to_message(&self, bag_msg: BagMessage) -> Message {
        match bag_msg {
            BagMessage::SensorStatus(status) => {
                let mut values = HashMap::new();
                // U16 values are stored as U32 in protobuf
                values.insert(
                    "wheel_left".to_string(),
                    SensorValue::U32(status.encoder.left as u32),
                );
                values.insert(
                    "wheel_right".to_string(),
                    SensorValue::U32(status.encoder.right as u32),
                );
                // I16 values are stored as I32 in protobuf
                values.insert(
                    "gyro_x".to_string(),
                    SensorValue::I32(status.gyro_raw[0] as i32),
                );
                values.insert(
                    "gyro_y".to_string(),
                    SensorValue::I32(status.gyro_raw[1] as i32),
                );
                values.insert(
                    "gyro_z".to_string(),
                    SensorValue::I32(status.gyro_raw[2] as i32),
                );
                values.insert(
                    "accel_x".to_string(),
                    SensorValue::I32(status.accel_raw[0] as i32),
                );
                values.insert(
                    "accel_y".to_string(),
                    SensorValue::I32(status.accel_raw[1] as i32),
                );
                values.insert(
                    "accel_z".to_string(),
                    SensorValue::I32(status.accel_raw[2] as i32),
                );

                Message::new(
                    "sensors/sensor_status",
                    "sensor_status",
                    status.timestamp_us,
                    values,
                )
            }
            BagMessage::Lidar(scan) => {
                let mut values = HashMap::new();
                values.insert("scan".to_string(), SensorValue::PointCloud2D(scan.data));

                Message::new("sensors/lidar", "lidar", scan.timestamp_us, values)
            }
            BagMessage::Odometry(pose) => {
                // Odometry not directly in SangamIO format, but useful for ground truth
                let mut values = HashMap::new();
                values.insert("x".to_string(), SensorValue::F32(pose.data.x));
                values.insert("y".to_string(), SensorValue::F32(pose.data.y));
                values.insert("theta".to_string(), SensorValue::F32(pose.data.theta));

                Message::new("sensors/odometry", "odometry", pose.timestamp_us, values)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::{Pose2D, Timestamped};
    use crate::io::bag::recorder::BagRecorder;
    use crate::io::bag::types::{EncoderTicks, SensorStatusMsg};
    use crate::io::sangam_client::LidarScan;
    use tempfile::TempDir;

    fn create_test_bag(path: &std::path::Path) {
        let mut recorder = BagRecorder::create(path).unwrap();

        // Record sensor messages
        for i in 0..10 {
            recorder
                .record_sensor_status(&SensorStatusMsg {
                    timestamp_us: i * 2000,
                    encoder: EncoderTicks::new(i as u16 * 10, i as u16 * 10 + 1),
                    gyro_raw: [i as i16, 0, 0],
                    accel_raw: [0, 0, 1638],
                })
                .unwrap();
        }

        // Record a lidar scan
        let scan: LidarScan = vec![(0.0, 1.0, 100), (0.1, 1.1, 100)];
        recorder
            .record_lidar(&Timestamped::new(scan, 20000))
            .unwrap();

        // Record odometry
        recorder
            .record_odometry(&Timestamped::new(Pose2D::new(1.0, 0.0, 0.0), 22000))
            .unwrap();

        recorder.finish().unwrap();
    }

    #[test]
    fn test_simulated_client_basic() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");
        create_test_bag(&bag_path);

        let mut client = SimulatedClient::from_bag(&bag_path).unwrap();

        let mut sensor_count = 0;
        let mut lidar_count = 0;

        while let Ok(msg) = client.recv() {
            if msg.encoder_ticks().is_some() {
                sensor_count += 1;
            }
            if msg.as_lidar().is_some() {
                lidar_count += 1;
            }
        }

        assert_eq!(sensor_count, 10);
        assert_eq!(lidar_count, 1);
        assert!(client.is_finished());
    }

    #[test]
    fn test_simulated_client_encoder_values() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");
        create_test_bag(&bag_path);

        let mut client = SimulatedClient::from_bag(&bag_path).unwrap();

        // First message should have encoder values (0, 1)
        let msg = client.recv().unwrap();
        let (left, right) = msg.encoder_ticks().unwrap();
        assert_eq!(left, 0);
        assert_eq!(right, 1);

        // Second message should have encoder values (10, 11)
        let msg = client.recv().unwrap();
        let (left, right) = msg.encoder_ticks().unwrap();
        assert_eq!(left, 10);
        assert_eq!(right, 11);
    }

    #[test]
    fn test_simulated_client_gyro_values() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");
        create_test_bag(&bag_path);

        let mut client = SimulatedClient::from_bag(&bag_path).unwrap();

        let msg = client.recv().unwrap();
        let gyro = msg.gyro_raw().unwrap();
        assert_eq!(gyro[0], 0); // First message has gyro_x = 0

        let msg = client.recv().unwrap();
        let gyro = msg.gyro_raw().unwrap();
        assert_eq!(gyro[0], 1); // Second message has gyro_x = 1
    }

    #[test]
    fn test_simulated_client_lidar() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");
        create_test_bag(&bag_path);

        let mut client = SimulatedClient::from_bag(&bag_path).unwrap();

        // Skip sensor messages
        for _ in 0..10 {
            client.recv().unwrap();
        }

        // Lidar message
        let msg = client.recv().unwrap();
        let lidar = msg.as_lidar().unwrap();
        assert_eq!(lidar.data.len(), 2);
        assert_eq!(lidar.timestamp_us, 20000);
    }

    #[test]
    fn test_simulated_client_rewind() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");
        create_test_bag(&bag_path);

        let mut client = SimulatedClient::from_bag(&bag_path).unwrap();

        // Consume all messages
        while client.recv().is_ok() {}
        assert!(client.is_finished());

        // Rewind
        client.rewind().unwrap();
        assert!(!client.is_finished());

        // Should be able to receive again
        let msg = client.recv().unwrap();
        assert!(msg.encoder_ticks().is_some());
    }

    #[test]
    fn test_simulated_client_disconnected_error() {
        let temp_dir = TempDir::new().unwrap();
        let bag_path = temp_dir.path().join("test.bag");

        // Create empty bag
        let recorder = BagRecorder::create(&bag_path).unwrap();
        recorder.finish().unwrap();

        let mut client = SimulatedClient::from_bag(&bag_path).unwrap();

        // Should immediately return Disconnected
        let result = client.recv();
        assert!(matches!(result, Err(ClientError::Disconnected)));
    }
}
