//! Test fixtures for generating synthetic bag files.

use std::path::{Path, PathBuf};
use tempfile::TempDir;

use super::recorder::BagRecorder;
use super::types::{EncoderTicks, SensorStatusMsg};
use crate::core::types::Timestamped;
use crate::io::sangam_client::LidarScan;

/// Test fixture for bag file testing.
///
/// Creates temporary bag files with synthetic data for testing.
/// The temporary directory is automatically cleaned up when the
/// fixture is dropped.
pub struct BagTestFixture {
    temp_dir: TempDir,
    bag_path: PathBuf,
}

impl BagTestFixture {
    /// Create a synthetic bag file with stationary robot (5 seconds).
    ///
    /// Contains:
    /// - 2500 sensor status messages at 500 Hz
    /// - 25 lidar scans at 5 Hz
    /// - Robot stationary at origin
    pub fn synthetic_5_seconds() -> Self {
        let temp_dir = TempDir::new().expect("Failed to create temp directory");
        let bag_path = temp_dir.path().join("stationary_5s.bag");

        let mut recorder = BagRecorder::create(&bag_path).expect("Failed to create recorder");

        let start_time_us = 1_000_000_000u64; // 1 second since epoch
        let duration_us = 5_000_000u64; // 5 seconds
        let sensor_period_us = 2000u64; // 500 Hz
        let lidar_period_us = 200_000u64; // 5 Hz

        let mut time = start_time_us;
        let encoder_value = 1000u16;

        while time < start_time_us + duration_us {
            // Sensor status at 500Hz
            recorder
                .record_sensor_status(&SensorStatusMsg {
                    timestamp_us: time,
                    encoder: EncoderTicks::new(encoder_value, encoder_value),
                    gyro_raw: [0, 0, 0],
                    accel_raw: [0, 0, 1638], // ~1g on Z axis
                    tilt_raw: [0, 0, 1000],  // Gravity vector on Z
                })
                .expect("Failed to record sensor status");

            // LiDAR at 5Hz (every 200ms)
            if (time - start_time_us) % lidar_period_us < sensor_period_us {
                let scan: LidarScan = (0..360)
                    .map(|i| {
                        let angle = (i as f32) * std::f32::consts::TAU / 360.0;
                        let distance = 2.0; // 2m constant range (circular room)
                        (angle, distance, 100u8)
                    })
                    .collect();

                recorder
                    .record_lidar(&Timestamped::new(scan, time))
                    .expect("Failed to record lidar");
            }

            time += sensor_period_us;
        }

        recorder.finish().expect("Failed to finish recording");

        Self { temp_dir, bag_path }
    }

    /// Create a bag simulating straight-line motion.
    ///
    /// The robot moves forward at the specified speed for the specified distance.
    /// Encoder ticks increase proportionally to position.
    ///
    /// # Parameters
    /// - `distance_m`: Total distance to travel in meters
    /// - `speed_mps`: Forward speed in meters per second
    /// - `ticks_per_meter`: Encoder ticks per meter
    pub fn straight_line(distance_m: f32, speed_mps: f32, ticks_per_meter: f32) -> Self {
        let temp_dir = TempDir::new().expect("Failed to create temp directory");
        let bag_path = temp_dir.path().join("straight_line.bag");

        let mut recorder = BagRecorder::create(&bag_path).expect("Failed to create recorder");

        let duration_s = distance_m / speed_mps;
        let duration_us = (duration_s * 1_000_000.0) as u64;
        let sensor_period_us = 2000u64; // 500 Hz

        let start_time_us = 1_000_000_000u64;
        let mut time = start_time_us;
        let mut position = 0.0f32;

        while time < start_time_us + duration_us {
            let dt = 0.002; // 2ms = 500Hz
            position += speed_mps * dt;

            let ticks = (position * ticks_per_meter) as u16;

            recorder
                .record_sensor_status(&SensorStatusMsg {
                    timestamp_us: time,
                    encoder: EncoderTicks::new(ticks, ticks),
                    gyro_raw: [0, 0, 0],
                    accel_raw: [0, 0, 1638],
                    tilt_raw: [0, 0, 1000],
                })
                .expect("Failed to record sensor status");

            time += sensor_period_us;
        }

        recorder.finish().expect("Failed to finish recording");

        Self { temp_dir, bag_path }
    }

    /// Create a bag simulating rotation in place.
    ///
    /// The robot rotates in place for the specified angle.
    /// Gyro provides angular velocity readings.
    ///
    /// # Parameters
    /// - `angle_rad`: Total rotation angle in radians
    /// - `angular_velocity_radps`: Angular velocity in radians per second
    /// - `gyro_scale`: Gyro scale factor (rad/s per LSB)
    pub fn rotation_in_place(angle_rad: f32, angular_velocity_radps: f32, gyro_scale: f32) -> Self {
        let temp_dir = TempDir::new().expect("Failed to create temp directory");
        let bag_path = temp_dir.path().join("rotation.bag");

        let mut recorder = BagRecorder::create(&bag_path).expect("Failed to create recorder");

        let duration_s = angle_rad.abs() / angular_velocity_radps.abs();
        let duration_us = (duration_s * 1_000_000.0) as u64;
        let sensor_period_us = 2000u64; // 500 Hz

        let start_time_us = 1_000_000_000u64;
        let mut time = start_time_us;

        // Gyro raw value
        let gyro_raw_z = (angular_velocity_radps / gyro_scale) as i16;
        let encoder_value = 1000u16; // Stationary wheels

        while time < start_time_us + duration_us {
            recorder
                .record_sensor_status(&SensorStatusMsg {
                    timestamp_us: time,
                    encoder: EncoderTicks::new(encoder_value, encoder_value),
                    gyro_raw: [0, 0, gyro_raw_z], // ROS REP-103: gyro_z is yaw
                    accel_raw: [0, 0, 1638],
                    tilt_raw: [0, 0, 1000],
                })
                .expect("Failed to record sensor status");

            time += sensor_period_us;
        }

        recorder.finish().expect("Failed to finish recording");

        Self { temp_dir, bag_path }
    }

    /// Create a bag simulating square path motion.
    ///
    /// The robot drives in a square pattern and returns to start.
    /// Includes both straight segments and 90° turns.
    ///
    /// # Parameters
    /// - `side_length_m`: Length of each side in meters
    /// - `linear_speed_mps`: Forward speed in m/s
    /// - `angular_speed_radps`: Turning speed in rad/s
    /// - `ticks_per_meter`: Encoder ticks per meter
    /// - `gyro_scale`: Gyro scale factor
    pub fn square_path(
        side_length_m: f32,
        linear_speed_mps: f32,
        angular_speed_radps: f32,
        ticks_per_meter: f32,
        gyro_scale: f32,
    ) -> Self {
        let temp_dir = TempDir::new().expect("Failed to create temp directory");
        let bag_path = temp_dir.path().join("square.bag");

        let mut recorder = BagRecorder::create(&bag_path).expect("Failed to create recorder");

        let straight_time_s = side_length_m / linear_speed_mps;
        let turn_time_s = (std::f32::consts::FRAC_PI_2) / angular_speed_radps;
        let sensor_period_us = 2000u64;

        let start_time_us = 1_000_000_000u64;
        let mut time = start_time_us;
        let mut total_distance = 0.0f32;

        // 4 sides with 4 turns
        for _ in 0..4 {
            // Straight segment
            let segment_duration_us = (straight_time_s * 1_000_000.0) as u64;
            let segment_end_time = time + segment_duration_us;

            while time < segment_end_time {
                let dt = 0.002;
                total_distance += linear_speed_mps * dt;

                let ticks = (total_distance * ticks_per_meter) as u16;

                recorder
                    .record_sensor_status(&SensorStatusMsg {
                        timestamp_us: time,
                        encoder: EncoderTicks::new(ticks, ticks),
                        gyro_raw: [0, 0, 0],
                        accel_raw: [0, 0, 1638],
                        tilt_raw: [0, 0, 1000],
                    })
                    .expect("Failed to record");

                time += sensor_period_us;
            }

            // Turn 90° left
            let turn_duration_us = (turn_time_s * 1_000_000.0) as u64;
            let turn_end_time = time + turn_duration_us;
            let gyro_raw_z = (angular_speed_radps / gyro_scale) as i16;
            let turn_start_ticks = (total_distance * ticks_per_meter) as u16;

            while time < turn_end_time {
                recorder
                    .record_sensor_status(&SensorStatusMsg {
                        timestamp_us: time,
                        encoder: EncoderTicks::new(turn_start_ticks, turn_start_ticks),
                        gyro_raw: [0, 0, gyro_raw_z], // ROS REP-103: gyro_z is yaw
                        accel_raw: [0, 0, 1638],
                        tilt_raw: [0, 0, 1000],
                    })
                    .expect("Failed to record");

                time += sensor_period_us;
            }
        }

        recorder.finish().expect("Failed to finish recording");

        Self { temp_dir, bag_path }
    }

    /// Get path to the bag file.
    pub fn path(&self) -> &Path {
        &self.bag_path
    }

    /// Get the temporary directory (for additional files).
    pub fn temp_dir(&self) -> &Path {
        self.temp_dir.path()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::bag::player::BagPlayer;

    #[test]
    fn test_synthetic_5_seconds() {
        let fixture = BagTestFixture::synthetic_5_seconds();
        let player = BagPlayer::open(fixture.path()).unwrap();

        assert!(player.message_count() > 0);
        assert!(player.duration_secs() > 4.5);
        assert!(player.duration_secs() < 5.5);
    }

    #[test]
    fn test_straight_line() {
        let fixture = BagTestFixture::straight_line(1.0, 0.2, 1000.0);
        let mut player = BagPlayer::open(fixture.path()).unwrap();

        // Check first and last encoder values
        let first = player.next_immediate().unwrap().unwrap();
        let first_ticks = first.as_sensor_status().unwrap().encoder.left;

        // Read all remaining
        let mut last = first;
        while let Some(msg) = player.next_immediate().unwrap() {
            last = msg;
        }

        let last_ticks = last.as_sensor_status().unwrap().encoder.left;

        // Should have traveled ~1m = 1000 ticks
        assert!(last_ticks > first_ticks);
        assert!((last_ticks as f32 - 1000.0).abs() < 100.0);
    }

    #[test]
    fn test_rotation_in_place() {
        let fixture = BagTestFixture::rotation_in_place(
            std::f32::consts::TAU, // 360°
            1.0,                   // 1 rad/s
            0.001,                 // gyro scale
        );

        let mut player = BagPlayer::open(fixture.path()).unwrap();

        // Check gyro values are non-zero
        let msg = player.next_immediate().unwrap().unwrap();
        let status = msg.as_sensor_status().unwrap();
        // ROS REP-103: gyro_raw[2] is the yaw (Z) axis - should be non-zero for rotation
        assert!(status.gyro_raw[2] != 0); // Should have rotation
        assert_eq!(status.encoder.left, status.encoder.right); // No wheel motion
    }

    #[test]
    fn test_square_path() {
        let fixture = BagTestFixture::square_path(
            1.0,    // 1m sides
            0.2,    // 0.2 m/s
            0.5,    // 0.5 rad/s
            1000.0, // 1000 ticks/m
            0.001,  // gyro scale
        );

        let player = BagPlayer::open(fixture.path()).unwrap();

        // Should have decent duration (4*1m @ 0.2m/s + 4*90° @ 0.5rad/s)
        // ~20s + ~6.3s = ~26s
        assert!(player.duration_secs() > 20.0);
    }
}
