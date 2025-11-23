//! Delta-2D Lidar Protocol Implementation
//! Packet format: [Header (8 bytes)] [Payload] [CRC (2 bytes)]

use crate::error::{Error, Result};
use std::io::Read;

/// Lidar scan point
#[derive(Debug, Clone)]
pub struct LidarPoint {
    pub angle: f32,    // radians
    pub distance: f32, // meters
    pub quality: u8,
}

/// Complete lidar scan
#[derive(Debug, Clone)]
pub struct LidarScan {
    pub points: Vec<LidarPoint>,
    pub timestamp_us: u64,
}

/// Command types from lidar
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CommandType {
    Health = 0xAE,
    Measurement = 0xAD,
    Unknown = 0xFF,
}

impl From<u8> for CommandType {
    fn from(value: u8) -> Self {
        match value {
            0xAE => CommandType::Health,
            0xAD => CommandType::Measurement,
            _ => CommandType::Unknown,
        }
    }
}

/// Packet reader for Delta-2D lidar
pub struct Delta2DPacketReader {
    buffer: Vec<u8>,
}

impl Delta2DPacketReader {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(2048),
        }
    }

    /// Try to read and parse a scan from the port
    pub fn read_scan<R: Read>(&mut self, port: &mut R) -> Result<Option<LidarScan>> {
        // Read available bytes
        let mut temp_buf = [0u8; 512];
        match port.read(&mut temp_buf) {
            Ok(0) => return Ok(None),
            Ok(n) => {
                self.buffer.extend_from_slice(&temp_buf[..n]);
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                return Ok(None);
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                return Ok(None);
            }
            Err(e) => return Err(Error::Io(e)),
        }

        // Try to parse a scan
        self.try_parse_scan()
    }

    fn try_parse_scan(&mut self) -> Result<Option<LidarScan>> {
        // Need at least header (8 bytes)
        if self.buffer.len() < 8 {
            return Ok(None);
        }

        // Find sync byte 0xA5
        let Some(sync_idx) = self.buffer.iter().position(|&b| b == 0xA5) else {
            // No sync found, clear buffer if too large
            if self.buffer.len() > 4096 {
                self.buffer.clear();
            }
            return Ok(None);
        };

        // Remove bytes before sync
        if sync_idx > 0 {
            self.buffer.drain(0..sync_idx);
        }

        // Check if we have complete header
        if self.buffer.len() < 8 {
            return Ok(None);
        }

        // Parse header
        // Byte 0: 0xA5 (sync)
        // Bytes 1-2: Length (big-endian)
        // Byte 3: Version
        // Byte 4: Chunk type
        // Byte 5: Command type
        // Bytes 6-7: Payload length (big-endian)

        let _header_len = ((self.buffer[1] as u16) << 8) | (self.buffer[2] as u16);
        let cmd_type = CommandType::from(self.buffer[5]);
        let payload_len = ((self.buffer[6] as u16) << 8) | (self.buffer[7] as u16);

        // Total packet size: header (8) + payload + CRC (2)
        let total_len = 8 + payload_len as usize + 2;

        // Wait for complete packet
        if self.buffer.len() < total_len {
            return Ok(None);
        }

        // Extract payload
        let payload = &self.buffer[8..8 + payload_len as usize];

        // Parse based on command type
        let scan = if cmd_type == CommandType::Measurement && payload_len > 5 {
            Some(self.parse_measurement(payload)?)
        } else {
            None
        };

        // Remove processed packet
        self.buffer.drain(0..total_len);

        Ok(scan)
    }

    fn parse_measurement(&self, payload: &[u8]) -> Result<LidarScan> {
        let mut points = Vec::with_capacity(100);

        if payload.len() < 5 {
            return Ok(LidarScan {
                points,
                timestamp_us: current_timestamp_us(),
            });
        }

        // Byte 0: Motor RPM * 3
        // Bytes 1-2: Offset angle (BE) * 0.01 degrees
        // Bytes 3-4: Start angle (BE) * 0.01 degrees

        let _motor_rpm = payload[0] as u16 * 3;
        let _offset_angle = ((payload[1] as u16) << 8) | (payload[2] as u16);
        let start_angle_raw = ((payload[3] as u16) << 8) | (payload[4] as u16);

        // Parse measurement points (3 bytes each)
        let mut i = 5;
        let mut point_index = 0;

        while i + 3 <= payload.len() {
            let quality = payload[i];
            let distance_raw = ((payload[i + 1] as u16) << 8) | (payload[i + 2] as u16);

            // Convert to physical units
            // Angle: start_angle + (point_index * angle_increment)
            // Distance: raw * 0.25mm = raw * 0.00025m
            let angle_deg = (start_angle_raw as f32 * 0.01) + (point_index as f32 * 1.0);
            let angle_rad = angle_deg.to_radians();
            let distance_m = distance_raw as f32 * 0.00025;

            // Only add valid points (distance > 0, quality > 0)
            if distance_raw > 0 && quality > 0 {
                points.push(LidarPoint {
                    angle: angle_rad,
                    distance: distance_m,
                    quality,
                });
            }

            i += 3;
            point_index += 1;
        }

        Ok(LidarScan {
            points,
            timestamp_us: current_timestamp_us(),
        })
    }
}

fn current_timestamp_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_type_from() {
        assert_eq!(CommandType::from(0xAE), CommandType::Health);
        assert_eq!(CommandType::from(0xAD), CommandType::Measurement);
        assert_eq!(CommandType::from(0x00), CommandType::Unknown);
    }
}
