//! 3iRobotix Delta-2D Lidar protocol implementation
//!
//! Packet format:
//! - Chunk Header (1 byte)
//! - Chunk Length (2 bytes, big-endian)
//! - Chunk Version (1 byte)
//! - Chunk Type (1 byte)
//! - Command Type (1 byte): 0xAE (health) or 0xAD (measurement)
//! - Payload Length (2 bytes, big-endian)
//! - Payload Data (variable length)
//! - Payload CRC (2 bytes, big-endian)

use crate::error::{Error, Result};
use crate::types::{LidarPoint, LidarScan};

/// Command types sent by the Lidar
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandType {
    /// Health check data (0xAE)
    Health = 0xAE,
    /// Measurement data with angles and distances (0xAD)
    Measurement = 0xAD,
}

impl CommandType {
    /// Parse command type from byte value
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0xAE => Some(CommandType::Health),
            0xAD => Some(CommandType::Measurement),
            _ => None,
        }
    }
}

/// Packet header received from the Lidar
#[derive(Debug)]
pub struct PacketHeader {
    #[allow(dead_code)]
    chunk_header: u8,
    #[allow(dead_code)]
    chunk_length: u16,
    #[allow(dead_code)]
    chunk_version: u8,
    #[allow(dead_code)]
    chunk_type: u8,
    pub command_type: CommandType,
    pub payload_length: u16,
}

impl PacketHeader {
    /// Parse header from 8-byte buffer
    pub fn parse(buf: &[u8; 8]) -> Result<Self> {
        let command_type = CommandType::from_u8(buf[5]).ok_or_else(|| {
            Error::InvalidPacket(format!("Unknown command type: 0x{:02X}", buf[5]))
        })?;

        Ok(PacketHeader {
            chunk_header: buf[0],
            chunk_length: u16::from_be_bytes([buf[1], buf[2]]),
            chunk_version: buf[3],
            chunk_type: buf[4],
            command_type,
            payload_length: u16::from_be_bytes([buf[6], buf[7]]),
        })
    }
}

/// Parse measurement data from payload
///
/// Format:
/// - Motor RPM (1 byte) * 3
/// - Offset Angle (2 bytes BE) * 0.01
/// - Start Angle (2 bytes BE) * 0.01
/// - Measurement data (3 bytes per measurement):
///   - Signal quality (1 byte)
///   - Distance (2 bytes BE) * 0.25mm
pub fn parse_measurement(payload: &[u8]) -> Result<LidarScan> {
    if payload.len() < 5 {
        return Err(Error::InvalidPacket(
            "Payload too short for measurement".into(),
        ));
    }

    let _motor_rpm = (payload[0] as u16) * 3;
    let _offset_angle = u16::from_be_bytes([payload[1], payload[2]]) as f32 * 0.01;
    let start_angle = u16::from_be_bytes([payload[3], payload[4]]) as f32 * 0.01;

    let sample_count = (payload.len() - 5) / 3;
    let mut points = Vec::with_capacity(sample_count);

    for i in 0..sample_count {
        let base = 5 + i * 3;
        if base + 3 > payload.len() {
            break;
        }

        let signal_quality = payload[base];
        let distance_mm = u16::from_be_bytes([payload[base + 1], payload[base + 2]]) as f32 * 0.25;
        let distance_m = distance_mm / 1000.0;

        let angle_deg = start_angle + (i as f32) * (360.0 / (16.0 * sample_count as f32));
        let angle_rad = angle_deg.to_radians();

        points.push(LidarPoint {
            angle: angle_rad,
            distance: distance_m,
            quality: signal_quality,
        });
    }

    let timestamp_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64;

    Ok(LidarScan {
        points,
        timestamp_ms: Some(timestamp_ms),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_type_parse() {
        assert_eq!(CommandType::from_u8(0xAE), Some(CommandType::Health));
        assert_eq!(CommandType::from_u8(0xAD), Some(CommandType::Measurement));
        assert_eq!(CommandType::from_u8(0xFF), None);
    }

    #[test]
    fn test_header_parse() {
        let buf = [0xA5, 0x00, 0x10, 0x01, 0x02, 0xAD, 0x00, 0x08];
        let header = PacketHeader::parse(&buf).unwrap();

        assert_eq!(header.command_type, CommandType::Measurement);
        assert_eq!(header.payload_length, 8);
    }

    #[test]
    fn test_measurement_parse() {
        // Minimum valid payload: motor_rpm + offset + start + one measurement
        let payload = vec![
            10, // Motor RPM = 10 * 3 = 30 RPM
            0x00, 0x64, // Offset angle = 100 * 0.01 = 1.0°
            0x01, 0x2C, // Start angle = 300 * 0.01 = 3.0°
            255,  // Signal quality
            0x04, 0x00, // Distance = 1024 * 0.25 = 256mm
        ];

        let scan = parse_measurement(&payload).unwrap();

        assert_eq!(scan.points.len(), 1);
        assert!(scan.timestamp_ms.is_some());
        assert_eq!(scan.points[0].quality, 255);
        assert!((scan.points[0].distance - 0.256).abs() < 0.001); // 256mm = 0.256m
    }
}
