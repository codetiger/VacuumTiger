//! GD32 protocol implementation
//!
//! Packet format: [0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC]
//! CRC: cmd_id XOR all_payload_bytes XOR cmd_id

use crate::error::{Error, Result};

/// Sync byte 1
pub const SYNC1: u8 = 0xFA;
/// Sync byte 2
pub const SYNC2: u8 = 0xFB;

/// Calculate CRC for GD32 packet
/// Formula: cmd_id XOR payload[0] XOR payload[1] XOR ... XOR cmd_id
fn calculate_crc(cmd_id: u8, payload: &[u8]) -> u8 {
    let mut crc = cmd_id;
    for &byte in payload {
        crc ^= byte;
    }
    crc ^= cmd_id;
    crc
}

/// GD32 command IDs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandId {
    /// Initialize/wake-up sequence
    Initialize = 0x08,
    /// Wake/enable motors
    Wake = 0x06,
    /// Heartbeat/keep-alive
    Heartbeat = 0x66,
    /// Motor speed control
    MotorSpeed = 0x67,
    /// Blower speed control
    BlowerSpeed = 0x68,
    /// Button LED state
    ButtonLedState = 0x8D,
    /// Lidar power control
    LidarPower = 0x97,
    /// Status data response
    StatusData = 0x15,
}

/// GD32 commands
#[derive(Debug, Clone)]
pub enum Gd32Command {
    /// Initialize device with 96-byte payload
    Initialize {
        /// Initialization payload (96 bytes)
        payload: [u8; 96]
    },
    /// Wake command
    Wake,
    /// Heartbeat with 8-byte data
    Heartbeat {
        /// Heartbeat data (8 bytes)
        data: [u8; 8]
    },
    /// Motor speed command
    MotorSpeed {
        /// Left motor speed (encoder ticks)
        left: i32,
        /// Right motor speed (encoder ticks)
        right: i32
    },
    /// Blower speed
    BlowerSpeed(u16),
    /// Button LED state
    ButtonLedState(u8),
    /// Lidar power control
    LidarPower(bool),
}

impl Gd32Command {
    /// Get command ID
    pub fn cmd_id(&self) -> u8 {
        match self {
            Gd32Command::Initialize { .. } => CommandId::Initialize as u8,
            Gd32Command::Wake => CommandId::Wake as u8,
            Gd32Command::Heartbeat { .. } => CommandId::Heartbeat as u8,
            Gd32Command::MotorSpeed { .. } => CommandId::MotorSpeed as u8,
            Gd32Command::BlowerSpeed(_) => CommandId::BlowerSpeed as u8,
            Gd32Command::ButtonLedState(_) => CommandId::ButtonLedState as u8,
            Gd32Command::LidarPower(_) => CommandId::LidarPower as u8,
        }
    }

    /// Build payload for command
    fn build_payload(&self) -> Vec<u8> {
        match self {
            Gd32Command::Initialize { payload } => payload.to_vec(),
            Gd32Command::Wake => vec![0x00],
            Gd32Command::Heartbeat { data } => data.to_vec(),
            Gd32Command::MotorSpeed { left, right } => {
                let mut payload = Vec::with_capacity(8);
                payload.extend_from_slice(&left.to_le_bytes());
                payload.extend_from_slice(&right.to_le_bytes());
                payload
            }
            Gd32Command::BlowerSpeed(speed) => speed.to_le_bytes().to_vec(),
            Gd32Command::ButtonLedState(state) => vec![*state],
            Gd32Command::LidarPower(on) => vec![if *on { 0x01 } else { 0x00 }],
        }
    }

    /// Encode command into packet bytes
    pub fn encode(&self) -> Vec<u8> {
        let cmd_id = self.cmd_id();
        let payload = self.build_payload();
        let crc = calculate_crc(cmd_id, &payload);

        let mut packet = Vec::with_capacity(4 + payload.len());
        packet.push(SYNC1);
        packet.push(SYNC2);
        packet.push((payload.len() + 2) as u8); // Length: CMD + PAYLOAD + CRC
        packet.push(cmd_id);
        packet.extend_from_slice(&payload);
        packet.push(crc);

        packet
    }

    /// Create default initialization command
    /// Payload: repeating pattern of [0x20, 0x08, 0x08] × 32 times
    pub fn default_initialize() -> Self {
        let mut payload = [0u8; 96];
        for i in 0..32 {
            payload[i * 3] = 0x20;
            payload[i * 3 + 1] = 0x08;
            payload[i * 3 + 2] = 0x08;
        }
        Gd32Command::Initialize { payload }
    }

    /// Create default heartbeat command (all zeros)
    pub fn default_heartbeat() -> Self {
        Gd32Command::Heartbeat { data: [0; 8] }
    }
}

/// GD32 response packet
#[derive(Debug, Clone)]
pub struct Gd32Response {
    /// Command ID
    pub cmd_id: u8,
    /// Raw payload
    pub payload: Vec<u8>,
    /// Battery voltage (V)
    pub battery_voltage: f32,
    /// Battery current (A)
    pub battery_current: f32,
    /// Battery level (0-100%)
    pub battery_level: u8,
    /// Left encoder count
    pub encoder_left: i32,
    /// Right encoder count
    pub encoder_right: i32,
    /// Error code
    pub error_code: u8,
}

impl Gd32Response {
    /// Decode response from packet bytes
    pub fn decode(data: &[u8]) -> Result<Self> {
        if data.len() < 6 {
            return Err(Error::InvalidPacket("Packet too short".into()));
        }

        // Verify sync bytes
        if data[0] != SYNC1 || data[1] != SYNC2 {
            return Err(Error::InvalidPacket("Invalid sync bytes".into()));
        }

        let length = data[2] as usize;
        if data.len() < 3 + length {
            return Err(Error::InvalidPacket("Incomplete packet".into()));
        }

        let cmd_id = data[3];
        let payload_len = length.saturating_sub(2); // Subtract CMD and CRC
        let payload = &data[4..4 + payload_len];
        let received_crc = data[4 + payload_len];

        // Verify CRC
        let expected_crc = calculate_crc(cmd_id, payload);
        if received_crc != expected_crc {
            return Err(Error::ChecksumError {
                expected: expected_crc,
                actual: received_crc,
            });
        }

        // Parse status packet (CMD=0x15)
        let (
            battery_voltage,
            battery_current,
            battery_level,
            encoder_left,
            encoder_right,
            error_code,
        ) = if cmd_id == CommandId::StatusData as u8 && payload.len() >= 96 {
            Self::parse_status_packet(payload)
        } else {
            (0.0, 0.0, 0, 0, 0, 0)
        };

        Ok(Gd32Response {
            cmd_id,
            payload: payload.to_vec(),
            battery_voltage,
            battery_current,
            battery_level,
            encoder_left,
            encoder_right,
            error_code,
        })
    }

    /// Parse status packet fields
    /// Note: These byte offsets are placeholders and need hardware verification
    fn parse_status_packet(payload: &[u8]) -> (f32, f32, u8, i32, i32, u8) {
        // Battery data (offsets need verification)
        let battery_voltage = if payload.len() >= 2 {
            u16::from_le_bytes([payload[0], payload[1]]) as f32 / 100.0
        } else {
            0.0
        };

        let battery_current = if payload.len() >= 4 {
            i16::from_le_bytes([payload[2], payload[3]]) as f32 / 100.0
        } else {
            0.0
        };

        let battery_level = if payload.len() >= 5 { payload[4] } else { 0 };

        // Encoder data (offsets need verification)
        let encoder_left = if payload.len() >= 12 {
            i32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]])
        } else {
            0
        };

        let encoder_right = if payload.len() >= 16 {
            i32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]])
        } else {
            0
        };

        // Error code (usually last byte)
        let error_code = if payload.len() >= 96 { payload[95] } else { 0 };

        (
            battery_voltage,
            battery_current,
            battery_level,
            encoder_left,
            encoder_right,
            error_code,
        )
    }

    /// Check if this is a status packet
    pub fn is_status_packet(&self) -> bool {
        self.cmd_id == CommandId::StatusData as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc_calculation() {
        // Example from docs: CMD=0x06, PAYLOAD=[0x00], CRC should be 0x00
        // CRC = 0x06 XOR 0x00 XOR 0x06 = 0x00
        let crc = calculate_crc(0x06, &[0x00]);
        assert_eq!(crc, 0x00);
    }

    #[test]
    fn test_wake_command_encoding() {
        let cmd = Gd32Command::Wake;
        let packet = cmd.encode();

        assert_eq!(packet[0], 0xFA); // SYNC1
        assert_eq!(packet[1], 0xFB); // SYNC2
        assert_eq!(packet[2], 0x03); // Length: CMD + PAYLOAD + CRC
        assert_eq!(packet[3], 0x06); // CMD ID
        assert_eq!(packet[4], 0x00); // Payload
        assert_eq!(packet[5], calculate_crc(0x06, &[0x00])); // CRC
    }

    #[test]
    fn test_lidar_power_encoding() {
        let cmd = Gd32Command::LidarPower(true);
        let packet = cmd.encode();

        assert_eq!(packet[0], 0xFA);
        assert_eq!(packet[1], 0xFB);
        assert_eq!(packet[3], 0x97); // Lidar power CMD
        assert_eq!(packet[4], 0x01); // ON
    }

    #[test]
    fn test_default_initialize() {
        let cmd = Gd32Command::default_initialize();
        let packet = cmd.encode();

        assert_eq!(packet.len(), 101); // 96 payload + SYNC1 + SYNC2 + LEN + CMD + CRC
        assert_eq!(packet[0], 0xFA);
        assert_eq!(packet[1], 0xFB);
        assert_eq!(packet[3], 0x08); // Initialize CMD

        // Check pattern
        if let Gd32Command::Initialize { payload } = cmd {
            assert_eq!(payload[0], 0x20);
            assert_eq!(payload[1], 0x08);
            assert_eq!(payload[2], 0x08);
        }
    }

    #[test]
    fn test_response_decode() {
        // Create a valid packet: [0xFA 0xFB] [0x03] [0x15] [0x00] [CRC]
        let cmd_id = 0x15;
        let payload = [0x00];
        let crc = calculate_crc(cmd_id, &payload);
        let packet = vec![0xFA, 0xFB, 0x03, cmd_id, payload[0], crc];

        let response = Gd32Response::decode(&packet).unwrap();
        assert_eq!(response.cmd_id, 0x15);
        assert!(response.is_status_packet());
    }

    #[test]
    fn test_response_decode_invalid_crc() {
        let packet = vec![0xFA, 0xFB, 0x03, 0x15, 0x00, 0xFF]; // Wrong CRC
        let result = Gd32Response::decode(&packet);
        assert!(matches!(result, Err(Error::ChecksumError { .. })));
    }
}
