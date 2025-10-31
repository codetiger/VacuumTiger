//! GD32 protocol implementation
//!
//! Packet format: [0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CHECKSUM]
//!
//! Checksum: 16-bit big-endian word sum
//! - Sum CMD + PAYLOAD as 16-bit big-endian words
//! - If odd number of bytes, XOR the last byte
//! - Special case: CMD 0x08 (Initialize) has no checksum
//!
//! Source: Decompiled from AuxCtrl binary (0x00055d58)
//! Verified: 99.8% success rate on 14,609 MITM packets

use crate::error::{Error, Result};

/// Sync byte 1
pub const SYNC1: u8 = 0xFA;
/// Sync byte 2
pub const SYNC2: u8 = 0xFB;

/// Calculate 2-byte checksum for GD32 packet
///
/// Algorithm: 16-bit big-endian word sum
/// - Combine CMD + PAYLOAD as one data array
/// - Sum consecutive bytes as 16-bit big-endian words
/// - If odd number of bytes, XOR the last byte with checksum
/// - Return as [high_byte, low_byte]
///
/// Special case: CMD 0x08 returns None (no checksum)
fn calculate_checksum(cmd_id: u8, payload: &[u8]) -> Option<[u8; 2]> {
    // CMD 0x08 (Initialize) has no checksum
    if cmd_id == 0x08 {
        return None;
    }

    let mut data = vec![cmd_id];
    data.extend_from_slice(payload);

    let mut checksum: u16 = 0;
    let mut i = 0;

    // Sum 16-bit big-endian words
    while i + 1 < data.len() {
        let word = ((data[i] as u16) << 8) | (data[i + 1] as u16);
        checksum = checksum.wrapping_add(word);
        i += 2;
    }

    // XOR odd byte if present
    if i < data.len() {
        checksum ^= data[i] as u16;
    }

    Some([(checksum >> 8) as u8, checksum as u8])
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
        payload: [u8; 96],
    },
    /// Wake command
    Wake,
    /// Heartbeat with 8-byte data
    Heartbeat {
        /// Heartbeat data (8 bytes)
        data: [u8; 8],
    },
    /// Motor speed command
    MotorSpeed {
        /// Left motor speed (encoder ticks)
        left: i32,
        /// Right motor speed (encoder ticks)
        right: i32,
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

        let mut packet = Vec::with_capacity(6 + payload.len());
        packet.push(SYNC1);
        packet.push(SYNC2);

        // Calculate checksum
        match calculate_checksum(cmd_id, &payload) {
            Some(checksum) => {
                // Normal packet with checksum
                // LENGTH = CMD (1) + PAYLOAD + CHECKSUM (2)
                packet.push((1 + payload.len() + 2) as u8);
                packet.push(cmd_id);
                packet.extend_from_slice(&payload);
                packet.extend_from_slice(&checksum);
            }
            None => {
                // CMD 0x08: No checksum, last 2 bytes are part of data
                // LENGTH = CMD (1) + PAYLOAD (includes last 2 bytes)
                packet.push((1 + payload.len()) as u8);
                packet.push(cmd_id);
                packet.extend_from_slice(&payload);
            }
        }

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
    /// Decode response from packet bytes, searching for sync bytes
    /// Returns (bytes_consumed, response) where bytes_consumed includes any garbage before the packet
    /// Tries multiple sync positions if CRC fails
    pub fn decode_with_sync(data: &[u8]) -> Result<(usize, Self)> {
        if data.len() < 6 {
            return Err(Error::InvalidPacket("Packet too short".into()));
        }

        // Try all possible sync positions
        let mut search_offset = 0;
        while search_offset + 6 <= data.len() {
            // Search for sync bytes starting from search_offset
            let remaining_data = &data[search_offset..];
            let sync_pos = remaining_data
                .windows(2)
                .position(|w| w[0] == SYNC1 && w[1] == SYNC2);

            let relative_sync_pos = match sync_pos {
                Some(pos) => pos,
                None => {
                    // No sync bytes found in remaining data
                    // Consume all but the last byte (in case it's the first half of a sync pair)
                    let bytes_to_consume = if data.len() > search_offset + 1 {
                        data.len() - 1
                    } else {
                        search_offset
                    };
                    return Err(Error::InvalidPacket(format!(
                        "No sync bytes found, consumed {} bytes",
                        bytes_to_consume
                    )));
                }
            };

            let absolute_sync_pos = search_offset + relative_sync_pos;
            let packet_data = &data[absolute_sync_pos..];

            if packet_data.len() < 6 {
                return Err(Error::InvalidPacket("Packet too short after sync".into()));
            }

            let length = packet_data[2] as usize;
            let packet_size = 3 + length; // SYNC(2) + LEN(1) + CMD+PAYLOAD+CRC(2)

            if packet_data.len() < packet_size {
                return Err(Error::InvalidPacket("Incomplete packet".into()));
            }

            let cmd_id = packet_data[3];

            // LENGTH field = CMD (1) + PAYLOAD + CHECKSUM (2)
            // Exception: CMD 0x08 has no checksum, so LENGTH = CMD (1) + PAYLOAD
            let has_checksum = cmd_id != 0x08;

            let (payload_len, _checksum_size) = if has_checksum {
                (length - 3, 2) // length includes CMD(1) + PAYLOAD + CHECKSUM(2)
            } else {
                (length - 1, 0) // length includes CMD(1) + PAYLOAD (no checksum)
            };

            let payload = &packet_data[4..4 + payload_len];

            // Verify checksum
            let checksum_valid = if has_checksum {
                let checksum_offset = 4 + payload_len;
                let received_checksum = [
                    packet_data[checksum_offset],
                    packet_data[checksum_offset + 1],
                ];

                match calculate_checksum(cmd_id, payload) {
                    Some(expected_checksum) => {
                        if received_checksum != expected_checksum {
                            log::debug!(
                                "Checksum mismatch at offset {}, expected [{:02X} {:02X}], got [{:02X} {:02X}]",
                                absolute_sync_pos,
                                expected_checksum[0],
                                expected_checksum[1],
                                received_checksum[0],
                                received_checksum[1]
                            );
                            false
                        } else {
                            true
                        }
                    }
                    None => {
                        log::warn!("calculate_checksum returned None for non-0x08 command");
                        false
                    }
                }
            } else {
                // CMD 0x08 has no checksum - always valid
                true
            };

            if !checksum_valid {
                // Checksum failed, try next sync position
                search_offset = absolute_sync_pos + 2;
                continue;
            }

            // Valid packet found!
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

            let response = Gd32Response {
                cmd_id,
                payload: payload.to_vec(),
                battery_voltage,
                battery_current,
                battery_level,
                encoder_left,
                encoder_right,
                error_code,
            };

            // Return bytes consumed (garbage + complete packet)
            return Ok((absolute_sync_pos + packet_size, response));
        }

        // No valid packet found after trying all sync positions
        Err(Error::InvalidPacket("No valid packet found".into()))
    }

    /// Decode response from packet bytes (legacy method, expects packet at start)
    pub fn decode(data: &[u8]) -> Result<Self> {
        match Self::decode_with_sync(data) {
            Ok((_bytes_consumed, response)) => Ok(response),
            Err(e) => Err(e),
        }
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
    fn test_checksum_calculation() {
        // Example 1: CMD=0x06, PAYLOAD=[0x00] (even total bytes)
        // data = [0x06, 0x00]
        // word[0,1] = 0x0600
        // checksum = 0x0600
        // Result: [0x06, 0x00]
        let checksum = calculate_checksum(0x06, &[0x00]).unwrap();
        assert_eq!(checksum, [0x06, 0x00]);

        // Example 2: CMD=0x8D, PAYLOAD=[0x01] (even total bytes)
        // data = [0x8D, 0x01]
        // word = 0x8D01
        // Result: [0x8D, 0x01]
        let checksum = calculate_checksum(0x8D, &[0x01]).unwrap();
        assert_eq!(checksum, [0x8D, 0x01]);

        // Example 3: CMD=0x66, PAYLOAD=[0x00 × 8] (odd total bytes)
        // data = [0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        // word[0,1] = 0x6600 → checksum = 0x6600
        // word[2,3] = 0x0000 → checksum = 0x6600
        // word[4,5] = 0x0000 → checksum = 0x6600
        // word[6,7] = 0x0000 → checksum = 0x6600
        // Odd byte: 0x6600 ^ 0x00 = 0x6600
        // Result: [0x66, 0x00]
        let checksum = calculate_checksum(0x66, &[0x00; 8]).unwrap();
        assert_eq!(checksum, [0x66, 0x00]);

        // Example 4: CMD 0x08 returns None (no checksum)
        let checksum = calculate_checksum(0x08, &[0x20; 96]);
        assert_eq!(checksum, None);
    }

    #[test]
    fn test_wake_command_encoding() {
        let cmd = Gd32Command::Wake;
        let packet = cmd.encode();

        assert_eq!(packet[0], 0xFA); // SYNC1
        assert_eq!(packet[1], 0xFB); // SYNC2
        assert_eq!(packet[2], 0x04); // Length: CMD(1) + PAYLOAD(1) + CHECKSUM(2)
        assert_eq!(packet[3], 0x06); // CMD ID
        assert_eq!(packet[4], 0x00); // Payload
        assert_eq!(packet[5], 0x06); // Checksum high byte
        assert_eq!(packet[6], 0x00); // Checksum low byte
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

        // CMD 0x08 has no checksum, so packet = SYNC(2) + LEN(1) + CMD(1) + PAYLOAD(96) = 100 bytes
        assert_eq!(packet.len(), 100);
        assert_eq!(packet[0], 0xFA);
        assert_eq!(packet[1], 0xFB);
        assert_eq!(packet[2], 97); // LEN = CMD(1) + PAYLOAD(96)
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
        // Create a valid packet with checksum
        let cmd_id = 0x06;
        let payload = [0x00];
        let checksum = calculate_checksum(cmd_id, &payload).unwrap(); // [0x06, 0x00]
        let packet = vec![
            0xFA,
            0xFB,
            0x04,
            cmd_id,
            payload[0],
            checksum[0],
            checksum[1],
        ];

        let response = Gd32Response::decode(&packet).unwrap();
        assert_eq!(response.cmd_id, 0x06);
    }

    #[test]
    fn test_response_decode_invalid_checksum() {
        // Wrong checksum (should be [0x06, 0x00])
        let packet = vec![0xFA, 0xFB, 0x04, 0x06, 0x00, 0xFF, 0xFF];
        let result = Gd32Response::decode(&packet);
        assert!(result.is_err());
    }
}
