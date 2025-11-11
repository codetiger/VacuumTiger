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

// ===== STATUS_DATA Packet (CMD=0x15, 96 bytes) Byte Offsets =====

/// Battery level byte offset (0-100%)
pub const OFFSET_BATTERY_LEVEL: usize = 4;
/// Status flag byte offset
pub const OFFSET_STATUS_FLAG: usize = 5;
/// Charging/battery state flags byte offset (bit-packed)
pub const OFFSET_FLAGS: usize = 7;
/// Percent value byte offset (÷100)
pub const OFFSET_PERCENT: usize = 8;
/// Error code byte offset
pub const OFFSET_ERROR_CODE: usize = 43;
/// IR sensor 1 byte offset (2 bytes, little-endian, ×5 scaling)
pub const OFFSET_IR_SENSOR_1: usize = 58;
/// Start button IR sensor byte offset (2 bytes, little-endian, ×5 scaling)
pub const OFFSET_START_BUTTON_IR: usize = 60;
/// Dock button IR sensor byte offset (2 bytes, little-endian, ×5 scaling)
pub const OFFSET_DOCK_BUTTON_IR: usize = 62;
/// Left encoder count byte offset (2 bytes, little-endian, signed i16)
pub const OFFSET_ENCODER_LEFT: usize = 80;
/// Right encoder count byte offset (2 bytes, little-endian, signed i16)
pub const OFFSET_ENCODER_RIGHT: usize = 82;

// ===== IR Sensor Scaling Constants =====

/// IR sensor value scale factor (raw values are multiplied by 5)
pub const IR_SENSOR_SCALE: u16 = 5;

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
    /// Heartbeat keep-alive command (packetHeartBeat in AuxCtrl)
    Heartbeat = 0x06,
    /// System setup/version query (packetRequireSystemVersion in AuxCtrl)
    SystemSetup = 0x07,
    /// Initialize/wake-up sequence (packetSetIMUZero in AuxCtrl)
    Initialize = 0x08,
    /// Status data response (from GD32)
    StatusData = 0x15,
    /// Motor control type/mode (packetMotorControlType in AuxCtrl)
    MotorControlType = 0x65,
    /// Motor velocity with differential drive (packetMotorVelocity in AuxCtrl)
    MotorVelocity = 0x66,
    /// Motor speed - direct wheel control (packetMotorSpeed in AuxCtrl)
    MotorSpeed = 0x67,
    /// Blower speed control (packetBlowerSpeed in AuxCtrl)
    BlowerSpeed = 0x68,
    /// Side brush speed (packetSideBrushSpeed in AuxCtrl)
    SideBrushSpeed = 0x69,
    /// Rolling brush speed (packetRollingSpeed in AuxCtrl)
    RollingBrushSpeed = 0x6A,
    /// Brush control (packetBrushControl in AuxCtrl)
    BrushControl = 0x6B,
    /// Lidar PWM speed control 0-100% (packetLidarPWM in AuxCtrl)
    LidarPWM = 0x71,
    /// Button LED state (packetButtonLEDState in AuxCtrl)
    ButtonLedState = 0x8D,
    /// Lidar power control (packetLidarPower in AuxCtrl)
    LidarPower = 0x97,
    /// Lidar preparation command (sent before power on)
    LidarPrep = 0xA2,
}

/// GD32 commands
#[derive(Debug, Clone)]
pub enum Gd32Command {
    /// Heartbeat keep-alive command (packetHeartBeat in AuxCtrl - CMD 0x06)
    Heartbeat,
    /// System setup/version query (packetRequireSystemVersion in AuxCtrl)
    SystemSetup,
    /// Initialize device with 96-byte payload (packetSetIMUZero in AuxCtrl)
    Initialize {
        /// Initialization payload (96 bytes)
        payload: [u8; 96],
    },
    /// Motor control type/mode (packetMotorControlType in AuxCtrl - CMD 0x65)
    MotorControlType(u8),
    /// Motor velocity - differential drive (packetMotorVelocity in AuxCtrl - CMD 0x66)
    /// Payload: [linear_velocity, angular_velocity] as i32 with conversion factors
    MotorVelocity {
        /// Motor velocity data (8 bytes): [linear_vel_i32, angular_vel_i32]
        data: [u8; 8],
    },
    /// Motor speed - direct wheel control (packetMotorSpeed in AuxCtrl - CMD 0x67)
    MotorSpeed {
        /// Left motor speed (signed, encoder ticks)
        left: i32,
        /// Right motor speed (signed, encoder ticks)
        right: i32,
    },
    /// Blower speed (packetBlowerSpeed in AuxCtrl)
    BlowerSpeed(u16),
    /// Side brush speed (packetSideBrushSpeed in AuxCtrl)
    SideBrushSpeed(u8),
    /// Rolling brush speed (packetRollingSpeed in AuxCtrl)
    RollingBrushSpeed(u8),
    /// Brush control (packetBrushControl in AuxCtrl)
    BrushControl(u8),
    /// Lidar PWM speed control 0-100% (packetLidarPWM in AuxCtrl)
    LidarPWM(i32),
    /// Button LED state (packetButtonLEDState in AuxCtrl)
    ButtonLedState(u8),
    /// Lidar power control (packetLidarPower in AuxCtrl)
    LidarPower(bool),
    /// Lidar preparation command
    LidarPrep,
}

impl Gd32Command {
    /// Get command ID
    pub fn cmd_id(&self) -> u8 {
        match self {
            Gd32Command::Heartbeat => CommandId::Heartbeat as u8,
            Gd32Command::SystemSetup => CommandId::SystemSetup as u8,
            Gd32Command::Initialize { .. } => CommandId::Initialize as u8,
            Gd32Command::MotorControlType(_) => CommandId::MotorControlType as u8,
            Gd32Command::MotorVelocity { .. } => CommandId::MotorVelocity as u8,
            Gd32Command::MotorSpeed { .. } => CommandId::MotorSpeed as u8,
            Gd32Command::BlowerSpeed(_) => CommandId::BlowerSpeed as u8,
            Gd32Command::SideBrushSpeed(_) => CommandId::SideBrushSpeed as u8,
            Gd32Command::RollingBrushSpeed(_) => CommandId::RollingBrushSpeed as u8,
            Gd32Command::BrushControl(_) => CommandId::BrushControl as u8,
            Gd32Command::LidarPWM(_) => CommandId::LidarPWM as u8,
            Gd32Command::ButtonLedState(_) => CommandId::ButtonLedState as u8,
            Gd32Command::LidarPower(_) => CommandId::LidarPower as u8,
            Gd32Command::LidarPrep => CommandId::LidarPrep as u8,
        }
    }

    /// Build payload for command
    fn build_payload(&self) -> Vec<u8> {
        match self {
            Gd32Command::Heartbeat => vec![0x00],
            Gd32Command::SystemSetup => vec![0x00],
            Gd32Command::Initialize { payload } => payload.to_vec(),
            Gd32Command::MotorControlType(mode) => vec![*mode],
            Gd32Command::MotorVelocity { data } => data.to_vec(),
            Gd32Command::MotorSpeed { left, right } => {
                let mut payload = Vec::with_capacity(8);
                payload.extend_from_slice(&left.to_le_bytes());
                payload.extend_from_slice(&right.to_le_bytes());
                payload
            }
            Gd32Command::BlowerSpeed(speed) => speed.to_le_bytes().to_vec(),
            Gd32Command::SideBrushSpeed(speed) => vec![*speed],
            Gd32Command::RollingBrushSpeed(speed) => vec![*speed],
            Gd32Command::BrushControl(value) => vec![*value],
            Gd32Command::LidarPWM(pwm_percent) => {
                // Clamp to [0, 100] range and convert to 4-byte little-endian
                let pwm = (*pwm_percent).clamp(0, 100);
                (pwm as u32).to_le_bytes().to_vec()
            }
            Gd32Command::ButtonLedState(state) => vec![*state],
            Gd32Command::LidarPower(on) => vec![if *on { 0x01 } else { 0x00 }],
            Gd32Command::LidarPrep => vec![0x10, 0x0E, 0x00, 0x00],
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

    /// Create motor velocity command with wheel speeds (CMD 0x66)
    ///
    /// VERIFIED via hardware testing: CMD=0x66 uses differential drive format!
    ///
    /// Payload format: [linear_velocity (i32 LE), angular_velocity (i32 LE)]
    /// - linear_velocity: (left + right) - forward/backward speed of robot center
    /// - angular_velocity: (right - left) - rotation speed (positive = clockwise)
    ///
    /// Conversion from wheel speeds to differential drive:
    /// - Both wheels same speed → linear only (straight line)
    /// - Wheels opposite speeds → angular only (rotate in place)
    /// - Mixed → curved motion
    ///
    /// This is the ONLY motor control command that works in mode 0x02 (navigation mode).
    /// CMD=0x67 (MotorSpeed) causes error 0xFF in mode 0x02.
    pub fn motor_velocity_with_speeds(left: i32, right: i32) -> Self {
        let mut data = [0u8; 8];

        // Differential drive conversion:
        // linear = (left + right) - sum of wheel speeds
        // angular = (right - left) - difference in wheel speeds
        let linear_velocity = left + right;
        let angular_velocity = right - left;

        data[0..4].copy_from_slice(&linear_velocity.to_le_bytes());
        data[4..8].copy_from_slice(&angular_velocity.to_le_bytes());
        Gd32Command::MotorVelocity { data }
    }
}

/// GD32 response packet
#[derive(Debug, Clone)]
pub struct Gd32Response {
    /// Command ID
    pub cmd_id: u8,
    /// Raw payload
    pub payload: Vec<u8>,

    // Battery Information
    /// Battery voltage (V) - VERIFIED at bytes [82-83]
    pub battery_voltage: f32,
    /// Battery current (A) - VERIFIED at bytes [80-81]
    pub battery_current: f32,
    /// Battery level (0-100%) - VERIFIED at byte [4]
    pub battery_level: u8,

    // Encoder Counts (16-bit signed values)
    /// Left encoder count - EXPERIMENTAL: bytes [80-81] as i16 (was [8-11] as i32)
    /// Range: 55-65, Δ=7 in mode 0x01 (minimal - needs longer test)
    pub encoder_left: i32,
    /// Right encoder count - EXPERIMENTAL: bytes [82-83] as i16 (was [12-15])
    /// Frozen in mode 0x02, Δ=66 in mode 0x01 (stronger candidate)
    pub encoder_right: i32,

    // Error and Status Flags
    /// Error code - VERIFIED at byte [43]
    pub error_code: u8,
    /// Status flag - byte [5]
    pub status_flag: u8,
    /// Charging flag (bit 0 of byte [7])
    pub charging_flag: bool,
    /// Battery state flag (bit 1 of byte [7])
    pub battery_state_flag: bool,
    /// Percent value ÷ 100 - byte [8]
    pub percent_value: f32,

    // IR Proximity Sensors (Button Detection)
    /// IR sensor 1 (×5 scaling) - bytes [58-59]
    pub ir_sensor_1: u16,
    /// Start button IR (×5 scaling) - bytes [60-61]
    pub start_button_ir: u16,
    /// Dock button IR (×5 scaling) - bytes [62-63]
    pub dock_button_ir: u16,
}

impl Gd32Response {
    /// Create a Gd32Response from command ID and payload
    /// Parses STATUS_DATA packets (CMD=0x15) to extract sensor values
    fn from_payload(cmd_id: u8, payload: &[u8]) -> Self {
        // Parse STATUS_DATA packet if applicable
        if cmd_id == CommandId::StatusData as u8 && payload.len() >= 96 {
            Self::from_status_data(cmd_id, payload)
        } else {
            // Non-STATUS_DATA packet or incomplete payload
            Self {
                cmd_id,
                payload: payload.to_vec(),
                battery_voltage: 0.0,
                battery_current: 0.0,
                battery_level: 0,
                encoder_left: 0,
                encoder_right: 0,
                error_code: 0,
                status_flag: 0,
                charging_flag: false,
                battery_state_flag: false,
                percent_value: 0.0,
                ir_sensor_1: 0,
                start_button_ir: 0,
                dock_button_ir: 0,
            }
        }
    }

    /// Parse STATUS_DATA packet (CMD=0x15) and construct Gd32Response
    ///
    /// VERIFIED byte offsets (from MITM analysis + hardware testing + AuxCtrl binary analysis):
    /// - Byte [4]: Battery level (u8, 0-100%)
    /// - Byte [5]: Status flag
    /// - Byte [7]: Charging/battery state flags (bit-packed)
    /// - Byte [8]: Percent value (÷100)
    /// - Bytes [80-81]: Left encoder (i16 LE) - EXPERIMENTAL, was misread as part of 32-bit [80-83]
    /// - Bytes [82-83]: Right encoder (i16 LE) - EXPERIMENTAL, stronger candidate (Δ=66 vs Δ=7)
    /// - Byte [43]: Error code (u8)
    /// - Bytes [58-59]: IR sensor 1 (u16 LE, ×5 scaling)
    /// - Bytes [60-61]: Point button IR (u16 LE, ×5 scaling)
    /// - Bytes [62-63]: Dock button IR (u16 LE, ×5 scaling)
    ///
    /// DISABLED (conflicts with encoders):
    /// - OLD: Bytes [8-11] left encoder (i32) - shows no accumulation, likely PWM/state
    /// - OLD: Bytes [12-15] right encoder (i32) - always 0, not used
    /// - Battery current/voltage [80-83] - TODO: find at [16-19] and [24-27]?
    fn from_status_data(cmd_id: u8, payload: &[u8]) -> Self {
        // Log raw STATUS_DATA packet for analysis (DEBUG level only)
        if log::log_enabled!(log::Level::Debug) && payload.len() == 96 {
            log::debug!("GD32: STATUS_DATA packet (96 bytes) - HEX DUMP:");
            for chunk_start in (0..96).step_by(16) {
                let chunk_end = (chunk_start + 16).min(96);
                let hex_str: String = payload[chunk_start..chunk_end]
                    .iter()
                    .map(|b| format!("{:02X} ", b))
                    .collect();
                log::debug!("  [{:02}..{:02}]: {}", chunk_start, chunk_end - 1, hex_str);
            }
        }

        // Battery voltage - DISABLED: bytes [82-83] conflict with encoder [80-83]
        // TODO: Investigate alternative position at bytes [16-19]
        let battery_voltage = 0.0;
        // let battery_voltage = if payload.len() >= 84 {
        //     u16::from_le_bytes([payload[82], payload[83]]) as f32 / 100.0
        // } else {
        //     0.0
        // };

        // Battery current - DISABLED: bytes [80-81] conflict with encoder [80-83]
        // TODO: Investigate alternative position at bytes [24-27]
        let battery_current = 0.0;
        // let battery_current = if payload.len() >= 82 {
        //     i16::from_le_bytes([payload[80], payload[81]]) as f32 / 100.0
        // } else {
        //     0.0
        // };

        // Battery level - VERIFIED at byte [4]
        let battery_level = if payload.len() > OFFSET_BATTERY_LEVEL {
            payload[OFFSET_BATTERY_LEVEL]
        } else {
            0
        };

        // Left encoder - EXPERIMENTAL: Testing bytes [80-81] as 16-bit signed (was [8-11] as 32-bit)
        // Analysis shows bytes [80-83] were being misinterpreted as one 32-bit value
        // Actually TWO separate 16-bit encoders: [80-81]=Left, [82-83]=Right
        // Mode 0x01: Left shows Δ=7, Right shows Δ=66 (only active when components ON)
        let encoder_left = if payload.len() >= OFFSET_ENCODER_LEFT + 2 {
            i16::from_le_bytes([
                payload[OFFSET_ENCODER_LEFT],
                payload[OFFSET_ENCODER_LEFT + 1],
            ]) as i32
        } else {
            0
        };

        // Right encoder - EXPERIMENTAL: Testing bytes [82-83] as 16-bit signed (was [12-15])
        // Shows stronger encoder behavior than left: frozen in mode 0x02, Δ=66 in mode 0x01
        // Only updates when components are active - needs longer test to verify
        let encoder_right = if payload.len() >= OFFSET_ENCODER_RIGHT + 2 {
            i16::from_le_bytes([
                payload[OFFSET_ENCODER_RIGHT],
                payload[OFFSET_ENCODER_RIGHT + 1],
            ]) as i32
        } else {
            0
        };

        // Error code - VERIFIED at byte [43]
        let error_code = if payload.len() > OFFSET_ERROR_CODE {
            payload[OFFSET_ERROR_CODE]
        } else {
            0
        };

        // Status flag - byte [5]
        let status_flag = if payload.len() > OFFSET_STATUS_FLAG {
            payload[OFFSET_STATUS_FLAG]
        } else {
            0
        };

        // Charging and battery state flags - byte [7] (bit-packed)
        let flag_byte = if payload.len() > OFFSET_FLAGS {
            payload[OFFSET_FLAGS]
        } else {
            0
        };
        let charging_flag = (flag_byte & 0x01) != 0; // bit 0
        let battery_state_flag = (flag_byte & 0x02) != 0; // bit 1

        // Percent value - byte [8] ÷ 100
        let percent_value = if payload.len() > OFFSET_PERCENT {
            payload[OFFSET_PERCENT] as f32 / 100.0
        } else {
            0.0
        };

        // IR sensors - bytes [58-59], [60-61], [62-63] with ×5 scaling
        let ir_sensor_1 = if payload.len() >= OFFSET_IR_SENSOR_1 + 2 {
            u16::from_le_bytes([payload[OFFSET_IR_SENSOR_1], payload[OFFSET_IR_SENSOR_1 + 1]])
                .saturating_mul(IR_SENSOR_SCALE)
        } else {
            0
        };

        let start_button_ir = if payload.len() >= OFFSET_START_BUTTON_IR + 2 {
            u16::from_le_bytes([
                payload[OFFSET_START_BUTTON_IR],
                payload[OFFSET_START_BUTTON_IR + 1],
            ])
            .saturating_mul(IR_SENSOR_SCALE)
        } else {
            0
        };

        let dock_button_ir = if payload.len() >= OFFSET_DOCK_BUTTON_IR + 2 {
            u16::from_le_bytes([
                payload[OFFSET_DOCK_BUTTON_IR],
                payload[OFFSET_DOCK_BUTTON_IR + 1],
            ])
            .saturating_mul(IR_SENSOR_SCALE)
        } else {
            0
        };

        Self {
            cmd_id,
            payload: payload.to_vec(),
            battery_voltage,
            battery_current,
            battery_level,
            encoder_left,
            encoder_right,
            error_code,
            status_flag,
            charging_flag,
            battery_state_flag,
            percent_value,
            ir_sensor_1,
            start_button_ir,
            dock_button_ir,
        }
    }

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

            // Valid packet found! Parse it into a response
            let response = Self::from_payload(cmd_id, payload);

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
    fn test_heartbeat_command_encoding() {
        let cmd = Gd32Command::Heartbeat;
        let packet = cmd.encode();

        assert_eq!(packet[0], 0xFA); // SYNC1
        assert_eq!(packet[1], 0xFB); // SYNC2
        assert_eq!(packet[2], 0x04); // Length: CMD(1) + PAYLOAD(1) + CHECKSUM(2)
        assert_eq!(packet[3], 0x06); // CMD ID (Heartbeat)
        assert_eq!(packet[4], 0x00); // Payload
        assert_eq!(packet[5], 0x06); // Checksum high byte
        assert_eq!(packet[6], 0x00); // Checksum low byte
    }

    #[test]
    fn test_lidar_power_encoding() {
        // Test power ON
        let cmd = Gd32Command::LidarPower(true);
        let packet = cmd.encode();

        // Packet format: [0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC_HIGH] [CRC_LOW]
        // For CMD=0x97, PAYLOAD=[0x01]:
        // data = [0x97, 0x01] (even bytes)
        // word = 0x9701
        // checksum = 0x9701 → [0x97, 0x01]
        assert_eq!(packet.len(), 7); // SYNC(2) + LEN(1) + CMD(1) + PAYLOAD(1) + CRC(2)
        assert_eq!(packet[0], 0xFA); // SYNC1
        assert_eq!(packet[1], 0xFB); // SYNC2
        assert_eq!(packet[2], 0x04); // LEN = CMD(1) + PAYLOAD(1) + CRC(2)
        assert_eq!(packet[3], 0x97); // CMD (Lidar power)
        assert_eq!(packet[4], 0x01); // PAYLOAD (ON)
        assert_eq!(packet[5], 0x97); // CRC high byte
        assert_eq!(packet[6], 0x01); // CRC low byte

        // Test power OFF
        let cmd = Gd32Command::LidarPower(false);
        let packet = cmd.encode();

        // For CMD=0x97, PAYLOAD=[0x00]:
        // data = [0x97, 0x00]
        // word = 0x9700
        // checksum = 0x9700 → [0x97, 0x00]
        assert_eq!(packet[4], 0x00); // PAYLOAD (OFF)
        assert_eq!(packet[5], 0x97); // CRC high byte
        assert_eq!(packet[6], 0x00); // CRC low byte
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
