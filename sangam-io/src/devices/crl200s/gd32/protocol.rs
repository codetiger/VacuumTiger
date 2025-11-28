//! GD32 Protocol Implementation
//! Packet format: [0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC_H] [CRC_L]

use crate::devices::crl200s::constants::{
    CMD_AIR_PUMP, CMD_BUTTON_LED, CMD_CHARGER_POWER, CMD_CLIFF_IR_CONTROL, CMD_CLIFF_IR_DIRECTION,
    CMD_COMPASS_CALIBRATE, CMD_COMPASS_CALIBRATION_STATE, CMD_HEARTBEAT, CMD_IMU_CALIBRATE_STATE,
    CMD_IMU_FACTORY_CALIBRATE, CMD_INITIALIZE, CMD_LIDAR_POWER, CMD_LIDAR_PWM,
    CMD_MAIN_BOARD_POWER, CMD_MAIN_BOARD_RESTART, CMD_MAIN_BRUSH, CMD_MCU_SLEEP, CMD_MOTOR_MODE,
    CMD_MOTOR_SPEED, CMD_MOTOR_VELOCITY, CMD_PROTOCOL_SYNC, CMD_REQUEST_STM32_DATA,
    CMD_RESET_ERROR_CODE, CMD_SIDE_BRUSH, CMD_VERSION, CMD_WAKEUP_ACK, CMD_WATER_PUMP,
    MAX_BUFFER_SIZE, MIN_PACKET_SIZE, SYNC_BYTE_1, SYNC_BYTE_2,
};
use crate::error::{Error, Result};
use std::io::Read;

#[derive(Debug, Clone)]
pub struct Packet {
    pub cmd: u8,
    pub payload: Vec<u8>,
}

impl Packet {
    pub fn new(cmd: u8, payload: Vec<u8>) -> Self {
        Self { cmd, payload }
    }

    /// Build packet bytes for sending
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut packet = Vec::new();

        // Header
        packet.push(SYNC_BYTE_1);
        packet.push(SYNC_BYTE_2);

        // Length (CMD + PAYLOAD + CRC)
        let len = 1 + self.payload.len() + 2;
        packet.push(len as u8);

        // Command
        packet.push(self.cmd);

        // Payload
        packet.extend_from_slice(&self.payload);

        // Checksum (except for CMD=0x08)
        if self.cmd != CMD_INITIALIZE {
            let crc = calculate_checksum(self.cmd, &self.payload);
            packet.push(crc.0); // CRC high
            packet.push(crc.1); // CRC low
        }

        packet
    }
}

/// Core checksum algorithm: 16-bit big-endian word sum with XOR for odd trailing byte
///
/// Used by both sending (calculate) and receiving (verify) code paths.
fn checksum_data(data: &[u8]) -> u16 {
    let mut checksum: u16 = 0;
    let mut i = 0;

    // Sum 16-bit words (big-endian pairs)
    while i + 1 < data.len() {
        let word = ((data[i] as u16) << 8) | (data[i + 1] as u16);
        checksum = checksum.wrapping_add(word);
        i += 2;
    }

    // Handle odd trailing byte with XOR
    if i < data.len() {
        checksum ^= data[i] as u16;
    }

    checksum
}

/// Calculate 16-bit big-endian word sum checksum for sending
fn calculate_checksum(cmd: u8, payload: &[u8]) -> (u8, u8) {
    // Create data to checksum: cmd + payload
    let mut data = Vec::with_capacity(1 + payload.len());
    data.push(cmd);
    data.extend_from_slice(payload);

    let checksum = checksum_data(&data);

    // Return as big-endian bytes
    ((checksum >> 8) as u8, (checksum & 0xFF) as u8)
}

/// Verify checksum of received packet
fn verify_checksum_packet(packet: &[u8]) -> bool {
    if packet.len() < MIN_PACKET_SIZE {
        return false;
    }

    let len = packet[2] as usize;
    let total_len = 3 + len;

    if packet.len() < total_len {
        return false;
    }

    // Get received CRC (big-endian)
    let crc_high = packet[total_len - 2];
    let crc_low = packet[total_len - 1];
    let received_crc = ((crc_high as u16) << 8) | (crc_low as u16);

    // Calculate checksum over CMD + PAYLOAD (bytes 3 to total_len-2)
    let data = &packet[3..total_len - 2];
    let calculated_crc = checksum_data(data);

    calculated_crc == received_crc
}

/// Packet reader with buffering for incomplete packets
#[derive(Default)]
pub struct PacketReader {
    buffer: Vec<u8>,
}

impl PacketReader {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(512),
        }
    }

    /// Read and parse a packet from the port
    pub fn read_packet<R: Read>(&mut self, port: &mut R) -> Result<Option<Packet>> {
        // Read available bytes
        let mut temp_buf = [0u8; 256];
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

        // Try to parse packet
        self.try_parse_packet()
    }

    fn try_parse_packet(&mut self) -> Result<Option<Packet>> {
        if self.buffer.len() < MIN_PACKET_SIZE {
            return Ok(None);
        }

        // Find sync bytes
        let Some(sync_idx) = self.find_sync_bytes() else {
            // No sync found, clear buffer if too large
            if self.buffer.len() > MAX_BUFFER_SIZE {
                self.buffer.clear();
            }
            return Ok(None);
        };

        // Remove bytes before sync
        if sync_idx > 0 {
            self.buffer.drain(0..sync_idx);
        }

        // Check if we have length byte
        if self.buffer.len() < 3 {
            return Ok(None);
        }

        let len = self.buffer[2] as usize;
        let total_len = 3 + len; // SYNC(2) + LEN(1) + DATA(len)

        // Wait for complete packet
        if self.buffer.len() < total_len {
            return Ok(None);
        }

        // Extract packet
        let cmd = self.buffer[3];
        let payload_end = total_len - 2; // Before CRC
        let payload = self.buffer[4..payload_end].to_vec();

        // Verify checksum (except for CMD=0x08)
        if cmd != CMD_INITIALIZE {
            let packet_slice = &self.buffer[0..total_len];
            if !verify_checksum_packet(packet_slice) {
                log::warn!("Checksum mismatch for CMD=0x{:02X}", cmd);
                // Remove bad packet
                self.buffer.drain(0..total_len);
                return Ok(None);
            }
        }

        // Remove processed packet
        self.buffer.drain(0..total_len);

        Ok(Some(Packet::new(cmd, payload)))
    }

    fn find_sync_bytes(&self) -> Option<usize> {
        (0..self.buffer.len().saturating_sub(1))
            .find(|&i| self.buffer[i] == SYNC_BYTE_1 && self.buffer[i + 1] == SYNC_BYTE_2)
    }
}

// Command builders

pub fn cmd_initialize() -> Packet {
    Packet::new(CMD_INITIALIZE, vec![])
}

pub fn cmd_heartbeat() -> Packet {
    Packet::new(CMD_HEARTBEAT, vec![])
}

pub fn cmd_version_request() -> Packet {
    Packet::new(CMD_VERSION, vec![])
}

pub fn cmd_motor_velocity(linear: i16, angular: i16) -> Packet {
    // Protocol uses 8 bytes: two i32 little-endian values
    let mut payload = Vec::with_capacity(8);
    payload.extend_from_slice(&(linear as i32).to_le_bytes());
    payload.extend_from_slice(&(angular as i32).to_le_bytes());
    Packet::new(CMD_MOTOR_VELOCITY, payload)
}

pub fn cmd_motor_speed(left: i16, right: i16) -> Packet {
    let mut payload = Vec::with_capacity(4);
    payload.extend_from_slice(&left.to_le_bytes());
    payload.extend_from_slice(&right.to_le_bytes());
    Packet::new(CMD_MOTOR_SPEED, payload)
}

pub fn cmd_air_pump(speed: u8) -> Packet {
    // BlowerSpeed uses u16 little-endian format
    // Scale 0-100% to device range (0-10000)
    let scaled = (speed as u16) * 100;
    Packet::new(CMD_AIR_PUMP, scaled.to_le_bytes().to_vec())
}

pub fn cmd_main_brush(speed: u8) -> Packet {
    Packet::new(CMD_MAIN_BRUSH, vec![speed])
}

pub fn cmd_side_brush(speed: u8) -> Packet {
    Packet::new(CMD_SIDE_BRUSH, vec![speed])
}

/// Water pump control command (0x6B)
///
/// Controls the water pump for 2-in-1 mop box.
/// - `speed`: 0-100% (0x00-0x64)
///
/// This command is dual-use:
/// - At boot: Sent as motor controller init handshake (2s pulse of 0x64)
/// - During operation: Controls water pump for mopping
pub fn cmd_water_pump(speed: u8) -> Packet {
    Packet::new(CMD_WATER_PUMP, vec![speed])
}

/// Build LED state command packet (0x8D)
///
/// Known state values:
/// - 0: Off
/// - 1: Charging/Active (blinking)
/// - 3: Discharging/In use
/// - 6: Fully charged
/// - 11: Standby/Waiting
///
/// Any value 0-255 is accepted for experimentation.
pub fn cmd_led_state(state: u8) -> Packet {
    Packet::new(CMD_BUTTON_LED, vec![state])
}

// Lidar control commands

pub fn cmd_motor_mode(mode: u8) -> Packet {
    Packet::new(CMD_MOTOR_MODE, vec![mode])
}

pub fn cmd_lidar_power(on: bool) -> Packet {
    Packet::new(CMD_LIDAR_POWER, vec![if on { 0x01 } else { 0x00 }])
}

pub fn cmd_lidar_pwm(speed: i32) -> Packet {
    let clamped = speed.clamp(0, 100);
    let payload = clamped.to_le_bytes().to_vec();
    Packet::new(CMD_LIDAR_PWM, payload)
}

/// IMU factory calibration command (0xA1)
///
/// Triggers factory-level IMU calibration. No payload required.
/// Typically used during manufacturing or to reset IMU to factory defaults.
pub fn cmd_imu_factory_calibrate() -> Packet {
    Packet::new(CMD_IMU_FACTORY_CALIBRATE, vec![])
}

/// IMU calibration state command (0xA2)
///
/// Called by upstream application before enabling lidar.
/// Observed payload in R2D logs: `[0x10, 0x0E, 0x00, 0x00]`
pub fn cmd_imu_calibrate_state(payload: Vec<u8>) -> Packet {
    Packet::new(CMD_IMU_CALIBRATE_STATE, payload)
}

/// Compass/geomagnetism calibration command (0xA3)
///
/// Starts compass calibration procedure. No payload required.
/// Robot typically needs to be rotated 360° during calibration.
pub fn cmd_compass_calibrate() -> Packet {
    Packet::new(CMD_COMPASS_CALIBRATE, vec![])
}

/// Compass calibration state query command (0xA4)
///
/// Queries current compass calibration state. No payload required.
pub fn cmd_compass_calibration_state() -> Packet {
    Packet::new(CMD_COMPASS_CALIBRATION_STATE, vec![])
}

/// Cliff IR control command (0x78)
///
/// Enables or disables cliff IR sensors.
pub fn cmd_cliff_ir_control(enable: bool) -> Packet {
    Packet::new(CMD_CLIFF_IR_CONTROL, vec![if enable { 0x01 } else { 0x00 }])
}

/// Cliff IR direction command (0x79)
///
/// Sets cliff IR direction/configuration.
pub fn cmd_cliff_ir_direction(direction: u8) -> Packet {
    Packet::new(CMD_CLIFF_IR_DIRECTION, vec![direction])
}

/// Request STM32 data command (0x0D)
///
/// Polls sensor status from STM32. Called internally by driver every ~3 seconds.
pub fn cmd_request_stm32_data() -> Packet {
    Packet::new(CMD_REQUEST_STM32_DATA, vec![])
}

// Power management commands

/// Main board (A33) power control command (0x99)
///
/// Controls power to the A33 main application board running Linux.
/// - `on = true`: Power on main board
/// - `on = false`: Power off main board
///
/// **WARNING**: Powering off will terminate the daemon and lose connectivity!
pub fn cmd_main_board_power(on: bool) -> Packet {
    Packet::new(CMD_MAIN_BOARD_POWER, vec![if on { 0x01 } else { 0x00 }])
}

/// Restart main board command (0x9A)
///
/// Triggers a full restart of the A33 Linux system. No payload required.
///
/// **WARNING**: This will terminate the daemon and require reconnection!
pub fn cmd_main_board_restart() -> Packet {
    Packet::new(CMD_MAIN_BOARD_RESTART, vec![])
}

/// Charger power control command (0x9B)
///
/// Controls the charger power rail.
/// - `on = true`: Enable charger power
/// - `on = false`: Disable charger power
pub fn cmd_charger_power(on: bool) -> Packet {
    Packet::new(CMD_CHARGER_POWER, vec![if on { 0x01 } else { 0x00 }])
}

// MCU control commands

/// MCU sleep command (0x04)
///
/// Puts the GD32 MCU into sleep/low-power mode.
/// Found in AuxCtrl as `packetStm32Sleep`. No payload required.
pub fn cmd_mcu_sleep() -> Packet {
    Packet::new(CMD_MCU_SLEEP, vec![])
}

/// Wakeup acknowledge command (0x05)
///
/// Acknowledges wakeup from sleep mode.
/// Found in AuxCtrl as `packetWakeupAck`. No payload required.
pub fn cmd_wakeup_ack() -> Packet {
    Packet::new(CMD_WAKEUP_ACK, vec![])
}

/// Reset error code command (0x0A)
///
/// Clears/resets error codes on the GD32.
/// Found in AuxCtrl as `packetResetErrorCode`. Typically called at end of operations.
/// No payload required.
pub fn cmd_reset_error_code() -> Packet {
    Packet::new(CMD_RESET_ERROR_CODE, vec![])
}

/// Protocol sync command (0x0C)
///
/// First command sent at boot to wake GD32 and synchronize protocol.
/// This is a fire-and-forget command - AuxCtrl does not wait for ACK.
/// GD32 echoes this command back after ~270ms.
///
/// Payload: `[0x01]` = enable/sync
pub fn cmd_protocol_sync() -> Packet {
    Packet::new(CMD_PROTOCOL_SYNC, vec![0x01])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initialize_packet() {
        let pkt = cmd_initialize();
        let bytes = pkt.to_bytes();

        // Should be: FA FB 01 08 (no checksum)
        assert_eq!(bytes.len(), 4);
        assert_eq!(bytes[0], 0xFA);
        assert_eq!(bytes[1], 0xFB);
        assert_eq!(bytes[2], 0x01);
        assert_eq!(bytes[3], 0x08);
    }

    #[test]
    fn test_heartbeat_packet() {
        let pkt = cmd_heartbeat();
        let bytes = pkt.to_bytes();

        // Should be: FA FB 03 06 XX XX (with checksum)
        assert_eq!(bytes.len(), 6);
        assert_eq!(bytes[0], 0xFA);
        assert_eq!(bytes[1], 0xFB);
        assert_eq!(bytes[2], 0x03);
        assert_eq!(bytes[3], 0x06);
    }
}
