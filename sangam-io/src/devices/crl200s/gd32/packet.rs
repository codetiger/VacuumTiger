//! Zero-allocation packet structures for GD32 protocol
//!
//! This module provides:
//! - `TxPacket`: Single reusable buffer for all outgoing commands
//! - `static_packets`: Pre-computed bytes for fixed commands
//! - `RxPacket`: Zero-copy view into received data
//!
//! # Pattern
//!
//! ```ignore
//! let mut pkt = TxPacket::new();   // Create once at thread start
//! pkt.set_lidar_pwm(80);           // Configure for any command
//! pkt.send_to(&mut port)?;         // Send
//! pkt.set_velocity(100, 50);       // Reuse for different command
//! pkt.send_to(&mut port)?;         // Send again
//! ```
//!
//! # Performance
//!
//! - Zero heap allocation in hot loop
//! - Single 14-byte buffer handles all commands
//! - Static packets for fixed commands avoid even CRC calculation

use crate::devices::crl200s::constants::*;
use std::io::{self, Write};

/// Maximum packet size (velocity command: 14 bytes)
const MAX_PACKET_SIZE: usize = 14;

/// Reusable TX packet buffer for all GD32 commands
///
/// Single 14-byte buffer that can be configured for any command.
/// Create once, reuse forever - zero allocation in hot loop.
///
/// # Example
///
/// ```ignore
/// let mut pkt = TxPacket::new();
///
/// // Use for velocity
/// pkt.set_velocity(100, 50);
/// pkt.send_to(&mut port)?;
///
/// // Reuse for air pump
/// pkt.set_air_pump(80);
/// pkt.send_to(&mut port)?;
///
/// // Reuse for lidar
/// pkt.set_lidar_pwm(60);
/// pkt.send_to(&mut port)?;
/// ```
pub struct TxPacket {
    data: [u8; MAX_PACKET_SIZE],
    len: usize,
}

impl TxPacket {
    /// Create new packet with sync bytes pre-filled
    pub const fn new() -> Self {
        let mut data = [0u8; MAX_PACKET_SIZE];
        data[0] = SYNC_BYTE_1; // 0xFA
        data[1] = SYNC_BYTE_2; // 0xFB
        Self { data, len: 0 }
    }

    /// Get packet bytes for sending
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// Send packet to any writer (serial port, etc.)
    #[inline]
    pub fn send_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
        writer.write_all(self.as_bytes())
    }

    // ========================================================================
    // Motion Commands
    // ========================================================================

    /// Set motor velocity (CMD 0x66)
    ///
    /// Linear: forward/backward speed (empirical units, positive = forward)
    /// Angular: rotation speed (empirical units, positive = counter-clockwise)
    /// Note: Units are converted from m/s and rad/s using VELOCITY_TO_DEVICE_UNITS (523)
    #[inline]
    pub fn set_velocity(&mut self, linear: i16, angular: i16) {
        self.data[2] = 11; // LEN = cmd(1) + payload(8) + crc(2)
        self.data[3] = CMD_MOTOR_VELOCITY;
        self.data[4..8].copy_from_slice(&(linear as i32).to_le_bytes());
        self.data[8..12].copy_from_slice(&(angular as i32).to_le_bytes());
        self.finalize(8);
    }

    /// Set motor mode (CMD 0x65)
    ///
    /// Mode: 0x00 = idle, 0x02 = navigation
    #[inline]
    pub fn set_motor_mode(&mut self, mode: u8) {
        self.data[2] = 4; // LEN = cmd(1) + payload(1) + crc(2)
        self.data[3] = CMD_MOTOR_MODE;
        self.data[4] = mode;
        self.finalize(1);
    }

    /// Set motor to navigation mode (0x02)
    #[inline]
    pub fn set_motor_mode_nav(&mut self) {
        self.set_motor_mode(0x02);
    }

    // ========================================================================
    // Actuator Commands
    // ========================================================================

    /// Set air pump / vacuum speed (CMD 0x68)
    ///
    /// Speed: 0-100% (internally scaled to 0-10000)
    #[inline]
    pub fn set_air_pump(&mut self, speed: u8) {
        // Scale 0-100% to 0-10000 range (matches original protocol.rs scaling)
        let scaled = ((speed as f32) * 16.5) as u16;
        self.data[2] = 5; // LEN = cmd(1) + payload(2) + crc(2)
        self.data[3] = CMD_AIR_PUMP;
        self.data[4..6].copy_from_slice(&scaled.to_le_bytes());
        self.finalize(2);
    }

    /// Set main brush speed (CMD 0x6A)
    ///
    /// Speed: 0-100%
    #[inline]
    pub fn set_main_brush(&mut self, speed: u8) {
        self.data[2] = 4;
        self.data[3] = CMD_MAIN_BRUSH;
        self.data[4] = speed;
        self.finalize(1);
    }

    /// Set side brush speed (CMD 0x69)
    ///
    /// Speed: 0-100%
    #[inline]
    pub fn set_side_brush(&mut self, speed: u8) {
        self.data[2] = 4;
        self.data[3] = CMD_SIDE_BRUSH;
        self.data[4] = speed;
        self.finalize(1);
    }

    /// Set water pump speed (CMD 0x6B)
    ///
    /// Speed: 0-100%
    #[inline]
    pub fn set_water_pump(&mut self, speed: u8) {
        self.data[2] = 4;
        self.data[3] = CMD_WATER_PUMP;
        self.data[4] = speed;
        self.finalize(1);
    }

    /// Set lidar motor PWM (CMD 0x71)
    ///
    /// Speed: 0-100%
    #[inline]
    pub fn set_lidar_pwm(&mut self, speed: u8) {
        let value = (speed as i32).clamp(0, 100);
        self.data[2] = 7; // LEN = cmd(1) + payload(4) + crc(2)
        self.data[3] = CMD_LIDAR_PWM;
        self.data[4..8].copy_from_slice(&value.to_le_bytes());
        self.finalize(4);
    }

    /// Set button LED state (CMD 0x8D)
    ///
    /// State: 0=off, 1=charging, 3=discharge, 6=charged, 11=standby
    #[inline]
    pub fn set_led(&mut self, state: u8) {
        self.data[2] = 4;
        self.data[3] = CMD_BUTTON_LED;
        self.data[4] = state;
        self.finalize(1);
    }

    // ========================================================================
    // System Commands
    // ========================================================================

    /// Set heartbeat command (CMD 0x06)
    #[inline]
    pub fn set_heartbeat(&mut self) {
        self.data[2] = 3; // LEN = cmd(1) + crc(2)
        self.data[3] = CMD_HEARTBEAT;
        self.finalize(0);
    }

    /// Set initialize command (CMD 0x08) - no checksum!
    #[inline]
    pub fn set_initialize(&mut self) {
        self.data[2] = 1; // LEN = cmd(1) only, no CRC
        self.data[3] = CMD_INITIALIZE;
        self.len = 4; // Special case: no CRC for init
    }

    /// Set version request command (CMD 0x07)
    #[inline]
    pub fn set_version_request(&mut self) {
        self.data[2] = 3;
        self.data[3] = CMD_VERSION;
        self.finalize(0);
    }

    /// Set STM32 data request (CMD 0x0D)
    #[inline]
    pub fn set_request_stm32(&mut self) {
        self.data[2] = 3;
        self.data[3] = CMD_REQUEST_STM32_DATA;
        self.finalize(0);
    }

    /// Set protocol sync (CMD 0x0C)
    #[inline]
    pub fn set_protocol_sync(&mut self) {
        self.data[2] = 4;
        self.data[3] = CMD_PROTOCOL_SYNC;
        self.data[4] = 0x01;
        self.finalize(1);
    }

    /// Set lidar power (CMD 0x97)
    #[inline]
    pub fn set_lidar_power(&mut self, on: bool) {
        self.data[2] = 4;
        self.data[3] = CMD_LIDAR_POWER;
        self.data[4] = if on { 0x01 } else { 0x00 };
        self.finalize(1);
    }

    /// Set cliff IR control (CMD 0x78)
    #[inline]
    pub fn set_cliff_ir(&mut self, enable: bool) {
        self.data[2] = 4;
        self.data[3] = CMD_CLIFF_IR_CONTROL;
        self.data[4] = if enable { 0x01 } else { 0x00 };
        self.finalize(1);
    }

    /// Set cliff IR direction (CMD 0x79)
    #[inline]
    pub fn set_cliff_ir_direction(&mut self, direction: u8) {
        self.data[2] = 4;
        self.data[3] = CMD_CLIFF_IR_DIRECTION;
        self.data[4] = direction;
        self.finalize(1);
    }

    /// Set motor speed - tank drive (CMD 0x64)
    ///
    /// Left/Right: individual wheel speeds in ticks/sec
    #[inline]
    pub fn set_motor_speed(&mut self, left: i16, right: i16) {
        self.data[2] = 7; // LEN = cmd(1) + payload(4) + crc(2)
        self.data[3] = CMD_MOTOR_SPEED;
        self.data[4..6].copy_from_slice(&left.to_le_bytes());
        self.data[6..8].copy_from_slice(&right.to_le_bytes());
        self.finalize(4);
    }

    // ========================================================================
    // IMU & Compass Commands
    // ========================================================================

    /// Set IMU factory calibrate (CMD 0xA1)
    #[inline]
    pub fn set_imu_factory_calibrate(&mut self) {
        self.data[2] = 3;
        self.data[3] = CMD_IMU_FACTORY_CALIBRATE;
        self.finalize(0);
    }

    /// Set IMU calibrate state (CMD 0xA2)
    ///
    /// Payload typically `[0x10, 0x0E, 0x00, 0x00]` observed in R2D logs
    #[inline]
    pub fn set_imu_calibrate_state(&mut self, payload: &[u8; 4]) {
        self.data[2] = 7; // LEN = cmd(1) + payload(4) + crc(2)
        self.data[3] = CMD_IMU_CALIBRATE_STATE;
        self.data[4..8].copy_from_slice(payload);
        self.finalize(4);
    }

    /// Set compass calibrate start (CMD 0xA3)
    #[inline]
    pub fn set_compass_calibrate(&mut self) {
        self.data[2] = 3;
        self.data[3] = CMD_COMPASS_CALIBRATE;
        self.finalize(0);
    }

    /// Set compass calibration state query (CMD 0xA4)
    #[inline]
    pub fn set_compass_calibration_state(&mut self) {
        self.data[2] = 3;
        self.data[3] = CMD_COMPASS_CALIBRATION_STATE;
        self.finalize(0);
    }

    // ========================================================================
    // Power Management Commands
    // ========================================================================

    /// Set main board (A33) power (CMD 0x99)
    #[inline]
    pub fn set_main_board_power(&mut self, on: bool) {
        self.data[2] = 4;
        self.data[3] = CMD_MAIN_BOARD_POWER;
        self.data[4] = if on { 0x01 } else { 0x00 };
        self.finalize(1);
    }

    /// Set main board restart (CMD 0x9A)
    #[inline]
    pub fn set_main_board_restart(&mut self) {
        self.data[2] = 3;
        self.data[3] = CMD_MAIN_BOARD_RESTART;
        self.finalize(0);
    }

    /// Set charger power (CMD 0x9B)
    #[inline]
    pub fn set_charger_power(&mut self, on: bool) {
        self.data[2] = 4;
        self.data[3] = CMD_CHARGER_POWER;
        self.data[4] = if on { 0x01 } else { 0x00 };
        self.finalize(1);
    }

    // ========================================================================
    // MCU Control Commands
    // ========================================================================

    /// Set MCU sleep (CMD 0x04)
    #[inline]
    pub fn set_mcu_sleep(&mut self) {
        self.data[2] = 3;
        self.data[3] = CMD_MCU_SLEEP;
        self.finalize(0);
    }

    /// Set wakeup ack (CMD 0x05)
    #[inline]
    pub fn set_wakeup_ack(&mut self) {
        self.data[2] = 3;
        self.data[3] = CMD_WAKEUP_ACK;
        self.finalize(0);
    }

    /// Set reset error code (CMD 0x0A)
    #[inline]
    pub fn set_reset_error_code(&mut self) {
        self.data[2] = 3;
        self.data[3] = CMD_RESET_ERROR_CODE;
        self.finalize(0);
    }

    // ========================================================================
    // Internal Helpers
    // ========================================================================

    /// Calculate CRC and set final packet length
    #[inline]
    fn finalize(&mut self, payload_len: usize) {
        let crc_pos = 4 + payload_len;
        let crc = checksum(&self.data[3..crc_pos]);
        self.data[crc_pos] = (crc >> 8) as u8;
        self.data[crc_pos + 1] = (crc & 0xFF) as u8;
        self.len = crc_pos + 2;
    }
}

// ============================================================================
// Checksum - single canonical implementation for GD32 protocol
// ============================================================================

/// GD32 16-bit checksum: big-endian word sum with XOR for odd trailing byte
///
/// This is the canonical checksum implementation used by both TX and RX paths.
/// The algorithm:
/// 1. Sum consecutive byte pairs as big-endian 16-bit words
/// 2. If odd byte remains, XOR it with the sum
///
/// # Example
/// ```ignore
/// // For CMD=0x06 (heartbeat, no payload):
/// // checksum(&[0x06]) = 0x0006 (just XOR of single byte)
///
/// // For CMD=0x65, payload=[0x02] (motor mode nav):
/// // checksum(&[0x65, 0x02]) = 0x6502
/// ```
#[inline]
pub fn checksum(data: &[u8]) -> u16 {
    let mut sum: u16 = 0;
    let mut i = 0;
    while i + 1 < data.len() {
        let word = ((data[i] as u16) << 8) | (data[i + 1] as u16);
        sum = sum.wrapping_add(word);
        i += 2;
    }
    if i < data.len() {
        sum ^= data[i] as u16;
    }
    sum
}

impl Default for TxPacket {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Pre-configured Packet Factory
// ============================================================================

/// Create a pre-configured heartbeat packet (reusable)
///
/// # Example
/// ```ignore
/// let heartbeat = heartbeat_packet();
/// loop {
///     heartbeat.send_to(&mut port)?;
///     thread::sleep(Duration::from_millis(20));
/// }
/// ```
pub fn heartbeat_packet() -> TxPacket {
    let mut pkt = TxPacket::new();
    pkt.set_heartbeat();
    pkt
}

/// Create a pre-configured motor mode navigation packet (reusable)
pub fn motor_mode_nav_packet() -> TxPacket {
    let mut pkt = TxPacket::new();
    pkt.set_motor_mode_nav();
    pkt
}

/// Create a pre-configured STM32 data request packet (reusable)
pub fn request_stm32_packet() -> TxPacket {
    let mut pkt = TxPacket::new();
    pkt.set_request_stm32();
    pkt
}

/// Create a pre-configured protocol sync packet (reusable)
pub fn protocol_sync_packet() -> TxPacket {
    let mut pkt = TxPacket::new();
    pkt.set_protocol_sync();
    pkt
}

/// Create a pre-configured initialize packet (reusable)
pub fn initialize_packet() -> TxPacket {
    let mut pkt = TxPacket::new();
    pkt.set_initialize();
    pkt
}

/// Create a pre-configured version request packet (reusable)
pub fn version_request_packet() -> TxPacket {
    let mut pkt = TxPacket::new();
    pkt.set_version_request();
    pkt
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heartbeat_packet() {
        let pkt = heartbeat_packet();
        // Heartbeat: FA FB 03 06 [CRC]
        assert_eq!(pkt.as_bytes().len(), 6);
        assert_eq!(pkt.as_bytes()[0], 0xFA);
        assert_eq!(pkt.as_bytes()[1], 0xFB);
        assert_eq!(pkt.as_bytes()[2], 0x03); // LEN
        assert_eq!(pkt.as_bytes()[3], CMD_HEARTBEAT);
    }

    #[test]
    fn test_motor_mode_nav() {
        let pkt = motor_mode_nav_packet();
        // Motor mode nav: FA FB 04 65 02 [CRC]
        assert_eq!(pkt.as_bytes().len(), 7);
        assert_eq!(pkt.as_bytes()[3], CMD_MOTOR_MODE);
        assert_eq!(pkt.as_bytes()[4], 0x02); // Nav mode
    }

    #[test]
    fn test_velocity_packet() {
        let mut pkt = TxPacket::new();
        pkt.set_velocity(100, 50);

        // Verify structure: [FA FB] [0B] [66] [payload 8] [crc 2] = 14 bytes
        assert_eq!(pkt.as_bytes().len(), 14);
        assert_eq!(pkt.as_bytes()[0], 0xFA);
        assert_eq!(pkt.as_bytes()[1], 0xFB);
        assert_eq!(pkt.as_bytes()[2], 11); // LEN
        assert_eq!(pkt.as_bytes()[3], CMD_MOTOR_VELOCITY);
    }

    #[test]
    fn test_air_pump_scaling() {
        let mut pkt = TxPacket::new();
        pkt.set_air_pump(100); // 100% should scale to ~1650

        // Verify structure: [FA FB] [05] [68] [u16 LE] [crc 2] = 8 bytes
        assert_eq!(pkt.as_bytes().len(), 8);
        assert_eq!(pkt.as_bytes()[3], CMD_AIR_PUMP);

        // Check scaled value (100 * 16.5 = 1650 = 0x0672)
        let scaled = u16::from_le_bytes([pkt.as_bytes()[4], pkt.as_bytes()[5]]);
        assert_eq!(scaled, 1650);
    }

    #[test]
    fn test_initialize_no_crc() {
        let pkt = initialize_packet();
        // Initialize: FA FB 01 08 (no CRC!)
        assert_eq!(pkt.as_bytes().len(), 4);
        assert_eq!(pkt.as_bytes()[3], CMD_INITIALIZE);
    }

    #[test]
    fn test_lidar_pwm() {
        let mut pkt = TxPacket::new();
        pkt.set_lidar_pwm(80);

        // Verify structure: [FA FB] [07] [71] [i32 LE] [crc 2] = 10 bytes
        assert_eq!(pkt.as_bytes().len(), 10);
        assert_eq!(pkt.as_bytes()[3], CMD_LIDAR_PWM);

        // Check value (80 as i32 LE = 0x50, 0x00, 0x00, 0x00)
        assert_eq!(pkt.as_bytes()[4], 80);
        assert_eq!(pkt.as_bytes()[5], 0);
        assert_eq!(pkt.as_bytes()[6], 0);
        assert_eq!(pkt.as_bytes()[7], 0);
    }

    #[test]
    fn test_packet_reuse() {
        let mut pkt = TxPacket::new();

        // Use for heartbeat
        pkt.set_heartbeat();
        assert_eq!(pkt.as_bytes().len(), 6);

        // Reuse for velocity (larger packet)
        pkt.set_velocity(100, 50);
        assert_eq!(pkt.as_bytes().len(), 14);

        // Reuse for brush (smaller packet)
        pkt.set_main_brush(50);
        assert_eq!(pkt.as_bytes().len(), 7);
    }
}
