//! GD32 Protocol Implementation
//! Packet format: [0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC_H] [CRC_L]

use crate::devices::crl200s::constants::*;
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

/// Calculate 16-bit big-endian word sum checksum for sending
fn calculate_checksum(cmd: u8, payload: &[u8]) -> (u8, u8) {
    let mut checksum: u16 = 0;

    // Create data to checksum: cmd + payload
    let mut data = Vec::with_capacity(1 + payload.len());
    data.push(cmd);
    data.extend_from_slice(payload);

    // Sum 16-bit words (big-endian pairs)
    let mut i = 0;
    while i + 1 < data.len() {
        let word = ((data[i] as u16) << 8) | (data[i + 1] as u16);
        checksum = checksum.wrapping_add(word);
        i += 2;
    }

    // Handle odd trailing byte with XOR
    if i < data.len() {
        checksum ^= data[i] as u16;
    }

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

    // Get received CRC
    let crc_high = packet[total_len - 2];
    let crc_low = packet[total_len - 1];
    let received_crc = ((crc_high as u16) << 8) | (crc_low as u16);

    // Calculate checksum over CMD + PAYLOAD (bytes 3 to total_len-2)
    let data = &packet[3..total_len - 2];

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

    checksum == received_crc
}

/// Packet reader with buffering for incomplete packets
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
        let sync_pos = self.find_sync_bytes();
        if sync_pos.is_none() {
            // No sync found, clear buffer if too large
            if self.buffer.len() > MAX_BUFFER_SIZE {
                self.buffer.clear();
            }
            return Ok(None);
        }

        let sync_idx = sync_pos.unwrap();

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
        (0..self.buffer.len().saturating_sub(1)).find(|&i| self.buffer[i] == SYNC_BYTE_1 && self.buffer[i + 1] == SYNC_BYTE_2)
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
    let mut payload = Vec::with_capacity(4);
    payload.extend_from_slice(&linear.to_le_bytes());
    payload.extend_from_slice(&angular.to_le_bytes());
    Packet::new(CMD_MOTOR_VELOCITY, payload)
}

pub fn cmd_motor_speed(left: i16, right: i16) -> Packet {
    let mut payload = Vec::with_capacity(4);
    payload.extend_from_slice(&left.to_le_bytes());
    payload.extend_from_slice(&right.to_le_bytes());
    Packet::new(CMD_MOTOR_SPEED, payload)
}

pub fn cmd_air_pump(speed: u8) -> Packet {
    Packet::new(CMD_AIR_PUMP, vec![speed])
}

pub fn cmd_main_brush(speed: u8) -> Packet {
    Packet::new(CMD_MAIN_BRUSH, vec![speed])
}

pub fn cmd_side_brush(speed: u8) -> Packet {
    Packet::new(CMD_SIDE_BRUSH, vec![speed])
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
