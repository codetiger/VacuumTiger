//! GD32 Protocol Implementation
//!
//! Packet format: [0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC_H] [CRC_L]
//!
//! This module provides:
//! - `PacketReader`: Ring-buffer based parser with O(1) advance (no drain!)
//! - `RxPacket`: Zero-allocation parsed packet with fixed-size payload buffer
//!
//! For sending commands, use `TxPacket` from `packet.rs`.

use super::packet::checksum;
use super::ring_buffer::RingBuffer;
use crate::devices::crl200s::constants::{
    CMD_INITIALIZE, MAX_BUFFER_SIZE, MIN_PACKET_SIZE, SYNC_BYTE_1, SYNC_BYTE_2,
};
use crate::error::{Error, Result};
use std::io::Read;

/// Maximum payload size for GD32 packets
///
/// Status packets (CMD=0x15) are ~96 bytes, version responses ~32 bytes.
/// 128 bytes provides headroom for any packet type.
pub const MAX_PAYLOAD_SIZE: usize = 128;

/// Zero-allocation parsed packet from GD32
///
/// Uses a fixed-size array instead of `Vec<u8>` to eliminate heap allocations.
/// At 500Hz packet rate, this saves ~48KB/sec of allocations.
#[derive(Debug, Clone, Copy)]
pub struct RxPacket {
    pub cmd: u8,
    payload: [u8; MAX_PAYLOAD_SIZE],
    payload_len: usize,
}

impl RxPacket {
    /// Create a new empty packet
    #[inline]
    pub const fn new() -> Self {
        Self {
            cmd: 0,
            payload: [0u8; MAX_PAYLOAD_SIZE],
            payload_len: 0,
        }
    }

    /// Get the payload as a slice
    #[inline]
    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_len]
    }

    /// Get payload length
    #[inline]
    pub fn payload_len(&self) -> usize {
        self.payload_len
    }

    /// Set command and payload from a slice
    #[inline]
    fn set(&mut self, cmd: u8, data: &[u8]) {
        self.cmd = cmd;
        let len = data.len().min(MAX_PAYLOAD_SIZE);
        self.payload[..len].copy_from_slice(&data[..len]);
        self.payload_len = len;
    }
}

impl Default for RxPacket {
    fn default() -> Self {
        Self::new()
    }
}

/// Ring-buffer based packet reader with O(1) advance
///
/// Uses `RingBuffer` instead of `Vec<u8>` to eliminate O(n) `drain()` operations.
/// At 500Hz packet rate, this saves ~50KB/sec of memory shifts.
///
/// Returns `RxPacket` with fixed-size payload buffer (no heap allocation).
pub struct PacketReader {
    buffer: RingBuffer<1024>,
    /// Reusable packet buffer - avoids allocation on every read
    packet: RxPacket,
}

impl PacketReader {
    pub fn new() -> Self {
        Self {
            buffer: RingBuffer::new(),
            packet: RxPacket::new(),
        }
    }

    /// Read and parse a packet from the port
    ///
    /// Returns a reference to the internal packet buffer. The data is valid
    /// until the next call to `read_packet`.
    pub fn read_packet<R: Read>(&mut self, port: &mut R) -> Result<Option<&RxPacket>> {
        // Read available bytes into temp buffer
        let mut temp_buf = [0u8; 256];
        match port.read(&mut temp_buf) {
            Ok(0) => return Ok(None),
            Ok(n) => {
                self.buffer.extend(&temp_buf[..n]);
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

    fn try_parse_packet(&mut self) -> Result<Option<&RxPacket>> {
        if self.buffer.len() < MIN_PACKET_SIZE {
            return Ok(None);
        }

        // Find sync bytes (0xFA 0xFB)
        let Some(sync_idx) = self.buffer.find_pattern_2(SYNC_BYTE_1, SYNC_BYTE_2) else {
            // No sync found, keep tail for partial match detection
            if self.buffer.len() > MAX_BUFFER_SIZE {
                // Keep last byte in case it's 0xFA
                self.buffer.advance(self.buffer.len() - 1);
            }
            return Ok(None);
        };

        // Skip bytes before sync - O(1) instead of O(n) drain!
        if sync_idx > 0 {
            self.buffer.advance(sync_idx);
        }

        // Check if we have length byte
        if self.buffer.len() < 3 {
            return Ok(None);
        }

        let Some(len) = self.buffer.get(2) else {
            return Ok(None);
        };
        let total_len = 3 + len as usize; // SYNC(2) + LEN(1) + DATA(len)

        // Wait for complete packet
        if self.buffer.len() < total_len {
            return Ok(None);
        }

        // Extract cmd
        let cmd = self.buffer.get(3).unwrap();

        // Verify checksum (except for CMD=0x08)
        if cmd != CMD_INITIALIZE && !self.verify_checksum(total_len) {
            log::warn!("Checksum mismatch for CMD=0x{:02X}", cmd);
            // Only advance past first sync byte - don't trust the corrupted len!
            // This allows find_pattern_2 to resync on the next valid packet.
            self.buffer.advance(1);
            return Ok(None);
        }

        // Extract payload into fixed buffer (no heap allocation!)
        // Payload starts AFTER CMD byte - offsets in constants.rs are relative to payload start
        let payload_len = total_len.saturating_sub(6); // header(3) + cmd(1) + crc(2)
        if payload_len > 0 {
            // Start at index 4 (after SYNC(2) + LEN(1) + CMD(1))
            let slice = self.buffer.get_slice(4, payload_len);
            if let Some(data) = slice {
                self.packet.set(cmd, data);
            } else {
                self.packet.set(cmd, &[]);
            }
        } else {
            self.packet.set(cmd, &[]);
        }

        // Consume packet - O(1) instead of O(n) drain!
        self.buffer.advance(total_len);

        Ok(Some(&self.packet))
    }

    /// Verify checksum of packet in buffer
    fn verify_checksum(&mut self, total_len: usize) -> bool {
        // Get received CRC (big-endian, at end of packet)
        let crc_high = self.buffer.get(total_len - 2).unwrap_or(0);
        let crc_low = self.buffer.get(total_len - 1).unwrap_or(0);
        let received_crc = ((crc_high as u16) << 8) | (crc_low as u16);

        // Calculate checksum over CMD + PAYLOAD (bytes 3 to total_len-2)
        let data_len = total_len - 5; // Exclude sync(2), len(1), crc(2)
        let data_slice = self.buffer.get_slice(3, data_len);

        let Some(data) = data_slice else {
            log::warn!(
                "Failed to get data slice for checksum: total_len={}, data_len={}",
                total_len,
                data_len
            );
            return false;
        };

        // Use shared checksum from packet.rs
        let calculated_crc = checksum(data);

        if calculated_crc != received_crc {
            log::debug!(
                "CRC mismatch: received=0x{:04X}, calculated=0x{:04X}, total_len={}, data_len={}, first_bytes={:02X?}",
                received_crc, calculated_crc, total_len, data_len, &data[..data_len.min(8)]
            );
        }

        calculated_crc == received_crc
    }
}

impl Default for PacketReader {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rx_packet_zero_alloc() {
        let mut pkt = RxPacket::new();
        assert_eq!(pkt.cmd, 0);
        assert_eq!(pkt.payload_len(), 0);

        // Set payload
        pkt.set(0x15, &[1, 2, 3, 4, 5]);
        assert_eq!(pkt.cmd, 0x15);
        assert_eq!(pkt.payload_len(), 5);
        assert_eq!(pkt.payload(), &[1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_rx_packet_max_payload() {
        let mut pkt = RxPacket::new();
        let large_data = [0xAA; 200]; // Larger than MAX_PAYLOAD_SIZE

        pkt.set(0x15, &large_data);
        assert_eq!(pkt.payload_len(), MAX_PAYLOAD_SIZE);
        assert_eq!(pkt.payload(), &[0xAA; MAX_PAYLOAD_SIZE]);
    }
}
