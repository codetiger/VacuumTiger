//! GD32 Protocol Implementation
//!
//! Packet format: [0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC_H] [CRC_L]
//!
//! This module provides:
//! - `PacketReader`: Simple Vec-based parser for reliable packet parsing
//! - `RxPacket`: Zero-allocation parsed packet with fixed-size payload buffer
//!
//! For sending commands, use `TxPacket` from `packet.rs`.

use super::packet::checksum;
use crate::devices::crl200s::constants::{CMD_INITIALIZE, SYNC_BYTE_1, SYNC_BYTE_2};
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
/// At 110Hz packet rate, this saves ~11KB/sec of allocations.
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

/// Simple Vec-based packet reader for reliable parsing
///
/// Uses a plain Vec<u8> with drain() for simplicity and correctness.
/// At 110Hz with ~100 byte packets, the O(n) drain is negligible.
pub struct PacketReader {
    buffer: Vec<u8>,
    /// Reusable packet buffer - avoids allocation on every read
    packet: RxPacket,
}

impl PacketReader {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(1024),
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

    /// Find sync pattern FA FB in buffer
    fn find_sync(&self) -> Option<usize> {
        self.buffer
            .windows(2)
            .position(|w| w[0] == SYNC_BYTE_1 && w[1] == SYNC_BYTE_2)
    }

    fn try_parse_packet(&mut self) -> Result<Option<&RxPacket>> {
        loop {
            // Step 1: Need at least 3 bytes to read FA FB [LEN]
            if self.buffer.len() < 3 {
                return Ok(None);
            }

            // Step 2: Find sync bytes (0xFA 0xFB)
            let Some(sync_idx) = self.find_sync() else {
                // No sync found, keep only last byte (could be start of FA)
                let keep = if self.buffer.last() == Some(&SYNC_BYTE_1) {
                    1
                } else {
                    0
                };
                self.buffer.drain(..self.buffer.len() - keep);
                return Ok(None);
            };

            // Remove bytes before sync
            if sync_idx > 0 {
                self.buffer.drain(..sync_idx);
            }

            // Need at least 3 bytes for header (FA FB LEN)
            if self.buffer.len() < 3 {
                return Ok(None);
            }

            // Step 3: Read length and wait for complete packet
            let len = self.buffer[2];
            let total_len = 3 + len as usize; // SYNC(2) + LEN(1) + DATA(len)

            // Wait until buffer has complete packet
            if self.buffer.len() < total_len {
                return Ok(None);
            }

            // Step 4: Extract packet data
            // Packet structure: [FA FB] [LEN] [CMD] [PAYLOAD...] [CRC_H] [CRC_L]
            // LEN = CMD(1) + PAYLOAD(n) + CRC(2), so payload_len = LEN - 3
            let cmd = self.buffer[3];
            let payload_len = (len as usize).saturating_sub(3); // LEN - CMD(1) - CRC(2)

            // Step 5: Verify checksum (except for CMD=0x08 which has no CRC)
            let crc_valid = if cmd == CMD_INITIALIZE {
                true
            } else {
                self.verify_checksum(total_len)
            };

            if crc_valid {
                // Extract payload before draining
                if payload_len > 0 && payload_len <= MAX_PAYLOAD_SIZE {
                    self.packet.set(cmd, &self.buffer[4..4 + payload_len]);
                } else {
                    self.packet.set(cmd, &[]);
                }

                // Remove valid packet from buffer
                self.buffer.drain(..total_len);
                return Ok(Some(&self.packet));
            } else {
                // CRC failed - this was likely a FALSE sync (FA FB in payload data)
                // Remove just the false FA byte and continue loop to find real sync
                self.buffer.drain(..1);
                // Continue loop to try again with remaining data
            }
        }
    }

    /// Verify checksum of packet in buffer
    fn verify_checksum(&self, total_len: usize) -> bool {
        // Get received CRC (big-endian, at end of packet)
        let crc_high = self.buffer[total_len - 2];
        let crc_low = self.buffer[total_len - 1];
        let received_crc = ((crc_high as u16) << 8) | (crc_low as u16);

        // Calculate checksum over CMD + PAYLOAD (bytes 3 to total_len-2)
        // This is: CMD + PAYLOAD, excluding SYNC(2), LEN(1), and CRC(2)
        let data = &self.buffer[3..total_len - 2];
        let calculated_crc = checksum(data);

        if calculated_crc != received_crc {
            log::warn!(
                "CRC mismatch: received=0x{:04X}, calculated=0x{:04X}, len={}, packet={:02X?}",
                received_crc,
                calculated_crc,
                total_len,
                &self.buffer[..total_len]
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
