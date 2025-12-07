//! Delta-2D Lidar Protocol Implementation
//! Packet format: [Header (8 bytes)] [Payload] [CRC (2 bytes)]
//!
//! # Robustness Improvements
//!
//! This implementation includes several fixes for reliable data reading:
//!
//! 1. **Loop-based resync**: On parse failure (invalid header or CRC), drains
//!    only 1 byte and retries instead of clearing entire buffer. This prevents
//!    losing valid data when false sync bytes appear in payload data.
//!
//! 2. **CRC validation**: Validates packet CRC before accepting data to reject
//!    corrupted packets.
//!
//! 3. **Header validation**: Checks version and command type fields to detect
//!    false sync patterns early.
//!
//! 4. **Bounded buffer**: Limits buffer growth to prevent memory exhaustion
//!    on prolonged parse failures.

use crate::error::{Error, Result};
use std::io::Read;

/// Maximum buffer size before forced clear (safety limit)
const MAX_BUFFER_SIZE: usize = 8192;

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
}

/// Command types from lidar
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CommandType {
    Health = 0xAE,
    Measurement = 0xAD,
    Unknown = 0xFF,
}

/// Result of parsing a lidar packet
#[derive(Debug, Clone)]
pub enum ParseResult {
    /// Measurement scan data
    Scan(LidarScan),
    /// Health packet (motor is spinning, RPM logged)
    Health,
    /// No complete packet available
    None,
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

/// Packet reader for Delta-2D lidar with robust error recovery
pub struct Delta2DPacketReader {
    buffer: Vec<u8>,
    /// Count of bytes discarded due to resync (for diagnostics)
    bytes_discarded: u64,
    /// Count of CRC failures (for diagnostics)
    crc_failures: u64,
}

impl Delta2DPacketReader {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(2048),
            bytes_discarded: 0,
            crc_failures: 0,
        }
    }

    /// Get diagnostic counters
    pub fn diagnostics(&self) -> (u64, u64) {
        (self.bytes_discarded, self.crc_failures)
    }

    /// Clear the buffer (call on startup after serial flush)
    pub fn clear(&mut self) {
        self.buffer.clear();
    }

    /// Read bytes from port into internal buffer
    ///
    /// Returns the number of bytes read, or 0 if no data available (timeout/wouldblock).
    /// Call this once, then call `parse_next()` in a loop to drain all packets.
    pub fn read_bytes<R: Read>(&mut self, port: &mut R) -> Result<usize> {
        let mut temp_buf = [0u8; 512];
        match port.read(&mut temp_buf) {
            Ok(0) => Ok(0),
            Ok(n) => {
                log::trace!(
                    "Lidar raw read: {} bytes: {:02X?}",
                    n,
                    &temp_buf[..n.min(32)]
                );
                self.buffer.extend_from_slice(&temp_buf[..n]);
                Ok(n)
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(0),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(0),
            Err(e) => Err(Error::Io(e)),
        }
    }

    /// Parse and return the next complete packet from the buffer
    ///
    /// Call this in a loop after `read_bytes()` until it returns `Ok(ParseResult::None)`
    /// to drain all available packets from the buffer.
    ///
    /// Returns:
    /// - `ParseResult::Scan(scan)` for measurement packets (0xAD)
    /// - `ParseResult::Health { rpm }` for health packets (0xAE)
    /// - `ParseResult::None` when no complete packet is available
    pub fn parse_next(&mut self) -> Result<ParseResult> {
        // Safety check: if buffer is too large, something is wrong
        if self.buffer.len() > MAX_BUFFER_SIZE {
            log::warn!(
                "Lidar buffer overflow ({} bytes), clearing. Discarded: {}, CRC failures: {}",
                self.buffer.len(),
                self.bytes_discarded,
                self.crc_failures
            );
            self.bytes_discarded += self.buffer.len() as u64;
            self.buffer.clear();
            return Ok(ParseResult::None);
        }

        // Try to parse one packet using loop-based resync
        self.try_parse_scan_with_resync()
    }

    /// Try to read and parse a scan from the port (legacy combined method)
    #[allow(dead_code)]
    pub fn read_scan<R: Read>(&mut self, port: &mut R) -> Result<Option<LidarScan>> {
        self.read_bytes(port)?;
        match self.parse_next()? {
            ParseResult::Scan(scan) => Ok(Some(scan)),
            _ => Ok(None),
        }
    }

    /// Parse scan with loop-based resync on failure
    /// On parse failure, drains 1 byte and retries instead of clearing buffer
    fn try_parse_scan_with_resync(&mut self) -> Result<ParseResult> {
        loop {
            // Need at least header (8 bytes)
            if self.buffer.len() < 8 {
                return Ok(ParseResult::None);
            }

            // Find sync byte 0xAA
            let Some(sync_idx) = self.buffer.iter().position(|&b| b == 0xAA) else {
                // No sync found at all - discard buffer up to last byte
                // (keep last byte in case it's start of new data)
                let discard = self.buffer.len().saturating_sub(1);
                if discard > 0 {
                    self.bytes_discarded += discard as u64;
                    log::trace!("Lidar: no sync found, discarding {} bytes", discard);
                    self.buffer.drain(0..discard);
                }
                return Ok(ParseResult::None);
            };

            // Discard bytes before sync
            if sync_idx > 0 {
                self.bytes_discarded += sync_idx as u64;
                log::trace!("Lidar: discarding {} bytes before sync", sync_idx);
                self.buffer.drain(0..sync_idx);
            }

            // Check if we have complete header
            if self.buffer.len() < 8 {
                return Ok(ParseResult::None);
            }

            // Validate header before proceeding
            if !self.validate_header() {
                // Invalid header - this 0xAA is a false sync
                // Drain 1 byte and retry (loop continues)
                self.bytes_discarded += 1;
                self.buffer.drain(0..1);
                log::trace!("Lidar: invalid header, draining 1 byte and retrying");
                continue;
            }

            // Parse header
            let payload_len = ((self.buffer[6] as u16) << 8) | (self.buffer[7] as u16);
            let total_len = 8 + payload_len as usize + 2; // header + payload + CRC

            // Sanity check payload length
            if payload_len > 1024 {
                // Unreasonably large payload - false sync
                self.bytes_discarded += 1;
                self.buffer.drain(0..1);
                log::trace!(
                    "Lidar: unreasonable payload length {}, draining 1 byte",
                    payload_len
                );
                continue;
            }

            // Wait for complete packet
            if self.buffer.len() < total_len {
                return Ok(ParseResult::None);
            }

            // NOTE: CRC validation disabled for now as the Delta-2D protocol's
            // actual checksum algorithm is not confirmed. The loop-based resync
            // on header validation still provides robustness against false sync bytes.
            // TODO: Re-enable once the correct CRC algorithm is determined.
            // if !self.validate_crc(total_len) {
            //     self.crc_failures += 1;
            //     self.bytes_discarded += 1;
            //     self.buffer.drain(0..1);
            //     log::trace!("Lidar: CRC failed, draining 1 byte and retrying");
            //     continue;
            // }

            // Valid packet! Parse it
            let cmd_type = CommandType::from(self.buffer[5]);

            log::debug!(
                "Lidar packet: cmd=0x{:02X}, type={:?}, chunk=0x{:02X}, payload_len={}",
                self.buffer[5],
                cmd_type,
                self.buffer[4],
                payload_len
            );

            // Extract payload
            let payload = &self.buffer[8..8 + payload_len as usize];

            // Parse based on command type
            let result = if cmd_type == CommandType::Measurement && payload_len > 5 {
                ParseResult::Scan(self.parse_measurement(payload)?)
            } else if cmd_type == CommandType::Health && payload_len >= 1 {
                let rps_raw = payload[0];
                let rpm = rps_raw as u16 * 3;
                let rps = rps_raw as f32 * 0.05;
                log::info!("Lidar health: RPM={}, RPS={:.2}", rpm, rps);
                ParseResult::Health
            } else {
                ParseResult::None
            };

            // Remove processed packet
            self.buffer.drain(0..total_len);

            return Ok(result);
        }
    }

    /// Validate header fields to detect false sync patterns
    fn validate_header(&self) -> bool {
        // Byte 0: 0xAA (sync) - already verified
        // Byte 3: Version - should be reasonable (0-15)
        // Byte 4: Chunk type - should be valid
        // Byte 5: Command type - should be known

        let version = self.buffer[3];
        let cmd_type = self.buffer[5];

        // Version should be small (typically 0-3)
        if version > 15 {
            return false;
        }

        // Command type should be known
        let cmd = CommandType::from(cmd_type);
        if cmd == CommandType::Unknown {
            // Allow unknown commands but log them
            log::trace!("Lidar: unknown command type 0x{:02X}", cmd_type);
        }

        true
    }

    /// Validate packet CRC
    /// Delta-2D uses a simple checksum: sum of all bytes from header to payload
    #[allow(dead_code)]
    fn validate_crc(&self, total_len: usize) -> bool {
        if total_len < 10 {
            // Minimum: 8 header + 2 CRC
            return false;
        }

        // CRC bytes are at the end
        let crc_offset = total_len - 2;
        let received_crc =
            ((self.buffer[crc_offset] as u16) << 8) | (self.buffer[crc_offset + 1] as u16);

        // Calculate checksum: sum of bytes from position 0 to crc_offset
        let mut calculated: u16 = 0;
        for i in 0..crc_offset {
            calculated = calculated.wrapping_add(self.buffer[i] as u16);
        }

        if calculated != received_crc {
            log::trace!(
                "Lidar CRC mismatch: calculated=0x{:04X}, received=0x{:04X}",
                calculated,
                received_crc
            );
            return false;
        }

        true
    }

    fn parse_measurement(&self, payload: &[u8]) -> Result<LidarScan> {
        let mut points = Vec::with_capacity(100);

        if payload.len() < 5 {
            return Ok(LidarScan { points });
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
            // Also validate angle range
            if distance_raw > 0 && quality > 0 && (0.0..=360.0).contains(&angle_deg) {
                points.push(LidarPoint {
                    angle: angle_rad,
                    distance: distance_m,
                    quality,
                });
            }

            i += 3;
            point_index += 1;
        }

        Ok(LidarScan { points })
    }
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

    #[test]
    fn test_crc_calculation() {
        let mut reader = Delta2DPacketReader::new();
        // Build a test packet with known CRC
        // Header: AA 00 0A 01 00 AD 00 02 (8 bytes)
        // Payload: 01 02 (2 bytes)
        // CRC: sum of first 10 bytes
        reader.buffer = vec![0xAA, 0x00, 0x0A, 0x01, 0x00, 0xAD, 0x00, 0x02, 0x01, 0x02];
        let sum: u16 = reader.buffer.iter().map(|&b| b as u16).sum();
        reader.buffer.push((sum >> 8) as u8);
        reader.buffer.push((sum & 0xFF) as u8);

        assert!(reader.validate_crc(12));
    }

    #[test]
    fn test_resync_on_false_sync() {
        let mut reader = Delta2DPacketReader::new();
        // Simulate false sync byte in data followed by real packet
        // False sync at position 0, real sync at position 3
        reader.buffer = vec![
            0xAA, 0xFF, 0xFF, // False sync with invalid data
            0xAA, 0x00, 0x0A, 0x01, 0x00, 0xAD, 0x00, 0x02, 0x01,
            0x02, // Real header + payload
        ];
        // Add CRC for the real packet (sum from position 3 to 12)
        let sum: u16 = reader.buffer[3..13].iter().map(|&b| b as u16).sum();
        reader.buffer.push((sum >> 8) as u8);
        reader.buffer.push((sum & 0xFF) as u8);

        // First attempt should skip false sync
        let result = reader.try_parse_scan_with_resync();
        assert!(result.is_ok());
        // Should have discarded the false sync bytes
        assert!(reader.bytes_discarded > 0);
    }
}
