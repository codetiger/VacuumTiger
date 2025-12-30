//! LiDAR-IRIS descriptor for loop closure detection.
//!
//! A compact 80-byte binary descriptor that encodes the structure of a lidar scan
//! in a rotation-invariant way for efficient place recognition.

use crate::core::LidarScan;

/// Number of radial rings in the descriptor.
const NUM_RINGS: usize = 8;

/// Number of angular sectors in the descriptor.
const NUM_SECTORS: usize = 80;

/// Total descriptor size in bytes (8 rings × 80 sectors = 640 bits = 80 bytes).
const DESCRIPTOR_SIZE: usize = NUM_RINGS * NUM_SECTORS / 8;

/// LiDAR-IRIS binary descriptor.
///
/// Encodes lidar scan structure in a compact 80-byte binary format.
/// Designed for fast Hamming distance computation for loop closure detection.
#[derive(Clone, Debug)]
pub struct LidarIris {
    /// Binary descriptor data (80 bytes = 640 bits).
    data: [u8; DESCRIPTOR_SIZE],
    /// Maximum range used when creating descriptor.
    max_range: f32,
}

impl LidarIris {
    /// Create a new descriptor from a lidar scan.
    ///
    /// The descriptor encodes occupancy in a polar grid:
    /// - 8 radial rings (from center outward)
    /// - 80 angular sectors (around 360°)
    ///
    /// Each cell is 1 if there's a lidar hit in that cell, 0 otherwise.
    pub fn from_scan(scan: &LidarScan) -> Self {
        let max_range = scan.range_max;
        let ring_width = max_range / NUM_RINGS as f32;
        let sector_width = std::f32::consts::TAU / NUM_SECTORS as f32;

        let mut data = [0u8; DESCRIPTOR_SIZE];

        // Process each valid scan point
        for (angle, range) in scan.valid_points() {
            // Determine which ring (0 = closest to robot)
            let ring = ((range / ring_width) as usize).min(NUM_RINGS - 1);

            // Determine which sector (normalize angle to 0..TAU)
            let normalized_angle = if angle < 0.0 {
                angle + std::f32::consts::TAU
            } else {
                angle
            };
            let sector = ((normalized_angle / sector_width) as usize).min(NUM_SECTORS - 1);

            // Calculate bit position
            // Layout: ring 0 sectors 0-79, ring 1 sectors 0-79, etc.
            let bit_index = ring * NUM_SECTORS + sector;
            let byte_index = bit_index / 8;
            let bit_offset = bit_index % 8;

            // Set the bit
            data[byte_index] |= 1 << bit_offset;
        }

        Self { data, max_range }
    }

    /// Create an empty descriptor.
    pub fn empty() -> Self {
        Self {
            data: [0u8; DESCRIPTOR_SIZE],
            max_range: 10.0,
        }
    }

    /// Get the raw descriptor data.
    pub fn data(&self) -> &[u8; DESCRIPTOR_SIZE] {
        &self.data
    }

    /// Calculate Hamming distance to another descriptor.
    ///
    /// Returns the number of differing bits (0 to 640).
    /// Lower distance = more similar scans.
    pub fn distance(&self, other: &LidarIris) -> u32 {
        let mut dist = 0u32;
        for i in 0..DESCRIPTOR_SIZE {
            dist += (self.data[i] ^ other.data[i]).count_ones();
        }
        dist
    }

    /// Calculate normalized similarity (0.0 to 1.0).
    ///
    /// 1.0 = identical, 0.0 = completely different.
    pub fn similarity(&self, other: &LidarIris) -> f32 {
        let dist = self.distance(other);
        1.0 - (dist as f32 / (DESCRIPTOR_SIZE * 8) as f32)
    }

    /// Count the number of set bits (occupied cells).
    pub fn popcount(&self) -> u32 {
        self.data.iter().map(|b| b.count_ones()).sum()
    }

    /// Check if descriptor is mostly empty (low occupancy).
    pub fn is_sparse(&self) -> bool {
        self.popcount() < 20
    }

    /// Rotate the descriptor by a number of sectors.
    ///
    /// This is useful for rotation-invariant matching.
    /// Positive rotation = counterclockwise.
    pub fn rotated(&self, sectors: i32) -> Self {
        let mut rotated_data = [0u8; DESCRIPTOR_SIZE];

        for ring in 0..NUM_RINGS {
            for sector in 0..NUM_SECTORS {
                // Calculate source sector (with wrapping)
                let src_sector =
                    ((sector as i32 - sectors).rem_euclid(NUM_SECTORS as i32)) as usize;

                // Copy bit from source to destination
                let src_bit_index = ring * NUM_SECTORS + src_sector;
                let src_byte = src_bit_index / 8;
                let src_offset = src_bit_index % 8;

                let dst_bit_index = ring * NUM_SECTORS + sector;
                let dst_byte = dst_bit_index / 8;
                let dst_offset = dst_bit_index % 8;

                if (self.data[src_byte] >> src_offset) & 1 != 0 {
                    rotated_data[dst_byte] |= 1 << dst_offset;
                }
            }
        }

        Self {
            data: rotated_data,
            max_range: self.max_range,
        }
    }

    /// Find the best rotation-invariant match.
    ///
    /// Tests all rotations and returns the minimum distance
    /// along with the rotation offset that achieved it.
    pub fn best_match(&self, other: &LidarIris) -> (u32, i32) {
        let mut best_dist = self.distance(other);
        let mut best_rotation = 0;

        // Test rotations at coarse resolution first
        let coarse_step = 4;
        for i in (coarse_step..NUM_SECTORS as i32).step_by(coarse_step as usize) {
            let rotated = self.rotated(i);
            let dist = rotated.distance(other);
            if dist < best_dist {
                best_dist = dist;
                best_rotation = i;
            }
        }

        // Fine search around best coarse rotation
        for offset in -coarse_step..=coarse_step {
            let rot = best_rotation + offset;
            if rot == best_rotation {
                continue;
            }
            let rotated = self.rotated(rot);
            let dist = rotated.distance(other);
            if dist < best_dist {
                best_dist = dist;
                best_rotation = rot;
            }
        }

        (best_dist, best_rotation)
    }
}

impl Default for LidarIris {
    fn default() -> Self {
        Self::empty()
    }
}

impl PartialEq for LidarIris {
    fn eq(&self, other: &Self) -> bool {
        self.data == other.data
    }
}

impl Eq for LidarIris {}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_scan(ranges: &[f32]) -> LidarScan {
        let num_points = ranges.len();
        let angles: Vec<f32> = (0..num_points)
            .map(|i| -std::f32::consts::PI + i as f32 * std::f32::consts::TAU / num_points as f32)
            .collect();
        LidarScan::new(ranges.to_vec(), angles, 0.1, 10.0)
    }

    #[test]
    fn test_empty_descriptor() {
        let desc = LidarIris::empty();
        assert_eq!(desc.popcount(), 0);
        assert!(desc.is_sparse());
    }

    #[test]
    fn test_from_scan() {
        // Create a scan with some points
        let ranges = vec![2.0; 360];
        let scan = create_test_scan(&ranges);

        let desc = LidarIris::from_scan(&scan);
        assert!(desc.popcount() > 0);
    }

    #[test]
    fn test_identical_distance() {
        let ranges = vec![2.0; 360];
        let scan = create_test_scan(&ranges);

        let desc1 = LidarIris::from_scan(&scan);
        let desc2 = LidarIris::from_scan(&scan);

        assert_eq!(desc1.distance(&desc2), 0);
        assert!((desc1.similarity(&desc2) - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_different_scans() {
        let scan1 = create_test_scan(&vec![2.0; 360]);
        let scan2 = create_test_scan(&vec![5.0; 360]); // Different range

        let desc1 = LidarIris::from_scan(&scan1);
        let desc2 = LidarIris::from_scan(&scan2);

        // Should be different (different rings)
        assert!(desc1.distance(&desc2) > 0);
    }

    #[test]
    fn test_rotation() {
        let ranges = vec![2.0; 360];
        let scan = create_test_scan(&ranges);
        let desc = LidarIris::from_scan(&scan);

        // Rotating by 0 should give same descriptor
        let rotated = desc.rotated(0);
        assert_eq!(desc.distance(&rotated), 0);

        // Full rotation should also give same descriptor
        let full_rotation = desc.rotated(NUM_SECTORS as i32);
        assert_eq!(desc.distance(&full_rotation), 0);
    }

    #[test]
    fn test_best_match() {
        let ranges = vec![2.0; 360];
        let scan = create_test_scan(&ranges);
        let desc = LidarIris::from_scan(&scan);

        // Match with itself should have distance 0
        let (dist, _) = desc.best_match(&desc);
        assert_eq!(dist, 0);
    }

    #[test]
    fn test_descriptor_size() {
        assert_eq!(DESCRIPTOR_SIZE, 80);
        assert_eq!(NUM_RINGS * NUM_SECTORS, 640);
    }
}
