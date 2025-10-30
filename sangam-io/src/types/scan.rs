//! Lidar scan types

/// A single lidar measurement point
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LidarPoint {
    /// Angle in radians (0 to 2π)
    pub angle: f32,
    /// Distance in meters
    pub distance: f32,
    /// Signal quality/intensity (0-255)
    pub quality: u8,
}

impl LidarPoint {
    /// Create new lidar point
    pub fn new(angle: f32, distance: f32, quality: u8) -> Self {
        Self {
            angle,
            distance,
            quality,
        }
    }

    /// Convert to Cartesian coordinates (x, y)
    pub fn to_cartesian(&self) -> (f32, f32) {
        let x = self.distance * self.angle.cos();
        let y = self.distance * self.angle.sin();
        (x, y)
    }
}

/// A complete 360° lidar scan
#[derive(Debug, Clone, PartialEq)]
pub struct LidarScan {
    /// Measurement points
    pub points: Vec<LidarPoint>,
    /// Scan timestamp in milliseconds (if available)
    pub timestamp_ms: Option<u64>,
}

impl LidarScan {
    /// Create a new empty scan
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            timestamp_ms: None,
        }
    }

    /// Create scan with capacity
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
            timestamp_ms: None,
        }
    }

    /// Add a point to the scan
    pub fn add_point(&mut self, point: LidarPoint) {
        self.points.push(point);
    }

    /// Get the number of points
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if scan is empty
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Get points within distance range
    pub fn points_in_range(&self, min_dist: f32, max_dist: f32) -> Vec<&LidarPoint> {
        self.points
            .iter()
            .filter(|p| p.distance >= min_dist && p.distance <= max_dist)
            .collect()
    }
}

impl Default for LidarScan {
    fn default() -> Self {
        Self::new()
    }
}
