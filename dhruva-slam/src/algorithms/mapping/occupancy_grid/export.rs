//! Export functionality for occupancy grids.
//!
//! Supports ROS-standard PGM format for map interchange.

use std::io;
use std::path::Path;

use super::OccupancyGrid;
use super::config::{CellState, OccupancyGridConfig};

/// Log-odds values for PGM import
const LOG_ODDS_FREE: f32 = -3.0;
const LOG_ODDS_OCCUPIED: f32 = 3.0;
const LOG_ODDS_UNKNOWN: f32 = 0.0;

impl OccupancyGrid {
    /// Get memory usage in bytes.
    pub fn memory_usage(&self) -> usize {
        self.width() * self.height() * std::mem::size_of::<f32>()
    }

    /// Count cells by state (for testing and diagnostics).
    ///
    /// Returns (free_count, unknown_count, occupied_count).
    pub fn count_cells(&self) -> (usize, usize, usize) {
        let mut free = 0;
        let mut unknown = 0;
        let mut occupied = 0;

        for cy in 0..self.height() {
            for cx in 0..self.width() {
                let state = self.get_state(cx, cy);
                match state {
                    CellState::Free => free += 1,
                    CellState::Unknown => unknown += 1,
                    CellState::Occupied => occupied += 1,
                }
            }
        }

        (free, unknown, occupied)
    }

    /// Load map from PGM file (ROS-standard format).
    ///
    /// Converts grayscale pixel values to log-odds:
    /// - Values near 255 → free (log_odds = -3.0)
    /// - Values near 0 → occupied (log_odds = 3.0)
    /// - Values near 205 → unknown (log_odds = 0.0)
    ///
    /// # Arguments
    /// * `path` - Path to PGM file
    /// * `resolution` - Map resolution in meters per pixel
    /// * `origin_x` - World X coordinate of bottom-left pixel
    /// * `origin_y` - World Y coordinate of bottom-left pixel
    pub fn load_pgm<P: AsRef<Path>>(
        path: P,
        resolution: f32,
        origin_x: f32,
        origin_y: f32,
    ) -> io::Result<Self> {
        let img = image::open(path)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?
            .into_luma8();

        let width = img.width() as usize;
        let height = img.height() as usize;

        // Create occupancy grid with matching dimensions
        let config = OccupancyGridConfig {
            resolution,
            initial_width: width as f32 * resolution,
            initial_height: height as f32 * resolution,
            ..Default::default()
        };

        let mut cells = vec![LOG_ODDS_UNKNOWN; width * height];

        // Convert pixels to log-odds
        // PGM has Y=0 at top, map has Y=0 at bottom, so flip Y
        for py in 0..height {
            for px in 0..width {
                let pixel = img.get_pixel(px as u32, py as u32).0[0];
                let cy = height - 1 - py; // Flip Y

                let log_odds = pixel_to_log_odds(pixel);
                cells[cy * width + px] = log_odds;
            }
        }

        Ok(Self::from_raw(
            config, cells, width, height, origin_x, origin_y,
        ))
    }
}

/// Convert pixel grayscale value to log-odds.
///
/// Uses threshold-based conversion matching ROS conventions:
/// - pixel >= 240 → free (white)
/// - pixel <= 20 → occupied (black)
/// - otherwise → unknown (gray)
fn pixel_to_log_odds(pixel: u8) -> f32 {
    if pixel >= 240 {
        LOG_ODDS_FREE
    } else if pixel <= 20 {
        LOG_ODDS_OCCUPIED
    } else {
        LOG_ODDS_UNKNOWN
    }
}
