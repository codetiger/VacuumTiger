//! Map loader for PGM+YAML format (ROS standard)
//!
//! Loads occupancy grid maps with optional cliff mask layer.

use crate::error::{Error, Result};
use image::GrayImage;
use serde::Deserialize;
use std::path::Path;

/// Map metadata from YAML file (ROS standard format)
#[derive(Debug, Deserialize)]
pub struct MapMetadata {
    /// PGM image filename (relative to YAML file)
    pub image: String,

    /// Map resolution in meters per pixel
    pub resolution: f32,

    /// Origin of map [x, y, yaw] - world coordinates of bottom-left pixel
    pub origin: [f32; 3],

    /// Threshold for occupied cells (0.0-1.0, pixel > threshold*255 = occupied)
    #[serde(default = "default_occupied_thresh")]
    pub occupied_thresh: f32,

    /// Optional cliff mask file (same dimensions as main map)
    #[serde(default)]
    pub cliff_mask: Option<String>,
}

fn default_occupied_thresh() -> f32 {
    0.65
}

/// Simulation map loaded from PGM+YAML
pub struct SimulationMap {
    /// Occupancy grid pixels
    pixels: GrayImage,

    /// Optional cliff mask layer
    cliff_mask: Option<GrayImage>,

    /// Resolution in meters per pixel
    resolution: f32,

    /// Origin (x, y) in world coordinates
    origin: (f32, f32),

    /// Pixel threshold for occupied (0-255)
    occupied_thresh: u8,
}

impl SimulationMap {
    /// Create map from test image (for unit tests)
    #[cfg(test)]
    pub fn from_test_image(pixels: GrayImage, resolution: f32, origin: (f32, f32)) -> Self {
        Self {
            pixels,
            cliff_mask: None,
            resolution,
            origin,
            occupied_thresh: 128,
        }
    }

    /// Load map from ROS-standard YAML + PGM files
    pub fn load<P: AsRef<Path>>(yaml_path: P) -> Result<Self> {
        let yaml_path = yaml_path.as_ref();

        // Read and parse YAML
        let yaml_content = std::fs::read_to_string(yaml_path)
            .map_err(|e| Error::Config(format!("Failed to read map YAML: {}", e)))?;

        let metadata: MapMetadata = serde_yaml::from_str(&yaml_content)
            .map_err(|e| Error::Config(format!("Failed to parse map YAML: {}", e)))?;

        // Determine base directory for relative paths
        let yaml_dir = yaml_path.parent().unwrap_or(Path::new("."));

        // Load main PGM image
        let pgm_path = yaml_dir.join(&metadata.image);
        let img = image::open(&pgm_path)
            .map_err(|e| {
                Error::Config(format!(
                    "Failed to load map image {}: {}",
                    pgm_path.display(),
                    e
                ))
            })?
            .into_luma8();

        // Load optional cliff mask
        let cliff_mask = if let Some(cliff_file) = &metadata.cliff_mask {
            let cliff_path = yaml_dir.join(cliff_file);
            let cliff_img = image::open(&cliff_path)
                .map_err(|e| {
                    Error::Config(format!(
                        "Failed to load cliff mask {}: {}",
                        cliff_path.display(),
                        e
                    ))
                })?
                .into_luma8();

            // Validate dimensions match
            if cliff_img.dimensions() != img.dimensions() {
                return Err(Error::Config(format!(
                    "Cliff mask dimensions {:?} don't match map dimensions {:?}",
                    cliff_img.dimensions(),
                    img.dimensions()
                )));
            }

            Some(cliff_img)
        } else {
            None
        };

        // Convert threshold to pixel value
        // occupied_thresh: pixels BELOW this are occupied (darker = occupied)
        let occupied_thresh = ((1.0 - metadata.occupied_thresh) * 255.0) as u8;

        Ok(Self {
            pixels: img,
            cliff_mask,
            resolution: metadata.resolution,
            origin: (metadata.origin[0], metadata.origin[1]),
            occupied_thresh,
        })
    }

    /// Get map width in pixels
    pub fn width(&self) -> u32 {
        self.pixels.width()
    }

    /// Get map height in pixels
    pub fn height(&self) -> u32 {
        self.pixels.height()
    }

    /// Get map resolution in meters per pixel
    pub fn resolution(&self) -> f32 {
        self.resolution
    }

    /// Get map origin (world coordinates of bottom-left pixel)
    pub fn origin(&self) -> (f32, f32) {
        self.origin
    }

    /// Convert pixel coordinates to world coordinates
    ///
    /// Returns (x, y) in meters. The center of the pixel is returned.
    pub fn pixel_to_world(&self, px: u32, py: u32) -> (f32, f32) {
        let x = self.origin.0 + (px as f32 + 0.5) * self.resolution;
        // Y is inverted: bottom of image = origin, top of image = +Y
        let y = self.origin.1 + (self.pixels.height() as f32 - 1.0 - py as f32 + 0.5) * self.resolution;
        (x, y)
    }

    /// Convert world coordinates to pixel coordinates
    fn world_to_pixel(&self, x: f32, y: f32) -> Option<(u32, u32)> {
        let px = ((x - self.origin.0) / self.resolution) as i32;
        // Y is inverted: bottom of image = origin, top of image = +Y
        let py = (self.pixels.height() as i32 - 1) - ((y - self.origin.1) / self.resolution) as i32;

        if px >= 0
            && py >= 0
            && (px as u32) < self.pixels.width()
            && (py as u32) < self.pixels.height()
        {
            Some((px as u32, py as u32))
        } else {
            None
        }
    }

    /// Check if world coordinate is occupied
    pub fn is_occupied(&self, x: f32, y: f32) -> bool {
        match self.world_to_pixel(x, y) {
            Some((px, py)) => {
                let pixel = self.pixels.get_pixel(px, py).0[0];
                pixel < self.occupied_thresh // Darker = occupied
            }
            None => true, // Out of bounds = occupied (wall)
        }
    }

    /// Check if world coordinate is a cliff (from mask or map boundary)
    pub fn is_cliff(&self, x: f32, y: f32) -> bool {
        match self.world_to_pixel(x, y) {
            Some((px, py)) => {
                if let Some(mask) = &self.cliff_mask {
                    let pixel = mask.get_pixel(px, py).0[0];
                    pixel < 128 // Black = cliff, White = safe
                } else {
                    false // No mask, not a cliff
                }
            }
            None => true, // Out of bounds = cliff
        }
    }

    /// Ray-cast from origin in direction, return distance to obstacle
    ///
    /// Returns max_range if no obstacle is hit.
    pub fn ray_cast(&self, ox: f32, oy: f32, angle: f32, max_range: f32) -> f32 {
        // Step size: half resolution for accuracy
        let step = self.resolution * 0.5;
        let dx = angle.cos() * step;
        let dy = angle.sin() * step;

        let mut x = ox;
        let mut y = oy;
        let mut distance = 0.0;

        while distance < max_range {
            x += dx;
            y += dy;
            distance += step;

            if self.is_occupied(x, y) {
                return distance;
            }
        }

        max_range // No hit
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_world_to_pixel() {
        // Create a simple 100x100 map with 0.1m resolution
        // Origin at (-5, -5), so map covers (-5,-5) to (5,5) in world coords
        let img = GrayImage::from_fn(100, 100, |_, _| image::Luma([255u8]));
        let map = SimulationMap {
            pixels: img,
            cliff_mask: None,
            resolution: 0.1,
            origin: (-5.0, -5.0),
            occupied_thresh: 128,
        };

        // Origin should map to bottom-left pixel
        assert_eq!(map.world_to_pixel(-5.0, -5.0), Some((0, 99)));

        // Center should map to center pixel
        assert_eq!(map.world_to_pixel(0.0, 0.0), Some((50, 49)));

        // Out of bounds should return None
        assert_eq!(map.world_to_pixel(-10.0, 0.0), None);
        assert_eq!(map.world_to_pixel(10.0, 0.0), None);
    }
}
