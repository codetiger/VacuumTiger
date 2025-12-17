//! Map storage and management using ROS-standard PGM+YAML format.
//!
//! This module provides:
//! - Saving and loading occupancy grid maps in PGM+YAML format
//! - Map metadata management (name, timestamps, area)
//! - Map list enumeration
//!
//! ## File Format
//!
//! Maps are stored as two files:
//! - `{map_id}.pgm` - Binary PGM grayscale image (occupancy grid)
//! - `{map_id}.yaml` - YAML metadata (resolution, origin, dhruva extensions)
//!
//! This format is compatible with ROS Nav2 map_server.
//!
//! Note: Some utility methods are defined for future use.

use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::algorithms::mapping::OccupancyGrid;
use crate::state::{CurrentMapData, MapList, MapSummary};

/// Default occupancy threshold for ROS compatibility.
const DEFAULT_OCCUPIED_THRESH: f32 = 0.65;

/// Map metadata in ROS-standard YAML format with Dhruva extensions.
///
/// The first fields (image, resolution, origin, occupied_thresh) are
/// ROS-standard and compatible with Nav2 map_server.
/// The remaining fields are Dhruva-specific extensions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapInfo {
    // === ROS-standard fields ===
    /// PGM image filename (relative to YAML file).
    pub image: String,

    /// Map resolution in meters per pixel.
    pub resolution: f32,

    /// Origin of map [x, y, yaw] - world coordinates of bottom-left pixel.
    pub origin: [f32; 3],

    /// Threshold for occupied cells (0.0-1.0).
    #[serde(default = "default_occupied_thresh")]
    pub occupied_thresh: f32,

    // === Dhruva extensions ===
    /// Unique map identifier.
    pub map_id: String,

    /// User-friendly name.
    pub name: String,

    /// Creation timestamp (microseconds since epoch).
    pub created_at: u64,

    /// Last update timestamp (microseconds since epoch).
    pub updated_at: u64,

    /// Map area in square meters.
    pub area_m2: f32,

    /// Number of detected rooms.
    pub room_count: u32,

    /// Whether mapping was completed.
    pub is_complete: bool,

    /// Map width in cells.
    pub width: usize,

    /// Map height in cells.
    pub height: usize,
}

fn default_occupied_thresh() -> f32 {
    DEFAULT_OCCUPIED_THRESH
}

/// Map manager for persistent map storage.
pub struct MapManager {
    /// Base directory for map storage.
    base_path: PathBuf,
    /// Cached map metadata (map_id -> MapInfo).
    map_cache: HashMap<String, MapInfo>,
}

impl MapManager {
    /// Create a new map manager.
    ///
    /// Creates the storage directory if it doesn't exist.
    pub fn new(base_path: &Path) -> Result<Self, String> {
        // Create directory if it doesn't exist
        if !base_path.exists() {
            fs::create_dir_all(base_path)
                .map_err(|e| format!("Failed to create map storage directory: {}", e))?;
        }

        let mut manager = Self {
            base_path: base_path.to_path_buf(),
            map_cache: HashMap::new(),
        };

        // Load existing map metadata
        manager.reload_cache()?;

        Ok(manager)
    }

    /// Reload the map cache from disk.
    pub fn reload_cache(&mut self) -> Result<(), String> {
        self.map_cache.clear();

        // Read all .yaml files in the directory
        let entries = fs::read_dir(&self.base_path)
            .map_err(|e| format!("Failed to read map directory: {}", e))?;

        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().is_some_and(|ext| ext == "yaml")
                && let Ok(info) = self.load_map_info(&path)
            {
                self.map_cache.insert(info.map_id.clone(), info);
            }
        }

        log::info!("Loaded {} maps from storage", self.map_cache.len());
        Ok(())
    }

    /// Load map metadata from a .yaml file.
    fn load_map_info(&self, path: &Path) -> Result<MapInfo, String> {
        let content =
            fs::read_to_string(path).map_err(|e| format!("Failed to read YAML file: {}", e))?;
        serde_yaml::from_str(&content).map_err(|e| format!("Failed to parse YAML: {}", e))
    }

    /// Save map metadata to a .yaml file.
    fn save_map_info(&self, info: &MapInfo) -> Result<(), String> {
        let path = self.yaml_path(&info.map_id);
        let file = File::create(&path).map_err(|e| format!("Failed to create YAML file: {}", e))?;
        let mut writer = BufWriter::new(file);

        // Write header comment for ROS compatibility
        writeln!(writer, "# Map saved by Dhruva SLAM")
            .map_err(|e| format!("Write error: {}", e))?;
        writeln!(writer, "# ROS-standard format with Dhruva extensions")
            .map_err(|e| format!("Write error: {}", e))?;
        writeln!(writer).map_err(|e| format!("Write error: {}", e))?;

        serde_yaml::to_writer(&mut writer, info).map_err(|e| format!("Failed to write YAML: {}", e))
    }

    /// Get path for PGM image file.
    fn pgm_path(&self, map_id: &str) -> PathBuf {
        self.base_path.join(format!("{}.pgm", map_id))
    }

    /// Get path for YAML metadata file.
    fn yaml_path(&self, map_id: &str) -> PathBuf {
        self.base_path.join(format!("{}.yaml", map_id))
    }

    /// Save current mapping session.
    ///
    /// Saves the map as PGM image + YAML metadata.
    pub fn save_current_map(
        &mut self,
        map_id: &str,
        name: &str,
        map_data: &CurrentMapData,
    ) -> Result<(), String> {
        let now_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);

        // Create map info with ROS-standard fields
        let info = MapInfo {
            // ROS-standard fields
            image: format!("{}.pgm", map_id),
            resolution: map_data.resolution,
            origin: [map_data.origin_x, map_data.origin_y, 0.0],
            occupied_thresh: DEFAULT_OCCUPIED_THRESH,

            // Dhruva extensions
            map_id: map_id.to_string(),
            name: name.to_string(),
            created_at: now_us,
            updated_at: now_us,
            area_m2: map_data.explored_area_m2,
            room_count: 0, // TODO: Room segmentation
            is_complete: true,
            width: map_data.width as usize,
            height: map_data.height as usize,
        };

        // Save YAML metadata
        self.save_map_info(&info)?;

        // Save PGM image
        self.save_map_pgm(map_id, map_data)?;

        // Update cache
        self.map_cache.insert(map_id.to_string(), info);

        log::info!("Saved map: {} ({})", name, map_id);
        Ok(())
    }

    /// Save map cell data as PGM image.
    fn save_map_pgm(&self, map_id: &str, map_data: &CurrentMapData) -> Result<(), String> {
        use std::io::Write;

        let path = self.pgm_path(map_id);
        let width = map_data.width as usize;
        let height = map_data.height as usize;

        // Build pixel data with Y-axis flip
        let mut pixels = Vec::with_capacity(width * height);
        for cy in (0..height).rev() {
            for cx in 0..width {
                let idx = cy * width + cx;
                let cell_value = map_data.cells[idx];

                // Convert cell value to PGM pixel
                // CurrentMapData uses: 0=free, 100=occupied, 255=unknown
                // PGM uses: 255=free, 0=occupied, 205=unknown
                let pixel_value = match cell_value {
                    0..=49 => 255u8, // Free -> white
                    50..=150 => 0u8, // Occupied -> black
                    _ => 205u8,      // Unknown -> gray
                };
                pixels.push(pixel_value);
            }
        }

        // Write binary PGM (P5) format
        let mut file = std::fs::File::create(&path)
            .map_err(|e| format!("Failed to create PGM file: {}", e))?;
        writeln!(file, "P5").map_err(|e| format!("Failed to write PGM header: {}", e))?;
        writeln!(file, "{} {}", width, height)
            .map_err(|e| format!("Failed to write PGM dimensions: {}", e))?;
        writeln!(file, "255").map_err(|e| format!("Failed to write PGM maxval: {}", e))?;
        file.write_all(&pixels)
            .map_err(|e| format!("Failed to write PGM data: {}", e))
    }

    /// Load a map from disk.
    ///
    /// Loads the map from PGM image using metadata from YAML.
    pub fn load_map(&self, map_id: &str) -> Result<OccupancyGrid, String> {
        let info = self
            .map_cache
            .get(map_id)
            .ok_or_else(|| format!("Map not found: {}", map_id))?;

        // Load PGM file
        let pgm_path = self.pgm_path(map_id);
        let grid =
            OccupancyGrid::load_pgm(&pgm_path, info.resolution, info.origin[0], info.origin[1])
                .map_err(|e| format!("Failed to load PGM: {}", e))?;

        log::info!(
            "Loaded map: {} ({}x{} @ {}m)",
            info.name,
            info.width,
            info.height,
            info.resolution
        );
        Ok(grid)
    }

    /// Rename a map.
    pub fn rename_map(&mut self, map_id: &str, new_name: &str) -> Result<(), String> {
        // Update cache entry
        {
            let info = self
                .map_cache
                .get_mut(map_id)
                .ok_or_else(|| format!("Map not found: {}", map_id))?;

            info.name = new_name.to_string();
            info.updated_at = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_micros() as u64)
                .unwrap_or(0);
        }

        // Save updated metadata (borrow is released now)
        if let Some(info) = self.map_cache.get(map_id) {
            self.save_map_info(info)?;
        }

        log::info!("Renamed map {} to: {}", map_id, new_name);
        Ok(())
    }

    /// Delete a map.
    pub fn delete_map(&mut self, map_id: &str) -> Result<(), String> {
        if !self.map_cache.contains_key(map_id) {
            return Err(format!("Map not found: {}", map_id));
        }

        // Delete files
        let pgm_path = self.pgm_path(map_id);
        let yaml_path = self.yaml_path(map_id);

        if pgm_path.exists() {
            fs::remove_file(&pgm_path).map_err(|e| format!("Failed to delete PGM file: {}", e))?;
        }
        if yaml_path.exists() {
            fs::remove_file(&yaml_path)
                .map_err(|e| format!("Failed to delete YAML file: {}", e))?;
        }

        // Remove from cache
        self.map_cache.remove(map_id);

        log::info!("Deleted map: {}", map_id);
        Ok(())
    }

    /// Get map info by ID.
    pub fn get_map_info(&self, map_id: &str) -> Option<&MapInfo> {
        self.map_cache.get(map_id)
    }

    /// Get all maps as MapList for streaming.
    pub fn as_map_list(&self) -> MapList {
        let maps: Vec<MapSummary> = self
            .map_cache
            .values()
            .map(|info| MapSummary {
                map_id: info.map_id.clone(),
                name: info.name.clone(),
                created_at_us: info.created_at,
                updated_at_us: info.updated_at,
                area_m2: info.area_m2,
                room_count: info.room_count,
                is_complete: info.is_complete,
            })
            .collect();

        MapList {
            maps,
            active_map_id: None,
        }
    }

    /// Get number of maps.
    pub fn len(&self) -> usize {
        self.map_cache.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.map_cache.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::env::temp_dir;

    fn create_test_map_data() -> CurrentMapData {
        CurrentMapData {
            map_id: "test_map".to_string(),
            name: "Test Map".to_string(),
            resolution: 0.05,
            width: 100,
            height: 100,
            origin_x: -2.5,
            origin_y: -2.5,
            cells: vec![255; 10000], // All unknown
            explored_area_m2: 25.0,
        }
    }

    #[test]
    fn test_map_manager_creation() {
        let dir = temp_dir().join("dhruva_test_maps");
        let _ = fs::remove_dir_all(&dir); // Clean up from previous runs

        let manager = MapManager::new(&dir).unwrap();
        assert!(manager.is_empty());

        // Cleanup
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_save_and_load_map() {
        let dir = temp_dir().join("dhruva_test_maps_save");
        let _ = fs::remove_dir_all(&dir);

        let mut manager = MapManager::new(&dir).unwrap();

        // Save map
        let map_data = create_test_map_data();
        manager
            .save_current_map("test_001", "Kitchen", &map_data)
            .unwrap();

        assert_eq!(manager.len(), 1);

        // Load map
        let _loaded = manager.load_map("test_001").unwrap();

        // Verify metadata
        let info = manager.get_map_info("test_001").unwrap();
        assert_eq!(info.name, "Kitchen");

        // Cleanup
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_rename_map() {
        let dir = temp_dir().join("dhruva_test_maps_rename");
        let _ = fs::remove_dir_all(&dir);

        let mut manager = MapManager::new(&dir).unwrap();
        let map_data = create_test_map_data();
        manager
            .save_current_map("test_001", "Kitchen", &map_data)
            .unwrap();

        // Rename
        manager.rename_map("test_001", "Living Room").unwrap();

        let info = manager.get_map_info("test_001").unwrap();
        assert_eq!(info.name, "Living Room");

        // Cleanup
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_delete_map() {
        let dir = temp_dir().join("dhruva_test_maps_delete");
        let _ = fs::remove_dir_all(&dir);

        let mut manager = MapManager::new(&dir).unwrap();
        let map_data = create_test_map_data();
        manager
            .save_current_map("test_001", "Kitchen", &map_data)
            .unwrap();

        assert_eq!(manager.len(), 1);

        // Delete
        manager.delete_map("test_001").unwrap();

        assert_eq!(manager.len(), 0);
        assert!(manager.get_map_info("test_001").is_none());

        // Cleanup
        let _ = fs::remove_dir_all(&dir);
    }
}
