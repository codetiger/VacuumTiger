//! Map noise and quality metrics for SLAM benchmarking.
//!
//! Measures map quality through several indicators:
//! - Wall coherence: How connected are occupied cells?
//! - Isolated cells: How many occupied cells have no neighbors?
//! - Cell statistics: Distribution of occupied/free/unknown cells

use serde::{Deserialize, Serialize};

use crate::algorithms::mapping::{CellState, OccupancyGrid};

/// Metrics for evaluating map noise and quality.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapNoiseMetrics {
    /// Total number of occupied cells.
    pub occupied_cells: u64,

    /// Total number of free cells.
    pub free_cells: u64,

    /// Total number of unknown cells.
    pub unknown_cells: u64,

    /// Percentage of map that has been explored (occupied + free) / total.
    pub coverage_percent: f64,

    /// Ratio of occupied to explored cells: occupied / (occupied + free).
    pub occupied_ratio: f64,

    /// Number of isolated occupied cells (no occupied neighbors).
    ///
    /// High values indicate scattered noise speckles.
    pub isolated_cells: u64,

    /// Percentage of occupied cells that have at least one occupied neighbor.
    ///
    /// High values indicate coherent walls, low values indicate scattered noise.
    pub wall_coherence: f64,

    /// Average number of occupied neighbors per occupied cell (0-8).
    ///
    /// Higher = thicker/more connected walls.
    pub avg_occupied_neighbors: f64,

    /// Map bounds in world coordinates (min_x, min_y, max_x, max_y).
    pub bounds: (f32, f32, f32, f32),
}

impl Default for MapNoiseMetrics {
    fn default() -> Self {
        Self {
            occupied_cells: 0,
            free_cells: 0,
            unknown_cells: 0,
            coverage_percent: 0.0,
            occupied_ratio: 0.0,
            isolated_cells: 0,
            wall_coherence: 0.0,
            avg_occupied_neighbors: 0.0,
            bounds: (0.0, 0.0, 0.0, 0.0),
        }
    }
}

impl std::fmt::Display for MapNoiseMetrics {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Map Noise Metrics:")?;
        writeln!(
            f,
            "  Cells: {} occupied, {} free, {} unknown",
            self.occupied_cells, self.free_cells, self.unknown_cells
        )?;
        writeln!(f, "  Coverage: {:.1}%", self.coverage_percent)?;
        writeln!(f, "  Occupied ratio: {:.1}%", self.occupied_ratio * 100.0)?;
        writeln!(
            f,
            "  Isolated cells: {} ({:.1}% of occupied)",
            self.isolated_cells,
            if self.occupied_cells > 0 {
                self.isolated_cells as f64 / self.occupied_cells as f64 * 100.0
            } else {
                0.0
            }
        )?;
        writeln!(f, "  Wall coherence: {:.1}%", self.wall_coherence * 100.0)?;
        writeln!(f, "  Avg neighbors: {:.2}", self.avg_occupied_neighbors)?;
        write!(
            f,
            "  Bounds: ({:.2}, {:.2}) to ({:.2}, {:.2})",
            self.bounds.0, self.bounds.1, self.bounds.2, self.bounds.3
        )
    }
}

/// Analyze an occupancy grid and compute noise metrics.
pub fn analyze_map_noise(grid: &OccupancyGrid) -> MapNoiseMetrics {
    let (width, height) = grid.dimensions();
    let (origin_x, origin_y) = grid.origin();
    let resolution = grid.resolution();

    let mut occupied_cells = 0u64;
    let mut free_cells = 0u64;
    let mut unknown_cells = 0u64;
    let mut isolated_cells = 0u64;
    let mut cells_with_neighbors = 0u64;
    let mut total_occupied_neighbors = 0u64;

    // Track bounds of non-unknown cells
    let mut min_x = f32::MAX;
    let mut min_y = f32::MAX;
    let mut max_x = f32::MIN;
    let mut max_y = f32::MIN;

    // 8-connected neighbor offsets
    const NEIGHBORS: [(i32, i32); 8] = [
        (-1, -1),
        (0, -1),
        (1, -1),
        (-1, 0),
        (1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
    ];

    for cy in 0..height {
        for cx in 0..width {
            let state = grid.get_state(cx, cy);

            match state {
                CellState::Occupied => {
                    occupied_cells += 1;

                    // Update bounds
                    let (wx, wy) = grid.cell_to_world(cx, cy);
                    min_x = min_x.min(wx);
                    min_y = min_y.min(wy);
                    max_x = max_x.max(wx);
                    max_y = max_y.max(wy);

                    // Count occupied neighbors
                    let mut neighbor_count = 0u32;
                    for (dx, dy) in NEIGHBORS {
                        let nx = cx as i32 + dx;
                        let ny = cy as i32 + dy;

                        if nx >= 0
                            && ny >= 0
                            && (nx as usize) < width
                            && (ny as usize) < height
                            && grid.get_state(nx as usize, ny as usize) == CellState::Occupied
                        {
                            neighbor_count += 1;
                        }
                    }

                    total_occupied_neighbors += neighbor_count as u64;

                    if neighbor_count == 0 {
                        isolated_cells += 1;
                    } else {
                        cells_with_neighbors += 1;
                    }
                }
                CellState::Free => {
                    free_cells += 1;

                    // Update bounds for explored area
                    let (wx, wy) = grid.cell_to_world(cx, cy);
                    min_x = min_x.min(wx);
                    min_y = min_y.min(wy);
                    max_x = max_x.max(wx);
                    max_y = max_y.max(wy);
                }
                CellState::Unknown => {
                    unknown_cells += 1;
                }
            }
        }
    }

    // Handle edge case of empty map
    if min_x == f32::MAX {
        min_x = origin_x;
        min_y = origin_y;
        max_x = origin_x + (width as f32) * resolution;
        max_y = origin_y + (height as f32) * resolution;
    }

    let total_cells = occupied_cells + free_cells + unknown_cells;
    let explored_cells = occupied_cells + free_cells;

    let coverage_percent = if total_cells > 0 {
        explored_cells as f64 / total_cells as f64 * 100.0
    } else {
        0.0
    };

    let occupied_ratio = if explored_cells > 0 {
        occupied_cells as f64 / explored_cells as f64
    } else {
        0.0
    };

    let wall_coherence = if occupied_cells > 0 {
        cells_with_neighbors as f64 / occupied_cells as f64
    } else {
        0.0
    };

    let avg_occupied_neighbors = if occupied_cells > 0 {
        total_occupied_neighbors as f64 / occupied_cells as f64
    } else {
        0.0
    };

    MapNoiseMetrics {
        occupied_cells,
        free_cells,
        unknown_cells,
        coverage_percent,
        occupied_ratio,
        isolated_cells,
        wall_coherence,
        avg_occupied_neighbors,
        bounds: (min_x, min_y, max_x, max_y),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::mapping::OccupancyGridConfig;

    fn create_test_grid() -> OccupancyGrid {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 5.0,
            initial_height: 5.0,
            ..Default::default()
        };
        OccupancyGrid::new(config)
    }

    #[test]
    fn test_empty_map() {
        let grid = create_test_grid();
        let metrics = analyze_map_noise(&grid);

        assert_eq!(metrics.occupied_cells, 0);
        assert_eq!(metrics.free_cells, 0);
        assert!(metrics.unknown_cells > 0);
        assert_eq!(metrics.coverage_percent, 0.0);
        assert_eq!(metrics.isolated_cells, 0);
    }

    #[test]
    fn test_single_occupied_cell() {
        let mut grid = create_test_grid();

        // Mark one cell as occupied
        for _ in 0..5 {
            grid.update_cell(25, 25, true);
        }

        let metrics = analyze_map_noise(&grid);

        assert_eq!(metrics.occupied_cells, 1);
        assert_eq!(metrics.isolated_cells, 1); // Single cell has no neighbors
        assert_eq!(metrics.wall_coherence, 0.0); // No cells have neighbors
        assert_eq!(metrics.avg_occupied_neighbors, 0.0);
    }

    #[test]
    fn test_horizontal_wall() {
        let mut grid = create_test_grid();

        // Create a horizontal line of 5 occupied cells
        for x in 20..25 {
            for _ in 0..5 {
                grid.update_cell(x, 25, true);
            }
        }

        let metrics = analyze_map_noise(&grid);

        assert_eq!(metrics.occupied_cells, 5);
        // End cells have 1 neighbor, middle cells have 2
        // So 2 + 2 + 2 + 2 + 2 = 10 (when counting both left and right)
        // Actually: cell 0 has 1 neighbor, cells 1-3 have 2 neighbors, cell 4 has 1 neighbor
        // Total = 1 + 2 + 2 + 2 + 1 = 8 neighbors
        assert_eq!(metrics.isolated_cells, 0); // All cells connected
        assert!(metrics.wall_coherence > 0.9); // All cells have at least 1 neighbor
    }

    #[test]
    fn test_scattered_noise() {
        let mut grid = create_test_grid();

        // Create scattered isolated cells (simulating noise)
        for _ in 0..5 {
            grid.update_cell(5, 5, true);
            grid.update_cell(15, 15, true);
            grid.update_cell(25, 25, true);
            grid.update_cell(35, 35, true);
            grid.update_cell(45, 45, true);
        }

        let metrics = analyze_map_noise(&grid);

        assert_eq!(metrics.occupied_cells, 5);
        assert_eq!(metrics.isolated_cells, 5); // All are isolated
        assert_eq!(metrics.wall_coherence, 0.0);
        assert_eq!(metrics.avg_occupied_neighbors, 0.0);
    }

    #[test]
    fn test_solid_block() {
        let mut grid = create_test_grid();

        // Create a 3x3 solid block
        for y in 20..23 {
            for x in 20..23 {
                for _ in 0..5 {
                    grid.update_cell(x, y, true);
                }
            }
        }

        let metrics = analyze_map_noise(&grid);

        assert_eq!(metrics.occupied_cells, 9);
        assert_eq!(metrics.isolated_cells, 0);
        assert!(metrics.wall_coherence > 0.99); // All cells connected
        // Corner cells have 3 neighbors, edge cells have 5, center has 8
        // 4*3 + 4*5 + 1*8 = 12 + 20 + 8 = 40, avg = 40/9 = 4.44
        assert!(metrics.avg_occupied_neighbors > 4.0);
    }

    #[test]
    fn test_coverage_calculation() {
        let mut grid = create_test_grid();

        // Mark some cells as occupied and free
        for _ in 0..5 {
            grid.update_cell(10, 10, true);
            grid.update_cell(20, 20, false);
            grid.update_cell(30, 30, false);
        }

        let metrics = analyze_map_noise(&grid);

        assert_eq!(metrics.occupied_cells, 1);
        assert_eq!(metrics.free_cells, 2);

        // Coverage = (1 + 2) / total * 100
        let total = metrics.occupied_cells + metrics.free_cells + metrics.unknown_cells;
        let expected_coverage = 3.0 / total as f64 * 100.0;
        assert!((metrics.coverage_percent - expected_coverage).abs() < 0.01);
    }

    #[test]
    fn test_display_format() {
        let metrics = MapNoiseMetrics {
            occupied_cells: 100,
            free_cells: 1000,
            unknown_cells: 500,
            coverage_percent: 68.75,
            occupied_ratio: 0.09,
            isolated_cells: 5,
            wall_coherence: 0.95,
            avg_occupied_neighbors: 2.5,
            bounds: (-1.0, -2.0, 3.0, 4.0),
        };

        let output = format!("{}", metrics);
        assert!(output.contains("100 occupied"));
        assert!(output.contains("1000 free"));
        assert!(output.contains("68.8%"));
        assert!(output.contains("95.0%"));
    }
}
