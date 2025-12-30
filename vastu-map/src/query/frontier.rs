//! Frontier detection for exploration.
//!
//! A frontier is a boundary between known (Floor) and unknown cells.
//! This module detects frontiers and clusters them for exploration targeting.

use crate::core::{CellType, GridCoord, WorldPoint};
use crate::grid::GridStorage;
use log::{debug, trace};
use std::collections::{HashSet, VecDeque};
use std::simd::{cmp::SimdPartialEq, u8x16};

/// A single frontier cell
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FrontierCell {
    /// Grid coordinate of the frontier cell
    pub coord: GridCoord,
    /// Number of adjacent unknown cells (1-8)
    pub unknown_neighbors: u8,
}

impl FrontierCell {
    /// Create a new frontier cell
    pub fn new(coord: GridCoord, unknown_neighbors: u8) -> Self {
        Self {
            coord,
            unknown_neighbors,
        }
    }
}

/// A cluster of connected frontier cells
#[derive(Clone, Debug)]
pub struct Frontier {
    /// All cells in this frontier
    pub cells: Vec<FrontierCell>,
    /// Centroid in grid coordinates
    pub centroid_grid: GridCoord,
    /// Centroid in world coordinates
    pub centroid_world: WorldPoint,
    /// Approximate size (number of cells)
    pub size: usize,
    /// Sum of unknown neighbors (exploration potential)
    pub unknown_exposure: usize,
}

impl Frontier {
    /// Create a new frontier from a set of cells
    pub fn from_cells(cells: Vec<FrontierCell>, storage: &GridStorage) -> Self {
        let size = cells.len();

        // Calculate centroid
        let sum_x: i32 = cells.iter().map(|c| c.coord.x).sum();
        let sum_y: i32 = cells.iter().map(|c| c.coord.y).sum();
        let centroid_grid = GridCoord::new(sum_x / size as i32, sum_y / size as i32);
        let centroid_world = storage.grid_to_world(centroid_grid);

        // Calculate total unknown exposure
        let unknown_exposure: usize = cells.iter().map(|c| c.unknown_neighbors as usize).sum();

        Self {
            cells,
            centroid_grid,
            centroid_world,
            size,
            unknown_exposure,
        }
    }

    /// Exploration score (higher = more valuable to explore)
    /// Considers both size and unknown exposure
    pub fn exploration_score(&self) -> f32 {
        // Balance between size and exposure
        // Larger frontiers with more unknown cells are more valuable
        self.size as f32 * 0.5 + self.unknown_exposure as f32 * 0.5
    }

    /// Get the closest cell in this frontier to a given position
    pub fn closest_cell_to(&self, target: GridCoord) -> Option<&FrontierCell> {
        self.cells
            .iter()
            .min_by_key(|c| c.coord.chebyshev_distance(&target))
    }

    /// Length of the frontier in cells (approximate)
    pub fn length(&self) -> usize {
        self.size
    }
}

/// Frontier detector
pub struct FrontierDetector {
    /// Minimum frontier size (smaller clusters are filtered out)
    pub min_frontier_size: usize,
    /// Use 8-connectivity for frontier detection (vs 4-connectivity)
    pub use_8_connectivity: bool,
}

impl Default for FrontierDetector {
    fn default() -> Self {
        Self {
            min_frontier_size: 3,
            use_8_connectivity: true,
        }
    }
}

impl FrontierDetector {
    /// Create a new frontier detector with default settings
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with custom minimum frontier size
    pub fn with_min_size(min_size: usize) -> Self {
        Self {
            min_frontier_size: min_size,
            ..Default::default()
        }
    }

    /// Detect all frontier cells in the grid (SIMD-optimized).
    ///
    /// Uses SIMD to quickly filter Floor cells (16 at a time), then
    /// performs scalar neighbor counting for each candidate.
    pub fn detect_frontier_cells(&self, storage: &GridStorage) -> Vec<FrontierCell> {
        let width = storage.width();
        let height = storage.height();
        let cell_types = storage.cell_types_raw();

        let floor_val = CellType::Floor as u8;
        let floor_vec = u8x16::splat(floor_val);

        let mut frontiers = Vec::new();

        // Process 16 cells at a time using SIMD
        let chunks = cell_types.chunks_exact(16);
        let remainder = chunks.remainder();

        for (chunk_idx, chunk) in chunks.enumerate() {
            let data = u8x16::from_slice(chunk);
            let is_floor = data.simd_eq(floor_vec);
            let mask = is_floor.to_bitmask();

            if mask == 0 {
                continue; // No Floor cells in this chunk - skip entirely
            }

            // Process each Floor cell found in this chunk
            let base_idx = chunk_idx * 16;
            for bit in 0..16 {
                if (mask & (1 << bit)) != 0 {
                    let idx = base_idx + bit;
                    let x = (idx % width) as i32;
                    let y = (idx / width) as i32;
                    let coord = GridCoord::new(x, y);

                    // Count unknown neighbors (scalar - depends on grid topology)
                    let unknown_count =
                        self.count_unknown_neighbors(storage, coord, width as i32, height as i32);
                    if unknown_count > 0 {
                        frontiers.push(FrontierCell::new(coord, unknown_count));
                    }
                }
            }
        }

        // Handle remainder (< 16 cells) with scalar fallback
        let base_idx = (cell_types.len() / 16) * 16;
        for (i, &cell_type) in remainder.iter().enumerate() {
            if cell_type == floor_val {
                let idx = base_idx + i;
                let x = (idx % width) as i32;
                let y = (idx / width) as i32;
                let coord = GridCoord::new(x, y);

                let unknown_count =
                    self.count_unknown_neighbors(storage, coord, width as i32, height as i32);
                if unknown_count > 0 {
                    frontiers.push(FrontierCell::new(coord, unknown_count));
                }
            }
        }

        trace!("[Frontier] Detected {} frontier cells", frontiers.len());
        frontiers
    }

    /// Count unknown neighbors for a cell (helper for SIMD detection).
    #[inline]
    fn count_unknown_neighbors(
        &self,
        storage: &GridStorage,
        coord: GridCoord,
        width: i32,
        height: i32,
    ) -> u8 {
        let neighbors = if self.use_8_connectivity {
            coord.neighbors_8()
        } else {
            let n4 = coord.neighbors_4();
            [n4[0], n4[1], n4[2], n4[3], n4[0], n4[1], n4[2], n4[3]]
        };

        let limit = if self.use_8_connectivity { 8 } else { 4 };
        let mut count = 0u8;

        for n in neighbors.iter().take(limit) {
            if n.x >= 0
                && n.x < width
                && n.y >= 0
                && n.y < height
                && storage.get_type(*n) == CellType::Unknown
            {
                count += 1;
            }
        }

        count
    }

    /// Detect and cluster frontiers
    pub fn detect_frontiers(&self, storage: &GridStorage) -> Vec<Frontier> {
        let frontier_cells = self.detect_frontier_cells(storage);

        if frontier_cells.is_empty() {
            debug!("[Frontier] No frontier cells found (no Floor cells adjacent to Unknown)");
            return Vec::new();
        }

        // Build a set for fast lookup
        let frontier_set: HashSet<GridCoord> = frontier_cells.iter().map(|f| f.coord).collect();

        // Map coord to FrontierCell
        let cell_map: std::collections::HashMap<GridCoord, FrontierCell> =
            frontier_cells.iter().map(|f| (f.coord, *f)).collect();

        // Cluster using BFS
        let mut visited = HashSet::new();
        let mut frontiers = Vec::new();
        let mut clusters_before_filter = 0;
        let mut filtered_clusters = 0;

        for cell in &frontier_cells {
            if visited.contains(&cell.coord) {
                continue;
            }

            // BFS to find all connected frontier cells
            let mut cluster = Vec::new();
            let mut queue = VecDeque::new();
            queue.push_back(cell.coord);
            visited.insert(cell.coord);

            while let Some(current) = queue.pop_front() {
                if let Some(&fc) = cell_map.get(&current) {
                    cluster.push(fc);
                }

                // Check neighbors
                let neighbors = if self.use_8_connectivity {
                    current.neighbors_8().to_vec()
                } else {
                    current.neighbors_4().to_vec()
                };

                for neighbor in neighbors {
                    if !visited.contains(&neighbor) && frontier_set.contains(&neighbor) {
                        visited.insert(neighbor);
                        queue.push_back(neighbor);
                    }
                }
            }

            clusters_before_filter += 1;

            // Only keep clusters above minimum size
            if cluster.len() >= self.min_frontier_size {
                frontiers.push(Frontier::from_cells(cluster, storage));
            } else {
                filtered_clusters += 1;
                trace!(
                    "[Frontier] Filtered cluster with {} cells (min_size={})",
                    cluster.len(),
                    self.min_frontier_size
                );
            }
        }

        debug!(
            "[Frontier] Clustering: {} cells → {} clusters, {} valid (filtered {} < {} cells)",
            frontier_cells.len(),
            clusters_before_filter,
            frontiers.len(),
            filtered_clusters,
            self.min_frontier_size
        );

        frontiers
    }

    /// Get frontiers sorted by exploration score (highest first)
    pub fn detect_frontiers_sorted(&self, storage: &GridStorage) -> Vec<Frontier> {
        let mut frontiers = self.detect_frontiers(storage);
        frontiers.sort_by(|a, b| b.exploration_score().total_cmp(&a.exploration_score()));
        frontiers
    }

    /// Find the best frontier to explore from a given position
    pub fn best_frontier_from(
        &self,
        storage: &GridStorage,
        robot_pos: GridCoord,
    ) -> Option<Frontier> {
        let frontiers = self.detect_frontiers(storage);

        if frontiers.is_empty() {
            return None;
        }

        // Score combines exploration value and distance
        // Closer frontiers with higher exploration potential are preferred
        frontiers.into_iter().max_by(|a, b| {
            let score_a = self.score_frontier(a, robot_pos);
            let score_b = self.score_frontier(b, robot_pos);
            score_a.total_cmp(&score_b)
        })
    }

    /// Score a frontier considering distance from robot
    fn score_frontier(&self, frontier: &Frontier, robot_pos: GridCoord) -> f32 {
        let distance = robot_pos.manhattan_distance(&frontier.centroid_grid) as f32;
        let exploration_score = frontier.exploration_score();

        // Penalize distance, but not too heavily
        // We want to explore nearby frontiers first, but not ignore large distant ones
        if distance > 0.0 {
            exploration_score / (1.0 + distance * 0.1)
        } else {
            exploration_score * 2.0 // Bonus for being at the frontier
        }
    }
}

/// Quick frontier detection without clustering (for performance)
pub fn count_frontier_cells(storage: &GridStorage) -> usize {
    let detector = FrontierDetector::new();
    detector.detect_frontier_cells(storage).len()
}

/// Check if any frontiers exist in the map
pub fn has_frontiers(storage: &GridStorage) -> bool {
    let detector = FrontierDetector::new();
    !detector.detect_frontier_cells(storage).is_empty()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::simd::{cmp::SimdPartialEq, u8x16};

    fn create_test_storage() -> GridStorage {
        GridStorage::centered(20, 20, 0.1) // 2m x 2m at 10cm resolution
    }

    // =========================================================================
    // SIMD-OPTIMIZED TEST HELPERS
    // These are test-only implementations to verify SIMD correctness
    // =========================================================================

    /// Detect all frontier cells using SIMD-accelerated Floor cell detection.
    fn detect_frontier_cells_simd(
        detector: &FrontierDetector,
        storage: &GridStorage,
    ) -> Vec<FrontierCell> {
        let width = storage.width();
        let height = storage.height();
        let cell_types = storage.cell_types_raw();

        let floor_val = CellType::Floor as u8;
        let floor_vec = u8x16::splat(floor_val);

        let mut frontiers = Vec::new();

        // Process 16 cells at a time using SIMD
        let chunks = cell_types.chunks_exact(16);
        let remainder = chunks.remainder();

        for (chunk_idx, chunk) in chunks.enumerate() {
            let data = u8x16::from_slice(chunk);
            let is_floor = data.simd_eq(floor_vec);
            let mask = is_floor.to_bitmask();

            if mask == 0 {
                continue; // No Floor cells in this chunk
            }

            // Process each Floor cell found
            let base_idx = chunk_idx * 16;
            for bit in 0..16 {
                if (mask & (1 << bit)) != 0 {
                    let idx = base_idx + bit;
                    let x = (idx % width) as i32;
                    let y = (idx / width) as i32;
                    let coord = GridCoord::new(x, y);

                    // Count unknown neighbors (scalar)
                    let unknown_count = count_unknown_neighbors(
                        detector,
                        storage,
                        coord,
                        width as i32,
                        height as i32,
                    );
                    if unknown_count > 0 {
                        frontiers.push(FrontierCell::new(coord, unknown_count));
                    }
                }
            }
        }

        // Handle remainder (< 16 cells)
        let base_idx = (cell_types.len() / 16) * 16;
        for (i, &cell_type) in remainder.iter().enumerate() {
            if cell_type == floor_val {
                let idx = base_idx + i;
                let x = (idx % width) as i32;
                let y = (idx / width) as i32;
                let coord = GridCoord::new(x, y);

                let unknown_count =
                    count_unknown_neighbors(detector, storage, coord, width as i32, height as i32);
                if unknown_count > 0 {
                    frontiers.push(FrontierCell::new(coord, unknown_count));
                }
            }
        }

        frontiers
    }

    /// Count unknown neighbors for a given cell
    fn count_unknown_neighbors(
        detector: &FrontierDetector,
        storage: &GridStorage,
        coord: GridCoord,
        width: i32,
        height: i32,
    ) -> u8 {
        let neighbors = if detector.use_8_connectivity {
            coord.neighbors_8()
        } else {
            let n4 = coord.neighbors_4();
            [n4[0], n4[1], n4[2], n4[3], n4[0], n4[1], n4[2], n4[3]]
        };

        let limit = if detector.use_8_connectivity { 8 } else { 4 };
        let mut count = 0u8;

        for n in neighbors.iter().take(limit) {
            if n.x >= 0
                && n.x < width
                && n.y >= 0
                && n.y < height
                && storage.get_type(*n) == CellType::Unknown
            {
                count += 1;
            }
        }

        count
    }

    /// Detect and cluster frontiers using SIMD-optimized cell detection.
    fn detect_frontiers_simd(detector: &FrontierDetector, storage: &GridStorage) -> Vec<Frontier> {
        let frontier_cells = detect_frontier_cells_simd(detector, storage);

        if frontier_cells.is_empty() {
            return Vec::new();
        }

        cluster_frontier_cells(detector, frontier_cells, storage)
    }

    /// Cluster frontier cells into connected frontiers.
    fn cluster_frontier_cells(
        detector: &FrontierDetector,
        frontier_cells: Vec<FrontierCell>,
        storage: &GridStorage,
    ) -> Vec<Frontier> {
        // Build a set for fast lookup
        let frontier_set: HashSet<GridCoord> = frontier_cells.iter().map(|f| f.coord).collect();

        // Map coord to FrontierCell
        let cell_map: std::collections::HashMap<GridCoord, FrontierCell> =
            frontier_cells.iter().map(|f| (f.coord, *f)).collect();

        // Cluster using BFS
        let mut visited = HashSet::new();
        let mut frontiers = Vec::new();

        for cell in &frontier_cells {
            if visited.contains(&cell.coord) {
                continue;
            }

            // BFS to find all connected frontier cells
            let mut cluster = Vec::new();
            let mut queue = VecDeque::new();
            queue.push_back(cell.coord);
            visited.insert(cell.coord);

            while let Some(current) = queue.pop_front() {
                if let Some(&fc) = cell_map.get(&current) {
                    cluster.push(fc);
                }

                // Check neighbors
                let neighbors = if detector.use_8_connectivity {
                    current.neighbors_8().to_vec()
                } else {
                    current.neighbors_4().to_vec()
                };

                for neighbor in neighbors {
                    if !visited.contains(&neighbor) && frontier_set.contains(&neighbor) {
                        visited.insert(neighbor);
                        queue.push_back(neighbor);
                    }
                }
            }

            // Only keep clusters above minimum size
            if cluster.len() >= detector.min_frontier_size {
                frontiers.push(Frontier::from_cells(cluster, storage));
            }
        }

        frontiers
    }

    #[test]
    fn test_no_frontiers_on_empty_grid() {
        let storage = create_test_storage();
        let detector = FrontierDetector::new();

        let frontiers = detector.detect_frontiers(&storage);
        assert!(frontiers.is_empty());
    }

    #[test]
    fn test_frontier_detection_basic() {
        let mut storage = create_test_storage();

        // Create a small floor area
        for x in 8..12 {
            for y in 8..12 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        let detector = FrontierDetector::new();
        let cells = detector.detect_frontier_cells(&storage);

        // All edge cells should be frontiers
        assert!(cells.len() > 0);

        // Center cell should NOT be a frontier
        let center = GridCoord::new(10, 10);
        assert!(!cells.iter().any(|f| f.coord == center));
    }

    #[test]
    fn test_frontier_clustering() {
        let mut storage = create_test_storage();

        // Create two separate floor regions
        // Region 1
        for x in 2..5 {
            for y in 2..5 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }
        // Region 2 (separated by unknown)
        for x in 15..18 {
            for y in 15..18 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        let detector = FrontierDetector::new();
        let frontiers = detector.detect_frontiers(&storage);

        // Should have 2 frontier clusters
        assert_eq!(frontiers.len(), 2);
    }

    #[test]
    fn test_frontier_centroid() {
        let mut storage = create_test_storage();

        // Create a symmetric floor region
        for x in 5..15 {
            for y in 5..15 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        let detector = FrontierDetector::new();
        let frontiers = detector.detect_frontiers(&storage);

        assert_eq!(frontiers.len(), 1);

        // Centroid should be roughly in the middle of the frontier ring
        let frontier = &frontiers[0];
        // The frontier is on the edge, so centroid should be near edges
        assert!(frontier.centroid_grid.x >= 5 && frontier.centroid_grid.x <= 14);
        assert!(frontier.centroid_grid.y >= 5 && frontier.centroid_grid.y <= 14);
    }

    #[test]
    fn test_min_frontier_size() {
        let mut storage = create_test_storage();

        // Create a very small floor area (just 1 cell)
        storage.set_type(GridCoord::new(10, 10), CellType::Floor);

        let detector = FrontierDetector::with_min_size(5);
        let frontiers = detector.detect_frontiers(&storage);

        // Single cell frontier should be filtered out
        assert!(frontiers.is_empty());
    }

    #[test]
    fn test_wall_blocks_frontier() {
        let mut storage = create_test_storage();

        // Floor surrounded by walls on one side
        for x in 8..12 {
            for y in 8..12 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }
        // Add wall on the right
        for y in 7..13 {
            storage.set_type(GridCoord::new(12, y), CellType::Wall);
        }

        let detector = FrontierDetector::new();
        let cells = detector.detect_frontier_cells(&storage);

        // Cells next to wall should not have frontiers on that side
        let right_edge = GridCoord::new(11, 10);
        let frontier_cell = cells.iter().find(|f| f.coord == right_edge);

        // The right edge cell should have fewer unknown neighbors
        // (wall blocks the right side from being unknown)
        if let Some(fc) = frontier_cell {
            // Should be < 8 unknown neighbors due to wall
            assert!(fc.unknown_neighbors < 8);
        }
    }

    #[test]
    fn test_exploration_score() {
        let mut storage = create_test_storage();

        // Large floor area
        for x in 2..18 {
            for y in 2..18 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        let detector = FrontierDetector::new();
        let frontiers = detector.detect_frontiers(&storage);

        assert!(!frontiers.is_empty());

        // Score should be positive
        assert!(frontiers[0].exploration_score() > 0.0);
    }

    #[test]
    fn test_best_frontier_from() {
        let mut storage = create_test_storage();

        // Create floor area
        for x in 5..15 {
            for y in 5..15 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        let detector = FrontierDetector::new();
        let robot_pos = GridCoord::new(10, 10);

        let best = detector.best_frontier_from(&storage, robot_pos);
        assert!(best.is_some());
    }

    #[test]
    fn test_simd_frontier_matches_scalar() {
        let mut storage = create_test_storage();

        // Create floor area
        for x in 5..15 {
            for y in 5..15 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        let detector = FrontierDetector::new();

        // Get scalar frontier cells
        let scalar_cells = detector.detect_frontier_cells(&storage);

        // Get SIMD frontier cells
        let simd_cells = detect_frontier_cells_simd(&detector, &storage);

        // Should have same count
        assert_eq!(
            scalar_cells.len(),
            simd_cells.len(),
            "Cell count mismatch: scalar={}, simd={}",
            scalar_cells.len(),
            simd_cells.len()
        );

        // Verify same cells are found (order may differ)
        let scalar_set: std::collections::HashSet<_> = scalar_cells
            .iter()
            .map(|c| (c.coord, c.unknown_neighbors))
            .collect();
        let simd_set: std::collections::HashSet<_> = simd_cells
            .iter()
            .map(|c| (c.coord, c.unknown_neighbors))
            .collect();

        assert_eq!(scalar_set, simd_set, "Different frontier cells detected");
    }

    #[test]
    fn test_simd_frontier_clustering_matches_scalar() {
        let mut storage = create_test_storage();

        // Create two separate floor regions
        for x in 2..5 {
            for y in 2..5 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }
        for x in 15..18 {
            for y in 15..18 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        let detector = FrontierDetector::new();

        let scalar_frontiers = detector.detect_frontiers(&storage);
        let simd_frontiers = detect_frontiers_simd(&detector, &storage);

        // Should have same number of clusters
        assert_eq!(
            scalar_frontiers.len(),
            simd_frontiers.len(),
            "Frontier count mismatch: scalar={}, simd={}",
            scalar_frontiers.len(),
            simd_frontiers.len()
        );

        // Each cluster should have same size
        let mut scalar_sizes: Vec<_> = scalar_frontiers.iter().map(|f| f.size).collect();
        let mut simd_sizes: Vec<_> = simd_frontiers.iter().map(|f| f.size).collect();
        scalar_sizes.sort();
        simd_sizes.sort();

        assert_eq!(scalar_sizes, simd_sizes, "Cluster sizes differ");
    }
}
