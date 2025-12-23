//! Exploration region detection and clustering.
//!
//! Groups nearby frontiers into exploration regions using hierarchical clustering.
//! This enables region-based exploration strategies that avoid ping-pong behavior.

use std::collections::HashMap;

use crate::Frontier;
use crate::core::{Bounds, Point2D};

/// Configuration for region detection.
#[derive(Clone, Debug)]
pub struct RegionConfig {
    /// Maximum distance for clustering frontiers into regions (meters).
    /// Default: 1.5
    pub cluster_distance: f32,

    /// Minimum number of frontiers to form a valid region.
    /// Default: 1
    pub min_region_size: usize,
}

impl Default for RegionConfig {
    fn default() -> Self {
        Self {
            cluster_distance: 1.5,
            min_region_size: 1,
        }
    }
}

impl RegionConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for cluster distance.
    pub fn with_cluster_distance(mut self, distance: f32) -> Self {
        self.cluster_distance = distance;
        self
    }

    /// Builder-style setter for minimum region size.
    pub fn with_min_region_size(mut self, size: usize) -> Self {
        self.min_region_size = size;
        self
    }
}

/// A clustered group of frontiers representing an unexplored area.
#[derive(Clone, Debug)]
pub struct ExplorationRegion {
    /// Unique region identifier.
    pub id: u32,

    /// Centroid of the region (mean of all frontier points).
    pub centroid: Point2D,

    /// Individual frontiers in this region.
    pub frontiers: Vec<Frontier>,

    /// Bounding box of the region.
    pub bounds: Bounds,

    /// Estimated unexplored area (heuristic based on spread).
    pub estimated_area: f32,

    /// Priority score (higher = explore first).
    pub priority: f32,

    /// Number of times we've attempted to visit this region.
    pub visit_count: u32,
}

impl ExplorationRegion {
    /// Create a new region from a cluster of frontiers.
    pub fn from_frontiers(id: u32, frontiers: Vec<Frontier>) -> Self {
        let centroid = compute_centroid(&frontiers);
        let bounds = compute_bounds(&frontiers);
        let estimated_area = estimate_area(&bounds);

        Self {
            id,
            centroid,
            frontiers,
            bounds,
            estimated_area,
            priority: 0.0,
            visit_count: 0,
        }
    }

    /// Get the number of frontiers in this region.
    pub fn frontier_count(&self) -> usize {
        self.frontiers.len()
    }

    /// Check if the region has no frontiers left.
    pub fn is_empty(&self) -> bool {
        self.frontiers.is_empty()
    }

    /// Remove frontiers that are no longer valid (not in the current frontier list).
    pub fn retain_valid_frontiers(&mut self, valid_frontiers: &[Frontier]) {
        self.frontiers.retain(|f| {
            valid_frontiers
                .iter()
                .any(|vf| f.viewpoint.distance(vf.viewpoint) < 0.1)
        });

        // Update centroid and bounds if we still have frontiers
        if !self.frontiers.is_empty() {
            self.centroid = compute_centroid(&self.frontiers);
            self.bounds = compute_bounds(&self.frontiers);
            self.estimated_area = estimate_area(&self.bounds);
        }
    }
}

/// Detect exploration regions from frontiers using hierarchical clustering.
pub struct RegionDetector {
    /// Reusable union-find parent array.
    parent: Vec<usize>,

    /// Reusable union-find rank array.
    rank: Vec<usize>,

    /// Next region ID to assign.
    next_id: u32,

    /// Spatial hash grid for efficient neighbor lookup.
    spatial_grid: HashMap<(i32, i32), Vec<usize>>,
}

impl Default for RegionDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl RegionDetector {
    /// Create a new region detector.
    pub fn new() -> Self {
        Self {
            parent: Vec::with_capacity(100),
            rank: Vec::with_capacity(100),
            next_id: 0,
            spatial_grid: HashMap::with_capacity(100),
        }
    }

    /// Detect regions from a list of frontiers.
    pub fn detect(
        &mut self,
        frontiers: &[Frontier],
        config: &RegionConfig,
    ) -> Vec<ExplorationRegion> {
        if frontiers.is_empty() {
            return Vec::new();
        }

        // Cluster frontiers using union-find
        let clusters = self.cluster_frontiers(frontiers, config.cluster_distance);

        // Convert clusters to regions
        let mut regions = Vec::new();
        for cluster in clusters {
            if cluster.len() >= config.min_region_size {
                let region_frontiers: Vec<Frontier> =
                    cluster.iter().map(|&idx| frontiers[idx].clone()).collect();

                let region = ExplorationRegion::from_frontiers(self.next_id, region_frontiers);
                self.next_id += 1;
                regions.push(region);
            }
        }

        regions
    }

    /// Cluster frontiers using union-find with spatial hashing.
    fn cluster_frontiers(&mut self, frontiers: &[Frontier], max_distance: f32) -> Vec<Vec<usize>> {
        let n = frontiers.len();

        // Initialize union-find
        self.parent.clear();
        self.parent.resize(n, 0);
        self.rank.clear();
        self.rank.resize(n, 0);
        for i in 0..n {
            self.parent[i] = i;
        }

        // Clear spatial grid completely to avoid stale indices
        self.spatial_grid.clear();

        // Build spatial hash grid (using viewpoint for clustering)
        let cell_size = max_distance;
        for (i, frontier) in frontiers.iter().enumerate() {
            let cell = (
                (frontier.viewpoint.x / cell_size).floor() as i32,
                (frontier.viewpoint.y / cell_size).floor() as i32,
            );

            self.spatial_grid.entry(cell).or_default().push(i);
        }

        // Collect pairs to union (avoids borrow conflict)
        let mut pairs_to_union: Vec<(usize, usize)> = Vec::new();

        for (i, frontier) in frontiers.iter().enumerate() {
            let cx = (frontier.viewpoint.x / cell_size).floor() as i32;
            let cy = (frontier.viewpoint.y / cell_size).floor() as i32;

            // Check 3x3 neighborhood
            for dx in -1..=1 {
                for dy in -1..=1 {
                    let neighbor_cell = (cx + dx, cy + dy);
                    if let Some(indices) = self.spatial_grid.get(&neighbor_cell) {
                        for &j in indices {
                            if i < j {
                                let dist = frontier.viewpoint.distance(frontiers[j].viewpoint);
                                if dist <= max_distance {
                                    pairs_to_union.push((i, j));
                                }
                            }
                        }
                    }
                }
            }
        }

        // Perform unions
        for (i, j) in pairs_to_union {
            self.union(i, j);
        }

        // Extract clusters
        let mut cluster_map: HashMap<usize, Vec<usize>> = HashMap::new();
        for i in 0..n {
            let root = self.find(i);
            cluster_map.entry(root).or_default().push(i);
        }

        cluster_map.into_values().collect()
    }

    /// Union-find: find with path compression.
    fn find(&mut self, x: usize) -> usize {
        if self.parent[x] != x {
            self.parent[x] = self.find(self.parent[x]);
        }
        self.parent[x]
    }

    /// Union-find: union by rank.
    fn union(&mut self, x: usize, y: usize) {
        let root_x = self.find(x);
        let root_y = self.find(y);

        if root_x != root_y {
            if self.rank[root_x] < self.rank[root_y] {
                self.parent[root_x] = root_y;
            } else if self.rank[root_x] > self.rank[root_y] {
                self.parent[root_y] = root_x;
            } else {
                self.parent[root_y] = root_x;
                self.rank[root_x] += 1;
            }
        }
    }

    /// Reset the region ID counter.
    pub fn reset_ids(&mut self) {
        self.next_id = 0;
    }
}

/// Compute the centroid of a cluster of frontiers (using viewpoints).
fn compute_centroid(frontiers: &[Frontier]) -> Point2D {
    if frontiers.is_empty() {
        return Point2D::new(0.0, 0.0);
    }

    let sum_x: f32 = frontiers.iter().map(|f| f.viewpoint.x).sum();
    let sum_y: f32 = frontiers.iter().map(|f| f.viewpoint.y).sum();
    let n = frontiers.len() as f32;

    Point2D::new(sum_x / n, sum_y / n)
}

/// Compute the bounding box of a cluster of frontiers (using viewpoints).
fn compute_bounds(frontiers: &[Frontier]) -> Bounds {
    if frontiers.is_empty() {
        return Bounds::empty();
    }

    let mut min_x = f32::INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut max_y = f32::NEG_INFINITY;

    for frontier in frontiers {
        min_x = min_x.min(frontier.viewpoint.x);
        min_y = min_y.min(frontier.viewpoint.y);
        max_x = max_x.max(frontier.viewpoint.x);
        max_y = max_y.max(frontier.viewpoint.y);
    }

    Bounds::new(Point2D::new(min_x, min_y), Point2D::new(max_x, max_y))
}

/// Estimate the unexplored area of a region based on its bounding box.
fn estimate_area(bounds: &Bounds) -> f32 {
    let width = bounds.max.x - bounds.min.x;
    let height = bounds.max.y - bounds.min.y;

    // Add a minimum area to avoid zero for single-point regions
    (width * height).max(0.1)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_frontier(x: f32, y: f32) -> Frontier {
        Frontier {
            viewpoint: Point2D::new(x, y),
            look_direction: Point2D::new(1.0, 0.0),
            endpoint: Point2D::new(x - 0.4, y),
            line_idx: 0,
            estimated_area: 4.0,
        }
    }

    #[test]
    fn test_empty_frontiers() {
        let mut detector = RegionDetector::new();
        let config = RegionConfig::default();
        let regions = detector.detect(&[], &config);
        assert!(regions.is_empty());
    }

    #[test]
    fn test_single_frontier() {
        let mut detector = RegionDetector::new();
        let config = RegionConfig::default();
        let frontiers = vec![make_frontier(1.0, 1.0)];
        let regions = detector.detect(&frontiers, &config);

        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].frontiers.len(), 1);
    }

    #[test]
    fn test_two_clusters() {
        let mut detector = RegionDetector::new();
        let config = RegionConfig::default().with_cluster_distance(1.0);

        // Two clusters: (0,0), (0.5, 0) and (10, 10), (10.5, 10)
        let frontiers = vec![
            make_frontier(0.0, 0.0),
            make_frontier(0.5, 0.0),
            make_frontier(10.0, 10.0),
            make_frontier(10.5, 10.0),
        ];

        let regions = detector.detect(&frontiers, &config);

        assert_eq!(regions.len(), 2);
        assert_eq!(regions[0].frontiers.len(), 2);
        assert_eq!(regions[1].frontiers.len(), 2);
    }

    #[test]
    fn test_centroid_calculation() {
        let frontiers = vec![
            make_frontier(0.0, 0.0),
            make_frontier(2.0, 0.0),
            make_frontier(1.0, 2.0),
        ];

        let centroid = compute_centroid(&frontiers);
        assert!((centroid.x - 1.0).abs() < 0.001);
        assert!((centroid.y - 0.667).abs() < 0.01);
    }

    #[test]
    fn test_bounds_calculation() {
        let frontiers = vec![
            make_frontier(1.0, 2.0),
            make_frontier(3.0, 4.0),
            make_frontier(2.0, 1.0),
        ];

        let bounds = compute_bounds(&frontiers);
        assert_eq!(bounds.min.x, 1.0);
        assert_eq!(bounds.min.y, 1.0);
        assert_eq!(bounds.max.x, 3.0);
        assert_eq!(bounds.max.y, 4.0);
    }

    #[test]
    fn test_min_region_size() {
        let mut detector = RegionDetector::new();
        let config = RegionConfig::default()
            .with_cluster_distance(0.5)
            .with_min_region_size(2);

        // One isolated frontier and two nearby ones
        let frontiers = vec![
            make_frontier(0.0, 0.0), // Isolated
            make_frontier(10.0, 10.0),
            make_frontier(10.2, 10.0), // Together
        ];

        let regions = detector.detect(&frontiers, &config);

        // Only the cluster with 2 frontiers should be included
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].frontiers.len(), 2);
    }
}
