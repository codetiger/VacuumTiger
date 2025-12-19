//! Spatial indexing for efficient map queries.
//!
//! Uses an R-tree to enable fast spatial queries:
//! - Nearest line to a point
//! - Lines within a bounding box
//! - Lines within a radius

use rstar::{AABB, PointDistance, RTree, RTreeObject};

use crate::core::{Bounds, Point2D};
use crate::features::Line2D;

/// An indexed line segment for R-tree storage.
#[derive(Clone, Debug)]
pub struct IndexedLine {
    /// The line segment.
    pub line: Line2D,
    /// Index of this line in the original collection.
    pub index: usize,
}

impl IndexedLine {
    /// Create a new indexed line.
    pub fn new(line: Line2D, index: usize) -> Self {
        Self { line, index }
    }
}

impl RTreeObject for IndexedLine {
    type Envelope = AABB<[f32; 2]>;

    fn envelope(&self) -> Self::Envelope {
        let min_x = self.line.start.x.min(self.line.end.x);
        let max_x = self.line.start.x.max(self.line.end.x);
        let min_y = self.line.start.y.min(self.line.end.y);
        let max_y = self.line.start.y.max(self.line.end.y);

        AABB::from_corners([min_x, min_y], [max_x, max_y])
    }
}

impl PointDistance for IndexedLine {
    fn distance_2(&self, point: &[f32; 2]) -> f32 {
        let p = Point2D::new(point[0], point[1]);
        let dist = self.line.distance_to_point(p);
        dist * dist // Squared distance
    }

    fn contains_point(&self, _point: &[f32; 2]) -> bool {
        false // Lines don't contain points (zero area)
    }
}

/// Spatial index for line segments using R-tree.
///
/// Enables efficient spatial queries for large maps.
///
/// # Example
/// ```rust
/// use vastu_map::integration::SpatialIndex;
/// use vastu_map::features::Line2D;
/// use vastu_map::core::Point2D;
///
/// let lines = vec![
///     Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
///     Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
/// ];
///
/// let index = SpatialIndex::new(&lines);
///
/// // Find nearest line to a point
/// if let Some((idx, dist)) = index.nearest_line(Point2D::new(3.0, 0.5)) {
///     println!("Nearest line: {}, distance: {:.3}", idx, dist);
/// }
/// ```
#[derive(Clone)]
pub struct SpatialIndex {
    tree: RTree<IndexedLine>,
}

impl SpatialIndex {
    /// Create a new spatial index from lines.
    pub fn new(lines: &[Line2D]) -> Self {
        let indexed: Vec<IndexedLine> = lines
            .iter()
            .enumerate()
            .map(|(i, line)| IndexedLine::new(*line, i))
            .collect();

        Self {
            tree: RTree::bulk_load(indexed),
        }
    }

    /// Create an empty spatial index.
    pub fn empty() -> Self {
        Self { tree: RTree::new() }
    }

    /// Number of lines in the index.
    pub fn len(&self) -> usize {
        self.tree.size()
    }

    /// Check if index is empty.
    pub fn is_empty(&self) -> bool {
        self.tree.size() == 0
    }

    /// Insert a line into the index.
    ///
    /// O(log n) insertion time.
    pub fn insert(&mut self, line: Line2D, index: usize) {
        self.tree.insert(IndexedLine::new(line, index));
    }

    /// Insert multiple lines into the index.
    ///
    /// For small batches (<20), uses incremental insert.
    /// For large batches, triggers a rebuild for better tree balance.
    ///
    /// # Arguments
    /// * `lines` - Slice of lines to insert
    /// * `start_index` - Starting index for the new lines
    pub fn insert_batch(&mut self, lines: &[Line2D], start_index: usize) {
        // Threshold for incremental vs rebuild
        const REBUILD_THRESHOLD: usize = 50;

        if lines.len() < REBUILD_THRESHOLD {
            // Incremental insert for small batches
            for (i, line) in lines.iter().enumerate() {
                self.tree.insert(IndexedLine::new(*line, start_index + i));
            }
        } else {
            // For large batches, it's more efficient to rebuild
            // Extract existing items and combine with new ones
            let existing: Vec<IndexedLine> = self.tree.iter().cloned().collect();
            let new: Vec<IndexedLine> = lines
                .iter()
                .enumerate()
                .map(|(i, line)| IndexedLine::new(*line, start_index + i))
                .collect();

            let mut all = existing;
            all.extend(new);
            self.tree = RTree::bulk_load(all);
        }
    }

    /// Find the nearest line to a point.
    ///
    /// Returns (index, distance) of the nearest line, or None if index is empty.
    pub fn nearest_line(&self, point: Point2D) -> Option<(usize, f32)> {
        let p = [point.x, point.y];
        self.tree.nearest_neighbor(&p).map(|indexed| {
            let dist = indexed.line.distance_to_point(point);
            (indexed.index, dist)
        })
    }

    /// Find the k nearest lines to a point.
    ///
    /// Returns vector of (index, distance) pairs, sorted by distance.
    pub fn k_nearest_lines(&self, point: Point2D, k: usize) -> Vec<(usize, f32)> {
        let p = [point.x, point.y];
        self.tree
            .nearest_neighbor_iter(&p)
            .take(k)
            .map(|indexed| {
                let dist = indexed.line.distance_to_point(point);
                (indexed.index, dist)
            })
            .collect()
    }

    /// Find all lines within a given distance of a point.
    pub fn lines_within_distance(&self, point: Point2D, max_distance: f32) -> Vec<(usize, f32)> {
        let p = [point.x, point.y];
        let max_dist_sq = max_distance * max_distance;

        self.tree
            .nearest_neighbor_iter(&p)
            .take_while(|indexed| {
                let dist = indexed.line.distance_to_point(point);
                dist * dist <= max_dist_sq
            })
            .map(|indexed| {
                let dist = indexed.line.distance_to_point(point);
                (indexed.index, dist)
            })
            .collect()
    }

    /// Find all lines intersecting a bounding box.
    pub fn lines_in_bounds(&self, bounds: &Bounds) -> Vec<usize> {
        let aabb = AABB::from_corners([bounds.min.x, bounds.min.y], [bounds.max.x, bounds.max.y]);

        self.tree
            .locate_in_envelope_intersecting(&aabb)
            .map(|indexed| indexed.index)
            .collect()
    }

    /// Find all lines that could potentially match a query line.
    ///
    /// Uses the query line's bounding box expanded by a margin.
    pub fn potential_matches(&self, query: &Line2D, margin: f32) -> Vec<usize> {
        let min_x = query.start.x.min(query.end.x) - margin;
        let max_x = query.start.x.max(query.end.x) + margin;
        let min_y = query.start.y.min(query.end.y) - margin;
        let max_y = query.start.y.max(query.end.y) + margin;

        let aabb = AABB::from_corners([min_x, min_y], [max_x, max_y]);

        self.tree
            .locate_in_envelope_intersecting(&aabb)
            .map(|indexed| indexed.index)
            .collect()
    }

    /// Rebuild the index with new lines.
    pub fn rebuild(&mut self, lines: &[Line2D]) {
        let indexed: Vec<IndexedLine> = lines
            .iter()
            .enumerate()
            .map(|(i, line)| IndexedLine::new(*line, i))
            .collect();

        self.tree = RTree::bulk_load(indexed);
    }

    /// Clear the index.
    pub fn clear(&mut self) {
        self.tree = RTree::new();
    }
}

impl Default for SpatialIndex {
    fn default() -> Self {
        Self::empty()
    }
}

impl std::fmt::Debug for SpatialIndex {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SpatialIndex")
            .field("size", &self.tree.size())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_lines() -> Vec<Line2D> {
        vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)), // Horizontal at y=0
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)), // Vertical at x=5
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(0.0, 5.0)), // Horizontal at y=5
            Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(0.0, 0.0)), // Vertical at x=0
            Line2D::new(Point2D::new(10.0, 0.0), Point2D::new(15.0, 0.0)), // Far horizontal
        ]
    }

    #[test]
    fn test_new_index() {
        let lines = make_test_lines();
        let index = SpatialIndex::new(&lines);

        assert_eq!(index.len(), 5);
        assert!(!index.is_empty());
    }

    #[test]
    fn test_empty_index() {
        let index = SpatialIndex::empty();

        assert_eq!(index.len(), 0);
        assert!(index.is_empty());
        assert!(index.nearest_line(Point2D::zero()).is_none());
    }

    #[test]
    fn test_nearest_line() {
        let lines = make_test_lines();
        let index = SpatialIndex::new(&lines);

        // Point near bottom line
        let (idx, dist) = index.nearest_line(Point2D::new(2.5, 0.5)).unwrap();
        assert_eq!(idx, 0);
        assert!((dist - 0.5).abs() < 0.01);

        // Point near right line
        let (idx, dist) = index.nearest_line(Point2D::new(4.5, 2.5)).unwrap();
        assert_eq!(idx, 1);
        assert!((dist - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_k_nearest() {
        let lines = make_test_lines();
        let index = SpatialIndex::new(&lines);

        // Point at corner (near lines 0 and 3)
        let nearest = index.k_nearest_lines(Point2D::new(0.5, 0.5), 2);

        assert_eq!(nearest.len(), 2);
        // Both should be at similar distances
        assert!(nearest[0].1 < 1.0);
        assert!(nearest[1].1 < 1.0);
    }

    #[test]
    fn test_lines_within_distance() {
        let lines = make_test_lines();
        let index = SpatialIndex::new(&lines);

        // Point at center of the room
        let nearby = index.lines_within_distance(Point2D::new(2.5, 2.5), 3.0);

        // Should find at least the 4 room walls (each at distance 2.5)
        // The far line is at distance ~7.9 so should not be included
        assert!(nearby.len() >= 4);
        // Verify all returned distances are within threshold
        for (_, dist) in &nearby {
            assert!(*dist <= 3.0, "Distance {} exceeds threshold", dist);
        }
    }

    #[test]
    fn test_lines_in_bounds() {
        let lines = make_test_lines();
        let index = SpatialIndex::new(&lines);

        // Bounds covering just the room
        let bounds = Bounds::new(Point2D::new(-1.0, -1.0), Point2D::new(6.0, 6.0));
        let in_bounds = index.lines_in_bounds(&bounds);

        // Should find the 4 room walls
        assert_eq!(in_bounds.len(), 4);

        // Bounds covering everything
        let big_bounds = Bounds::new(Point2D::new(-1.0, -1.0), Point2D::new(20.0, 10.0));
        let all = index.lines_in_bounds(&big_bounds);
        assert_eq!(all.len(), 5);
    }

    #[test]
    fn test_potential_matches() {
        let lines = make_test_lines();
        let index = SpatialIndex::new(&lines);

        // Query line similar to the bottom wall
        let query = Line2D::new(Point2D::new(1.0, 0.1), Point2D::new(4.0, 0.1));
        let matches = index.potential_matches(&query, 0.5);

        // Should find the bottom wall
        assert!(matches.contains(&0));
    }

    #[test]
    fn test_insert() {
        let mut index = SpatialIndex::empty();

        index.insert(
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            0,
        );
        index.insert(
            Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0)),
            1,
        );

        assert_eq!(index.len(), 2);

        let (idx, _) = index.nearest_line(Point2D::new(0.5, 0.1)).unwrap();
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_rebuild() {
        let lines1 = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0))];
        let lines2 = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(1.0, 1.0)),
        ];

        let mut index = SpatialIndex::new(&lines1);
        assert_eq!(index.len(), 1);

        index.rebuild(&lines2);
        assert_eq!(index.len(), 2);
    }

    #[test]
    fn test_clear() {
        let lines = make_test_lines();
        let mut index = SpatialIndex::new(&lines);

        assert_eq!(index.len(), 5);

        index.clear();
        assert!(index.is_empty());
    }
}
