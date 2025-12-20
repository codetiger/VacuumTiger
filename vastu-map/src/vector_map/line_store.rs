//! LineStore: Encapsulates parallel line data structures.
//!
//! This module provides a wrapper that keeps multiple representations
//! of line features in sync:
//! - `Vec<Line2D>` for iteration and direct access
//! - `LineCollection` (SoA format) for SIMD operations
//! - `SpatialIndex` (R-tree) for spatial queries
//! - `Bounds` for bounding box queries

use crate::core::{Bounds, Point2D};
use crate::features::{Line2D, LineCollection};
use crate::integration::SpatialIndex;

/// Encapsulates line storage with synchronized auxiliary structures.
///
/// All modifications go through this struct to ensure consistency
/// between the primary line storage and derived data structures.
pub struct LineStore {
    /// Primary line storage.
    lines: Vec<Line2D>,
    /// SoA format for SIMD batch operations.
    collection: LineCollection,
    /// R-tree spatial index for fast queries.
    index: SpatialIndex,
    /// Bounding box of all lines.
    bounds: Option<Bounds>,
}

impl LineStore {
    /// Create a new empty LineStore.
    pub fn new() -> Self {
        Self {
            lines: Vec::new(),
            collection: LineCollection::new(),
            index: SpatialIndex::empty(),
            bounds: None,
        }
    }

    /// Create a LineStore with pre-allocated capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            lines: Vec::with_capacity(capacity),
            collection: LineCollection::with_capacity(capacity),
            index: SpatialIndex::empty(),
            bounds: None,
        }
    }

    /// Check if the store is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.lines.is_empty()
    }

    /// Get the number of lines.
    #[inline]
    pub fn len(&self) -> usize {
        self.lines.len()
    }

    /// Get all lines.
    #[inline]
    pub fn lines(&self) -> &[Line2D] {
        &self.lines
    }

    /// Get the LineCollection (SoA format).
    #[inline]
    pub fn collection(&self) -> &LineCollection {
        &self.collection
    }

    /// Get the spatial index.
    #[inline]
    pub fn index(&self) -> &SpatialIndex {
        &self.index
    }

    /// Get the current bounds.
    #[inline]
    pub fn bounds(&self) -> Option<&Bounds> {
        self.bounds.as_ref()
    }

    /// Add a single line to the store.
    ///
    /// Updates all auxiliary structures incrementally.
    pub fn add(&mut self, line: Line2D) {
        let idx = self.lines.len();
        self.lines.push(line);
        self.collection.push_line(&line);
        self.update_bounds(&line);
        self.index.insert(line, idx);
    }

    /// Add multiple lines to the store.
    ///
    /// Uses incremental updates for small additions, full rebuild for large ones.
    pub fn add_many(&mut self, lines: &[Line2D]) {
        if lines.is_empty() {
            return;
        }

        // Threshold for incremental vs full rebuild
        const INCREMENTAL_THRESHOLD: usize = 20;

        let start_idx = self.lines.len();

        for line in lines {
            self.lines.push(*line);
            self.collection.push_line(line);
            self.update_bounds(line);
        }

        if lines.len() <= INCREMENTAL_THRESHOLD && self.lines.len() > lines.len() * 2 {
            // Incremental insert for small additions to large maps
            for (i, line) in lines.iter().enumerate() {
                self.index.insert(*line, start_idx + i);
            }
        } else {
            // Full rebuild for large additions or small maps
            self.rebuild_index();
        }
    }

    /// Update a line at a specific index.
    ///
    /// Rebuilds auxiliary structures after modification.
    pub fn update(&mut self, idx: usize, new_line: Line2D) {
        if idx < self.lines.len() {
            self.lines[idx] = new_line;
            self.rebuild_all();
        }
    }

    /// Get mutable access to the lines vector.
    ///
    /// After modifying lines directly, call `rebuild_all()` to sync structures.
    pub fn lines_mut(&mut self) -> &mut Vec<Line2D> {
        &mut self.lines
    }

    /// Rebuild all auxiliary structures from the lines vector.
    ///
    /// Call this after making direct modifications to `lines_mut()`.
    pub fn rebuild_all(&mut self) {
        self.rebuild_collection();
        self.rebuild_index();
        self.recompute_bounds();
    }

    /// Clear all lines and auxiliary structures.
    pub fn clear(&mut self) {
        self.lines.clear();
        self.collection.clear();
        self.index.clear();
        self.bounds = None;
    }

    /// Update bounds to include a line.
    fn update_bounds(&mut self, line: &Line2D) {
        match &mut self.bounds {
            Some(b) => {
                b.expand_to_include(line.start);
                b.expand_to_include(line.end);
            }
            None => {
                let mut bounds = Bounds::from_point(line.start);
                bounds.expand_to_include(line.end);
                self.bounds = Some(bounds);
            }
        }
    }

    /// Recompute bounds from all lines.
    fn recompute_bounds(&mut self) {
        if self.lines.is_empty() {
            self.bounds = None;
            return;
        }

        let mut bounds = Bounds::from_point(self.lines[0].start);
        for line in &self.lines {
            bounds.expand_to_include(line.start);
            bounds.expand_to_include(line.end);
        }
        self.bounds = Some(bounds);
    }

    /// Rebuild the spatial index from lines.
    fn rebuild_index(&mut self) {
        self.index.rebuild(&self.lines);
    }

    /// Rebuild the LineCollection from lines.
    fn rebuild_collection(&mut self) {
        self.collection = LineCollection::from_lines(&self.lines);
    }

    /// Query lines within distance of a point.
    pub fn query_near(&self, point: Point2D, distance: f32) -> Vec<(usize, f32)> {
        self.index.lines_within_distance(point, distance)
    }
}

impl Default for LineStore {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_store_is_empty() {
        let store = LineStore::new();
        assert!(store.is_empty());
        assert_eq!(store.len(), 0);
        assert!(store.bounds().is_none());
    }

    #[test]
    fn test_add_single_line() {
        let mut store = LineStore::new();
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));

        store.add(line);

        assert_eq!(store.len(), 1);
        assert!(!store.is_empty());
        assert!(store.bounds().is_some());
        assert_eq!(store.lines()[0], line);
    }

    #[test]
    fn test_add_many_lines() {
        let mut store = LineStore::new();
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(0.0, 5.0)),
        ];

        store.add_many(&lines);

        assert_eq!(store.len(), 3);
        assert_eq!(store.collection().len(), 3);
    }

    #[test]
    fn test_bounds_grow() {
        let mut store = LineStore::new();

        store.add(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)));
        let bounds1 = store.bounds().unwrap().clone();

        store.add(Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(5.0, 5.0)));
        let bounds2 = store.bounds().unwrap();

        assert!(bounds2.width() > bounds1.width());
        assert!(bounds2.height() > bounds1.height());
    }

    #[test]
    fn test_clear() {
        let mut store = LineStore::new();
        store.add(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)));

        store.clear();

        assert!(store.is_empty());
        assert!(store.bounds().is_none());
    }

    #[test]
    fn test_rebuild_after_mutation() {
        let mut store = LineStore::new();
        store.add(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)));

        // Modify directly
        store.lines_mut()[0] = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));
        store.rebuild_all();

        // Bounds should reflect the new line
        let bounds = store.bounds().unwrap();
        assert!(bounds.max.x >= 10.0);
    }
}
