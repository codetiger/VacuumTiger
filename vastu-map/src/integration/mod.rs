//! Map integration for combining scan features with the map.
//!
//! This module provides algorithms for:
//! - Spatial indexing (R-tree) for efficient queries
//! - Line association (matching scan lines to map lines)
//! - Feature merging (updating map lines with observations)
//!
//! # Integration Pipeline
//!
//! 1. **Associate**: Find correspondences between scan and map lines
//! 2. **Merge**: Update matched map lines with new observations
//! 3. **Add**: Insert unmatched scan lines as new map features
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_map::integration::{
//!     SpatialIndex, find_associations, batch_merge, create_new_line,
//!     AssociationConfig, MergerConfig,
//! };
//! use vastu_map::features::Line2D;
//!
//! // Extract features from scan
//! let scan_lines: Vec<Line2D> = extract_from_scan();
//!
//! // Find associations with map
//! let index = SpatialIndex::new(&map_lines);
//! let assoc_config = AssociationConfig::default();
//! let associations = find_associations(&scan_lines, &map_lines, Some(&index), &assoc_config);
//!
//! // Merge matched lines
//! let merger_config = MergerConfig::default();
//! batch_merge(&scan_lines, &mut map_lines, &associations, &merger_config);
//!
//! // Add unmatched lines as new features
//! let unmatched = find_unmatched_scan_lines(&scan_lines, &map_lines, Some(&index), &assoc_config);
//! for idx in unmatched {
//!     map_lines.push(create_new_line(&scan_lines[idx]));
//! }
//! ```

pub mod association;
pub mod merger;
pub mod spatial_index;

// Re-export main types
pub use association::{
    Association, AssociationConfig, find_associations, find_unique_associations,
    find_unmatched_scan_lines,
};
pub use merger::{
    MergeResult, MergerConfig, batch_merge, create_new_line, merge_collinear_lines, merge_lines,
};
pub use spatial_index::{IndexedLine, SpatialIndex};
