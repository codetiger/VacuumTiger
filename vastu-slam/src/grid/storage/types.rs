//! Types used by grid storage.

use crate::core::CellType;
use serde::{Deserialize, Serialize};

/// Mutable reference to a single cell's fields in the SoA storage.
///
/// This allows modifying individual cell properties without reconstructing
/// the entire Cell struct.
pub struct CellMut<'a> {
    /// Cell type as raw u8.
    pub cell_type: &'a mut u8,
    /// Confidence level.
    pub confidence: &'a mut u8,
    /// Observation count.
    pub observation_count: &'a mut u8,
    /// Swept flag (0 = not swept, 1 = swept).
    pub swept: &'a mut u8,
}

impl<'a> CellMut<'a> {
    /// Get the cell type.
    #[inline]
    pub fn get_type(&self) -> CellType {
        CellType::from_u8(*self.cell_type)
    }

    /// Set the cell type.
    #[inline]
    pub fn set_type(&mut self, cell_type: CellType) {
        *self.cell_type = cell_type as u8;
    }

    /// Check if traversable.
    #[inline]
    pub fn is_traversable(&self) -> bool {
        *self.cell_type == CellType::Floor as u8
    }

    /// Check if obstacle.
    #[inline]
    pub fn is_obstacle(&self) -> bool {
        let t = *self.cell_type;
        t == CellType::Wall as u8 || t == CellType::Cliff as u8 || t == CellType::Bump as u8
    }

    /// Update with a new observation.
    pub fn observe(&mut self, observed_type: CellType) -> bool {
        let new_type = observed_type as u8;
        let changed = *self.cell_type != new_type;

        if changed {
            *self.cell_type = new_type;
            *self.confidence = 1;
            *self.observation_count = 1;
        } else {
            *self.confidence = self.confidence.saturating_add(1);
            *self.observation_count = self.observation_count.saturating_add(1);
        }

        changed
    }

    /// Update with priority rules.
    pub fn observe_with_priority(&mut self, observed_type: CellType) -> bool {
        let old_type = CellType::from_u8(*self.cell_type);
        let should_update = match (old_type, observed_type) {
            (CellType::Unknown, _) => true,
            (CellType::Bump, CellType::Bump) => true,
            (CellType::Bump, _) => false,
            (CellType::Cliff, CellType::Bump | CellType::Cliff) => true,
            (CellType::Cliff, _) => false,
            (CellType::Wall, CellType::Bump | CellType::Cliff | CellType::Wall) => true,
            (CellType::Wall, _) => false,
            (CellType::Floor, _) => true,
        };

        if should_update {
            self.observe(observed_type)
        } else {
            *self.observation_count = self.observation_count.saturating_add(1);
            false
        }
    }
}

/// Cell counts by type.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct CellCounts {
    /// Unknown cells (not yet observed).
    pub unknown: usize,
    /// Floor cells (traversable).
    pub floor: usize,
    /// Wall cells (lidar-detected obstacles).
    pub wall: usize,
    /// Cliff cells (floor drop-offs).
    pub cliff: usize,
    /// Bump cells (invisible obstacles).
    pub bump: usize,
}

impl CellCounts {
    /// Total known cells.
    pub fn known(&self) -> usize {
        self.floor + self.wall + self.cliff + self.bump
    }

    /// Total cells.
    pub fn total(&self) -> usize {
        self.unknown + self.known()
    }

    /// Total obstacle cells.
    pub fn obstacles(&self) -> usize {
        self.wall + self.cliff + self.bump
    }
}
