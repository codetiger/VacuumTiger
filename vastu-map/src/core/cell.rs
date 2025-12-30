//! Cell types for the occupancy grid.
//!
//! VastuMap uses semantic cell types that distinguish between different
//! obstacle sources (walls, cliffs, invisible obstacles like glass).

use serde::{Deserialize, Serialize};

/// Semantic cell type - what kind of obstacle/surface is this?
///
/// The cell type hierarchy:
/// - `Unknown` - Not yet observed by any sensor
/// - `Floor` - Traversable surface (lidar sees through, no cliff, no bumper hit)
/// - `Wall` - Lidar-detected obstacle (solid wall, furniture)
/// - `Cliff` - Floor drop-off detected by cliff sensors (stairs, ledges)
/// - `Bump` - Invisible to lidar but bumper hit (glass doors, mirrors, thin legs)
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[repr(u8)]
pub enum CellType {
    /// Cell has never been observed by any sensor
    #[default]
    Unknown = 0,

    /// Traversable floor surface
    /// - Lidar rays pass through (or hit far away)
    /// - Cliff sensors report safe
    /// - No bumper collision history
    Floor = 1,

    /// Solid wall or obstacle detected by lidar
    /// - Lidar ray terminates here
    /// - Typical: walls, furniture, doors
    Wall = 2,

    /// Floor drop-off detected by cliff sensors
    /// - Lidar may see far (looks like floor)
    /// - Cliff sensor triggered at this position
    /// - Typical: stairs, ledges, thresholds
    Cliff = 3,

    /// Invisible obstacle - lidar passes through but physical collision
    /// - Lidar ray passes through (appears as Floor)
    /// - Bumper triggered collision at this position
    /// - Typical: glass doors, mirrors, thin table legs
    Bump = 4,
}

impl CellType {
    /// Can the robot traverse this cell?
    #[inline]
    pub fn is_traversable(self) -> bool {
        matches!(self, CellType::Floor)
    }

    /// Is this cell an obstacle (any type)?
    #[inline]
    pub fn is_obstacle(self) -> bool {
        matches!(self, CellType::Wall | CellType::Cliff | CellType::Bump)
    }

    /// Has this cell been observed?
    #[inline]
    pub fn is_known(self) -> bool {
        self != CellType::Unknown
    }

    /// Convert from u8 (for deserialization)
    #[inline]
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => CellType::Unknown,
            1 => CellType::Floor,
            2 => CellType::Wall,
            3 => CellType::Cliff,
            4 => CellType::Bump,
            _ => CellType::Unknown,
        }
    }

    /// Single character representation for debugging
    pub fn as_char(self) -> char {
        match self {
            CellType::Unknown => '?',
            CellType::Floor => '.',
            CellType::Wall => '#',
            CellType::Cliff => 'v',
            CellType::Bump => 'x',
        }
    }
}

/// A single cell in the grid with metadata
#[derive(Clone, Copy, Debug, Default)]
pub struct Cell {
    /// What type of cell is this?
    pub cell_type: CellType,

    /// Confidence level (0-255)
    /// - 0 = unknown/unobserved
    /// - Higher = more confident (many consistent observations)
    pub confidence: u8,

    /// Observation count (saturates at 255)
    pub observation_count: u8,

    /// Has this cell been swept/cleaned?
    pub swept: bool,
}

impl Cell {
    /// Create a new unknown cell
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a cell with a specific type
    #[inline]
    pub fn with_type(cell_type: CellType) -> Self {
        Self {
            cell_type,
            confidence: 1,
            observation_count: 1,
            swept: false,
        }
    }

    /// Is this cell traversable?
    #[inline]
    pub fn is_traversable(&self) -> bool {
        self.cell_type.is_traversable()
    }

    /// Is this cell an obstacle?
    #[inline]
    pub fn is_obstacle(&self) -> bool {
        self.cell_type.is_obstacle()
    }

    /// Has this cell been observed?
    #[inline]
    pub fn is_known(&self) -> bool {
        self.cell_type.is_known()
    }

    /// Update cell with a new observation of the same type
    /// Returns true if cell type changed
    pub fn observe(&mut self, observed_type: CellType) -> bool {
        let changed = self.cell_type != observed_type;

        if changed {
            // Type changed - reset confidence
            self.cell_type = observed_type;
            self.confidence = 1;
            self.observation_count = 1;
        } else {
            // Same type - increase confidence
            self.confidence = self.confidence.saturating_add(1);
            self.observation_count = self.observation_count.saturating_add(1);
        }

        changed
    }

    /// Update cell with a new observation, with priority rules
    ///
    /// Priority (higher overrides lower):
    /// 1. Bump (physical collision is definitive)
    /// 2. Cliff (safety critical)
    /// 3. Wall (lidar detection)
    /// 4. Floor (absence of obstacles)
    /// 5. Unknown (no data)
    pub fn observe_with_priority(&mut self, observed_type: CellType) -> bool {
        let should_update = match (self.cell_type, observed_type) {
            // Unknown can be updated by anything
            (CellType::Unknown, _) => true,

            // Bump is highest priority - only update with another Bump
            (CellType::Bump, CellType::Bump) => true,
            (CellType::Bump, _) => false,

            // Cliff can be updated by Bump or Cliff
            (CellType::Cliff, CellType::Bump) => true,
            (CellType::Cliff, CellType::Cliff) => true,
            (CellType::Cliff, _) => false,

            // Wall can be updated by Bump, Cliff, or Wall
            (CellType::Wall, CellType::Bump) => true,
            (CellType::Wall, CellType::Cliff) => true,
            (CellType::Wall, CellType::Wall) => true,
            (CellType::Wall, _) => false,

            // Floor can be updated by anything
            (CellType::Floor, _) => true,
        };

        if should_update {
            self.observe(observed_type)
        } else {
            // Still count the observation even if type didn't change
            self.observation_count = self.observation_count.saturating_add(1);
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cell_type_traversable() {
        assert!(!CellType::Unknown.is_traversable());
        assert!(CellType::Floor.is_traversable());
        assert!(!CellType::Wall.is_traversable());
        assert!(!CellType::Cliff.is_traversable());
        assert!(!CellType::Bump.is_traversable());
    }

    #[test]
    fn test_cell_type_obstacle() {
        assert!(!CellType::Unknown.is_obstacle());
        assert!(!CellType::Floor.is_obstacle());
        assert!(CellType::Wall.is_obstacle());
        assert!(CellType::Cliff.is_obstacle());
        assert!(CellType::Bump.is_obstacle());
    }

    #[test]
    fn test_cell_observe() {
        let mut cell = Cell::new();
        assert!(!cell.is_known());

        // First observation
        assert!(cell.observe(CellType::Floor));
        assert_eq!(cell.cell_type, CellType::Floor);
        assert_eq!(cell.confidence, 1);

        // Same observation increases confidence
        assert!(!cell.observe(CellType::Floor));
        assert_eq!(cell.confidence, 2);

        // Different observation resets confidence
        assert!(cell.observe(CellType::Wall));
        assert_eq!(cell.cell_type, CellType::Wall);
        assert_eq!(cell.confidence, 1);
    }

    #[test]
    fn test_cell_priority() {
        let mut cell = Cell::with_type(CellType::Floor);

        // Wall overrides Floor
        assert!(cell.observe_with_priority(CellType::Wall));
        assert_eq!(cell.cell_type, CellType::Wall);

        // Floor does NOT override Wall
        assert!(!cell.observe_with_priority(CellType::Floor));
        assert_eq!(cell.cell_type, CellType::Wall);

        // Bump overrides Wall
        assert!(cell.observe_with_priority(CellType::Bump));
        assert_eq!(cell.cell_type, CellType::Bump);

        // Nothing overrides Bump except Bump
        assert!(!cell.observe_with_priority(CellType::Floor));
        assert!(!cell.observe_with_priority(CellType::Wall));
        assert!(!cell.observe_with_priority(CellType::Cliff));
        assert_eq!(cell.cell_type, CellType::Bump);
    }
}
