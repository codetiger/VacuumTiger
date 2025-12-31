//! Grid storage implementation for the occupancy grid.
//!
//! Uses Structure-of-Arrays (SoA) layout for SIMD optimization.
//! Cell data is stored in separate arrays for each field, enabling
//! efficient vectorized operations on ARM NEON and x86 SSE/AVX.
//!
//! ## Memory Layout
//!
//! ```text
//! Traditional AoS:  [Cell₀] [Cell₁] [Cell₂] [Cell₃] ...
//!                    ↓       ↓       ↓       ↓
//!                   TCOSLTCOSLTCOSLTCOSL  (interleaved)
//!
//! Our SoA Layout:   Types:    [T T T T T T T T T T T T T T T T|...]
//!                   Conf:     [C C C C C C C C C C C C C C C C|...]
//!                   Obs:      [O O O O O O O O O O O O O O O O|...]
//!                   Swept:    [S S S S S S S S S S S S S S S S|...]
//!                   LogOdds:  [L L L L L L L L L L L L L L L L|...]
//!                   Distance: [D D D D D D D D D D D D D D D D|...]
//!                              └──────── 16 bytes ────────────┘
//!                                    SIMD u8x16 load
//! ```
//!
//! ## Key Types
//!
//! - [`GridStorage`]: Main storage struct with coordinate conversion,
//!   log-odds updates, and SIMD-optimized queries
//! - [`CellMut`]: Mutable reference to a single cell's fields
//! - [`CellCounts`]: Statistics for each cell type

mod core;
mod types;

pub use self::core::GridStorage;
pub use types::{CellCounts, CellMut};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{CellType, GridCoord, WorldPoint};
    use crate::grid::LogOddsConfig;

    #[test]
    fn test_grid_creation() {
        let grid = GridStorage::new(100, 100, 0.05, WorldPoint::ZERO);
        assert_eq!(grid.width(), 100);
        assert_eq!(grid.height(), 100);
        assert_eq!(grid.resolution(), 0.05);
        assert_eq!(grid.cell_count(), 10000);
    }

    #[test]
    fn test_centered_grid() {
        let grid = GridStorage::centered(100, 100, 0.05);
        let (min, max) = grid.bounds();

        assert!((min.x + 2.5).abs() < 1e-6);
        assert!((min.y + 2.5).abs() < 1e-6);
        assert!((max.x - 2.5).abs() < 1e-6);
        assert!((max.y - 2.5).abs() < 1e-6);
    }

    #[test]
    fn test_world_to_grid_conversion() {
        let grid = GridStorage::new(100, 100, 0.05, WorldPoint::ZERO);

        let coord = grid.world_to_grid(WorldPoint::new(0.0, 0.0));
        assert_eq!(coord, GridCoord::new(0, 0));

        let coord = grid.world_to_grid(WorldPoint::new(1.0, 1.0));
        assert_eq!(coord, GridCoord::new(20, 20));
    }

    #[test]
    fn test_grid_to_world_conversion() {
        let grid = GridStorage::new(100, 100, 0.05, WorldPoint::ZERO);

        let point = grid.grid_to_world(GridCoord::new(0, 0));
        assert!((point.x - 0.025).abs() < 1e-6);
        assert!((point.y - 0.025).abs() < 1e-6);
    }

    #[test]
    fn test_get_set_cell() {
        let mut grid = GridStorage::new(10, 10, 0.1, WorldPoint::ZERO);

        assert_eq!(grid.get_type(GridCoord::new(5, 5)), CellType::Unknown);

        grid.set_type(GridCoord::new(5, 5), CellType::Floor);
        assert_eq!(grid.get_type(GridCoord::new(5, 5)), CellType::Floor);

        assert_eq!(grid.get_type(GridCoord::new(100, 100)), CellType::Unknown);
    }

    #[test]
    fn test_grid_expansion() {
        let mut grid = GridStorage::centered(10, 10, 0.1);

        assert_eq!(grid.width(), 10);
        assert_eq!(grid.height(), 10);

        let far_point = WorldPoint::new(2.0, 2.0);
        assert!(grid.expand_to_include(far_point, 100, 100));

        assert!(grid.width() > 10);
        assert!(grid.height() > 10);
        assert!(grid.contains_point(far_point));
    }

    #[test]
    fn test_cell_counts() {
        let mut grid = GridStorage::new(10, 10, 0.1, WorldPoint::ZERO);

        grid.set_type(GridCoord::new(0, 0), CellType::Floor);
        grid.set_type(GridCoord::new(1, 0), CellType::Wall);
        grid.set_type(GridCoord::new(2, 0), CellType::Cliff);
        grid.set_type(GridCoord::new(3, 0), CellType::Bump);

        let counts = grid.count_by_type();
        assert_eq!(counts.floor, 1);
        assert_eq!(counts.wall, 1);
        assert_eq!(counts.cliff, 1);
        assert_eq!(counts.bump, 1);
        assert_eq!(counts.unknown, 96);
        assert_eq!(counts.known(), 4);
    }

    #[test]
    fn test_soa_iter() {
        let mut grid = GridStorage::new(10, 10, 0.1, WorldPoint::ZERO);
        grid.set_type(GridCoord::new(5, 5), CellType::Wall);

        let mut found_wall = false;
        for (coord, cell) in grid.iter() {
            if coord == GridCoord::new(5, 5) {
                assert_eq!(cell.cell_type, CellType::Wall);
                found_wall = true;
            }
        }
        assert!(found_wall);
    }

    #[test]
    fn test_cell_priority() {
        let mut grid = GridStorage::new(10, 10, 0.1, WorldPoint::ZERO);

        grid.set_type(GridCoord::new(0, 0), CellType::Floor);
        assert_eq!(grid.get_type(GridCoord::new(0, 0)), CellType::Floor);

        grid.set_type_with_priority(GridCoord::new(0, 0), CellType::Wall);
        assert_eq!(grid.get_type(GridCoord::new(0, 0)), CellType::Wall);

        grid.set_type_with_priority(GridCoord::new(0, 0), CellType::Floor);
        assert_eq!(grid.get_type(GridCoord::new(0, 0)), CellType::Wall);

        grid.set_type_with_priority(GridCoord::new(0, 0), CellType::Bump);
        assert_eq!(grid.get_type(GridCoord::new(0, 0)), CellType::Bump);
    }

    #[test]
    fn test_log_odds_probability_conversion() {
        assert!((GridStorage::log_odds_to_probability(0) - 0.5).abs() < 0.01);
        assert!((GridStorage::log_odds_to_probability(100) - 0.73).abs() < 0.01);
        assert!((GridStorage::log_odds_to_probability(-100) - 0.27).abs() < 0.01);
        assert!((GridStorage::log_odds_to_probability(200) - 0.88).abs() < 0.01);
        assert!((GridStorage::log_odds_to_probability(-200) - 0.12).abs() < 0.01);
    }

    #[test]
    fn test_log_odds_apply_hit() {
        let aggressive = LogOddsConfig::aggressive();
        let mut grid = GridStorage::centered_with_config(10, 10, 0.1, aggressive.clone());
        let coord = GridCoord::new(5, 5);

        assert_eq!(grid.get_log_odds(coord), 0);
        assert!(!grid.is_occupied(coord));

        for i in 0..3 {
            let became_occupied = grid.apply_hit(coord);
            if i == 0 {
                assert!(became_occupied);
            }
        }

        assert!(grid.is_occupied(coord));
        assert_eq!(grid.get_type(coord), CellType::Wall);

        let log_odds = grid.get_log_odds(coord);
        assert!(log_odds > grid.log_odds_config().l_occupied_threshold);
    }

    #[test]
    fn test_log_odds_apply_miss() {
        let aggressive = LogOddsConfig::aggressive();
        let mut grid = GridStorage::centered_with_config(10, 10, 0.1, aggressive);
        let coord = GridCoord::new(5, 5);

        for i in 0..5 {
            let became_free = grid.apply_miss(coord);
            if i == 1 {
                assert!(became_free);
            }
        }

        assert!(grid.is_free(coord));
        assert_eq!(grid.get_type(coord), CellType::Floor);
    }

    #[test]
    fn test_log_odds_cartographer_default() {
        let mut grid = GridStorage::new(10, 10, 0.1, WorldPoint::ZERO);
        let coord = GridCoord::new(5, 5);

        assert!(!grid.apply_hit(coord));
        assert!(!grid.apply_hit(coord));
        assert!(grid.apply_hit(coord));

        assert!(grid.is_occupied(coord));
    }

    #[test]
    fn test_log_odds_balanced_config() {
        let mut grid = GridStorage::centered_with_config(10, 10, 0.1, LogOddsConfig::balanced());
        let coord = GridCoord::new(5, 5);

        assert!(!grid.apply_hit(coord));
        assert!(grid.apply_hit(coord));

        assert!(grid.is_occupied(coord));
    }

    #[test]
    fn test_log_odds_conflicting_observations() {
        let aggressive = LogOddsConfig::aggressive();
        let mut grid = GridStorage::centered_with_config(10, 10, 0.1, aggressive);
        let coord = GridCoord::new(5, 5);

        for _ in 0..3 {
            grid.apply_hit(coord);
        }
        assert!(grid.is_occupied(coord));
        let initial_log_odds = grid.get_log_odds(coord);

        for _ in 0..2 {
            grid.apply_miss(coord);
        }

        let after_miss = grid.get_log_odds(coord);
        assert!(after_miss < initial_log_odds);
        assert!(grid.is_occupied(coord));
    }

    #[test]
    fn test_log_odds_clamping() {
        let mut grid = GridStorage::new(10, 10, 0.1, WorldPoint::ZERO);
        let coord = GridCoord::new(5, 5);
        let cfg = grid.log_odds_config().clone();

        for _ in 0..100 {
            grid.apply_hit(coord);
        }
        assert_eq!(grid.get_log_odds(coord), cfg.l_max);

        for _ in 0..100 {
            grid.apply_miss(coord);
        }
        assert_eq!(grid.get_log_odds(coord), cfg.l_min);
    }

    #[test]
    fn test_simd_count_matches_scalar() {
        let mut grid = GridStorage::new(100, 100, 0.05, WorldPoint::ZERO);

        for i in 0..100 {
            grid.set_type(GridCoord::new(i, 0), CellType::Floor);
            grid.set_type(GridCoord::new(i, 1), CellType::Wall);
        }
        for i in 0..50 {
            grid.set_type(GridCoord::new(i, 2), CellType::Cliff);
            grid.set_type(GridCoord::new(i, 3), CellType::Bump);
        }

        let simd_counts = grid.count_by_type();
        let scalar_counts = grid.count_by_type_scalar();

        assert_eq!(simd_counts.unknown, scalar_counts.unknown);
        assert_eq!(simd_counts.floor, scalar_counts.floor);
        assert_eq!(simd_counts.wall, scalar_counts.wall);
        assert_eq!(simd_counts.cliff, scalar_counts.cliff);
        assert_eq!(simd_counts.bump, scalar_counts.bump);
    }
}
