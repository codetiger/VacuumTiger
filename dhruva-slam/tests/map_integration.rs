//! Map integration tests for coordinate transform validation.
//!
//! These tests verify that laser scans are correctly transformed
//! based on robot pose when integrated into the occupancy grid.
//!
//! Phase 3 of unit-test-proposal.md

use dhruva_slam::algorithms::mapping::{
    CellState, MapIntegrator, MapIntegratorConfig, OccupancyGrid, OccupancyGridConfig,
};
use dhruva_slam::core::types::{LaserScan, Pose2D};
use std::f32::consts::PI;

fn create_test_grid() -> OccupancyGrid {
    OccupancyGrid::new(OccupancyGridConfig {
        resolution: 0.05,
        initial_width: 10.0,
        initial_height: 10.0,
        ..Default::default()
    })
}

/// Create a scan with a single point at given polar coordinates.
fn create_single_point_scan(range: f32, angle: f32) -> LaserScan {
    LaserScan {
        angle_min: angle,
        angle_max: angle,
        angle_increment: 0.0,
        range_min: 0.1,
        range_max: 12.0,
        ranges: vec![range],
        intensities: None,
        angles: Some(vec![angle]),
    }
}

/// Check if a cell at given world coordinates is occupied.
fn is_cell_occupied(grid: &OccupancyGrid, x: f32, y: f32) -> bool {
    if let Some((cx, cy)) = grid.world_to_cell(x, y) {
        grid.get_state(cx, cy) == CellState::Occupied
    } else {
        false
    }
}

/// Check if a cell at given world coordinates is free.
fn is_cell_free(grid: &OccupancyGrid, x: f32, y: f32) -> bool {
    if let Some((cx, cy)) = grid.world_to_cell(x, y) {
        grid.get_state(cx, cy) == CellState::Free
    } else {
        false
    }
}

#[test]
fn test_integrate_scan_at_identity_pose() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Single point 1m directly in front (angle=0, local +X)
    let scan = create_single_point_scan(1.0, 0.0);
    let robot_pose = Pose2D::identity();

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Point should be at world (1.0, 0.0)
    assert!(
        is_cell_occupied(&grid, 1.0, 0.0),
        "Point at identity pose should be at (1, 0)"
    );
}

#[test]
fn test_integrate_scan_at_90_degree_rotation() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Single point 1m in front (local +X)
    let scan = create_single_point_scan(1.0, 0.0);

    // Robot at origin, facing +Y (90° CCW from +X)
    let robot_pose = Pose2D::new(0.0, 0.0, PI / 2.0);

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Point should be at world (0.0, 1.0), NOT (1.0, 0.0)
    assert!(
        is_cell_occupied(&grid, 0.0, 1.0),
        "90° rotation: point should be at (0, 1)"
    );

    // Verify it's NOT at wrong location
    assert!(
        !is_cell_occupied(&grid, 1.0, 0.0),
        "90° rotation: point should NOT be at (1, 0)"
    );
}

#[test]
fn test_integrate_scan_at_180_degree_rotation() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Single point 1m in front
    let scan = create_single_point_scan(1.0, 0.0);

    // Robot at origin, facing -X (180° from +X)
    let robot_pose = Pose2D::new(0.0, 0.0, PI);

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Point should be at world (-1.0, 0.0)
    assert!(
        is_cell_occupied(&grid, -1.0, 0.0),
        "180° rotation: point should be at (-1, 0)"
    );
}

#[test]
fn test_integrate_scan_at_270_degree_rotation() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Single point 1m in front
    let scan = create_single_point_scan(1.0, 0.0);

    // Robot at origin, facing -Y (270° CCW = -90°)
    let robot_pose = Pose2D::new(0.0, 0.0, -PI / 2.0);

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Point should be at world (0.0, -1.0)
    assert!(
        is_cell_occupied(&grid, 0.0, -1.0),
        "270° rotation: point should be at (0, -1)"
    );
}

#[test]
fn test_integrate_scan_at_45_degree_rotation() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Single point 1m in front
    let scan = create_single_point_scan(1.0, 0.0);

    // Robot at origin, rotated 45° CCW
    let robot_pose = Pose2D::new(0.0, 0.0, PI / 4.0);

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Point should be at world (cos(45°), sin(45°)) ≈ (0.707, 0.707)
    let expected_x = (PI / 4.0).cos();
    let expected_y = (PI / 4.0).sin();

    assert!(
        is_cell_occupied(&grid, expected_x, expected_y),
        "45° rotation: point should be at ({:.3}, {:.3})",
        expected_x,
        expected_y
    );
}

#[test]
fn test_integrate_scan_at_translated_pose() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Single point 1m in front
    let scan = create_single_point_scan(1.0, 0.0);

    // Robot at (2, 3), facing +X
    let robot_pose = Pose2D::new(2.0, 3.0, 0.0);

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Point should be at world (3.0, 3.0)
    assert!(
        is_cell_occupied(&grid, 3.0, 3.0),
        "Translated pose: point should be at (3, 3)"
    );
}

#[test]
fn test_integrate_scan_combined_transform() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Single point 1m in front
    let scan = create_single_point_scan(1.0, 0.0);

    // Robot at (1, 1), facing +Y (90° CCW)
    let robot_pose = Pose2D::new(1.0, 1.0, PI / 2.0);

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Local (1, 0) → rotate 90° → (0, 1) → translate by (1, 1) → (1, 2)
    assert!(
        is_cell_occupied(&grid, 1.0, 2.0),
        "Combined transform: point should be at (1, 2)"
    );
}

#[test]
fn test_integrate_two_scans_alignment() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // First scan: robot at origin facing +X, point 2m ahead
    let scan1 = create_single_point_scan(2.0, 0.0);
    let pose1 = Pose2D::identity();
    integrator.integrate_scan(&mut grid, &scan1, &pose1);

    // Second scan: robot moved to (1, 0), still facing +X, point 1m ahead
    // This should place a point at world (2, 0) - same as scan1
    let scan2 = create_single_point_scan(1.0, 0.0);
    let pose2 = Pose2D::new(1.0, 0.0, 0.0);
    integrator.integrate_scan(&mut grid, &scan2, &pose2);

    // Both scans should have contributed to cell at (2, 0)
    assert!(
        is_cell_occupied(&grid, 2.0, 0.0),
        "Both scans should mark (2, 0) as occupied"
    );
}

#[test]
fn test_integrate_scan_with_scan_angle() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Point at 45° in scan frame, 1m away
    let scan = create_single_point_scan(1.0, PI / 4.0);

    // Robot at origin, facing +X
    let robot_pose = Pose2D::identity();

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Point should be at world (cos(45°), sin(45°))
    let expected_x = (PI / 4.0).cos();
    let expected_y = (PI / 4.0).sin();

    assert!(
        is_cell_occupied(&grid, expected_x, expected_y),
        "Scan angle 45°: point should be at ({:.3}, {:.3})",
        expected_x,
        expected_y
    );
}

#[test]
fn test_integrate_scan_combined_robot_and_scan_angles() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Point at 45° in scan frame, 1m away
    let scan = create_single_point_scan(1.0, PI / 4.0);

    // Robot at origin, facing +Y (90° CCW)
    let robot_pose = Pose2D::new(0.0, 0.0, PI / 2.0);

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Scan angle 45° + robot angle 90° = 135° global
    // Point should be at (cos(135°), sin(135°)) ≈ (-0.707, 0.707)
    let global_angle = PI / 4.0 + PI / 2.0;
    let expected_x = global_angle.cos();
    let expected_y = global_angle.sin();

    assert!(
        is_cell_occupied(&grid, expected_x, expected_y),
        "Combined angles: point should be at ({:.3}, {:.3})",
        expected_x,
        expected_y
    );
}

#[test]
fn test_ray_tracing_marks_free_space() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Point 2m directly in front
    let scan = create_single_point_scan(2.0, 0.0);
    let robot_pose = Pose2D::identity();

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Cells between robot and endpoint should be free
    assert!(
        is_cell_free(&grid, 0.5, 0.0),
        "Cell at (0.5, 0) should be free"
    );
    assert!(
        is_cell_free(&grid, 1.0, 0.0),
        "Cell at (1.0, 0) should be free"
    );
    assert!(
        is_cell_free(&grid, 1.5, 0.0),
        "Cell at (1.5, 0) should be free"
    );

    // Endpoint should be occupied
    assert!(
        is_cell_occupied(&grid, 2.0, 0.0),
        "Endpoint at (2.0, 0) should be occupied"
    );
}

#[test]
fn test_ray_tracing_with_rotated_pose() {
    let mut grid = create_test_grid();
    let integrator = MapIntegrator::new(MapIntegratorConfig::default());

    // Point 2m directly in front
    let scan = create_single_point_scan(2.0, 0.0);

    // Robot at origin, facing +Y
    let robot_pose = Pose2D::new(0.0, 0.0, PI / 2.0);

    integrator.integrate_scan(&mut grid, &scan, &robot_pose);

    // Cells between robot and endpoint (along +Y) should be free
    assert!(
        is_cell_free(&grid, 0.0, 0.5),
        "Cell at (0, 0.5) should be free"
    );
    assert!(
        is_cell_free(&grid, 0.0, 1.0),
        "Cell at (0, 1.0) should be free"
    );
    assert!(
        is_cell_free(&grid, 0.0, 1.5),
        "Cell at (0, 1.5) should be free"
    );

    // Endpoint should be occupied
    assert!(
        is_cell_occupied(&grid, 0.0, 2.0),
        "Endpoint at (0, 2) should be occupied"
    );
}
