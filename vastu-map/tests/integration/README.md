# Integration Tests

YAML-based integration testing framework for vastu-map SLAM algorithms using sangam-io's mock device simulator.

## Quick Start

```bash
# Run all integration tests
cargo test --features integration-tests -- --nocapture

# Run a specific scenario
cargo test --features integration-tests -- test_straight_line --nocapture

# Run all YAML scenarios with summary
cargo test --features integration-tests -- test_yaml_scenarios --nocapture
```

## Architecture

```
tests/integration/
├── mod.rs              # Module exports
├── harness.rs          # Core test harness (physics + SLAM simulation)
├── metrics.rs          # Validation metrics (position/orientation error, wall accuracy)
├── visualization.rs    # SVG trajectory visualization
├── yaml_config.rs      # YAML scenario configuration structs
├── yaml_runner.rs      # Scenario loader and executor
├── path_generator.rs   # PathSegment for motion commands
├── adapters.rs         # Type conversions (sangam-io ↔ vastu-map)
└── scenarios/
    ├── yaml_tests.rs   # Test entry points
    ├── maps/           # Map definitions (PGM + YAML)
    └── *.yaml          # Test scenarios
```

## YAML Scenario Format

Each scenario defines a robot path through a simulated environment:

```yaml
name: "my_scenario"
map: "maps/medium_room.yaml"
start_pose:
  x: 4.0
  y: 4.0
  theta: 0.0  # radians, 0 = facing right (+X)
path:
  - {left: 1.0, right: 1.0}              # Forward 1m
  - {left: -0.183, right: 0.183}         # Turn left 90°
  - {left: 0.5, right: 0.5, speed: 0.3}  # Forward 0.5m at 0.3 m/s
  - {stop: 2.0}                          # Stationary for 2 seconds
```

### Wheel Commands

Uses differential drive kinematics (wheel_base = 0.233m):

| Motion | Left Wheel | Right Wheel | Notes |
|--------|------------|-------------|-------|
| Forward 1m | 1.0 | 1.0 | Equal distances |
| Backward 1m | -1.0 | -1.0 | Negative = reverse |
| Turn left 90° | -0.183 | 0.183 | θ × wheel_base / 2 |
| Turn right 90° | 0.183 | -0.183 | Opposite signs |
| Arc left | 0.5 | 0.7 | Outer wheel travels more |

### Common Turn Values

| Angle | Per-wheel distance |
|-------|-------------------|
| 90° (π/2) | 0.183m |
| 180° (π) | 0.366m |
| 360° (2π) | 0.732m |

## Adding New Scenarios

1. Create a YAML file in `scenarios/`:

```yaml
name: "my_new_test"
map: "maps/medium_room.yaml"
start_pose:
  x: 2.0
  y: 4.0
  theta: 0.0
path:
  - {left: 1.0, right: 1.0}
```

2. Add a test function in `scenarios/yaml_tests.rs`:

```rust
#[test]
fn test_my_new_test() {
    run_single_scenario("my_new_test");
}
```

3. Run the test:

```bash
cargo test --features integration-tests -- test_my_new_test --nocapture
```

## Available Scenarios

### Basic Tests
- `static_scan_test` - Stationary observation (5 seconds)
- `straight_line_test` - Pure forward motion
- `pure_rotation_test` - Rotation only, no translation
- `simple_room_straight_line` - Simple room with straight motion

### Complex Scenarios
- `loop_closure_test` - 2m square path, returns to start
- `medium_room_perimeter` - Full room perimeter
- `medium_room_exploration` - Multi-room exploration
- `medium_room_obstacles` - Navigation around obstacles

### Stress Tests
- `icp_sparse_corridor` - Narrow corridor (sparse features)
- `drift_continuous_curve` - Curved path (odometry drift)
- `drift_long_path` - Extended path (cumulative error)

## Test Output

### Metrics

Each test reports:
- **Position error**: Euclidean distance from ground truth (meters)
- **Orientation error**: Angular difference from ground truth (degrees)
- **Wall accuracy**: Percentage of detected lines near actual walls
- **ICP stats**: Average/max iterations, convergence failures

### Visualization

SVG files are generated in `results/`:

```bash
open results/straight_line_test.svg
```

Shows:
- Ground truth trajectory (blue)
- SLAM estimated trajectory (red → yellow → green by confidence)
- Detected walls (teal)
- Map walls (gray background)

## Test Harness

The `TestHarness` provides:

1. **Physics simulation** via sangam-io's mock device
2. **Lidar ray-casting** at 5Hz
3. **SLAM pipeline** execution
4. **Ground truth comparison**

```rust
let config = HarnessConfig {
    map_file: "path/to/map.yaml".into(),
    start_x: 2.5,
    start_y: 2.5,
    start_theta: 0.0,
    ..Default::default()
};

let mut harness = TestHarness::new(config)?
    .with_visualization("output.svg");

let result = harness.run_path(&path_segments);
println!("{}", result.metrics.summary());
```

## Maps

Maps are defined as PGM images with YAML metadata:

```
scenarios/maps/
├── simple_room.yaml    # 5m × 5m basic room
├── medium_room.yaml    # 8m × 8m with obstacles
├── large_room.yaml     # Expansive space
└── corridor.yaml       # Narrow corridor
```

Each map YAML references a PGM file and defines resolution/origin.
