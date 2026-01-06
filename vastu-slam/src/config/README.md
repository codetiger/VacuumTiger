# Config Module

Unified configuration loading for VastuSLAM from YAML files.

## Module Structure

```
config/
├── mod.rs          # Module exports
├── vastu.rs        # VastuConfig - main configuration struct
├── grid.rs         # GridSection - grid dimensions and origin
├── sensor.rs       # SensorSection - robot geometry and sensors
├── slam.rs         # SlamSection - SLAM algorithm settings
├── persistence.rs  # PersistenceSection - output settings
├── defaults.rs     # Default value functions for serde
└── error.rs        # ConfigLoadError enum
```

## Configuration Hierarchy

```
VastuConfig
├── grid: GridSection
│   ├── resolution (0.025m)
│   ├── initial_width (800 cells)
│   ├── initial_height (800 cells)
│   ├── origin_mode ("center" | "corner")
│   └── auto_expand (true)
│
├── sensor: SensorSection
│   ├── lidar: LidarSettings
│   │   ├── offset_x (-0.110m)
│   │   ├── offset_y (0.0m)
│   │   ├── min_range (0.15m)
│   │   └── max_range (8.0m)
│   │
│   ├── robot: RobotSettings
│   │   └── radius (0.17m)
│   │
│   ├── cliff_sensors: CliffSensorSettings
│   │   ├── left_side [0.12, 0.10]
│   │   ├── left_front [0.15, 0.05]
│   │   ├── right_front [0.15, -0.05]
│   │   └── right_side [0.12, -0.10]
│   │
│   └── bumper: BumperSettings
│       ├── left_angle (0.5 rad)
│       └── right_angle (-0.5 rad)
│
├── slam: SlamSection
│   ├── log_odds: LogOddsConfig
│   ├── correlative: CorrelativeMatcherConfig
│   ├── loop_closure: LoopClosureConfig
│   ├── pose_graph: PoseGraphConfig
│   └── background_optimizer: BackgroundOptimizerConfig
│
└── persistence: PersistenceSection
    ├── output_format ("both")
    ├── output_dir ("./output")
    └── auto_save_interval (30s)
```

## Usage Examples

### Loading Configuration

```rust
use vastu_slam::config::VastuConfig;
use std::path::Path;

// Load from specific file
let config = VastuConfig::load(Path::new("config.yaml"))?;

// Load from default path (configs/config.yaml)
let config = VastuConfig::load_default()?;

// Parse from YAML string
let yaml = r#"
grid:
  resolution: 0.025
  initial_width: 1000
sensor:
  robot:
    radius: 0.17
"#;
let config = VastuConfig::from_yaml(yaml)?;

// Use defaults (no file needed)
let config = VastuConfig::default();
```

### Accessing Settings

```rust
let config = VastuConfig::default();

// Direct access to sections
let resolution = config.grid.resolution;
let robot_radius = config.sensor.robot.radius;
let lidar_range = config.sensor.lidar.max_range;

// Convenient methods
let radius = config.robot_radius();

// Convert to runtime configs
let map_config = config.to_map_config();
let matcher_config = config.correlative_matcher_config();
let loop_config = config.loop_closure_config();
```

## YAML Configuration File

### Complete Example

```yaml
# config.yaml - VastuSLAM Configuration

# Grid settings
grid:
  resolution: 0.025          # Cell size in meters (2.5cm)
  initial_width: 800         # Initial grid width in cells (20m)
  initial_height: 800        # Initial grid height in cells (20m)
  origin_mode: center        # "center" or "corner"
  auto_expand: true          # Grow grid as needed

# Sensor settings
sensor:
  lidar:
    offset_x: -0.110         # LiDAR offset from robot center (m)
    offset_y: 0.0            # LiDAR lateral offset (m)
    min_range: 0.15          # Minimum valid range (m)
    max_range: 8.0           # Maximum valid range (m)

  robot:
    radius: 0.17             # Robot radius for collision checking (m)

  cliff_sensors:             # Positions in robot frame [x, y]
    left_side: [0.12, 0.10]
    left_front: [0.15, 0.05]
    right_front: [0.15, -0.05]
    right_side: [0.12, -0.10]

  bumper:
    left_angle: 0.5          # Left bumper angle from forward (rad)
    right_angle: -0.5        # Right bumper angle from forward (rad)

# SLAM settings (Cartographer-style)
slam:
  log_odds:
    l_hit: 20                # Log-odds for obstacle hit
    l_miss: -4               # Log-odds for free space
    l_occupied_threshold: 50 # Wall classification threshold
    l_free_threshold: -50    # Floor classification threshold

  correlative:
    enabled: true
    search_x: 0.30           # X search window (m)
    search_y: 0.30           # Y search window (m)
    search_theta: 0.15       # Theta search window (rad)
    linear_resolution: 0.02  # Search step (m)
    angular_resolution: 0.02 # Search step (rad)
    multi_resolution: true
    use_gaussian_scoring: true
    gaussian_sigma: 0.10

  loop_closure:
    enabled: true
    min_score_threshold: 0.6
    min_distance_threshold: 5.0
    descriptor_threshold: 100

  pose_graph:
    max_iterations: 100
    convergence_threshold: 1e-5
    odometry_weight: 10.0
    loop_weight: 50.0

  background_optimizer:
    max_iterations: 50
    use_background_thread: false

# Output settings
persistence:
  output_format: both        # "vastu", "pgm", or "both"
  output_dir: ./output
  auto_save_interval: 30     # Seconds (0 = disabled)
```

### Minimal Example

```yaml
# minimal.yaml - Only override what you need
grid:
  resolution: 0.05  # 5cm cells for faster processing

sensor:
  robot:
    radius: 0.20    # Larger robot
```

All missing values use sensible defaults.

## Configuration Sections

### GridSection

Controls the occupancy grid dimensions:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `resolution` | 0.025 | Cell size in meters (2.5cm) |
| `initial_width` | 800 | Grid width in cells (20m) |
| `initial_height` | 800 | Grid height in cells (20m) |
| `origin_mode` | "center" | Grid origin placement |
| `auto_expand` | true | Grow grid as needed |

**Origin Modes:**
- `"center"`: Robot starts at grid center
- `"corner"`: Robot starts at grid corner (origin_x, origin_y)

### SensorSection

Robot geometry and sensor positions:

| Subsection | Parameter | Default | Description |
|------------|-----------|---------|-------------|
| lidar | offset_x | -0.110 | LiDAR X offset from center (m) |
| lidar | offset_y | 0.0 | LiDAR Y offset from center (m) |
| lidar | min_range | 0.15 | Minimum valid range (m) |
| lidar | max_range | 8.0 | Maximum valid range (m) |
| robot | radius | 0.17 | Robot radius (m) |

**CRL-200S Default Positions:**
```
           Front
      LF ─────── RF
       ╲    ↑    ╱
        ╲   │   ╱
    LS ──┬──┼──┬── RS     Cliff sensors
         │     │
      L ─┴─────┴─ R       Bumpers (left: +28°, right: -28°)
           │
         LIDAR            (-11cm from center)
```

### SlamSection

SLAM algorithm parameters:

| Subsection | Description |
|------------|-------------|
| `log_odds` | Bayesian occupancy update parameters |
| `correlative` | Scan matcher search window and resolution |
| `loop_closure` | Loop detection thresholds |
| `pose_graph` | Graph optimization settings |
| `background_optimizer` | Async optimization settings |

### PersistenceSection

Output settings:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_format` | "both" | "vastu", "pgm", or "both" |
| `output_dir` | "./output" | Output directory path |
| `auto_save_interval` | 30 | Auto-save interval in seconds |

## Conversion to Runtime Configs

The YAML configuration is converted to typed runtime configs:

```rust
let config = VastuConfig::load(path)?;

// For OccupancyGridMap
let map_config: MapConfig = config.to_map_config();

// For CorrelativeMatcher
let matcher: CorrelativeMatcherConfig = config.correlative_matcher_config();

// For LoopClosureDetector
let loop_config: LoopClosureConfig = config.loop_closure_config();

// For PoseGraph
let graph_config: PoseGraphConfig = config.pose_graph_config();

// For BackgroundOptimizer
let opt_config: BackgroundOptimizerConfig = config.background_optimizer_config();
```

## Error Handling

```rust
use vastu_slam::config::{VastuConfig, ConfigLoadError};

match VastuConfig::load(path) {
    Ok(config) => { /* use config */ }
    Err(ConfigLoadError::Io(msg)) => {
        eprintln!("Failed to read file: {}", msg);
    }
    Err(ConfigLoadError::Parse(msg)) => {
        eprintln!("Invalid YAML: {}", msg);
    }
}
```

## Thread Safety

`VastuConfig` and all section types are `Clone + Send + Sync`.
Configuration is typically loaded once at startup and shared immutably.
