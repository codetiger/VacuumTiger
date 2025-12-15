# Mock Device User Guide

This guide explains how to use SangamIO's mock device driver for simulation-based development and testing.

## Overview

The mock device driver provides a complete simulation of CRL-200S hardware, enabling:

- **Algorithm development** without physical robots
- **Unit testing** with deterministic sensor data
- **CI/CD integration** for automated testing
- **Accelerated testing** with speed factor

## Quick Start

### 1. Build with Mock Feature

```bash
cd sangam-io
cargo build --release --features mock
```

### 2. Create or Use an Existing Map

Use the example map or create your own:

```bash
ls maps/
# example.yaml  example.pgm
```

### 3. Run Simulation

```bash
# Using default mock.toml
cargo run --release --features mock -- mock.toml

# Or with explicit config flag
cargo run --release --features mock -- --config mock.toml

# With debug logging
RUST_LOG=debug cargo run --release --features mock -- mock.toml
```

### 4. Connect Your Client

Connect to `localhost:5555` using the same protocol as real hardware:
- **UDP** for sensor data (automatic when TCP connects)
- **TCP** for commands

## Map Creation

### PGM + YAML Format

Maps use the ROS-standard format for maximum compatibility.

**Step 1: Create PGM Image**

Use any image editor (GIMP, Photoshop, or programmatically):

| Color | Value | Meaning |
|-------|-------|---------|
| White | 255 | Free space |
| Black | 0 | Wall/obstacle |
| Gray | 205 | Unknown |

Save as 8-bit grayscale PGM (P5 binary or P2 ASCII).

**Step 2: Create YAML Metadata**

```yaml
# maps/my_map.yaml
image: my_map.pgm           # PGM filename (relative to YAML)
resolution: 0.02            # meters per pixel
origin: [-5.0, -5.0, 0.0]   # [x, y, yaw] of bottom-left pixel
occupied_thresh: 0.65       # Pixels darker than this are occupied

# Optional: cliff mask (same dimensions as main map)
cliff_mask: my_map_cliffs.pgm
```

**Step 3: Calculate Origin**

The origin is the world coordinate of the bottom-left pixel:

```
origin_x = -width_pixels * resolution / 2
origin_y = -height_pixels * resolution / 2
```

For a 500x500 pixel map at 0.02 m/px:
```yaml
origin: [-5.0, -5.0, 0.0]  # Centers the map
```

### Using ROS Maps

Maps saved by ROS tools work directly:

```bash
# From ROS map_server
ros2 run nav2_map_server map_saver_cli -f my_map

# Produces my_map.pgm and my_map.yaml
# Copy to sangam-io/maps/
```

### Creating Cliff Masks

Cliff masks define areas where cliff sensors should trigger:

1. Create a PGM with same dimensions as main map
2. Paint black (0) where cliffs exist
3. Leave white (255) for safe floor
4. Reference in YAML: `cliff_mask: my_map_cliffs.pgm`

## Configuration Reference

### Minimal Configuration

```toml
[device]
type = "mock"
name = "Mock Robot"

[device.simulation]
map_file = "maps/example.yaml"

[network]
bind_address = "0.0.0.0:5555"
```

### Full Configuration

See `mock.toml` for all available options. Key sections:

#### Simulation Control

```toml
[device.simulation]
map_file = "maps/example.yaml"
start_x = 1.5            # Initial X position (meters)
start_y = 3.5            # Initial Y position (meters)
start_theta = 0.0        # Initial heading (radians, 0 = +X)
speed_factor = 1.0       # Time multiplier (2.0 = 2x speed)
random_seed = 42         # 0 = random, >0 = deterministic
log_level = "standard"   # "minimal", "standard", "verbose"
```

#### Robot Parameters

```toml
[device.simulation.robot]
wheel_base = 0.233       # Distance between wheels (meters)
ticks_per_meter = 4464.0 # Encoder resolution
max_linear_speed = 0.3   # Speed limit (m/s)
max_angular_speed = 1.0  # Turn rate limit (rad/s)
robot_radius = 0.17      # Collision radius (meters)
collision_mode = "stop"  # "stop", "slide", "passthrough"
```

#### Lidar Simulation

```toml
[device.simulation.lidar]
num_rays = 360           # Points per scan
scan_rate_hz = 5.0       # Scans per second
min_range = 0.15         # Minimum detection (meters)
max_range = 8.0          # Maximum detection (meters)
mounting_x = -0.110      # Lidar offset from center
angle_offset = 0.2182    # Angular offset (radians)

[device.simulation.lidar.noise]
range_stddev = 0.005     # Distance noise (meters)
angle_stddev = 0.001     # Angular noise (radians)
miss_rate = 0.01         # Invalid reading probability
quality_base = 200       # Base quality (0-255)
```

#### IMU Simulation

```toml
[device.simulation.imu.gyro_noise]
stddev = [5.0, 5.0, 10.0]  # Per-axis noise
bias = [0.0, 0.0, 0.0]     # Constant bias
drift_rate = 0.0           # Bias drift (units/sec)

[device.simulation.imu.accel_noise]
stddev = [10.0, 10.0, 20.0]
bias = [0.0, 0.0, 0.0]
```

#### Encoder Simulation

```toml
[device.simulation.encoder.noise]
slip_stddev = 0.002      # Wheel slip noise (0-1)
slip_bias = 0.0          # Systematic slip
quantization_noise = true # ±0.5 tick jitter
```

## Speed Factor Usage

The `speed_factor` setting accelerates simulation time:

| Factor | sensor_status | lidar | Use Case |
|--------|---------------|-------|----------|
| 1.0 | 110 Hz | 5 Hz | Real-time development |
| 2.0 | 220 Hz | 10 Hz | Faster iteration |
| 5.0 | 550 Hz | 25 Hz | Quick integration tests |
| 10.0 | 1100 Hz | 50 Hz | Stress testing |

**Note:** Higher speeds increase CPU usage and may not be sustainable on all systems.

## Deterministic Testing

For reproducible tests, set a fixed random seed:

```toml
[device.simulation]
random_seed = 12345  # Same seed = same sensor noise sequence
```

This ensures:
- Identical lidar noise patterns
- Same encoder slip variations
- Reproducible IMU readings

## Collision Modes

Three collision handling modes are available:

| Mode | Behavior | Use Case |
|------|----------|----------|
| `stop` | Robot stops on collision | Realistic behavior |
| `slide` | Robot slides along walls | Wall-following tests |
| `passthrough` | No collision | Debugging, path planning |

## Troubleshooting

### "Map file not found"

```
Error: Failed to read map YAML: No such file or directory
```

Check that `map_file` path is relative to where you run the command, or use absolute path.

### "Mock device not available"

```
Error: Mock device not available: rebuild with --features mock
```

The mock feature is not enabled. Rebuild:
```bash
cargo build --release --features mock
```

### High CPU Usage

If CPU usage is high:
1. Reduce `speed_factor`
2. Check `log_level` (verbose logging is expensive)
3. Ensure release build: `--release`

### No Lidar Data

Lidar must be explicitly enabled:
```python
# Send lidar enable command via TCP
cmd = ComponentControl(id="lidar", action=Enable())
```

### Client Not Receiving UDP

1. Ensure TCP connection is established first
2. Check firewall allows UDP on port 5555
3. Verify client is binding to correct port

## Integration with SLAM

The mock device produces identical sensor data format to real hardware:

```python
# Same code works for both real and simulated robot
client = SangamClient(host="localhost", port=5555)
# or
client = SangamClient(host="192.168.68.101", port=5555)
```

## Example Workflow

1. **Develop algorithm** using mock device on laptop
2. **Write unit tests** with deterministic random seed
3. **Run CI tests** with mock device (no hardware needed)
4. **Deploy to robot** - same client code works
5. **Record real data** using bag files
6. **Replay and compare** mock vs real performance
