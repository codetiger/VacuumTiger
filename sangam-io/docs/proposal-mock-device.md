# Proposal: Mock Device Driver for SangamIO

## Overview

This proposal outlines the implementation of a **mock device driver** for SangamIO that simulates CRL-200S robot hardware using a pre-recorded map file. This enables algorithm development, testing, and debugging without physical robot hardware.

## Goals

1. Enable SLAM/navigation algorithm testing in simulation
2. Use **ROS-standard PGM + YAML map format** for interoperability
3. Match CRL-200S physical characteristics exactly
4. Provide configurable noise for realistic sensor simulation
5. Support deterministic/repeatable testing scenarios
6. Full CRL-200S sensor parity from initial release

## Requirements Summary

Based on clarification discussions, the following decisions were made:

| Topic | Decision |
|-------|----------|
| **Collision handling** | Stop or slide based on config; slide friction configurable |
| **Noise parameters** | Start with datasheet estimates; calibrate later based on SLAM performance |
| **Actuator commands** | Log state changes only (no simulation effect) |
| **Map source** | Manual creation (GIMP) or external sources; no DhruvaSLAM export initially |
| **Cliff sensors** | Separate mask file (`map_cliffs.pgm`) |
| **Time scaling** | Configurable speed factor (1x, 2x, 5x, 10x) |
| **Bumper geometry** | Configurable trigger zones (angles + distances) |
| **Logging level** | Standard: log velocity commands, collisions, sensor triggers |
| **Button simulation** | Fixed state (never pressed) |
| **Fast mode behavior** | Increase packet rate (220Hz at 2x, 550Hz at 5x) |
| **Time source** | Wall-clock multiplied by speed_factor |
| **MVP scope** | Full CRL-200S parity (all sensors from day one) |

## Map File Format (ROS Standard)

### PGM + YAML Format

Maps use the ROS-standard two-file format:

**YAML Metadata File** (`map.yaml`):
```yaml
image: map.pgm
resolution: 0.02          # meters per pixel (matches dhruva-slam default)
origin: [-10.0, -10.0, 0.0]  # [x, y, yaw] of bottom-left pixel in world frame
occupied_thresh: 0.65     # Pixel value > 65% = occupied
free_thresh: 0.196        # Pixel value < 19.6% = free
negate: 0                 # 0 = white is free, 1 = white is occupied

# Optional: cliff mask file (same dimensions as main map)
cliff_mask: map_cliffs.pgm  # White = safe, Black = cliff
```

**PGM Image File** (`map.pgm`):
- Grayscale image (8-bit, 0-255)
- **White (255)** = free space
- **Black (0)** = occupied
- **Gray (205)** = unknown

**Cliff Mask File** (`map_cliffs.pgm`) - Optional:
- Grayscale image (same dimensions as main map)
- **White (255)** = safe floor
- **Black (0)** = cliff/drop-off
- If not specified, cliffs only trigger at map boundary

### Map Compatibility

This format is compatible with:
- ROS `map_server` / Nav2 `map_server`
- Cartographer
- SLAM Toolbox
- gmapping
- Any standard image editor (GIMP, Photoshop)

## Configuration

### Mock Configuration File (`mock.toml`)

This configuration mirrors CRL-200S as closely as possible:

```toml
# Mock Device Configuration for SangamIO
# Simulates CRL-200S hardware for algorithm testing

[device]
type = "mock"
name = "Mock CRL-200S"

# ============================================================================
# Simulation Environment
# ============================================================================

[device.simulation]
# Map file (ROS-standard PGM+YAML format)
# Point to the .yaml file, PGM is referenced inside
map_file = "maps/apartment.yaml"

# Initial robot pose in world coordinates (ROS REP-103)
# X = forward, Y = left, Theta = CCW positive
start_x = 0.0          # meters
start_y = 0.0          # meters
start_theta = 0.0      # radians (0 = facing +X direction)

# Simulation speed multiplier (1.0 = real-time)
# Higher values increase packet rate proportionally:
#   1.0 = 110Hz sensor_status, 5Hz lidar
#   2.0 = 220Hz sensor_status, 10Hz lidar
#   5.0 = 550Hz sensor_status, 25Hz lidar
#  10.0 = 1100Hz sensor_status, 50Hz lidar
speed_factor = 1.0

# Random seed for reproducible noise (0 = random seed each run)
random_seed = 0

# Logging level: "minimal", "standard", "verbose"
# - minimal: errors and warnings only
# - standard: velocity commands, collisions, sensor triggers
# - verbose: pose and sensor values at configurable interval
log_level = "standard"

# ============================================================================
# Robot Physical Parameters (CRL-200S Specifications)
# ============================================================================

[device.simulation.robot]
# Differential drive parameters
wheel_base = 0.233           # meters (distance between wheel centers)
wheel_radius = 0.033         # meters
ticks_per_meter = 4464.0     # encoder ticks per meter of travel

# Velocity limits
max_linear_speed = 0.3       # m/s
max_angular_speed = 1.0      # rad/s

# Robot physical dimensions (for collision detection)
robot_radius = 0.17          # meters (approximate circular footprint)

# Collision behavior: "stop", "slide", "passthrough"
# - stop: robot velocity becomes zero on collision
# - slide: robot slides along obstacle surface
# - passthrough: ignore collisions (for debugging)
collision_mode = "stop"

# Slide friction coefficient (only used when collision_mode = "slide")
# 0.0 = frictionless sliding, 1.0 = high friction (slow slide)
slide_friction = 0.3

# ============================================================================
# Lidar Configuration (Delta-2D Specifications)
# ============================================================================

[device.simulation.lidar]
# Scan parameters
num_rays = 360               # points per 360° scan
scan_rate_hz = 5.0           # scans per second (Delta-2D native rate)

# Range limits
min_range = 0.15             # meters (matches dhruva-slam preprocessing)
max_range = 8.0              # meters (Delta-2D max range)

# Lidar mounting position (relative to robot center)
# CRL-200S: lidar is mounted behind center
mounting_x = -0.110          # meters (behind robot center)
mounting_y = 0.0             # meters (centered on axis)
optical_offset = 0.025       # meters (radial offset to laser origin)
angle_offset = 0.2182        # radians (~12.5°, lidar's 0° vs robot forward)

# Noise configuration (initial estimates - calibrate based on SLAM performance)
[device.simulation.lidar.noise]
# Distance noise (Gaussian)
range_stddev = 0.005         # meters (5mm standard deviation)
range_bias = 0.0             # meters (systematic offset)

# Angular noise (Gaussian)
angle_stddev = 0.001         # radians (~0.06°)

# Quality simulation
quality_base = 200           # base quality value (0-255)
quality_distance_decay = 10  # quality reduction per meter distance

# Miss rate (probability of invalid reading)
miss_rate = 0.01             # 1% chance of invalid point

# ============================================================================
# IMU Configuration (GD32 Specifications)
# ============================================================================

[device.simulation.imu]
update_rate_hz = 110.0       # matches GD32 status packet rate

# Gyroscope noise (raw i16 units, same as hardware)
# Initial estimates based on typical MPU6050 - calibrate later
[device.simulation.imu.gyro_noise]
stddev = [5, 5, 10]          # [x, y, z] standard deviation
bias = [0, 0, 0]             # [x, y, z] constant bias
drift_rate = 0.0             # bias drift per second

# Accelerometer noise (raw i16 units)
[device.simulation.imu.accel_noise]
stddev = [10, 10, 20]        # [x, y, z] standard deviation
bias = [0, 0, 0]             # [x, y, z] constant bias

# Tilt sensor noise (low-pass filtered gravity vector)
[device.simulation.imu.tilt_noise]
stddev = [5, 5, 5]           # [x, y, z] standard deviation

# ============================================================================
# Encoder Configuration
# ============================================================================

[device.simulation.encoder]
update_rate_hz = 110.0       # matches GD32 status packet rate

# Encoder noise (initial estimates - calibrate later)
[device.simulation.encoder.noise]
# Slip simulation (multiplicative noise on tick count)
slip_stddev = 0.002          # 0.2% standard deviation
slip_bias = 0.0              # systematic over/under counting

# Quantization (encoder resolution)
quantization_noise = true    # add ±0.5 tick quantization noise

# ============================================================================
# Bumper Configuration (Configurable Zones)
# ============================================================================

[device.simulation.bumpers]
# Bumper trigger distance from robot edge
trigger_distance = 0.01      # meters (1cm)

# Left bumper arc (angles relative to robot forward, CCW positive)
[device.simulation.bumpers.left]
start_angle = 0.3            # radians (~17°)
end_angle = 1.57             # radians (90°, robot left side)

# Right bumper arc
[device.simulation.bumpers.right]
start_angle = -1.57          # radians (-90°, robot right side)
end_angle = -0.3             # radians (~-17°)

# ============================================================================
# Cliff Sensor Configuration
# ============================================================================

[device.simulation.cliffs]
# Cliff detection sources:
# - Map boundary (always active)
# - Cliff mask file (if specified in map YAML)
enabled = true

# Cliff sensor positions (relative to robot center)
# CRL-200S has 4 cliff sensors
[[device.simulation.cliffs.sensors]]
name = "left_side"
x = 0.12                     # meters forward from center
y = 0.10                     # meters left from center

[[device.simulation.cliffs.sensors]]
name = "left_front"
x = 0.15
y = 0.05

[[device.simulation.cliffs.sensors]]
name = "right_front"
x = 0.15
y = -0.05

[[device.simulation.cliffs.sensors]]
name = "right_side"
x = 0.12
y = -0.10

# ============================================================================
# Binary Sensors Configuration
# ============================================================================

[device.simulation.sensors]
# Battery simulation (fixed values)
[device.simulation.sensors.battery]
voltage = 14.8               # volts (between 13.5-15.5V range)
level = 85                   # percentage (0-100)
is_charging = false

# Fixed sensor states
dustbox_attached = true
dock_connected = false

# Buttons - always fixed state (never pressed)
start_button = 0
dock_button = 0

# ============================================================================
# Actuator Logging (no simulation effect)
# ============================================================================

[device.simulation.actuators]
# When actuator commands are received, log state changes
# Supported actuators: vacuum, main_brush, side_brush, water_pump, led, lidar
log_state_changes = true

# ============================================================================
# Coordinate Frame Transforms (matches CRL-200S sangamio.toml)
# ============================================================================

[device.hardware.frame_transforms]

# Lidar angle transform: output = scale * input + offset
# CRL-200S: mounted backward (offset=π), clockwise angles (scale=-1)
[device.hardware.frame_transforms.lidar]
scale = -1.0
offset = 3.14159265

# IMU gyroscope axis remap: [source_axis, sign]
# CRL-200S: x=yaw, y=pitch, z=roll -> remap to ROS standard
[device.hardware.frame_transforms.imu_gyro]
x = [2, 1]    # output_x (roll) = input_z * 1
y = [1, 1]    # output_y (pitch) = input_y * 1
z = [0, -1]   # output_z (yaw) = input_x * -1

# IMU accelerometer axis remap (identity for CRL-200S)
[device.hardware.frame_transforms.imu_accel]
x = [0, 1]
y = [1, 1]
z = [2, 1]

# ============================================================================
# Network Configuration
# ============================================================================

[network]
bind_address = "0.0.0.0:5555"
udp_streaming_port = 5556
```

## Architecture

### Module Structure

```
sangam-io/src/devices/
├── mod.rs                  # Add "mock" to device factory
└── mock/
    ├── mod.rs              # MockDriver struct + DeviceDriver impl
    ├── config.rs           # SimulationConfig deserialization
    ├── map_loader.rs       # PGM+YAML parser + cliff mask
    ├── physics.rs          # Differential drive kinematics + collision
    ├── noise.rs            # Configurable noise generator
    ├── lidar_sim.rs        # Ray-casting lidar simulation
    ├── imu_sim.rs          # IMU data generation with noise
    ├── encoder_sim.rs      # Encoder simulation with slip/noise
    └── sensor_sim.rs       # Bumpers, cliffs, battery (binary sensors)
```

### Component Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                          MockDriver                                  │
├─────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────────┐│
│  │                    Configuration (TOML)                         ││
│  │  • Robot params (wheel_base, ticks_per_meter, limits)          ││
│  │  • Sensor noise configs (stddev, bias, drift)                  ││
│  │  • Map file path + cliff mask + initial pose                   ││
│  │  • Bumper zones, cliff sensor positions                        ││
│  │  • Speed factor, collision mode, logging level                 ││
│  └─────────────────────────────────────────────────────────────────┘│
│                              │                                       │
│                              ▼                                       │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────────────────┐ │
│  │  MapLoader   │   │ PhysicsEngine│   │   Sensor Simulators      │ │
│  │  (PGM+YAML)  │   │              │   │                          │ │
│  │              │   │ • pose       │   │ • LidarSim (ray-cast)    │ │
│  │ • load()     │──▶│ • velocity   │──▶│ • ImuSim (gyro/accel)    │ │
│  │ • ray_cast() │   │ • collision  │   │ • EncoderSim (ticks)     │ │
│  │ • is_cliff() │   │ • slide      │   │ • BumperSim (zones)      │ │
│  │ • is_free()  │   │ • update()   │   │ • CliffSim (mask)        │ │
│  └──────────────┘   └──────────────┘   └──────────────────────────┘ │
│         │                  │                     │                   │
│         ▼                  ▼                     ▼                   │
│  ┌─────────────────────────────────────────────────────────────────┐│
│  │         Sensor Groups (identical to CRL-200S output)            ││
│  │                                                                  ││
│  │  sensor_status (~110Hz × speed_factor):                         ││
│  │  • wheel_left, wheel_right (U16 encoder ticks)                  ││
│  │  • gyro_x/y/z, accel_x/y/z, tilt_x/y/z (I16 raw)              ││
│  │  • bumper_left, bumper_right (Bool)                             ││
│  │  • cliff_left_side/front, cliff_right_front/side (Bool)        ││
│  │  • battery_voltage (F32), battery_level (U8), is_charging (Bool)││
│  │  • dustbox_attached, is_dock_connected (Bool)                   ││
│  │  • start_button, dock_button (U16) - always 0                   ││
│  │                                                                  ││
│  │  lidar (5Hz × speed_factor):                                    ││
│  │  • scan: PointCloud2D [(angle_rad, distance_m, quality)]        ││
│  │                                                                  ││
│  │  device_version (one-time):                                     ││
│  │  • version_string: "MockDevice v1.0"                            ││
│  │  • version_code: 100                                            ││
│  └─────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────┘
```

### Simulation Loop

```
┌─────────────────────────────────────────────────────────────────────┐
│            Simulation Thread (~110Hz × speed_factor)                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  Time source: wall_clock × speed_factor                             │
│  At 2x speed: loop runs every ~4.5ms instead of ~9ms                │
│                                                                      │
│  Every tick (110Hz × speed_factor):                                 │
│                                                                      │
│  1. Compute dt = (wall_time_delta × speed_factor)                   │
│                                                                      │
│  2. Read velocity command (from atomic state, set by TCP)           │
│     └─ linear_vel (m/s), angular_vel (rad/s)                        │
│                                                                      │
│  3. Update physics (differential drive kinematics)                  │
│     ├─ Compute wheel velocities from (v, ω)                         │
│     ├─ Integrate pose: x, y, theta                                  │
│     ├─ Check collision against map                                  │
│     │   ├─ "stop": zero velocity, log collision                     │
│     │   ├─ "slide": project velocity along surface                  │
│     │   └─ "passthrough": ignore collision                          │
│     └─ Update encoder tick counters (with slip noise)               │
│                                                                      │
│  4. Generate sensor_status data:                                    │
│     ├─ Encoders: wheel_left, wheel_right (U16 + noise)             │
│     ├─ IMU: gyro_x/y/z (from ω + noise)                            │
│     │       accel_x/y/z (gravity + centripetal + noise)            │
│     │       tilt_x/y/z (gravity vector + noise)                    │
│     ├─ Bumpers: check configurable arc zones against map           │
│     ├─ Cliffs: check sensor positions against cliff mask           │
│     ├─ Battery: constant from config                                │
│     └─ Buttons: always 0                                            │
│                                                                      │
│  5. Publish to streaming channel                                    │
│                                                                      │
│  6. Log state changes (if log_level >= "standard"):                 │
│     ├─ Velocity commands received                                   │
│     ├─ Collision events                                             │
│     ├─ Bumper/cliff triggers                                        │
│     └─ Actuator state changes                                       │
│                                                                      │
│  Every (200ms / speed_factor) - lidar tick:                         │
│                                                                      │
│  7. Generate lidar scan:                                            │
│     ├─ For each of 360 rays:                                        │
│     │   ├─ Transform ray origin by lidar mount offset               │
│     │   ├─ Ray-cast against occupancy grid                          │
│     │   ├─ Add range noise (Gaussian)                               │
│     │   ├─ Apply miss rate (random invalid points)                  │
│     │   └─ Compute quality from distance                            │
│     └─ Apply frame transform (scale, offset)                        │
│                                                                      │
│  8. Update lidar sensor group                                       │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

## Implementation Details

### 1. Map Loader with Cliff Mask (`map_loader.rs`)

```rust
use std::path::Path;
use image::GrayImage;

/// Parsed map YAML metadata
#[derive(Debug, Deserialize)]
pub struct MapMetadata {
    pub image: String,
    pub resolution: f32,
    pub origin: [f32; 3],
    pub occupied_thresh: f32,
    pub free_thresh: f32,
    #[serde(default)]
    pub negate: u8,
    #[serde(default)]
    pub cliff_mask: Option<String>,  // Optional cliff mask file
}

/// Simulation map loaded from PGM+YAML
pub struct SimulationMap {
    pixels: GrayImage,
    cliff_mask: Option<GrayImage>,  // Optional cliff layer
    resolution: f32,
    origin: (f32, f32),
    occupied_thresh: u8,
    free_thresh: u8,
}

impl SimulationMap {
    pub fn load<P: AsRef<Path>>(yaml_path: P) -> Result<Self> {
        let yaml_content = std::fs::read_to_string(&yaml_path)?;
        let metadata: MapMetadata = serde_yaml::from_str(&yaml_content)?;

        let yaml_dir = yaml_path.as_ref().parent().unwrap_or(Path::new("."));
        let pgm_path = yaml_dir.join(&metadata.image);
        let img = image::open(&pgm_path)?.into_luma8();

        // Load optional cliff mask
        let cliff_mask = if let Some(cliff_file) = &metadata.cliff_mask {
            let cliff_path = yaml_dir.join(cliff_file);
            Some(image::open(&cliff_path)?.into_luma8())
        } else {
            None
        };

        let occupied_thresh = (metadata.occupied_thresh * 255.0) as u8;
        let free_thresh = ((1.0 - metadata.free_thresh) * 255.0) as u8;

        Ok(Self {
            pixels: img,
            cliff_mask,
            resolution: metadata.resolution,
            origin: (metadata.origin[0], metadata.origin[1]),
            occupied_thresh,
            free_thresh,
        })
    }

    /// Check if position is a cliff (from mask or map boundary)
    pub fn is_cliff(&self, x: f32, y: f32) -> bool {
        match self.world_to_pixel(x, y) {
            Some((px, py)) => {
                if let Some(mask) = &self.cliff_mask {
                    let pixel = mask.get_pixel(px, py).0[0];
                    pixel < 128  // Black = cliff
                } else {
                    false  // No mask, not a cliff
                }
            }
            None => true,  // Out of bounds = cliff
        }
    }

    // ... rest of implementation (is_occupied, is_free, ray_cast, world_to_pixel)
}
```

### 2. Physics Engine with Collision Modes (`physics.rs`)

```rust
pub enum CollisionMode {
    Stop,
    Slide { friction: f32 },
    Passthrough,
}

pub struct PhysicsState {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub linear_vel: f32,
    pub angular_vel: f32,
    pub left_ticks: u16,
    pub right_ticks: u16,
    collision_mode: CollisionMode,
    robot_radius: f32,
}

impl PhysicsState {
    pub fn update(&mut self, dt: f32, map: &SimulationMap, config: &RobotConfig) -> bool {
        // Compute new position
        let new_x = self.x + self.linear_vel * self.theta.cos() * dt;
        let new_y = self.y + self.linear_vel * self.theta.sin() * dt;
        let new_theta = self.theta + self.angular_vel * dt;

        // Check collision
        let would_collide = self.check_collision(new_x, new_y, map);

        match &self.collision_mode {
            CollisionMode::Stop => {
                if would_collide {
                    // Stop movement, log collision
                    log::info!("Collision at ({:.3}, {:.3})", new_x, new_y);
                    return true; // Collision occurred
                }
                self.x = new_x;
                self.y = new_y;
            }
            CollisionMode::Slide { friction } => {
                if would_collide {
                    // Find collision normal and slide along surface
                    let (slide_x, slide_y) = self.compute_slide_vector(
                        new_x, new_y, *friction, map
                    );
                    self.x += slide_x * dt;
                    self.y += slide_y * dt;
                    log::info!("Sliding at ({:.3}, {:.3})", self.x, self.y);
                    return true;
                }
                self.x = new_x;
                self.y = new_y;
            }
            CollisionMode::Passthrough => {
                self.x = new_x;
                self.y = new_y;
            }
        }

        self.theta = normalize_angle(new_theta);
        false // No collision
    }

    fn check_collision(&self, x: f32, y: f32, map: &SimulationMap) -> bool {
        // Check multiple points around robot circumference
        let num_checks = 8;
        for i in 0..num_checks {
            let angle = (i as f32 / num_checks as f32) * TAU;
            let check_x = x + self.robot_radius * angle.cos();
            let check_y = y + self.robot_radius * angle.sin();
            if map.is_occupied(check_x, check_y) {
                return true;
            }
        }
        false
    }

    fn compute_slide_vector(&self, x: f32, y: f32, friction: f32, map: &SimulationMap) -> (f32, f32) {
        // Simplified: find surface normal and project velocity
        // Full implementation would use gradient of distance field
        let vel_mag = self.linear_vel * (1.0 - friction);

        // Try perpendicular directions
        let perp1 = (self.theta + FRAC_PI_2).sin();
        let perp2 = (self.theta + FRAC_PI_2).cos();

        // Return the direction that doesn't collide
        if !map.is_occupied(x + perp1 * 0.01, y + perp2 * 0.01) {
            (perp1 * vel_mag, perp2 * vel_mag)
        } else {
            (-perp1 * vel_mag, -perp2 * vel_mag)
        }
    }
}
```

### 3. Bumper Simulator with Configurable Zones (`sensor_sim.rs`)

```rust
pub struct BumperZone {
    pub start_angle: f32,  // radians, relative to robot forward
    pub end_angle: f32,
}

pub struct BumperSimulator {
    left_zone: BumperZone,
    right_zone: BumperZone,
    trigger_distance: f32,
}

impl BumperSimulator {
    pub fn check(&self, map: &SimulationMap, x: f32, y: f32, theta: f32, robot_radius: f32)
        -> (bool, bool)
    {
        let left = self.check_zone(&self.left_zone, map, x, y, theta, robot_radius);
        let right = self.check_zone(&self.right_zone, map, x, y, theta, robot_radius);
        (left, right)
    }

    fn check_zone(&self, zone: &BumperZone, map: &SimulationMap,
                  x: f32, y: f32, theta: f32, robot_radius: f32) -> bool
    {
        // Check multiple rays within the bumper arc
        let num_rays = 5;
        let angle_range = zone.end_angle - zone.start_angle;

        for i in 0..num_rays {
            let local_angle = zone.start_angle + (i as f32 / (num_rays - 1) as f32) * angle_range;
            let world_angle = theta + local_angle;

            // Ray from robot edge in this direction
            let start_x = x + robot_radius * world_angle.cos();
            let start_y = y + robot_radius * world_angle.sin();

            let check_x = start_x + self.trigger_distance * world_angle.cos();
            let check_y = start_y + self.trigger_distance * world_angle.sin();

            if map.is_occupied(check_x, check_y) {
                return true;
            }
        }
        false
    }
}
```

### 4. Cliff Simulator (`sensor_sim.rs`)

```rust
pub struct CliffSensor {
    pub name: String,
    pub x: f32,  // position relative to robot center
    pub y: f32,
}

pub struct CliffSimulator {
    sensors: Vec<CliffSensor>,
}

impl CliffSimulator {
    pub fn check(&self, map: &SimulationMap, robot_x: f32, robot_y: f32, robot_theta: f32)
        -> HashMap<String, bool>
    {
        let mut results = HashMap::new();

        for sensor in &self.sensors {
            // Transform sensor position to world frame
            let world_x = robot_x
                + sensor.x * robot_theta.cos()
                - sensor.y * robot_theta.sin();
            let world_y = robot_y
                + sensor.x * robot_theta.sin()
                + sensor.y * robot_theta.cos();

            let is_cliff = map.is_cliff(world_x, world_y);
            results.insert(sensor.name.clone(), is_cliff);
        }

        results
    }
}
```

### 5. Actuator Logger (`mod.rs`)

```rust
pub struct ActuatorState {
    vacuum: bool,
    vacuum_speed: u8,
    main_brush: bool,
    main_brush_speed: u8,
    side_brush: bool,
    side_brush_speed: u8,
    water_pump: bool,
    water_pump_speed: u8,
    led_state: u8,
    lidar_enabled: bool,
    lidar_pwm: u8,
}

impl ActuatorState {
    pub fn handle_command(&mut self, id: &str, action: &ComponentAction) {
        match (id, action) {
            ("vacuum", ComponentAction::Enable { .. }) => {
                self.vacuum = true;
                log::info!("Actuator: vacuum ENABLED");
            }
            ("vacuum", ComponentAction::Disable { .. }) => {
                self.vacuum = false;
                log::info!("Actuator: vacuum DISABLED");
            }
            ("vacuum", ComponentAction::Configure { config }) => {
                if let Some(SensorValue::U8(speed)) = config.get("speed") {
                    self.vacuum_speed = *speed;
                    log::info!("Actuator: vacuum speed = {}%", speed);
                }
            }
            // ... similar for other actuators
            _ => {
                log::debug!("Actuator: ignoring command for '{}'", id);
            }
        }
    }
}
```

## Device Factory Integration

Update `src/devices/mod.rs`:

```rust
pub mod crl200s;
pub mod mock;

use crl200s::CRL200SDriver;
use mock::MockDriver;

pub fn create_device(config: &Config) -> Result<Box<dyn DeviceDriver>> {
    match config.device.device_type.as_str() {
        "crl200s" => {
            let driver = CRL200SDriver::new(config.device.clone())?;
            Ok(Box::new(driver))
        }
        "mock" => {
            let driver = MockDriver::new(config)?;
            Ok(Box::new(driver))
        }
        _ => Err(Error::UnknownDevice(config.device.device_type.clone())),
    }
}
```

## Usage Workflow

### 1. Create a Map

```bash
# Create map in GIMP or other image editor
# - White (255) = free space
# - Black (0) = walls/obstacles
# - Gray (205) = unknown

# Save as PGM (P5 binary or P2 ASCII)
# Create YAML metadata file
```

Example `maps/apartment.yaml`:
```yaml
image: apartment.pgm
resolution: 0.02
origin: [-5.0, -5.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
cliff_mask: apartment_cliffs.pgm  # Optional
```

### 2. Create Mock Config

Copy the template from this proposal and customize:
- Set `map_file` to your map YAML path
- Adjust `start_x`, `start_y`, `start_theta` for initial pose
- Set `speed_factor` for faster testing (if needed)
- Tune noise parameters based on SLAM performance

### 3. Run Simulation

```bash
# Terminal 1: Start mock SangamIO
cd sangam-io
cargo run --release -- --config mock.toml

# Terminal 2: Connect SLAM/client
# (configure client to connect to 127.0.0.1:5555)
```

## Dependencies

**New dependencies** (add to `Cargo.toml`):
```toml
[dependencies]
image = "0.25"           # PGM loading
serde_yaml = "0.9"       # YAML parsing
rand = "0.8"             # Noise generation
rand_distr = "0.4"       # Normal distribution
```

## Testing Strategy

### Unit Tests
- Map loader: PGM+YAML parsing, cliff mask loading
- Physics: Differential drive kinematics, collision modes
- Noise: Statistical properties (mean, stddev match config)
- Bumpers: Zone detection accuracy
- Cliffs: Mask lookup, boundary detection

### Integration Tests
- **Protocol compatibility**: Mock sensor groups match CRL-200S format exactly
- **Determinism**: Same `random_seed` produces identical sensor streams
- **Speed factor**: Verify packet rates scale correctly (110Hz × factor)

### Manual Testing
- Connect Drishti and verify all sensor displays work
- Test collision modes (stop, slide, passthrough)
- Verify bumper/cliff triggers at expected positions

## Future Enhancements

1. **DhruvaSLAM map export**: Add PGM+YAML export to DhruvaSLAM for record→replay workflow
2. **Dynamic obstacles**: Moving obstacles in simulation
3. **Multiple robots**: Simulate multi-robot scenarios
4. **Sensor failures**: Inject failures for resilience testing
5. **GUI visualization**: Optional debug visualization window
6. **Virtual clock**: Support pause/step/resume for debugging
