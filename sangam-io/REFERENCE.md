# SangamIO Reference

Technical reference for SangamIO architecture, protocols, and API.

## Architecture Overview

SangamIO provides a multi-layer architecture with a unified API for robot control:

```
┌──────────────────────────────────────────────┐
│        Application Layer                     │
│        (SLAM, Navigation, Control)           │
└────────────────┬─────────────────────────────┘
                 │
┌────────────────▼─────────────────────────────┐
│        SangamIO (Unified HAL)                │
│  • Motion control & trajectory execution     │
│  • Odometry tracking (Δx, Δy, Δθ)           │
│  • Sensor monitoring (battery, buttons, IR)  │
│  • Lidar scanning integration                │
└────────────────┬─────────────────────────────┘
                 │
        ┌────────┴────────┐
        │                 │
┌───────▼───────┐  ┌──────▼─────────┐
│  GD32 Driver  │  │ Delta2D Driver │
│  (Motor+MCU)  │  │    (Lidar)     │
└───────┬───────┘  └──────┬─────────┘
        │                 │
        └────────┬────────┘
                 │
┌────────────────▼─────────────────────────────┐
│        Transport Layer (Serial I/O)          │
│        /dev/ttyS3, /dev/ttyS1                │
└──────────────────────────────────────────────┘
```

### SangamIO Layer (Unified HAL)

**Purpose**: Single entry point for robot control, hiding hardware complexity

**Key Components**:
- **Motion Controller**: Velocity/position commands with safety constraints
- **Odometry Tracker**: Computes deltas (Δx, Δy, Δθ) for SLAM
- **Sensor Monitor**: Battery, buttons, IR sensors, telemetry freshness
- **Lidar Integration**: Scanning control and data retrieval

**Main API**:
```rust
impl SangamIO {
    // Initialization
    pub fn crl200s(motor_port: &str, lidar_port: &str) -> Result<Self>;

    // Motion control
    pub fn set_velocity(&self, linear: f32, angular: f32) -> Result<()>;
    pub fn move_forward(&self, distance: f32) -> Result<()>;
    pub fn rotate(&self, angle: f32) -> Result<()>;
    pub fn stop(&self) -> Result<()>;

    // Odometry for SLAM
    pub fn get_odometry_delta(&self) -> Result<OdometryDelta>;
    pub fn reset_odometry(&self) -> Result<()>;

    // Sensors
    pub fn get_battery_level(&self) -> Option<u8>;
    pub fn get_ir_sensors(&self) -> Option<(u16, u16, u16)>;
    pub fn is_telemetry_fresh(&self) -> bool;

    // Lidar
    pub fn get_scan(&self) -> Result<Option<LidarScan>>;

    // Cleaning components
    pub fn set_blower_speed(&self, speed: u8) -> Result<()>;
    pub fn set_brush_speed(&self, speed: u8) -> Result<()>;
}
```

### Device Driver Layer

**GD32 Driver** (`src/devices/gd32/`):
- Motor control via GD32F103 MCU
- Background heartbeat thread (20ms)
- Encoder reading and telemetry
- Cleaning component control

**Delta2D Driver** (`src/devices/delta2d/`):
- 3iRobotix Delta-2D lidar control
- Packet parsing and scan assembly
- Background scanning thread

### Transport Layer

**SerialTransport** (`src/transport/serial.rs`):
- Linux `/dev/tty*` devices
- Configurable baud rate, timeouts
- Read/write/flush operations

### Key Design Patterns

**1. Automatic Resource Management**
- Heartbeat thread starts/stops automatically
- Serial ports closed via RAII (Drop trait)
- No manual cleanup required

**2. Thread-Safe State Sharing**
- `Arc<Mutex<State>>` for shared state
- Main thread and heartbeat thread coordinate safely
- `parking_lot::Mutex` for better performance

**3. Type-Safe APIs**
- Velocities in m/s, angles in radians
- Battery level 0-100%, distance in meters
- Compile-time safety prevents unit errors

## GD32 Protocol

### Packet Structure

```
[0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC]
```

- **SYNC1, SYNC2**: Fixed bytes (0xFA, 0xFB)
- **LEN**: Packet length (CMD + PAYLOAD + CRC)
- **CMD**: Command ID
- **PAYLOAD**: Variable length data
- **CRC**: XOR checksum: `CMD ⊕ PAYLOAD[0] ⊕ PAYLOAD[1] ⊕ ... ⊕ CMD`

### Key Commands

| CMD  | Name | Direction | Payload | Purpose |
|------|------|-----------|---------|---------|
| 0x08 | Initialize | CPU→GD32 | 96 bytes | Wake-up sequence |
| 0x06 | Wake | CPU→GD32 | 1 byte | Enable motors |
| 0x66 | Heartbeat | CPU→GD32 | 8 bytes | Keep-alive (every 20ms) |
| 0x67 | Motor Speed | CPU→GD32 | 8 bytes | Set wheel speeds |
| 0x68 | Blower Speed | CPU→GD32 | 2 bytes | Control vacuum |
| 0x97 | Lidar Power | CPU→GD32 | 1 byte | On/off lidar motor |
| 0x15 | Status Data | GD32→CPU | 96 bytes | Sensor readings |

### Initialization Sequence

The GD32 requires a specific boot sequence:

1. **Send CMD=0x08** repeatedly every 200ms
2. **Wait for CMD=0x15** response (up to 5 seconds)
3. **Send CMD=0x06** to wake motors
4. **Start heartbeat thread** sending CMD=0x67 every 20ms

If heartbeat stops for >50ms, GD32 enters error state and stops motors.

### Heartbeat Implementation

SangamIO runs a background OS thread (not async) for real-time guarantees:

```rust
// Heartbeat thread (simplified)
loop {
    // Get target speeds from shared state
    let (left, right) = state.lock();

    // Send motor command (doubles as heartbeat)
    send_packet(MotorSpeed { left, right })?;

    // Process incoming status packets
    if let Some(response) = try_read_response()? {
        update_state(response);  // Update encoders, battery, etc.
    }

    sleep(Duration::from_millis(20));  // 50Hz
}
```

## Delta-2D Lidar Protocol

### Packet Structure

```
[HEADER 8B] [PAYLOAD variable] [CRC 2B]
```

**Header** (8 bytes):
- Chunk header (1 byte)
- Chunk length (2 bytes, big-endian)
- Chunk version (1 byte)
- Chunk type (1 byte)
- Command type (1 byte): 0xAE (health) or 0xAD (measurement)
- Payload length (2 bytes, big-endian)

**Measurement Payload**:
- Motor RPM (1 byte) × 3 = actual RPM
- Offset angle (2 bytes BE) × 0.01 = degrees
- Start angle (2 bytes BE) × 0.01 = degrees
- Measurements (3 bytes each):
  - Signal quality (1 byte)
  - Distance (2 bytes BE) × 0.25 = millimeters

### Data Flow

```
Lidar → UART (115200) → Delta2DDriver.get_scan()
  → Parse packet → Convert units (mm→m, deg→rad)
  → Return LidarScan { points: Vec<LidarPoint> }
```

## API Guide

### Complete API Documentation

Run `cargo doc --open` to see full API documentation with examples.

### Basic Usage Patterns

**Initialize SangamIO**:
```rust
use sangam_io::SangamIO;

// CRL-200S hardware configuration
let mut sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;
```

**Motion Control**:
```rust
// Velocity control
sangam.set_velocity(0.3, 0.0)?;        // 0.3 m/s forward
sangam.set_velocity(0.0, 1.5)?;        // Rotate at 1.5 rad/s

// Position control
sangam.move_forward(1.0)?;              // Move 1 meter forward
sangam.rotate(std::f32::consts::PI)?;  // Rotate 180°

// Stop
sangam.stop()?;                         // Emergency stop
```

**Odometry for SLAM**:
```rust
// Get delta since last call
let delta = sangam.get_odometry_delta()?;
println!("Δx: {:.3}m, Δy: {:.3}m, Δθ: {:.3}rad",
    delta.delta_x, delta.delta_y, delta.delta_theta);

// Reset odometry
sangam.reset_odometry()?;
```

**Sensor Monitoring**:
```rust
// Battery level
if let Some(level) = sangam.get_battery_level() {
    println!("Battery: {}%", level);
}

// IR sensors (obstacle detection)
if let Some((ir1, start_ir, dock_ir)) = sangam.get_ir_sensors() {
    if ir1 > 500 {
        println!("Obstacle detected!");
    }
}

// Telemetry freshness (detect communication loss)
if !sangam.is_telemetry_fresh() {
    println!("Warning: Lost communication with motor controller");
}
```

**Lidar Scanning**:
```rust
// Get latest scan
if let Some(scan) = sangam.get_scan()? {
    println!("Scanned {} points", scan.points.len());
    for point in &scan.points {
        println!("{:.1}° → {:.2}m",
            point.angle.to_degrees(), point.distance);
    }
}
```

**Cleaning Components**:
```rust
sangam.set_blower_speed(75)?;   // Vacuum at 75%
sangam.set_brush_speed(50)?;    // Side brush at 50%
```

## Motion Control System

The motion controller provides safety-constrained movement:

**Features**:
- Velocity ramping with acceleration limits
- Emergency stop with higher deceleration
- Max velocity constraints (linear and angular)
- Trajectory execution for position-based commands

**Configuration** (`src/config.rs`):
```rust
max_linear_velocity: 0.5,     // m/s
max_angular_velocity: 2.0,    // rad/s
linear_acceleration: 0.3,     // m/s²
angular_acceleration: 1.5,    // rad/s²
emergency_deceleration: 1.0,  // m/s²
```

**Control Loop**:
- Runs at 50Hz (20ms period)
- Applies acceleration limits
- Updates motor commands smoothly
- Monitors trajectory completion

## Odometry Tracking

The odometry tracker computes motion deltas for SLAM integration:

**OdometryDelta Structure**:
```rust
pub struct OdometryDelta {
    pub delta_x: f32,      // meters
    pub delta_y: f32,      // meters
    pub delta_theta: f32,  // radians
    pub timestamp: Duration,
}
```

**Algorithm**:
1. Read encoder values from GD32
2. Convert ticks to wheel distances
3. Apply differential drive kinematics
4. Compute global pose change
5. Return delta since last call

**Usage Pattern**:
```rust
loop {
    let delta = sangam.get_odometry_delta()?;
    slam.update(delta.delta_x, delta.delta_y, delta.delta_theta);
    thread::sleep(Duration::from_millis(100));
}
```

## Extending SangamIO

### Adding Support for New Robot Hardware

To add a new robot configuration (e.g., different motor controller or lidar):

1. **Implement Device Drivers**:
   - Create new module in `src/devices/mydevice/`
   - Implement `MotorDriver` or `LidarDriver` traits
   - Handle protocol encoding/decoding

2. **Create Configuration**:
   ```rust
   impl SangamConfig {
       pub fn my_robot_defaults() -> Self {
           Self {
               wheel_base: 0.25,  // Your robot's measurements
               wheel_radius: 0.04,
               ticks_per_revolution: 2000.0,
               // ... other parameters
           }
       }
   }
   ```

3. **Add Constructor**:
   ```rust
   impl SangamIO {
       pub fn my_robot(motor_port: &str, lidar_port: &str) -> Result<Self> {
           let config = SangamConfig::my_robot_defaults();
           Self::new(motor_port, lidar_port, config)
       }
   }
   ```

4. **Export in `src/devices/mod.rs`**:
   ```rust
   pub mod mydevice;
   ```

### Example: Adding STM32 Motor Controller

```rust
// src/devices/stm32/mod.rs
pub struct Stm32Driver { /* ... */ }

impl MotorDriver for Stm32Driver {
    fn set_velocity(&mut self, linear: f32, angular: f32) -> Result<()> {
        // Convert to STM32 protocol
        // Send command via transport
        Ok(())
    }
    // ... implement other methods
}

// Usage
impl SangamIO {
    pub fn with_stm32(motor_port: &str, lidar_port: &str) -> Result<Self> {
        let motor = Box::new(Stm32Driver::new(motor_port)?);
        // ... initialize with custom motor driver
    }
}
```

## Hardware Specifications

### Serial Port Assignments

| Device | Port | Baud | Protocol |
|--------|------|------|----------|
| GD32F103 Motor Controller | `/dev/ttyS3` | 115200 | GD32 Protocol |
| 3iRobotix Delta-2D Lidar | `/dev/ttyS1` | 115200 | Delta-2D Protocol |

### GD32F103 Specifications

- **MCU**: ARM Cortex-M3 @ 108MHz
- **Functions**: Motor control, IMU, sensors, real-time control
- **Communication**: UART 115200 8N1
- **Heartbeat**: Required every 20-50ms (stops if missed)

### Delta-2D Lidar Specifications

- **Range**: 12 meters
- **Scan Rate**: 5-10 Hz (full 360°)
- **Sample Rate**: ~360 points per scan
- **Interface**: UART 115200 8N1
- **Power**: Controlled via GD32 (CMD=0x97)

## Troubleshooting Reference

### Technical Error Codes

**`Error::InitializationFailed`**: GD32 didn't respond to CMD=0x08
- Check serial port path
- Verify baud rate (115200)
- Ensure GD32 is powered
- Kill any process using `/dev/ttyS3`

**`Error::ChecksumError`**: CRC mismatch in received packet
- Serial interference or noise
- Wrong baud rate
- Check physical connections

**`Error::Timeout`**: Operation timed out
- Device not responding
- Check if device is powered and connected
- Verify correct serial port

**`Error::InvalidPacket`**: Malformed packet
- Protocol mismatch
- Buffer overflow
- Serial configuration error

For common user-facing issues, see [GUIDE.md](GUIDE.md#troubleshooting).

## Design Decisions

### Why Synchronous Threading?

**Decision**: Use OS threads instead of async/tokio for heartbeat

**Rationale**: The 20ms heartbeat requirement needs real-time guarantees. OS scheduler provides better determinism than cooperative async scheduling. Missing a heartbeat stops the motors.

**Trade-off**: Slightly higher resource usage (thread stack), but critical timing is guaranteed.

### Why Unified SangamIO API?

**Decision**: Single struct with all functionality, not separate device drivers

**Rationale**: SLAM and navigation algorithms need coordinated access to odometry, motion, and sensors. A unified API simplifies integration and ensures proper initialization order. Users don't need to manage multiple drivers.

### Why Automatic Heartbeat?

**Decision**: Background thread manages heartbeat automatically

**Rationale**: Heartbeat is critical for GD32 operation. Users shouldn't have to remember to call it. Automatic management is safer and more ergonomic.

### Why Internal Trait-Based Abstraction?

**Decision**: Device drivers use traits internally, but hide them from public API

**Rationale**: Traits provide flexibility for adding new hardware without exposing complexity to users. The `SangamIO` facade provides a simpler, concrete API while maintaining internal modularity.

## Performance Characteristics

### Heartbeat Timing

- **Target**: 20ms (50Hz)
- **Measured**: ~20.1ms average (OS scheduler dependent)
- **Jitter**: ±2ms typical
- **CPU**: <1% on Allwinner A33

### Memory Usage

- **Binary size**: ~500KB (statically linked)
- **Heap allocations**: Minimal (mostly for packet buffers)
- **Stack**: Main thread ~8KB, heartbeat thread ~8KB

### Latency

- **Command to execution**: <25ms (one heartbeat cycle)
- **Sensor read**: <5ms (serial read time)
- **Lidar scan**: 100-200ms (depends on packet arrival)

## Project Status

### Implemented ✅

- SangamIO unified hardware abstraction layer
- Motion control system with safety constraints
- Odometry tracking for SLAM (delta updates)
- GD32 driver with automatic heartbeat (20ms)
- Delta-2D lidar driver with scanning
- Sensor monitoring (battery, buttons, IR sensors)
- Configuration system (SangamConfig)
- Complete CRL-200S hardware integration
- Example programs (quick_demo)

### In Progress 🚧

- Advanced motion planning (path following, trajectories)
- Hardware calibration refinement
- Performance optimization and tuning

### Planned 📋

- Additional robot platforms (STM32-based, other lidars)
- Collision avoidance primitives
- Docking station navigation
- ROS/ROS2 bridge for ecosystem integration
- Multi-robot coordination support

## Contributing

### Code Style

- Follow Rust standard formatting (`cargo fmt`)
- Pass clippy without warnings (`cargo clippy`)
- Add tests for new features
- Document public APIs

### Adding Hardware Support

1. Fork the repository
2. Create feature branch
3. Implement traits in `src/devices/yourdevice/`
4. Add tests
5. Update documentation
6. Submit pull request

### Reporting Bugs

Open an issue with:
- Hardware details (which robot, which device)
- Steps to reproduce
- Expected vs actual behavior
- Relevant logs (with `RUST_LOG=debug`)

---

For practical deployment and development guides, see [GUIDE.md](GUIDE.md).
For API details, run `cargo doc --open`.
