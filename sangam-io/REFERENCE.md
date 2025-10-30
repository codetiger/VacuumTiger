# SangamIO Reference

Technical reference for SangamIO architecture, protocols, and API.

## Architecture Overview

SangamIO uses a three-layer architecture for hardware abstraction and modularity:

```
┌──────────────────────────────────┐
│       Application Layer          │
│    (Your robot control code)     │
└────────────────┬─────────────────┘
                 │
┌────────────────▼─────────────────┐
│       Device Drivers Layer       │
│    Gd32Driver │ Delta2DDriver    │
│    (Concrete implementations)    │
└────────────────┬─────────────────┘
                 │
┌────────────────▼─────────────────┐
│       Driver Traits Layer        │
│    MotorDriver │ LidarDriver     │
└────────────────┬─────────────────┘
                 │
┌────────────────▼─────────────────┐
│      Transport Layer (I/O)       │
│ SerialTransport │ MockTransport  │
└──────────────────────────────────┘
```

### Layer 1: Transport

**Purpose**: Abstract I/O operations (serial, USB, network, mock)

```rust
pub trait Transport: Send {
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize>;
    fn write(&mut self, data: &[u8]) -> Result<usize>;
    fn flush(&mut self) -> Result<()>;
    fn available(&mut self) -> Result<usize>;
}
```

**Implementations**:
- `SerialTransport`: Linux `/dev/tty*` devices
- `MockTransport`: Testing without hardware

### Layer 2: Driver Traits

**Purpose**: Hardware-agnostic interfaces for robot components

**MotorDriver**: Motor control and odometry
```rust
pub trait MotorDriver: Send {
    fn set_velocity(&mut self, linear: f32, angular: f32) -> Result<()>;
    fn set_wheel_velocity(&mut self, left: f32, right: f32) -> Result<()>;
    fn get_odometry(&mut self) -> Result<Odometry>;
    fn stop(&mut self) -> Result<()>;
}
```

**LidarDriver**: Laser range finder control
```rust
pub trait LidarDriver: Send {
    fn start(&mut self) -> Result<()>;
    fn get_scan(&mut self) -> Result<Option<LidarScan>>;
    fn stop(&mut self) -> Result<()>;
    fn is_scanning(&self) -> bool;
}
```

**Benefits**:
- Swap implementations easily (e.g., test with MockMotorDriver)
- Add new hardware by implementing traits
- Application code doesn't depend on specific protocols

### Layer 3: Device Implementations

**Gd32Driver**: Motor controller implementation
- Implements `MotorDriver` trait
- Manages GD32F103 protocol
- Runs background heartbeat thread (20ms)
- Handles initialization sequence

**Delta2DDriver**: Lidar implementation
- Implements `LidarDriver` trait
- Manages 3iRobotix Delta-2D protocol
- Parses measurement packets
- Returns structured scan data

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

**Initialize devices**:
```rust
let transport = SerialTransport::open("/dev/ttyS3", 115200)?;
let mut gd32 = Gd32Driver::new(transport)?;  // Auto-initializes
```

**Control motors**:
```rust
gd32.set_velocity(0.5, 0.0)?;           // Linear and angular velocity
gd32.set_wheel_velocity(1.0, 1.0)?;    // Direct wheel control
gd32.stop()?;                            // Emergency stop
```

**Read sensors**:
```rust
let odom = gd32.get_odometry()?;
println!("Position: ({:.2}, {:.2})", odom.x, odom.y);
println!("Encoders: L={}, R={}", odom.encoder_left, odom.encoder_right);
```

**Control peripherals**:
```rust
gd32.set_lidar_power(true)?;     // Power on lidar motor
gd32.set_blower_speed(75)?;      // Vacuum at 75%
```

**Scan with lidar**:
```rust
let transport = SerialTransport::open("/dev/ttyS2", 115200)?;
let mut lidar = Delta2DDriver::new(transport)?;

lidar.start()?;
while let Ok(Some(scan)) = lidar.get_scan() {
    for point in &scan.points {
        println!("{:.1}° → {:.2}m",
            point.angle.to_degrees(), point.distance);
    }
}
```

### Testing with Mocks

```rust
use sangam_io::devices::MockMotorDriver;

let mut motor = MockMotorDriver::new();
motor.set_velocity(0.5, 0.0)?;
assert_eq!(motor.get_velocity(), (0.5, 0.0));  // Check internal state
```

## Extending SangamIO

### Adding a New Motor Controller

1. **Create module**: `src/devices/mystm32/mod.rs`

2. **Define protocol**: `src/devices/mystm32/protocol.rs`
   ```rust
   pub enum Stm32Command {
       SetSpeed { left: i32, right: i32 },
       // ... other commands
   }

   impl Stm32Command {
       pub fn encode(&self) -> Vec<u8> { /* ... */ }
   }
   ```

3. **Implement MotorDriver**:
   ```rust
   pub struct Stm32Driver {
       transport: Arc<Mutex<Box<dyn Transport>>>,
       // ... state
   }

   impl MotorDriver for Stm32Driver {
       fn set_velocity(&mut self, linear: f32, angular: f32) -> Result<()> {
           // Convert to motor units
           // Send command via transport
           Ok(())
       }
       // ... other methods
   }
   ```

4. **Add feature flag** in `Cargo.toml`:
   ```toml
   [features]
   stm32 = []
   ```

5. **Export** in `src/devices/mod.rs`:
   ```rust
   #[cfg(feature = "stm32")]
   pub mod stm32;
   ```

### Adding a New Lidar

Follow the same pattern, implementing `LidarDriver` trait.

### Testing New Devices

Use `MockTransport` to test without hardware:

```rust
let mock = MockTransport::new();
mock.inject_read(&expected_response);

let driver = MyDriver::new(mock)?;
let result = driver.command()?;

assert_eq!(mock.get_written(), expected_command);
```

## Hardware Specifications

### Serial Port Assignments

| Device | Port | Baud | Protocol |
|--------|------|------|----------|
| GD32F103 Motor Controller | `/dev/ttyS3` | 115200 | GD32 Protocol |
| 3iRobotix Delta-2D Lidar | `/dev/ttyS2` | 115200 | Delta-2D Protocol |

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

### Why Single Crate?

**Decision**: One library with feature flags, not a workspace

**Rationale**: Simpler deployment and dependency management. Can split later if needed. Users just add one dependency.

### Why Automatic Heartbeat?

**Decision**: Background thread manages heartbeat automatically

**Rationale**: Heartbeat is critical for GD32 operation. Users shouldn't have to remember to call it. Automatic management is safer and more ergonomic.

### Why Trait-Based Abstraction?

**Decision**: Define device types as traits, not concrete types

**Rationale**: Maximum flexibility for testing (mocks) and extending (new hardware). Application code doesn't depend on specific implementations.

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

- GD32 driver with automatic heartbeat
- Delta-2D lidar driver
- Transport abstraction (serial, mock)
- Device driver traits
- Type-safe data structures
- Unit tests (9/9 passing)
- Example programs

### In Progress 🚧

- Hardware calibration (conversion constants need testing)
- Status packet field mapping (needs hardware verification)
- Velocity calculation from encoders

### Planned 📋

- Additional motor controllers (STM32, ESP32)
- More lidar models (RPLIDAR, YDLIDAR)
- IMU and battery driver traits
- High-level Robot API
- ROS/ROS2 bridge

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
