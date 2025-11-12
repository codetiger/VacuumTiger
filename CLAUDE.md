# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

VacuumTiger is an open-source firmware project for robotic vacuum cleaners based on the CRL-200S hardware platform. The project provides a unified hardware abstraction layer (SangamIO) for robot control, odometry tracking, and sensor monitoring, designed for SLAM and autonomous navigation.

**Target Platform**: Embedded Linux (ARM) - specifically Allwinner A33 running Tina Linux
**Primary Language**: Rust (edition 2024)
**Cross-compilation Target**: `armv7-unknown-linux-musleabihf`
**Main API**: `SangamIO` - unified hardware abstraction with motion control and odometry

## Build Commands

### Development Builds

```bash
# Build library
cd sangam-io
cargo build

# Build for ARM target (production)
cargo build --release --target armv7-unknown-linux-musleabihf

# Build specific example
cargo build --example quick_demo --release --target armv7-unknown-linux-musleabihf

# Run tests (host machine only)
cargo test

# Check code without building
cargo check

# Format code
cargo fmt

# Lint code
cargo clippy

# Generate documentation
cargo doc --open
```

### Testing a Single Test

```bash
# Run specific test by name
cargo test test_name

# Run tests in a specific module
cargo test devices::gd32

# Run with verbose output
cargo test -- --nocapies --test-threads=1
```

### Deployment to Robot

```bash
# Stop original firmware (required for exclusive serial port access)
ssh root@vacuum "killall -9 AuxCtrl"

# Deploy binary
scp target/armv7-unknown-linux-musleabihf/release/examples/test_all_components root@vacuum:/tmp/test
ssh root@vacuum "chmod +x /tmp/test"

# Run on robot with debug logging
ssh root@vacuum "RUST_LOG=debug /tmp/test"
```

## Architecture

### Layered Architecture

The codebase follows a layered architecture with a unified API:

```
Application Code (examples/, external)
       ↓
SangamIO (Unified HAL - src/sangam.rs)
  • Motion control with safety constraints
  • Odometry tracking for SLAM
  • Sensor monitoring
  • Lidar integration
       ↓
Device Drivers (src/devices/)
  • Gd32Driver (GD32F103 motor controller)
  • Delta2DDriver (3iRobotix lidar)
       ↓
Transport Layer (src/transport/)
  • SerialTransport (Linux /dev/tty*)
```

**Critical Design**: `SangamIO` provides the public API as a concrete struct. Device drivers (Gd32Driver, Delta2DDriver) are directly used for maximum simplicity. Only the LidarDriver trait is retained to support future lidar models.

### Key Design Patterns

1. **Unified API**: `SangamIO` is the single entry point. Initialize once, access all hardware:
   ```rust
   let sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;
   sangam.set_velocity(0.3, 0.0)?;  // Motion control
   let delta = sangam.get_odometry_delta()?;  // Odometry
   ```

2. **Automatic Resource Management**: Background threads manage heartbeat (20ms) and control loops (50Hz). Threads start on initialization and stop on `Drop`. Missing a heartbeat stops motors for safety.

3. **Configuration-Driven**: Physical parameters in `SangamConfig`:
   - Wheel geometry (base, radius, encoder ticks)
   - Motion constraints (max velocity, acceleration)
   - Control loop timing

4. **Thread-Safe State Sharing**: `Arc<Mutex<State>>` pattern using `parking_lot::Mutex`. Multiple threads coordinate safely.

5. **Type-Safe Units**: All physical quantities use proper units:
   - Velocities: m/s (meters per second)
   - Angles: radians
   - Odometry deltas: Δx, Δy (meters), Δθ (radians)
   - Distances: meters
   - Battery: 0-100%

   Never use raw integers for physical quantities. Always use proper unit conversions.

### Critical Hardware Constraints

**GD32F103 Heartbeat**: The motor controller requires CMD=0x66 heartbeat packets every 20-50ms. If heartbeat stops, motors enter safety mode and stop. The heartbeat thread uses OS threads (not async) for real-time guarantees.

**Serial Port Exclusivity**: `/dev/ttyS3` (GD32) and `/dev/ttyS1` (lidar) can only be opened by one process at a time. The original `AuxCtrl` firmware must be killed before running SangamIO code.

**Initialization Sequence**: GD32 requires a specific wake-up sequence (CMD=0x08 repeated every 200ms for up to 5 seconds) before it will respond to any commands. This is not optional.

### Protocol Implementation

**GD32 Protocol**:
- Packet format: `[0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC]`
- CRC algorithm: 16-bit big-endian word sum checksum
- Bidirectional: CPU sends commands, GD32 sends status packets (CMD=0x15, 96 bytes)
- Located in: `src/devices/gd32/protocol.rs`

**Delta-2D Lidar Protocol**:
- Variable-length packets with 8-byte headers
- Measurement data: angle (0.01° units), distance (0.25mm units)
- Located in: `src/devices/delta2d/protocol.rs`

### Module Organization

```
sangam-io/
├── src/
│   ├── lib.rs              # Public API surface - exports SangamIO
│   ├── error.rs            # Error types (never use unwrap/panic)
│   ├── sangam.rs           # SangamIO - unified hardware abstraction
│   ├── config.rs           # SangamConfig - robot parameters
│   ├── odometry.rs         # Odometry tracking (delta computation)
│   ├── motion/             # Motion control subsystem
│   │   ├── mod.rs
│   │   ├── commands.rs     # Motion commands (velocity, position)
│   │   ├── constraints.rs  # Safety limits and acceleration
│   │   └── controller.rs   # Motion controller (50Hz loop)
│   ├── transport/          # I/O abstraction layer
│   │   ├── mod.rs
│   │   └── serial.rs       # SerialTransport
│   ├── devices/            # Device drivers and lidar trait
│   │   ├── mod.rs          # LidarDriver trait definition
│   │   ├── gd32/           # GD32F103 motor controller
│   │   │   ├── mod.rs      # Gd32Driver implementation
│   │   │   ├── protocol.rs # Packet encoding/decoding
│   │   │   ├── heartbeat.rs # Background thread (20ms)
│   │   │   └── state.rs    # Shared state structures
│   │   └── delta2d/        # 3iRobotix Delta-2D lidar
│   │       ├── mod.rs      # Delta2DDriver (implements LidarDriver)
│   │       └── protocol.rs # Packet parsing
│   └── types/              # Common data structures
│       └── scan.rs         # LidarPoint, LidarScan, Pose2D, etc.
└── examples/
    └── quick_demo.rs       # 20-second hardware test
```

### Configuration System

Physical parameters and motion constraints are configured via `SangamConfig`:

```rust
// src/config.rs
impl SangamConfig {
    pub fn crl200s_defaults() -> Self {
        Self {
            wheel_base: 0.235,                // meters
            wheel_radius: 0.0325,             // meters
            ticks_per_revolution: 1560.0,     // encoder ticks
            max_linear_velocity: 0.5,         // m/s
            max_angular_velocity: 2.0,        // rad/s
            linear_acceleration: 0.3,         // m/s²
            // ... other parameters
        }
    }
}
```

When adding new robot platforms, create a new configuration method.

## Development Workflow

### Adding a New Robot Platform

**For Motor Controllers:**
1. **Implement Driver**: Create `src/devices/yourdevice/mod.rs` (no trait required - direct implementation)
2. **Define Protocol**: Add `protocol.rs` with packet encoding/decoding
3. **Update SangamIO**: Modify `src/sangam.rs` to use your driver instead of Gd32Driver
4. **Create Configuration**: Add `SangamConfig::your_robot_defaults()` with physical parameters
5. **Add Constructor**: Add `SangamIO::your_robot()` constructor
6. **Export Module**: Add to `src/devices/mod.rs`
7. **Add Example**: Create test program in `examples/`

**For Lidar Models:**
1. **Implement LidarDriver trait**: Create `src/devices/yourlidar/mod.rs` implementing the `LidarDriver` trait
2. **Define Protocol**: Add `protocol.rs` with packet parsing
3. **Export Module**: Add to `src/devices/mod.rs`
4. **Update SangamIO constructor**: Modify to accept your lidar driver
5. **Add Example**: Create test program in `examples/`

### API Usage Pattern

Always use the `SangamIO` API, not device drivers directly:

```rust
use sangam_io::SangamIO;

// Initialize
let mut sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;

// Motion control
sangam.set_velocity(0.3, 0.0)?;
sangam.move_forward(1.0)?;

// Odometry for SLAM
let delta = sangam.get_odometry_delta()?;

// Sensors
let battery = sangam.get_battery_level();

// Lidar
let scan = sangam.get_scan()?;
```

### Hardware Testing Procedure

**IMPORTANT**: The robot's monitoring system will auto-restart AuxCtrl if it's killed. You must rename the binary to prevent this.

**Complete Deployment Workflow**:

1. **Build for ARM target**:
   ```bash
   cd sangam-io
   cargo build --release --example quick_demo --target armv7-unknown-linux-musleabihf
   ```

2. **Deploy binary** (SCP doesn't work - device lacks sftp-server):
   ```bash
   cat target/armv7-unknown-linux-musleabihf/release/examples/quick_demo | \
     sshpass -p "vacuum@123" ssh root@vacuum "cat > /tmp/test && chmod +x /tmp/test"
   ```

3. **Disable AuxCtrl** (rename to prevent auto-restart):
   ```bash
   sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak && killall -9 AuxCtrl"
   ```

4. **Run test with logging**:
   ```bash
   sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "RUST_LOG=debug /tmp/test"
   ```

5. **CRITICAL: Restore AuxCtrl** after testing:
   ```bash
   sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl"
   ```

**One-Command Test** (with auto-restore on failure):
```bash
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "
  mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak && \
  killall -9 AuxCtrl 2>/dev/null; \
  RUST_LOG=debug /tmp/test; \
  EXIT_CODE=\$?; \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl; \
  exit \$EXIT_CODE
"
```

**Why These Steps**:
- **cat over SSH**: Device lacks sftp-server, so standard `scp` fails
- **Rename AuxCtrl**: Watchdog will restart killed process within 2-3 seconds
- **Always restore**: Robot won't function without AuxCtrl (or will on reboot)

### Safety Considerations

This code controls physical hardware. Always:
- Test in open space away from obstacles
- Have emergency stop accessible (physical power disconnect)
- Start with low speeds when testing new motion code
- Monitor battery levels (low battery causes erratic behavior)
- Never bypass the heartbeat mechanism
- Never use `unwrap()` or `panic!()` in production code paths

## Important Constraints and Conventions

### Code Style

- Follow Rust 2024 edition idioms
- Use `?` for error propagation, never `unwrap()` in production paths
- All public APIs must have documentation comments
- Use `log::debug!()`, `log::info!()`, etc. for logging (never `println!()` except in examples)
- Prefer explicit types over type inference in public APIs
- Use meaningful variable names (no single-letter except loop counters)

### Error Handling

- All errors go through the `sangam_io::Error` enum
- Never panic in library code
- Use `Result<T>` return types for fallible operations
- Provide context in error messages (e.g., "Failed to initialize GD32: timeout after 5s")

### Threading Model

- Heartbeat thread uses OS threads (`std::thread::spawn`), not async
- Rationale: 20ms timing requirement needs real-time guarantees
- All shared state must be `Send + Sync`
- Use `Arc<Mutex<T>>` for shared state between threads
- Prefer `parking_lot::Mutex` over `std::sync::Mutex` for performance

### Performance Characteristics

- Heartbeat timing: 20ms target (actual ~20.1ms with ±2ms jitter)
- Binary size: ~500KB statically linked
- CPU usage: <1% on Allwinner A33
- Command latency: <25ms (one heartbeat cycle)

## Common Pitfalls

1. **Forgetting Heartbeat**: The GD32 driver manages heartbeat automatically. Don't try to send manual heartbeat commands.

2. **Wrong Serial Port**: The port is `/dev/ttyS3` for GD32, not `/dev/ttyS1`. Always verify with `ls -l /dev/ttyS*`.

3. **Missing Initialization**: GD32 must receive CMD=0x08 initialization sequence before any other commands work.

4. **Unit Confusion**: Always use meters, m/s, and radians. Never pass raw encoder counts or PWM values to public APIs.

5. **Blocking I/O**: Serial operations have timeouts. Don't assume immediate response.

6. **Public API**: Always use `SangamIO`, not device drivers directly. The drivers are internal implementation details.

## Documentation References

- **sangam-io/GUIDE.md**: Complete deployment and development guide with troubleshooting
- **sangam-io/REFERENCE.md**: Architecture details, protocol specs, API reference
- **examples/README.md**: Example program documentation

When modifying protocol implementations, refer to the verified protocol specifications in the source code comments (`src/devices/gd32/protocol.rs` and `src/devices/delta2d/protocol.rs`). These were reverse-engineered and verified, so deviations will break hardware communication.

## Version and Status

- **Current Version**: 0.1.0 (Initial Release)
- **Status**: Production-ready for GD32 + Delta-2D hardware
- **Testing**: All unit tests passing, hardware verified on Allwinner A33
- **Planned**: Additional motor controllers (STM32, ESP32), more lidar models, high-level Robot API

When making changes, update CHANGELOG.md following the established format (Keep a Changelog style).
- Stopping AuxCtrl means, we need to rename the app. Killing the app is not enough as it is getting immediately restarted by the monitor app