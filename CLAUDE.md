# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

VacuumTiger is an open-source firmware project for robotic vacuum cleaners based on the CRL-200S hardware platform. The project provides type-safe Rust APIs for interfacing with the GD32F103 motor controller and 3iRobotix Delta-2D lidar.

**Target Platform**: Embedded Linux (ARM) - specifically Allwinner A33 running Tina Linux
**Primary Language**: Rust (edition 2024)
**Cross-compilation Target**: `armv7-unknown-linux-musleabihf`

## Build Commands

### Development Builds

```bash
# Build library with all features
cd sangam-io
cargo build --features="std,gd32,lidar"

# Build for ARM target (production)
cargo build --release --target armv7-unknown-linux-musleabihf --features="std,gd32,lidar"

# Build specific example
cargo build --example test_all_components --release --target armv7-unknown-linux-musleabihf --features="std,gd32,lidar"

# Run tests (host machine only)
cargo test

# Check code without building
cargo check --features="std,gd32,lidar"

# Format code
cargo fmt

# Lint code
cargo clippy --features="std,gd32,lidar"

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

### Three-Layer Design

The codebase follows a strict layered architecture that must be preserved:

```
Application Code (examples/)
       ↓
Device Drivers (src/devices/)  ← Concrete implementations (Gd32Driver, Delta2DDriver)
       ↓
Driver Traits (src/drivers/)   ← Abstract interfaces (MotorDriver, LidarDriver)
       ↓
Transport Layer (src/transport/) ← I/O abstraction (Serial, Mock)
```

**Critical Rule**: Higher layers depend on lower layers only through traits, never concrete types. This enables testing with mocks and adding new hardware without changing application code.

### Key Design Patterns

1. **Automatic Resource Management**: The GD32 driver spawns a background OS thread (not async) that sends heartbeat packets every 20ms. This thread starts automatically on `Gd32Driver::new()` and stops on `Drop`. Missing a heartbeat stops the motors for safety.

2. **Thread-Safe State Sharing**: `Arc<Mutex<State>>` pattern using `parking_lot::Mutex` for lower overhead. The main thread and heartbeat thread coordinate through shared state.

3. **Type-Safe Units**: All physical quantities use proper units:
   - Velocities: m/s (meters per second)
   - Angles: radians
   - Distances: meters
   - Battery: 0-100%

   Never use raw integers for motor speeds or angles. Always convert through proper unit calculations.

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
│   ├── lib.rs              # Public API surface, re-exports
│   ├── error.rs            # Error types (never use unwrap/panic)
│   ├── transport/          # I/O abstraction layer
│   │   ├── mod.rs          # Transport trait definition
│   │   ├── serial.rs       # SerialTransport (requires 'std' feature)
│   │   └── mock.rs         # MockTransport for testing
│   ├── drivers/            # Hardware-agnostic traits
│   │   ├── mod.rs
│   │   ├── motor.rs        # MotorDriver trait
│   │   ├── lidar.rs        # LidarDriver trait
│   │   ├── battery.rs      # BatteryDriver trait (planned)
│   │   └── imu.rs          # ImuDriver trait (planned)
│   ├── devices/            # Concrete hardware implementations
│   │   ├── gd32/           # GD32F103 motor controller
│   │   │   ├── mod.rs      # Public API, Gd32Driver struct
│   │   │   ├── protocol.rs # Packet encoding/decoding
│   │   │   ├── heartbeat.rs # Background thread loop
│   │   │   └── state.rs    # Shared state structures
│   │   ├── delta2d/        # 3iRobotix Delta-2D lidar
│   │   │   ├── mod.rs      # Delta2DDriver
│   │   │   └── protocol.rs # Packet parsing
│   │   └── mock/           # Mock drivers for testing
│   │       ├── motor.rs
│   │       └── lidar.rs
│   └── types/              # Common data structures
│       ├── motion.rs       # Velocity, Odometry, Pose2D
│       ├── scan.rs         # LidarPoint, LidarScan
│       ├── battery.rs      # BatteryStatus
│       └── imu.rs          # ImuData
└── examples/
    └── test_all_components.rs  # Complete integration example
```

### Feature Flags

The library uses cargo features for modular compilation:

- `std`: Enables standard library (SerialTransport, threading)
- `gd32`: Includes GD32 driver code
- `lidar`: Includes Delta-2D lidar driver code
- `default = ["std", "gd32"]`

When adding new devices, create a corresponding feature flag.

## Development Workflow

### Adding a New Motor Controller

1. Create module: `src/devices/yourcontroller/mod.rs`
2. Define protocol: `src/devices/yourcontroller/protocol.rs`
3. Implement `MotorDriver` trait in `mod.rs`
4. Add feature flag to `Cargo.toml`: `yourcontroller = []`
5. Export in `src/devices/mod.rs`: `#[cfg(feature = "yourcontroller")] pub mod yourcontroller;`
6. Write unit tests using `MockTransport`
7. Add example program in `examples/`

### Adding a New Lidar

Follow the same pattern as motor controllers, implementing the `LidarDriver` trait.

### Testing Without Hardware

Use `MockTransport` to simulate serial I/O:

```rust
let mut mock = MockTransport::new();
mock.inject_read(&expected_response_bytes);

let driver = MyDriver::new(Box::new(mock))?;
driver.send_command()?;

assert_eq!(mock.get_written(), expected_command_bytes);
```

### Hardware Testing Procedure

**IMPORTANT**: The robot's monitoring system will auto-restart AuxCtrl if it's killed. You must rename the binary to prevent this.

**Complete Deployment Workflow**:

1. **Build for ARM target**:
   ```bash
   cd sangam-io
   cargo build --release --example test_all_components --features="std,gd32,lidar"
   ```

2. **Deploy binary** (SCP doesn't work - device lacks sftp-server):
   ```bash
   cat ../target/armv7-unknown-linux-musleabihf/release/examples/test_all_components | \
     sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "cat > /tmp/test && chmod +x /tmp/test"
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

6. **Feature Flags**: When building, always specify features: `--features="std,gd32,lidar"` or builds will fail.

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