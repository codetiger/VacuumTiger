# SangamIO

**High-level robot API for robotic vacuum cleaners running on embedded Linux**

SangamIO provides a clean, type-safe Rust API for controlling vacuum robots. Instead of dealing with low-level serial protocols and device management, you write simple robot code.

## Quick Example

```rust
use sangam_io::devices::Gd32Driver;
use sangam_io::drivers::MotorDriver;
use sangam_io::transport::SerialTransport;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let transport = SerialTransport::open("/dev/ttyS3", 115200)?;
    let mut robot = Gd32Driver::new(transport)?;

    robot.set_velocity(0.5, 0.0)?;  // Forward at 0.5 m/s
    robot.set_lidar_power(true)?;    // Power on lidar

    let odom = robot.get_odometry()?;
    println!("Position: ({:.2}, {:.2})", odom.x, odom.y);

    Ok(())
}
```

SangamIO handles initialization sequences, background heartbeat threads, state synchronization, and error recovery automatically.

## Features

- **Clean Architecture**: Layered design (Transport → Traits → Devices)
- **Automatic Management**: Background heartbeat, initialization, cleanup
- **Type Safety**: Velocities in m/s, angles in radians, compile-time checks
- **Hardware Abstraction**: Swap devices easily, test with mocks
- **Production Ready**: 9/9 tests passing, runs on Allwinner A33

## Supported Hardware

| Component | Device | Status |
|-----------|--------|--------|
| Motor Controller | GD32F103 (CRL-200S) | ✅ Verified |
| Lidar | 3iRobotix Delta-2D | ✅ Verified |
| Motor Controller | STM32, ESP32 | 📋 Planned |
| Lidar | RPLIDAR, YDLIDAR | 📋 Planned |

**Platform**: Embedded Linux (ARM/x86) with standard library

## Getting Started

### 1. Install Prerequisites

```bash
# Rust toolchain
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# ARM cross-compilation
rustup target add armv7-unknown-linux-musleabihf
```

### 2. Add to Your Project

```toml
[dependencies]
sangam-io = { version = "0.1", features = ["gd32", "lidar"] }
```

### 3. Deploy and Run

See **[GUIDE.md](GUIDE.md)** for complete step-by-step instructions on building, deploying to hardware, and troubleshooting.

## Documentation

- **[GUIDE.md](GUIDE.md)** - Complete deployment and development guide
- **[REFERENCE.md](REFERENCE.md)** - Architecture, protocols, and API reference
- **[examples/](examples/)** - Working code examples

## Project Status

**Version**: 0.1.0 (Initial Release)

**What's working**:
- Complete GD32 driver with automatic heartbeat (20ms)
- Delta-2D lidar driver with packet parsing
- Transport abstraction (serial + mock)
- Device driver traits for extensibility
- Full test coverage (9/9 tests passing)

**Next steps**:
- Additional motor controllers and lidars
- High-level Robot API layer
- Hardware calibration validation
- Performance optimization

See [REFERENCE.md](REFERENCE.md#project-status) for detailed roadmap.

## Architecture

```
Application → Device Drivers → Driver Traits → Transport
              (Gd32Driver)     (MotorDriver)    (Serial/Mock)
```

Three clean layers provide hardware abstraction and testability. Device implementations are swappable, protocols are isolated, and applications stay hardware-agnostic.

See [REFERENCE.md](REFERENCE.md#architecture-overview) for detailed architecture documentation.

## Testing

```bash
# Run unit tests
cargo test --all-features

# Run on hardware
cargo run --example test_all_components --features="std,gd32,lidar"
```

Mock implementations let you test without hardware. See [GUIDE.md](GUIDE.md#building-your-own-application) for development workflow.

## License

Licensed under Apache License 2.0. See [LICENSE](../LICENSE) for details.

## Related Projects

- **[VacuumTiger](https://github.com/codetiger/VacuumTiger)** - Complete firmware and PCB designs
- **[VacuumRobot](https://github.com/codetiger/VacuumRobot)** - Original protocol research
