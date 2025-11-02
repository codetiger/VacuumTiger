# VacuumTiger

Open-source firmware components for robotic vacuum cleaners based on CRL-200S hardware.

## What is VacuumTiger?

VacuumTiger provides tools for building custom vacuum robot firmware. The project focuses on interfacing with the GD32F103 motor controller and 3iRobotix Delta-2D lidar through clean, type-safe Rust APIs.

Based on reverse engineering work from the 3irobotix CRL-200S platform.

## Repository Structure

```
VacuumTiger/
├── sangam-io/              # High-level robot control library
│   ├── src/
│   │   ├── devices/        # GD32 driver, Delta-2D lidar driver
│   │   ├── drivers/        # Device traits (MotorDriver, LidarDriver)
│   │   ├── transport/      # I/O abstraction (serial, mock)
│   │   └── types/          # Type-safe data structures
│   ├── examples/           # Working robot control examples
│   ├── GUIDE.md            # Deployment and development guide
│   └── REFERENCE.md        # Technical reference
├── protocol-mitm/          # Protocol reverse-engineering tools
│   ├── src/                # Serial MITM logger (Rust)
│   ├── scripts/            # Robot-side automation scripts
│   ├── tools/              # Development machine tools
│   ├── logs/               # Captured protocol sessions
│   └── docs/               # MITM logging documentation
├── docs/                   # Protocol documentation
│   ├── GD32_PROTOCOL.md
│   └── HARDWARE_REFERENCE.md
└── firmware-gd32/          # Future: Custom GD32 firmware (planned)
```

## Quick Start

**1. Install Rust and ARM toolchain:**

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup target add armv7-unknown-linux-musleabihf
```

**2. Build and deploy:**

```bash
cd sangam-io
cargo build --release --target armv7-unknown-linux-musleabihf \
  --features="std,gd32,lidar"
```

**3. See full deployment guide:**

For complete instructions, see **[sangam-io/GUIDE.md](sangam-io/GUIDE.md)**

## Example Usage

```rust
use sangam_io::devices::{Gd32Driver, Delta2DDriver};
use sangam_io::drivers::{MotorDriver, LidarDriver};
use sangam_io::transport::SerialTransport;

// Initialize motor controller
let transport = SerialTransport::open("/dev/ttyS3", 115200)?;
let mut gd32 = Gd32Driver::new(transport)?;

// Move robot
gd32.set_velocity(0.5, 0.0)?;  // Forward at 0.5 m/s

// Initialize lidar
let lidar_transport = SerialTransport::open("/dev/ttyS2", 115200)?;
let mut lidar = Delta2DDriver::new(lidar_transport)?;

// Start scanning
gd32.set_lidar_power(true)?;
lidar.start()?;

while let Ok(Some(scan)) = lidar.get_scan() {
    println!("Received {} points", scan.points.len());
}
```

See **[examples/test_lidar_scenario.rs](sangam-io/examples/test_lidar_scenario.rs)** for complete working example.

## SangamIO Library

**High-level robot API for robotic vacuum cleaners running on embedded Linux**

### Features

- **Clean Architecture**: Layered design (Transport → Traits → Devices)
- **Automatic Management**: Background heartbeat, initialization, cleanup
- **Type Safety**: Velocities in m/s, angles in radians, compile-time checks
- **Hardware Abstraction**: Swap devices easily, test with mocks
- **Production Ready**: All tests passing, runs on Allwinner A33

### Supported Hardware

| Component | Device | Status |
|-----------|--------|--------|
| Motor Controller | GD32F103 (CRL-200S) | ✅ Verified |
| Lidar | 3iRobotix Delta-2D | ✅ Verified |
| Motor Controller | STM32, ESP32 | 📋 Planned |
| Lidar | RPLIDAR, YDLIDAR | 📋 Planned |

**Platform**: Embedded Linux (ARM/x86) with standard library

### Documentation

- **[sangam-io/GUIDE.md](sangam-io/GUIDE.md)** - Complete deployment and development guide
- **[sangam-io/REFERENCE.md](sangam-io/REFERENCE.md)** - Architecture, protocols, and API reference
- **[sangam-io/examples/](sangam-io/examples/)** - Working code examples

## Project Status

**Version**: 0.1.0 (Initial Release)

| Component | Status | Notes |
|-----------|--------|-------|
| SangamIO Library | ✅ Complete | High-level API with traits |
| GD32 Driver | ✅ Verified | Automatic heartbeat (20ms) |
| Delta-2D Lidar | ✅ Verified | Full packet parsing |
| Examples | ✅ Complete | test_lidar_scenario |
| Protocol MITM Tools | ✅ Complete | Serial interception & logging |
| Documentation | ✅ Complete | GUIDE.md + REFERENCE.md |
| Additional Controllers | 📋 Planned | STM32, ESP32 |
| Custom Firmware | 📋 Planned | GD32 firmware development |
| PCB Design | 📋 Planned | Custom controller board |

Legend: ✅ Complete | 🚧 In Progress | 📋 Planned

## Hardware Reference

### Verified Platform

- **Main CPU**: Allwinner A33 (or any Linux SBC with UART)
- **Motor MCU**: GigaDevice GD32F103VCT6 (Cortex-M3 @ 108MHz)
- **Lidar**: 3iRobotix Delta-2D (360°, 12m range)
- **Sensors**: IMU, wheel encoders, cliff sensors, bumpers
- **Actuators**: Drive motors, vacuum blower, brushes

See **[docs/HARDWARE_REFERENCE.md](docs/HARDWARE_REFERENCE.md)** for specifications.

## Protocol Documentation

- **[docs/GD32_PROTOCOL.md](docs/GD32_PROTOCOL.md)** - Complete GD32 communication spec
- **[docs/HARDWARE_REFERENCE.md](docs/HARDWARE_REFERENCE.md)** - Component specs, pinouts, BOM
- **[protocol-mitm/](protocol-mitm/)** - MITM tools for protocol reverse-engineering

## Contributing

Contributions welcome! Areas where help is needed:

- Additional motor controllers (STM32, ESP32)
- More lidar models (RPLIDAR, YDLIDAR)
- Hardware testing and validation
- Documentation improvements

## Safety Notice

⚠️ **IMPORTANT**: This firmware controls physical hardware that can cause injury or damage:

- Always test in a controlled, safe environment
- Keep emergency stop accessible
- Verify all connections before powering on
- Monitor battery voltage during operation

## License

Licensed under Apache License 2.0. See [LICENSE](LICENSE) for details.

## Related Links

- **[SangamIO Library](sangam-io/)** - Main library documentation
- **[Original Research](https://github.com/codetiger/VacuumRobot)** - Protocol reverse engineering work
- **[GD32 Protocol](docs/GD32_PROTOCOL.md)** - Communication specification
- **[Hardware Reference](docs/HARDWARE_REFERENCE.md)** - Component specifications
