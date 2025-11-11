# SangamIO

**Hardware abstraction library for robotic vacuum cleaners on embedded Linux**

SangamIO provides a unified, type-safe Rust API for controlling vacuum robots. It abstracts motor control, odometry tracking, lidar scanning, and sensor monitoring into a single ergonomic interface.

## Quick Example

```rust
use sangam_io::SangamIO;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize hardware (CRL-200S configuration)
    let mut sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;

    // Set velocity (forward at 0.5 m/s, no rotation)
    sangam.set_velocity(0.5, 0.0)?;

    // Move forward 1 meter
    sangam.move_forward(1.0)?;

    // Get odometry delta for SLAM
    let delta = sangam.get_odometry_delta()?;
    println!("Δx: {:.3}m, Δy: {:.3}m, Δθ: {:.3}°",
        delta.delta_x, delta.delta_y, delta.delta_theta.to_degrees());

    // Get lidar scan
    if let Some(scan) = sangam.get_scan()? {
        println!("Scanned {} points", scan.points.len());
    }

    Ok(())
}
```

SangamIO handles initialization, background threads, heartbeat management, and state synchronization automatically.

## Features

- **Unified API**: Single `SangamIO` struct abstracts all hardware components
- **Motion Control**: Velocity and position-based commands with safety constraints
- **Odometry Tracking**: Delta updates for SLAM integration (Δx, Δy, Δθ)
- **Automatic Management**: Background threads, heartbeat, initialization, cleanup
- **Type Safety**: Velocities in m/s, angles in radians, compile-time checks
- **Sensor Monitoring**: Battery, buttons, IR sensors, telemetry freshness
- **Production Ready**: Runs on Allwinner A33, CRL-200S hardware verified

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
sangam-io = "0.1"
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
- SangamIO unified hardware abstraction layer
- Motion control system with safety constraints
- Odometry tracking for SLAM integration
- GD32F103 motor controller driver (automatic heartbeat)
- Delta-2D lidar driver with scanning support
- Sensor monitoring (battery, buttons, IR sensors)
- Complete CRL-200S hardware integration

**Next steps**:
- Additional robot hardware support (STM32, other lidars)
- Advanced motion planning and path following
- ROS/ROS2 bridge for ecosystem integration
- Performance optimization and tuning

See [REFERENCE.md](REFERENCE.md#project-status) for detailed roadmap.

## Architecture

```
Application Code
       ↓
  SangamIO (Unified API)
       ↓
Motion Control + Odometry + Sensors
       ↓
Device Drivers (GD32, Delta2D)
       ↓
Transport Layer (Serial)
```

Layered architecture provides hardware abstraction for SLAM and navigation. The `SangamIO` API hides low-level details while exposing motion control, odometry deltas, and sensor data.

See [REFERENCE.md](REFERENCE.md#architecture-overview) for detailed architecture documentation.

## Testing

```bash
# Build for development
cargo build

# Build for ARM target (production)
cargo build --release --target armv7-unknown-linux-musleabihf

# Run hardware demo on robot
cargo build --example quick_demo --release --target armv7-unknown-linux-musleabihf
# Deploy and run (see GUIDE.md for deployment instructions)
```

See [GUIDE.md](GUIDE.md) for complete deployment workflow and troubleshooting.

## License

Licensed under Apache License 2.0. See [LICENSE](../LICENSE) for details.

## Related Projects

- **[VacuumTiger](https://github.com/codetiger/VacuumTiger)** - Complete firmware and PCB designs
- **[VacuumRobot](https://github.com/codetiger/VacuumRobot)** - Original protocol research
