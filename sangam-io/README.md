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

    // Start lidar with callback
    sangam.start_lidar(|scan| {
        println!("Scanned {} points at {:.3}°",
            scan.points.len(), scan.angle.to_degrees());
    })?;

    // Monitor sensors
    if let Some(battery) = sangam.get_battery_level() {
        println!("Battery: {}%", battery);
    }

    Ok(())
}
```

The `SangamIO` struct handles all hardware initialization, background threads, and communication automatically - you just call the methods you need.

## Features

- 🎯 **Simple API**: One struct (`SangamIO`), intuitive methods, minimal boilerplate
- 🤖 **Motion Control**: Velocity commands, position-based movement, rotation with safety limits
- 📍 **SLAM-Ready**: Real-time odometry deltas (Δx, Δy, Δθ) for mapping and localization
- 🔄 **Automatic Management**: Hardware initialization, background threads, heartbeat - all handled for you
- 🔒 **Type Safety**: Physical units (m/s, radians), compile-time checks, no raw integers
- 📊 **Sensor Access**: Battery, buttons, IR sensors, connection health monitoring
- ✅ **Production Ready**: Verified on CRL-200S hardware, Allwinner A33 platform

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
- ✅ **Motion Control**: Velocity commands, position-based movement, rotation
- ✅ **Odometry**: Real-time delta tracking for SLAM (Δx, Δy, Δθ)
- ✅ **Lidar Scanning**: Callback-based scan processing
- ✅ **Sensors**: Battery level, charging status, button detection
- ✅ **Monitoring**: Telemetry freshness, connection health
- ✅ **Hardware**: CRL-200S robot (GD32F103 + Delta-2D lidar)

**Coming soon**:
- 🔧 Additional hardware support (STM32, RPLIDAR, YDLIDAR)
- 🔧 Advanced motion planning and trajectory following
- 🔧 ROS/ROS2 integration for ecosystem compatibility

See [REFERENCE.md](REFERENCE.md#project-status) for detailed roadmap.

## Core API

The `SangamIO` struct provides a unified interface for robot control:

### Motion Control
```rust
// Velocity control (continuous motion)
sangam.set_velocity(0.3, 0.0)?;  // 0.3 m/s forward, no rotation
sangam.stop()?;                  // Gradual stop

// Position-based movement (non-blocking)
sangam.move_forward(1.0)?;       // Move 1 meter forward
sangam.move_backward(0.5)?;      // Move 0.5 meters back
sangam.rotate(1.57)?;            // Rotate 90° (π/2 radians)
```

### Odometry for SLAM
```rust
// Get odometry delta since last call
let delta = sangam.get_odometry_delta()?;
println!("Δx: {:.3}m, Δy: {:.3}m, Δθ: {:.3}rad",
    delta.delta_x, delta.delta_y, delta.delta_theta);
```

### Sensors & Monitoring
```rust
// Battery and charging status
if let Some(level) = sangam.get_battery_level() {
    println!("Battery: {}%", level);
}
let is_charging = sangam.is_charging();

// Button detection
if sangam.is_start_button_pressed() == Some(true) {
    println!("Start button pressed!");
}

// Connection health
if !sangam.is_telemetry_fresh() {
    println!("Warning: Lost communication");
}
```

### Lidar Scanning
```rust
sangam.start_lidar(|scan| {
    println!("Scanned {} points", scan.points.len());
})?;
```

See [REFERENCE.md](REFERENCE.md) for complete API documentation.

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
