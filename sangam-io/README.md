# SangamIO

Hardware abstraction daemon for the CRL-200S robotic vacuum platform, providing real-time sensor streaming and command control via TCP.

## Overview

SangamIO acts as a bridge between high-level clients (SLAM, visualization) and low-level hardware (GD32 motor controller, Delta-2D lidar). It runs as a standalone daemon on the robot, streaming sensor data at 500Hz and accepting commands over TCP.

```
┌─────────────────────────────────────────────────┐
│         Client Applications (SLAM/Drishti)      │
└─────────────┬───────────────┬───────────────────┘
              │ TCP 5555      │
┌─────────────▼───────────────▼───────────────────┐
│            SangamIO Daemon (Robot)              │
│  • Telemetry streaming @ 500Hz (sensors)        │
│  • Lidar streaming @ 5Hz (360° scans)           │
│  • Command processing (motion/actuators)        │
│  • Real-time control loop (50Hz heartbeat)      │
└─────────────┬───────────────┬───────────────────┘
              │ /dev/ttyS3    │ /dev/ttyS1
       ┌──────▼─────────┐  ┌──▼────────────┐
       │ GD32F103 MCU   │  │ Delta-2D Lidar│
       │ (Motor Control)│  │ (360° Scan)   │
       └────────────────┘  └───────────────┘
```

### Key Specifications

| Property | Value |
|----------|-------|
| Language | Rust 2021 edition |
| Version | 0.3.0 |
| Target | ARM (armv7-unknown-linux-musleabihf) |
| Binary Size | ~350KB (statically linked) |
| Memory Usage | <10MB RSS |
| CPU Usage | <1% on Allwinner A33 |

## Building

### Development (Host Machine)

```bash
# Build
cargo build --release

# Run with default config
cargo run --release

# Run with custom config
cargo run --release -- sangamio.toml

# Enable verbose logging
RUST_LOG=debug cargo run --release

# Run tests
cargo test
```

### Production (ARM Robot)

```bash
# Add ARM target (one-time)
rustup target add armv7-unknown-linux-musleabihf

# Build for ARM
cargo build --release --target armv7-unknown-linux-musleabihf

# Strip debug symbols (reduces size ~40%)
arm-linux-gnueabihf-strip \
  target/armv7-unknown-linux-musleabihf/release/sangam-io
```

## Deployment

**SSH**: `root@vacuum` (see project docs for credentials)

```bash
# Deploy binary (device lacks sftp-server, use cat over SSH)
cat target/armv7-unknown-linux-musleabihf/release/sangam-io | \
  ssh root@vacuum "cat > /usr/sbin/sangamio && chmod +x /usr/sbin/sangamio"

# Deploy configuration
ssh root@vacuum "cat > /etc/sangamio.toml" < sangamio.toml

# Disable original firmware (rename to prevent auto-restart)
ssh root@vacuum "mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak && killall -9 AuxCtrl"

# Run daemon
ssh root@vacuum "RUST_LOG=info /usr/sbin/sangamio"
```

> **Important**: Always overwrite `/usr/sbin/sangamio` directly. The robot monitor auto-restarts processes, so renaming AuxCtrl prevents conflicts.

## Configuration

Edit `sangamio.toml`:

```toml
[device]
type = "crl200s"
name = "CRL-200S Vacuum Robot"

[device.hardware]
gd32_port = "/dev/ttyS3"
lidar_port = "/dev/ttyS1"
heartbeat_interval_ms = 20

# Coordinate frame transforms (ROS REP-103)
[device.hardware.frame_transforms.lidar]
scale = -1.0           # Convert CW to CCW
offset = 3.14159265    # Rotate 180° (lidar mounted backward)

[device.hardware.frame_transforms.imu_gyro]
x = [2, 1]    # output_x (Roll) = input_z
y = [1, 1]    # output_y (Pitch) = input_y
z = [0, -1]   # output_z (Yaw) = input_x * -1

[network]
bind_address = "0.0.0.0:5555"
```

| Parameter | Description | Valid Values |
|-----------|-------------|--------------|
| `gd32_port` | Motor controller serial port | Device path |
| `lidar_port` | Lidar sensor serial port | Device path |
| `heartbeat_interval_ms` | Safety heartbeat interval | **20-50ms only** |
| `bind_address` | TCP server bind address | `host:port` |
| `frame_transforms` | Coordinate transforms (optional) | See below |

> **Critical**: The GD32 has a hardware watchdog requiring heartbeats every 20-50ms. Values outside this range will cause motors to stop.

### Coordinate Frame Transforms

SangamIO transforms raw sensor data to **ROS REP-103** convention:
- **X = forward** (direction robot drives)
- **Y = left** (port side)
- **Z = up**
- **Angles = counter-clockwise (CCW) positive**

All transforms default to identity (no change) if not specified. The CRL-200S requires transforms because:
- **Lidar**: Mounted backward (0° = rear), clockwise angles
- **IMU**: Non-standard axis mapping (gyro_x = yaw, not roll)

| Transform | Type | Formula | CRL-200S Value |
|-----------|------|---------|----------------|
| `lidar` | AffineTransform1D | `out = scale * in + offset` | `scale=-1, offset=π` |
| `imu_gyro` | AxisTransform3D | `[source_axis, sign]` | Remap + flip yaw |
| `imu_accel` | AxisTransform3D | `[source_axis, sign]` | Identity (default)

## TCP Protocol

### Message Format

All messages use length-prefixed framing with Protobuf payloads:

```
┌──────────────────┬─────────────────────┐
│ Length (4 bytes) │ Protobuf Payload    │
│ Big-endian u32   │ (binary)            │
└──────────────────┴─────────────────────┘
```

See `proto/sangamio.proto` for the complete schema.

### Topics

| Direction | Topic | Rate | Description |
|-----------|-------|------|-------------|
| Out | `sensors/sensor_status` | 500Hz | All sensor data |
| Out | `sensors/device_version` | Once | Version info |
| Out | `sensors/lidar` | 5Hz | 360° point cloud |
| In | `command` | On-demand | Robot commands |

### Python Client Example

```python
import socket
import struct
from proto import sangamio_pb2

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.68.101', 5555))

while True:
    # Read length prefix
    length = struct.unpack('>I', sock.recv(4))[0]

    # Read Protobuf message
    data = sock.recv(length)
    msg = sangamio_pb2.Message()
    msg.ParseFromString(data)

    if msg.topic == 'sensors/sensor_status':
        sg = msg.sensor_group
        charging = sg.values['is_charging'].bool_val
        print(f"Charging: {charging}")
```

## Module Structure

```
sangam-io/
├── src/
│   ├── main.rs              # Entry point, TCP listener
│   ├── config.rs            # TOML configuration loading
│   ├── error.rs             # Error types
│   │
│   ├── core/
│   │   ├── driver.rs        # DeviceDriver trait
│   │   └── types.rs         # SensorValue, Command types
│   │
│   ├── devices/
│   │   ├── mod.rs           # Device factory
│   │   └── crl200s/
│   │       ├── mod.rs       # CRL200S orchestrator
│   │       ├── constants.rs # Hardware constants
│   │       ├── gd32/        # Motor controller driver
│   │       │   ├── mod.rs       # Driver core
│   │       │   ├── commands.rs  # Command handlers
│   │       │   ├── heartbeat.rs # 20ms watchdog
│   │       │   ├── reader.rs    # Status parsing
│   │       │   ├── packet.rs    # Packet encoding
│   │       │   ├── protocol.rs  # Packet framing
│   │       │   ├── ring_buffer.rs
│   │       │   └── state.rs     # Atomic state
│   │       └── delta2d/     # Lidar driver
│   │           ├── mod.rs       # Driver core
│   │           └── protocol.rs  # Packet parsing
│   │
│   └── streaming/
│       ├── wire.rs          # Protobuf serialization
│       ├── tcp_publisher.rs # Outbound streaming
│       └── tcp_receiver.rs  # Command handling
│
├── proto/
│   └── sangamio.proto       # Protobuf schema
├── sangamio.toml            # Default configuration
├── COMMANDS.md              # GD32 command reference
└── SENSORSTATUS.md          # Sensor packet documentation
```

## Supported Components

Commands use a unified `ComponentControl` interface:

| Component | Enable | Disable | Configure |
|-----------|--------|---------|-----------|
| `drive` | Mode 0x02 | Stop + Mode 0x00 | `linear`, `angular` (m/s, rad/s) |
| `vacuum` | 100% | 0% | `speed` (0-100%) |
| `main_brush` | 100% | 0% | `speed` (0-100%) |
| `side_brush` | 100% | 0% | `speed` (0-100%) |
| `water_pump` | 100% | 0% | `speed` (0-100%) |
| `lidar` | Power on | Power off | - |
| `led` | - | - | `state` (0-18) |

## Sensors

The `sensor_status` group includes:

| Sensor | Type | Description |
|--------|------|-------------|
| `bumper_left`, `bumper_right` | Bool | Bumper contact |
| `cliff_front_left/right`, `cliff_left/right` | Bool | Cliff detection |
| `is_charging` | Bool | Charging state |
| `battery_voltage` | F32 | Battery voltage (V) |
| `battery_percent` | F32 | Estimated charge % |
| `wheel_left`, `wheel_right` | U16 | Encoder ticks |
| `gyro_x` | I16 | Roll rate (after transform) |
| `gyro_y` | I16 | Pitch rate (after transform) |
| `gyro_z` | I16 | Yaw rate (after transform, CCW positive) |
| `accel_x`, `accel_y`, `accel_z` | I16 | Acceleration (raw units) |
| `start_button`, `dock_button` | Bool | Button states |
| `water_tank_level` | U8 | Mop water level |

> **Note**: IMU values are in ROS REP-103 frame after applying `frame_transforms`.

## Thread Model

| Thread | Purpose | Timing |
|--------|---------|--------|
| Main | TCP listener | - |
| GD32 Heartbeat | Safety watchdog | 20ms ±2ms |
| GD32 Reader | Status parsing | Continuous |
| Lidar Reader | Scan accumulation | Continuous |
| TCP Publisher | Per-client streaming | 1ms loop |
| TCP Receiver | Per-client commands | On-demand |

## Hardware Constraints

- **Heartbeat**: GD32 requires CMD=0x66 every 20-50ms or motors stop
- **Serial Exclusivity**: Only one process can open `/dev/ttyS3` or `/dev/ttyS1`
- **Initialization**: GD32 requires ~5 second wake sequence at boot
- **Baud Rate**: Both serial ports run at 115200

## Performance

| Metric | Value |
|--------|-------|
| Sensor latency | ~20ms |
| Command latency | ~30ms |
| Protobuf bandwidth | ~95KB/s |

## Documentation

- [COMMANDS.md](COMMANDS.md) - GD32 command reference (27 commands)
- [SENSORSTATUS.md](SENSORSTATUS.md) - Status packet byte layout (96 bytes)

## Debugging

```bash
# Enable debug logging
RUST_LOG=debug ./sangamio

# Test with virtual serial ports
socat -d -d pty,raw,echo=0 pty,raw,echo=0

# Monitor on robot
ssh root@vacuum "journalctl -u sangamio -f"

# Check resource usage
ssh root@vacuum "top -p $(pgrep sangamio)"
```

## License

See the root [LICENSE](../LICENSE) file.
