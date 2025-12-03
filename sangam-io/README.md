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
cargo run --release -- my_config.json

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
ssh root@vacuum "cat > /etc/hardware.json" < hardware.json

# Disable original firmware (rename to prevent auto-restart)
ssh root@vacuum "mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak && killall -9 AuxCtrl"

# Run daemon
ssh root@vacuum "RUST_LOG=info /usr/sbin/sangamio"
```

> **Important**: Always overwrite `/usr/sbin/sangamio` directly. The robot monitor auto-restarts processes, so renaming AuxCtrl prevents conflicts.

## Configuration

Edit `hardware.json`:

```json
{
  "device": {
    "type": "crl200s",
    "name": "CRL-200S Vacuum Robot",
    "hardware": {
      "gd32_port": "/dev/ttyS3",
      "lidar_port": "/dev/ttyS1",
      "heartbeat_interval_ms": 20
    }
  },
  "network": {
    "bind_address": "0.0.0.0:5555",
    "wire_format": "json"
  }
}
```

| Parameter | Description | Valid Values |
|-----------|-------------|--------------|
| `gd32_port` | Motor controller serial port | Device path |
| `lidar_port` | Lidar sensor serial port | Device path |
| `heartbeat_interval_ms` | Safety heartbeat interval | **20-50ms only** |
| `bind_address` | TCP server bind address | `host:port` |
| `wire_format` | Message serialization | `json` or `postcard` |

> **Critical**: The GD32 has a hardware watchdog requiring heartbeats every 20-50ms. Values outside this range will cause motors to stop.

## TCP Protocol

### Message Format

All messages use length-prefixed framing:

```
┌──────────────────┬─────────────────────┐
│ Length (4 bytes) │ JSON Payload        │
│ Big-endian u32   │                     │
└──────────────────┴─────────────────────┘
```

### Topics

| Direction | Topic | Rate | Description |
|-----------|-------|------|-------------|
| Out | `sensors/sensor_status` | 500Hz | All sensor data |
| Out | `sensors/device_version` | Once | Version info |
| Out | `sensors/lidar` | 5Hz | 360° point cloud |
| In | `command` | On-demand | Robot commands |

### Example Sensor Message

```json
{
  "topic": "sensors/sensor_status",
  "payload": {
    "type": "SensorGroup",
    "group_id": "sensor_status",
    "timestamp_us": 1701612345000000,
    "values": {
      "is_charging": { "Bool": false },
      "wheel_left": { "U16": 12345 },
      "wheel_right": { "U16": 12356 },
      "bumper_left": { "Bool": false },
      "battery_percent": { "F32": 85.5 }
    }
  }
}
```

### Example Command Message

```json
{
  "topic": "command",
  "payload": {
    "type": "Command",
    "command": {
      "ComponentControl": {
        "component": "drive",
        "action": {
          "Configure": {
            "config": {
              "linear": { "F32": 0.2 },
              "angular": { "F32": 0.0 }
            }
          }
        }
      }
    }
  }
}
```

### Python Client Example

```python
import socket
import struct
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.68.101', 5555))

while True:
    # Read length prefix
    length = struct.unpack('>I', sock.recv(4))[0]

    # Read JSON message
    data = sock.recv(length)
    msg = json.loads(data)

    if msg['topic'] == 'sensors/sensor_status':
        values = msg['payload']['values']
        charging = values['is_charging']['Bool']
        print(f"Charging: {charging}")
```

## Module Structure

```
sangam-io/
├── src/
│   ├── main.rs              # Entry point, TCP listener
│   ├── config.rs            # JSON configuration loading
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
│       ├── messages.rs      # Protocol definitions
│       ├── wire.rs          # Serialization layer
│       ├── tcp_publisher.rs # Outbound streaming
│       └── tcp_receiver.rs  # Command handling
│
├── hardware.json            # Default configuration
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
| `gyro_x`, `gyro_y`, `gyro_z` | F32 | Angular velocity (rad/s) |
| `accel_x`, `accel_y`, `accel_z` | F32 | Acceleration (m/s²) |
| `start_button`, `dock_button` | Bool | Button states |
| `water_tank_level` | U8 | Mop water level |

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
| JSON bandwidth | ~235KB/s |
| Postcard bandwidth | ~95KB/s |

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
