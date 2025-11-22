# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

VacuumTiger is an open-source robotic vacuum firmware project for the CRL-200S hardware platform. The project consists of two main components:

1. **SangamIO**: A daemon providing hardware abstraction and real-time control via TCP streaming
2. **Drishti**: Python visualization and control client for monitoring and commanding the robot

**Target Platform**: Embedded Linux (ARM) - Allwinner A33 running Tina Linux
**Primary Language**: Rust (edition 2024) for SangamIO, Python 3.8+ for Drishti
**Cross-compilation Target**: `armv7-unknown-linux-musleabihf`
**Architecture**: Daemon-based with TCP streaming (not a library)

## System Architecture

### Overview

```
┌─────────────────────────────────────────────────┐
│         Client Applications (SLAM/Drishti)      │
└─────────────┬───────────────┬───────────────────┘
              │ TCP 5555/5556 │
┌─────────────▼───────────────▼───────────────────┐
│            SangamIO Daemon (Robot)              │
│  • Telemetry streaming @ 500Hz (sensors)        │
│  • Lidar streaming @ 5Hz (360° scans)           │
│  • Command processing (motion/actuators)        │
│  • Real-time control loop (50Hz)                │
└─────────────┬───────────────┬───────────────────┘
              │ /dev/ttyS3    │ /dev/ttyS1
       ┌──────▼─────────┐  ┌──▼────────────┐
       │ GD32F103 MCU   │  │ Delta-2D Lidar │
       │ (Motor Control)│  │ (360° Scanning)│
       └────────────────┘  └────────────────┘
```

### Key Design Principles

1. **Daemon, Not Library**: SangamIO runs as a standalone service, not imported as a library
2. **Concrete Over Abstract**: Direct implementations instead of trait objects (except LidarDriver)
3. **Lock-Free Streaming**: Uses lock-free queues for sensor data flow
4. **Real-Time Guarantees**: OS threads for heartbeat (20ms) instead of async
5. **TCP Protocol**: JSON over TCP for all communication (configurable wire format)

## Build and Deployment

### Development (Host Machine)

```bash
# Build daemon
cd sangam-io
cargo build --release

# Run with configuration
cargo run --release -- --config sangamio.toml

# Run with verbose logging
RUST_LOG=debug cargo run --release
```

### Production (ARM Robot)

```bash
# Add ARM target
rustup target add armv7-unknown-linux-musleabihf

# Build for ARM (produces static binary with musl)
cargo build --release --target armv7-unknown-linux-musleabihf

# Strip debug symbols (reduces size ~40%)
arm-linux-gnueabihf-strip target/armv7-unknown-linux-musleabihf/release/sangamio
```

### Deployment to Robot

**IMPORTANT**: The robot monitor auto-restarts killed processes. Rename binaries to prevent this.

**SSH Credentials**: `root@vacuum`, password: `vacuum@123`

**Complete Deployment**:

```bash
# Build ARM binary
cd sangam-io
cargo build --release --target armv7-unknown-linux-musleabihf

# Deploy binary (device lacks sftp-server, use cat over SSH)
cat target/armv7-unknown-linux-musleabihf/release/sangamio | \
  sshpass -p "vacuum@123" ssh root@vacuum "cat > /usr/sbin/sangamio && chmod +x /usr/sbin/sangamio"

# Deploy configuration
sshpass -p "vacuum@123" ssh root@vacuum "cat > /etc/sangamio.toml" < sangamio.toml

# Disable original firmware (rename to prevent auto-restart)
sshpass -p "vacuum@123" ssh root@vacuum "mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak && killall -9 AuxCtrl"

# Run daemon
sshpass -p "vacuum@123" ssh root@vacuum "RUST_LOG=info /usr/sbin/sangamio"
```

**CRITICAL**: Always overwrite `/usr/sbin/sangamio` directly. Do NOT write to `/tmp` folder.

## TCP Protocol

### Message Format

All messages use length-prefixed framing with JSON payloads:

```
+------------------+------------------+
| Length (4 bytes) | JSON Message     |
| (big-endian)     |                  |
+------------------+------------------+
```

### Message Structure

```json
{
    "topic": "sensors/gd32_status",
    "payload": {
        "type": "SensorGroup",
        "group_id": "gd32_status",
        "timestamp_us": 1234567890,
        "values": {
            "is_charging": {"Bool": false},
            "wheel_left": {"U16": 12345},
            "bumper_left": {"Bool": false},
            ...
        }
    }
}
```

### Topics

**Outbound (Daemon → Client)**:
- `sensors/gd32_status`: Sensor data @ 500Hz (all 13 sensors)
- `sensors/gd32_version`: Version info (one-time)
- `sensors/lidar`: LidarScan @ 5Hz (point cloud)

**Inbound (Client → Daemon)**:
- `command`: RobotCommand (SetVelocity, Stop, actuator control, etc.)

### Example Python Client

```python
import socket
import struct
import json

# Connect to daemon
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.68.101', 5555))

# Read messages
while True:
    # Read length prefix
    length = struct.unpack('>I', sock.recv(4))[0]

    # Read JSON message
    data = sock.recv(length)
    msg = json.loads(data)

    topic = msg['topic']
    if topic == 'sensors/gd32_status':
        values = msg['payload']['values']
        charging = values['is_charging']['Bool']
        print(f"Charging: {charging}")
```

## Configuration

Edit `sangamio.toml`:

```toml
[hardware]
gd32_port = "/dev/ttyS3"    # Motor controller
lidar_port = "/dev/ttyS1"   # Delta-2D lidar

[robot]
max_ticks_per_sec = 3000.0  # Calibration constant

[streaming]
tcp_pub_address = "0.0.0.0:5555"  # Telemetry output
tcp_cmd_address = "0.0.0.0:5556"  # Command input

[logging]
level = "info"
output = "stdout"
```

## Module Organization

```
VacuumTiger/
├── sangam-io/                 # Hardware abstraction daemon
│   ├── src/
│   │   ├── main.rs           # Entry point, arg parsing
│   │   ├── app.rs            # Application orchestration
│   │   ├── config.rs         # Configuration structures
│   │   ├── error.rs          # Error types
│   │   ├── serial_io.rs      # Serial transport layer
│   │   ├── devices/          # Hardware drivers
│   │   │   ├── mod.rs        # LidarDriver trait
│   │   │   ├── gd32/         # Motor controller
│   │   │   │   ├── mod.rs    # Driver implementation
│   │   │   │   ├── protocol.rs # Packet encoding
│   │   │   │   ├── heartbeat.rs # 20ms watchdog
│   │   │   │   └── state.rs  # Shared state
│   │   │   └── delta2d/      # 3iRobotix lidar
│   │   │       ├── mod.rs    # LidarDriver impl
│   │   │       └── protocol.rs # Packet parsing
│   │   └── streaming/        # TCP communication
│   │       ├── mod.rs        # Message types
│   │       ├── messages.rs   # Protocol definitions
│   │       ├── tcp_publisher.rs # Outbound stream
│   │       └── tcp_receiver.rs  # Command handler
│   └── sangamio.toml         # Configuration file
│
└── drishti/                  # Python visualization client
    ├── drishti.py           # Console client
    ├── drishti_ui.py        # GUI application
    └── ui/                  # Web interface
```

## Critical Hardware Constraints

### GD32F103 Heartbeat
- **Requirement**: CMD=0x66 heartbeat every 20-50ms
- **Consequence**: Motors stop if heartbeat missed
- **Implementation**: Dedicated OS thread, not async
- **Recovery**: 5-second re-initialization sequence

### Serial Port Exclusivity
- **Ports**: `/dev/ttyS3` (GD32), `/dev/ttyS1` (lidar)
- **Issue**: Only one process can open port
- **Solution**: Kill/rename AuxCtrl before running

### Initialization Sequence
- **GD32 Wake**: CMD=0x08 repeated for up to 5 seconds
- **Required**: Must complete before any commands work
- **Automatic**: Driver handles this internally

## Protocol Specifications

### GD32 Protocol
- **Format**: `[0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC]`
- **CRC**: 16-bit big-endian word sum checksum
- **Status**: CMD=0x15, 96 bytes @ 500Hz
- **Location**: `src/devices/gd32/protocol.rs`

### Delta-2D Protocol
- **Format**: Variable-length with 8-byte headers
- **Data**: Angle (0.01° units), distance (0.25mm units)
- **Rate**: ~5Hz complete scans
- **Location**: `src/devices/delta2d/protocol.rs`

## Performance Characteristics

- **Binary Size**: ~350KB statically linked
- **Memory Usage**: <10MB RSS
- **CPU Usage**: <1% on Allwinner A33
- **Latency**: <25ms command-to-action
- **Streaming Rates**:
  - Telemetry: 500Hz (every GD32 packet)
  - Lidar: 5Hz (native scan rate)
  - Commands: On-demand

## Thread Model

| Thread | Purpose | Timing | Priority |
|--------|---------|--------|----------|
| Main | Initialization | - | Normal |
| GD32 Heartbeat | Safety watchdog | 20ms ± 2ms | High |
| GD32 Reader | Parse status | Continuous | Normal |
| Lidar Reader | Parse scans | Continuous | Normal |
| TCP Publisher | Broadcast data | On-demand | Normal |
| TCP Receiver | Handle commands | On-demand | Normal |

## Safety and Error Handling

- **Never use `unwrap()` or `panic!()`** in production code
- **All errors through `sangam_io::Error` enum**
- **Motors stop on any error condition**
- **Heartbeat continues even if commands fail**
- **Use `Result<T>` for fallible operations**

## Testing and Debugging

### Local Testing

```bash
# Unit tests
cargo test

# Virtual serial ports
socat -d -d pty,raw,echo=0 pty,raw,echo=0

# Run with PTY
RUST_LOG=debug cargo run -- --config test.toml
```

### Hardware Testing

```bash
# Deploy and run
./deploy.sh

# Monitor logs
ssh root@vacuum "journalctl -u sangamio -f"

# Check resources
ssh root@vacuum "top -p $(pgrep sangamio)"
```

## Drishti Visualization

### GUI Application

```bash
cd drishti
source venv/bin/activate
python drishti_ui.py --robot 192.168.68.101
```

Features:
- Full-screen robot diagram with physically-correct layout
- Real-time sensor overlays (all 13 sensors)
- Bumper/cliff highlighting on trigger
- Wheel encoder tick counters
- Battery/charging status indicator
- Start/dock button states

### Console Client (Legacy)

```bash
python drishti.py --robot 192.168.68.101 --verbose
```

## Common Pitfalls and Solutions

1. **AuxCtrl Auto-Restart**: Must rename binary, not just kill process
2. **Serial Port Access**: Check `/dev/ttyS3` for GD32, not `/dev/ttyS1`
3. **No SCP Support**: Use `cat | ssh` for file transfer
4. **Unit Confusion**: Always use meters, m/s, radians in API
5. **Blocking I/O**: Serial has timeouts, don't assume immediate response

## Version History

- **v0.2.0-dev** (2024-11-14): Major cleanup, daemon architecture, performance optimizations
- **v0.1.0** (2024-11-13): Initial release with basic functionality

## Documentation References

- **sangam-io/README.md**: Quick start and overview
- **sangam-io/ARCHITECTURE.md**: Design decisions and extensibility
- **sangam-io/PROTOCOL.md**: TCP message specifications
- **sangam-io/DEPLOYMENT.md**: Production installation guide
- **sangam-io/CHANGELOG.md**: Version history and changes
- **drishti/README.md**: Python client documentation
- **drishti/UI_README.md**: GUI application guide

## Future Roadmap

### Planned Features
- WebSocket interface for browser monitoring
- ROS2 bridge for ecosystem integration
- Additional lidar models (RPLIDAR, Hokuyo)
- Multi-robot coordination support

### Explicitly Not Goals
- Generic HAL for all hardware
- Plugin system (static compilation preferred)
- Remote/cloud operation (LAN-only)
- AI/ML integration (keep in higher layers)

## Development Guidelines

When modifying this codebase:

1. **Maintain simplicity**: Avoid unnecessary abstractions
2. **Prioritize reliability**: Real-time constraints matter
3. **Document changes**: Update CHANGELOG.md
4. **Test on hardware**: Virtual tests insufficient
5. **Preserve safety**: Never bypass heartbeat mechanism