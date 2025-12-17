# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

VacuumTiger is an open-source robotic vacuum firmware project for the CRL-200S hardware platform. The project consists of two main components:

1. **SangamIO**: A daemon providing hardware abstraction and real-time control via TCP streaming
2. **Drishti**: Python visualization and control client for monitoring and commanding the robot

**Target Platform**: Embedded Linux (ARM) - Allwinner A33 running Tina Linux
**Primary Language**: Rust (edition 2024) for SangamIO, Python 3.8+ for Drishti
**Cross-compilation Target**: `armv7-unknown-linux-musleabihf`
**Architecture**: Daemon-based with UDP/TCP streaming (not a library)

## System Architecture

### Overview

```
┌─────────────────────────────────────────────────┐
│         Client Applications (SLAM/Drishti)      │
└─────────────┬───────────────┬───────────────────┘
              │ UDP 5555      │ TCP 5555
              │ (sensors)     │ (commands)
┌─────────────▼───────────────▼───────────────────┐
│            SangamIO Daemon (Robot)              │
│  • Telemetry streaming @ 110Hz (UDP unicast)    │
│  • Lidar streaming @ 5Hz (360° scans)           │
│  • Command processing (TCP, reliable)           │
│  • Real-time control loop (20ms heartbeat)      │
└─────────────┬───────────────┬───────────────────┘
              │ /dev/ttyS3    │ /dev/ttyS1
       ┌──────▼─────────┐  ┌──▼────────────┐
       │ GD32F103 MCU   │  │ Delta-2D Lidar│
       │ (Motor Control)│  │ (360° Scan)   │
       └────────────────┘  └───────────────┘
```

### Key Design Principles

1. **Daemon, Not Library**: SangamIO runs as a standalone service, not imported as a library
2. **Concrete Over Abstract**: Direct implementations instead of trait objects (except LidarDriver)
3. **Lock-Free Streaming**: Uses lock-free queues for sensor data flow
4. **Real-Time Guarantees**: OS threads for heartbeat (20ms) instead of async
5. **Hybrid Protocol**: UDP for sensor streaming (low latency), TCP for commands (reliable)

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
arm-linux-gnueabihf-strip target/armv7-unknown-linux-musleabihf/release/sangam-io
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
cat target/armv7-unknown-linux-musleabihf/release/sangam-io | \
  sshpass -p "vacuum@123" ssh root@vacuum "cat > /usr/sbin/sangamio && chmod +x /usr/sbin/sangamio"

# Deploy configuration
sshpass -p "vacuum@123" ssh root@vacuum "cat > /etc/sangamio.toml" < sangamio.toml

# Disable original firmware (rename to prevent auto-restart)
sshpass -p "vacuum@123" ssh root@vacuum "mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak && killall -9 AuxCtrl"

# Run daemon
sshpass -p "vacuum@123" ssh root@vacuum "RUST_LOG=info /usr/sbin/sangamio"
```

**CRITICAL**: Always overwrite `/usr/sbin/sangamio` directly. Do NOT write to `/tmp` folder.

## Network Protocol

SangamIO uses a hybrid UDP/TCP architecture:
- **UDP** for sensor streaming (low latency, no head-of-line blocking)
- **TCP** for commands (reliable delivery)

Both use the same port (5555) for simpler configuration.

### Message Format

All messages use length-prefixed framing with Protobuf payloads:

```
+------------------+------------------+
| Length (4 bytes) | Protobuf Message |
| (big-endian)     | (binary)         |
+------------------+------------------+
```

### Proto Schema

See `sangam-io/proto/sangamio.proto` for SangamIO messages (sensors, commands) and `dhruva-slam/proto/dhruva.proto` for SLAM output messages (odometry, map, scan).

### Topics (UDP Streaming)

**Outbound (Daemon → Client via UDP)**:
- `sensors/sensor_status`: Sensor data @ 110Hz (all 13 sensors)
- `sensors/device_version`: Version info (one-time)
- `sensors/lidar`: LidarScan @ 5Hz (point cloud)

**Inbound (Client → Daemon via TCP)**:
- `command`: RobotCommand (SetVelocity, Stop, actuator control, etc.)

### Client Registration

UDP uses unicast (not broadcast). Clients are registered automatically:
1. Client connects TCP to port 5555
2. Server records client IP address
3. UDP packets are sent to client on same port
4. When TCP disconnects, UDP streaming stops

### Example Python Client

```python
import socket
import struct
import threading
from proto import sangamio_pb2

ROBOT_IP = '192.168.68.101'
PORT = 5555

# TCP for commands
tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_sock.connect((ROBOT_IP, PORT))

# UDP for sensor data (same port)
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(('0.0.0.0', PORT))

def receive_sensors():
    while True:
        data, addr = udp_sock.recvfrom(4096)
        length = struct.unpack('>I', data[:4])[0]
        msg = sangamio_pb2.Message()
        msg.ParseFromString(data[4:4+length])

        if msg.topic == 'sensors/sensor_status':
            sg = msg.sensor_group
            print(f"Battery: {sg.values['battery_level'].u8_val}%")

# Start receiving in background
threading.Thread(target=receive_sensors, daemon=True).start()

# Send commands via TCP
def send_command(cmd):
    payload = cmd.SerializeToString()
    tcp_sock.send(struct.pack('>I', len(payload)) + payload)
```

## Configuration

Edit `sangamio.toml`:

```toml
[device]
type = "crl200s"
name = "CRL-200S Vacuum Robot"

[device.hardware]
gd32_port = "/dev/ttyS3"
lidar_port = "/dev/ttyS1"

[network]
bind_address = "0.0.0.0:5555"
```

## Module Organization

```
VacuumTiger/
├── sangam-io/                 # Hardware abstraction daemon
│   ├── src/
│   │   ├── main.rs           # Entry point, TCP/UDP listener
│   │   ├── config.rs         # TOML configuration loading
│   │   ├── error.rs          # Error types
│   │   ├── core/             # Core abstractions
│   │   │   ├── mod.rs
│   │   │   ├── driver.rs     # DeviceDriver trait
│   │   │   └── types.rs      # SensorValue, Command types
│   │   ├── devices/          # Hardware drivers
│   │   │   ├── mod.rs        # Device factory (crl200s, mock)
│   │   │   ├── crl200s/      # Real hardware driver
│   │   │   │   ├── mod.rs
│   │   │   │   ├── constants.rs      # Offsets, flags, timing
│   │   │   │   ├── COMMANDS.md       # GD32 command reference
│   │   │   │   ├── SENSORSTATUS.md   # Sensor packet docs
│   │   │   │   ├── gd32/             # Motor controller driver
│   │   │   │   │   ├── mod.rs        # Driver orchestration
│   │   │   │   │   ├── protocol.rs   # RxPacket, PacketReader
│   │   │   │   │   ├── packet.rs     # TxPacket, checksum
│   │   │   │   │   ├── ring_buffer.rs # Zero-copy ring buffer
│   │   │   │   │   ├── heartbeat.rs  # 20ms watchdog loop
│   │   │   │   │   ├── reader.rs     # Status parsing
│   │   │   │   │   ├── commands.rs   # Command handlers
│   │   │   │   │   └── state.rs      # Shared atomic state
│   │   │   │   └── delta2d/          # 3iRobotix lidar
│   │   │   │       ├── mod.rs        # LidarDriver impl
│   │   │   │       └── protocol.rs   # Packet parsing
│   │   │   └── mock/         # Simulation driver (--features mock)
│   │   │       ├── mod.rs           # MockDriver, simulation loop
│   │   │       ├── config.rs        # SimulationConfig structs
│   │   │       ├── physics.rs       # Differential drive kinematics
│   │   │       ├── lidar_sim.rs     # Ray-casting lidar
│   │   │       ├── imu_sim.rs       # IMU with noise
│   │   │       ├── encoder_sim.rs   # Encoder with slip
│   │   │       ├── sensor_sim.rs    # Bumpers, cliffs
│   │   │       ├── map_loader.rs    # PGM+YAML loader
│   │   │       └── noise.rs         # Noise generator
│   │   └── streaming/        # Network communication
│   │       ├── mod.rs        # Re-exports
│   │       ├── wire.rs       # Protobuf serialization
│   │       ├── udp_publisher.rs # UDP sensor streaming
│   │       └── tcp_receiver.rs  # TCP command handler
│   ├── proto/                # Protobuf schema
│   │   └── sangamio.proto    # Message definitions
│   ├── maps/                 # Example simulation maps
│   ├── docs/                 # Documentation
│   │   └── mock-device-guide.md
│   ├── sangamio.toml         # Hardware configuration
│   └── mock.toml             # Simulation configuration
│
└── drishti/                  # Python visualization client
    ├── drishti.py           # Console client
    ├── drishti_ui.py        # GUI application
    └── ui/                  # PyQt widgets
        ├── main_window.py
        ├── processors/       # Data processing
        ├── threads/          # Background workers
        └── widgets/          # UI components
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
- **Status**: CMD=0x15, 96 bytes @ 110Hz
- **Location**: `src/devices/crl200s/gd32/protocol.rs`

### Delta-2D Protocol
- **Format**: Variable-length with 8-byte headers
- **Data**: Angle (0.01° units), distance (0.25mm units)
- **Rate**: ~5Hz complete scans
- **Location**: `src/devices/crl200s/delta2d/protocol.rs`

## Performance Characteristics

- **Binary Size**: ~350KB statically linked
- **Memory Usage**: <10MB RSS
- **CPU Usage**: <1% on Allwinner A33
- **Latency**: <25ms command-to-action
- **Streaming Rates**:
  - Telemetry: 110Hz (every GD32 packet)
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

- **sangam-io/COMMANDS.md**: GD32 command reference with TCP API
- **sangam-io/SENSORSTATUS.md**: Sensor packet byte layout
- **drishti/README.md**: Python client documentation
- **protocol-mitm/README.md**: MITM reverse engineering tools
- **protocol-mitm/docs/**: Protocol discovery documentation

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
3. **Document changes**: Update relevant documentation
4. **Test on hardware**: Virtual tests insufficient
5. **Preserve safety**: Never bypass heartbeat mechanism