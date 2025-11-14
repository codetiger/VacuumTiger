# SangamIO

Hardware abstraction daemon for robotic vacuum cleaners, providing unified control over motors, sensors, and lidar through a TCP streaming interface.

[![Version](https://img.shields.io/badge/version-0.2.0--dev-blue)](CHANGELOG.md)
[![License](https://img.shields.io/badge/license-Apache%202.0-green)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-ARM%20Linux-orange)](DEPLOYMENT.md)

## Overview

SangamIO is a standalone daemon that manages low-level hardware communication for vacuum robots based on the CRL-200S platform. It acts as a bridge between SLAM/navigation software and the robot's hardware, handling:

- **Motion Control**: Differential drive with odometry tracking
- **Lidar Integration**: 360° scanning at 5Hz (Delta-2D)
- **Sensor Monitoring**: Battery, bumpers, cliff sensors, buttons
- **Real-time Control**: 50Hz control loop with 20ms heartbeat

The daemon communicates via TCP sockets, streaming sensor data to clients and accepting motion commands.

> **Note**: Version 0.2.0 includes major architectural simplifications and performance improvements. See [CHANGELOG.md](CHANGELOG.md) for details.

## Quick Start

### Connect to Running Daemon

```python
import socket
import struct
import msgpack

# Connect to daemon
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 5000))

# Read telemetry stream
while True:
    # Read frame length (4 bytes big-endian)
    length_bytes = sock.recv(4)
    if not length_bytes:
        break
    length = struct.unpack('>I', length_bytes)[0]

    # Read frame data (topic string + null + payload)
    frame = sock.recv(length)

    # Parse topic (null-terminated string)
    null_idx = frame.index(0)
    topic = frame[:null_idx].decode('utf-8')

    # Parse payload (MessagePack after null byte)
    payload = frame[null_idx + 1:]
    data = msgpack.unpackb(payload)

    if topic == "telemetry":  # SensorUpdate
        print(f"Battery: {data['battery_level']}%")
    elif topic == "lidar":  # LidarScan
        print(f"Lidar points: {len(data['points'])}")
```

### Send Commands

```python
# Send velocity command
cmd = {
    'type': 'SetVelocity',
    'linear': 0.3,   # m/s
    'angular': 0.0   # rad/s
}

# Serialize command to MessagePack
payload = msgpack.packb(cmd)

# Build frame: "command" + null + payload
frame = b'command\x00' + payload

# Send with length prefix
sock.send(struct.pack('>I', len(frame)) + frame)
```

## Building

### For Development (x86_64)

```bash
cargo build --release
```

### For Robot (ARM)

```bash
# Install cross-compilation toolchain
rustup target add armv7-unknown-linux-musleabihf

# Build for ARM (produces static binary with musl)
cargo build --release --target armv7-unknown-linux-musleabihf
```

The musl target automatically produces a statically-linked binary suitable for deployment on embedded Linux systems without requiring additional runtime dependencies.

## Configuration

Edit `sangamio.toml` to configure:

```toml
[robot]
type = "crl200s"

[hardware]
gd32_port = "/dev/ttyS3"
lidar_port = "/dev/ttyS1"

[network]
tcp_port = 5000
udp_discovery = true

[motion]
max_linear_velocity = 0.5   # m/s
max_angular_velocity = 2.0   # rad/s
```

## Architecture

- **Single daemon process** managing all hardware
- **Lock-free queues** for sensor data streaming
- **Direct hardware access** (no intermediate abstractions)
- **Extensible design** via configuration (one robot type at a time)

See [ARCHITECTURE.md](ARCHITECTURE.md) for design details.

## Documentation

- [PROTOCOL.md](PROTOCOL.md) - TCP message format and commands
- [DEPLOYMENT.md](DEPLOYMENT.md) - Production installation guide
- [ARCHITECTURE.md](ARCHITECTURE.md) - Design decisions and extensibility

## Performance

- **Binary Size**: ~350KB statically linked
- **Memory Usage**: <10MB RSS
- **CPU Usage**: <1% on Allwinner A33
- **Latency**: <25ms command-to-action
- **Throughput**: 50Hz control, 5Hz lidar, 20Hz telemetry

## License

MIT License - See LICENSE file for details