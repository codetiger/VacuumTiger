# VacuumTiger

Open-source firmware components for robotic vacuum cleaners based on CRL-200S hardware.

## What is VacuumTiger?

VacuumTiger provides tools for building custom vacuum robot firmware. The project focuses on interfacing with the GD32F103 motor controller and 3iRobotix Delta-2D lidar through clean, type-safe Rust APIs.

Based on reverse engineering work from the 3irobotix CRL-200S platform.

## Repository Structure

```
VacuumTiger/
├── sangam-io/              # Hardware abstraction daemon
│   ├── src/
│   │   ├── app.rs          # Main application loop
│   │   ├── devices/        # GD32 driver, Delta-2D lidar driver
│   │   ├── streaming/      # TCP streaming server
│   │   └── types/          # Common data structures
│   ├── examples/           # Hardware test examples
│   ├── README.md           # Daemon overview
│   ├── PROTOCOL.md         # TCP protocol specification
│   ├── ARCHITECTURE.md     # Design decisions
│   └── DEPLOYMENT.md       # Production deployment guide
├── drishti/                # Python client for robot control & visualization
│   ├── drishti.py          # Main client library
│   ├── drishti_ui.py       # GUI interface
│   ├── ui/                 # UI components
│   ├── README.md           # Client documentation
│   └── requirements.txt    # Python dependencies
├── protocol-mitm/          # Protocol reverse-engineering tools
│   ├── src/                # Serial MITM logger (Rust)
│   ├── scripts/            # Robot-side automation scripts
│   ├── tools/              # Development machine tools
│   ├── logs/               # Captured protocol sessions
│   └── docs/               # MITM logging documentation
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
cargo build --release --target armv7-unknown-linux-musleabihf
```

**3. See deployment guide:**

For complete instructions, see **[sangam-io/DEPLOYMENT.md](sangam-io/DEPLOYMENT.md)**

## Example Usage

SangamIO runs as a daemon providing TCP streaming interface on port 5000:

```python
import socket
import struct
import msgpack

# Connect to daemon
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 5000))

# Send velocity command
cmd = {
    'type': 'SetVelocity',
    'linear': 0.3,   # m/s
    'angular': 0.0   # rad/s
}
payload = msgpack.packb(cmd)
frame = b'command\x00' + payload
sock.send(struct.pack('>I', len(frame)) + frame)

# Read telemetry stream
while True:
    # Read frame length
    length = struct.unpack('>I', sock.recv(4))[0]
    frame = sock.recv(length)

    # Parse topic and payload
    null_idx = frame.index(0)
    topic = frame[:null_idx].decode('utf-8')
    data = msgpack.unpackb(frame[null_idx + 1:])

    if topic == "telemetry":
        print(f"Battery: {data['battery_level']}%")
```

See **[sangam-io/PROTOCOL.md](sangam-io/PROTOCOL.md)** for complete protocol specification.

## SangamIO Daemon

**Hardware abstraction daemon for robotic vacuum cleaners with TCP streaming interface**

### Features

- **TCP Streaming**: Real-time telemetry and command interface on port 5000
- **Lock-Free Queues**: High-performance sensor data streaming
- **Automatic Management**: Background heartbeat (20ms), control loops (50Hz)
- **Direct Hardware Access**: Optimized for low latency (<25ms command-to-action)
- **Production Ready**: Binary size ~350KB, <1% CPU on Allwinner A33

### Supported Hardware

| Component | Device | Status |
|-----------|--------|--------|
| Motor Controller | GD32F103 (CRL-200S) | ✅ Verified |
| Lidar | 3iRobotix Delta-2D | ✅ Verified |
| Motor Controller | STM32, ESP32 | 📋 Planned |
| Lidar | RPLIDAR, YDLIDAR | 📋 Planned |

**Platform**: Embedded Linux (ARM/x86) with standard library

### Documentation

- **[sangam-io/README.md](sangam-io/README.md)** - Daemon overview and quick start
- **[sangam-io/PROTOCOL.md](sangam-io/PROTOCOL.md)** - TCP message format and commands
- **[sangam-io/DEPLOYMENT.md](sangam-io/DEPLOYMENT.md)** - Production installation guide
- **[sangam-io/ARCHITECTURE.md](sangam-io/ARCHITECTURE.md)** - Design decisions and extensibility

## Drishti Client

**Python client for real-time robot monitoring and control**

### Features

- **Real-time Telemetry**: Subscribe to sensor data streams (battery, encoders, sensors)
- **Lidar Visualization**: Receive and display 360° point cloud scans
- **Robot Control**: Send motion and actuator commands via TCP
- **Connection Monitoring**: Track communication health and latency
- **Cross-Platform**: Pure Python, runs on Mac/Linux/Windows

### Quick Start

```bash
cd drishti
pip install -r requirements.txt
python drishti.py --host 192.168.1.100  # Robot IP
```

See **[drishti/README.md](drishti/README.md)** for complete documentation.

## Project Status

**Version**: SangamIO v0.2.0-dev (Major cleanup release)

| Component | Status | Notes |
|-----------|--------|-------|
| SangamIO Daemon | ✅ Complete | TCP streaming interface, ~350KB binary |
| GD32 Driver | ✅ Verified | Automatic heartbeat (20ms), dual-thread |
| Delta-2D Lidar | ✅ Verified | 5Hz scanning, lock-free streaming |
| Drishti Client | ✅ Complete | Python client for monitoring & control |
| TCP Protocol | ✅ Complete | MessagePack over TCP with string topics |
| Protocol MITM Tools | ✅ Complete | Serial interception & logging |
| Documentation | ✅ Complete | Architecture + Protocol + Deployment guides |
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

## Development Tools

- **[protocol-mitm/](protocol-mitm/)** - MITM tools for protocol reverse-engineering and analysis

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

- **[SangamIO Daemon](sangam-io/)** - Hardware abstraction daemon documentation
- **[Drishti Client](drishti/)** - Python client for robot control & visualization
- **[TCP Protocol](sangam-io/PROTOCOL.md)** - Complete protocol specification
- **[Original Research](https://github.com/codetiger/VacuumRobot)** - Protocol reverse engineering work
