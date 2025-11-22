# VacuumTiger

Open-source firmware stack for robotic vacuum cleaners.

## What is VacuumTiger?

VacuumTiger is a modular software stack designed to make it easy to build and support various robot vacuum platforms. The architecture separates hardware abstraction from higher-level control, allowing the same control software to work across different robot models.

**Currently supported:** CRL-200S based models (3iRobotix platform)

### Components

- **SangamIO**: Hardware abstraction daemon with TCP streaming interface (Rust)
- **Drishti**: Diagnostic tool for sensor testing and protocol verification (Python)

The design prioritizes:
- **Portability**: Clean abstraction layer for supporting new robot hardware
- **Simplicity**: Direct implementations over complex abstractions
- **Real-time performance**: Low latency control loops with safety guarantees

## System Architecture

```
┌─────────────────────────────────────────────────┐
│         Client Applications (Drishti/SLAM)      │
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
       ┌──────▼─────────┐  ┌──▼─────────────┐
       │ GD32F103 MCU   │  │ Delta-2D Lidar │
       │ (Motor Control)│  │ (360° Scanning)│
       └────────────────┘  └────────────────┘
```

## Repository Structure

```
VacuumTiger/
├── sangam-io/              # Hardware abstraction daemon (Rust)
│   ├── src/
│   │   ├── app.rs          # Main application loop
│   │   ├── devices/        # GD32 driver, Delta-2D lidar driver
│   │   └── streaming/      # TCP streaming server
│   ├── PROTOCOL.md         # TCP protocol specification
│   ├── ARCHITECTURE.md     # Design decisions
│   └── DEPLOYMENT.md       # Production deployment guide
├── drishti/                # Python visualization client
│   ├── drishti.py          # Console client library
│   ├── drishti_ui.py       # GUI application
│   └── ui/                 # PyQt5 components
├── protocol-mitm/          # Protocol reverse-engineering tools
│   ├── src/                # Serial MITM logger (Rust)
│   ├── scripts/            # Robot-side automation scripts
│   └── tools/              # Development machine tools
└── firmware-gd32/          # Future: Custom GD32 firmware (planned)
```

## Quick Start

### 1. Install Dependencies

**Rust and ARM toolchain:**
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup target add armv7-unknown-linux-musleabihf
```

**Python (for Drishti):**
```bash
cd drishti
pip install -r requirements.txt
```

### 2. Build and Deploy SangamIO

```bash
cd sangam-io
cargo build --release --target armv7-unknown-linux-musleabihf

# Deploy to robot (see DEPLOYMENT.md for full details)
cat target/armv7-unknown-linux-musleabihf/release/sangamio | \
  ssh root@vacuum "cat > /usr/sbin/sangamio && chmod +x /usr/sbin/sangamio"
```

### 3. Launch Drishti Visualization

```bash
cd drishti
python drishti_ui.py --robot 192.168.68.101
```

## TCP Protocol

SangamIO provides a JSON-based TCP streaming interface:

- **Port 5555**: Telemetry stream (sensors @ 500Hz, lidar @ 5Hz)
- **Port 5556**: Command channel (motion, actuators, lifecycle)

**Message Format:** Length-prefixed JSON framing
```
[4-byte length (big-endian)][topic\0][JSON payload]
```

**Example Python Client:**
```python
import socket
import struct
import json

# Connect to telemetry stream
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('vacuum', 5555))

while True:
    # Read frame length
    length = struct.unpack('>I', sock.recv(4))[0]
    frame = sock.recv(length)

    # Parse topic and payload
    null_idx = frame.index(0)
    topic = frame[:null_idx].decode('utf-8')
    data = json.loads(frame[null_idx + 1:])

    if topic == "telemetry":
        sensor = data.get("SensorUpdate", {})
        print(f"Battery: {sensor.get('battery_level')}%")
```

See **[sangam-io/PROTOCOL.md](sangam-io/PROTOCOL.md)** for complete protocol specification.

## SangamIO Daemon

**Hardware abstraction daemon for robotic vacuum cleaners with TCP streaming interface**

### Features

- **TCP Streaming**: Real-time telemetry on port 5555, commands on port 5556
- **JSON Wire Format**: Human-readable protocol with configurable binary option
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

### Command Categories

- **Motion Control**: SetVelocity, SetTankDrive, Stop, EmergencyStop
- **Actuator Control**: SetActuator, SetActuatorMultiple
- **Sensor Configuration**: SetSensorConfig, ResetSensor, Enable/Disable
- **Safety Configuration**: SetSafetyLimits, ClearEmergencyStop
- **System Lifecycle**: Sleep, Wake, Shutdown, Restart

### Documentation

- **[sangam-io/PROTOCOL.md](sangam-io/PROTOCOL.md)** - TCP message format and commands
- **[sangam-io/DEPLOYMENT.md](sangam-io/DEPLOYMENT.md)** - Production installation guide
- **[sangam-io/ARCHITECTURE.md](sangam-io/ARCHITECTURE.md)** - Design decisions and extensibility

## Drishti Diagnostic Tool

**Sensor testing and TCP protocol verification**

Drishti provides a visual interface for monitoring robot sensors and testing the TCP protocol. It displays a physically-correct robot diagram with real-time sensor overlays.

```bash
cd drishti
pip install -r requirements.txt
python drishti_ui.py --robot 192.168.68.101
```

See [drishti/README.md](drishti/README.md) for details.

## Project Status

**Current Versions**: SangamIO v0.3.0 | Drishti v2.0.0

| Component | Status | Notes |
|-----------|--------|-------|
| SangamIO Daemon | ✅ Complete | TCP streaming with JSON protocol, ~350KB binary |
| GD32 Driver | ✅ Verified | Automatic heartbeat (20ms), DeviceDriver trait |
| Delta-2D Lidar | ✅ Verified | 5Hz scanning, lock-free streaming |
| Drishti UI | ✅ Complete | Full-screen robot diagram with sensor overlays |
| TCP Protocol | ✅ Complete | JSON over TCP with configurable binary option |
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

- **[TCP Protocol](sangam-io/PROTOCOL.md)** - Complete protocol specification
- **[Deployment Guide](sangam-io/DEPLOYMENT.md)** - Production installation
- **[Architecture](sangam-io/ARCHITECTURE.md)** - Design decisions
- **[Original Research](https://github.com/codetiger/VacuumRobot)** - Protocol reverse engineering work
