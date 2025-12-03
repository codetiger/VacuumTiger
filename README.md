# VacuumTiger

**Open-source firmware for hackable vacuum robots**

VacuumTiger is a modular, configuration-driven firmware stack that makes it easy to build autonomous vacuum robots. Define your sensors and actuators in JSON, and the system handles everything else—from real-time control to network streaming.

## Why VacuumTiger?

- **Configurable**: Define sensors, actuators, and hardware in a single JSON file
- **Extensible**: Add new robot platforms by implementing one trait
- **Generic Protocol**: TCP streaming works with any SLAM application
- **Real-time**: 500Hz sensor updates, <25ms command latency
- **Hackable**: Small codebase (~4K lines Rust), no dependencies on proprietary SDKs

## How It Works

```
┌─────────────────────────────────────────────────┐
│          SLAM Application (Planned)             │
│   Reads hardware.json to discover capabilities  │
└─────────────┬───────────────────────────────────┘
              │ TCP Protocol (Generic)
┌─────────────▼───────────────────────────────────┐
│            SangamIO Daemon                      │
│   Configuration-driven hardware abstraction     │
│   • Streams sensor groups defined in config     │
│   • Routes commands to configured actuators     │
└─────────────┬───────────────────────────────────┘
              │ DeviceDriver Trait
┌─────────────▼───────────────────────────────────┐
│   Robot-Specific Driver (e.g., CRL-200S)        │
│   Implements protocol for specific hardware     │
└─────────────────────────────────────────────────┘
```

## Configuration-Driven Design

Everything is defined in `hardware.json`:

```json
{
  "device": {
    "type": "crl200s",
    "name": "CRL-200S Vacuum Robot",
    "hardware": {
      "gd32_port": "/dev/ttyS3",
      "lidar_port": "/dev/ttyS1",
      "heartbeat_interval_ms": 20
    },
    "sensor_groups": [
      {
        "id": "sensor_status",
        "sensors": [
          { "id": "wheel_left", "type": "U16" },
          { "id": "wheel_right", "type": "U16" },
          { "id": "bumper_left", "type": "Bool" },
          { "id": "cliff_left_front", "type": "Bool" }
        ]
      },
      {
        "id": "lidar",
        "sensors": [
          { "id": "scan", "type": "PointCloud2D" }
        ]
      }
    ]
  },
  "network": {
    "bind_address": "0.0.0.0:5555",
    "wire_format": "json"
  }
}
```

**SLAM applications read this same config** to discover what sensors and actuators are available, enabling truly portable navigation algorithms.

## Generic TCP Protocol

The protocol is robot-agnostic. Any client that speaks the wire format can control any VacuumTiger robot.

### Message Format

```
[4-byte length (big-endian)][JSON payload]
```

### Sensor Streaming (Daemon → Client)

```json
{
  "topic": "sensors/navigation",
  "payload": {
    "type": "SensorGroup",
    "group_id": "navigation",
    "timestamp_us": 1700000000000,
    "values": {
      "wheel_left": { "U16": 12345 },
      "wheel_right": { "U16": 12340 },
      "cliff_left": { "Bool": false },
      "bumper_front": { "Bool": false }
    }
  }
}
```

### Commands (Client → Daemon)

All commands use the unified `ComponentControl` pattern:

```json
{
  "topic": "command",
  "payload": {
    "type": "Command",
    "command": {
      "type": "ComponentControl",
      "id": "drive",
      "action": {
        "type": "Configure",
        "config": { "linear": {"F32": 0.2}, "angular": {"F32": 0.0} }
      }
    }
  }
}
```

### Component Actions

| Action | Description | Example |
|--------|-------------|---------|
| `Enable` | Activate component | `{"type": "Enable"}` |
| `Disable` | Deactivate component | `{"type": "Disable"}` |
| `Reset` | Emergency stop / factory reset | `{"type": "Reset"}` |
| `Configure` | Set parameters (velocity, speed) | `{"type": "Configure", "config": {...}}` |

### Supported Components

| Component | Enable | Disable | Configure |
|-----------|--------|---------|-----------|
| `drive` | Nav mode | Stop | `{linear, angular}` or `{left, right}` |
| `vacuum` | 100% | Off | `{speed: U8}` |
| `main_brush` | 100% | Off | `{speed: U8}` |
| `side_brush` | 100% | Off | `{speed: U8}` |
| `lidar` | Power on | Power off | - |
| `led` | - | - | `{state: U8}` |

## Adding a New Robot Platform

VacuumTiger is designed to support any robot hardware. Here's how to add a new platform:

### 1. Create Configuration

```json
{
  "device": {
    "type": "my_robot",
    "hardware": {
      "motor_port": "/dev/ttyUSB0",
      "heartbeat_interval_ms": 20
    },
    "sensor_groups": [
      {
        "id": "odometry",
        "sensors": [
          { "id": "left_encoder", "type": "I32" },
          { "id": "right_encoder", "type": "I32" }
        ]
      }
    ]
  },
  "network": {
    "bind_address": "0.0.0.0:5555",
    "wire_format": "json"
  }
}
```

### 2. Implement DeviceDriver Trait

```rust
// src/devices/my_robot/mod.rs
pub struct MyRobotDriver { /* ... */ }

impl DeviceDriver for MyRobotDriver {
    fn initialize(
        &mut self,
        sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>
    ) -> Result<()> {
        // Open serial ports
        // Spawn reader threads
        // Update sensor_data in real-time
    }

    fn send_command(&mut self, cmd: Command) -> Result<()> {
        match cmd {
            Command::ComponentControl { id, action } => {
                // Handle drive, vacuum, lidar, etc.
            }
            Command::Shutdown => { /* ... */ }
        }
    }
}
```

### 3. Register in Factory

```rust
// src/devices/mod.rs
pub fn create_device(config: &Config) -> Result<Box<dyn DeviceDriver>> {
    match config.device.device_type.as_str() {
        "crl200s" => Ok(Box::new(CRL200SDriver::new(...))),
        "my_robot" => Ok(Box::new(MyRobotDriver::new(...))),
        _ => Err(Error::UnknownDevice(...)),
    }
}
```

That's it! Your new robot works with any existing SLAM application.

## Repository Structure

```
VacuumTiger/
├── sangam-io/              # Hardware abstraction daemon (Rust)
│   ├── src/
│   │   ├── core/           # DeviceDriver trait, SensorValue types
│   │   ├── devices/        # Robot-specific implementations
│   │   │   └── crl200s/    # CRL-200S driver (GD32 + Delta-2D)
│   │   └── streaming/      # TCP protocol, wire format
│   ├── hardware.json       # Robot configuration
│   ├── COMMANDS.md         # GD32 command reference
│   └── SENSORSTATUS.md     # Sensor packet documentation
├── drishti/                # Diagnostic visualization (Python)
│   ├── drishti_ui.py       # PyQt GUI with sensor overlays
│   ├── drishti.py          # Console client
│   └── ui/                 # PyQt widgets and processors
└── protocol-mitm/          # Reverse engineering tools
```

## Quick Start

### Build SangamIO

```bash
# Install Rust with ARM target
rustup target add armv7-unknown-linux-musleabihf

# Build for robot
cd sangam-io
cargo build --release --target armv7-unknown-linux-musleabihf
```

### Deploy to Robot

```bash
# Copy binary to robot
cat target/armv7-unknown-linux-musleabihf/release/sangam-io | \
  ssh root@vacuum "cat > /usr/sbin/sangamio && chmod +x /usr/sbin/sangamio"

# Run daemon
ssh root@vacuum "RUST_LOG=info /usr/sbin/sangamio"
```

### Connect with Drishti

```bash
cd drishti
pip install -r requirements.txt
python drishti_ui.py --robot 192.168.68.101
```

![Drishti UI](drishti/drishti.png)

## Project Status

| Component | Status | Notes |
|-----------|--------|-------|
| SangamIO Daemon | ✅ Complete | Config-driven, ~350KB binary |
| CRL-200S Driver | ✅ Verified | GD32 motor controller + Delta-2D lidar |
| TCP Protocol | ✅ Complete | Generic JSON streaming |
| Drishti UI | ✅ Complete | Real-time sensor visualization |
| SLAM Application | 📋 Planned | Navigation and mapping |
| Additional Platforms | 📋 Planned | Roomba, Turtlebot, etc. |

## Contributing

Help us build the best open-source vacuum robot firmware:

- **New platforms**: Implement DeviceDriver for your robot
- **SLAM**: Build navigation algorithms using the generic protocol
- **Sensors**: Add support for new lidar models, IMUs
- **Documentation**: Improve guides and examples

## Safety Notice

⚠️ This firmware controls physical hardware. Always:
- Test in a safe, controlled environment
- Keep emergency stop accessible
- Monitor battery voltage during operation

## License

Apache License 2.0. See [LICENSE](LICENSE) for details.

## Links

- [Original Research](https://github.com/codetiger/VacuumRobot)
