# SangamIO

Hardware abstraction daemon for the CRL-200S robotic vacuum platform, providing real-time sensor streaming via UDP and command control via TCP.

## Overview

SangamIO acts as a bridge between high-level clients (SLAM, visualization) and low-level hardware (GD32 motor controller, Delta-2D lidar). It runs as a standalone daemon on the robot, streaming sensor data at 110Hz via UDP and accepting commands over TCP.

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

### Key Specifications

| Property | Value |
|----------|-------|
| Language | Rust 2024 edition |
| Version | 0.3.0 |
| Target | ARM (armv7-unknown-linux-musleabihf) or host (simulation) |
| Binary Size | ~350KB (statically linked) |
| Memory Usage | <10MB RSS |
| CPU Usage | <1% on Allwinner A33 |

## Building

### Development (Host Machine)

```bash
# Build for real hardware
cargo build --release --target armv7-unknown-linux-musleabihf

# Build for simulation (mock device)
cargo build --release --features mock

# Run with configuration file
cargo run --release --features mock -- mock.toml
cargo run --release --features mock -- --config mock.toml

# Enable verbose logging
RUST_LOG=debug cargo run --release --features mock

# Run tests
cargo test --features mock
```

### Production (ARM Robot)

```bash
# Add ARM target (one-time)
rustup target add armv7-unknown-linux-musleabihf

# Build for ARM (real hardware)
cargo build --release --target armv7-unknown-linux-musleabihf

# Strip debug symbols (reduces size ~40%)
arm-linux-gnueabihf-strip \
  target/armv7-unknown-linux-musleabihf/release/sangam-io
```

### Feature Flags

| Feature | Description |
|---------|-------------|
| (default) | Real hardware support only |
| `mock` | Enable mock device simulation |

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

## Network Protocol

SangamIO uses a hybrid UDP/TCP architecture for optimal performance:

| Channel | Protocol | Purpose | Why |
|---------|----------|---------|-----|
| Sensor data | UDP unicast | Low-latency streaming | No head-of-line blocking |
| Commands | TCP | Reliable control | Guaranteed delivery |

### Message Format

Both UDP and TCP use length-prefixed Protobuf framing:

```
┌──────────────────┬─────────────────────┐
│ Length (4 bytes) │ Protobuf Payload    │
│ Big-endian u32   │ (binary)            │
└──────────────────┴─────────────────────┘
```

See `proto/sangamio.proto` for the complete schema.

### Topics (UDP Streaming)

| Topic | Rate | Size | Description |
|-------|------|------|-------------|
| `sensors/sensor_status` | 110Hz | ~150B | Encoders, IMU, bumpers, cliffs, battery |
| `sensors/lidar` | 5Hz | ~2KB | 360° point cloud (angle, distance, quality) |
| `sensors/device_version` | Once | ~50B | Firmware version info |

### Commands (TCP)

| Command | Description |
|---------|-------------|
| `ComponentControl::drive` | Set linear/angular velocity |
| `ComponentControl::lidar` | Enable/disable lidar motor |
| `ComponentControl::vacuum` | Control vacuum motor (0-100%) |
| `ComponentControl::*_brush` | Control brushes (0-100%) |
| `Shutdown` | Graceful daemon shutdown |

### Client Registration

UDP uses unicast (not broadcast). Clients are registered automatically:

1. Client connects TCP to port 5555
2. Server records client IP address
3. UDP packets are sent to client on same port
4. When TCP disconnects, UDP streaming stops

### Python Client Example

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

## Module Structure

```
sangam-io/
├── src/
│   ├── main.rs              # Entry point, TCP/UDP listener
│   ├── config.rs            # TOML configuration loading
│   ├── error.rs             # Error types
│   │
│   ├── core/
│   │   ├── driver.rs        # DeviceDriver trait
│   │   └── types.rs         # SensorValue, Command types
│   │
│   ├── devices/
│   │   ├── mod.rs           # Device factory (crl200s, mock)
│   │   ├── crl200s/         # Real hardware driver
│   │   │   ├── mod.rs       # CRL200S orchestrator
│   │   │   ├── constants.rs # Hardware constants
│   │   │   ├── COMMANDS.md  # GD32 command reference
│   │   │   ├── SENSORSTATUS.md # Sensor packet docs
│   │   │   ├── gd32/        # Motor controller driver
│   │   │   │   ├── mod.rs       # Driver core
│   │   │   │   ├── commands.rs  # Command handlers
│   │   │   │   ├── heartbeat.rs # 20ms watchdog
│   │   │   │   ├── reader.rs    # Status parsing
│   │   │   │   ├── packet.rs    # Packet encoding
│   │   │   │   ├── protocol.rs  # Packet framing
│   │   │   │   └── state.rs     # Atomic state
│   │   │   └── delta2d/     # Lidar driver
│   │   │       ├── mod.rs       # Driver core
│   │   │       └── protocol.rs  # Packet parsing
│   │   │
│   │   └── mock/            # Simulation driver (--features mock)
│   │       ├── mod.rs           # MockDriver, simulation loop
│   │       ├── config.rs        # SimulationConfig structs
│   │       ├── physics.rs       # Differential drive kinematics
│   │       ├── lidar_sim.rs     # Ray-casting lidar
│   │       ├── imu_sim.rs       # IMU with noise
│   │       ├── encoder_sim.rs   # Encoder with slip
│   │       ├── sensor_sim.rs    # Bumpers, cliffs
│   │       ├── map_loader.rs    # PGM+YAML loader
│   │       └── noise.rs         # Noise generator
│   │
│   └── streaming/
│       ├── mod.rs           # Streaming module docs
│       ├── wire.rs          # Protobuf serialization
│       ├── udp_publisher.rs # UDP sensor streaming
│       └── tcp_receiver.rs  # TCP command handling
│
├── proto/
│   └── sangamio.proto       # Protobuf schema
├── maps/                    # Example simulation maps
│   ├── example.yaml         # Map metadata
│   └── example.pgm          # Occupancy grid
├── sangamio.toml            # Hardware configuration
└── mock.toml                # Simulation configuration
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
| `cliff_left_side`, `cliff_left_front` | Bool | Left cliff sensors |
| `cliff_right_front`, `cliff_right_side` | Bool | Right cliff sensors |
| `is_charging` | Bool | Charging state |
| `battery_voltage` | F32 | Battery voltage (V) |
| `battery_level` | U8 | Estimated charge (0-100%) |
| `wheel_left`, `wheel_right` | U16 | Encoder ticks |
| `gyro_x`, `gyro_y`, `gyro_z` | I16 | Angular velocity (Roll, Pitch, Yaw after transform) |
| `accel_x`, `accel_y`, `accel_z` | I16 | Acceleration (raw units) |
| `tilt_x`, `tilt_y`, `tilt_z` | I16 | LP-filtered gravity vector |
| `start_button`, `dock_button` | U16 | Button press states |
| `dustbox_attached` | Bool | Dustbox present |
| `is_dock_connected` | Bool | On charging dock |

> **Note**: IMU values are in ROS REP-103 frame after applying `frame_transforms`.

## Thread Model

### Real Hardware (crl200s)

| Thread | Purpose | Timing |
|--------|---------|--------|
| Main | TCP listener, client accept | - |
| GD32 Heartbeat | Safety watchdog | 20ms ±2ms |
| GD32 Reader | Status parsing → channel | Continuous |
| Lidar Reader | Scan accumulation | Continuous |
| UDP Publisher | Sensor streaming | <1ms loop |
| TCP Receiver | Per-client commands | On-demand |

### Simulation (mock)

| Thread | Purpose | Timing |
|--------|---------|--------|
| Main | TCP listener, client accept | - |
| Simulation Loop | Physics + sensors | 9ms (110Hz) |
| UDP Publisher | Sensor streaming | <1ms loop |
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

## Mock Device Simulation

The mock device driver enables algorithm development and testing without physical hardware.

### Quick Start

```bash
# Build with mock feature
cargo build --release --features mock

# Run simulation
cargo run --release --features mock -- mock.toml
```

### Map Format (ROS Standard)

Maps use PGM + YAML format compatible with ROS Navigation:

**maps/example.yaml:**
```yaml
image: example.pgm          # Grayscale occupancy grid
resolution: 0.02            # meters per pixel
origin: [-5.0, -5.0, 0.0]   # [x, y, yaw] of bottom-left
occupied_thresh: 0.65       # Darker = occupied
cliff_mask: cliffs.pgm      # Optional cliff layer
```

**PGM pixel values:**
- White (255) = Free space
- Black (0) = Wall/obstacle
- Gray (205) = Unknown

### Configuration

**mock.toml:**
```toml
[device]
type = "mock"
name = "Mock CRL-200S"

[device.simulation]
map_file = "maps/example.yaml"
start_x = 1.5               # meters
start_y = 3.5               # meters
start_theta = 0.0           # radians
speed_factor = 1.0          # 2.0 = 2x speed
random_seed = 42            # 0 = random each run

[device.simulation.robot]
wheel_base = 0.233          # meters
ticks_per_meter = 4464.0
collision_mode = "stop"     # "stop", "slide", "passthrough"

[device.simulation.lidar]
num_rays = 360
scan_rate_hz = 5.0
max_range = 8.0

[device.simulation.lidar.noise]
range_stddev = 0.005        # 5mm noise
miss_rate = 0.01            # 1% invalid readings

[network]
bind_address = "0.0.0.0:5555"
```

### Simulated Components

| Component | Method | Configurability |
|-----------|--------|-----------------|
| Lidar | Ray-casting | Noise, miss rate, quality |
| Encoders | Kinematics | Slip, quantization |
| IMU | Physics-based | Per-axis noise, drift |
| Bumpers | Collision detect | Zone angles, trigger distance |
| Cliffs | Mask lookup | Sensor positions |
| Battery | Fixed values | Voltage, level, charging |

### Speed Factor

Accelerate simulation for faster testing:

| Factor | sensor_status | lidar | Wall time for 1 min sim |
|--------|---------------|-------|-------------------------|
| 1.0 | 110 Hz | 5 Hz | 60 sec |
| 2.0 | 220 Hz | 10 Hz | 30 sec |
| 5.0 | 550 Hz | 25 Hz | 12 sec |

### Creating Maps

1. **GIMP/Photoshop**: Draw black (walls) on white (free space)
2. **ROS map_server**: Save maps from SLAM runs
3. **Cartographer**: Export as PGM
4. Save as 8-bit grayscale PGM with YAML metadata

## Documentation

### CRL-200S Hardware
- [src/devices/crl200s/COMMANDS.md](src/devices/crl200s/COMMANDS.md) - GD32 command reference (27 commands)
- [src/devices/crl200s/SENSORSTATUS.md](src/devices/crl200s/SENSORSTATUS.md) - Status packet byte layout (96 bytes)

### Mock Device Simulation
- [docs/mock-device-guide.md](docs/mock-device-guide.md) - Mock device user guide
- [docs/proposal-mock-device.md](docs/proposal-mock-device.md) - Mock device design document

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
