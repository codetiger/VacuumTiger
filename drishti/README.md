# Drishti - Robot Diagnostic Tool

PyQt5-based real-time visualization and control client for the CRL-200S robotic vacuum with two operating modes:

- **Sensor Mode**: Connect to SangamIO for raw sensor visualization (robot view)
- **SLAM Mode**: Connect to DhruvaSLAM for map visualization and autonomous navigation

## Installation

```bash
cd drishti
python -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -r requirements.txt
```

### Dependencies

- `protobuf>=4.0` - Protocol buffer messaging
- `PyQt5>=5.15.0` - GUI framework
- `pyqtgraph>=0.13.0` - High-performance plotting/visualization
- `PyOpenGL>=3.1.6` - 3D graphics via OpenGL
- `numpy>=1.21.0` - Numerical computing

## Usage

### Sensor Mode

Connect to SangamIO for raw sensor data visualization:

```bash
python drishti_ui.py sensor --robot 192.168.68.101
```

Options:
| Option | Default | Description |
|--------|---------|-------------|
| `--robot` | 192.168.68.101 | SangamIO hostname or IP |
| `--port` | 5555 | SangamIO port (TCP+UDP) |
| `--log-raw-packets` | disabled | Log raw GD32 status packets |
| `--verbose` | disabled | Enable debug logging |

### SLAM Mode

Connect to DhruvaSLAM for SLAM visualization and control:

```bash
python drishti_ui.py slam --host localhost
```

Options:
| Option | Default | Description |
|--------|---------|-------------|
| `--host` | localhost | DhruvaSLAM hostname |
| `--port` | 5557 | DhruvaSLAM port (TCP+UDP) |
| `--command-port` | same as --port | Optional separate command port |
| `--verbose` | disabled | Enable debug logging |

### Console Demo (Legacy)

```bash
python drishti.py --robot 192.168.68.101 --verbose
```

## Features

### Sensor Mode

**3D Visualization:**
- Full-screen OpenGL wireframe robot model
- Animated wheels (spin based on encoder data)
- Animated lidar turret (spins when enabled)
- Animated brushes (main and side, when spinning)
- Animated vacuum fan (when active)
- Bumper indicators (green normal, red triggered)
- Cliff sensor indicators (4 pairs)
- Battery bar with charging animation
- IMU orientation overlay (roll/pitch/yaw)
- Multiple camera presets (iso, top, front, side)
- Auto-rotate camera when idle

**Hardware Control:**
- Drive motor enable/disable with speed control
- Lidar enable/disable with scan counter
- Vacuum pump speed 0-100%
- Main brush speed 0-100%
- Side brush speed 0-100%
- Emergency stop (immediate watchdog reset)

**Sensor Display:**
- Battery level + charging state
- Bumper state (left/right)
- Cliff sensors (4 pairs)
- Wheel encoder ticks
- Lidar range and quality
- IMU gyro/accelerometer data
- Motor current, voltage status
- Error codes and diagnostics

![Drishti UI](drishti.png)

### SLAM Mode

**Mapping:**
- Start/stop mapping sessions
- Clear current map
- Save maps to persistent storage
- List all saved maps with metadata (size, creation time, area)
- Enable/rename/delete saved maps

**Visualization:**
- Real-time occupancy grid (10Hz update)
- Robot pose display (x, y, heading)
- Trajectory trail (2000-point history)
- Current lidar scan overlay
- Navigation path visualization
- Goal marker display
- Grid overlay with axis labels

**Navigation:**
- Set goal via Ctrl+Click on map
- Visualize planned path (waypoints)
- Track navigation progress (distance remaining, ETA)
- Cancel active navigation goal

**Camera Control:**
- Auto-follow mode (centers on robot)
- Manual pan (click+drag)
- Manual zoom (mouse wheel)

## Keyboard Shortcuts

### Sensor Mode

| Key | Action |
|-----|--------|
| `M` | Toggle drive motor (enable/disable) |
| `L` | Toggle lidar |
| `V` | Toggle vacuum pump |
| `B` | Toggle main brush |
| `S` | Toggle side brush |
| `↑` | Drive forward |
| `↓` | Drive backward |
| `←` | Turn left |
| `→` | Turn right |
| `↑+←` | Drive forward while turning left |
| `↑+→` | Drive forward while turning right |
| `↓+←` | Drive backward while turning left |
| `↓+→` | Drive backward while turning right |
| `Space` | Emergency stop (immediate reset) |
| `Esc` | Quit application |

### SLAM Mode

| Key/Action | Description |
|------------|-------------|
| `Ctrl+Click` | Set navigation goal on map |
| Mouse wheel | Zoom map in/out |
| Click+drag | Pan map view |
| `Esc` | Quit application |

## Architecture

### Directory Structure

```
drishti/
├── drishti_ui.py              # Entry point (dual-mode launcher)
├── drishti.py                 # Legacy console client
├── debug_proto.py             # Proto message debug tool
├── requirements.txt           # Python dependencies
├── proto/                     # Protocol buffer definitions
│   ├── sangamio_pb2.py       # SangamIO protobuf (generated)
│   └── dhruva_pb2.py         # DhruvaSLAM protobuf (generated)
└── ui/
    ├── main_window.py        # Sensor mode main window
    ├── slam_main_window.py   # SLAM mode main window
    ├── threads/
    │   ├── telemetry_thread.py    # SangamIO data (TCP+UDP)
    │   ├── slam_thread.py         # DhruvaSLAM data (TCP)
    │   └── odometry_thread.py     # Legacy SLAM odometry
    ├── widgets/
    │   ├── robot_3d_view.py       # 3D wireframe visualization
    │   ├── robot_3d_geometry.py   # 3D geometry primitives
    │   ├── lidar_scan_overlay.py  # Polar lidar display
    │   ├── slam_view.py           # Map + trajectory view
    │   ├── control_panel.py       # Sensor mode controls
    │   ├── slam_control_panel.py  # SLAM mode controls
    │   ├── navigation_panel.py    # Navigation status
    │   ├── diagnostics_panel.py   # Timing diagnostics
    │   ├── compact_actuator.py    # Actuator controls
    │   └── collapsible_group.py   # Collapsible sections
    └── processors/
        └── imu_processor.py       # Mahony AHRS filter
```

### Thread Model

**Sensor Mode (TelemetryThread):**
1. Connects via TCP (port 5555) for registration
2. Receives UDP unicast sensor data (port 5555)
3. TCP connection kept alive for sending commands
4. Emits: `sensor_data_received(dict)`, `connection_status(bool, str)`

**SLAM Mode (SlamThread):**
1. Connects via TCP (port 5557, unified architecture)
2. Two separate connections: streaming (receive-only) and commands (request/response)
3. Emits: `robot_status_received`, `sensor_status_received`, `current_map_received`, `map_list_received`, `navigation_status_received`

## Network Protocol

### Sensor Mode (SangamIO)

Uses hybrid UDP/TCP architecture on same port (5555):
- **UDP**: Sensor streaming (low latency, no head-of-line blocking)
- **TCP**: Commands (reliable delivery)

**Client Registration:**
1. Client connects TCP to port 5555
2. Server records client IP address
3. UDP packets sent to client on same port
4. When TCP disconnects, UDP streaming stops

**Message Format:** Length-prefixed Protobuf
```
[4-byte length (big-endian)][Protobuf payload]
```

**UDP Topics (Outbound @ 110Hz):**
- `sensors/sensor_status`: All 13 sensors
- `sensors/device_version`: Version info (one-time)
- `sensors/lidar`: LidarScan @ 5Hz

**TCP Commands (Inbound):**
- `ComponentControl`: Enable/disable/configure components
- `ProtocolSync`: Synchronization
- `Shutdown`: Graceful shutdown

**Sensor Data Available:**
| Sensor | Type | Description |
|--------|------|-------------|
| `is_charging` | bool | Charging state |
| `battery_level` | u8 | Battery percentage (0-100) |
| `bumper_left` | bool | Left bumper triggered |
| `bumper_right` | bool | Right bumper triggered |
| `cliff_left` | bool | Left cliff detected |
| `cliff_center` | bool | Center cliff detected |
| `cliff_right` | bool | Right cliff detected |
| `wheel_left` | u32 | Left encoder ticks |
| `wheel_right` | u32 | Right encoder ticks |
| `lidar_speed` | u16 | Lidar RPM (0 when disabled) |
| `gyro_x/y/z` | i16 | Gyroscope (0.1°/s units) |
| `tilt_x/y/z` | i16 | Accelerometer (mG units) |

**Command Examples:**

Enable drive motor:
```python
cmd = sangamio_pb2.Command()
cmd.component_control.id = "drive"
cmd.component_control.action.type = ComponentAction.ENABLE
```

Set velocity:
```python
cmd.component_control.id = "drive"
cmd.component_control.action.type = ComponentAction.CONFIGURE
cmd.component_control.action.config["linear"].f32_val = 0.2
cmd.component_control.action.config["angular"].f32_val = 0.0
```

### SLAM Mode (DhruvaSLAM)

Uses unified port architecture (default: 5557) for both streaming and commands.

**Protocol:** Length-prefixed Protobuf (see `dhruva-slam/proto/dhruva.proto`)

**DhruvaStream (Inbound):**
| Message | Rate | Content |
|---------|------|---------|
| `robot_status` | 10Hz | Pose, state, battery, mapping progress |
| `sensor_status` | 10Hz | Lidar, encoders, IMU |
| `current_map` | 1Hz | Occupancy grid |
| `map_list` | on change | Saved maps |
| `navigation_status` | 5Hz | Path, goal, progress |

**DhruvaCommand (Outbound):**
| Command | Description |
|---------|-------------|
| `StartMapping(map_name)` | Begin SLAM session |
| `StopMapping(save)` | End session, optionally save |
| `ClearMap()` | Discard current map |
| `EnableMap(map_id)` | Load saved map |
| `RenameMap(map_id, new_name)` | Rename saved map |
| `DeleteMap(map_id)` | Delete saved map |
| `SetGoal(x, y, theta)` | Set navigation goal |
| `CancelGoal()` | Stop navigation |
| `EmergencyStop()` | Halt all motion |

**Occupancy Grid Encoding:**
| Value | Meaning | Display Color |
|-------|---------|---------------|
| 0 | Free space | Light gray |
| 100 | Occupied | White |
| 255 | Unknown | Dark gray |

## IMU Processing

The `imu_processor.py` implements a **Mahony AHRS filter** with quaternion-based orientation estimation.

**Calibration:**
- Collects bias samples for ~3 seconds (330 samples @ 110Hz)
- Computes average gyro bias to subtract from measurements
- Returns zeros during calibration phase

**Filter Parameters:**
- `KP = 5.0`: Proportional gain for fast settling
- `KI = 0.1`: Integral gain for bias correction

**Output:** Roll, pitch, yaw in degrees (ROS REP-103 convention)

## Coordinate Conventions

All coordinates use **ROS REP-103** standard:

```
      +X (forward)
       ↑
       |
←------+------→ +Y (left)
       |
    +Z (up, out of page)

Angles: 0 rad = +X direction, increases counter-clockwise
```

- **Roll**: Rotation around X (left/right tilt)
- **Pitch**: Rotation around Y (nose up/down)
- **Yaw**: Rotation around Z (heading, CCW positive)

## Debug Utilities

### debug_proto.py

Capture and analyze DhruvaSLAM proto messages:

```bash
python debug_proto.py
```

- Connects to localhost:5557
- Sends StartMapping command
- Analyzes received proto messages
- Saves raw bytes to `/tmp/proto_message.bin`
- Useful for debugging protocol issues

## Troubleshooting

### Sensor Mode

**No data received:**
- Verify SangamIO is running: `ssh root@vacuum "ps aux | grep sangam"`
- Check network: `ping 192.168.68.101`
- Verify port 5555 is accessible (both TCP and UDP)
- Check firewall allows UDP on port 5555

**Connection refused:**
- Robot may not be running SangamIO
- AuxCtrl may still be running (rename it first)

**Drive not responding:**
- Make sure motor is enabled first (press `M`)
- Check status bar for "Drive ENABLED"
- Verify no error codes displayed

**3D view not rendering:**
- Ensure PyOpenGL is installed
- Check OpenGL drivers are up to date

### SLAM Mode

**No map data:**
- Verify DhruvaSLAM is running: `ps aux | grep dhruva`
- Check connection status in UI status bar
- Start mapping session using control panel

**Parse errors:**
- Failed proto messages saved to `/tmp/failed_proto.bin`
- Use `debug_proto.py` to analyze protocol issues

**Navigation not working:**
- Ensure a map is loaded (Enable Map)
- Robot must be localized first
- Check localization confidence indicator

**Map not updating:**
- Verify mapping is active (state = "Mapping")
- Check trajectory is being drawn
- Large maps may have slower update rate

## UI Components

### Control Panel (Sensor Mode)

Collapsible sections:
- **Motion Control**: Drive motor enable + speed slider
- **Cleaning Systems**: Vacuum, main brush, side brush
- **Sensors**: Lidar toggle, cliff calibration
- **System**: LED control, dustbox status
- **Calibration**: IMU, compass calibration
- **Power Management**: Voltage displays

### SLAM Control Panel

- **Status Display**: State, active map, pose, confidence
- **Mapping Controls**: Start/Stop/Clear/Emergency Stop
- **Map Management**: Enable/Rename/Delete
- **Map List**: Scrollable list with metadata

### Navigation Panel

- Current state (Idle, Planning, Navigating, Goal Reached)
- Target description
- Distance remaining + ETA
- Progress bar
- Waypoint counter
- Cancel Navigation button
