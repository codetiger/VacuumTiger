# Drishti - Robot Diagnostic Tool

PyQt-based monitoring tool for testing sensor diagnosis and TCP protocol communication with SangamIO.

## Installation

```bash
cd drishti
pip install -r requirements.txt
```

## Usage

### GUI Application

```bash
python drishti_ui.py --robot 192.168.68.101
```

Options:
- `--robot` - Robot IP address (default: 192.168.68.101)
- `--port` - TCP port (default: 5555)
- `--verbose` - Enable debug logging

### Console Demo

```bash
python drishti.py --robot 192.168.68.101
```

## Features

- Full-screen robot diagram with sensor overlays
- Real-time telemetry monitoring (500Hz)
- JSON protocol over TCP
- Visual indicators for all 13 sensors
- Drive control with arrow keys
- Actuator control (vacuum, brushes, lidar)

![Drishti UI](drishti.png)

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `M` | Toggle drive motor (enable/disable) |
| `L` | Toggle lidar |
| `V` | Toggle vacuum |
| `B` | Toggle main brush |
| `S` | Toggle side brush |
| `↑` | Drive forward (when motor enabled) |
| `↓` | Drive backward |
| `←` | Turn left |
| `→` | Turn right |
| `Space` | Emergency stop |
| `Esc` | Quit |

## TCP Protocol

Connects to SangamIO on port 5555 for telemetry stream.

### Message Format

Length-prefixed JSON:
```
[4-byte length (big-endian)][JSON payload]
```

### Receiving Sensor Data

```json
{
  "topic": "sensors/gd32_status",
  "payload": {
    "type": "SensorGroup",
    "group_id": "gd32_status",
    "timestamp_us": 1700000000000,
    "values": {
      "is_charging": {"Bool": false},
      "wheel_left": {"U16": 12345},
      "bumper_left": {"Bool": false}
    }
  }
}
```

### Sending Commands

All commands use the `ComponentControl` pattern:

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
        "config": {
          "linear": {"F32": 0.2},
          "angular": {"F32": 0.0}
        }
      }
    }
  }
}
```

### Command Examples

**Enable drive motor:**
```json
{"type": "ComponentControl", "id": "drive", "action": {"type": "Enable"}}
```

**Set velocity:**
```json
{"type": "ComponentControl", "id": "drive", "action": {"type": "Configure", "config": {"linear": {"F32": 0.5}, "angular": {"F32": 0.0}}}}
```

**Enable lidar:**
```json
{"type": "ComponentControl", "id": "lidar", "action": {"type": "Enable"}}
```

**Set vacuum to 75%:**
```json
{"type": "ComponentControl", "id": "vacuum", "action": {"type": "Configure", "config": {"speed": {"U8": 75}}}}
```

## Troubleshooting

**No data received:**
- Verify SangamIO is running on robot
- Check network connectivity: `ping 192.168.68.101`
- Verify port 5555 is accessible

**Connection refused:**
- Robot may not be running SangamIO
- Check with: `ssh root@vacuum "ps aux | grep sangam"`

**Drive not responding:**
- Make sure motor is enabled first (press `M`)
- Check status bar for "Drive ENABLED"
