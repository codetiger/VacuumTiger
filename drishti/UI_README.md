# Drishti UI - Robot Vacuum Visualization

Graphical user interface for monitoring and controlling VacuumTiger robots.

## Features

### 1. Robot Outline Visualization (Top-Left)
- **Lidar Turret**: Rotating indicator showing scan direction
- **Drive Wheels**: Animated when encoders change (left/right)
- **Cliff Sensors** (4x): Color-coded by distance (Green=safe, Yellow=caution, Red=cliff)
- **Front Bumper**: Flashes red when pressed
- **Front IR Sensor**: Color-coded beam showing obstacle distance
- **Battery Status**: Level percentage with charging indicator
- **Brushes**: Animated when active (main roller + side brush)
- **Button Panel**: Start/Dock buttons with LED indicators
- **Encoder Values**: Real-time tick counts displayed

### 2. Lidar Point Cloud (Middle)
- 360° polar plot showing lidar scan data
- Updates at ~5 Hz (native scan rate)
- Color-coded by signal quality (Green=good, Red=poor)
- ~300-400 points per scan
- Robot position marked at center

### 3. Control Panel (Top-Right)
- **Motion Control**: Directional buttons (Forward/Back/Left/Right/Stop)
- **Actuators**: Sliders for side brush, main brush, air pump (0-100%)
- **Lidar Control**: Enable/disable with PWM speed control
- **Emergency Stop**: Prominent red button

### 4. Telemetry Graphs (Bottom)
- **Battery Level**: Real-time percentage over 30-second window
- **Encoders**: Left/right wheel tick counts
- **Connection Quality**: Success rate percentage

## Usage

### Basic Start
```bash
cd drishti
source venv/bin/activate
python drishti_ui.py
```

### Connect to Specific Robot
```bash
python drishti_ui.py --robot 192.168.68.101
```

### With Verbose Logging
```bash
python drishti_ui.py --robot 192.168.68.101 --verbose
```

### Custom Ports
```bash
python drishti_ui.py --robot 192.168.68.101 --pub-port 5555 --cmd-port 5556
```

## Robot Visualization Details

Based on iLife A11/Xiaomi-style circular vacuum robot design:

**Dimensions (Scale)**:
- Robot diameter: 300px (represents 320mm actual)
- Robot outline: Semi-transparent white with gray border
- Components positioned accurately based on reference images

**Sensor Positions**:
- Lidar: Center-top
- Cliff sensors: 4 corners (FL, FR, RL, RR)
- Front bumper: 180° arc covering front half
- Front IR: Center-front
- Wheels: Left/right sides
- Main brush: Center-bottom
- Side brush: Right-bottom

**Color Coding**:
- Green: Safe/good/active
- Yellow: Caution/warning
- Red: Alert/pressed/error
- Cyan: Lidar indicator
- Orange: Active brushes

## Performance

- **UI Update Rate**: 30 FPS for smooth animations
- **Telemetry Rate**: Downsampled from 500 Hz to 30 Hz for display
- **Lidar Update**: Native 5 Hz scan rate
- **Responsiveness**: Immediate feedback on all controls

## Requirements

- Python 3.8+
- PyQt5 (GUI framework)
- pyqtgraph (high-performance plotting)
- numpy (numerical computations)
- msgpack (already installed for drishti.py)

All dependencies are listed in `requirements.txt` and installed via pip.

## Architecture

```
drishti_ui.py               # Main entry point
ui/
├── main_window.py          # Main window with 4-panel layout
├── threads/
│   └── telemetry_thread.py # Background data reception
└── widgets/
    ├── robot_outline_widget.py   # Top-down robot visualization
    ├── lidar_widget.py           # Point cloud display
    ├── control_panel.py          # Control buttons/sliders
    └── telemetry_graphs.py       # Real-time graphs
```

## Keyboard Shortcuts

- **Ctrl+C**: Quit application (in terminal)
- **Close window**: Cleanly disconnects from robot

## Troubleshooting

### "Connection Failed"
- Verify robot is running SangamIO (`ps aux | grep sangamio`)
- Check network connectivity (`ping 192.168.68.101`)
- Ensure ports 5555 and 5556 are accessible

### "No Lidar Data"
- Enable lidar using the control panel
- Verify lidar is powered on (physical check)
- Check SangamIO logs for lidar errors

### "Module Not Found"
- Ensure virtual environment is activated: `source venv/bin/activate`
- Reinstall dependencies: `pip install -r requirements.txt`

### Slow Performance
- Close other applications
- Reduce graph window (built-in: 30 seconds)
- Check network latency to robot

## Development

The UI is built with:
- **PyQt5**: Native desktop GUI framework
- **pyqtgraph**: OpenGL-accelerated plotting (faster than matplotlib)
- **Threading**: Background telemetry reception, separate from UI thread
- **Signals/Slots**: Thread-safe communication between components

All widgets are modular and can be tested independently.

## Future Enhancements

Potential features (not yet implemented):
- Data recording/playback
- Odometry trajectory visualization
- Map building from lidar scans
- Autonomous navigation integration
- Multi-robot monitoring

## Notes

- The original `drishti.py` client library is unchanged - UI is a layer on top
- All sensor data comes from MessagePack arrays (not dicts) - handled automatically
- Connection errors are non-fatal - application continues running
- Emergency stop affects all motors including lidar
