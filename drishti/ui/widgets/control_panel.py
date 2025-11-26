"""
Control panel widget for Drishti robot visualization.
Provides UI controls for vacuum, side brush, and main brush with speed control.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QPushButton, QGroupBox, QFrame, QGridLayout, QSpinBox
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QKeyEvent


class ActuatorControl(QWidget):
    """Individual actuator control with ON/OFF toggle and speed slider."""

    # Signal: (actuator_id, speed_percent)
    command_changed = pyqtSignal(str, float)

    def __init__(self, name: str, actuator_id: str, parent=None):
        super().__init__(parent)
        self.name = name
        self.actuator_id = actuator_id
        self.enabled = False
        self.speed = 0  # 0, 25, 50, 75, 100

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # Header with name and toggle button
        header = QHBoxLayout()

        name_label = QLabel(self.name)
        name_label.setFont(QFont("Arial", 10, QFont.Bold))
        header.addWidget(name_label)

        header.addStretch()

        self.toggle_btn = QPushButton("OFF")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setFixedWidth(50)
        self.toggle_btn.clicked.connect(self._on_toggle)
        self._update_toggle_style()
        header.addWidget(self.toggle_btn)

        layout.addLayout(header)

        # Speed slider (snapped to 25% increments)
        slider_layout = QHBoxLayout()

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(4)  # 0, 1, 2, 3, 4 -> 0%, 25%, 50%, 75%, 100%
        self.slider.setValue(0)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setTickInterval(1)
        self.slider.valueChanged.connect(self._on_slider_changed)
        slider_layout.addWidget(self.slider)

        self.speed_label = QLabel("0%")
        self.speed_label.setFixedWidth(35)
        self.speed_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        slider_layout.addWidget(self.speed_label)

        layout.addLayout(slider_layout)

    def _on_toggle(self):
        self.enabled = self.toggle_btn.isChecked()
        self._update_toggle_style()

        # When toggling ON with slider at 0, set to 100%
        if self.enabled and self.speed == 0:
            self.speed = 100
            self.slider.setValue(4)  # 4 * 25 = 100%
            self.speed_label.setText("100%")

        self._emit_command()

    def _on_slider_changed(self, value):
        self.speed = value * 25
        self.speed_label.setText(f"{self.speed}%")

        # If slider is moved above 0, enable; if moved to 0, disable
        if self.speed > 0 and not self.enabled:
            self.enabled = True
            self.toggle_btn.setChecked(True)
            self._update_toggle_style()
        elif self.speed == 0 and self.enabled:
            self.enabled = False
            self.toggle_btn.setChecked(False)
            self._update_toggle_style()

        self._emit_command()

    def _update_toggle_style(self):
        if self.enabled:
            self.toggle_btn.setText("ON")
            self.toggle_btn.setStyleSheet(
                "background-color: #4CAF50; color: white; font-weight: bold;"
            )
        else:
            self.toggle_btn.setText("OFF")
            self.toggle_btn.setStyleSheet(
                "background-color: #666; color: white;"
            )

    def _emit_command(self):
        # Emit actual speed: if disabled, send 0; otherwise send the speed
        actual_speed = float(self.speed) if self.enabled else 0.0
        self.command_changed.emit(self.actuator_id, actual_speed)


class LidarControl(QWidget):
    """Widget for lidar control with toggle button and packet count."""

    # Signal: (enabled)
    lidar_toggled = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.packet_count = 0
        self.enabled = False
        self._setup_ui()

        # Timeout timer for data freshness
        self.timeout_timer = QTimer(self)
        self.timeout_timer.timeout.connect(self._on_timeout)
        self.timeout_timer.setSingleShot(True)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)

        # Header with name and toggle button
        header = QHBoxLayout()

        title = QLabel("Lidar")
        title.setFont(QFont("Arial", 10, QFont.Bold))
        header.addWidget(title)

        header.addStretch()

        self.toggle_btn = QPushButton("OFF")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setFixedWidth(50)
        self.toggle_btn.clicked.connect(self._on_toggle)
        self._update_toggle_style()
        header.addWidget(self.toggle_btn)

        layout.addLayout(header)

        # Status row with icon indicator
        status_layout = QHBoxLayout()
        status_layout.setSpacing(8)

        # Status indicator dot
        self.status_dot = QLabel()
        self.status_dot.setFixedSize(12, 12)
        self.status_dot.setStyleSheet(
            "background-color: #666; border-radius: 6px;"
        )
        status_layout.addWidget(self.status_dot)

        # Status text
        self.status_label = QLabel("No data")
        self.status_label.setStyleSheet("color: #888;")
        status_layout.addWidget(self.status_label)

        status_layout.addStretch()
        layout.addLayout(status_layout)

        # Packet count
        self.count_label = QLabel("Packets: 0")
        self.count_label.setStyleSheet("color: #888; font-size: 9px;")
        layout.addWidget(self.count_label)

    def _on_toggle(self):
        self.enabled = self.toggle_btn.isChecked()
        self._update_toggle_style()
        self.lidar_toggled.emit(self.enabled)

    def _update_toggle_style(self):
        if self.enabled:
            self.toggle_btn.setText("ON")
            self.toggle_btn.setStyleSheet(
                "background-color: #4CAF50; color: white; font-weight: bold;"
            )
        else:
            self.toggle_btn.setText("OFF")
            self.toggle_btn.setStyleSheet(
                "background-color: #666; color: white;"
            )

    def set_enabled(self, enabled: bool):
        """Set lidar enabled state (from keyboard shortcut)."""
        self.enabled = enabled
        self.toggle_btn.setChecked(enabled)
        self._update_toggle_style()

    def update_status(self, scan_received: bool):
        """Update lidar status when scan data is received."""
        if scan_received:
            self.packet_count += 1
            self.status_label.setText("Active")
            self.status_label.setStyleSheet("color: #333; font-weight: bold;")
            self.status_dot.setStyleSheet(
                "background-color: #4CAF50; border-radius: 6px;"
            )
            self.count_label.setText(f"Packets: {self.packet_count}")
            self.count_label.setStyleSheet("color: #333; font-size: 9px;")

            # Reset timeout timer (2 seconds)
            self.timeout_timer.start(2000)

    def _on_timeout(self):
        """Called when no data received for 2 seconds."""
        self.status_label.setText("No data")
        self.status_label.setStyleSheet("color: #888;")
        self.status_dot.setStyleSheet(
            "background-color: #666; border-radius: 6px;"
        )


class DustboxStatus(QWidget):
    """Widget showing dustbox status as info display."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)

        # Title
        title = QLabel("Dustbox")
        title.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(title)

        # Status row with icon indicator
        status_layout = QHBoxLayout()
        status_layout.setSpacing(8)

        # Status indicator dot
        self.status_dot = QLabel()
        self.status_dot.setFixedSize(12, 12)
        self.status_dot.setStyleSheet(
            "background-color: #4CAF50; border-radius: 6px;"
        )
        status_layout.addWidget(self.status_dot)

        # Status text
        self.status_label = QLabel("Present")
        self.status_label.setStyleSheet("color: #333; font-weight: bold;")
        status_layout.addWidget(self.status_label)

        status_layout.addStretch()
        layout.addLayout(status_layout)

        # Type indicator (placeholder - needs protocol support)
        self.type_label = QLabel("Type: Unknown")
        self.type_label.setStyleSheet("color: #888; font-size: 9px;")
        layout.addWidget(self.type_label)

    def update_status(self, attached: bool, dustbox_type: str = None):
        """Update dustbox status display.

        Args:
            attached: Whether dustbox is present
            dustbox_type: "dry" or "2in1" (optional, may not be available)
        """
        if attached:
            self.status_label.setText("Present")
            self.status_label.setStyleSheet("color: #333; font-weight: bold;")
            self.status_dot.setStyleSheet(
                "background-color: #4CAF50; border-radius: 6px;"
            )
        else:
            self.status_label.setText("Missing")
            self.status_label.setStyleSheet("color: #f44336; font-weight: bold;")
            self.status_dot.setStyleSheet(
                "background-color: #f44336; border-radius: 6px;"
            )

        # Update type if available
        if dustbox_type:
            if dustbox_type == "2in1":
                self.type_label.setText("Type: 2-in-1 (Wet/Dry)")
            else:
                self.type_label.setText("Type: Dry Only")
        else:
            self.type_label.setText("Type: Unknown")


class LEDControl(QWidget):
    """LED mode control with spinbox for 0-255 value selection."""

    # Signal: (led_value)
    command_changed = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # Header
        title = QLabel("LED Mode")
        title.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(title)

        # Spinbox row
        spinbox_layout = QHBoxLayout()

        self.spinbox = QSpinBox()
        self.spinbox.setMinimum(0)
        self.spinbox.setMaximum(20)
        self.spinbox.setValue(0)
        self.spinbox.valueChanged.connect(self._on_value_changed)
        spinbox_layout.addWidget(self.spinbox)

        self.send_btn = QPushButton("Set")
        self.send_btn.setFixedWidth(40)
        self.send_btn.clicked.connect(self._on_send_clicked)
        spinbox_layout.addWidget(self.send_btn)

        layout.addLayout(spinbox_layout)

        # Hint label
        hint = QLabel("0=Off, 1=Blue, 7=Red, 17=Purple")
        hint.setStyleSheet("color: #888; font-size: 9px;")
        hint.setWordWrap(True)
        layout.addWidget(hint)

    def _on_value_changed(self, value: int):
        # Don't auto-send on value change, wait for button click
        pass

    def _on_send_clicked(self):
        self.command_changed.emit(self.spinbox.value())


class DriveControl(QWidget):
    """Arrow key-style drive control widget for robot motion."""

    # Signal: (linear_velocity, angular_velocity)
    velocity_changed = pyqtSignal(float, float)
    # Signal: (enabled)
    motor_toggled = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.linear_velocity = 0.0  # m/s
        self.angular_velocity = 0.0  # rad/s
        self.linear_step = 0.5  # m/s per press
        self.angular_step = 0.5  # rad/s per press
        self.motor_enabled = False
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # Header with title and motor toggle
        header = QHBoxLayout()

        title = QLabel("Drive Control")
        title.setFont(QFont("Arial", 10, QFont.Bold))
        header.addWidget(title)

        header.addStretch()

        self.motor_toggle = QPushButton("OFF")
        self.motor_toggle.setCheckable(True)
        self.motor_toggle.setFixedWidth(50)
        self.motor_toggle.clicked.connect(self._on_motor_toggle)
        self._update_motor_style()
        header.addWidget(self.motor_toggle)

        layout.addLayout(header)

        # Arrow buttons grid
        # Layout:  Row 0:     [▲]
        #          Row 1: [◀] [▼] [▶]
        grid = QGridLayout()
        grid.setSpacing(3)

        # Forward button (up arrow) - row 0, col 1
        self.btn_forward = QPushButton("▲")
        self.btn_forward.setFixedSize(40, 40)
        self.btn_forward.setFont(QFont("Arial", 14))
        self.btn_forward.pressed.connect(self._on_forward_pressed)
        self.btn_forward.released.connect(self._on_released)
        grid.addWidget(self.btn_forward, 0, 1)

        # Left button - row 1, col 0
        self.btn_left = QPushButton("◀")
        self.btn_left.setFixedSize(40, 40)
        self.btn_left.setFont(QFont("Arial", 14))
        self.btn_left.pressed.connect(self._on_left_pressed)
        self.btn_left.released.connect(self._on_released)
        grid.addWidget(self.btn_left, 1, 0)

        # Backward button (down arrow) - row 1, col 1 (same column as forward)
        self.btn_backward = QPushButton("▼")
        self.btn_backward.setFixedSize(40, 40)
        self.btn_backward.setFont(QFont("Arial", 14))
        self.btn_backward.pressed.connect(self._on_backward_pressed)
        self.btn_backward.released.connect(self._on_released)
        grid.addWidget(self.btn_backward, 1, 1)

        # Right button - row 1, col 2
        self.btn_right = QPushButton("▶")
        self.btn_right.setFixedSize(40, 40)
        self.btn_right.setFont(QFont("Arial", 14))
        self.btn_right.pressed.connect(self._on_right_pressed)
        self.btn_right.released.connect(self._on_released)
        grid.addWidget(self.btn_right, 1, 2)

        # Center the grid
        grid_container = QHBoxLayout()
        grid_container.addStretch()
        grid_container.addLayout(grid)
        grid_container.addStretch()
        layout.addLayout(grid_container)

        # Velocity display
        self.velocity_label = QLabel("Lin: 0.00 m/s  Ang: 0.00 rad/s")
        self.velocity_label.setAlignment(Qt.AlignCenter)
        self.velocity_label.setStyleSheet("color: #666; font-size: 9px;")
        layout.addWidget(self.velocity_label)

    def _on_forward_pressed(self):
        if not self.motor_enabled:
            return
        self.linear_velocity = self.linear_step
        self.angular_velocity = 0.0
        self._emit_velocity()

    def _on_backward_pressed(self):
        if not self.motor_enabled:
            return
        self.linear_velocity = -self.linear_step
        self.angular_velocity = 0.0
        self._emit_velocity()

    def _on_left_pressed(self):
        if not self.motor_enabled:
            return
        self.linear_velocity = 0.0
        self.angular_velocity = self.angular_step
        self._emit_velocity()

    def _on_right_pressed(self):
        if not self.motor_enabled:
            return
        self.linear_velocity = 0.0
        self.angular_velocity = -self.angular_step
        self._emit_velocity()

    def _on_released(self):
        if not self.motor_enabled:
            return
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self._emit_velocity()

    def _emit_velocity(self):
        self.velocity_label.setText(
            f"Lin: {self.linear_velocity:.2f} m/s  Ang: {self.angular_velocity:.2f} rad/s"
        )
        self.velocity_changed.emit(self.linear_velocity, self.angular_velocity)

    def _on_motor_toggle(self):
        self.motor_enabled = self.motor_toggle.isChecked()
        self._update_motor_style()
        self.motor_toggled.emit(self.motor_enabled)

    def _update_motor_style(self):
        if self.motor_enabled:
            self.motor_toggle.setText("ON")
            self.motor_toggle.setStyleSheet(
                "background-color: #4CAF50; color: white; font-weight: bold;"
            )
        else:
            self.motor_toggle.setText("OFF")
            self.motor_toggle.setStyleSheet(
                "background-color: #666; color: white;"
            )

    def set_motor_enabled(self, enabled: bool):
        """Set motor enabled state (from keyboard shortcut)."""
        self.motor_enabled = enabled
        self.motor_toggle.setChecked(enabled)
        self._update_motor_style()

    def set_velocity(self, linear: float, angular: float):
        """Set velocity from external source (keyboard)."""
        self.linear_velocity = linear
        self.angular_velocity = angular
        self.velocity_label.setText(
            f"Lin: {self.linear_velocity:.2f} m/s  Ang: {self.angular_velocity:.2f} rad/s"
        )


class ControlPanel(QWidget):
    """Control panel with actuator controls and status widgets."""

    # Signal: (command_dict)
    command_requested = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(180)
        self.setMaximumWidth(250)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Title
        title = QLabel("Device Control")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        # Vacuum control
        self.vacuum_ctrl = ActuatorControl("Vacuum", "vacuum")
        self.vacuum_ctrl.command_changed.connect(self._on_actuator_command)
        layout.addWidget(self.vacuum_ctrl)

        # Side brush control
        self.side_brush_ctrl = ActuatorControl("Side Brush", "side_brush")
        self.side_brush_ctrl.command_changed.connect(self._on_actuator_command)
        layout.addWidget(self.side_brush_ctrl)

        # Main brush control
        self.main_brush_ctrl = ActuatorControl("Main Brush", "brush")
        self.main_brush_ctrl.command_changed.connect(self._on_actuator_command)
        layout.addWidget(self.main_brush_ctrl)

        # LED control
        self.led_ctrl = LEDControl()
        self.led_ctrl.command_changed.connect(self._on_led_command)
        layout.addWidget(self.led_ctrl)

        # Separator
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line2)

        # Dustbox status
        self.dustbox_status = DustboxStatus()
        layout.addWidget(self.dustbox_status)

        # Lidar control
        self.lidar_control = LidarControl()
        self.lidar_control.lidar_toggled.connect(self._on_lidar_toggled)
        layout.addWidget(self.lidar_control)

        # Separator
        line3 = QFrame()
        line3.setFrameShape(QFrame.HLine)
        line3.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line3)

        # Drive control
        self.drive_control = DriveControl()
        self.drive_control.velocity_changed.connect(self._on_velocity_changed)
        self.drive_control.motor_toggled.connect(self._on_motor_toggled)
        layout.addWidget(self.drive_control)

        # Push everything to top
        layout.addStretch()

    def _on_actuator_command(self, actuator_id: str, speed: float):
        """Handle actuator control command using ComponentControl protocol."""
        # Map old actuator IDs to new component IDs
        component_id = actuator_id
        if actuator_id == "brush":
            component_id = "main_brush"

        if speed > 0:
            command = {
                "type": "ComponentControl",
                "id": component_id,
                "action": {
                    "type": "Configure",
                    "config": {"speed": {"U8": int(speed)}}
                }
            }
        else:
            command = {
                "type": "ComponentControl",
                "id": component_id,
                "action": {"type": "Disable"}
            }
        self.command_requested.emit(command)

    def _on_led_command(self, value: int):
        """Handle LED control command using ComponentControl protocol."""
        command = {
            "type": "ComponentControl",
            "id": "led",
            "action": {
                "type": "Configure",
                "config": {"state": {"U8": value}}
            }
        }
        self.command_requested.emit(command)

    def update_sensors(self, data: dict):
        """Update control panel with sensor data."""
        # Update dustbox status
        dustbox_attached = data.get('dustbox_attached', True)
        # dustbox_type would come from protocol if available
        dustbox_type = data.get('dustbox_type', None)
        self.dustbox_status.update_status(dustbox_attached, dustbox_type)

        # Update lidar status if scan data present
        if 'scan' in data:
            self.lidar_control.update_status(True)

    def _on_lidar_toggled(self, enabled: bool):
        """Handle lidar toggle from UI button using ComponentControl protocol."""
        if enabled:
            command = {
                "type": "ComponentControl",
                "id": "lidar",
                "action": {"type": "Enable"}
            }
        else:
            command = {
                "type": "ComponentControl",
                "id": "lidar",
                "action": {"type": "Disable"}
            }
        self.command_requested.emit(command)

    def _on_velocity_changed(self, linear: float, angular: float):
        """Handle velocity change from drive control using ComponentControl protocol."""
        command = {
            "type": "ComponentControl",
            "id": "drive",
            "action": {
                "type": "Configure",
                "config": {
                    "linear": {"F32": linear},
                    "angular": {"F32": angular}
                }
            }
        }
        self.command_requested.emit(command)

    def _on_motor_toggled(self, enabled: bool):
        """Handle motor toggle from drive control using ComponentControl protocol."""
        if enabled:
            command = {
                "type": "ComponentControl",
                "id": "drive",
                "action": {"type": "Enable"}
            }
        else:
            command = {
                "type": "ComponentControl",
                "id": "drive",
                "action": {"type": "Disable"}
            }
        self.command_requested.emit(command)

    def set_lidar_enabled(self, enabled: bool):
        """Set lidar enabled state (from keyboard shortcut)."""
        self.lidar_control.set_enabled(enabled)

    def set_motor_enabled(self, enabled: bool):
        """Set motor enabled state (from keyboard shortcut)."""
        self.drive_control.set_motor_enabled(enabled)

    def set_velocity(self, linear: float, angular: float):
        """Set velocity display (from keyboard)."""
        self.drive_control.set_velocity(linear, angular)

    def set_actuator_state(self, actuator_id: str, enabled: bool, speed: int = 100):
        """Set actuator state (from keyboard shortcut)."""
        if actuator_id == "vacuum":
            self.vacuum_ctrl.enabled = enabled
            self.vacuum_ctrl.speed = speed if enabled else 0
            self.vacuum_ctrl.toggle_btn.setChecked(enabled)
            self.vacuum_ctrl._update_toggle_style()
            self.vacuum_ctrl.slider.setValue(speed // 25 if enabled else 0)
        elif actuator_id == "side_brush":
            self.side_brush_ctrl.enabled = enabled
            self.side_brush_ctrl.speed = speed if enabled else 0
            self.side_brush_ctrl.toggle_btn.setChecked(enabled)
            self.side_brush_ctrl._update_toggle_style()
            self.side_brush_ctrl.slider.setValue(speed // 25 if enabled else 0)
        elif actuator_id == "brush":
            self.main_brush_ctrl.enabled = enabled
            self.main_brush_ctrl.speed = speed if enabled else 0
            self.main_brush_ctrl.toggle_btn.setChecked(enabled)
            self.main_brush_ctrl._update_toggle_style()
            self.main_brush_ctrl.slider.setValue(speed // 25 if enabled else 0)
