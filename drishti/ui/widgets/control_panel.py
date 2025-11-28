"""
Control panel widget for Drishti robot visualization.

Provides UI controls organized in collapsible groups:
- Motion Control (drive, e-stop)
- Cleaning Systems (vacuum, brushes, water pump)
- Sensors (lidar, cliff IR)
- System (LED, dustbox status)
- Calibration (IMU, compass)
- Power Management (MCU, charger, main board)
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QPushButton, QFrame, QGridLayout, QSpinBox,
    QScrollArea
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont

from .collapsible_group import CollapsibleGroup
from .compact_actuator import CompactActuatorRow


class LidarControl(QWidget):
    """Widget for lidar control with toggle button and packet count."""

    lidar_toggled = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.packet_count = 0
        self.enabled = False
        self._setup_ui()

        self.timeout_timer = QTimer(self)
        self.timeout_timer.timeout.connect(self._on_timeout)
        self.timeout_timer.setSingleShot(True)

    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title = QLabel("Lidar")
        title.setStyleSheet("color: #aaa;")
        layout.addWidget(title)

        self.toggle_btn = QPushButton("OFF")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setFixedWidth(45)
        self.toggle_btn.clicked.connect(self._on_toggle)
        self._update_toggle_style()
        layout.addWidget(self.toggle_btn)

        self.status_dot = QLabel()
        self.status_dot.setFixedSize(10, 10)
        self.status_dot.setStyleSheet("background-color: #666; border-radius: 5px;")
        layout.addWidget(self.status_dot)

        self.status_label = QLabel("--")
        self.status_label.setStyleSheet("color: #777; font-size: 9px;")
        layout.addWidget(self.status_label)

        layout.addStretch()

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
            self.toggle_btn.setStyleSheet("background-color: #555; color: #ccc;")

    def set_enabled(self, enabled: bool):
        self.enabled = enabled
        self.toggle_btn.setChecked(enabled)
        self._update_toggle_style()

    def update_status(self, scan_received: bool):
        if scan_received:
            self.packet_count += 1
            self.status_label.setText(f"{self.packet_count}")
            self.status_label.setStyleSheet("color: #4CAF50; font-size: 9px;")
            self.status_dot.setStyleSheet("background-color: #4CAF50; border-radius: 5px;")
            self.timeout_timer.start(2000)

    def _on_timeout(self):
        self.status_label.setText("--")
        self.status_label.setStyleSheet("color: #777; font-size: 9px;")
        self.status_dot.setStyleSheet("background-color: #555; border-radius: 5px;")


class CliffIRControl(QWidget):
    """Cliff IR sensor control."""

    command_requested = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.enabled = False
        self._setup_ui()

    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title = QLabel("Cliff IR")
        title.setStyleSheet("color: #aaa;")
        layout.addWidget(title)

        self.toggle_btn = QPushButton("OFF")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setFixedWidth(45)
        self.toggle_btn.clicked.connect(self._on_toggle)
        self._update_toggle_style()
        layout.addWidget(self.toggle_btn)

        layout.addStretch()

        dir_label = QLabel("Dir:")
        dir_label.setStyleSheet("color: #777; font-size: 9px;")
        layout.addWidget(dir_label)

        self.dir_spinbox = QSpinBox()
        self.dir_spinbox.setRange(0, 255)
        self.dir_spinbox.setFixedWidth(50)
        self.dir_spinbox.setFocusPolicy(Qt.ClickFocus)
        self.dir_spinbox.setStyleSheet(
            "QSpinBox { background-color: #3a3a3a; color: #e0e0e0; border: 1px solid #555; }"
        )
        layout.addWidget(self.dir_spinbox)

        self.set_btn = QPushButton("Set")
        self.set_btn.setFixedWidth(35)
        self.set_btn.setStyleSheet("background-color: #555; color: #e0e0e0;")
        self.set_btn.clicked.connect(self._on_set_direction)
        layout.addWidget(self.set_btn)

    def _on_toggle(self):
        self.enabled = self.toggle_btn.isChecked()
        self._update_toggle_style()
        action = "Enable" if self.enabled else "Disable"
        self.command_requested.emit({
            "type": "ComponentControl",
            "id": "cliff_ir",
            "action": {"type": action}
        })

    def _update_toggle_style(self):
        if self.enabled:
            self.toggle_btn.setText("ON")
            self.toggle_btn.setStyleSheet(
                "background-color: #4CAF50; color: white; font-weight: bold;"
            )
        else:
            self.toggle_btn.setText("OFF")
            self.toggle_btn.setStyleSheet("background-color: #555; color: #ccc;")

    def _on_set_direction(self):
        self.command_requested.emit({
            "type": "ComponentControl",
            "id": "cliff_ir",
            "action": {
                "type": "Configure",
                "config": {"direction": {"U8": self.dir_spinbox.value()}}
            }
        })


class DustboxStatus(QWidget):
    """Widget showing dustbox status with water level and pump control."""

    water_pump_changed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_2in1 = False
        self.water_pump_enabled = False
        self.water_pump_speed = 0
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        # Status row
        status_row = QHBoxLayout()
        status_row.setSpacing(8)

        self.status_dot = QLabel()
        self.status_dot.setFixedSize(10, 10)
        self.status_dot.setStyleSheet("background-color: #4CAF50; border-radius: 5px;")
        status_row.addWidget(self.status_dot)

        self.status_label = QLabel("Present")
        self.status_label.setStyleSheet("color: #e0e0e0;")
        status_row.addWidget(self.status_label)

        self.type_label = QLabel("Unknown")
        self.type_label.setStyleSheet("color: #777; font-size: 9px;")
        status_row.addWidget(self.type_label)

        status_row.addStretch()
        layout.addLayout(status_row)

        # Water level row
        self.water_row = QWidget()
        water_layout = QHBoxLayout(self.water_row)
        water_layout.setContentsMargins(0, 0, 0, 0)
        water_layout.setSpacing(8)

        self.water_dot = QLabel()
        self.water_dot.setFixedSize(10, 10)
        self.water_dot.setStyleSheet("background-color: #666; border-radius: 5px;")
        water_layout.addWidget(self.water_dot)

        self.water_label = QLabel("Tank: --")
        self.water_label.setStyleSheet("color: #777;")
        water_layout.addWidget(self.water_label)

        water_layout.addStretch()
        layout.addWidget(self.water_row)

        # Water pump control
        self.pump_row = QWidget()
        pump_layout = QHBoxLayout(self.pump_row)
        pump_layout.setContentsMargins(0, 0, 0, 0)
        pump_layout.setSpacing(6)

        pump_label = QLabel("Pump:")
        pump_label.setStyleSheet("color: #777; font-size: 9px;")
        pump_layout.addWidget(pump_label)

        self.pump_toggle = QPushButton("OFF")
        self.pump_toggle.setCheckable(True)
        self.pump_toggle.setFixedSize(40, 20)
        self.pump_toggle.clicked.connect(self._on_pump_toggle)
        self._update_pump_style()
        pump_layout.addWidget(self.pump_toggle)

        self.pump_slider = QSlider(Qt.Horizontal)
        self.pump_slider.setRange(0, 4)
        self.pump_slider.setFocusPolicy(Qt.NoFocus)
        self.pump_slider.valueChanged.connect(self._on_pump_slider)
        pump_layout.addWidget(self.pump_slider)

        self.pump_speed_label = QLabel("0%")
        self.pump_speed_label.setFixedWidth(25)
        self.pump_speed_label.setStyleSheet("color: #aaa; font-size: 9px;")
        pump_layout.addWidget(self.pump_speed_label)

        layout.addWidget(self.pump_row)

        # Initially hide water controls
        self.water_row.setVisible(False)
        self.pump_row.setVisible(False)

    def _on_pump_toggle(self):
        self.water_pump_enabled = self.pump_toggle.isChecked()
        self._update_pump_style()
        if self.water_pump_enabled and self.water_pump_speed == 0:
            self.water_pump_speed = 100
            self.pump_slider.setValue(4)
            self.pump_speed_label.setText("100%")
        self._emit_pump_command()

    def _on_pump_slider(self, value):
        self.water_pump_speed = value * 25
        self.pump_speed_label.setText(f"{self.water_pump_speed}%")
        if value > 0 and not self.water_pump_enabled:
            self.water_pump_enabled = True
            self.pump_toggle.setChecked(True)
            self._update_pump_style()
        elif value == 0 and self.water_pump_enabled:
            self.water_pump_enabled = False
            self.pump_toggle.setChecked(False)
            self._update_pump_style()
        self._emit_pump_command()

    def _update_pump_style(self):
        if self.water_pump_enabled:
            self.pump_toggle.setText("ON")
            self.pump_toggle.setStyleSheet(
                "background-color: #2196F3; color: white; font-weight: bold; font-size: 9px;"
            )
        else:
            self.pump_toggle.setText("OFF")
            self.pump_toggle.setStyleSheet(
                "background-color: #555; color: #ccc; font-size: 9px;"
            )

    def _emit_pump_command(self):
        speed = float(self.water_pump_speed) if self.water_pump_enabled else 0.0
        self.water_pump_changed.emit(speed)

    def update_status(self, attached: bool, dustbox_type: str = None, water_level: int = None):
        if attached:
            self.status_label.setText("Present")
            self.status_label.setStyleSheet("color: #e0e0e0;")
            self.status_dot.setStyleSheet("background-color: #4CAF50; border-radius: 5px;")
        else:
            self.status_label.setText("Missing")
            self.status_label.setStyleSheet("color: #ff6b6b;")
            self.status_dot.setStyleSheet("background-color: #f44336; border-radius: 5px;")

        if water_level is not None and water_level > 0:
            self.is_2in1 = True

        if self.is_2in1:
            self.type_label.setText("2-in-1")
        elif dustbox_type == "dry":
            self.type_label.setText("Dry")
        else:
            self.type_label.setText("")

        self.water_row.setVisible(attached)
        self.pump_row.setVisible(attached)

        if water_level is not None and water_level > 0:
            self.water_label.setText(f"Tank: {water_level}%")
            self.water_label.setStyleSheet("color: #00BCD4;")
            self.water_dot.setStyleSheet("background-color: #00BCD4; border-radius: 5px;")
        else:
            self.water_label.setText("Tank: --")
            self.water_label.setStyleSheet("color: #777;")
            self.water_dot.setStyleSheet("background-color: #666; border-radius: 5px;")


class LEDControl(QWidget):
    """LED mode control."""

    command_changed = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title = QLabel("LED")
        title.setStyleSheet("color: #aaa;")
        layout.addWidget(title)

        self.spinbox = QSpinBox()
        self.spinbox.setRange(0, 20)
        self.spinbox.setFixedWidth(50)
        self.spinbox.setFocusPolicy(Qt.ClickFocus)
        self.spinbox.setStyleSheet(
            "QSpinBox { background-color: #3a3a3a; color: #e0e0e0; border: 1px solid #555; }"
        )
        layout.addWidget(self.spinbox)

        self.send_btn = QPushButton("Set")
        self.send_btn.setFixedWidth(35)
        self.send_btn.setStyleSheet("background-color: #555; color: #e0e0e0;")
        self.send_btn.clicked.connect(lambda: self.command_changed.emit(self.spinbox.value()))
        layout.addWidget(self.send_btn)

        hint = QLabel("0=Off 1=Blue 7=Red")
        hint.setStyleSheet("color: #666; font-size: 8px;")
        layout.addWidget(hint)

        layout.addStretch()


class CalibrationControl(QWidget):
    """Calibration controls for IMU and compass."""

    command_requested = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        btn_style = "background-color: #555; color: #e0e0e0; padding: 3px 6px;"

        # IMU row
        imu_row = QHBoxLayout()
        imu_label = QLabel("IMU")
        imu_label.setStyleSheet("color: #aaa;")
        imu_label.setFixedWidth(50)
        imu_row.addWidget(imu_label)

        imu_factory = QPushButton("Factory")
        imu_factory.setStyleSheet(btn_style)
        imu_factory.clicked.connect(lambda: self._send("imu", "Reset"))
        imu_row.addWidget(imu_factory)

        imu_query = QPushButton("Query")
        imu_query.setStyleSheet(btn_style)
        imu_query.clicked.connect(lambda: self._send("imu", "Enable"))
        imu_row.addWidget(imu_query)

        imu_row.addStretch()
        layout.addLayout(imu_row)

        # Compass row
        compass_row = QHBoxLayout()
        compass_label = QLabel("Compass")
        compass_label.setStyleSheet("color: #aaa;")
        compass_label.setFixedWidth(50)
        compass_row.addWidget(compass_label)

        compass_cal = QPushButton("Start Cal")
        compass_cal.setStyleSheet(btn_style)
        compass_cal.clicked.connect(lambda: self._send("compass", "Reset"))
        compass_row.addWidget(compass_cal)

        compass_query = QPushButton("Query")
        compass_query.setStyleSheet(btn_style)
        compass_query.clicked.connect(lambda: self._send("compass", "Enable"))
        compass_row.addWidget(compass_query)

        compass_row.addStretch()
        layout.addLayout(compass_row)

    def _send(self, component_id: str, action_type: str):
        self.command_requested.emit({
            "type": "ComponentControl",
            "id": component_id,
            "action": {"type": action_type}
        })


class PowerControl(QWidget):
    """Power management controls."""

    command_requested = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        btn_style = "background-color: #555; color: #e0e0e0; padding: 3px 6px;"
        btn_warn = "background-color: #8B4513; color: #e0e0e0; padding: 3px 6px;"

        # MCU row
        mcu_row = QHBoxLayout()
        mcu_label = QLabel("MCU")
        mcu_label.setStyleSheet("color: #aaa;")
        mcu_label.setFixedWidth(45)
        mcu_row.addWidget(mcu_label)

        mcu_wake = QPushButton("Wake")
        mcu_wake.setStyleSheet(btn_style)
        mcu_wake.clicked.connect(lambda: self._send("mcu", "Enable"))
        mcu_row.addWidget(mcu_wake)

        mcu_sleep = QPushButton("Sleep")
        mcu_sleep.setStyleSheet(btn_style)
        mcu_sleep.clicked.connect(lambda: self._send("mcu", "Disable"))
        mcu_row.addWidget(mcu_sleep)

        mcu_reset = QPushButton("Rst Err")
        mcu_reset.setStyleSheet(btn_style)
        mcu_reset.clicked.connect(lambda: self._send("mcu", "Reset"))
        mcu_row.addWidget(mcu_reset)

        layout.addLayout(mcu_row)

        # Charger row
        charger_row = QHBoxLayout()
        charger_label = QLabel("Charger")
        charger_label.setStyleSheet("color: #aaa;")
        charger_label.setFixedWidth(45)
        charger_row.addWidget(charger_label)

        charger_on = QPushButton("ON")
        charger_on.setStyleSheet(btn_style)
        charger_on.clicked.connect(lambda: self._send("charger", "Enable"))
        charger_row.addWidget(charger_on)

        charger_off = QPushButton("OFF")
        charger_off.setStyleSheet(btn_style)
        charger_off.clicked.connect(lambda: self._send("charger", "Disable"))
        charger_row.addWidget(charger_off)

        charger_row.addStretch()
        layout.addLayout(charger_row)

        # Board row
        board_row = QHBoxLayout()
        board_label = QLabel("Board")
        board_label.setStyleSheet("color: #aaa;")
        board_label.setFixedWidth(45)
        board_row.addWidget(board_label)

        board_on = QPushButton("ON")
        board_on.setStyleSheet(btn_style)
        board_on.clicked.connect(lambda: self._send("main_board", "Enable"))
        board_row.addWidget(board_on)

        board_off = QPushButton("OFF")
        board_off.setStyleSheet(btn_warn)
        board_off.clicked.connect(lambda: self._send("main_board", "Disable"))
        board_row.addWidget(board_off)

        board_restart = QPushButton("Restart")
        board_restart.setStyleSheet(btn_warn)
        board_restart.clicked.connect(lambda: self._send("main_board", "Reset"))
        board_row.addWidget(board_restart)

        layout.addLayout(board_row)

        # Warning
        warn = QLabel("OFF/Restart terminates daemon!")
        warn.setStyleSheet("color: #ff6b6b; font-size: 8px;")
        layout.addWidget(warn)

    def _send(self, component_id: str, action_type: str):
        self.command_requested.emit({
            "type": "ComponentControl",
            "id": component_id,
            "action": {"type": action_type}
        })


class DriveControl(QWidget):
    """Arrow key-style drive control with E-Stop."""

    velocity_changed = pyqtSignal(float, float)
    motor_toggled = pyqtSignal(bool)
    emergency_stop = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_step = 0.5
        self.angular_step = 0.5
        self.motor_enabled = False
        self._setup_ui()

    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # Arrow buttons grid (left side)
        arrow_widget = QWidget()
        grid = QGridLayout(arrow_widget)
        grid.setSpacing(2)
        grid.setContentsMargins(0, 0, 0, 0)

        btn_style = """
            QPushButton {
                background-color: #555;
                color: #e0e0e0;
                border: none;
                border-radius: 3px;
            }
            QPushButton:hover { background-color: #666; }
            QPushButton:pressed { background-color: #4CAF50; }
        """

        self.btn_forward = QPushButton("▲")
        self.btn_forward.setFixedSize(36, 36)
        self.btn_forward.setStyleSheet(btn_style)
        self.btn_forward.pressed.connect(self._on_forward)
        self.btn_forward.released.connect(self._on_released)
        grid.addWidget(self.btn_forward, 0, 1)

        self.btn_left = QPushButton("◀")
        self.btn_left.setFixedSize(36, 36)
        self.btn_left.setStyleSheet(btn_style)
        self.btn_left.pressed.connect(self._on_left)
        self.btn_left.released.connect(self._on_released)
        grid.addWidget(self.btn_left, 1, 0)

        self.btn_backward = QPushButton("▼")
        self.btn_backward.setFixedSize(36, 36)
        self.btn_backward.setStyleSheet(btn_style)
        self.btn_backward.pressed.connect(self._on_backward)
        self.btn_backward.released.connect(self._on_released)
        grid.addWidget(self.btn_backward, 1, 1)

        self.btn_right = QPushButton("▶")
        self.btn_right.setFixedSize(36, 36)
        self.btn_right.setStyleSheet(btn_style)
        self.btn_right.pressed.connect(self._on_right)
        self.btn_right.released.connect(self._on_released)
        grid.addWidget(self.btn_right, 1, 2)

        layout.addWidget(arrow_widget)

        # Right side: Motor toggle, velocity, E-Stop
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(4)

        # Motor toggle row
        motor_row = QHBoxLayout()
        motor_label = QLabel("Motor")
        motor_label.setStyleSheet("color: #aaa; font-size: 10px;")
        motor_row.addWidget(motor_label)

        self.motor_toggle = QPushButton("OFF")
        self.motor_toggle.setCheckable(True)
        self.motor_toggle.setFixedSize(45, 22)
        self.motor_toggle.clicked.connect(self._on_motor_toggle)
        self._update_motor_style()
        motor_row.addWidget(self.motor_toggle)

        motor_row.addStretch()
        right_layout.addLayout(motor_row)

        # Velocity display
        self.velocity_label = QLabel("0.0 / 0.0")
        self.velocity_label.setStyleSheet("color: #777; font-size: 9px;")
        right_layout.addWidget(self.velocity_label)

        # E-Stop button
        self.estop_btn = QPushButton("E-STOP")
        self.estop_btn.setFixedHeight(28)
        self.estop_btn.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: white;
                font-weight: bold;
                border: 2px solid #b71c1c;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #f44336; }
            QPushButton:pressed { background-color: #b71c1c; }
        """)
        self.estop_btn.clicked.connect(self._on_estop)
        right_layout.addWidget(self.estop_btn)

        layout.addWidget(right_widget)
        layout.addStretch()

    def _on_forward(self):
        if not self.motor_enabled:
            return
        self.linear_velocity = self.linear_step
        self.angular_velocity = 0.0
        self._emit_velocity()

    def _on_backward(self):
        if not self.motor_enabled:
            return
        self.linear_velocity = -self.linear_step
        self.angular_velocity = 0.0
        self._emit_velocity()

    def _on_left(self):
        if not self.motor_enabled:
            return
        self.linear_velocity = 0.0
        self.angular_velocity = self.angular_step
        self._emit_velocity()

    def _on_right(self):
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
        self.velocity_label.setText(f"{self.linear_velocity:.1f} / {self.angular_velocity:.1f}")
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
            self.motor_toggle.setStyleSheet("background-color: #555; color: #ccc;")

    def _on_estop(self):
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.velocity_label.setText("STOP")
        self.emergency_stop.emit()

    def set_motor_enabled(self, enabled: bool):
        self.motor_enabled = enabled
        self.motor_toggle.setChecked(enabled)
        self._update_motor_style()

    def set_velocity(self, linear: float, angular: float):
        self.linear_velocity = linear
        self.angular_velocity = angular
        self.velocity_label.setText(f"{linear:.1f} / {angular:.1f}")


class ControlPanel(QWidget):
    """Control panel with collapsible groups."""

    command_requested = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(250)
        self._setup_ui()

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setStyleSheet("""
            QScrollArea { border: none; background-color: transparent; }
            QScrollBar:vertical { background-color: #3a3a3a; width: 8px; }
            QScrollBar::handle:vertical { background-color: #555; border-radius: 4px; min-height: 20px; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        """)

        content = QWidget()
        content.setFixedWidth(242)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(6)

        # === Motion Control (TOP) ===
        self.motion_group = CollapsibleGroup("Motion Control")
        self.drive_control = DriveControl()
        self.drive_control.velocity_changed.connect(self._on_velocity_changed)
        self.drive_control.motor_toggled.connect(self._on_motor_toggled)
        self.drive_control.emergency_stop.connect(self._on_emergency_stop)
        self.motion_group.add_widget(self.drive_control)
        layout.addWidget(self.motion_group)

        # === Cleaning Systems ===
        self.cleaning_group = CollapsibleGroup("Cleaning Systems")
        self.actuator_row = CompactActuatorRow()
        self.actuator_row.command_changed.connect(self._on_actuator_command)
        self.cleaning_group.add_widget(self.actuator_row)

        # Water pump (in dustbox)
        self.dustbox_status = DustboxStatus()
        self.dustbox_status.water_pump_changed.connect(self._on_water_pump_command)
        self.cleaning_group.add_widget(self.dustbox_status)
        layout.addWidget(self.cleaning_group)

        # === Sensors ===
        self.sensors_group = CollapsibleGroup("Sensors")
        self.lidar_control = LidarControl()
        self.lidar_control.lidar_toggled.connect(self._on_lidar_toggled)
        self.sensors_group.add_widget(self.lidar_control)

        self.cliff_ir_control = CliffIRControl()
        self.cliff_ir_control.command_requested.connect(self._forward_command)
        self.sensors_group.add_widget(self.cliff_ir_control)
        layout.addWidget(self.sensors_group)

        # === System ===
        self.system_group = CollapsibleGroup("System")
        self.led_ctrl = LEDControl()
        self.led_ctrl.command_changed.connect(self._on_led_command)
        self.system_group.add_widget(self.led_ctrl)
        layout.addWidget(self.system_group)

        # === Calibration ===
        self.calibration_group = CollapsibleGroup("Calibration")
        self.calibration_control = CalibrationControl()
        self.calibration_control.command_requested.connect(self._forward_command)
        self.calibration_group.add_widget(self.calibration_control)
        layout.addWidget(self.calibration_group)

        # === Power Management ===
        self.power_group = CollapsibleGroup("Power Management")
        self.power_control = PowerControl()
        self.power_control.command_requested.connect(self._forward_command)
        self.power_group.add_widget(self.power_control)
        layout.addWidget(self.power_group)

        layout.addStretch()
        scroll.setWidget(content)
        main_layout.addWidget(scroll)

    def _on_actuator_command(self, actuator_id: str, speed: int):
        component_id = "main_brush" if actuator_id == "brush" else actuator_id
        if speed > 0:
            cmd = {
                "type": "ComponentControl",
                "id": component_id,
                "action": {"type": "Configure", "config": {"speed": {"U8": speed}}}
            }
        else:
            cmd = {
                "type": "ComponentControl",
                "id": component_id,
                "action": {"type": "Disable"}
            }
        self.command_requested.emit(cmd)

    def _on_led_command(self, value: int):
        self.command_requested.emit({
            "type": "ComponentControl",
            "id": "led",
            "action": {"type": "Configure", "config": {"state": {"U8": value}}}
        })

    def _on_water_pump_command(self, speed: float):
        if speed > 0:
            cmd = {
                "type": "ComponentControl",
                "id": "water_pump",
                "action": {"type": "Configure", "config": {"speed": {"U8": int(speed)}}}
            }
        else:
            cmd = {
                "type": "ComponentControl",
                "id": "water_pump",
                "action": {"type": "Disable"}
            }
        self.command_requested.emit(cmd)

    def _on_lidar_toggled(self, enabled: bool):
        action = "Enable" if enabled else "Disable"
        self.command_requested.emit({
            "type": "ComponentControl",
            "id": "lidar",
            "action": {"type": action}
        })

    def _on_velocity_changed(self, linear: float, angular: float):
        self.command_requested.emit({
            "type": "ComponentControl",
            "id": "drive",
            "action": {
                "type": "Configure",
                "config": {"linear": {"F32": linear}, "angular": {"F32": angular}}
            }
        })

    def _on_motor_toggled(self, enabled: bool):
        action = "Enable" if enabled else "Disable"
        self.command_requested.emit({
            "type": "ComponentControl",
            "id": "drive",
            "action": {"type": action}
        })

    def _on_emergency_stop(self):
        self.command_requested.emit({
            "type": "ComponentControl",
            "id": "drive",
            "action": {"type": "Reset"}
        })

    def _forward_command(self, command: dict):
        self.command_requested.emit(command)

    def update_sensors(self, data: dict):
        dustbox_attached = data.get('dustbox_attached', True)
        water_level = data.get('water_tank_level', None)
        dustbox_type = "2in1" if water_level and water_level > 0 else None
        self.dustbox_status.update_status(dustbox_attached, dustbox_type, water_level)

        if 'scan' in data:
            self.lidar_control.update_status(True)

    def set_lidar_enabled(self, enabled: bool):
        self.lidar_control.set_enabled(enabled)

    def set_motor_enabled(self, enabled: bool):
        self.drive_control.set_motor_enabled(enabled)

    def set_velocity(self, linear: float, angular: float):
        self.drive_control.set_velocity(linear, angular)

    def set_actuator_state(self, actuator_id: str, enabled: bool, speed: int = 100):
        self.actuator_row.set_actuator_state(actuator_id, enabled, speed)
