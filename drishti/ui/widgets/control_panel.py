"""
Control panel widget for Drishti robot visualization.
Provides UI controls for vacuum, side brush, and main brush with speed control.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QPushButton, QGroupBox, QFrame, QGridLayout, QSpinBox,
    QScrollArea
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
        name_label.setStyleSheet("color: #e0e0e0;")
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
        self.slider.setFocusPolicy(Qt.NoFocus)  # Don't steal arrow keys
        self.slider.valueChanged.connect(self._on_slider_changed)
        slider_layout.addWidget(self.slider)

        self.speed_label = QLabel("0%")
        self.speed_label.setFixedWidth(35)
        self.speed_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.speed_label.setStyleSheet("color: #aaa;")
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
                "background-color: #555; color: #ccc;"
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
        title.setStyleSheet("color: #e0e0e0;")
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
        self.status_label.setStyleSheet("color: #777;")
        status_layout.addWidget(self.status_label)

        status_layout.addStretch()
        layout.addLayout(status_layout)

        # Packet count
        self.count_label = QLabel("Packets: 0")
        self.count_label.setStyleSheet("color: #777; font-size: 9px;")
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
                "background-color: #555; color: #ccc;"
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
            self.status_label.setStyleSheet("color: #e0e0e0; font-weight: bold;")
            self.status_dot.setStyleSheet(
                "background-color: #4CAF50; border-radius: 6px;"
            )
            self.count_label.setText(f"Packets: {self.packet_count}")
            self.count_label.setStyleSheet("color: #aaa; font-size: 9px;")

            # Reset timeout timer (2 seconds)
            self.timeout_timer.start(2000)

    def _on_timeout(self):
        """Called when no data received for 2 seconds."""
        self.status_label.setText("No data")
        self.status_label.setStyleSheet("color: #777;")
        self.status_dot.setStyleSheet(
            "background-color: #555; border-radius: 6px;"
        )


class DustboxStatus(QWidget):
    """Widget showing dustbox status with water level and pump control for 2-in-1 box."""

    # Signal: (speed_percent)
    water_pump_changed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_2in1 = False
        self.water_pump_enabled = False
        self.water_pump_speed = 0
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)

        # Title
        title = QLabel("Dustbox")
        title.setFont(QFont("Arial", 10, QFont.Bold))
        title.setStyleSheet("color: #e0e0e0;")
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
        self.status_label.setStyleSheet("color: #e0e0e0; font-weight: bold;")
        status_layout.addWidget(self.status_label)

        status_layout.addStretch()
        layout.addLayout(status_layout)

        # Type indicator
        self.type_label = QLabel("Type: Unknown")
        self.type_label.setStyleSheet("color: #777; font-size: 9px;")
        layout.addWidget(self.type_label)

        # Water tank level (only shown for 2-in-1 box)
        self.water_level_layout = QHBoxLayout()
        self.water_level_layout.setSpacing(8)

        self.water_dot = QLabel()
        self.water_dot.setFixedSize(12, 12)
        self.water_dot.setStyleSheet(
            "background-color: #666; border-radius: 6px;"
        )
        self.water_level_layout.addWidget(self.water_dot)

        self.water_level_label = QLabel("Tank: --")
        self.water_level_label.setStyleSheet("color: #777;")
        self.water_level_layout.addWidget(self.water_level_label)

        self.water_level_layout.addStretch()
        layout.addLayout(self.water_level_layout)

        # Water pump control (only shown for 2-in-1 box)
        self.pump_widget = QWidget()
        pump_layout = QVBoxLayout(self.pump_widget)
        pump_layout.setContentsMargins(0, 5, 0, 0)
        pump_layout.setSpacing(3)

        # Pump header with toggle
        pump_header = QHBoxLayout()
        pump_label = QLabel("Water Pump")
        pump_label.setStyleSheet("color: #aaa; font-size: 9px;")
        pump_header.addWidget(pump_label)

        pump_header.addStretch()

        self.pump_toggle = QPushButton("OFF")
        self.pump_toggle.setCheckable(True)
        self.pump_toggle.setFixedWidth(40)
        self.pump_toggle.setFixedHeight(20)
        self.pump_toggle.clicked.connect(self._on_pump_toggle)
        self._update_pump_style()
        pump_header.addWidget(self.pump_toggle)

        pump_layout.addLayout(pump_header)

        # Pump speed slider
        slider_layout = QHBoxLayout()

        self.pump_slider = QSlider(Qt.Horizontal)
        self.pump_slider.setMinimum(0)
        self.pump_slider.setMaximum(4)  # 0, 25, 50, 75, 100
        self.pump_slider.setValue(0)
        self.pump_slider.setTickPosition(QSlider.TicksBelow)
        self.pump_slider.setTickInterval(1)
        self.pump_slider.setFocusPolicy(Qt.NoFocus)
        self.pump_slider.valueChanged.connect(self._on_pump_slider_changed)
        slider_layout.addWidget(self.pump_slider)

        self.pump_speed_label = QLabel("0%")
        self.pump_speed_label.setFixedWidth(30)
        self.pump_speed_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.pump_speed_label.setStyleSheet("color: #aaa; font-size: 9px;")
        slider_layout.addWidget(self.pump_speed_label)

        pump_layout.addLayout(slider_layout)

        layout.addWidget(self.pump_widget)

        # Initially hide water controls
        self._set_water_controls_visible(False)

    def _set_water_controls_visible(self, visible: bool):
        """Show or hide water-related controls."""
        self.water_dot.setVisible(visible)
        self.water_level_label.setVisible(visible)
        self.pump_widget.setVisible(visible)

    def _on_pump_toggle(self):
        self.water_pump_enabled = self.pump_toggle.isChecked()
        self._update_pump_style()

        # When toggling ON with slider at 0, set to 100%
        if self.water_pump_enabled and self.water_pump_speed == 0:
            self.water_pump_speed = 100
            self.pump_slider.setValue(4)
            self.pump_speed_label.setText("100%")

        self._emit_pump_command()

    def _on_pump_slider_changed(self, value):
        self.water_pump_speed = value * 25
        self.pump_speed_label.setText(f"{self.water_pump_speed}%")

        # If slider moved above 0, enable; if moved to 0, disable
        if self.water_pump_speed > 0 and not self.water_pump_enabled:
            self.water_pump_enabled = True
            self.pump_toggle.setChecked(True)
            self._update_pump_style()
        elif self.water_pump_speed == 0 and self.water_pump_enabled:
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
        actual_speed = float(self.water_pump_speed) if self.water_pump_enabled else 0.0
        self.water_pump_changed.emit(actual_speed)

    def update_status(self, attached: bool, dustbox_type: str = None, water_level: int = None):
        """Update dustbox status display.

        Args:
            attached: Whether dustbox is present
            dustbox_type: "dry" or "2in1" (optional, may not be available)
            water_level: Water tank level 0-100 (only for 2-in-1 box)
        """
        if attached:
            self.status_label.setText("Present")
            self.status_label.setStyleSheet("color: #e0e0e0; font-weight: bold;")
            self.status_dot.setStyleSheet(
                "background-color: #4CAF50; border-radius: 6px;"
            )
        else:
            self.status_label.setText("Missing")
            self.status_label.setStyleSheet("color: #ff6b6b; font-weight: bold;")
            self.status_dot.setStyleSheet(
                "background-color: #f44336; border-radius: 6px;"
            )

        # Detect 2-in-1 box if water level > 0 (pump was activated and water detected)
        if water_level is not None and water_level > 0:
            self.is_2in1 = True

        # Update type label
        if self.is_2in1:
            self.type_label.setText("Type: 2-in-1 (Wet/Dry)")
        elif dustbox_type == "dry":
            self.type_label.setText("Type: Dry Only")
        else:
            self.type_label.setText("Type: Unknown")

        # Always show water pump controls when dustbox is attached
        # (user can test if it's a 2-in-1 box by activating pump)
        self._set_water_controls_visible(attached)

        # Update water level display
        if water_level is not None:
            if water_level > 0:
                self.water_level_label.setText(f"Tank: {water_level}%")
                self.water_level_label.setStyleSheet("color: #00BCD4; font-weight: bold;")
                self.water_dot.setStyleSheet(
                    "background-color: #00BCD4; border-radius: 6px;"
                )
            else:
                self.water_level_label.setText("Tank: 0%")
                self.water_level_label.setStyleSheet("color: #777;")
                self.water_dot.setStyleSheet(
                    "background-color: #666; border-radius: 6px;"
                )
        else:
            self.water_level_label.setText("Tank: --")
            self.water_level_label.setStyleSheet("color: #777;")
            self.water_dot.setStyleSheet(
                "background-color: #666; border-radius: 6px;"
            )


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
        title.setStyleSheet("color: #e0e0e0;")
        layout.addWidget(title)

        # Spinbox row
        spinbox_layout = QHBoxLayout()

        self.spinbox = QSpinBox()
        self.spinbox.setMinimum(0)
        self.spinbox.setMaximum(20)
        self.spinbox.setValue(0)
        self.spinbox.setFocusPolicy(Qt.ClickFocus)  # Only focus on click
        self.spinbox.setStyleSheet(
            "QSpinBox { background-color: #3a3a3a; color: #e0e0e0; border: 1px solid #555; }"
        )
        self.spinbox.valueChanged.connect(self._on_value_changed)
        spinbox_layout.addWidget(self.spinbox)

        self.send_btn = QPushButton("Set")
        self.send_btn.setFixedWidth(40)
        self.send_btn.setStyleSheet("background-color: #555; color: #e0e0e0;")
        self.send_btn.clicked.connect(self._on_send_clicked)
        spinbox_layout.addWidget(self.send_btn)

        layout.addLayout(spinbox_layout)

        # Hint label
        hint = QLabel("0=Off, 1=Blue, 7=Red, 17=Purple")
        hint.setStyleSheet("color: #777; font-size: 9px;")
        hint.setWordWrap(True)
        layout.addWidget(hint)

    def _on_value_changed(self, value: int):
        # Don't auto-send on value change, wait for button click
        pass

    def _on_send_clicked(self):
        self.command_changed.emit(self.spinbox.value())


class PowerControl(QWidget):
    """Power management controls for MCU, charger, and main board."""

    command_requested = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # Title
        title = QLabel("Power")
        title.setFont(QFont("Arial", 10, QFont.Bold))
        title.setStyleSheet("color: #e0e0e0;")
        layout.addWidget(title)

        # MCU row
        mcu_layout = QHBoxLayout()
        mcu_label = QLabel("MCU")
        mcu_label.setStyleSheet("color: #aaa;")
        mcu_label.setFixedWidth(50)
        mcu_layout.addWidget(mcu_label)

        btn_style = "background-color: #555; color: #e0e0e0; padding: 3px 6px;"
        btn_style_warn = "background-color: #8B4513; color: #e0e0e0; padding: 3px 6px;"

        self.mcu_wake_btn = QPushButton("Wake")
        self.mcu_wake_btn.setStyleSheet(btn_style)
        self.mcu_wake_btn.clicked.connect(lambda: self._send_command("mcu", "Enable"))
        mcu_layout.addWidget(self.mcu_wake_btn)

        self.mcu_sleep_btn = QPushButton("Sleep")
        self.mcu_sleep_btn.setStyleSheet(btn_style)
        self.mcu_sleep_btn.clicked.connect(lambda: self._send_command("mcu", "Disable"))
        mcu_layout.addWidget(self.mcu_sleep_btn)

        self.mcu_reset_btn = QPushButton("Reset Err")
        self.mcu_reset_btn.setStyleSheet(btn_style)
        self.mcu_reset_btn.clicked.connect(lambda: self._send_command("mcu", "Reset"))
        mcu_layout.addWidget(self.mcu_reset_btn)

        layout.addLayout(mcu_layout)

        # Charger row
        charger_layout = QHBoxLayout()
        charger_label = QLabel("Charger")
        charger_label.setStyleSheet("color: #aaa;")
        charger_label.setFixedWidth(50)
        charger_layout.addWidget(charger_label)

        self.charger_on_btn = QPushButton("ON")
        self.charger_on_btn.setStyleSheet(btn_style)
        self.charger_on_btn.clicked.connect(lambda: self._send_command("charger", "Enable"))
        charger_layout.addWidget(self.charger_on_btn)

        self.charger_off_btn = QPushButton("OFF")
        self.charger_off_btn.setStyleSheet(btn_style)
        self.charger_off_btn.clicked.connect(lambda: self._send_command("charger", "Disable"))
        charger_layout.addWidget(self.charger_off_btn)

        charger_layout.addStretch()
        layout.addLayout(charger_layout)

        # Main Board row
        board_layout = QHBoxLayout()
        board_label = QLabel("Board")
        board_label.setStyleSheet("color: #aaa;")
        board_label.setFixedWidth(50)
        board_layout.addWidget(board_label)

        self.board_on_btn = QPushButton("ON")
        self.board_on_btn.setStyleSheet(btn_style)
        self.board_on_btn.clicked.connect(lambda: self._send_command("main_board", "Enable"))
        board_layout.addWidget(self.board_on_btn)

        self.board_off_btn = QPushButton("OFF")
        self.board_off_btn.setStyleSheet(btn_style_warn)
        self.board_off_btn.clicked.connect(lambda: self._send_command("main_board", "Disable"))
        board_layout.addWidget(self.board_off_btn)

        self.board_restart_btn = QPushButton("Restart")
        self.board_restart_btn.setStyleSheet(btn_style_warn)
        self.board_restart_btn.clicked.connect(lambda: self._send_command("main_board", "Reset"))
        board_layout.addWidget(self.board_restart_btn)

        layout.addLayout(board_layout)

        # Warning label
        warn_label = QLabel("Board OFF/Restart will terminate daemon!")
        warn_label.setStyleSheet("color: #ff6b6b; font-size: 8px;")
        warn_label.setWordWrap(True)
        layout.addWidget(warn_label)

    def _send_command(self, component_id: str, action_type: str):
        command = {
            "type": "ComponentControl",
            "id": component_id,
            "action": {"type": action_type}
        }
        self.command_requested.emit(command)


class CalibrationControl(QWidget):
    """Calibration controls for IMU and compass."""

    command_requested = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # Title
        title = QLabel("Calibration")
        title.setFont(QFont("Arial", 10, QFont.Bold))
        title.setStyleSheet("color: #e0e0e0;")
        layout.addWidget(title)

        btn_style = "background-color: #555; color: #e0e0e0; padding: 3px 6px;"

        # IMU row
        imu_layout = QHBoxLayout()
        imu_label = QLabel("IMU")
        imu_label.setStyleSheet("color: #aaa;")
        imu_label.setFixedWidth(55)
        imu_layout.addWidget(imu_label)

        self.imu_factory_btn = QPushButton("Factory Cal")
        self.imu_factory_btn.setStyleSheet(btn_style)
        self.imu_factory_btn.clicked.connect(lambda: self._send_command("imu", "Reset"))
        imu_layout.addWidget(self.imu_factory_btn)

        self.imu_query_btn = QPushButton("Query")
        self.imu_query_btn.setStyleSheet(btn_style)
        self.imu_query_btn.clicked.connect(lambda: self._send_command("imu", "Enable"))
        imu_layout.addWidget(self.imu_query_btn)

        layout.addLayout(imu_layout)

        # Compass row
        compass_layout = QHBoxLayout()
        compass_label = QLabel("Compass")
        compass_label.setStyleSheet("color: #aaa;")
        compass_label.setFixedWidth(55)
        compass_layout.addWidget(compass_label)

        self.compass_cal_btn = QPushButton("Start Cal")
        self.compass_cal_btn.setStyleSheet(btn_style)
        self.compass_cal_btn.clicked.connect(lambda: self._send_command("compass", "Reset"))
        compass_layout.addWidget(self.compass_cal_btn)

        self.compass_query_btn = QPushButton("Query")
        self.compass_query_btn.setStyleSheet(btn_style)
        self.compass_query_btn.clicked.connect(lambda: self._send_command("compass", "Enable"))
        compass_layout.addWidget(self.compass_query_btn)

        layout.addLayout(compass_layout)

    def _send_command(self, component_id: str, action_type: str):
        command = {
            "type": "ComponentControl",
            "id": component_id,
            "action": {"type": action_type}
        }
        self.command_requested.emit(command)


class CliffIRControl(QWidget):
    """Cliff IR sensor control."""

    command_requested = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.enabled = False
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # Header with title and toggle
        header = QHBoxLayout()

        title = QLabel("Cliff Sensors")
        title.setFont(QFont("Arial", 10, QFont.Bold))
        title.setStyleSheet("color: #e0e0e0;")
        header.addWidget(title)

        header.addStretch()

        self.toggle_btn = QPushButton("OFF")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setFixedWidth(50)
        self.toggle_btn.clicked.connect(self._on_toggle)
        self._update_toggle_style()
        header.addWidget(self.toggle_btn)

        layout.addLayout(header)

        # Direction control row
        dir_layout = QHBoxLayout()
        dir_label = QLabel("Direction")
        dir_label.setStyleSheet("color: #aaa;")
        dir_layout.addWidget(dir_label)

        self.dir_spinbox = QSpinBox()
        self.dir_spinbox.setMinimum(0)
        self.dir_spinbox.setMaximum(255)
        self.dir_spinbox.setValue(0)
        self.dir_spinbox.setFocusPolicy(Qt.ClickFocus)
        self.dir_spinbox.setStyleSheet(
            "QSpinBox { background-color: #3a3a3a; color: #e0e0e0; border: 1px solid #555; }"
        )
        dir_layout.addWidget(self.dir_spinbox)

        self.dir_set_btn = QPushButton("Set")
        self.dir_set_btn.setFixedWidth(40)
        self.dir_set_btn.setStyleSheet("background-color: #555; color: #e0e0e0;")
        self.dir_set_btn.clicked.connect(self._on_set_direction)
        dir_layout.addWidget(self.dir_set_btn)

        layout.addLayout(dir_layout)

    def _on_toggle(self):
        self.enabled = self.toggle_btn.isChecked()
        self._update_toggle_style()

        if self.enabled:
            command = {
                "type": "ComponentControl",
                "id": "cliff_ir",
                "action": {"type": "Enable"}
            }
        else:
            command = {
                "type": "ComponentControl",
                "id": "cliff_ir",
                "action": {"type": "Disable"}
            }
        self.command_requested.emit(command)

    def _update_toggle_style(self):
        if self.enabled:
            self.toggle_btn.setText("ON")
            self.toggle_btn.setStyleSheet(
                "background-color: #4CAF50; color: white; font-weight: bold;"
            )
        else:
            self.toggle_btn.setText("OFF")
            self.toggle_btn.setStyleSheet(
                "background-color: #555; color: #ccc;"
            )

    def _on_set_direction(self):
        value = self.dir_spinbox.value()
        command = {
            "type": "ComponentControl",
            "id": "cliff_ir",
            "action": {
                "type": "Configure",
                "config": {"direction": {"U8": value}}
            }
        }
        self.command_requested.emit(command)


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
        title.setStyleSheet("color: #e0e0e0;")
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
        self.velocity_label.setStyleSheet("color: #aaa; font-size: 9px;")
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
                "background-color: #555; color: #ccc;"
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
        self.setFixedWidth(250)
        self._setup_ui()

    def _setup_ui(self):
        # Main layout with scroll area
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Scroll area for all controls (vertical only)
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

        # Content widget inside scroll area
        content = QWidget()
        content.setFixedWidth(242)  # Fixed width for content (250 - 8 for scrollbar)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Title
        title = QLabel("Device Control")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #e0e0e0;")
        layout.addWidget(title)

        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("background-color: #444;")
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
        line2.setStyleSheet("background-color: #444;")
        layout.addWidget(line2)

        # Dustbox status
        self.dustbox_status = DustboxStatus()
        self.dustbox_status.water_pump_changed.connect(self._on_water_pump_command)
        layout.addWidget(self.dustbox_status)

        # Lidar control
        self.lidar_control = LidarControl()
        self.lidar_control.lidar_toggled.connect(self._on_lidar_toggled)
        layout.addWidget(self.lidar_control)

        # Cliff IR control
        self.cliff_ir_control = CliffIRControl()
        self.cliff_ir_control.command_requested.connect(self._forward_command)
        layout.addWidget(self.cliff_ir_control)

        # Separator
        line3 = QFrame()
        line3.setFrameShape(QFrame.HLine)
        line3.setFrameShadow(QFrame.Sunken)
        line3.setStyleSheet("background-color: #444;")
        layout.addWidget(line3)

        # Calibration control
        self.calibration_control = CalibrationControl()
        self.calibration_control.command_requested.connect(self._forward_command)
        layout.addWidget(self.calibration_control)

        # Separator
        line4 = QFrame()
        line4.setFrameShape(QFrame.HLine)
        line4.setFrameShadow(QFrame.Sunken)
        line4.setStyleSheet("background-color: #444;")
        layout.addWidget(line4)

        # Power control
        self.power_control = PowerControl()
        self.power_control.command_requested.connect(self._forward_command)
        layout.addWidget(self.power_control)

        # Separator
        line5 = QFrame()
        line5.setFrameShape(QFrame.HLine)
        line5.setFrameShadow(QFrame.Sunken)
        line5.setStyleSheet("background-color: #444;")
        layout.addWidget(line5)

        # Drive control
        self.drive_control = DriveControl()
        self.drive_control.velocity_changed.connect(self._on_velocity_changed)
        self.drive_control.motor_toggled.connect(self._on_motor_toggled)
        layout.addWidget(self.drive_control)

        # Push everything to top
        layout.addStretch()

        # Set scroll area content
        scroll.setWidget(content)
        main_layout.addWidget(scroll)

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

    def _on_water_pump_command(self, speed: float):
        """Handle water pump control command using ComponentControl protocol."""
        if speed > 0:
            command = {
                "type": "ComponentControl",
                "id": "water_pump",
                "action": {
                    "type": "Configure",
                    "config": {"speed": {"U8": int(speed)}}
                }
            }
        else:
            command = {
                "type": "ComponentControl",
                "id": "water_pump",
                "action": {"type": "Disable"}
            }
        self.command_requested.emit(command)

    def update_sensors(self, data: dict):
        """Update control panel with sensor data."""
        # Update dustbox status
        dustbox_attached = data.get('dustbox_attached', True)
        # Water tank level from status offset 0x46 (0 or 100)
        water_level = data.get('water_tank_level', None)
        # Derive box type: if water_tank_level > 0, it's a 2-in-1 mop box
        # (water level is only reported when 2-in-1 box is attached)
        dustbox_type = "2in1" if water_level is not None and water_level > 0 else None
        self.dustbox_status.update_status(dustbox_attached, dustbox_type, water_level)

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

    def _forward_command(self, command: dict):
        """Forward a command from nested control widgets."""
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
