"""
Control panel for manual robot operation.

Provides buttons and sliders for motion control, actuators, and lidar.
"""

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
                              QPushButton, QSlider, QLabel, QGridLayout)
from PyQt5.QtGui import QFont


class ControlPanel(QWidget):
    """Manual control panel for robot operation"""

    # Signals emitted when controls are used
    velocity_command = pyqtSignal(float, float)  # left_ticks, right_ticks
    side_brush_command = pyqtSignal(int)  # speed 0-100%
    main_brush_command = pyqtSignal(int)  # speed 0-100%
    air_pump_command = pyqtSignal(int)  # speed 0-100%
    lidar_enable_command = pyqtSignal(int)  # pwm 0-100%
    lidar_disable_command = pyqtSignal()
    emergency_stop_command = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # Main layout
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        # Title
        title = QLabel("Robot Control Panel")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setStyleSheet("color: white; padding: 10px;")
        main_layout.addWidget(title)

        # Motion Control Section
        motion_group = self._create_motion_control()
        main_layout.addWidget(motion_group)

        # Actuator Control Section
        actuator_group = self._create_actuator_control()
        main_layout.addWidget(actuator_group)

        # Lidar Control Section
        lidar_group = self._create_lidar_control()
        main_layout.addWidget(lidar_group)

        # Emergency Stop (prominent)
        main_layout.addStretch()
        stop_btn = QPushButton("⚠ EMERGENCY STOP ⚠")
        stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: white;
                font-size: 16pt;
                font-weight: bold;
                padding: 20px;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #f44336;
            }
            QPushButton:pressed {
                background-color: #b71c1c;
            }
        """)
        stop_btn.clicked.connect(self.emergency_stop_command.emit)
        main_layout.addWidget(stop_btn)

    def _create_motion_control(self):
        """Create motion control buttons"""
        group = QGroupBox("Motion Control")
        group.setStyleSheet("QGroupBox { color: white; font-weight: bold; }")
        layout = QVBoxLayout()

        # Direction buttons grid
        grid = QGridLayout()

        # Forward button
        forward_btn = QPushButton("↑\nForward")
        forward_btn.clicked.connect(lambda: self.velocity_command.emit(1000.0, 1000.0))
        grid.addWidget(forward_btn, 0, 1)

        # Left button
        left_btn = QPushButton("←\nLeft")
        left_btn.clicked.connect(lambda: self.velocity_command.emit(-500.0, 500.0))
        grid.addWidget(left_btn, 1, 0)

        # Stop button
        stop_btn = QPushButton("■\nStop")
        stop_btn.setStyleSheet("background-color: #ff6b6b;")
        stop_btn.clicked.connect(lambda: self.velocity_command.emit(0.0, 0.0))
        grid.addWidget(stop_btn, 1, 1)

        # Right button
        right_btn = QPushButton("→\nRight")
        right_btn.clicked.connect(lambda: self.velocity_command.emit(500.0, -500.0))
        grid.addWidget(right_btn, 1, 2)

        # Backward button
        backward_btn = QPushButton("↓\nBackward")
        backward_btn.clicked.connect(lambda: self.velocity_command.emit(-1000.0, -1000.0))
        grid.addWidget(backward_btn, 2, 1)

        # Style buttons
        for i in range(grid.count()):
            widget = grid.itemAt(i).widget()
            if widget:
                widget.setMinimumHeight(60)
                widget.setStyleSheet("""
                    QPushButton {
                        background-color: #4a4a4a;
                        color: white;
                        font-size: 11pt;
                        border-radius: 5px;
                    }
                    QPushButton:hover {
                        background-color: #5a5a5a;
                    }
                    QPushButton:pressed {
                        background-color: #3a3a3a;
                    }
                """)

        layout.addLayout(grid)

        # Velocity info
        info_label = QLabel("Forward/Back: ±1000 ticks/sec | Turn: ±500 ticks/sec")
        info_label.setStyleSheet("color: #aaa; font-size: 8pt;")
        info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(info_label)

        group.setLayout(layout)
        return group

    def _create_actuator_control(self):
        """Create actuator control sliders"""
        group = QGroupBox("Actuator Control")
        group.setStyleSheet("QGroupBox { color: white; font-weight: bold; }")
        layout = QVBoxLayout()

        # Side brush slider
        side_layout = QHBoxLayout()
        side_label = QLabel("Side Brush:")
        side_label.setStyleSheet("color: white;")
        self.side_brush_slider = QSlider(Qt.Horizontal)
        self.side_brush_slider.setRange(0, 100)
        self.side_brush_slider.setValue(0)
        self.side_brush_value_label = QLabel("0%")
        self.side_brush_value_label.setStyleSheet("color: white; min-width: 40px;")
        self.side_brush_slider.valueChanged.connect(
            lambda v: (self.side_brush_value_label.setText(f"{v}%"),
                       self.side_brush_command.emit(v))
        )
        side_layout.addWidget(side_label)
        side_layout.addWidget(self.side_brush_slider)
        side_layout.addWidget(self.side_brush_value_label)
        layout.addLayout(side_layout)

        # Main brush slider
        main_layout = QHBoxLayout()
        main_label = QLabel("Main Brush:")
        main_label.setStyleSheet("color: white;")
        self.main_brush_slider = QSlider(Qt.Horizontal)
        self.main_brush_slider.setRange(0, 100)
        self.main_brush_slider.setValue(0)
        self.main_brush_value_label = QLabel("0%")
        self.main_brush_value_label.setStyleSheet("color: white; min-width: 40px;")
        self.main_brush_slider.valueChanged.connect(
            lambda v: (self.main_brush_value_label.setText(f"{v}%"),
                       self.main_brush_command.emit(v))
        )
        main_layout.addWidget(main_label)
        main_layout.addWidget(self.main_brush_slider)
        main_layout.addWidget(self.main_brush_value_label)
        layout.addLayout(main_layout)

        # Air pump slider
        pump_layout = QHBoxLayout()
        pump_label = QLabel("Air Pump:")
        pump_label.setStyleSheet("color: white;")
        self.air_pump_slider = QSlider(Qt.Horizontal)
        self.air_pump_slider.setRange(0, 100)
        self.air_pump_slider.setValue(0)
        self.air_pump_value_label = QLabel("0%")
        self.air_pump_value_label.setStyleSheet("color: white; min-width: 40px;")
        self.air_pump_slider.valueChanged.connect(
            lambda v: (self.air_pump_value_label.setText(f"{v}%"),
                       self.air_pump_command.emit(v))
        )
        pump_layout.addWidget(pump_label)
        pump_layout.addWidget(self.air_pump_slider)
        pump_layout.addWidget(self.air_pump_value_label)
        layout.addLayout(pump_layout)

        group.setLayout(layout)
        return group

    def _create_lidar_control(self):
        """Create lidar control section"""
        group = QGroupBox("Lidar Control")
        group.setStyleSheet("QGroupBox { color: white; font-weight: bold; }")
        layout = QVBoxLayout()

        # Enable/Disable buttons
        button_layout = QHBoxLayout()
        enable_btn = QPushButton("Enable Lidar")
        enable_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #5BC05B;
            }
        """)
        enable_btn.clicked.connect(lambda: self.lidar_enable_command.emit(self.lidar_pwm_slider.value()))

        disable_btn = QPushButton("Disable Lidar")
        disable_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #ff5a4e;
            }
        """)
        disable_btn.clicked.connect(self.lidar_disable_command.emit)

        button_layout.addWidget(enable_btn)
        button_layout.addWidget(disable_btn)
        layout.addLayout(button_layout)

        # PWM slider
        pwm_layout = QHBoxLayout()
        pwm_label = QLabel("PWM Speed:")
        pwm_label.setStyleSheet("color: white;")
        self.lidar_pwm_slider = QSlider(Qt.Horizontal)
        self.lidar_pwm_slider.setRange(0, 100)
        self.lidar_pwm_slider.setValue(80)
        self.lidar_pwm_value_label = QLabel("80%")
        self.lidar_pwm_value_label.setStyleSheet("color: white; min-width: 40px;")
        self.lidar_pwm_slider.valueChanged.connect(
            lambda v: self.lidar_pwm_value_label.setText(f"{v}%")
        )
        pwm_layout.addWidget(pwm_label)
        pwm_layout.addWidget(self.lidar_pwm_slider)
        pwm_layout.addWidget(self.lidar_pwm_value_label)
        layout.addLayout(pwm_layout)

        group.setLayout(layout)
        return group
