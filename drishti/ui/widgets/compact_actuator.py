"""
Compact actuator controls for horizontal layout.
"""

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider
)


class MiniActuator(QWidget):
    """
    Compact single actuator control with toggle + slider.

    Stacked vertically: Label, Toggle button, Speed slider
    Width: ~70px to fit 3 in a row
    """

    command_changed = pyqtSignal(str, int)  # actuator_id, speed (0-100)

    def __init__(self, actuator_id: str, label: str, parent=None):
        super().__init__(parent)
        self.actuator_id = actuator_id
        self._enabled = False
        self._speed = 0

        self._setup_ui(label)

    def _setup_ui(self, label: str):
        """Set up the widget layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(4)
        layout.setAlignment(Qt.AlignCenter)

        # Label
        self._label = QLabel(label)
        self._label.setStyleSheet("""
            color: #aaa;
            font-size: 9px;
            background: transparent;
        """)
        self._label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self._label)

        # Toggle button
        self._toggle_btn = QPushButton("OFF")
        self._toggle_btn.setFixedSize(50, 22)
        self._toggle_btn.setCheckable(True)
        self._toggle_btn.clicked.connect(self._on_toggle)
        self._update_toggle_style()
        layout.addWidget(self._toggle_btn, alignment=Qt.AlignCenter)

        # Speed slider (horizontal)
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, 4)  # 0%, 25%, 50%, 75%, 100%
        self._slider.setValue(0)
        self._slider.setFixedWidth(60)
        self._slider.setFixedHeight(16)
        self._slider.setStyleSheet("""
            QSlider::groove:horizontal {
                background: #444;
                height: 6px;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #888;
                width: 12px;
                margin: -4px 0;
                border-radius: 6px;
            }
            QSlider::handle:horizontal:hover {
                background: #aaa;
            }
            QSlider::sub-page:horizontal {
                background: #4CAF50;
                border-radius: 3px;
            }
        """)
        self._slider.valueChanged.connect(self._on_slider_changed)
        layout.addWidget(self._slider, alignment=Qt.AlignCenter)

        # Speed label
        self._speed_label = QLabel("0%")
        self._speed_label.setStyleSheet("""
            color: #888;
            font-size: 8px;
            background: transparent;
        """)
        self._speed_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self._speed_label)

    def _update_toggle_style(self):
        """Update toggle button appearance based on state."""
        if self._enabled:
            self._toggle_btn.setText("ON")
            self._toggle_btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    border-radius: 3px;
                    font-size: 10px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #66BB6A; }
            """)
        else:
            self._toggle_btn.setText("OFF")
            self._toggle_btn.setStyleSheet("""
                QPushButton {
                    background-color: #555;
                    color: #ccc;
                    border: none;
                    border-radius: 3px;
                    font-size: 10px;
                }
                QPushButton:hover { background-color: #666; }
            """)

    def _on_toggle(self, checked: bool):
        """Handle toggle button click."""
        self._enabled = checked
        self._update_toggle_style()

        if self._enabled:
            # Turn on at current slider position or 100% if at 0
            if self._slider.value() == 0:
                self._slider.setValue(4)  # 100%
            self._speed = self._slider.value() * 25
        else:
            self._speed = 0

        self._speed_label.setText(f"{self._speed}%")
        self.command_changed.emit(self.actuator_id, self._speed)

    def _on_slider_changed(self, value: int):
        """Handle slider value change."""
        self._speed = value * 25
        self._speed_label.setText(f"{self._speed}%")

        # Auto-enable when slider moves above 0
        if value > 0 and not self._enabled:
            self._enabled = True
            self._toggle_btn.setChecked(True)
            self._update_toggle_style()
        # Auto-disable when slider moves to 0
        elif value == 0 and self._enabled:
            self._enabled = False
            self._toggle_btn.setChecked(False)
            self._update_toggle_style()

        self.command_changed.emit(self.actuator_id, self._speed)

    def set_state(self, enabled: bool, speed: int = 100):
        """Set the actuator state programmatically."""
        self._enabled = enabled
        self._speed = speed if enabled else 0
        self._toggle_btn.setChecked(enabled)
        self._slider.setValue(self._speed // 25)
        self._speed_label.setText(f"{self._speed}%")
        self._update_toggle_style()

    def is_enabled(self) -> bool:
        """Return whether the actuator is enabled."""
        return self._enabled

    def get_speed(self) -> int:
        """Return the current speed (0-100)."""
        return self._speed


class CompactActuatorRow(QWidget):
    """
    Horizontal row of 3 compact actuator controls.

    Displays Vacuum, Side Brush, and Main Brush in a single row.
    """

    command_changed = pyqtSignal(str, int)  # actuator_id, speed

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        # Vacuum
        self.vacuum = MiniActuator("vacuum", "Vacuum")
        self.vacuum.command_changed.connect(self._forward_command)
        layout.addWidget(self.vacuum)

        # Side Brush
        self.side_brush = MiniActuator("side_brush", "Side")
        self.side_brush.command_changed.connect(self._forward_command)
        layout.addWidget(self.side_brush)

        # Main Brush
        self.main_brush = MiniActuator("main_brush", "Main")
        self.main_brush.command_changed.connect(self._forward_command)
        layout.addWidget(self.main_brush)

    def _forward_command(self, actuator_id: str, speed: int):
        """Forward command from child actuators."""
        self.command_changed.emit(actuator_id, speed)

    def set_actuator_state(self, actuator_id: str, enabled: bool, speed: int = 100):
        """Set state of a specific actuator."""
        if actuator_id == "vacuum":
            self.vacuum.set_state(enabled, speed)
        elif actuator_id == "side_brush":
            self.side_brush.set_state(enabled, speed)
        elif actuator_id == "main_brush" or actuator_id == "brush":
            self.main_brush.set_state(enabled, speed)
