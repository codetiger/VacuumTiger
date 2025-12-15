"""
SLAM control panel widget for DhruvaSLAM.

Provides UI controls for:
- Mapping control (start, stop, clear)
- Map management (list, enable, rename, delete)
- Status display (state, progress, localization confidence)
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QFrame, QProgressBar, QLineEdit,
    QScrollArea, QListWidget, QListWidgetItem, QInputDialog,
    QMessageBox
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont

from .collapsible_group import CollapsibleGroup


class StatusDisplay(QWidget):
    """Displays current SLAM status."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        # State row
        state_row = QHBoxLayout()
        state_label = QLabel("State:")
        state_label.setStyleSheet("color: #aaa;")
        state_label.setFixedWidth(80)
        state_row.addWidget(state_label)

        self.state_value = QLabel("Idle")
        self.state_value.setStyleSheet("color: #4CAF50; font-weight: bold;")
        state_row.addWidget(self.state_value)
        state_row.addStretch()
        layout.addLayout(state_row)

        # Active map row
        map_row = QHBoxLayout()
        map_label = QLabel("Active Map:")
        map_label.setStyleSheet("color: #aaa;")
        map_label.setFixedWidth(80)
        map_row.addWidget(map_label)

        self.map_value = QLabel("None")
        self.map_value.setStyleSheet("color: #ccc;")
        map_row.addWidget(self.map_value)
        map_row.addStretch()
        layout.addLayout(map_row)

        # Pose row
        pose_row = QHBoxLayout()
        pose_label = QLabel("Pose:")
        pose_label.setStyleSheet("color: #aaa;")
        pose_label.setFixedWidth(80)
        pose_row.addWidget(pose_label)

        self.pose_value = QLabel("(0.00, 0.00, 0.0°)")
        self.pose_value.setStyleSheet("color: #ccc; font-family: monospace;")
        pose_row.addWidget(self.pose_value)
        pose_row.addStretch()
        layout.addLayout(pose_row)

        # Confidence bar (for localization)
        conf_row = QHBoxLayout()
        conf_label = QLabel("Confidence:")
        conf_label.setStyleSheet("color: #aaa;")
        conf_label.setFixedWidth(80)
        conf_row.addWidget(conf_label)

        self.confidence_bar = QProgressBar()
        self.confidence_bar.setRange(0, 100)
        self.confidence_bar.setValue(0)
        self.confidence_bar.setTextVisible(True)
        self.confidence_bar.setStyleSheet("""
            QProgressBar {
                background-color: #333;
                border: 1px solid #555;
                border-radius: 3px;
                text-align: center;
                color: white;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
            }
        """)
        conf_row.addWidget(self.confidence_bar)
        layout.addLayout(conf_row)

    def update_status(self, data: dict):
        """Update status display from robot_status data."""
        state = data.get('state', 'Unknown')
        self.state_value.setText(state)

        # Color code state
        state_colors = {
            'Idle': '#777',
            'Mapping': '#4CAF50',
            'Localizing': '#2196F3',
            'Lost': '#f44336',
        }
        color = state_colors.get(state, '#ccc')
        self.state_value.setStyleSheet(f"color: {color}; font-weight: bold;")

        # Active map
        map_id = data.get('active_map_id', '')
        self.map_value.setText(map_id if map_id else "None")

        # Pose
        pose = data.get('pose', {})
        x = pose.get('x', 0.0)
        y = pose.get('y', 0.0)
        theta = pose.get('theta', 0.0)
        import math
        theta_deg = math.degrees(theta)
        self.pose_value.setText(f"({x:.2f}, {y:.2f}, {theta_deg:.1f}°)")

        # Confidence
        confidence = data.get('localization_confidence', 0.0)
        self.confidence_bar.setValue(int(confidence * 100))

        # Color code confidence bar
        if confidence > 0.7:
            self.confidence_bar.setStyleSheet("""
                QProgressBar { background-color: #333; border: 1px solid #555; border-radius: 3px; text-align: center; color: white; }
                QProgressBar::chunk { background-color: #4CAF50; }
            """)
        elif confidence > 0.3:
            self.confidence_bar.setStyleSheet("""
                QProgressBar { background-color: #333; border: 1px solid #555; border-radius: 3px; text-align: center; color: white; }
                QProgressBar::chunk { background-color: #FFC107; }
            """)
        else:
            self.confidence_bar.setStyleSheet("""
                QProgressBar { background-color: #333; border: 1px solid #555; border-radius: 3px; text-align: center; color: white; }
                QProgressBar::chunk { background-color: #f44336; }
            """)


class MappingProgress(QWidget):
    """Displays mapping progress."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        # Distance traveled
        dist_row = QHBoxLayout()
        dist_label = QLabel("Distance:")
        dist_label.setStyleSheet("color: #aaa;")
        dist_label.setFixedWidth(80)
        dist_row.addWidget(dist_label)

        self.distance_value = QLabel("0.0 m")
        self.distance_value.setStyleSheet("color: #ccc;")
        dist_row.addWidget(self.distance_value)
        dist_row.addStretch()
        layout.addLayout(dist_row)

        # Keyframes
        kf_row = QHBoxLayout()
        kf_label = QLabel("Keyframes:")
        kf_label.setStyleSheet("color: #aaa;")
        kf_label.setFixedWidth(80)
        kf_row.addWidget(kf_label)

        self.keyframes_value = QLabel("0")
        self.keyframes_value.setStyleSheet("color: #ccc;")
        kf_row.addWidget(self.keyframes_value)
        kf_row.addStretch()
        layout.addLayout(kf_row)

        # Loop closures
        lc_row = QHBoxLayout()
        lc_label = QLabel("Loops:")
        lc_label.setStyleSheet("color: #aaa;")
        lc_label.setFixedWidth(80)
        lc_row.addWidget(lc_label)

        self.loops_value = QLabel("0")
        self.loops_value.setStyleSheet("color: #ccc;")
        lc_row.addWidget(self.loops_value)
        lc_row.addStretch()
        layout.addLayout(lc_row)

        # Map area
        area_row = QHBoxLayout()
        area_label = QLabel("Map Area:")
        area_label.setStyleSheet("color: #aaa;")
        area_label.setFixedWidth(80)
        area_row.addWidget(area_label)

        self.area_value = QLabel("0.0 m²")
        self.area_value.setStyleSheet("color: #ccc;")
        area_row.addWidget(self.area_value)
        area_row.addStretch()
        layout.addLayout(area_row)

    def update_progress(self, data: dict):
        """Update progress display from robot_status data."""
        distance = data.get('distance_traveled_m', 0.0)
        self.distance_value.setText(f"{distance:.1f} m")

        keyframes = data.get('keyframe_count', 0)
        self.keyframes_value.setText(str(keyframes))

        loops = data.get('loop_closures', 0)
        self.loops_value.setText(str(loops))

        area = data.get('map_area_m2', 0.0)
        self.area_value.setText(f"{area:.1f} m²")


class MappingControls(QWidget):
    """Mapping start/stop/clear controls."""

    start_mapping = pyqtSignal(str)  # map_name
    stop_mapping = pyqtSignal(bool)  # save
    clear_map = pyqtSignal()
    emergency_stop = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._is_mapping = False
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        btn_style = "background-color: #555; color: #e0e0e0; padding: 6px 12px; border-radius: 3px;"
        btn_start = "background-color: #4CAF50; color: white; padding: 6px 12px; border-radius: 3px; font-weight: bold;"
        btn_stop = "background-color: #FF9800; color: white; padding: 6px 12px; border-radius: 3px; font-weight: bold;"
        btn_danger = "background-color: #f44336; color: white; padding: 6px 12px; border-radius: 3px; font-weight: bold;"

        # Map name input
        name_row = QHBoxLayout()
        name_label = QLabel("Name:")
        name_label.setStyleSheet("color: #aaa;")
        name_label.setFixedWidth(50)
        name_row.addWidget(name_label)

        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Optional map name")
        self.name_input.setStyleSheet(
            "QLineEdit { background-color: #3a3a3a; color: #e0e0e0; border: 1px solid #555; padding: 4px; }"
        )
        name_row.addWidget(self.name_input)
        layout.addLayout(name_row)

        # Toggle Start/Stop button
        btn_row = QHBoxLayout()

        self.toggle_btn = QPushButton("Start Mapping")
        self.toggle_btn.setStyleSheet(btn_start)
        self.toggle_btn.clicked.connect(self._on_toggle)
        btn_row.addWidget(self.toggle_btn)

        layout.addLayout(btn_row)

        # Clear Map and Emergency Stop buttons
        action_row = QHBoxLayout()

        self.clear_btn = QPushButton("Clear Map")
        self.clear_btn.setStyleSheet(btn_style)
        self.clear_btn.clicked.connect(self._on_clear)
        self.clear_btn.setVisible(False)  # Hidden until mapping is complete/stopped
        action_row.addWidget(self.clear_btn)

        self.emergency_btn = QPushButton("EMERGENCY STOP")
        self.emergency_btn.setStyleSheet(btn_danger)
        self.emergency_btn.clicked.connect(self._on_emergency_stop)
        action_row.addWidget(self.emergency_btn)

        layout.addLayout(action_row)

        # Store styles for toggling
        self._btn_start_style = btn_start
        self._btn_stop_style = btn_stop

    def _on_toggle(self):
        if self._is_mapping:
            # Currently mapping - stop and save
            self.stop_mapping.emit(True)
        else:
            # Not mapping - start
            name = self.name_input.text().strip()
            self.start_mapping.emit(name)

    def _on_clear(self):
        reply = QMessageBox.question(
            self, "Clear Map",
            "Are you sure you want to clear the current map?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.clear_map.emit()

    def _on_emergency_stop(self):
        # No confirmation - emergency stop should be immediate
        self.emergency_stop.emit()

    def set_mapping_state(self, is_mapping: bool):
        """Update button states based on mapping state."""
        self._is_mapping = is_mapping

        if is_mapping:
            self.toggle_btn.setText("Stop Mapping")
            self.toggle_btn.setStyleSheet(self._btn_stop_style)
            self.clear_btn.setVisible(False)
        else:
            self.toggle_btn.setText("Start Mapping")
            self.toggle_btn.setStyleSheet(self._btn_start_style)
            # Show clear button when not mapping (to clear a completed/stopped map)
            self.clear_btn.setVisible(True)

        self.name_input.setEnabled(not is_mapping)


class MapListWidget(QWidget):
    """Widget to display and manage saved maps."""

    enable_map = pyqtSignal(str)  # map_id
    rename_map = pyqtSignal(str, str)  # map_id, new_name
    delete_map = pyqtSignal(str)  # map_id

    def __init__(self, parent=None):
        super().__init__(parent)
        self._maps = []
        self._active_map_id = ""
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        # Map list
        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet("""
            QListWidget {
                background-color: #2a2a2a;
                border: 1px solid #444;
                border-radius: 3px;
            }
            QListWidget::item {
                color: #ccc;
                padding: 6px;
                border-bottom: 1px solid #333;
            }
            QListWidget::item:selected {
                background-color: #3a3a3a;
                color: white;
            }
            QListWidget::item:hover {
                background-color: #333;
            }
        """)
        self.list_widget.setMinimumHeight(120)
        layout.addWidget(self.list_widget)

        # Action buttons
        btn_row = QHBoxLayout()
        btn_style = "background-color: #555; color: #e0e0e0; padding: 4px 8px; border-radius: 3px; font-size: 11px;"

        self.enable_btn = QPushButton("Enable")
        self.enable_btn.setStyleSheet(btn_style)
        self.enable_btn.clicked.connect(self._on_enable)
        btn_row.addWidget(self.enable_btn)

        self.rename_btn = QPushButton("Rename")
        self.rename_btn.setStyleSheet(btn_style)
        self.rename_btn.clicked.connect(self._on_rename)
        btn_row.addWidget(self.rename_btn)

        self.delete_btn = QPushButton("Delete")
        self.delete_btn.setStyleSheet("background-color: #8B4513; color: #e0e0e0; padding: 4px 8px; border-radius: 3px; font-size: 11px;")
        self.delete_btn.clicked.connect(self._on_delete)
        btn_row.addWidget(self.delete_btn)

        layout.addLayout(btn_row)

    def update_maps(self, data: dict):
        """Update map list from map_list data."""
        self._maps = data.get('maps', [])
        self._active_map_id = data.get('active_map_id', '')

        self.list_widget.clear()
        for m in self._maps:
            map_id = m.get('map_id', '')
            name = m.get('name', map_id)
            area = m.get('area_m2', 0.0)
            is_complete = m.get('is_complete', False)

            # Format display text
            status = "✓" if is_complete else "…"
            active = " [ACTIVE]" if map_id == self._active_map_id else ""
            text = f"{status} {name} ({area:.1f} m²){active}"

            item = QListWidgetItem(text)
            item.setData(Qt.UserRole, map_id)
            if map_id == self._active_map_id:
                item.setForeground(Qt.green)
            self.list_widget.addItem(item)

    def _get_selected_map_id(self) -> str:
        """Get currently selected map ID."""
        item = self.list_widget.currentItem()
        if item:
            return item.data(Qt.UserRole)
        return None

    def _on_enable(self):
        map_id = self._get_selected_map_id()
        if map_id:
            self.enable_map.emit(map_id)

    def _on_rename(self):
        map_id = self._get_selected_map_id()
        if not map_id:
            return

        # Find current name
        current_name = map_id
        for m in self._maps:
            if m.get('map_id') == map_id:
                current_name = m.get('name', map_id)
                break

        new_name, ok = QInputDialog.getText(
            self, "Rename Map",
            "Enter new name:",
            text=current_name
        )
        if ok and new_name.strip():
            self.rename_map.emit(map_id, new_name.strip())

    def _on_delete(self):
        map_id = self._get_selected_map_id()
        if not map_id:
            return

        reply = QMessageBox.question(
            self, "Delete Map",
            f"Are you sure you want to delete map '{map_id}'?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.delete_map.emit(map_id)


class SlamControlPanel(QWidget):
    """SLAM control panel with status, mapping controls, and map management."""

    # Signals for SLAM commands
    start_mapping_requested = pyqtSignal(str)  # map_name
    stop_mapping_requested = pyqtSignal(bool)  # save
    clear_map_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    enable_map_requested = pyqtSignal(str)  # map_id
    rename_map_requested = pyqtSignal(str, str)  # map_id, new_name
    delete_map_requested = pyqtSignal(str)  # map_id

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(280)
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
        content.setFixedWidth(272)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(6)

        # === Status Group ===
        self.status_group = CollapsibleGroup("SLAM Status")
        self.status_display = StatusDisplay()
        self.status_group.add_widget(self.status_display)
        layout.addWidget(self.status_group)

        # === Mapping Progress Group ===
        self.progress_group = CollapsibleGroup("Mapping Progress")
        self.mapping_progress = MappingProgress()
        self.progress_group.add_widget(self.mapping_progress)
        layout.addWidget(self.progress_group)

        # === Mapping Controls Group ===
        self.controls_group = CollapsibleGroup("Mapping Controls")
        self.mapping_controls = MappingControls()
        self.mapping_controls.start_mapping.connect(self.start_mapping_requested)
        self.mapping_controls.stop_mapping.connect(self.stop_mapping_requested)
        self.mapping_controls.clear_map.connect(self.clear_map_requested)
        self.mapping_controls.emergency_stop.connect(self.emergency_stop_requested)
        self.controls_group.add_widget(self.mapping_controls)
        layout.addWidget(self.controls_group)

        # === Saved Maps Group ===
        self.maps_group = CollapsibleGroup("Saved Maps")
        self.map_list = MapListWidget()
        self.map_list.enable_map.connect(self.enable_map_requested)
        self.map_list.rename_map.connect(self.rename_map_requested)
        self.map_list.delete_map.connect(self.delete_map_requested)
        self.maps_group.add_widget(self.map_list)
        layout.addWidget(self.maps_group)

        # === Connection Status ===
        self.connection_frame = QFrame()
        self.connection_frame.setStyleSheet("""
            QFrame {
                background-color: #252525;
                border-top: 1px solid #444;
                padding: 5px;
            }
        """)
        conn_layout = QHBoxLayout(self.connection_frame)
        conn_layout.setContentsMargins(5, 5, 5, 5)

        self.connection_dot = QLabel()
        self.connection_dot.setFixedSize(10, 10)
        self.connection_dot.setStyleSheet("background-color: #666; border-radius: 5px;")
        conn_layout.addWidget(self.connection_dot)

        self.connection_label = QLabel("Disconnected")
        self.connection_label.setStyleSheet("color: #777;")
        conn_layout.addWidget(self.connection_label)
        conn_layout.addStretch()

        layout.addStretch()
        scroll.setWidget(content)
        main_layout.addWidget(scroll)
        main_layout.addWidget(self.connection_frame)

    def update_robot_status(self, data: dict):
        """Update from robot_status data."""
        self.status_display.update_status(data)
        self.mapping_progress.update_progress(data)

        # Update mapping controls state
        state = data.get('state', 'Idle')
        is_mapping = state == 'Mapping'
        self.mapping_controls.set_mapping_state(is_mapping)

    def update_map_list(self, data: dict):
        """Update from map_list data."""
        self.map_list.update_maps(data)

    def set_connection_status(self, connected: bool, message: str = ""):
        """Update connection status display."""
        if connected:
            self.connection_dot.setStyleSheet("background-color: #4CAF50; border-radius: 5px;")
            self.connection_label.setText("Connected")
            self.connection_label.setStyleSheet("color: #4CAF50;")
        else:
            self.connection_dot.setStyleSheet("background-color: #f44336; border-radius: 5px;")
            self.connection_label.setText(message or "Disconnected")
            self.connection_label.setStyleSheet("color: #f44336;")
