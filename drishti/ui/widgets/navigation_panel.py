"""
Navigation status panel widget.

Displays:
- Current navigation state
- Target description
- Progress (distance remaining, ETA)
- Path length and waypoint count
- Cancel button
- Instructions for Ctrl+Click

Emits:
- cancel_navigation: User clicked cancel button
"""

import logging
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QPushButton,
    QProgressBar, QGroupBox
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont

logger = logging.getLogger(__name__)


class NavigationPanel(QWidget):
    """Navigation status and control panel."""

    # Signal emitted when user clicks cancel
    cancel_navigation = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self._current_state = 'Idle'
        self._setup_ui()

    def _setup_ui(self):
        """Setup the UI components."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Title
        title = QLabel("Navigation")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        # Instructions
        instructions = QLabel("Ctrl+Click on map to set goal")
        instructions.setStyleSheet("color: #888; font-size: 11px; font-style: italic;")
        layout.addWidget(instructions)

        # Status frame
        status_frame = self._create_status_frame()
        layout.addWidget(status_frame)

        # Progress frame
        progress_frame = self._create_progress_frame()
        layout.addWidget(progress_frame)

        # Cancel button
        self._cancel_btn = QPushButton("Cancel Navigation")
        self._cancel_btn.setEnabled(False)
        self._cancel_btn.setStyleSheet("""
            QPushButton {
                background-color: #a04040;
                color: white;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #c04040; }
            QPushButton:disabled {
                background-color: #444;
                color: #666;
            }
        """)
        self._cancel_btn.clicked.connect(self._on_cancel_clicked)
        layout.addWidget(self._cancel_btn)

        layout.addStretch()

    def _create_status_frame(self) -> QFrame:
        """Create the status display frame."""
        frame = QGroupBox("Status")
        frame.setStyleSheet("""
            QGroupBox {
                color: #aaa;
                border: 1px solid #444;
                border-radius: 4px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QLabel { color: #ccc; }
        """)
        layout = QVBoxLayout(frame)
        layout.setSpacing(6)

        # State row
        state_layout = QHBoxLayout()
        state_layout.addWidget(QLabel("State:"))
        self._state_label = QLabel("Idle")
        self._state_label.setStyleSheet("color: #888; font-weight: bold;")
        state_layout.addWidget(self._state_label)
        state_layout.addStretch()
        layout.addLayout(state_layout)

        # Target row
        target_layout = QHBoxLayout()
        target_layout.addWidget(QLabel("Target:"))
        self._target_label = QLabel("-")
        self._target_label.setStyleSheet("color: #aaa;")
        self._target_label.setWordWrap(True)
        target_layout.addWidget(self._target_label, 1)
        layout.addLayout(target_layout)

        # Status message row
        msg_layout = QHBoxLayout()
        msg_layout.addWidget(QLabel("Status:"))
        self._message_label = QLabel("-")
        self._message_label.setStyleSheet("color: #aaa;")
        self._message_label.setWordWrap(True)
        msg_layout.addWidget(self._message_label, 1)
        layout.addLayout(msg_layout)

        return frame

    def _create_progress_frame(self) -> QFrame:
        """Create the progress display frame."""
        frame = QGroupBox("Progress")
        frame.setStyleSheet("""
            QGroupBox {
                color: #aaa;
                border: 1px solid #444;
                border-radius: 4px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QLabel { color: #ccc; }
        """)
        layout = QVBoxLayout(frame)
        layout.setSpacing(6)

        # Progress bar
        self._progress_bar = QProgressBar()
        self._progress_bar.setRange(0, 100)
        self._progress_bar.setValue(0)
        self._progress_bar.setTextVisible(True)
        self._progress_bar.setFormat("%p%")
        self._progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #444;
                border-radius: 3px;
                background-color: #333;
                text-align: center;
                color: white;
            }
            QProgressBar::chunk {
                background-color: #4a9;
                border-radius: 2px;
            }
        """)
        layout.addWidget(self._progress_bar)

        # Distance row
        dist_layout = QHBoxLayout()
        dist_layout.addWidget(QLabel("Remaining:"))
        self._distance_label = QLabel("-")
        self._distance_label.setStyleSheet("color: #aaa;")
        dist_layout.addWidget(self._distance_label)
        dist_layout.addStretch()
        layout.addLayout(dist_layout)

        # Path info row
        path_layout = QHBoxLayout()
        path_layout.addWidget(QLabel("Path:"))
        self._path_label = QLabel("-")
        self._path_label.setStyleSheet("color: #aaa;")
        path_layout.addWidget(self._path_label)
        path_layout.addStretch()
        layout.addLayout(path_layout)

        # ETA row
        eta_layout = QHBoxLayout()
        eta_layout.addWidget(QLabel("ETA:"))
        self._eta_label = QLabel("-")
        self._eta_label.setStyleSheet("color: #aaa;")
        eta_layout.addWidget(self._eta_label)
        eta_layout.addStretch()
        layout.addLayout(eta_layout)

        return frame

    def _on_cancel_clicked(self):
        """Handle cancel button click."""
        logger.info("Cancel navigation clicked")
        self.cancel_navigation.emit()

    def update_navigation_status(self, nav_data: dict):
        """Update the panel with navigation status data.

        Args:
            nav_data: Dict with state, goal_description, path, distance_remaining_m,
                      estimated_time_remaining_s, status_message, etc.
        """
        state = nav_data.get('state', 'Idle')
        self._current_state = state

        # Update state label with color coding
        state_colors = {
            'Idle': '#888',
            'Planning': '#ff9',
            'Navigating': '#4f4',
            'Rotating': '#4ff',
            'Reached': '#4f4',
            'Failed': '#f44',
            'Cancelled': '#888',
        }
        state_color = state_colors.get(state, '#aaa')
        self._state_label.setText(state)
        self._state_label.setStyleSheet(f"color: {state_color}; font-weight: bold;")

        # Update target
        goal_desc = nav_data.get('goal_description', '')
        goal = nav_data.get('goal')
        if goal_desc:
            self._target_label.setText(goal_desc)
        elif goal:
            self._target_label.setText(f"({goal['x']:.1f}, {goal['y']:.1f})")
        else:
            self._target_label.setText("-")

        # Update status message
        status_msg = nav_data.get('status_message', '')
        failure_reason = nav_data.get('failure_reason', '')
        if failure_reason and state == 'Failed':
            self._message_label.setText(failure_reason)
            self._message_label.setStyleSheet("color: #f88;")
        elif status_msg:
            self._message_label.setText(status_msg)
            self._message_label.setStyleSheet("color: #aaa;")
        else:
            self._message_label.setText("-")
            self._message_label.setStyleSheet("color: #aaa;")

        # Update progress
        path_length = nav_data.get('path_length_m', 0.0)
        distance_remaining = nav_data.get('distance_remaining_m', 0.0)
        eta = nav_data.get('estimated_time_remaining_s', 0.0)
        path = nav_data.get('path', [])
        current_wp = nav_data.get('current_waypoint_index', 0)

        # Calculate progress percentage
        if path_length > 0:
            progress = int((1.0 - distance_remaining / path_length) * 100)
            progress = max(0, min(100, progress))
        else:
            progress = 0

        self._progress_bar.setValue(progress)

        # Update distance
        if distance_remaining > 0:
            self._distance_label.setText(f"{distance_remaining:.2f} m")
        else:
            self._distance_label.setText("-")

        # Update path info
        if path:
            self._path_label.setText(f"{len(path)} pts, {path_length:.1f}m total")
        else:
            self._path_label.setText("-")

        # Update ETA
        if eta > 0:
            if eta < 60:
                self._eta_label.setText(f"{eta:.0f} sec")
            else:
                minutes = int(eta / 60)
                seconds = int(eta % 60)
                self._eta_label.setText(f"{minutes}:{seconds:02d}")
        else:
            self._eta_label.setText("-")

        # Enable/disable cancel button
        active_states = ('Planning', 'Navigating', 'Rotating')
        self._cancel_btn.setEnabled(state in active_states)

    def clear(self):
        """Clear all display fields."""
        self._state_label.setText("Idle")
        self._state_label.setStyleSheet("color: #888; font-weight: bold;")
        self._target_label.setText("-")
        self._message_label.setText("-")
        self._message_label.setStyleSheet("color: #aaa;")
        self._progress_bar.setValue(0)
        self._distance_label.setText("-")
        self._path_label.setText("-")
        self._eta_label.setText("-")
        self._cancel_btn.setEnabled(False)
