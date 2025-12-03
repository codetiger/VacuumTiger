"""
SLAM diagnostics panel for displaying timing, match quality, and component stats.
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QProgressBar, QGridLayout
)

from .collapsible_group import CollapsibleGroup


class TimingBar(QWidget):
    """Horizontal timing bar with label and value."""

    def __init__(self, name: str, max_value: int = 100000, parent=None):
        """
        Args:
            name: Display name for the timing component
            max_value: Maximum value in microseconds for the bar (default 100ms)
        """
        super().__init__(parent)
        self.name = name
        self.max_value = max_value

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        # Name label (fixed width)
        self.name_label = QLabel(name)
        self.name_label.setFixedWidth(80)
        self.name_label.setStyleSheet("color: #aaa; font-size: 10px;")
        layout.addWidget(self.name_label)

        # Progress bar
        self.bar = QProgressBar()
        self.bar.setMaximum(100)
        self.bar.setTextVisible(False)
        self.bar.setFixedHeight(12)
        self.bar.setStyleSheet("""
            QProgressBar {
                background-color: #2a2a2a;
                border: none;
                border-radius: 3px;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
                border-radius: 3px;
            }
        """)
        layout.addWidget(self.bar)

        # Value label
        self.value_label = QLabel("0 ms")
        self.value_label.setFixedWidth(60)
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value_label.setStyleSheet("color: #ccc; font-size: 10px;")
        layout.addWidget(self.value_label)

    def set_value(self, value_us: int):
        """Set the timing value in microseconds."""
        # Calculate percentage
        percent = min(100, int(value_us * 100 / self.max_value))
        self.bar.setValue(percent)

        # Format display value
        if value_us >= 1000:
            self.value_label.setText(f"{value_us / 1000:.1f} ms")
        else:
            self.value_label.setText(f"{value_us} us")

        # Color code based on percentage
        if percent < 50:
            color = "#4CAF50"  # Green
        elif percent < 80:
            color = "#FFC107"  # Yellow
        else:
            color = "#f44336"  # Red

        self.bar.setStyleSheet(f"""
            QProgressBar {{
                background-color: #2a2a2a;
                border: none;
                border-radius: 3px;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 3px;
            }}
        """)


class MatchQualityBar(QWidget):
    """Match quality display with color-coded progress bar."""

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        # Label
        self.label = QLabel("Match Score")
        self.label.setFixedWidth(80)
        self.label.setStyleSheet("color: #aaa; font-size: 10px;")
        layout.addWidget(self.label)

        # Progress bar
        self.bar = QProgressBar()
        self.bar.setMaximum(100)
        self.bar.setTextVisible(False)
        self.bar.setFixedHeight(12)
        layout.addWidget(self.bar)

        # Value label
        self.value_label = QLabel("0%")
        self.value_label.setFixedWidth(40)
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value_label.setStyleSheet("color: #ccc; font-size: 10px;")
        layout.addWidget(self.value_label)

        self.set_score(0.0)

    def set_score(self, score: float):
        """Set the match score (0.0 - 1.0)."""
        percent = int(score * 100)
        self.bar.setValue(percent)
        self.value_label.setText(f"{percent}%")

        # Color code based on score
        if score >= 0.7:
            color = "#4CAF50"  # Green
        elif score >= 0.3:
            color = "#FFC107"  # Yellow
        else:
            color = "#f44336"  # Red

        self.bar.setStyleSheet(f"""
            QProgressBar {{
                background-color: #2a2a2a;
                border: none;
                border-radius: 3px;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 3px;
            }}
        """)


class StatRow(QWidget):
    """Row showing a labeled statistic value."""

    def __init__(self, label: str, parent=None):
        super().__init__(parent)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self.label = QLabel(label)
        self.label.setStyleSheet("color: #aaa; font-size: 10px;")
        layout.addWidget(self.label)

        layout.addStretch()

        self.value = QLabel("-")
        self.value.setStyleSheet("color: #fff; font-size: 10px; font-weight: bold;")
        layout.addWidget(self.value)

    def set_value(self, value: str):
        """Set the displayed value."""
        self.value.setText(value)


class DiagnosticsPanel(QWidget):
    """
    Panel displaying SLAM diagnostics including timing, match quality,
    and component statistics.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMaximumWidth(300)
        self.setMinimumWidth(250)

        self._setup_ui()

    def _setup_ui(self):
        """Build the diagnostics panel UI."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Title
        title = QLabel("SLAM Diagnostics")
        title.setStyleSheet("""
            color: #fff;
            font-size: 12px;
            font-weight: bold;
            padding: 4px;
        """)
        layout.addWidget(title)

        # =====================================================================
        # Timing Section
        # =====================================================================
        timing_group = CollapsibleGroup("Timing", expanded=True)

        # Timing bars
        self.total_time_bar = TimingBar("Total", max_value=200000)
        timing_group.add_widget(self.total_time_bar)

        self.scan_match_bar = TimingBar("Scan Match", max_value=100000)
        timing_group.add_widget(self.scan_match_bar)

        self.map_update_bar = TimingBar("Map Update", max_value=50000)
        timing_group.add_widget(self.map_update_bar)

        self.keyframe_bar = TimingBar("Keyframe", max_value=10000)
        timing_group.add_widget(self.keyframe_bar)

        # Average time row
        self.avg_time_row = StatRow("Avg Cycle")
        timing_group.add_widget(self.avg_time_row)

        layout.addWidget(timing_group)

        # =====================================================================
        # Scan Match Section
        # =====================================================================
        match_group = CollapsibleGroup("Scan Matching", expanded=True)

        self.match_quality_bar = MatchQualityBar()
        match_group.add_widget(self.match_quality_bar)

        self.method_row = StatRow("Method")
        match_group.add_widget(self.method_row)

        self.iterations_row = StatRow("Iterations")
        match_group.add_widget(self.iterations_row)

        self.mse_row = StatRow("MSE")
        match_group.add_widget(self.mse_row)

        self.converged_row = StatRow("Converged")
        match_group.add_widget(self.converged_row)

        layout.addWidget(match_group)

        # =====================================================================
        # Mapping Section
        # =====================================================================
        mapping_group = CollapsibleGroup("Mapping", expanded=False)

        self.submaps_row = StatRow("Submaps")
        mapping_group.add_widget(self.submaps_row)

        self.active_submap_row = StatRow("Active Submap")
        mapping_group.add_widget(self.active_submap_row)

        self.memory_row = StatRow("Memory")
        mapping_group.add_widget(self.memory_row)

        layout.addWidget(mapping_group)

        # =====================================================================
        # Loop Closure Section
        # =====================================================================
        loop_group = CollapsibleGroup("Loop Closure", expanded=False)

        self.pose_graph_nodes_row = StatRow("Pose Graph Nodes")
        loop_group.add_widget(self.pose_graph_nodes_row)

        self.closures_row = StatRow("Closures")
        loop_group.add_widget(self.closures_row)

        layout.addWidget(loop_group)

        # Stretch at bottom
        layout.addStretch()

    def update_diagnostics(self, data: dict):
        """
        Update the panel with new diagnostics data.

        Args:
            data: Dictionary with timing, scan_match, mapping, loop_closure keys
        """
        # Timing
        timing = data.get('timing', {})
        self.total_time_bar.set_value(timing.get('total_us', 0))
        self.scan_match_bar.set_value(timing.get('scan_matching_us', 0))
        self.map_update_bar.set_value(timing.get('map_update_us', 0))
        self.keyframe_bar.set_value(timing.get('keyframe_check_us', 0))

        avg_us = timing.get('avg_total_us', 0)
        if avg_us >= 1000:
            self.avg_time_row.set_value(f"{avg_us / 1000:.1f} ms")
        else:
            self.avg_time_row.set_value(f"{avg_us:.0f} us")

        # Scan match stats
        scan_match = data.get('scan_match', {})
        self.match_quality_bar.set_score(scan_match.get('score', 0.0))
        self.method_row.set_value(scan_match.get('method', 'none'))
        self.iterations_row.set_value(str(scan_match.get('iterations', 0)))
        self.mse_row.set_value(f"{scan_match.get('mse', 0.0):.4f}")
        self.converged_row.set_value("Yes" if scan_match.get('converged', False) else "No")

        # Mapping stats
        mapping = data.get('mapping', {})
        self.submaps_row.set_value(str(mapping.get('num_submaps', 0)))
        active_id = mapping.get('active_submap_id')
        self.active_submap_row.set_value(str(active_id) if active_id is not None else "-")

        memory_bytes = mapping.get('map_size_bytes', 0)
        if memory_bytes >= 1024 * 1024:
            self.memory_row.set_value(f"{memory_bytes / (1024*1024):.1f} MB")
        elif memory_bytes >= 1024:
            self.memory_row.set_value(f"{memory_bytes / 1024:.1f} KB")
        else:
            self.memory_row.set_value(f"{memory_bytes} B")

        # Loop closure stats
        loop_closure = data.get('loop_closure', {})
        self.pose_graph_nodes_row.set_value(str(loop_closure.get('pose_graph_nodes', 0)))
        self.closures_row.set_value(str(loop_closure.get('closures_accepted', 0)))
