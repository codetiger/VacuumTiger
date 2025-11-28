"""
Collapsible group widget for organizing control panel sections.
"""

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame


class CollapsibleGroup(QWidget):
    """
    A collapsible container widget with a clickable header.

    Click the header to expand/collapse the content area.
    """

    toggled = pyqtSignal(bool)  # Emits True when expanded, False when collapsed

    def __init__(self, title: str, parent=None, expanded: bool = True):
        super().__init__(parent)
        self._expanded = expanded
        self._title = title

        self._setup_ui()
        self._update_arrow()

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Header (clickable)
        self._header = QFrame()
        self._header.setStyleSheet("""
            QFrame {
                background-color: #3a3a3a;
                border-radius: 4px;
                padding: 0px;
            }
            QFrame:hover {
                background-color: #454545;
            }
        """)
        self._header.setCursor(Qt.PointingHandCursor)
        self._header.setFixedHeight(28)
        self._header.mousePressEvent = self._on_header_click

        header_layout = QHBoxLayout(self._header)
        header_layout.setContentsMargins(10, 4, 10, 4)
        header_layout.setSpacing(8)

        # Arrow indicator
        self._arrow_label = QLabel()
        self._arrow_label.setStyleSheet("color: #aaa; font-size: 10px; background: transparent;")
        self._arrow_label.setFixedWidth(12)
        header_layout.addWidget(self._arrow_label)

        # Title
        self._title_label = QLabel(self._title)
        self._title_label.setStyleSheet("""
            color: #e0e0e0;
            font-weight: bold;
            font-size: 11px;
            background: transparent;
        """)
        header_layout.addWidget(self._title_label)
        header_layout.addStretch()

        layout.addWidget(self._header)

        # Content area
        self._content = QWidget()
        self._content_layout = QVBoxLayout(self._content)
        self._content_layout.setContentsMargins(0, 6, 0, 6)
        self._content_layout.setSpacing(4)
        layout.addWidget(self._content)

        # Set initial state
        self._content.setVisible(self._expanded)

    def _on_header_click(self, event):
        """Handle header click to toggle expansion."""
        self.toggle()

    def _update_arrow(self):
        """Update the arrow indicator based on expansion state."""
        self._arrow_label.setText("▼" if self._expanded else "▶")

    def toggle(self):
        """Toggle the expanded/collapsed state."""
        self._expanded = not self._expanded
        self._content.setVisible(self._expanded)
        self._update_arrow()
        self.toggled.emit(self._expanded)

    def expand(self):
        """Expand the content area."""
        if not self._expanded:
            self.toggle()

    def collapse(self):
        """Collapse the content area."""
        if self._expanded:
            self.toggle()

    def is_expanded(self) -> bool:
        """Return whether the content is expanded."""
        return self._expanded

    def set_title(self, title: str):
        """Set the group title."""
        self._title = title
        self._title_label.setText(title)

    def add_widget(self, widget: QWidget):
        """Add a widget to the content area."""
        self._content_layout.addWidget(widget)

    def add_layout(self, layout):
        """Add a layout to the content area."""
        self._content_layout.addLayout(layout)

    def content_layout(self):
        """Return the content layout for direct manipulation."""
        return self._content_layout
