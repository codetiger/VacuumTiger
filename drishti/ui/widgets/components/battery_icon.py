"""Battery icon renderer with charging indicator"""

from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPainterPath
from PyQt5.QtWidgets import QWidget


class BatteryIcon(QWidget):
    """Battery status display with percentage and charging indicator"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.battery_level = 100  # 0-100%
        self.is_charging = False
        self.setMinimumSize(80, 40)

    def set_battery_data(self, level, is_charging):
        """
        Update battery status

        Args:
            level: Battery percentage (0-100)
            is_charging: Boolean charging status
        """
        self.battery_level = level if level is not None else 0
        self.is_charging = is_charging if is_charging is not None else False
        self.update()  # Trigger repaint

    def paintEvent(self, event):
        """Draw battery icon"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Battery body rectangle
        body_rect = QRectF(10, 10, 50, 25)

        # Battery terminal (small rectangle on right)
        terminal_rect = QRectF(60, 17, 5, 11)

        # Determine color based on level
        if self.battery_level > 50:
            fill_color = QColor(76, 175, 80)  # Green
        elif self.battery_level > 20:
            fill_color = QColor(255, 193, 7)  # Yellow
        else:
            fill_color = QColor(244, 67, 54)  # Red

        # Draw battery outline
        painter.setPen(QPen(QColor(200, 200, 200), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(body_rect)
        painter.drawRect(terminal_rect)

        # Fill battery based on level
        fill_width = (body_rect.width() - 4) * (self.battery_level / 100.0)
        fill_rect = QRectF(body_rect.x() + 2, body_rect.y() + 2, fill_width, body_rect.height() - 4)
        painter.setBrush(QBrush(fill_color))
        painter.setPen(Qt.NoPen)
        painter.drawRect(fill_rect)

        # Draw charging bolt if charging
        if self.is_charging:
            self._draw_charging_bolt(painter, body_rect)

        # Draw percentage text
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        text_rect = QRectF(0, body_rect.bottom() + 2, 70, 15)
        painter.drawText(text_rect, Qt.AlignCenter, f"{self.battery_level}%")

    def _draw_charging_bolt(self, painter, battery_rect):
        """Draw charging bolt icon overlay"""
        # Lightning bolt path
        bolt = QPainterPath()
        center_x = battery_rect.center().x()
        center_y = battery_rect.center().y()

        # Bolt coordinates (scaled to fit)
        bolt.moveTo(center_x, center_y - 8)
        bolt.lineTo(center_x - 4, center_y)
        bolt.lineTo(center_x + 2, center_y)
        bolt.lineTo(center_x - 2, center_y + 8)
        bolt.lineTo(center_x + 6, center_y + 2)
        bolt.lineTo(center_x + 2, center_y + 2)
        bolt.closeSubpath()

        painter.setBrush(QBrush(QColor(255, 235, 59)))  # Yellow
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.drawPath(bolt)
