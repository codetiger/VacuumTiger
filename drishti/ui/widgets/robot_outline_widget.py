"""
Robot outline widget showing top-down view with all sensors and components.

Based on iLife A11/Xiaomi-style circular vacuum robot design.
"""

from PyQt5.QtCore import Qt, QRectF, QPointF, QTimer, QLineF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPainterPath, QLinearGradient, QRadialGradient
from PyQt5.QtWidgets import QWidget
import math


class RobotOutlineWidget(QWidget):
    """Top-down robot visualization with real-time sensor updates"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)

        # Robot dimensions (scale: 320mm diameter → 300px)
        self.robot_diameter = 300
        self.robot_radius = self.robot_diameter / 2

        # Sensor data (initialized to safe/default values)
        self.battery_level = 100
        self.is_charging = False
        self.encoder_left = 0
        self.encoder_right = 0
        self.prev_encoder_left = 0
        self.prev_encoder_right = 0
        self.cliff_sensors = [100, 100, 100, 100]  # Safe values
        self.bumper_pressed = False
        self.front_ir = 200  # Safe distance
        self.start_button_ir = 0
        self.dock_button_ir = 0
        self.side_brush_speed = 0
        self.main_brush_speed = 0
        self.lidar_angle = 0.0  # Last scan angle in radians

        # Animation state
        self.wheel_animation_offset = 0
        self.side_brush_angle = 0
        self.main_brush_offset = 0

        # Animation timer (30 FPS)
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._update_animation)
        self.animation_timer.start(33)  # ~30 FPS

    def update_sensor_data(self, sensor_data):
        """
        Update from SensorUpdate message (list format from MessagePack)

        Args:
            sensor_data: List [timestamp, battery_level, is_charging, ir_sensor_1, ...]
        """
        if not sensor_data or len(sensor_data) < 11:
            return

        self.battery_level = sensor_data[1] if sensor_data[1] is not None else 0
        self.is_charging = sensor_data[2] if sensor_data[2] is not None else False
        self.front_ir = sensor_data[3] if sensor_data[3] is not None else 200
        self.start_button_ir = sensor_data[4] if sensor_data[4] is not None else 0
        self.dock_button_ir = sensor_data[5] if sensor_data[5] is not None else 0
        self.bumper_pressed = sensor_data[6] if sensor_data[6] is not None else False
        cliff_sensors_data = sensor_data[7]
        if cliff_sensors_data and len(cliff_sensors_data) == 4:
            self.cliff_sensors = cliff_sensors_data

        # Encoders
        self.prev_encoder_left = self.encoder_left
        self.prev_encoder_right = self.encoder_right
        self.encoder_left = sensor_data[9] if sensor_data[9] is not None else 0
        self.encoder_right = sensor_data[10] if sensor_data[10] is not None else 0

        self.update()

    def update_lidar_angle(self, angle):
        """Update lidar scan direction"""
        self.lidar_angle = angle
        self.update()

    def set_brush_speeds(self, side_speed, main_speed):
        """Set brush speeds for animation"""
        self.side_brush_speed = side_speed
        self.main_brush_speed = main_speed

    def _update_animation(self):
        """Update animation state (called by timer)"""
        # Animate wheels if encoders are changing
        if self.encoder_left != self.prev_encoder_left:
            self.wheel_animation_offset += 5
        if self.encoder_right != self.prev_encoder_right:
            self.wheel_animation_offset += 5

        # Animate side brush
        if self.side_brush_speed > 0:
            self.side_brush_angle += self.side_brush_speed * 0.1

        # Animate main brush
        if self.main_brush_speed > 0:
            self.main_brush_offset += self.main_brush_speed * 0.05

        self.wheel_animation_offset %= 20  # Loop animation
        self.side_brush_angle %= 360
        self.main_brush_offset %= 30

        self.update()

    def paintEvent(self, event):
        """Draw robot outline and all components"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Fill background
        painter.fillRect(self.rect(), QColor(43, 43, 43))

        # Center point
        center_x = self.width() / 2
        center_y = self.height() / 2
        center = QPointF(center_x, center_y)

        # Draw robot body
        self._draw_robot_body(painter, center)

        # Draw components (order matters for layering)
        self._draw_cliff_sensors(painter, center)
        self._draw_wheels(painter, center)
        self._draw_main_brush(painter, center)
        self._draw_side_brush(painter, center)
        self._draw_front_bumper(painter, center)
        self._draw_front_ir(painter, center)
        self._draw_lidar_turret(painter, center)
        self._draw_button_panel(painter, center)
        self._draw_battery(painter, center)
        self._draw_encoder_values(painter, center)

    def _draw_robot_body(self, painter, center):
        """Draw main robot body circle"""
        body_rect = QRectF(center.x() - self.robot_radius, center.y() - self.robot_radius,
                           self.robot_diameter, self.robot_diameter)

        # Gradient for depth
        gradient = QRadialGradient(center, self.robot_radius)
        gradient.setColorAt(0, QColor(240, 240, 240, 80))
        gradient.setColorAt(1, QColor(200, 200, 200, 80))

        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(QColor(150, 150, 150), 2))
        painter.drawEllipse(body_rect)

    def _draw_lidar_turret(self, painter, center):
        """Draw lidar turret with rotating indicator"""
        turret_center = QPointF(center.x(), center.y() - 80)
        turret_radius = 30

        # Turret body
        painter.setBrush(QBrush(QColor(100, 100, 100)))
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.drawEllipse(turret_center, turret_radius, turret_radius)

        # Rotating scan indicator line
        line_length = 25
        end_x = turret_center.x() + line_length * math.cos(self.lidar_angle)
        end_y = turret_center.y() + line_length * math.sin(self.lidar_angle)
        painter.setPen(QPen(QColor(0, 255, 255), 3))  # Cyan
        painter.drawLine(turret_center, QPointF(end_x, end_y))

        # Label
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 8))
        painter.drawText(QRectF(turret_center.x() - 30, turret_center.y() + 35, 60, 15),
                         Qt.AlignCenter, "LIDAR")

    def _draw_button_panel(self, painter, center):
        """Draw start/dock button panel with LED indicators"""
        panel_rect = QRectF(center.x() - 30, center.y() - 20, 60, 40)

        # Panel background
        painter.setBrush(QBrush(QColor(80, 80, 80)))
        painter.setPen(QPen(QColor(60, 60, 60), 1))
        painter.drawRoundedRect(panel_rect, 5, 5)

        # Start button LED (left) - Green when pressed
        start_led_center = QPointF(panel_rect.left() + 15, panel_rect.center().y())
        start_color = QColor(0, 255, 0) if self.start_button_ir > 100 else QColor(40, 40, 40)
        painter.setBrush(QBrush(start_color))
        painter.drawEllipse(start_led_center, 5, 5)

        # Dock button LED (right) - Yellow when pressed
        dock_led_center = QPointF(panel_rect.right() - 15, panel_rect.center().y())
        dock_color = QColor(255, 255, 0) if self.dock_button_ir > 100 else QColor(40, 40, 40)
        painter.setBrush(QBrush(dock_color))
        painter.drawEllipse(dock_led_center, 5, 5)

        # Labels
        painter.setPen(QPen(QColor(200, 200, 200)))
        painter.setFont(QFont("Arial", 7))
        painter.drawText(QRectF(start_led_center.x() - 10, start_led_center.y() + 8, 20, 10),
                         Qt.AlignCenter, "START")
        painter.drawText(QRectF(dock_led_center.x() - 10, dock_led_center.y() + 8, 20, 10),
                         Qt.AlignCenter, "DOCK")

    def _draw_wheels(self, painter, center):
        """Draw left and right drive wheels with animation"""
        wheel_width = 20
        wheel_height = 50

        # Left wheel
        left_wheel_rect = QRectF(center.x() - self.robot_radius - 10, center.y() - wheel_height / 2,
                                 wheel_width, wheel_height)
        self._draw_wheel(painter, left_wheel_rect, self.encoder_left != self.prev_encoder_left)

        # Right wheel
        right_wheel_rect = QRectF(center.x() + self.robot_radius - 10, center.y() - wheel_height / 2,
                                  wheel_width, wheel_height)
        self._draw_wheel(painter, right_wheel_rect, self.encoder_right != self.prev_encoder_right)

    def _draw_wheel(self, painter, rect, is_moving):
        """Draw individual wheel with tread pattern"""
        color = QColor(0, 200, 0) if is_moving else QColor(60, 60, 60)
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(QColor(40, 40, 40), 2))
        painter.drawRect(rect)

        # Tread lines (animated if moving)
        if is_moving:
            painter.setPen(QPen(QColor(200, 200, 200), 1))
            for i in range(5):
                y_offset = (i * 10 + self.wheel_animation_offset) % int(rect.height())
                painter.drawLine(int(rect.left()), int(rect.top() + y_offset),
                                 int(rect.right()), int(rect.top() + y_offset))

    def _draw_main_brush(self, painter, center):
        """Draw main rolling brush"""
        brush_rect = QRectF(center.x() - 60, center.y() + 40, 120, 15)

        color = QColor(255, 140, 0) if self.main_brush_speed > 0 else QColor(100, 100, 100)
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(QColor(80, 80, 80), 1))
        painter.drawRect(brush_rect)

        # Bristle pattern (animated if moving)
        if self.main_brush_speed > 0:
            painter.setPen(QPen(QColor(200, 100, 0), 1))
            for i in range(12):
                x_offset = (i * 10 + self.main_brush_offset) % int(brush_rect.width())
                painter.drawLine(int(brush_rect.left() + x_offset), int(brush_rect.top()),
                                 int(brush_rect.left() + x_offset), int(brush_rect.bottom()))

        # Label
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 7))
        painter.drawText(QRectF(brush_rect.left(), brush_rect.bottom() + 2, brush_rect.width(), 12),
                         Qt.AlignCenter, "MAIN BRUSH")

    def _draw_side_brush(self, painter, center):
        """Draw side brush with rotation animation"""
        brush_center = QPointF(center.x() + 100, center.y() + 80)
        brush_radius = 20

        color = QColor(255, 140, 0) if self.side_brush_speed > 0 else QColor(100, 100, 100)
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.drawEllipse(brush_center, brush_radius, brush_radius)

        # Rotating spokes (if active)
        if self.side_brush_speed > 0:
            painter.setPen(QPen(QColor(200, 100, 0), 3))
            for i in range(3):
                angle = math.radians(self.side_brush_angle + i * 120)
                end_x = brush_center.x() + brush_radius * math.cos(angle)
                end_y = brush_center.y() + brush_radius * math.sin(angle)
                painter.drawLine(brush_center, QPointF(end_x, end_y))

        # Label
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 7))
        painter.drawText(QRectF(brush_center.x() - 25, brush_center.y() + 25, 50, 12),
                         Qt.AlignCenter, "SIDE BRUSH")

    def _draw_cliff_sensors(self, painter, center):
        """Draw 4 cliff sensors with color-coded status"""
        positions = [
            (center.x() - 80, center.y() - 120, "FL"),  # Front-left
            (center.x() + 80, center.y() - 120, "FR"),  # Front-right
            (center.x() - 80, center.y() + 120, "RL"),  # Rear-left
            (center.x() + 80, center.y() + 120, "RR"),  # Rear-right
        ]

        for i, (x, y, label) in enumerate(positions):
            if i < len(self.cliff_sensors):
                value = self.cliff_sensors[i]
                # Color code: Green (safe), Yellow (caution), Red (cliff)
                if value > 100:
                    color = QColor(76, 175, 80)  # Green
                elif value > 50:
                    color = QColor(255, 235, 59)  # Yellow
                else:
                    color = QColor(244, 67, 54)  # Red

                painter.setBrush(QBrush(color))
                painter.setPen(QPen(QColor(0, 0, 0), 1))
                painter.drawEllipse(QPointF(x, y), 8, 8)

                # Label
                painter.setPen(QPen(QColor(255, 255, 255)))
                painter.setFont(QFont("Arial", 7))
                painter.drawText(QRectF(x - 15, y + 10, 30, 12), Qt.AlignCenter, label)

    def _draw_front_bumper(self, painter, center):
        """Draw front bumper arc (180°) with press indication"""
        # Create arc path for front semicircle
        arc_rect = QRectF(center.x() - self.robot_radius, center.y() - self.robot_radius,
                          self.robot_diameter, self.robot_diameter)

        path = QPainterPath()
        path.arcMoveTo(arc_rect, 225)  # Start at 225° (front-left)
        path.arcTo(arc_rect, 225, 90)  # Arc 90° to front (315° total)

        # Color: Red if pressed, Green if normal
        color = QColor(244, 67, 54, 150) if self.bumper_pressed else QColor(76, 175, 80, 100)
        painter.setPen(QPen(color, 15, Qt.SolidLine, Qt.RoundCap))
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(path)

        # Flashing effect if pressed
        if self.bumper_pressed:
            painter.setPen(QPen(QColor(255, 0, 0), 3))
            painter.drawPath(path)

    def _draw_front_ir(self, painter, center):
        """Draw front IR distance sensor with beam visualization"""
        sensor_pos = QPointF(center.x(), center.y() - 150)

        # Sensor indicator
        if self.front_ir > 150:
            color = QColor(76, 175, 80)  # Green (clear)
        elif self.front_ir > 50:
            color = QColor(255, 235, 59)  # Yellow (object detected)
        else:
            color = QColor(244, 67, 54)  # Red (obstacle close)

        painter.setBrush(QBrush(color))
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.drawEllipse(sensor_pos, 7, 7)

        # Beam visualization (cone)
        beam_path = QPainterPath()
        beam_path.moveTo(sensor_pos)
        beam_path.lineTo(sensor_pos.x() - 20, sensor_pos.y() - 40)
        beam_path.lineTo(sensor_pos.x() + 20, sensor_pos.y() - 40)
        beam_path.closeSubpath()

        painter.setBrush(QBrush(QColor(color.red(), color.green(), color.blue(), 50)))
        painter.setPen(Qt.NoPen)
        painter.drawPath(beam_path)

        # Label
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 7))
        painter.drawText(QRectF(sensor_pos.x() - 20, sensor_pos.y() + 10, 40, 12),
                         Qt.AlignCenter, "IR")

    def _draw_battery(self, painter, center):
        """Draw battery status in top-right"""
        battery_x = center.x() + 120
        battery_y = center.y() - 130

        # Battery body
        body_rect = QRectF(battery_x - 20, battery_y, 40, 20)
        terminal_rect = QRectF(battery_x + 20, battery_y + 7, 3, 6)

        # Color based on level
        if self.battery_level > 50:
            fill_color = QColor(76, 175, 80)
        elif self.battery_level > 20:
            fill_color = QColor(255, 193, 7)
        else:
            fill_color = QColor(244, 67, 54)

        painter.setPen(QPen(QColor(200, 200, 200), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(body_rect)
        painter.drawRect(terminal_rect)

        # Fill
        fill_width = (body_rect.width() - 4) * (self.battery_level / 100.0)
        fill_rect = QRectF(body_rect.x() + 2, body_rect.y() + 2, fill_width, body_rect.height() - 4)
        painter.setBrush(QBrush(fill_color))
        painter.setPen(Qt.NoPen)
        painter.drawRect(fill_rect)

        # Charging bolt
        if self.is_charging:
            bolt_path = QPainterPath()
            cx, cy = body_rect.center().x(), body_rect.center().y()
            bolt_path.moveTo(cx, cy - 6)
            bolt_path.lineTo(cx - 3, cy)
            bolt_path.lineTo(cx + 1, cy)
            bolt_path.lineTo(cx - 1, cy + 6)
            bolt_path.lineTo(cx + 4, cy + 1)
            bolt_path.lineTo(cx + 1, cy + 1)
            bolt_path.closeSubpath()
            painter.setBrush(QBrush(QColor(255, 235, 59)))
            painter.setPen(QPen(QColor(0, 0, 0), 1))
            painter.drawPath(bolt_path)

        # Percentage text
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        painter.drawText(QRectF(battery_x - 20, battery_y + 22, 40, 15),
                         Qt.AlignCenter, f"{self.battery_level}%")

    def _draw_encoder_values(self, painter, center):
        """Draw encoder tick values below wheels"""
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 9))

        # Left encoder
        left_text_rect = QRectF(center.x() - self.robot_radius - 40, center.y() + 40, 80, 20)
        painter.drawText(left_text_rect, Qt.AlignCenter, f"L: {self.encoder_left}")

        # Right encoder
        right_text_rect = QRectF(center.x() + self.robot_radius - 40, center.y() + 40, 80, 20)
        painter.drawText(right_text_rect, Qt.AlignCenter, f"R: {self.encoder_right}")
