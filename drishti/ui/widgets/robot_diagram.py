"""
Physically-correct robot diagram widget for CRL-200S vacuum robot.
All dimensions based on actual robot measurements.
"""

from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, QRectF, QPointF, QTimer
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPainterPath, QRadialGradient

import math


class RobotDiagram(QWidget):
    """
    Full-screen widget showing top-down view of robot with sensor overlays.

    Coordinate system:
    - Origin at robot center
    - +X points right
    - +Y points forward (up on screen)
    - All dimensions in mm, scaled to pixels
    """

    # Robot physical dimensions (mm) - CRL-200S / Xiaomi style
    ROBOT_DIAMETER = 350.0
    ROBOT_RADIUS = ROBOT_DIAMETER / 2

    # Bumper - thin front arc (like the real robot)
    BUMPER_ARC_ANGLE = 220  # degrees, centered on front
    BUMPER_WIDTH = 6  # mm thickness (thin like real robot)

    # Lidar turret - at rear/top of robot
    LIDAR_RADIUS = 35.0
    LIDAR_Y_OFFSET = -100.0  # toward rear (negative Y = top of screen)

    # Drive wheels - positioned on sides
    WHEEL_WIDTH = 25.0
    WHEEL_LENGTH = 50.0
    WHEEL_X_OFFSET = 130.0  # distance from center
    WHEEL_Y_OFFSET = 0.0  # centered vertically

    # Main brush roller
    MAIN_BRUSH_WIDTH = 140.0
    MAIN_BRUSH_HEIGHT = 30.0
    MAIN_BRUSH_Y_OFFSET = -30.0  # toward rear

    # Side brush (3-arm) - front-left
    SIDE_BRUSH_RADIUS = 35.0
    SIDE_BRUSH_X_OFFSET = -100.0  # left side
    SIDE_BRUSH_Y_OFFSET = 80.0  # toward front (positive Y = bottom of screen)

    # Cliff sensors - all in front half of robot (positive Y = front/bottom of screen)
    CLIFF_SENSOR_RADIUS = 8.0
    CLIFF_POSITIONS = {
        'left_front': (-70.0, 140.0),    # front-left corner
        'right_front': (70.0, 140.0),     # front-right corner
        'left_side': (-130.0, 60.0),      # mid-left side
        'right_side': (130.0, 60.0),      # mid-right side
    }

    # Buttons - vertical pill-shaped panel at front of robot (like real robot)
    BUTTON_PANEL_WIDTH = 20.0
    BUTTON_PANEL_HEIGHT = 50.0
    BUTTON_Y_OFFSET = 100.0  # toward front (positive Y = bottom of screen)

    # Caster wheel - at front
    CASTER_RADIUS = 12.0
    CASTER_Y_OFFSET = 110.0  # front (positive Y = bottom of screen)

    # Dustbox - at rear
    DUSTBOX_WIDTH = 120.0
    DUSTBOX_HEIGHT = 60.0
    DUSTBOX_Y_OFFSET = -80.0  # rear (negative Y = top of screen)

    # Colors
    COLOR_BODY = QColor(240, 240, 240)
    COLOR_BODY_BORDER = QColor(180, 180, 180)
    COLOR_BUMPER_OK = QColor(100, 100, 100)
    COLOR_BUMPER_TRIGGERED = QColor(255, 60, 60)
    COLOR_LIDAR = QColor(60, 60, 60)
    COLOR_WHEEL = QColor(50, 50, 50)
    COLOR_WHEEL_ACTIVE = QColor(80, 200, 80)
    COLOR_BRUSH_MAIN = QColor(255, 120, 60)
    COLOR_BRUSH_SIDE = QColor(200, 200, 200)
    COLOR_CLIFF_OK = QColor(60, 200, 60)
    COLOR_CLIFF_ALERT = QColor(255, 60, 60)
    COLOR_BUTTON_OFF = QColor(80, 80, 80)
    COLOR_BUTTON_ON = QColor(60, 200, 60)
    COLOR_CHARGING = QColor(255, 200, 60)
    COLOR_DUSTBOX_OK = QColor(180, 180, 180)
    COLOR_DUSTBOX_MISSING = QColor(255, 100, 100, 100)
    COLOR_TEXT = QColor(40, 40, 40)
    COLOR_BACKGROUND = QColor(245, 245, 245)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)

        # Sensor state
        self.sensor_data = {
            'is_charging': False,
            'is_battery_connected': True,
            'battery_level': 0,
            'battery_voltage': 0.0,
            'wheel_left': 0,
            'wheel_right': 0,
            'bumper_left': False,
            'bumper_right': False,
            'cliff_left_side': False,
            'cliff_left_front': False,
            'cliff_right_front': False,
            'cliff_right_side': False,
            'dustbox_attached': True,
            'start_button': 0,
            'dock_button': 0,
        }

        # Actuator speeds (0-100%)
        self.actuator_speeds = {
            'vacuum': 0.0,
            'main_brush': 0.0,
            'side_brush': 0.0,
        }

        # Animation state
        self.wheel_left_angle = 0.0
        self.wheel_right_angle = 0.0
        self.side_brush_angle = 0.0
        self.main_brush_angle = 0.0
        self.last_wheel_left = 0
        self.last_wheel_right = 0
        self.charging_animation_phase = 0.0  # 0.0 to 1.0 for charging animation
        self.charging_debounce_count = 0  # Debounce charging state changes

        # Animation timer
        self.animation_timer = QTimer(self)
        self.animation_timer.timeout.connect(self._update_animation)
        self.animation_timer.start(33)  # ~30 FPS

    def update_sensors(self, data: dict):
        """Update sensor data from telemetry."""
        # Debounce charging state at sensor update rate (500Hz)
        if 'is_charging' in data:
            raw_charging = data.get('is_charging', False)
            if raw_charging:
                self.charging_debounce_count = min(self.charging_debounce_count + 1, 100)
            else:
                self.charging_debounce_count = max(self.charging_debounce_count - 1, 0)

        self.sensor_data.update(data)
        self.update()

    def update_actuator(self, actuator_id: str, speed: float):
        """Update actuator speed for animation.

        Args:
            actuator_id: 'vacuum', 'brush', or 'side_brush'
            speed: Speed in percent (0-100)
        """
        if actuator_id == 'brush':
            self.actuator_speeds['main_brush'] = speed
        elif actuator_id == 'side_brush':
            self.actuator_speeds['side_brush'] = speed
        elif actuator_id == 'vacuum':
            self.actuator_speeds['vacuum'] = speed

    def _update_animation(self):
        """Update animation angles based on sensor changes."""
        # Wheel animation based on encoder changes
        wheel_left = self.sensor_data.get('wheel_left', 0)
        wheel_right = self.sensor_data.get('wheel_right', 0)

        if wheel_left != self.last_wheel_left:
            delta = wheel_left - self.last_wheel_left
            self.wheel_left_angle += delta * 0.1
            self.last_wheel_left = wheel_left

        if wheel_right != self.last_wheel_right:
            delta = wheel_right - self.last_wheel_right
            self.wheel_right_angle += delta * 0.1
            self.last_wheel_right = wheel_right

        # Brush rotation based on actuator speeds
        # Scale rotation speed by actuator percentage (0-100%)
        side_brush_speed = self.actuator_speeds.get('side_brush', 0.0)
        main_brush_speed = self.actuator_speeds.get('main_brush', 0.0)

        if side_brush_speed > 0:
            self.side_brush_angle += 15.0 * (side_brush_speed / 100.0)
        if main_brush_speed > 0:
            self.main_brush_angle += 9.0 * (main_brush_speed / 100.0)

        # Charging animation (oscillates 0 to 1 and back)
        # Consider charging if debounce count is above threshold (needs ~50ms of consistent charging)
        is_charging_stable = self.charging_debounce_count >= 25

        if is_charging_stable:
            self.charging_animation_phase += 0.03
            if self.charging_animation_phase > 2.0:
                self.charging_animation_phase = 0.0
        else:
            self.charging_animation_phase = 0.0

        self.update()

    def paintEvent(self, event):
        """Render the robot diagram."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Calculate scale to fit widget
        widget_size = min(self.width(), self.height()) - 40
        scale = widget_size / self.ROBOT_DIAMETER

        # Center transform
        # +Y points down (forward/front of robot at bottom of screen)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.scale(scale, scale)

        # Draw components back to front (in robot coordinates)
        self._draw_body(painter)
        self._draw_main_brush(painter)
        self._draw_wheels(painter)
        self._draw_side_brush(painter)
        self._draw_cliff_sensors(painter)
        self._draw_bumper(painter)
        self._draw_lidar(painter)
        self._draw_buttons(painter)

        # Draw overlays in screen coordinates
        self._draw_labels(painter, scale)
        self._draw_battery_indicator(painter)

    def _draw_body(self, painter):
        """Draw robot circular body with clean white design."""
        # Main body - white with subtle gradient
        gradient = QRadialGradient(0, -50, self.ROBOT_RADIUS * 1.2)
        gradient.setColorAt(0, QColor(255, 255, 255))
        gradient.setColorAt(1, QColor(245, 245, 245))

        painter.setPen(QPen(self.COLOR_BODY_BORDER, 1.5))
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(QPointF(0, 0), self.ROBOT_RADIUS, self.ROBOT_RADIUS)

    def _draw_bumper(self, painter):
        """Draw front bumper arc with left/right sections using simple arc stroke."""
        bumper_left = self.sensor_data.get('bumper_left', False)
        bumper_right = self.sensor_data.get('bumper_right', False)

        # Bumper arc parameters
        radius = self.ROBOT_RADIUS - self.BUMPER_WIDTH / 2  # Center of bumper thickness
        half_angle = self.BUMPER_ARC_ANGLE / 2

        # Create rect for arc drawing
        rect = QRectF(-radius, -radius, radius * 2, radius * 2)

        # Draw as thick stroked arcs (much simpler and cleaner)
        painter.setBrush(Qt.NoBrush)

        # Right half: from (270 - half_angle) to 270, draw as arc
        color_right = self.COLOR_BUMPER_TRIGGERED if bumper_right else self.COLOR_BUMPER_OK
        pen_right = QPen(color_right, self.BUMPER_WIDTH)
        pen_right.setCapStyle(Qt.FlatCap)
        painter.setPen(pen_right)
        # Qt drawArc uses 1/16th of a degree, and positive angles go counter-clockwise
        start_angle_right = int((270 - half_angle) * 16)
        span_angle_right = int(half_angle * 16)
        painter.drawArc(rect, start_angle_right, span_angle_right)

        # Left half: from 270 to (270 + half_angle)
        color_left = self.COLOR_BUMPER_TRIGGERED if bumper_left else self.COLOR_BUMPER_OK
        pen_left = QPen(color_left, self.BUMPER_WIDTH)
        pen_left.setCapStyle(Qt.FlatCap)
        painter.setPen(pen_left)
        start_angle_left = int(270 * 16)
        span_angle_left = int(half_angle * 16)
        painter.drawArc(rect, start_angle_left, span_angle_left)

    def _draw_lidar(self, painter):
        """Draw lidar turret."""
        # Outer ring
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.setBrush(QBrush(self.COLOR_LIDAR))
        painter.drawEllipse(QPointF(0, self.LIDAR_Y_OFFSET),
                           self.LIDAR_RADIUS, self.LIDAR_RADIUS)

        # Inner circle (slightly lighter)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(75, 75, 75)))
        painter.drawEllipse(QPointF(0, self.LIDAR_Y_OFFSET),
                           self.LIDAR_RADIUS * 0.6, self.LIDAR_RADIUS * 0.6)

    def _draw_wheels(self, painter):
        """Draw drive wheels with animation."""
        for side in ['left', 'right']:
            x = -self.WHEEL_X_OFFSET if side == 'left' else self.WHEEL_X_OFFSET
            y = self.WHEEL_Y_OFFSET

            # Check if wheel is moving
            encoder_key = f'wheel_{side}'
            is_moving = abs(self.sensor_data.get(encoder_key, 0) -
                          (self.last_wheel_left if side == 'left' else self.last_wheel_right)) > 0

            color = self.COLOR_WHEEL_ACTIVE if is_moving else self.COLOR_WHEEL

            painter.save()
            painter.translate(x, y)

            # Wheel body
            painter.setPen(QPen(QColor(30, 30, 30), 1))
            painter.setBrush(QBrush(color))
            rect = QRectF(-self.WHEEL_WIDTH/2, -self.WHEEL_LENGTH/2,
                         self.WHEEL_WIDTH, self.WHEEL_LENGTH)
            painter.drawRoundedRect(rect, 3, 3)

            # Tread lines
            angle = self.wheel_left_angle if side == 'left' else self.wheel_right_angle
            painter.setPen(QPen(QColor(20, 20, 20), 1))
            for i in range(5):
                offset = ((angle + i * 10) % 50) - 25
                painter.drawLine(QPointF(-self.WHEEL_WIDTH/2 + 2, offset),
                               QPointF(self.WHEEL_WIDTH/2 - 2, offset))

            painter.restore()

    def _draw_main_brush(self, painter):
        """Draw main brush roller."""
        painter.save()
        painter.translate(0, self.MAIN_BRUSH_Y_OFFSET)

        # Brush housing
        painter.setPen(QPen(QColor(80, 80, 80), 1))
        painter.setBrush(QBrush(self.COLOR_BRUSH_MAIN))
        rect = QRectF(-self.MAIN_BRUSH_WIDTH/2, -self.MAIN_BRUSH_HEIGHT/2,
                     self.MAIN_BRUSH_WIDTH, self.MAIN_BRUSH_HEIGHT)
        painter.drawRoundedRect(rect, 5, 5)

        # Brush pattern (rotating lines)
        painter.setPen(QPen(QColor(200, 80, 40), 2))
        for i in range(7):
            x = -self.MAIN_BRUSH_WIDTH/2 + 10 + i * 20
            offset = math.sin(math.radians(self.main_brush_angle + i * 30)) * 3
            painter.drawLine(QPointF(x, -self.MAIN_BRUSH_HEIGHT/2 + 5 + offset),
                           QPointF(x, self.MAIN_BRUSH_HEIGHT/2 - 5 + offset))

        painter.restore()

    def _draw_side_brush(self, painter):
        """Draw 3-arm side brush."""
        painter.save()
        painter.translate(self.SIDE_BRUSH_X_OFFSET, self.SIDE_BRUSH_Y_OFFSET)
        painter.rotate(self.side_brush_angle)

        # Center hub
        painter.setPen(QPen(QColor(60, 60, 60), 1))
        painter.setBrush(QBrush(QColor(80, 80, 80)))
        painter.drawEllipse(QPointF(0, 0), 8, 8)

        # 3 arms
        painter.setPen(QPen(self.COLOR_BRUSH_SIDE, 3))
        for i in range(3):
            angle = math.radians(i * 120)
            painter.drawLine(QPointF(0, 0),
                           QPointF(self.SIDE_BRUSH_RADIUS * math.cos(angle),
                                  self.SIDE_BRUSH_RADIUS * math.sin(angle)))

        painter.restore()

    def _draw_caster(self, painter):
        """Draw front caster wheel."""
        painter.setPen(QPen(QColor(60, 60, 60), 1))
        painter.setBrush(QBrush(QColor(100, 100, 100)))
        painter.drawEllipse(QPointF(0, self.CASTER_Y_OFFSET),
                           self.CASTER_RADIUS, self.CASTER_RADIUS)

    def _draw_cliff_sensors(self, painter):
        """Draw 4 cliff sensors with status."""
        sensor_map = {
            'left_front': 'cliff_left_front',
            'right_front': 'cliff_right_front',
            'left_side': 'cliff_left_side',
            'right_side': 'cliff_right_side',
        }

        for pos_name, (x, y) in self.CLIFF_POSITIONS.items():
            sensor_key = sensor_map[pos_name]
            triggered = self.sensor_data.get(sensor_key, False)

            color = self.COLOR_CLIFF_ALERT if triggered else self.COLOR_CLIFF_OK

            painter.setPen(QPen(QColor(40, 40, 40), 1))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(QPointF(x, y),
                               self.CLIFF_SENSOR_RADIUS, self.CLIFF_SENSOR_RADIUS)

    def _draw_dustbox(self, painter):
        """Draw dustbox compartment."""
        attached = self.sensor_data.get('dustbox_attached', True)

        color = self.COLOR_DUSTBOX_OK if attached else self.COLOR_DUSTBOX_MISSING

        painter.setPen(QPen(QColor(120, 120, 120), 1))
        painter.setBrush(QBrush(color))
        rect = QRectF(-self.DUSTBOX_WIDTH/2, self.DUSTBOX_Y_OFFSET - self.DUSTBOX_HEIGHT/2,
                     self.DUSTBOX_WIDTH, self.DUSTBOX_HEIGHT)
        painter.drawRoundedRect(rect, 5, 5)

        if not attached:
            # Draw X mark
            painter.setPen(QPen(QColor(200, 60, 60), 2))
            painter.drawLine(QPointF(-20, self.DUSTBOX_Y_OFFSET - 15),
                           QPointF(20, self.DUSTBOX_Y_OFFSET + 15))
            painter.drawLine(QPointF(-20, self.DUSTBOX_Y_OFFSET + 15),
                           QPointF(20, self.DUSTBOX_Y_OFFSET - 15))

    def _draw_buttons(self, painter):
        """Draw start and dock buttons in vertical pill-shaped panel (like real robot)."""
        start_value = self.sensor_data.get('start_button', 0)
        dock_value = self.sensor_data.get('dock_button', 0)

        x = 0
        y = self.BUTTON_Y_OFFSET
        w = self.BUTTON_PANEL_WIDTH
        h = self.BUTTON_PANEL_HEIGHT

        start_pressed = start_value > 100
        dock_pressed = dock_value > 100

        # Draw pill-shaped panel background (vertical orientation like real robot)
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        painter.setBrush(QBrush(QColor(250, 250, 250)))
        panel_rect = QRectF(x - w/2, y - h/2, w, h)
        painter.drawRoundedRect(panel_rect, w/2, w/2)

        # Dock button (top - home/charging icon)
        dock_y = y - h/4 - 2

        # Draw glow effect when dock button is pressed
        if dock_pressed:
            glow = QRadialGradient(x, dock_y, 12)
            glow.setColorAt(0, QColor(60, 200, 60, 200))
            glow.setColorAt(0.5, QColor(60, 200, 60, 100))
            glow.setColorAt(1, QColor(60, 200, 60, 0))
            painter.setBrush(QBrush(glow))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(x, dock_y), 12, 12)

        color = self.COLOR_BUTTON_ON if dock_pressed else self.COLOR_BUTTON_OFF
        line_width = 2.0 if dock_pressed else 1.5

        # Draw home/dock icon (house shape with base)
        painter.setPen(QPen(color, line_width, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.setBrush(Qt.NoBrush)
        # Roof
        painter.drawLine(QPointF(x - 5, dock_y), QPointF(x, dock_y - 5))
        painter.drawLine(QPointF(x, dock_y - 5), QPointF(x + 5, dock_y))
        # Walls
        painter.drawLine(QPointF(x - 4, dock_y), QPointF(x - 4, dock_y + 4))
        painter.drawLine(QPointF(x + 4, dock_y), QPointF(x + 4, dock_y + 4))
        # Base
        painter.drawLine(QPointF(x - 4, dock_y + 4), QPointF(x + 4, dock_y + 4))

        # Start button (bottom - power symbol)
        start_y = y + h/4 + 2

        # Draw glow effect when start button is pressed
        if start_pressed:
            glow = QRadialGradient(x, start_y, 12)
            glow.setColorAt(0, QColor(60, 200, 60, 200))
            glow.setColorAt(0.5, QColor(60, 200, 60, 100))
            glow.setColorAt(1, QColor(60, 200, 60, 0))
            painter.setBrush(QBrush(glow))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(x, start_y), 12, 12)

        color = self.COLOR_BUTTON_ON if start_pressed else self.COLOR_BUTTON_OFF
        line_width = 2.0 if start_pressed else 1.5

        # Draw power icon (circle with vertical line at top)
        painter.setPen(QPen(color, line_width, Qt.SolidLine, Qt.RoundCap))
        painter.setBrush(Qt.NoBrush)
        # Arc (open at top)
        painter.drawArc(QRectF(x - 5, start_y - 5, 10, 10), 50 * 16, 260 * 16)
        # Vertical line
        painter.drawLine(QPointF(x, start_y - 6), QPointF(x, start_y - 1))

    def _draw_battery_indicator(self, painter):
        """Draw battery/charging indicator in top-right corner of widget."""
        # Reset transform to screen coordinates
        painter.resetTransform()

        # Use debounced charging state for stable display
        is_charging = self.charging_debounce_count >= 25
        battery_level = self.sensor_data.get('battery_level', 0)
        battery_voltage = self.sensor_data.get('battery_voltage', 0.0)

        # Position at top-right corner of widget
        x = self.width() - 60
        y = 25
        w, h = 36, 18

        # Battery outline
        painter.setPen(QPen(QColor(100, 100, 100), 1.5))
        painter.setBrush(QBrush(QColor(240, 240, 240)))
        painter.drawRoundedRect(QRectF(x - w/2, y - h/2, w, h), 2, 2)

        # Battery terminal (positive end)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(100, 100, 100)))
        painter.drawRect(QRectF(x + w/2, y - 4, 3, 8))

        # Battery fill based on level
        fill_x = x - w/2 + 2
        max_fill_width = w - 4

        if is_charging:
            # Animated fill: oscillates from current level to 100%
            # Phase 0-1: expand from battery_level to 100%
            # Phase 1-2: contract from 100% back to battery_level
            if self.charging_animation_phase <= 1.0:
                anim_level = battery_level + (100 - battery_level) * self.charging_animation_phase
            else:
                anim_level = 100 - (100 - battery_level) * (self.charging_animation_phase - 1.0)
            fill_width = max_fill_width * (anim_level / 100.0)
            fill_color = self.COLOR_CHARGING  # Orange/yellow for charging
        else:
            fill_width = max_fill_width * (battery_level / 100.0)
            # Color based on level
            if battery_level > 50:
                fill_color = QColor(76, 175, 80)  # Green
            elif battery_level > 20:
                fill_color = QColor(255, 193, 7)  # Yellow/Amber
            else:
                fill_color = QColor(244, 67, 54)  # Red

        painter.setBrush(QBrush(fill_color))
        painter.drawRoundedRect(QRectF(fill_x, y - h/2 + 2, fill_width, h - 4), 1, 1)

        # Charging bolt overlay (on top of fill)
        if is_charging:
            painter.setPen(QPen(QColor(80, 80, 80), 1))
            painter.setBrush(QBrush(QColor(255, 220, 0)))  # Yellow bolt
            # Filled lightning bolt centered in battery
            bolt = QPainterPath()
            bolt.moveTo(x + 2, y - 5)
            bolt.lineTo(x - 1, y - 1)
            bolt.lineTo(x + 1, y - 1)
            bolt.lineTo(x - 2, y + 5)
            bolt.lineTo(x + 1, y + 1)
            bolt.lineTo(x - 1, y + 1)
            bolt.closeSubpath()
            painter.drawPath(bolt)

        # Battery percentage and voltage text below
        painter.setPen(QColor(80, 80, 80))
        font = QFont('Arial', 8)
        painter.setFont(font)
        text = f"{battery_level}% ({battery_voltage:.1f}V)"
        painter.drawText(QRectF(x - 40, y + h/2 + 2, 80, 15),
                        Qt.AlignCenter, text)

    def _draw_labels(self, painter, scale):
        """Draw text labels for sensor values."""
        # Reset transform for text (don't flip Y)
        painter.resetTransform()
        painter.translate(self.width() / 2, self.height() / 2)

        font = QFont('Arial', 8)
        painter.setFont(font)
        painter.setPen(self.COLOR_TEXT)

        # Wheel encoder values
        left_ticks = self.sensor_data.get('wheel_left', 0)
        right_ticks = self.sensor_data.get('wheel_right', 0)

        # Scale positions back to pixel space
        # Wheels are at Y=0 in robot coords, which is center in screen coords
        # Place labels below wheels (positive Y in screen = down)
        left_x = -self.WHEEL_X_OFFSET * scale
        right_x = self.WHEEL_X_OFFSET * scale
        wheel_y = (self.WHEEL_LENGTH / 2 + 10) * scale  # Below wheel bottom

        painter.drawText(QRectF(left_x - 30, wheel_y, 60, 20),
                        Qt.AlignCenter, f"L:{left_ticks}")
        painter.drawText(QRectF(right_x - 30, wheel_y, 60, 20),
                        Qt.AlignCenter, f"R:{right_ticks}")

        # Status text at bottom
        status_parts = []
        if not self.sensor_data.get('dustbox_attached', True):
            status_parts.append("NO DUSTBOX")
        if self.sensor_data.get('bumper_left', False) or self.sensor_data.get('bumper_right', False):
            status_parts.append("BUMPER")

        any_cliff = any([
            self.sensor_data.get('cliff_left_front', False),
            self.sensor_data.get('cliff_right_front', False),
            self.sensor_data.get('cliff_left_side', False),
            self.sensor_data.get('cliff_right_side', False),
        ])
        if any_cliff:
            status_parts.append("CLIFF!")

        if status_parts:
            status_text = " | ".join(status_parts)
            painter.setPen(QColor(200, 60, 60))
            painter.drawText(QRectF(-100, self.height()/2 - 30, 200, 20),
                           Qt.AlignCenter, status_text)
