"""
Wireframe geometry generation for 3D robot visualization.

All functions return Nx3 numpy arrays of line segment endpoints
for use with pyqtgraph.opengl.GLLinePlotItem in 'lines' mode.
"""

import numpy as np
import math


def create_cylinder_wireframe(radius: float, height: float, segments: int = 48,
                              vertical_lines: int = 12, horizontal_rings: int = 1) -> np.ndarray:
    """
    Create wireframe edges for a cylinder with denser geometry.

    Args:
        radius: Cylinder radius
        height: Cylinder height
        segments: Number of segments around circumference
        vertical_lines: Number of vertical edge lines
        horizontal_rings: Number of intermediate horizontal rings (0 = top/bottom only)

    Returns:
        Nx3 array of line segment endpoints (pairs of vertices)
    """
    angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    points = []

    # Create all horizontal rings (including top and bottom)
    z_positions = np.linspace(-height / 2, height / 2, horizontal_rings + 2)

    for z in z_positions:
        for i in range(segments):
            x1 = radius * np.cos(angles[i])
            y1 = radius * np.sin(angles[i])
            x2 = radius * np.cos(angles[(i + 1) % segments])
            y2 = radius * np.sin(angles[(i + 1) % segments])
            points.extend([[x1, y1, z], [x2, y2, z]])

    # Vertical edges (evenly spaced)
    step = max(1, segments // vertical_lines)
    for i in range(0, segments, step):
        x = radius * np.cos(angles[i])
        y = radius * np.sin(angles[i])
        points.extend([[x, y, height / 2], [x, y, -height / 2]])

    return np.array(points, dtype=np.float32)


def create_arc_wireframe(radius: float, start_angle: float, end_angle: float,
                         thickness: float = 0.0, segments: int = 32,
                         height: float = 0.0, height_segments: int = 0) -> np.ndarray:
    """
    Create wireframe edges for an arc (portion of a circle) with optional 3D extrusion.

    Args:
        radius: Arc radius
        start_angle: Start angle in degrees
        end_angle: End angle in degrees
        thickness: Optional radial thickness (creates inner and outer arcs)
        segments: Number of segments along the arc
        height: Optional height for 3D arc (extrusion)
        height_segments: Number of vertical divisions for 3D arc

    Returns:
        Nx3 array of line segment endpoints
    """
    start_rad = math.radians(start_angle)
    end_rad = math.radians(end_angle)
    angles = np.linspace(start_rad, end_rad, segments)
    points = []

    # Determine Z positions
    if height > 0 and height_segments > 0:
        z_positions = np.linspace(-height / 2, height / 2, height_segments + 1)
    else:
        z_positions = [0]

    # Outer arc at each height
    for z in z_positions:
        for i in range(len(angles) - 1):
            x1 = radius * np.cos(angles[i])
            y1 = radius * np.sin(angles[i])
            x2 = radius * np.cos(angles[i + 1])
            y2 = radius * np.sin(angles[i + 1])
            points.extend([[x1, y1, z], [x2, y2, z]])

    # Inner arc (if thickness specified)
    if thickness > 0:
        inner_radius = radius - thickness
        for z in z_positions:
            for i in range(len(angles) - 1):
                x1 = inner_radius * np.cos(angles[i])
                y1 = inner_radius * np.sin(angles[i])
                x2 = inner_radius * np.cos(angles[i + 1])
                y2 = inner_radius * np.sin(angles[i + 1])
                points.extend([[x1, y1, z], [x2, y2, z]])

        # End caps (connect inner to outer) at each height
        for z in z_positions:
            for angle in [angles[0], angles[-1]]:
                x_outer = radius * np.cos(angle)
                y_outer = radius * np.sin(angle)
                x_inner = inner_radius * np.cos(angle)
                y_inner = inner_radius * np.sin(angle)
                points.extend([[x_outer, y_outer, z], [x_inner, y_inner, z]])

    # Vertical lines connecting heights (for 3D arcs)
    if height > 0 and len(z_positions) > 1:
        # Add vertical lines at regular intervals along outer arc
        vert_step = max(1, len(angles) // 8)
        for i in range(0, len(angles), vert_step):
            x = radius * np.cos(angles[i])
            y = radius * np.sin(angles[i])
            points.extend([[x, y, z_positions[0]], [x, y, z_positions[-1]]])

        # Add vertical lines along inner arc too
        if thickness > 0:
            for i in range(0, len(angles), vert_step):
                x = inner_radius * np.cos(angles[i])
                y = inner_radius * np.sin(angles[i])
                points.extend([[x, y, z_positions[0]], [x, y, z_positions[-1]]])

    return np.array(points, dtype=np.float32)


def create_box_wireframe(width: float, length: float, height: float) -> np.ndarray:
    """
    Create wireframe edges for a rectangular box.

    Args:
        width: Box width (X dimension)
        length: Box length (Y dimension)
        height: Box height (Z dimension)

    Returns:
        Nx3 array of line segment endpoints
    """
    w, l, h = width / 2, length / 2, height / 2
    points = []

    # Bottom rectangle
    corners_bottom = [(-w, -l, -h), (w, -l, -h), (w, l, -h), (-w, l, -h)]
    for i in range(4):
        points.extend([corners_bottom[i], corners_bottom[(i + 1) % 4]])

    # Top rectangle
    corners_top = [(-w, -l, h), (w, -l, h), (w, l, h), (-w, l, h)]
    for i in range(4):
        points.extend([corners_top[i], corners_top[(i + 1) % 4]])

    # Vertical edges
    for i in range(4):
        points.extend([corners_bottom[i], corners_top[i]])

    return np.array(points, dtype=np.float32)


def create_circle_wireframe(radius: float, segments: int = 32, z: float = 0) -> np.ndarray:
    """
    Create wireframe edges for a circle.

    Args:
        radius: Circle radius
        segments: Number of segments (default 32 for smooth appearance)
        z: Z position of the circle

    Returns:
        Nx3 array of line segment endpoints
    """
    angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    points = []

    for i in range(segments):
        x1 = radius * np.cos(angles[i])
        y1 = radius * np.sin(angles[i])
        x2 = radius * np.cos(angles[(i + 1) % segments])
        y2 = radius * np.sin(angles[(i + 1) % segments])
        points.extend([[x1, y1, z], [x2, y2, z]])

    return np.array(points, dtype=np.float32)


def create_3arm_brush_wireframe(radius: float, hub_radius: float = 8.0) -> np.ndarray:
    """
    Create wireframe for a 3-arm side brush with denser geometry.

    Args:
        radius: Length of brush arms
        hub_radius: Radius of center hub

    Returns:
        Nx3 array of line segment endpoints
    """
    points = []

    # Center hub (small circle) - denser
    hub_points = create_circle_wireframe(hub_radius, segments=24)
    points.extend(hub_points.tolist())

    # Inner hub ring for visual depth
    inner_hub = create_circle_wireframe(hub_radius * 0.5, segments=16)
    points.extend(inner_hub.tolist())

    # 3 main arms at 120 degree intervals with bristle details
    for i in range(3):
        angle = math.radians(i * 120)
        x_tip = radius * math.cos(angle)
        y_tip = radius * math.sin(angle)

        # Main arm spine
        points.extend([[0, 0, 0], [x_tip, y_tip, 0]])

        # Bristle lines along the arm (perpendicular to arm direction)
        num_bristles = 5
        for j in range(1, num_bristles + 1):
            t = j / (num_bristles + 1)  # Position along arm
            arm_x = t * x_tip
            arm_y = t * y_tip

            # Perpendicular direction
            perp_angle = angle + math.pi / 2
            bristle_len = radius * 0.15 * (1 - t * 0.5)  # Bristles get shorter toward tip
            bx = bristle_len * math.cos(perp_angle)
            by = bristle_len * math.sin(perp_angle)

            # Bristles on both sides
            points.extend([[arm_x - bx, arm_y - by, 0], [arm_x + bx, arm_y + by, 0]])

        # Arm tip arc (brush end)
        tip_arc_radius = radius * 0.1
        for k in range(4):
            a1 = angle - 0.5 + k * 0.33
            a2 = angle - 0.5 + (k + 1) * 0.33
            x1 = x_tip + tip_arc_radius * math.cos(a1)
            y1 = y_tip + tip_arc_radius * math.sin(a1)
            x2 = x_tip + tip_arc_radius * math.cos(a2)
            y2 = y_tip + tip_arc_radius * math.sin(a2)
            points.extend([[x1, y1, 0], [x2, y2, 0]])

    return np.array(points, dtype=np.float32)


def create_roller_brush_wireframe(width: float, radius: float, segments: int = 32,
                                  bristle_lines: int = 12) -> np.ndarray:
    """
    Create wireframe for a roller brush (main brush) with denser geometry.

    The roller is oriented along the X axis (rotates around X).

    Args:
        width: Brush width (length along X axis)
        radius: Brush radius
        segments: Number of segments around circumference
        bristle_lines: Number of axial bristle/spiral lines

    Returns:
        Nx3 array of line segment endpoints
    """
    points = []
    half_width = width / 2
    angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)

    # End circles (in Y-Z plane at X = ±half_width)
    for i in range(segments):
        y1 = radius * np.cos(angles[i])
        z1 = radius * np.sin(angles[i])
        y2 = radius * np.cos(angles[(i + 1) % segments])
        z2 = radius * np.sin(angles[(i + 1) % segments])
        # Left end cap
        points.extend([[-half_width, y1, z1], [-half_width, y2, z2]])
        # Right end cap
        points.extend([[half_width, y1, z1], [half_width, y2, z2]])

    # Middle ring for visual reference
    for i in range(segments):
        y1 = radius * np.cos(angles[i])
        z1 = radius * np.sin(angles[i])
        y2 = radius * np.cos(angles[(i + 1) % segments])
        z2 = radius * np.sin(angles[(i + 1) % segments])
        points.extend([[0, y1, z1], [0, y2, z2]])

    # Straight axial bristle lines
    bristle_angles = np.linspace(0, 2 * np.pi, bristle_lines, endpoint=False)
    for angle in bristle_angles:
        y = radius * np.cos(angle)
        z = radius * np.sin(angle)
        points.extend([[-half_width, y, z], [half_width, y, z]])

    # Spiral pattern lines for visual interest (makes rotation visible)
    num_spirals = 4
    spiral_segments = 16
    for s in range(num_spirals):
        base_angle = s * (2 * np.pi / num_spirals)
        for i in range(spiral_segments):
            t1 = i / spiral_segments
            t2 = (i + 1) / spiral_segments
            x1 = -half_width + t1 * width
            x2 = -half_width + t2 * width
            angle1 = base_angle + t1 * np.pi  # Half rotation along length
            angle2 = base_angle + t2 * np.pi
            y1 = radius * np.cos(angle1)
            z1 = radius * np.sin(angle1)
            y2 = radius * np.cos(angle2)
            z2 = radius * np.sin(angle2)
            points.extend([[x1, y1, z1], [x2, y2, z2]])

    # Inner hub circles (smaller radius) for visual depth
    hub_radius = radius * 0.3
    for i in range(segments // 2):  # Fewer segments for inner
        y1 = hub_radius * np.cos(angles[i * 2])
        z1 = hub_radius * np.sin(angles[i * 2])
        y2 = hub_radius * np.cos(angles[(i * 2 + 2) % segments])
        z2 = hub_radius * np.sin(angles[(i * 2 + 2) % segments])
        points.extend([[-half_width, y1, z1], [-half_width, y2, z2]])
        points.extend([[half_width, y1, z1], [half_width, y2, z2]])

    return np.array(points, dtype=np.float32)


def create_fan_wireframe(radius: float, blades: int = 8, hub_radius: float = 8.0) -> np.ndarray:
    """
    Create wireframe for a fan/impeller with denser geometry.

    Args:
        radius: Fan radius
        blades: Number of blades
        hub_radius: Radius of center hub

    Returns:
        Nx3 array of line segment endpoints
    """
    points = []

    # Center hub - denser circle
    hub_points = create_circle_wireframe(hub_radius, segments=24)
    points.extend(hub_points.tolist())

    # Inner hub for depth
    inner_hub = create_circle_wireframe(hub_radius * 0.5, segments=16)
    points.extend(inner_hub.tolist())

    # Outer rim circle
    outer_rim = create_circle_wireframe(radius * 0.95, segments=48)
    points.extend(outer_rim.tolist())

    # Blades (curved, more detailed)
    blade_angle_span = 0.4  # radians - width of blade at base
    for i in range(blades):
        base_angle = math.radians(i * (360 / blades))

        # Blade leading edge
        x_base_lead = hub_radius * math.cos(base_angle - blade_angle_span / 2)
        y_base_lead = hub_radius * math.sin(base_angle - blade_angle_span / 2)

        # Blade trailing edge
        x_base_trail = hub_radius * math.cos(base_angle + blade_angle_span / 2)
        y_base_trail = hub_radius * math.sin(base_angle + blade_angle_span / 2)

        # Blade tip (curved forward)
        tip_angle = base_angle + 0.15  # Curved blade
        x_tip = radius * 0.9 * math.cos(tip_angle)
        y_tip = radius * 0.9 * math.sin(tip_angle)

        # Draw blade outline
        points.extend([[x_base_lead, y_base_lead, 0], [x_tip, y_tip, 0]])
        points.extend([[x_tip, y_tip, 0], [x_base_trail, y_base_trail, 0]])
        points.extend([[x_base_lead, y_base_lead, 0], [x_base_trail, y_base_trail, 0]])

        # Add internal blade line for detail
        mid_radius = (hub_radius + radius * 0.9) / 2
        x_mid = mid_radius * math.cos(base_angle)
        y_mid = mid_radius * math.sin(base_angle)
        points.extend([[x_base_lead, y_base_lead, 0], [x_mid, y_mid, 0]])
        points.extend([[x_mid, y_mid, 0], [x_tip, y_tip, 0]])

    # Spoke lines from hub to outer rim (between blades)
    for i in range(blades):
        angle = math.radians(i * (360 / blades) + (180 / blades))  # Between blades
        x_hub = hub_radius * math.cos(angle)
        y_hub = hub_radius * math.sin(angle)
        x_outer = radius * 0.95 * math.cos(angle)
        y_outer = radius * 0.95 * math.sin(angle)
        points.extend([[x_hub, y_hub, 0], [x_outer, y_outer, 0]])

    return np.array(points, dtype=np.float32)


def create_wheel_wireframe(width: float, radius: float,
                           tread_lines: int = 8) -> np.ndarray:
    """
    Create wireframe for a drive wheel as a cylinder.

    The wheel is a cylinder oriented with its axis along X.
    Rotation happens around the X axis (rolling forward/backward).

    Args:
        width: Wheel width/thickness (X dimension)
        radius: Wheel radius (Y-Z plane)
        tread_lines: Number of tread pattern lines around circumference

    Returns:
        Nx3 array of line segment endpoints
    """
    points = []
    half_width = width / 2
    segments = 32  # Smooth circle

    angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)

    # Left face circle (at -half_width on X axis)
    for i in range(segments):
        y1 = radius * np.cos(angles[i])
        z1 = radius * np.sin(angles[i])
        y2 = radius * np.cos(angles[(i + 1) % segments])
        z2 = radius * np.sin(angles[(i + 1) % segments])
        points.extend([[-half_width, y1, z1], [-half_width, y2, z2]])

    # Right face circle (at +half_width on X axis)
    for i in range(segments):
        y1 = radius * np.cos(angles[i])
        z1 = radius * np.sin(angles[i])
        y2 = radius * np.cos(angles[(i + 1) % segments])
        z2 = radius * np.sin(angles[(i + 1) % segments])
        points.extend([[half_width, y1, z1], [half_width, y2, z2]])

    # Connecting lines between faces (tread lines)
    tread_step = max(1, segments // tread_lines)
    for i in range(0, segments, tread_step):
        y = radius * np.cos(angles[i])
        z = radius * np.sin(angles[i])
        points.extend([[-half_width, y, z], [half_width, y, z]])

    # Hub circles (smaller inner circles for visual detail)
    hub_radius = radius * 0.35
    for i in range(segments):
        y1 = hub_radius * np.cos(angles[i])
        z1 = hub_radius * np.sin(angles[i])
        y2 = hub_radius * np.cos(angles[(i + 1) % segments])
        z2 = hub_radius * np.sin(angles[(i + 1) % segments])
        points.extend([[-half_width, y1, z1], [-half_width, y2, z2]])
        points.extend([[half_width, y1, z1], [half_width, y2, z2]])

    # Spoke lines from hub to rim (on both faces) - makes rotation visible
    spoke_count = 5
    spoke_step = segments // spoke_count
    for i in range(0, segments, spoke_step):
        y_hub = hub_radius * np.cos(angles[i])
        z_hub = hub_radius * np.sin(angles[i])
        y_rim = radius * np.cos(angles[i])
        z_rim = radius * np.sin(angles[i])
        points.extend([[-half_width, y_hub, z_hub], [-half_width, y_rim, z_rim]])
        points.extend([[half_width, y_hub, z_hub], [half_width, y_rim, z_rim]])

    return np.array(points, dtype=np.float32)


def rotate_wheel_x(points: np.ndarray, angle_deg: float) -> np.ndarray:
    """
    Rotate wheel geometry around X axis (for wheel rolling motion).

    This rotates in the Y-Z plane, which makes the wheel appear to roll.

    Args:
        points: Nx3 array of points
        angle_deg: Rotation angle in degrees

    Returns:
        Rotated points array
    """
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    rotated = points.copy()
    # Rotate Y and Z coordinates around X axis
    rotated[:, 1] = points[:, 1] * cos_a - points[:, 2] * sin_a
    rotated[:, 2] = points[:, 1] * sin_a + points[:, 2] * cos_a
    return rotated


def translate_geometry(points: np.ndarray, dx: float, dy: float, dz: float) -> np.ndarray:
    """Translate geometry by given offset."""
    offset = np.array([dx, dy, dz], dtype=np.float32)
    return points + offset


def rotate_geometry_z(points: np.ndarray, angle_deg: float) -> np.ndarray:
    """Rotate geometry around Z axis."""
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    rotated = points.copy()
    rotated[:, 0] = points[:, 0] * cos_a - points[:, 1] * sin_a
    rotated[:, 1] = points[:, 0] * sin_a + points[:, 1] * cos_a
    return rotated


def rotate_geometry_x(points: np.ndarray, angle_deg: float) -> np.ndarray:
    """Rotate geometry around X axis."""
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    rotated = points.copy()
    rotated[:, 1] = points[:, 1] * cos_a - points[:, 2] * sin_a
    rotated[:, 2] = points[:, 1] * sin_a + points[:, 2] * cos_a
    return rotated


def rotate_geometry_y(points: np.ndarray, angle_deg: float) -> np.ndarray:
    """Rotate geometry around Y axis (for wheel rotation)."""
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    rotated = points.copy()
    rotated[:, 0] = points[:, 0] * cos_a + points[:, 2] * sin_a
    rotated[:, 2] = -points[:, 0] * sin_a + points[:, 2] * cos_a
    return rotated


def create_lidar_turret_wireframe(radius: float, height: float,
                                   segments: int = 32, vertical_lines: int = 8) -> np.ndarray:
    """
    Create wireframe for the lidar turret outer housing.

    Args:
        radius: Turret radius
        height: Turret height
        segments: Number of segments around circumference
        vertical_lines: Number of vertical lines

    Returns:
        Nx3 array of line segment endpoints
    """
    return create_cylinder_wireframe(radius, height, segments, vertical_lines)


def create_lidar_spinner_wireframe(radius: float, height: float,
                                    segments: int = 16, blades: int = 4) -> np.ndarray:
    """
    Create wireframe for the lidar inner spinning element.

    Shows a smaller cylinder with visible internal structure that rotates.

    Args:
        radius: Spinner radius (smaller than turret)
        height: Spinner height
        segments: Number of segments around circumference
        blades: Number of blade/spoke lines

    Returns:
        Nx3 array of line segment endpoints
    """
    points = []
    half_height = height / 2

    angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)

    # Top circle
    for i in range(segments):
        x1 = radius * np.cos(angles[i])
        y1 = radius * np.sin(angles[i])
        x2 = radius * np.cos(angles[(i + 1) % segments])
        y2 = radius * np.sin(angles[(i + 1) % segments])
        points.extend([[x1, y1, half_height], [x2, y2, half_height]])

    # Bottom circle
    for i in range(segments):
        x1 = radius * np.cos(angles[i])
        y1 = radius * np.sin(angles[i])
        x2 = radius * np.cos(angles[(i + 1) % segments])
        y2 = radius * np.sin(angles[(i + 1) % segments])
        points.extend([[x1, y1, -half_height], [x2, y2, -half_height]])

    # Vertical lines
    step = max(1, segments // blades)
    for i in range(0, segments, step):
        x = radius * np.cos(angles[i])
        y = radius * np.sin(angles[i])
        points.extend([[x, y, half_height], [x, y, -half_height]])

    # Internal cross/blade structure (makes rotation visible)
    for i in range(blades):
        angle = math.radians(i * (360 / blades))
        x_outer = radius * 0.9 * math.cos(angle)
        y_outer = radius * 0.9 * math.sin(angle)
        # Blade from center to edge at multiple heights
        for z in [-half_height * 0.5, 0, half_height * 0.5]:
            points.extend([[0, 0, z], [x_outer, y_outer, z]])

    return np.array(points, dtype=np.float32)
