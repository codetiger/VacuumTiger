"""
IMU Processor - Mahony AHRS filter with quaternion orientation.

Coordinate Convention (ROS REP-103):
  SangamIO transforms raw IMU data to standard robot frame:
  - gyro_x = Roll rate  (X axis rotation, left/right tilt)
  - gyro_y = Pitch rate (Y axis rotation, nose up/down)
  - gyro_z = Yaw rate   (Z axis rotation, CCW positive)

  Robot frame:
  - X = forward (direction robot drives)
  - Y = left (port side)
  - Z = up

Note: The axis remapping from hardware to ROS convention is done in SangamIO
via the [device.hardware.frame_transforms.imu_gyro] config. This processor
receives already-transformed data.
"""

import math


class IMUProcessor:
    """Mahony AHRS filter for stable orientation estimation."""

    # Mahony filter gains
    # Higher KP = faster correction, Higher KI = faster bias adaptation
    KP = 5.0      # Proportional gain (increased for faster settling)
    KI = 0.1      # Integral gain (increased for faster bias correction)

    # Scale factor: raw gyro to rad/s
    # Raw units are 0.1 deg/s, convert to rad/s
    GYRO_SCALE = 0.1 * (math.pi / 180.0)

    # Gravity scale (LP filtered gravity vector)
    GRAVITY_SCALE = 1.0 / 1000.0

    def __init__(self, sample_rate_hz: float = 500.0):
        self.dt = 1.0 / sample_rate_hz

        # Quaternion [w, x, y, z]
        self.q = [1.0, 0.0, 0.0, 0.0]

        # Integral error for bias correction
        self.integral_error = [0.0, 0.0, 0.0]

        # Gyro bias (calibrated from initial samples)
        self.gyro_bias = [0.0, 0.0, 0.0]
        self._calibration_samples = []
        self._calibration_count = 0
        self._calibration_duration = 1500  # ~3 seconds at 500Hz

        # UI decimation
        self._counter = 0
        self._decimation = 5

    def process(self, gyro_x: int, gyro_y: int, gyro_z: int,
                tilt_x: int, tilt_y: int, tilt_z: int) -> tuple:
        """
        Process IMU data using Mahony AHRS filter.

        Args:
            gyro_x: Roll rate (X axis rotation) - already transformed by SangamIO
            gyro_y: Pitch rate (Y axis rotation) - already transformed by SangamIO
            gyro_z: Yaw rate (Z axis rotation, CCW positive) - already transformed by SangamIO
            tilt_x: LP gravity X
            tilt_y: LP gravity Y
            tilt_z: LP gravity Z

        Note: SangamIO applies axis transforms via [device.hardware.frame_transforms.imu_gyro]
        so input data is already in ROS REP-103 convention.

        Returns:
            (roll, pitch, yaw) in degrees
        """

        # Calibration phase - collect bias samples
        if self._calibration_count < self._calibration_duration:
            self._calibration_samples.append((gyro_x, gyro_y, gyro_z))
            self._calibration_count += 1

            if self._calibration_count == self._calibration_duration:
                # Calculate average bias
                n = len(self._calibration_samples)
                self.gyro_bias[0] = sum(s[0] for s in self._calibration_samples) / n
                self.gyro_bias[1] = sum(s[1] for s in self._calibration_samples) / n
                self.gyro_bias[2] = sum(s[2] for s in self._calibration_samples) / n
                self._calibration_samples = []  # Free memory

            # During calibration, return zeros
            return (0.0, 0.0, 0.0)

        # Remove bias and convert to rad/s
        # SangamIO sends gyro data in ROS REP-103 convention:
        #   gyro_x = Roll rate (from hardware gyro_z)
        #   gyro_y = Pitch rate (from hardware gyro_y)
        #   gyro_z = Yaw rate (from hardware gyro_x, sign flipped)
        #
        # The OLD code (before SangamIO transforms) worked with raw hardware:
        #   gx = raw_gyro_z (Roll)
        #   gy = raw_gyro_y (Pitch)
        #   gz = raw_gyro_x (Yaw)
        #
        # Now SangamIO remaps, so new gyro_x = old gyro_z, etc.
        # But gyro_z has a sign flip, so we negate it to match old behavior.
        gx = (gyro_x - self.gyro_bias[0]) * self.GYRO_SCALE  # Roll
        gy = (gyro_y - self.gyro_bias[1]) * self.GYRO_SCALE  # Pitch
        gz = -(gyro_z - self.gyro_bias[2]) * self.GYRO_SCALE  # Yaw (negate to undo SangamIO sign flip)

        # Get current quaternion values
        q0, q1, q2, q3 = self.q

        # Normalize LP gravity vector
        ax = tilt_x * self.GRAVITY_SCALE
        ay = tilt_y * self.GRAVITY_SCALE
        az = tilt_z * self.GRAVITY_SCALE

        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm > 0.01:
            ax /= norm
            ay /= norm
            az /= norm

            # Estimated gravity direction from quaternion
            # v = [2(q1q3 - q0q2), 2(q0q1 + q2q3), q0² - q1² - q2² + q3²]
            vx = 2.0 * (q1 * q3 - q0 * q2)
            vy = 2.0 * (q0 * q1 + q2 * q3)
            vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

            # Error is cross product between estimated and measured gravity
            ex = ay * vz - az * vy
            ey = az * vx - ax * vz
            ez = ax * vy - ay * vx

            # Integral feedback
            self.integral_error[0] += ex * self.dt
            self.integral_error[1] += ey * self.dt
            self.integral_error[2] += ez * self.dt

            # Apply feedback
            gx += self.KP * ex + self.KI * self.integral_error[0]
            gy += self.KP * ey + self.KI * self.integral_error[1]
            gz += self.KP * ez + self.KI * self.integral_error[2]

        # Quaternion derivative
        dq0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        dq1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        dq2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        dq3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        # Integrate
        self.q[0] += dq0 * self.dt
        self.q[1] += dq1 * self.dt
        self.q[2] += dq2 * self.dt
        self.q[3] += dq3 * self.dt

        # Normalize quaternion
        norm = math.sqrt(sum(x * x for x in self.q))
        if norm > 0:
            self.q = [x / norm for x in self.q]

        # Convert quaternion to Euler angles (ZYX convention)
        q0, q1, q2, q3 = self.q

        # Roll (X axis rotation)
        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

        # Pitch (Y axis rotation)
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(90.0, sinp)
        else:
            pitch = math.asin(sinp) * (180.0 / math.pi)

        # Yaw (Z axis rotation)
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

        return (roll, pitch, yaw)

    def should_update_ui(self) -> bool:
        self._counter += 1
        if self._counter >= self._decimation:
            self._counter = 0
            return True
        return False

    def reset(self):
        """Reset orientation to identity."""
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.integral_error = [0.0, 0.0, 0.0]
        self._counter = 0

    def recalibrate(self):
        """Reset and restart gyro bias calibration."""
        self.reset()
        self.gyro_bias = [0.0, 0.0, 0.0]
        self._calibration_samples = []
        self._calibration_count = 0
