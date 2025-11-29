"""
IMU Processor - Mahony AHRS filter with quaternion orientation.

Movement-validated axis mapping:
  B40-41: Gyro Yaw rate   (most active during flat rotation) -> Z axis
  B44-45: Gyro Pitch rate (most active during nose up/down)  -> Y axis
  B48-49: Gyro Roll rate  (most active during left/right tilt) -> X axis
  B52-57: LP Gravity vector for tilt correction

Physical axes:
  Y axis = Nose up/down (pitch)
  X axis = Left/Right tilt (roll)
  Z axis = Rotating on floor (yaw)
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
            gyro_x: Raw gyro X (B40-41) - Yaw rate (Z rotation)
            gyro_y: Raw gyro Y (B44-45) - Pitch rate (Y rotation)
            gyro_z: Raw gyro Z (B48-49) - Roll rate (X rotation)
            tilt_x: LP gravity X (B52-53)
            tilt_y: LP gravity Y (B54-55)
            tilt_z: LP gravity Z (B56-57)

        Data order is interleaved: [Gx][Ax][Gy][Ay][Gz][Az][LP_Ax][LP_Ay][LP_Az]

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
        # With interleaved data order [Gx][Ax][Gy][Ay][Gz][Az]:
        # gyro_x (B40-41) = Yaw rate -> gz (Z axis rotation)
        # gyro_y (B44-45) = Pitch rate -> gy (Y axis nose up/down)
        # gyro_z (B48-49) = Roll rate -> gx (X axis tilt)
        gx = (gyro_z - self.gyro_bias[2]) * self.GYRO_SCALE  # Roll
        gy = (gyro_y - self.gyro_bias[1]) * self.GYRO_SCALE  # Pitch
        gz = (gyro_x - self.gyro_bias[0]) * self.GYRO_SCALE  # Yaw

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
