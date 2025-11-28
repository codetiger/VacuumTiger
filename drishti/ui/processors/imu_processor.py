"""
IMU Processor - Orientation estimation from tilt vector and gyroscope.

The tilt vector from GD32 is already low-pass filtered, so we use it
directly for roll/pitch. Gyroscope is integrated for yaw (which will drift).
"""

import math


class IMUProcessor:
    """
    IMU orientation processor.

    Uses the pre-filtered tilt vector directly for roll/pitch angles.
    Integrates gyroscope for yaw (no absolute reference without magnetometer).
    """

    # Scale factor for raw gyro I16 values to degrees/sec
    # Typical MPU6050: 131 LSB/(deg/s) at ±250 deg/s range
    GYRO_SCALE = 1.0 / 131.0  # degrees/sec per LSB

    def __init__(self, sample_rate_hz: float = 500.0):
        """
        Initialize the IMU processor.

        Args:
            sample_rate_hz: Expected sample rate of incoming data
        """
        self.dt = 1.0 / sample_rate_hz

        # Orientation state (degrees)
        self.roll = 0.0   # X-axis rotation (tilt left/right)
        self.pitch = 0.0  # Y-axis rotation (tilt forward/back)
        self.yaw = 0.0    # Z-axis rotation (heading)

        # Decimation for UI updates (500Hz -> 100Hz for smoother response)
        self._counter = 0
        self._decimation = 5  # Update every 5th sample = 100Hz

    def process(self, gyro_x: int, gyro_y: int, gyro_z: int,
                tilt_x: int, tilt_y: int, tilt_z: int) -> tuple:
        """
        Process one IMU sample and return orientation.

        Args:
            gyro_x, gyro_y, gyro_z: Raw gyroscope values (i16)
            tilt_x, tilt_y, tilt_z: Low-pass filtered gravity vector (i16)

        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        # Calculate magnitude of tilt vector
        magnitude = math.sqrt(tilt_x**2 + tilt_y**2 + tilt_z**2)
        if magnitude < 100:  # Near zero, sensor error
            return (self.roll, self.pitch, self.yaw)

        # Roll and Pitch directly from tilt vector (already filtered by GD32)
        # Roll: rotation around X-axis (left/right tilt)
        self.roll = math.degrees(
            math.atan2(tilt_y, math.sqrt(tilt_x**2 + tilt_z**2))
        )

        # Pitch: rotation around Y-axis (forward/back tilt)
        self.pitch = math.degrees(
            math.atan2(-tilt_x, math.sqrt(tilt_y**2 + tilt_z**2))
        )

        # Yaw: integrate gyroscope Z (will drift without magnetometer)
        gz = gyro_z * self.GYRO_SCALE
        self.yaw += gz * self.dt
        # Normalize to -180 to +180 range
        self.yaw = ((self.yaw + 180) % 360) - 180

        return (self.roll, self.pitch, self.yaw)

    def should_update_ui(self) -> bool:
        """
        Rate limit UI updates to ~100Hz.

        Call this after process() to determine if UI should be updated.

        Returns:
            True if UI should be updated, False otherwise
        """
        self._counter += 1
        if self._counter >= self._decimation:
            self._counter = 0
            return True
        return False

    def reset(self):
        """Reset orientation to zero."""
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._counter = 0
