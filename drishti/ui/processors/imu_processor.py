"""
IMU Processor - Orientation estimation from gyroscope integration.

Integrates all three gyroscope axes for real-time responsive rotation.
Note: Will drift over time without absolute reference (accelerometer/magnetometer).
"""


class IMUProcessor:
    """
    IMU orientation processor using pure gyroscope integration.

    Integrates gyroscope for all axes (roll, pitch, yaw) at 500Hz.
    Provides smooth real-time orientation but will drift over time.
    """

    # Scale factor: raw gyro I16 values to degrees/sec
    # Empirically tuned to match physical rotation on CRL-200S
    GYRO_SCALE = 1.0 / 10.0

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

        # Decimation for UI updates (500Hz -> 100Hz for label updates)
        self._counter = 0
        self._decimation = 5  # Update every 5th sample = 100Hz

    def process(self, gyro_x: int, gyro_y: int, gyro_z: int,
                tilt_x: int, tilt_y: int, tilt_z: int) -> tuple:
        """
        Process one IMU sample and return orientation.

        Args:
            gyro_x, gyro_y, gyro_z: Raw gyroscope values (i16)
            tilt_x, tilt_y, tilt_z: Filtered tilt values (unused, for API compat)

        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        # Convert raw gyro to degrees/sec and integrate
        # Axis mapping tuned for CRL-200S physical orientation
        gx = gyro_x * self.GYRO_SCALE
        gy = gyro_y * self.GYRO_SCALE
        gz = gyro_z * self.GYRO_SCALE

        self.roll += gz * self.dt    # Z gyro -> roll
        self.pitch += gy * self.dt   # Y gyro -> pitch
        self.yaw += gx * self.dt     # X gyro -> yaw

        # Normalize to -180 to +180 range
        self.roll = ((self.roll + 180) % 360) - 180
        self.pitch = ((self.pitch + 180) % 360) - 180
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
