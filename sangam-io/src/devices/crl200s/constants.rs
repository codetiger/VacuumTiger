//! Constants for CRL-200S device (GD32 motor controller)

// Sync bytes
pub const SYNC_BYTE_1: u8 = 0xFA;
pub const SYNC_BYTE_2: u8 = 0xFB;

// Command IDs
pub const CMD_INITIALIZE: u8 = 0x08; // Wake up device (no checksum)
pub const CMD_HEARTBEAT: u8 = 0x06; // Keep-alive
pub const CMD_VERSION: u8 = 0x07; // Version request/response
pub const CMD_STATUS: u8 = 0x15; // Sensor status data (96 bytes)

// Motor control commands
pub const CMD_MOTOR_VELOCITY: u8 = 0x66; // Differential drive (linear + angular)
pub const CMD_MOTOR_SPEED: u8 = 0x67; // Direct wheel control

// Actuator commands
pub const CMD_AIR_PUMP: u8 = 0x68; // BlowerSpeed 0-100%
pub const CMD_SIDE_BRUSH: u8 = 0x69; // SideBrushSpeed 0-100%
pub const CMD_MAIN_BRUSH: u8 = 0x6A; // RollingBrushSpeed 0-100%
pub const CMD_WATER_PUMP: u8 = 0x6B; // Water pump for 2-in-1 mop box (0-100%)
pub const CMD_BUTTON_LED: u8 = 0x8D; // LED state (0=off, 1=charging, 3=discharge, 6=charged, 11=standby)

// Lidar control commands
pub const CMD_MOTOR_MODE: u8 = 0x65; // Motor mode switch (0x02 = navigation mode)
pub const CMD_LIDAR_POWER: u8 = 0x97; // Lidar power on/off
pub const CMD_LIDAR_PWM: u8 = 0x71; // Lidar motor speed (0-100%)

// Sensor control commands
pub const CMD_CLIFF_IR_CONTROL: u8 = 0x78; // Cliff IR on/off (0=off, 1=on)
pub const CMD_CLIFF_IR_DIRECTION: u8 = 0x79; // Cliff IR direction

// Calibration commands
pub const CMD_IMU_FACTORY_CALIBRATE: u8 = 0xA1; // Trigger factory IMU calibration
pub const CMD_IMU_CALIBRATE_STATE: u8 = 0xA2; // Query IMU factory calibration state
pub const CMD_COMPASS_CALIBRATE: u8 = 0xA3; // Start compass/geomagnetism calibration
pub const CMD_COMPASS_CALIBRATION_STATE: u8 = 0xA4; // Query compass calibration state

// System polling commands
pub const CMD_REQUEST_STM32_DATA: u8 = 0x0D; // Request STM32 sensor data (polled every ~3s)

// MCU control commands
pub const CMD_MCU_SLEEP: u8 = 0x04; // Put GD32 MCU to sleep
pub const CMD_WAKEUP_ACK: u8 = 0x05; // Acknowledge wakeup from sleep
pub const CMD_RESET_ERROR_CODE: u8 = 0x0A; // Reset/clear error codes
pub const CMD_PROTOCOL_SYNC: u8 = 0x0C; // Protocol sync - first command at boot, wakes GD32

// Power management commands
pub const CMD_MAIN_BOARD_POWER: u8 = 0x99; // Main board (A33) power control
pub const CMD_MAIN_BOARD_RESTART: u8 = 0x9A; // Restart main board (A33 Linux system)
pub const CMD_CHARGER_POWER: u8 = 0x9B; // Charger power control

// Timing constants
// 5ms timeout optimized for ~110Hz packet rate (limited by 115200 baud with ~100 byte packets)
pub const SERIAL_READ_TIMEOUT_MS: u64 = 5;
pub const INIT_RETRY_DELAY_MS: u64 = 100;

// Packet sizes
pub const STATUS_PAYLOAD_MIN_SIZE: usize = 80;

// Sensor data offsets in status packet
pub const OFFSET_CHARGING_FLAGS: usize = 0x07;
pub const OFFSET_BATTERY_VOLTAGE_RAW: usize = 0x08; // Raw voltage byte (divide by 10 for volts)
pub const OFFSET_BUMPER_FLAGS: usize = 0x01;
pub const OFFSET_CLIFF_FLAGS: usize = 0x03;
pub const OFFSET_DUSTBOX_FLAGS: usize = 0x04;
pub const OFFSET_WHEEL_RIGHT_ENCODER: usize = 0x10;
pub const OFFSET_WHEEL_LEFT_ENCODER: usize = 0x18;
pub const OFFSET_START_BUTTON: usize = 0x3A;
pub const OFFSET_DOCK_BUTTON: usize = 0x3E;

// IMU data offsets (interleaved: [Gx][Ax][Gy][Ay][Gz][Az][LP_Ax][LP_Ay][LP_Az])
// NOTE: Raw hardware positions. Axis transformation to ROS REP-103 frame is applied
// via [device.hardware.frame_transforms.imu_gyro] and [device.hardware.frame_transforms.imu_accel]
// in sangamio.toml. Default identity transform = pass-through (no remapping).
//
// CRL-200S hardware axis mapping (before transform):
//   B40-41 (OFFSET_GYRO_X): Gyro Yaw rate   (most active during flat rotation)
//   B44-45 (OFFSET_GYRO_Y): Gyro Pitch rate (most active during nose up/down)
//   B48-49 (OFFSET_GYRO_Z): Gyro Roll rate  (most active during left/right tilt)
//
// CRL-200S transform remaps to ROS standard: raw_x->out_z (with sign flip), raw_y->out_y, raw_z->out_x
pub const OFFSET_GYRO_X: usize = 0x28; // B40-41: Gyro X raw (i16 LE)
pub const OFFSET_ACCEL_X: usize = 0x2A; // B42-43: Accel X raw (i16 LE)
pub const OFFSET_GYRO_Y: usize = 0x2C; // B44-45: Gyro Y raw (i16 LE)
pub const OFFSET_ACCEL_Y: usize = 0x2E; // B46-47: Accel Y raw (i16 LE)
pub const OFFSET_GYRO_Z: usize = 0x30; // B48-49: Gyro Z raw (i16 LE)
pub const OFFSET_ACCEL_Z: usize = 0x32; // B50-51: Accel Z raw (i16 LE)

// LP filtered gravity vector for tilt correction
pub const OFFSET_TILT_X: usize = 0x34; // B52-53: LP Gravity X (i16 LE)
pub const OFFSET_TILT_Y: usize = 0x36; // B54-55: LP Gravity Y (i16 LE)
pub const OFFSET_TILT_Z: usize = 0x38; // B56-57: LP Gravity Z (i16 LE)

// Battery voltage thresholds (from CFactoryBatteryControl)
pub const BATTERY_VOLTAGE_MIN: f32 = 13.5; // Critical low (0%)
pub const BATTERY_VOLTAGE_MAX: f32 = 15.5; // Fully charged (100%)

// Flag masks
pub const FLAG_CHARGING: u8 = 0x02;
pub const FLAG_DOCK_CONNECTED: u8 = 0x01;
pub const FLAG_BUMPER_RIGHT: u8 = 0x02;
pub const FLAG_BUMPER_LEFT: u8 = 0x04;
pub const FLAG_CLIFF_LEFT_SIDE: u8 = 0x01;
pub const FLAG_CLIFF_LEFT_FRONT: u8 = 0x02;
pub const FLAG_CLIFF_RIGHT_FRONT: u8 = 0x04;
pub const FLAG_CLIFF_RIGHT_SIDE: u8 = 0x08;
pub const FLAG_DUSTBOX_ATTACHED: u8 = 0x04;
