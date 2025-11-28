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
pub const SERIAL_READ_TIMEOUT_MS: u64 = 50;
pub const INIT_RETRY_DELAY_MS: u64 = 100;

// Packet sizes
pub const MIN_PACKET_SIZE: usize = 6; // SYNC(2) + LEN(1) + CMD(1) + CRC(2)
pub const STATUS_PAYLOAD_MIN_SIZE: usize = 80;
pub const MAX_BUFFER_SIZE: usize = 1024;

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
pub const OFFSET_WATER_TANK_LEVEL: usize = 0x46; // Water tank level (0 or 100) for 2-in-1 box

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
