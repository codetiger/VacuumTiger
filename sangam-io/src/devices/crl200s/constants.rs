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

// Lidar control commands
pub const CMD_MOTOR_MODE: u8 = 0x65; // Motor mode switch (0x02 = navigation mode)
pub const CMD_LIDAR_PREP: u8 = 0xA2; // Lidar preparation
pub const CMD_LIDAR_POWER: u8 = 0x97; // Lidar power on/off
pub const CMD_LIDAR_PWM: u8 = 0x71; // Lidar motor speed (0-100%)

// Timing constants
pub const SERIAL_READ_TIMEOUT_MS: u64 = 50;
pub const INIT_RETRY_DELAY_MS: u64 = 100;

// Packet sizes
pub const MIN_PACKET_SIZE: usize = 6; // SYNC(2) + LEN(1) + CMD(1) + CRC(2)
pub const STATUS_PAYLOAD_MIN_SIZE: usize = 80;
pub const MAX_BUFFER_SIZE: usize = 1024;

// Sensor data offsets in status packet
pub const OFFSET_CHARGING_FLAGS: usize = 7;
pub const OFFSET_BUMPER_FLAGS: usize = 1;
pub const OFFSET_CLIFF_FLAGS: usize = 3;
pub const OFFSET_DUSTBOX_FLAGS: usize = 4;
pub const OFFSET_WHEEL_RIGHT_ENCODER: usize = 0x10;
pub const OFFSET_WHEEL_LEFT_ENCODER: usize = 0x18;
pub const OFFSET_START_BUTTON: usize = 0x3A;
pub const OFFSET_DOCK_BUTTON: usize = 0x3E;

// Flag masks
pub const FLAG_CHARGING: u8 = 0x02;
pub const FLAG_BATTERY_CONNECTED: u8 = 0x01;
pub const FLAG_BUMPER_RIGHT: u8 = 0x02;
pub const FLAG_BUMPER_LEFT: u8 = 0x04;
pub const FLAG_CLIFF_LEFT_SIDE: u8 = 0x01;
pub const FLAG_CLIFF_LEFT_FRONT: u8 = 0x02;
pub const FLAG_CLIFF_RIGHT_FRONT: u8 = 0x04;
pub const FLAG_CLIFF_RIGHT_SIDE: u8 = 0x08;
pub const FLAG_DUSTBOX_ATTACHED: u8 = 0x04;
