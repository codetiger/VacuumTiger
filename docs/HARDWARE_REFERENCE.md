# Hardware Reference

**Source**: Reverse engineered from 3irobotix CRL-200S vacuum robot
**Target**: Custom vacuum robot firmware

## Hardware Architecture Overview

The 3irobotix CRL-200S uses a dual-processor architecture that serves as reference for understanding the hardware requirements:

```
┌─────────────────────────────────────────────────────┐
│                 Main Processor                      │
│          Allwinner A33 (ARM Cortex-A7)              │
│         - Quad-Core @ 1.2GHz                        │
│         - Runs Tina Linux (OpenWrt-based)           │
│         - Handles: WiFi, UI, Navigation             │
│                                                     │
│  ┌──────────────┐              ┌─────────────────┐  │
│  │   /dev/ttyS3 │◄────UART────►│   /dev/ttyS1    │  │
└──┴──────────────┴──────────────┴─────────────────┴──┘
         │                                  │
         │ 115200 baud                      │ 115200 baud
         │ 8N1                              │ 8N1
         ▼                                  ▼
┌──────────────────┐              ┌──────────────────┐
│   GD32F103VCT6   │              │  3iRobotix       │
│   (Cortex-M3)    │              │  Delta-2D Lidar  │
│   @ 108MHz       │              │                  │
│                  │              │  360° scanning   │
│  - Motor control │              │  12m range       │
│  - Sensor I/O    │              │                  │
│  - IMU reading   │              └──────────────────┘
│  - Real-time     │
│    control       │
└──────────────────┘
```

## Component Specifications

### Main Processor: Allwinner A33

| Specification | Details |
|---------------|---------|
| **Architecture** | ARM Cortex-A7 Quad-Core |
| **Clock Speed** | Up to 1.2 GHz |
| **Operating System** | Tina Linux (OpenWrt-based) |
| **Purpose** | High-level logic, WiFi, navigation algorithms |

**Key Features**:
- Multiple UART interfaces (ttyS0, ttyS1, ttyS2, ttyS3)
- GPIO for peripheral control
- WiFi connectivity (via Realtek RTL8189ETV)
- Storage interface for firmware and maps

### Motor Control MCU: GigaDevice GD32F103VCT6

| Specification | Details |
|---------------|---------|
| **Architecture** | ARM Cortex-M3 |
| **Clock Speed** | 108 MHz |
| **Flash** | 256 KB |
| **SRAM** | 48 KB |
| **Package** | LQFP100 |
| **Purpose** | Real-time motor control, sensor interfacing |

**Key Features**:
- Multiple timers for PWM motor control
- ADC for analog sensor reading
- UART for communication with main processor
- GPIO for digital sensors and actuators
- SPI/I2C for IMU communication

### Sensors and Peripherals

#### Lidar: 3iRobotix Delta-2D

| Specification | Details |
|---------------|---------|
| **Interface** | UART (115200 baud, 8N1) |
| **Range** | 12 meters |
| **Scanning** | 360 degrees |
| **Sample Rate** | ~5-10 Hz (full rotation) |
| **Motor Power** | Requires separate 5V supply |
| **Data Protocol** | Custom binary (documented in VacuumRobot repo) |

**Pin Configuration** (5-pin connector):
1. Motor Power (+)
2. Motor Power (GND)
3. UART TX (Lidar → CPU)
4. UART RX (CPU → Lidar) - May be unused
5. GND

#### IMU (Inertial Measurement Unit)

Integrated on GD32 side, accessible through GD32 protocol:
- **Accelerometer**: 3-axis acceleration measurement
- **Gyroscope**: 3-axis angular velocity
- **Magnetometer**: 3-axis magnetic field (compass)

Used for:
- Orientation tracking
- Collision detection
- Gyro-assisted navigation

#### Wheel Encoders

Connected to GD32:
- **Type**: Quadrature encoders (assumed)
- **Purpose**: Odometry (distance and direction tracking)
- **Data**: Left and right wheel encoder counts in CMD=0x15 status packets

#### Cliff Sensors

Infrared sensors for drop-off detection:
- **Control**: CMD=0x78 (enable/disable)
- **Direction**: CMD=0x79 (set direction)
- **Data**: Readings in CMD=0x15 status packets

#### Bumper Sensors

Collision detection switches:
- **Type**: Digital contact switches
- **Data**: State reported in CMD=0x15 status packets

#### Button and LEDs

- **Button State**: Reported in CMD=0x15 status packets
- **LED Control**: CMD=0x8D (set button LED state)

### Actuators

#### Drive Motors

- **Type**: DC motors with encoders
- **Control**: Via GD32 (CMD=0x67 for direct speed)
- **Control Mode**: CMD=0x65 (set control type)

#### Vacuum Blower

- **Type**: Brushless DC motor
- **Control**: CMD=0x68 (set speed, uint16)
- **Speed Range**: 0-65535 (PWM duty cycle)

#### Side Brush Motor

- **Type**: DC motor
- **Control**: CMD=0x69 (set speed, uint8)
- **Speed Range**: 0-255

#### Rolling Brush Motor

- **Type**: DC motor
- **Control**: CMD=0x6A (set speed, uint8)
- **Speed Range**: 0-255

### Power System

#### Battery

- **Type**: 4-cell Li-ion
- **Voltage**: ~14.8V nominal
- **Charge Controller**: CN3704
- **Monitoring**: Voltage and current reported via CMD=0x15

#### Power Management IC

- **Chip**: X-Powers AXP223
- **Channels**: 21-channel PMIC
- **Purpose**: Power distribution, voltage regulation

#### Power Control Commands

- **Lidar Power**: CMD=0x97
- **GD32 Power**: CMD=0x99
- **Charger Control**: CMD=0x9B

### Memory

| Component | Specification |
|-----------|---------------|
| **DDR3L SDRAM** | 4Gbit (512 MB) |
| **NAND Flash** | 2Gbit (256 MB) |

### Wireless

| Component | Specification |
|-----------|---------------|
| **WiFi Module** | Realtek RTL8189ETV |
| **Standard** | 802.11n |
| **Interface** | SDIO |

## Connector Pinouts

### Main Processor ↔ GD32 (UART)

| Pin | Signal | Direction | Description |
|-----|--------|-----------|-------------|
| 1 | TX | A33→GD32 | Transmit data |
| 2 | RX | GD32→A33 | Receive data |
| 3 | GND | - | Ground |

**Configuration**: 115200 baud, 8N1, no flow control

### Lidar Connector

| Pin | Signal | Voltage | Description |
|-----|--------|---------|-------------|
| 1 | V_MOTOR+ | 5V | Motor power positive |
| 2 | V_MOTOR- | GND | Motor power ground |
| 3 | UART_TX | 3.3V | Lidar → CPU data |
| 4 | UART_RX | 3.3V | CPU → Lidar (may be unused) |
| 5 | GND | GND | Signal ground |

## GPIO Assignments

**Note**: GPIO numbers are device-specific. The following are examples from the reference hardware:

| GPIO | Function | Notes |
|------|----------|-------|
| gpio-39 | Unknown | May be related to GD32 control |
| gpio-107 | Lidar Motor | Possible motor enable/control |

## Implementation Considerations

### For Custom Hardware

When building custom vacuum robot hardware:

1. **Processor Selection**:
   - Main CPU: Any Linux-capable ARM/x86 with UART support
   - MCU: GD32F103 or compatible (STM32F103 with modified firmware)
   - Ensure sufficient UART interfaces (2 minimum: GD32 + Lidar)

2. **Real-time Requirements**:
   - GD32 heartbeat must be maintained at 20-50ms intervals
   - Motor control loop runs at real-time on MCU
   - Main CPU handles navigation at lower frequency (~10-20 Hz)

3. **Power Budget**:
   - Drive motors: ~10-20W total
   - Vacuum blower: ~20-30W
   - Lidar motor: ~2W
   - Electronics: ~5W
   - Total: ~40-60W typical operation

4. **Sensor Integration**:
   - IMU connected to MCU (SPI/I2C)
   - Encoders connected to MCU timer inputs
   - Cliff/bumper sensors connected to MCU GPIO
   - Lidar connected to main CPU UART

5. **Safety Features**:
   - Cliff sensors prevent falls
   - Bumper sensors detect collisions
   - Watchdog timer on MCU (heartbeat timeout)
   - Battery voltage monitoring

## Bill of Materials (BOM) Reference

Based on reverse-engineered hardware:

| Category | Component | Quantity | Notes |
|----------|-----------|----------|-------|
| **Processors** |
| Main CPU | Allwinner A33 or equivalent | 1 | Or any Linux SBC with UART |
| Motor MCU | GD32F103VCT6 | 1 | Or STM32F103 |
| **Sensors** |
| Lidar | 3iRobotix Delta-2D | 1 | Or compatible 360° lidar |
| IMU | 9-axis IMU (MPU9250 or similar) | 1 | Connected to MCU |
| Wheel Encoders | Quadrature encoders | 2 | Left and right wheels |
| Cliff Sensors | IR distance sensors | 3-4 | Front and sides |
| Bumper Sensors | Microswitches | 2-4 | Collision detection |
| **Actuators** |
| Drive Motors | DC motors with encoders | 2 | Left and right |
| Vacuum Blower | BLDC motor | 1 | High power |
| Side Brush | DC motor | 1 | Low power |
| Rolling Brush | DC motor | 1 | Medium power |
| **Power** |
| Battery | 4S Li-ion pack | 1 | 14.8V, 2000-3000mAh |
| Charge Controller | CN3704 or similar | 1 | 4-cell Li-ion |
| PMIC | AXP223 or equivalent | 1 | Multi-channel |
| **Communication** |
| WiFi Module | RTL8189ETV or similar | 1 | 802.11n |

## PCB Design Considerations

1. **Dual-layer or 4-layer**: 4-layer recommended for better signal integrity
2. **Power traces**: Wide traces for motor current paths
3. **Analog isolation**: Separate analog ground for sensors
4. **EMI considerations**: Proper grounding, decoupling capacitors
5. **Motor driver placement**: Close to motors, heat dissipation
6. **Connectors**: Robust connectors for motors and sensors

## Related Documentation

- **[GD32_PROTOCOL.md](GD32_PROTOCOL.md)** - Complete GD32 communication protocol
- **[VacuumRobot Research](https://github.com/codetiger/VacuumRobot)** - Detailed hardware analysis with photos

## License

This documentation is released under Apache 2.0 license for educational and research purposes.
