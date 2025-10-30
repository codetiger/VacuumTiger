# Changelog

All notable changes to the VacuumTiger project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

#### SangamIO Library (v0.1.0) - 2025-10-30

**Core Infrastructure:**
- Monorepo workspace structure with Cargo workspace
- `sangam-io` library crate with `no_std` support
- Feature flags for modular compilation (`std`, `gd32`, `lidar`, `serde`)
- Comprehensive error handling with custom `Error` enum
- Common utilities module with XOR checksum calculation

**GD32F103 Protocol Implementation (✅ Complete):**
- Full command set implementation (40+ commands)
  - System control (sleep, wake, initialize, reset, restart)
  - Motor control (speed, control type, heartbeat)
  - Actuators (blower, brushes)
  - Sensors (cliff IR)
  - Peripherals (LED, lidar power, charger)
  - IMU calibration commands
- Packet encoding/decoding with sync byte detection
- CRC validation (XOR checksum)
- Status packet parsing (96-byte sensor data)
  - Battery voltage, current, charge level
  - Wheel encoder counts
  - IMU data (accelerometer, gyroscope)
  - Button states and error codes
- Serial communication interface (`Gd32Serial`)
  - Automatic initialization with timeout
  - Heartbeat loop support
  - Non-blocking status packet reading
  - Firmware version query

**Delta-2D Lidar Driver (✅ Complete):**
- Full protocol implementation for 3iRobotix Delta-2D lidar
- Packet parsing with measurement data extraction
- Serial communication with automatic scanning
- Data types: `LidarPoint`, `LidarScan`
- Integration with GD32 for motor power control

**Examples:**
- `test_lidar_scenario.rs` - Complete GD32 + lidar integration test
  - Demonstrates initialization, scanning, and clean shutdown
  - Automatic heartbeat management
  - Lidar power control and data collection

**Testing:**
- 9 unit tests covering:
  - Checksum calculation
  - Command encoding
  - Packet encoding/decoding
  - Transport layer operations
  - Mock implementations
- All tests passing ✅
- Hardware verification on Allwinner A33 platform

**Documentation:**
- Complete GD32 protocol specification (verified)
- Delta-2D lidar protocol documentation
- Hardware reference with component specs and pinouts
- Library API documentation with examples
- Comprehensive guides:
  - GUIDE.md - Deployment and development workflow
  - REFERENCE.md - Architecture and technical reference
- Safety warnings and troubleshooting guides

### Changed
- N/A (initial release)

### Deprecated
- N/A (initial release)

### Removed
- N/A (initial release)

### Fixed
- N/A (initial release)

### Security
- N/A (initial release)

---

## Project Roadmap

### Phase 1: Core Communication ✅ COMPLETE
- [x] Project structure and monorepo setup
- [x] GD32 protocol implementation
- [x] Delta-2D lidar protocol implementation
- [x] Complete example with hardware integration
- [x] Hardware testing and validation on Allwinner A33
- [x] Comprehensive documentation (GUIDE.md, REFERENCE.md)

### Phase 2: Advanced Features (Planned)
- [ ] Additional motor controllers (STM32, ESP32)
- [ ] More lidar models (RPLIDAR, YDLIDAR)
- [ ] IMU driver trait and implementation
- [ ] Battery driver trait and monitoring
- [ ] Advanced motor control (PID, trajectories)
- [ ] Error recovery and fault handling
- [ ] High-level Robot API layer

### Phase 3: Firmware Development (Planned)
- [ ] Custom GD32 firmware
- [ ] Real-time motor control loops
- [ ] Sensor fusion algorithms
- [ ] Power management

### Phase 4: Hardware Design (Planned)
- [ ] Custom PCB design
- [ ] BOM optimization
- [ ] Assembly documentation
- [ ] Test fixtures

---

**Legend:**
- ✅ Complete
- 🚧 In Progress
- 📋 Planned
- ⚠️ Needs Testing
- 🔍 Under Investigation
