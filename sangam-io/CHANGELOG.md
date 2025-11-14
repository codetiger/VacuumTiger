# Changelog

All notable changes to SangamIO will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] - 2024-11-14

Major cleanup release focusing on simplification, performance optimization, and removing unnecessary abstractions. The codebase is now leaner and more maintainable.

### Changed
- Major codebase cleanup and simplification
- Transitioned from library to daemon-focused architecture
- Documentation overhaul to match actual implementation

### Added
- TODO comments for potential future functionality
- Comprehensive architecture documentation
- Proper TCP protocol specification

### Removed
- Transport trait abstraction (SerialTransport is now direct implementation)
- Unused dependencies: `libc`, `ctrlc`, `anyhow`
- Unused getter methods in Gd32Driver (telemetry is streamed via TCP)
- Streaming error variant (not used in current implementation)
- Old documentation files that referenced library architecture
- Unnecessary rustflags for ARM target (musl handles static linking by default)
- Duplicate [profile.release] section (workspace root handles this)

### Fixed
- TCP protocol documentation now correctly shows string topics instead of numeric IDs
- Client implementation examples updated with correct protocol format
- Lidar scan cloning performance issue (12KB per scan saved)
- TCP publisher buffer reuse (1500+ allocations/sec eliminated)
- Lidar parsing buffer reuse (per-scan heap allocation eliminated)

### Performance
- Binary size reduced by ~30% (from ~500KB to ~350KB)
- Memory usage optimized with buffer reuse
- Reduced heap allocations in hot paths (1500+ allocations/sec eliminated)
- Eliminated 12KB allocation per lidar scan through ownership transfer
- Removed unnecessary dependencies reducing compile time and binary size

## [0.1.0] - 2024-11-13

### Added
- Initial release
- GD32F103 motor controller driver
- Delta-2D lidar driver
- TCP streaming for telemetry and commands
- Dual-thread architecture for reliable heartbeat
- Lock-free queues for sensor data streaming