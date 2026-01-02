# Dhruva: High-Level Application Layer Design

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **Language** | Rust | Runs on robot ARM, consistent with SangamIO/VastuSLAM |
| **Initial Mode** | Auto Clean only | Focus on core systematic coverage |
| **Room Detection** | Automatic | Morphological operations on free space |
| **Docking** | Hybrid | Lidar for approach, IR for final precision |

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Drishti (UI)                            │
└─────────────────────────────┬───────────────────────────────────┘
                              │ Status/Commands
┌─────────────────────────────▼───────────────────────────────────┐
│                         DHRUVA (on robot)                       │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ Mission Controller                                      │    │
│  │ • State machine (Idle/Clean/Dock/Charge/Error)         │    │
│  └─────────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ Coverage Planner                                        │    │
│  │ • Boustrophedon, wall-follow, room sequencing          │    │
│  └─────────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ Navigation Layer                                        │    │
│  │ • A* path planning, DWA avoidance, recovery behaviors   │    │
│  └─────────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ Sensor Fusion                                           │    │
│  │ • EKF (encoders 110Hz + lidar 5Hz + IMU), VastuSLAM    │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────┬───────────────────────────────────┘
                              │ UDP/TCP (port 5555)
┌─────────────────────────────▼───────────────────────────────────┐
│                    SangamIO Daemon                              │
└─────────────────────────────────────────────────────────────────┘
```

---

## High-Level Functionalities

### 1. State Estimation & Sensor Fusion

**Purpose**: Create accurate, high-rate pose estimates from multiple sensors

| Feature | Description | Priority |
|---------|-------------|----------|
| **Odometry Fusion** | Combine wheel encoders (110Hz) with lidar scan matching (5Hz) | Critical |
| **IMU Integration** | Use gyro for angular velocity, detect wheel slip | High |
| **EKF/UKF Filter** | Probabilistic state estimation with sensor noise models | High |
| **Timestamp Alignment** | Buffer and synchronize sensors with different rates | Critical |
| **Localization Watchdog** | Detect and recover from localization failures | High |

**Key Challenges**:
- 22:1 rate difference between odometry and lidar
- Wheel slip on carpet transitions
- Initial pose uncertainty after dock departure

---

### 2. Navigation Layer

**Purpose**: Safe, efficient movement from A to B

| Feature | Description | Priority |
|---------|-------------|----------|
| **Global Path Planning** | A*/D* Lite on occupancy grid for room-scale paths | Critical |
| **Local Obstacle Avoidance** | VFH+ or Dynamic Window Approach for reactive control | Critical |
| **Trajectory Following** | Pure pursuit or MPC for smooth path execution | High |
| **Cliff Response** | Immediate stop + backup when cliff detected | Critical |
| **Bumper Response** | Stop, backup, turn away from obstacle | Critical |
| **Narrow Passage Detection** | Identify tight spaces robot may not fit through | Medium |

**Suggested Controllers**:
```
┌────────────────────────────────────────────────────────┐
│ Navigation Stack                                       │
│                                                        │
│  Global Planner ──────► Path ──────► Local Planner    │
│  (A* on grid)           │            (DWA/VFH+)       │
│                         │                    │         │
│                         │                    ▼         │
│  Recovery Manager ◄─────┴───────► Velocity Commands   │
│  (stuck/cliff/bump)                    │              │
│                                        ▼              │
│                              SangamIO Drive API       │
└────────────────────────────────────────────────────────┘
```

---

### 3. Coverage Planning

**Purpose**: Systematic cleaning that covers entire floor area

| Feature | Description | Priority |
|---------|-------------|----------|
| **Boustrophedon (Lawn Mower)** | Parallel lines with U-turns at boundaries | Critical |
| **Room-Based Coverage** | Clean one room at a time, then move to next | High |
| **Spiral Pattern** | Outward spiral for spot cleaning mode | High |
| **Edge/Wall Following** | Hug walls and furniture edges | High |
| **Coverage Tracking** | Mark cells as swept, identify gaps | Critical |
| **Gap Filling** | Return to missed areas after main pass | Medium |

**Coverage Strategy**:
```
1. Exploration pass (build initial map)
2. Room segmentation (identify distinct rooms)
3. For each room:
   a. Edge pass (wall following)
   b. Interior pass (boustrophedon)
   c. Gap filling (revisit missed spots)
4. Return to dock
```

---

### 4. Room & Zone Management

**Purpose**: Semantic understanding of the environment

| Feature | Description | Priority |
|---------|-------------|----------|
| **Room Segmentation** | Detect room boundaries from map topology | High |
| **Room Naming/Labeling** | User-defined room names (Kitchen, Bedroom) | Medium |
| **Virtual Walls** | Software barriers that robot won't cross | High |
| **No-Go Zones** | Rectangular regions to avoid (pet bowls, cables) | High |
| **Priority Zones** | Areas to clean first or more frequently | Medium |
| **Multi-Floor Maps** | Store separate maps per floor | Low |

**Implementation Notes**:
- Room segmentation using morphological operations on free space
- Virtual walls stored as line segments in map metadata
- Zone data persisted with map (.vastu format)

---

### 5. Cleaning Modes

**Purpose**: Different behaviors for different use cases

| Mode | Description | Actuator Settings |
|------|-------------|-------------------|
| **Auto Clean** | Full house coverage, systematic | Vacuum 80%, Brushes 100% |
| **Quick Clean** | Fast pass, main areas only | Vacuum 60%, Brushes 80% |
| **Deep Clean** | Two passes, slower speed | Vacuum 100%, Brushes 100% |
| **Spot Clean** | 1m² spiral at current location | Vacuum 100%, Brushes 100% |
| **Edge Clean** | Wall-following perimeter pass | Vacuum 80%, Side brush 100% |
| **Room Clean** | Clean selected room(s) only | User configurable |
| **Quiet Mode** | Reduced suction for noise-sensitive times | Vacuum 40%, Brushes 60% |
| **Mop Mode** | Water pump enabled, slower movement | Pump ON, Speed 50% |

---

### 6. Docking & Charging

**Purpose**: Autonomous charging management

| Feature | Description | Priority |
|---------|-------------|----------|
| **Dock Detection** | Recognize dock signature in lidar scans | Critical |
| **Dock Approach** | Navigate to dock vicinity | Critical |
| **Final Docking** | Precision approach using IR alignment | Critical |
| **Charging Monitor** | Track battery state, detect full charge | High |
| **Resume After Charge** | Continue cleaning from where it stopped | High |
| **Low Battery Preemption** | Return to dock before battery dies | Critical |

**Dock Approach Sequence**:
```
1. Navigate to saved dock location (global path)
2. Detect dock signature in lidar (characteristic shape)
3. Align perpendicular to dock front
4. Slow approach with IR homing
5. Contact detection → stop
6. Verify `is_charging` status
```

---

### 7. Error Handling & Recovery

**Purpose**: Graceful handling of real-world problems

| Error Type | Detection | Recovery Action |
|------------|-----------|-----------------|
| **Stuck (no motion)** | Encoder stall + IMU confirms no movement | Wiggle, backup, turn, retry |
| **Stuck (spinning)** | Encoders moving but pose unchanged | Stop, backup, new path |
| **Cliff Triggered** | Any cliff sensor TRUE | Emergency stop, backup 10cm, turn 90° |
| **Bumper Collision** | Bumper sensor TRUE | Stop, backup 5cm, turn 30° away |
| **Wheel Lifted** | Wheel drop sensors (if available) | Stop all motors, wait for ground |
| **Battery Critical** | battery_level < 10% | Abort cleaning, emergency dock |
| **Localization Lost** | Scan match score too low | Stop, attempt relocalization |
| **Path Blocked** | No valid path to goal | Try alternate routes, report to user |
| **Dustbox Full** | dustbox_attached FALSE | Pause, notify user |

**Recovery Escalation**:
```
Level 1: Simple retry (same action)
Level 2: Local recovery (backup, turn)
Level 3: Replan path (find alternate route)
Level 4: Relocalization (scan-to-map matching)
Level 5: Return to dock (abort mission)
Level 6: Stop and wait (critical error)
```

---

### 8. Mission Orchestration

**Purpose**: High-level task management

| Feature | Description | Priority |
|---------|-------------|----------|
| **Mission State Machine** | Idle → Cleaning → Docking → Charging cycle | Critical |
| **Cleaning Schedules** | Time-based automatic cleaning | Medium |
| **Multi-Room Sequencing** | Optimal order for room cleaning | Medium |
| **Resume After Charge** | Remember progress, continue where stopped | High |
| **Pause/Resume** | User-triggered pause with state preservation | High |
| **Mission History** | Log of past cleaning sessions | Low |

**State Machine**:
```
         ┌───────────────────────────────────────────────┐
         │                                               │
         ▼                                               │
    ┌─────────┐      Start      ┌───────────┐           │
    │  IDLE   │ ───────────────►│ CLEANING  │           │
    └────┬────┘                 └─────┬─────┘           │
         │                            │                  │
         │                   Done/Low │Battery           │
         │                            ▼                  │
         │                      ┌───────────┐           │
         │                      │  DOCKING  │           │
         │                      └─────┬─────┘           │
         │                            │                  │
         │                     Docked │                  │
         │                            ▼                  │
         │                      ┌───────────┐   Charged  │
         │                      │ CHARGING  │────────────┘
         │                      └─────┬─────┘   (if resume)
         │                            │
         │                     Full   │Charge
         │◄───────────────────────────┘
         │
    ┌────▼────┐
    │  ERROR  │◄── (any state on critical error)
    └─────────┘
```

---

### 9. Communication & External Interface

**Purpose**: Integration with user interfaces and external systems

| Feature | Description | Priority |
|---------|-------------|----------|
| **Status Broadcasting** | Current state, battery, area cleaned | High |
| **Map Streaming** | Live occupancy grid updates for visualization | High |
| **Remote Commands** | Start, Stop, Pause, Dock, Go-To-Point | Critical |
| **Cleaning Reports** | Summary after each session | Medium |
| **Error Notifications** | Push alerts for stuck, dustbin full, etc. | High |
| **OTA Updates** | Firmware update mechanism | Medium |

---

## Implementation Plan

### Phase 1: Foundation (MVP)

**Goal**: Basic autonomous cleaning loop

| Step | Module | Description | Files |
|------|--------|-------------|-------|
| 1.1 | `interface/client.rs` | SangamIO UDP/TCP client | Connect, send commands, receive sensors |
| 1.2 | `fusion/sync.rs` | Sensor timestamp alignment | Buffer 110Hz encoders, align with 5Hz lidar |
| 1.3 | `fusion/odometry.rs` | Differential drive odometry | Convert encoder ticks to dx, dy, dtheta |
| 1.4 | `fusion/ekf.rs` | Extended Kalman Filter | Fuse odometry + lidar scan matching |
| 1.5 | `navigation/recovery.rs` | Cliff/bumper response | Emergency stop, backup, turn |
| 1.6 | `navigation/planner.rs` | A* path planner | Grid-based global planning |
| 1.7 | `navigation/local_planner.rs` | DWA obstacle avoidance | Reactive local control |
| 1.8 | `mission/state_machine.rs` | Mission FSM | Idle → Cleaning → Docking → Charging |

### Phase 2: Coverage & Docking

**Goal**: Systematic cleaning + autonomous charging

| Step | Module | Description |
|------|--------|-------------|
| 2.1 | `coverage/boustrophedon.rs` | Lawn mower pattern generator |
| 2.2 | `coverage/wall_follow.rs` | Edge cleaning controller |
| 2.3 | `coverage/tracker.rs` | Coverage tracking (swept cells) |
| 2.4 | `docking/detector.rs` | Lidar dock signature detection |
| 2.5 | `docking/approach.rs` | Hybrid approach controller (lidar + IR) |
| 2.6 | `docking/charging.rs` | Battery management, resume logic |

### Phase 3: Room Intelligence

**Goal**: Semantic understanding of environment

| Step | Module | Description |
|------|--------|-------------|
| 3.1 | `zones/segmentation.rs` | Automatic room detection |
| 3.2 | `zones/virtual_walls.rs` | No-go barriers |
| 3.3 | `coverage/room_sequencer.rs` | Optimal room order |

### Phase 4: Polish

**Goal**: Production readiness

| Step | Module | Description |
|------|--------|-------------|
| 4.1 | `interface/api.rs` | Status API for Drishti |
| 4.2 | `mission/history.rs` | Mission logging |
| 4.3 | `modes/` | Additional cleaning modes |

---

## Module Structure

```
dhruva/
├── Cargo.toml
├── dhruva.toml                   # Configuration
├── proto/
│   └── dhruva.proto              # Extended protobuf (status, maps)
└── src/
    ├── lib.rs
    ├── main.rs                   # Daemon entry
    │
    ├── core/
    │   ├── mod.rs
    │   ├── config.rs             # TOML config loading
    │   ├── state.rs              # RobotState, MissionState
    │   └── events.rs             # Event channel types
    │
    ├── fusion/
    │   ├── mod.rs
    │   ├── sync.rs               # Timestamp alignment
    │   ├── odometry.rs           # Wheel encoder → pose delta
    │   ├── imu.rs                # Gyro/accel integration
    │   └── ekf.rs                # Extended Kalman Filter
    │
    ├── navigation/
    │   ├── mod.rs
    │   ├── planner.rs            # A* global planner
    │   ├── local_planner.rs      # DWA reactive control
    │   ├── controller.rs         # Velocity output
    │   └── recovery.rs           # Cliff/bumper/stuck recovery
    │
    ├── coverage/
    │   ├── mod.rs
    │   ├── boustrophedon.rs      # Parallel line coverage
    │   ├── wall_follow.rs        # Edge cleaning
    │   ├── tracker.rs            # Coverage % tracking
    │   └── room_sequencer.rs     # Multi-room order
    │
    ├── zones/
    │   ├── mod.rs
    │   ├── segmentation.rs       # Morphological room detection
    │   └── virtual_walls.rs      # No-go zones
    │
    ├── docking/
    │   ├── mod.rs
    │   ├── detector.rs           # Lidar signature matching
    │   ├── approach.rs           # Hybrid lidar + IR homing
    │   └── charging.rs           # Battery management
    │
    ├── mission/
    │   ├── mod.rs
    │   ├── state_machine.rs      # Mission FSM
    │   └── history.rs            # Session logging
    │
    └── interface/
        ├── mod.rs
        ├── client.rs             # SangamIO UDP/TCP client
        ├── api.rs                # External status API
        └── telemetry.rs          # Map streaming
```

---

## Key Algorithms

### Boustrophedon Coverage
```
1. Decompose free space into cells using critical point method
2. Generate vertical sweeping lines within each cell
3. Connect lines with U-turns at boundaries
4. Track swept areas, mark remaining gaps
5. Fill gaps with targeted paths
```

### Hybrid Dock Approach
```
1. Navigate to saved dock location (A* path)
2. Switch to lidar mode: detect dock signature (concave shape)
3. Align perpendicular to dock at 0.5m distance
4. Switch to IR mode: follow IR gradient for final 0.5m
5. Detect contact (bumper or charging status)
6. Verify is_charging = true
```

### Recovery Escalation
```
Level 1: Retry (1-2 attempts)
Level 2: Backup 10cm + turn 30°
Level 3: Replan global path
Level 4: Relocalization scan
Level 5: Return to dock
Level 6: Stop + notify user
```

---

## Dependencies

```toml
[dependencies]
vastu-slam = { path = "../vastu-slam" }   # SLAM library
tokio = { version = "1", features = ["full"] }  # Async runtime
prost = "0.12"                             # Protobuf
serde = { version = "1", features = ["derive"] }
toml = "0.8"                               # Config
nalgebra = "0.32"                          # Linear algebra (EKF)
crossbeam-channel = "0.5"                  # Lock-free channels
tracing = "0.1"                            # Logging

[build-dependencies]
prost-build = "0.12"
```

---

## Critical Files Reference

| Component | Path | Purpose |
|-----------|------|---------|
| VastuSLAM API | `vastu-slam/src/lib.rs` | Map, observe, scan matching |
| SangamIO Proto | `sangam-io/proto/sangamio.proto` | Sensor/command messages |
| Sensor Types | `sangam-io/src/core/types.rs` | SensorValue enum |
| GD32 Protocol | `sangam-io/src/devices/crl200s/gd32/` | Motor control reference |
| Map Config | `vastu-slam/src/config/` | Grid resolution, matching params |

---

## Brainstorming Notes

<!-- Add your ideas and discussions here -->
