//! Mock device driver for hardware-free robot simulation
//!
//! This module provides a complete simulation of CRL-200S robot hardware,
//! enabling SLAM and navigation algorithm development without physical robots.
//!
//! # Overview
//!
//! The mock driver simulates all sensors and actuators of the CRL-200S:
//!
//! | Component | Simulation Method |
//! |-----------|-------------------|
//! | Lidar (Delta-2D) | Ray-casting against occupancy grid |
//! | Wheel encoders | Differential drive kinematics + slip noise |
//! | IMU (gyro/accel) | Physics-based with configurable noise |
//! | Bumpers | Collision detection against map |
//! | Cliff sensors | Cliff mask lookup or boundary detection |
//! | Battery/buttons | Fixed configurable states |
//!
//! # Map Format
//!
//! Uses ROS-standard PGM + YAML format for maximum compatibility:
//!
//! ```yaml
//! # maps/example.yaml
//! image: example.pgm        # Grayscale occupancy grid
//! resolution: 0.02          # meters per pixel
//! origin: [-5.0, -5.0, 0.0] # [x, y, yaw] of bottom-left pixel
//! occupied_thresh: 0.65     # Darker = occupied
//! cliff_mask: cliffs.pgm    # Optional cliff layer
//! ```
//!
//! PGM pixel values:
//! - **White (255)**: Free space
//! - **Black (0)**: Occupied/wall
//! - **Gray (205)**: Unknown
//!
//! # Configuration
//!
//! Enable the `mock` feature to use this driver:
//!
//! ```bash
//! cargo build --features mock
//! cargo run --features mock -- mock.toml
//! ```
//!
//! Example configuration (`mock.toml`):
//!
//! ```toml
//! [device]
//! type = "mock"
//! name = "Mock CRL-200S"
//!
//! [device.simulation]
//! map_file = "maps/example.yaml"
//! start_x = 1.5
//! start_y = 3.5
//! start_theta = 0.0
//! speed_factor = 1.0    # 2.0 = 2x speed
//! random_seed = 42      # 0 = random each run
//!
//! [network]
//! bind_address = "0.0.0.0:5555"
//! ```
//!
//! # Simulation Loop
//!
//! The simulation runs at 110Hz (matching CRL-200S hardware):
//!
//! ```text
//! Every ~9ms:
//! 1. Read velocity commands from shared state
//! 2. Update physics (pose, collision detection)
//! 3. Generate encoder ticks from wheel velocities
//! 4. Generate IMU readings (gyro, accel, tilt)
//! 5. Check bumpers and cliff sensors
//! 6. Publish sensor_status via streaming channel
//!
//! Every ~200ms (5Hz):
//! 7. Generate lidar scan via ray-casting
//! 8. Update lidar sensor group
//! ```
//!
//! # Speed Factor
//!
//! The `speed_factor` setting accelerates simulation time:
//!
//! | Factor | sensor_status | lidar | Use Case |
//! |--------|---------------|-------|----------|
//! | 1.0 | 110 Hz | 5 Hz | Real-time testing |
//! | 2.0 | 220 Hz | 10 Hz | Faster algorithm iteration |
//! | 5.0 | 550 Hz | 25 Hz | Quick integration tests |
//!
//! # Noise Models
//!
//! All sensors include configurable noise for realistic testing:
//!
//! - **Lidar**: Range stddev, angle stddev, miss rate, quality decay
//! - **Encoders**: Wheel slip (multiplicative), quantization noise
//! - **IMU**: Per-axis stddev, bias, gyro drift rate
//!
//! # Thread Model
//!
//! ```text
//! ┌─────────────────┐
//! │   Main Thread   │
//! │  (initialize)   │
//! └────────┬────────┘
//!          │ spawns
//!          ▼
//! ┌─────────────────┐     ┌─────────────────┐
//! │ Simulation Loop │────▶│ Streaming Chan  │
//! │  (mock-sim)     │     │ (sensor_status) │
//! └─────────────────┘     └─────────────────┘
//! ```
//!
//! # Module Structure
//!
//! - [`config`]: Configuration structures for all simulation parameters
//! - [`physics`]: Differential drive kinematics and collision handling
//! - [`lidar_sim`]: Ray-casting lidar simulation
//! - [`imu_sim`]: IMU data generation with noise
//! - [`encoder_sim`]: Wheel encoder simulation with slip
//! - [`sensor_sim`]: Bumper, cliff, and binary sensor simulation
//! - [`map_loader`]: PGM + YAML map loading
//! - [`noise`]: Configurable noise generator

pub mod config;
mod encoder_sim;
mod imu_sim;
mod lidar_sim;
mod map_loader;
mod noise;
mod physics;
mod sensor_sim;

use crate::config::DeviceConfig;
use crate::core::driver::{DeviceDriver, DriverInitResult};
use crate::core::types::{Command, SensorGroupData, SensorValue};
use crate::error::{Error, Result};

use config::SimulationConfig;
use encoder_sim::EncoderSimulator;
use imu_sim::ImuSimulator;
use lidar_sim::LidarSimulator;
use map_loader::SimulationMap;
use noise::NoiseGenerator;
use physics::PhysicsState;
use sensor_sim::{ActuatorState, BumperSimulator, CliffSimulator};

use crossbeam_channel::Sender;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// Atomic f32 wrapper using AtomicU32
struct AtomicF32(AtomicU32);

impl AtomicF32 {
    fn new(val: f32) -> Self {
        Self(AtomicU32::new(val.to_bits()))
    }

    fn load(&self, order: Ordering) -> f32 {
        f32::from_bits(self.0.load(order))
    }

    fn store(&self, val: f32, order: Ordering) {
        self.0.store(val.to_bits(), order);
    }
}

/// Component state for tracking enable/disable commands
/// Mirrors CRL-200S behavior where lidar and motors must be explicitly enabled
pub struct ComponentState {
    /// Whether lidar motor is enabled (must be enabled to get scan data)
    pub lidar_enabled: AtomicBool,
    /// Whether wheel motors are enabled (must be enabled for drive commands to work)
    pub wheel_motor_enabled: AtomicBool,
}

impl ComponentState {
    fn new() -> Self {
        Self {
            lidar_enabled: AtomicBool::new(false),
            wheel_motor_enabled: AtomicBool::new(false),
        }
    }
}

/// Shared state for thread communication
struct SharedState {
    linear_vel: AtomicF32,
    angular_vel: AtomicF32,
    shutdown: AtomicBool,
    components: ComponentState,
}

impl SharedState {
    fn new() -> Self {
        Self {
            linear_vel: AtomicF32::new(0.0),
            angular_vel: AtomicF32::new(0.0),
            shutdown: AtomicBool::new(false),
            components: ComponentState::new(),
        }
    }
}

/// Mock device driver for simulating CRL-200S hardware
pub struct MockDriver {
    config: SimulationConfig,
    shared_state: Arc<SharedState>,
    simulation_handle: Option<JoinHandle<()>>,
    actuator_state: Arc<Mutex<ActuatorState>>,
}

impl MockDriver {
    /// Create a new mock driver from device configuration
    pub fn new(device_config: DeviceConfig) -> Result<Self> {
        let sim_config = device_config.simulation.ok_or_else(|| {
            Error::Config("Mock device requires simulation configuration".to_string())
        })?;

        Ok(Self {
            config: sim_config,
            shared_state: Arc::new(SharedState::new()),
            simulation_handle: None,
            actuator_state: Arc::new(Mutex::new(ActuatorState::new())),
        })
    }

    fn shutdown_all(&mut self) {
        self.shared_state.shutdown.store(true, Ordering::Relaxed);

        if let Some(handle) = self.simulation_handle.take() {
            let _ = handle.join();
        }
    }
}

impl DeviceDriver for MockDriver {
    fn initialize(&mut self) -> Result<DriverInitResult> {
        log::info!("Initializing mock device driver");

        // Load map
        let map = SimulationMap::load(&self.config.map_file)?;
        log::info!(
            "Loaded map: {}x{} pixels, resolution: {} m/px",
            map.width(),
            map.height(),
            map.resolution()
        );

        // Initialize physics state
        let physics = PhysicsState::new(
            self.config.start_x,
            self.config.start_y,
            self.config.start_theta,
            &self.config.robot,
        );

        // Create noise generator
        let noise = NoiseGenerator::new(self.config.random_seed);

        // Create sensor groups
        let sensor_status = Arc::new(Mutex::new(SensorGroupData::new("sensor_status")));
        let lidar_data = Arc::new(Mutex::new(SensorGroupData::new("lidar")));
        let version_data = Arc::new(Mutex::new(SensorGroupData::new("device_version")));

        // Set version info
        {
            let mut ver = version_data
                .lock()
                .map_err(|e| Error::MutexPoisoned(format!("version_data lock failed: {}", e)))?;
            ver.set(
                "version_string",
                SensorValue::String("MockDevice v1.0".to_string()),
            );
            ver.set("version_code", SensorValue::I32(100));
            ver.touch();
        }

        // Create streaming channel for sensor_status
        let (stream_tx, stream_rx) = crate::core::types::create_stream_channel();

        // Clone references for simulation thread
        let sim_config = self.config.clone();
        let shared = Arc::clone(&self.shared_state);
        let sensor_status_clone = Arc::clone(&sensor_status);
        let lidar_data_clone = Arc::clone(&lidar_data);
        let actuator_state_clone = Arc::clone(&self.actuator_state);

        // Spawn simulation thread
        let handle = thread::Builder::new()
            .name("mock-simulation".to_string())
            .spawn(move || {
                simulation_loop(
                    sim_config,
                    shared,
                    map,
                    physics,
                    noise,
                    sensor_status_clone,
                    lidar_data_clone,
                    stream_tx,
                    actuator_state_clone,
                );
            })
            .map_err(|e| Error::Other(format!("Failed to spawn simulation thread: {}", e)))?;

        self.simulation_handle = Some(handle);

        log::info!("Mock device driver initialized");

        // Build result
        let mut sensor_data = HashMap::new();
        sensor_data.insert("sensor_status".to_string(), sensor_status);
        sensor_data.insert("lidar".to_string(), lidar_data);
        sensor_data.insert("device_version".to_string(), version_data);

        let mut stream_receivers = HashMap::new();
        stream_receivers.insert("sensor_status".to_string(), stream_rx);

        Ok(DriverInitResult {
            sensor_data,
            stream_receivers,
        })
    }

    fn send_command(&mut self, cmd: Command) -> Result<()> {
        match cmd {
            Command::ComponentControl { id, action } => {
                match id.as_str() {
                    "drive" => {
                        // Handle drive commands (matches CRL-200S behavior)
                        // - Enable: enables wheel motors (required before Configure works)
                        // - Configure: sets velocity (only if wheel motors enabled)
                        // - Disable: stops and disables wheel motors
                        match &action {
                            crate::core::types::ComponentAction::Enable { .. } => {
                                self.shared_state
                                    .components
                                    .wheel_motor_enabled
                                    .store(true, Ordering::Relaxed);
                                log::info!("Drive enabled (wheel motors on)");
                            }
                            crate::core::types::ComponentAction::Configure { config } => {
                                if !self
                                    .shared_state
                                    .components
                                    .wheel_motor_enabled
                                    .load(Ordering::Relaxed)
                                {
                                    log::warn!(
                                        "Drive command ignored: wheel motors not enabled (send Enable first)"
                                    );
                                    return Ok(());
                                }
                                if let Some(SensorValue::F32(v)) = config.get("linear") {
                                    self.shared_state.linear_vel.store(*v, Ordering::Relaxed);
                                }
                                if let Some(SensorValue::F32(w)) = config.get("angular") {
                                    self.shared_state.angular_vel.store(*w, Ordering::Relaxed);
                                }
                                log::debug!(
                                    "Drive command: linear={}, angular={}",
                                    self.shared_state.linear_vel.load(Ordering::Relaxed),
                                    self.shared_state.angular_vel.load(Ordering::Relaxed)
                                );
                            }
                            crate::core::types::ComponentAction::Disable { .. } => {
                                self.shared_state
                                    .components
                                    .wheel_motor_enabled
                                    .store(false, Ordering::Relaxed);
                                self.shared_state.linear_vel.store(0.0, Ordering::Relaxed);
                                self.shared_state.angular_vel.store(0.0, Ordering::Relaxed);
                                log::info!("Drive disabled (wheel motors off)");
                            }
                            _ => {}
                        }
                    }
                    "lidar" => {
                        // Handle lidar enable/disable
                        match &action {
                            crate::core::types::ComponentAction::Enable { .. } => {
                                self.shared_state
                                    .components
                                    .lidar_enabled
                                    .store(true, Ordering::Relaxed);
                                log::info!("Lidar enabled");
                            }
                            crate::core::types::ComponentAction::Disable { .. } => {
                                self.shared_state
                                    .components
                                    .lidar_enabled
                                    .store(false, Ordering::Relaxed);
                                log::info!("Lidar disabled");
                            }
                            crate::core::types::ComponentAction::Configure { config } => {
                                // Configure also enables lidar (like CRL-200S)
                                self.shared_state
                                    .components
                                    .lidar_enabled
                                    .store(true, Ordering::Relaxed);
                                if let Some(SensorValue::U8(pwm)) = config.get("pwm") {
                                    log::info!("Lidar enabled (PWM={}%)", pwm);
                                } else {
                                    log::info!("Lidar enabled via configure");
                                }
                            }
                            _ => {}
                        }
                    }
                    _ => {
                        // Handle other actuator commands (vacuum, brushes, etc.)
                        if let Ok(mut state) = self.actuator_state.lock() {
                            state.handle_command(
                                &id,
                                &action,
                                self.config.actuators.log_state_changes,
                            );
                        }
                    }
                }
            }
            Command::Shutdown => {
                log::info!("Shutdown command received");
                self.shutdown_all();
            }
            Command::ProtocolSync => {
                // Nothing to do for mock
            }
        }
        Ok(())
    }
}

impl Drop for MockDriver {
    fn drop(&mut self) {
        self.shutdown_all();
    }
}

/// Main simulation loop
fn simulation_loop(
    config: SimulationConfig,
    shared: Arc<SharedState>,
    map: SimulationMap,
    mut physics: PhysicsState,
    noise: NoiseGenerator,
    sensor_status: Arc<Mutex<SensorGroupData>>,
    lidar_data: Arc<Mutex<SensorGroupData>>,
    stream_tx: Sender<SensorGroupData>,
    _actuator_state: Arc<Mutex<ActuatorState>>,
) {
    // Create simulators
    let mut lidar_sim = LidarSimulator::new(&config.lidar, noise.clone());
    let mut imu_sim = ImuSimulator::new(&config.imu, noise.clone());
    let mut encoder_sim =
        EncoderSimulator::new(&config.encoder, config.robot.ticks_per_meter, noise.clone());
    let bumper_sim = BumperSimulator::new(&config.bumpers);
    let cliff_sim = CliffSimulator::new(&config.cliffs);

    // Timing
    let base_interval_us = (1_000_000.0 / 110.0) as u64; // ~9090us for 110Hz
    let scaled_interval =
        Duration::from_micros((base_interval_us as f32 / config.speed_factor) as u64);
    let lidar_interval_ticks = (110.0 / config.lidar.scan_rate_hz) as u32; // ~22 ticks at 5Hz

    let mut last_time = Instant::now();
    let mut lidar_tick_counter: u32 = 0;

    log::info!(
        "Simulation loop started: speed_factor={}, interval={:?}",
        config.speed_factor,
        scaled_interval
    );

    while !shared.shutdown.load(Ordering::Relaxed) {
        let loop_start = Instant::now();

        // Calculate delta time
        let now = Instant::now();
        let wall_dt = now.duration_since(last_time).as_secs_f32();
        let sim_dt = wall_dt * config.speed_factor;
        last_time = now;

        // Read velocity commands
        let linear_vel = shared.linear_vel.load(Ordering::Relaxed);
        let angular_vel = shared.angular_vel.load(Ordering::Relaxed);

        // Update physics
        let collision = physics.update(sim_dt, linear_vel, angular_vel, &map, &config.robot);
        if collision && config.log_level != "minimal" {
            log::info!("Collision at ({:.3}, {:.3})", physics.x(), physics.y());
        }

        // Generate encoder data
        let (left_vel, right_vel) =
            physics.wheel_velocities(linear_vel, angular_vel, config.robot.wheel_base);
        let (wheel_left, wheel_right) = encoder_sim.update(left_vel, right_vel, sim_dt);

        // Generate IMU data
        let imu_reading = imu_sim.generate(angular_vel, linear_vel, sim_dt);

        // Check bumpers and cliffs
        let (bumper_left, bumper_right) = bumper_sim.check(
            &map,
            physics.x(),
            physics.y(),
            physics.theta(),
            config.robot.robot_radius,
        );
        let cliff_states = cliff_sim.check(&map, physics.x(), physics.y(), physics.theta());

        // Build sensor_status
        if let Ok(mut status) = sensor_status.lock() {
            status.set("wheel_left", SensorValue::U16(wheel_left));
            status.set("wheel_right", SensorValue::U16(wheel_right));

            status.set("gyro_x", SensorValue::I16(imu_reading.gyro[0]));
            status.set("gyro_y", SensorValue::I16(imu_reading.gyro[1]));
            status.set("gyro_z", SensorValue::I16(imu_reading.gyro[2]));
            status.set("accel_x", SensorValue::I16(imu_reading.accel[0]));
            status.set("accel_y", SensorValue::I16(imu_reading.accel[1]));
            status.set("accel_z", SensorValue::I16(imu_reading.accel[2]));
            status.set("tilt_x", SensorValue::I16(imu_reading.tilt[0]));
            status.set("tilt_y", SensorValue::I16(imu_reading.tilt[1]));
            status.set("tilt_z", SensorValue::I16(imu_reading.tilt[2]));

            status.set("bumper_left", SensorValue::Bool(bumper_left));
            status.set("bumper_right", SensorValue::Bool(bumper_right));

            // Cliff sensors
            status.set(
                "cliff_left_side",
                SensorValue::Bool(*cliff_states.get("left_side").unwrap_or(&false)),
            );
            status.set(
                "cliff_left_front",
                SensorValue::Bool(*cliff_states.get("left_front").unwrap_or(&false)),
            );
            status.set(
                "cliff_right_front",
                SensorValue::Bool(*cliff_states.get("right_front").unwrap_or(&false)),
            );
            status.set(
                "cliff_right_side",
                SensorValue::Bool(*cliff_states.get("right_side").unwrap_or(&false)),
            );

            // Battery (fixed)
            status.set(
                "battery_voltage",
                SensorValue::F32(config.sensors.battery.voltage),
            );
            status.set(
                "battery_level",
                SensorValue::U8(config.sensors.battery.level),
            );
            status.set(
                "is_charging",
                SensorValue::Bool(config.sensors.battery.is_charging),
            );

            // Other sensors (fixed)
            status.set(
                "dustbox_attached",
                SensorValue::Bool(config.sensors.dustbox_attached),
            );
            status.set(
                "is_dock_connected",
                SensorValue::Bool(config.sensors.dock_connected),
            );
            status.set(
                "start_button",
                SensorValue::U16(config.sensors.start_button),
            );
            status.set("dock_button", SensorValue::U16(config.sensors.dock_button));

            status.touch();

            // Send to streaming channel
            let _ = stream_tx.try_send(status.clone());
        }

        // Generate lidar scan periodically (only if lidar is enabled)
        lidar_tick_counter += 1;
        if lidar_tick_counter >= lidar_interval_ticks {
            lidar_tick_counter = 0;

            // Only generate scan if lidar is enabled
            if shared.components.lidar_enabled.load(Ordering::Relaxed) {
                let scan = lidar_sim.generate_scan(&map, physics.x(), physics.y(), physics.theta());

                if let Ok(mut lidar) = lidar_data.lock() {
                    log::info!(
                        "Generated lidar scan: {} points at ({:.2}, {:.2}, {:.2}°)",
                        scan.len(),
                        physics.x(),
                        physics.y(),
                        physics.theta().to_degrees()
                    );
                    lidar.set("scan", SensorValue::PointCloud2D(scan));
                    lidar.touch();
                }
            }
        }

        // Sleep for remaining interval
        let elapsed = loop_start.elapsed();
        if elapsed < scaled_interval {
            thread::sleep(scaled_interval - elapsed);
        }
    }

    log::info!("Simulation loop terminated");
}
