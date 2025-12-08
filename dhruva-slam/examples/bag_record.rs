//! CLI tool for recording bag files from SangamIO.
//!
//! Records sensor data from a live robot connection to a bag file
//! for later playback and algorithm development.
//!
//! # Initialization Sequence
//!
//! The recorder performs the following initialization before recording:
//! 1. Enable lidar and IMU
//! 2. Wait 15 seconds for sensors to stabilize
//! 3. Wait 3 seconds for IMU calibration (gyro bias calculation)
//! 4. Start recording with calibrated sensors
//!
//! # Usage
//!
//! ```bash
//! # Record stationary for 60 seconds
//! cargo run --example bag_record -- --sangam 192.168.68.101:5555 --output recording.bag --duration 60
//!
//! # Record with forward motion at 0.1 m/s
//! cargo run --example bag_record -- -s 192.168.68.101:5555 -o forward.bag --linear 0.1
//!
//! # Record rotation at 20 deg/s
//! cargo run --example bag_record -- -s 192.168.68.101:5555 -o rotate.bag --angular 0.35
//! ```

use dhruva_slam::io::bag::{BagRecorder, EncoderTicks, SensorStatusMsg};
use dhruva_slam::utils::setup_ctrl_c_handler;
use dhruva_slam::{ComponentActionType, SangamClient, SensorValue, Timestamped};
use std::collections::HashMap;
use std::env;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

/// IMU bias calibrator that collects samples and computes average bias.
struct ImuBiasCalibrator {
    samples: Vec<[i16; 3]>,
}

impl ImuBiasCalibrator {
    fn new() -> Self {
        Self {
            samples: Vec::with_capacity(330), // ~3 seconds at 110Hz
        }
    }

    fn add_sample(&mut self, gyro_raw: [i16; 3]) {
        self.samples.push(gyro_raw);
    }

    fn sample_count(&self) -> usize {
        self.samples.len()
    }

    fn compute_bias(&self) -> [f32; 3] {
        if self.samples.is_empty() {
            return [0.0, 0.0, 0.0];
        }

        let n = self.samples.len() as f64;
        let sum: (f64, f64, f64) = self.samples.iter().fold((0.0, 0.0, 0.0), |acc, s| {
            (
                acc.0 + s[0] as f64,
                acc.1 + s[1] as f64,
                acc.2 + s[2] as f64,
            )
        });

        [(sum.0 / n) as f32, (sum.1 / n) as f32, (sum.2 / n) as f32]
    }
}

fn main() {
    env_logger::init();

    let args: Vec<String> = env::args().collect();
    let config = match parse_args(&args) {
        Ok(config) => config,
        Err(e) => {
            eprintln!("Error: {}", e);
            print_usage(&args[0]);
            std::process::exit(1);
        }
    };

    if let Err(e) = run(config) {
        eprintln!("Error: {}", e);
        std::process::exit(1);
    }
}

struct Config {
    sangam_address: String,
    output_path: String,
    duration_secs: Option<u64>,
    /// Linear velocity in m/s (None = stationary)
    linear_velocity: Option<f32>,
    /// Angular velocity in rad/s (None = no rotation)
    angular_velocity: Option<f32>,
}

fn parse_args(args: &[String]) -> Result<Config, String> {
    let mut sangam_address = None;
    let mut output_path = None;
    let mut duration_secs = None;
    let mut linear_velocity = None;
    let mut angular_velocity = None;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--sangam" | "-s" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --sangam".to_string());
                }
                sangam_address = Some(args[i].clone());
            }
            "--output" | "-o" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --output".to_string());
                }
                output_path = Some(args[i].clone());
            }
            "--duration" | "-d" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --duration".to_string());
                }
                duration_secs = Some(
                    args[i]
                        .parse()
                        .map_err(|_| format!("Invalid duration: {}", args[i]))?,
                );
            }
            "--linear" | "-l" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --linear".to_string());
                }
                linear_velocity = Some(
                    args[i]
                        .parse()
                        .map_err(|_| format!("Invalid linear velocity: {}", args[i]))?,
                );
            }
            "--angular" | "-a" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --angular".to_string());
                }
                angular_velocity = Some(
                    args[i]
                        .parse()
                        .map_err(|_| format!("Invalid angular velocity: {}", args[i]))?,
                );
            }
            "--help" | "-h" => {
                return Err("Help requested".to_string());
            }
            _ => {
                return Err(format!("Unknown argument: {}", args[i]));
            }
        }
        i += 1;
    }

    let sangam_address = sangam_address.ok_or("Missing --sangam argument")?;
    let output_path = output_path.ok_or("Missing --output argument")?;

    Ok(Config {
        sangam_address,
        output_path,
        duration_secs,
        linear_velocity,
        angular_velocity,
    })
}

fn print_usage(program: &str) {
    eprintln!(
        r#"
Usage: {} [OPTIONS]

Record sensor data from SangamIO to a bag file.

INITIALIZATION:
    The recorder always performs the following initialization:
    1. Enable lidar and IMU
    2. Wait 15 seconds for sensors to stabilize
    3. Wait 3 seconds for IMU calibration (gyro bias calculation)
    4. Start recording with calibrated sensors

OPTIONS:
    -s, --sangam <ADDRESS>   SangamIO address (e.g., 192.168.68.101:5555)
    -o, --output <PATH>      Output bag file path
    -d, --duration <SECS>    Recording duration in seconds (default: until Ctrl-C)
    -l, --linear <M/S>       Linear velocity in m/s (e.g., 0.1 for forward)
    -a, --angular <RAD/S>    Angular velocity in rad/s (e.g., 0.35 for ~20 deg/s)
    -h, --help               Show this help message

VELOCITY:
    If --linear or --angular is specified, the drive will be enabled before recording.
    Both can be combined for arc motion.

EXAMPLES:
    # Record stationary for 60 seconds
    {} --sangam 192.168.68.101:5555 --output recording.bag --duration 60

    # Record with forward motion at 0.1 m/s
    {} -s 192.168.68.101:5555 -o forward.bag -l 0.1 -d 30

    # Record rotation at ~20 deg/s (0.35 rad/s)
    {} -s 192.168.68.101:5555 -o rotate.bag -a 0.35 -d 60
"#,
        program, program, program, program
    );
}

fn run(config: Config) -> Result<(), Box<dyn std::error::Error>> {
    // Set up Ctrl-C handler
    let running = setup_ctrl_c_handler()?;

    println!("Connecting to SangamIO at {}...", config.sangam_address);
    let mut client = SangamClient::connect(&config.sangam_address)?;
    println!("Connected! (wire format: Protobuf)");

    // ===== INITIALIZATION PHASE =====
    println!("\n=== Initialization Phase ===");

    // Step 1: Enable lidar and IMU
    println!("Enabling lidar...");
    client.send_component_command("lidar", ComponentActionType::Enable)?;
    println!("Enabling IMU...");
    client.send_component_command("imu", ComponentActionType::Enable)?;

    // Step 2: Wait for sensors to stabilize (15 seconds)
    // We must consume messages during this time to prevent TCP buffer overflow
    println!("Waiting for sensors to stabilize (15 seconds)...");
    let stabilization_start = Instant::now();
    let stabilization_duration = Duration::from_secs(15);
    let mut last_progress = Instant::now();
    let mut discarded_count = 0u64;

    while stabilization_start.elapsed() < stabilization_duration && running.load(Ordering::SeqCst) {
        // Consume messages to prevent buffer overflow
        match client.recv_timeout(Duration::from_millis(100)) {
            Ok(Some(_)) => discarded_count += 1,
            Ok(None) => {}
            Err(_) => {}
        }

        // Progress indicator every second
        if last_progress.elapsed() >= Duration::from_secs(1) {
            let remaining = 15 - stabilization_start.elapsed().as_secs();
            print!("\r  {} seconds remaining...  ", remaining);
            std::io::Write::flush(&mut std::io::stdout())?;
            last_progress = Instant::now();
        }
    }
    println!(
        "\r  Sensors stabilized. (discarded {} msgs)    ",
        discarded_count
    );

    // Flush any buffered messages before calibration
    let mut flushed = 0;
    loop {
        match client.recv_timeout(Duration::from_millis(50)) {
            Ok(Some(_)) => flushed += 1,
            Ok(None) => break,
            Err(_) => break,
        }
    }
    if flushed > 0 {
        println!("Flushed {} buffered messages.", flushed);
    }

    // Step 3: Wait 3 seconds while collecting IMU samples for bias calibration
    println!("Calibrating IMU (3 seconds - keep robot stationary)...");
    let mut bias_calibrator = ImuBiasCalibrator::new();
    let calibration_start = Instant::now();
    let calibration_duration = Duration::from_secs(3);
    let mut last_progress = Instant::now();

    while calibration_start.elapsed() < calibration_duration && running.load(Ordering::SeqCst) {
        match client.recv_timeout(Duration::from_millis(100)) {
            Ok(Some(msg)) => {
                if let Some(gyro) = msg.gyro_raw() {
                    bias_calibrator.add_sample(gyro);
                }
            }
            Ok(None) => {}
            Err(e) => {
                eprintln!("Error during calibration: {}", e);
                break;
            }
        }

        // Progress indicator every 500ms
        if last_progress.elapsed() >= Duration::from_millis(500) {
            let elapsed = calibration_start.elapsed().as_secs_f32();
            print!(
                "\r  Collected {} samples ({:.1}s / 3.0s)...",
                bias_calibrator.sample_count(),
                elapsed
            );
            std::io::Write::flush(&mut std::io::stdout())?;
            last_progress = Instant::now();
        }
    }
    println!();

    // Step 4: Calculate and display IMU bias
    let gyro_bias = bias_calibrator.compute_bias();
    println!(
        "IMU Calibration complete: {} samples",
        bias_calibrator.sample_count()
    );
    println!(
        "  Gyro bias: [{:.2}, {:.2}, {:.2}] (raw units, 0.1 deg/s)",
        gyro_bias[0], gyro_bias[1], gyro_bias[2]
    );

    // Check if calibration was interrupted
    if !running.load(Ordering::SeqCst) {
        println!("\nCalibration interrupted. Stopping lidar...");
        client.send_component_command("lidar", ComponentActionType::Disable)?;
        return Ok(());
    }

    // ===== MOTION SETUP (if requested) =====
    let motion_enabled = config.linear_velocity.is_some() || config.angular_velocity.is_some();
    if motion_enabled {
        let linear = config.linear_velocity.unwrap_or(0.0);
        let angular = config.angular_velocity.unwrap_or(0.0);
        println!(
            "\nEnabling drive: linear={:.2} m/s, angular={:.2} rad/s ({:.1} deg/s)...",
            linear,
            angular,
            angular.to_degrees()
        );
        client.send_component_command("drive", ComponentActionType::Enable)?;
        std::thread::sleep(Duration::from_millis(100));
        let mut velocity_config = HashMap::new();
        velocity_config.insert("linear".to_string(), SensorValue::F32(linear));
        velocity_config.insert("angular".to_string(), SensorValue::F32(angular));
        client.send_component_command_with_config(
            "drive",
            ComponentActionType::Configure,
            velocity_config,
        )?;
    }

    // ===== RECORDING PHASE =====
    println!("\n=== Recording Phase ===");
    println!("Creating bag file: {}", config.output_path);
    let mut recorder = BagRecorder::create(&config.output_path)?;

    let start_time = Instant::now();
    let end_time = config
        .duration_secs
        .map(|d| start_time + Duration::from_secs(d));

    let mut sensor_count = 0u64;
    let mut lidar_count = 0u64;
    let mut last_report = Instant::now();

    println!("Recording... (Ctrl-C to stop)");
    if let Some(duration) = config.duration_secs {
        println!("Will record for {} seconds", duration);
    }

    while running.load(Ordering::SeqCst) {
        // Check duration limit
        if let Some(end) = end_time
            && Instant::now() >= end
        {
            println!("\nDuration limit reached.");
            break;
        }

        // Receive message with timeout
        match client.recv() {
            Ok(msg) => {
                let timestamp_us = msg.timestamp_us();

                // Check if it's sensor status or lidar
                if let Some((left, right)) = msg.encoder_ticks() {
                    // Extract all sensor data
                    let gyro = msg.gyro_raw().unwrap_or([0, 0, 0]);
                    let accel = msg.accel_raw().unwrap_or([0, 0, 0]);
                    let tilt = msg.tilt_raw().unwrap_or([0, 0, 0]);

                    recorder.record_sensor_status(&SensorStatusMsg {
                        timestamp_us,
                        encoder: EncoderTicks::new(left, right),
                        gyro_raw: gyro,
                        accel_raw: accel,
                        tilt_raw: tilt,
                    })?;
                    sensor_count += 1;
                }

                if let Some(lidar) = msg.as_lidar() {
                    recorder.record_lidar(&Timestamped::new(lidar.data.clone(), timestamp_us))?;
                    lidar_count += 1;
                }

                // Progress report every second
                if last_report.elapsed() >= Duration::from_secs(1) {
                    let elapsed = start_time.elapsed().as_secs_f64();
                    let rate = sensor_count as f64 / elapsed;
                    print!(
                        "\rRecorded: {} sensors ({:.1} Hz), {} lidar scans, {:.1}s elapsed",
                        sensor_count, rate, lidar_count, elapsed
                    );
                    std::io::Write::flush(&mut std::io::stdout())?;
                    last_report = Instant::now();
                }
            }
            Err(e) => {
                eprintln!("\nConnection error: {}", e);
                break;
            }
        }
    }

    // Stop motion if it was enabled
    if motion_enabled {
        println!("\n\nStopping drive...");
        client.send_component_command("drive", ComponentActionType::Disable)?;
        std::thread::sleep(Duration::from_millis(500));
    }

    // Always stop lidar (we always start it)
    println!("\nStopping lidar...");
    client.send_component_command("lidar", ComponentActionType::Disable)?;
    std::thread::sleep(Duration::from_millis(500));

    println!("\nFinalizing bag file...");
    let info = recorder.finish()?;

    println!("\nRecording complete!");
    println!("  File: {}", info.path.display());
    println!("  Duration: {:.2} seconds", info.duration_secs());
    println!("  Messages: {}", info.message_count);
    println!("    Sensor status: {}", info.sensor_count);
    println!("    LiDAR scans: {}", info.lidar_count);
    println!("  File size: {:.2} MB", info.file_size_mb());
    println!("  Average rate: {:.1} Hz", info.message_rate_hz());

    Ok(())
}
