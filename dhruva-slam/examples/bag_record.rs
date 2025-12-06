//! CLI tool for recording bag files from SangamIO.
//!
//! Records sensor data from a live robot connection to a bag file
//! for later playback and algorithm development.
//!
//! # Usage
//!
//! ```bash
//! # Record for 60 seconds
//! cargo run --example bag_record -- --sangam 192.168.68.101:5555 --output recording.bag --duration 60
//!
//! # Record until Ctrl-C
//! cargo run --example bag_record -- --sangam 192.168.68.101:5555 --output recording.bag
//!
//! # Record with IMU calibration at start
//! cargo run --example bag_record -- --sangam 192.168.68.101:5555 --output recording.bag --imu-calibrate
//! ```

use dhruva_slam::io::bag::{BagRecorder, EncoderTicks, SensorStatusMsg};
use dhruva_slam::utils::{
    DEFAULT_LIDAR_PWM, DEFAULT_LINEAR_VEL, DEFAULT_ROTATION_ANGULAR_VEL, setup_ctrl_c_handler,
};
use dhruva_slam::{ComponentActionType, SangamClient, SensorValue, Timestamped};
use std::collections::HashMap;
use std::env;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

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
    imu_calibrate: bool,
    lidar_enable: bool,
    rotate: bool,
    forward: bool,
}

fn parse_args(args: &[String]) -> Result<Config, String> {
    let mut sangam_address = None;
    let mut output_path = None;
    let mut duration_secs = None;
    let mut imu_calibrate = false;
    let mut lidar_enable = false;
    let mut rotate = false;
    let mut forward = false;

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
            "--imu-calibrate" | "-i" => {
                imu_calibrate = true;
            }
            "--lidar" | "-l" => {
                lidar_enable = true;
            }
            "--rotate" | "-r" => {
                rotate = true;
            }
            "--forward" | "-f" => {
                forward = true;
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

    if rotate && forward {
        return Err("Cannot use --rotate and --forward together".to_string());
    }

    Ok(Config {
        sangam_address,
        output_path,
        duration_secs,
        imu_calibrate,
        lidar_enable,
        rotate,
        forward,
    })
}

fn print_usage(program: &str) {
    eprintln!(
        r#"
Usage: {} [OPTIONS]

Record sensor data from SangamIO to a bag file.

OPTIONS:
    -s, --sangam <ADDRESS>   SangamIO address (e.g., 192.168.68.101:5555)
    -o, --output <PATH>      Output bag file path
    -d, --duration <SECS>    Recording duration in seconds (default: until Ctrl-C)
    -i, --imu-calibrate      Send IMU calibration command before recording
    -l, --lidar              Enable lidar before recording
    -r, --rotate             Enable rotation during recording (20 deg/s)
    -f, --forward            Enable forward motion during recording (0.1 m/s)
    -h, --help               Show this help message

EXAMPLES:
    # Record for 60 seconds
    {} --sangam 192.168.68.101:5555 --output recording.bag --duration 60

    # Record until Ctrl-C
    {} --sangam 192.168.68.101:5555 --output recording.bag

    # Record with IMU calibration and lidar
    {} --sangam 192.168.68.101:5555 --output recording.bag --imu-calibrate --lidar -d 120
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

    // Enable lidar if requested - do this before IMU calibration
    if config.lidar_enable {
        println!("Enabling lidar at {}% PWM...", DEFAULT_LIDAR_PWM);
        let mut lidar_config = HashMap::new();
        lidar_config.insert("pwm".to_string(), SensorValue::U32(DEFAULT_LIDAR_PWM));
        client.send_component_command_with_config(
            "lidar",
            ComponentActionType::Configure,
            lidar_config,
        )?;
        std::thread::sleep(Duration::from_secs(1));
        println!("Waiting 3 seconds for lidar to spin up...");
        std::thread::sleep(Duration::from_secs(3));
        println!("Lidar enabled.");
    }

    // Send IMU calibration command if requested (after lidar is running)
    if config.imu_calibrate {
        println!("Sending IMU calibration command...");
        client.send_component_command("imu", ComponentActionType::Enable)?;
        println!("Waiting 2 seconds for IMU calibration to complete...");
        std::thread::sleep(Duration::from_secs(2));
        println!("IMU calibration complete.");
    }

    // Enable rotation if requested
    if config.rotate {
        println!(
            "Enabling drive and starting rotation at {:.0} deg/s...",
            DEFAULT_ROTATION_ANGULAR_VEL.to_degrees()
        );
        // Enable drive first
        client.send_component_command("drive", ComponentActionType::Enable)?;
        std::thread::sleep(Duration::from_millis(100));
        // Set angular velocity
        let mut velocity_config = HashMap::new();
        velocity_config.insert("linear".to_string(), SensorValue::F32(0.0));
        velocity_config.insert(
            "angular".to_string(),
            SensorValue::F32(DEFAULT_ROTATION_ANGULAR_VEL),
        );
        client.send_component_command_with_config(
            "drive",
            ComponentActionType::Configure,
            velocity_config,
        )?;
        println!("Rotation enabled.");
    }

    // Enable forward motion if requested
    if config.forward {
        println!(
            "Enabling drive and starting forward motion at {:.2} m/s...",
            DEFAULT_LINEAR_VEL
        );
        // Enable drive first
        client.send_component_command("drive", ComponentActionType::Enable)?;
        std::thread::sleep(Duration::from_millis(100));
        // Set linear velocity
        let mut velocity_config = HashMap::new();
        velocity_config.insert("linear".to_string(), SensorValue::F32(DEFAULT_LINEAR_VEL));
        velocity_config.insert("angular".to_string(), SensorValue::F32(0.0));
        client.send_component_command_with_config(
            "drive",
            ComponentActionType::Configure,
            velocity_config,
        )?;
        println!("Forward motion enabled.");
    }

    // Flush TCP buffer - discard any accumulated messages during setup
    println!("Flushing TCP buffer...");
    let mut flushed = 0;
    loop {
        match client.recv_timeout(Duration::from_millis(50)) {
            Ok(Some(_)) => flushed += 1,
            Ok(None) => break, // Timeout with no data - buffer is empty
            Err(_) => break,   // Error - stop flushing
        }
    }
    println!("Flushed {} buffered messages.", flushed);

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

                    recorder.record_sensor_status(&SensorStatusMsg {
                        timestamp_us,
                        encoder: EncoderTicks::new(left, right),
                        gyro_raw: gyro,
                        accel_raw: accel,
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

    // Stop rotation if it was enabled
    if config.rotate {
        println!("\n\nStopping rotation...");
        client.send_component_command("drive", ComponentActionType::Disable)?;
        std::thread::sleep(Duration::from_millis(500));
        println!("Rotation stopped.");
    }

    // Stop forward motion if it was enabled
    if config.forward {
        println!("\n\nStopping forward motion...");
        client.send_component_command("drive", ComponentActionType::Disable)?;
        std::thread::sleep(Duration::from_millis(500));
        println!("Forward motion stopped.");
    }

    // Stop lidar if it was enabled
    if config.lidar_enable {
        println!("\nStopping lidar...");
        client.send_component_command("lidar", ComponentActionType::Disable)?;
        std::thread::sleep(Duration::from_secs(1));
        println!("Lidar stopped.");
    }

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
