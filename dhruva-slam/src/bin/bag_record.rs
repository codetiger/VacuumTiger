//! CLI tool for recording bag files from SangamIO.
//!
//! Records sensor data from a live robot connection to a bag file
//! for later playback and algorithm development.
//!
//! # Usage
//!
//! ```bash
//! # Record for 60 seconds
//! bag_record --sangam 192.168.68.101:5555 --output recording.bag --duration 60
//!
//! # Record until Ctrl-C
//! bag_record --sangam 192.168.68.101:5555 --output recording.bag
//! ```

use std::env;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use dhruva_slam::io::bag::{BagRecorder, EncoderTicks, SensorStatusMsg};
use dhruva_slam::{SangamClient, Timestamped, WireFormat};

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
    wire_format: WireFormat,
}

fn parse_args(args: &[String]) -> Result<Config, String> {
    let mut sangam_address = None;
    let mut output_path = None;
    let mut duration_secs = None;
    let mut wire_format = WireFormat::Json;

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
            "--format" | "-f" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --format".to_string());
                }
                wire_format = match args[i].to_lowercase().as_str() {
                    "json" => WireFormat::Json,
                    "postcard" => WireFormat::Postcard,
                    _ => {
                        return Err(format!(
                            "Invalid format: {} (use json or postcard)",
                            args[i]
                        ));
                    }
                };
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
        wire_format,
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
    -f, --format <FORMAT>    Wire format: json or postcard (default: json)
    -h, --help               Show this help message

EXAMPLES:
    # Record for 60 seconds
    {} --sangam 192.168.68.101:5555 --output recording.bag --duration 60

    # Record until Ctrl-C
    {} --sangam 192.168.68.101:5555 --output recording.bag
"#,
        program, program, program
    );
}

fn run(config: Config) -> Result<(), Box<dyn std::error::Error>> {
    // Set up Ctrl-C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    println!("Connecting to SangamIO at {}...", config.sangam_address);
    let mut client = SangamClient::connect_with_format(&config.sangam_address, config.wire_format)?;
    println!("Connected!");

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
                let timestamp_us = match &msg.payload {
                    dhruva_slam::io::sangam_client::Payload::SensorGroup {
                        timestamp_us, ..
                    } => *timestamp_us,
                };

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

    println!("\n\nFinalizing bag file...");
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
