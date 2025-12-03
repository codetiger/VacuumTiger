//! CLI tool for inspecting bag files.
//!
//! Displays metadata and statistics about recorded bag files.
//!
//! # Usage
//!
//! ```bash
//! bag_info recording.bag
//! bag_info --verbose recording.bag
//! ```

use std::env;

use dhruva_slam::BagPlayer;

fn main() {
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
    bag_path: String,
    verbose: bool,
    count_messages: bool,
}

fn parse_args(args: &[String]) -> Result<Config, String> {
    let mut bag_path = None;
    let mut verbose = false;
    let mut count_messages = false;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--verbose" | "-v" => {
                verbose = true;
            }
            "--count" | "-c" => {
                count_messages = true;
            }
            "--help" | "-h" => {
                return Err("Help requested".to_string());
            }
            arg if !arg.starts_with('-') => {
                if bag_path.is_some() {
                    return Err("Multiple bag files specified".to_string());
                }
                bag_path = Some(arg.to_string());
            }
            _ => {
                return Err(format!("Unknown argument: {}", args[i]));
            }
        }
        i += 1;
    }

    let bag_path = bag_path.ok_or("Missing bag file argument")?;

    Ok(Config {
        bag_path,
        verbose,
        count_messages,
    })
}

fn print_usage(program: &str) {
    eprintln!(
        r#"
Usage: {} [OPTIONS] <BAG_FILE>

Display information about a bag file.

OPTIONS:
    -v, --verbose   Show detailed message breakdown
    -c, --count     Count actual messages (reads entire file)
    -h, --help      Show this help message

EXAMPLES:
    {} recording.bag
    {} --verbose recording.bag
"#,
        program, program, program
    );
}

fn run(config: Config) -> Result<(), Box<dyn std::error::Error>> {
    let mut player = BagPlayer::open(&config.bag_path)?;

    // Extract header values before using player mutably
    let version = player.header().version;
    let flags = player.header().flags;
    let start_time_us = player.header().start_time_us;
    let end_time_us = player.header().end_time_us;
    let duration_secs = player.header().duration_secs();
    let message_count = player.header().message_count;

    println!("Bag File Information");
    println!("====================");
    println!("File: {}", config.bag_path);
    println!();

    println!("Header Information:");
    println!("  Format version: {}", version);
    println!("  Flags: 0x{:04x}", flags);
    println!(
        "  Start time: {} us ({:.3} s)",
        start_time_us,
        start_time_us as f64 / 1_000_000.0
    );
    println!(
        "  End time: {} us ({:.3} s)",
        end_time_us,
        end_time_us as f64 / 1_000_000.0
    );
    println!("  Duration: {:.3} seconds", duration_secs);
    println!("  Message count (header): {}", message_count);
    println!();

    // Count actual messages if requested or verbose
    if config.count_messages || config.verbose {
        println!("Scanning messages...");
        player.rewind()?;

        let mut sensor_count = 0u64;
        let mut lidar_count = 0u64;
        let mut odometry_count = 0u64;
        let mut total_count = 0u64;

        let mut first_timestamp: Option<u64> = None;
        let mut last_timestamp = 0u64;

        while let Some(msg) = player.next_immediate()? {
            total_count += 1;
            let ts = msg.timestamp_us();

            if first_timestamp.is_none() {
                first_timestamp = Some(ts);
            }
            last_timestamp = ts;

            if msg.is_sensor_status() {
                sensor_count += 1;
            } else if msg.is_lidar() {
                lidar_count += 1;
            } else if msg.is_odometry() {
                odometry_count += 1;
            }
        }

        let actual_duration = last_timestamp.saturating_sub(first_timestamp.unwrap_or(0));

        println!();
        println!("Message Statistics:");
        println!("  Total messages: {}", total_count);
        println!(
            "  Sensor status: {} ({:.1}%)",
            sensor_count,
            100.0 * sensor_count as f64 / total_count.max(1) as f64
        );
        println!(
            "  LiDAR scans: {} ({:.1}%)",
            lidar_count,
            100.0 * lidar_count as f64 / total_count.max(1) as f64
        );
        println!(
            "  Odometry: {} ({:.1}%)",
            odometry_count,
            100.0 * odometry_count as f64 / total_count.max(1) as f64
        );
        println!();

        let duration_secs = actual_duration as f64 / 1_000_000.0;
        if duration_secs > 0.0 {
            println!("Rates:");
            println!("  Overall: {:.1} Hz", total_count as f64 / duration_secs);
            if sensor_count > 0 {
                println!(
                    "  Sensor status: {:.1} Hz",
                    sensor_count as f64 / duration_secs
                );
            }
            if lidar_count > 0 {
                println!("  LiDAR: {:.1} Hz", lidar_count as f64 / duration_secs);
            }
            if odometry_count > 0 {
                println!(
                    "  Odometry: {:.1} Hz",
                    odometry_count as f64 / duration_secs
                );
            }
        }

        if config.verbose && sensor_count > 0 {
            // Show first few sensor messages
            println!();
            println!("First 5 sensor messages:");
            player.rewind()?;
            let mut shown = 0;
            while let Some(msg) = player.next_immediate()? {
                if let Some(status) = msg.as_sensor_status() {
                    println!(
                        "  [{:>12} us] encoders: ({:>5}, {:>5}), gyro: ({:>6}, {:>6}, {:>6})",
                        status.timestamp_us,
                        status.encoder.left,
                        status.encoder.right,
                        status.gyro_raw[0],
                        status.gyro_raw[1],
                        status.gyro_raw[2]
                    );
                    shown += 1;
                    if shown >= 5 {
                        break;
                    }
                }
            }
        }

        if config.verbose && lidar_count > 0 {
            // Show first lidar scan info
            println!();
            println!("First LiDAR scan:");
            player.rewind()?;
            while let Some(msg) = player.next_immediate()? {
                if let Some(lidar) = msg.as_lidar() {
                    println!("  Timestamp: {} us", lidar.timestamp_us);
                    println!("  Points: {}", lidar.data.len());
                    if !lidar.data.is_empty() {
                        let (min_range, max_range) = lidar
                            .data
                            .iter()
                            .map(|(_, r, _)| *r)
                            .fold((f32::MAX, f32::MIN), |(min, max), r| {
                                (min.min(r), max.max(r))
                            });
                        println!("  Range: {:.3} - {:.3} m", min_range, max_range);
                    }
                    break;
                }
            }
        }
    }

    // File size info
    let metadata = std::fs::metadata(&config.bag_path)?;
    let file_size = metadata.len();
    println!();
    println!("File Size:");
    println!(
        "  {} bytes ({:.2} KB, {:.2} MB)",
        file_size,
        file_size as f64 / 1024.0,
        file_size as f64 / 1_048_576.0
    );

    if message_count > 0 {
        println!(
            "  Average per message: {:.1} bytes",
            file_size as f64 / message_count as f64
        );
    }

    Ok(())
}
