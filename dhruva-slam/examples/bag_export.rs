//! Export bag file data to CSV format for external processing.
//!
//! Exports lidar scans and sensor data to CSV files that can be read by
//! Python scripts (e.g., for Open3D-based ICP calibration).
//!
//! # Usage
//!
//! ```bash
//! cargo run --example bag_to_csv -- --bag bags/rotate_3min.bag --output tools/
//! ```
//!
//! # Output Files
//!
//! - `scans.csv`: Lidar scan data (timestamp_us, scan_index, angle_rad, distance_m, quality)
//! - `odom.csv`: Sensor data (timestamp_us, encoder_left, encoder_right, gyro_z)

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use clap::Parser;

use dhruva_slam::io::bag::{BagMessage, BagPlayer};

#[derive(Parser)]
#[command(name = "bag-to-csv")]
#[command(about = "Export bag file data to CSV format for external processing")]
struct Args {
    /// Input bag file
    #[arg(short, long)]
    bag: String,

    /// Output directory for CSV files
    #[arg(short, long, default_value = ".")]
    output: String,

    /// Maximum number of scans to export (0 = all)
    #[arg(long, default_value = "0")]
    max_scans: usize,

    /// Minimum range filter (meters)
    #[arg(long, default_value = "0.15")]
    min_range: f32,

    /// Maximum range filter (meters)
    #[arg(long, default_value = "8.0")]
    max_range: f32,
}

fn main() {
    let args = Args::parse();

    if let Err(e) = run(&args) {
        eprintln!("Error: {}", e);
        std::process::exit(1);
    }
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    println!("╔══════════════════════════════════════════════════════════════════╗");
    println!("║                     BAG TO CSV EXPORTER                          ║");
    println!("╚══════════════════════════════════════════════════════════════════╝\n");

    println!("Input bag: {}", args.bag);
    println!("Output directory: {}", args.output);
    println!(
        "Range filter: {:.2}m - {:.2}m",
        args.min_range, args.max_range
    );
    if args.max_scans > 0 {
        println!("Max scans: {}", args.max_scans);
    }
    println!();

    // Create output directory if needed
    std::fs::create_dir_all(&args.output)?;

    // Open bag file
    let mut player = BagPlayer::open(&args.bag)?;

    // Open output files
    let scans_path = Path::new(&args.output).join("scans.csv");
    let odom_path = Path::new(&args.output).join("odom.csv");

    let mut scans_file = BufWriter::new(File::create(&scans_path)?);
    let mut odom_file = BufWriter::new(File::create(&odom_path)?);

    // Write headers
    writeln!(
        scans_file,
        "timestamp_us,scan_index,angle_rad,distance_m,quality"
    )?;
    writeln!(odom_file, "timestamp_us,encoder_left,encoder_right,gyro_z")?;

    let mut scan_count = 0usize;
    let mut sensor_count = 0usize;
    let mut point_count = 0usize;

    print!("Processing");
    std::io::stdout().flush()?;

    while let Ok(Some(msg)) = player.next_immediate() {
        match msg {
            BagMessage::SensorStatus(status) => {
                writeln!(
                    odom_file,
                    "{},{},{},{}",
                    status.timestamp_us,
                    status.encoder.left,
                    status.encoder.right,
                    status.gyro_raw[2] // Z axis (yaw) per ROS REP-103
                )?;
                sensor_count += 1;
            }
            BagMessage::Lidar(timestamped) => {
                // Check max scans limit
                if args.max_scans > 0 && scan_count >= args.max_scans {
                    break;
                }

                // Export each point in the scan
                for (angle_rad, distance_m, quality) in &timestamped.data {
                    // Apply range filter
                    if *distance_m >= args.min_range && *distance_m <= args.max_range {
                        writeln!(
                            scans_file,
                            "{},{},{:.6},{:.4},{}",
                            timestamped.timestamp_us, scan_count, angle_rad, distance_m, quality
                        )?;
                        point_count += 1;
                    }
                }

                scan_count += 1;

                // Progress indicator
                if scan_count % 100 == 0 {
                    print!(".");
                    std::io::stdout().flush()?;
                }
            }
            BagMessage::Odometry(_) => {
                // Skip odometry messages
            }
        }
    }

    println!(" done!\n");

    // Flush and close files
    scans_file.flush()?;
    odom_file.flush()?;

    println!("=== Export Summary ===\n");
    println!("Scans exported: {}", scan_count);
    println!("Points exported: {}", point_count);
    println!("Sensor messages: {}", sensor_count);
    println!();
    println!("Output files:");
    println!("  Scans: {}", scans_path.display());
    println!("  Odometry: {}", odom_path.display());

    // Show file sizes
    let scans_size = std::fs::metadata(&scans_path)?.len();
    let odom_size = std::fs::metadata(&odom_path)?.len();
    println!();
    println!("File sizes:");
    println!("  Scans: {:.2} MB", scans_size as f64 / 1_048_576.0);
    println!("  Odometry: {:.2} MB", odom_size as f64 / 1_048_576.0);

    Ok(())
}
