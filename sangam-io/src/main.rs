//! SangamIO - Hardware Abstraction Daemon for Robotic Vacuum Cleaners
//!
//! A pure hardware abstraction layer that provides:
//! - Raw sensor data streaming (telemetry + lidar)
//! - Command execution (motion + actuators)
//! - Simple unit conversion (ticks/sec ← → motor units)
//!
//! No odometry computation, no motion control, no kinematics.
//! All intelligence lives in consumers (SLAM/Navigation layers).

use clap::Parser;
use log::info;
use std::path::PathBuf;

mod app;
mod config;
mod devices;
mod error;
mod serial_io;
mod streaming;

// Re-export error for easier access
use error::Result;

/// SangamIO - Hardware Abstraction Daemon
#[derive(Parser, Debug)]
#[command(name = "sangamio")]
#[command(version, about, long_about = None)]
struct Args {
    /// Path to configuration file
    #[arg(short, long, value_name = "FILE", default_value = "sangamio.toml")]
    config: PathBuf,

    /// Generate default configuration file and exit
    #[arg(long)]
    generate_config: bool,

    /// Verbose logging (debug level)
    #[arg(short, long)]
    verbose: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    // Initialize logging
    let log_level = if args.verbose { "debug" } else { "info" };
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_level)).init();

    info!("SangamIO v{} starting...", env!("CARGO_PKG_VERSION"));

    // Generate config if requested
    if args.generate_config {
        let config = config::AppConfig::crl200s_defaults();
        config.to_file(&args.config)?;
        info!(
            "Generated default configuration at: {}",
            args.config.display()
        );
        return Ok(());
    }

    // Load configuration
    let config = if args.config.exists() {
        info!("Loading configuration from: {}", args.config.display());
        config::AppConfig::from_file(&args.config)?
    } else {
        info!("Configuration file not found, using defaults");
        config::AppConfig::crl200s_defaults()
    };

    // Display configuration
    info!("Configuration loaded successfully");
    info!("  GD32 port: {}", config.hardware.gd32_port);
    info!("  Lidar port: {}", config.hardware.lidar_port);
    info!("  Publisher: {}", config.streaming.tcp_pub_address);
    info!("  Commands: {}", config.streaming.tcp_cmd_address);
    info!("  Max ticks/sec: {}", config.robot.max_ticks_per_sec);

    // Create and run the application
    let mut app = app::SangamApp::new(config)?;

    info!("SangamIO started successfully");
    app.run()?;

    info!("SangamIO stopped gracefully");
    Ok(())
}
