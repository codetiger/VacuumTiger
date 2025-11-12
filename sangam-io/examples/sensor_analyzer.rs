//! GD32 Sensor Data Analyzer
//!
//! Minimal tool for reverse-engineering GD32 status packet (CMD=0x15) byte offsets.
//! Initializes SangamIO, runs for 30 seconds with debug logging, then exits.
//!
//! Usage:
//! ```bash
//! RUST_LOG=debug /tmp/sensor_analyzer
//! ```

use sangam_io::SangamIO;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    println!("Initializing SangamIO...");
    let sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;
    sangam.set_brushes(0, 50)?;
    std::thread::sleep(std::time::Duration::from_secs(1));
    println!("Running for 30 seconds. Perform sensor tests now.");

    std::thread::sleep(std::time::Duration::from_secs(30));

    sangam.set_brushes(0, 0)?;
    std::thread::sleep(std::time::Duration::from_secs(1));
    println!("Shutting down.");
    Ok(())
}
