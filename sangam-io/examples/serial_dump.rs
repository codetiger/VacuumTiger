//! Serial port raw byte dumper
//!
//! Opens the GD32 serial port and dumps all received bytes in hex format
//! for protocol analysis and debugging.

use sangam_io::transport::SerialTransport;
use sangam_io::transport::Transport;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    log::info!("=== GD32 Serial Port Dumper ===");
    log::info!("");

    // Open serial port
    log::info!("Opening serial port /dev/ttyS3 at 115200 baud...");
    let mut transport = SerialTransport::open("/dev/ttyS3", 115200)?;
    log::info!("Serial port opened successfully");
    log::info!("");

    // Flush any old data
    log::info!("Flushing old data...");
    let mut discard = vec![0u8; 1024];
    let mut total_flushed = 0;
    while transport.available()? > 0 {
        let read = transport.read(&mut discard)?;
        total_flushed += read;
        if read == 0 {
            break;
        }
    }
    log::info!("Flushed {} bytes", total_flushed);
    log::info!("");

    // Wait a bit for buffer to stabilize
    std::thread::sleep(Duration::from_millis(200));

    log::info!("Starting capture for 10 seconds...");
    log::info!("Looking for sync bytes: FA FB");
    log::info!("");

    let start = Instant::now();
    let duration = Duration::from_secs(10);
    let mut buffer = vec![0u8; 256];
    let mut total_bytes = 0;
    let mut packet_count = 0;

    while start.elapsed() < duration {
        let available = transport.available()?;
        if available > 0 {
            let bytes_read = transport.read(&mut buffer)?;

            if bytes_read > 0 {
                total_bytes += bytes_read;

                // Format as hex string
                let hex_line: String = buffer[..bytes_read]
                    .iter()
                    .map(|b| format!("{:02X}", b))
                    .collect::<Vec<_>>()
                    .join(" ");

                println!("[{:06}] {} bytes: {}", total_bytes, bytes_read, hex_line);

                // Count sync bytes (FA FB)
                for i in 0..bytes_read.saturating_sub(1) {
                    if buffer[i] == 0xFA && buffer[i + 1] == 0xFB {
                        packet_count += 1;
                        println!("  --> Sync found at offset {}", i);

                        // If we have enough bytes, show the packet header
                        if i + 6 <= bytes_read {
                            let length = buffer[i + 2];
                            let cmd_id = buffer[i + 3];
                            println!(
                                "      LEN=0x{:02X} ({}) CMD=0x{:02X}",
                                length, length, cmd_id
                            );
                        }
                    }
                }
            }
        } else {
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    log::info!("");
    log::info!("=== Capture Complete ===");
    log::info!("Total bytes received: {}", total_bytes);
    log::info!("Sync bytes (FA FB) found: {}", packet_count);
    log::info!(
        "Average bytes per packet: {}",
        if packet_count > 0 {
            total_bytes / packet_count
        } else {
            0
        }
    );

    Ok(())
}
