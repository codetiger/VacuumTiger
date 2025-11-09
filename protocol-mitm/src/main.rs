//! Serial Man-in-the-Middle Logger for GD32 Protocol Capture
//!
//! Creates a virtual serial port at /tmp/ttyS3_tap that AuxCtrl connects to.
//! This program forwards all data between the virtual port and real /dev/ttyS3_hardware,
//! while logging everything bidirectionally with microsecond timestamps.
//!
//! Features:
//! - Graceful shutdown on SIGTERM/SIGINT
//! - Run counter integration for multi-session captures
//! - GD32 protocol-aware packet logging
//! - Transparent PTY-based forwarding
//!
//! Setup (automated by mitm_boot_setup.sh):
//!   1. Move /dev/ttyS3 → /dev/ttyS3_hardware
//!   2. Run this program (creates /tmp/ttyS3_tap)
//!   3. Symlink /dev/ttyS3 → /tmp/ttyS3_tap
//!   4. Monitor/AuxCtrl starts normally, transparently uses MITM
//!
//! Manual usage:
//!   ./serial_mitm [run_number]
//!
//! Exit:
//!   Send SIGTERM (kill <pid>) or Ctrl+C for graceful shutdown

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::io::{AsRawFd, RawFd};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

const REAL_PORT: &str = "/dev/ttyS3_hardware";
const VIRTUAL_PORT: &str = "/tmp/ttyS3_tap";
const LOG_DIR: &str = "/tmp"; // Use RAM instead of flash to avoid write latency
const RUN_COUNTER_FILE: &str = "/mnt/UDISK/mitm_run_counter";
const GPIO_233_VALUE: &str = "/sys/class/gpio/gpio233/value";

/// Global flag for graceful shutdown
static RUNNING: AtomicBool = AtomicBool::new(true);

/// Signal handler for SIGTERM and SIGINT
extern "C" fn signal_handler(_sig: libc::c_int) {
    RUNNING.store(false, Ordering::Relaxed);
}

fn timestamp_micros() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros()
}

fn format_timestamp(micros: u128) -> String {
    format!("{}.{:06}", micros / 1_000_000, micros % 1_000_000)
}

fn log_packet(log: &mut File, direction: &str, data: &[u8]) -> io::Result<()> {
    let ts = timestamp_micros();

    writeln!(
        log,
        "[{}] {} {} bytes",
        format_timestamp(ts),
        direction,
        data.len()
    )?;

    // Hex dump
    write!(log, "  HEX: ")?;
    for byte in data {
        write!(log, "{:02X} ", byte)?;
    }
    writeln!(log)?;

    // Try to decode as GD32 packet (FA FB header)
    if data.len() >= 4 && data[0] == 0xFA && data[1] == 0xFB {
        let len = data[2] as usize;
        let cmd = data[3];
        writeln!(log, "  PKT: CMD=0x{:02X} LEN={}", cmd, len)?;

        // Decode common commands
        match cmd {
            0x08 => writeln!(log, "       (INIT - Initialization sequence)")?,
            0x15 => writeln!(log, "       (STATUS - Sensor/IMU/Battery data)")?,
            0x66 => writeln!(log, "       (HEARTBEAT - Keepalive packet)")?,
            0x06 => writeln!(log, "       (WAKE - Enable command)")?,
            0x07 => writeln!(log, "       (SETUP - System configuration)")?,
            0x8D => writeln!(log, "       (LED - Button LED state)")?,
            0x97 => {
                if data.len() >= 5 {
                    let power_state = data[4];
                    writeln!(
                        log,
                        "       (LIDAR_POWER - GPIO 233 = {})",
                        if power_state == 1 { "ON" } else { "OFF" }
                    )?;
                } else {
                    writeln!(log, "       (LIDAR_POWER - Incomplete packet)")?;
                }
            }
            0x71 => {
                if data.len() >= 8 {
                    let pwm = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
                    writeln!(log, "       (LIDAR_PWM - Speed = {}%)", pwm)?;
                } else {
                    writeln!(log, "       (LIDAR_PWM - Incomplete packet)")?;
                }
            }
            _ => {}
        }
    }

    writeln!(log)?;
    log.flush()?;
    Ok(())
}

fn configure_serial(fd: RawFd) -> io::Result<()> {
    unsafe {
        let mut termios: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut termios) != 0 {
            return Err(io::Error::last_os_error());
        }

        // Raw mode
        libc::cfmakeraw(&mut termios);

        // 115200 baud
        libc::cfsetispeed(&mut termios, libc::B115200);
        libc::cfsetospeed(&mut termios, libc::B115200);

        // 8N1
        termios.c_cflag &= !libc::PARENB; // No parity
        termios.c_cflag &= !libc::CSTOPB; // 1 stop bit
        termios.c_cflag &= !libc::CSIZE;
        termios.c_cflag |= libc::CS8; // 8 data bits
        termios.c_cflag |= libc::CREAD | libc::CLOCAL;

        // Non-blocking
        termios.c_cc[libc::VTIME] = 0;
        termios.c_cc[libc::VMIN] = 0;

        if libc::tcsetattr(fd, libc::TCSANOW, &termios) != 0 {
            return Err(io::Error::last_os_error());
        }
    }
    Ok(())
}

fn create_pty() -> io::Result<(RawFd, String)> {
    unsafe {
        let mut master: libc::c_int = 0;
        let mut slave: libc::c_int = 0;

        if libc::openpty(
            &mut master,
            &mut slave,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        ) != 0
        {
            return Err(io::Error::last_os_error());
        }

        // Get slave name
        let mut buf: [libc::c_char; 256] = [0; 256];
        if libc::ttyname_r(slave, buf.as_mut_ptr(), buf.len()) != 0 {
            libc::close(master);
            libc::close(slave);
            return Err(io::Error::last_os_error());
        }

        let slave_name = std::ffi::CStr::from_ptr(buf.as_ptr())
            .to_string_lossy()
            .into_owned();

        libc::close(slave);

        Ok((master, slave_name))
    }
}

fn read_run_counter() -> u32 {
    std::fs::read_to_string(RUN_COUNTER_FILE)
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(1)
}

/// Read current GPIO 233 value (0 or 1)
fn read_gpio_233() -> io::Result<u8> {
    let mut content = String::new();
    File::open(GPIO_233_VALUE)?.read_to_string(&mut content)?;
    content
        .trim()
        .parse()
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "Invalid GPIO value"))
}

/// Log GPIO state change
fn log_gpio_change(log: &mut File, old_state: u8, new_state: u8) -> io::Result<()> {
    let ts = timestamp_micros();
    writeln!(
        log,
        "[{}] GPIO_233: {} -> {} (Lidar Power {})",
        format_timestamp(ts),
        old_state,
        new_state,
        if new_state == 1 { "ON" } else { "OFF" }
    )?;
    writeln!(log)?;
    log.flush()?;
    Ok(())
}

/// Spawn GPIO monitoring thread
fn spawn_gpio_monitor(log_file: Arc<Mutex<File>>) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut last_state = read_gpio_233().unwrap_or(0);

        // Log initial state
        {
            let ts = timestamp_micros();
            if let Ok(mut log) = log_file.lock() {
                let _ = writeln!(
                    log,
                    "[{}] GPIO_233: Initial state = {}",
                    format_timestamp(ts),
                    last_state
                );
                let _ = writeln!(log);
                let _ = log.flush();
            }
        }

        while RUNNING.load(Ordering::Relaxed) {
            if let Ok(current_state) = read_gpio_233()
                && current_state != last_state
            {
                // State changed - log it!
                if let Ok(mut log) = log_file.lock() {
                    let _ = log_gpio_change(&mut log, last_state, current_state);
                }
                last_state = current_state;
            }

            // Poll every 10ms (fast enough to catch all transitions)
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    })
}

fn main() -> io::Result<()> {
    // Install signal handlers
    unsafe {
        libc::signal(libc::SIGTERM, signal_handler as libc::sighandler_t);
        libc::signal(libc::SIGINT, signal_handler as libc::sighandler_t);
    }

    // Determine run number
    let run_number = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(read_run_counter);

    println!("========================================");
    println!("  Serial MITM Logger - GD32 Protocol");
    println!("  Run: {}", run_number);
    println!("  Real port: {}", REAL_PORT);
    println!("========================================\n");

    // Create log directory if it doesn't exist
    let _ = std::fs::create_dir_all(LOG_DIR);

    // Generate log filename with timestamp and run number
    let timestamp = chrono::Local::now().format("%Y%m%d_%H%M%S");
    let log_filename = format!(
        "{}/mitm_capture_run{}_{}.log",
        LOG_DIR, run_number, timestamp
    );
    println!("Log file: {}\n", log_filename);

    // Open log file (wrapped in Arc<Mutex<>> for sharing with GPIO thread)
    let log_file = Arc::new(Mutex::new(
        OpenOptions::new()
            .create(true)
            .append(true)
            .open(&log_filename)?,
    ));

    // Write session header
    {
        let mut log = log_file.lock().unwrap();
        writeln!(log, "=")?;
        writeln!(log, "=== MITM Session Started ===")?;
        writeln!(log, "Run Number: {}", run_number)?;
        writeln!(log, "Timestamp: {}", chrono::Local::now().to_rfc3339())?;
        writeln!(log, "Real port: {}", REAL_PORT)?;
        writeln!(log, "Virtual port: {}", VIRTUAL_PORT)?;
        writeln!(log, "GPIO Monitoring: Enabled (GPIO 233)")?;
        writeln!(log, "===\n")?;
        log.flush()?;
    }

    // Start GPIO monitoring thread
    println!("Starting GPIO 233 monitoring thread...");
    let gpio_thread = spawn_gpio_monitor(Arc::clone(&log_file));
    println!("✓ GPIO monitoring active");

    // Open real serial port
    println!("Opening real serial port {}...", REAL_PORT);
    let real_serial = OpenOptions::new().read(true).write(true).open(REAL_PORT)?;

    let real_fd = real_serial.as_raw_fd();
    configure_serial(real_fd)?;
    println!("✓ Real port opened and configured");

    // Create pseudo-terminal
    println!("Creating virtual serial port...");
    let (pty_master, pty_slave_name) = create_pty()?;

    // Configure PTY to raw mode to prevent data escaping
    configure_serial(pty_master)?;
    println!(
        "✓ Virtual port created and configured at: {}",
        pty_slave_name
    );

    // Create symlink
    println!("Creating symlink {} -> {}", VIRTUAL_PORT, pty_slave_name);
    let _ = std::fs::remove_file(VIRTUAL_PORT);
    std::os::unix::fs::symlink(&pty_slave_name, VIRTUAL_PORT)?;
    println!("✓ Symlink created");

    println!("\nMITM proxy is running!");
    println!("AuxCtrl will connect to {} (redirected)", VIRTUAL_PORT);
    println!(
        "Send SIGTERM (kill {}) or Ctrl+C to stop gracefully\n",
        unsafe { libc::getpid() }
    );

    // Statistics counters
    let mut tx_packets = 0u64;
    let mut rx_packets = 0u64;
    let mut tx_bytes = 0u64;
    let mut rx_bytes = 0u64;

    // Proxy loop
    let mut real_buf = [0u8; 1024];
    let mut pty_buf = [0u8; 1024];

    while RUNNING.load(Ordering::Relaxed) {
        // Check for data from AuxCtrl (via PTY) -> Real Serial
        unsafe {
            let n = libc::read(
                pty_master,
                pty_buf.as_mut_ptr() as *mut libc::c_void,
                pty_buf.len(),
            );
            if n > 0 {
                let n = n as usize;
                tx_packets += 1;
                tx_bytes += n as u64;

                // Log TX (AuxCtrl -> GD32)
                if let Ok(mut log) = log_file.lock() {
                    let _ = log_packet(&mut log, "TX", &pty_buf[..n]);
                }

                // Forward to real serial port
                let written = libc::write(real_fd, pty_buf.as_ptr() as *const libc::c_void, n);
                if written < 0 {
                    eprintln!("Error writing to real port");
                }
            }
        }

        // Check for data from Real Serial -> AuxCtrl (via PTY)
        unsafe {
            let n = libc::read(
                real_fd,
                real_buf.as_mut_ptr() as *mut libc::c_void,
                real_buf.len(),
            );
            if n > 0 {
                let n = n as usize;
                rx_packets += 1;
                rx_bytes += n as u64;

                // Log RX (GD32 -> AuxCtrl)
                if let Ok(mut log) = log_file.lock() {
                    let _ = log_packet(&mut log, "RX", &real_buf[..n]);
                }

                // Forward to PTY (AuxCtrl)
                let written = libc::write(pty_master, real_buf.as_ptr() as *const libc::c_void, n);
                if written < 0 {
                    eprintln!("Error writing to PTY");
                }
            }
        }

        // Small sleep to prevent busy-waiting (100μs = 0.1ms)
        std::thread::sleep(std::time::Duration::from_micros(100));
    }

    // Graceful shutdown
    println!("\n\nShutting down gracefully...");

    // Wait for GPIO monitoring thread to finish
    println!("Stopping GPIO monitoring thread...");
    let _ = gpio_thread.join();
    println!("✓ GPIO thread stopped");

    // Write session footer with statistics
    {
        let mut log = log_file.lock().unwrap();
        writeln!(log, "\n=== MITM Session Ended ===")?;
        writeln!(log, "End Timestamp: {}", chrono::Local::now().to_rfc3339())?;
        writeln!(log, "TX Packets: {}", tx_packets)?;
        writeln!(log, "RX Packets: {}", rx_packets)?;
        writeln!(log, "TX Bytes: {}", tx_bytes)?;
        writeln!(log, "RX Bytes: {}", rx_bytes)?;
        writeln!(log, "===\n")?;
        log.flush()?;
    }

    println!("Statistics:");
    println!("  TX: {} packets, {} bytes", tx_packets, tx_bytes);
    println!("  RX: {} packets, {} bytes", rx_packets, rx_bytes);
    println!("  Log: {}", log_filename);

    // Cleanup
    unsafe {
        libc::close(pty_master);
        libc::close(real_fd);
    }
    let _ = std::fs::remove_file(VIRTUAL_PORT);

    println!("✓ MITM proxy stopped cleanly\n");

    Ok(())
}
