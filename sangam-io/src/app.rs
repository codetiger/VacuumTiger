//! Application orchestration for SangamIO daemon
//!
//! Manages hardware initialization, streaming, command subscription, and graceful shutdown.

use crate::config::AppConfig;
use crate::devices::{Delta2DDriver, Gd32Driver, LidarDriver};
use crate::error::Result;
use crate::serial_io::SerialTransport;
use crate::streaming::{TcpCommandReceiver, TcpPublisher};
use log::{debug, error, info, warn};
use parking_lot::Mutex;
use signal_hook::consts::{SIGINT, SIGTERM};
use signal_hook::iterator::Signals;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// Main application structure that manages all components
pub struct SangamApp {
    config: AppConfig,
    publisher: Arc<TcpPublisher>,
    receiver: Arc<Mutex<TcpCommandReceiver>>,
    // TODO: Keep reference to GD32 driver for future use (e.g., manual control, diagnostics)
    // Currently only used to pass to TcpCommandReceiver, but may be needed for direct access
    #[allow(dead_code)]
    gd32_driver: Arc<Mutex<Gd32Driver>>,
    lidar_driver: Option<Arc<Mutex<Delta2DDriver>>>,
    shutdown: Arc<AtomicBool>,
}

impl SangamApp {
    /// Create new SangamApp instance
    ///
    /// Initializes all hardware components and streaming services
    pub fn new(config: AppConfig) -> Result<Self> {
        info!("Initializing SangamIO application");

        // Initialize TCP publisher (telemetry + lidar)
        info!(
            "Setting up TCP publisher on {}",
            config.streaming.tcp_pub_address
        );
        let publisher = Arc::new(TcpPublisher::new(config.streaming.tcp_pub_address.clone())?);

        // Initialize GD32 motor controller with streaming publisher
        info!("Initializing GD32 driver on {}", config.hardware.gd32_port);
        let gd32_transport = SerialTransport::open(&config.hardware.gd32_port, 115200)?;
        let gd32_driver = Gd32Driver::new_with_publisher(gd32_transport, Arc::clone(&publisher))?;
        let gd32_driver = Arc::new(Mutex::new(gd32_driver));

        // Initialize Delta-2D lidar
        info!(
            "Initializing Delta-2D lidar on {}",
            config.hardware.lidar_port
        );
        let lidar_transport = SerialTransport::open(&config.hardware.lidar_port, 115200)?;
        let lidar_driver = Delta2DDriver::new(lidar_transport)?;
        let lidar_driver = Some(Arc::new(Mutex::new(lidar_driver)));

        // Initialize TCP command receiver
        info!(
            "Setting up TCP command receiver on {}",
            config.streaming.tcp_cmd_address
        );
        let receiver = TcpCommandReceiver::new(
            config.streaming.tcp_cmd_address.clone(),
            Arc::clone(&gd32_driver),
            config.robot.max_ticks_per_sec,
        )?;
        let receiver = Arc::new(Mutex::new(receiver));

        // Setup shutdown flag
        let shutdown = Arc::new(AtomicBool::new(false));

        info!("✓ Hardware and streaming initialized successfully");

        Ok(Self {
            config,
            publisher,
            receiver,
            gd32_driver,
            lidar_driver,
            shutdown,
        })
    }

    /// Start all background threads and run the main loop
    pub fn run(&mut self) -> Result<()> {
        info!("Starting application threads");

        // Start GD32 telemetry publishing thread
        self.start_gd32_telemetry_thread()?;

        // Start lidar scanning with callback
        if let Some(ref lidar) = self.lidar_driver {
            self.start_lidar_with_publisher(Arc::clone(lidar))?;
        } else {
            warn!("Lidar driver not available, skipping lidar streaming");
        }

        // Start command receiver thread
        self.start_command_receiver_thread()?;

        // Setup signal handler for graceful shutdown
        self.setup_signal_handler();

        info!("✓ All threads started successfully");
        info!("Publishing on: {}", self.config.streaming.tcp_pub_address);
        info!(
            "Receiving commands on: {}",
            self.config.streaming.tcp_cmd_address
        );
        info!("");
        info!("Press Ctrl+C to stop");

        // Main loop - keep alive while streaming
        let mut last_stats = Instant::now();

        while !self.shutdown.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(100));

            // Print statistics every 10 seconds
            if last_stats.elapsed().as_secs() >= 10 {
                self.log_statistics();
                last_stats = Instant::now();
            }
        }

        info!("Shutdown signal received, stopping threads...");
        self.stop_all_threads()?;

        Ok(())
    }

    /// Start GD32 telemetry publishing thread
    ///
    /// NOTE: This is now handled automatically by the GD32 driver's READ and WRITE threads.
    /// - READ thread publishes SensorUpdate at ~500 Hz
    /// - WRITE thread publishes ConnectionQuality at 1 Hz
    ///
    /// This method is kept as a placeholder for potential future use.
    fn start_gd32_telemetry_thread(&self) -> Result<()> {
        // No longer needed - GD32 driver handles publishing internally
        debug!("GD32 telemetry publishing integrated into driver threads");
        Ok(())
    }

    /// Start lidar with publisher callback
    fn start_lidar_with_publisher(&self, lidar_driver: Arc<Mutex<Delta2DDriver>>) -> Result<()> {
        // CRITICAL: Power on lidar motor via GD32 before starting scanner
        // This sequence is required: lidar_prep → lidar_power → lidar_pwm
        // From working commit fc8e8f6, this was done before starting lidar scanning
        info!("Powering on lidar motor via GD32...");
        {
            let mut gd32 = self.gd32_driver.lock();

            // Step 1: Send lidar preparation command (CMD 0xA2)
            gd32.send_lidar_prep()?;
            std::thread::sleep(Duration::from_millis(50));

            // Step 2: Enable lidar power (CMD 0x97)
            gd32.set_lidar_power(true)?;
            std::thread::sleep(Duration::from_millis(50));

            // Step 3: Set lidar PWM to 100% (CMD 0x71)
            gd32.set_lidar_pwm(100)?;
            std::thread::sleep(Duration::from_millis(50));

            info!("✓ Lidar motor powered on");
        }

        // Get lidar queue from publisher (lock-free, non-blocking)
        let lidar_queue = self.publisher.get_lidar_queue();

        let callback = move |scan: crate::streaming::messages::LidarScan| {
            // Push to queue (non-blocking, returns immediately)
            if lidar_queue.push(scan).is_err() {
                // Queue full - drop scan (lidar is best-effort)
                if log::log_enabled!(log::Level::Trace) {
                    log::trace!("Lidar queue full, dropped scan");
                }
            }
        };

        let mut lidar = lidar_driver.lock();
        lidar.start(callback)?;

        info!("✓ Lidar scanning started");
        Ok(())
    }

    /// Start command receiver thread
    fn start_command_receiver_thread(&self) -> Result<()> {
        let receiver = Arc::clone(&self.receiver);
        let shutdown = Arc::clone(&self.shutdown);

        std::thread::Builder::new()
            .name("command-receiver".to_string())
            .spawn(move || {
                debug!("Command receiver thread started");

                while !shutdown.load(Ordering::Relaxed) {
                    let mut rcv = receiver.lock();

                    // Process one command with timeout
                    if let Err(e) = rcv.process_command(Duration::from_millis(100)) {
                        debug!("Command processing error: {}", e);
                    }
                }

                debug!("Command receiver thread exiting");
            })?;

        info!("✓ Command receiver started");
        Ok(())
    }

    /// Setup signal handler for graceful shutdown
    fn setup_signal_handler(&self) {
        let shutdown = Arc::clone(&self.shutdown);

        std::thread::Builder::new()
            .name("signal-handler".to_string())
            .spawn(move || {
                let mut signals =
                    Signals::new([SIGINT, SIGTERM]).expect("Failed to register signal handlers");

                if let Some(sig) = signals.forever().next() {
                    info!("Received signal {:?}, initiating shutdown...", sig);
                    shutdown.store(true, Ordering::Relaxed);
                }
            })
            .expect("Failed to spawn signal handler thread");
    }

    /// Log application statistics
    fn log_statistics(&self) {
        info!(
            "Application running... Streaming: telemetry (~500Hz) + lidar (~5Hz) + commands (on-demand)"
        );

        // Get lidar statistics if available
        if let Some(ref lidar) = self.lidar_driver
            && let Some(driver) = lidar.try_lock()
        {
            let (scans, errors) = driver.get_stats();
            let error_rate = if scans > 0 {
                (errors as f32 / scans as f32) * 100.0
            } else {
                0.0
            };
            info!(
                "Lidar: Scans={} Errors={} Loss={:.1}%",
                scans, errors, error_rate
            );
        }
    }

    /// Stop all background threads
    fn stop_all_threads(&mut self) -> Result<()> {
        info!("Stopping all threads...");

        // Signal shutdown to all threads
        self.shutdown.store(true, Ordering::Relaxed);

        // Stop lidar scanning
        if let Some(ref lidar) = self.lidar_driver {
            info!("Stopping lidar...");
            if let Some(mut driver) = lidar.try_lock() {
                driver.stop()?;
            }

            // Power off lidar motor via GD32
            info!("Powering off lidar motor...");
            {
                let mut gd32 = self.gd32_driver.lock();

                // Set PWM to 0
                gd32.set_lidar_pwm(0)?;
                std::thread::sleep(Duration::from_millis(50));

                // Disable lidar power
                gd32.set_lidar_power(false)?;

                info!("✓ Lidar motor powered off");
            }
        }

        // Wait a moment for threads to finish
        std::thread::sleep(Duration::from_millis(200));

        info!("✓ All threads stopped");
        Ok(())
    }
}

impl Drop for SangamApp {
    fn drop(&mut self) {
        debug!("SangamApp cleaning up...");

        // Ensure shutdown is signaled
        self.shutdown.store(true, Ordering::Relaxed);

        // Try to stop threads gracefully
        if let Err(e) = self.stop_all_threads() {
            error!("Error during cleanup: {}", e);
        }
    }
}
