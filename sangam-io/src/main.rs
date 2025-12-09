//! SangamIO - Hardware abstraction daemon for robot vacuum

mod config;
mod core;
mod devices;
mod error;
mod streaming;

use crate::config::Config;
use crate::devices::create_device;
use crate::error::Result;
use crate::streaming::{TcpPublisher, TcpReceiver, create_serializer};
use std::env;
use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

fn main() -> Result<()> {
    // Initialize logger
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    log::info!("SangamIO v0.3.0 starting...");

    // Get config path from args or default
    let config_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "/etc/sangamio.toml".to_string());

    // Load configuration
    log::info!("Using config: {}", config_path);
    let config = Config::load(&config_path)?;

    log::info!(
        "Device: {} ({})",
        config.device.name,
        config.device.device_type
    );

    // Create device driver
    let mut driver = create_device(&config)?;

    // Initialize driver and get sensor data + streaming channels
    let init_result = driver.initialize()?;
    let sensor_data = init_result.sensor_data;
    let stream_receivers = init_result.stream_receivers;
    log::info!(
        "Initialized {} sensor groups ({} with streaming channels)",
        sensor_data.len(),
        stream_receivers.len()
    );

    let driver = Arc::new(Mutex::new(driver));

    // Set up shutdown signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = Arc::clone(&running);

    ctrlc::set_handler(move || {
        log::info!("Received shutdown signal");
        r.store(false, Ordering::Relaxed);
    })
    .map_err(|e| error::Error::Other(format!("Error setting Ctrl-C handler: {}", e)))?;

    log::info!("Wire format: Protobuf");

    // Start TCP listener
    let bind_addr = &config.network.bind_address;
    let listener = TcpListener::bind(bind_addr)
        .map_err(|e| error::Error::Other(format!("Failed to bind to {}: {}", bind_addr, e)))?;
    if let Err(e) = listener.set_nonblocking(true) {
        log::warn!("Failed to set nonblocking mode: {}", e);
    }

    log::info!("TCP server listening on {}", bind_addr);
    log::info!("SangamIO running. Press Ctrl-C to stop.");

    // Main loop - accept connections and spawn handlers
    while running.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, addr)) => {
                log::info!("Client connected: {}", addr);

                // Clone resources for threads
                let sensor_data_clone = sensor_data.clone();
                let stream_receivers_clone = stream_receivers.clone();
                let driver_clone = Arc::clone(&driver);

                // Create serializers for each thread
                let pub_serializer = create_serializer();
                let recv_serializer = create_serializer();

                // Spawn publisher thread
                let pub_running = Arc::clone(&running);
                let pub_stream = match stream.try_clone() {
                    Ok(s) => s,
                    Err(e) => {
                        log::error!("Failed to clone stream: {}", e);
                        continue;
                    }
                };
                let _pub_handle = thread::Builder::new()
                    .name("tcp-publisher".to_string())
                    .spawn(move || {
                        let publisher = TcpPublisher::new(
                            pub_serializer,
                            sensor_data_clone,
                            stream_receivers_clone,
                            pub_running,
                        );
                        if let Err(e) = publisher.run(pub_stream) {
                            log::error!("Publisher error: {}", e);
                        }
                    });

                // Spawn receiver thread
                let recv_running = Arc::clone(&running);
                let _recv_handle = thread::Builder::new()
                    .name("tcp-receiver".to_string())
                    .spawn(move || {
                        let mut receiver =
                            TcpReceiver::new(recv_serializer, driver_clone, recv_running);
                        if let Err(e) = receiver.run(stream) {
                            log::error!("Receiver error: {}", e);
                        }
                    });
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                // No connection pending, sleep briefly
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
            Err(e) => {
                log::error!("Accept error: {}", e);
            }
        }
    }

    // Shutdown
    log::info!("Shutting down...");
    {
        let mut driver = driver.lock().map_err(|_| error::Error::MutexPoisoned)?;
        driver.send_command(crate::core::types::Command::Shutdown)?;
    }

    log::info!("SangamIO stopped");
    Ok(())
}
