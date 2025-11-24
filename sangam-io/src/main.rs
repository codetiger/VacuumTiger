//! SangamIO - Hardware abstraction daemon for robot vacuum

mod config;
mod core;
mod devices;
mod error;
mod streaming;

use crate::config::Config;
use crate::core::types::{SensorGroupData, SensorValue};
use crate::devices::create_device;
use crate::error::Result;
use crate::streaming::{create_serializer, TcpPublisher, TcpReceiver, WireFormat};
use std::collections::HashMap;
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
        .unwrap_or_else(|| "hardware.json".to_string());

    // Load configuration
    log::info!("Loading config from: {}", config_path);
    let config = Config::load(&config_path)?;

    log::info!(
        "Device: {} ({})",
        config.device.name,
        config.device.device_type
    );

    // Create device driver
    let driver = create_device(&config)?;
    let driver = Arc::new(Mutex::new(driver));

    // Create pre-allocated sensor data for each group
    let mut sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>> = HashMap::new();

    for group in &config.device.sensor_groups {
        // Pre-allocate with default values for each sensor
        let keys: Vec<(&str, SensorValue)> = group
            .sensors
            .iter()
            .map(|s| {
                let default = match s.sensor_type {
                    crate::core::types::SensorType::Bool => SensorValue::Bool(false),
                    crate::core::types::SensorType::U8 => SensorValue::U8(0),
                    crate::core::types::SensorType::U16 => SensorValue::U16(0),
                    crate::core::types::SensorType::U32 => SensorValue::U32(0),
                    crate::core::types::SensorType::U64 => SensorValue::U64(0),
                    crate::core::types::SensorType::I8 => SensorValue::I8(0),
                    crate::core::types::SensorType::I16 => SensorValue::I16(0),
                    crate::core::types::SensorType::I32 => SensorValue::I32(0),
                    crate::core::types::SensorType::I64 => SensorValue::I64(0),
                    crate::core::types::SensorType::F32 => SensorValue::F32(0.0),
                    crate::core::types::SensorType::F64 => SensorValue::F64(0.0),
                    crate::core::types::SensorType::String => SensorValue::String(String::new()),
                    crate::core::types::SensorType::Vector3 => {
                        SensorValue::Vector3([0.0, 0.0, 0.0])
                    }
                    crate::core::types::SensorType::PointCloud2D => {
                        SensorValue::PointCloud2D(vec![])
                    }
                };
                (s.id.as_str(), default)
            })
            .collect();

        let data = SensorGroupData::new_with_keys(&group.id, &keys);
        sensor_data.insert(group.id.clone(), Arc::new(Mutex::new(data)));

        log::info!(
            "Created sensor group '{}' with {} sensors",
            group.id,
            group.sensors.len()
        );
    }

    // Initialize driver with shared sensor data
    {
        let mut driver = driver.lock().map_err(|_| error::Error::MutexPoisoned)?;
        driver.initialize(sensor_data.clone())?;
    }

    // Set up shutdown signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = Arc::clone(&running);

    ctrlc::set_handler(move || {
        log::info!("Received shutdown signal");
        r.store(false, Ordering::Relaxed);
    })
    .map_err(|e| error::Error::Other(format!("Error setting Ctrl-C handler: {}", e)))?;

    // Get wire format from config
    let wire_format: WireFormat = config.network.wire_format.into();
    log::info!("Wire format: {:?}", wire_format);

    // Start TCP listener
    let bind_addr = &config.network.bind_address;
    let listener = TcpListener::bind(bind_addr)
        .map_err(|e| error::Error::Other(format!("Failed to bind to {}: {}", bind_addr, e)))?;
    listener.set_nonblocking(true).ok();

    log::info!("TCP server listening on {}", bind_addr);
    log::info!("SangamIO running. Press Ctrl-C to stop.");

    // Main loop - accept connections and spawn handlers
    while running.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, addr)) => {
                log::info!("Client connected: {}", addr);

                // Clone resources for threads
                let sensor_data_clone = sensor_data.clone();
                let driver_clone = Arc::clone(&driver);

                // Create serializers for each thread
                let pub_serializer = create_serializer(wire_format);
                let recv_serializer = create_serializer(wire_format);

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
                        let publisher =
                            TcpPublisher::new(pub_serializer, sensor_data_clone, pub_running);
                        if let Err(e) = publisher.run(pub_stream) {
                            log::error!("Publisher error: {}", e);
                        }
                    });

                // Spawn receiver thread
                let recv_running = Arc::clone(&running);
                let _recv_handle = thread::Builder::new()
                    .name("tcp-receiver".to_string())
                    .spawn(move || {
                        let receiver =
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
