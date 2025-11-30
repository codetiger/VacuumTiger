//! DeviceDriver trait definition

use crate::core::types::{Command, SensorGroupData};
use crate::error::Result;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Device driver trait for hardware abstraction
pub trait DeviceDriver: Send {
    /// Initialize hardware and return sensor data groups
    ///
    /// The driver should:
    /// 1. Create sensor data groups for its sensors
    /// 2. Open serial ports
    /// 3. Start internal threads (heartbeat, readers)
    /// 4. Return the HashMap of sensor groups to be used by TCP publisher
    fn initialize(&mut self) -> Result<HashMap<String, Arc<Mutex<SensorGroupData>>>>;

    /// Send command to hardware
    fn send_command(&mut self, cmd: Command) -> Result<()>;
}
