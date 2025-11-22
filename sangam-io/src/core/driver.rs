//! DeviceDriver trait definition

use crate::core::types::{Command, SensorGroupData};
use crate::error::Result;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Device driver trait for hardware abstraction
pub trait DeviceDriver: Send {
    /// Initialize hardware and start updating sensor data
    ///
    /// The driver should:
    /// 1. Open serial ports
    /// 2. Start internal threads (heartbeat, readers)
    /// 3. Update shared sensor data directly (no allocations in loop)
    /// 4. Update timestamp when data changes
    fn initialize(
        &mut self,
        sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>,
    ) -> Result<()>;

    /// Send command to hardware
    fn send_command(&mut self, cmd: Command) -> Result<()>;
}
