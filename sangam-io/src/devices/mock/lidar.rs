//! Mock lidar driver for testing

use crate::drivers::LidarDriver;
use crate::error::Result;
use crate::types::LidarScan;
use std::sync::{Arc, Mutex};

/// Mock lidar driver
#[derive(Clone)]
pub struct MockLidarDriver {
    state: Arc<Mutex<MockLidarState>>,
}

#[derive(Debug, Clone)]
struct MockLidarState {
    scanning: bool,
    last_scan: Option<LidarScan>,
}

impl MockLidarDriver {
    /// Create new mock lidar driver
    pub fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(MockLidarState {
                scanning: false,
                last_scan: None,
            })),
        }
    }

    /// Inject a scan for testing
    pub fn inject_scan(&self, scan: LidarScan) {
        let mut state = self.state.lock().unwrap();
        state.last_scan = Some(scan);
    }
}

impl Default for MockLidarDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl LidarDriver for MockLidarDriver {
    fn start(&mut self) -> Result<()> {
        let mut state = self.state.lock().unwrap();
        state.scanning = true;
        Ok(())
    }

    fn get_scan(&mut self) -> Result<Option<LidarScan>> {
        let mut state = self.state.lock().unwrap();
        Ok(state.last_scan.take())
    }

    fn stop(&mut self) -> Result<()> {
        let mut state = self.state.lock().unwrap();
        state.scanning = false;
        Ok(())
    }

    fn is_scanning(&self) -> bool {
        let state = self.state.lock().unwrap();
        state.scanning
    }
}
