//! Mock transport for testing

use super::Transport;
use crate::error::Result;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

/// Mock transport for unit testing
#[derive(Clone)]
pub struct MockTransport {
    inner: Arc<Mutex<MockTransportInner>>,
}

struct MockTransportInner {
    read_buffer: VecDeque<u8>,
    write_buffer: Vec<u8>,
}

impl MockTransport {
    /// Create a new mock transport
    pub fn new() -> Self {
        MockTransport {
            inner: Arc::new(Mutex::new(MockTransportInner {
                read_buffer: VecDeque::new(),
                write_buffer: Vec::new(),
            })),
        }
    }

    /// Inject data to be read
    pub fn inject_read(&self, data: &[u8]) {
        let mut inner = self.inner.lock().unwrap();
        inner.read_buffer.extend(data);
    }

    /// Get all written data
    pub fn get_written(&self) -> Vec<u8> {
        let inner = self.inner.lock().unwrap();
        inner.write_buffer.clone()
    }

    /// Clear written data
    pub fn clear_written(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.write_buffer.clear();
    }

    /// Clear read buffer
    pub fn clear_read(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.read_buffer.clear();
    }
}

impl Transport for MockTransport {
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        let mut inner = self.inner.lock().unwrap();
        let available = inner.read_buffer.len().min(buffer.len());

        for item in buffer.iter_mut().take(available) {
            *item = inner.read_buffer.pop_front().unwrap();
        }

        Ok(available)
    }

    fn write(&mut self, data: &[u8]) -> Result<usize> {
        let mut inner = self.inner.lock().unwrap();
        inner.write_buffer.extend_from_slice(data);
        Ok(data.len())
    }

    fn flush(&mut self) -> Result<()> {
        Ok(())
    }

    fn available(&mut self) -> Result<usize> {
        let inner = self.inner.lock().unwrap();
        Ok(inner.read_buffer.len())
    }
}

impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}
