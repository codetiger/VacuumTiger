//! Signal handling utilities for graceful shutdown.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// Set up a Ctrl-C handler that sets the returned flag to false.
///
/// # Example
/// ```ignore
/// let running = setup_ctrl_c_handler()?;
/// while running.load(Ordering::SeqCst) {
///     // ... do work ...
/// }
/// ```
pub fn setup_ctrl_c_handler() -> Result<Arc<AtomicBool>, Box<dyn std::error::Error>> {
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;
    Ok(running)
}
