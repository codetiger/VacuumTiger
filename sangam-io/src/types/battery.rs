//! Battery status types

/// Battery status information
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryStatus {
    /// Voltage in volts
    pub voltage: f32,
    /// Current in amps (negative when charging)
    pub current: f32,
    /// Charge level (0-100%)
    pub level: u8,
    /// Is charging
    pub charging: bool,
}

impl BatteryStatus {
    /// Create new battery status
    pub fn new(voltage: f32, current: f32, level: u8, charging: bool) -> Self {
        Self {
            voltage,
            current,
            level,
            charging,
        }
    }

    /// Check if battery is low (< 20%)
    pub fn is_low(&self) -> bool {
        self.level < 20
    }

    /// Check if battery is critical (< 10%)
    pub fn is_critical(&self) -> bool {
        self.level < 10
    }

    /// Get estimated power consumption (watts)
    pub fn power(&self) -> f32 {
        self.voltage * self.current.abs()
    }
}

impl Default for BatteryStatus {
    fn default() -> Self {
        Self {
            voltage: 0.0,
            current: 0.0,
            level: 0,
            charging: false,
        }
    }
}
