//! Statistical utility functions.

/// Compute the sample standard deviation of i16 values.
pub fn std_dev_i16(values: &[i16]) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
    let sum: i64 = values.iter().map(|&v| v as i64).sum();
    let mean = sum as f64 / values.len() as f64;
    let variance: f64 = values
        .iter()
        .map(|&v| (v as f64 - mean).powi(2))
        .sum::<f64>()
        / (values.len() - 1) as f64;
    variance.sqrt()
}

/// Compute statistics for i16 sensor values.
pub struct I16Stats {
    pub min: i16,
    pub max: i16,
    pub mean: f64,
    pub std: f64,
}

impl I16Stats {
    /// Compute statistics from a slice of i16 values.
    pub fn compute(values: &[i16]) -> Option<Self> {
        if values.is_empty() {
            return None;
        }
        let min = *values.iter().min().unwrap();
        let max = *values.iter().max().unwrap();
        let sum: i64 = values.iter().map(|&v| v as i64).sum();
        let mean = sum as f64 / values.len() as f64;
        let std = std_dev_i16(values);
        Some(Self {
            min,
            max,
            mean,
            std,
        })
    }

    /// Print statistics with a label.
    pub fn print(&self, name: &str) {
        println!(
            "{}: min={}, max={}, mean={:.2}, std={:.2}",
            name, self.min, self.max, self.mean, self.std
        );
    }
}
