//! Matcher types and scratch buffers.

/// Scratch buffers for SIMD operations to avoid per-call allocations.
pub struct ScratchBuffers {
    /// Transformed world X coordinates
    pub world_xs: Vec<f32>,
    /// Transformed world Y coordinates
    pub world_ys: Vec<f32>,
    /// Grid X coordinates
    pub grid_xs: Vec<i32>,
    /// Grid Y coordinates
    pub grid_ys: Vec<i32>,
}

impl ScratchBuffers {
    /// Create scratch buffers for a given number of points.
    pub fn new(capacity: usize) -> Self {
        Self {
            world_xs: vec![0.0; capacity],
            world_ys: vec![0.0; capacity],
            grid_xs: vec![0; capacity],
            grid_ys: vec![0; capacity],
        }
    }

    /// Resize buffers if needed.
    pub fn resize(&mut self, n: usize) {
        if self.world_xs.len() < n {
            self.world_xs.resize(n, 0.0);
            self.world_ys.resize(n, 0.0);
            self.grid_xs.resize(n, 0);
            self.grid_ys.resize(n, 0);
        }
    }
}
