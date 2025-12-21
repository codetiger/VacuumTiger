//! SIMD utilities and documentation.
//!
//! This crate uses Rust's `std::simd` (portable_simd) for SIMD operations.
//! No custom wrapper types are needed - use `std::simd` types directly.
//!
//! # Usage
//!
//! ```rust,ignore
//! use std::simd::{f32x4, num::SimdFloat};
//!
//! // Contiguous load from slice
//! let xs = f32x4::from_slice(&data[base..]);
//!
//! // Broadcast scalar to all lanes
//! let scale = f32x4::splat(2.0);
//!
//! // Element-wise operations
//! let result = xs * scale;
//!
//! // Horizontal reduction
//! let sum = result.reduce_sum();
//! ```
//!
//! # Key Operations
//!
//! | Operation              | Code                          |
//! |------------------------|-------------------------------|
//! | Load from array        | `f32x4::from_array([a,b,c,d])` |
//! | Load from slice        | `f32x4::from_slice(&slice[..])` |
//! | Broadcast scalar       | `f32x4::splat(x)`             |
//! | Horizontal sum         | `.reduce_sum()`               |
//! | Horizontal min/max     | `.reduce_min()`, `.reduce_max()` |
//! | Element-wise min/max   | `.simd_min(other)`, `.simd_max(other)` |
//! | Absolute value         | `.abs()`                      |
//! | Store to array         | `.to_array()`                 |
//!
//! # Performance Notes
//!
//! - Use contiguous `from_slice()` loads for best performance
//! - Avoid indexed/gather access in hot loops
//! - Pad arrays to SIMD width (4) for clean iteration
//! - Use `reduce_*` methods for horizontal operations
