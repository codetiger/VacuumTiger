//! SIMD types for auto-vectorization.
//!
//! This module provides SIMD-friendly types that encourage LLVM to
//! emit vectorized instructions across different architectures.
//!
//! # Design Philosophy
//!
//! We rely on LLVM's auto-vectorization rather than platform-specific intrinsics.
//! The [`Float4`] type uses:
//! - 16-byte alignment matching common 128-bit SIMD registers
//! - `#[inline(always)]` to expose vectorizable patterns
//! - Simple element-wise operations that map to SIMD instructions
//!
//! This approach provides:
//! - **Portability**: Works on ARM NEON, x86 SSE/AVX, and other architectures
//! - **Simplicity**: No unsafe intrinsics or target-specific code
//! - **Maintainability**: Standard Rust that's easy to debug
//!
//! Users can configure appropriate compiler flags for their target platform.
//!
//! # Usage Example
//!
//! ```rust
//! use vastu_map::simd::Float4;
//!
//! // Transform 4 points at once
//! let xs = Float4::new([1.0, 2.0, 3.0, 4.0]);
//! let ys = Float4::new([0.0, 0.0, 0.0, 0.0]);
//! let cos_theta = Float4::splat(0.866);  // cos(30°)
//! let sin_theta = Float4::splat(0.5);    // sin(30°)
//!
//! // Rotation: x' = x*cos - y*sin, y' = x*sin + y*cos
//! let new_xs = ys.neg_mul_add(sin_theta, xs * cos_theta);
//! let new_ys = xs.mul_add(sin_theta, ys * cos_theta);
//! ```

mod f32x4;

pub use f32x4::Float4;
