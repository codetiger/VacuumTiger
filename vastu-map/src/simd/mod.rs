//! SIMD types for auto-vectorization.
//!
//! This module provides SIMD-friendly types that encourage LLVM to
//! emit vectorized instructions on ARM NEON and x86 SSE/AVX.

mod f32x4;

pub use f32x4::Float4;
