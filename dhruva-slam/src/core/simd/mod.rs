//! SIMD vector types for optimized numerical computations.
//!
//! Provides platform-agnostic SIMD abstractions designed to enable
//! LLVM auto-vectorization on ARMv7 NEON and other architectures.
//!
//! # Design
//!
//! The implementation uses a scalar representation with 16-byte alignment
//! and `#[inline(always)]` annotations to encourage LLVM to generate
//! SIMD instructions when targeting platforms with vector extensions.
//!
//! When compiled with `-C target-feature=+neon` on ARMv7, LLVM will
//! typically auto-vectorize these operations to NEON instructions.

mod f32x4;

pub use f32x4::Float4;
