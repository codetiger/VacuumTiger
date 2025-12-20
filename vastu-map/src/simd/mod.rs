//! SIMD types for auto-vectorization.
//!
//! This module provides SIMD-friendly types that encourage LLVM to
//! emit vectorized instructions on ARM NEON and x86 SSE/AVX.
//!
//! # Target Platform
//!
//! Primary target: Allwinner A33 (Cortex-A7) with:
//! - ARMv7-A architecture
//! - NEON 128-bit SIMD (16 Q-registers)
//! - VFPv4 with fused multiply-add (FMA) support
//!
//! # Design Philosophy
//!
//! Rather than using explicit NEON intrinsics, we rely on LLVM's auto-vectorization.
//! The [`Float4`] type uses:
//! - 16-byte alignment matching NEON's 128-bit registers
//! - `#[inline(always)]` to expose vectorizable patterns
//! - Simple element-wise operations that map directly to NEON instructions
//!
//! This approach provides:
//! - **Portability**: Works on x86 SSE/AVX without code changes
//! - **Simplicity**: No unsafe intrinsics or target-specific code
//! - **Maintainability**: Standard Rust that's easy to debug
//!
//! # Performance Characteristics
//!
//! On ARM Cortex-A7, most operations complete in a single cycle:
//!
//! | Operation | NEON Instruction | Latency |
//! |-----------|------------------|---------|
//! | Add/Sub   | `vaddq_f32`/`vsubq_f32` | 1 cycle |
//! | Multiply  | `vmulq_f32` | 1 cycle |
//! | Min/Max   | `vminq_f32`/`vmaxq_f32` | 1 cycle |
//! | FMA       | `vfmaq_f32` | 1 cycle |
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
//!
//! # Compilation Flags
//!
//! For optimal NEON codegen, compile with:
//! ```text
//! RUSTFLAGS="-C target-feature=+neon -C target-cpu=cortex-a7"
//! ```

mod f32x4;

pub use f32x4::Float4;
