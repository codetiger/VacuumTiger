//! 4-wide f32 SIMD vector type.
//!
//! This implementation is designed for LLVM auto-vectorization on ARMv7 NEON.
//! The struct uses 16-byte alignment to match NEON's 128-bit registers.
//!
//! # Target Platform
//!
//! Baseline: Allwinner A33 (Cortex-A7)
//! - ARMv7-A architecture
//! - NEON 128-bit SIMD (16 Q-registers)
//! - VFPv4 with FMA support
//!
//! # NEON Instruction Mapping
//!
//! | Rust Operation | NEON Intrinsic | Cycles |
//! |----------------|----------------|--------|
//! | `Float4 + Float4` | `vaddq_f32` | 1 |
//! | `Float4 - Float4` | `vsubq_f32` | 1 |
//! | `Float4 * Float4` | `vmulq_f32` | 1 |
//! | `Float4::min()` | `vminq_f32` | 1 |
//! | `Float4::max()` | `vmaxq_f32` | 1 |
//! | `Float4::splat()` | `vdupq_n_f32` | 1 |
//! | `mul_add()` | `vfmaq_f32` | 1 |

use std::ops::{Add, AddAssign, Mul, Sub};

/// A 4-wide f32 vector optimized for SIMD auto-vectorization.
///
/// # Memory Layout
///
/// Uses `#[repr(C, align(16))]` to ensure:
/// - C-compatible memory layout for predictable access patterns
/// - 16-byte alignment matching NEON's 128-bit (16-byte) Q-registers
///
/// # Auto-vectorization
///
/// All methods use `#[inline(always)]` to encourage LLVM to inline
/// and recognize vectorizable patterns. When compiled with
/// `-C target-feature=+neon -C target-cpu=cortex-a7`, LLVM will
/// typically emit NEON instructions for these operations.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C, align(16))]
pub struct Float4(pub [f32; 4]);

impl Float4 {
    /// Creates a new `Float4` from an array of 4 f32 values.
    ///
    /// # Example
    /// ```
    /// use vastu_map::simd::Float4;
    /// let v = Float4::new([1.0, 2.0, 3.0, 4.0]);
    /// ```
    #[inline(always)]
    pub const fn new(arr: [f32; 4]) -> Self {
        Self(arr)
    }

    /// Creates a `Float4` with all elements set to the same value.
    ///
    /// This is equivalent to NEON's `vdupq_n_f32` intrinsic.
    ///
    /// # Example
    /// ```
    /// use vastu_map::simd::Float4;
    /// let v = Float4::splat(5.0);
    /// assert_eq!(v.to_array(), [5.0, 5.0, 5.0, 5.0]);
    /// ```
    #[inline(always)]
    pub const fn splat(v: f32) -> Self {
        Self([v, v, v, v])
    }

    /// Creates a `Float4` with all elements set to zero.
    #[inline(always)]
    pub const fn zero() -> Self {
        Self([0.0, 0.0, 0.0, 0.0])
    }

    /// Extracts the elements as an array.
    ///
    /// # Example
    /// ```
    /// use vastu_map::simd::Float4;
    /// let v = Float4::new([1.0, 2.0, 3.0, 4.0]);
    /// let arr = v.to_array();
    /// assert_eq!(arr, [1.0, 2.0, 3.0, 4.0]);
    /// ```
    #[inline(always)]
    pub const fn to_array(self) -> [f32; 4] {
        self.0
    }

    /// Element-wise minimum of two vectors.
    ///
    /// Returns a new vector where each element is the minimum of the
    /// corresponding elements in `self` and `other`.
    ///
    /// This is equivalent to NEON's `vminq_f32` intrinsic.
    ///
    /// # Example
    /// ```
    /// use vastu_map::simd::Float4;
    /// let a = Float4::new([1.0, 5.0, 3.0, 8.0]);
    /// let b = Float4::new([2.0, 3.0, 4.0, 1.0]);
    /// let min = a.min(b);
    /// assert_eq!(min.to_array(), [1.0, 3.0, 3.0, 1.0]);
    /// ```
    #[inline(always)]
    pub fn min(self, other: Self) -> Self {
        Self([
            self.0[0].min(other.0[0]),
            self.0[1].min(other.0[1]),
            self.0[2].min(other.0[2]),
            self.0[3].min(other.0[3]),
        ])
    }

    /// Element-wise maximum of two vectors.
    ///
    /// Returns a new vector where each element is the maximum of the
    /// corresponding elements in `self` and `other`.
    ///
    /// This is equivalent to NEON's `vmaxq_f32` intrinsic.
    ///
    /// # Example
    /// ```
    /// use vastu_map::simd::Float4;
    /// let a = Float4::new([1.0, 5.0, 3.0, 8.0]);
    /// let b = Float4::new([2.0, 3.0, 4.0, 1.0]);
    /// let max = a.max(b);
    /// assert_eq!(max.to_array(), [2.0, 5.0, 4.0, 8.0]);
    /// ```
    #[inline(always)]
    pub fn max(self, other: Self) -> Self {
        Self([
            self.0[0].max(other.0[0]),
            self.0[1].max(other.0[1]),
            self.0[2].max(other.0[2]),
            self.0[3].max(other.0[3]),
        ])
    }

    /// Fused multiply-add: `(self * a) + b`.
    ///
    /// Computes `self * a + b` with better precision and performance than
    /// separate multiply and add operations. On ARM NEON, this maps to
    /// `vfmaq_f32` (VFPv4).
    ///
    /// # Example
    /// ```
    /// use vastu_map::simd::Float4;
    /// let a = Float4::new([1.0, 2.0, 3.0, 4.0]);
    /// let b = Float4::new([2.0, 2.0, 2.0, 2.0]);
    /// let c = Float4::new([10.0, 10.0, 10.0, 10.0]);
    /// let result = a.mul_add(b, c);
    /// // (1*2+10, 2*2+10, 3*2+10, 4*2+10) = (12, 14, 16, 18)
    /// assert_eq!(result.to_array(), [12.0, 14.0, 16.0, 18.0]);
    /// ```
    #[inline(always)]
    pub fn mul_add(self, a: Self, b: Self) -> Self {
        Self([
            self.0[0].mul_add(a.0[0], b.0[0]),
            self.0[1].mul_add(a.0[1], b.0[1]),
            self.0[2].mul_add(a.0[2], b.0[2]),
            self.0[3].mul_add(a.0[3], b.0[3]),
        ])
    }

    /// Fused negative multiply-add: `-(self * a) + b` = `b - (self * a)`.
    ///
    /// Useful for computing expressions like `tx - xs * sin_theta`.
    ///
    /// # Example
    /// ```
    /// use vastu_map::simd::Float4;
    /// let a = Float4::new([1.0, 2.0, 3.0, 4.0]);
    /// let b = Float4::new([2.0, 2.0, 2.0, 2.0]);
    /// let c = Float4::new([10.0, 10.0, 10.0, 10.0]);
    /// let result = a.neg_mul_add(b, c);
    /// // (10-1*2, 10-2*2, 10-3*2, 10-4*2) = (8, 6, 4, 2)
    /// assert_eq!(result.to_array(), [8.0, 6.0, 4.0, 2.0]);
    /// ```
    #[inline(always)]
    pub fn neg_mul_add(self, a: Self, b: Self) -> Self {
        Self([
            (-self.0[0]).mul_add(a.0[0], b.0[0]),
            (-self.0[1]).mul_add(a.0[1], b.0[1]),
            (-self.0[2]).mul_add(a.0[2], b.0[2]),
            (-self.0[3]).mul_add(a.0[3], b.0[3]),
        ])
    }

    /// Horizontal sum of all elements.
    ///
    /// Returns the sum of all 4 elements in the vector.
    #[inline(always)]
    pub fn sum(self) -> f32 {
        self.0[0] + self.0[1] + self.0[2] + self.0[3]
    }

    /// Horizontal minimum of all elements.
    ///
    /// Returns the minimum of all 4 elements in the vector.
    #[inline(always)]
    pub fn horizontal_min(self) -> f32 {
        self.0[0].min(self.0[1]).min(self.0[2]).min(self.0[3])
    }

    /// Horizontal maximum of all elements.
    ///
    /// Returns the maximum of all 4 elements in the vector.
    #[inline(always)]
    pub fn horizontal_max(self) -> f32 {
        self.0[0].max(self.0[1]).max(self.0[2]).max(self.0[3])
    }
}

impl Add for Float4 {
    type Output = Self;

    /// Element-wise addition.
    ///
    /// This is equivalent to NEON's `vaddq_f32` intrinsic.
    #[inline(always)]
    fn add(self, rhs: Self) -> Self {
        Self([
            self.0[0] + rhs.0[0],
            self.0[1] + rhs.0[1],
            self.0[2] + rhs.0[2],
            self.0[3] + rhs.0[3],
        ])
    }
}

impl Sub for Float4 {
    type Output = Self;

    /// Element-wise subtraction.
    ///
    /// This is equivalent to NEON's `vsubq_f32` intrinsic.
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self {
        Self([
            self.0[0] - rhs.0[0],
            self.0[1] - rhs.0[1],
            self.0[2] - rhs.0[2],
            self.0[3] - rhs.0[3],
        ])
    }
}

impl Mul for Float4 {
    type Output = Self;

    /// Element-wise multiplication.
    ///
    /// This is equivalent to NEON's `vmulq_f32` intrinsic.
    #[inline(always)]
    fn mul(self, rhs: Self) -> Self {
        Self([
            self.0[0] * rhs.0[0],
            self.0[1] * rhs.0[1],
            self.0[2] * rhs.0[2],
            self.0[3] * rhs.0[3],
        ])
    }
}

impl AddAssign for Float4 {
    /// In-place element-wise addition.
    #[inline(always)]
    fn add_assign(&mut self, rhs: Self) {
        self.0[0] += rhs.0[0];
        self.0[1] += rhs.0[1];
        self.0[2] += rhs.0[2];
        self.0[3] += rhs.0[3];
    }
}

impl Default for Float4 {
    fn default() -> Self {
        Self::zero()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_and_to_array() {
        let v = Float4::new([1.0, 2.0, 3.0, 4.0]);
        assert_eq!(v.to_array(), [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_splat() {
        let v = Float4::splat(5.0);
        assert_eq!(v.to_array(), [5.0, 5.0, 5.0, 5.0]);
    }

    #[test]
    fn test_zero() {
        let v = Float4::zero();
        assert_eq!(v.to_array(), [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_add() {
        let a = Float4::new([1.0, 2.0, 3.0, 4.0]);
        let b = Float4::new([5.0, 6.0, 7.0, 8.0]);
        let sum = a + b;
        assert_eq!(sum.to_array(), [6.0, 8.0, 10.0, 12.0]);
    }

    #[test]
    fn test_sub() {
        let a = Float4::new([5.0, 6.0, 7.0, 8.0]);
        let b = Float4::new([1.0, 2.0, 3.0, 4.0]);
        let diff = a - b;
        assert_eq!(diff.to_array(), [4.0, 4.0, 4.0, 4.0]);
    }

    #[test]
    fn test_mul() {
        let a = Float4::new([1.0, 2.0, 3.0, 4.0]);
        let b = Float4::new([2.0, 3.0, 4.0, 5.0]);
        let prod = a * b;
        assert_eq!(prod.to_array(), [2.0, 6.0, 12.0, 20.0]);
    }

    #[test]
    fn test_add_assign() {
        let mut a = Float4::new([1.0, 2.0, 3.0, 4.0]);
        let b = Float4::new([5.0, 6.0, 7.0, 8.0]);
        a += b;
        assert_eq!(a.to_array(), [6.0, 8.0, 10.0, 12.0]);
    }

    #[test]
    fn test_min() {
        let a = Float4::new([1.0, 5.0, 3.0, 8.0]);
        let b = Float4::new([2.0, 3.0, 4.0, 1.0]);
        let min = a.min(b);
        assert_eq!(min.to_array(), [1.0, 3.0, 3.0, 1.0]);
    }

    #[test]
    fn test_max() {
        let a = Float4::new([1.0, 5.0, 3.0, 8.0]);
        let b = Float4::new([2.0, 3.0, 4.0, 1.0]);
        let max = a.max(b);
        assert_eq!(max.to_array(), [2.0, 5.0, 4.0, 8.0]);
    }

    #[test]
    fn test_mul_add() {
        let a = Float4::new([1.0, 2.0, 3.0, 4.0]);
        let b = Float4::new([2.0, 2.0, 2.0, 2.0]);
        let c = Float4::new([10.0, 10.0, 10.0, 10.0]);
        let result = a.mul_add(b, c);
        assert_eq!(result.to_array(), [12.0, 14.0, 16.0, 18.0]);
    }

    #[test]
    fn test_neg_mul_add() {
        let a = Float4::new([1.0, 2.0, 3.0, 4.0]);
        let b = Float4::new([2.0, 2.0, 2.0, 2.0]);
        let c = Float4::new([10.0, 10.0, 10.0, 10.0]);
        let result = a.neg_mul_add(b, c);
        assert_eq!(result.to_array(), [8.0, 6.0, 4.0, 2.0]);
    }

    #[test]
    fn test_sum() {
        let v = Float4::new([1.0, 2.0, 3.0, 4.0]);
        assert_eq!(v.sum(), 10.0);
    }

    #[test]
    fn test_horizontal_min() {
        let v = Float4::new([3.0, 1.0, 4.0, 2.0]);
        assert_eq!(v.horizontal_min(), 1.0);
    }

    #[test]
    fn test_horizontal_max() {
        let v = Float4::new([3.0, 1.0, 4.0, 2.0]);
        assert_eq!(v.horizontal_max(), 4.0);
    }

    #[test]
    fn test_alignment() {
        let v = Float4::new([1.0, 2.0, 3.0, 4.0]);
        let ptr = &v as *const Float4 as usize;
        assert_eq!(ptr % 16, 0, "Float4 should be 16-byte aligned");
    }

    #[test]
    fn test_splat_special_values() {
        let inf = Float4::splat(f32::INFINITY);
        assert!(inf.to_array().iter().all(|&x| x == f32::INFINITY));

        let neg_inf = Float4::splat(f32::NEG_INFINITY);
        assert!(neg_inf.to_array().iter().all(|&x| x == f32::NEG_INFINITY));

        let zero = Float4::splat(0.0);
        assert_eq!(zero.to_array(), [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_fma_rotation_transform() {
        // Test FMA for 2D rotation: x' = x*cos - y*sin, y' = x*sin + y*cos
        let xs = Float4::new([1.0, 2.0, 3.0, 4.0]);
        let ys = Float4::new([0.0, 0.0, 0.0, 0.0]);
        let cos_t = Float4::splat(0.0); // cos(90°) = 0
        let sin_t = Float4::splat(1.0); // sin(90°) = 1

        // x' = x*cos - y*sin
        let new_xs = ys.neg_mul_add(sin_t, xs * cos_t);
        // y' = x*sin + y*cos
        let new_ys = xs.mul_add(sin_t, ys * cos_t);

        assert_eq!(new_xs.to_array(), [0.0, 0.0, 0.0, 0.0]);
        assert_eq!(new_ys.to_array(), [1.0, 2.0, 3.0, 4.0]);
    }
}
