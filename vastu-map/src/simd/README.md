# SIMD Module

SIMD-friendly types designed for LLVM auto-vectorization.

## Components

| File | Description |
|------|-------------|
| `f32x4.rs` | `Float4` - 4-wide float vector for SIMD operations |

## Design Philosophy

We use LLVM auto-vectorization rather than platform-specific intrinsics:

- **Portability**: Works on ARM NEON, x86 SSE/AVX, and other architectures
- **Simplicity**: No unsafe intrinsics or target-specific code
- **Maintainability**: Standard Rust that's easy to debug

Users can configure appropriate compiler flags for their target platform.

## Float4

4-wide float vector aligned for SIMD:

```rust
use vastu_map::simd::Float4;

// Create vectors
let a = Float4::new([1.0, 2.0, 3.0, 4.0]);
let b = Float4::splat(2.0);  // [2.0, 2.0, 2.0, 2.0]

// Arithmetic
let sum = a + b;             // Element-wise add
let diff = a - b;            // Element-wise subtract
let prod = a * b;            // Element-wise multiply

// Fused multiply-add
let result = a.mul_add(b, c);     // a * b + c
let result = a.neg_mul_add(b, c); // c - a * b

// Reduction
let sum = a.sum();           // Sum all elements
let min = a.min_element();   // Minimum element
let max = a.max_element();   // Maximum element

// Element access
let x = a[0];
let arr: [f32; 4] = a.into();
```

## Example: Point Transform

```rust
// Transform 4 points at once
let xs = Float4::new([1.0, 2.0, 3.0, 4.0]);
let ys = Float4::new([0.0, 0.0, 0.0, 0.0]);
let cos_theta = Float4::splat(0.866);  // cos(30deg)
let sin_theta = Float4::splat(0.5);    // sin(30deg)
let tx = Float4::splat(1.0);
let ty = Float4::splat(2.0);

// Rotation: x' = x*cos - y*sin + tx
//           y' = x*sin + y*cos + ty
let new_xs = ys.neg_mul_add(sin_theta, xs * cos_theta) + tx;
let new_ys = xs.mul_add(sin_theta, ys * cos_theta) + ty;
```

## Data Layout Patterns

For best SIMD utilization, use Struct-of-Arrays (SoA):

```
AoS (poor SIMD):            SoA (good SIMD):
[Point{x,y}]                xs: [x0, x1, x2, x3, ...]
[Point{x,y}]    →           ys: [y0, y1, y2, y3, ...]
[Point{x,y}]
[Point{x,y}]
```

SoA allows processing 4 coordinates per SIMD instruction:

```rust
// Process 4 points at once
for i in (0..xs.len()).step_by(4) {
    let x = Float4::new([xs[i], xs[i+1], xs[i+2], xs[i+3]]);
    let y = Float4::new([ys[i], ys[i+1], ys[i+2], ys[i+3]]);
    // Transform all 4 points simultaneously
}
```

## Alignment

`Float4` is 16-byte aligned (128-bit), matching common SIMD register sizes. This ensures optimal memory access patterns and enables aligned loads/stores.
