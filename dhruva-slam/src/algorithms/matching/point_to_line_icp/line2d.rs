//! 2D line representation for point-to-line ICP.

use crate::core::types::Point2D;

/// A line in 2D space represented as ax + by + c = 0.
///
/// Normalized so that a² + b² = 1.
#[derive(Debug, Clone, Copy)]
pub struct Line2D {
    /// Normal vector x component
    pub a: f32,
    /// Normal vector y component
    pub b: f32,
    /// Distance from origin
    pub c: f32,
    /// Fit quality (R² value, 0-1)
    pub quality: f32,
}

impl Line2D {
    /// Fit a line through a set of points using least squares.
    ///
    /// Returns None if fewer than 2 points or points are collinear.
    pub fn fit(points: &[Point2D]) -> Option<Self> {
        if points.len() < 2 {
            return None;
        }

        let n = points.len() as f32;

        // Compute centroid
        let cx: f32 = points.iter().map(|p| p.x).sum::<f32>() / n;
        let cy: f32 = points.iter().map(|p| p.y).sum::<f32>() / n;

        // Compute covariance matrix elements
        let mut sxx = 0.0f32;
        let mut syy = 0.0f32;
        let mut sxy = 0.0f32;

        for p in points {
            let dx = p.x - cx;
            let dy = p.y - cy;
            sxx += dx * dx;
            syy += dy * dy;
            sxy += dx * dy;
        }

        // Eigenvalue decomposition for 2x2 symmetric matrix
        // The line direction is the eigenvector with larger eigenvalue
        let trace = sxx + syy;
        let det = sxx * syy - sxy * sxy;
        let discriminant = (trace * trace / 4.0 - det).max(0.0);
        let sqrt_disc = discriminant.sqrt();

        let lambda1 = trace / 2.0 + sqrt_disc; // Larger eigenvalue
        let lambda2 = trace / 2.0 - sqrt_disc; // Smaller eigenvalue

        // Quality is ratio of eigenvalues (1.0 = perfect line)
        let quality = if lambda1 > 1e-10 {
            1.0 - (lambda2 / lambda1)
        } else {
            0.0
        };

        // Normal vector is eigenvector of smaller eigenvalue
        // For [sxx, sxy; sxy, syy], eigenvector of lambda2 is [sxy, lambda2 - sxx]
        // or [-lambda2 + syy, sxy] (perpendicular to line direction)
        let (a, b) = if sxy.abs() > 1e-10 {
            (sxy, lambda2 - sxx)
        } else if sxx > syy {
            // Line is horizontal (y = const)
            (0.0, 1.0)
        } else {
            // Line is vertical (x = const)
            (1.0, 0.0)
        };

        // Normalize
        let norm = (a * a + b * b).sqrt();
        if norm < 1e-10 {
            return None;
        }

        let a = a / norm;
        let b = b / norm;

        // c = -(ax + by) for a point on the line (use centroid)
        let c = -(a * cx + b * cy);

        Some(Self { a, b, c, quality })
    }

    /// Compute signed distance from a point to the line.
    #[inline]
    pub fn distance(&self, p: &Point2D) -> f32 {
        self.a * p.x + self.b * p.y + self.c
    }

    /// Project a point onto the line (used in tests).
    #[cfg(test)]
    pub fn project(&self, p: &Point2D) -> Point2D {
        let d = self.distance(p);
        Point2D::new(p.x - self.a * d, p.y - self.b * d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_line_fit_horizontal() {
        let points = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
            Point2D::new(3.0, 0.0),
        ];

        let line = Line2D::fit(&points).unwrap();

        // Horizontal line: y = 0, or 0*x + 1*y + 0 = 0
        assert!(
            line.quality > 0.99,
            "Quality should be high: {}",
            line.quality
        );
        assert!(line.b.abs() > 0.99, "Should be near horizontal");
    }

    #[test]
    fn test_line_fit_vertical() {
        let points = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(0.0, 1.0),
            Point2D::new(0.0, 2.0),
            Point2D::new(0.0, 3.0),
        ];

        let line = Line2D::fit(&points).unwrap();

        assert!(line.quality > 0.99);
        assert!(line.a.abs() > 0.99, "Should be near vertical");
    }

    #[test]
    fn test_line_fit_diagonal() {
        let points = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
            Point2D::new(2.0, 2.0),
            Point2D::new(3.0, 3.0),
        ];

        let line = Line2D::fit(&points).unwrap();

        assert!(line.quality > 0.99);
        // For y = x, normal is [1, -1]/sqrt(2) or [-1, 1]/sqrt(2)
        assert_relative_eq!(line.a.abs(), line.b.abs(), epsilon = 0.01);
    }

    #[test]
    fn test_line_distance() {
        let line = Line2D {
            a: 0.0,
            b: 1.0,
            c: -1.0, // y = 1
            quality: 1.0,
        };

        let p1 = Point2D::new(0.0, 1.0);
        let p2 = Point2D::new(0.0, 2.0);
        let p3 = Point2D::new(0.0, 0.0);

        assert_relative_eq!(line.distance(&p1), 0.0, epsilon = 0.001);
        assert_relative_eq!(line.distance(&p2), 1.0, epsilon = 0.001);
        assert_relative_eq!(line.distance(&p3), -1.0, epsilon = 0.001);
    }

    #[test]
    fn test_line_project() {
        let line = Line2D {
            a: 0.0,
            b: 1.0,
            c: -1.0, // y = 1
            quality: 1.0,
        };

        let p = Point2D::new(5.0, 3.0);
        let projected = line.project(&p);

        assert_relative_eq!(projected.x, 5.0, epsilon = 0.001);
        assert_relative_eq!(projected.y, 1.0, epsilon = 0.001);
    }
}
