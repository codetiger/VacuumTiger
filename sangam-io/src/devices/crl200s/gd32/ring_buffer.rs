//! Zero-copy ring buffer for packet parsing
//!
//! Provides O(1) consume operations instead of O(n) Vec::drain().

/// Fixed-capacity ring buffer with O(1) advance
///
/// Generic const parameter `N` sets buffer capacity.
pub struct RingBuffer<const N: usize = 1024> {
    data: [u8; N],
    head: usize,        // Write position (next empty slot)
    tail: usize,        // Read position (first valid byte)
    len: usize,         // Number of bytes available
    staging: [u8; 256], // For non-contiguous slice access
}

impl<const N: usize> RingBuffer<N> {
    /// Create a new empty ring buffer
    pub const fn new() -> Self {
        Self {
            data: [0u8; N],
            head: 0,
            tail: 0,
            len: 0,
            staging: [0u8; 256],
        }
    }

    /// Append bytes to the buffer
    ///
    /// Bytes that would overflow are silently dropped.
    #[inline]
    pub fn extend(&mut self, bytes: &[u8]) {
        for &b in bytes {
            if self.len < N {
                self.data[self.head] = b;
                self.head = (self.head + 1) % N;
                self.len += 1;
            }
        }
    }

    /// Consume n bytes from the front - O(1)!
    ///
    /// This is the key optimization: unlike Vec::drain() which shifts all
    /// remaining bytes, this just advances a pointer.
    #[inline]
    pub fn advance(&mut self, n: usize) {
        let n = n.min(self.len);
        self.tail = (self.tail + n) % N;
        self.len -= n;
    }

    /// Number of bytes available to read
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Read byte at logical index (handles wraparound)
    #[inline]
    pub fn get(&self, index: usize) -> Option<u8> {
        if index < self.len {
            Some(self.data[(self.tail + index) % N])
        } else {
            None
        }
    }

    /// Find 2-byte sync pattern, returns offset from tail
    pub fn find_pattern_2(&self, b1: u8, b2: u8) -> Option<usize> {
        if self.len < 2 {
            return None;
        }
        (0..self.len - 1).find(|&i| {
            self.data[(self.tail + i) % N] == b1 && self.data[(self.tail + i + 1) % N] == b2
        })
    }

    /// Get contiguous slice (copies to staging if data wraps around)
    ///
    /// Returns a slice view into either the main buffer (if contiguous)
    /// or the staging buffer (if data spans the wraparound point).
    pub fn get_slice(&mut self, start: usize, len: usize) -> Option<&[u8]> {
        if start + len > self.len {
            return None;
        }

        if len > 256 {
            // Staging buffer limit
            return None;
        }

        let real_start = (self.tail + start) % N;

        // Check if data is contiguous (doesn't wrap around buffer end)
        if real_start + len <= N {
            // Contiguous in main buffer - return direct slice
            Some(&self.data[real_start..real_start + len])
        } else {
            // Spans wraparound - copy to staging buffer
            for i in 0..len {
                self.staging[i] = self.data[(real_start + i) % N];
            }
            Some(&self.staging[..len])
        }
    }
}

impl<const N: usize> Default for RingBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_operations() {
        let mut rb: RingBuffer<16> = RingBuffer::new();
        assert_eq!(rb.len(), 0);

        rb.extend(&[1, 2, 3, 4, 5]);
        assert_eq!(rb.len(), 5);
        assert_eq!(rb.get(0), Some(1));
        assert_eq!(rb.get(4), Some(5));
        assert_eq!(rb.get(5), None);
    }

    #[test]
    fn test_advance() {
        let mut rb: RingBuffer<16> = RingBuffer::new();
        rb.extend(&[1, 2, 3, 4, 5]);

        rb.advance(2);
        assert_eq!(rb.len(), 3);
        assert_eq!(rb.get(0), Some(3));
        assert_eq!(rb.get(2), Some(5));
    }

    #[test]
    fn test_wraparound() {
        let mut rb: RingBuffer<8> = RingBuffer::new();

        // Fill buffer partially
        rb.extend(&[1, 2, 3, 4, 5]);
        assert_eq!(rb.len(), 5);

        // Consume some
        rb.advance(3);
        assert_eq!(rb.len(), 2);
        assert_eq!(rb.get(0), Some(4));

        // Add more (will wrap)
        rb.extend(&[6, 7, 8, 9]);
        assert_eq!(rb.len(), 6);
        assert_eq!(rb.get(0), Some(4));
        assert_eq!(rb.get(1), Some(5));
        assert_eq!(rb.get(2), Some(6));
    }

    #[test]
    fn test_find_pattern_2() {
        let mut rb: RingBuffer<32> = RingBuffer::new();
        rb.extend(&[0x00, 0xFF, 0xFA, 0xFB, 0x03, 0x06]);

        assert_eq!(rb.find_pattern_2(0xFA, 0xFB), Some(2));
        assert_eq!(rb.find_pattern_2(0x00, 0xFF), Some(0));
        assert_eq!(rb.find_pattern_2(0xAA, 0xBB), None);
    }

    #[test]
    fn test_get_slice_contiguous() {
        let mut rb: RingBuffer<32> = RingBuffer::new();
        rb.extend(&[0xFA, 0xFB, 0x03, 0x06, 0x00, 0x06]);

        let slice = rb.get_slice(2, 2).unwrap();
        assert_eq!(slice, &[0x03, 0x06]);
    }

    #[test]
    fn test_get_slice_wrapped() {
        let mut rb: RingBuffer<8> = RingBuffer::new();

        // Position tail near end of buffer
        rb.extend(&[1, 2, 3, 4, 5, 6]);
        rb.advance(5); // tail=5, head=6, len=1
        rb.extend(&[7, 8, 9]); // head wraps to 1

        // Data is now: [8, 9, _, _, _, 6, 7, _] with tail=5
        // Logical view: [6, 7, 8, 9]
        assert_eq!(rb.len(), 4);

        let slice = rb.get_slice(0, 4).unwrap();
        assert_eq!(slice, &[6, 7, 8, 9]);
    }
}
