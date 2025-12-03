  Performance Analysis: dhruva-slam TCP & Memory Optimization

  Executive Summary

  The dhruva-slam codebase is generally well-architected with sensible memory management. However, there are several optimization opportunities, particularly in hot paths that execute at 500Hz (sensor
  data) and 5Hz (lidar processing).

  ---
  1. TCP Packet Reading (sangam_client.rs)

  Current Implementation (Good):
  - Uses a reusable 64KB buffer that grows lazily
  - Length-prefixed framing avoids dynamic allocation for each message
  - Buffer resizes only when message exceeds current capacity

  Bottleneck: JSON deserialization allocates heavily on every message.

  | Issue                                   | Location    | Frequency | Impact |
  |-----------------------------------------|-------------|-----------|--------|
  | HashMap<String, SensorValue> allocation | Line 107    | 500 Hz    | High   |
  | String allocations for topic/group_id   | Lines 95-97 | 500 Hz    | Medium |
  | Vec<LidarPoint> for each scan           | Line 129    | 5 Hz      | Medium |

  Recommendations:

  // 1. Consider switching to Postcard binary format (already supported!)
  // In WireFormat::Postcard mode, serialization is 3-5x faster

  // 2. For JSON, use serde_json::from_slice with borrowed strings
  #[derive(Deserialize)]
  pub struct Message<'a> {
      #[serde(borrow)]
      pub topic: &'a str,  // Zero-copy from buffer
      pub payload: Payload<'a>,
  }

  // 3. Pre-size lidar Vec based on expected scan size (~540 points)
  const EXPECTED_LIDAR_POINTS: usize = 540;

  ---
  2. TCP Publisher Serialization (tcp_publisher.rs)

  Bottleneck: serde_json::to_vec() allocates a new Vec<u8> for every broadcast (50+ Hz combined).

  | Pattern                       | Line     | Frequency       | Allocation                |
  |-------------------------------|----------|-----------------|---------------------------|
  | serde_json::to_vec(msg)       | 242, 254 | 50+ Hz          | ~60-500 bytes per message |
  | Vec::new() for failed_indices | 279      | Every broadcast | Small                     |

  Recommendations:

  // 1. Add a reusable serialization buffer to OdometryPublisher
  pub struct OdometryPublisher {
      clients: Arc<Mutex<Vec<Client>>>,
      running: Arc<AtomicBool>,
      listener_handle: Option<thread::JoinHandle<()>>,
      // NEW: Reusable buffer for serialization
      serialize_buffer: RefCell<Vec<u8>>,
  }

  // 2. Use serde_json::to_writer to serialize directly into buffer
  fn broadcast_odometry(&self, msg: &OdometryMessage) {
      let mut buf = self.serialize_buffer.borrow_mut();
      buf.clear();
      if let Err(e) = serde_json::to_writer(&mut *buf, msg) {
          log::error!("Serialization failed: {}", e);
          return;
      }
      self.broadcast_bytes(&buf);
  }

  // 3. Pre-allocate failed_indices with small capacity
  let mut failed_indices: SmallVec<[usize; 4]> = SmallVec::new();

  ---
  3. Lidar Scan Conversion (scan.rs:59-98)

  Bottleneck: from_lidar_scan() creates a full copy to sort.

  // Line 73: Clones entire scan just to sort
  let mut sorted: Vec<_> = scan.to_vec();  // ALLOCATION: 540 × 12 bytes = 6.5KB
  sorted.sort_by(...);

  Recommendations:

  // Option A: Sort in-place if SangamIO can provide mutable reference
  pub fn from_lidar_scan_mut(scan: &mut LidarScan) -> Self {
      scan.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
      // ... use sorted data directly
  }

  // Option B: Check if already sorted (often is from hardware)
  pub fn from_lidar_scan(scan: &LidarScan) -> Self {
      let is_sorted = scan.windows(2).all(|w| w[0].0 <= w[1].0);

      if is_sorted {
          // Use directly, no clone needed
          Self::from_sorted_lidar(scan)
      } else {
          // Only clone when necessary
          let mut sorted = scan.to_vec();
          sorted.sort_by(...);
          Self::from_sorted_lidar(&sorted)
      }
  }

  // Option C: Use argsort indices (avoids moving data)
  let mut indices: Vec<usize> = (0..scan.len()).collect();
  indices.sort_by(|&a, &b| scan[a].0.partial_cmp(&scan[b].0).unwrap_or(...));

  ---
  4. Point Cloud Transform (scan.rs:298-316)

  Current: Creates new Vec<Point2D> for each transform.

  pub fn transform(&self, pose: &Pose2D) -> PointCloud2D {
      let transformed_points: Vec<Point2D> = self.points
          .iter()
          .map(|p| { ... })
          .collect();  // ALLOCATION: ~200 points × 8 bytes
      ...
  }

  Good News: transform_mut() already exists at line 319! Prefer this when possible.

  Recommendation: Audit call sites to use transform_mut() where the original isn't needed.

  ---
  5. Map Serialization (slam_messages.rs:204-215)

  Bottleneck: Double allocation - Vec<u8> + Base64 string.

  // Line 204: First allocation
  let mut cells_u8 = Vec::with_capacity(width * height);  // 40KB for 200×200

  // Line 215: Second allocation (Base64 expands by ~33%)
  let cells = BASE64.encode(&cells_u8);  // ~53KB string

  Recommendations:

  // 1. Encode directly to pre-sized String
  let encoded_len = base64::encoded_len(width * height, base64::Config::STANDARD);
  let mut cells = String::with_capacity(encoded_len);

  // Use base64::write::EncoderStringWriter to encode chunks
  let mut encoder = base64::write::EncoderStringWriter::new(&mut cells, &BASE64);
  for y in 0..height {
      for x in 0..width {
          encoder.write_all(&[Self::log_odds_to_u8(grid.get_log_odds(x, y), config)])?;
      }
  }
  encoder.finish();

  // 2. Alternative: Stream encoding with iterator
  use base64::engine::general_purpose::STANDARD;
  let cells = STANDARD.encode(
      (0..height).flat_map(|y| (0..width).map(move |x| {
          Self::log_odds_to_u8(grid.get_log_odds(x, y), config)
      }))
  );

  ---
  6. Diagnostics Cloning (tcp_publisher.rs:212, 236)

  // Line 212
  let msg = OdometryMessage::Diagnostics(diagnostics.clone());

  // Line 236
  let msg = SlamMessage::Diagnostics(diagnostics.clone());

  Impact: Low (1 Hz publishing, ~20 bytes each).

  Recommendation: Accept by value if caller can transfer ownership:

  pub fn publish_diagnostics(&self, diagnostics: OdometryDiagnostics) {
      let msg = OdometryMessage::Diagnostics(diagnostics);  // No clone
      self.broadcast_odometry(&msg);
  }

  ---
  7. Downsampler Early Returns (downsampler.rs:59, 64)

  if input_count == 0 {
      return scan.clone();  // ALLOCATION but rare
  }
  if input_count <= self.config.target_points {
      return scan.clone();  // ALLOCATION: common case!
  }

  Issue: Cloning entire scan when no downsampling needed.

  Recommendations:

  // Option A: Return Cow<LaserScan>
  use std::borrow::Cow;

  pub fn apply<'a>(&self, scan: &'a LaserScan) -> Cow<'a, LaserScan> {
      if input_count <= self.config.target_points {
          Cow::Borrowed(scan)
      } else {
          Cow::Owned(self.downsample(scan))
      }
  }

  // Option B: Return enum
  pub enum DownsampleResult<'a> {
      Unchanged(&'a LaserScan),
      Downsampled(LaserScan),
  }

  ---
  8. Quick Wins Summary

  | Optimization                  | Effort | Impact     | Files                             |
  |-------------------------------|--------|------------|-----------------------------------|
  | Use Postcard wire format      | Low    | High       | sangam_client.rs, SangamIO config |
  | Reusable serialization buffer | Medium | Medium     | tcp_publisher.rs                  |
  | Check if lidar already sorted | Low    | Medium     | scan.rs                           |
  | Use transform_mut()           | Low    | Low-Medium | Call sites                        |
  | Avoid downsampler clone       | Medium | Medium     | downsampler.rs                    |
  | Pre-allocate failed_indices   | Low    | Low        | tcp_publisher.rs                  |
  | Stream base64 encoding        | Medium | Low (1Hz)  | slam_messages.rs                  |

  ---
  9. Profiling Recommendations

  Before implementing, I recommend adding benchmark targets:

  # Add to Cargo.toml
  [[bench]]
  name = "tcp_throughput"
  harness = false

  [[bench]]
  name = "scan_processing"
  harness = false

  Key metrics to measure:
  1. TCP message receive rate (target: sustain 500 Hz)
  2. Lidar scan conversion time (target: <1ms for 540 points)
  3. Map serialization time (target: <10ms for 200×200 grid)
  4. Memory high-water mark during operation

  ---
  10. Architecture-Level Optimization

  For maximum performance on the embedded target, consider:

  1. Zero-copy message passing using a ring buffer between TCP receiver and processing threads
  2. SIMD for point transforms using packed_simd or wide crates
  3. Pre-allocated object pools for frequently allocated structs like PointCloud2D

