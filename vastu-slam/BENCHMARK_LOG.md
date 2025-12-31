# VastuSLAM Benchmark Log

This file tracks performance, accuracy, and quality metrics across commits for optimization reference.

## Standard Metrics Framework

### 1. Performance Metrics (Latency)

| Metric | Description | Target | Unit |
|--------|-------------|--------|------|
| `scan_matching_180` | Scan matching with 180 points | <1.5ms | µs |
| `scan_matching_360` | Scan matching with 360 points (typical) | <3.0ms | µs |
| `scan_matching_720` | Scan matching with 720 points (high-res) | <10ms | µs |
| `grid_update_180` | Grid update with 180 points | <200µs | µs |
| `grid_update_360` | Grid update with 360 points | <400µs | µs |
| `grid_update_720` | Grid update with 720 points | <800µs | µs |
| `coverage_stats` | Coverage computation (800x800 grid) | <200µs | µs |
| `ate_100` | ATE computation (100 poses) | <5µs | µs |
| `rpe_100` | RPE computation (100 poses) | <5µs | µs |
| `relations_metrics_100` | Relations metrics (100 poses) | <100µs | µs |
| `ground_truth_gen_100` | Ground truth generation (100 poses) | <50µs | µs |

### 2. Accuracy Metrics

| Metric | Description | Target | Unit |
|--------|-------------|--------|------|
| `ate_trans_mean` | ATE translation mean error | <0.01 | m |
| `ate_rot_mean` | ATE rotation mean error | <0.5 | deg |
| `rpe_trans_mean` | RPE translation mean error | <0.005 | m |
| `rpe_rot_mean` | RPE rotation mean error | <0.3 | deg |
| `relations_trans_mean` | Relations translation error | <0.01 | m |
| `relations_rot_mean` | Relations rotation error | <0.5 | deg |

### 3. Quality Metrics

| Metric | Description | Target | Unit |
|--------|-------------|--------|------|
| `loop_closure_precision` | Loop closure true positive rate | >0.9 | ratio |
| `loop_closure_recall` | Loop closure detection rate | >0.8 | ratio |
| `map_wall_count` | Detected wall cells in test room | >100 | cells |
| `map_floor_count` | Detected floor cells in test room | >1000 | cells |

### 4. Test Coverage

| Metric | Description |
|--------|-------------|
| `unit_tests_passed` | Number of unit tests passing |
| `integration_tests_passed` | Number of integration tests passing |
| `all_tests_pass` | Boolean: all tests pass |

---

## Benchmark History

### Entry: 2025-12-31

**Commit:** `1321540e164d6955c07759ce8a200a7313bcb57a`
**Message:** Simplify vastu-slam configuration and increase default grid size
**Platform:** macOS Darwin 25.2.0 (Apple Silicon)
**Rust:** nightly (portable_simd feature)

#### Performance Results

| Metric | Value | Change | Status |
|--------|-------|--------|--------|
| scan_matching_180 | 964.96 µs | -3.1% | ✅ improved |
| scan_matching_360 | 1793.7 µs | -2.9% | ✅ improved |
| scan_matching_720 | 6546.6 µs | -3.3% | ✅ improved |
| grid_update_180 | 104.96 µs | +0.3% | ⚪ stable |
| grid_update_360 | 209.88 µs | +0.4% | ⚪ stable |
| grid_update_720 | 418.05 µs | -0.5% | ⚪ stable |
| coverage_stats | 125.15 µs | -4.8% | ✅ improved |
| ate_100 | 1.51 µs | +0.1% | ⚪ stable |
| rpe_100 | 3.04 µs | -0.3% | ⚪ stable |
| relations_metrics_100 | 58.32 µs | +0.6% | ⚪ stable |
| ground_truth_gen_100 | 19.96 µs | -0.1% | ⚪ stable |

#### Throughput Analysis

| Operation | Rate | Real-time Factor |
|-----------|------|------------------|
| Scan matching (360pt) | 557 scans/sec | 111x @ 5Hz lidar |
| Grid update (360pt) | 4765 updates/sec | 953x @ 5Hz lidar |
| Combined (match + update) | 499 cycles/sec | 100x @ 5Hz lidar |

#### Test Results

| Category | Passed | Total | Status |
|----------|--------|-------|--------|
| Unit tests | 195 | 195 | ✅ |
| Accuracy tests | 10 | 10 | ✅ |
| Performance tests | 6 | 6 | ✅ |
| Quality tests | 10 | 10 | ✅ |
| **Total** | **221** | **221** | ✅ |

#### Notes

- First baseline entry after config simplification
- Scan matching improved ~3% (likely due to reduced config overhead)
- Coverage stats improved ~5% (SIMD optimization benefit)
- All accuracy/quality thresholds met

---

## Benchmark Procedures

### Running Benchmarks

```bash
# Full benchmark suite
cd vastu-slam
cargo +nightly bench

# Specific benchmark group
cargo +nightly bench --bench scan_matching
cargo +nightly bench --bench grid_operations
cargo +nightly bench --bench evaluation

# Save results to file
cargo +nightly bench 2>&1 | tee benchmark_output.txt
```

### Running Tests

```bash
# All tests (debug mode)
cargo test

# Release mode tests (faster)
cargo test --release

# Specific test category
cargo test --test accuracy
cargo test --test performance
cargo test --test quality
```

### Adding New Benchmark Entry

1. Run full benchmark suite: `cargo +nightly bench`
2. Run all tests: `cargo test`
3. Get commit info: `git rev-parse HEAD && git log -1 --format='%s'`
4. Add new entry section below with results
5. Calculate percentage changes from previous entry
6. Note any significant changes or regressions

### Regression Detection

A regression is flagged when:
- Scan matching latency increases >10%
- Grid update latency increases >20%
- Any accuracy test fails
- Any metric exceeds its target threshold

---

## Target Platform Performance

### ARM Cortex-A7 @ 1GHz (Allwinner A33)

Expected performance scaling from Apple Silicon:
- Scan matching: ~5-15ms (10x slower)
- Grid update: ~1-2ms (10x slower)
- Real-time factor: ~10-20x @ 5Hz lidar (still real-time capable)

### Memory Footprint

| Component | Size |
|-----------|------|
| Binary (stripped, musl) | ~350 KB |
| Grid storage (800x800) | ~2.5 MB |
| Submap (200x200 + 60 scans) | ~324 KB |
| Total runtime (typical) | <10 MB |

---

## Historical Trends

```
scan_matching_360 (µs)
├─ 2025-12-31: 1793.7 µs (baseline)
│
grid_update_360 (µs)
├─ 2025-12-31: 209.88 µs (baseline)
│
coverage_stats (µs)
├─ 2025-12-31: 125.15 µs (baseline)
```

*Add new entries above as benchmarks are run*
