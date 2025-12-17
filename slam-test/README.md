# SLAM Round-Trip Test Framework

Integration testing for DhruvaSLAM mapping accuracy against mock simulation ground truth.

## Quick Start

```bash
cd slam-test

# First time: compile protobuf
make proto

# Run with defaults (2 min duration)
make test

# Or run directly with custom options
./test_slam.sh --output ./results/mytest --duration 60
```

## Requirements

- Rust toolchain (for building sangam-io and dhruva-slam)
- Python 3.8+ with packages:

```bash
pip install protobuf numpy pillow matplotlib
```

- protoc (protocol buffer compiler):

```bash
# macOS
brew install protobuf

# Linux
apt install protobuf-compiler
```

## Available Commands

```bash
make proto       # Compile protobuf files (required first time)
make test        # Run test (2 min duration)
make test-quick  # Run quick test (30 sec)
make test-long   # Run long test (5 min)
make clean       # Clean results directory
```

## Manual Test Options

```bash
./test_slam.sh [OPTIONS]

Options:
  --output DIR     Output directory (default: results/TIMESTAMP)
  --duration SEC   Test duration in seconds (default: 120)
  --config FILE    DhruvaSLAM config file (default: dhruva-test.toml)
  --help           Show help message
```

## Structure

```
slam-test/
├── README.md                 # This file
├── Makefile                  # Build and test commands
├── test_slam.sh              # Main test orchestrator
├── test_config.toml          # Test parameters and thresholds
├── dhruva-test.toml          # SLAM config (resolution=0.05m)
├── round-trip-test-proposal.md  # Detailed design doc
├── scripts/
│   ├── send_command.py       # Send commands to DhruvaSLAM
│   ├── analyze_map.py        # Compare maps, generate reports
│   └── proto/
│       └── dhruva_pb2.py     # Generated protobuf (run make proto)
└── results/                  # Test outputs (gitignored)
```

## Test Flow

1. **Setup**: Start SangamIO (mock) and DhruvaSLAM
2. **Mapping**: Send StartMapping, robot explores autonomously
3. **Capture**: Stop mapping, save output map
4. **Analysis**: Compare against ground truth, generate reports

## Outputs

Each test run creates a timestamped directory with:

| File | Description |
|------|-------------|
| `sangamio.log` | SangamIO process logs |
| `dhruva.log` | DhruvaSLAM process logs |
| `ground_truth.pgm` | Input map copy |
| `slam_output.pgm` | SLAM generated map |
| `comparison.png` | Visual diff overlay |
| `analysis.json` | Numeric metrics |
| `summary.txt` | Human-readable results |

## Visual Diff Colors

- **White**: Free space (both agree)
- **Green**: Occupied (both agree - correct detection)
- **Red**: False positive (SLAM occupied, ground truth free)
- **Blue**: False negative (SLAM free, ground truth occupied)
- **Gray**: Unknown in SLAM output

## Success Criteria

| Metric | Pass | Warning |
|--------|------|---------|
| Occupied overlap | > 70% | > 60% |
| Free space overlap | > 85% | > 75% |
| Coverage ratio | > 75% | > 60% |

## Configuration

### Test Config (`test_config.toml`)

Test parameters and pass/fail thresholds.

### SLAM Config (`dhruva-test.toml`)

DhruvaSLAM configuration with **resolution matching ground truth** (0.05m = 5cm).

Key settings that must match `sangam-io/mock.toml`:
- `[map] resolution = 0.05`
- `[lidar]` mounting offsets
- `[odometry]` encoder parameters

## Troubleshooting

### "Proto files not compiled"
```bash
make proto
```

### "Could not connect to port 5557"
Check if DhruvaSLAM started:
```bash
tail -f results/*/dhruva.log
```

### "SangamIO failed to start"
Check if port 5555 is in use:
```bash
lsof -i :5555
```

## See Also

- [Detailed Proposal](round-trip-test-proposal.md)
- [DhruvaSLAM README](../dhruva-slam/README.md)
- [SangamIO Mock Guide](../sangam-io/docs/mock-device-guide.md)
