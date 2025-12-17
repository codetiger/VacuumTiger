#!/usr/bin/env python3
"""
Analyze SLAM output map against ground truth.

Generates:
- comparison.png: Visual diff overlay
- trajectory.png: Robot path (if available)
- analysis.json: Numeric metrics
- summary.txt: Human-readable summary
"""

import argparse
import json
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
from PIL import Image

# Try to import matplotlib for visualization
try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not available, skipping visualizations")


@dataclass
class MapData:
    """Loaded map data."""
    pixels: np.ndarray  # 2D array of pixel values (0-255)
    resolution: float   # meters per pixel
    origin_x: float     # world X of pixel (0,0)
    origin_y: float     # world Y of pixel (0,0)
    width: int
    height: int


@dataclass
class ComparisonResult:
    """Map comparison metrics."""
    # Cell counts
    total_gt_occupied: int
    total_gt_free: int
    total_slam_occupied: int
    total_slam_free: int
    total_slam_unknown: int

    # Overlap metrics
    true_positive: int   # Both say occupied
    true_negative: int   # Both say free
    false_positive: int  # SLAM occupied, GT free
    false_negative: int  # SLAM free, GT occupied

    # Computed metrics
    occupied_overlap_pct: float
    free_overlap_pct: float
    false_positive_pct: float
    false_negative_pct: float

    # Coverage
    explored_cells: int
    coverage_ratio: float


def load_pgm(path: Path, resolution: float = 0.05,
             origin_x: float = 0.0, origin_y: float = 0.0) -> MapData:
    """Load PGM map file."""
    img = Image.open(path).convert('L')
    pixels = np.array(img)

    return MapData(
        pixels=pixels,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        width=pixels.shape[1],
        height=pixels.shape[0]
    )


def load_yaml_metadata(yaml_path: Path) -> dict:
    """Load map metadata from YAML file."""
    metadata = {}
    try:
        import yaml
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
            if 'resolution' in data:
                metadata['resolution'] = float(data['resolution'])
            if 'origin' in data:
                origin = data['origin']
                if isinstance(origin, list) and len(origin) >= 2:
                    metadata['origin_x'] = float(origin[0])
                    metadata['origin_y'] = float(origin[1])
    except ImportError:
        # Fallback to simple parsing if PyYAML not available
        try:
            with open(yaml_path) as f:
                for line in f:
                    if ':' in line:
                        key, value = line.split(':', 1)
                        key = key.strip()
                        value = value.strip()
                        if key == 'resolution':
                            metadata['resolution'] = float(value)
                        elif key == 'origin' and value:
                            # Parse [x, y, theta] format
                            value = value.strip('[]')
                            parts = [float(x.strip()) for x in value.split(',')]
                            metadata['origin_x'] = parts[0]
                            metadata['origin_y'] = parts[1]
        except Exception as e:
            print(f"Warning: Could not parse YAML metadata: {e}")
    except Exception as e:
        print(f"Warning: Could not parse YAML metadata: {e}")
    return metadata


def classify_pixel(value: int) -> str:
    """Classify pixel as free, occupied, or unknown."""
    if value >= 240:  # White = free
        return 'free'
    elif value <= 20:  # Black = occupied
        return 'occupied'
    else:  # Gray = unknown
        return 'unknown'


def align_maps(ground_truth: MapData, slam_map: MapData,
               robot_start: Tuple[float, float, float]) -> Tuple[np.ndarray, np.ndarray]:
    """
    Align SLAM map to ground truth using robot start position.

    The SLAM system builds maps relative to the robot's starting position,
    treating it as (0,0) in SLAM coordinates. The robot_start parameter
    provides the robot's actual starting position in world coordinates,
    allowing us to translate between coordinate systems:

    - SLAM (0,0) = World (robot_start.x, robot_start.y)
    - SLAM origin (-10,-10) in SLAM coords = World origin + robot_start

    Returns aligned arrays of same dimensions as ground truth.
    """
    # Robot start position in world coordinates
    start_x, start_y, _ = robot_start

    # Convert SLAM origin from SLAM-local to world coordinates
    # SLAM's (0,0) corresponds to world's (start_x, start_y)
    slam_world_origin_x = slam_map.origin_x + start_x
    slam_world_origin_y = slam_map.origin_y + start_y

    # Create output array matching ground truth dimensions
    aligned_slam = np.full_like(ground_truth.pixels, 128, dtype=np.uint8)  # Unknown

    # For each pixel in ground truth, find corresponding SLAM pixel
    for gt_py in range(ground_truth.height):
        for gt_px in range(ground_truth.width):
            # Ground truth pixel to world coordinates
            # Note: PGM has Y=0 at top, but world has Y=0 at bottom
            world_x = ground_truth.origin_x + gt_px * ground_truth.resolution
            world_y = ground_truth.origin_y + (ground_truth.height - 1 - gt_py) * ground_truth.resolution

            # World to SLAM pixel coordinates (using world-adjusted origin)
            slam_px = int((world_x - slam_world_origin_x) / slam_map.resolution)
            slam_py = slam_map.height - 1 - int((world_y - slam_world_origin_y) / slam_map.resolution)

            # Check bounds
            if 0 <= slam_px < slam_map.width and 0 <= slam_py < slam_map.height:
                aligned_slam[gt_py, gt_px] = slam_map.pixels[slam_py, slam_px]

    return ground_truth.pixels, aligned_slam


def compare_maps(gt_pixels: np.ndarray, slam_pixels: np.ndarray) -> ComparisonResult:
    """Compare aligned maps and compute metrics."""
    # Classify all pixels
    gt_free = gt_pixels >= 240
    gt_occupied = gt_pixels <= 20

    slam_free = slam_pixels >= 240
    slam_occupied = slam_pixels <= 20
    slam_unknown = (slam_pixels > 20) & (slam_pixels < 240)

    # Count cells
    total_gt_occupied = np.sum(gt_occupied)
    total_gt_free = np.sum(gt_free)
    total_slam_occupied = np.sum(slam_occupied)
    total_slam_free = np.sum(slam_free)
    total_slam_unknown = np.sum(slam_unknown)

    # Compute overlaps (only where SLAM has data)
    slam_has_data = ~slam_unknown

    true_positive = np.sum(gt_occupied & slam_occupied)
    true_negative = np.sum(gt_free & slam_free)
    false_positive = np.sum(gt_free & slam_occupied)
    false_negative = np.sum(gt_occupied & slam_free)

    # Compute percentages
    if total_gt_occupied > 0:
        occupied_overlap_pct = 100.0 * true_positive / total_gt_occupied
        false_negative_pct = 100.0 * false_negative / total_gt_occupied
    else:
        occupied_overlap_pct = 0.0
        false_negative_pct = 0.0

    if total_gt_free > 0:
        free_overlap_pct = 100.0 * true_negative / total_gt_free
        false_positive_pct = 100.0 * false_positive / total_gt_free
    else:
        free_overlap_pct = 0.0
        false_positive_pct = 0.0

    # Coverage ratio
    explored_cells = np.sum(slam_has_data)
    total_navigable = total_gt_free + total_gt_occupied
    coverage_ratio = explored_cells / total_navigable if total_navigable > 0 else 0.0

    return ComparisonResult(
        total_gt_occupied=int(total_gt_occupied),
        total_gt_free=int(total_gt_free),
        total_slam_occupied=int(total_slam_occupied),
        total_slam_free=int(total_slam_free),
        total_slam_unknown=int(total_slam_unknown),
        true_positive=int(true_positive),
        true_negative=int(true_negative),
        false_positive=int(false_positive),
        false_negative=int(false_negative),
        occupied_overlap_pct=occupied_overlap_pct,
        free_overlap_pct=free_overlap_pct,
        false_positive_pct=false_positive_pct,
        false_negative_pct=false_negative_pct,
        explored_cells=int(explored_cells),
        coverage_ratio=coverage_ratio
    )


def create_diff_image(gt_pixels: np.ndarray, slam_pixels: np.ndarray,
                      output_path: Path) -> None:
    """Create color-coded diff image."""
    if not HAS_MATPLOTLIB:
        return

    height, width = gt_pixels.shape
    diff_rgb = np.zeros((height, width, 3), dtype=np.uint8)

    # Classify pixels
    gt_free = gt_pixels >= 240
    gt_occupied = gt_pixels <= 20

    slam_free = slam_pixels >= 240
    slam_occupied = slam_pixels <= 20
    slam_unknown = (slam_pixels > 20) & (slam_pixels < 240)

    # White: Both free
    both_free = gt_free & slam_free
    diff_rgb[both_free] = [255, 255, 255]

    # Green: Both occupied (correct detection)
    both_occupied = gt_occupied & slam_occupied
    diff_rgb[both_occupied] = [0, 200, 0]

    # Red: False positive (SLAM occupied, GT free)
    false_pos = gt_free & slam_occupied
    diff_rgb[false_pos] = [255, 0, 0]

    # Blue: False negative (SLAM free, GT occupied)
    false_neg = gt_occupied & slam_free
    diff_rgb[false_neg] = [0, 0, 255]

    # Gray: Unknown in SLAM
    diff_rgb[slam_unknown] = [180, 180, 180]

    # Light gray: GT unknown (neither free nor occupied)
    gt_unknown = ~gt_free & ~gt_occupied
    diff_rgb[gt_unknown & ~slam_unknown] = [220, 220, 220]

    # Save image
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.imshow(diff_rgb)
    ax.set_title("Map Comparison: SLAM vs Ground Truth")
    ax.axis('off')

    # Legend
    legend_elements = [
        mpatches.Patch(color='white', label='Free (both agree)'),
        mpatches.Patch(color='green', label='Occupied (both agree)'),
        mpatches.Patch(color='red', label='False positive'),
        mpatches.Patch(color='blue', label='False negative'),
        mpatches.Patch(color='gray', label='Unknown (SLAM)'),
    ]
    ax.legend(handles=legend_elements, loc='upper right')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  Saved: {output_path}")


def write_analysis_json(result: ComparisonResult, resolution: float,
                        duration: int, output_path: Path) -> None:
    """Write analysis results to JSON."""
    data = {
        "test_info": {
            "timestamp": datetime.now().isoformat(),
            "duration_seconds": duration,
            "resolution_m": resolution
        },
        "geometric_accuracy": {
            "occupied_overlap_pct": round(result.occupied_overlap_pct, 2),
            "free_overlap_pct": round(result.free_overlap_pct, 2),
            "false_positive_pct": round(result.false_positive_pct, 2),
            "false_negative_pct": round(result.false_negative_pct, 2)
        },
        "cell_counts": {
            "gt_occupied": result.total_gt_occupied,
            "gt_free": result.total_gt_free,
            "slam_occupied": result.total_slam_occupied,
            "slam_free": result.total_slam_free,
            "slam_unknown": result.total_slam_unknown
        },
        "coverage": {
            "explored_cells": result.explored_cells,
            "coverage_ratio": round(result.coverage_ratio, 3)
        },
        "confusion_matrix": {
            "true_positive": result.true_positive,
            "true_negative": result.true_negative,
            "false_positive": result.false_positive,
            "false_negative": result.false_negative
        }
    }

    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"  Saved: {output_path}")


def write_summary(result: ComparisonResult, resolution: float,
                  duration: int, output_path: Path) -> None:
    """Write human-readable summary."""
    # Determine pass/fail
    pass_threshold = 70.0
    status = "PASS" if result.occupied_overlap_pct >= pass_threshold else "FAIL"

    area_m2 = result.explored_cells * (resolution ** 2)

    lines = [
        "SLAM Round-Trip Test Summary",
        "=" * 40,
        f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"Duration: {duration} seconds",
        f"Result: {status} ({result.occupied_overlap_pct:.1f}% overlap)",
        "",
        "Accuracy Metrics:",
        f"  - Occupied overlap: {result.occupied_overlap_pct:.1f}%",
        f"  - Free space overlap: {result.free_overlap_pct:.1f}%",
        f"  - False positive rate: {result.false_positive_pct:.1f}%",
        f"  - False negative rate: {result.false_negative_pct:.1f}%",
        "",
        "Coverage:",
        f"  - Explored area: {area_m2:.2f} m²",
        f"  - Coverage ratio: {result.coverage_ratio * 100:.1f}%",
        "",
        "Cell Counts:",
        f"  - Ground truth occupied: {result.total_gt_occupied}",
        f"  - Ground truth free: {result.total_gt_free}",
        f"  - SLAM occupied: {result.total_slam_occupied}",
        f"  - SLAM free: {result.total_slam_free}",
        f"  - SLAM unknown: {result.total_slam_unknown}",
    ]

    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))
    print(f"  Saved: {output_path}")

    # Also print to console
    print("")
    for line in lines:
        print(line)


def main():
    parser = argparse.ArgumentParser(description="Analyze SLAM map vs ground truth")
    parser.add_argument("--ground-truth", required=True, help="Ground truth PGM file")
    parser.add_argument("--slam-output", required=True, help="SLAM output PGM file")
    parser.add_argument("--start-pose", default="1.5,3.5,0",
                        help="Robot start pose: x,y,theta")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="Map resolution (meters/pixel)")
    parser.add_argument("--output-dir", required=True, help="Output directory")
    parser.add_argument("--duration", type=int, default=120,
                        help="Test duration for reporting")

    args = parser.parse_args()

    gt_path = Path(args.ground_truth)
    slam_path = Path(args.slam_output)
    output_dir = Path(args.output_dir)

    # Parse robot start pose
    start_parts = [float(x) for x in args.start_pose.split(',')]
    robot_start = (start_parts[0], start_parts[1], start_parts[2] if len(start_parts) > 2 else 0.0)

    print("Loading maps...")

    # Load ground truth
    if not gt_path.exists():
        print(f"ERROR: Ground truth not found: {gt_path}")
        sys.exit(1)

    gt_yaml = gt_path.with_suffix('.yaml')
    gt_meta = load_yaml_metadata(gt_yaml) if gt_yaml.exists() else {}
    gt_resolution = gt_meta.get('resolution', args.resolution)
    gt_origin_x = gt_meta.get('origin_x', 0.0)
    gt_origin_y = gt_meta.get('origin_y', 0.0)

    ground_truth = load_pgm(gt_path, gt_resolution, gt_origin_x, gt_origin_y)
    print(f"  Ground truth: {ground_truth.width}x{ground_truth.height} @ {gt_resolution}m/px")

    # Load SLAM output
    if not slam_path.exists():
        print(f"ERROR: SLAM output not found: {slam_path}")
        sys.exit(1)

    slam_yaml = slam_path.with_suffix('.yaml')
    slam_meta = load_yaml_metadata(slam_yaml) if slam_yaml.exists() else {}
    slam_resolution = slam_meta.get('resolution', args.resolution)
    slam_origin_x = slam_meta.get('origin_x', robot_start[0])
    slam_origin_y = slam_meta.get('origin_y', robot_start[1])

    slam_map = load_pgm(slam_path, slam_resolution, slam_origin_x, slam_origin_y)
    print(f"  SLAM output: {slam_map.width}x{slam_map.height} @ {slam_resolution}m/px")

    print("Aligning maps...")
    gt_aligned, slam_aligned = align_maps(ground_truth, slam_map, robot_start)

    print("Comparing maps...")
    result = compare_maps(gt_aligned, slam_aligned)

    print("Generating outputs...")
    output_dir.mkdir(parents=True, exist_ok=True)

    # Create diff image
    create_diff_image(gt_aligned, slam_aligned, output_dir / "comparison.png")

    # Write JSON report
    write_analysis_json(result, args.resolution, args.duration,
                        output_dir / "analysis.json")

    # Write summary
    write_summary(result, args.resolution, args.duration,
                  output_dir / "summary.txt")


if __name__ == "__main__":
    main()
