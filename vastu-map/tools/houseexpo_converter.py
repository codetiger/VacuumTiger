#!/usr/bin/env python3
"""
HouseExpo Dataset Converter

Converts HouseExpo floor plan images to ROS-compatible PGM+YAML maps.

HouseExpo format:
- Binary grayscale images (0=wall, 255=free)
- Resolution: 1 pixel = 1 cm
- ~35,000 floor plans from real Chinese apartments

This converter:
- Scales images from 1cm/px to 5cm/px (vastu-map standard)
- Adds wall borders for closed environments
- Optionally adds furniture obstacles
- Supports batch conversion with filtering

Usage:
    # Convert single image
    python houseexpo_converter.py dataset/0004d52d1aeeb8ae6de39d6bd993e992.png -o maps/

    # Batch convert with size filtering
    python houseexpo_converter.py dataset/ -o maps/ --min-size 5 --max-size 15 --count 50

    # Convert with furniture
    python houseexpo_converter.py dataset/ -o maps/ --add-furniture --count 20

Author: VacuumTiger Project
"""

import argparse
import os
import sys
from pathlib import Path
from typing import List, Tuple, Optional
import random
import yaml

try:
    import cv2
    import numpy as np
    from PIL import Image
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install with: pip install opencv-python numpy pillow pyyaml")
    sys.exit(1)


# HouseExpo resolution: 1 pixel = 1 cm = 0.01 m
HOUSEEXPO_RESOLUTION = 0.01  # meters per pixel

# Target resolution for vastu-map
TARGET_RESOLUTION = 0.05  # meters per pixel (5 cm)

# Scale factor: 1cm -> 5cm means divide by 5
SCALE_FACTOR = int(TARGET_RESOLUTION / HOUSEEXPO_RESOLUTION)


def load_houseexpo_image(path: str) -> Tuple[np.ndarray, float, float]:
    """
    Load HouseExpo image and return with physical dimensions.

    Returns:
        Tuple of (image_array, width_meters, height_meters)
    """
    img = Image.open(path).convert('L')
    arr = np.array(img)

    # Calculate physical size (1px = 1cm)
    width_m = img.size[0] * HOUSEEXPO_RESOLUTION
    height_m = img.size[1] * HOUSEEXPO_RESOLUTION

    return arr, width_m, height_m


def scale_to_target_resolution(img: np.ndarray) -> np.ndarray:
    """Scale image from 1cm/px to 5cm/px resolution."""
    h, w = img.shape
    new_w = w // SCALE_FACTOR
    new_h = h // SCALE_FACTOR

    # Use INTER_AREA for downscaling (best for reducing aliasing)
    scaled = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Re-binarize after scaling (threshold at 128)
    _, binary = cv2.threshold(scaled, 128, 255, cv2.THRESH_BINARY)

    return binary


def add_border(img: np.ndarray, thickness: int = 2) -> np.ndarray:
    """Add wall border around the map."""
    bordered = cv2.copyMakeBorder(
        img,
        thickness, thickness, thickness, thickness,
        cv2.BORDER_CONSTANT,
        value=0
    )
    return bordered


def clean_map(img: np.ndarray) -> np.ndarray:
    """Clean up the map with morphological operations."""
    kernel = np.ones((2, 2), np.uint8)

    # Close small gaps in walls
    closed = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

    # Remove small noise
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel)

    return opened


def add_furniture(
    img: np.ndarray,
    density: float = 0.05,
    seed: Optional[int] = None
) -> np.ndarray:
    """Add random furniture obstacles."""
    if seed is not None:
        np.random.seed(seed)

    result = img.copy()
    h, w = result.shape

    # Furniture templates (width, height in pixels at 5cm resolution)
    furniture = [
        (16, 20),   # 80cm x 100cm table
        (40, 16),   # 200cm x 80cm sofa
        (36, 40),   # 180cm x 200cm bed
        (24, 12),   # 120cm x 60cm desk
        (8, 8),     # 40cm x 40cm chair
    ]

    free_pixels = np.sum(result == 255)
    target_furniture = int(free_pixels * density)
    placed = 0
    attempts = 0

    while placed < target_furniture and attempts < 500:
        attempts += 1

        fw, fh = furniture[np.random.randint(len(furniture))]
        if np.random.random() > 0.5:
            fw, fh = fh, fw

        x = np.random.randint(4, max(5, w - fw - 4))
        y = np.random.randint(4, max(5, h - fh - 4))

        region = result[y:y+fh, x:x+fw]
        if region.size > 0 and np.all(region == 255):
            result[y:y+fh, x:x+fw] = 0
            placed += fw * fh

    return result


def convert_houseexpo(
    input_path: str,
    add_furniture_flag: bool = False,
    furniture_density: float = 0.05,
    furniture_seed: Optional[int] = None,
) -> Tuple[np.ndarray, dict]:
    """
    Convert a HouseExpo image to vastu-map format.

    Returns:
        Tuple of (pgm_array, metadata_dict)
    """
    # Load original image
    img, width_m, height_m = load_houseexpo_image(input_path)

    # Scale to 5cm resolution
    scaled = scale_to_target_resolution(img)

    # Clean up
    cleaned = clean_map(scaled)

    # Add border
    bordered = add_border(cleaned, thickness=2)

    # Add furniture if requested
    if add_furniture_flag:
        bordered = add_furniture(bordered, furniture_density, furniture_seed)

    # Calculate final dimensions
    h, w = bordered.shape
    final_width_m = w * TARGET_RESOLUTION
    final_height_m = h * TARGET_RESOLUTION

    metadata = {
        'resolution': TARGET_RESOLUTION,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'source': 'HouseExpo',
        'original_size_m': {
            'width': width_m,
            'height': height_m,
        },
        'physical_size': {
            'width_m': final_width_m,
            'height_m': final_height_m,
        },
    }

    return bordered, metadata


def save_pgm(img: np.ndarray, output_path: str, comment: str = ""):
    """Save image as ASCII PGM (P2 format)."""
    h, w = img.shape
    with open(output_path, 'w') as f:
        f.write("P2\n")
        if comment:
            for line in comment.split('\n'):
                f.write(f"# {line}\n")
        f.write(f"{w} {h}\n")
        f.write("255\n")
        for row in img:
            f.write(' '.join(map(str, row)) + '\n')


def save_yaml(metadata: dict, output_path: str, pgm_filename: str):
    """Save YAML metadata file."""
    yaml_data = {
        'image': pgm_filename,
        'resolution': metadata['resolution'],
        'origin': metadata['origin'],
        'negate': metadata['negate'],
        'occupied_thresh': metadata['occupied_thresh'],
        'free_thresh': metadata['free_thresh'],
    }
    with open(output_path, 'w') as f:
        f.write(f"# HouseExpo floor plan: {pgm_filename}\n")
        f.write(f"# Original size: {metadata['original_size_m']['width']:.1f}m x {metadata['original_size_m']['height']:.1f}m\n")
        f.write(f"# Scaled size: {metadata['physical_size']['width_m']:.1f}m x {metadata['physical_size']['height_m']:.1f}m\n")
        yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)


def get_image_size(path: str) -> Tuple[float, float]:
    """Get physical size of HouseExpo image in meters."""
    img = Image.open(path)
    return img.size[0] * HOUSEEXPO_RESOLUTION, img.size[1] * HOUSEEXPO_RESOLUTION


def filter_by_size(
    files: List[str],
    input_dir: str,
    min_size: float,
    max_size: float,
) -> List[str]:
    """Filter images by physical size (in meters)."""
    filtered = []
    for f in files:
        try:
            w, h = get_image_size(os.path.join(input_dir, f))
            # Check if both dimensions are within range
            if min_size <= w <= max_size and min_size <= h <= max_size:
                filtered.append(f)
        except Exception:
            continue
    return filtered


def main():
    parser = argparse.ArgumentParser(
        description='Convert HouseExpo floor plans to vastu-map PGM format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
HouseExpo Dataset Info:
- ~35,000 real floor plans from Chinese apartments
- Resolution: 1 pixel = 1 cm
- Binary format: 0=wall (black), 255=free (white)

Examples:
  # Convert a single floor plan
  python houseexpo_converter.py dataset/0004d52d1aeeb8ae6de39d6bd993e992.png -o maps/

  # Batch convert 50 medium-sized apartments (5-15m)
  python houseexpo_converter.py dataset/ -o maps/ --min-size 5 --max-size 15 --count 50

  # Convert with furniture for exploration testing
  python houseexpo_converter.py dataset/ -o maps/ --add-furniture --count 20

  # Convert all apartments between 8-12m
  python houseexpo_converter.py dataset/ -o maps/ --min-size 8 --max-size 12
        """
    )

    parser.add_argument('input', help='Input PNG file or directory of HouseExpo images')
    parser.add_argument('--output', '-o', required=True, help='Output directory')
    parser.add_argument('--min-size', type=float, default=0,
                        help='Minimum dimension in meters (filter)')
    parser.add_argument('--max-size', type=float, default=100,
                        help='Maximum dimension in meters (filter)')
    parser.add_argument('--count', '-n', type=int, default=0,
                        help='Maximum number of maps to convert (0=all)')
    parser.add_argument('--add-furniture', '-f', action='store_true',
                        help='Add furniture obstacles')
    parser.add_argument('--furniture-density', type=float, default=0.05,
                        help='Furniture density 0-1 (default: 0.05)')
    parser.add_argument('--seed', '-s', type=int,
                        help='Random seed for reproducibility')
    parser.add_argument('--prefix', '-p', default='houseexpo',
                        help='Output filename prefix (default: houseexpo)')
    parser.add_argument('--shuffle', action='store_true',
                        help='Shuffle file order before selecting')

    args = parser.parse_args()

    # Create output directory
    os.makedirs(args.output, exist_ok=True)

    input_path = Path(args.input)

    if input_path.is_file():
        # Single file conversion
        files = [input_path.name]
        input_dir = str(input_path.parent)
    else:
        # Directory batch conversion
        input_dir = str(input_path)
        files = [f for f in os.listdir(input_dir) if f.endswith('.png')]
        files.sort()

        # Filter by size
        if args.min_size > 0 or args.max_size < 100:
            print(f"Filtering by size: {args.min_size}m - {args.max_size}m...")
            files = filter_by_size(files, input_dir, args.min_size, args.max_size)
            print(f"Found {len(files)} matching files")

        # Shuffle if requested
        if args.shuffle:
            random.seed(args.seed)
            random.shuffle(files)

        # Limit count
        if args.count > 0:
            files = files[:args.count]

    if not files:
        print("No files to convert")
        return

    print(f"Converting {len(files)} HouseExpo floor plan(s)...")

    for i, filename in enumerate(files):
        input_file = os.path.join(input_dir, filename)

        try:
            # Convert
            pgm, metadata = convert_houseexpo(
                input_file,
                add_furniture_flag=args.add_furniture,
                furniture_density=args.furniture_density,
                furniture_seed=args.seed + i if args.seed else None,
            )

            # Generate output names
            base_name = Path(filename).stem
            if len(files) > 1:
                output_name = f"{args.prefix}_{i:04d}"
            else:
                output_name = f"{args.prefix}_{base_name[:8]}"

            pgm_path = os.path.join(args.output, f"{output_name}.pgm")
            yaml_path = os.path.join(args.output, f"{output_name}.yaml")

            # Save
            comment = f"HouseExpo: {filename}\n"
            comment += f"Size: {metadata['physical_size']['width_m']:.1f}m x {metadata['physical_size']['height_m']:.1f}m"

            save_pgm(pgm, pgm_path, comment)
            save_yaml(metadata, yaml_path, f"{output_name}.pgm")

            h, w = pgm.shape
            print(f"[{i+1}/{len(files)}] {filename} -> {output_name}.pgm "
                  f"({w}x{h}px, {metadata['physical_size']['width_m']:.1f}x{metadata['physical_size']['height_m']:.1f}m)")

        except Exception as e:
            print(f"[{i+1}/{len(files)}] Error converting {filename}: {e}")

    print(f"\nDone! Converted {len(files)} maps to {args.output}/")


if __name__ == '__main__':
    main()
