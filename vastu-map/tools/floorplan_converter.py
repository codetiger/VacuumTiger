#!/usr/bin/env python3
"""
Floor Plan to PGM Converter

Converts floor plan images (PNG, JPG, SVG, PDF) to ROS-compatible PGM+YAML maps
for use with vastu-map SLAM testing.

Usage:
    python floorplan_converter.py input.png --width 10.0 --output apartment.pgm
    python floorplan_converter.py input.jpg --resolution 0.05 --output test_map.pgm
    python floorplan_converter.py input.png --width 15.0 --add-furniture --output furnished.pgm

Dependencies:
    pip install opencv-python numpy pillow pyyaml

Author: VacuumTiger Project
"""

import argparse
import os
import sys
from pathlib import Path
from typing import Tuple, Optional, List
import yaml

try:
    import cv2
    import numpy as np
    from PIL import Image
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install with: pip install opencv-python numpy pillow pyyaml")
    sys.exit(1)


class FloorPlanConverter:
    """Convert floor plan images to ROS-compatible PGM+YAML format."""

    def __init__(
        self,
        resolution: float = 0.05,
        wall_thickness: int = 2,
        occupied_thresh: float = 0.65,
        free_thresh: float = 0.196,
        invert_colors: bool = False,
    ):
        """
        Initialize converter.

        Args:
            resolution: Meters per pixel (default 0.05m = 5cm)
            wall_thickness: Minimum wall thickness in pixels after processing
            occupied_thresh: Threshold for occupied space (0-1, normalized)
            free_thresh: Threshold for free space (0-1, normalized)
            invert_colors: If True, treat dark areas as free space
        """
        self.resolution = resolution
        self.wall_thickness = wall_thickness
        self.occupied_thresh = occupied_thresh
        self.free_thresh = free_thresh
        self.invert_colors = invert_colors

    def load_image(self, path: str) -> np.ndarray:
        """Load image from various formats."""
        path = Path(path)

        if not path.exists():
            raise FileNotFoundError(f"Image not found: {path}")

        # Handle different formats
        suffix = path.suffix.lower()

        if suffix in ['.png', '.jpg', '.jpeg', '.bmp', '.tiff', '.tif']:
            img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if img is None:
                raise ValueError(f"Failed to load image: {path}")
            return img

        elif suffix == '.svg':
            # SVG requires cairosvg or similar
            try:
                import cairosvg
                from io import BytesIO
                png_data = cairosvg.svg2png(url=str(path))
                pil_img = Image.open(BytesIO(png_data)).convert('L')
                return np.array(pil_img)
            except ImportError:
                raise ImportError("SVG support requires: pip install cairosvg")

        elif suffix == '.pdf':
            # PDF requires pdf2image
            try:
                from pdf2image import convert_from_path
                images = convert_from_path(str(path), dpi=300)
                if images:
                    return np.array(images[0].convert('L'))
                raise ValueError("No pages in PDF")
            except ImportError:
                raise ImportError("PDF support requires: pip install pdf2image")

        else:
            raise ValueError(f"Unsupported format: {suffix}")

    def preprocess(self, img: np.ndarray) -> np.ndarray:
        """Apply preprocessing to enhance wall detection."""
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(img, (5, 5), 0)

        # Apply adaptive thresholding for better wall detection
        # This works well with varying lighting in floor plans
        binary = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            blockSize=11,
            C=2
        )

        return binary

    def detect_walls_otsu(self, img: np.ndarray) -> np.ndarray:
        """Detect walls using Otsu's thresholding."""
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return binary

    def detect_walls_adaptive(self, img: np.ndarray) -> np.ndarray:
        """Detect walls using adaptive thresholding."""
        blurred = cv2.GaussianBlur(img, (3, 3), 0)
        binary = cv2.adaptiveThreshold(
            blurred, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            blockSize=15,
            C=3
        )
        return binary

    def detect_walls_canny(self, img: np.ndarray) -> np.ndarray:
        """Detect walls using Canny edge detection + morphology."""
        # Edge detection
        edges = cv2.Canny(img, 50, 150)

        # Dilate to connect nearby edges
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(edges, kernel, iterations=2)

        # Fill holes
        filled = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel, iterations=3)

        # Invert so walls are black (0)
        return 255 - filled

    def clean_walls(self, binary: np.ndarray) -> np.ndarray:
        """Clean up detected walls with morphological operations."""
        kernel = np.ones((self.wall_thickness, self.wall_thickness), np.uint8)

        # Remove small noise
        opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # Fill small gaps in walls
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

        return closed

    def scale_to_resolution(
        self,
        img: np.ndarray,
        target_width_m: Optional[float] = None,
        target_height_m: Optional[float] = None,
    ) -> np.ndarray:
        """
        Scale image to match desired physical dimensions.

        Args:
            img: Input image
            target_width_m: Target width in meters (None to use height ratio)
            target_height_m: Target height in meters (None to use width ratio)

        Returns:
            Scaled image where each pixel = self.resolution meters
        """
        h, w = img.shape[:2]

        if target_width_m is not None:
            target_width_px = int(target_width_m / self.resolution)
            scale = target_width_px / w
            target_height_px = int(h * scale)
        elif target_height_m is not None:
            target_height_px = int(target_height_m / self.resolution)
            scale = target_height_px / h
            target_width_px = int(w * scale)
        else:
            # Keep original size, assume source resolution matches
            return img

        return cv2.resize(img, (target_width_px, target_height_px), interpolation=cv2.INTER_NEAREST)

    def add_border(self, img: np.ndarray, border_px: int = 4) -> np.ndarray:
        """Add solid wall border around the map."""
        # 0 = occupied (wall), 255 = free
        bordered = cv2.copyMakeBorder(
            img,
            border_px, border_px, border_px, border_px,
            cv2.BORDER_CONSTANT,
            value=0
        )
        return bordered

    def add_furniture(
        self,
        img: np.ndarray,
        furniture_density: float = 0.1,
        seed: Optional[int] = None
    ) -> np.ndarray:
        """
        Add random furniture obstacles to the map.

        Args:
            img: Input map (255=free, 0=occupied)
            furniture_density: Fraction of free space to fill with furniture
            seed: Random seed for reproducibility
        """
        if seed is not None:
            np.random.seed(seed)

        result = img.copy()
        h, w = result.shape

        # Find free space pixels
        free_mask = result == 255
        free_pixels = np.sum(free_mask)

        # Common furniture pieces (width, height in pixels at 5cm resolution)
        furniture_templates = [
            ('table', (16, 20)),      # 80cm x 100cm dining table
            ('sofa', (40, 16)),       # 200cm x 80cm sofa
            ('bed', (36, 40)),        # 180cm x 200cm bed
            ('desk', (24, 12)),       # 120cm x 60cm desk
            ('chair', (8, 8)),        # 40cm x 40cm chair
            ('wardrobe', (24, 12)),   # 120cm x 60cm wardrobe
            ('nightstand', (8, 8)),   # 40cm x 40cm nightstand
            ('bookshelf', (16, 6)),   # 80cm x 30cm bookshelf
        ]

        target_furniture_pixels = int(free_pixels * furniture_density)
        placed_pixels = 0
        attempts = 0
        max_attempts = 1000

        while placed_pixels < target_furniture_pixels and attempts < max_attempts:
            attempts += 1

            # Pick random furniture
            name, (fw, fh) = furniture_templates[np.random.randint(len(furniture_templates))]

            # Random rotation (0 or 90 degrees)
            if np.random.random() > 0.5:
                fw, fh = fh, fw

            # Random position
            x = np.random.randint(0, max(1, w - fw))
            y = np.random.randint(0, max(1, h - fh))

            # Check if area is free (with margin)
            margin = 2
            x1, y1 = max(0, x - margin), max(0, y - margin)
            x2, y2 = min(w, x + fw + margin), min(h, y + fh + margin)

            region = result[y1:y2, x1:x2]
            if np.all(region == 255):
                # Place furniture (walls = 0)
                result[y:y+fh, x:x+fw] = 0
                placed_pixels += fw * fh

        return result

    def convert(
        self,
        input_path: str,
        width_m: Optional[float] = None,
        height_m: Optional[float] = None,
        detection_method: str = 'adaptive',
        add_border: bool = True,
        add_furniture: bool = False,
        furniture_density: float = 0.05,
        furniture_seed: Optional[int] = None,
    ) -> Tuple[np.ndarray, dict]:
        """
        Convert floor plan image to occupancy map.

        Args:
            input_path: Path to input image
            width_m: Physical width in meters
            height_m: Physical height in meters
            detection_method: 'adaptive', 'otsu', or 'canny'
            add_border: Add wall border around map
            add_furniture: Add random furniture obstacles
            furniture_density: Density of furniture (0-1)
            furniture_seed: Random seed for furniture placement

        Returns:
            Tuple of (pgm_array, yaml_metadata)
        """
        # Load image
        img = self.load_image(input_path)

        # Invert if needed (some floor plans have white walls)
        if self.invert_colors:
            img = 255 - img

        # Detect walls
        if detection_method == 'adaptive':
            binary = self.detect_walls_adaptive(img)
        elif detection_method == 'otsu':
            binary = self.detect_walls_otsu(img)
        elif detection_method == 'canny':
            binary = self.detect_walls_canny(img)
        else:
            raise ValueError(f"Unknown detection method: {detection_method}")

        # Clean up
        cleaned = self.clean_walls(binary)

        # Scale to physical dimensions
        scaled = self.scale_to_resolution(cleaned, width_m, height_m)

        # Add border
        if add_border:
            scaled = self.add_border(scaled, border_px=2)

        # Add furniture
        if add_furniture:
            scaled = self.add_furniture(scaled, furniture_density, furniture_seed)

        # Calculate physical dimensions
        h, w = scaled.shape
        physical_width = w * self.resolution
        physical_height = h * self.resolution

        # Create YAML metadata
        metadata = {
            'resolution': self.resolution,
            'origin': [0.0, 0.0, 0.0],
            'negate': 0,
            'occupied_thresh': self.occupied_thresh,
            'free_thresh': self.free_thresh,
            'physical_size': {
                'width_m': physical_width,
                'height_m': physical_height,
            },
        }

        return scaled, metadata

    def save_pgm(self, img: np.ndarray, output_path: str, comment: str = ""):
        """Save image as ASCII PGM (P2 format)."""
        h, w = img.shape

        with open(output_path, 'w') as f:
            f.write("P2\n")
            if comment:
                # Split comment into multiple lines if needed
                for line in comment.split('\n'):
                    f.write(f"# {line}\n")
            f.write(f"{w} {h}\n")
            f.write("255\n")

            for row in img:
                f.write(' '.join(map(str, row)) + '\n')

    def save_yaml(self, metadata: dict, output_path: str, pgm_filename: str):
        """Save YAML metadata file."""
        # Create ROS-compatible YAML
        yaml_data = {
            'image': pgm_filename,
            'resolution': metadata['resolution'],
            'origin': metadata['origin'],
            'negate': metadata['negate'],
            'occupied_thresh': metadata['occupied_thresh'],
            'free_thresh': metadata['free_thresh'],
        }

        with open(output_path, 'w') as f:
            # Add comment header
            f.write(f"# Map: {pgm_filename}\n")
            f.write(f"# Physical size: {metadata['physical_size']['width_m']:.1f}m x {metadata['physical_size']['height_m']:.1f}m\n")
            yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)


def main():
    parser = argparse.ArgumentParser(
        description='Convert floor plan images to ROS-compatible PGM+YAML maps',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert with specific physical width (height calculated proportionally)
  python floorplan_converter.py apartment.png --width 12.0 --output apartment.pgm

  # Convert with both dimensions specified
  python floorplan_converter.py house.jpg --width 15.0 --height 10.0 --output house.pgm

  # Add furniture obstacles for exploration testing
  python floorplan_converter.py layout.png --width 8.0 --add-furniture --output furnished.pgm

  # Use Canny edge detection for blueprints with thin lines
  python floorplan_converter.py blueprint.png --width 20.0 --method canny --output blueprint.pgm

  # Invert colors (for floor plans with white walls)
  python floorplan_converter.py inverted.png --width 10.0 --invert --output map.pgm
        """
    )

    parser.add_argument('input', help='Input floor plan image (PNG, JPG, SVG, PDF)')
    parser.add_argument('--output', '-o', required=True, help='Output PGM file path')
    parser.add_argument('--width', '-w', type=float, help='Physical width in meters')
    parser.add_argument('--height', '-H', type=float, help='Physical height in meters')
    parser.add_argument('--resolution', '-r', type=float, default=0.05,
                        help='Meters per pixel (default: 0.05)')
    parser.add_argument('--method', '-m', choices=['adaptive', 'otsu', 'canny'],
                        default='adaptive', help='Wall detection method (default: adaptive)')
    parser.add_argument('--invert', '-i', action='store_true',
                        help='Invert colors (treat dark as free space)')
    parser.add_argument('--no-border', action='store_true',
                        help='Do not add wall border around map')
    parser.add_argument('--add-furniture', '-f', action='store_true',
                        help='Add random furniture obstacles')
    parser.add_argument('--furniture-density', type=float, default=0.05,
                        help='Furniture density 0-1 (default: 0.05)')
    parser.add_argument('--furniture-seed', type=int,
                        help='Random seed for furniture placement')
    parser.add_argument('--wall-thickness', type=int, default=2,
                        help='Minimum wall thickness in pixels (default: 2)')
    parser.add_argument('--preview', '-p', action='store_true',
                        help='Show preview window (requires display)')

    args = parser.parse_args()

    # Validate input
    if args.width is None and args.height is None:
        parser.error("At least one of --width or --height must be specified")

    # Create converter
    converter = FloorPlanConverter(
        resolution=args.resolution,
        wall_thickness=args.wall_thickness,
        invert_colors=args.invert,
    )

    print(f"Converting: {args.input}")
    print(f"Resolution: {args.resolution}m/pixel")

    # Convert
    try:
        pgm, metadata = converter.convert(
            args.input,
            width_m=args.width,
            height_m=args.height,
            detection_method=args.method,
            add_border=not args.no_border,
            add_furniture=args.add_furniture,
            furniture_density=args.furniture_density,
            furniture_seed=args.furniture_seed,
        )
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Generate output paths
    output_pgm = Path(args.output)
    output_yaml = output_pgm.with_suffix('.yaml')

    # Create comment for PGM
    comment = f"Converted from: {Path(args.input).name}\n"
    comment += f"Size: {metadata['physical_size']['width_m']:.1f}m x {metadata['physical_size']['height_m']:.1f}m"
    if args.add_furniture:
        comment += f"\nFurniture density: {args.furniture_density}"

    # Save files
    converter.save_pgm(pgm, str(output_pgm), comment)
    converter.save_yaml(metadata, str(output_yaml), output_pgm.name)

    h, w = pgm.shape
    print(f"Output: {output_pgm}")
    print(f"        {output_yaml}")
    print(f"Size: {w}x{h} pixels ({metadata['physical_size']['width_m']:.1f}m x {metadata['physical_size']['height_m']:.1f}m)")

    # Preview if requested
    if args.preview:
        try:
            cv2.imshow('Converted Map', pgm)
            print("Press any key to close preview...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"Preview not available: {e}")

    print("Done!")


if __name__ == '__main__':
    main()
