#!/usr/bin/env python3
"""
Visualization utilities for Marina map parser.
"""
import os
import math
from typing import Tuple, List, Dict
from PIL import Image, ImageDraw
from config import MapConfig


class TrackImageGenerator:
    """Generates track images from boundary data."""

    def __init__(self, config: MapConfig):
        self.config = config

    def create_track_image(self, output_dir: str, resolution: float = 0.05, scaled_trackbounds: Dict = None, translation_offset: tuple = None) -> Tuple[float, float]:
        """Create PNG image of track from boundary data."""
        print("Creating track image from boundary data...")

        # Initialize boundary points
        boundary_points_left = []
        boundary_points_right = []
        centerline_points = []

        # Try to use provided trackbounds data first (new format with 'left'/'right' arrays)
        if scaled_trackbounds and 'left' in scaled_trackbounds and 'right' in scaled_trackbounds:
            boundary_points_left = [(x, y)
                                    for x, y in scaled_trackbounds['left']]
            boundary_points_right = [(x, y)
                                     for x, y in scaled_trackbounds['right']]

            # Generate centerline points from left/right boundaries
            if len(boundary_points_left) == len(boundary_points_right):
                centerline_points = [
                    ((lx + rx) / 2, (ly + ry) / 2)
                    for (lx, ly), (rx, ry) in zip(boundary_points_left, boundary_points_right)
                ]
            elif boundary_points_left:
                centerline_points = boundary_points_left[:]
            elif boundary_points_right:
                centerline_points = boundary_points_right[:]

            print(
                f"Using scaled trackbounds: {len(boundary_points_left)} left, {len(boundary_points_right)} right points")

        # Fall back to markers format if available
        elif scaled_trackbounds and 'markers' in scaled_trackbounds and scaled_trackbounds['markers']:
            boundary_points_left, boundary_points_right, centerline_points = self._load_boundary_data_from_markers(
                scaled_trackbounds['markers'])
            print(
                f"Using trackbounds from markers: {len(scaled_trackbounds['markers'])} markers")

        if not boundary_points_left or not boundary_points_right:
            print("No boundary data found - creating placeholder")
            return self._create_placeholder_image(output_dir)

        print(
            f"Found {len(boundary_points_left)} left and {len(boundary_points_right)} right boundary points")
        print(
            f"Track width multiplier: {self.config.width_multiplier} (track width is now {self.config.width_multiplier} times original)")

        # Note: Translation offset already applied to trackbounds data in scaling process
        # No need to apply again here for JSON input

        # Calculate image bounds
        min_x, max_x, min_y, max_y = self._calculate_image_bounds(
            boundary_points_left, boundary_points_right)

        # Create image
        width_px, height_px = self._calculate_image_dimensions(
            min_x, max_x, min_y, max_y, resolution)

        print(
            f"Image bounds with scaled padding: X=[{min_x:.3f}, {max_x:.3f}], Y=[{min_y:.3f}, {max_y:.3f}]")
        print(
            f"Image size: {width_px}x{height_px} pixels ({max_x-min_x:.1f}x{max_y-min_y:.1f}m)")

        # Create and draw image
        img = Image.new('RGB', (width_px, height_px), color='black')
        self._draw_track(img, boundary_points_left, boundary_points_right, centerline_points,
                         min_x, max_x, min_y, max_y, resolution)

        # Save image
        target_image = os.path.join(
            output_dir, f"{self.config.output_map_name}.png")
        img.save(target_image)
        print(f"Created track image: {target_image}")

        return min_x, min_y

    def _load_boundary_data(self) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]], List[Tuple[float, float]]]:
        """Load boundary data from CSV file."""
        boundary_points_left = []
        boundary_points_right = []
        centerline_points = []

        try:
            with open(self.config.csv_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        values = line.split(',')
                        if len(values) >= 46:  # Ensure we have enough columns
                            try:
                                # Get track boundary points (columns 41-45)
                                tb_left_x = float(values[41])
                                tb_left_y = float(values[42])
                                tb_right_x = float(values[44])
                                tb_right_y = float(values[45])

                                # Get centerline points
                                cl_x = float(values[27])
                                cl_y = float(values[28])

                                # Check for NaN or infinite values
                                if (math.isnan(tb_left_x) or math.isnan(tb_left_y) or
                                    math.isnan(tb_right_x) or math.isnan(tb_right_y) or
                                    math.isnan(cl_x) or math.isnan(cl_y) or
                                    math.isinf(tb_left_x) or math.isinf(tb_left_y) or
                                    math.isinf(tb_right_x) or math.isinf(tb_right_y) or
                                        math.isinf(cl_x) or math.isinf(cl_y)):
                                    continue

                                # Apply scale factor
                                tb_left_x *= self.config.scale_factor
                                tb_left_y *= self.config.scale_factor
                                tb_right_x *= self.config.scale_factor
                                tb_right_y *= self.config.scale_factor
                                cl_x *= self.config.scale_factor
                                cl_y *= self.config.scale_factor

                                # Apply width multiplier by expanding boundaries outward from centerline
                                # Calculate direction vectors from centerline to boundaries
                                left_dx = tb_left_x - cl_x
                                left_dy = tb_left_y - cl_y
                                right_dx = tb_right_x - cl_x
                                right_dy = tb_right_y - cl_y

                                # Scale the boundary distances
                                expanded_left_x = cl_x + left_dx * self.config.width_multiplier
                                expanded_left_y = cl_y + left_dy * self.config.width_multiplier
                                expanded_right_x = cl_x + right_dx * self.config.width_multiplier
                                expanded_right_y = cl_y + right_dy * self.config.width_multiplier

                                boundary_points_left.append(
                                    (expanded_left_x, expanded_left_y))
                                boundary_points_right.append(
                                    (expanded_right_x, expanded_right_y))
                                centerline_points.append((cl_x, cl_y))

                            except (ValueError, IndexError):
                                continue
        except Exception as e:
            print(f"Error loading boundary data: {e}")

        return boundary_points_left, boundary_points_right, centerline_points

    def _load_boundary_data_from_markers(self, markers: List[Dict]) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]], List[Tuple[float, float]]]:
        """Load boundary data from trackbounds markers (JSON input)."""
        boundary_points_left = []
        boundary_points_right = []
        centerline_points = []

        # For JSON input, markers are already scaled and translated
        # We just need to extract the coordinates
        for marker in markers:
            try:
                if 'pose' in marker and 'position' in marker['pose']:
                    x = marker['pose']['position']['x']
                    y = marker['pose']['position']['y']

                    # Check marker namespace to separate left/right boundaries
                    ns = marker.get('ns', '')
                    if 'left' in ns:
                        boundary_points_left.append((x, y))
                    elif 'right' in ns:
                        boundary_points_right.append((x, y))
                    else:
                        # Default to treating as general boundary points
                        # For JSON input without left/right separation, use as both
                        boundary_points_left.append((x, y))
                        boundary_points_right.append((x, y))

            except (KeyError, ValueError):
                continue

        # If we don't have separate left/right boundaries, create them from the general boundary points
        if not boundary_points_left and not boundary_points_right and markers:
            # Extract all boundary points and try to separate them
            all_boundary_points = []
            for marker in markers:
                try:
                    if 'pose' in marker and 'position' in marker['pose']:
                        x = marker['pose']['position']['x']
                        y = marker['pose']['position']['y']
                        all_boundary_points.append((x, y))
                except (KeyError, ValueError):
                    continue

            # For simplicity, treat all points as both left and right boundaries
            # This will create a wider track visualization
            boundary_points_left = all_boundary_points
            boundary_points_right = all_boundary_points

        # Generate approximate centerline from boundaries if available
        if boundary_points_left and boundary_points_right and len(boundary_points_left) == len(boundary_points_right):
            for (lx, ly), (rx, ry) in zip(boundary_points_left, boundary_points_right):
                center_x = (lx + rx) / 2.0
                center_y = (ly + ry) / 2.0
                centerline_points.append((center_x, center_y))
        elif boundary_points_left:
            # Use left boundary as approximation
            centerline_points = boundary_points_left[:]

        return boundary_points_left, boundary_points_right, centerline_points

    def _calculate_image_bounds(self, left_points: List[Tuple[float, float]],
                                right_points: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
        """Calculate image bounds with padding."""
        # Filter out NaN and infinite values
        all_points = left_points + right_points
        valid_points = []

        for x, y in all_points:
            if not (math.isnan(x) or math.isnan(y) or math.isinf(x) or math.isinf(y)):
                valid_points.append((x, y))

        if not valid_points:
            print("Warning: No valid boundary points found, using default bounds")
            return (-50.0, 50.0, -50.0, 50.0)

        all_x = [p[0] for p in valid_points]
        all_y = [p[1] for p in valid_points]

        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        # Add padding (scaled to match coordinates)
        padding = 10.0 * self.config.scale_factor
        min_x -= padding
        max_x += padding
        min_y -= padding
        max_y += padding

        return min_x, max_x, min_y, max_y

    def _calculate_image_dimensions(self, min_x: float, max_x: float, min_y: float,
                                    max_y: float, resolution: float) -> Tuple[int, int]:
        """Calculate image dimensions in pixels."""
        width_m = max_x - min_x
        height_m = max_y - min_y
        width_px = int(width_m / resolution)
        height_px = int(height_m / resolution)
        return width_px, height_px

    def _draw_track(self, img: Image.Image, left_points: List[Tuple[float, float]],
                    right_points: List[Tuple[float, float]], centerline_points: List[Tuple[float, float]],
                    min_x: float, max_x: float, min_y: float, max_y: float, resolution: float):
        """Draw track on image using simple boundary filling approach."""
        draw = ImageDraw.Draw(img)

        # Convert world coordinates to image coordinates
        def world_to_image(x: float, y: float) -> Tuple[int, int]:
            # Validate input coordinates
            if math.isnan(x) or math.isnan(y) or math.isinf(x) or math.isinf(y):
                return (0, 0)  # Return default position for invalid coordinates

            img_x = int((x - min_x) / resolution)
            img_y = int((max_y - y) / resolution)  # Flip Y axis

            # Clamp to image boundaries
            img_x = max(0, min(img_x, int((max_x - min_x) / resolution) - 1))
            img_y = max(0, min(img_y, int((max_y - min_y) / resolution) - 1))

            return img_x, img_y

        # Convert boundary points to image coordinates
        left_boundary_img = []
        right_boundary_img = []

        for x, y in left_points:
            if not (math.isnan(x) or math.isnan(y) or math.isinf(x) or math.isinf(y)):
                left_boundary_img.append(world_to_image(x, y))

        for x, y in right_points:
            if not (math.isnan(x) or math.isnan(y) or math.isinf(x) or math.isinf(y)):
                right_boundary_img.append(world_to_image(x, y))

        print(
            f"Valid boundary points: left={len(left_boundary_img)}, right={len(right_boundary_img)}")

        # Create solid track fill using polygon approach
        if left_boundary_img and right_boundary_img:
            print("Drawing track using solid polygon fill approach")

            # Create a complete polygon by combining both boundaries
            # Method: Use all left boundary points, then all right boundary points in reverse
            track_polygon = []

            # Add all left boundary points in order
            track_polygon.extend(left_boundary_img)

            # Add all right boundary points in reverse order to close the polygon
            track_polygon.extend(reversed(right_boundary_img))

            # Ensure the polygon is properly closed by connecting back to start
            if track_polygon and len(track_polygon) > 2:
                # Always close the polygon explicitly to avoid gaps
                start_point = track_polygon[0]
                last_point = track_polygon[-1]
                distance = math.sqrt(
                    (start_point[0] - last_point[0])**2 + (start_point[1] - last_point[1])**2)

                # Always add the start point to explicitly close the polygon
                track_polygon.append(start_point)
                print(
                    f"Closed polygon: start-end distance={distance:.1f} pixels")

                # Also add connecting lines to ensure no gaps at start/end
                # Connect first left point to first right point
                if left_boundary_img and right_boundary_img:
                    first_left = left_boundary_img[0]
                    first_right = right_boundary_img[0]
                    last_left = left_boundary_img[-1]
                    last_right = right_boundary_img[-1]

                    # Draw explicit closing lines to eliminate any gaps
                    try:
                        draw.line([first_left, first_right],
                                  fill='white', width=3)
                        draw.line([last_left, last_right],
                                  fill='white', width=3)
                        print("Drew explicit start/end closing lines")
                    except Exception as e:
                        print(f"Warning: Failed to draw closing lines: {e}")

                print(
                    f"Created solid track polygon with {len(track_polygon)} points")
                print(f"  Left boundary: {len(left_boundary_img)} points")
                print(
                    f"  Right boundary: {len(right_boundary_img)} points (reversed)")

                try:
                    # Draw the filled polygon
                    draw.polygon(track_polygon, fill='white', outline='white')
                    print("Successfully drew solid track polygon")

                    # Also draw boundary outlines for better definition
                    if len(left_boundary_img) > 1:
                        draw.line(left_boundary_img, fill='white', width=2)
                    if len(right_boundary_img) > 1:
                        draw.line(right_boundary_img, fill='white', width=2)

                except Exception as e:
                    print(f"Warning: Failed to draw polygon: {e}")
                    # Fallback: draw thick connecting lines with no gaps
                    self._draw_track_with_thick_lines(
                        draw, left_boundary_img, right_boundary_img)
            else:
                print("Warning: Insufficient points for polygon creation")
                self._draw_track_with_thick_lines(
                    draw, left_boundary_img, right_boundary_img)
        else:
            print("Warning: No valid boundary points found for track drawing")

    def _draw_track_with_thick_lines(self, draw: ImageDraw, left_boundary_img: List[Tuple[int, int]],
                                     right_boundary_img: List[Tuple[int, int]]):
        """Fallback method to draw track using thick connecting lines to avoid gaps."""
        print("Using thick line fallback method to avoid gaps")

        # Draw very thick connecting lines between boundaries
        if left_boundary_img and right_boundary_img:
            # Use the longer boundary as reference for better coverage
            if len(left_boundary_img) >= len(right_boundary_img):
                ref_boundary = left_boundary_img
                other_boundary = right_boundary_img
            else:
                ref_boundary = right_boundary_img
                other_boundary = left_boundary_img

            # Draw thick lines with overlap to ensure no gaps
            for i in range(len(ref_boundary)):
                # Find corresponding point on other boundary
                other_idx = int((i / len(ref_boundary)) * len(other_boundary))
                other_idx = min(other_idx, len(other_boundary) - 1)
                other_point = other_boundary[other_idx]
                ref_point = ref_boundary[i]

                # Draw very thick line to fill gaps
                try:
                    draw.line([ref_point, other_point], fill='white', width=8)
                except Exception as e:
                    continue

            # Draw boundary lines themselves with thick width
            if len(left_boundary_img) > 1:
                draw.line(left_boundary_img, fill='white', width=6)
            if len(right_boundary_img) > 1:
                draw.line(right_boundary_img, fill='white', width=6)

        print("Completed thick line fallback drawing")

    def _resample_boundary_by_distance(self, points: List[Tuple[int, int]], target_size: int) -> List[Tuple[int, int]]:
        """Resample boundary points based on arc length to preserve track geometry."""
        if not points or target_size <= 0:
            return points

        if len(points) <= 1:
            return points

        if len(points) == target_size:
            return points

        # Calculate cumulative distances along the boundary
        distances = [0.0]
        for i in range(1, len(points)):
            dx = points[i][0] - points[i-1][0]
            dy = points[i][1] - points[i-1][1]
            dist = math.sqrt(dx*dx + dy*dy)
            distances.append(distances[-1] + dist)

        total_length = distances[-1]
        if total_length == 0:
            return points

        # Create target distances for resampling
        resampled_points = []

        for i in range(target_size):
            if target_size == 1:
                target_dist = 0
            else:
                target_dist = (i / (target_size - 1)) * total_length

            # Find the segment containing this distance
            for j in range(len(distances) - 1):
                if distances[j] <= target_dist <= distances[j + 1]:
                    # Interpolate between points[j] and points[j+1]
                    if distances[j + 1] == distances[j]:
                        # Same distance, use first point
                        resampled_points.append(points[j])
                    else:
                        # Linear interpolation
                        t = (target_dist - distances[j]) / \
                            (distances[j + 1] - distances[j])
                        x1, y1 = points[j]
                        x2, y2 = points[j + 1]

                        x_interp = int(x1 + t * (x2 - x1))
                        y_interp = int(y1 + t * (y2 - y1))

                        resampled_points.append((x_interp, y_interp))
                    break
            else:
                # If we didn't find a segment (shouldn't happen), use last point
                resampled_points.append(points[-1])

        return resampled_points

    def _interpolate_boundary_points(self, points: List[Tuple[int, int]], target_size: int) -> List[Tuple[int, int]]:
        """Interpolate boundary points to match target size for proper track closure."""
        if not points or target_size <= 0:
            return points

        if len(points) == target_size:
            return points

        if len(points) == 1:
            # If only one point, repeat it
            return points * target_size

        # Linear interpolation between points
        interpolated = []

        for i in range(target_size):
            # Calculate position in original array
            pos = (i / (target_size - 1)) * \
                (len(points) - 1) if target_size > 1 else 0

            # Find surrounding points for interpolation
            idx = int(pos)
            frac = pos - idx

            if idx >= len(points) - 1:
                # Use last point
                interpolated.append(points[-1])
            elif frac == 0:
                # Exact match
                interpolated.append(points[idx])
            else:
                # Interpolate between points[idx] and points[idx+1]
                x1, y1 = points[idx]
                x2, y2 = points[idx + 1]

                x_interp = int(x1 + frac * (x2 - x1))
                y_interp = int(y1 + frac * (y2 - y1))

                interpolated.append((x_interp, y_interp))

        return interpolated

    def _create_placeholder_image(self, output_dir: str) -> Tuple[float, float]:
        """Create a placeholder image when no boundary data is available."""
        print("Creating placeholder track image...")

        # Create simple rectangular track
        img = Image.new('RGB', (800, 600), color='black')
        draw = ImageDraw.Draw(img)

        # Draw simple oval track
        track_outer = [100, 100, 700, 500]
        track_inner = [200, 200, 600, 400]

        draw.ellipse(track_outer, fill='white')
        draw.ellipse(track_inner, fill='black')

        # Save image
        target_image = os.path.join(
            output_dir, f"{self.config.output_map_name}.png")
        img.save(target_image)
        print(f"Created placeholder track image: {target_image}")

        return -40.0, -80.0  # Default origin
