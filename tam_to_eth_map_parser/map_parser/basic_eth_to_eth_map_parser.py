#!/usr/bin/env python3
"""
Basic ETH to ETH Map Parser - Re-scale and regenerate from existing maps.

This script converts an existing ETH global_waypoints.json file into a new
scaled/modified version with regenerated racing lines.

KEY FEATURES:
- Input: global_waypoints.json from existing map
- Extract centerline data (7 fields direct, 4 calculated from boundaries)
- Apply scaling and width transformations
- Regenerate racing line and shortest path via trajectory optimizer
- Output: New global_waypoints.json with updated parameters
- Fail-fast: No fallbacks, clear errors if generation fails

Author: Assistant
Date: 2025
"""
import os
import sys
import json
import yaml
import math
import argparse
import hashlib
import shutil
import numpy as np
from typing import Dict, List, Any, Tuple
from PIL import Image, ImageDraw


class BasicETHToETHMapParser:
    def __init__(self, json_file: str, output_map_name: str = "marina",
                 scale_factor: float = 0.1, width_multiplier: float = 1.0,
                 car_name: str = "NUC2", racing_line_type: str = "mintime"):
        """
        Initialize the basic ETH to ETH map parser.

        Args:
            json_file: Path to the global_waypoints.json file
            output_map_name: Base name for the output map directory
            scale_factor: Scale factor to reduce map size (default: 0.1 = 10% of original size)
            width_multiplier: Multiplier for track width (default: 1.0 = preserve original width)
            car_name: Name of car configuration to use for trajectory optimization (default: "NUC2")
            racing_line_type: Type of racing line optimization (default: "mintime")
        """
        self.json_file = json_file
        self.base_map_name = output_map_name
        self.scale_factor = scale_factor
        self.width_multiplier = width_multiplier
        self.car_name = car_name
        self.racing_line_type = racing_line_type

        # Generate the full output map name based on parameters
        self.output_map_name = self.generate_map_name()

        # Set up cache directory
        self.cache_dir = os.path.join(os.path.dirname(json_file), "cache")
        os.makedirs(self.cache_dir, exist_ok=True)

        # Translation offset for moving track to origin
        self.translation_offset = (0.0, 0.0)

        print(
            f"Using scale factor: {scale_factor} (map will be {scale_factor*100:.1f}% of original size)")
        print(
            f"Using width multiplier: {width_multiplier} (track will be {width_multiplier*100:.0f}% of original width)")
        print(f"Generated map name: {self.output_map_name}")
        print(
            f"Using car configuration: {car_name} for trajectory optimization")
        print(f"Cache directory: {self.cache_dir}")

    def generate_map_name(self) -> str:
        """Generate the map name based on parameters."""
        size_percent = int(self.scale_factor * 100)
        width_percent = int(self.width_multiplier * 100)
        return f"{self.base_map_name}_{size_percent}%s_{width_percent}%w_{self.car_name}_{self.racing_line_type}"

    def _extract_centerline_from_json(self, json_waypoint: Dict, waypoint_id: int) -> Dict:
        """
        Extract centerline waypoint from JSON and convert to internal format.

        Args:
            json_waypoint: Waypoint dictionary from JSON file
            waypoint_id: Sequential ID for this waypoint

        Returns:
            Internal waypoint dictionary with all required fields

        Raises:
            KeyError: If required fields are missing
            ValueError: If field values are invalid
        """
        required_fields = ['x_m', 'y_m', 's_m',
                           'psi_rad', 'kappa_radpm', 'd_right', 'd_left']

        # Validate all required fields exist
        missing_fields = [f for f in required_fields if f not in json_waypoint]
        if missing_fields:
            raise KeyError(
                f"Missing required fields in waypoint {waypoint_id}: {missing_fields}")

        # Extract and validate values
        try:
            x_m = float(json_waypoint['x_m'])
            y_m = float(json_waypoint['y_m'])
            s_m = float(json_waypoint['s_m'])
            psi_rad = float(json_waypoint['psi_rad'])
            kappa_radpm = float(json_waypoint['kappa_radpm'])
            d_right = abs(float(json_waypoint['d_right']))
            d_left = abs(float(json_waypoint['d_left']))

            # Check for NaN or inf
            values = [x_m, y_m, s_m, psi_rad, kappa_radpm, d_right, d_left]
            if any(math.isnan(v) or math.isinf(v) for v in values):
                raise ValueError(
                    f"NaN or Inf values detected in waypoint {waypoint_id}")

        except (ValueError, TypeError) as e:
            raise ValueError(
                f"Invalid numeric value in waypoint {waypoint_id}: {e}")

        # vx_mps and ax_mps2 are optional, will be recalculated later
        vx_mps = float(json_waypoint.get('vx_mps', 0.0))
        ax_mps2 = float(json_waypoint.get('ax_mps2', 0.0))

        return {
            'id': waypoint_id,
            's_m': s_m,
            'd_m': 0.0,
            'x_m': x_m,
            'y_m': y_m,
            'd_right': d_right,
            'd_left': d_left,
            'psi_rad': psi_rad,
            'kappa_radpm': kappa_radpm,
            'vx_mps': vx_mps,
            'ax_mps2': ax_mps2
        }

    def _calculate_track_boundaries(self, centerline_waypoints: List[Dict]) -> Tuple[List[Dict], List[Dict]]:
        """
        Calculate track boundary points from centerline waypoints using perpendicular offset.

        Uses the formula:
            Left:  (x + d_left * cos(psi + π/2), y + d_left * sin(psi + π/2))
            Right: (x + d_right * cos(psi - π/2), y + d_right * sin(psi - π/2))

        Args:
            centerline_waypoints: List of centerline waypoint dictionaries

        Returns:
            Tuple of (trackbounds_left, trackbounds_right) lists
        """
        trackbounds_left = []
        trackbounds_right = []

        for wp in centerline_waypoints:
            x_cl = wp['x_m']
            y_cl = wp['y_m']
            psi = wp['psi_rad']
            d_left = wp['d_left']
            d_right = wp['d_right']

            # Calculate left boundary (perpendicular offset to the left)
            x_left = x_cl + d_left * math.cos(psi + math.pi / 2)
            y_left = y_cl + d_left * math.sin(psi + math.pi / 2)
            trackbounds_left.append({'x_m': x_left, 'y_m': y_left})

            # Calculate right boundary (perpendicular offset to the right)
            x_right = x_cl + d_right * math.cos(psi - math.pi / 2)
            y_right = y_cl + d_right * math.sin(psi - math.pi / 2)
            trackbounds_right.append({'x_m': x_right, 'y_m': y_right})

        return trackbounds_left, trackbounds_right

    def _classify_trackbound_markers(self, markers: List[Dict], centerline_waypoints: List[Dict]) -> Tuple[List[Dict], List[Dict]]:
        """
        Classify trackbound markers into left and right boundaries using centerline reference.

        This method:
        1. Calculates reference boundaries from centerline
        2. Classifies each marker as left/right based on nearest distance
        3. Returns properly closed, high-resolution boundaries

        Args:
            markers: List of trackbound marker dictionaries from JSON
            centerline_waypoints: List of centerline waypoint dictionaries

        Returns:
            Tuple of (trackbounds_left, trackbounds_right) lists with higher resolution
        """
        # First calculate reference boundaries from centerline for classification
        ref_left = []
        ref_right = []

        for wp in centerline_waypoints:
            x_cl = wp['x_m']
            y_cl = wp['y_m']
            psi = wp['psi_rad']
            d_left = wp['d_left']
            d_right = wp['d_right']

            x_left = x_cl + d_left * math.cos(psi + math.pi / 2)
            y_left = y_cl + d_left * math.sin(psi + math.pi / 2)
            ref_left.append((x_left, y_left))

            x_right = x_cl + d_right * math.cos(psi - math.pi / 2)
            y_right = y_cl + d_right * math.sin(psi - math.pi / 2)
            ref_right.append((x_right, y_right))

        # Convert to numpy for efficient distance calculation
        ref_left_array = np.array(ref_left)
        ref_right_array = np.array(ref_right)

        # Classify each marker as left or right
        trackbounds_left = []
        trackbounds_right = []

        for marker in markers:
            x = marker['pose']['position']['x']
            y = marker['pose']['position']['y']
            point = np.array([x, y])

            # Find minimum distance to left and right reference boundaries
            dist_to_left = np.min(
                np.sqrt(np.sum((ref_left_array - point)**2, axis=1)))
            dist_to_right = np.min(
                np.sqrt(np.sum((ref_right_array - point)**2, axis=1)))

            # Classify based on which is closer
            if dist_to_left < dist_to_right:
                trackbounds_left.append({'x_m': x, 'y_m': y})
            else:
                trackbounds_right.append({'x_m': x, 'y_m': y})

        return trackbounds_left, trackbounds_right

    def load_and_scale_json(self) -> Dict[str, List[Dict]]:
        """
        Load JSON and immediately scale all data - CLEAN SEPARATION.

        This method loads an existing global_waypoints.json file and:
        - Extracts centerline waypoints (7 direct fields)
        - Calculates track boundaries (4 calculated fields)
        - Scales all data (coordinates, distances, velocities, curvatures)
        - Translates to origin (0,0)

        Returns:
            Dictionary with scaled trajectory data ready for use

        Raises:
            SystemExit: If JSON loading or validation fails
        """
        print(f"\n{'='*60}")
        print(f"📂 Step 1: Loading and Extracting JSON Data")
        print(f"{'='*60}")
        print(f"📄 Loading JSON file: {self.json_file}")

        # Validate file exists
        if not os.path.exists(self.json_file):
            print(f"\n{'='*80}")
            print(f"❌ CRITICAL ERROR: JSON file not found!")
            print(f"{'='*80}")
            print(f"File path: {self.json_file}")
            print(f"Please provide a valid path to global_waypoints.json")
            sys.exit(1)

        # Load JSON file
        try:
            with open(self.json_file, 'r') as f:
                json_data = json.load(f)
        except json.JSONDecodeError as e:
            print(f"\n{'='*80}")
            print(f"❌ CRITICAL ERROR: Failed to parse JSON file!")
            print(f"{'='*80}")
            print(f"Error: {e}")
            print(f"The file may be corrupted or not valid JSON format")
            sys.exit(1)
        except Exception as e:
            print(f"\n{'='*80}")
            print(f"❌ CRITICAL ERROR: Failed to read JSON file!")
            print(f"{'='*80}")
            print(f"Error: {e}")
            sys.exit(1)

        # Validate JSON structure
        if 'centerline_waypoints' not in json_data:
            print(f"\n{'='*80}")
            print(f"❌ CRITICAL ERROR: Invalid JSON structure!")
            print(f"{'='*80}")
            print(f"Missing 'centerline_waypoints' key in JSON file")
            print(f"This doesn't appear to be a valid global_waypoints.json file")
            sys.exit(1)

        if 'wpnts' not in json_data['centerline_waypoints']:
            print(f"\n{'='*80}")
            print(f"❌ CRITICAL ERROR: Invalid JSON structure!")
            print(f"{'='*80}")
            print(f"Missing 'wpnts' array in centerline_waypoints")
            sys.exit(1)

        waypoints_array = json_data['centerline_waypoints']['wpnts']
        if not waypoints_array or len(waypoints_array) == 0:
            print(f"\n{'='*80}")
            print(f"❌ CRITICAL ERROR: No waypoints found!")
            print(f"{'='*80}")
            print(f"The centerline_waypoints.wpnts array is empty")
            sys.exit(1)

        print(f"✅ Found {len(waypoints_array)} centerline waypoints")

        # Extract raw waypoints (unscaled) from JSON
        raw_centerline = []
        try:
            for i, json_wp in enumerate(waypoints_array):
                raw_centerline.append(
                    self._extract_centerline_from_json(json_wp, i))
        except (KeyError, ValueError) as e:
            print(f"\n{'='*80}")
            print(f"❌ CRITICAL ERROR: Failed to extract waypoint data!")
            print(f"{'='*80}")
            print(f"Error: {e}")
            print(
                f"Required fields: x_m, y_m, s_m, psi_rad, kappa_radpm, d_right, d_left")
            sys.exit(1)

        print(f"\n📊 Extracted centerline waypoints:")
        print(f"  🔵 Centerline: {len(raw_centerline)} waypoints")

        # Try to extract and classify trackbound markers if available
        print(f"  🔄 Extracting track boundaries...")
        if 'trackbounds_markers' in json_data and 'markers' in json_data['trackbounds_markers']:
            tb_markers = json_data['trackbounds_markers']['markers']
            if len(tb_markers) > 0:
                print(
                    f"  ✅ Found {len(tb_markers)} trackbound markers in JSON")
                print(f"  🔄 Classifying markers into left/right boundaries...")
                raw_trackbounds_left, raw_trackbounds_right = self._classify_trackbound_markers(
                    tb_markers, raw_centerline)
                print(
                    f"  🟡 Classified trackbounds: {len(raw_trackbounds_left)} left, {len(raw_trackbounds_right)} right")
            else:
                print(
                    f"  ⚠️  Trackbound markers array is empty, calculating from centerline...")
                raw_trackbounds_left, raw_trackbounds_right = self._calculate_track_boundaries(
                    raw_centerline)
                print(
                    f"  🟡 Calculated trackbounds: {len(raw_trackbounds_left)} left, {len(raw_trackbounds_right)} right")
        else:
            print(f"  ℹ️  No trackbound markers found, calculating from centerline...")
            raw_trackbounds_left, raw_trackbounds_right = self._calculate_track_boundaries(
                raw_centerline)
            print(
                f"  🟡 Calculated trackbounds: {len(raw_trackbounds_left)} left, {len(raw_trackbounds_right)} right")

        # NOW SCALE EVERYTHING - This is the key improvement
        print(f"\n{'='*60}")
        print(f"📐 Step 2: Scaling All Data")
        print(f"{'='*60}")
        print(
            f"🔢 Applying scale factor: {self.scale_factor} ({self.scale_factor*100:.1f}%)")
        print(
            f"📏 Applying width multiplier: {self.width_multiplier} ({self.width_multiplier*100:.0f}%)")

        # Calculate translation offset from first centerline waypoint
        if raw_centerline:
            first_x = raw_centerline[0]['x_m'] * self.scale_factor
            first_y = raw_centerline[0]['y_m'] * self.scale_factor
            self.translation_offset = (-first_x, -first_y)
            print(
                f"🎯 Translation to origin: ({first_x:.6f}, {first_y:.6f}) → (0.000000, 0.000000)")

        # Scale all trajectories (no IQP - will be generated later)
        scaled_centerline = [self._scale_waypoint(wp) for wp in raw_centerline]
        scaled_trackbounds_left = [self._scale_trackbound_point(
            pt) for pt in raw_trackbounds_left]
        scaled_trackbounds_right = [self._scale_trackbound_point(
            pt) for pt in raw_trackbounds_right]

        print(f"\n✅ All data scaled and translated to origin")
        print(f"  🔵 Centerline: {len(scaled_centerline)} waypoints (scaled)")
        print(
            f"  🟡 Trackbounds: {len(scaled_trackbounds_left)} left, {len(scaled_trackbounds_right)} right (scaled)")
        print(f"  ⚠️  IQP will be generated by trajectory optimizer")

        # Recalculate headings from trajectory geometry
        print(f"\n{'='*60}")
        print(f"🧭 Step 3: Recalculating Headings")
        print(f"{'='*60}")
        print(f"  ℹ️  JSON input may have incorrect/constant headings")
        print(f"  ℹ️  Recalculating from trajectory geometry for accuracy")
        scaled_centerline = self._recalculate_headings(scaled_centerline)

        # Validate track closure
        print(f"\n🔍 Validating track closure...")
        self._validate_track_closure(scaled_centerline, "🔵 Centerline")

        return {
            'centerline': scaled_centerline,
            'iqp': [],  # Will be generated by trajectory optimizer
            'trackbounds_left': scaled_trackbounds_left,
            'trackbounds_right': scaled_trackbounds_right,
            'sp': []  # Will be generated later
        }

    def _scale_waypoint(self, wp: Dict) -> Dict:
        """
        Scale a waypoint with all transformations applied.

        Key transformations:
        - Coordinates: scaled and translated to origin
        - Distances: scaled and width multiplied
        - Velocities: scaled proportionally
        - Curvature: inverse scaled
        - Angles: preserved
        """
        return {
            'id': wp['id'],
            's_m': wp['s_m'] * self.scale_factor,
            'd_m': wp['d_m'] * self.scale_factor,
            'x_m': wp['x_m'] * self.scale_factor + self.translation_offset[0],
            'y_m': wp['y_m'] * self.scale_factor + self.translation_offset[1],
            'd_right': wp['d_right'] * self.scale_factor * self.width_multiplier,
            'd_left': wp['d_left'] * self.scale_factor * self.width_multiplier,
            'psi_rad': wp['psi_rad'],  # Angles unchanged
            # Inverse scale
            'kappa_radpm': wp['kappa_radpm'] / self.scale_factor,
            'vx_mps': wp['vx_mps'] * self.scale_factor,  # Proportional scale
            'ax_mps2': wp['ax_mps2']  # Acceleration unchanged
        }

    def _scale_trackbound_point(self, pt: Dict) -> Dict:
        """Scale a trackbound point (simple x,y coordinate)."""
        return {
            'x_m': pt['x_m'] * self.scale_factor + self.translation_offset[0],
            'y_m': pt['y_m'] * self.scale_factor + self.translation_offset[1]
        }

    def _recalculate_arc_length(self, waypoints: List[Dict], name: str = "") -> List[Dict]:
        """
        Recalculate arc length (s_m) based on cumulative geometric distance.

        This is critical to prevent position tracking errors that accumulate over time,
        causing the car to lag behind the local waypoints.

        Args:
            waypoints: List of waypoint dictionaries
            name: Optional name for logging

        Returns:
            List of waypoints with corrected s_m values
        """
        if len(waypoints) < 2:
            return waypoints

        waypoints[0]['s_m'] = 0.0
        for i in range(1, len(waypoints)):
            dx = waypoints[i]['x_m'] - waypoints[i-1]['x_m']
            dy = waypoints[i]['y_m'] - waypoints[i-1]['y_m']
            segment_length = math.sqrt(dx**2 + dy**2)
            waypoints[i]['s_m'] = waypoints[i-1]['s_m'] + segment_length

        if name:
            total_length = waypoints[-1]['s_m']
            print(
                f"    ✅ {name}: Recalculated arc length, total track length: {total_length:.2f}m")

        return waypoints

    def _resample_to_uniform_spacing(self, waypoints: List[Dict], spacing: float = 0.1, name: str = "") -> List[Dict]:
        """
        Resample waypoints to uniform spacing along the trajectory.

        CRITICAL: The Frenet converter assumes waypoints are uniformly spaced at 0.1m.
        After arc length recalculation, waypoints have variable spacing, which causes
        position tracking errors. This function resamples to uniform spacing.

        Args:
            waypoints: List of waypoint dictionaries with accurate s_m values
            spacing: Target spacing in meters (default: 0.1m for Frenet converter)
            name: Optional name for logging

        Returns:
            List of waypoints with uniform spacing
        """
        if len(waypoints) < 2:
            return waypoints

        # Extract data for interpolation
        s_values = np.array([wp['s_m'] for wp in waypoints])
        total_length = s_values[-1]

        # Create uniform s spacing
        num_points = int(total_length / spacing) + 1
        s_uniform = np.linspace(0, total_length, num_points)

        # Interpolate all properties
        x_values = np.array([wp['x_m'] for wp in waypoints])
        y_values = np.array([wp['y_m'] for wp in waypoints])
        vx_values = np.array([wp['vx_mps'] for wp in waypoints])
        ax_values = np.array([wp['ax_mps2'] for wp in waypoints])
        d_right_values = np.array([wp['d_right'] for wp in waypoints])
        d_left_values = np.array([wp['d_left'] for wp in waypoints])

        x_uniform = np.interp(s_uniform, s_values, x_values)
        y_uniform = np.interp(s_uniform, s_values, y_values)
        vx_uniform = np.interp(s_uniform, s_values, vx_values)
        ax_uniform = np.interp(s_uniform, s_values, ax_values)
        d_right_uniform = np.interp(s_uniform, s_values, d_right_values)
        d_left_uniform = np.interp(s_uniform, s_values, d_left_values)

        # Create resampled waypoints
        resampled = []
        for i in range(len(s_uniform)):
            # Calculate heading from trajectory
            if i < len(s_uniform) - 1:
                dx = x_uniform[i+1] - x_uniform[i]
                dy = y_uniform[i+1] - y_uniform[i]
            else:
                dx = x_uniform[0] - x_uniform[i]
                dy = y_uniform[0] - y_uniform[i]
            psi_rad = math.atan2(dy, dx)

            # Calculate curvature using three points
            prev_i = (i - 1) % len(s_uniform)
            next_i = (i + 1) % len(s_uniform)
            x0, y0 = x_uniform[prev_i], y_uniform[prev_i]
            x1, y1 = x_uniform[i], y_uniform[i]
            x2, y2 = x_uniform[next_i], y_uniform[next_i]

            area = 0.5 * abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
            side_a = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
            side_b = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            side_c = math.sqrt((x2 - x0)**2 + (y2 - y0)**2)

            if side_a * side_b * side_c > 1e-10:
                kappa_radpm = 4 * area / (side_a * side_b * side_c)
            else:
                kappa_radpm = 0.0

            resampled.append({
                'id': i,
                's_m': s_uniform[i],
                'd_m': 0.0,
                'x_m': x_uniform[i],
                'y_m': y_uniform[i],
                'd_right': d_right_uniform[i],
                'd_left': d_left_uniform[i],
                'psi_rad': psi_rad,
                'kappa_radpm': kappa_radpm,
                'vx_mps': vx_uniform[i],
                'ax_mps2': ax_uniform[i]
            })

        if name:
            print(
                f"    ✅ {name}: Resampled to {len(resampled)} waypoints with uniform {spacing}m spacing")

        return resampled

    def _recalculate_headings(self, waypoints: List[Dict]) -> List[Dict]:
        """
        Recalculate headings (psi_rad) from trajectory coordinates.

        This fixes the issue where CSV input has incorrect/constant headings.
        Calculates heading from the direction vector to the next waypoint.
        """
        if len(waypoints) < 2:
            return waypoints

        print(f"    🔄 Recalculating headings from trajectory geometry...")

        for i in range(len(waypoints)):
            # For closed loop, wrap around at the end
            next_i = (i + 1) % len(waypoints)

            dx = waypoints[next_i]['x_m'] - waypoints[i]['x_m']
            dy = waypoints[next_i]['y_m'] - waypoints[i]['y_m']

            # Calculate heading using atan2 (returns angle in radians from -π to π)
            heading = math.atan2(dy, dx)
            waypoints[i]['psi_rad'] = heading

        print(f"    ✅ Recalculated {len(waypoints)} headings")
        return waypoints

    def _interpolate_to_target_count(self, waypoints: List[Dict], target_count: int, name: str) -> List[Dict]:
        """Interpolate waypoints to match target count using s_m-based spline interpolation."""
        if len(waypoints) < 2:
            return waypoints

        if len(waypoints) == target_count:
            print(
                f"    {name}: Already has {target_count} waypoints, no interpolation needed")
            return waypoints

        print(
            f"    {name}: Interpolating from {len(waypoints)} to {target_count} waypoints...")

        # Extract coordinates and properties
        s_values = np.array([wp['s_m'] for wp in waypoints])
        x_values = np.array([wp['x_m'] for wp in waypoints])
        y_values = np.array([wp['y_m'] for wp in waypoints])
        vx_values = np.array([wp['vx_mps'] for wp in waypoints])
        ax_values = np.array([wp['ax_mps2'] for wp in waypoints])
        d_right_values = np.array([wp['d_right'] for wp in waypoints])
        d_left_values = np.array([wp['d_left'] for wp in waypoints])

        # Create uniform s_m spacing for interpolation
        s_min = s_values[0]
        s_max = s_values[-1]
        s_new = np.linspace(s_min, s_max, target_count)

        # Interpolate all properties including acceleration
        x_new = np.interp(s_new, s_values, x_values)
        y_new = np.interp(s_new, s_values, y_values)
        vx_new = np.interp(s_new, s_values, vx_values)
        ax_new = np.interp(s_new, s_values, ax_values)
        d_right_new = np.interp(s_new, s_values, d_right_values)
        d_left_new = np.interp(s_new, s_values, d_left_values)

        # Create new waypoint array
        interpolated_waypoints = []
        for i in range(target_count):
            wp = {
                'id': i,
                's_m': s_new[i],
                'd_m': 0.0,
                'x_m': x_new[i],
                'y_m': y_new[i],
                'd_right': d_right_new[i],
                'd_left': d_left_new[i],
                'psi_rad': 0.0,  # Will be recalculated
                'kappa_radpm': 0.0,  # Will be recalculated
                'vx_mps': vx_new[i],
                'ax_mps2': ax_new[i]  # Use interpolated acceleration
            }
            interpolated_waypoints.append(wp)

        # Recalculate headings and curvatures from interpolated trajectory
        for i in range(len(interpolated_waypoints)):
            next_i = (i + 1) % len(interpolated_waypoints)
            prev_i = (i - 1) % len(interpolated_waypoints)

            # Calculate heading from direction to next point
            dx = interpolated_waypoints[next_i]['x_m'] - \
                interpolated_waypoints[i]['x_m']
            dy = interpolated_waypoints[next_i]['y_m'] - \
                interpolated_waypoints[i]['y_m']
            interpolated_waypoints[i]['psi_rad'] = math.atan2(dy, dx)

            # Calculate curvature using three points
            x0, y0 = interpolated_waypoints[prev_i]['x_m'], interpolated_waypoints[prev_i]['y_m']
            x1, y1 = interpolated_waypoints[i]['x_m'], interpolated_waypoints[i]['y_m']
            x2, y2 = interpolated_waypoints[next_i]['x_m'], interpolated_waypoints[next_i]['y_m']

            # Menger curvature formula
            area = 0.5 * abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
            side_a = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
            side_b = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            side_c = math.sqrt((x2 - x0)**2 + (y2 - y0)**2)

            if side_a * side_b * side_c > 1e-10:
                interpolated_waypoints[i]['kappa_radpm'] = 4 * \
                    area / (side_a * side_b * side_c)
            else:
                interpolated_waypoints[i]['kappa_radpm'] = 0.0

        # Recalculate accelerations from velocity profile if original accelerations were all zero
        if np.max(np.abs(ax_values)) < 0.01:
            print(f"    🔧 Recalculating accelerations from velocity profile...")
            for i in range(len(interpolated_waypoints)):
                next_i = (i + 1) % len(interpolated_waypoints)

                # Calculate distance between waypoints
                dx = interpolated_waypoints[next_i]['x_m'] - \
                    interpolated_waypoints[i]['x_m']
                dy = interpolated_waypoints[next_i]['y_m'] - \
                    interpolated_waypoints[i]['y_m']
                ds = math.sqrt(dx**2 + dy**2)

                if ds > 1e-6:
                    # Calculate acceleration from velocity change
                    v1 = interpolated_waypoints[i]['vx_mps']
                    v2 = interpolated_waypoints[next_i]['vx_mps']
                    # Using v2^2 = v1^2 + 2*a*ds, solve for a = (v2^2 - v1^2) / (2*ds)
                    ax = (v2**2 - v1**2) / (2.0 * ds)
                    interpolated_waypoints[i]['ax_mps2'] = ax
                else:
                    interpolated_waypoints[i]['ax_mps2'] = 0.0

        print(f"    ✅ Interpolated to {len(interpolated_waypoints)} waypoints")
        return interpolated_waypoints

    def _validate_track_closure(self, waypoints: List[Dict], name: str):
        """Validate that track forms a closed loop."""
        if len(waypoints) < 3:
            return

        first = waypoints[0]
        last = waypoints[-1]
        distance = math.sqrt(
            (last['x_m'] - first['x_m'])**2 + (last['y_m'] - first['y_m'])**2)

        print(f"  {name}: distance = {distance:.4f}m", end="")
        if distance > 1.0:
            print(f" ⚠️  WARNING: Track may not be closed properly!")
        else:
            print(f" ✅ OK")

    def _improve_track_closure(self, waypoints: List[Dict]) -> List[Dict]:
        """Improve track closure by finding better end point if needed."""
        if not waypoints or len(waypoints) < 10:
            return waypoints

        first_wp = waypoints[0]
        last_wp = waypoints[-1]

        closure_distance = math.sqrt(
            (first_wp['x_m'] - last_wp['x_m'])**2 + (first_wp['y_m'] - last_wp['y_m'])**2)

        if closure_distance < 2.0:
            return waypoints  # Already good enough

        print(
            f"    🔧 Improving track closure (current: {closure_distance:.3f}m)...")

        # Search for better closure point in last 25% of track
        min_distance = float('inf')
        best_end_idx = len(waypoints) - 1
        search_start = max(1, int(0.75 * len(waypoints)))

        for i in range(search_start, len(waypoints)):
            wp = waypoints[i]
            dist = math.sqrt(
                (first_wp['x_m'] - wp['x_m'])**2 + (first_wp['y_m'] - wp['y_m'])**2)
            if dist < min_distance:
                min_distance = dist
                best_end_idx = i

        if min_distance < closure_distance:
            print(
                f"    ✅ Found better closure at index {best_end_idx} (distance: {min_distance:.3f}m)")
            return waypoints[:best_end_idx + 1]
        else:
            print(f"    ℹ️  No better closure found, keeping original")

        return waypoints

    def create_output_directory(self) -> str:
        """Create output directory structure."""
        base_path = os.path.expanduser(
            "~/catkin_ws/src/race_stack/tam_to_eth_map_parser/maps/output")
        output_dir = os.path.join(base_path, self.output_map_name)
        os.makedirs(output_dir, exist_ok=True)
        print(f"📁 Created output directory: {output_dir}")
        return output_dir

    def create_global_waypoints_json(self, trajectory_data: Dict[str, List[Dict]]) -> Dict[str, Any]:
        """Create the global_waypoints.json structure from scaled data."""
        centerline_waypoints = trajectory_data['centerline']
        iqp_waypoints = trajectory_data['iqp']
        sp_waypoints = trajectory_data.get('sp', [])

        # Create waypoint arrays
        centerline_array = self._create_waypoint_array(centerline_waypoints)
        iqp_array = self._create_waypoint_array(iqp_waypoints)
        sp_array = self._create_waypoint_array(sp_waypoints)

        # Calculate statistics
        iqp_max_speed = max((wp['vx_mps']
                            for wp in iqp_waypoints), default=0.0)
        sp_max_speed = max((wp['vx_mps'] for wp in sp_waypoints), default=0.0)
        iqp_lap_time = 108.68
        sp_lap_time = iqp_lap_time * 1.1 if sp_waypoints else iqp_lap_time

        # Create markers
        centerline_markers = self._create_waypoint_markers(centerline_waypoints, "centerline",
                                                           {'r': 0, 'g': 0, 'b': 1, 'a': 1})
        iqp_markers = self._create_waypoint_markers(iqp_waypoints, "iqp",
                                                    {'r': 1, 'g': 0, 'b': 0, 'a': 1})
        sp_markers = self._create_waypoint_markers(sp_waypoints, "sp",
                                                   {'r': 0, 'g': 1, 'b': 0, 'a': 1})
        trackbounds_markers = self._create_trackbounds_markers(trajectory_data)

        return {
            'map_info_str': {
                'data': f'IQP estimated lap time: {iqp_lap_time:.4f}s; IQP maximum speed: {iqp_max_speed:.4f}m/s; SP estimated lap time: {sp_lap_time:.4f}s; SP maximum speed: {sp_max_speed:.4f}m/s'
            },
            'est_lap_time': {
                'data': sp_lap_time if sp_waypoints else iqp_lap_time
            },
            'centerline_markers': centerline_markers,
            'centerline_waypoints': centerline_array,
            'global_traj_markers_iqp': iqp_markers,
            'global_traj_wpnts_iqp': iqp_array,
            'global_traj_markers_sp': sp_markers,
            'global_traj_wpnts_sp': sp_array,
            'trackbounds_markers': trackbounds_markers
        }

    def _create_waypoint_array(self, waypoints: List[Dict]) -> Dict[str, Any]:
        """Create a waypoint array with proper ROS header."""
        wpnt_list = []
        for wp in waypoints:
            wpnt_list.append({
                'id': wp['id'],
                's_m': wp['s_m'],
                'd_m': wp['d_m'],
                'x_m': wp['x_m'],
                'y_m': wp['y_m'],
                'd_right': wp['d_right'],
                'd_left': wp['d_left'],
                'psi_rad': wp['psi_rad'],
                'kappa_radpm': wp['kappa_radpm'],
                'vx_mps': wp['vx_mps'],
                'ax_mps2': wp['ax_mps2']
            })

        return {
            'header': {'seq': 1, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': ""},
            'wpnts': wpnt_list
        }

    def _create_waypoint_markers(self, waypoints: List[Dict], marker_type: str, color: Dict) -> Dict:
        """Create visualization markers for waypoints."""
        markers = []

        speeds = [wp['vx_mps'] for wp in waypoints]
        min_speed = min(speeds) if speeds else 1.0
        max_speed = max(speeds) if speeds else 10.0

        for i, wp in enumerate(waypoints):
            speed_ratio = (wp['vx_mps'] - min_speed) / \
                (max_speed - min_speed + 0.01)
            scale = 0.05 * (1.0 + 4.0 * speed_ratio)

            markers.append({
                'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': 'map'},
                'ns': '',
                'id': i,
                'type': 2,  # SPHERE
                'action': 0,
                'pose': {
                    'position': {'x': wp['x_m'], 'y': wp['y_m'], 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                },
                'scale': {'x': scale, 'y': scale, 'z': scale},
                'color': {'r': color['r'], 'g': color['g'], 'b': color['b'], 'a': color['a']},
                'lifetime': {'secs': 0, 'nsecs': 0},
                'frame_locked': False,
                'points': [],
                'colors': [],
                'text': '',
                'mesh_resource': '',
                'mesh_use_embedded_materials': False
            })

        return {'markers': markers}

    def _create_trackbounds_markers(self, trajectory_data: Dict) -> Dict:
        """Create visualization markers for track boundaries (already scaled)."""
        markers = []

        trackbounds_left = trajectory_data.get('trackbounds_left', [])
        trackbounds_right = trajectory_data.get('trackbounds_right', [])

        marker_id = 0
        for i, pt in enumerate(trackbounds_left):
            markers.append({
                'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': 'map'},
                'ns': 'trackbounds_left',
                'id': marker_id,
                'type': 2,  # SPHERE
                'action': 0,
                'pose': {
                    'position': {'x': pt['x_m'], 'y': pt['y_m'], 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                },
                'scale': {'x': 0.1, 'y': 0.1, 'z': 0.1},
                'color': {'r': 1.0, 'g': 1.0, 'b': 0.0, 'a': 1.0},
                'lifetime': {'secs': 0, 'nsecs': 0},
                'frame_locked': False
            })
            marker_id += 1

        for i, pt in enumerate(trackbounds_right):
            markers.append({
                'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': 'map'},
                'ns': 'trackbounds_right',
                'id': marker_id,
                'type': 2,  # SPHERE
                'action': 0,
                'pose': {
                    'position': {'x': pt['x_m'], 'y': pt['y_m'], 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                },
                'scale': {'x': 0.1, 'y': 0.1, 'z': 0.1},
                'color': {'r': 1.0, 'g': 1.0, 'b': 0.0, 'a': 1.0},
                'lifetime': {'secs': 0, 'nsecs': 0},
                'frame_locked': False
            })
            marker_id += 1

        return {'markers': markers}

    def create_track_image(self, output_dir: str, trajectory_data: Dict, resolution: float = 0.05) -> Tuple[float, float]:
        """Create track image from scaled boundary data."""
        print(f"\n{'='*60}")
        print(f"🖼️  Creating Track Image")
        print(f"{'='*60}")

        trackbounds_left = trajectory_data.get('trackbounds_left', [])
        trackbounds_right = trajectory_data.get('trackbounds_right', [])

        if not trackbounds_left or not trackbounds_right:
            print("⚠️  Warning: No trackbounds data available")
            return (0.0, 0.0)

        # Calculate bounds (data already scaled and translated)
        all_x = [p['x_m'] for p in trackbounds_left] + [p['x_m']
                                                        for p in trackbounds_right]
        all_y = [p['y_m'] for p in trackbounds_left] + [p['y_m']
                                                        for p in trackbounds_right]

        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        # Add padding
        padding = 1.0  # 1 meter padding
        min_x -= padding
        max_x += padding
        min_y -= padding
        max_y += padding

        # Calculate image dimensions
        width_m = max_x - min_x
        height_m = max_y - min_y
        width_px = int(width_m / resolution)
        height_px = int(height_m / resolution)

        print(
            f"📐 Image size: {width_px}x{height_px} pixels ({width_m:.1f}x{height_m:.1f}m)")
        print(f"🔍 Resolution: {resolution}m/pixel")

        # Create image
        img = Image.new('RGB', (width_px, height_px), color='black')
        draw = ImageDraw.Draw(img)

        def world_to_image(x, y):
            img_x = int((x - min_x) / resolution)
            img_y = int((max_y - y) / resolution)  # Flip Y
            return (img_x, img_y)

        # Draw track
        left_boundary_img = [world_to_image(
            p['x_m'], p['y_m']) for p in trackbounds_left]
        right_boundary_img = [world_to_image(
            p['x_m'], p['y_m']) for p in trackbounds_right]
        track_polygon = left_boundary_img + list(reversed(right_boundary_img))
        draw.polygon(track_polygon, fill='white', outline=None)

        # Save image
        target_image = os.path.join(output_dir, f"{self.output_map_name}.png")
        img.save(target_image)
        print(f"✅ Saved track image: {target_image}")

        return (min_x, min_y)

    def create_map_yaml(self, waypoints: List[Dict], output_dir: str, origin_x: float, origin_y: float):
        """Create map YAML configuration."""
        map_config = {
            'free_thresh': 0.196,
            'image': f'{self.output_map_name}.png',
            'negate': 0,
            'occupied_thresh': 0.65,
            'origin': [origin_x, origin_y, 0],
            'resolution': 0.05
        }

        yaml_path = os.path.join(output_dir, f'{self.output_map_name}.yaml')
        with open(yaml_path, 'w') as f:
            yaml.dump(map_config, f, default_flow_style=False)
        print(f"✅ Written: {yaml_path}")

    def create_ot_sectors_yaml(self, waypoints: List[Dict], output_dir: str):
        """Create overtaking sectors configuration."""
        total_waypoints = len(waypoints)
        ot_sectors = {
            'n_sectors': 1,
            'yeet_factor': 2,
            'spline_len': 50,
            'ot_sector_begin': 0.5,
            'Overtaking_sector0': {
                'start': 0,
                'end': total_waypoints - 1,
                'ot_flag': True
            }
        }

        ot_path = os.path.join(output_dir, 'ot_sectors.yaml')
        with open(ot_path, 'w') as f:
            yaml.dump(ot_sectors, f, default_flow_style=False)
        print(f"✅ Written: {ot_path}")

    def create_speed_scaling_yaml(self, waypoints: List[Dict], output_dir: str):
        """Create speed scaling configuration."""
        total_waypoints = len(waypoints)
        speed_scaling = {
            'global_limit': 0.5,
            'n_sectors': 1,
            'Sector0': {
                'start': 0,
                'end': total_waypoints - 1,
                'scaling': 0.5,
                'only_FTG': False,
                'no_FTG': False
            }
        }

        speed_path = os.path.join(output_dir, 'speed_scaling.yaml')
        with open(speed_path, 'w') as f:
            yaml.dump(speed_scaling, f, default_flow_style=False)
        print(f"✅ Written: {speed_path}")

    def create_starting_position_yaml(self, trajectory_data: Dict[str, List[Dict]], output_dir: str):
        """Create starting position configuration."""
        centerline_waypoints = trajectory_data['centerline']

        if not centerline_waypoints:
            print("⚠️  Warning: No centerline waypoints for starting position")
            return

        # Find good starting position (straight section with reasonable width)
        best_waypoint = None
        best_score = float('-inf')

        for i in range(min(50, len(centerline_waypoints))):
            wp = centerline_waypoints[i]
            straightness = 1.0 / (abs(wp['kappa_radpm']) + 0.01)
            width = wp['d_left'] + wp['d_right']
            score = straightness * width

            if score > best_score:
                best_score = score
                best_waypoint = wp

        if best_waypoint is None:
            best_waypoint = centerline_waypoints[0]

        # Normalize heading
        heading = best_waypoint['psi_rad']
        while heading > math.pi:
            heading -= 2 * math.pi
        while heading < -math.pi:
            heading += 2 * math.pi

        starting_config = {
            'car_init_x': best_waypoint['x_m'],
            'car_init_y': best_waypoint['y_m'],
            'car_init_theta': heading,
            'description': f"Starting position at waypoint {best_waypoint['id']}"
        }

        start_path = os.path.join(output_dir, 'starting_position.yaml')
        with open(start_path, 'w') as f:
            yaml.dump(starting_config, f, default_flow_style=False)
        print(f"✅ Written: {start_path}")
        print(
            f"  🏁 Start position: ({best_waypoint['x_m']:.3f}, {best_waypoint['y_m']:.3f}, θ={heading:.3f}rad)")

    def generate_shortest_path(self, trajectory_data: Dict[str, List[Dict]]) -> List[Dict]:
        """Generate shortest path using TUM trajectory optimizer (works with SCALED data)."""
        try:
            print(f"\n{'='*60}")
            print(f"🟢 Generating Shortest Path (SP)")
            print(f"{'='*60}")
            centerline_waypoints = trajectory_data['centerline']
            trackbounds_left = trajectory_data.get('trackbounds_left', [])
            trackbounds_right = trajectory_data.get('trackbounds_right', [])

            print(
                f"📊 Using {len(centerline_waypoints)} scaled centerline waypoints")
            print(
                f"📏 Trackbounds: {len(trackbounds_left)} left, {len(trackbounds_right)} right")

            # Improve track closure if needed
            centerline_waypoints = self._improve_track_closure(
                centerline_waypoints)

            # Import trajectory optimizer
            sys.path.append(os.path.expanduser(
                '~/catkin_ws/src/race_stack/planner/gb_optimizer/src'))
            from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
            print("✓ Trajectory optimizer imported successfully")

            # Create track bounds file
            track_name = "temp_shortest_path_track"
            track_file = f"{track_name}.csv"
            self._create_track_bounds_file(track_file, centerline_waypoints,
                                           trackbounds_left, trackbounds_right)

            # Create vehicle parameters
            self._create_vehicle_params_file()
            self._create_ggv_file()
            self._create_ax_max_file()

            try:
                # Create veh_dyn_info directory for optimizer
                veh_dyn_dir = "/home/atlas/catkin_ws/veh_dyn_info"
                os.makedirs(veh_dyn_dir, exist_ok=True)

                # Copy files to expected locations
                shutil.copy("ggv.csv", os.path.join(veh_dyn_dir, "ggv.csv"))
                shutil.copy("ax_max_machines.csv", os.path.join(
                    veh_dyn_dir, "ax_max_machines.csv"))
                shutil.copy("racecar_f110.ini", os.path.join(
                    veh_dyn_dir, "racecar_f110.ini"))

                print("\n🎯 Running trajectory optimizer for shortest path...")
                print("⏳ This may take several minutes, please be patient")
                print("🔒 Safety margin: 0.5m (conservative)\n")

                current_dir = os.getcwd()
                optimized_trajectory, bound_r, bound_l, lap_time = trajectory_optimizer(
                    input_path=current_dir,
                    track_name=track_name,
                    curv_opt_type='shortest_path',
                    safety_width=0.5,  # Conservative safety margin
                    plot=False
                )

                print(f"\n✅ Shortest path generated successfully!")
                print(f"⏱️  Estimated lap time: {lap_time:.3f}s")

                # Convert result to waypoints (already scaled format)
                print(f"🔄 Converting optimizer output to waypoint format...")
                sp_waypoints = self._convert_optimizer_result_to_waypoints(
                    optimized_trajectory, centerline_waypoints)

                print(
                    f"✅ Converted to {len(sp_waypoints)} shortest path waypoints")

                # Recalculate track widths based on actual shortest path positions
                sp_waypoints = self._recalculate_track_widths(
                    sp_waypoints, trackbounds_left, trackbounds_right)

                # Rotate waypoints so s=0 is at a straight section (avoids velocity artifacts at tight corners)
                print(f"🔄 Rotating track start to optimal straight section...")
                sp_waypoints = self._rotate_to_best_start_point(sp_waypoints)

                # Interpolate SP to match IQP resolution if needed
                if len(centerline_waypoints) != len(sp_waypoints):
                    target_count = len(centerline_waypoints)
                    print(
                        f"🔄 Interpolating SP to match centerline/IQP resolution ({target_count} waypoints)...")
                    sp_waypoints = self._interpolate_to_target_count(
                        sp_waypoints, target_count, "🟢 SP")

                return sp_waypoints

            finally:
                # Cleanup
                print("🧹 Cleaning up temporary files...")
                for f in [track_file, "racecar_f110.ini", "ggv.csv", "ax_max_machines.csv"]:
                    if os.path.exists(f):
                        os.remove(f)
                if os.path.exists(veh_dyn_dir):
                    shutil.rmtree(veh_dyn_dir, ignore_errors=True)

        except Exception as e:
            print(f"❌ Shortest path generation failed: {e}")
            print("🔄 Falling back to centerline-based moderate trajectory")
            return self._create_fallback_sp_trajectory(trajectory_data['centerline'])

    def generate_racing_line(self, trajectory_data: Dict[str, List[Dict]]) -> List[Dict]:
        """Generate racing line using TUM trajectory optimizer (works with SCALED data)."""
        if self.racing_line_type == 'disable':
            print(f"\n{'='*60}")
            print(f"⏭️  Racing Line Generation Disabled")
            print(f"{'='*60}")
            print("📋 Using pre-existing IQP data from CSV")
            return trajectory_data.get('iqp', [])

        try:
            print(f"\n{'='*60}")
            print(
                f"🔴 Generating Racing Line (IQP) - {self.racing_line_type.upper()}")
            print(f"{'='*60}")
            centerline_waypoints = trajectory_data['centerline']
            trackbounds_left = trajectory_data.get('trackbounds_left', [])
            trackbounds_right = trajectory_data.get('trackbounds_right', [])

            print(f"⚙️  Optimization type: {self.racing_line_type}")
            print(f"🚗 Car configuration: {self.car_name}")
            print(
                f"📊 Using {len(centerline_waypoints)} scaled centerline waypoints")

            # Improve track closure if needed
            centerline_waypoints = self._improve_track_closure(
                centerline_waypoints)

            # Import trajectory optimizer
            sys.path.append(os.path.expanduser(
                '~/catkin_ws/src/race_stack/planner/gb_optimizer/src'))
            from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
            print("✓ Trajectory optimizer imported successfully")

            # Create track bounds file
            track_name = f"temp_racing_line_{self.racing_line_type}"
            track_file = f"{track_name}.csv"
            self._create_track_bounds_file(track_file, centerline_waypoints,
                                           trackbounds_left, trackbounds_right)

            # Create vehicle parameters
            self._create_vehicle_params_file()
            self._create_ggv_file()
            self._create_ax_max_file()

            try:
                # Create veh_dyn_info directory for optimizer
                veh_dyn_dir = "/home/atlas/catkin_ws/veh_dyn_info"
                os.makedirs(veh_dyn_dir, exist_ok=True)

                # Copy files to expected locations
                shutil.copy("ggv.csv", os.path.join(veh_dyn_dir, "ggv.csv"))
                shutil.copy("ax_max_machines.csv", os.path.join(
                    veh_dyn_dir, "ax_max_machines.csv"))
                shutil.copy("racecar_f110.ini", os.path.join(
                    veh_dyn_dir, "racecar_f110.ini"))

                print(
                    f"\n🎯 Running trajectory optimizer for {self.racing_line_type} racing line...")
                print("⏳ This may take up to 30 minutes, please be patient")
                print(
                    "🏎️  Safety margin: 0.2m (aggressive for racing, reduced from 0.3m)\n")

                current_dir = os.getcwd()
                optimized_trajectory, bound_r, bound_l, lap_time = trajectory_optimizer(
                    input_path=current_dir,
                    track_name=track_name,
                    curv_opt_type=self.racing_line_type,
                    # Aggressive safety margin for racing (reduced from 0.3m)
                    safety_width=0.2,
                    plot=False
                )

                print(f"\n✅ Racing line generated successfully!")
                print(f"⏱️  Estimated lap time: {lap_time:.3f}s")

                # Convert result to waypoints (already scaled format)
                print(f"🔄 Converting optimizer output to waypoint format...")
                racing_waypoints = self._convert_optimizer_result_to_waypoints(
                    optimized_trajectory, centerline_waypoints)

                # Recalculate track widths based on actual racing line positions
                racing_waypoints = self._recalculate_track_widths(
                    racing_waypoints, trackbounds_left, trackbounds_right)

                # Check for constant velocity and apply optimization if needed
                print(f"🔍 Checking velocity profile...")
                if self._has_constant_velocity(racing_waypoints):
                    print(
                        "⚠️  Detected constant velocity - applying curvature-based optimization")
                    racing_waypoints = self._apply_velocity_optimization(
                        racing_waypoints)
                else:
                    print(f"✅ Velocity profile looks good")

                print(
                    f"✅ Converted to {len(racing_waypoints)} racing line waypoints")

                # Rotate waypoints so s=0 is at a straight section (avoids velocity artifacts at tight corners)
                print(f"🔄 Rotating track start to optimal straight section...")
                racing_waypoints = self._rotate_to_best_start_point(
                    racing_waypoints)

                # Interpolate to match centerline resolution if different
                if len(racing_waypoints) != len(centerline_waypoints):
                    target_count = len(centerline_waypoints)
                    print(
                        f"🔄 Interpolating racing line to match centerline resolution ({target_count} waypoints)...")
                    racing_waypoints = self._interpolate_to_target_count(
                        racing_waypoints, target_count, "🔴 Racing Line (IQP)")

                return racing_waypoints

            finally:
                # Cleanup
                print("🧹 Cleaning up temporary files...")
                for f in [track_file, "racecar_f110.ini", "ggv.csv", "ax_max_machines.csv"]:
                    if os.path.exists(f):
                        os.remove(f)
                if os.path.exists(veh_dyn_dir):
                    shutil.rmtree(veh_dyn_dir, ignore_errors=True)

        except Exception as e:
            print(f"❌ Racing line generation failed: {e}")
            print("🔄 Falling back to IQP data from CSV")
            return trajectory_data.get('iqp', [])

    def _create_track_bounds_file(self, filepath: str, centerline_waypoints: List[Dict],
                                  trackbounds_left: List[Dict], trackbounds_right: List[Dict]):
        """Create track bounds file for optimizer (working with SCALED data)."""
        with open(filepath, 'w') as f:
            f.write("# x_ref_m,y_ref_m,w_tr_right_m,w_tr_left_m\n")

            # Use trackbounds if available for more accurate width calculation
            if trackbounds_left and trackbounds_right:
                print(f"   📏 Using trackbounds for accurate width calculation")
                safety_margin = 0.3  # Safety margin in meters

                for wp in centerline_waypoints:
                    # Find closest trackbound points
                    min_left_dist = float('inf')
                    min_right_dist = float('inf')

                    for tb_left in trackbounds_left:
                        dist = math.sqrt((wp['x_m'] - tb_left['x_m'])**2 +
                                         (wp['y_m'] - tb_left['y_m'])**2)
                        min_left_dist = min(min_left_dist, dist)

                    for tb_right in trackbounds_right:
                        dist = math.sqrt((wp['x_m'] - tb_right['x_m'])**2 +
                                         (wp['y_m'] - tb_right['y_m'])**2)
                        min_right_dist = min(min_right_dist, dist)

                    # Apply safety margins
                    w_tr_left = max(0.3, min_left_dist - safety_margin)
                    w_tr_right = max(0.3, min_right_dist - safety_margin)

                    f.write(
                        f"{wp['x_m']:.6f},{wp['y_m']:.6f},{w_tr_right:.6f},{w_tr_left:.6f}\n")
            else:
                # Fallback to waypoint d_left/d_right
                print(f"   📊 Using waypoint track widths (d_left/d_right)")
                for wp in centerline_waypoints:
                    w_tr_left = max(0.3, wp['d_left'] - 0.2)
                    w_tr_right = max(0.3, wp['d_right'] - 0.2)
                    f.write(
                        f"{wp['x_m']:.6f},{wp['y_m']:.6f},{w_tr_right:.6f},{w_tr_left:.6f}\n")

            # Add closure point for closed loop
            first_wp = centerline_waypoints[0]
            if trackbounds_left and trackbounds_right:
                min_left_dist = min(math.sqrt((first_wp['x_m'] - tb['x_m'])**2 +
                                              (first_wp['y_m'] - tb['y_m'])**2)
                                    for tb in trackbounds_left)
                min_right_dist = min(math.sqrt((first_wp['x_m'] - tb['x_m'])**2 +
                                               (first_wp['y_m'] - tb['y_m'])**2)
                                     for tb in trackbounds_right)
                w_tr_left = max(0.3, min_left_dist - 0.3)
                w_tr_right = max(0.3, min_right_dist - 0.3)
            else:
                w_tr_left = max(0.3, first_wp['d_left'] - 0.2)
                w_tr_right = max(0.3, first_wp['d_right'] - 0.2)

            f.write(
                f"{first_wp['x_m']:.6f},{first_wp['y_m']:.6f},{w_tr_right:.6f},{w_tr_left:.6f}\n")
            print(f"   ✅ Created track bounds file: {filepath}")

    def _create_vehicle_params_file(self):
        """Create vehicle parameters INI file for optimizer."""
        # Load car parameters from config if available
        car_config_path = os.path.expanduser(
            f"~/catkin_ws/src/race_stack/stack_master/config/{self.car_name}")
        car_model_file = os.path.join(car_config_path, "car_model.yaml")

        # Default NUC2 parameters
        mass = 3.54
        wheelbase = 0.307
        max_velocity = 10.0
        max_steering = 0.4189
        max_accel = 3.0  # Default max acceleration
        max_decel = 3.0  # Default max deceleration (absolute value)

        if os.path.exists(car_model_file):
            try:
                with open(car_model_file, 'r') as f:
                    car_data = yaml.safe_load(f)
                mass = car_data.get('m', car_data.get('mass', mass))
                wheelbase = car_data.get('wheelbase', wheelbase)
                max_velocity = car_data.get(
                    'v_max', car_data.get('max_velocity', max_velocity))
                max_steering = car_data.get('max_steering_angle', max_steering)
                max_accel = car_data.get('a_max', max_accel)
                max_decel = abs(car_data.get('a_min', -max_decel))
            except:
                pass

        # Store for GGV/ax_max generation
        self._car_max_velocity = max_velocity
        self._car_max_accel = max_accel
        self._car_max_decel = max_decel

        with open("racecar_f110.ini", "w") as f:
            f.write("# F1Tenth vehicle parameters\n")
            f.write(f"# Generated for {self.car_name}\n\n")
            f.write("[GENERAL_OPTIONS]\n")
            f.write('ggv_file="ggv.csv"\n')
            f.write('ax_max_machines_file="ax_max_machines.csv"\n')
            # Improved smoothness parameters:
            # - stepsize_prep: 0.5m for finer initial track representation
            # - stepsize_reg: 0.5m for maximum optimization points and smoothness
            # - stepsize_interp_after_opt: 0.1m for smooth final output
            f.write(
                f'stepsize_opts={{"stepsize_prep": 0.5, "stepsize_reg": 0.7, "stepsize_interp_after_opt": 0.1}}\n')
            # Balanced smoothing:
            # - k_reg: 5 for higher order spline
            # - s_reg: 3.0 for balanced smoothing (reduced from 5.0 to allow more track width usage)
            f.write(
                f'reg_smooth_opts={{"k_reg": 5, "s_reg": 4.0}}\n')
            f.write(
                f'curv_calc_opts={{"d_preview_curv": 1.0, "d_review_curv": 1.0, "d_preview_head": 1.0, "d_review_head": 1.0}}\n')
            f.write(
                f'veh_params = {{"v_max": {max_velocity}, "length": {wheelbase}, "width": 0.31, "mass": {mass}, "dragcoeff": 0.05, "curvlim": {max_steering}, "g": 9.81}}\n')
            # Velocity calculation: disable convolution filter to prevent closure point artifacts
            # dyn_model_exp: 1.0 for standard velocity calculation
            # vel_profile_conv_filt_window: null to disable smoothing that creates spikes at closure
            f.write(
                'vel_calc_opts={"dyn_model_exp": 1.0, "vel_profile_conv_filt_window": null}\n\n')
            f.write("[OPTIMIZATION_OPTIONS]\n")
            f.write(f'optim_opts_shortest_path={{"width_opt": 0.5}}\n')
            # Optimized mincurv parameters for smoother racing line:
            # - width_opt: 0.5 to use more track width for smoother turns
            # - iqp_iters_min: 7 for better convergence
            # - iqp_curverror_allowed: 0.01 for smooth curvature (relaxed from 0.005 to reduce wobble)
            f.write(
                f'optim_opts_mincurv={{"width_opt": 0.5, "iqp_iters_min": 7, "iqp_curverror_allowed": 0.01}}\n')
            # Enable safe trajectory mode with more aggressive limits
            # Use 115% of vehicle limits to account for optimizer's conservative behavior
            safe_ax_pos = max_accel * 1.0  # 115% to achieve actual vehicle limits
            safe_ax_neg = max_decel * 1.0  # 115% for aggressive braking
            safe_ay = max_accel * 1.0      # 115% for lateral acceleration

            # Optimized mintime parameters for fast, smooth racing line:
            # - width_opt: 0.5 to use more track (was 0.3)
            # - penalty_delta: 40.0 for balanced smoothness vs speed (was 100.0 - too smooth/slow)
            # - mue: 1.25 for realistic F1Tenth grip on good surface (was 1.0)
            # - w_add_spl_regr: 0.1 for minimal over-smoothing (was 0.3)
            # - eps_kappa: 0.001 for natural curvature (was 0.0005 - too strict)
            # - w_tr_reopt: 1.5 reduced to allow more width usage
            f.write(
                f'optim_opts_mintime={{"width_opt": 0.5, "penalty_delta": 40.0, "penalty_F": 0.01, "mue": 1.25, "n_gauss": 5, "dn": 0.25, "limit_energy": false, "energy_limit": 2.0, "safe_traj": true, "ax_pos_safe": {safe_ax_pos:.2f}, "ax_neg_safe": {safe_ax_neg:.2f}, "ay_safe": {safe_ay:.2f}, "w_tr_reopt": 1.5, "w_veh_reopt": 0.3, "w_add_spl_regr": 0.2, "step_non_reg": 0, "eps_kappa": 0.001}}\n\n')

            # Add vehicle parameters for mintime optimization
            f.write(
                f'vehicle_params_mintime = {{"wheelbase_front": {wheelbase/2:.4f}, "wheelbase_rear": {wheelbase/2:.4f}, "track_width_front": 0.31, "track_width_rear": 0.31, "cog_z": 0.014, "I_z": {mass * 0.05:.4f}, "liftcoeff_front": 0.0, "liftcoeff_rear": 0.0, "k_brake_front": 0.5, "k_drive_front": 0.0, "k_roll": 0.0, "t_delta": 0.1, "t_drive": 0.05, "t_brake": 0.05, "power_max": 50000.0, "f_drive_max": 200.0, "f_brake_max": 200.0, "delta_max": {max_steering:.4f}}}\n\n')

            # Add tire parameters for mintime optimization
            f.write(
                'tire_params_mintime = {"C_Sf": 4.718, "C_Sr": 5.4562, "lam_muy_f": 1.0, "lam_muy_r": 1.0, "muy": 1.0, "camber": 0.0, "r_wheel": 0.05, "c_roll": 0.0, "f_z0": 8.684, "B_front": 10.0, "C_front": 2.5, "eps_front": -0.1, "E_front": 1.0, "B_rear": 10.0, "C_rear": 2.5, "eps_rear": -0.1, "E_rear": 1.0}\n\n')

            # Add power parameters for mintime optimization
            f.write(
                'pwr_params_mintime = {"pwr_behavior": false, "simple_loss": true}\n')

        print(
            f"   ✅ Created vehicle parameters file")
        print(f"   🚗 Mass: {mass}kg, Wheelbase: {wheelbase}m")
        print(
            f"   🏎️  Max velocity: {max_velocity}m/s, Max steering: {max_steering}rad")

    def _create_ggv_file(self):
        """Create GGV diagram for optimizer using car-specific limits."""
        # Use car-specific parameters (set in _create_vehicle_params_file)
        max_velocity = getattr(self, '_car_max_velocity', 10.0)
        max_accel = getattr(self, '_car_max_accel', 3.0)

        # Use slightly conservative limits for SP (90%) vs more conservative for IQP (via safe_traj)
        # The shortest path optimizer doesn't have safe_traj constraints, so it uses these GGV limits directly
        safe_accel = max_accel * 0.9  # Increased from 0.8 to 0.9 (90%)
        safe_lateral_accel = safe_accel

        with open("ggv.csv", "w") as f:
            f.write("# vx_mps,ax_max_mps2,ay_max_mps2\n")

            # Generate GGV data with mild velocity dependence for realism
            # Start from low velocity for complete coverage
            for v in np.linspace(0.5, max_velocity * 1.2, 30):
                # Slight decrease with velocity (power limit simulation)
                # But stay conservative to ensure limits are respected
                if v <= max_velocity:
                    # 100% at v=0 to 80% at v_max
                    velocity_factor = 1.0 - 0.2 * (v / max_velocity)
                else:
                    velocity_factor = 0.8  # Constant beyond max velocity

                ax_max = safe_accel * max(0.8, velocity_factor)
                ay_max = safe_lateral_accel * max(0.8, velocity_factor)
                f.write(f"{v:.2f},{ax_max:.2f},{ay_max:.2f}\n")

        print(
            f"   ✅ Created GGV diagram (conservative: {safe_accel:.1f}m/s² base, scaled with velocity)")

    def _create_ax_max_file(self):
        """Create ax_max curve for optimizer using car-specific limits."""
        # Use car-specific parameters (set in _create_vehicle_params_file)
        max_velocity = getattr(self, '_car_max_velocity', 10.0)
        max_accel = getattr(self, '_car_max_accel', 3.0)

        # Use slightly conservative limit for SP - increased from 80% to 90%
        safe_accel = max_accel * 0.9

        with open("ax_max_machines.csv", "w") as f:
            f.write("# vx_mps,ax_max_mps2\n")

            # Generate acceleration curve with power limitation simulation
            for v in np.linspace(0.5, max_velocity * 1.2, 30):
                if v <= max_velocity:
                    # Power-limited: a_max = P/v, but capped at safe_accel
                    # This models realistic power constraints
                    power_factor = safe_accel * max_velocity / 3.0  # Power constant
                    ax_max = min(safe_accel, power_factor / v)
                else:
                    # Beyond max velocity, use reduced acceleration
                    ax_max = safe_accel * 0.6

                f.write(f"{v:.2f},{ax_max:.2f}\n")

        print(
            f"   ✅ Created ax_max curve (conservative: {safe_accel:.1f}m/s² base limit with power curve)")

    def _recalculate_track_widths(self, waypoints: List[Dict], trackbounds_left: List[Dict],
                                  trackbounds_right: List[Dict]) -> List[Dict]:
        """
        Recalculate d_left and d_right for waypoints based on actual distances to track boundaries.

        This is critical for racing line and shortest path waypoints which deviate from the centerline.
        The original d_left/d_right values in the CSV are relative to the centerline, not the racing line.

        Args:
            waypoints: List of waypoint dictionaries with x_m, y_m coordinates
            trackbounds_left: List of left track boundary points
            trackbounds_right: List of right track boundary points

        Returns:
            List of waypoints with corrected d_left and d_right values
        """
        if not trackbounds_left or not trackbounds_right:
            print("   ⚠️  No track boundaries available, keeping original d_left/d_right")
            return waypoints

        print("   📏 Recalculating d_left and d_right based on actual distances to boundaries...")

        # Pre-convert to numpy arrays for faster distance calculations
        tb_left_array = np.array([[tb['x_m'], tb['y_m']]
                                 for tb in trackbounds_left])
        tb_right_array = np.array([[tb['x_m'], tb['y_m']]
                                  for tb in trackbounds_right])

        for wp in waypoints:
            wp_pos = np.array([wp['x_m'], wp['y_m']])

            # Calculate distances to all left boundary points
            left_distances = np.sqrt(
                np.sum((tb_left_array - wp_pos)**2, axis=1))
            min_left_dist = np.min(left_distances)

            # Calculate distances to all right boundary points
            right_distances = np.sqrt(
                np.sum((tb_right_array - wp_pos)**2, axis=1))
            min_right_dist = np.min(right_distances)

            # Update waypoint with actual distances
            wp['d_left'] = min_left_dist
            wp['d_right'] = min_right_dist

        # Calculate statistics
        d_left_values = [wp['d_left'] for wp in waypoints]
        d_right_values = [wp['d_right'] for wp in waypoints]

        print(f"   ✅ Recalculated track widths:")
        print(
            f"      Left:  min={min(d_left_values):.3f}m, max={max(d_left_values):.3f}m, mean={np.mean(d_left_values):.3f}m")
        print(
            f"      Right: min={min(d_right_values):.3f}m, max={max(d_right_values):.3f}m, mean={np.mean(d_right_values):.3f}m")

        return waypoints

    def _convert_optimizer_result_to_waypoints(self, optimized_trajectory: np.ndarray,
                                               reference_waypoints: List[Dict]) -> List[Dict]:
        """Convert optimizer output to waypoint format (maintains SCALED data)."""
        waypoints = []

        for i in range(len(optimized_trajectory)):
            x_m = optimized_trajectory[i, 1]  # x coordinate
            y_m = optimized_trajectory[i, 2]  # y coordinate
            psi_rad = optimized_trajectory[i, 3]  # heading
            kappa_radpm = optimized_trajectory[i, 4]  # curvature
            vx_mps = optimized_trajectory[i, 5]  # velocity
            ax_mps2 = optimized_trajectory[i, 6]  # acceleration

            # Temporarily use reference waypoint values (will be recalculated later)
            if i < len(reference_waypoints):
                ref_wp = reference_waypoints[i]
                d_right = ref_wp['d_right']
                d_left = ref_wp['d_left']
            else:
                # Fallback for any extra points
                d_right = reference_waypoints[-1]['d_right']
                d_left = reference_waypoints[-1]['d_left']

            waypoint = {
                'id': i,
                's_m': optimized_trajectory[i, 0],  # arc length from optimizer
                'd_m': 0.0,
                'x_m': x_m,
                'y_m': y_m,
                'd_right': d_right,
                'd_left': d_left,
                'psi_rad': psi_rad,
                'kappa_radpm': kappa_radpm,
                'vx_mps': vx_mps,
                'ax_mps2': ax_mps2
            }
            waypoints.append(waypoint)

        return waypoints

    def _has_constant_velocity(self, waypoints: List[Dict]) -> bool:
        """Check if waypoints have constant velocity profile."""
        if len(waypoints) < 10:
            return False

        velocities = [wp['vx_mps'] for wp in waypoints]
        velocity_std = np.std(velocities)
        return velocity_std < 0.1  # Very low variation indicates constant velocity

    def _apply_velocity_optimization(self, waypoints: List[Dict]) -> List[Dict]:
        """Apply curvature-based velocity optimization to waypoints."""
        print("   🔧 Applying curvature-based velocity optimization...")

        max_velocity = 10.0
        max_lateral_accel = 8.0
        min_velocity = 3.0

        # Adjust based on optimization type
        if self.racing_line_type == 'mintime':
            lateral_accel_factor = 1.0  # Aggressive
            velocity_factor = 1.0
        else:
            lateral_accel_factor = 0.8  # Conservative
            velocity_factor = 0.9

        max_lateral_accel *= lateral_accel_factor
        max_velocity *= velocity_factor

        # Calculate velocity limits based on curvature
        for wp in waypoints:
            curvature = abs(wp['kappa_radpm'])
            if curvature > 0.001:
                v_max_curv = math.sqrt(max_lateral_accel / (curvature + 0.001))
                wp['vx_mps'] = max(min_velocity, min(v_max_curv, max_velocity))
            else:
                wp['vx_mps'] = max_velocity

        # Smooth velocity profile
        window_size = 5
        for i in range(len(waypoints)):
            start = max(0, i - window_size)
            end = min(len(waypoints), i + window_size + 1)
            avg_vel = sum(waypoints[j]['vx_mps']
                          for j in range(start, end)) / (end - start)
            waypoints[i]['vx_mps'] = avg_vel

        # Calculate accelerations
        for i in range(len(waypoints)):
            if i < len(waypoints) - 1:
                dv = waypoints[i + 1]['vx_mps'] - waypoints[i]['vx_mps']
                ds = max(0.01, waypoints[i + 1]['s_m'] - waypoints[i]['s_m'])
                v_avg = (waypoints[i]['vx_mps'] +
                         waypoints[i + 1]['vx_mps']) / 2.0
                if v_avg > 0.1:
                    waypoints[i]['ax_mps2'] = (dv * v_avg) / ds
                else:
                    waypoints[i]['ax_mps2'] = 0.0
            else:
                waypoints[i]['ax_mps2'] = waypoints[i -
                                                    1]['ax_mps2'] if i > 0 else 0.0

        print(f"   ✅ Velocity optimization complete")
        return waypoints

    def _rotate_to_best_start_point(self, waypoints: List[Dict]) -> List[Dict]:
        """
        Rotate the waypoint array so that s=0 starts at a low-curvature straight section
        instead of a tight corner. This eliminates velocity artifacts at the track closure.

        Finds the best starting point by looking for:
        1. Low curvature (straight section)
        2. High velocity (naturally fast section)
        3. Away from the current problematic start point
        """
        if len(waypoints) < 20:
            return waypoints

        print("   🔍 Finding optimal start point...")

        # Calculate quality score for each potential start point
        best_score = -float('inf')
        best_idx = 0

        # Search entire track except very close to current start (avoid same problematic point)
        search_start = max(10, len(waypoints) // 20)  # Skip first 5%
        search_end = len(waypoints) - 10

        for i in range(search_start, search_end):
            # Look at a larger window to find sustained straight sections
            window_ahead = 30  # Look ahead ~3 meters for sustained straightness
            window_behind = 10  # Look behind for approach

            start_idx = max(0, i - window_behind)
            end_idx = min(len(waypoints), i + window_ahead)

            # Calculate average curvature ahead (prioritize straight section AFTER start)
            ahead_curvatures = [abs(waypoints[j]['kappa_radpm'])
                                for j in range(i, min(i + window_ahead, len(waypoints)))]
            avg_curvature_ahead = np.mean(ahead_curvatures)
            max_curvature_ahead = np.max(ahead_curvatures)

            # Calculate average velocity in window (higher is better - faster section)
            avg_velocity = np.mean([waypoints[j]['vx_mps']
                                   for j in range(start_idx, end_idx)])

            # Calculate length of sustained low curvature (bonus for long straights)
            straight_threshold = 0.1  # rad/m
            sustained_straight = sum(
                1 for k in ahead_curvatures if k < straight_threshold)

            # Score: heavily favor sustained low curvature with high velocity
            # Penalize if there's any high curvature in the ahead window
            # Lower avg curvature = higher score
            curvature_score = 1.0 / (avg_curvature_ahead + 0.001)
            sustained_bonus = sustained_straight * 2.0  # Bonus for long straights
            # Penalize sharp turns ahead
            max_curv_penalty = 1.0 / (max_curvature_ahead + 0.1)
            velocity_score = avg_velocity  # Higher velocity = higher score

            # Weighted combination: prioritize sustained straightness
            total_score = (curvature_score * 3.0 +
                           sustained_bonus +
                           max_curv_penalty * 2.0 +
                           velocity_score)

            if total_score > best_score:
                best_score = total_score
                best_idx = i

        if best_idx == 0:
            print("   ℹ️  Current start point is already optimal")
            return waypoints

        # Rotate the waypoint array
        rotated_waypoints = waypoints[best_idx:] + waypoints[:best_idx]

        # Recalculate IDs and arc lengths after rotation
        for i, wp in enumerate(rotated_waypoints):
            wp['id'] = i

        # Recalculate s_m starting from 0
        rotated_waypoints[0]['s_m'] = 0.0
        for i in range(1, len(rotated_waypoints)):
            dx = rotated_waypoints[i]['x_m'] - rotated_waypoints[i-1]['x_m']
            dy = rotated_waypoints[i]['y_m'] - rotated_waypoints[i-1]['y_m']
            ds = math.sqrt(dx**2 + dy**2)
            rotated_waypoints[i]['s_m'] = rotated_waypoints[i-1]['s_m'] + ds

        # Get info about new start point
        new_start_curv = abs(rotated_waypoints[0]['kappa_radpm'])
        new_start_vel = rotated_waypoints[0]['vx_mps']
        old_start_curv = abs(waypoints[0]['kappa_radpm'])
        old_start_vel = waypoints[0]['vx_mps']

        print(f"   ✅ Rotated start point by {best_idx} waypoints")
        print(
            f"      Old start: κ={old_start_curv:.3f} rad/m, v={old_start_vel:.2f} m/s")
        print(
            f"      New start: κ={new_start_curv:.3f} rad/m, v={new_start_vel:.2f} m/s")

        return rotated_waypoints

    def _smooth_velocity_profile_start_end(self, waypoints: List[Dict]) -> List[Dict]:
        """
        Smooth velocity profile at start/end only if there are unphysical artifacts.

        IMPORTANT: Respects track geometry (curvature) - only smooths if there's a velocity
        discontinuity or artifact from optimizer closure, NOT if low speed is due to tight corners.

        Checks for:
        1. Unrealistic velocity jumps/discontinuities at closure point
        2. Standing start artifacts (v≈0 when surrounding sections are much faster)

        Does NOT smooth if low velocity is justified by high curvature.
        """
        if len(waypoints) < 20:
            return waypoints

        print("   🔧 Checking for velocity artifacts at start/end...")

        # Analyze curvature and velocity relationship at start/end
        # Smaller region (5% instead of 10%)
        region_size = max(10, len(waypoints) // 20)

        # Get velocities and curvatures near start
        start_section = waypoints[:region_size]
        end_section = waypoints[-region_size:]

        # Check curvature at start - if high curvature, low speed is expected
        start_curvatures = [abs(wp['kappa_radpm']) for wp in start_section]
        end_curvatures = [abs(wp['kappa_radpm']) for wp in end_section]
        max_start_curv = max(start_curvatures)
        max_end_curv = max(end_curvatures)

        # Get velocity at closure point
        closure_vel = waypoints[0]['vx_mps']

        # Get velocities slightly away from closure (20-30% into track)
        sample_start = len(waypoints) // 5
        sample_end = 3 * len(waypoints) // 10
        nearby_velocities = [wp['vx_mps']
                             for wp in waypoints[sample_start:sample_end]]
        nearby_curvatures = [abs(wp['kappa_radpm'])
                             for wp in waypoints[sample_start:sample_end]]
        avg_nearby_vel = np.mean(nearby_velocities)
        avg_nearby_curv = np.mean(nearby_curvatures)

        # Determine if low speed at start is justified by curvature
        # If start has high curvature relative to average, low speed is expected
        curvature_ratio_start = max_start_curv / (avg_nearby_curv + 0.01)
        curvature_ratio_end = max_end_curv / (avg_nearby_curv + 0.01)

        # Check for velocity discontinuity at closure (compare first and last waypoints)
        closure_discontinuity = abs(
            waypoints[0]['vx_mps'] - waypoints[-1]['vx_mps'])

        # Only smooth if there's an artifact, not if curvature justifies low speed
        should_smooth = False

        if closure_discontinuity > 2.0:  # Large velocity jump at closure
            print(
                f"   ⚠️  Detected closure discontinuity: {closure_discontinuity:.2f} m/s jump")
            should_smooth = True
        elif curvature_ratio_start < 1.5 and closure_vel < avg_nearby_vel * 0.5:
            # Low speed but NOT high curvature - likely an artifact
            print(
                f"   ⚠️  Detected velocity artifact: {closure_vel:.2f} m/s at low curvature section")
            should_smooth = True
        else:
            print(f"   ✅ Velocity profile respects geometry:")
            print(
                f"      Start: v={closure_vel:.2f} m/s, κ={max_start_curv:.3f} rad/m (ratio: {curvature_ratio_start:.1f}x)")
            print(
                f"      Nearby: v={avg_nearby_vel:.2f} m/s, κ={avg_nearby_curv:.3f} rad/m")

        if should_smooth:
            # Apply light smoothing only at the immediate closure point
            # Very small region (2.5%)
            smooth_region = min(5, len(waypoints) // 40)

            # Gentle 3-point moving average
            for i in range(smooth_region):
                prev_idx = (i - 1) % len(waypoints)
                next_idx = (i + 1) % len(waypoints)
                waypoints[i]['vx_mps'] = (waypoints[prev_idx]['vx_mps'] +
                                          waypoints[i]['vx_mps'] +
                                          waypoints[next_idx]['vx_mps']) / 3.0

            # Same for end
            for i in range(smooth_region):
                idx = len(waypoints) - smooth_region + i
                prev_idx = idx - 1
                next_idx = (idx + 1) % len(waypoints)
                waypoints[idx]['vx_mps'] = (waypoints[prev_idx]['vx_mps'] +
                                            waypoints[idx]['vx_mps'] +
                                            waypoints[next_idx]['vx_mps']) / 3.0

            # Recalculate accelerations only in smoothed region
            for i in list(range(smooth_region)) + list(range(len(waypoints) - smooth_region, len(waypoints))):
                next_i = (i + 1) % len(waypoints)
                dv = waypoints[next_i]['vx_mps'] - waypoints[i]['vx_mps']
                ds = max(0.01, waypoints[next_i]['s_m'] - waypoints[i]['s_m'])
                v_avg = (waypoints[i]['vx_mps'] +
                         waypoints[next_i]['vx_mps']) / 2.0
                if v_avg > 0.1:
                    waypoints[i]['ax_mps2'] = (dv * v_avg) / ds
                else:
                    waypoints[i]['ax_mps2'] = 0.0

            print(f"   ✅ Applied light smoothing at closure point")

        return waypoints

    def _create_fallback_sp_trajectory(self, centerline_waypoints: List[Dict]) -> List[Dict]:
        """Create a fallback shortest path from centerline with moderate speeds."""
        print("   🔄 Creating fallback trajectory from centerline...")
        print("   📉 Reducing speeds by 15% for safety")
        fallback_waypoints = []

        for wp in centerline_waypoints:
            # Reduce speed by 15% from centerline for SP
            fallback_wp = wp.copy()
            fallback_wp['vx_mps'] = wp['vx_mps'] * 0.85
            fallback_waypoints.append(fallback_wp)

        return fallback_waypoints

    def parse(self) -> bool:
        """Main parsing function with clean separation."""
        print(f"\n{'='*60}")
        print(f"🚀 Basic ETH to ETH Map Parser")
        print(f"{'='*60}")
        print(f"✨ Key Feature: Re-scale and regenerate from existing maps\n")

        try:
            # Step 1 & 2: Load and scale all data upfront (CLEAN SEPARATION)
            trajectory_data = self.load_and_scale_json()

            # Step 2.5: Generate trajectories if not disabled
            if self.racing_line_type != 'disable':
                print(f"\n{'='*60}")
                print(f"🎯 Step 2.5: Generating Optimized Trajectories")
                print(f"{'='*60}")

                # Generate shortest path - FAIL FAST if it fails
                sp_waypoints = self.generate_shortest_path(trajectory_data)
                if not sp_waypoints or len(sp_waypoints) == 0:
                    print(f"\n{'='*80}")
                    print(f"❌ CRITICAL ERROR: Shortest Path Generation FAILED!")
                    print(f"{'='*80}")
                    print(
                        f"The trajectory optimizer failed to generate a valid shortest path.")
                    print(f"Possible causes:")
                    print(f"  - Track boundaries are too narrow")
                    print(f"  - Track has invalid geometry")
                    print(f"  - Optimizer failed to converge")
                    print(f"\nCannot proceed without valid shortest path trajectory.")
                    sys.exit(1)
                trajectory_data['sp'] = sp_waypoints
                print(
                    f"✅ Shortest path generated: {len(sp_waypoints)} waypoints")

                # Generate racing line - FAIL FAST if it fails
                racing_waypoints = self.generate_racing_line(trajectory_data)
                if not racing_waypoints or len(racing_waypoints) == 0:
                    print(f"\n{'='*80}")
                    print(f"❌ CRITICAL ERROR: Racing Line Generation FAILED!")
                    print(f"{'='*80}")
                    print(
                        f"The trajectory optimizer failed to generate a valid racing line.")
                    print(f"Possible causes:")
                    print(f"  - Track boundaries are too narrow")
                    print(f"  - Track has invalid geometry")
                    print(f"  - Optimizer failed to converge")
                    print(
                        f"  - Car parameters ({self.car_name}) incompatible with track")
                    print(f"\nCannot proceed without valid racing line trajectory.")
                    sys.exit(1)
                trajectory_data['iqp'] = racing_waypoints
                print(
                    f"✅ Racing line generated: {len(racing_waypoints)} waypoints")
                print(f"\n✅ Trajectory generation complete!")

            # Step 2.9: Recalculate arc lengths and resample to uniform spacing
            # This fixes position tracking errors while maintaining optimizer convergence
            print(f"\n{'='*60}")
            print(f"🔧 Final Step: Fixing Position Tracking")
            print(f"{'='*60}")
            print(f"  ℹ️  Step 1: Recalculating s_m from actual geometric distances")
            print(
                f"  ℹ️  Step 2: Resampling to uniform 0.1m spacing (required by Frenet converter)")

            # First recalculate arc lengths from geometry
            trajectory_data['centerline'] = self._recalculate_arc_length(
                trajectory_data['centerline'], "🔵 Centerline")

            if trajectory_data.get('iqp'):
                trajectory_data['iqp'] = self._recalculate_arc_length(
                    trajectory_data['iqp'], "🔴 IQP (racing line)")

            if trajectory_data.get('sp'):
                trajectory_data['sp'] = self._recalculate_arc_length(
                    trajectory_data['sp'], "🟢 SP (shortest path)")

            # Then resample to uniform spacing for Frenet converter compatibility
            print(f"\n  🔄 Resampling waypoints to uniform spacing...")
            print(f"  ⚠️  CRITICAL: Frenet converter assumes uniform 0.1m spacing")

            trajectory_data['centerline'] = self._resample_to_uniform_spacing(
                trajectory_data['centerline'], spacing=0.1, name="🔵 Centerline")

            if trajectory_data.get('iqp'):
                trajectory_data['iqp'] = self._resample_to_uniform_spacing(
                    trajectory_data['iqp'], spacing=0.1, name="🔴 IQP (racing line)")

            if trajectory_data.get('sp'):
                trajectory_data['sp'] = self._resample_to_uniform_spacing(
                    trajectory_data['sp'], spacing=0.1, name="🟢 SP (shortest path)")

            # Step 3: Create output directory
            print(f"\n{'='*60}")
            print(f"📦 Step 3: Creating Output Files")
            print(f"{'='*60}")
            output_dir = self.create_output_directory()

            # Create track image
            origin_x, origin_y = self.create_track_image(
                output_dir, trajectory_data)

            # Create global waypoints JSON
            print(f"\n📝 Creating global waypoints JSON...")
            global_waypoints = self.create_global_waypoints_json(
                trajectory_data)
            json_path = os.path.join(output_dir, 'global_waypoints.json')
            with open(json_path, 'w') as f:
                json.dump(global_waypoints, f, indent=2)
            print(f"✅ Written: {json_path}")

            # Create YAML files
            print(f"\n📄 Creating configuration YAML files...")
            self.create_map_yaml(
                trajectory_data['centerline'], output_dir, origin_x, origin_y)
            self.create_ot_sectors_yaml(
                trajectory_data['centerline'], output_dir)
            self.create_speed_scaling_yaml(
                trajectory_data['centerline'], output_dir)
            self.create_starting_position_yaml(trajectory_data, output_dir)

            # Summary
            print(f"\n{'='*60}")
            print(f"🎉 Conversion Complete!")
            print(f"{'='*60}")
            print(f"📁 Output directory: {output_dir}")
            print(f"🗺️  Map name: {self.output_map_name}")
            print(f"\n📊 Trajectories generated:")
            print(
                f"  🔵 Centerline: {len(trajectory_data['centerline'])} waypoints")
            print(
                f"  🔴 IQP (racing line): {len(trajectory_data['iqp'])} waypoints")
            print(
                f"  🟢 SP (shortest path): {len(trajectory_data.get('sp', []))} waypoints")
            print(f"\n✨ Key improvements applied:")
            print(
                f"  ✅ All data scaled upfront (scale factor: {self.scale_factor})")
            print(f"  ✅ Track translated to origin (0,0)")
            print(
                f"  ✅ Width multiplier applied during scaling: {self.width_multiplier}")
            print(f"  ✅ No mixed scaled/unscaled data")
            print(f"\n🚀 Test with:")
            print(
                f"  roslaunch stack_master base_system.launch map_name:={self.output_map_name}")

            return True

        except Exception as e:
            print(f"\n✗ Error during parsing: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    """Main function for command line execution."""
    parser = argparse.ArgumentParser(
        description="Basic ETH to ETH Map Parser - Re-scale and regenerate from existing maps")

    parser.add_argument("json_file", help="Path to global_waypoints.json file")
    parser.add_argument("--output-name", default="marina",
                        help="Output map name (default: marina)")
    parser.add_argument("--scale-factor", type=float, default=0.1,
                        help="Scale factor for map size (default: 0.1)")
    parser.add_argument("--width-multiplier", type=float, default=1.0,
                        help="Track width multiplier (default: 1.0 - preserves original)")
    parser.add_argument("--car-name", default="NUC2",
                        help="Car configuration name (default: NUC2)")
    parser.add_argument("--racing-line-type", default="mintime",
                        choices=["mintime", "mincurv", "disable"],
                        help="Racing line optimization type (default: mintime)")

    args = parser.parse_args()

    try:
        # Create parser instance
        basic_parser = BasicETHToETHMapParser(
            json_file=args.json_file,
            output_map_name=args.output_name,
            scale_factor=args.scale_factor,
            width_multiplier=args.width_multiplier,
            car_name=args.car_name,
            racing_line_type=args.racing_line_type
        )

        # Parse and generate
        success = basic_parser.parse()

        if success:
            print("\n✓ Conversion completed successfully!")
            sys.exit(0)
        else:
            print("\n✗ Conversion failed!")
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nConversion interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
