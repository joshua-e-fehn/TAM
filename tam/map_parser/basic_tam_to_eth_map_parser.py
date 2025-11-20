#!/usr/bin/env python3
"""
Basic TAM to ETH Map Parser - Simplified version with clean scaling separation.

This script converts TAM/Marina raceline CSV data into the F1Tenth race stack 
map format, specifically generating the global_waypoints.json file and 
configuration files for the specified map.

KEY IMPROVEMENTS:
- Clean separation: All input data scaled upfront before any processing
- Translation to origin (0,0) applied during scaling
- Trajectory generation works on pre-scaled data only
- No mixed scaled/unscaled data confusion
- Default width multiplier = 1.0 (preserves original track width)

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


class BasicTAMToETHMapParser:
    def __init__(self, csv_file: str, output_map_name: str = "marina",
                 scale_factor: float = 0.1, width_multiplier: float = 1.0,
                 car_name: str = "NUC2", racing_line_type: str = "mintime"):
        """
        Initialize the basic TAM to ETH map parser.

        Args:
            csv_file: Path to the TAM CSV file
            output_map_name: Base name for the output map directory
            scale_factor: Scale factor to reduce map size (default: 0.1 = 10% of original size)
            width_multiplier: Multiplier for track width (default: 1.0 = preserve original width)
            car_name: Name of car configuration to use for trajectory optimization (default: "NUC2")
            racing_line_type: Type of racing line optimization (default: "mintime")
        """
        self.csv_file = csv_file
        self.base_map_name = output_map_name
        self.scale_factor = scale_factor
        self.width_multiplier = width_multiplier
        self.car_name = car_name
        self.racing_line_type = racing_line_type

        # Generate the full output map name based on parameters
        self.output_map_name = self.generate_map_name()

        # Set up cache directory
        self.cache_dir = os.path.join(os.path.dirname(csv_file), "cache")
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

        # Define column mapping based on CSV header analysis
        self.column_mapping = {
            'rl_x_m': 0, 'rl_y_m': 1, 'rl_vx_mps': 3, 'rl_psi_rad': 5,
            'rl_ax_mps2': 6, 'rl_n_m': 4, 'ref_rl_s_m': 11, 'ref_rl_x_m': 12,
            'ref_rl_y_m': 13, 'ref_rl_psi_rad': 15, 'ref_rl_kappa_radpm': 18,
            'ref_rl_d_right': 21, 'ref_rl_d_left': 22, 'ref_cl_s_m': 26,
            'ref_cl_x_m': 27, 'ref_cl_y_m': 28, 'ref_cl_psi_rad': 30,
            'ref_cl_kappa_radpm': 33, 'ref_cl_d_right': 36, 'ref_cl_d_left': 37,
            'tb_left_x': 41, 'tb_left_y': 42, 'tb_right_x': 44, 'tb_right_y': 45,
        }

    def generate_map_name(self) -> str:
        """Generate the map name based on parameters."""
        size_percent = int(self.scale_factor * 100)
        width_percent = int(self.width_multiplier * 100)
        return f"{self.base_map_name}_{size_percent}%s_{width_percent}%w_{self.car_name}_{self.racing_line_type}"

    def safe_float(self, value_str: str, default: float = 0.0) -> float:
        """Safely convert string to float, handling 'nan' values."""
        try:
            val = float(value_str)
            return default if (math.isnan(val) or math.isinf(val)) else val
        except (ValueError, TypeError):
            return default

    def load_and_scale_csv(self) -> Dict[str, List[Dict]]:
        """
        Load CSV and immediately scale all data - CLEAN SEPARATION.

        This is the key improvement: everything is scaled upfront, including:
        - Coordinates (x, y)
        - Distances (s, d_left, d_right)
        - Velocities (scaled proportionally)
        - Curvatures (inverse scaled)
        - Translation to origin (0,0)

        Returns:
            Dictionary with scaled trajectory data ready for use
        """
        print(f"\n{'='*60}")
        print(f"📂 Step 1: Loading and Scaling CSV Data")
        print(f"{'='*60}")
        print(f"📄 Loading TAM CSV: {self.csv_file}")

        # Read CSV, skip header lines
        data_lines = []
        with open(self.csv_file, 'r') as f:
            for line in f:
                line = line.strip()
                # Skip comments AND header line (starts with 'x_rl_m')
                if line and not line.startswith('#') and not line.startswith('x_rl_m'):
                    data_lines.append(line)

        print(f"✅ Found {len(data_lines)} data lines")

        # Parse raw waypoints (unscaled)
        raw_centerline = []
        raw_iqp = []
        raw_trackbounds_left = []
        raw_trackbounds_right = []

        for i, line in enumerate(data_lines):
            values = line.split(',')
            if len(values) < 46:
                continue

            raw_centerline.append(
                self._create_raw_centerline_waypoint(values, i))

            # Only add IQP waypoint if it has valid racing line data
            raw_iqp_wp = self._create_raw_iqp_waypoint(values, i)
            # Skip waypoints with zero coordinates (from NaN fallback)
            if raw_iqp_wp['x_m'] != 0.0 or raw_iqp_wp['y_m'] != 0.0:
                raw_iqp.append(raw_iqp_wp)

            # Extract trackbounds
            tb_left_x = self.safe_float(
                values[self.column_mapping['tb_left_x']])
            tb_left_y = self.safe_float(
                values[self.column_mapping['tb_left_y']])
            tb_right_x = self.safe_float(
                values[self.column_mapping['tb_right_x']])
            tb_right_y = self.safe_float(
                values[self.column_mapping['tb_right_y']])

            if tb_left_x != 0.0 or tb_left_y != 0.0:
                raw_trackbounds_left.append(
                    {'x_m': tb_left_x, 'y_m': tb_left_y})
            if tb_right_x != 0.0 or tb_right_y != 0.0:
                raw_trackbounds_right.append(
                    {'x_m': tb_right_x, 'y_m': tb_right_y})

        print(f"\n📊 Parsed raw waypoints:")
        print(f"  🔵 Centerline: {len(raw_centerline)} waypoints")
        print(f"  🔴 IQP (racing line): {len(raw_iqp)} waypoints")
        print(
            f"  🟡 Trackbounds: {len(raw_trackbounds_left)} left, {len(raw_trackbounds_right)} right")

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

        # Scale all trajectories
        scaled_centerline = [self._scale_waypoint(wp) for wp in raw_centerline]
        scaled_iqp = [self._scale_waypoint(wp) for wp in raw_iqp]
        scaled_trackbounds_left = [self._scale_trackbound_point(
            pt) for pt in raw_trackbounds_left]
        scaled_trackbounds_right = [self._scale_trackbound_point(
            pt) for pt in raw_trackbounds_right]

        print(f"\n✅ All data scaled and translated to origin")
        print(f"  🔵 Centerline: {len(scaled_centerline)} waypoints (scaled)")
        print(f"  🔴 IQP: {len(scaled_iqp)} waypoints (scaled)")
        print(
            f"  🟡 Trackbounds: {len(scaled_trackbounds_left)} left, {len(scaled_trackbounds_right)} right (scaled)")

        # Validate track closure
        print(f"\n🔍 Validating track closure...")
        self._validate_track_closure(scaled_centerline, "🔵 Centerline")
        self._validate_track_closure(scaled_iqp, "🔴 IQP")

        return {
            'centerline': scaled_centerline,
            'iqp': scaled_iqp,
            'trackbounds_left': scaled_trackbounds_left,
            'trackbounds_right': scaled_trackbounds_right,
            'sp': []  # Will be generated later
        }

    def _create_raw_centerline_waypoint(self, values: List[str], waypoint_id: int) -> Dict:
        """Create raw (unscaled) centerline waypoint from CSV data."""
        x_m = self.safe_float(values[self.column_mapping['ref_cl_x_m']])
        y_m = self.safe_float(values[self.column_mapping['ref_cl_y_m']])
        s_m = self.safe_float(values[self.column_mapping['ref_cl_s_m']])
        psi_rad = self.safe_float(
            values[self.column_mapping['ref_cl_psi_rad']])
        kappa_radpm = self.safe_float(
            values[self.column_mapping['ref_cl_kappa_radpm']])

        # Track widths (will be multiplied by width_multiplier during scaling)
        d_right = abs(self.safe_float(
            values[self.column_mapping['ref_cl_d_right']], 2.0))
        d_left = abs(self.safe_float(
            values[self.column_mapping['ref_cl_d_left']], 2.0))

        # Conservative speed profile based on curvature
        base_speed = 15.0
        min_speed = 5.0
        abs_curvature = abs(kappa_radpm)
        if abs_curvature > 0.001:
            curvature_speed = min(base_speed, math.sqrt(8.0 / abs_curvature))
            vx_mps = max(min_speed, curvature_speed)
        else:
            vx_mps = base_speed

        return {
            'id': waypoint_id, 's_m': s_m, 'd_m': 0.0, 'x_m': x_m, 'y_m': y_m,
            'd_right': d_right, 'd_left': d_left, 'psi_rad': psi_rad,
            'kappa_radpm': kappa_radpm, 'vx_mps': vx_mps, 'ax_mps2': 0.0
        }

    def _create_raw_iqp_waypoint(self, values: List[str], waypoint_id: int) -> Dict:
        """Create raw (unscaled) IQP waypoint from CSV data."""
        # Try racing line data first, fallback to centerline
        x_m = self.safe_float(values[self.column_mapping['rl_x_m']])
        y_m = self.safe_float(values[self.column_mapping['rl_y_m']])
        if x_m == 0.0 and y_m == 0.0:
            x_m = self.safe_float(values[self.column_mapping['ref_rl_x_m']])
            y_m = self.safe_float(values[self.column_mapping['ref_rl_y_m']])

        psi_rad = self.safe_float(values[self.column_mapping['rl_psi_rad']])
        if psi_rad == 0.0:
            psi_rad = self.safe_float(
                values[self.column_mapping['ref_rl_psi_rad']])

        vx_mps = self.safe_float(
            values[self.column_mapping['rl_vx_mps']], 15.0)
        ax_mps2 = self.safe_float(
            values[self.column_mapping['rl_ax_mps2']], 0.0)

        s_m = self.safe_float(values[self.column_mapping['ref_rl_s_m']])
        if s_m == 0.0:
            s_m = self.safe_float(values[self.column_mapping['ref_cl_s_m']])

        kappa_radpm = self.safe_float(
            values[self.column_mapping['ref_rl_kappa_radpm']])
        if kappa_radpm == 0.0:
            kappa_radpm = self.safe_float(
                values[self.column_mapping['ref_cl_kappa_radpm']])

        d_right = abs(self.safe_float(
            values[self.column_mapping['ref_rl_d_right']], 0.0))
        d_left = abs(self.safe_float(
            values[self.column_mapping['ref_rl_d_left']], 0.0))

        if d_right == 0.0:
            d_right = abs(self.safe_float(
                values[self.column_mapping['ref_cl_d_right']], 2.0))
        if d_left == 0.0:
            d_left = abs(self.safe_float(
                values[self.column_mapping['ref_cl_d_left']], 2.0))

        return {
            'id': waypoint_id, 's_m': s_m, 'd_m': 0.0, 'x_m': x_m, 'y_m': y_m,
            'd_right': d_right, 'd_left': d_left, 'psi_rad': psi_rad,
            'kappa_radpm': kappa_radpm, 'vx_mps': vx_mps, 'ax_mps2': ax_mps2
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
            "~/catkin_ws/src/race_stack/tam/maps/output")
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
            'est_lap_time': {'data': sp_lap_time if sp_waypoints else iqp_lap_time},
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
                's_m': {'data': wp['s_m']},
                'd_m': {'data': wp['d_m']},
                'd_right': {'data': wp['d_right']},
                'd_left': {'data': wp['d_left']},
                'x_m': {'data': wp['x_m']},
                'y_m': {'data': wp['y_m']},
                'psi_rad': {'data': wp['psi_rad']},
                'kappa_radpm': {'data': wp['kappa_radpm']},
                'vx_mps': {'data': wp['vx_mps']},
                'ax_mps2': {'data': wp['ax_mps2']}
            })

        return {
            'header': {'seq': 1, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': ""},
            'wpnts': wpnt_list
        }

    def _create_waypoint_markers(self, waypoints: List[Dict], marker_type: str, color: Dict) -> Dict:
        """Create visualization markers for waypoints."""
        markers = []
        sample_rate = 8

        speeds = [wp['vx_mps'] for wp in waypoints]
        min_speed = min(speeds) if speeds else 1.0
        max_speed = max(speeds) if speeds else 10.0

        for i, wp in enumerate(waypoints[::sample_rate]):
            speed_ratio = (wp['vx_mps'] - min_speed) / \
                (max_speed - min_speed + 0.01)
            scale = 0.05 * (1.0 + 4.0 * speed_ratio)

            markers.append({
                'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': 'map'},
                'ns': marker_type,
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
                'frame_locked': False
            })

        return {'markers': markers}

    def _create_trackbounds_markers(self, trajectory_data: Dict) -> Dict:
        """Create visualization markers for track boundaries (already scaled)."""
        markers = []
        sample_rate = 8

        trackbounds_left = trajectory_data.get('trackbounds_left', [])
        trackbounds_right = trajectory_data.get('trackbounds_right', [])

        marker_id = 0
        for i, pt in enumerate(trackbounds_left[::sample_rate]):
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

        for i, pt in enumerate(trackbounds_right[::sample_rate]):
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
                print("🏎️  Safety margin: 0.3m (aggressive for racing)\n")

                current_dir = os.getcwd()
                optimized_trajectory, bound_r, bound_l, lap_time = trajectory_optimizer(
                    input_path=current_dir,
                    track_name=track_name,
                    curv_opt_type=self.racing_line_type,
                    safety_width=0.3,  # Aggressive safety margin for racing
                    plot=False
                )

                print(f"\n✅ Racing line generated successfully!")
                print(f"⏱️  Estimated lap time: {lap_time:.3f}s")

                # Convert result to waypoints (already scaled format)
                print(f"🔄 Converting optimizer output to waypoint format...")
                racing_waypoints = self._convert_optimizer_result_to_waypoints(
                    optimized_trajectory, centerline_waypoints)

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
                max_velocity = car_data.get('v_max', car_data.get('max_velocity', max_velocity))
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
            f.write(
                f'stepsize_opts={{"stepsize_prep": 1.0, "stepsize_reg": 3.0, "stepsize_interp_after_opt": 0.2}}\n')
            f.write(
                f'reg_smooth_opts={{"k_reg": 3, "s_reg": 3.0}}\n')
            f.write(
                f'curv_calc_opts={{"d_preview_curv": 1.0, "d_review_curv": 1.0, "d_preview_head": 1.0, "d_review_head": 1.0}}\n')
            f.write(
                f'veh_params = {{"v_max": {max_velocity}, "length": {wheelbase}, "width": 0.31, "mass": {mass}, "dragcoeff": 0.05, "curvlim": {max_steering}, "g": 9.81}}\n')
            f.write(
                'vel_calc_opts={"dyn_model_exp": 1.0, "vel_profile_conv_filt_window": null}\n\n')
            f.write("[OPTIMIZATION_OPTIONS]\n")
            f.write(f'optim_opts_shortest_path={{"width_opt": 0.5}}\n')
            f.write(
                f'optim_opts_mincurv={{"width_opt": 0.3, "iqp_iters_min": 3, "iqp_curverror_allowed": 0.01}}\n')
            f.write(
                f'optim_opts_mintime={{"width_opt": 0.3, "penalty_delta": 50.0, "penalty_F": 0.01, "mue": 1.0, "n_gauss": 5, "dn": 0.25, "limit_energy": false, "energy_limit": 2.0, "safe_traj": false, "ax_pos_safe": null, "ax_neg_safe": null, "ay_safe": null, "w_tr_reopt": 2.0, "w_veh_reopt": 0.3, "w_add_spl_regr": 0.2, "step_non_reg": 0, "eps_kappa": 0.001}}\n\n')

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
        
        # Lateral acceleration limit (typically 1-1.5x longitudinal for race cars)
        # For F1Tenth, use similar magnitude as longitudinal
        max_lateral_accel = max_accel * 1.2
        
        with open("ggv.csv", "w") as f:
            f.write("# v_mps, ax_max_mps2, ay_max_mps2\n")
            f.write(f"# Generated for {self.car_name}: max_accel={max_accel}m/s², max_velocity={max_velocity}m/s\n")
            
            # Generate GGV data with velocity-dependent acceleration limits
            # Acceleration decreases with velocity due to power/drag limitations
            for v in np.linspace(0.5, max_velocity * 1.5, 30):
                # Linear decrease: full accel at low speed, ~60% at max speed
                velocity_factor = max(0.6, 1.0 - 0.4 * (v / max_velocity))
                ax_max = max_accel * velocity_factor
                ay_max = max_lateral_accel * velocity_factor
                f.write(f"{v:.2f}, {ax_max:.2f}, {ay_max:.2f}\n")
        
        print(f"   ✅ Created GGV diagram (max: {max_accel:.1f}m/s² longitudinal, {max_lateral_accel:.1f}m/s² lateral)")

    def _create_ax_max_file(self):
        """Create ax_max curve for optimizer using car-specific limits."""
        # Use car-specific parameters (set in _create_vehicle_params_file)
        max_velocity = getattr(self, '_car_max_velocity', 10.0)
        max_accel = getattr(self, '_car_max_accel', 3.0)
        max_decel = getattr(self, '_car_max_decel', 3.0)
        
        with open("ax_max_machines.csv", "w") as f:
            f.write("# v_mps, ax_max_machines_mps2\n")
            f.write(f"# Generated for {self.car_name}: accel={max_accel}m/s², decel={max_decel}m/s²\n")
            
            # Generate acceleration limits for both positive (accel) and negative (brake)
            for v in np.linspace(0.0, max_velocity * 1.5, 30):
                # Positive acceleration: decreases with velocity (power limit)
                velocity_factor = max(0.6, 1.0 - 0.4 * (v / max_velocity))
                ax_max_accel = max_accel * velocity_factor
                
                # Negative acceleration (braking): more constant, slight decrease at very high speed
                ax_max_brake = -max_decel * max(0.9, 1.0 - 0.1 * (v / max_velocity))
                
                # Write both limits (optimizer uses envelope)
                f.write(f"{v:.2f}, {ax_max_accel:.2f}\n")
                
        print(f"   ✅ Created ax_max curve (accel: {max_accel:.1f}m/s², decel: {max_decel:.1f}m/s²)")

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

            # Get track bounds from nearest reference waypoint
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
                's_m': optimized_trajectory[i, 0],  # arc length
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
        print(f"🚀 Basic TAM to ETH Map Parser")
        print(f"{'='*60}")
        print(f"✨ Key Feature: Clean scaling separation - all data scaled upfront\n")

        try:
            # Step 1 & 2: Load and scale all data upfront (CLEAN SEPARATION)
            trajectory_data = self.load_and_scale_csv()

            # Step 2.5: Generate shortest path if not disabled
            if self.racing_line_type != 'disable':
                print(f"\n{'='*60}")
                print(f"🎯 Step 2.5: Generating Optimized Trajectories")
                print(f"{'='*60}")

                # Generate shortest path
                sp_waypoints = self.generate_shortest_path(trajectory_data)
                trajectory_data['sp'] = sp_waypoints

                # Generate racing line (or use existing IQP if disabled)
                racing_waypoints = self.generate_racing_line(trajectory_data)
                if racing_waypoints:  # Only replace if generation succeeded
                    trajectory_data['iqp'] = racing_waypoints
                    print(f"\n✅ Trajectory generation complete!")
                else:
                    print(f"\n⚠️  Racing line generation failed, using CSV IQP data")

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
        description="Basic TAM to ETH Map Parser - Clean scaling separation version")

    parser.add_argument("csv_file", help="Path to TAM CSV file")
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
        basic_parser = BasicTAMToETHMapParser(
            csv_file=args.csv_file,
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
