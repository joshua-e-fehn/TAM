#!/usr/bin/env python3
"""
Parser to convert Marina map CSV format to F1Tenth Race Stack format with optimized racing lines.

This script converts the Marina raceline CSV data into the F1Tenth race stack 
map format, specifically generating the global_waypoints.json file and 
configuration files for the specified map.

NEW FEATURES:
- Generates car-specific optimized racing lines using trajectory optimization
- Supports minimum lap time optimization for aggressive racing lines
- Supports minimum curvature optimization for smooth racing lines
- Uses actual car parameters (mass, wheelbase, steering limits, etc.)
- Generates realistic speed profiles based on vehicle dynamics
- Replaces pre-calculated racing line data with optimized trajectories

Author: Assistant
Date: 2025
"""

import json
import math
import os
import shutil
import signal
import sys
import yaml
import hashlib
from typing import Dict, List, Any
import argparse
import numpy as np
from PIL import Image, ImageDraw


class MarinaMapParser:
    def __init__(self, csv_file: str, output_map_name: str = "marina", scale_factor: float = 0.1, width_multiplier: float = 2.0, car_name: str = "NUC2", racing_line_type: str = "mintime"):
        """
        Initialize the Marina map parser.

        Args:
            csv_file: Path to the Marina CSV file
            output_map_name: Base name for the output map directory (will be extended with parameters)
            scale_factor: Scale factor to reduce map size (default: 0.1 = 10% of original size)
            width_multiplier: Multiplier for track width (default: 2.0 = double width)
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

        print(
            f"Using scale factor: {scale_factor} (map will be {scale_factor*100:.1f}% of original size)")
        print(
            f"Using width multiplier: {width_multiplier} (track will be {width_multiplier*100:.0f}% of original width)")
        print(f"Generated map name: {self.output_map_name}")
        print(
            f"Using car configuration: {car_name} for trajectory optimization")
        print(f"Cache directory: {self.cache_dir}")

        # Define column mapping based on CSV header analysis
        # Marina CSV columns: x_rl_m, y_rl_m, z_rl_m, v_rl_mps, n_rl_m, chi_rl_rad, ax_rl_mps2, ay_rl_mps2, jx_rl_mps3, jy_rl_mps3, tire_util_rl, s_ref_rl_m, x_ref_rl_m, y_ref_rl_m, z_ref_rl_m, theta_ref_rl_rad, mu_ref_rl_rad, phi_ref_rl_rad, dtheta_ref_rl_radpm, dmu_ref_rl_radpm, dphi_ref_rl_radpm, w_tr_right_ref_rl_m, w_tr_left_ref_rl_m, omega_x_ref_rl_radpm, omega_y_ref_rl_radpm, omega_z_ref_rl_radpm, s_ref_cl_m, x_ref_cl_m, y_ref_cl_m, z_ref_cl_m, theta_ref_cl_rad, mu_ref_cl_rad, phi_ref_cl_rad, dtheta_ref_cl_radpm, dmu_ref_cl_radpm, dphi_ref_cl_radpm, w_tr_right_ref_cl_m, w_tr_left_ref_cl_m, omega_x_ref_cl_radpm, omega_y_ref_cl_radpm, omega_z_ref_cl_radpm, tb_left_x_ref_rl_m, tb_left_y_ref_rl_m, tb_left_z_ref_rl_m, tb_right_x_ref_rl_m, tb_right_y_ref_rl_m, tb_right_z_ref_rl_m
        # Column mapping for Marina CSV format - corrected based on actual data structure
        self.column_mapping = {
            # Raw racing line data (most aggressive - use for IQP)
            'rl_x_m': 0,        # x_rl_m
            'rl_y_m': 1,        # y_rl_m
            'rl_vx_mps': 3,     # v_rl_mps
            'rl_psi_rad': 5,    # chi_rl_rad
            'rl_ax_mps2': 6,    # ax_rl_mps2
            'rl_n_m': 4,        # n_rl_m (lateral offset)

            # Reference racing line data (refined - use for SP)
            'ref_rl_s_m': 11,       # s_ref_rl_m
            'ref_rl_x_m': 12,       # x_ref_rl_m
            'ref_rl_y_m': 13,       # y_ref_rl_m
            'ref_rl_psi_rad': 15,   # theta_ref_rl_rad
            'ref_rl_kappa_radpm': 18,  # dtheta_ref_rl_radpm (curvature)
            'ref_rl_d_right': 21,   # w_tr_right_ref_rl_m
            'ref_rl_d_left': 22,    # w_tr_left_ref_rl_m

            # Reference centerline data (conservative - use for centerline)
            'ref_cl_s_m': 26,       # s_ref_cl_m
            'ref_cl_x_m': 27,       # x_ref_cl_m
            'ref_cl_y_m': 28,       # y_ref_cl_m
            'ref_cl_psi_rad': 30,   # theta_ref_cl_rad
            'ref_cl_kappa_radpm': 33,  # dtheta_ref_cl_radpm
            'ref_cl_d_right': 36,   # w_tr_right_ref_cl_m
            'ref_cl_d_left': 37,    # w_tr_left_ref_cl_m

            # Track boundary data
            'tb_left_x': 41,        # tb_left_x_ref_rl_m
            'tb_left_y': 42,        # tb_left_y_ref_rl_m
            'tb_right_x': 44,       # tb_right_x_ref_rl_m
            'tb_right_y': 45,       # tb_right_y_ref_rl_m
        }

    def generate_map_name(self) -> str:
        """Generate the map name based on parameters.

        Schema: "{map name}_{size multiplier in %}%s_{width multiplier in %}%w_{race car}_{racing line type}"

        Returns:
            Generated map name string
        """
        # Convert factors to percentages
        size_percent = int(self.scale_factor * 100)
        width_percent = int(self.width_multiplier * 100)

        # Generate the name according to schema
        map_name = f"{self.base_map_name}_{size_percent}%s_{width_percent}%w_{self.car_name}_{self.racing_line_type}"

        return map_name

    def get_cache_key(self, trajectory_type: str, car_name: str = None) -> str:
        """Generate a cache key for the given configuration.

        Args:
            trajectory_type: Type of trajectory ('shortest_path', 'racing_line')
            car_name: Car name for racing lines (ignored for shortest_path)

        Returns:
            String cache key
        """
        # Get CSV file hash for uniqueness
        with open(self.csv_file, 'rb') as f:
            csv_hash = hashlib.md5(f.read()).hexdigest()[:8]

        # Shortest path is independent of car configuration
        if trajectory_type == 'shortest_path':
            return f"sp_{csv_hash}_s{self.scale_factor}_w{self.width_multiplier}"

        # Racing line is car-specific
        if trajectory_type == 'racing_line':
            return f"rl_{csv_hash}_s{self.scale_factor}_w{self.width_multiplier}_{car_name or self.car_name}"

        return f"{trajectory_type}_{csv_hash}_s{self.scale_factor}_w{self.width_multiplier}"

    def save_trajectory_cache(self, trajectory_type: str, waypoints: List[Dict], optimization_type: str = None, car_name: str = None):
        """Save trajectory waypoints to cache.

        Args:
            trajectory_type: Type of trajectory ('shortest_path', 'racing_line')
            waypoints: List of waypoint dictionaries to cache
            optimization_type: Optimization type for racing lines ('mintime', 'mincurv')
            car_name: Car name for racing lines
        """
        if not waypoints:
            return

        cache_key = self.get_cache_key(trajectory_type, car_name)
        cache_file = os.path.join(self.cache_dir, f"{cache_key}.json")

        cache_data = {
            'trajectory_type': trajectory_type,
            'optimization_type': optimization_type,
            'car_name': car_name,
            'scale_factor': self.scale_factor,
            'width_multiplier': self.width_multiplier,
            'csv_file': os.path.basename(self.csv_file),
            'waypoints': waypoints,
            'metadata': {
                'waypoint_count': len(waypoints),
                'generated_timestamp': __import__('time').time()
            }
        }

        try:
            with open(cache_file, 'w') as f:
                json.dump(cache_data, f, indent=2)
            print(
                f"✓ Cached {len(waypoints)} {trajectory_type} waypoints to: {cache_file}")
        except Exception as e:
            print(f"⚠️  Failed to save cache: {e}")

    def load_trajectory_cache(self, trajectory_type: str, optimization_type: str = None, car_name: str = None) -> List[Dict]:
        """Load trajectory waypoints from cache.

        Args:
            trajectory_type: Type of trajectory ('shortest_path', 'racing_line')
            optimization_type: Required optimization type for racing lines
            car_name: Car name for racing lines

        Returns:
            List of cached waypoints or empty list if not found/invalid
        """
        # Check if force regeneration is enabled
        if getattr(self, 'force_regenerate', False):
            return []

        cache_key = self.get_cache_key(trajectory_type, car_name)
        cache_file = os.path.join(self.cache_dir, f"{cache_key}.json")

        if not os.path.exists(cache_file):
            return []

        try:
            with open(cache_file, 'r') as f:
                cache_data = json.load(f)

            # Validate cache data
            # Note: Cached waypoints are stored UNSCALED, so we don't validate scale_factor
            # Only validate width_multiplier and CSV file
            if (cache_data.get('width_multiplier') != self.width_multiplier or
                    cache_data.get('csv_file') != os.path.basename(self.csv_file)):
                print(
                    f"⚠️  Cache mismatch for {trajectory_type}, regenerating...")
                return []

            # For racing lines, also check optimization type
            if trajectory_type == 'racing_line' and optimization_type:
                if cache_data.get('optimization_type') != optimization_type:
                    print(
                        f"⚠️  Optimization type mismatch for {trajectory_type} (cached: {cache_data.get('optimization_type')}, requested: {optimization_type})")
                    return []

            waypoints = cache_data.get('waypoints', [])
            if waypoints:
                print(
                    f"✓ Loaded {len(waypoints)} cached {trajectory_type} waypoints from: {cache_file}")
                if optimization_type:
                    print(
                        f"  Optimization type: {cache_data.get('optimization_type')}")
                return waypoints

        except Exception as e:
            print(f"⚠️  Failed to load cache {cache_file}: {e}")

        return []

    def clear_cache(self, trajectory_type: str = None, car_name: str = None):
        """Clear cached trajectories.

        Args:
            trajectory_type: Specific trajectory type to clear, or None for all
            car_name: Specific car name to clear, or None for all
        """
        if not os.path.exists(self.cache_dir):
            return

        files_removed = 0
        for filename in os.listdir(self.cache_dir):
            if not filename.endswith('.json'):
                continue

            should_remove = False
            if trajectory_type is None:
                should_remove = True  # Clear all
            elif trajectory_type == 'shortest_path' and filename.startswith('sp_'):
                should_remove = True
            elif trajectory_type == 'racing_line' and filename.startswith('rl_'):
                if car_name is None or car_name in filename:
                    should_remove = True

            if should_remove:
                cache_file = os.path.join(self.cache_dir, filename)
                try:
                    os.remove(cache_file)
                    files_removed += 1
                except Exception as e:
                    print(f"⚠️  Failed to remove cache file {filename}: {e}")

        if files_removed > 0:
            print(f"✓ Removed {files_removed} cache files")

    def seed_cache_from_existing_maps(self):
        """Seed cache from existing marina map files if available."""
        print("\n=== Checking for existing marina maps to seed cache ===")

        # Look for existing marina maps in the output directory
        maps_dir = os.path.join(os.path.dirname(self.csv_file), "output")
        if not os.path.exists(maps_dir):
            print("No existing maps directory found")
            return

        # Find marina map directories
        marina_dirs = []
        for item in os.listdir(maps_dir):
            if item.startswith('marina') and os.path.isdir(os.path.join(maps_dir, item)):
                marina_dirs.append(item)

        if not marina_dirs:
            print("No existing marina maps found")
            return

        print(f"Found {len(marina_dirs)} existing marina maps: {marina_dirs}")

        # Try to extract shortest path and racing line data
        for map_dir in marina_dirs:
            global_waypoints_file = os.path.join(
                maps_dir, map_dir, "global_waypoints.json")
            if not os.path.exists(global_waypoints_file):
                continue

            try:
                with open(global_waypoints_file, 'r') as f:
                    data = json.load(f)

                # Extract shortest path if available and not already cached
                sp_waypoints_data = data.get(
                    'global_traj_wpnts_sp', {}).get('wpnts', [])
                if sp_waypoints_data and not self.load_trajectory_cache('shortest_path'):
                    # Convert from ROS message format to our waypoint format
                    sp_waypoints = self.convert_ros_waypoints_to_cache_format(
                        sp_waypoints_data)
                    if sp_waypoints:
                        print(
                            f"Seeding shortest path cache from {map_dir} ({len(sp_waypoints)} waypoints)")
                        self.save_trajectory_cache(
                            'shortest_path', sp_waypoints)

                # Extract racing line data if available
                iqp_waypoints_data = data.get(
                    'global_traj_wpnts_iqp', {}).get('wpnts', [])
                if iqp_waypoints_data:
                    # Try to determine what car this was generated for from the map name
                    car_from_name = self.extract_car_from_map_name(map_dir)
                    if car_from_name and not self.load_trajectory_cache('racing_line', 'mintime', car_from_name):
                        iqp_waypoints = self.convert_ros_waypoints_to_cache_format(
                            iqp_waypoints_data)
                        if iqp_waypoints:
                            print(
                                f"Seeding {car_from_name} racing line cache from {map_dir} ({len(iqp_waypoints)} waypoints)")
                            self.save_trajectory_cache(
                                'racing_line', iqp_waypoints, 'mintime', car_from_name)

            except Exception as e:
                print(f"⚠️  Failed to process {map_dir}: {e}")

    def extract_car_from_map_name(self, map_name: str) -> str:
        """Extract car name from map directory name."""
        # Common patterns: marina_test_car, marina_NUC2, etc.
        if 'test' in map_name.lower():
            return 'TEST'
        elif 'nuc2' in map_name.lower():
            return 'NUC2'
        elif 'tam' in map_name.lower():
            return 'TAM'
        return None

    def convert_ros_waypoints_to_cache_format(self, ros_waypoints: List[Dict]) -> List[Dict]:
        """Convert ROS waypoint format to our cache format.

        Args:
            ros_waypoints: List of ROS waypoint messages

        Returns:
            List of waypoints in our internal format
        """
        waypoints = []
        for i, ros_wp in enumerate(ros_waypoints):
            try:
                # ROS waypoint format: {'id': ..., 's_m': ..., 'd_m': ..., 'x_m': ..., etc.}
                waypoint = {
                    'id': ros_wp.get('id', i),
                    's_m': ros_wp.get('s_m', 0.0),
                    'd_m': ros_wp.get('d_m', 0.0),
                    'x_m': ros_wp.get('x_m', 0.0),
                    'y_m': ros_wp.get('y_m', 0.0),
                    'd_right': ros_wp.get('d_right', 2.0),
                    'd_left': ros_wp.get('d_left', 2.0),
                    'psi_rad': ros_wp.get('psi_rad', 0.0),
                    'kappa_radpm': ros_wp.get('kappa_radpm', 0.0),
                    'vx_mps': ros_wp.get('vx_mps', 5.0),
                    'ax_mps2': ros_wp.get('ax_mps2', 0.0)
                }
                waypoints.append(waypoint)
            except Exception as e:
                print(f"⚠️  Failed to convert waypoint {i}: {e}")
                continue

        return waypoints

    def list_cache(self):
        """List all cached trajectories with details."""
        if not os.path.exists(self.cache_dir):
            print("No cache directory found")
            return

        cache_files = [f for f in os.listdir(
            self.cache_dir) if f.endswith('.json')]
        if not cache_files:
            print("No cached trajectories found")
            return

        print(f"\n=== Cached Trajectories ({len(cache_files)} files) ===")
        for filename in sorted(cache_files):
            cache_file = os.path.join(self.cache_dir, filename)
            try:
                with open(cache_file, 'r') as f:
                    cache_data = json.load(f)

                traj_type = cache_data.get('trajectory_type', 'unknown')
                opt_type = cache_data.get('optimization_type', 'N/A')
                car = cache_data.get('car_name', 'N/A')
                waypoint_count = cache_data.get(
                    'metadata', {}).get('waypoint_count', 0)

                print(f"  {filename}")
                print(
                    f"    Type: {traj_type}, Optimization: {opt_type}, Car: {car}")
                print(
                    f"    Waypoints: {waypoint_count}, Scale: {cache_data.get('scale_factor')}, Width: {cache_data.get('width_multiplier')}")

            except Exception as e:
                print(f"  {filename} - Error reading: {e}")

    def load_marina_csv(self) -> Dict[str, List[Dict]]:
        """Load and parse the Marina CSV file into different trajectory types.

        Note: This method creates waypoints with width_multiplier applied but WITHOUT scale_factor.
        The scale_factor is applied later after shortest path generation.
        """
        print(f"Loading Marina CSV: {self.csv_file}")

        # Read CSV, skip header lines (comments start with #)
        data_lines = []
        with open(self.csv_file, 'r') as f:
            for line_num, line in enumerate(f):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                # Skip the header line with column names
                if any(header_indicator in line.lower() for header_indicator in ['x_rl_m', 'x_ref_cl_m', 'theta_ref']):
                    continue
                # Skip completely empty lines, but NOT nan lines (they may have valid centerline data)
                if not any(c.isdigit() for c in line):
                    continue
                data_lines.append(line)

        print(f"Found {len(data_lines)} data lines")

        # Parse the CSV data into different trajectory types (unscaled but with width multiplier)
        centerline_waypoints = []
        iqp_waypoints = []
        sp_waypoints = []

        for i, line in enumerate(data_lines):
            try:
                # Split by comma and clean values
                values = [v.strip() for v in line.split(',')]

                # Create centerline waypoints (all lines have centerline data)
                cl_waypoint = self.create_centerline_waypoint_unscaled(
                    values, len(centerline_waypoints))
                centerline_waypoints.append(cl_waypoint)

                # Create IQP waypoints (only for lines with valid raceline data)
                # Check if raceline data is valid (not nan)
                rl_x_str = values[self.column_mapping['rl_x_m']]
                if rl_x_str.strip().lower() != 'nan' and rl_x_str.strip():
                    rl_x = self.safe_float(rl_x_str)
                    if not math.isnan(rl_x):
                        iqp_waypoint = self.create_iqp_waypoint_unscaled(
                            values, len(iqp_waypoints))
                        iqp_waypoints.append(iqp_waypoint)

                # Create SP waypoints (moderate racing line using reference data)
                sp_waypoint = self.create_sp_waypoint_unscaled(
                    values, len(sp_waypoints))
                sp_waypoints.append(sp_waypoint)

            except (ValueError, IndexError) as e:
                print(f"Warning: Could not parse line {i}: {e}")
                if len(values) > 0:
                    print(f"  First few values: {values[:5]}")
                continue

        print(f"Successfully parsed waypoints (unscaled):")
        print(f"  - Centerline: {len(centerline_waypoints)} waypoints")
        print(f"  - IQP: {len(iqp_waypoints)} waypoints")
        print(f"  - SP: {len(sp_waypoints)} waypoints")

        print("\n=== Track Closure Validation ===")

        # Check if track is properly closed and fix if necessary
        for traj_name, waypoints in [("centerline", centerline_waypoints), ("iqp", iqp_waypoints), ("sp", sp_waypoints)]:
            if waypoints:
                first_wp = waypoints[0]
                last_wp = waypoints[-1]

                # Calculate distance between first and last waypoint
                distance = ((first_wp['x_m'] - last_wp['x_m']) **
                            2 + (first_wp['y_m'] - last_wp['y_m'])**2)**0.5
                print(f"{traj_name}: distance between start/end = {distance:.3f}m")

                # If track is not closed (distance > 1m), we need to address this
                if distance > 1.0:
                    print(
                        f"WARNING: {traj_name} track is not properly closed! This will cause issues with trajectory optimization.")
                    print(
                        f"First point: ({first_wp['x_m']:.2f}, {first_wp['y_m']:.2f})")
                    print(
                        f"Last point:  ({last_wp['x_m']:.2f}, {last_wp['y_m']:.2f})")

                    # Note: We don't automatically close here because the optimization algorithm handles this
                    # The track bounds file creation will ensure proper closure
                else:
                    print(f"✓ {traj_name} track is properly closed")

        print("==========================================\n")

        # Print track width statistics
        if centerline_waypoints:
            d_rights = [wp['d_right'] for wp in centerline_waypoints]
            d_lefts = [wp['d_left'] for wp in centerline_waypoints]
            total_widths = [wp['d_right'] + wp['d_left']
                            for wp in centerline_waypoints]

            print("\n=== Track Width Statistics (with width multiplier applied) ===")
            print(f"Width multiplier: {self.width_multiplier}x")
            print(
                f"Right bounds:  min={min(d_rights):.2f}m, max={max(d_rights):.2f}m, avg={sum(d_rights)/len(d_rights):.2f}m")
            print(
                f"Left bounds:   min={min(d_lefts):.2f}m, max={max(d_lefts):.2f}m, avg={sum(d_lefts)/len(d_lefts):.2f}m")
            print(
                f"Total width:   min={min(total_widths):.2f}m, max={max(total_widths):.2f}m, avg={sum(total_widths)/len(total_widths):.2f}m")
            print("================================================================\n")

        return {
            'centerline': centerline_waypoints,
            'iqp': iqp_waypoints,
            'sp': sp_waypoints  # SP waypoints re-enabled
        }

    def safe_float(self, value_str: str, default: float = 0.0) -> float:
        """Safely convert string to float, handling 'nan' values."""
        try:
            val = float(value_str)
            if math.isnan(val):
                return default
            return val
        except (ValueError, TypeError):
            return default

    def create_centerline_waypoint_unscaled(self, values: List[str], waypoint_id: int) -> Dict:
        """Create a centerline waypoint using reference centerline data (unscaled, width multiplier applied)."""
        # Use reference centerline data (most conservative) - NO scale factor applied yet
        x_m = self.safe_float(values[self.column_mapping['ref_cl_x_m']])
        y_m = self.safe_float(values[self.column_mapping['ref_cl_y_m']])
        s_m = self.safe_float(values[self.column_mapping['ref_cl_s_m']])
        psi_rad = self.safe_float(
            values[self.column_mapping['ref_cl_psi_rad']])
        kappa_radpm = self.safe_float(
            values[self.column_mapping['ref_cl_kappa_radpm']])
        # Apply width multiplier but no scale factor yet
        # Track widths in CSV are signed distances - use absolute values
        d_right = abs(self.safe_float(
            values[self.column_mapping['ref_cl_d_right']], 2.0)) * self.width_multiplier
        d_left = abs(self.safe_float(
            values[self.column_mapping['ref_cl_d_left']], 2.0)) * self.width_multiplier

        # Generate conservative speed profile for centerline independent of raceline data
        # Base speed on track curvature (higher curvature = lower speed)
        base_speed = 15.0  # Base speed for straight sections (m/s)
        min_speed = 5.0    # Minimum speed for tight corners (m/s)

        # Calculate speed based on curvature (more conservative than raceline)
        abs_curvature = abs(kappa_radpm)
        if abs_curvature > 0.001:  # If there's significant curvature
            # Reduce speed based on curvature - more conservative than raceline
            curvature_factor = max(0.3, 1.0 / (1.0 + abs_curvature * 10.0))
            vx_mps = max(min_speed, base_speed * curvature_factor)
        else:
            vx_mps = base_speed

        # Conservative acceleration (always 0 for centerline - no aggressive maneuvers)
        ax_mps2 = 0.0

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

    def create_iqp_waypoint_unscaled(self, values: List[str], waypoint_id: int, speed_factor: float = 1.0) -> Dict:
        """Create an IQP waypoint using racing line data with fallback to centerline (unscaled, width multiplier applied)."""
        # Try raw racing line data first, fallback to centerline if nan
        x_m = self.safe_float(values[self.column_mapping['rl_x_m']])
        y_m = self.safe_float(values[self.column_mapping['rl_y_m']])
        if x_m == 0.0 and y_m == 0.0:  # If raceline is nan, use centerline
            x_m = self.safe_float(values[self.column_mapping['ref_cl_x_m']])
            y_m = self.safe_float(values[self.column_mapping['ref_cl_y_m']])

        psi_rad = self.safe_float(values[self.column_mapping['rl_psi_rad']])
        if psi_rad == 0.0:  # If raceline psi is nan, use centerline
            psi_rad = self.safe_float(
                values[self.column_mapping['ref_cl_psi_rad']])

        # Apply speed factor but NO scale factor yet
        vx_mps = self.safe_float(
            values[self.column_mapping['rl_vx_mps']], 15.0) * speed_factor  # 15 m/s default
        ax_mps2 = self.safe_float(
            values[self.column_mapping['rl_ax_mps2']], 0.0)

        # Use reference racing line for s_m, curvature and track bounds (more stable), fallback to centerline
        s_m = self.safe_float(values[self.column_mapping['ref_rl_s_m']])
        if s_m == 0.0:  # If ref raceline s_m is nan, use centerline
            s_m = self.safe_float(values[self.column_mapping['ref_cl_s_m']])

        kappa_radpm = self.safe_float(
            values[self.column_mapping['ref_rl_kappa_radpm']])
        if kappa_radpm == 0.0:  # If ref raceline curvature is nan, use centerline
            kappa_radpm = self.safe_float(
                values[self.column_mapping['ref_cl_kappa_radpm']])

        # Apply width multiplier but no scale factor yet
        # Track widths in CSV are signed distances - use absolute values, fallback to centerline
        d_right = abs(self.safe_float(
            values[self.column_mapping['ref_rl_d_right']], 0.0)) * self.width_multiplier
        d_left = abs(self.safe_float(
            values[self.column_mapping['ref_rl_d_left']], 0.0)) * self.width_multiplier

        # If raceline track bounds are nan, use centerline
        if d_right == 0.0:
            d_right = abs(self.safe_float(
                values[self.column_mapping['ref_cl_d_right']], 2.0)) * self.width_multiplier
        if d_left == 0.0:
            d_left = abs(self.safe_float(
                values[self.column_mapping['ref_cl_d_left']], 2.0)) * self.width_multiplier

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

    def create_sp_waypoint_unscaled(self, values: List[str], waypoint_id: int) -> Dict:
        """Create an SP waypoint using reference racing line data with fallback to centerline (unscaled, width multiplier applied).

        Note: This is NOT a true shortest path - it's just a moderate racing line.
        For a real shortest path, use trajectory_optimizer with curv_opt_type='shortest_path'.
        """
        # Use reference racing line data (refined, balanced approach) - NO scale factor applied yet
        # Fallback to centerline if raceline data is nan
        s_m = self.safe_float(values[self.column_mapping['ref_rl_s_m']])
        if s_m == 0.0:  # If ref raceline s_m is nan, use centerline
            s_m = self.safe_float(values[self.column_mapping['ref_cl_s_m']])

        x_m = self.safe_float(values[self.column_mapping['ref_rl_x_m']])
        y_m = self.safe_float(values[self.column_mapping['ref_rl_y_m']])
        if x_m == 0.0 and y_m == 0.0:  # If ref raceline position is nan, use centerline
            x_m = self.safe_float(values[self.column_mapping['ref_cl_x_m']])
            y_m = self.safe_float(values[self.column_mapping['ref_cl_y_m']])

        psi_rad = self.safe_float(
            values[self.column_mapping['ref_rl_psi_rad']])
        if psi_rad == 0.0:  # If ref raceline psi is nan, use centerline
            psi_rad = self.safe_float(
                values[self.column_mapping['ref_cl_psi_rad']])

        kappa_radpm = self.safe_float(
            values[self.column_mapping['ref_rl_kappa_radpm']])
        if kappa_radpm == 0.0:  # If ref raceline curvature is nan, use centerline
            kappa_radpm = self.safe_float(
                values[self.column_mapping['ref_cl_kappa_radpm']])

        # Apply width multiplier but no scale factor yet
        # Track widths in CSV are signed distances - use absolute values, fallback to centerline
        d_right = abs(self.safe_float(
            values[self.column_mapping['ref_rl_d_right']], 0.0)) * self.width_multiplier
        d_left = abs(self.safe_float(
            values[self.column_mapping['ref_rl_d_left']], 0.0)) * self.width_multiplier

        # If raceline track bounds are nan, use centerline
        if d_right == 0.0:
            d_right = abs(self.safe_float(
                values[self.column_mapping['ref_cl_d_right']], 2.0)) * self.width_multiplier
        if d_left == 0.0:
            d_left = abs(self.safe_float(
                values[self.column_mapping['ref_cl_d_left']], 2.0)) * self.width_multiplier

        # Moderate speed and acceleration - use raceline data but with fallback for nan values
        rl_vx_mps = self.safe_float(
            values[self.column_mapping['rl_vx_mps']], 12.0)  # 12 m/s default
        rl_ax_mps2 = self.safe_float(
            values[self.column_mapping['rl_ax_mps2']], 0.0)

        # If raceline speed is not available (nan), generate speed based on curvature like centerline
        # This means we used the default (raceline was nan)
        if rl_vx_mps == 12.0:
            # Generate moderate speed profile based on curvature (between centerline and IQP)
            base_speed = 20.0  # Higher base speed than centerline (15.0)
            min_speed = 8.0    # Higher minimum speed than centerline (5.0)

            abs_curvature = abs(kappa_radpm)
            if abs_curvature > 0.001:
                # Less conservative than centerline but more than IQP
                curvature_factor = max(0.4, 1.0 / (1.0 + abs_curvature * 7.0))
                vx_mps = max(min_speed, base_speed * curvature_factor)
            else:
                vx_mps = base_speed
            ax_mps2 = 0.0  # Conservative acceleration
        else:
            # Apply speed reductions but NO scale factor yet
            # 15% speed reduction (scale factor applied later)
            vx_mps = rl_vx_mps * 0.85
            # 20% acceleration reduction (no scaling needed)
            ax_mps2 = rl_ax_mps2 * 0.8

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

    def create_centerline_waypoint(self, values: List[str], waypoint_id: int) -> Dict:
        """Create a centerline waypoint using reference centerline data (SCALED VERSION)."""
        # Use reference centerline data (most conservative)
        x_m = float(values[self.column_mapping['ref_cl_x_m']]
                    ) * self.scale_factor
        y_m = float(values[self.column_mapping['ref_cl_y_m']]
                    ) * self.scale_factor
        s_m = float(values[self.column_mapping['ref_cl_s_m']]
                    ) * self.scale_factor
        psi_rad = float(values[self.column_mapping['ref_cl_psi_rad']])
        kappa_radpm = float(
            values[self.column_mapping['ref_cl_kappa_radpm']]) / self.scale_factor
        # Track widths in CSV are signed distances - use absolute values
        d_right = abs(float(
            values[self.column_mapping['ref_cl_d_right']])) * self.scale_factor * self.width_multiplier
        d_left = abs(float(
            values[self.column_mapping['ref_cl_d_left']])) * self.scale_factor * self.width_multiplier

        # Conservative speed and acceleration (use raw racing line but reduce significantly)
        rl_vx_mps = float(values[self.column_mapping['rl_vx_mps']])
        rl_ax_mps2 = float(values[self.column_mapping['rl_ax_mps2']])

        # Apply scaling to velocity (smaller track = proportionally slower speeds)
        # 30% speed reduction + scale factor
        vx_mps = rl_vx_mps * 0.7 * self.scale_factor
        # 50% acceleration reduction (no additional scaling needed)
        ax_mps2 = rl_ax_mps2 * 0.5

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

    # Note: create_sp_waypoint is intentionally removed here
    # The shortest path should be generated using the proper trajectory_optimizer
    # with curv_opt_type='shortest_path' rather than just copying/modifying existing data

    def create_iqp_waypoint(self, values: List[str], waypoint_id: int, speed_factor: float = 1.0) -> Dict:
        """Create an IQP waypoint using raw racing line data (most aggressive)."""
        # Use raw racing line data directly (most aggressive)
        x_m = float(values[self.column_mapping['rl_x_m']]) * self.scale_factor
        y_m = float(values[self.column_mapping['rl_y_m']]) * self.scale_factor
        psi_rad = float(values[self.column_mapping['rl_psi_rad']])
        # Apply scaling to velocity (smaller track = proportionally slower speeds)
        vx_mps = float(values[self.column_mapping['rl_vx_mps']]
                       ) * speed_factor * self.scale_factor
        ax_mps2 = float(values[self.column_mapping['rl_ax_mps2']])

        # Use reference racing line for s_m, curvature and track bounds (more stable)
        s_m = float(values[self.column_mapping['ref_rl_s_m']]
                    ) * self.scale_factor
        kappa_radpm = float(
            values[self.column_mapping['ref_rl_kappa_radpm']]) / self.scale_factor
        # Track widths in CSV are signed distances - use absolute values
        d_right = abs(float(
            values[self.column_mapping['ref_rl_d_right']])) * self.scale_factor * self.width_multiplier
        d_left = abs(float(
            values[self.column_mapping['ref_rl_d_left']])) * self.scale_factor * self.width_multiplier

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

    def create_sp_waypoint(self, values: List[str], waypoint_id: int) -> Dict:
        """Create an SP waypoint using reference racing line data (refined/moderate).

        Note: This is NOT a true shortest path - it's just a moderate racing line.
        For a real shortest path, use trajectory_optimizer with curv_opt_type='shortest_path'.
        """
        # Use reference racing line data (refined, balanced approach)
        s_m = float(values[self.column_mapping['ref_rl_s_m']]
                    ) * self.scale_factor
        x_m = float(values[self.column_mapping['ref_rl_x_m']]
                    ) * self.scale_factor
        y_m = float(values[self.column_mapping['ref_rl_y_m']]
                    ) * self.scale_factor
        psi_rad = float(values[self.column_mapping['ref_rl_psi_rad']])
        kappa_radpm = float(
            values[self.column_mapping['ref_rl_kappa_radpm']]) / self.scale_factor
        # Track widths in CSV are signed distances - use absolute values
        d_right = abs(float(
            values[self.column_mapping['ref_rl_d_right']])) * self.scale_factor * self.width_multiplier
        d_left = abs(float(
            values[self.column_mapping['ref_rl_d_left']])) * self.scale_factor * self.width_multiplier

        # Moderate speed and acceleration (use raw racing line but reduce moderately)
        rl_vx_mps = float(values[self.column_mapping['rl_vx_mps']])
        rl_ax_mps2 = float(values[self.column_mapping['rl_ax_mps2']])

        # Apply scaling to velocity (smaller track = proportionally slower speeds)
        # 15% speed reduction + scale factor
        vx_mps = rl_vx_mps * 0.85 * self.scale_factor
        # 20% acceleration reduction (no additional scaling needed)
        ax_mps2 = rl_ax_mps2 * 0.8

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

    def create_global_waypoints_json(self, trajectory_data: Dict[str, List[Dict]]) -> Dict[str, Any]:
        """Create the global_waypoints.json structure."""

        centerline_waypoints = trajectory_data['centerline']
        iqp_waypoints = trajectory_data['iqp']
        sp_waypoints = trajectory_data['sp']

        # Create waypoint arrays with proper ROS headers
        centerline_array = self.create_waypoint_array(centerline_waypoints)
        iqp_array = self.create_waypoint_array(iqp_waypoints)
        sp_array = self.create_waypoint_array(sp_waypoints)  # Will be empty

        # Calculate lap statistics from different trajectories
        lap_time = 108.68526373056437  # From CSV header
        iqp_max_speed = max(wp['vx_mps']
                            for wp in iqp_waypoints) if iqp_waypoints else 0.0

        # Calculate SP statistics if available
        sp_max_speed = max(wp['vx_mps']
                           for wp in sp_waypoints) if sp_waypoints else 0.0
        iqp_lap_time = lap_time  # Original optimized time
        # Estimate 10% slower than IQP
        sp_lap_time = lap_time * 1.1 if sp_waypoints else 0.0

        # Create visualization markers for different trajectories
        centerline_markers = self.create_waypoint_markers(
            centerline_waypoints, "centerline", color={'r': 0, 'g': 0, 'b': 1, 'a': 1})  # Blue
        iqp_markers = self.create_waypoint_markers(
            iqp_waypoints, "iqp", color={'r': 1, 'g': 0, 'b': 0, 'a': 1})  # Red
        sp_markers = self.create_waypoint_markers(
            sp_waypoints, "sp", color={'r': 0, 'g': 1, 'b': 0, 'a': 1})  # Green (empty)
        trackbounds_markers = self.create_trackbounds_markers(
            iqp_waypoints)  # Use IQP for bounds

        # Create full global waypoints structure in the correct order
        global_waypoints = {
            'map_info_str': {
                'data': f'IQP estimated lap time: {iqp_lap_time:.4f}s; IQP maximum speed: {iqp_max_speed:.4f}m/s; SP estimated lap time: {sp_lap_time:.4f}s; SP maximum speed: {sp_max_speed:.4f}m/s'
            },
            'est_lap_time': {
                # Use SP as estimate if available, otherwise IQP
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

        return global_waypoints

    def create_waypoint_array(self, waypoints: List[Dict]) -> Dict[str, Any]:
        """Create a waypoint array with proper ROS header."""
        wpnt_list = []

        for wp in waypoints:
            wpnt_msg = {
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
            }
            wpnt_list.append(wpnt_msg)

        return {
            'header': {
                'seq': 1,
                'stamp': {
                    'secs': 0,
                    'nsecs': 0
                },
                'frame_id': ""
            },
            'wpnts': wpnt_list
        }

    def create_map_yaml(self, waypoints: List[Dict], origin_x: float = None, origin_y: float = None) -> Dict[str, Any]:
        """Create the map YAML configuration."""

        if origin_x is None or origin_y is None:
            # Calculate map bounds from waypoints as fallback (waypoints are already scaled)
            x_coords = [wp['x_m'] for wp in waypoints]
            y_coords = [wp['y_m'] for wp in waypoints]

            min_x, max_x = min(x_coords), max(x_coords)
            min_y, max_y = min(y_coords), max(y_coords)

            # Set origin (bottom-left corner with some padding) - already in scaled coordinates
            padding = 5.0 * self.scale_factor  # Scale the padding too
            origin_x = min_x - padding
            origin_y = min_y - padding
            print(
                f"Calculated scaled origin: ({origin_x:.3f}, {origin_y:.3f}) with padding {padding:.3f}m")

        map_config = {
            'free_thresh': 0.196,
            'image': f'{self.output_map_name}.png',
            'negate': 0,
            'occupied_thresh': 0.65,
            'origin': [origin_x, origin_y, 0],
            'resolution': 0.05000000074505806  # 5cm resolution like other maps
        }

        print(
            f"Map configuration: origin=[{origin_x:.3f}, {origin_y:.3f}, 0], resolution={map_config['resolution']}")
        return map_config

    def create_ot_sectors_yaml(self, waypoints: List[Dict]) -> Dict[str, Any]:
        """Create overtaking sectors configuration."""

        total_waypoints = len(waypoints)

        # Create single sector covering the whole track
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

        return ot_sectors

    def create_speed_scaling_yaml(self, waypoints: List[Dict]) -> Dict[str, Any]:
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

        return speed_scaling

    def create_starting_position_config(self, trajectory_data: Dict[str, List[Dict]]) -> Dict[str, Any]:
        """Create starting position configuration based on the best centerline waypoint."""
        centerline_waypoints = trajectory_data['centerline']

        if not centerline_waypoints:
            print("Warning: No centerline waypoints found for starting position")
            return {}

        # Find a good starting position - look for a straight section with reasonable width
        best_waypoint = None
        best_score = float('-inf')

        # Check multiple waypoints to find the best starting position
        for i in range(min(50, len(centerline_waypoints))):  # Check first 50 waypoints
            wp = centerline_waypoints[i]

            # Score based on track width and curvature
            track_width = wp['d_right'] + wp['d_left']
            # Prefer straight sections
            curvature_penalty = abs(wp['kappa_radpm']) * 10
            width_score = min(track_width, 4.0)  # Cap at 4m width

            score = width_score - curvature_penalty

            if score > best_score:
                best_score = score
                best_waypoint = wp

        # Fall back to first waypoint if no good candidate found
        if best_waypoint is None:
            best_waypoint = centerline_waypoints[0]
            print("Warning: Using first waypoint as no optimal starting position found")
        else:
            print(
                f"Selected waypoint {best_waypoint['id']} as starting position (score: {best_score:.2f})")

        # Validate that starting position is within track boundaries
        # For debugging: print track boundaries at starting position
        print(
            f"Starting position track width: left={best_waypoint['d_left']:.2f}m, right={best_waypoint['d_right']:.2f}m")

        # Validate reasonable track width (should be > 1m for F1Tenth car)
        total_width = best_waypoint['d_left'] + best_waypoint['d_right']
        if total_width < 1.0:
            print(
                f"WARNING: Track width at starting position is very narrow ({total_width:.2f}m)")

        # Normalize heading angle to [-π, π] range for ROS compatibility
        heading = best_waypoint['psi_rad']
        while heading > math.pi:
            heading -= 2 * math.pi
        while heading < -math.pi:
            heading += 2 * math.pi

        starting_config = {
            'car_init_x': best_waypoint['x_m'],
            'car_init_y': best_waypoint['y_m'],
            'car_init_theta': heading,  # Use normalized heading
            'description': f"Starting position based on optimal centerline waypoint {best_waypoint['id']} at ({best_waypoint['x_m']:.3f}, {best_waypoint['y_m']:.3f}) with heading {heading:.3f} rad"
        }

        print(
            f"Starting position: x={best_waypoint['x_m']:.3f}, y={best_waypoint['y_m']:.3f}, theta={heading:.3f}")
        print(
            f"Track curvature at start: {best_waypoint['kappa_radpm']:.4f} rad/m")
        print(
            f"Heading normalized from {best_waypoint['psi_rad']:.3f} to {heading:.3f} rad")

        return starting_config

    def create_output_directory(self) -> str:
        """Create output directory structure."""
        # Use a path relative to the current catkin workspace or home directory
        base_path = os.path.expanduser(
            "~/catkin_ws/src/race_stack/tam/maps/output")
        output_dir = os.path.join(base_path, self.output_map_name)

        # Create directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        print(f"Created output directory: {output_dir}")

        return output_dir

    def write_yaml_file(self, data: Dict, filepath: str):
        """Write YAML file."""
        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        print(f"Written: {filepath}")

    def write_json_file(self, data: Dict, filepath: str):
        """Write JSON file."""
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Written: {filepath}")

    def create_track_image(self, output_dir: str, resolution: float = 0.05):
        """Create a PNG image of the track based on track boundaries."""
        print("Creating track image from boundary data...")

        # Load CSV data to get original track boundaries and expand them based on width multiplier
        original_left_points = []
        original_right_points = []
        centerline_points = []
        boundary_points_left = []
        boundary_points_right = []

        with open(self.csv_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if line.startswith('nan') or not any(c.isdigit() for c in line):
                    continue

                try:
                    values = [v.strip() for v in line.split(',')]
                    if len(values) < 47:
                        continue

                    # Get centerline coordinates
                    cl_x = float(
                        values[self.column_mapping['ref_cl_x_m']]) * self.scale_factor
                    cl_y = float(
                        values[self.column_mapping['ref_cl_y_m']]) * self.scale_factor

                    # Get original track boundary coordinates
                    orig_left_x = float(
                        values[self.column_mapping['tb_left_x']]) * self.scale_factor
                    orig_left_y = float(
                        values[self.column_mapping['tb_left_y']]) * self.scale_factor
                    orig_right_x = float(
                        values[self.column_mapping['tb_right_x']]) * self.scale_factor
                    orig_right_y = float(
                        values[self.column_mapping['tb_right_y']]) * self.scale_factor

                    # Calculate the original track width (distance between left and right boundaries)
                    orig_track_width = math.sqrt(
                        (orig_right_x - orig_left_x)**2 + (orig_right_y - orig_left_y)**2)

                    # Calculate the new track width
                    new_track_width = orig_track_width * self.width_multiplier

                    # Calculate the centerline of the track (midpoint between boundaries)
                    track_center_x = (orig_left_x + orig_right_x) / 2
                    track_center_y = (orig_left_y + orig_right_y) / 2

                    # Calculate the direction vector from left to right boundary
                    if orig_track_width > 0:
                        right_dir_x = (
                            orig_right_x - orig_left_x) / orig_track_width
                        right_dir_y = (
                            orig_right_y - orig_left_y) / orig_track_width

                        # Place new boundaries at half the new width from track center
                        half_new_width = new_track_width / 2
                        new_left_x = track_center_x - right_dir_x * half_new_width
                        new_left_y = track_center_y - right_dir_y * half_new_width
                        new_right_x = track_center_x + right_dir_x * half_new_width
                        new_right_y = track_center_y + right_dir_y * half_new_width
                    else:
                        # Handle degenerate case
                        new_left_x = orig_left_x
                        new_left_y = orig_left_y
                        new_right_x = orig_right_x
                        new_right_y = orig_right_y

                    original_left_points.append((orig_left_x, orig_left_y))
                    original_right_points.append((orig_right_x, orig_right_y))
                    boundary_points_left.append((new_left_x, new_left_y))
                    boundary_points_right.append((new_right_x, new_right_y))
                    centerline_points.append((cl_x, cl_y))

                except (ValueError, IndexError):
                    continue

        if not boundary_points_left or not boundary_points_right:
            print("Warning: No valid boundary points found. Creating placeholder image.")
            self.copy_placeholder_image(output_dir)
            return

        print(f"Found {len(boundary_points_left)} boundary points")
        print(
            f"Track width multiplier: {self.width_multiplier} (track width is now {self.width_multiplier} times original, preserving track shape)")

        # Calculate image bounds with padding
        all_x = [p[0] for p in boundary_points_left] + [p[0]
                                                        for p in boundary_points_right]
        all_y = [p[1] for p in boundary_points_left] + [p[1]
                                                        for p in boundary_points_right]

        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        # Add padding (scaled to match the scaled coordinates)
        padding = 10.0 * self.scale_factor  # Scale the padding to match the scaled track
        min_x -= padding
        max_x += padding
        min_y -= padding
        max_y += padding

        print(
            f"Image bounds with scaled padding ({padding:.3f}m): X=[{min_x:.3f}, {max_x:.3f}], Y=[{min_y:.3f}, {max_y:.3f}]")

        # Calculate image dimensions
        width_m = max_x - min_x
        height_m = max_y - min_y
        width_px = int(width_m / resolution)
        height_px = int(height_m / resolution)

        print(
            f"Image size: {width_px}x{height_px} pixels ({width_m:.1f}x{height_m:.1f}m)")

        # Create image (black background)
        img = Image.new('RGB', (width_px, height_px), color='black')
        draw = ImageDraw.Draw(img)

        # Convert world coordinates to image coordinates
        def world_to_image(x, y):
            img_x = int((x - min_x) / resolution)
            # Flip Y axis for image coordinates
            img_y = int((max_y - y) / resolution)
            return (img_x, img_y)

        # Convert boundary points to image coordinates
        left_boundary_img = [world_to_image(x, y)
                             for x, y in boundary_points_left]
        right_boundary_img = [world_to_image(
            x, y) for x, y in boundary_points_right]
        centerline_img = [world_to_image(x, y) for x, y in centerline_points]

        # Create the track outline by connecting left and right boundaries
        # Close the loop by connecting the end of right boundary to start of left boundary
        track_polygon = left_boundary_img + list(reversed(right_boundary_img))

        # Fill the track area (inside) with white
        draw.polygon(track_polygon, fill='white', outline=None)

        # Note: For navigation maps, we should NOT draw colored lines as they become obstacles
        # The centerline and boundaries are only for visualization in RViz, not for navigation
        # If you need them visible, make them white (same as track) or very light gray

        # Option 1: Don't draw centerline and boundaries (recommended for navigation)
        # Option 2: Draw them in white so they don't become obstacles
        # Option 3: Draw them in very light gray (250,250,250) - barely visible but not obstacles

        # For now, let's not draw the centerline and boundaries in the navigation map
        # They will still be visible in RViz through the trackbounds_markers

        # If you want to see them in the map image for debugging, uncomment these lines:
        # centerline_width = max(1, int(0.1 / resolution))  # Thinner line
        # for i in range(len(centerline_img) - 1):
        #     draw.line([centerline_img[i], centerline_img[i + 1]],
        #               fill='lightgray', width=centerline_width)
        # if len(centerline_img) > 1:
        #     draw.line([centerline_img[-1], centerline_img[0]],
        #               fill='lightgray', width=centerline_width)

        # Save the image
        target_image = os.path.join(output_dir, f"{self.output_map_name}.png")
        img.save(target_image)
        print(f"Created track image: {target_image}")

        # Return origin for map.yaml (already scaled since boundary_points are scaled)
        return min_x, min_y

    def copy_placeholder_image(self, output_dir: str):
        """Copy a placeholder image file."""
        # Try to find an existing map image in common locations
        possible_sources = [
            os.path.expanduser(
                "~/catkin_ws/src/race_stack/stack_master/maps/f/f.png"),
            os.path.expanduser("~/catkin_ws/src/race_stack/tam/maps/f.png"),
            os.path.expanduser("~/Documents/race_stack/maps/f/f.png")
        ]

        target_image = os.path.join(output_dir, f"{self.output_map_name}.png")

        for source_image in possible_sources:
            if os.path.exists(source_image):
                shutil.copy2(source_image, target_image)
                print(f"Copied placeholder image: {target_image}")
                return

        print(f"Warning: Could not find a source image to copy")
        print("You will need to provide a track image manually.")
        print(f"Expected location: {target_image}")

    def create_waypoint_markers(self, waypoints: List[Dict], marker_type: str, color: Dict) -> Dict:
        """Create visualization markers for waypoints with speed-based scaling."""
        markers = []

        # Sample every 10th waypoint to avoid too many markers
        # Limit to ~500 markers max
        sample_rate = 8

        # Calculate speed scaling parameters
        speeds = [wp['vx_mps'] for wp in waypoints]
        min_speed = min(speeds) if speeds else 1.0
        max_speed = max(speeds) if speeds else 10.0

        # Define base scale and scaling range
        base_scale = 0.05
        max_scale_multiplier = 5.0  # Maximum scale will be 5x base scale

        for i, wp in enumerate(waypoints[::sample_rate]):
            # Calculate dynamic scale based on speed (like the original F1tenth maps)
            # Normalize speed to 0-1 range, then scale
            if max_speed > min_speed:
                speed_ratio = (wp['vx_mps'] - min_speed) / \
                    (max_speed - min_speed)
            else:
                speed_ratio = 1.0

            # Calculate dynamic scale (higher speed = larger marker)
            scale_multiplier = 1.0 + \
                (speed_ratio * (max_scale_multiplier - 1.0))
            dynamic_scale = base_scale * scale_multiplier

            marker = {
                'header': {
                    'seq': 0,
                    'stamp': {
                        'secs': 0,
                        'nsecs': 0
                    },
                    'frame_id': 'map'
                },
                'ns': '',
                'id': i,
                'type': 2,  # SPHERE marker type
                'action': 0,  # ADD action
                'pose': {
                    'position': {
                        'x': wp['x_m'],
                        'y': wp['y_m'],
                        'z': 0
                    },
                    'orientation': {
                        'x': 0,
                        'y': 0,
                        'z': 0,
                        'w': 1
                    }
                },
                'scale': {
                    'x': base_scale,
                    'y': base_scale,
                    'z': dynamic_scale  # Speed-based Z scale like F1tenth maps
                },
                'color': color,
                'lifetime': {
                    'secs': 0,
                    'nsecs': 0
                },
                'frame_locked': False,
                'points': [],
                'colors': [],
                'text': '',
                'mesh_resource': '',
                'mesh_use_embedded_materials': False
            }
            markers.append(marker)

        return {'markers': markers}

    def create_trackbounds_markers(self, waypoints: List[Dict]) -> Dict:
        """Create visualization markers for track boundaries using expanded boundaries."""
        markers = []

        print("Creating trackbounds markers with expanded boundaries...")

        # Load the CSV again to get trackbounds data and expand them
        data_lines = []
        with open(self.csv_file, 'r') as f:
            for line_num, line in enumerate(f):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if line.startswith('nan') or not any(c.isdigit() for c in line):
                    continue
                data_lines.append(line)

        # Sample every 8th waypoint to avoid too many markers
        sample_rate = 8
        marker_id = 0
        successful_markers = 0

        for i, line in enumerate(data_lines[::sample_rate]):
            try:
                values = [v.strip() for v in line.split(',')]

                # Skip if not enough columns
                if len(values) < 47:
                    continue

                # Get centerline coordinates
                cl_x = float(
                    values[self.column_mapping['ref_cl_x_m']]) * self.scale_factor
                cl_y = float(
                    values[self.column_mapping['ref_cl_y_m']]) * self.scale_factor

                # Get original track boundary coordinates
                orig_left_x = float(
                    values[self.column_mapping['tb_left_x']]) * self.scale_factor
                orig_left_y = float(
                    values[self.column_mapping['tb_left_y']]) * self.scale_factor
                orig_right_x = float(
                    values[self.column_mapping['tb_right_x']]) * self.scale_factor
                orig_right_y = float(
                    values[self.column_mapping['tb_right_y']]) * self.scale_factor

                # Calculate the original track width (distance between left and right boundaries)
                orig_track_width = math.sqrt(
                    (orig_right_x - orig_left_x)**2 + (orig_right_y - orig_left_y)**2)

                # Calculate the new track width
                new_track_width = orig_track_width * self.width_multiplier

                # Calculate the centerline of the track (midpoint between boundaries)
                track_center_x = (orig_left_x + orig_right_x) / 2
                track_center_y = (orig_left_y + orig_right_y) / 2

                # Calculate the direction vector from left to right boundary
                if orig_track_width > 0:
                    right_dir_x = (orig_right_x - orig_left_x) / \
                        orig_track_width
                    right_dir_y = (orig_right_y - orig_left_y) / \
                        orig_track_width

                    # Place new boundaries at half the new width from track center
                    half_new_width = new_track_width / 2
                    expanded_left_x = track_center_x - right_dir_x * half_new_width
                    expanded_left_y = track_center_y - right_dir_y * half_new_width
                    expanded_right_x = track_center_x + right_dir_x * half_new_width
                    expanded_right_y = track_center_y + right_dir_y * half_new_width
                else:
                    # Handle degenerate case
                    expanded_left_x = orig_left_x
                    expanded_left_y = orig_left_y
                    expanded_right_x = orig_right_x
                    expanded_right_y = orig_right_y

                # Create left boundary marker
                left_marker = {
                    'header': {
                        'seq': 0,
                        'stamp': {'secs': 0, 'nsecs': 0},
                        'frame_id': 'map'
                    },
                    'ns': 'trackbounds_left',
                    'id': marker_id,
                    'type': 2,  # CUBE
                    'action': 0,
                    'pose': {
                        'position': {'x': expanded_left_x, 'y': expanded_left_y, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    },
                    'scale': {'x': 0.1, 'y': 0.1, 'z': 0.1},
                    'color': {'r': 1, 'g': 0.5, 'b': 0, 'a': 0.7},
                    'lifetime': {'secs': 0, 'nsecs': 0},
                    'frame_locked': False,
                    'points': [],
                    'colors': [],
                    'text': '',
                    'mesh_resource': '',
                    'mesh_use_embedded_materials': False
                }
                markers.append(left_marker)
                marker_id += 1

                # Create right boundary marker
                right_marker = {
                    'header': {
                        'seq': 0,
                        'stamp': {'secs': 0, 'nsecs': 0},
                        'frame_id': 'map'
                    },
                    'ns': 'trackbounds_right',
                    'id': marker_id,
                    'type': 1,  # CUBE
                    'action': 0,
                    'pose': {
                        'position': {'x': expanded_right_x, 'y': expanded_right_y, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    },
                    'scale': {'x': 0.1, 'y': 0.1, 'z': 0.1},
                    'color': {'r': 1, 'g': 0.5, 'b': 0, 'a': 0.7},
                    'lifetime': {'secs': 0, 'nsecs': 0},
                    'frame_locked': False,
                    'points': [],
                    'colors': [],
                    'text': '',
                    'mesh_resource': '',
                    'mesh_use_embedded_materials': False
                }
                markers.append(right_marker)
                marker_id += 1
                successful_markers += 2

            except (ValueError, IndexError) as e:
                continue

        print(
            f"Successfully created {successful_markers} expanded trackbounds markers ({successful_markers//2} left, {successful_markers//2} right)")
        print(
            f"Track width multiplied by {self.width_multiplier}x (distance between boundaries is now {self.width_multiplier} times original)")
        return {'markers': markers}

    def generate_real_shortest_path(self, output_dir: str, centerline_waypoints: List[Dict], safety_width_sp: float = 0.7) -> List[Dict]:
        """
        Generate a true shortest path using the TUM trajectory optimizer.

        This method creates the proper input files and calls trajectory_optimizer
        with curv_opt_type='shortest_path' to generate a geometrically optimal path.

        Args:
            output_dir: Directory where map files are stored
            centerline_waypoints: Centerline waypoints to use as reference (UNSCALED, width multiplier applied)
            safety_width_sp: Safety width for shortest path optimization (unscaled)

        Returns:
            List of shortest path waypoints (UNSCALED), or empty list if generation fails
        """
        try:
            # Import trajectory_optimizer with timeout protection
            import sys
            import os
            import signal

            def timeout_handler(signum, frame):
                raise TimeoutError(
                    "Import timed out - trajectory optimizer module may have issues")

            # Set a 60-second timeout for the entire trajectory optimization process
            signal.signal(signal.SIGALRM, timeout_handler)
            signal.alarm(60)

            # Add the path to the trajectory optimizer
            sys.path.append(os.path.expanduser(
                '~/catkin_ws/src/race_stack/planner/gb_optimizer/src'))

            print("Importing trajectory optimizer with timeout protection...")
            from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
            print("Trajectory optimizer imported successfully!")

            print(f"Generating real shortest path using trajectory_optimizer...")
            print(
                f"Original track size: {len(centerline_waypoints)} waypoints")

            # CRITICAL FIX: Use the same waypoint count as the IQP/raceline data
            # The issue is that centerline and IQP have different lengths, causing jumps
            # We need to ensure the track bounds match the expected raceline length

            # Load the CSV to get the actual raceline length that IQP uses
            data_lines = []
            with open(self.csv_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    data_lines.append(line)

            # This is the actual length used by IQP
            iqp_length = len(data_lines)
            centerline_length = len(centerline_waypoints)

            print(f"IQP/Raceline data length from CSV: {iqp_length} waypoints")
            print(f"Centerline data length: {centerline_length} waypoints")

            # CRITICAL: Truncate centerline to match IQP length to prevent jumps
            if centerline_length > iqp_length:
                print(f"⚠️  Centerline is longer than IQP data - truncating to match")
                sampled_waypoints = centerline_waypoints[:iqp_length]
                print(
                    f"Truncated centerline from {centerline_length} to {len(sampled_waypoints)} waypoints")
            elif centerline_length < iqp_length:
                print(
                    f"⚠️  Centerline is shorter than IQP data - using available centerline")
                sampled_waypoints = centerline_waypoints
            else:
                print(f"✓ Centerline and IQP lengths match perfectly")
                sampled_waypoints = centerline_waypoints

            print(
                f"Using {len(sampled_waypoints)} waypoints for track bounds (matches raceline data)")

            # CRITICAL: Ensure the sampled track forms a proper closed loop
            # This is essential for shortest path optimization to work correctly
            if sampled_waypoints:
                first_wp = sampled_waypoints[0]
                last_wp = sampled_waypoints[-1]

                # Calculate distance between first and last waypoint
                closure_distance = (
                    (first_wp['x_m'] - last_wp['x_m'])**2 + (first_wp['y_m'] - last_wp['y_m'])**2)**0.5
                print(f"Track closure distance: {closure_distance:.3f}m")

                # If the track is not properly closed, we need to ensure closure
                if closure_distance > 2.0:  # 2m tolerance
                    print(
                        f"WARNING: Track is not properly closed (distance: {closure_distance:.3f}m)")
                    print(
                        "This may cause the optimizer to generate a path spanning multiple laps")
                    print(
                        "Attempting to improve closure by adjusting the track data...")

                    # Option 1: Find the point closest to the start to create better closure
                    min_distance = float('inf')
                    best_end_idx = len(sampled_waypoints) - 1

                    # Look for a waypoint that's closer to the start (within the last 25% of points)
                    search_start = max(1, int(0.75 * len(sampled_waypoints)))
                    for i in range(search_start, len(sampled_waypoints)):
                        wp = sampled_waypoints[i]
                        dist = ((first_wp['x_m'] - wp['x_m']) **
                                2 + (first_wp['y_m'] - wp['y_m'])**2)**0.5
                        if dist < min_distance:
                            min_distance = dist
                            best_end_idx = i

                    if min_distance < closure_distance:
                        print(
                            f"Found better closure point at index {best_end_idx} (distance: {min_distance:.3f}m)")
                        # Truncate the track at the better closure point
                        sampled_waypoints = sampled_waypoints[:best_end_idx + 1]
                        print(
                            f"Truncated track to {len(sampled_waypoints)} waypoints for better closure")
                    else:
                        print(
                            "Could not find a better closure point - proceeding with original track")
                else:
                    print(
                        f"✓ Track has acceptable closure (distance: {closure_distance:.3f}m)")

            print(f"Safety width: {safety_width_sp}m (unscaled)")
            print(
                f"Width multiplier: {self.width_multiplier} (already applied to track bounds)")
            print(
                f"Scale factor: {self.scale_factor} (will be applied after optimization)")

            # WORKAROUND: The trajectory optimizer has a bug on line 112 where it doesn't properly join
            # the input_path with the track_name. It expects the track file to be in the CURRENT WORKING DIRECTORY
            # instead of the input_path. We need to place the file in the current directory.

            # Create track file in current working directory (workaround for trajectory optimizer bug)
            track_name = 'temp_track_bounds'
            # Current directory, not output_dir
            temp_track_file = f'{track_name}.csv'
            self.create_track_bounds_file_unscaled(
                temp_track_file, sampled_waypoints)

            # Copy vehicle parameters to current directory - use specified car configuration
            # The trajectory_optimizer looks for the vehicle parameter file in the input_path directory
            # Current directory (input_path)
            vehicle_params_dst = f'racecar_{self.car_name.lower()}.ini'
            self.create_car_specific_vehicle_params(
                vehicle_params_dst, self.car_name)
            print(
                f"Created {self.car_name} vehicle parameters: {vehicle_params_dst}")

            # Create vehicle dynamics info for the specific car
            ggv_file = 'ggv.csv'
            ax_max_file = 'ax_max_machines.csv'
            self.create_car_specific_ggv(ggv_file, self.car_name)
            self.create_car_specific_ax_max(ax_max_file, self.car_name)

            # Copy vehicle dynamics info directory as well (required by trajectory optimizer)
            veh_dyn_src = os.path.expanduser(
                '~/catkin_ws/src/race_stack/stack_master/config/gb_optimizer/veh_dyn_info')
            veh_dyn_dst = 'veh_dyn_info'  # Current directory
            if os.path.exists(veh_dyn_src):
                if os.path.exists(veh_dyn_dst):
                    shutil.rmtree(veh_dyn_dst)  # Remove existing directory
                shutil.copytree(veh_dyn_src, veh_dyn_dst)
                print(f"Copied vehicle dynamics info to {veh_dyn_dst}")
            else:
                print(
                    f"Warning: Vehicle dynamics info not found at {veh_dyn_src}")

            # Call the trajectory optimizer for shortest path (on unscaled data)
            # Use current directory as input_path due to trajectory optimizer bug
            print("Starting trajectory optimization (this may take several minutes)...")
            print(
                "Note: Large tracks may require significant memory. Monitor system resources.")
            print(
                f"Using {self.car_name} vehicle parameters for optimization")

            # Reduce safety width for smaller tracks to avoid over-constraining
            # Cap at 0.3m for stability
            effective_safety_width = min(safety_width_sp, 0.3)

            # The trajectory_optimizer function uses a hardcoded "racecar_f110.ini" file
            # We need to either use that name or modify the function temporarily
            # Let's rename our file to match what it expects
            if os.path.exists(vehicle_params_dst):
                os.rename(vehicle_params_dst, 'racecar_f110.ini')
                print(
                    f"Renamed {self.car_name} parameters to racecar_f110.ini for trajectory optimizer")

            # Call trajectory optimizer with the correct API
            # IMPORTANT NOTE: 'shortest_path' optimization focuses on geometric shortest distance
            # and may produce conservative speed profiles. For realistic racing speeds,
            # consider using 'mintime' optimization instead.
            print("Attempting shortest path optimization...")
            print(
                "Note: shortest_path optimization prioritizes geometric optimality over speed")
            print("      The speed profile is generated but may be conservative")
            print("      For aggressive speed profiles, use 'mintime' optimization type")

            shortest_path_data, bound_r, bound_l, sp_lap_time = trajectory_optimizer(
                input_path='.',
                track_name=track_name,
                curv_opt_type='shortest_path',  # Use shortest_path for real shortest path
                safety_width=effective_safety_width,
                plot=False
            )

            # Cancel the timeout - we succeeded
            signal.alarm(0)

            print(
                f"Successfully generated shortest path with lap time: {sp_lap_time:.4f}s (on unscaled track)")
            print(f"Shortest path data shape: {shortest_path_data.shape}")
            print(f"Processing {len(shortest_path_data)} trajectory points...")

            # VALIDATION: Check if the generated trajectory is a proper single lap
            if len(shortest_path_data) > 0:
                # Check start and end points of the generated trajectory
                start_point = shortest_path_data[0, 1:3]  # x, y coordinates
                end_point = shortest_path_data[-1, 1:3]   # x, y coordinates
                traj_closure_distance = np.linalg.norm(end_point - start_point)

                print(
                    f"Generated trajectory closure distance: {traj_closure_distance:.3f}m")
                print(
                    f"Start point: ({start_point[0]:.2f}, {start_point[1]:.2f})")
                print(f"End point:   ({end_point[0]:.2f}, {end_point[1]:.2f})")

                # Check if trajectory spans more than expected distance (indicating multiple laps)
                total_distance = shortest_path_data[-1, 0]  # s_m (arc length)
                print(f"Total trajectory distance: {total_distance:.2f}m")

                # Estimate track length from centerline data
                if sampled_waypoints:
                    estimated_track_length = max(
                        wp['s_m'] for wp in sampled_waypoints)
                    print(
                        f"Estimated single lap length: {estimated_track_length:.2f}m")

                    # If trajectory is much longer than expected, it might span multiple laps
                    if total_distance > estimated_track_length * 1.5:
                        print(
                            f"WARNING: Generated trajectory may span multiple laps!")
                        print(
                            f"Trajectory distance ({total_distance:.2f}m) >> expected lap length ({estimated_track_length:.2f}m)")

                        # Truncate to single lap if possible
                        single_lap_indices = shortest_path_data[:,
                                                                0] <= estimated_track_length * 1.1
                        if np.any(single_lap_indices):
                            shortest_path_data = shortest_path_data[single_lap_indices]
                            print(
                                f"Truncated trajectory to {len(shortest_path_data)} points for single lap")

                if traj_closure_distance > 5.0:
                    print(
                        f"WARNING: Generated trajectory is not properly closed (distance: {traj_closure_distance:.3f}m)")
                    print(
                        "This indicates the optimization may not have worked correctly")
                else:
                    print(
                        f"✓ Generated trajectory has acceptable closure (distance: {traj_closure_distance:.3f}m)")

            # Convert the optimized data to our waypoint format (UNSCALED)
            # The returned data from trajectory_optimizer is numpy arrays with columns:
            # [s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2]

            print("=== Post-processing Generated Shortest Path ===")
            print(f"Raw trajectory data shape: {shortest_path_data.shape}")

            # Check for and fix closure issues BEFORE waypoint conversion
            if len(shortest_path_data) > 1:
                start_point = shortest_path_data[0, 1:3]  # x, y of first point
                end_point = shortest_path_data[-1, 1:3]   # x, y of last point
                closure_gap = np.linalg.norm(end_point - start_point)

                print(f"Initial closure gap: {closure_gap:.3f}m")

                # If closure gap is significant, fix it
                if closure_gap > 2.0:  # 2m threshold
                    print("❌ Large closure gap detected - applying closure fix")

                    # Method 1: Gradually adjust the last few points to close the loop
                    num_transition_points = min(
                        10, len(shortest_path_data) // 10)
                    print(
                        f"Smoothly transitioning last {num_transition_points} points to close the loop")

                    for i in range(num_transition_points):
                        idx = len(shortest_path_data) - \
                            num_transition_points + i
                        blend_factor = (i + 1) / \
                            num_transition_points  # 0 to 1

                        # Blend the position toward the start point
                        current_pos = shortest_path_data[idx, 1:3]
                        target_pos = start_point
                        new_pos = current_pos * \
                            (1 - blend_factor) + target_pos * blend_factor
                        shortest_path_data[idx, 1:3] = new_pos

                        # Also blend the heading toward the start heading
                        current_psi = shortest_path_data[idx, 3]
                        target_psi = shortest_path_data[0, 3]

                        # Handle angle wrapping
                        angle_diff = target_psi - current_psi
                        if angle_diff > np.pi:
                            angle_diff -= 2 * np.pi
                        elif angle_diff < -np.pi:
                            angle_diff += 2 * np.pi

                        new_psi = current_psi + angle_diff * blend_factor
                        shortest_path_data[idx, 3] = new_psi

                    # Verify the fix
                    fixed_end_point = shortest_path_data[-1, 1:3]
                    fixed_closure_gap = np.linalg.norm(
                        fixed_end_point - start_point)
                    print(f"✓ Closure gap after fix: {fixed_closure_gap:.3f}m")

                    if fixed_closure_gap > 1.0:
                        print("⚠️  Warning: Closure gap still large after fix")
                        # Method 2: More aggressive fix - force last point to equal first point
                        print(
                            "Applying aggressive closure fix: forcing last point to match first point")
                        # Force position match
                        shortest_path_data[-1, 1:3] = start_point
                        # Force heading match
                        shortest_path_data[-1, 3] = shortest_path_data[0, 3]

                        final_closure_gap = np.linalg.norm(
                            shortest_path_data[-1, 1:3] - start_point)
                        print(f"✓ Final closure gap: {final_closure_gap:.3f}m")
                else:
                    print(f"✓ Acceptable closure gap: {closure_gap:.3f}m")

            # Remove potential duplicate points that might cause issues
            print("Removing duplicate consecutive points...")
            if len(shortest_path_data) > 1:
                # Calculate distances between consecutive points
                diff_vectors = np.diff(shortest_path_data[:, 1:3], axis=0)
                distances = np.linalg.norm(diff_vectors, axis=1)

                # Keep points that are at least 0.01m apart
                keep_indices = [0]  # Always keep first point
                for i in range(1, len(shortest_path_data)):
                    if distances[i-1] > 0.01:  # 1cm minimum distance
                        keep_indices.append(i)

                if len(keep_indices) < len(shortest_path_data):
                    print(
                        f"Removed {len(shortest_path_data) - len(keep_indices)} duplicate points")
                    shortest_path_data = shortest_path_data[keep_indices]
                    print(
                        f"Cleaned trajectory shape: {shortest_path_data.shape}")

            print("================================================")

            sp_waypoints = []
            print("Converting trajectory data to waypoint format...")
            print(f"Trajectory shape: {shortest_path_data.shape}")
            print(
                f"Centerline waypoints available: {len(centerline_waypoints)}")

            # Pre-calculate track bounds to avoid expensive lookup in the loop
            if centerline_waypoints:
                # Use average track bounds to avoid expensive per-point lookup
                avg_d_right = sum(
                    wp['d_right'] for wp in centerline_waypoints) / len(centerline_waypoints)
                avg_d_left = sum(
                    wp['d_left'] for wp in centerline_waypoints) / len(centerline_waypoints)
                print(
                    f"Using average track bounds: d_right={avg_d_right:.3f}, d_left={avg_d_left:.3f}")
            else:
                avg_d_right = 2.0 * self.width_multiplier
                avg_d_left = 2.0 * self.width_multiplier
                print(
                    f"Using fallback track bounds: d_right={avg_d_right:.3f}, d_left={avg_d_left:.3f}")

            print("Starting waypoint conversion loop...")
            for i in range(len(shortest_path_data)):
                if i % 5000 == 0:  # Progress indicator every 5000 points
                    print(
                        f"Processing waypoint {i}/{len(shortest_path_data)} ({100*i/len(shortest_path_data):.1f}%)")

                # Extract data from numpy arrays
                s_m = shortest_path_data[i,
                                         0] if shortest_path_data.shape[1] > 0 else i * 0.1
                x_m = shortest_path_data[i,
                                         1] if shortest_path_data.shape[1] > 1 else 0.0
                y_m = shortest_path_data[i,
                                         2] if shortest_path_data.shape[1] > 2 else 0.0
                psi_rad = shortest_path_data[i,
                                             3] if shortest_path_data.shape[1] > 3 else 0.0
                kappa_radpm = shortest_path_data[i,
                                                 4] if shortest_path_data.shape[1] > 4 else 0.0
                vx_mps = shortest_path_data[i,
                                            5] if shortest_path_data.shape[1] > 5 else 5.0
                ax_mps2 = shortest_path_data[i,
                                             6] if shortest_path_data.shape[1] > 6 else 0.0

                # DEBUG: Print speed extraction info for first few waypoints
                if i < 5:
                    print(
                        f"  Waypoint {i}: extracted speed = {vx_mps:.2f} m/s from trajectory optimizer")
                    if shortest_path_data.shape[1] <= 5:
                        print(
                            f"    Warning: Using fallback speed, data shape = {shortest_path_data.shape}")

                # IMPORTANT: The speed (vx_mps) here is extracted directly from the trajectory optimizer
                # This is NOT a custom calculation - it comes from the trajectory optimizer's
                # speed profile generation based on the car parameters and track geometry

                # Use pre-calculated average track bounds instead of expensive lookup
                d_right = avg_d_right
                d_left = avg_d_left

                waypoint = {
                    'id': i,
                    's_m': s_m,  # Unscaled arc length
                    'd_m': 0.0,
                    'x_m': x_m,  # Unscaled coordinates
                    'y_m': y_m,  # Unscaled coordinates
                    'd_right': d_right,  # Track bounds with width_multiplier applied
                    'd_left': d_left,
                    'psi_rad': psi_rad,
                    'kappa_radpm': kappa_radpm,  # Unscaled curvature
                    'vx_mps': vx_mps,  # Unscaled velocity
                    'ax_mps2': ax_mps2  # Acceleration
                }
                sp_waypoints.append(waypoint)

            print(
                f"Converted {len(sp_waypoints)} waypoints to internal format")
            print("Waypoint conversion completed successfully!")

            # Final validation of the shortest path closure
            print("\n=== Final Shortest Path Validation ===")
            if len(sp_waypoints) >= 2:
                start_wp = sp_waypoints[0]
                end_wp = sp_waypoints[-1]

                start_pos = np.array([start_wp['x_m'], start_wp['y_m']])
                end_pos = np.array([end_wp['x_m'], end_wp['y_m']])
                final_closure_gap = np.linalg.norm(end_pos - start_pos)

                start_heading = start_wp['psi_rad']
                end_heading = end_wp['psi_rad']
                heading_diff = abs(end_heading - start_heading)
                if heading_diff > np.pi:
                    heading_diff = 2 * np.pi - heading_diff

                print(f"Final waypoint closure gap: {final_closure_gap:.3f}m")
                print(
                    f"Final waypoint heading difference: {heading_diff:.3f}rad ({np.degrees(heading_diff):.1f}°)")

                if final_closure_gap < 1.0 and heading_diff < 0.5:
                    print("✅ Shortest path closure validation PASSED")
                elif final_closure_gap < 2.0:
                    print(
                        "⚠️  Shortest path closure validation WARNING - acceptable gap")
                else:
                    print(
                        "❌ Shortest path closure validation FAILED - large gap remains")
                    print("   This may cause discontinuities when following the path")

                # Also check for reasonable path length
                total_path_length = sp_waypoints[-1]['s_m'] if sp_waypoints[-1]['s_m'] > 0 else len(
                    sp_waypoints) * 0.1
                track_length_estimate = len(
                    centerline_waypoints) * 0.1 if centerline_waypoints else 1000.0

                print(f"Shortest path length: {total_path_length:.1f}m")
                print(f"Estimated track length: {track_length_estimate:.1f}m")

                if total_path_length > track_length_estimate * 1.5:
                    print("⚠️  Warning: Shortest path is much longer than expected")
                    print("   This might indicate the path spans multiple laps")
                elif total_path_length < track_length_estimate * 0.8:
                    print("✅ Shortest path length looks reasonable")
                else:
                    print("✅ Shortest path length within expected range")

            print("==========================================\n")

            # No interpolation needed since we're using the correct track length
            print(
                "Track length properly matched to raceline data - no interpolation needed")

            # Clean up temporary files (in current directory due to trajectory optimizer bug)
            print("Cleaning up temporary files...")
            if os.path.exists(temp_track_file):
                os.remove(temp_track_file)
                print(f"Removed {temp_track_file}")
            # We renamed our car-specific file to this
            if os.path.exists('racecar_f110.ini'):
                os.remove('racecar_f110.ini')
                print("Removed racecar_f110.ini")
            if os.path.exists(veh_dyn_dst):
                shutil.rmtree(veh_dyn_dst)  # Remove copied directory
                print(f"Removed {veh_dyn_dst}")

            print(
                f"Generated {len(sp_waypoints)} shortest path waypoints (unscaled, ready for scaling)")
            print("Shortest path generation completed successfully!")
            return sp_waypoints

        except TimeoutError as e:
            signal.alarm(0)  # Cancel alarm
            print(f"Warning: Trajectory optimization timed out: {e}")
            print("This suggests the trajectory optimizer is hanging or taking too long")
            print("Falling back to moderate racing line as SP trajectory")
            return []
        except ImportError as e:
            signal.alarm(0)  # Cancel alarm
            print(f"Warning: Could not import trajectory_optimizer: {e}")
            print("Falling back to moderate racing line as SP trajectory")
            return []
        except Exception as e:
            signal.alarm(0)  # Cancel alarm
            print(f"Warning: Failed to generate real shortest path: {e}")
            print("Falling back to moderate racing line as SP trajectory")
            return []

    def apply_curvature_based_velocity_optimization(self, waypoints: List[Dict], optimization_type: str = "mintime") -> List[Dict]:
        """
        Apply curvature-based velocity optimization to fix constant velocity profiles.

        This method implements a physics-based velocity calculation that considers:
        - Track curvature (tighter turns = slower speeds)
        - Vehicle dynamics (lateral acceleration limits)
        - Optimization type (mintime = aggressive, mincurv = conservative)

        Args:
            waypoints: List of racing line waypoints with constant velocity
            optimization_type: Type of optimization ('mintime' or 'mincurv')

        Returns:
            List of waypoints with optimized velocity profiles
        """
        import math

        if not waypoints:
            return waypoints

        print(
            f"Applying {optimization_type} velocity optimization based on track curvature...")

        # Load car parameters for optimization
        car_config_path = os.path.expanduser(
            f"~/catkin_ws/src/race_stack/stack_master/config/{self.car_name}")
        car_model_file = os.path.join(car_config_path, "car_model.yaml")

        # Default parameters
        max_velocity = 10.0
        max_lateral_accel = 8.0  # m/s² (aggressive for F1Tenth)
        min_velocity = 3.0       # m/s (minimum safe speed)

        if os.path.exists(car_model_file):
            try:
                import yaml
                with open(car_model_file, 'r') as f:
                    car_data = yaml.safe_load(f)
                max_velocity = float(car_data.get('max_velocity', 10.0))
                max_lateral_accel = float(car_data.get(
                    'max_lateral_acceleration', 8.0))
                # 30% of max velocity minimum
                min_velocity = max(2.0, max_velocity * 0.3)
            except Exception as e:
                print(f"Warning: Could not load car parameters: {e}")

        # Adjust parameters based on optimization type
        if optimization_type == "mintime":
            # Aggressive optimization - higher lateral acceleration limits
            lateral_accel_factor = 1.0
            velocity_factor = 1.0
            smoothing_factor = 0.3
        else:  # mincurv
            # Conservative optimization - lower lateral acceleration limits
            lateral_accel_factor = 0.7
            velocity_factor = 0.85
            smoothing_factor = 0.5

        max_lateral_accel *= lateral_accel_factor
        max_velocity *= velocity_factor

        print(f"Velocity optimization parameters:")
        print(f"  Max velocity: {max_velocity:.1f} m/s")
        print(f"  Max lateral acceleration: {max_lateral_accel:.1f} m/s²")
        print(f"  Min velocity: {min_velocity:.1f} m/s")
        print(f"  Optimization type: {optimization_type}")

        # Step 1: Calculate velocity limits based on curvature
        optimized_waypoints = []
        for i, wp in enumerate(waypoints):
            # Calculate maximum safe velocity for this curvature
            # v_max = sqrt(a_lat_max / |curvature|)
            abs_curvature = abs(wp['kappa_radpm'])

            if abs_curvature > 0.001:  # Avoid division by very small numbers
                # Calculate maximum velocity based on lateral acceleration limit
                curvature_limited_velocity = math.sqrt(
                    max_lateral_accel / abs_curvature)
                # Clamp to vehicle limits
                target_velocity = min(max_velocity, max(
                    min_velocity, curvature_limited_velocity))
            else:
                # Straight section - use maximum velocity
                target_velocity = max_velocity

            # Create optimized waypoint
            optimized_wp = wp.copy()
            optimized_wp['vx_mps'] = target_velocity
            optimized_waypoints.append(optimized_wp)

        # Step 2: Apply smoothing to avoid abrupt velocity changes
        print("Applying velocity profile smoothing...")
        smoothed_waypoints = []
        # Adaptive window size
        window_size = max(3, min(15, len(optimized_waypoints) // 100))

        for i, wp in enumerate(optimized_waypoints):
            # Calculate smoothed velocity using local averaging
            start_idx = max(0, i - window_size // 2)
            end_idx = min(len(optimized_waypoints), i + window_size // 2 + 1)

            # Get velocities in the window
            window_velocities = [optimized_waypoints[j]['vx_mps']
                                 for j in range(start_idx, end_idx)]

            # Apply weighted smoothing (center point has higher weight)
            weights = []
            for j in range(len(window_velocities)):
                distance_from_center = abs(j - len(window_velocities) // 2)
                weight = 1.0 / (1.0 + distance_from_center * smoothing_factor)
                weights.append(weight)

            # Calculate weighted average
            weighted_sum = sum(
                v * w for v, w in zip(window_velocities, weights))
            weight_sum = sum(weights)
            smoothed_velocity = weighted_sum / weight_sum

            # Create smoothed waypoint
            smoothed_wp = wp.copy()
            smoothed_wp['vx_mps'] = smoothed_velocity
            smoothed_waypoints.append(smoothed_wp)

        # Step 3: Calculate acceleration profile based on velocity changes
        print("Calculating optimized acceleration profile...")
        final_waypoints = []
        for i, wp in enumerate(smoothed_waypoints):
            if i == 0:
                # First waypoint - assume zero acceleration
                acceleration = 0.0
            else:
                # Calculate acceleration based on velocity change
                prev_velocity = smoothed_waypoints[i-1]['vx_mps']
                curr_velocity = wp['vx_mps']

                # Estimate distance between waypoints
                if i < len(smoothed_waypoints) - 1:
                    next_wp = smoothed_waypoints[i+1]
                    ds = ((next_wp['x_m'] - wp['x_m'])**2 +
                          (next_wp['y_m'] - wp['y_m'])**2)**0.5
                else:
                    ds = 1.0  # Default distance

                # Calculate acceleration: a = (v² - v₀²) / (2 * ds)
                if ds > 0.001:
                    acceleration = (curr_velocity**2 -
                                    prev_velocity**2) / (2 * ds)
                else:
                    acceleration = 0.0

                # Limit acceleration to reasonable values
                max_accel = 5.0  # m/s²
                min_accel = -8.0  # m/s² (braking)
                acceleration = max(min_accel, min(max_accel, acceleration))

            # Create final waypoint
            final_wp = wp.copy()
            final_wp['ax_mps2'] = acceleration
            final_waypoints.append(final_wp)

        # Report optimization results
        velocities = [wp['vx_mps'] for wp in final_waypoints]
        accelerations = [wp['ax_mps2'] for wp in final_waypoints]

        print(f"Curvature-based velocity optimization completed:")
        print(
            f"  Velocity range: {min(velocities):.2f} - {max(velocities):.2f} m/s")
        print(
            f"  Acceleration range: {min(accelerations):.2f} - {max(accelerations):.2f} m/s²")
        print(f"  Processed {len(final_waypoints)} waypoints")

        return final_waypoints

    def generate_racing_line_with_speed_profile(self, output_dir: str, centerline_waypoints: List[Dict], optimization_type: str = "mintime") -> List[Dict]:
        """
        Generate an optimal racing line with speed profile using the TUM trajectory optimizer.

        This method creates the proper input files and calls trajectory_optimizer
        with either 'mintime' or 'mincurv' optimization to generate a racing line
        with realistic speed profiles for the specified car.

        Args:
            output_dir: Directory where map files are stored
            centerline_waypoints: Centerline waypoints to use as reference (UNSCALED, width multiplier applied)
            optimization_type: Type of optimization ('mintime' for fastest lap, 'mincurv' for smooth racing line)

        Returns:
            List of racing line waypoints (UNSCALED) with speed profile, or empty list if generation fails
        """
        try:
            # Import trajectory_optimizer with timeout protection
            import sys
            import os
            import signal

            def timeout_handler(signum, frame):
                raise TimeoutError(
                    "Import timed out - trajectory optimizer module may have issues")

            # Set a 120-second timeout for racing line optimization (more complex than shortest path)
            signal.signal(signal.SIGALRM, timeout_handler)
            signal.alarm(120)

            # Add the path to the trajectory optimizer
            sys.path.append(os.path.expanduser(
                '~/catkin_ws/src/race_stack/planner/gb_optimizer/src'))

            print("Importing trajectory optimizer for racing line generation...")
            from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
            print("Trajectory optimizer imported successfully!")

            print(
                f"Generating racing line using {optimization_type} optimization...")
            print(f"Car configuration: {self.car_name}")
            print(
                f"Original track size: {len(centerline_waypoints)} waypoints")

            # Use the same waypoint preparation logic as shortest path
            data_lines = []
            with open(self.csv_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    data_lines.append(line)

            iqp_length = len(data_lines)
            centerline_length = len(centerline_waypoints)

            print(f"Target racing line length: {iqp_length} waypoints")
            print(f"Centerline data length: {centerline_length} waypoints")

            # Match lengths to prevent trajectory jumps
            if centerline_length > iqp_length:
                print(f"⚠️  Centerline is longer than target - truncating to match")
                sampled_waypoints = centerline_waypoints[:iqp_length]
                print(
                    f"Truncated centerline from {centerline_length} to {len(sampled_waypoints)} waypoints")
            elif centerline_length < iqp_length:
                print(
                    f"⚠️  Centerline is shorter than target - using available centerline")
                sampled_waypoints = centerline_waypoints
            else:
                print(f"✓ Centerline and target lengths match perfectly")
                sampled_waypoints = centerline_waypoints

            print(f"Using {len(sampled_waypoints)} waypoints for track bounds")

            # Ensure proper track closure for racing line optimization
            if sampled_waypoints:
                first_wp = sampled_waypoints[0]
                last_wp = sampled_waypoints[-1]

                closure_distance = (
                    (first_wp['x_m'] - last_wp['x_m'])**2 + (first_wp['y_m'] - last_wp['y_m'])**2)**0.5
                print(f"Track closure distance: {closure_distance:.3f}m")

                if closure_distance > 2.0:
                    print(
                        f"WARNING: Track is not properly closed (distance: {closure_distance:.3f}m)")

                    # Find better closure point
                    min_distance = float('inf')
                    best_end_idx = len(sampled_waypoints) - 1
                    search_start = max(1, int(0.75 * len(sampled_waypoints)))

                    for i in range(search_start, len(sampled_waypoints)):
                        wp = sampled_waypoints[i]
                        dist = ((first_wp['x_m'] - wp['x_m']) **
                                2 + (first_wp['y_m'] - wp['y_m'])**2)**0.5
                        if dist < min_distance:
                            min_distance = dist
                            best_end_idx = i

                    if min_distance < closure_distance:
                        print(
                            f"Found better closure point at index {best_end_idx} (distance: {min_distance:.3f}m)")
                        sampled_waypoints = sampled_waypoints[:best_end_idx + 1]
                        print(
                            f"Truncated track to {len(sampled_waypoints)} waypoints for better closure")
                else:
                    print(
                        f"✓ Track has acceptable closure (distance: {closure_distance:.3f}m)")

            # Create track bounds file for racing line optimization
            track_name = f'racing_line_track_{optimization_type}'
            temp_track_file = f'{track_name}.csv'
            self.create_track_bounds_file_unscaled(
                temp_track_file, sampled_waypoints)

            # Create car-specific vehicle parameters
            vehicle_params_dst = f'racecar_{self.car_name.lower()}.ini'
            self.create_car_specific_vehicle_params(
                vehicle_params_dst, self.car_name)
            print(
                f"Created {self.car_name} vehicle parameters: {vehicle_params_dst}")

            # Create vehicle dynamics files
            ggv_file = 'ggv.csv'
            ax_max_file = 'ax_max_machines.csv'
            self.create_car_specific_ggv(ggv_file, self.car_name)
            self.create_car_specific_ax_max(ax_max_file, self.car_name)

            # Copy vehicle dynamics info
            veh_dyn_src = os.path.expanduser(
                '~/catkin_ws/src/race_stack/stack_master/config/gb_optimizer/veh_dyn_info')
            veh_dyn_dst = 'veh_dyn_info'
            if os.path.exists(veh_dyn_src):
                if os.path.exists(veh_dyn_dst):
                    shutil.rmtree(veh_dyn_dst)
                shutil.copytree(veh_dyn_src, veh_dyn_dst)
                print(f"Copied vehicle dynamics info to {veh_dyn_dst}")

            # Rename vehicle params to expected name
            if os.path.exists(vehicle_params_dst):
                os.rename(vehicle_params_dst, 'racecar_f110.ini')
                print(
                    f"Renamed {self.car_name} parameters to racecar_f110.ini for trajectory optimizer")

            # Set optimization parameters based on type
            if optimization_type == "mintime":
                print("=== Minimum Lap Time Racing Line Generation ===")
                print(
                    "This will generate the fastest possible racing line for the given car")
                print(
                    "Optimization considers vehicle dynamics, tire limits, and power constraints")
                safety_width = 0.8  # More aggressive for mintime
            elif optimization_type == "mincurv":
                print("=== Minimum Curvature Racing Line Generation ===")
                print("This will generate a smooth racing line with moderate speeds")
                print("Optimization balances speed and smoothness")
                safety_width = 0.9  # More conservative for mincurv
            else:
                print(
                    f"Warning: Unknown optimization type '{optimization_type}', defaulting to mintime")
                optimization_type = "mintime"
                safety_width = 0.8

            print(f"Safety width: {safety_width}m")
            print(f"Car configuration: {self.car_name}")
            print("Starting racing line optimization (this may take several minutes)...")

            # Ensure we're in the correct directory and all files exist
            current_dir = os.getcwd()
            print(f"DEBUG: Current working directory: {current_dir}")

            required_files = [
                f'{track_name}.csv',
                'racecar_f110.ini',
                'ggv.csv',
                'ax_max_machines.csv'
            ]

            for file in required_files:
                if os.path.exists(file):
                    print(f"DEBUG: Found required file: {file}")
                else:
                    print(f"ERROR: Missing required file: {file}")

            # List all files in current directory for debugging
            all_files = os.listdir('.')
            print(f"DEBUG: All files in current directory: {all_files}")

            # Call trajectory optimizer with racing line optimization
            # Fix the parameter mapping for the trajectory optimizer
            if optimization_type == "mintime":
                opt_type = "mintime"
            elif optimization_type == "mincurv":
                opt_type = "mincurv"
            else:
                opt_type = "mintime"

            print(
                f"DEBUG: Calling trajectory_optimizer with opt_type='{opt_type}'")

            try:
                racing_line_data, bound_r, bound_l, racing_lap_time = trajectory_optimizer(
                    input_path='.',
                    track_name=track_name,
                    curv_opt_type=opt_type,
                    safety_width=safety_width,
                    plot=False,
                    debug=True
                )
            except Exception as opt_error:
                print(
                    f"Trajectory optimizer error: {type(opt_error).__name__}: {opt_error}")
                print(f"Error details: {str(opt_error)}")

                # Try with different parameters
                print("Retrying with simplified parameters...")
                try:
                    racing_line_data, bound_r, bound_l, racing_lap_time = trajectory_optimizer(
                        input_path='.',
                        track_name=track_name,
                        curv_opt_type=opt_type,
                        safety_width=safety_width,
                        plot=False
                    )
                except Exception as retry_error:
                    print(
                        f"Retry failed: {type(retry_error).__name__}: {retry_error}")
                    raise retry_error

            # Cancel the timeout - we succeeded
            signal.alarm(0)

            print(f"Successfully generated {optimization_type} racing line!")
            print(
                f"Estimated lap time: {racing_lap_time:.4f}s (on unscaled track)")
            print(f"Racing line data shape: {racing_line_data.shape}")
            print(f"Processing {len(racing_line_data)} trajectory points...")

            # Validate the generated racing line
            if len(racing_line_data) > 0:
                start_point = racing_line_data[0, 1:3]  # x, y coordinates
                end_point = racing_line_data[-1, 1:3]   # x, y coordinates
                traj_closure_distance = np.linalg.norm(end_point - start_point)

                print(
                    f"Racing line closure distance: {traj_closure_distance:.3f}m")
                print(
                    f"Start point: ({start_point[0]:.2f}, {start_point[1]:.2f})")
                print(f"End point:   ({end_point[0]:.2f}, {end_point[1]:.2f})")

                # Check total distance
                total_distance = racing_line_data[-1, 0]  # s_m (arc length)
                print(f"Total racing line distance: {total_distance:.2f}m")

                # Speed profile analysis
                speeds = racing_line_data[:, 5]  # vx_mps column
                accelerations = racing_line_data[:, 6]  # ax_mps2 column

                print(
                    f"Speed profile: min={np.min(speeds):.2f} m/s, max={np.max(speeds):.2f} m/s, avg={np.mean(speeds):.2f} m/s")
                print(
                    f"Acceleration: min={np.min(accelerations):.2f} m/s², max={np.max(accelerations):.2f} m/s², avg={np.mean(accelerations):.2f} m/s²")

                if traj_closure_distance > 5.0:
                    print(
                        f"WARNING: Racing line is not properly closed (distance: {traj_closure_distance:.3f}m)")
                else:
                    print(
                        f"✓ Racing line has acceptable closure (distance: {traj_closure_distance:.3f}m)")

            # Convert the optimized data to our waypoint format (UNSCALED)
            print("=== Post-processing Generated Racing Line ===")

            racing_line_waypoints = []
            for i, row in enumerate(racing_line_data):
                s_m = float(row[0])
                x_m = float(row[1])
                y_m = float(row[2])
                psi_rad = float(row[3])
                kappa_radpm = float(row[4])
                vx_mps = float(row[5])
                ax_mps2 = float(row[6])

                # Calculate track bounds from centerline (interpolated to racing line points)
                # Use the average track width from the centerline data
                if sampled_waypoints:
                    avg_d_right = sum(
                        wp['d_right'] for wp in sampled_waypoints) / len(sampled_waypoints)
                    avg_d_left = sum(
                        wp['d_left'] for wp in sampled_waypoints) / len(sampled_waypoints)
                else:
                    avg_d_right = avg_d_left = 2.0  # Default fallback

                waypoint = {
                    'id': i,
                    's_m': s_m,
                    'd_m': 0.0,  # Racing line offset from centerline
                    'x_m': x_m,
                    'y_m': y_m,
                    'd_right': avg_d_right,
                    'd_left': avg_d_left,
                    'psi_rad': psi_rad,
                    'kappa_radpm': kappa_radpm,
                    'vx_mps': vx_mps,
                    'ax_mps2': ax_mps2
                }
                racing_line_waypoints.append(waypoint)

            print(
                f"Converted {len(racing_line_waypoints)} racing line waypoints")

            # Apply velocity optimization post-processing if all velocities are constant
            if racing_line_waypoints:
                speeds = [wp['vx_mps'] for wp in racing_line_waypoints]
                if min(speeds) == max(speeds):
                    print(
                        "⚠️  Detected constant velocity profile - applying curvature-based optimization")
                    racing_line_waypoints = self.apply_curvature_based_velocity_optimization(
                        racing_line_waypoints, optimization_type)
                    # Report new velocity profile
                    new_speeds = [wp['vx_mps'] for wp in racing_line_waypoints]
                    print(
                        f"✓ Optimized velocity profile: min={min(new_speeds):.2f} m/s, max={max(new_speeds):.2f} m/s, avg={sum(new_speeds)/len(new_speeds):.2f} m/s")

            print(f"Racing line generation complete!")

            # Cleanup temporary files
            temp_files = [temp_track_file,
                          'racecar_f110.ini', ggv_file, ax_max_file]
            for temp_file in temp_files:
                if os.path.exists(temp_file):
                    os.remove(temp_file)
                    print(f"Cleaned up temporary file: {temp_file}")

            if os.path.exists(veh_dyn_dst):
                shutil.rmtree(veh_dyn_dst)
                print(f"Cleaned up temporary directory: {veh_dyn_dst}")

            return racing_line_waypoints

        except TimeoutError as e:
            signal.alarm(0)  # Cancel alarm
            print(f"Warning: Racing line optimization timed out: {e}")
            print(
                "Racing line generation is computationally intensive and may require more time")
            print("Falling back to pre-calculated racing line data")
            return []
        except ImportError as e:
            signal.alarm(0)  # Cancel alarm
            print(
                f"Warning: Could not import trajectory_optimizer for racing line: {e}")
            print("Falling back to pre-calculated racing line data")
            return []
        except Exception as e:
            signal.alarm(0)  # Cancel alarm
            print(f"Warning: Failed to generate racing line: {e}")
            print("Falling back to pre-calculated racing line data")
            return []

    def interpolate_waypoints(self, waypoints: List[Dict], target_count: int) -> List[Dict]:
        """Interpolate waypoints to achieve target count.

        Args:
            waypoints: List of waypoints to interpolate
            target_count: Target number of waypoints

        Returns:
            List of interpolated waypoints
        """
        print(
            f"Starting interpolation: {len(waypoints)} -> {target_count} waypoints")

        if len(waypoints) >= target_count:
            print("No interpolation needed - already have enough waypoints")
            return waypoints

        try:
            print("Importing numpy and scipy for interpolation...")
            import numpy as np
            from scipy.interpolate import interp1d

            print("Extracting arrays for interpolation...")
            # Extract arrays for interpolation
            s_values = np.array([wp['s_m'] for wp in waypoints])
            x_values = np.array([wp['x_m'] for wp in waypoints])
            y_values = np.array([wp['y_m'] for wp in waypoints])
            psi_values = np.array([wp['psi_rad'] for wp in waypoints])
            kappa_values = np.array([wp['kappa_radpm'] for wp in waypoints])
            vx_values = np.array([wp['vx_mps'] for wp in waypoints])
            ax_values = np.array([wp['ax_mps2'] for wp in waypoints])

            print(
                f"Array shapes: s={s_values.shape}, x={x_values.shape}, y={y_values.shape}")

            print("Creating interpolation functions...")
            # Create interpolation functions
            x_interp = interp1d(s_values, x_values, kind='cubic',
                                bounds_error=False, fill_value='extrapolate')
            y_interp = interp1d(s_values, y_values, kind='cubic',
                                bounds_error=False, fill_value='extrapolate')
            psi_interp = interp1d(
                s_values, psi_values, kind='linear', bounds_error=False, fill_value='extrapolate')
            kappa_interp = interp1d(
                s_values, kappa_values, kind='linear', bounds_error=False, fill_value='extrapolate')
            vx_interp = interp1d(s_values, vx_values, kind='linear',
                                 bounds_error=False, fill_value='extrapolate')
            ax_interp = interp1d(s_values, ax_values, kind='linear',
                                 bounds_error=False, fill_value='extrapolate')

            # Create new s values
            s_new = np.linspace(s_values[0], s_values[-1], target_count)

            # Interpolate all values
            interpolated_waypoints = []
            for i, s in enumerate(s_new):
                # Use first waypoint as template for track bounds
                d_right = waypoints[0]['d_right']
                d_left = waypoints[0]['d_left']

                waypoint = {
                    'id': i,
                    's_m': float(s),
                    'd_m': 0.0,
                    'x_m': float(x_interp(s)),
                    'y_m': float(y_interp(s)),
                    'd_right': d_right,
                    'd_left': d_left,
                    'psi_rad': float(psi_interp(s)),
                    'kappa_radpm': float(kappa_interp(s)),
                    'vx_mps': float(vx_interp(s)),
                    'ax_mps2': float(ax_interp(s))
                }
                interpolated_waypoints.append(waypoint)

            return interpolated_waypoints

        except ImportError:
            print("Warning: scipy not available, using simple linear interpolation")
            # Simple linear interpolation fallback
            interpolated_waypoints = []
            ratio = len(waypoints) / target_count

            for i in range(target_count):
                # Find the closest original waypoint
                source_idx = min(int(i * ratio), len(waypoints) - 1)
                source_wp = waypoints[source_idx].copy()
                source_wp['id'] = i
                interpolated_waypoints.append(source_wp)

            return interpolated_waypoints

    def create_track_bounds_file_unscaled(self, filepath: str, centerline_waypoints: List[Dict]):
        """Create a track bounds file for trajectory optimization (UNSCALED data).

        Note: The centerline_waypoints are unscaled but have width_multiplier applied.
        """
        with open(filepath, 'w') as f:
            # Write header
            f.write("# x_m,y_m,w_tr_right_m,w_tr_left_m\n")

            # Write track bounds data (unscaled but width-multiplied)
            for wp in centerline_waypoints:
                f.write(
                    f"{wp['x_m']:.6f},{wp['y_m']:.6f},{wp['d_right']:.6f},{wp['d_left']:.6f}\n")

            # CRITICAL: Ensure track is properly closed by adding the first point at the end
            # This is required for the trajectory optimizer to work correctly
            if centerline_waypoints:
                first_wp = centerline_waypoints[0]
                f.write(
                    f"{first_wp['x_m']:.6f},{first_wp['y_m']:.6f},{first_wp['d_right']:.6f},{first_wp['d_left']:.6f}\n")
                print(
                    f"Track properly closed: added first waypoint at end to ensure closure")

        print(f"Created track bounds file: {filepath}")
        print(
            f"Track has {len(centerline_waypoints)} waypoints + 1 closure point")
        print(
            f"Track bounds are unscaled but widths multiplied by {self.width_multiplier}")

    def create_track_bounds_file(self, filepath: str, centerline_waypoints: List[Dict]):
        """Create a track bounds file for trajectory optimization.

        Note: The centerline_waypoints are already scaled by scale_factor and 
        track widths are already multiplied by width_multiplier.
        """
        with open(filepath, 'w') as f:
            # Write header
            f.write("# x_m,y_m,w_tr_right_m,w_tr_left_m\n")

            # Write track bounds data (already scaled and width-multiplied)
            for wp in centerline_waypoints:
                f.write(
                    f"{wp['x_m']:.6f},{wp['y_m']:.6f},{wp['d_right']:.6f},{wp['d_left']:.6f}\n")

        print(f"Created track bounds file: {filepath}")
        print(
            f"Track bounds are pre-scaled by {self.scale_factor} and widths multiplied by {self.width_multiplier}")

    def scale_waypoints(self, waypoints: List[Dict]) -> List[Dict]:
        """Scale waypoints from original size to scaled size for final output.

        This applies the scale_factor to coordinates, distances, and velocities.
        Should be called AFTER shortest path generation on unscaled data.
        """
        scaled_waypoints = []

        for wp in waypoints:
            scaled_wp = {
                'id': wp['id'],
                's_m': wp['s_m'] * self.scale_factor,
                'd_m': wp['d_m'] * self.scale_factor,
                'x_m': wp['x_m'] * self.scale_factor,
                'y_m': wp['y_m'] * self.scale_factor,
                # Already has width_multiplier applied
                'd_right': wp['d_right'] * self.scale_factor,
                # Already has width_multiplier applied
                'd_left': wp['d_left'] * self.scale_factor,
                'psi_rad': wp['psi_rad'],  # Angles don't need scaling
                # Inverse scaling for curvature
                'kappa_radpm': wp['kappa_radpm'] / self.scale_factor,
                'vx_mps': wp['vx_mps'] * self.scale_factor,    # Scale velocity
                'ax_mps2': wp['ax_mps2']   # Acceleration doesn't need scaling
            }
            scaled_waypoints.append(scaled_wp)

        return scaled_waypoints

    def parse(self):
        """Main parsing function."""
        print("=== Marina Map CSV to F1Tenth Format Parser ===")

        # Load and parse trajectories
        trajectory_data = self.load_marina_csv()

        if not trajectory_data or not trajectory_data['centerline']:
            print("Error: No waypoints could be parsed from CSV")
            return False

        # Create output directory
        output_dir = self.create_output_directory()

        # Try to seed cache from existing marina maps
        self.seed_cache_from_existing_maps()

        # STEP 2: Generate REAL shortest path waypoints using trajectory optimizer
        print("\n=== Generating Real Shortest Path ===")
        print("Checking cache for shortest path waypoints...")
        print("Note: Shortest path is independent of car configuration")

        # Try to load from cache first
        cached_sp_waypoints = self.load_trajectory_cache('shortest_path')

        if cached_sp_waypoints:
            print(
                f"✓ Using cached shortest path with {len(cached_sp_waypoints)} waypoints!")
            trajectory_data['sp'] = cached_sp_waypoints
            sp_description = f"cached shortest path ({len(cached_sp_waypoints)} waypoints)"
        else:
            print("No cached shortest path found. Generating new one...")
            print(
                "This uses geometric optimization to find the shortest distance path between track boundaries.")

            try:
                real_sp_waypoints = self.generate_real_shortest_path(
                    output_dir, trajectory_data['centerline'], safety_width_sp=0.5)

                if real_sp_waypoints:
                    print(
                        f"✓ Successfully generated {len(real_sp_waypoints)} real shortest path waypoints!")
                    trajectory_data['sp'] = real_sp_waypoints
                    sp_description = f"real shortest path ({len(real_sp_waypoints)} waypoints)"

                    # Cache the generated shortest path (unscaled)
                    self.save_trajectory_cache(
                        'shortest_path', real_sp_waypoints)
                else:
                    print(
                        "✗ Real shortest path generation failed. Using moderate racing line as fallback.")
                    sp_description = f"fallback moderate racing line ({len(trajectory_data['sp'])} waypoints)"

            except Exception as e:
                print(
                    f"✗ Real shortest path generation failed with error: {e}")
                print("Using moderate racing line as fallback.")
                sp_description = f"fallback moderate racing line ({len(trajectory_data['sp'])} waypoints)"

        # STEP 2.5: Generate REAL racing line with speed profile using trajectory optimizer
        print("\n=== Generating Optimized Racing Line ===")

        # Check if racing line generation is enabled
        racing_line_type = getattr(self, 'racing_line_type', 'mintime')

        if racing_line_type == 'disable':
            print("Racing line generation disabled - using pre-calculated data from CSV")
            iqp_description = f"pre-calculated racing line from CSV ({len(trajectory_data['iqp'])} waypoints)"
        else:
            print(
                f"Checking cache for {racing_line_type} racing line for car: {self.car_name}...")
            print(f"Racing line optimization type: {racing_line_type}")

            # Try to load from cache first
            cached_iqp_waypoints = self.load_trajectory_cache(
                'racing_line', racing_line_type, self.car_name)

            if cached_iqp_waypoints:
                print(
                    f"✓ Using cached {racing_line_type} racing line with {len(cached_iqp_waypoints)} waypoints!")
                trajectory_data['iqp'] = cached_iqp_waypoints
                iqp_description = f"cached {racing_line_type} racing line for {self.car_name} ({len(cached_iqp_waypoints)} waypoints)"
            else:
                print(
                    f"No cached {racing_line_type} racing line found for {self.car_name}. Generating new one...")
                print(
                    f"This will generate car-specific optimized racing line for {self.car_name}")

                try:
                    # Generate racing line with specified optimization type
                    real_iqp_waypoints = self.generate_racing_line_with_speed_profile(
                        output_dir, trajectory_data['centerline'], optimization_type=racing_line_type)

                    if real_iqp_waypoints:
                        print(
                            f"✓ Successfully generated {len(real_iqp_waypoints)} optimized racing line waypoints!")
                        trajectory_data['iqp'] = real_iqp_waypoints
                        iqp_description = f"optimized {racing_line_type} racing line for {self.car_name} ({len(real_iqp_waypoints)} waypoints)"

                        # Cache the generated racing line (unscaled)
                        self.save_trajectory_cache(
                            'racing_line', real_iqp_waypoints, racing_line_type, self.car_name)
                    else:
                        print(
                            "✗ Racing line generation failed. Using pre-calculated IQP data as fallback.")
                        iqp_description = f"fallback pre-calculated racing line ({len(trajectory_data['iqp'])} waypoints)"

                except Exception as e:
                    print(f"✗ Racing line generation failed with error: {e}")
                    print("Using pre-calculated IQP data as fallback.")
                    iqp_description = f"fallback pre-calculated racing line ({len(trajectory_data['iqp'])} waypoints)"

            # Also generate a minimum curvature racing line for SP if we don't have real shortest path
            if not trajectory_data.get('sp') or len(trajectory_data['sp']) == 0:
                print(
                    "\n=== Generating Minimum Curvature Racing Line as SP fallback ===")

                # Check cache for mincurv racing line
                cached_mincurv_waypoints = self.load_trajectory_cache(
                    'racing_line', 'mincurv', self.car_name)

                if cached_mincurv_waypoints:
                    print(
                        f"✓ Using cached minimum curvature racing line with {len(cached_mincurv_waypoints)} waypoints!")
                    trajectory_data['sp'] = cached_mincurv_waypoints
                    sp_description = f"cached minimum curvature racing line for {self.car_name} ({len(cached_mincurv_waypoints)} waypoints)"
                else:
                    print("Generating new minimum curvature racing line...")
                    real_mincurv_waypoints = self.generate_racing_line_with_speed_profile(
                        output_dir, trajectory_data['centerline'], optimization_type="mincurv")

                    if real_mincurv_waypoints:
                        print(
                            f"✓ Successfully generated {len(real_mincurv_waypoints)} minimum curvature racing line waypoints!")
                        trajectory_data['sp'] = real_mincurv_waypoints
                        sp_description = f"minimum curvature racing line for {self.car_name} ({len(real_mincurv_waypoints)} waypoints)"

                        # Cache the generated mincurv racing line
                        self.save_trajectory_cache(
                            'racing_line', real_mincurv_waypoints, 'mincurv', self.car_name)
                    else:
                        print(
                            "✗ Minimum curvature racing line generation failed. Keeping existing SP data.")

        # STEP 3: Scale all trajectory data for final output
        print("\nScaling all trajectory data for output...")
        print(f"Applying scale factor: {self.scale_factor}")

        scaled_trajectory_data = {}
        for traj_type, waypoints in trajectory_data.items():
            scaled_waypoints = self.scale_waypoints(waypoints)
            scaled_trajectory_data[traj_type] = scaled_waypoints
            print(f"  - {traj_type}: {len(scaled_waypoints)} waypoints scaled")

        # Replace unscaled data with scaled data
        trajectory_data = scaled_trajectory_data

        # Create and write all files
        print("\nCreating output files...")

        # 1. Create track image first (returns origin coordinates)
        try:
            origin_x, origin_y = self.create_track_image(output_dir)
        except Exception as e:
            print(f"Warning: Failed to create track image: {e}")
            print("Using placeholder image instead.")
            self.copy_placeholder_image(output_dir)
            origin_x, origin_y = None, None

        # 2. global_waypoints.json
        global_waypoints = self.create_global_waypoints_json(trajectory_data)
        json_path = os.path.join(output_dir, 'global_waypoints.json')
        self.write_json_file(global_waypoints, json_path)

        # 3. map.yaml (use origin from generated image if available)
        map_config = self.create_map_yaml(
            trajectory_data['centerline'], origin_x, origin_y)
        yaml_path = os.path.join(output_dir, f'{self.output_map_name}.yaml')
        self.write_yaml_file(map_config, yaml_path)

        # 4. ot_sectors.yaml
        ot_sectors = self.create_ot_sectors_yaml(trajectory_data['centerline'])
        ot_path = os.path.join(output_dir, 'ot_sectors.yaml')
        self.write_yaml_file(ot_sectors, ot_path)

        # 5. speed_scaling.yaml
        speed_scaling = self.create_speed_scaling_yaml(
            trajectory_data['centerline'])
        speed_path = os.path.join(output_dir, 'speed_scaling.yaml')
        self.write_yaml_file(speed_scaling, speed_path)

        # 6. starting_position.yaml
        starting_position = self.create_starting_position_config(
            trajectory_data)
        start_path = os.path.join(output_dir, 'starting_position.yaml')
        self.write_yaml_file(starting_position, start_path)

        print(f"\n=== Conversion Complete ===")
        print(f"Output directory: {output_dir}")
        print(f"Waypoints per trajectory:")
        print(
            f"  - Centerline: {len(trajectory_data['centerline'])} waypoints")
        print(f"  - IQP: {len(trajectory_data['iqp'])} waypoints")
        print(f"  - SP: {len(trajectory_data['sp'])} waypoints")

        # Add debugging information about the track
        centerline_waypoints = trajectory_data['centerline']
        if centerline_waypoints:
            print(f"\n=== Track Analysis ===")
            x_coords = [wp['x_m'] for wp in centerline_waypoints]
            y_coords = [wp['y_m'] for wp in centerline_waypoints]
            print(
                f"Track bounds: X=[{min(x_coords):.2f}, {max(x_coords):.2f}], Y=[{min(y_coords):.2f}, {max(y_coords):.2f}]")

            # Check track width statistics
            widths = [wp['d_left'] + wp['d_right']
                      for wp in centerline_waypoints]
            print(
                f"Track width: min={min(widths):.2f}m, max={max(widths):.2f}m, avg={sum(widths)/len(widths):.2f}m")

            # Check curvature statistics
            curvatures = [abs(wp['kappa_radpm'])
                          for wp in centerline_waypoints]
            print(
                f"Curvature: max={max(curvatures):.4f} rad/m, avg={sum(curvatures)/len(curvatures):.4f} rad/m")

        print("\nTrajectory types generated:")
        print(
            f"  - Centerline: {len(trajectory_data['centerline'])} waypoints (conservative)")
        print(
            f"  - IQP: {len(trajectory_data['iqp'])} waypoints ({iqp_description if 'iqp_description' in locals() else 'racing line'})")
        print(
            f"  - SP: {len(trajectory_data['sp'])} waypoints ({sp_description if 'sp_description' in locals() else 'shortest path/moderate racing line'})")

        # Handle different trajectory lengths properly
        if len(trajectory_data['iqp']) != len(trajectory_data['centerline']):
            print(
                f"\nNote: IQP trajectory has {len(trajectory_data['iqp'])} waypoints vs centerline {len(trajectory_data['centerline'])} waypoints")
            print(
                "This is normal as optimized racing lines have different sampling than centerline data")

        print(f"\nCar-specific optimization details:")
        print(f"  - Vehicle configuration: {self.car_name}")
        print(
            f"  - Racing line optimization: Uses {self.car_name} vehicle dynamics, tire limits, and power constraints")
        print(
            f"  - Speed profiles: Generated specifically for {self.car_name} performance characteristics")

        print("\nGenerated files:")
        print(f"  - {self.output_map_name}.yaml (map configuration)")
        print(f"  - global_waypoints.json (waypoint data with 3 trajectory types)")
        print(f"  - ot_sectors.yaml (overtaking sectors)")
        print(f"  - speed_scaling.yaml (speed limits)")
        print(f"  - starting_position.yaml (car initial position)")
        print(
            f"  - {self.output_map_name}.png (track image generated from boundaries)")

        print("\nNext steps:")
        print("1. Review the generated track image for accuracy")
        print("2. Use the starting position from starting_position.yaml in your simulator launch")
        print("3. Adjust the origin and resolution in the .yaml file if needed")
        print("4. Configure overtaking sectors and speed scaling as desired")
        print(
            f"5. Test with: roslaunch stack_master base_system.launch map_name:={self.output_map_name}")

        print(f"\n=== Racing Line Generation Summary ===")
        print(
            f"This script now generates optimized racing lines specifically for the {self.car_name} vehicle:")
        print("• IQP (Aggressive): Minimum lap time optimization using car dynamics and tire limits")
        print("• SP (Moderate): Shortest path or minimum curvature optimization for smooth driving")
        print("• Centerline (Conservative): Traditional safe driving line for collision avoidance")
        print(
            f"• All speed profiles are calculated for {self.car_name} vehicle parameters")
        print("• Racing lines consider vehicle mass, wheelbase, steering limits, and power constraints")
        print("• The generated trajectories provide realistic lap times and accelerations")

        print(f"\n=== Debugging Tips ===")
        print("If the car starts outside track boundaries or spins:")
        print("1. Check that the map origin in the .yaml file matches the track image")
        print("2. Verify the starting position is on the centerline (check starting_position.yaml)")
        print("3. Ensure the track width multiplier hasn't made the track too narrow")
        print("4. Check that waypoint headings are correctly oriented along the track")
        print("5. Verify the coordinate system matches between map, waypoints, and starting position")

        return True

    def create_car_specific_vehicle_params(self, filepath: str, car_name: str):
        """Create car-specific vehicle parameters file for trajectory optimization.

        Args:
            filepath: Path where to write the vehicle parameters file
            car_name: Name of the car configuration (e.g., "NUC2", "TEST", "TAM_10%")
        """
        print(f"Creating vehicle parameters for car: {car_name}")

        # Load car-specific parameters from the configuration files
        car_config_path = os.path.expanduser(
            f"~/catkin_ws/src/race_stack/stack_master/config/{car_name}")

        if not os.path.exists(car_config_path):
            print(
                f"Warning: Car configuration directory not found: {car_config_path}")
            print("Using default NUC2 parameters as fallback")
            car_name = "NUC2"
            car_config_path = os.path.expanduser(
                f"~/catkin_ws/src/race_stack/stack_master/config/{car_name}")

        # Try to load car_model.yaml
        car_model_file = os.path.join(car_config_path, "car_model.yaml")

        # Default NUC2 parameters as fallback
        default_params = {
            'mass': 3.54,
            'wheelbase_front': 0.162,
            'wheelbase_rear': 0.145,
            'track_width_front': 0.281,
            'track_width_rear': 0.281,
            'cog_z': 0.014,
            'I_z': 0.05797,
            'max_steering_angle': 0.4189,
            'max_velocity': 10.0,
            'max_acceleration': 3.0,
            'min_acceleration': -3.0,
        }

        car_params = default_params.copy()

        if os.path.exists(car_model_file):
            try:
                import yaml
                with open(car_model_file, 'r') as f:
                    car_model_data = yaml.safe_load(f)

                # Extract relevant parameters
                if car_model_data:
                    car_params.update({
                        'mass': car_model_data.get('m', default_params['mass']),
                        'wheelbase_front': car_model_data.get('lf', default_params['wheelbase_front']),
                        'wheelbase_rear': car_model_data.get('lr', default_params['wheelbase_rear']),
                        'track_width_front': car_model_data.get('w_front', default_params['track_width_front']),
                        'track_width_rear': car_model_data.get('w_rear', default_params['track_width_rear']),
                        'cog_z': car_model_data.get('h_cg', default_params['cog_z']),
                        'I_z': car_model_data.get('I_z', default_params['I_z']),
                        'max_steering_angle': car_model_data.get('s_max', default_params['max_steering_angle']),
                    })

                print(f"✓ Loaded car parameters from {car_model_file}")

            except Exception as e:
                print(f"Warning: Could not load car_model.yaml: {e}")
                print("Using default NUC2 parameters")
        else:
            print(f"Car model file not found: {car_model_file}")
            print("Using default NUC2 parameters")

        print(f"Using {car_name} vehicle parameters:")
        print(f"  Mass: {car_params['mass']} kg")
        print(
            f"  Wheelbase: {car_params['wheelbase_front'] + car_params['wheelbase_rear']:.3f} m")
        print(
            f"  Max steering: {car_params['max_steering_angle']:.3f} rad ({car_params['max_steering_angle']*180/3.14159:.1f}°)")
        print(f"  Max velocity: {car_params['max_velocity']} m/s")

        # Create the INI file content with the loaded parameters
        ini_content = f"""# ----------------------------------------------------------------------------------------------------------------------
[GENERAL_OPTIONS]

### set name of ggv diagram and ax_max_machines files to use
ggv_file="ggv.csv"
ax_max_machines_file="ax_max_machines.csv"

### stepsize options
stepsize_opts={{"stepsize_prep": 0.05,
               "stepsize_reg": 0.2,
               "stepsize_interp_after_opt": 0.1}}

### spline regression smooth options
reg_smooth_opts={{"k_reg": 3,
                 "s_reg": 1}}

### preview and review distances for numerical curvature calculation
curv_calc_opts={{"d_preview_curv": 2.0,
                "d_review_curv": 2.0,
                "d_preview_head": 1.0,
                "d_review_head": 1.0}}

### general driving parameters ({car_name} specific)
veh_params={{"v_max": {car_params['max_velocity']:.1f},
             "length": {car_params['wheelbase_front'] + car_params['wheelbase_rear']:.6f},
             "width": {car_params['track_width_front']:.6f},
             "mass": {car_params['mass']:.2f},
             "g": 9.81,
             "dragcoeff": 0.0136,
             "curvlim": 3.0}}

### velocity profile calculation options ({car_name} specific)
vel_calc_opts={{"dyn_model_exp": 1,
               "mu_max": 1.2,
               "v_max": {car_params['max_velocity']:.1f},
               "v_min": 2.0,
               "length": {car_params['wheelbase_front'] + car_params['wheelbase_rear']:.6f},
               "width": {car_params['track_width_front']:.6f},
               "mass": {car_params['mass']:.2f},
               "dragcoeff": 0.0136,
               "curvlim": 3.0,
               "g": 9.81,
               "liftcoeff_front": 0.001,
               "liftcoeff_rear": 0.0015,
               "k_brake_front": 0.5,
               "k_drive_front": 0.0,
               "k_roll": 0.5}}

[OPTIMIZATION_OPTIONS]

### shortest path optimization options - optimized for better speed profiles
optim_opts_shortest_path={{"width_opt": 0.8}}

### minimum curvature optimization options  
optim_opts_mincurv={{"width_opt": 0.8,
                    "iqp_iters_min": 5,
                    "iqp_curverror_allowed": 0.1}}

### minimum time optimization options ({car_name} specific)
optim_opts_mintime={{"width_opt": 0.9,
                    "penalty_delta": 500.0,
                    "penalty_F": 100.0,
                    "mue": 1.2,
                    "n_gauss": 3,
                    "dn": 0.025,
                    "limit_energy": false,
                    "energy_limit": 2.0,
                    "safe_traj": false,
                    "ax_pos_safe": null,
                    "ax_neg_safe": null,
                    "ay_safe": null,
                    "w_tr_reopt": 1.0,
                    "w_veh_reopt": 0.8,
                    "w_add_spl_regr": 0.0,
                    "step_non_reg": 0,
                    "eps_kappa": 1e-3}}

### vehicle parameters (minimum lap time optimization) - {car_name} specific
vehicle_params_mintime={{"wheelbase": {car_params['wheelbase_front'] + car_params['wheelbase_rear']:.6f},
                        "wheelbase_front": {car_params['wheelbase_front']:.6f},
                        "wheelbase_rear": {car_params['wheelbase_rear']:.6f},
                        "track_width_front": {car_params['track_width_front']:.6f},
                        "track_width_rear": {car_params['track_width_rear']:.6f},
                        "cog_z": {car_params['cog_z']:.6f},
                        "I_z": {car_params['I_z']:.6f},
                        "liftcoeff_front": 0.001,
                        "liftcoeff_rear": 0.0015,
                        "k_brake_front": 0.5,
                        "k_drive_front": 0.0,
                        "k_roll": 0.5,
                        "t_delta": 0.05,
                        "t_drive": 0.05,
                        "t_brake": 0.05,
                        "power_max": 267,
                        "f_drive_max": {car_params['mass'] * 9.81 * 1.2:.1f},
                        "f_brake_max": {car_params['mass'] * 9.81 * 1.5:.1f},
                        "delta_max": {car_params['max_steering_angle']:.6f}}}

### tire parameters (minimum lap time optimization)
tire_params_mintime={{"c_roll": 0.013,
                     "f_z0": 300,
                     "B_front": 10.0,
                     "C_front": 1.8,
                     "eps_front": -0.1,
                     "E_front": 1.0,
                     "B_rear": 10.0,
                     "C_rear": 1.8,
                     "eps_rear": -0.1,
                     "E_rear": 1.0}}

### powertrain behavior
pwr_params_mintime={{"pwr_behavior": false}}
"""

        with open(filepath, 'w') as f:
            f.write(ini_content)

        print(
            f"Created {car_name}-specific vehicle parameters file: {filepath}")
        return car_params

    def create_car_specific_ggv(self, filepath: str, car_name: str):
        """Create car-specific GGV (g-g velocity) diagram for trajectory optimization.

        This creates a performance envelope that defines the maximum accelerations
        the car can achieve at different speeds.
        """
        print(f"Creating GGV diagram for car: {car_name}")

        # Default values (can be tuned per car)
        base_ax_max = 8.0  # m/s^2 - maximum longitudinal acceleration at low speed
        base_ay_max = 12.0  # m/s^2 - maximum lateral acceleration at low speed

        # Car-specific tuning
        if car_name == "NUC2":
            base_ax_max = 8.0
            base_ay_max = 12.0
        elif car_name == "TEST":
            base_ax_max = 6.0  # Smaller/lighter car
            base_ay_max = 10.0
        elif "TAM" in car_name:
            base_ax_max = 5.0  # Scaled car with lower performance
            base_ay_max = 8.0

        with open(filepath, 'w') as f:
            f.write("vx_mps,ax_max_mps2,ay_max_mps2\n")

            for v in range(0, 21):  # 0-20 m/s
                # Acceleration decreases with speed (tire friction limit)
                ax_max = max(1.0, base_ax_max - v * 0.2)
                ay_max = max(2.0, base_ay_max - v * 0.2)
                f.write(f"{v},{ax_max:.2f},{ay_max:.2f}\n")

        print(f"Created GGV diagram: {filepath}")

    def create_car_specific_ax_max(self, filepath: str, car_name: str):
        """Create car-specific maximum acceleration curve for trajectory optimization."""
        print(f"Creating ax_max curve for car: {car_name}")

        # Default values (can be tuned per car)
        base_ax_max = 8.0

        # Car-specific tuning
        if car_name == "NUC2":
            base_ax_max = 8.0
        elif car_name == "TEST":
            base_ax_max = 6.0
        elif "TAM" in car_name:
            base_ax_max = 5.0

        with open(filepath, 'w') as f:
            f.write("v_mps,ax_max_mps2\n")

            for v in range(0, 21):  # 0-20 m/s
                ax_max = max(1.0, base_ax_max - v * 0.2)
                f.write(f"{v},{ax_max:.2f}\n")

        print(f"Created ax_max curve: {filepath}")

    def create_nuc2_vehicle_params(self, filepath: str):
        """Create NUC2-specific vehicle parameters file for trajectory optimization.

        This uses the actual NUC2 car parameters from the config files instead of 
        generic F110 parameters.
        """
        # NUC2 car parameters from ~/catkin_ws/src/race_stack/stack_master/config/NUC2/car_model.yaml
        nuc2_params = {
            'mass': 3.54,  # kg
            'wheelbase_front': 0.162,  # lf from car_model.yaml
            'wheelbase_rear': 0.145,   # lr from car_model.yaml
            'track_width_front': 0.281,  # Keep standard F110 track width
            'track_width_rear': 0.281,
            'cog_z': 0.014,  # h_cg from car_model.yaml
            'I_z': 0.05797,  # Iz from car_model.yaml
            'max_steering_angle': 0.4189,  # max_steering_angle from car_model.yaml
            'max_velocity': 10.0,  # v_max from car_model.yaml
            'max_acceleration': 3.0,  # a_max from car_model.yaml
            'min_acceleration': -3.0,  # a_min from car_model.yaml
        }

        print(f"Using NUC2 vehicle parameters:")
        print(f"  Mass: {nuc2_params['mass']} kg")
        print(
            f"  Wheelbase: {nuc2_params['wheelbase_front'] + nuc2_params['wheelbase_rear']:.3f} m")
        print(
            f"  Max steering: {nuc2_params['max_steering_angle']:.3f} rad ({nuc2_params['max_steering_angle']*180/3.14159:.1f}°)")
        print(f"  Max velocity: {nuc2_params['max_velocity']} m/s")

        # Create the INI file content based on the original F110 template but with NUC2 values
        # NOTE: The trajectory optimizer expects proper JSON format within the INI file
        ini_content = f"""# ----------------------------------------------------------------------------------------------------------------------
[GENERAL_OPTIONS]

### set name of ggv diagram and ax_max_machines files to use
ggv_file="ggv.csv"
ax_max_machines_file="ax_max_machines.csv"

### stepsize options
stepsize_opts={{"stepsize_prep": 0.05,
               "stepsize_reg": 0.2,
               "stepsize_interp_after_opt": 0.1}}

### spline regression smooth options
reg_smooth_opts={{"k_reg": 3,
                 "s_reg": 1}}

### preview and review distances for numerical curvature calculation
curv_calc_opts={{"d_preview_curv": 2.0,
                "d_review_curv": 2.0,
                "d_preview_head": 1.0,
                "d_review_head": 1.0}}

### general driving parameters (NUC2 specific)
veh_params={{"v_max": {nuc2_params['max_velocity']:.1f},
             "length": {nuc2_params['wheelbase_front'] + nuc2_params['wheelbase_rear']:.6f},
             "width": {nuc2_params['track_width_front']:.6f},
             "mass": {nuc2_params['mass']:.2f},
             "g": 9.81,
             "dragcoeff": 0.0136,
             "curvlim": 1.0}}

### velocity profile calculation options (NUC2 specific)
vel_calc_opts={{"dyn_model_exp": 1,
               "mu_max": 1.0,
               "v_max": {nuc2_params['max_velocity']:.1f},
               "length": {nuc2_params['wheelbase_front'] + nuc2_params['wheelbase_rear']:.6f},
               "width": {nuc2_params['track_width_front']:.6f},
               "mass": {nuc2_params['mass']:.2f},
               "dragcoeff": 0.0136,
               "curvlim": 1.0,
               "g": 9.81,
               "liftcoeff_front": 0.001,
               "liftcoeff_rear": 0.0015,
               "k_brake_front": 0.5,
               "k_drive_front": 0.0,
               "k_roll": 0.5}}

[OPTIMIZATION_OPTIONS]

### shortest path optimization options
optim_opts_shortest_path={{"width_opt": 0.5}}

### minimum curvature optimization options  
optim_opts_mincurv={{"width_opt": 0.5,
                    "iqp_iters_min": 5,
                    "iqp_curverror_allowed": 0.1}}

### minimum time optimization options (NUC2 specific)
optim_opts_mintime={{"width_opt": 0.5,
                    "penalty_delta": 1.0,
                    "penalty_F": 0.1,
                    "mue": 1.0,
                    "n_gauss": 5,
                    "dn": 0.025,
                    "limit_energy": false,
                    "energy_limit": 2.0,
                    "safe_traj": false,
                    "ax_pos_safe": null,
                    "ax_neg_safe": null,
                    "ay_safe": null,
                    "w_tr_reopt": 1.0,
                    "w_veh_reopt": 0.8,
                    "w_add_spl_regr": 0.0,
                    "step_non_reg": 0,
                    "eps_kappa": 1e-3}}

### vehicle parameters (minimum lap time optimization) - NUC2 specific
vehicle_params_mintime={{"wheelbase": {nuc2_params['wheelbase_front'] + nuc2_params['wheelbase_rear']:.6f},
                        "wheelbase_front": {nuc2_params['wheelbase_front']:.6f},
                        "wheelbase_rear": {nuc2_params['wheelbase_rear']:.6f},
                        "track_width_front": {nuc2_params['track_width_front']:.6f},
                        "track_width_rear": {nuc2_params['track_width_rear']:.6f},
                        "cog_z": {nuc2_params['cog_z']:.6f},
                        "I_z": {nuc2_params['I_z']:.6f},
                        "liftcoeff_front": 0.001,
                        "liftcoeff_rear": 0.0015,
                        "k_brake_front": 0.5,
                        "k_drive_front": 0.0,
                        "k_roll": 0.5,
                        "t_delta": 0.1,
                        "t_drive": 0.1,
                        "t_brake": 0.1,
                        "power_max": 267,
                        "f_drive_max": 33.4,
                        "f_brake_max": 47.4,
                        "delta_max": {nuc2_params['max_steering_angle']:.6f}}}

### tire parameters (minimum lap time optimization)
tire_params_mintime={{"c_roll": 0.013,
                     "f_z0": 300,
                     "B_front": 10.0,
                     "C_front": 2.5,
                     "eps_front": -0.1,
                     "E_front": 1.0,
                     "B_rear": 10.0,
                     "C_rear": 2.5,
                     "eps_rear": -0.1,
                     "E_rear": 1.0}}

### powertrain behavior
pwr_params_mintime={{"pwr_behavior": false}}
"""

        with open(filepath, 'w') as f:
            f.write(ini_content)

        print(f"Created NUC2-specific vehicle parameters file: {filepath}")


def main():
    """Main function for command line execution."""
    parser = argparse.ArgumentParser(
        description="Convert Marina CSV format to F1Tenth Race Stack format with optimized racing lines")
    parser.add_argument("csv_file", help="Path to Marina CSV file")
    parser.add_argument("--output-name", default="marina",
                        help="Output map name (default: marina)")
    parser.add_argument("--scale-factor", type=float, default=0.1,
                        help="Scale factor for map size (default: 0.1)")
    parser.add_argument("--width-multiplier", type=float, default=2.0,
                        help="Track width multiplier (default: 2.0)")
    parser.add_argument("--car-name", default="NUC2",
                        help="Car configuration name for trajectory optimization (default: NUC2)")
    parser.add_argument("--racing-line-type", default="mintime",
                        choices=["mintime", "mincurv", "disable"],
                        help="Racing line optimization type: 'mintime' for fastest lap, 'mincurv' for smooth line, 'disable' to use pre-calculated data (default: mintime)")

    # Cache management options
    parser.add_argument("--clear-cache", action="store_true",
                        help="Clear all cached trajectories before processing")
    parser.add_argument("--clear-shortest-path-cache", action="store_true",
                        help="Clear only shortest path cache before processing")
    parser.add_argument("--clear-racing-line-cache", action="store_true",
                        help="Clear only racing line cache for specified car before processing")
    parser.add_argument("--list-cache", action="store_true",
                        help="List all cached trajectories and exit")
    parser.add_argument("--force-regenerate", action="store_true",
                        help="Force regeneration of trajectories, ignoring cache")

    args = parser.parse_args()

    # Create parser instance
    marina_parser = MarinaMapParser(
        csv_file=args.csv_file,
        output_map_name=args.output_name,
        scale_factor=args.scale_factor,
        width_multiplier=args.width_multiplier,
        car_name=args.car_name,
        racing_line_type=args.racing_line_type
    )

    # Handle cache management options
    if args.list_cache:
        marina_parser.list_cache()
        return

    if args.clear_cache:
        marina_parser.clear_cache()
        print("✓ Cleared all cached trajectories")

    if args.clear_shortest_path_cache:
        marina_parser.clear_cache('shortest_path')
        print("✓ Cleared shortest path cache")

    if args.clear_racing_line_cache:
        marina_parser.clear_cache('racing_line', args.car_name)
        print(f"✓ Cleared racing line cache for {args.car_name}")

    # Store configuration options
    marina_parser.force_regenerate = args.force_regenerate

    if args.force_regenerate:
        print("⚠️  Force regeneration enabled - ignoring all cached trajectories")

    # Parse and generate output files
    marina_parser.parse()


if __name__ == "__main__":
    main()
