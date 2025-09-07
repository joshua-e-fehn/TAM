#!/usr/bin/env python3
"""
Cache management for trajectory data.
"""
import json
import os
import hashlib
import time
from typing import List, Dict, Any, Optional
from config import Waypoint, MapConfig, TrajectoryType


class CacheManager:
    """Manages caching of trajectory data."""

    def __init__(self, config: MapConfig):
        self.config = config

    def get_cache_key(self, trajectory_type: str, car_name: Optional[str] = None) -> str:
        """Generate cache key for the given configuration."""
        # Get CSV file hash for uniqueness
        with open(self.config.csv_file, 'rb') as f:
            csv_content = f.read()
            csv_hash = hashlib.md5(csv_content).hexdigest()[:8]

        # Shortest path is independent of car configuration
        if trajectory_type == TrajectoryType.SHORTEST_PATH:
            return f"sp_{csv_hash}_s{self.config.scale_factor}_w{self.config.width_multiplier}"

        # Racing line is car-specific
        if trajectory_type == TrajectoryType.RACING_LINE:
            car_suffix = f"_{car_name}" if car_name else f"_{self.config.car_name}"
            return f"rl_{csv_hash}_s{self.config.scale_factor}_w{self.config.width_multiplier}{car_suffix}"

        return f"{trajectory_type}_{csv_hash}_s{self.config.scale_factor}_w{self.config.width_multiplier}"

    def save_trajectory_cache(self, trajectory_type: str, waypoints: List[Waypoint],
                              optimization_type: Optional[str] = None, car_name: Optional[str] = None):
        """Save trajectory waypoints to cache."""
        if not waypoints:
            print(
                f"Warning: Cannot cache empty waypoint list for {trajectory_type}")
            return

        cache_key = self.get_cache_key(trajectory_type, car_name)
        cache_file = os.path.join(self.config.cache_dir, f"{cache_key}.json")

        # Convert waypoints to dictionaries
        waypoint_dicts = [wp.to_dict() for wp in waypoints]

        cache_data = {
            'trajectory_type': trajectory_type,
            'optimization_type': optimization_type,
            'car_name': car_name or self.config.car_name,
            'scale_factor': self.config.scale_factor,
            'width_multiplier': self.config.width_multiplier,
            'csv_file': os.path.basename(self.config.csv_file),
            'waypoints': waypoint_dicts,
            'metadata': {
                'waypoint_count': len(waypoints),
                'generated_timestamp': time.time()
            }
        }

        try:
            with open(cache_file, 'w') as f:
                json.dump(cache_data, f, indent=2)
            print(
                f"✓ Cached {len(waypoints)} {trajectory_type} waypoints to: {cache_file}")
        except Exception as e:
            print(f"Warning: Failed to save cache: {e}")

    def load_trajectory_cache(self, trajectory_type: str, optimization_type: Optional[str] = None,
                              car_name: Optional[str] = None, silent: bool = False) -> List[Waypoint]:
        """Load trajectory waypoints from cache."""
        # Check if force regeneration is enabled
        if self.config.force_regenerate:
            if not silent:
                print(
                    f"Force regeneration enabled - skipping {trajectory_type} cache")
            return []

        cache_key = self.get_cache_key(trajectory_type, car_name)
        cache_file = os.path.join(self.config.cache_dir, f"{cache_key}.json")

        if not os.path.exists(cache_file):
            if not silent:
                print(f"No cached {trajectory_type} found")
            return []

        try:
            with open(cache_file, 'r') as f:
                cache_data = json.load(f)

            # Validate cache data
            if not self._validate_cache_data(cache_data, trajectory_type, optimization_type, car_name):
                return []

            # Convert dictionaries back to Waypoint objects
            waypoints = []
            for wp_dict in cache_data['waypoints']:
                # Clean waypoint data - extract values from nested dictionaries if needed
                cleaned_wp_dict = {}
                for key, value in wp_dict.items():
                    if isinstance(value, dict) and 'data' in value:
                        # Extract value from nested dictionary structure
                        cleaned_wp_dict[key] = value['data']
                    else:
                        cleaned_wp_dict[key] = value

                waypoints.append(Waypoint(**cleaned_wp_dict))

            if not silent:
                print(
                    f"✓ Loaded {len(waypoints)} cached {trajectory_type} waypoints from: {cache_file}")
            return waypoints

        except Exception as e:
            if not silent:
                print(f"Warning: Failed to load {trajectory_type} cache: {e}")
            return []

    def _validate_cache_data(self, cache_data: Dict[str, Any], trajectory_type: str,
                             optimization_type: Optional[str], car_name: Optional[str]) -> bool:
        """Validate that cached data matches current configuration."""
        try:
            # Check basic parameters
            if (cache_data.get('scale_factor') != self.config.scale_factor or
                cache_data.get('width_multiplier') != self.config.width_multiplier or
                    cache_data.get('csv_file') != os.path.basename(self.config.csv_file)):
                print(
                    f"Cache parameters don't match current configuration for {trajectory_type}")
                return False

            # Check trajectory-specific parameters
            if trajectory_type == TrajectoryType.RACING_LINE:
                if (optimization_type and cache_data.get('optimization_type') != optimization_type):
                    print(
                        f"Cached optimization type doesn't match for {trajectory_type}")
                    return False

                cached_car = cache_data.get('car_name')
                expected_car = car_name or self.config.car_name
                if cached_car != expected_car:
                    print(
                        f"Cached car name doesn't match for {trajectory_type}")
                    return False

            return True

        except Exception as e:
            print(f"Cache validation failed for {trajectory_type}: {e}")
            return False

    def clear_cache(self, trajectory_type: Optional[str] = None, car_name: Optional[str] = None):
        """Clear cached trajectories."""
        if not os.path.exists(self.config.cache_dir):
            print("No cache directory found")
            return

        files_removed = 0
        for filename in os.listdir(self.config.cache_dir):
            if not filename.endswith('.json'):
                continue

            # Filter by trajectory type if specified
            if trajectory_type:
                if trajectory_type == TrajectoryType.SHORTEST_PATH and not filename.startswith('sp_'):
                    continue
                elif trajectory_type == TrajectoryType.RACING_LINE and not filename.startswith('rl_'):
                    continue
                elif trajectory_type not in filename:
                    continue

            # Filter by car name if specified
            if car_name and car_name not in filename:
                continue

            filepath = os.path.join(self.config.cache_dir, filename)
            try:
                os.remove(filepath)
                files_removed += 1
                print(f"Removed cache file: {filename}")
            except Exception as e:
                print(f"Failed to remove {filename}: {e}")

        if files_removed > 0:
            print(f"Cleared {files_removed} cache files")
        else:
            print("No matching cache files found to clear")

    def list_cache(self):
        """List all cached trajectories with details."""
        if not os.path.exists(self.config.cache_dir):
            print("No cache directory found")
            return

        cache_files = [f for f in os.listdir(
            self.config.cache_dir) if f.endswith('.json')]
        if not cache_files:
            print("No cached trajectories found")
            return

        print(f"\n=== Cached Trajectories ({len(cache_files)} files) ===")
        for filename in sorted(cache_files):
            filepath = os.path.join(self.config.cache_dir, filename)
            try:
                with open(filepath, 'r') as f:
                    cache_data = json.load(f)

                traj_type = cache_data.get('trajectory_type', 'unknown')
                waypoint_count = cache_data.get(
                    'metadata', {}).get('waypoint_count', 0)
                car_name = cache_data.get('car_name', 'N/A')
                opt_type = cache_data.get('optimization_type', 'N/A')
                timestamp = cache_data.get('metadata', {}).get(
                    'generated_timestamp', 0)

                age_hours = (time.time() - timestamp) / \
                    3600 if timestamp else 0

                print(f"  {filename}:")
                print(f"    Type: {traj_type} ({opt_type})")
                print(f"    Car: {car_name}")
                print(f"    Waypoints: {waypoint_count}")
                print(f"    Age: {age_hours:.1f}h")

            except Exception as e:
                print(f"  {filename}: Error reading file - {e}")

        print("=" * 50)

    def _seed_cache_from_existing_maps(self) -> bool:
        """Seed cache from existing map files if available."""

        # Shortest Path = SP and Race Line = IQP!
        print("\n=== Checking for existing maps to seed cache ===")

        # Define possible map directories
        possible_map_dirs = [
            os.path.join(os.path.dirname(self.config.csv_file),
                         "output"),  # TAM output directory
            os.path.join(os.path.dirname(self.config.csv_file), "..",
                         "..", "stack_master", "maps")  # Stack master maps
        ]

        # Make paths absolute and check if they exist
        valid_map_dirs = []
        for maps_dir in possible_map_dirs:
            abs_path = os.path.abspath(maps_dir)
            if os.path.exists(abs_path):
                valid_map_dirs.append(abs_path)
                print(f"Found map directory: {abs_path}")

        if not valid_map_dirs:
            print("No map directories found - skipping cache seeding")
            return False

        # Find map directories that match our current configuration
        matching_dirs = []
        base_name = self.config.base_map_name.lower()
        scale_str = f"{int(self.config.scale_factor * 100)}%s"
        width_str = f"{int(self.config.width_multiplier * 100)}%w"
        car_name = self.config.car_name
        racing_type = self.config.racing_line_type

        for maps_dir in valid_map_dirs:
            for item in os.listdir(maps_dir):
                item_path = os.path.join(maps_dir, item)
                if os.path.isdir(item_path):
                    item_lower = item.lower()
                    # Check if this directory matches our configuration pattern
                    if (base_name in item_lower and
                        scale_str in item and
                        width_str in item and
                        car_name in item and
                            racing_type in item):
                        matching_dirs.append(item_path)

        if not matching_dirs:
            print(f"No existing maps found matching current configuration:")
            print(f"  Base name: {base_name}")
            print(f"  Scale: {scale_str}, Width: {width_str}")
            print(f"  Car: {car_name}, Racing type: {racing_type}")
            return False

        print(f"Found {len(matching_dirs)} matching maps:")
        for dir_path in matching_dirs:
            print(f"  - {dir_path}")

        # Try to seed cache from existing maps
        seeded_count = 0
        for map_path in matching_dirs:
            waypoints_file = os.path.join(map_path, "global_waypoints.json")

            print(f"Checking for waypoints in: {waypoints_file}")

            if os.path.exists(waypoints_file):
                try:
                    # Load existing waypoint data
                    with open(waypoints_file, 'r') as f:
                        waypoint_data = json.load(f)

                    print(
                        f"Found waypoint data with keys: {list(waypoint_data.keys())}")

                    # Extract trajectories and seed cache
                    # Check for racing line (IQP) - try both possible key names
                    iqp_data = None
                    if 'global_traj_wpnts_iqp' in waypoint_data:
                        # Handle nested structure with 'wpnts' key
                        if isinstance(waypoint_data['global_traj_wpnts_iqp'], dict) and 'wpnts' in waypoint_data['global_traj_wpnts_iqp']:
                            iqp_data = waypoint_data['global_traj_wpnts_iqp']['wpnts']
                        elif isinstance(waypoint_data['global_traj_wpnts_iqp'], list):
                            iqp_data = waypoint_data['global_traj_wpnts_iqp']
                    elif 'iqp' in waypoint_data and len(waypoint_data['iqp']) > 0:
                        iqp_data = waypoint_data['iqp']

                    if iqp_data and len(iqp_data) > 0:
                        iqp_waypoints = [Waypoint(**wp) for wp in iqp_data]
                        self.save_trajectory_cache('racing_line', iqp_waypoints,
                                                   self.config.racing_line_type, self.config.car_name)
                        seeded_count += 1
                        print(
                            f"✓ Seeded racing line cache with {len(iqp_waypoints)} waypoints from {os.path.basename(map_path)}")

                    # Check for shortest path (SP) - try both possible key names
                    sp_data = None
                    if 'global_traj_wpnts_sp' in waypoint_data:
                        # Handle nested structure with 'wpnts' key
                        if isinstance(waypoint_data['global_traj_wpnts_sp'], dict) and 'wpnts' in waypoint_data['global_traj_wpnts_sp']:
                            sp_data = waypoint_data['global_traj_wpnts_sp']['wpnts']
                        elif isinstance(waypoint_data['global_traj_wpnts_sp'], list):
                            sp_data = waypoint_data['global_traj_wpnts_sp']
                    elif 'sp' in waypoint_data and len(waypoint_data['sp']) > 0:
                        sp_data = waypoint_data['sp']

                    if sp_data and len(sp_data) > 0:
                        sp_waypoints = [Waypoint(**wp) for wp in sp_data]
                        self.save_trajectory_cache(
                            'shortest_path', sp_waypoints)
                        seeded_count += 1
                        print(
                            f"✓ Seeded shortest path cache with {len(sp_waypoints)} waypoints from {os.path.basename(map_path)}")

                except Exception as e:
                    print(
                        f"⚠ Failed to seed cache from {os.path.basename(map_path)}: {e}")
                    continue
            else:
                print(f"⚠ No waypoints file found at: {waypoints_file}")

        if seeded_count > 0:
            print(
                f"✓ Successfully seeded {seeded_count} trajectory types from existing maps")
            return True
        else:
            print("⚠ No trajectory data could be seeded from existing maps")
            return False

    def check_cache(self) -> Dict[str, List[Waypoint]]:
        # First, load existing cache data (silently to avoid premature "not found" messages)
        cache_content = {}

        # Load shortest path cache
        cached_sp = self.load_trajectory_cache('shortest_path', silent=True)
        cache_content['shortest_path'] = cached_sp

        # Load racing line cache
        cached_rl = self.load_trajectory_cache(
            'racing_line', self.config.racing_line_type, self.config.car_name, silent=True)
        cache_content['racing_line'] = cached_rl

        # Check if any trajectories are missing and try to seed from existing maps
        missing_trajectories = [
            k for k, v in cache_content.items() if len(v) == 0]

        if missing_trajectories:
            seeded_cache = self._seed_cache_from_existing_maps()

            # Reload cache after seeding
            if seeded_cache:
                if 'shortest_path' in missing_trajectories:
                    cached_sp = self.load_trajectory_cache('shortest_path')
                    cache_content['shortest_path'] = cached_sp

                if 'racing_line' in missing_trajectories:
                    cached_rl = self.load_trajectory_cache(
                        'racing_line', self.config.racing_line_type, self.config.car_name)
                    cache_content['racing_line'] = cached_rl
            else:
                # If seeding failed, show which trajectories are missing
                final_missing = [
                    k for k, v in cache_content.items() if len(v) == 0]
                if final_missing:
                    for traj_type in final_missing:
                        print(f"No cached {traj_type} found")

        return cache_content
