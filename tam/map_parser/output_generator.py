#!/usr/bin/env python3
"""
Output file generation for Marina map parser.
"""
import json
import math
import os
import sys
import yaml
from typing import List, Dict, Any, Tuple
from config import Waypoint, MapConfig


class OutputGenerator:
    """Generates output files for the race stack."""

    def __init__(self, config: MapConfig):
        self.config = config

    def create_output_directory(self) -> str:
        """Create and return output directory path."""
        base_path = os.path.expanduser(
            "~/catkin_ws/src/race_stack/tam/maps/output")
        output_dir = os.path.join(base_path, self.config.output_map_name)
        os.makedirs(output_dir, exist_ok=True)
        print(f"Created output directory: {output_dir}")
        return output_dir

    def generate_all_files(self, trajectory_data: Dict[str, List[Waypoint]],
                           output_dir: str, scaled_trackbounds: Dict = None, translation_offset: tuple = None) -> bool:
        """Generate all output files."""
        try:
            print(
                f"DEBUG: generate_all_files called with scaled_trackbounds keys: {list(scaled_trackbounds.keys()) if scaled_trackbounds else 'None'}")
            if scaled_trackbounds:
                print(
                    f"DEBUG: trackbounds_left count: {len(scaled_trackbounds.get('trackbounds_left', []))}")
                print(
                    f"DEBUG: trackbounds_right count: {len(scaled_trackbounds.get('trackbounds_right', []))}")

            # Convert trackbounds from waypoint format to coordinate format for track image
            trackbounds_for_image = None
            if scaled_trackbounds:
                trackbounds_for_image = self._convert_trackbounds_waypoints_to_coordinates(
                    scaled_trackbounds)

            # Create track image first (returns origin coordinates) with translation offset and trackbounds
            origin_x, origin_y = self.create_track_image(
                output_dir, scaled_trackbounds=trackbounds_for_image, translation_offset=translation_offset)

            # Generate configuration files
            self.create_global_waypoints_json(
                trajectory_data, output_dir, scaled_trackbounds)
            self.create_map_yaml(
                trajectory_data['centerline'], output_dir, origin_x, origin_y)
            self.create_ot_sectors_yaml(
                trajectory_data['centerline'], output_dir)
            self.create_speed_scaling_yaml(
                trajectory_data['centerline'], output_dir)
            self.create_starting_position_yaml(trajectory_data, output_dir)

            return True

        except Exception as e:
            print(f"Error generating output files: {e}")
            return False

    def create_global_waypoints_json(self, trajectory_data: Dict[str, List[Waypoint]],
                                     output_dir: str, scaled_trackbounds: Dict = None):
        """Create global_waypoints.json file."""
        centerline_waypoints = trajectory_data['centerline']
        iqp_waypoints = trajectory_data['iqp']
        sp_waypoints = trajectory_data['sp']

        # Create waypoint arrays with proper ROS headers
        centerline_array = self._create_waypoint_array(centerline_waypoints)
        iqp_array = self._create_waypoint_array(iqp_waypoints)
        sp_array = self._create_waypoint_array(sp_waypoints)

        # Calculate statistics
        lap_time = 108.68  # Approximate
        iqp_max_speed = max(
            wp.vx_mps for wp in iqp_waypoints) if iqp_waypoints else 0.0
        sp_max_speed = max(
            wp.vx_mps for wp in sp_waypoints) if sp_waypoints else 0.0
        iqp_lap_time = lap_time
        sp_lap_time = lap_time * 1.1 if sp_waypoints else 0.0

        # Create visualization markers
        centerline_markers = self._create_waypoint_markers(centerline_waypoints, "centerline",
                                                           {'r': 0, 'g': 0, 'b': 1, 'a': 1})
        iqp_markers = self._create_waypoint_markers(iqp_waypoints, "iqp",
                                                    {'r': 1, 'g': 0, 'b': 0, 'a': 1})
        sp_markers = self._create_waypoint_markers(sp_waypoints, "sp",
                                                   {'r': 0, 'g': 1, 'b': 0, 'a': 1})

        # Use scaled trackbounds if provided, otherwise create them from CSV
        if scaled_trackbounds:
            trackbounds_markers = self._convert_scaled_trackbounds_to_markers(
                self._convert_trackbounds_waypoints_to_coordinates(scaled_trackbounds))
            print("✓ Using pre-scaled trackbounds markers")

        # Create global waypoints structure
        global_waypoints = {
            'map_info_str': {
                'data': f'IQP estimated lap time: {iqp_lap_time:.4f}s; IQP maximum speed: {iqp_max_speed:.4f}m/s; SP estimated lap time: {sp_lap_time:.4f}s; SP maximum speed: {sp_max_speed:.4f}m/s'
            },
            'est_lap_time': {
                'data': float(sp_lap_time if sp_waypoints else iqp_lap_time)
            },
            'centerline_markers': centerline_markers,
            'centerline_waypoints': centerline_array,
            'global_traj_markers_iqp': iqp_markers,
            'global_traj_wpnts_iqp': iqp_array,
            'global_traj_markers_sp': sp_markers,
            'global_traj_wpnts_sp': sp_array,
            'trackbounds_markers': trackbounds_markers
        }

        # Write file
        json_path = os.path.join(output_dir, 'global_waypoints.json')
        with open(json_path, 'w') as f:
            json.dump(global_waypoints, f, indent=2)
        print(f"Written: {json_path}")

    def create_map_yaml(self, centerline_waypoints: List[Waypoint], output_dir: str,
                        origin_x: float, origin_y: float):
        """Create map YAML configuration."""
        map_config = {
            'free_thresh': 0.196,
            'image': f'{self.config.output_map_name}.png',
            'negate': 0,
            'occupied_thresh': 0.65,
            'origin': [origin_x, origin_y, 0],
            'resolution': 0.05000000074505806
        }

        yaml_path = os.path.join(
            output_dir, f'{self.config.output_map_name}.yaml')
        with open(yaml_path, 'w') as f:
            yaml.dump(map_config, f, default_flow_style=False)
        print(f"Written: {yaml_path}")

    def create_ot_sectors_yaml(self, centerline_waypoints: List[Waypoint], output_dir: str):
        """Create overtaking sectors configuration."""
        total_waypoints = len(centerline_waypoints)

        # Account for waypoint indexing (0-based) and ensure sectors don't exceed array bounds
        max_index = total_waypoints - 1

        # Create multiple overtaking sectors to avoid KeyError
        ot_sectors = {
            'n_sectors': 2,  # Changed from 1 to 2
            'yeet_factor': 2,
            'spline_len': 50,
            'ot_sector_begin': 0.5,
            'Overtaking_sector0': {
                'start': 0,
                # First half of track, ensure valid index
                'end': min(total_waypoints // 2 - 1, max_index - 1),
                'ot_flag': True
            },
            'Overtaking_sector1': {
                'start': total_waypoints // 2,
                'end': max_index - 1,  # Second half of track, ensure we don't access out of bounds
                'ot_flag': True
            }
        }

        ot_path = os.path.join(output_dir, 'ot_sectors.yaml')
        with open(ot_path, 'w') as f:
            yaml.dump(ot_sectors, f, default_flow_style=False)
        print(f"Written: {ot_path}")

    def create_speed_scaling_yaml(self, centerline_waypoints: List[Waypoint], output_dir: str):
        """Create speed scaling configuration."""
        total_waypoints = len(centerline_waypoints)

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
        print(f"Written: {speed_path}")

    def create_starting_position_yaml(self, trajectory_data: Dict[str, List[Waypoint]],
                                      output_dir: str):
        """Create starting position configuration."""
        centerline_waypoints = trajectory_data['centerline']

        if not centerline_waypoints:
            print("No centerline waypoints available for starting position")
            return

        # Find best starting position
        best_waypoint = self._find_best_starting_waypoint(centerline_waypoints)

        # Normalize heading angle
        heading = best_waypoint.psi_rad
        while heading > math.pi:
            heading -= 2 * math.pi
        while heading < -math.pi:
            heading += 2 * math.pi

        starting_config = {
            'car_init_x': best_waypoint.x_m,
            'car_init_y': best_waypoint.y_m,
            'car_init_theta': heading,
            'description': f"Starting position based on optimal centerline waypoint {best_waypoint.id}"
        }

        start_path = os.path.join(output_dir, 'starting_position.yaml')
        with open(start_path, 'w') as f:
            yaml.dump(starting_config, f, default_flow_style=False)
        print(f"Written: {start_path}")

        print(
            f"Starting position: x={best_waypoint.x_m:.3f}, y={best_waypoint.y_m:.3f}, theta={heading:.3f}")

    def create_track_image(self, output_dir: str, resolution: float = 0.05, scaled_trackbounds: Dict = None, translation_offset: tuple = None) -> Tuple[float, float]:
        """Create track image from boundary data."""
        try:
            import sys
            import os

            # Add the current directory to the Python path for imports
            current_dir = os.path.dirname(os.path.abspath(__file__))
            if current_dir not in sys.path:
                sys.path.insert(0, current_dir)

            from track_background_generator import TrackImageGenerator

            image_generator = TrackImageGenerator(self.config)
            return image_generator.create_track_image(output_dir, resolution, scaled_trackbounds, translation_offset)

        except Exception as e:
            print(f"Failed to create track image: {e}")
            # Return default origin
            return (-50.0, -100.0)

    def _create_waypoint_array(self, waypoints: List[Waypoint]) -> Dict[str, Any]:
        """Create waypoint array with ROS header."""
        wpnt_list = []

        # Ensure waypoints are properly ordered by their sequential ID for spline compatibility
        sorted_waypoints = sorted(waypoints, key=lambda wp: wp.id)

        # For closed tracks, remove the last waypoint if it duplicates the first one
        # This prevents spline interpolation errors due to duplicate x-coordinates
        if (len(sorted_waypoints) > 1 and
            abs(sorted_waypoints[0].x_m - sorted_waypoints[-1].x_m) < 1e-6 and
                abs(sorted_waypoints[0].y_m - sorted_waypoints[-1].y_m) < 1e-6):
            print(f"Removing duplicate endpoint waypoint for spline compatibility")
            sorted_waypoints = sorted_waypoints[:-1]

        # Ensure strictly increasing s_m (arc length) values for spline interpolation
        # Recalculate s_m based on cumulative distance between waypoints
        if len(sorted_waypoints) > 0:
            sorted_waypoints[0].s_m = 0.0
            for i in range(1, len(sorted_waypoints)):
                dx = sorted_waypoints[i].x_m - sorted_waypoints[i-1].x_m
                dy = sorted_waypoints[i].y_m - sorted_waypoints[i-1].y_m
                segment_length = (dx**2 + dy**2)**0.5
                sorted_waypoints[i].s_m = sorted_waypoints[i -
                                                           1].s_m + segment_length

        # Additional check to ensure strictly increasing x-coordinates for splines
        x_coords = [wp.x_m for wp in sorted_waypoints]
        if len(x_coords) != len(set(x_coords)):
            print(
                f"Warning: Found duplicate x-coordinates, attempting to fix for spline compatibility")
            # Find minimum x and reorder the track starting from there
            min_x_idx = min(range(len(sorted_waypoints)),
                            key=lambda i: sorted_waypoints[i].x_m)
            # Reorder waypoints starting from minimum x
            reordered_waypoints = sorted_waypoints[min_x_idx:] + \
                sorted_waypoints[:min_x_idx]

            # Recalculate s_m for reordered waypoints
            if len(reordered_waypoints) > 0:
                reordered_waypoints[0].s_m = 0.0
                for i in range(1, len(reordered_waypoints)):
                    dx = reordered_waypoints[i].x_m - \
                        reordered_waypoints[i-1].x_m
                    dy = reordered_waypoints[i].y_m - \
                        reordered_waypoints[i-1].y_m
                    segment_length = (dx**2 + dy**2)**0.5
                    reordered_waypoints[i].s_m = reordered_waypoints[i -
                                                                     1].s_m + segment_length

            # Check if this helps
            x_coords_new = [wp.x_m for wp in reordered_waypoints]
            if len(x_coords_new) == len(set(x_coords_new)):
                print(f"Successfully reordered waypoints for spline compatibility")
                sorted_waypoints = reordered_waypoints
            else:
                print(f"Warning: Still have duplicate x-coordinates after reordering")
                # Add tiny perturbations to ensure strictly increasing x
                for i in range(1, len(sorted_waypoints)):
                    if sorted_waypoints[i].x_m <= sorted_waypoints[i-1].x_m:
                        sorted_waypoints[i].x_m = sorted_waypoints[i-1].x_m + 1e-6
                print(
                    f"Added small perturbations to ensure strictly increasing x-coordinates")

        for wp in sorted_waypoints:
            wpnt_dict = {
                'id': wp.id,
                's_m': wp.s_m,
                'd_m': wp.d_m,
                'x_m': wp.x_m,
                'y_m': wp.y_m,
                'd_right': wp.d_right,
                'd_left': wp.d_left,
                'psi_rad': wp.psi_rad,
                'kappa_radpm': wp.kappa_radpm,
                'vx_mps': wp.vx_mps,
                'ax_mps2': wp.ax_mps2
            }
            wpnt_list.append(wpnt_dict)

        return {
            'header': {
                'seq': 1,
                'stamp': {'secs': 0, 'nsecs': 0},
                'frame_id': ""
            },
            'wpnts': wpnt_list
        }

    def _create_waypoint_markers(self, waypoints: List[Waypoint], marker_type: str,
                                 color: Dict) -> Dict:
        """Create visualization markers for waypoints."""
        markers = []
        sample_rate = 8  # Sample every 8th waypoint

        # Calculate speed scaling
        speeds = [wp.vx_mps for wp in waypoints]
        min_speed = min(speeds) if speeds else 1.0
        max_speed = max(speeds) if speeds else 10.0
        speed_range = max_speed - min_speed if max_speed > min_speed else 1.0

        base_scale = 0.05
        max_scale_multiplier = 5.0

        for i, wp in enumerate(waypoints[::sample_rate]):
            # Speed-based scaling
            speed_normalized = (wp.vx_mps - min_speed) / \
                speed_range if speed_range > 0 else 0.5
            scale_multiplier = 1.0 + speed_normalized * \
                (max_scale_multiplier - 1.0)
            scale = base_scale * scale_multiplier

            marker = {
                'header': {'frame_id': 'map'},
                'id': i,
                'type': 2,  # SPHERE
                'action': 0,  # ADD
                'pose': {
                    'position': {'x': wp.x_m, 'y': wp.y_m, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                },
                'scale': {'x': scale, 'y': scale, 'z': scale},
                'color': color,
                'lifetime': {'secs': 0, 'nsecs': 0}
            }
            markers.append(marker)

        return {'markers': markers}

    def _convert_scaled_trackbounds_to_markers(self, trackbounds_coords: Dict) -> Dict:
        """Convert trackbounds coordinates dict to marker structure."""
        try:
            markers = []

            # Extract left and right coordinate arrays
            left_points = trackbounds_coords.get('left', [])
            right_points = trackbounds_coords.get('right', [])

            # Create markers for left boundary points
            for i, (left_x, left_y) in enumerate(left_points):
                left_marker = {
                    'header': {'frame_id': 'map'},
                    'ns': 'trackbounds_left',
                    'id': i,
                    'type': 2,  # SPHERE
                    'action': 0,
                    'pose': {
                        'position': {'x': left_x, 'y': left_y, 'z': 0.0},
                        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                    },
                    'scale': {'x': 0.05, 'y': 0.05, 'z': 0.05},
                    'color': {'r': 1, 'g': 0, 'b': 0, 'a': 1},
                    'lifetime': {'secs': 0, 'nsecs': 0},
                    'frame_locked': False,
                    'points': [],
                    'colors': [],
                    'text': '',
                    'mesh_resource': '',
                    'mesh_use_embedded_materials': False
                }
                markers.append(left_marker)

            # Create markers for right boundary points
            for i, (right_x, right_y) in enumerate(right_points):
                right_marker = {
                    'header': {'frame_id': 'map'},
                    'ns': 'trackbounds_right',
                    'id': i + len(left_points),  # Offset ID to avoid conflicts
                    'type': 2,  # SPHERE
                    'action': 0,
                    'pose': {
                        'position': {'x': right_x, 'y': right_y, 'z': 0.0},
                        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                    },
                    'scale': {'x': 0.05, 'y': 0.05, 'z': 0.05},
                    'color': {'r': 0, 'g': 0, 'b': 1, 'a': 1},
                    'lifetime': {'secs': 0, 'nsecs': 0},
                    'frame_locked': False,
                    'points': [],
                    'colors': [],
                    'text': '',
                    'mesh_resource': '',
                    'mesh_use_embedded_materials': False
                }
                markers.append(right_marker)

            print(
                f"✓ Converted {len(left_points)} left + {len(right_points)} right trackbounds to {len(markers)} markers")
            return {'markers': markers}

        except Exception as e:
            print(
                f"Warning: Failed to convert scaled trackbounds to markers: {e}")
            return {'markers': []}

    def _convert_trackbounds_waypoints_to_coordinates(self, scaled_trackbounds: Dict) -> Dict:
        """Convert trackbounds from Waypoint objects to coordinate tuples for track image generation."""
        try:
            left_waypoints = scaled_trackbounds.get('trackbounds_left', [])
            right_waypoints = scaled_trackbounds.get('trackbounds_right', [])

            print(
                f"DEBUG: Converting trackbounds - left: {len(left_waypoints)}, right: {len(right_waypoints)}")

            # Convert waypoints to (x, y) coordinate tuples
            left_coords = [(wp.x_m, wp.y_m) for wp in left_waypoints]
            right_coords = [(wp.x_m, wp.y_m) for wp in right_waypoints]

            print(
                f"✓ Converted trackbounds: {len(left_coords)} left + {len(right_coords)} right coordinates")

            return {
                'left': left_coords,
                'right': right_coords
            }

        except Exception as e:
            print(
                f"Warning: Failed to convert trackbounds waypoints to coordinates: {e}")
            return {'left': [], 'right': []}

    def _find_best_starting_waypoint(self, waypoints: List[Waypoint]) -> Waypoint:
        """Find the best starting waypoint."""
        best_waypoint = None
        best_score = float('-inf')

        # Check first 50 waypoints for best starting position
        for i in range(min(50, len(waypoints))):
            wp = waypoints[i]

            # Score based on track width, curvature, and position
            total_width = wp.d_left + wp.d_right
            width_score = min(total_width / 3.0, 1.0)  # Prefer wider sections
            # Prefer straighter sections
            curvature_score = max(0, 1.0 - abs(wp.kappa_radpm) * 10.0)

            score = width_score * 0.6 + curvature_score * 0.4

            if score > best_score:
                best_score = score
                best_waypoint = wp

        return best_waypoint or waypoints[0]

    def generate_output(self, trajectory_data: Dict[str, List[Waypoint]], scaled_trackbounds: Dict = None, translation_offset: tuple = None) -> bool:
        # Create output directory
        output_dir = self.create_output_directory()

        # Generate output files with translation offset
        return (self.generate_all_files(
            trajectory_data, output_dir, scaled_trackbounds, translation_offset), output_dir)
