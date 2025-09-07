from output_generator import OutputGenerator
from trajectory_generator import TrajectoryGenerator
from cache_manager import CacheManager
from track_scaler import TrackScaler
from input_processor import InputProcessor
from config import MapConfig, Waypoint
from typing import Dict, List, Tuple
import argparse
import sys
import os
import math

# Add the current directory to Python path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class MapParser:
    """Main parser class that orchestrates the conversion process."""

    def __init__(self, input_file: str, output_map_name: str = "marina",
                 scale_factor: float = 0.1, width_multiplier: float = 2.0,
                 car_name: str = "NUC2", racing_line_type: str = "mintime"):
        """Initialize the parser with configuration."""
        self.config = MapConfig(
            input_file=input_file,
            base_map_name=output_map_name,
            scale_factor=scale_factor,
            width_multiplier=width_multiplier,
            car_name=car_name,
            racing_line_type=racing_line_type
        )

        # Initialize components
        self.input_processor = InputProcessor(self.config)
        self.cache_manager = CacheManager(self.config)
        self.trajectory_generator = TrajectoryGenerator(
            self.config, self.cache_manager)
        self.scaler = TrackScaler(
            self.config.scale_factor, self.config.input_file)
        self.output_generator = OutputGenerator(self.config)

        self._print_configuration()

    def parse(self) -> bool:
        """Main parsing function following the specified logic flow."""
        print(
            f"=== {self.config.base_map_name.title()} Map CSV to F1Tenth Format Parser ===")

        try:
            print("\n=== Step 1: Processing Input ===")
            input_data = self._process_input()

            print("\n=== Step 2: Scaling Track ===")
            scaled_input_data = self._scale_track(input_data)

            print("\n=== Step 3: Checking Cache ===")
            cache_content = self._check_cache(scaled_input_data)

            print("\n=== Step 4: Create Trajectories ===")
            trajectory_data = self._create_trajectories(
                scaled_input_data, cache_content)

            print("\n=== Step 5: Generating Output ===")
            return self._generate_output(trajectory_data)
        except Exception as e:
            print(f"Error during parsing: {e}")
            return False

    def _process_input(self) -> Dict[str, List[Waypoint]]:
        input_data = self.input_processor.load_data()

        if not input_data or not input_data['centerline']:
            print("Error: No valid centerline data found")
            raise Exception("No valid centerline data found")

        print(
            f"✓ Loaded {len(input_data['centerline'])} centerline waypoints")
        return input_data

    def _scale_track(self, input_data: Dict[str, List[Waypoint]]) -> Dict[str, List[Waypoint]]:
        """Scale the input track data including centerline, trackbounds, and fallback raceline."""
        print(
            f"Scaling input track data with factor: {self.config.scale_factor}")

        scaled_data = {}
        translation_offset = None

        # Scale centerline first to determine translation offset
        if 'centerline' in input_data and input_data['centerline']:
            scaled_centerline = self.scaler.scale_and_translate_waypoints(
                input_data['centerline'])
            scaled_data['centerline'] = scaled_centerline

            # Calculate translation offset from first centerline waypoint
            if scaled_centerline:
                first_original = input_data['centerline'][0]
                first_scaled = self.scaler.scale_waypoint(first_original)
                translation_offset = (-first_scaled.x_m, -first_scaled.y_m)
                print(
                    f"✓ Translation offset determined: ({-translation_offset[0]:.6f}, {-translation_offset[1]:.6f}) → (0.000000, 0.000000)")

        # Scale trackbounds using the same translation offset
        if translation_offset is not None:
            for bounds_key in ['trackbounds_left', 'trackbounds_right']:
                if bounds_key in input_data and input_data[bounds_key]:
                    scaled_waypoints = self.scaler.scale_waypoints(
                        input_data[bounds_key])
                    translated_waypoints = self.scaler.translate_waypoints(
                        scaled_waypoints, translation_offset[0], translation_offset[1])
                    scaled_data[bounds_key] = translated_waypoints
                    print(
                        f"✓ Scaled {bounds_key}: {len(translated_waypoints)} waypoints")

        # Scale fallback raceline if available
        if 'fallback_raceline' in input_data and input_data['fallback_raceline']:
            scaled_waypoints = self.scaler.scale_waypoints(
                input_data['fallback_raceline'])
            if translation_offset:
                translated_waypoints = self.scaler.translate_waypoints(
                    scaled_waypoints, translation_offset[0], translation_offset[1])
                scaled_data['fallback_raceline'] = translated_waypoints
                print(
                    f"✓ Scaled fallback_raceline: {len(translated_waypoints)} waypoints")

        # Store translation offset for later use
        self.scaler.last_translation_offset = translation_offset

        print(
            f"✓ Track scaling complete. Scale factor: {self.config.scale_factor}")
        return scaled_data

    def _check_cache(self, scaled_input_data: Dict[str, List[Waypoint]]) -> Dict[str, List[Waypoint]]:
        cache_content = self.cache_manager.check_cache()

        # Print cache status summary
        cached_items = []
        missing_items = []

        for trajectory_type, waypoints in cache_content.items():
            if len(waypoints) > 0:
                cached_items.append(
                    f"{trajectory_type} ({len(waypoints)} waypoints)")
            else:
                missing_items.append(trajectory_type)

        if cached_items:
            print(f"✓ Found in cache: {', '.join(cached_items)}")
        if missing_items:
            print(f"⚠ Missing from cache: {', '.join(missing_items)}")
        if not cached_items and not missing_items:
            print("⚠ No cache manager available")

        return cache_content

    def _create_trajectories(self, scaled_input_data: Dict[str, List[Waypoint]], cache_content: Dict[str, List[Waypoint]]) -> Dict[str, List[Waypoint]]:
        print(
            f"DEBUG: _create_trajectories called with scaled_input_data keys: {list(scaled_input_data.keys())}")
        for key, value in scaled_input_data.items():
            print(f"DEBUG: {key}: {len(value)} waypoints")

        trajectory_data = self.trajectory_generator.create_trajectories(
            scaled_input_data, cache_content)

        print(
            f"DEBUG: trajectory_data returned with keys: {list(trajectory_data.keys())}")
        for key, value in trajectory_data.items():
            if isinstance(value, list):
                print(f"DEBUG: {key}: {len(value)} waypoints")

        self._validate_trajectory_data(trajectory_data)
        self._print_trajectory_summary(trajectory_data)
        return trajectory_data

    def _generate_output(self, trajectory_data: Dict[str, List[Waypoint]]) -> bool:
        # Extract trackbounds from trajectory_data (they were scaled earlier)
        scaled_trackbounds = {
            'trackbounds_left': trajectory_data.get('trackbounds_left', []),
            'trackbounds_right': trajectory_data.get('trackbounds_right', [])
        }
        translation_offset = getattr(
            self.scaler, 'last_translation_offset', (0.0, 0.0))

        (success, output_dir) = self.output_generator.generate_output(
            trajectory_data, scaled_trackbounds, translation_offset)

        if not success:
            raise Exception("Error: Failed to generate output files")
        self._print_completion_summary(trajectory_data, output_dir)
        return success

    def _validate_trajectory_data(self, trajectory_data: Dict[str, List[Waypoint]]) -> None:
        """Validate trajectory data and raise exception if invalid."""
        if not trajectory_data:
            raise Exception("No trajectory data generated")

        required_trajectories = ['centerline', 'iqp', 'sp']
        for traj_type in required_trajectories:
            if traj_type not in trajectory_data:
                raise Exception(f"Missing required trajectory: {traj_type}")

            waypoints = trajectory_data[traj_type]
            if not waypoints:
                raise Exception(f"Empty trajectory data for: {traj_type}")

            if len(waypoints) < 10:  # Minimum reasonable number of waypoints
                raise Exception(
                    f"Insufficient waypoints ({len(waypoints)}) for trajectory: {traj_type}")

        print("✓ Trajectory data validation passed")

    def _print_trajectory_summary(self, trajectory_data: Dict[str, List[Waypoint]]) -> None:
        """Print a summary of generated trajectory data."""
        print("\n--- Trajectory Generation Summary ---")

        trajectory_descriptions = {
            'centerline': 'Conservative centerline',
            'iqp': f'Optimized racing line ({self.config.racing_line_type})',
            'sp': 'Shortest path'
        }

        for traj_type, waypoints in trajectory_data.items():
            # Skip trackbounds data - it's not waypoint objects
            if traj_type == 'trackbounds':
                continue

            desc = trajectory_descriptions.get(traj_type, traj_type)
            print(f"  • {desc}: {len(waypoints)} waypoints")

            if waypoints:
                # Calculate basic statistics
                speeds = [
                    wp.vx_mps for wp in waypoints if hasattr(wp, 'vx_mps')]
                if speeds:
                    avg_speed = sum(speeds) / len(speeds)
                    max_speed = max(speeds)
                    min_speed = min(speeds)
                    print(
                        f"    Speed: avg={avg_speed:.1f} m/s, range=[{min_speed:.1f}, {max_speed:.1f}] m/s")

                # Calculate track length
                if len(waypoints) > 1:
                    total_distance = 0
                    for i in range(1, len(waypoints)):
                        dx = waypoints[i].x_m - waypoints[i-1].x_m
                        dy = waypoints[i].y_m - waypoints[i-1].y_m
                        total_distance += (dx**2 + dy**2)**0.5
                    print(f"    Track length: {total_distance:.1f} m")

        print("-------------------------------------")

    def _print_configuration(self):
        """Print parser configuration."""
        print(
            f"Using scale factor: {self.config.scale_factor} (map will be {self.config.scale_factor*100:.1f}% of original size)")
        print(
            f"Using width multiplier: {self.config.width_multiplier} (track will be {self.config.width_multiplier*100:.0f}% of original width)")
        print(f"Generated map name: {self.config.output_map_name}")
        print(
            f"Using car configuration: {self.config.car_name} for trajectory optimization")
        print(f"Cache directory: {self.config.cache_dir}")

    def _print_completion_summary(self, trajectory_data: Dict[str, List[Waypoint]], output_dir: str):
        """Print completion summary."""
        print(f"\n=== Conversion Complete ===")
        print(f"Output directory: {output_dir}")
        print(f"Waypoints per trajectory:")

        descriptions = {
            'centerline': 'conservative',
            'iqp': f'optimized {self.config.racing_line_type} racing line for {self.config.car_name}',
            'sp': 'shortest path'
        }

        for traj_type, waypoints in trajectory_data.items():
            # Skip trackbounds in trajectory descriptions
            if traj_type in ['trackbounds_left', 'trackbounds_right']:
                continue

            desc = descriptions.get(traj_type, traj_type)
            print(
                f"  - {traj_type.capitalize()}: {len(waypoints)} waypoints ({desc})")

        # Print track analysis
        centerline_waypoints = trajectory_data.get('centerline', [])
        if centerline_waypoints:
            x_coords = [wp.x_m for wp in centerline_waypoints]
            y_coords = [wp.y_m for wp in centerline_waypoints]
            widths = [wp.d_left + wp.d_right for wp in centerline_waypoints]
            curvatures = [abs(wp.kappa_radpm) for wp in centerline_waypoints]

            print(f"\n=== Track Analysis ===")
            print(
                f"Track bounds: X=[{min(x_coords):.2f}, {max(x_coords):.2f}], Y=[{min(y_coords):.2f}, {max(y_coords):.2f}]")
            print(
                f"Track width: min={min(widths):.2f}m, max={max(widths):.2f}m, avg={sum(widths)/len(widths):.2f}m")
            print(
                f"Curvature: max={max(curvatures):.4f} rad/m, avg={sum(curvatures)/len(curvatures):.4f} rad/m")

        print(f"\nGenerated files:")
        print(f"  - {self.config.output_map_name}.yaml (map configuration)")
        print(f"  - global_waypoints.json (waypoint data with 3 trajectory types)")
        print(f"  - ot_sectors.yaml (overtaking sectors)")
        print(f"  - speed_scaling.yaml (speed limits)")
        print(f"  - starting_position.yaml (car initial position)")
        print(f"  - {self.config.output_map_name}.png (track image)")

        print(f"\nNext steps:")
        print(f"1. Review the generated track image for accuracy")
        print(f"2. Use the starting position from starting_position.yaml in your simulator launch")
        print(
            f"3. Test with: roslaunch stack_master base_system.launch map_name:={self.config.output_map_name}")

    # Cache management methods
    def clear_cache(self, trajectory_type: str = None, car_name: str = None):
        self.cache_manager.clear_cache(trajectory_type, car_name)

    def list_cache(self):
        self.cache_manager.list_cache()

    def set_force_regenerate(self, force: bool):
        self.config.force_regenerate = force


def main():
    """Main function for command line execution."""
    parser = argparse.ArgumentParser(
        description="Convert TAM CSV format to F1Tenth Race Stack format with optimized racing lines")

    # Required arguments
    parser.add_argument("input_file", help="Path to input file (CSV or JSON)")

    # Optional configuration
    parser.add_argument("--output-name", default="marina",
                        help="Output map name (default: marina)")
    parser.add_argument("--scale-factor", type=float, default=0.1,
                        help="Scale factor for map size (default: 0.1)")
    parser.add_argument("--width-multiplier", type=float, default=1.0,
                        help="Track width multiplier (default: 1.0)")
    parser.add_argument("--car-name", default="NUC2",
                        help="Car configuration name for trajectory optimization (default: NUC2)")
    parser.add_argument("--racing-line-type", default="mintime",
                        choices=["mintime", "mincurv", "disable"],
                        help="Racing line optimization type (default: mintime)")

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

    try:
        # Create parser instance
        marina_parser = MapParser(
            input_file=args.input_file,
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
            print("Clearing all cached trajectories...")
            marina_parser.clear_cache()

        if args.clear_shortest_path_cache:
            print("Clearing shortest path cache...")
            marina_parser.clear_cache(trajectory_type="shortest_path")

        if args.clear_racing_line_cache:
            print(f"Clearing racing line cache for car: {args.car_name}...")
            marina_parser.clear_cache(
                trajectory_type="racing_line", car_name=args.car_name)

        # Set force regenerate flag
        if args.force_regenerate:
            print("Force regeneration enabled - ignoring all cached data")
            marina_parser.set_force_regenerate(True)

        # Parse and generate output files
        success = marina_parser.parse()

        if success:
            print(
                f"\n✓ {args.output_name.title()} map conversion completed successfully!")
            sys.exit(0)
        else:
            print(f"\n✗ {args.output_name.title()} map conversion failed!")
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nConversion interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error during conversion: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
