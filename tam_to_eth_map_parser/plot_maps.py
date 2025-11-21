#!/usr/bin/env python3
"""
Script to plot the Marina raceline trajectories from the generated map data.

This script visualizes the three trajectory types:
- Centerline (blue): Conservative path through track center
- IQP (red): Aggressive racing line for minimum lap time
- SP (green): Moderate racing line balancing speed and safety
"""

import json
import matplotlib.pyplot as plt
import numpy as np
import os


def load_waypoints_data(json_file):
    """Load waypoints data from the global_waypoints.json file."""
    with open(json_file, 'r') as f:
        data = json.load(f)

    # Extract waypoints for each trajectory type
    centerline_waypoints = data['centerline_waypoints']['wpnts']
    iqp_waypoints = data['global_traj_wpnts_iqp']['wpnts']
    sp_waypoints = data['global_traj_wpnts_sp']['wpnts']

    # Extract trackbounds markers
    trackbounds_markers = data.get(
        'trackbounds_markers', {}).get('markers', [])

    return centerline_waypoints, iqp_waypoints, sp_waypoints, trackbounds_markers


def extract_trackbounds_coordinates(trackbounds_markers):
    """Extract trackbounds coordinates from markers with robust error handling."""
    left_bounds_x, left_bounds_y = [], []
    right_bounds_x, right_bounds_y = [], []

    print(f"Processing {len(trackbounds_markers)} trackbounds markers...")

    valid_markers = 0
    invalid_markers = 0
    left_count = 0
    right_count = 0

    # Process all markers in the list
    for i, marker in enumerate(trackbounds_markers):
        try:
            # Check if marker has proper structure and position data
            if (isinstance(marker, dict) and
                'ns' in marker and
                marker.get('ns') in ['trackbounds_left', 'trackbounds_right'] and
                'pose' in marker and
                isinstance(marker['pose'], dict) and
                'position' in marker['pose'] and
                isinstance(marker['pose']['position'], dict) and
                'x' in marker['pose']['position'] and
                    'y' in marker['pose']['position']):

                x = float(marker['pose']['position']['x'])
                y = float(marker['pose']['position']['y'])

                # Add to appropriate boundary based on namespace
                if marker['ns'] == 'trackbounds_left':
                    left_bounds_x.append(x)
                    left_bounds_y.append(y)
                    left_count += 1
                elif marker['ns'] == 'trackbounds_right':
                    right_bounds_x.append(x)
                    right_bounds_y.append(y)
                    right_count += 1

                valid_markers += 1

        except (KeyError, TypeError, ValueError) as e:
            # Count invalid markers for debugging
            invalid_markers += 1
            continue

    print(f"Successfully extracted {valid_markers} valid trackbounds markers")
    print(f"Invalid markers skipped: {invalid_markers}")
    print(
        f"Left boundary points: {left_count}, Right boundary points: {right_count}")

    return left_bounds_x, left_bounds_y, right_bounds_x, right_bounds_y


def extract_coordinates_and_speeds(waypoints):
    """Extract x, y coordinates and speeds from waypoints."""
    # Handle both direct values and nested {'data': value} structure
    def get_value(wp, key):
        val = wp[key]
        return val['data'] if isinstance(val, dict) and 'data' in val else val

    x_coords = [get_value(wp, 'x_m') for wp in waypoints]
    y_coords = [get_value(wp, 'y_m') for wp in waypoints]
    speeds = [get_value(wp, 'vx_mps') for wp in waypoints]
    return x_coords, y_coords, speeds


def extract_full_trajectory_data(waypoints):
    """Extract all trajectory data including acceleration."""
    # Handle both direct values and nested {'data': value} structure
    def get_value(wp, key):
        val = wp[key]
        return val['data'] if isinstance(val, dict) and 'data' in val else val

    x_coords = [get_value(wp, 'x_m') for wp in waypoints]
    y_coords = [get_value(wp, 'y_m') for wp in waypoints]
    speeds = [get_value(wp, 'vx_mps') for wp in waypoints]
    accelerations = [get_value(wp, 'ax_mps2') for wp in waypoints]
    arc_lengths = [get_value(wp, 's_m') for wp in waypoints]
    return x_coords, y_coords, speeds, accelerations, arc_lengths


def plot_trajectories(centerline_waypoints, iqp_waypoints, sp_waypoints, trackbounds_markers):
    """Plot available trajectory types with track boundaries."""

    # Extract coordinates and speeds for available trajectories
    cl_x, cl_y, cl_speeds = extract_coordinates_and_speeds(
        centerline_waypoints)

    # Only extract IQP data if waypoints exist
    if iqp_waypoints:
        iqp_x, iqp_y, iqp_speeds = extract_coordinates_and_speeds(
            iqp_waypoints)
    else:
        iqp_x, iqp_y, iqp_speeds = [], [], []

    # Only extract SP data if waypoints exist
    if sp_waypoints:
        sp_x, sp_y, sp_speeds = extract_coordinates_and_speeds(sp_waypoints)
    else:
        sp_x, sp_y, sp_speeds = [], [], []

    # Extract track boundaries
    left_bounds_x, left_bounds_y, right_bounds_x, right_bounds_y = extract_trackbounds_coordinates(
        trackbounds_markers)

    # Create figure with subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))

    # Plot 1: Track layout with all trajectories and boundaries
    # Plot track boundaries first (so they appear behind trajectories)
    if left_bounds_x and left_bounds_y:
        ax1.plot(left_bounds_x, left_bounds_y, 'black', linewidth=2, alpha=0.8,
                 label=f'Left Track Boundary ({len(left_bounds_x)} points)')
    if right_bounds_x and right_bounds_y:
        ax1.plot(right_bounds_x, right_bounds_y, 'black', linewidth=2, alpha=0.8,
                 label=f'Right Track Boundary ({len(right_bounds_x)} points)')

    # Plot trajectories (only those that exist)
    ax1.plot(cl_x, cl_y, 'b-', linewidth=2,
             label=f'Centerline (avg: {np.mean(cl_speeds):.1f} m/s)', alpha=0.8)

    if iqp_waypoints:
        ax1.plot(iqp_x, iqp_y, 'r-', linewidth=2,
                 label=f'IQP - Aggressive (avg: {np.mean(iqp_speeds):.1f} m/s)', alpha=0.8)

    if sp_waypoints:
        ax1.plot(sp_x, sp_y, 'g-', linewidth=2,
                 label=f'SP - Moderate (avg: {np.mean(sp_speeds):.1f} m/s)', alpha=0.8)

    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title(
        'Marina Race Track (10% Scaled with Dimensionless Physics) - Available Trajectories')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # Add start/finish markers (use available trajectory)
    if iqp_waypoints:
        ax1.plot(iqp_x[0], iqp_y[0], 'ko', markersize=10, label='Start/Finish')
        ax1.text(iqp_x[0], iqp_y[0], '  Start/Finish', fontsize=10, ha='left')
    else:
        ax1.plot(cl_x[0], cl_y[0], 'ko', markersize=10, label='Start/Finish')
        ax1.text(cl_x[0], cl_y[0], '  Start/Finish', fontsize=10, ha='left')

    # Plot 2: Speed profile comparison (only for available trajectories)
    # Use arc length for x-axis (approximate)
    # Helper function to extract values
    def get_value(wp, key):
        val = wp[key]
        return val['data'] if isinstance(val, dict) and 'data' in val else val

    cl_s = [get_value(wp, 's_m') for wp in centerline_waypoints]

    ax2.plot(cl_s, cl_speeds, 'b-', linewidth=2,
             label=f'Centerline (max: {max(cl_speeds):.1f} m/s)')

    if iqp_waypoints:
        iqp_s = [get_value(wp, 's_m') for wp in iqp_waypoints]
        ax2.plot(iqp_s, iqp_speeds, 'r-', linewidth=2,
                 label=f'IQP (max: {max(iqp_speeds):.1f} m/s)')

    if sp_waypoints:
        sp_s = [get_value(wp, 's_m') for wp in sp_waypoints]
        ax2.plot(sp_s, sp_speeds, 'g-', linewidth=2,
                 label=f'SP (max: {max(sp_speeds):.1f} m/s)')

    ax2.set_xlabel('Track Distance (m)')
    ax2.set_ylabel('Speed (m/s)')
    ax2.set_title('Speed Profiles Along Track')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_single_trajectory_with_boundaries(waypoints, trajectory_name, trajectory_color, trackbounds_markers=None, use_gradient=False):
    """Create a plot for a single trajectory with track boundaries."""
    x_coords, y_coords, speeds = extract_coordinates_and_speeds(waypoints)

    fig, ax = plt.subplots(figsize=(15, 12))

    # Plot track boundaries first if available
    if trackbounds_markers:
        left_bounds_x, left_bounds_y, right_bounds_x, right_bounds_y = extract_trackbounds_coordinates(
            trackbounds_markers)
        if left_bounds_x and left_bounds_y:
            ax.plot(left_bounds_x, left_bounds_y, 'black',
                    linewidth=2, alpha=0.8, label=f'Left Track Boundary ({len(left_bounds_x)} points)')
        if right_bounds_x and right_bounds_y:
            ax.plot(right_bounds_x, right_bounds_y, 'black',
                    linewidth=2, alpha=0.8, label=f'Right Track Boundary ({len(right_bounds_x)} points)')

    # Plot the trajectory line with optional gradient
    if use_gradient and len(x_coords) > 1:
        # Create gradient coloring from start (blue) to end (red)
        from matplotlib.collections import LineCollection
        import matplotlib.colors as mcolors

        # Create line segments
        points = np.array([x_coords, y_coords]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # Create gradient colors (blue to red)
        colors = np.linspace(0, 1, len(segments))

        # Create line collection with gradient
        lc = LineCollection(segments, cmap='coolwarm', linewidths=4, alpha=0.9)
        lc.set_array(colors)
        line = ax.add_collection(lc)

        # Add colorbar for gradient
        cbar = plt.colorbar(line, ax=ax, shrink=0.8, pad=0.02)
        cbar.set_label('Progress along trajectory (Start → End)',
                       rotation=270, labelpad=20)
        cbar.set_ticks([0, 0.5, 1])
        cbar.set_ticklabels(['Start', 'Middle', 'End'])

        # Add trajectory to legend
        ax.plot([], [], 'g-', linewidth=4, alpha=0.9,
                label=f'{trajectory_name} (avg: {np.mean(speeds):.1f} m/s)')
    else:
        # Standard single-color line
        ax.plot(x_coords, y_coords, trajectory_color, linewidth=3, alpha=0.9,
                label=f'{trajectory_name} (avg: {np.mean(speeds):.1f} m/s)')

    # Add start/finish markers with enhanced visibility
    ax.plot(x_coords[0], y_coords[0], 'go', markersize=12, markeredgecolor='black',
            markeredgewidth=2, label='Start', zorder=10)
    ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=12, markeredgecolor='black',
            markeredgewidth=2, label='End', zorder=10)

    # Add text labels for start and end
    ax.text(x_coords[0], y_coords[0], '  START', fontsize=12, ha='left',
            fontweight='bold', bbox=dict(boxstyle="round,pad=0.3", facecolor='lightgreen', alpha=0.7))
    ax.text(x_coords[-1], y_coords[-1], '  END', fontsize=12, ha='left',
            fontweight='bold', bbox=dict(boxstyle="round,pad=0.3", facecolor='lightcoral', alpha=0.7))

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title(
        f'Marina Race Track - {trajectory_name} with Track Boundaries')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    return fig


def plot_speed_heatmap(waypoints, trajectory_name, trackbounds_markers=None):
    """Create a speed heatmap for a single trajectory with optional track boundaries."""
    x_coords, y_coords, speeds = extract_coordinates_and_speeds(waypoints)

    fig, ax = plt.subplots(figsize=(12, 10))

    # Plot track boundaries first if available
    if trackbounds_markers:
        left_bounds_x, left_bounds_y, right_bounds_x, right_bounds_y = extract_trackbounds_coordinates(
            trackbounds_markers)
        if left_bounds_x and left_bounds_y:
            ax.plot(left_bounds_x, left_bounds_y, 'gray',
                    linewidth=2, alpha=0.6, label='Left Boundary')
        if right_bounds_x and right_bounds_y:
            ax.plot(right_bounds_x, right_bounds_y, 'gray',
                    linewidth=2, alpha=0.6, label='Right Boundary')

    # Create scatter plot with speed as color
    scatter = ax.scatter(x_coords, y_coords, c=speeds,
                         cmap='viridis', s=20, alpha=0.8)

    # Add colorbar
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('Speed (m/s)')

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title(f'{trajectory_name} - Speed Heatmap with Track Boundaries')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    if trackbounds_markers:
        ax.legend()

    return fig


def plot_acceleration_profiles(centerline_waypoints, iqp_waypoints, sp_waypoints):
    """Create acceleration profile comparison plot for all available trajectories."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 10))

    # Extract data for available trajectories
    cl_x, cl_y, cl_speeds, cl_accels, cl_s = extract_full_trajectory_data(
        centerline_waypoints)

    # Plot acceleration vs track distance
    ax1.plot(cl_s, cl_accels, 'b-', linewidth=2, alpha=0.8,
             label=f'Centerline (max: {max(cl_accels):.2f} m/s²)')

    if iqp_waypoints:
        iqp_x, iqp_y, iqp_speeds, iqp_accels, iqp_s = extract_full_trajectory_data(
            iqp_waypoints)
        ax1.plot(iqp_s, iqp_accels, 'r-', linewidth=2, alpha=0.8,
                 label=f'IQP - Racing Line (max: {max(iqp_accels):.2f} m/s²)')

    if sp_waypoints:
        sp_x, sp_y, sp_speeds, sp_accels, sp_s = extract_full_trajectory_data(
            sp_waypoints)
        ax1.plot(sp_s, sp_accels, 'g-', linewidth=2, alpha=0.8,
                 label=f'SP - Shortest Path (max: {max(sp_accels):.2f} m/s²)')

    # Add reference lines for typical acceleration limits
    ax1.axhline(y=3.0, color='orange', linestyle='--', linewidth=1.5, alpha=0.7,
                label='Car Limit (+3 m/s²)')
    ax1.axhline(y=-3.0, color='purple', linestyle='--', linewidth=1.5, alpha=0.7,
                label='Car Limit (-3 m/s²)')

    ax1.set_xlabel('Track Distance (m)')
    ax1.set_ylabel('Acceleration (m/s²)')
    ax1.set_title('Acceleration Profiles Along Track')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Plot speed vs acceleration (g-g diagram style)
    ax2.scatter(cl_speeds, cl_accels, c='blue',
                s=20, alpha=0.6, label='Centerline')

    if iqp_waypoints:
        ax2.scatter(iqp_speeds, iqp_accels, c='red', s=20,
                    alpha=0.6, label='IQP - Racing Line')

    if sp_waypoints:
        ax2.scatter(sp_speeds, sp_accels, c='green', s=20,
                    alpha=0.6, label='SP - Shortest Path')

    # Add reference lines
    ax2.axhline(y=3.0, color='orange', linestyle='--', linewidth=1.5, alpha=0.7,
                label='Car Limit (+3 m/s²)')
    ax2.axhline(y=-3.0, color='purple', linestyle='--', linewidth=1.5, alpha=0.7,
                label='Car Limit (-3 m/s²)')

    ax2.set_xlabel('Speed (m/s)')
    ax2.set_ylabel('Acceleration (m/s²)')
    ax2.set_title('Speed vs Acceleration (g-g Diagram Style)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_track_boundaries_only(trackbounds_markers):
    """Create a plot showing only the track boundaries for detailed view."""
    left_bounds_x, left_bounds_y, right_bounds_x, right_bounds_y = extract_trackbounds_coordinates(
        trackbounds_markers)

    fig, ax = plt.subplots(figsize=(15, 12))

    if left_bounds_x and left_bounds_y:
        ax.plot(left_bounds_x, left_bounds_y, 'red', linewidth=2, alpha=0.8,
                label=f'Left Track Boundary ({len(left_bounds_x)} points)')
    if right_bounds_x and right_bounds_y:
        ax.plot(right_bounds_x, right_bounds_y, 'blue', linewidth=2, alpha=0.8,
                label=f'Right Track Boundary ({len(right_bounds_x)} points)')

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Marina Track Boundaries')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    return fig


def find_available_maps():
    """Find all available maps with global_waypoints.json files."""
    maps = []

    # Search in tam/maps/output directory
    tam_output_dir = "/home/atlas/catkin_ws/src/race_stack/tam_to_eth_map_parser/maps/output"
    if os.path.exists(tam_output_dir):
        for map_dir in os.listdir(tam_output_dir):
            map_path = os.path.join(tam_output_dir, map_dir)
            if os.path.isdir(map_path):
                json_file = os.path.join(map_path, "global_waypoints.json")
                if os.path.exists(json_file):
                    maps.append({
                        'name': map_dir,
                        'path': json_file,
                        'source': 'tam/output'
                    })

    # Search in stack_master/maps directory
    stack_maps_dir = "/home/atlas/catkin_ws/src/race_stack/stack_master/maps"
    if os.path.exists(stack_maps_dir):
        for map_dir in os.listdir(stack_maps_dir):
            map_path = os.path.join(stack_maps_dir, map_dir)
            if os.path.isdir(map_path):
                json_file = os.path.join(map_path, "global_waypoints.json")
                if os.path.exists(json_file):
                    maps.append({
                        'name': map_dir,
                        'path': json_file,
                        'source': 'stack_master'
                    })

    return maps


def select_map_interactive(available_maps):
    """Let user select which map to plot."""
    if not available_maps:
        print("No maps with global_waypoints.json found!")
        return None

    if len(available_maps) == 1:
        selected = available_maps[0]
        print(f"Auto-selecting only available map: {selected['name']}")
        return selected

    print("\nAvailable maps:")
    for i, map_info in enumerate(available_maps):
        print(f"  {i+1}. {map_info['name']} (from {map_info['source']})")

    while True:
        try:
            choice = input(
                f"\nSelect map to plot (1-{len(available_maps)}) or 'a' for all: ").strip()

            if choice.lower() == 'a':
                return 'all'

            choice_idx = int(choice) - 1
            if 0 <= choice_idx < len(available_maps):
                return available_maps[choice_idx]
            else:
                print(
                    f"Please enter a number between 1 and {len(available_maps)}")
        except ValueError:
            print("Please enter a valid number or 'a'")


def plot_map(map_info, plots_dir):
    """Plot a single map."""
    print(f"\n=== Plotting map: {map_info['name']} ===")
    print(f"Using waypoints file: {map_info['path']}")

    try:
        centerline_waypoints, iqp_waypoints, sp_waypoints, trackbounds_markers = load_waypoints_data(
            map_info['path'])

        # Report what trajectories are available
        print(f"Loaded {len(centerline_waypoints)} centerline waypoints")
        print(f"Loaded {len(iqp_waypoints)} IQP waypoints")
        print(f"Loaded {len(sp_waypoints)} SP waypoints")
        print(f"Loaded {len(trackbounds_markers)} trackbounds markers")

        if not centerline_waypoints:
            print("Error: No centerline waypoints found!")
            return False

        # Clean map name for file naming
        clean_name = map_info['name'].replace(
            '%', '_percent_').replace('/', '_')

        # Plot all trajectories with track boundaries
        print("Creating trajectory comparison plot with track boundaries...")
        fig1 = plot_trajectories(
            centerline_waypoints, iqp_waypoints, sp_waypoints, trackbounds_markers)

        # Save the plot
        output_path = os.path.join(plots_dir, f"{clean_name}_trajectories.png")
        fig1.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Saved trajectory plot: {output_path}")

        # Create individual trajectory plots with track boundaries
        print("Creating individual trajectory plots with track boundaries...")

        # Centerline trajectory plot
        print("Creating centerline trajectory plot...")
        fig_centerline = plot_single_trajectory_with_boundaries(
            centerline_waypoints, "Centerline", 'b-', trackbounds_markers)
        centerline_path = os.path.join(
            plots_dir, f"{clean_name}_centerline.png")
        fig_centerline.savefig(centerline_path, dpi=300, bbox_inches='tight')
        print(f"Saved centerline plot: {centerline_path}")

        # IQP trajectory plot (only if available)
        if iqp_waypoints:
            print("Creating IQP trajectory plot...")
            fig_iqp = plot_single_trajectory_with_boundaries(
                iqp_waypoints, "IQP (Aggressive)", 'r-', trackbounds_markers)
            iqp_path = os.path.join(plots_dir, f"{clean_name}_iqp.png")
            fig_iqp.savefig(iqp_path, dpi=300, bbox_inches='tight')
            print(f"Saved IQP plot: {iqp_path}")
        else:
            print("Skipping IQP trajectory plot (no IQP waypoints available)")

        # SP trajectory plot (only if available)
        if sp_waypoints:
            print("Creating SP trajectory plot with gradient coloring...")
            fig_sp = plot_single_trajectory_with_boundaries(
                sp_waypoints, "SP (Moderate)", 'g-', trackbounds_markers, use_gradient=True)
            sp_path = os.path.join(plots_dir, f"{clean_name}_sp.png")
            fig_sp.savefig(sp_path, dpi=300, bbox_inches='tight')
            print(f"Saved SP plot with gradient: {sp_path}")
        else:
            print("Skipping SP trajectory plot (no SP waypoints available)")

        # Create speed heatmaps for available trajectories with track boundaries
        print("Creating speed heatmaps with track boundaries...")

        # IQP heatmap (only if available)
        if iqp_waypoints:
            fig2 = plot_speed_heatmap(
                iqp_waypoints, "IQP (Aggressive)", trackbounds_markers)
            iqp_heatmap_path = os.path.join(
                plots_dir, f"{clean_name}_iqp_heatmap.png")
            fig2.savefig(iqp_heatmap_path, dpi=300, bbox_inches='tight')
            print(f"Saved IQP heatmap: {iqp_heatmap_path}")
        else:
            print("Skipping IQP heatmap (no IQP waypoints available)")

        # SP heatmap (only if available)
        if sp_waypoints:
            fig3 = plot_speed_heatmap(
                sp_waypoints, "SP (Moderate)", trackbounds_markers)
            sp_heatmap_path = os.path.join(
                plots_dir, f"{clean_name}_sp_heatmap.png")
            fig3.savefig(sp_heatmap_path, dpi=300, bbox_inches='tight')
            print(f"Saved SP heatmap: {sp_heatmap_path}")
        else:
            print("Skipping SP heatmap (no SP waypoints available)")

        # Create track boundaries only plot
        print("Creating track boundaries plot...")
        fig4 = plot_track_boundaries_only(trackbounds_markers)
        boundaries_path = os.path.join(
            plots_dir, f"{clean_name}_track_boundaries.png")
        fig4.savefig(boundaries_path, dpi=300, bbox_inches='tight')
        print(f"Saved track boundaries plot: {boundaries_path}")

        # Create acceleration profile plot
        print("Creating acceleration profile plot...")
        fig_accel = plot_acceleration_profiles(
            centerline_waypoints, iqp_waypoints, sp_waypoints)
        accel_path = os.path.join(
            plots_dir, f"{clean_name}_acceleration_profiles.png")
        fig_accel.savefig(accel_path, dpi=300, bbox_inches='tight')
        print(f"Saved acceleration profiles plot: {accel_path}")

        # Show statistics for available trajectories
        print(f"\n=== {map_info['name']} Statistics ===")
        left_bounds_x, left_bounds_y, right_bounds_x, right_bounds_y = extract_trackbounds_coordinates(
            trackbounds_markers)
        print(
            f"Track Boundaries - Left: {len(left_bounds_x)} points, Right: {len(right_bounds_x)} points")

        # Helper function to extract values
        def get_value(wp, key):
            val = wp[key]
            return val['data'] if isinstance(val, dict) and 'data' in val else val

        cl_speeds = [get_value(wp, 'vx_mps') for wp in centerline_waypoints]
        cl_accels = [get_value(wp, 'ax_mps2') for wp in centerline_waypoints]
        print(
            f"Centerline - Speed: Min {min(cl_speeds):.2f} m/s, Max {max(cl_speeds):.2f} m/s, Avg {np.mean(cl_speeds):.2f} m/s")
        print(
            f"             Accel: Min {min(cl_accels):.2f} m/s², Max {max(cl_accels):.2f} m/s², Avg {np.mean(cl_accels):.2f} m/s²")

        if iqp_waypoints:
            iqp_speeds = [get_value(wp, 'vx_mps') for wp in iqp_waypoints]
            iqp_accels = [get_value(wp, 'ax_mps2') for wp in iqp_waypoints]
            print(
                f"IQP        - Speed: Min {min(iqp_speeds):.2f} m/s, Max {max(iqp_speeds):.2f} m/s, Avg {np.mean(iqp_speeds):.2f} m/s")
            print(
                f"             Accel: Min {min(iqp_accels):.2f} m/s², Max {max(iqp_accels):.2f} m/s², Avg {np.mean(iqp_accels):.2f} m/s²")
        else:
            print("IQP        - Not available")

        if sp_waypoints:
            sp_speeds = [get_value(wp, 'vx_mps') for wp in sp_waypoints]
            sp_accels = [get_value(wp, 'ax_mps2') for wp in sp_waypoints]
            print(
                f"SP         - Speed: Min {min(sp_speeds):.2f} m/s, Max {max(sp_speeds):.2f} m/s, Avg {np.mean(sp_speeds):.2f} m/s")
            print(
                f"             Accel: Min {min(sp_accels):.2f} m/s², Max {max(sp_accels):.2f} m/s², Avg {np.mean(sp_accels):.2f} m/s²")
        else:
            print("SP         - Not available")

        # Track length (use available trajectory with most waypoints)
        if iqp_waypoints:
            track_length = max([get_value(wp, 's_m') for wp in iqp_waypoints])
        else:
            track_length = max([get_value(wp, 's_m')
                               for wp in centerline_waypoints])
        print(f"\nTrack length: {track_length:.2f} m")

        # Estimated lap times (rough calculation) for available trajectories
        def estimate_lap_time(waypoints):
            if not waypoints:
                return 0
            total_time = 0
            for i in range(len(waypoints)-1):
                wp1 = waypoints[i]
                wp2 = waypoints[i+1]
                x1 = get_value(wp1, 'x_m')
                y1 = get_value(wp1, 'y_m')
                x2 = get_value(wp2, 'x_m')
                y2 = get_value(wp2, 'y_m')
                v1 = get_value(wp1, 'vx_mps')
                v2 = get_value(wp2, 'vx_mps')
                distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                avg_speed = (v1 + v2) / 2
                if avg_speed > 0:
                    total_time += distance / avg_speed
            return total_time

        print(f"\nEstimated lap times:")
        cl_time = estimate_lap_time(centerline_waypoints)
        print(f"Centerline: {cl_time:.2f} seconds")

        if iqp_waypoints:
            iqp_time = estimate_lap_time(iqp_waypoints)
            print(f"IQP:        {iqp_time:.2f} seconds")
        else:
            print(f"IQP:        Not available")

        if sp_waypoints:
            sp_time = estimate_lap_time(sp_waypoints)
            print(f"SP:         {sp_time:.2f} seconds")
        else:
            print(f"SP:         Not available")

        return True

    except Exception as e:
        print(f"Error plotting {map_info['name']}: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main plotting function."""
    print("=== Marina Race Track Plotting Tool ===")

    # Ensure plots directory exists
    plots_dir = "/home/atlas/catkin_ws/src/race_stack/tam_to_eth_map_parser/plots"
    os.makedirs(plots_dir, exist_ok=True)

    # Find available maps
    available_maps = find_available_maps()

    if not available_maps:
        print("No maps found! Make sure you've run the Marina map parser first.")
        print("Looking for global_waypoints.json files in:")
        print("  - /home/atlas/catkin_ws/src/race_stack/tam/maps/output/")
        print("  - /home/atlas/catkin_ws/src/race_stack/stack_master/maps/")
        return

    # Select map(s) to plot
    selection = select_map_interactive(available_maps)

    if selection is None:
        return

    plt.ion()  # Enable interactive mode

    if selection == 'all':
        print(f"\nPlotting all {len(available_maps)} available maps...")
        success_count = 0
        for map_info in available_maps:
            if plot_map(map_info, plots_dir):
                success_count += 1
        print(f"\n=== Summary ===")
        print(
            f"Successfully plotted {success_count}/{len(available_maps)} maps")
    else:
        # Plot single selected map
        if plot_map(selection, plots_dir):
            print(f"\nSuccessfully plotted {selection['name']}")
            # Display plots for single map
            plt.show()
        else:
            print(f"Failed to plot {selection['name']}")

    print(f"\nAll plots saved to: {plots_dir}")

    print(f"\nAll plots saved to: {plots_dir}")


if __name__ == "__main__":
    main()
