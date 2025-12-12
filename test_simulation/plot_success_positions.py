#!/usr/bin/env python3
"""
Success Position Plotter

This script plots successful race completion positions (overtakes, race wins)
on a map showing the track with the raceline. Each planner is clearly distinguishable.

Output: High-quality PNG and PDF plots suitable for thesis publication.

Usage:
    python3 plot_success_positions.py [--mode MODE] [--output OUTPUT_DIR]
    
Example:
    python3 plot_success_positions.py --mode single_car_with_obstacle
    python3 plot_success_positions.py --mode multi_car
    python3 plot_success_positions.py --mode combined
"""

from frenet_converter.frenet_converter import FrenetConverter
import os
import sys
import csv
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from matplotlib.collections import LineCollection
from pathlib import Path
from typing import List, Dict, Tuple
from dataclasses import dataclass

# Add the frenet_converter to the path
sys.path.insert(0, str(Path(__file__).parent.parent /
                       "f110_utils/libs/frenet_conversion/src"))


# Set thesis-quality plot style
plt.rcParams.update({
    'font.size': 14,
    'axes.titlesize': 18,
    'axes.labelsize': 16,
    'xtick.labelsize': 14,
    'ytick.labelsize': 14,
    'legend.fontsize': 12,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'font.family': 'DejaVu Sans',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'axes.axisbelow': True,
    'figure.facecolor': 'white',
    'axes.facecolor': 'white',
    'axes.edgecolor': 'black',
    'axes.linewidth': 1.0,
})


@dataclass
class RaceRecord:
    """Data class for a single race record"""
    mode: str
    map_name: str
    planner: str
    planner_car2: str
    performance_factor: float
    race_end: str
    race_end_timestamp: float
    car_s: float
    car_d: float
    car_lap: int
    details: str
    simulation_id: str
    batch_id: str
    duration_s: float


# Planner colors - using distinct, colorblind-friendly academic palette
PLANNER_STYLES = {
    'spliner': {
        'color': '#1f77b4',  # Strong blue
        'name': 'Spliner',
        'order': 1
    },
    'tam_sampling': {
        'color': '#d62728',  # Deep red
        'name': 'Sampling-based',
        'order': 2
    },
    'predictive_spliner': {
        'color': '#2ca02c',  # Forest green
        'name': 'Predictive Spliner',
        'order': 3
    },
    'predictive_sampler': {
        'color': '#ff7f0e',  # Bright orange
        'name': 'Predictive Sampler',
        'order': 4
    },
}

# Success type markers - professional academic style
SUCCESS_MARKERS = {
    'success': {
        'marker': 'o',  # Circle - represents completion/success
        'size': 60,
        'edgecolor': 'black',
        'linewidth': 1.0,
        'label': 'Successful Overtake'
    },
    'finished': {
        'marker': 'D',  # Diamond - represents race finish
        'size': 70,
        'edgecolor': 'black',
        'linewidth': 1.0,
        'label': 'Race Finished'
    },
}

# Map configurations
MAP_CONFIGS = {
    'f': {
        'name': 'F Track',
        'short_name': 'F',
        'waypoints_path': 'src/race_stack/stack_master/maps/f_100%s_100%w_NUC2_mintime/global_waypoints.json',
        'figsize': (12, 10),
    },
    'marina': {
        'name': 'Yas Marina',
        'short_name': 'Marina',
        'waypoints_path': 'src/race_stack/stack_master/maps/my_map_20%s_100%w_NUC2_mintime/global_waypoints.json',
        'figsize': (14, 10),
    }
}


def load_waypoints(waypoints_path: Path) -> dict:
    """Load waypoints from JSON file"""
    with open(waypoints_path, 'r') as f:
        data = json.load(f)
    return data


def extract_raceline(waypoints_data: dict) -> tuple:
    """Extract raceline coordinates from waypoints data"""
    wpnts = waypoints_data['global_traj_wpnts_iqp']['wpnts']
    x = np.array([w['x_m'] for w in wpnts])
    y = np.array([w['y_m'] for w in wpnts])
    s = np.array([w['s_m'] for w in wpnts])
    d_left = np.array([w['d_left'] for w in wpnts])
    d_right = np.array([w['d_right'] for w in wpnts])
    psi = np.array([w['psi_rad'] for w in wpnts])
    return x, y, s, d_left, d_right, psi


def extract_trackbounds(waypoints_data: dict) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Extract track boundary coordinates from waypoints data"""
    trackbounds_markers = waypoints_data.get(
        'trackbounds_markers', {}).get('markers', [])

    left_x, left_y = [], []
    right_x, right_y = [], []

    for marker in trackbounds_markers:
        try:
            if (isinstance(marker, dict) and
                'ns' in marker and
                marker.get('ns') in ['trackbounds_left', 'trackbounds_right'] and
                'pose' in marker and
                isinstance(marker['pose'], dict) and
                    'position' in marker['pose']):

                x = float(marker['pose']['position']['x'])
                y = float(marker['pose']['position']['y'])

                if marker['ns'] == 'trackbounds_left':
                    left_x.append(x)
                    left_y.append(y)
                elif marker['ns'] == 'trackbounds_right':
                    right_x.append(x)
                    right_y.append(y)
        except (KeyError, TypeError, ValueError):
            continue

    return np.array(left_x), np.array(left_y), np.array(right_x), np.array(right_y)


def compute_track_boundaries_from_raceline(x: np.ndarray, y: np.ndarray, psi: np.ndarray,
                                           d_left: np.ndarray, d_right: np.ndarray) -> tuple:
    """Compute left and right track boundaries from raceline data"""
    # Left boundary (positive d direction)
    left_x = x + d_left * np.cos(psi + np.pi / 2)
    left_y = y + d_left * np.sin(psi + np.pi / 2)

    # Right boundary (negative d direction)
    right_x = x - d_right * np.cos(psi + np.pi / 2)
    right_y = y - d_right * np.sin(psi + np.pi / 2)

    return (left_x, left_y), (right_x, right_y)


def frenet_to_cartesian(converter: FrenetConverter, s: float, d: float) -> tuple:
    """Convert Frenet coordinates to Cartesian"""
    s = s % converter.raceline_length
    coords = converter.get_cartesian(s, d)
    return coords[0], coords[1]


def load_race_data(csv_path: Path) -> List[RaceRecord]:
    """Load race summary data from CSV"""
    records = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                record = RaceRecord(
                    mode=row['mode'],
                    map_name=row['map'],
                    planner=row['planner'],
                    planner_car2=row.get('planner_car2', ''),
                    performance_factor=float(row['performance_factor']),
                    race_end=row['race_end'],
                    race_end_timestamp=float(row['race_end_timestamp']),
                    car_s=float(row['car_s']),
                    car_d=float(row['car_d']),
                    car_lap=int(row['car_lap']),
                    details=row['details'],
                    simulation_id=row['simulation_id'],
                    batch_id=row['batch_id'],
                    duration_s=float(row['duration_s'])
                )
                records.append(record)
            except (ValueError, KeyError) as e:
                continue
    return records


def filter_records(records: List[RaceRecord], map_name: str = None,
                   mode: str = None, race_ends: List[str] = None) -> List[RaceRecord]:
    """Filter records by criteria"""
    result = records
    if map_name:
        result = [r for r in result if r.map_name == map_name]
    if mode:
        result = [r for r in result if r.mode == mode]
    if race_ends:
        result = [r for r in result if r.race_end in race_ends]
    return result


def save_figure(fig, output_dir: Path, filename: str):
    """Save figure in both PNG and high-quality PDF formats"""
    # Save PNG
    png_path = output_dir / f"{filename}.png"
    fig.savefig(png_path, dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print(f"   📊 Saved: {png_path}")

    # Save high-quality PDF for thesis
    pdf_path = output_dir / f"{filename}.pdf"
    fig.savefig(pdf_path, format='pdf', dpi=600, bbox_inches='tight',
                facecolor='white', edgecolor='none', backend='pdf')
    print(f"   📄 Saved: {pdf_path}")


def plot_track_base(ax, waypoints_data: dict, map_name: str):
    """Plot the base track layout (boundaries, raceline, start/finish)"""
    # Extract raceline data
    x, y, s, d_left, d_right, psi = extract_raceline(waypoints_data)

    # Try to get track boundaries from markers first
    left_x, left_y, right_x, right_y = extract_trackbounds(waypoints_data)

    # If no markers, compute from raceline
    if len(left_x) == 0:
        (left_x, left_y), (right_x, right_y) = compute_track_boundaries_from_raceline(
            x, y, psi, d_left, d_right)

    # Plot track boundaries
    ax.plot(left_x, left_y, 'k-', linewidth=2.5, alpha=0.9, zorder=1)
    ax.plot(right_x, right_y, 'k-', linewidth=2.5, alpha=0.9, zorder=1)

    # Fill track surface
    track_x = np.concatenate([left_x, right_x[::-1], [left_x[0]]])
    track_y = np.concatenate([left_y, right_y[::-1], [left_y[0]]])
    ax.fill(track_x, track_y, color='#e8e8e8', alpha=0.6, zorder=0)

    # Plot raceline
    ax.plot(x, y, '--', color='#666666', linewidth=1.5, alpha=0.7, zorder=2)

    # Plot start/finish marker
    ax.plot(x[0], y[0], 'o', markersize=14, markerfacecolor='white',
            markeredgecolor='black', markeredgewidth=2.5, zorder=10)
    ax.annotate('Start', (x[0], y[0]), textcoords='offset points',
                xytext=(10, 10), fontsize=12, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                          edgecolor='black', alpha=0.9))

    # Create converter for Frenet->Cartesian
    converter = FrenetConverter(x, y, psi)

    return converter


def plot_success_positions(ax, successes: List[RaceRecord], converter: FrenetConverter,
                           success_type: str):
    """Plot success positions for each planner"""
    stats = {}

    for planner, style in sorted(PLANNER_STYLES.items(), key=lambda x: x[1]['order']):
        planner_successes = [r for r in successes if r.planner ==
                             planner and r.race_end == success_type]

        if len(planner_successes) > 0:
            cart_coords = []
            for record in planner_successes:
                try:
                    cx, cy = frenet_to_cartesian(
                        converter, record.car_s, record.car_d)
                    cart_coords.append((cx, cy))
                except Exception:
                    pass

            if cart_coords:
                cx_arr = np.array([c[0] for c in cart_coords])
                cy_arr = np.array([c[1] for c in cart_coords])

                marker_style = SUCCESS_MARKERS[success_type]
                ax.scatter(cx_arr, cy_arr,
                           c=style['color'],
                           marker=marker_style['marker'],
                           s=marker_style['size'],
                           edgecolors=marker_style['edgecolor'],
                           linewidths=marker_style['linewidth'],
                           alpha=0.75,
                           zorder=5)

                stats[planner] = len(cart_coords)

    return stats


def create_legend_elements(include_finished: bool = True):
    """Create legend elements for planners and success types"""
    elements = []

    # Add planners
    for planner, style in sorted(PLANNER_STYLES.items(), key=lambda x: x[1]['order']):
        elements.append(
            Patch(facecolor=style['color'], edgecolor='black',
                  linewidth=1, label=style['name'])
        )

    # Add spacer
    elements.append(Patch(facecolor='white', edgecolor='white', label=''))

    # Add success types
    for success_type, marker_style in SUCCESS_MARKERS.items():
        if success_type == 'finished' and not include_finished:
            continue
        elements.append(
            Line2D([0], [0], marker=marker_style['marker'], color='w',
                   markerfacecolor='gray', markersize=10,
                   markeredgecolor='black', markeredgewidth=1,
                   label=marker_style['label'])
        )

    return elements


def plot_single_map_mode(map_name: str, waypoints_data: dict, successes: List[RaceRecord],
                         mode: str, output_dir: Path, show_plot: bool = True):
    """Create a single plot for one map and mode"""
    config = MAP_CONFIGS[map_name]

    # Use consistent figure size for all maps (includes space for legend on right)
    fig_width = 16
    fig_height = 10

    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    # Plot track base
    converter = plot_track_base(ax, waypoints_data, map_name)

    # Plot successes
    success_stats = plot_success_positions(ax, successes, converter, 'success')
    finished_stats = plot_success_positions(
        ax, successes, converter, 'finished')

    # Title and labels
    mode_display = 'Single-Car With Obstacle' if mode == 'single_car_with_obstacle' else 'Multi-Car'
    ax.set_title(f"{config['name']} - {mode_display}\nSuccessful Race Completion Positions",
                 fontsize=16, fontweight='bold', pad=15)
    ax.set_xlabel('X Position (m)', fontsize=14)
    ax.set_ylabel('Y Position (m)', fontsize=14)
    ax.set_aspect('equal')

    # Statistics box - place in corner with proper spacing from plot boundary
    total_success = sum(success_stats.values())
    total_finished = sum(finished_stats.values())
    stats_text = f"Overtakes: {total_success}\nFinished: {total_finished}\nTotal: {total_success + total_finished}"

    props = dict(boxstyle='round,pad=0.5', facecolor='lightgreen',
                 edgecolor='black', alpha=0.9)
    ax.text(0.04, 0.04, stats_text, transform=ax.transAxes, fontsize=12,
            verticalalignment='bottom', horizontalalignment='left', bbox=props,
            fontfamily='monospace')

    # Create legend outside plot area on the right
    include_finished = total_finished > 0
    legend_elements = create_legend_elements(include_finished=include_finished)
    ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.02, 0.5),
              fontsize=11, framealpha=0.95, edgecolor='black', fancybox=False)

    plt.tight_layout(rect=[0, 0, 0.85, 1])  # Leave space on right for legend

    # Save figures
    filename = f"success_positions_{map_name}_{mode}"
    save_figure(fig, output_dir, filename)

    if show_plot:
        plt.show()
    else:
        plt.close()

    return success_stats, finished_stats


def plot_combined_map(map_name: str, waypoints_data: dict,
                      scwo_records: List[RaceRecord], multi_records: List[RaceRecord],
                      output_dir: Path, show_plot: bool = True):
    """Create a combined plot with both modes side by side"""
    config = MAP_CONFIGS[map_name]

    # Use consistent figure size for all maps
    fig_width = 22
    fig_height = 10

    fig, axes = plt.subplots(1, 2, figsize=(fig_width, fig_height))

    datasets = [
        (scwo_records, 'Single-Car With Obstacle', axes[0]),
        (multi_records, 'Multi-Car', axes[1])
    ]

    all_stats = []
    has_finished = False

    for records, mode_title, ax in datasets:
        # Filter for this map and success types
        map_successes = [r for r in records if r.map_name == map_name
                         and r.race_end in ['success', 'finished']]

        # Plot track base
        converter = plot_track_base(ax, waypoints_data, map_name)

        # Plot successes
        success_stats = plot_success_positions(
            ax, map_successes, converter, 'success')
        finished_stats = plot_success_positions(
            ax, map_successes, converter, 'finished')

        all_stats.append((success_stats, finished_stats))
        if sum(finished_stats.values()) > 0:
            has_finished = True

        # Title and labels
        ax.set_title(mode_title, fontsize=16, fontweight='bold', pad=10)
        ax.set_xlabel('X Position (m)', fontsize=14)
        ax.set_ylabel('Y Position (m)', fontsize=14)
        ax.set_aspect('equal')

        # Statistics box - with proper spacing from plot boundary
        total_success = sum(success_stats.values())
        total_finished = sum(finished_stats.values())
        stats_text = f"Overtakes: {total_success}\nFinished: {total_finished}"

        props = dict(boxstyle='round,pad=0.4', facecolor='lightgreen',
                     edgecolor='black', alpha=0.9)
        ax.text(0.04, 0.96, stats_text, transform=ax.transAxes, fontsize=11,
                verticalalignment='top', horizontalalignment='left', bbox=props,
                fontfamily='monospace')

    # Main title
    fig.suptitle(f"{config['name']} - Success Position Analysis",
                 fontsize=18, fontweight='bold', y=0.98)

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.10, top=0.92)

    # Create shared legend at bottom (after tight_layout)
    legend_elements = create_legend_elements(include_finished=has_finished)
    fig.legend(handles=legend_elements, loc='upper center', ncol=6, fontsize=12,
               bbox_to_anchor=(0.5, 0.04), framealpha=0.95, edgecolor='black',
               fancybox=False)

    # Save figures
    filename = f"success_positions_{map_name}_combined"
    save_figure(fig, output_dir, filename)

    if show_plot:
        plt.show()
    else:
        plt.close()

    return all_stats


def plot_all_successes_single_figure(records: List[RaceRecord], waypoints_data_f: dict,
                                     waypoints_data_marina: dict, output_dir: Path,
                                     show_plot: bool = True):
    """Create a 2x2 grid showing all maps and modes"""
    fig, axes = plt.subplots(2, 2, figsize=(20, 18))

    configurations = [
        ('f', 'single_car_with_obstacle', waypoints_data_f,
         'F Track - Single-Car', axes[0, 0]),
        ('f', 'multi_car', waypoints_data_f,
         'F Track - Multi-Car', axes[0, 1]),
        ('marina', 'single_car_with_obstacle',
         waypoints_data_marina, 'Marina - Single-Car', axes[1, 0]),
        ('marina', 'multi_car', waypoints_data_marina,
         'Marina - Multi-Car', axes[1, 1]),
    ]

    has_finished = False

    for map_name, mode, waypoints_data, title, ax in configurations:
        # Filter records
        map_successes = [r for r in records if r.map_name == map_name
                         and r.mode == mode and r.race_end in ['success', 'finished']]

        # Plot track base
        converter = plot_track_base(ax, waypoints_data, map_name)

        # Plot successes
        success_stats = plot_success_positions(
            ax, map_successes, converter, 'success')
        finished_stats = plot_success_positions(
            ax, map_successes, converter, 'finished')

        if sum(finished_stats.values()) > 0:
            has_finished = True

        # Title and labels
        ax.set_title(title, fontsize=14, fontweight='bold', pad=8)
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_aspect('equal')

        # Compact stats - with proper spacing from plot boundary
        total_success = sum(success_stats.values())
        total_finished = sum(finished_stats.values())
        ax.text(0.04, 0.96, f"Ovt: {total_success} | Fin: {total_finished}",
                transform=ax.transAxes, fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen',
                          edgecolor='black', alpha=0.9))

    fig.suptitle('Success Position Analysis - All Scenarios',
                 fontsize=18, fontweight='bold', y=0.98)

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.08, top=0.94, hspace=0.15, wspace=0.1)

    # Shared legend at bottom (after layout adjustments)
    legend_elements = create_legend_elements(include_finished=has_finished)
    fig.legend(handles=legend_elements, loc='upper center', ncol=6, fontsize=12,
               bbox_to_anchor=(0.5, 0.03), framealpha=0.95, edgecolor='black',
               fancybox=False)

    # Save figures
    filename = "success_positions_all_scenarios"
    save_figure(fig, output_dir, filename)

    if show_plot:
        plt.show()
    else:
        plt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Plot success positions on track maps")
    parser.add_argument("--mode", "-m",
                        choices=['single_car_with_obstacle',
                                 'multi_car', 'both', 'combined', 'all'],
                        default='combined',
                        help="Which mode to plot (combined=side-by-side, all=2x2 grid)")
    parser.add_argument("--map", choices=['f', 'marina', 'both'], default='both',
                        help="Which map to plot")
    parser.add_argument("--output", "-o", default="plots",
                        help="Output directory for plots")
    parser.add_argument("--csv", "-c", default=None,
                        help="Path to race_summary.csv")
    parser.add_argument("--no-show", action="store_true",
                        help="Don't display plots, just save")

    args = parser.parse_args()

    # Resolve paths
    script_dir = Path(__file__).parent
    workspace_root = script_dir.parent.parent.parent

    # Find CSV file
    if args.csv:
        csv_path = Path(args.csv)
    else:
        csv_path = script_dir / "logs" / "race_summary.csv"

    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}")
        return 1

    # Create output directory
    output_dir = script_dir / args.output
    output_dir.mkdir(exist_ok=True)

    # Load race data
    print(f"📂 Loading race data from: {csv_path}")
    records = load_race_data(csv_path)
    print(f"   Total records: {len(records)}")

    # Count success types
    success_count = len([r for r in records if r.race_end == 'success'])
    finished_count = len([r for r in records if r.race_end == 'finished'])
    print(f"   Successful overtakes: {success_count}")
    print(f"   Finished races: {finished_count}")

    # Determine which maps to process
    maps_to_process = ['f', 'marina'] if args.map == 'both' else [args.map]

    # Load waypoints for all needed maps
    waypoints_cache = {}
    for map_name in maps_to_process:
        waypoints_path = workspace_root / \
            MAP_CONFIGS[map_name]['waypoints_path']
        if waypoints_path.exists():
            waypoints_cache[map_name] = load_waypoints(waypoints_path)
            print(f"   Loaded waypoints for {MAP_CONFIGS[map_name]['name']}")
        else:
            print(f"   Warning: Waypoints file not found: {waypoints_path}")

    # Generate plots based on mode
    if args.mode == 'all' and len(maps_to_process) == 2:
        print("\n🎨 Creating comprehensive 2x2 success analysis plot...")
        plot_all_successes_single_figure(
            records,
            waypoints_cache.get('f'),
            waypoints_cache.get('marina'),
            output_dir,
            show_plot=not args.no_show
        )
    else:
        for map_name in maps_to_process:
            if map_name not in waypoints_cache:
                continue

            print(f"\n🗺️  Processing {MAP_CONFIGS[map_name]['name']}...")
            waypoints_data = waypoints_cache[map_name]

            if args.mode == 'combined':
                scwo_records = filter_records(
                    records, mode='single_car_with_obstacle')
                multi_records = filter_records(records, mode='multi_car')
                plot_combined_map(map_name, waypoints_data, scwo_records, multi_records,
                                  output_dir, show_plot=not args.no_show)
            else:
                modes_to_process = ['single_car_with_obstacle',
                                    'multi_car'] if args.mode == 'both' else [args.mode]

                for mode in modes_to_process:
                    successes = filter_records(records, map_name=map_name, mode=mode,
                                               race_ends=['success', 'finished'])

                    if len(successes) == 0:
                        print(
                            f"   No successes found for {mode} on {map_name}")
                        continue

                    print(f"   Plotting {mode}: {len(successes)} successes")
                    plot_single_map_mode(map_name, waypoints_data, successes,
                                         mode, output_dir, show_plot=not args.no_show)

    print(f"\n✅ All plots saved to: {output_dir}")
    return 0


if __name__ == "__main__":
    exit(main())
