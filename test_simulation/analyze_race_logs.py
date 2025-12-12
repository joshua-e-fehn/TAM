#!/usr/bin/env python3
"""
Race Log Analyzer

This script analyzes race simulation logs from multiple test setups and generates
a consolidated summary CSV file.

Usage:
    python3 analyze_race_logs.py [--output OUTPUT_FILE] [--logs-dir LOGS_DIR]
    
Example:
    python3 analyze_race_logs.py --output race_summary.csv
"""

import os
import re
import csv
import yaml
import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum


class RaceEnd(Enum):
    """Categories for race ending reasons"""
    SUCCESS = "success"           # Overtake succeeded
    COLLISION = "collision"       # Collision with obstacle or car2
    TRACKBOUNDS = "trackbounds"   # Track boundary collision
    DEPLETION = "depletion"       # Max laps reached
    TIMEOUT = "timeout"           # Simulation timeout
    UNKNOWN = "unknown"           # Unknown/unclear reason


@dataclass
class SimulationResult:
    """Data class holding all information for a single simulation run"""
    mode: str = ""
    map_name: str = ""
    planner: str = ""
    planner_car2: str = ""  # For multi-car mode
    performance_factor: float = 0.0
    race_end: str = ""
    race_end_timestamp: float = 0.0  # Timestamp of the event that ended the race
    car_s: float = 0.0
    car_d: float = 0.0
    car_lap: int = 0
    details: str = ""
    simulation_id: str = ""
    batch_id: str = ""
    timestamp: float = 0.0
    csv_file_path: str = ""  # Path to the source CSV file


def extract_map_short_name(global_map: str) -> str:
    """Extract short map name from global_map string"""
    if not global_map:
        return "unknown"

    # Handle patterns like "f_100%s_100%w_NUC2_mintime" or "my_map_20%s_100%w_NUC2_mintime"
    if global_map.startswith("f_"):
        return "f"
    elif "my_map" in global_map.lower() or "marina" in global_map.lower():
        return "marina"
    else:
        # Return first part before underscore
        return global_map.split("_")[0]


def classify_race_end(events: List[Dict]) -> Tuple[RaceEnd, str, float]:
    """
    Classify the race ending based on events in the CSV.
    Returns (RaceEnd category, details string, timestamp of ending event)
    """
    # Find the last significant event (excluding state_transition)
    last_event = None
    last_details = ""
    last_timestamp = 0.0

    for event in reversed(events):
        event_type = event.get("event_type", "")

        if event_type == "state_transition":
            continue

        last_event = event_type
        last_details = event.get("details", "")
        try:
            last_timestamp = float(event.get("timestamp", 0))
        except (ValueError, TypeError):
            last_timestamp = 0.0
        break

    # Check for specific events in all events
    has_overtake = any(e.get("event_type") == "overtake_lead" for e in events)
    has_collision = any(e.get("event_type") == "collision" for e in events)
    has_track_crash = any(e.get("event_type") == "track_crash" for e in events)
    has_lap_depletion = any("lap_depletion" in e.get("details", "").lower() or
                            "finished lap 10" in e.get("details", "").lower() or
                            "maximum lap" in e.get("details", "").lower()
                            for e in events)
    has_race_complete = any(e.get("event_type") ==
                            "race_complete" for e in events)
    has_timeout = any("timeout" in e.get("details", "").lower()
                      for e in events)

    # Priority-based classification
    if has_timeout:
        for e in reversed(events):
            if "timeout" in e.get("details", "").lower():
                try:
                    ts = float(e.get("timestamp", 0))
                except (ValueError, TypeError):
                    ts = 0.0
                return RaceEnd.TIMEOUT, e.get("details", "timeout"), ts
        return RaceEnd.TIMEOUT, "timeout", last_timestamp

    if has_overtake and last_event == "overtake_lead":
        return RaceEnd.SUCCESS, last_details, last_timestamp

    if has_collision:
        # Find the collision event details
        for e in reversed(events):
            if e.get("event_type") == "collision":
                try:
                    ts = float(e.get("timestamp", 0))
                except (ValueError, TypeError):
                    ts = 0.0
                return RaceEnd.COLLISION, e.get("details", "collision"), ts
        return RaceEnd.COLLISION, "collision", last_timestamp

    if has_track_crash:
        for e in reversed(events):
            if e.get("event_type") == "track_crash":
                try:
                    ts = float(e.get("timestamp", 0))
                except (ValueError, TypeError):
                    ts = 0.0
                return RaceEnd.TRACKBOUNDS, e.get("details", "track boundary collision"), ts
        return RaceEnd.TRACKBOUNDS, "track boundary collision", last_timestamp

    if has_lap_depletion or has_race_complete:
        if has_race_complete:
            for e in reversed(events):
                if e.get("event_type") == "race_complete":
                    details = e.get("details", "")
                    try:
                        ts = float(e.get("timestamp", 0))
                    except (ValueError, TypeError):
                        ts = 0.0
                    if "overtake" in details.lower():
                        return RaceEnd.SUCCESS, details, ts
                    return RaceEnd.DEPLETION, details, ts
        return RaceEnd.DEPLETION, last_details, last_timestamp

    # Check if it was a successful overtake based on details
    if has_overtake:
        for e in reversed(events):
            if e.get("event_type") == "overtake_lead":
                try:
                    ts = float(e.get("timestamp", 0))
                except (ValueError, TypeError):
                    ts = 0.0
                return RaceEnd.SUCCESS, e.get("details", "successful overtake"), ts

    return RaceEnd.UNKNOWN, last_details if last_details else "unknown", last_timestamp


def parse_csv_events(csv_path: Path) -> List[Dict]:
    """Parse CSV file and return list of event dictionaries"""
    events = []
    try:
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                events.append(row)
    except Exception as e:
        print(f"Error parsing {csv_path}: {e}")
    return events


def get_final_car_state(events: List[Dict]) -> Tuple[float, float, int]:
    """Get final car state (s, d, lap) from events"""
    car_s = 0.0
    car_d = 0.0
    car_lap = 0

    for event in reversed(events):
        try:
            s = float(event.get("car1_s", 0))
            d = float(event.get("car1_d", 0))
            lap = int(event.get("car1_lap", 0))
            if s != 0 or d != 0 or lap != 0:
                return s, d, lap
        except (ValueError, TypeError):
            continue

    return car_s, car_d, car_lap


def find_config_for_sim_id(config: Dict, sim_id: str) -> Optional[Dict]:
    """Find the configuration entry for a specific simulation ID"""
    test_matrix = config.get("test_matrix", [])
    for entry in test_matrix:
        if entry.get("simulation_id") == sim_id:
            return entry
    return None


def process_batch(batch_path: Path, mode: str) -> List[SimulationResult]:
    """Process all simulations in a batch directory"""
    results = []

    # Find config file
    config_files = list(batch_path.glob("*_config.yaml"))
    config_files = [f for f in config_files if "default" not in f.name]

    if not config_files:
        print(f"No config file found in {batch_path}")
        return results

    config_path = config_files[0]

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config {config_path}: {e}")
        return results

    # Process each CSV file
    csv_files = sorted(batch_path.glob("race_events_sim*.csv"))

    for csv_file in csv_files:
        # Extract simulation ID from filename
        # Pattern: race_events_sim<type>_<id>_<date>_<time>.csv
        # Example: race_events_simscwo_0001_20251129_234744.csv -> scwo_0001
        match = re.search(
            r'race_events_sim([a-z]+)_(\d+)_\d{8}_\d+\.csv', csv_file.name)
        if not match:
            continue

        sim_type = match.group(1)
        sim_num = match.group(2)
        sim_id = f"{sim_type}_{sim_num}"

        # Find config for this simulation
        sim_config = find_config_for_sim_id(config, sim_id)
        if not sim_config:
            print(f"No config found for {sim_id} in {batch_path}")
            continue

        # Parse CSV events
        events = parse_csv_events(csv_file)

        # Handle empty CSV files (only header, no events)
        if not events:
            # Get config data even for empty files
            global_map = sim_config.get("global_map", "")
            map_name = extract_map_short_name(global_map)

            if mode == "multi_car":
                planner = sim_config.get(
                    "planner_car1", sim_config.get("planner", ""))
                planner_car2 = sim_config.get("planner_car2", "")
                performance_factor = sim_config.get(
                    "speed_multiplier_car2", 1.0)
            else:
                planner = sim_config.get("planner", "")
                planner_car2 = ""
                performance_factor = sim_config.get("obstacle_speed", 1.0)

            result = SimulationResult(
                mode=mode,
                map_name=map_name,
                planner=planner,
                planner_car2=planner_car2,
                performance_factor=performance_factor,
                race_end=RaceEnd.UNKNOWN.value,
                race_end_timestamp=0.0,
                car_s=0.0,
                car_d=0.0,
                car_lap=0,
                details="empty CSV file (no events recorded)",
                simulation_id=sim_id,
                batch_id=batch_path.name,
                timestamp=0.0,
                csv_file_path=str(csv_file)
            )
            results.append(result)
            continue

        # Classify race end
        race_end, details, race_end_ts = classify_race_end(events)

        # Get final car state
        car_s, car_d, car_lap = get_final_car_state(events)

        # Extract map name
        global_map = sim_config.get("global_map", "")
        map_name = extract_map_short_name(global_map)

        # Extract planner
        if mode == "multi_car":
            planner = sim_config.get(
                "planner_car1", sim_config.get("planner", ""))
            planner_car2 = sim_config.get("planner_car2", "")
            # Performance factor for multi-car is car2's speed multiplier
            performance_factor = sim_config.get("speed_multiplier_car2", 1.0)
        else:
            planner = sim_config.get("planner", "")
            planner_car2 = ""
            # Performance factor for single car obstacle is obstacle_speed
            performance_factor = sim_config.get("obstacle_speed", 1.0)

        # Create result
        result = SimulationResult(
            mode=mode,
            map_name=map_name,
            planner=planner,
            planner_car2=planner_car2,
            performance_factor=performance_factor,
            race_end=race_end.value,
            race_end_timestamp=race_end_ts,
            car_s=car_s,
            car_d=car_d,
            car_lap=car_lap,
            details=details,
            simulation_id=sim_id,
            batch_id=batch_path.name,
            timestamp=float(events[-1].get("timestamp", 0)) if events else 0.0,
            csv_file_path=str(csv_file)
        )

        results.append(result)

    return results


def discover_batches(logs_dir: Path) -> List[Tuple[Path, str]]:
    """
    Discover all batch directories and their modes.
    Returns list of (batch_path, mode) tuples.
    """
    batches = []

    # Single car no obstacle: logs/single_car_no_obstacle/<map>/batch_*
    scno_dir = logs_dir / "single_car_no_obstacle"
    if scno_dir.exists():
        for map_dir in scno_dir.iterdir():
            if map_dir.is_dir():
                for batch_dir in map_dir.glob("batch_*"):
                    batches.append((batch_dir, "single_car_no_obstacle"))

    # Single car with obstacle: logs/single_car_obstacle/<map>/<planner>/batch_*
    scwo_dir = logs_dir / "single_car_obstacle"
    if scwo_dir.exists():
        for map_dir in scwo_dir.iterdir():
            if map_dir.is_dir():
                for planner_dir in map_dir.iterdir():
                    if planner_dir.is_dir():
                        for batch_dir in planner_dir.glob("batch_*"):
                            batches.append(
                                (batch_dir, "single_car_with_obstacle"))

    # Multi-car: logs/multi_car/<map>/<planner1>/<planner2>/batch_*
    # Example: logs/multi_car/marina/predictive_spliner/predictive_sampler/batch_20251130033845
    mc_dir = logs_dir / "multi_car"
    if mc_dir.exists():
        # Direct batches (legacy structure)
        for batch_dir in mc_dir.glob("batch_*"):
            batches.append((batch_dir, "multi_car"))
        # Nested structure: map/planner1/planner2/batch_*
        for map_dir in mc_dir.iterdir():
            if map_dir.is_dir() and not map_dir.name.startswith("batch_"):
                for planner1_dir in map_dir.iterdir():
                    if planner1_dir.is_dir():
                        for planner2_dir in planner1_dir.iterdir():
                            if planner2_dir.is_dir():
                                for batch_dir in planner2_dir.glob("batch_*"):
                                    batches.append((batch_dir, "multi_car"))

    return batches


def write_summary_csv(results: List[SimulationResult], output_path: Path):
    """Write results to summary CSV file"""
    fieldnames = [
        "mode",
        "map",
        "planner",
        "planner_car2",
        "performance_factor",
        "race_end",
        "race_end_timestamp",
        "car_s",
        "car_d",
        "car_lap",
        "details",
        "simulation_id",
        "batch_id",
        "duration_s"
    ]

    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for result in results:
            writer.writerow({
                "mode": result.mode,
                "map": result.map_name,
                "planner": result.planner,
                "planner_car2": result.planner_car2,
                "performance_factor": result.performance_factor,
                "race_end": result.race_end,
                "race_end_timestamp": f"{result.race_end_timestamp:.3f}",
                "car_s": f"{result.car_s:.2f}",
                "car_d": f"{result.car_d:.2f}",
                "car_lap": result.car_lap,
                "details": result.details,
                "simulation_id": result.simulation_id,
                "batch_id": result.batch_id,
                "duration_s": f"{result.timestamp:.2f}"
            })

    print(f"Summary written to: {output_path}")


def get_setup_key(result: SimulationResult) -> str:
    """Generate a unique key for each setup configuration"""
    if result.mode == "single_car_with_obstacle":
        return f"scwo_{result.map_name}_{result.planner}"
    elif result.mode == "multi_car":
        return f"multi_{result.map_name}_{result.planner}_{result.planner_car2}"
    else:
        return None  # Skip single_car_no_obstacle


def write_per_setup_csvs(results: List[SimulationResult], output_dir: Path):
    """Write separate CSV files for each setup configuration"""
    # Group results by setup
    setups = {}
    for result in results:
        key = get_setup_key(result)
        if key is None:  # Skip single_car_no_obstacle
            continue
        if key not in setups:
            setups[key] = []
        setups[key].append(result)

    fieldnames = [
        "mode",
        "map",
        "planner",
        "planner_car2",
        "performance_factor",
        "race_end",
        "race_end_timestamp",
        "car_s",
        "car_d",
        "car_lap",
        "details",
        "simulation_id",
        "batch_id",
        "duration_s"
    ]

    # Create output directory for per-setup files
    setup_dir = output_dir / "per_setup"
    setup_dir.mkdir(exist_ok=True)

    written_files = []
    for setup_key, setup_results in sorted(setups.items()):
        # Generate filename
        filename = f"{setup_key}.csv"
        filepath = setup_dir / filename

        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            for result in setup_results:
                writer.writerow({
                    "mode": result.mode,
                    "map": result.map_name,
                    "planner": result.planner,
                    "planner_car2": result.planner_car2,
                    "performance_factor": result.performance_factor,
                    "race_end": result.race_end,
                    "race_end_timestamp": f"{result.race_end_timestamp:.3f}",
                    "car_s": f"{result.car_s:.2f}",
                    "car_d": f"{result.car_d:.2f}",
                    "car_lap": result.car_lap,
                    "details": result.details,
                    "simulation_id": result.simulation_id,
                    "batch_id": result.batch_id,
                    "duration_s": f"{result.timestamp:.2f}"
                })

        written_files.append((filepath, len(setup_results)))

    print(f"\n📁 Per-setup CSV files written to: {setup_dir}")
    for filepath, count in written_files:
        print(f"   - {filepath.name}: {count} simulations")

    return written_files


def get_unique_key(result: SimulationResult) -> str:
    """Generate a unique key for each simulation (mode, map, planner(s), performance_factor)"""
    if result.mode == "single_car_with_obstacle":
        return f"{result.mode}|{result.map_name}|{result.planner}|{result.performance_factor}"
    elif result.mode == "multi_car":
        return f"{result.mode}|{result.map_name}|{result.planner}|{result.planner_car2}|{result.performance_factor}"
    else:
        return f"{result.mode}|{result.map_name}|{result.planner}|{result.performance_factor}"


def find_and_remove_duplicates(results: List[SimulationResult], output_dir: Path) -> Tuple[List[SimulationResult], List[SimulationResult]]:
    """
    Find duplicates and return (unique_results, duplicate_results).
    A duplicate is when the same (mode, map, planner(s), performance_factor) appears multiple times.
    Keeps the first occurrence.
    """
    seen = {}
    unique_results = []
    duplicates = []

    for result in results:
        key = get_unique_key(result)
        if key in seen:
            duplicates.append(result)
        else:
            seen[key] = result
            unique_results.append(result)

    # Write duplicates CSV if any found
    if duplicates:
        duplicates_path = output_dir / "duplicates.csv"
        fieldnames = [
            "mode",
            "map",
            "planner",
            "planner_car2",
            "performance_factor",
            "simulation_id",
            "batch_id",
            "csv_file_path"
        ]

        with open(duplicates_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            for result in duplicates:
                writer.writerow({
                    "mode": result.mode,
                    "map": result.map_name,
                    "planner": result.planner,
                    "planner_car2": result.planner_car2,
                    "performance_factor": result.performance_factor,
                    "simulation_id": result.simulation_id,
                    "batch_id": result.batch_id,
                    "csv_file_path": result.csv_file_path
                })

        print(f"\n⚠️  Found {len(duplicates)} duplicate simulations!")
        print(f"   Duplicates written to: {duplicates_path}")

    return unique_results, duplicates


def generate_setup_mode_statistics(results: List[SimulationResult], output_dir: Path) -> str:
    """
    Generate statistics per setup mode (without performance factors).
    Returns the statistics as a string.
    """
    # Group by setup mode (mode, map, planner(s))
    setup_modes = {}

    for result in results:
        if result.mode == "single_car_with_obstacle":
            key = f"scwo|{result.map_name}|{result.planner}"
            display_key = f"single_car_with_obstacle | {result.map_name} | {result.planner}"
        elif result.mode == "multi_car":
            key = f"multi|{result.map_name}|{result.planner}|{result.planner_car2}"
            display_key = f"multi_car | {result.map_name} | car1:{result.planner} vs car2:{result.planner_car2}"
        else:
            continue

        if key not in setup_modes:
            setup_modes[key] = {"display": display_key, "count": 0,
                                "mode": result.mode, "map": result.map_name}
        setup_modes[key]["count"] += 1

    # Build statistics string
    lines = []
    lines.append("=" * 80)
    lines.append("SETUP MODE STATISTICS")
    lines.append("=" * 80)
    lines.append("")

    # Summary counts
    total_sims = len(results)
    scwo_results = [r for r in results if r.mode == "single_car_with_obstacle"]
    multi_results = [r for r in results if r.mode == "multi_car"]

    scwo_f = len([r for r in scwo_results if r.map_name == "f"])
    scwo_marina = len([r for r in scwo_results if r.map_name == "marina"])
    multi_f = len([r for r in multi_results if r.map_name == "f"])
    multi_marina = len([r for r in multi_results if r.map_name == "marina"])

    lines.append(f"Total simulations: {total_sims}")
    lines.append("")
    lines.append(f"Single Car With Obstacle: {len(scwo_results)} simulations")
    lines.append(f"  - F map: {scwo_f} simulations")
    lines.append(f"  - Marina map: {scwo_marina} simulations")
    lines.append("")
    lines.append(f"Multi-Car: {len(multi_results)} simulations")
    lines.append(f"  - F map: {multi_f} simulations")
    lines.append(f"  - Marina map: {multi_marina} simulations")
    lines.append("")

    # Count unique setup modes
    scwo_modes = [k for k in setup_modes.keys() if k.startswith("scwo")]
    multi_modes = [k for k in setup_modes.keys() if k.startswith("multi")]

    scwo_f_modes = len([k for k in scwo_modes if "|f|" in k])
    scwo_marina_modes = len([k for k in scwo_modes if "|marina|" in k])
    multi_f_modes = len([k for k in multi_modes if "|f|" in k])
    multi_marina_modes = len([k for k in multi_modes if "|marina|" in k])

    lines.append(f"Unique setup modes: {len(setup_modes)}")
    lines.append(f"  - Single Car With Obstacle: {len(scwo_modes)} modes")
    lines.append(f"      - F map: {scwo_f_modes} modes")
    lines.append(f"      - Marina map: {scwo_marina_modes} modes")
    lines.append(f"  - Multi-Car: {len(multi_modes)} modes")
    lines.append(f"      - F map: {multi_f_modes} modes")
    lines.append(f"      - Marina map: {multi_marina_modes} modes")
    lines.append("")
    lines.append("-" * 80)
    lines.append("DETAILED BREAKDOWN BY SETUP MODE")
    lines.append("-" * 80)
    lines.append("")

    # Single car with obstacle
    lines.append("📦 SINGLE CAR WITH OBSTACLE")
    lines.append("")
    for key in sorted(scwo_modes):
        info = setup_modes[key]
        lines.append(f"  {info['display']}: {info['count']} simulations")
    lines.append("")

    # Multi-car
    lines.append("🏎️  MULTI-CAR")
    lines.append("")
    for key in sorted(multi_modes):
        info = setup_modes[key]
        lines.append(f"  {info['display']}: {info['count']} simulations")
    lines.append("")
    lines.append("=" * 80)

    stats_text = "\n".join(lines)

    # Write to file
    stats_path = output_dir / "setup_mode_statistics.txt"
    with open(stats_path, 'w') as f:
        f.write(stats_text)

    print(f"\n📊 Setup mode statistics written to: {stats_path}")

    return stats_text


def print_statistics(results: List[SimulationResult]):
    """Print summary statistics"""
    print("\n" + "=" * 70)
    print("RACE LOG ANALYSIS SUMMARY")
    print("=" * 70)

    # Group by mode
    modes = {}
    for r in results:
        if r.mode not in modes:
            modes[r.mode] = []
        modes[r.mode].append(r)

    for mode, mode_results in sorted(modes.items()):
        print(f"\n📊 Mode: {mode}")
        print("-" * 50)

        # Count race ends
        end_counts = {}
        for r in mode_results:
            end_counts[r.race_end] = end_counts.get(r.race_end, 0) + 1

        print(f"  Total simulations: {len(mode_results)}")
        print("  Race end breakdown:")
        for end_type, count in sorted(end_counts.items()):
            pct = 100 * count / len(mode_results)
            emoji = {"success": "✅", "collision": "💥", "trackbounds": "🚧",
                     "depletion": "🔄", "timeout": "⏱️", "unknown": "❓"}.get(end_type, "")
            print(f"    {emoji} {end_type}: {count} ({pct:.1f}%)")

        # Group by planner
        planners = {}
        for r in mode_results:
            if r.planner not in planners:
                planners[r.planner] = []
            planners[r.planner].append(r)

        print("\n  By planner:")
        for planner, planner_results in sorted(planners.items()):
            success = sum(
                1 for r in planner_results if r.race_end == "success")
            total = len(planner_results)
            print(
                f"    {planner}: {success}/{total} success ({100*success/total:.1f}%)")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze race simulation logs")
    parser.add_argument("--output", "-o", default="race_summary.csv",
                        help="Output CSV file path")
    parser.add_argument("--logs-dir", "-d",
                        default="src/race_stack/test_simulation/logs",
                        help="Path to logs directory")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Verbose output")

    args = parser.parse_args()

    # Resolve paths
    script_dir = Path(__file__).parent
    logs_dir = Path(args.logs_dir)

    if not logs_dir.is_absolute():
        # Try relative to script directory
        if (script_dir / args.logs_dir).exists():
            logs_dir = script_dir / args.logs_dir
        # Try relative to workspace root
        elif (script_dir.parent.parent.parent / args.logs_dir).exists():
            logs_dir = script_dir.parent.parent.parent / args.logs_dir
        else:
            logs_dir = script_dir / "logs"

    if not logs_dir.exists():
        print(f"Error: Logs directory not found: {logs_dir}")
        return 1

    print(f"📂 Scanning logs directory: {logs_dir}")

    # Discover all batches
    batches = discover_batches(logs_dir)
    print(f"📋 Found {len(batches)} batch directories")

    if args.verbose:
        for batch_path, mode in batches:
            print(f"   - {batch_path.relative_to(logs_dir)} ({mode})")

    # Process all batches
    all_results = []
    for batch_path, mode in batches:
        if args.verbose:
            print(f"\n🔍 Processing: {batch_path.name} ({mode})")
        results = process_batch(batch_path, mode)
        all_results.extend(results)
        if args.verbose:
            print(f"   Found {len(results)} simulations")

    print(f"\n✅ Total simulations processed: {len(all_results)}")

    # Filter out single_car_no_obstacle mode
    filtered_results = [
        r for r in all_results if r.mode != "single_car_no_obstacle"]
    print(
        f"📊 Simulations after filtering (excluding single_car_no_obstacle): {len(filtered_results)}")

    # Find and remove duplicates
    unique_results, duplicates = find_and_remove_duplicates(
        filtered_results, logs_dir)
    print(f"📋 Unique simulations: {len(unique_results)}")

    # Determine output path
    output_path = Path(args.output)
    if not output_path.is_absolute():
        output_path = logs_dir / args.output

    # Write summary CSV (unique results only)
    write_summary_csv(unique_results, output_path)

    # Write per-setup CSV files (unique results only)
    write_per_setup_csvs(unique_results, logs_dir)

    # Generate and print setup mode statistics
    stats_text = generate_setup_mode_statistics(unique_results, logs_dir)
    print("\n" + stats_text)

    # Print race end statistics (unique results)
    print_statistics(unique_results)

    # Print duplicate summary at the end
    if duplicates:
        print("\n" + "=" * 70)
        print("⚠️  DUPLICATE SIMULATIONS SUMMARY")
        print("=" * 70)
        print(
            f"\nFound {len(duplicates)} duplicates (same mode, map, planner(s), performance_factor):")
        for dup in duplicates:
            if dup.mode == "multi_car":
                print(
                    f"  - {dup.map_name} | {dup.planner} vs {dup.planner_car2} | pf={dup.performance_factor}")
            else:
                print(
                    f"  - {dup.map_name} | {dup.planner} | pf={dup.performance_factor}")
            print(f"    File: {dup.csv_file_path}")
        print(f"\nDuplicates CSV: {logs_dir / 'duplicates.csv'}")

    return 0


if __name__ == "__main__":
    exit(main())
