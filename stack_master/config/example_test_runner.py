#!/usr/bin/env python3
"""
Example script demonstrating how to use the planner test configurations.

This script shows how to:
1. Load test configurations from YAML
2. Iterate through configurations
3. Apply parameters to a planner (example using dynamic reconfigure)
4. Run tests and collect results

Usage:
    python3 example_test_runner.py --planner spliner
    python3 example_test_runner.py --planner predictive_spliner --config aggressive
"""

import yaml
import argparse
import os
from pathlib import Path


def load_configurations(config_file):
    """Load test configurations from YAML file."""
    with open(config_file, 'r') as f:
        return yaml.safe_load(f)


def load_parameter_definitions(params_file):
    """Load parameter definitions with min/max/default values."""
    with open(params_file, 'r') as f:
        return yaml.safe_load(f)


def get_config_names(configs, planner_name):
    """Get all configuration names for a specific planner."""
    prefix = f"{planner_name}_"
    return [name for name in configs.keys() if name.startswith(prefix)]


def apply_configuration(planner_name, config_name, params):
    """
    Apply configuration parameters to the planner.
    
    In a real implementation, this would use dynamic_reconfigure or ROS parameters.
    This is a placeholder showing the structure.
    """
    print(f"\n{'='*60}")
    print(f"Applying configuration: {config_name}")
    print(f"Planner: {planner_name}")
    print(f"{'='*60}")
    
    for param_name, value in params.items():
        print(f"  {param_name}: {value}")
    
    # Example: Using dynamic reconfigure (pseudo-code)
    # client = dynamic_reconfigure.client.Client(f"/{planner_name}/tuner")
    # client.update_configuration(params)
    
    return True


def run_test(planner_name, config_name, params):
    """
    Run a single test with the given configuration.
    
    Returns results dictionary with metrics.
    """
    # Apply configuration
    if not apply_configuration(planner_name, config_name, params):
        return None
    
    # Placeholder for actual test execution
    # In real implementation:
    # 1. Start the simulation/car
    # 2. Run for N laps or time
    # 3. Collect metrics (lap time, safety distance, lateral error, etc.)
    
    print(f"\nRunning test: {config_name}...")
    print("  (In real implementation, would run simulation/car here)")
    
    # Example results
    results = {
        'config_name': config_name,
        'planner': planner_name,
        'parameters': params,
        'lap_time': 0.0,  # Placeholder
        'avg_lateral_error': 0.0,  # Placeholder
        'min_obstacle_distance': 0.0,  # Placeholder
        'success': True,
    }
    
    return results


def main():
    parser = argparse.ArgumentParser(description='Run planner test configurations')
    parser.add_argument('--planner', 
                       choices=['spliner', 'predictive_spliner'],
                       required=True,
                       help='Planner to test')
    parser.add_argument('--config',
                       help='Specific configuration to run (default: run all)')
    parser.add_argument('--config-file',
                       default='single_car_with_obstacles_config.yaml',
                       help='Path to configuration file')
    parser.add_argument('--params-file',
                       default='planner_params_variable.yaml',
                       help='Path to parameter definitions file')
    args = parser.parse_args()
    
    # Get script directory
    script_dir = Path(__file__).parent
    config_file = script_dir / args.config_file
    params_file = script_dir / args.params_file
    
    # Load configurations
    print(f"Loading configurations from: {config_file}")
    configs = load_configurations(config_file)
    
    print(f"Loading parameter definitions from: {params_file}")
    param_defs = load_parameter_definitions(params_file)
    
    # Get configurations to run
    if args.config:
        config_full_name = f"{args.planner}_{args.config}"
        if config_full_name not in configs:
            print(f"Error: Configuration '{config_full_name}' not found")
            return
        configs_to_run = {config_full_name: configs[config_full_name]}
    else:
        # Run all configurations for the planner
        config_names = get_config_names(configs, args.planner)
        configs_to_run = {name: configs[name] for name in config_names}
    
    print(f"\nFound {len(configs_to_run)} configurations to run")
    
    # Run tests
    results = []
    for config_name, params in configs_to_run.items():
        result = run_test(args.planner, config_name, params)
        if result:
            results.append(result)
    
    # Display summary
    print(f"\n{'='*60}")
    print("TEST SUMMARY")
    print(f"{'='*60}")
    print(f"Total tests run: {len(results)}")
    print(f"Successful: {sum(1 for r in results if r['success'])}")
    print(f"Failed: {sum(1 for r in results if not r['success'])}")
    
    # Save results (placeholder)
    # In real implementation, save to file for analysis
    print(f"\nResults would be saved to file for analysis")


if __name__ == '__main__':
    main()
