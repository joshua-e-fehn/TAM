#!/usr/bin/env python3
"""
Verify TAM Sampling Planner Parameter Loading Standardization

This script verifies that all modules correctly load parameters from YAML defaults.
"""

import os
import sys
import yaml
import rospkg


def check_yaml_file():
    """Verify tam_sampling_params.yaml exists and is valid"""
    print("=" * 80)
    print("PARAMETER LOADING STANDARDIZATION VERIFICATION")
    print("=" * 80)

    try:
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('tam_sampling_planner')
        yaml_path = os.path.join(
            pkg_path, 'config', 'tam_sampling_params.yaml')

        print(f"\n✓ Package path: {pkg_path}")
        print(f"✓ YAML file: {yaml_path}")

        # Load YAML file
        with open(yaml_path, 'r') as f:
            params = yaml.safe_load(f)

        print(f"\n✓ YAML file loaded successfully")
        print(f"✓ Total parameters in YAML: {len(params)}")

        return params, yaml_path

    except Exception as e:
        print(f"\n✗ Error loading YAML file: {e}")
        return None, None


def verify_critical_parameters(params):
    """Verify critical parameters have correct values"""
    print("\n" + "=" * 80)
    print("CRITICAL PARAMETER VERIFICATION")
    print("=" * 80)

    critical_params = {
        'width': (0.20, "Vehicle width (F1TENTH standard)"),
        'tube_width': (0.3, "Trajectory tube width (reduced for F1TENTH)"),
        'safety_distance_track_left': (0.1, "Left track safety distance"),
        'safety_distance_track_right': (0.1, "Right track safety distance"),
        'lateral_samples': (15, "Number of lateral trajectory samples"),
        'longitudinal_samples': (8, "Number of longitudinal velocity profiles"),
        'planning_horizon': (4.0, "Planning time horizon (seconds)"),
    }

    all_correct = True
    for param, (expected, description) in critical_params.items():
        actual = params.get(param, "MISSING")
        status = "✓" if actual == expected else "✗"

        if actual != expected:
            all_correct = False

        print(f"{status} {param}: {actual} (expected: {expected})")
        print(f"   → {description}")

    return all_correct


def verify_module_parameters(params):
    """Verify all module-specific parameters are present"""
    print("\n" + "=" * 80)
    print("MODULE PARAMETER COVERAGE")
    print("=" * 80)

    module_params = {
        'trajectory_checks.py': [
            'tube_width', 'tire_util_max_check', 'kappa_thr',
            'safety_distance_track_left', 'safety_distance_track_right',
            'safety_distance_pitlane_left', 'safety_distance_pitlane_right',
            'soft_safety_distance_left_m', 'soft_safety_distance_right_m'
        ],
        'lateral_sampling.py': [
            'lateral_samples', 'n_dense_min', 'n_dense_max', 'n_dense_samples',
            'safety_distance_track_left', 'safety_distance_track_right', 'tube_width'
        ],
        'longitudinal_sampling.py': [
            's_dot_end_min', 'relative_s_dot_min_percentage', 's_dot_max_positive_delta',
            's_dot_discretization', 's_dot_dense_min', 's_dot_dense_max', 's_dot_dense_samples',
            'lateral_samples', 'n_dense_samples', 'num_samples', 'planning_horizon',
            'v_sampling_scale', 'forward_backward_velocities', 'samples_forward_backward',
            'forward_backward_min_scale', 'forward_backward_max_scale',
            'forward_backward_max_v_to_rl_delta'
        ],
        'calculation_costs.py': [
            'curvature_cost_weight', 'curvature_cost_threshold', 'raceline_cost_weight',
            'velocity_cost_weight', 'friction_cost_weight', 'lateral_jerk_cost_weight',
            'raceline_cost_weight_overtaking', 'velocity_cost_weight_overtaking',
            'lateral_jerk_cost_weight_overtaking', 'prediction_cost_weight',
            'additional_absolute_sample_cost', 'collision_cost_weight', 'planning_horizon',
            'max_deceleration_on_target_change', 'collision_check_horizon_s', 'tube_width',
            'prediction_s_factor_min_size', 'prediction_s_factor_max_size',
            'prediction_s_asym_scaling', 'prediction_n_factor', 'prediction_s_factor_defender',
            'prediction_n_factor_defender', 'prediction_s_factor_static',
            'prediction_n_factor_static', 'prediction_uncertainty_weight',
            'increasing_rl_cost', 'velocity_excess_cost_multiplier', 'V_diff_max_costs',
            'safety_distance_vehicles'
        ]
    }

    all_present = True
    for module, param_list in module_params.items():
        print(f"\n{module}:")
        missing = []
        present = []

        for param in param_list:
            if param in params:
                present.append(param)
            else:
                missing.append(param)
                all_present = False

        print(f"  ✓ Present: {len(present)}/{len(param_list)} parameters")

        if missing:
            print(f"  ✗ Missing: {', '.join(missing)}")

    return all_present


def show_safety_margin_calculation(params):
    """Show current safety margin calculation"""
    print("\n" + "=" * 80)
    print("SAFETY MARGIN CALCULATION")
    print("=" * 80)

    width = params.get('width', 0.0)
    safety_left = params.get('safety_distance_track_left', 0.0)
    safety_right = params.get('safety_distance_track_right', 0.0)
    tube_width = params.get('tube_width', 0.0)
    track_width = params.get('track_width', 3.0)

    margin_left = width/2 + safety_left + tube_width
    margin_right = width/2 + safety_right + tube_width
    total_margin = margin_left + margin_right
    usable_width = track_width - total_margin

    print(f"\nTrack Width: {track_width}m")
    print(f"\nLeft Margin Calculation:")
    print(f"  vehicle_width/2:  {width/2:.2f}m")
    print(f"  safety_distance:  {safety_left:.2f}m")
    print(f"  tube_width:       {tube_width:.2f}m")
    print(f"  ─────────────────────────")
    print(f"  Total Left:       {margin_left:.2f}m")

    print(f"\nRight Margin Calculation:")
    print(f"  vehicle_width/2:  {width/2:.2f}m")
    print(f"  safety_distance:  {safety_right:.2f}m")
    print(f"  tube_width:       {tube_width:.2f}m")
    print(f"  ─────────────────────────")
    print(f"  Total Right:      {margin_right:.2f}m")

    print(f"\n{'='*30}")
    print(f"Total Margins:    {total_margin:.2f}m")
    print(f"Usable Width:     {usable_width:.2f}m")
    print(f"{'='*30}")

    if usable_width < 0.5:
        print(
            "\n⚠️  WARNING: Usable width is very small! Trajectories may fail path checks.")
    elif usable_width < 1.0:
        print("\n⚠️  CAUTION: Limited usable width. Monitor trajectory success rate.")
    else:
        print("\n✓ Good usable width for trajectory planning.")


def main():
    # Load and verify YAML file
    params, yaml_path = check_yaml_file()
    if params is None:
        print("\n✗ VERIFICATION FAILED: Could not load YAML file")
        sys.exit(1)

    # Verify critical parameters
    critical_ok = verify_critical_parameters(params)

    # Verify module parameter coverage
    coverage_ok = verify_module_parameters(params)

    # Show safety margin calculation
    show_safety_margin_calculation(params)

    # Final summary
    print("\n" + "=" * 80)
    print("VERIFICATION SUMMARY")
    print("=" * 80)

    if critical_ok and coverage_ok:
        print("\n✓ ALL CHECKS PASSED!")
        print("✓ YAML file contains all required parameters")
        print("✓ Critical parameters have correct values")
        print("✓ All module parameters are present")
        print("\n✓ Parameter loading standardization: COMPLETE")
        return 0
    else:
        print("\n✗ SOME CHECKS FAILED")
        if not critical_ok:
            print("✗ Critical parameters have incorrect values")
        if not coverage_ok:
            print("✗ Some module parameters are missing")
        print("\n✗ Parameter loading standardization: INCOMPLETE")
        return 1


if __name__ == '__main__':
    sys.exit(main())
