#!/usr/bin/env python3
"""
TAM Parameter Diagnostic Tool

Prints all relevant TAM sampling parameters to diagnose path collision issues.
"""

import rospy


def print_params():
    rospy.init_node('tam_param_diagnostics', anonymous=True)

    print("\n" + "="*60)
    print("TAM SAMPLING PLANNER PARAMETER DIAGNOSTICS")
    print("="*60)

    print("\n### VEHICLE PARAMETERS ###")
    vehicle_width = rospy.get_param('/car1/width', 0.20)
    vehicle_length = rospy.get_param('/car1/length', 0.50)
    print(f"  Vehicle width:  {vehicle_width:.3f} m")
    print(f"  Vehicle length: {vehicle_length:.3f} m")

    print("\n### SAFETY DISTANCES ###")
    safety_left = rospy.get_param(
        '/car1/safety_distances/safety_distance_track_left', 0.0)
    safety_right = rospy.get_param(
        '/car1/safety_distances/safety_distance_track_right', 0.0)
    tube_width = rospy.get_param('/car1/behavior/tube_width', 1.15)
    print(f"  Safety distance left:  {safety_left:.3f} m")
    print(f"  Safety distance right: {safety_right:.3f} m")
    print(f"  Tube width:            {tube_width:.3f} m")

    total_margin_left = vehicle_width/2.0 + safety_left + tube_width
    total_margin_right = vehicle_width/2.0 + safety_right + tube_width
    print(f"\n  TOTAL margin left:  {total_margin_left:.3f} m")
    print(f"  TOTAL margin right: {total_margin_right:.3f} m")
    print(
        f"  TOTAL margin both:  {total_margin_left + total_margin_right:.3f} m")

    print("\n### SAMPLING PARAMETERS ###")
    n_samples = rospy.get_param('/car1/discretization/n_samples', 20)
    n_dense_samples = rospy.get_param(
        '/car1/discretization/n_dense_samples', 5)
    n_dense_min = rospy.get_param('/car1/discretization/n_dense_min', -0.5)
    n_dense_max = rospy.get_param('/car1/discretization/n_dense_max', 0.5)
    print(f"  Lateral samples:       {n_samples}")
    print(f"  Dense samples:         {n_dense_samples}")
    print(f"  Dense range:           [{n_dense_min:.2f}, {n_dense_max:.2f}] m")
    print(f"  Total trajectories:    ~{n_samples + n_dense_samples}")

    print("\n### LONGITUDINAL SAMPLING ###")
    s_dot_discretization = rospy.get_param(
        '/car1/discretization/s_dot_discretization', 2.0)
    samples_fb = rospy.get_param('/car1/behavior/samples_forward_backward', 3)
    horizon = rospy.get_param('/car1/behavior/horizon', 4.0)
    print(f"  Velocity discretization: {s_dot_discretization:.2f} m/s")
    print(f"  Forward-backward samples: {samples_fb}")
    print(f"  Planning horizon:        {horizon:.2f} s")

    print("\n### TRACK CONSTRAINTS ###")
    kappa_thr = rospy.get_param('/car1/behavior/kappa_thr', 0.1)
    print(f"  Max curvature (kappa_thr): {kappa_thr:.3f} rad/m")

    print("\n### ANALYSIS ###")
    print("  If all trajectories fail path check, the issue is likely:")
    if total_margin_left + total_margin_right > 2.0:
        print(
            f"  ⚠️  EXCESSIVE MARGINS: Total margins ({total_margin_left + total_margin_right:.2f}m) reduce available track width significantly")
        print(
            "  → Try reducing tube_width from {:.2f} to 0.3-0.5".format(tube_width))

    if tube_width > 1.0:
        print(
            f"  ⚠️  LARGE TUBE_WIDTH: {tube_width:.2f}m is quite large for F1TENTH")
        print("  → Recommended: 0.3-0.5 m for 1/10 scale racing")

    if vehicle_width > 0.25:
        print(f"  ⚠️  LARGE VEHICLE WIDTH: {vehicle_width:.2f}m seems large")
        print("  → F1TENTH typical width: 0.15-0.20 m")

    print("\n" + "="*60 + "\n")


if __name__ == '__main__':
    try:
        print_params()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
