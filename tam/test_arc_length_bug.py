#!/usr/bin/env python3
"""
Test script to verify arc length (s_m) calculation accuracy.

This script demonstrates the bug where interpolated s_m values don't match
actual geometric distances, causing position tracking drift.
"""

import math
import numpy as np


def calculate_arc_length_geometric(waypoints):
    """Calculate arc length from actual geometric distances (CORRECT)."""
    if not waypoints:
        return waypoints
    
    corrected = []
    for i, wp in enumerate(waypoints):
        new_wp = wp.copy()
        if i == 0:
            new_wp['s_m'] = 0.0
        else:
            dx = waypoints[i]['x_m'] - waypoints[i-1]['x_m']
            dy = waypoints[i]['y_m'] - waypoints[i-1]['y_m']
            segment_length = math.sqrt(dx**2 + dy**2)
            new_wp['s_m'] = corrected[i-1]['s_m'] + segment_length
        corrected.append(new_wp)
    
    return corrected


def test_interpolation_bug():
    """Demonstrate the s_m interpolation bug."""
    
    print("\n" + "="*80)
    print("Arc Length (s_m) Bug Demonstration")
    print("="*80)
    
    # Create a simple curved trajectory
    original_waypoints = []
    for i in range(10):
        angle = i * math.pi / 9  # 0 to π
        x = 10.0 * math.cos(angle)
        y = 10.0 * math.sin(angle)
        original_waypoints.append({
            'id': i,
            'x_m': x,
            'y_m': y,
            's_m': 0.0  # Will be calculated
        })
    
    # Calculate correct arc lengths for original waypoints
    original_waypoints = calculate_arc_length_geometric(original_waypoints)
    
    print(f"\n📊 Original trajectory: {len(original_waypoints)} waypoints")
    print(f"   Total arc length: {original_waypoints[-1]['s_m']:.3f}m")
    
    # Simulate buggy interpolation (linearly interpolate s_m)
    print(f"\n⚠️  BUGGY APPROACH: Linearly interpolate s_m values")
    target_count = 20
    s_values = np.array([wp['s_m'] for wp in original_waypoints])
    x_values = np.array([wp['x_m'] for wp in original_waypoints])
    y_values = np.array([wp['y_m'] for wp in original_waypoints])
    
    s_new = np.linspace(s_values[0], s_values[-1], target_count)
    x_new = np.interp(s_new, s_values, x_values)
    y_new = np.interp(s_new, s_values, y_values)
    
    buggy_waypoints = []
    for i in range(target_count):
        buggy_waypoints.append({
            'id': i,
            'x_m': x_new[i],
            'y_m': y_new[i],
            's_m': s_new[i]  # BUG: Using interpolated s_m directly
        })
    
    # Calculate actual arc lengths
    actual_waypoints = calculate_arc_length_geometric(buggy_waypoints)
    
    print(f"\n📊 After interpolation to {target_count} waypoints:")
    print(f"   Buggy s_m (last): {buggy_waypoints[-1]['s_m']:.3f}m")
    print(f"   Actual arc length: {actual_waypoints[-1]['s_m']:.3f}m")
    print(f"   ❌ ERROR: {abs(buggy_waypoints[-1]['s_m'] - actual_waypoints[-1]['s_m']):.3f}m")
    
    # Calculate accumulated error at each waypoint
    max_error = 0.0
    avg_error = 0.0
    for i in range(len(buggy_waypoints)):
        error = abs(buggy_waypoints[i]['s_m'] - actual_waypoints[i]['s_m'])
        max_error = max(max_error, error)
        avg_error += error
    
    avg_error /= len(buggy_waypoints)
    
    print(f"\n📈 Error Statistics:")
    print(f"   Maximum error: {max_error:.3f}m")
    print(f"   Average error: {avg_error:.3f}m")
    print(f"   Error growth rate: {max_error/actual_waypoints[-1]['s_m']*100:.1f}% of track length")
    
    # Show impact on position tracking
    print(f"\n🚗 Impact on car position tracking:")
    print(f"   If a car travels {actual_waypoints[-1]['s_m']:.1f}m,")
    print(f"   it thinks it traveled {buggy_waypoints[-1]['s_m']:.1f}m")
    print(f"   Position error: {max_error:.1f}m ({max_error/actual_waypoints[-1]['s_m']*100:.1f}%)")
    print(f"   ⚠️  This error ACCUMULATES over multiple laps!")
    
    print(f"\n✅ CORRECT APPROACH: Always recalculate s_m from geometry")
    print(f"   1. Interpolate x,y coordinates")
    print(f"   2. Recalculate s_m from actual distances")
    print(f"   3. Ensures s_m = Σ√((x[i]-x[i-1])² + (y[i]-y[i-1])²)")
    
    print("\n" + "="*80)


if __name__ == "__main__":
    test_interpolation_bug()
