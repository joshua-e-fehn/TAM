#!/usr/bin/env python3
"""
Simple GGGV Manager for TAM Sampling Planner
Provides conservative acceleration limits when real GGGV data is not available

This is a fallback implementation that uses typical racing car parameters
to approximate the vehicle's acceleration envelope.
"""

import numpy as np
import rospy


class SimpleGGGVManager:
    """
    Simplified GGGV manager that provides conservative acceleration limits
    based on typical racing car parameters.

    This is designed as a fallback when real GGGV diagrams are not available.
    """

    def __init__(self):
        """Initialize with conservative racing car parameters."""
        # Conservative acceleration limits [m/s²]
        self.max_longitudinal_accel = 8.0      # Typical for F1/Formula cars
        self.max_longitudinal_decel = -12.0    # Strong braking capability
        self.max_lateral_accel = 15.0          # High cornering capability

        # Machine limits (engine/brake limitations)
        self.ax_machine_limits = np.array(
            [8.0, 7.5, 7.0, 6.5, 6.0, 5.5, 5.0, 4.5, 4.0, 3.5])

        # G-G diagram exponents (shape of the acceleration envelope)
        self.gg_exponent_ax_pos = 2.0  # Positive acceleration envelope shape
        self.gg_exponent_ax_neg = 2.0  # Negative acceleration envelope shape

        # Velocity-dependent scaling
        self.velocity_breakpoints = np.array(
            [0, 5, 10, 15, 20, 25, 30, 40, 50, 60])  # m/s
        self.accel_scale_factors = np.array(
            [0.8, 0.9, 1.0, 1.0, 0.98, 0.95, 0.9, 0.8, 0.7, 0.6])

        rospy.loginfo(
            "SimpleGGGVManager initialized with conservative parameters")
        rospy.logwarn(
            "Using simplified GGGV model - consider obtaining real vehicle data for better performance")

    def acc_interpolator(self, V, g_tilde, s, n, use_friction=True, debug=False):
        """
        Interpolate acceleration limits based on velocity and conditions.

        Args:
            V: Velocity [m/s] (can be scalar or array)
            g_tilde: Apparent gravitational acceleration [m/s²]
            s: Arc length position [m]
            n: Lateral position [m]
            use_friction: Whether to apply friction limitations
            debug: Enable debug output

        Returns:
            tuple: (friction_coeff, ax_min, ax_max, ay_max, debug_info)
        """
        # Ensure inputs are arrays for consistent processing
        V = np.atleast_1d(V)
        g_tilde = np.atleast_1d(g_tilde)
        s = np.atleast_1d(s)
        n = np.atleast_1d(n)

        # Initialize output arrays
        friction_coeff = np.ones_like(V) * 1.0  # Assume good grip
        ax_min = np.ones_like(V) * self.max_longitudinal_decel
        ax_max = np.ones_like(V) * self.max_longitudinal_accel
        ay_max = np.ones_like(V) * self.max_lateral_accel

        # Apply velocity-dependent scaling
        for i, v in enumerate(V):
            scale = np.interp(v, self.velocity_breakpoints,
                              self.accel_scale_factors)
            ax_max[i] *= scale
            # Deceleration typically less affected by speed
            ax_min[i] *= min(scale + 0.1, 1.0)
            ay_max[i] *= scale

        # Apply gravitational effects (simplified)
        if use_friction:
            # Reduce limits slightly if high apparent gravity
            g_factor = np.minimum(g_tilde / 9.81, 1.2)  # Cap at 1.2g
            # Slight reduction with high g
            friction_coeff *= (2.0 - g_factor * 0.1)
            ax_max *= friction_coeff
            ax_min *= friction_coeff
            ay_max *= friction_coeff

        # Machine limitations (engine/brake limits)
        ax_machine = np.interp(V, np.linspace(0, 60, len(self.ax_machine_limits)),
                               self.ax_machine_limits)
        ax_max = np.minimum(ax_max, ax_machine)

        # Debug output
        debug_info = {
            'velocity_scale': np.interp(V, self.velocity_breakpoints, self.accel_scale_factors),
            'friction_coeff': friction_coeff,
            'machine_limit': ax_machine
        } if debug else None

        # Return scalars if input was scalar
        if len(V) == 1:
            return (float(friction_coeff[0]), float(ax_min[0]),
                    float(ax_max[0]), float(ay_max[0]), debug_info)
        else:
            return friction_coeff, ax_min, ax_max, ay_max, debug_info

    def get_max_acceleration_at_speed(self, velocity):
        """
        Get maximum acceleration capability at given speed.

        Args:
            velocity: Vehicle speed [m/s]

        Returns:
            dict: Dictionary with acceleration limits
        """
        _, ax_min, ax_max, ay_max, _ = self.acc_interpolator(
            velocity, 9.81, 0.0, 0.0, use_friction=True
        )

        return {
            'max_accel': ax_max,
            'max_decel': abs(ax_min),
            'max_lateral': ay_max,
            'velocity': velocity
        }

    def validate_limits(self):
        """Validate that the acceleration limits are reasonable."""
        test_speeds = np.array([5, 15, 25, 35, 45])

        rospy.loginfo("GGGV Validation:")
        for v in test_speeds:
            limits = self.get_max_acceleration_at_speed(v)
            rospy.loginfo(f"  Speed {v:2.0f} m/s: ax_max={limits['max_accel']:.1f}, "
                          f"ax_min={-limits['max_decel']:.1f}, ay_max={limits['max_lateral']:.1f}")

        # Check for reasonable limits
        if self.max_longitudinal_accel > 15.0:
            rospy.logwarn(
                "Very high longitudinal acceleration limit - check parameters")
        if self.max_lateral_accel > 20.0:
            rospy.logwarn(
                "Very high lateral acceleration limit - check parameters")

        return True


class GripMap:
    """Simplified grip map - provides constant grip across the track."""

    def __init__(self, default_grip=1.0):
        """
        Initialize with constant grip.

        Args:
            default_grip: Default friction coefficient (typically 0.8-1.2)
        """
        self.default_grip = default_grip
        rospy.loginfo(
            f"GripMap initialized with constant grip coefficient: {default_grip}")

    def get_grip_at_position(self, s, n):
        """
        Get grip coefficient at track position.

        Args:
            s: Arc length position [m]
            n: Lateral position [m]

        Returns:
            float: Grip coefficient
        """
        return self.default_grip


def create_simple_gggv_manager():
    """Factory function to create a simple GGGV manager."""
    return SimpleGGGVManager()


def create_simple_grip_map(grip_coefficient=1.0):
    """Factory function to create a simple grip map."""
    return GripMap(grip_coefficient)


# Example usage and testing
if __name__ == "__main__":
    # Test the simple GGGV manager
    rospy.init_node('test_simple_gggv', anonymous=True)

    gggv = create_simple_gggv_manager()
    grip_map = create_simple_grip_map()

    # Validate the limits
    gggv.validate_limits()

    # Test interpolation
    test_velocities = [10, 20, 30, 40]
    for v in test_velocities:
        friction, ax_min, ax_max, ay_max, debug = gggv.acc_interpolator(
            v, 9.81, 0.0, 0.0, use_friction=True, debug=True
        )
        print(
            f"V={v} m/s: ax_min={ax_min:.2f}, ax_max={ax_max:.2f}, ay_max={ay_max:.2f}")
