"""
Pacejka tire model implementation for calculating maximum tire forces.

This module implements a simplified Pacejka "Magic Formula" tire model
for estimating maximum longitudinal (braking/acceleration) and lateral
(cornering) forces based on normal load and vehicle parameters.

DESIGN NOTE:
This is a lightweight implementation specifically for the TAM sampling planner's
emergency trajectory generation. While the TAM stack has comprehensive Pacejka
implementations in other modules (bicycle_model.py, std_kinematics.cpp), this
standalone version is preferred here because it:
  1. Only needs maximum force limits (not full slip dynamics)
  2. Operates independently without requiring full vehicle dynamics integration
  3. Uses the same parameter format (C_Pf, C_Pr) from vehicle config files
  4. Provides simple scalar/array interfaces for trajectory calculations

For full vehicle dynamics simulation, see:
  - src/race_stack/controller/mpc/src/single_track_mpc/bicycle_model.py
  - src/race_stack/base_system/f110-simulator/src/std_kinematics.cpp

References:
- Pacejka, H. B. (2012). Tire and vehicle dynamics (3rd ed.). 
  Butterworth-Heinemann.
"""

import numpy as np
import rospy
from typing import Union


class PacejkaTireModel:
    """
    Simplified Pacejka tire model for maximum force calculation.

    Uses the Magic Formula to compute tire force limits based on:
    - Normal load (Fz)
    - Slip ratio (for longitudinal force)
    - Slip angle (for lateral force)
    - Combined slip effects
    """

    def __init__(self):
        """Initialize and load Pacejka parameters from ROS parameter server."""
        self.load_parameters()

    def load_parameters(self):
        """Load vehicle and tire parameters from ROS parameter server.

        Note: Parameter names match NUC2_pacejka.yaml and other vehicle configs.
        This implementation is compatible with the existing TAM stack parameter structure.
        """
        # Vehicle parameters (standard names from vehicle config files)
        self.mass = rospy.get_param(
            '/m', rospy.get_param('/vehicle_mass', 3.54))  # [kg]
        self.l_f = rospy.get_param('/l_f', 0.162)  # [m] - front axle to CG
        self.l_r = rospy.get_param('/l_r', 0.145)  # [m] - rear axle to CG
        self.h_cg = rospy.get_param('/h_cg', 0.014)  # [m] - CG height
        self.mu = rospy.get_param('/mu', 1.0)  # [-] - friction coefficient

        # Pacejka coefficients for longitudinal force (front)
        # Format: C_Pf = [B, C, D, E] matching NUC2_pacejka.yaml
        C_Pf = rospy.get_param('/C_Pf', [4.8, 2.16, 0.65, 0.37])
        self.B_xf = C_Pf[0]  # Stiffness factor
        self.C_xf = C_Pf[1]  # Shape factor
        self.D_xf = C_Pf[2]  # Peak factor (normalized by Fz)
        self.E_xf = C_Pf[3]  # Curvature factor

        # Pacejka coefficients for longitudinal force (rear)
        # Format: C_Pr = [B, C, D, E] matching NUC2_pacejka.yaml
        C_Pr = rospy.get_param('/C_Pr', [20.0, 1.5, 0.62, 0.0])
        self.B_xr = C_Pr[0]  # Stiffness factor
        self.C_xr = C_Pr[1]  # Shape factor
        self.D_xr = C_Pr[2]  # Peak factor (normalized by Fz)
        self.E_xr = C_Pr[3]  # Curvature factor

        # For lateral forces, use same coefficients (simplified model)
        # In a full model, these would be separate C_alpha parameters
        self.B_yf = self.B_xf * 0.8  # Typically softer in lateral
        self.C_yf = self.C_xf
        self.D_yf = self.D_xf * 1.1  # Typically higher lateral capacity
        self.E_yf = self.E_xf

        self.B_yr = self.B_xr * 0.8
        self.C_yr = self.C_xr
        self.D_yr = self.D_xr * 1.1
        self.E_yr = self.E_xr

        # Nominal load for normalization [N]
        self.Fz_nominal = self.mass * 9.81 / 4.0  # Assume equal weight distribution

        # Combined slip parameters
        self.combined_slip_exponent = 2.0  # Friction ellipse exponent

        rospy.loginfo(
            f"Pacejka tire model initialized: m={self.mass}kg, mu={self.mu}")

    def magic_formula(self, x: Union[float, np.ndarray], B: float, C: float,
                      D: float, E: float) -> Union[float, np.ndarray]:
        """
        Pacejka Magic Formula.

        Args:
            x: Slip variable (slip ratio or slip angle)
            B: Stiffness factor
            C: Shape factor
            D: Peak value factor
            E: Curvature factor

        Returns:
            Force coefficient [-]
        """
        # Magic Formula: y = D * sin(C * arctan(B*x - E*(B*x - arctan(B*x))))
        Bx = B * x
        arctan_Bx = np.arctan(Bx)
        y = D * np.sin(C * np.arctan(Bx - E * (Bx - arctan_Bx)))
        return y

    def calc_normal_loads(self, ax_hat: Union[float, np.ndarray],
                          ay_hat: Union[float, np.ndarray]) -> tuple:
        """
        Calculate normal loads on front and rear axles considering weight transfer.

        Args:
            ax_hat: Longitudinal acceleration [m/s²]
            ay_hat: Lateral acceleration [m/s²]

        Returns:
            (Fz_front, Fz_rear): Normal loads [N]
        """
        g = 9.81
        l_wb = self.l_f + self.l_r

        # Static loads
        Fz_front_static = self.mass * g * self.l_r / l_wb
        Fz_rear_static = self.mass * g * self.l_f / l_wb

        # Longitudinal weight transfer (braking adds to front, accel to rear)
        delta_Fz_long = self.mass * np.abs(ax_hat) * self.h_cg / l_wb

        # For braking (negative ax), add to front
        # For acceleration (positive ax), add to rear
        Fz_front = Fz_front_static + \
            np.where(ax_hat < 0, delta_Fz_long, -delta_Fz_long)
        Fz_rear = Fz_rear_static + \
            np.where(ax_hat < 0, -delta_Fz_long, delta_Fz_long)

        # Ensure positive loads
        Fz_front = np.maximum(Fz_front, 0.1)
        Fz_rear = np.maximum(Fz_rear, 0.1)

        return Fz_front, Fz_rear

    def calc_max_longitudinal_force(self, Fz: Union[float, np.ndarray],
                                    axle: str = 'front') -> Union[float, np.ndarray]:
        """
        Calculate maximum longitudinal force (braking or acceleration).

        Args:
            Fz: Normal load [N]
            axle: 'front' or 'rear'

        Returns:
            Fx_max: Maximum longitudinal force [N]
        """
        # Select appropriate coefficients
        if axle == 'front':
            B, C, D, E = self.B_xf, self.C_xf, self.D_xf, self.E_xf
        else:
            B, C, D, E = self.B_xr, self.C_xr, self.D_xr, self.E_xr

        # Load-dependent peak force
        # D coefficient is typically normalized by Fz
        # The peak force is: Fx_max = D * Fz
        Fx_max = self.mu * D * Fz

        return Fx_max

    def calc_max_lateral_force(self, Fz: Union[float, np.ndarray],
                               axle: str = 'front') -> Union[float, np.ndarray]:
        """
        Calculate maximum lateral force (cornering).

        Args:
            Fz: Normal load [N]
            axle: 'front' or 'rear'

        Returns:
            Fy_max: Maximum lateral force [N]
        """
        # Select appropriate coefficients
        if axle == 'front':
            B, C, D, E = self.B_yf, self.C_yf, self.D_yf, self.E_yf
        else:
            B, C, D, E = self.B_yr, self.C_yr, self.D_yr, self.E_yr

        # Load-dependent peak force
        Fy_max = self.mu * D * Fz

        return Fy_max

    def calc_combined_limits(self, Fz_front: Union[float, np.ndarray],
                             Fz_rear: Union[float, np.ndarray],
                             ay_current: Union[float, np.ndarray]) -> tuple:
        """
        Calculate longitudinal and lateral force limits considering combined slip.

        Args:
            Fz_front: Normal load on front axle [N]
            Fz_rear: Normal load on rear axle [N]
            ay_current: Current lateral acceleration [m/s²]

        Returns:
            (Fx_max_total, Fy_max_total): Maximum forces [N]
        """
        # Calculate individual axle limits
        Fx_max_front = self.calc_max_longitudinal_force(Fz_front, 'front')
        Fx_max_rear = self.calc_max_longitudinal_force(Fz_rear, 'rear')
        Fy_max_front = self.calc_max_lateral_force(Fz_front, 'front')
        Fy_max_rear = self.calc_max_lateral_force(Fz_rear, 'rear')

        # Total vehicle limits (sum of both axles)
        Fx_max_total = Fx_max_front + Fx_max_rear
        Fy_max_total = Fy_max_front + Fy_max_rear

        # Current lateral force
        Fy_current = self.mass * np.abs(ay_current)

        # Combined slip: friction ellipse model
        # sqrt((Fx/Fx_max)^n + (Fy/Fy_max)^n) <= 1
        # Rearrange: Fx_available = Fx_max * sqrt(1 - (Fy/Fy_max)^n)
        n = self.combined_slip_exponent

        # Lateral usage factor
        lateral_usage = np.minimum(Fy_current / Fy_max_total, 1.0)

        # Remaining longitudinal capacity
        longitudinal_capacity = np.power(
            np.maximum(1.0 - np.power(lateral_usage, n), 0.0),
            1.0 / n
        )

        # Available longitudinal force
        Fx_available = Fx_max_total * longitudinal_capacity

        return Fx_available, Fy_max_total

    def calc_max_braking_acceleration(self, ay_current: Union[float, np.ndarray],
                                      ax_current: Union[float, np.ndarray] = 0.0) -> Union[float, np.ndarray]:
        """
        Calculate maximum available braking acceleration considering current lateral acceleration.

        Args:
            ay_current: Current lateral acceleration [m/s²]
            ax_current: Current longitudinal acceleration [m/s²] (for weight transfer)

        Returns:
            ax_max_braking: Maximum braking acceleration [m/s²] (negative value)
        """
        # Calculate normal loads with weight transfer
        Fz_front, Fz_rear = self.calc_normal_loads(ax_current, ay_current)

        # Calculate combined slip limits
        Fx_available, Fy_max = self.calc_combined_limits(
            Fz_front, Fz_rear, ay_current)

        # Convert to acceleration
        ax_max_braking = -Fx_available / self.mass

        return ax_max_braking

    def calc_max_accelerations_g(self, ay_current_g: Union[float, np.ndarray],
                                 ax_current_g: Union[float, np.ndarray] = 0.0) -> tuple:
        """
        Calculate maximum accelerations in multiples of g (for compatibility with fallback).

        Args:
            ay_current_g: Current lateral acceleration [g]
            ax_current_g: Current longitudinal acceleration [g]

        Returns:
            (ax_max_braking_g, ay_max_g): Maximum accelerations [g]
        """
        g = 9.81

        # Convert to m/s²
        ay_current = ay_current_g * g
        ax_current = ax_current_g * g

        # Calculate normal loads and limits
        Fz_front, Fz_rear = self.calc_normal_loads(ax_current, ay_current)

        # Maximum forces
        Fx_available, Fy_max = self.calc_combined_limits(
            Fz_front, Fz_rear, ay_current)

        # Convert to g
        ax_max_braking_g = -Fx_available / self.mass / g
        ay_max_g = Fy_max / self.mass / g

        return ax_max_braking_g, ay_max_g


if __name__ == "__main__":
    """Simple test of the Pacejka tire model."""
    # Test without ROS (use default values)
    import sys

    print("=" * 70)
    print("Pacejka Tire Model Test")
    print("=" * 70)

    # Mock rospy for standalone testing
    class MockRospy:
        @staticmethod
        def get_param(name, default):
            # Use default values from NUC2_pacejka.yaml
            defaults = {
                '/m': 3.54,  # Standard parameter name in vehicle configs
                '/vehicle_mass': 3.54,  # Fallback compatibility
                '/l_f': 0.162,
                '/l_r': 0.145,
                '/h_cg': 0.014,
                '/mu': 1.0,
                '/C_Pf': [4.798521440254997, 2.1640281784621833, 0.6502296853018044, 0.3732212044732381],
                '/C_Pr': [19.999999999999996, 1.4999999999999998, 0.6184183350099146, 1.1322308905491715e-16],
            }
            return defaults.get(name, default)

        @staticmethod
        def loginfo(msg):
            print(f"[INFO] {msg}")

    # Temporarily replace rospy module
    original_rospy = rospy
    import types
    mock_module = types.ModuleType('rospy')
    mock_module.get_param = MockRospy.get_param
    mock_module.loginfo = MockRospy.loginfo

    # Create model with mocked rospy
    globals()['rospy'] = mock_module

    try:
        model = PacejkaTireModel()

        # Test 1: No lateral acceleration (pure braking)
        print("\nTest 1: Pure braking (no cornering)")
        ax_max = model.calc_max_braking_acceleration(ay_current=0.0)
        print(f"  Max braking: {ax_max:.2f} m/s² = {ax_max/9.81:.2f} g")

        # Test 2: With lateral acceleration
        print("\nTest 2: Braking while cornering (1.0g lateral)")
        ax_max = model.calc_max_braking_acceleration(ay_current=1.0*9.81)
        print(f"  Max braking: {ax_max:.2f} m/s² = {ax_max/9.81:.2f} g")

        # Test 3: Higher lateral acceleration
        print("\nTest 3: Braking while cornering (1.5g lateral)")
        ax_max = model.calc_max_braking_acceleration(ay_current=1.5*9.81)
        print(f"  Max braking: {ax_max:.2f} m/s² = {ax_max/9.81:.2f} g")

        # Test 4: Array inputs
        print("\nTest 4: Array of lateral accelerations")
        ay_array = np.array([0.0, 0.5, 1.0, 1.5, 2.0]) * 9.81
        ax_array = model.calc_max_braking_acceleration(ay_current=ay_array)
        for ay, ax in zip(ay_array/9.81, ax_array/9.81):
            print(f"  ay={ay:.1f}g -> ax_max={ax:.2f}g")

        print("\n" + "=" * 70)
        print("Test completed successfully!")
        print("=" * 70)

    finally:
        # Restore original rospy
        globals()['rospy'] = original_rospy
