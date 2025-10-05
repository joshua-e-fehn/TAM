#!/usr/bin/env python3
"""
Track Handler for Global Waypoints Format
Replaces track_handler_py with implementation based on global_waypoints format

This module provides track geometry information using the global waypoints format
instead of the original TAM track_handler_py which uses postprocessed_raceline format.

Uses the existing FrenetConverter from f110_utils for coordinate transformations
to ensure consistency with other components in the system.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Union
import warnings

# Import the existing FrenetConverter
try:
    from frenet_converter.frenet_converter import FrenetConverter
except ImportError:
    # Fallback for different import paths
    try:
        import sys
        import os
        sys.path.append(
            '/home/atlas/catkin_ws/src/race_stack/f110_utils/libs/frenet_conversion/src')
        from frenet_converter.frenet_converter import FrenetConverter
    except ImportError:
        FrenetConverter = None
        warnings.warn(
            "FrenetConverter not available - coordinate conversion will be limited")


class GlobalWaypointsTrackHandler:
    """
    Track handler that works with global_waypoints format instead of track_handler_py.

    Provides the same interface as the original TAM Track class but uses global waypoints
    data structure with fields like s_m, x_m, y_m, d_left, d_right, etc.
    """

    def __init__(self, global_waypoints: Dict = None):
        """
        Initialize track handler with global waypoints data.

        Args:
            global_waypoints: Dictionary with 'wpnts' key containing list of waypoint dicts
                             Each waypoint should have: s_m, x_m, y_m, d_left, d_right, 
                             psi_rad, kappa_radpm, vx_mps, etc.
        """
        self._global_waypoints = None
        self._s_coords = None
        self._x_coords = None
        self._y_coords = None
        self._d_coords = None
        self._d_left_coords = None
        self._d_right_coords = None
        self._psi_coords = None
        self._kappa_coords = None
        self._track_length = None
        self._frenet_converter = None

        # Default track width fallback
        self._default_track_width = 3.0

        if global_waypoints is not None:
            self.update_waypoints(global_waypoints)

    def update_waypoints(self, global_waypoints: Dict):
        """
        Update track handler with new global waypoints data.

        Args:
            global_waypoints: Dictionary with 'wpnts' key containing waypoint list
        """
        if 'wpnts' not in global_waypoints:
            raise ValueError("global_waypoints must contain 'wpnts' key")

        waypoints = global_waypoints['wpnts']
        if len(waypoints) == 0:
            raise ValueError("waypoints list cannot be empty")

        self._global_waypoints = global_waypoints

        # Extract coordinate arrays for efficient interpolation
        self._s_coords = np.array([wp.get('s_m', 0.0) for wp in waypoints])
        self._x_coords = np.array([wp.get('x_m', 0.0) for wp in waypoints])
        self._y_coords = np.array([wp.get('y_m', 0.0) for wp in waypoints])
        # Lateral offset from centerline
        self._d_coords = np.array([wp.get('d_m', 0.0) for wp in waypoints])
        self._d_left_coords = np.array(
            [wp.get('d_left', self._default_track_width/2) for wp in waypoints])
        self._d_right_coords = np.array(
            [wp.get('d_right', self._default_track_width/2) for wp in waypoints])
        self._psi_coords = np.array(
            [wp.get('psi_rad', 0.0) for wp in waypoints])
        self._kappa_coords = np.array(
            [wp.get('kappa_radpm', 0.0) for wp in waypoints])

        # Calculate track length
        self._track_length = self._s_coords[-1] if len(
            self._s_coords) > 0 else 0.0

        # Initialize FrenetConverter if available
        self._initialize_frenet_converter()

        # Validate data
        self._validate_waypoints_data()

    def _initialize_frenet_converter(self):
        """Initialize FrenetConverter with track centerline data."""
        if FrenetConverter is None:
            self._frenet_converter = None
            return

        try:
            if (len(self._x_coords) > 1 and len(self._y_coords) > 1 and
                    len(self._psi_coords) > 1):
                # Initialize with x, y, and psi coordinates
                self._frenet_converter = FrenetConverter(
                    self._x_coords,
                    self._y_coords,
                    self._psi_coords
                )
            elif len(self._x_coords) > 1 and len(self._y_coords) > 1:
                # Initialize with just x, y coordinates
                self._frenet_converter = FrenetConverter(
                    self._x_coords,
                    self._y_coords
                )
            else:
                self._frenet_converter = None
        except Exception as e:
            warnings.warn(f"Failed to initialize FrenetConverter: {e}")
            self._frenet_converter = None

    def _validate_waypoints_data(self):
        """Validate waypoints data for consistency."""
        if len(self._s_coords) < 2:
            warnings.warn("Track has fewer than 2 waypoints")

        # Check for monotonic s coordinates
        if not np.all(np.diff(self._s_coords) >= 0):
            warnings.warn(
                "Arc length coordinates (s_m) are not monotonically increasing")

        # Check for reasonable track widths
        if np.any(self._d_left_coords <= 0) or np.any(self._d_right_coords <= 0):
            warnings.warn(
                "Some track widths are non-positive, using default fallback")

        # Check coordinate validity
        if np.all(self._x_coords == 0) and np.all(self._y_coords == 0):
            warnings.warn(
                "All waypoint coordinates are zero - check waypoint data")

    def s_coord(self) -> np.ndarray:
        """
        Get arc length coordinate array.

        Returns:
            np.ndarray: Array of arc length coordinates [m]
        """
        if self._s_coords is None:
            raise RuntimeError("Track handler not initialized with waypoints")
        return self._s_coords

    def trackwidth_left(self, s: Union[float, np.ndarray] = None) -> Union[float, np.ndarray]:
        """
        Get left track width at given arc length position(s), or entire array if s is None.

        Args:
            s: Arc length position(s) [m]. If None, returns entire array. Can be scalar or array.

        Returns:
            float or np.ndarray: Left track width [m] (positive = available width to the left)
        """
        if self._d_left_coords is None:
            if s is None:
                return np.full(len(self._s_coords) if self._s_coords is not None else 1,
                               self._default_track_width / 2)
            if isinstance(s, np.ndarray):
                return np.full_like(s, self._default_track_width / 2)
            return self._default_track_width / 2

        if s is None:
            return self._d_left_coords.copy()

        result = np.interp(s, self._s_coords,
                           self._d_left_coords, period=self._track_length)
        return result if isinstance(s, np.ndarray) else float(result)

    def trackwidth_right(self, s: Union[float, np.ndarray] = None) -> Union[float, np.ndarray]:
        """
        Get right track width at given arc length position(s), or entire array if s is None.

        Args:
            s: Arc length position(s) [m]. If None, returns entire array. Can be scalar or array.

        Returns:
            float or np.ndarray: Right track width [m] (positive = available width to the right)
        """
        if self._d_right_coords is None:
            if s is None:
                return np.full(len(self._s_coords) if self._s_coords is not None else 1,
                               self._default_track_width / 2)
            if isinstance(s, np.ndarray):
                return np.full_like(s, self._default_track_width / 2)
            return self._default_track_width / 2

        if s is None:
            return self._d_right_coords.copy()

        result = np.interp(s, self._s_coords,
                           self._d_right_coords, period=self._track_length)
        return result if isinstance(s, np.ndarray) else float(result)

    def lateral_offset(self, s: float) -> float:
        """
        Get lateral offset (d_m) from centerline at given arc length position.

        Args:
            s: Arc length position [m]

        Returns:
            float: Lateral offset [m] (positive = left of centerline, negative = right of centerline)
        """
        if self._d_coords is None:
            return 0.0

        return float(np.interp(s, self._s_coords, self._d_coords, period=self._track_length))

    def sn2cartesian(self, s: float, n: float) -> Tuple[float, float]:
        """
        Convert Frenet coordinates (s, n) to Cartesian coordinates (x, y).

        Args:
            s: Arc length position [m]
            n: Lateral offset [m] (positive = left of centerline)

        Returns:
            Tuple[float, float]: (x, y) coordinates [m]
        """
        # Use FrenetConverter if available
        if self._frenet_converter is not None:
            try:
                result = self._frenet_converter.get_cartesian(s, n)
                return float(result[0]), float(result[1])
            except Exception as e:
                warnings.warn(f"FrenetConverter failed, using fallback: {e}")

        # Fallback to manual calculation
        if self._x_coords is None or self._y_coords is None or self._psi_coords is None:
            raise RuntimeError(
                "Track handler not initialized with coordinate data")

        # Interpolate centerline position and heading at s
        x_cl = np.interp(s, self._s_coords, self._x_coords,
                         period=self._track_length)
        y_cl = np.interp(s, self._s_coords, self._y_coords,
                         period=self._track_length)
        psi = np.interp(s, self._s_coords, self._psi_coords,
                        period=self._track_length)

        # Calculate normal vector (points to the left of centerline)
        normal_x = -np.sin(psi)  # Perpendicular to heading, pointing left
        normal_y = np.cos(psi)

        # Convert to Cartesian coordinates
        x = x_cl + n * normal_x
        y = y_cl + n * normal_y

        return float(x), float(y)

    def calc_chi_from_2d_heading(self, s: float, psi_2d: float) -> float:
        """
        Calculate chi angle from 2D heading angle.

        Chi represents the angle between vehicle velocity vector and track tangent.

        Args:
            s: Arc length position [m]
            psi_2d: 2D heading angle [rad]

        Returns:
            float: Chi angle [rad]
        """
        if self._psi_coords is None:
            return 0.0

        # Get track tangent angle at position s
        track_heading = np.interp(
            s, self._s_coords, self._psi_coords, period=self._track_length)

        # Chi is the difference between vehicle heading and track heading
        chi = psi_2d - track_heading

        # Normalize to [-pi, pi]
        chi = np.arctan2(np.sin(chi), np.cos(chi))

        return float(chi)

    def omega_z(self, s: Union[float, np.ndarray] = None) -> Union[float, np.ndarray]:
        """
        Get angular velocity (curvature * velocity) at given arc length position(s), or entire array if s is None.

        For planning purposes, this approximates the yaw rate contribution from track curvature.

        Args:
            s: Arc length position(s) [m]. If None, returns entire array. Can be scalar or array.

        Returns:
            float or np.ndarray: Angular velocity contribution [rad/s per m/s]
        """
        if self._kappa_coords is None:
            if s is None:
                return np.zeros(len(self._s_coords) if self._s_coords is not None else 1)
            if isinstance(s, np.ndarray):
                return np.zeros_like(s)
            return 0.0

        if s is None:
            return self._kappa_coords.copy()

        # Return curvature at position s
        # Note: omega_z = kappa * s_dot, but since we don't have s_dot here,
        # we return just curvature and let the caller multiply by velocity
        result = np.interp(s, self._s_coords,
                           self._kappa_coords, period=self._track_length)

        return result if isinstance(s, np.ndarray) else float(result)

    def get_track_length(self) -> float:
        """
        Get total track length.

        Returns:
            float: Track length [m]
        """
        return self._track_length if self._track_length is not None else 0.0

    def get_waypoint_at_s(self, s: float) -> Dict:
        """
        Get interpolated waypoint data at given arc length position.

        Args:
            s: Arc length position [m]

        Returns:
            Dict: Interpolated waypoint with all available fields
        """
        if self._global_waypoints is None:
            raise RuntimeError("Track handler not initialized")

        waypoint = {}
        waypoint['s_m'] = s
        waypoint['x_m'] = np.interp(
            s, self._s_coords, self._x_coords, period=self._track_length)
        waypoint['y_m'] = np.interp(
            s, self._s_coords, self._y_coords, period=self._track_length)
        waypoint['d_m'] = np.interp(
            s, self._s_coords, self._d_coords, period=self._track_length)
        waypoint['d_left'] = self.trackwidth_left(s)
        waypoint['d_right'] = self.trackwidth_right(s)
        waypoint['psi_rad'] = np.interp(
            s, self._s_coords, self._psi_coords, period=self._track_length)
        waypoint['kappa_radpm'] = np.interp(
            s, self._s_coords, self._kappa_coords, period=self._track_length)

        return waypoint

    def is_initialized(self) -> bool:
        """
        Check if track handler is properly initialized.

        Returns:
            bool: True if initialized with valid waypoints
        """
        return (self._global_waypoints is not None and
                self._s_coords is not None and
                len(self._s_coords) > 0)

    def get_bounds_at_s(self, s: float) -> Tuple[float, float]:
        """
        Get track boundaries (in n-coordinate) at given arc length position.

        Args:
            s: Arc length position [m]

        Returns:
            Tuple[float, float]: (n_right_bound, n_left_bound) in Frenet coordinates [m]
                                n_right_bound is negative (right side)
                                n_left_bound is positive (left side)
        """
        d_right = self.trackwidth_right(s)
        d_left = self.trackwidth_left(s)

        # In Frenet coordinates: negative n = right side, positive n = left side
        n_right_bound = -d_right
        n_left_bound = d_left

        return n_right_bound, n_left_bound

    def set_default_track_width(self, width: float):
        """
        Set default track width used when waypoint data is missing.

        Args:
            width: Default track width [m]
        """
        if width <= 0:
            raise ValueError("Track width must be positive")
        self._default_track_width = width

    def get_frenet_converter(self):
        """
        Get the underlying FrenetConverter instance.

        Returns:
            FrenetConverter or None: The FrenetConverter instance if available
        """
        return self._frenet_converter

    def has_frenet_converter(self) -> bool:
        """
        Check if FrenetConverter is available and initialized.

        Returns:
            bool: True if FrenetConverter is available
        """
        return self._frenet_converter is not None

    def calc_apparent_acceleration(self, s: Union[float, np.ndarray], n: Union[float, np.ndarray],
                                   chi: Union[float, np.ndarray], ax_hat: Union[float, np.ndarray],
                                   ay_hat: Union[float, np.ndarray], V: Union[float, np.ndarray]) -> Tuple:
        """
        Calculate apparent acceleration components for vehicle dynamics.

        This is a simplified implementation that provides the basic functionality
        needed for longitudinal sampling. For more advanced dynamics, a full
        implementation would be needed.

        Args:
            s: Arc length position(s) [m] - can be scalar or array
            n: Lateral offset(s) [m] - can be scalar or array
            chi: Vehicle heading angle(s) relative to track [rad] - can be scalar or array
            ax_hat: Longitudinal acceleration(s) in vehicle frame [m/s²] - can be scalar or array
            ay_hat: Lateral acceleration(s) in vehicle frame [m/s²] - can be scalar or array
            V: Vehicle velocity(ies) [m/s] - can be scalar or array

        Returns:
            Tuple: (ax_tilde, ay_tilde, g_tilde) - same type as inputs (scalar or array)
                ax_tilde: Transformed longitudinal acceleration
                ay_tilde: Transformed lateral acceleration  
                g_tilde: Apparent gravitational acceleration
        """
        # Simplified implementation - in a full implementation this would
        # account for track banking, elevation changes, and coordinate transformations

        # For flat track assumption, the transformations are minimal
        ax_tilde = ax_hat  # No transformation needed for flat track
        ay_tilde = ay_hat  # No transformation needed for flat track

        # Handle both scalar and array inputs for gravity
        if isinstance(s, np.ndarray):
            # Standard gravity for flat track
            g_tilde = np.full_like(s, 9.81, dtype=float)
        else:
            g_tilde = 9.81  # Standard gravity for flat track

        return ax_tilde, ay_tilde, g_tilde

    def calc_acceleration(self, s: float, chi: float, ax_tilde: float, ay_tilde: float) -> Tuple[float, float]:
        """
        Transform accelerations from track-tilted frame back to velocity frame.

        This is the inverse transformation of calc_apparent_acceleration.
        For flat track, this is a simple pass-through.

        Args:
            s: Arc length position [m]
            chi: Vehicle heading angle relative to track [rad]
            ax_tilde: Longitudinal acceleration in track-tilted frame [m/s²]
            ay_tilde: Lateral acceleration in track-tilted frame [m/s²]

        Returns:
            Tuple[float, float]: (ax_hat, ay_hat) accelerations in velocity frame
        """
        # Simplified implementation for flat track
        ax_hat = ax_tilde
        ay_hat = ay_tilde

        return ax_hat, ay_hat

    def d_omega_z(self, s: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """
        Get derivative of angular velocity (rate of change of curvature) with respect to s.

        Args:
            s: Arc length position(s) [m]

        Returns:
            float or np.ndarray: d(omega_z)/ds [rad/m²]
        """
        if self._kappa_coords is None:
            if isinstance(s, np.ndarray):
                return np.zeros_like(s)
            return 0.0

        # Calculate derivative using finite differences
        if isinstance(s, np.ndarray):
            d_kappa = np.gradient(self._kappa_coords, self._s_coords)
            return np.interp(s, self._s_coords, d_kappa, period=self._track_length)
        else:
            d_kappa = np.gradient(self._kappa_coords, self._s_coords)
            return float(np.interp(s, self._s_coords, d_kappa, period=self._track_length))

    def omega_x(self, s: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """
        Get angular velocity around x-axis (roll rate contribution from track banking).

        For flat track, this returns zero.

        Args:
            s: Arc length position(s) [m]

        Returns:
            float or np.ndarray: Angular velocity around x-axis [rad/s per m/s]
        """
        if isinstance(s, np.ndarray):
            return np.zeros_like(s)
        return 0.0

    def omega_y(self, s: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """
        Get angular velocity around y-axis (pitch rate contribution from elevation changes).

        For flat track, this returns zero.

        Args:
            s: Arc length position(s) [m]

        Returns:
            float or np.ndarray: Angular velocity around y-axis [rad/s per m/s]
        """
        if isinstance(s, np.ndarray):
            return np.zeros_like(s)
        return 0.0

    def phi(self, s: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """
        Get track banking angle at given arc length position(s).

        For flat track, this returns zero.

        Args:
            s: Arc length position(s) [m]

        Returns:
            float or np.ndarray: Banking angle [rad]
        """
        if isinstance(s, np.ndarray):
            return np.zeros_like(s)
        return 0.0

    def mu(self, s: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """
        Get track pitch/elevation angle at given arc length position(s).

        For flat track, this returns zero.

        Args:
            s: Arc length position(s) [m]

        Returns:
            float or np.ndarray: Pitch angle [rad]
        """
        if isinstance(s, np.ndarray):
            return np.zeros_like(s)
        return 0.0

    def calc_2d_heading_from_chi(self, s: Union[float, np.ndarray], chi: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """
        Calculate 2D heading angle (psi) from chi angle and track position.

        Chi represents the angle between vehicle velocity and track tangent.
        Psi is the absolute heading angle in global frame.

        Args:
            s: Arc length position(s) [m]
            chi: Chi angle(s) [rad]

        Returns:
            float or np.ndarray: 2D heading angle(s) [rad]
        """
        if self._psi_coords is None:
            if isinstance(s, np.ndarray):
                return chi
            return float(chi)

        # Get track tangent angle at position s
        if isinstance(s, np.ndarray):
            track_heading = np.interp(
                s, self._s_coords, self._psi_coords, period=self._track_length)
            psi_2d = track_heading + chi
            # Normalize to [-pi, pi]
            return np.arctan2(np.sin(psi_2d), np.cos(psi_2d))
        else:
            track_heading = np.interp(
                s, self._s_coords, self._psi_coords, period=self._track_length)
            psi_2d = track_heading + chi
            # Normalize to [-pi, pi]
            return float(np.arctan2(np.sin(psi_2d), np.cos(psi_2d)))

    def angles_to_velocity_frame(self, s: Union[float, np.ndarray], chi: Union[float, np.ndarray]) -> np.ndarray:
        """
        Calculate Euler angles (yaw, pitch, roll) from inertial to velocity frame.

        For flat track, this primarily involves the yaw angle transformation.

        Args:
            s: Arc length position(s) [m]
            chi: Chi angle(s) [rad]

        Returns:
            np.ndarray: Euler angles as [yaw, pitch, roll] or array of such if vectorized
        """
        # For flat track, velocity frame transformation is primarily yaw
        psi = self.calc_2d_heading_from_chi(s, chi)

        if isinstance(s, np.ndarray):
            # Return array of [yaw, pitch, roll] for each position
            n_points = len(s) if hasattr(s, '__len__') else 1
            angles = np.zeros((n_points, 3))
            angles[:, 0] = psi  # yaw
            angles[:, 1] = 0.0  # pitch (flat track)
            angles[:, 2] = 0.0  # roll (flat track)
            return angles
        else:
            # Return single [yaw, pitch, roll]
            return np.array([psi, 0.0, 0.0])


# Compatibility alias for easy replacement
Track = GlobalWaypointsTrackHandler


def create_track_handler_from_global_waypoints(global_waypoints: Dict) -> GlobalWaypointsTrackHandler:
    """
    Factory function to create track handler from global waypoints.

    Args:
        global_waypoints: Dictionary with 'wpnts' key containing waypoint list

    Returns:
        GlobalWaypointsTrackHandler: Initialized track handler
    """
    return GlobalWaypointsTrackHandler(global_waypoints)
