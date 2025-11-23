#!/usr/bin/env python3
"""
TAM Calculation Costs Module
Multi-objective cost function calculations following original TAM approach

Ported from tam_race_stack/mod_planning/sampling_planner/sampling_planner/calculation_costs.py

ROS Parameters (relative namespace):
- costs/curvature_cost_weight (default: 500000.0): Weight for curvature cost
- costs/curvature_cost_threshold (default: 30.0): Speed threshold for curvature cost
- costs/raceline_cost_weight (default: 3.5): Weight for raceline deviation cost
- costs/velocity_cost_weight (default: 3.0): Weight for velocity cost
- costs/friction_cost_weight (default: 5000.0): Weight for friction violation cost
- costs/lateral_jerk_cost_weight (default: 0.0): Weight for lateral jerk cost
- overtaking_weights/raceline_cost_weight_overtaking (default: 2.0): Raceline cost weight when overtaking
- overtaking_weights/velocity_cost_weight_overtaking (default: 8.0): Velocity cost weight when overtaking
- overtaking_weights/lateral_jerk_cost_weight_overtaking (default: 0.0): Lateral jerk cost weight when overtaking
- costs/prediction_cost_weight (default: 100000.0): Weight for prediction cost
- costs/additional_absolute_sample_cost (default: 50.0): Additional cost for absolute samples
- costs/collision_cost_weight (default: 100000000.0): Weight for collision cost
- behavior/horizon (default: 4.0): Planning horizon in seconds
- costs/prediction_s_factor_min_size (default: 0.03): Minimum longitudinal prediction factor
- costs/prediction_s_factor_max_size (default: 0.012): Maximum longitudinal prediction factor
- costs/prediction_s_asym_scaling (default: 1.5): Asymmetric scaling for prediction
- costs/prediction_n_factor (default: 0.2): Lateral prediction factor
- costs/prediction_s_factor_defender (default: 0.03): Longitudinal prediction factor for defender
- costs/prediction_n_factor_defender (default: 0.2): Lateral prediction factor for defender
- costs/prediction_s_factor_static (default: 0.05): Longitudinal prediction factor for static objects
- costs/prediction_n_factor_static (default: 0.35): Lateral prediction factor for static objects
- costs/prediction_uncertainty_weight (default: 4.0): Weight for prediction uncertainty
- costs/increasing_rl_cost (default: True): Whether to use increasing raceline cost over horizon
- costs/velocity_excess_cost_multiplier (default: 2.0): Multiplier for velocity excess cost
- behavior/max_deceleration_on_target_change (default: 5.0): Maximum deceleration on target change
- costs/V_diff_max_costs (default: 15.0): Maximum velocity difference costs
- behavior/collision_check_horizon_s (default: 1.5): Collision check horizon in seconds
- behavior/tube_width (default: 1.0): Tube width for collision checking
- safety_distances/safety_distance_vehicles (default: 0.0): Safety distance to other vehicles
"""

from track_handler_global_waypoints import GlobalWaypointsTrackHandler
from simple_helper_utils import interpolate_with_period
import numpy as np
from dataclasses import dataclass
import rospy


@dataclass(init=False)
class CalculationCostsParams():

    curvature_cost_weight: float
    curvature_cost_threshold: float
    raceline_cost_weight: float
    velocity_cost_weight: float
    friction_cost_weight: float
    lateral_jerk_cost_weight: float
    raceline_cost_weight_overtaking: float
    velocity_cost_weight_overtaking: float
    lateral_jerk_cost_weight_overtaking: float
    prediction_cost_weight: float
    additional_absolute_sample_cost: float
    collision_cost_weight: float
    horizon: float
    prediction_s_factor_min_size: float
    prediction_s_factor_max_size: float
    prediction_s_asym_scaling: float
    prediction_n_factor: float
    prediction_s_factor_defender: float
    prediction_n_factor_defender: float
    prediction_s_factor_static: float
    prediction_n_factor_static: float
    prediction_uncertainty_weight: float
    increasing_rl_cost: bool
    velocity_excess_cost_multiplier: float
    max_deceleration_on_target_change: float
    V_diff_max_costs: float
    collision_check_horizon_s: float
    tube_width: float
    safety_distance_vehicles: float


class CalculationCosts():
    def __init__(self, debugging=False):
        """
        Initialize CalculationCosts with ROS parameters.

        Args:
            params: Deprecated - parameters are now loaded from ROS parameter server
            debugging: Enable debug output
        """
        self.params = CalculationCostsParams()
        self.debugging = debugging
        self.initialized_params = False
        skip_update = getattr(self, '_skip_param_updates', False)
        self.declare_and_update_parameters(skip_update=skip_update)

    def _convert_global_waypoints_to_prediction(self, prediction_waypoints, time_horizon=None, dt=0.1):
        """
        Convert global waypoints format to time-series prediction format with proper temporal interpolation.

        This implementation handles:
        - Variable velocity between waypoints
        - Proper time-based interpolation
        - Acceleration/deceleration dynamics
        - Configurable temporal resolution

        Args:
            prediction_waypoints: Dict with "wpnts" key containing waypoint list
            time_horizon: Time horizon for prediction (optional)
            dt: Time step for interpolation [s]

        Returns:
            Dict with time-series arrays for s, n, vel, time_w_offset
        """
        if "wpnts" not in prediction_waypoints:
            return prediction_waypoints  # Return as-is if not global waypoints format

        waypoints = prediction_waypoints["wpnts"]
        if len(waypoints) == 0:
            return {"s": [], "n": [], "vel": [], "time_w_offset": []}

        # Extract data from waypoints
        s_coords = np.array([wp.get("s_m", 0.0) for wp in waypoints])
        n_coords = np.array([wp.get("d_m", 0.0)
                            for wp in waypoints])  # d_m is lateral offset
        vel_coords = np.array([wp.get("vx_mps", 0.0) for wp in waypoints])

        # Handle edge cases
        if len(waypoints) == 1:
            return {
                "s": s_coords,
                "n": n_coords,
                "vel": vel_coords,
                "time_w_offset": np.array([0.0])
            }

        if time_horizon is None:
            time_horizon = self.params.horizon

        # Calculate proper time coordinates using physics-based integration
        time_coords = self._calculate_temporal_coordinates(
            s_coords, vel_coords)

        # Determine actual prediction time span
        total_prediction_time = time_coords[-1] if len(
            time_coords) > 0 else time_horizon
        actual_horizon = min(total_prediction_time, time_horizon)

        # Create high-resolution time grid for interpolation
        time_grid = np.arange(0, actual_horizon + dt, dt)

        # Interpolate all quantities on the time grid with proper handling of dynamics
        s_interp = self._temporal_interpolate(time_coords, s_coords, time_grid,
                                              extrapolation='linear')
        n_interp = self._temporal_interpolate(time_coords, n_coords, time_grid,
                                              extrapolation='constant')
        vel_interp = self._temporal_interpolate(time_coords, vel_coords, time_grid,
                                                extrapolation='constant')

        return {
            "s": s_interp,
            "n": n_interp,
            "vel": vel_interp,
            "time_w_offset": time_grid
        }

    def _calculate_temporal_coordinates(self, s_coords, vel_coords):
        """
        Calculate time coordinates using physics-based integration.

        Properly handles variable velocity between waypoints by integrating ds/v.

        Args:
            s_coords: Arc length coordinates [m]
            vel_coords: Velocity at each waypoint [m/s]

        Returns:
            np.ndarray: Time coordinates [s]
        """
        if len(s_coords) <= 1:
            return np.array([0.0])

        time_coords = np.zeros_like(s_coords)

        for i in range(1, len(s_coords)):
            ds = s_coords[i] - s_coords[i-1]

            # Handle zero or very small velocities
            v_prev = max(vel_coords[i-1], 0.1)  # Minimum 0.1 m/s
            v_curr = max(vel_coords[i], 0.1)

            if abs(ds) < 1e-6:  # Very small distance
                dt = 0.0
            elif abs(v_curr - v_prev) < 1e-3:  # Constant velocity
                dt = ds / ((v_prev + v_curr) / 2.0)
            else:  # Variable velocity - use physics integration
                # For linear velocity change: v(t) = v0 + a*t
                # where a = (v1 - v0) / dt, and s = v0*t + 0.5*a*t^2
                # Solving for t: at^2 + 2*v0*t - 2*s = 0
                # We'll solve for dt where this acceleration occurs
                a = (v_curr - v_prev)

                if abs(a) < 1e-3:  # Nearly constant acceleration
                    dt = ds / ((v_prev + v_curr) / 2.0)
                else:
                    # Quadratic formula: t = (-2*v0 + sqrt(4*v0^2 + 8*a*s)) / (2*a)
                    discriminant = 4 * v_prev**2 + 8 * a * ds
                    if discriminant >= 0:
                        dt = (-2 * v_prev + np.sqrt(discriminant)) / (2 * a)
                        dt = max(dt, 0.01)  # Minimum time step
                    else:
                        # Fallback to average velocity
                        dt = ds / ((v_prev + v_curr) / 2.0)

            time_coords[i] = time_coords[i-1] + abs(dt)

        return time_coords

    def _temporal_interpolate(self, time_coords, values, time_grid, extrapolation='constant'):
        """
        Perform temporal interpolation with proper extrapolation handling.

        Args:
            time_coords: Original time coordinates
            values: Values to interpolate
            time_grid: Target time grid
            extrapolation: How to handle extrapolation ('constant', 'linear', 'zero')

        Returns:
            np.ndarray: Interpolated values
        """
        if len(time_coords) == 0 or len(values) == 0:
            return np.zeros_like(time_grid)

        if len(time_coords) == 1:
            # Single point - return constant value
            return np.full_like(time_grid, values[0])

        # Perform interpolation within the time range
        interpolated = np.interp(time_grid, time_coords, values)

        # Handle extrapolation beyond the prediction time range
        if extrapolation == 'constant':
            # Use last known value for extrapolation
            beyond_mask = time_grid > time_coords[-1]
            interpolated[beyond_mask] = values[-1]

            before_mask = time_grid < time_coords[0]
            interpolated[before_mask] = values[0]

        elif extrapolation == 'zero':
            # Set extrapolated values to zero
            beyond_mask = time_grid > time_coords[-1]
            interpolated[beyond_mask] = 0.0

            before_mask = time_grid < time_coords[0]
            interpolated[before_mask] = 0.0

        elif extrapolation == 'linear':
            # Linear extrapolation using last two points
            if len(time_coords) >= 2:
                beyond_mask = time_grid > time_coords[-1]
                if np.any(beyond_mask):
                    # Calculate slope from last two points
                    dt = time_coords[-1] - time_coords[-2]
                    dv = values[-1] - values[-2]
                    if dt > 1e-6:
                        slope = dv / dt
                        extrapolated_times = time_grid[beyond_mask] - \
                            time_coords[-1]
                        interpolated[beyond_mask] = values[-1] + \
                            slope * extrapolated_times

                before_mask = time_grid < time_coords[0]
                if np.any(before_mask):
                    # Calculate slope from first two points
                    dt = time_coords[1] - time_coords[0]
                    dv = values[1] - values[0]
                    if dt > 1e-6:
                        slope = dv / dt
                        extrapolated_times = time_grid[before_mask] - \
                            time_coords[0]
                        interpolated[before_mask] = values[0] + \
                            slope * extrapolated_times

        return interpolated

    def _convert_global_waypoints_to_prediction_advanced(self, prediction_waypoints, time_horizon=None, dt=0.1,
                                                         prediction_type="dynamic", uncertainty_growth=0.1):
        """
        Advanced conversion with uncertainty modeling and prediction type handling.

        This version adds:
        - Prediction uncertainty that grows over time
        - Different handling for static vs dynamic objects
        - Smooth trajectory generation using spline interpolation
        - Physics-consistent motion modeling

        Args:
            prediction_waypoints: Dict with "wpnts" key containing waypoint list
            time_horizon: Time horizon for prediction [s]
            dt: Time step for interpolation [s]
            prediction_type: "static", "dynamic", or "estimated"
            uncertainty_growth: Rate of uncertainty growth over time [m/s]

        Returns:
            Dict with time-series arrays including uncertainty bounds
        """
        if "wpnts" not in prediction_waypoints:
            return prediction_waypoints

        waypoints = prediction_waypoints["wpnts"]
        if len(waypoints) == 0:
            return {"s": [], "n": [], "vel": [], "time_w_offset": [],
                    "s_uncertainty": [], "n_uncertainty": []}

        # Extract waypoint data
        s_coords = np.array([wp.get("s_m", 0.0) for wp in waypoints])
        n_coords = np.array([wp.get("d_m", 0.0) for wp in waypoints])
        vel_coords = np.array([wp.get("vx_mps", 0.0) for wp in waypoints])

        # Get additional fields if available
        curvature = np.array([wp.get("kappa_radpm", 0.0) for wp in waypoints])
        acceleration = np.array([wp.get("ax_mps2", 0.0) for wp in waypoints])

        if time_horizon is None:
            time_horizon = self.params.horizon

        # Handle different prediction types
        if prediction_type == "static":
            # Static objects: simple constant extrapolation
            time_grid = np.arange(0, time_horizon + dt, dt)
            s_interp = np.full_like(
                time_grid, s_coords[0] if len(s_coords) > 0 else 0.0)
            n_interp = np.full_like(
                time_grid, n_coords[0] if len(n_coords) > 0 else 0.0)
            vel_interp = np.zeros_like(time_grid)

            # Static objects have low positional uncertainty but it still grows
            s_uncertainty = uncertainty_growth * 0.1 * time_grid
            n_uncertainty = uncertainty_growth * 0.1 * time_grid

        else:
            # Dynamic objects: physics-based prediction
            time_coords = self._calculate_temporal_coordinates_advanced(
                s_coords, vel_coords, acceleration, curvature)

            total_prediction_time = time_coords[-1] if len(
                time_coords) > 0 else time_horizon
            actual_horizon = min(total_prediction_time, time_horizon)
            time_grid = np.arange(0, actual_horizon + dt, dt)

            # Use spline interpolation for smoother trajectories
            try:
                from scipy.interpolate import CubicSpline, UnivariateSpline

                if len(time_coords) >= 4:  # Cubic spline needs at least 4 points
                    s_spline = CubicSpline(
                        time_coords, s_coords, extrapolate=True)
                    n_spline = CubicSpline(
                        time_coords, n_coords, extrapolate=True)
                    vel_spline = CubicSpline(
                        time_coords, vel_coords, extrapolate=True)

                    s_interp = s_spline(time_grid)
                    n_interp = n_spline(time_grid)
                    # Ensure non-negative velocity
                    vel_interp = np.maximum(vel_spline(time_grid), 0.0)

                elif len(time_coords) >= 2:
                    # Linear interpolation fallback
                    s_interp = np.interp(time_grid, time_coords, s_coords)
                    n_interp = np.interp(time_grid, time_coords, n_coords)
                    vel_interp = np.interp(time_grid, time_coords, vel_coords)
                else:
                    # Single point
                    s_interp = np.full_like(time_grid, s_coords[0])
                    n_interp = np.full_like(time_grid, n_coords[0])
                    vel_interp = np.full_like(time_grid, vel_coords[0])

            except ImportError:
                # Fallback to linear interpolation if scipy not available
                s_interp = self._temporal_interpolate(
                    time_coords, s_coords, time_grid, 'linear')
                n_interp = self._temporal_interpolate(
                    time_coords, n_coords, time_grid, 'constant')
                vel_interp = self._temporal_interpolate(
                    time_coords, vel_coords, time_grid, 'constant')

            # Model prediction uncertainty growth
            base_uncertainty_s = 0.5  # Base longitudinal uncertainty [m]
            base_uncertainty_n = 0.3  # Base lateral uncertainty [m]

            # Uncertainty grows quadratically with time (motion model uncertainty)
            s_uncertainty = base_uncertainty_s + uncertainty_growth * time_grid**2
            n_uncertainty = base_uncertainty_n + uncertainty_growth * 0.5 * time_grid**2

            # Add velocity-dependent uncertainty
            avg_vel = np.mean(vel_interp) if len(vel_interp) > 0 else 10.0
            # Higher uncertainty at higher speeds
            vel_factor = np.sqrt(avg_vel / 10.0)
            s_uncertainty *= vel_factor
            n_uncertainty *= vel_factor

        return {
            "s": s_interp,
            "n": n_interp,
            "vel": vel_interp,
            "time_w_offset": time_grid,
            "s_uncertainty": s_uncertainty,
            "n_uncertainty": n_uncertainty,
            "prediction_type": prediction_type
        }

    def _calculate_temporal_coordinates_advanced(self, s_coords, vel_coords, acceleration, curvature):
        """
        Advanced temporal coordinate calculation using acceleration and curvature information.

        Args:
            s_coords: Arc length coordinates [m]
            vel_coords: Velocity at each waypoint [m/s]
            acceleration: Longitudinal acceleration [m/s²]
            curvature: Path curvature [rad/m]

        Returns:
            np.ndarray: Time coordinates [s]
        """
        if len(s_coords) <= 1:
            return np.array([0.0])

        time_coords = np.zeros_like(s_coords)

        for i in range(1, len(s_coords)):
            ds = s_coords[i] - s_coords[i-1]
            v_prev = max(vel_coords[i-1], 0.1)
            v_curr = max(vel_coords[i], 0.1)

            # Use acceleration information if available
            if len(acceleration) > i and abs(acceleration[i-1]) > 1e-3:
                a = acceleration[i-1]

                # Kinematic equation: v_f^2 = v_i^2 + 2*a*ds
                # Solve for time: t = (v_f - v_i) / a
                dv = v_curr - v_prev

                if abs(a) > 1e-3:
                    dt = dv / a

                    # Verify with position equation: ds = v_i*t + 0.5*a*t^2
                    expected_ds = v_prev * dt + 0.5 * a * dt**2

                    # If inconsistent, fall back to average velocity
                    if abs(expected_ds - ds) > 0.1 * abs(ds):
                        dt = ds / ((v_prev + v_curr) / 2.0)
                else:
                    dt = ds / ((v_prev + v_curr) / 2.0)

                dt = max(abs(dt), 0.01)  # Minimum time step
            else:
                # No acceleration info, use physics-based calculation
                dt = abs(ds) / ((v_prev + v_curr) / 2.0)

            time_coords[i] = time_coords[i-1] + dt

        return time_coords

    def _load_yaml_defaults(self):
        """Load default parameters from tam_sampling_params.yaml"""
        import rospkg
        import yaml
        import os
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('tam_sampling_planner')
            config_file = os.path.join(
                pkg_path, 'config', 'tam_sampling_params.yaml')

            with open(config_file, 'r') as f:
                yaml_params = yaml.safe_load(f)
                return yaml_params if yaml_params else {}
        except Exception as e:
            rospy.logwarn(
                f"CalculationCosts: Could not load YAML defaults: {e}")
            return {}

    def declare_and_update_parameters(self, skip_update=False):
        if skip_update:
            return
        
        if not self.initialized_params:
            yaml_defaults = self._load_yaml_defaults()

            # Load cost parameters
            self.params.curvature_cost_weight = yaml_defaults.get(
                'curvature_cost_weight',
                rospy.get_param("costs/curvature_cost_weight", 500000.0))
            rospy.set_param("costs/curvature_cost_weight",
                            self.params.curvature_cost_weight)
            self.params.curvature_cost_threshold = yaml_defaults.get(
                'curvature_cost_threshold',
                rospy.get_param("costs/curvature_cost_threshold", 30.0))
            rospy.set_param("costs/curvature_cost_threshold",
                            self.params.curvature_cost_threshold)
            self.params.raceline_cost_weight = yaml_defaults.get(
                'raceline_cost_weight',
                rospy.get_param("costs/raceline_cost_weight", 3.5))
            rospy.set_param("costs/raceline_cost_weight",
                            self.params.raceline_cost_weight)
            self.params.velocity_cost_weight = yaml_defaults.get(
                'velocity_cost_weight',
                rospy.get_param("costs/velocity_cost_weight", 3.0))
            rospy.set_param("costs/velocity_cost_weight",
                            self.params.velocity_cost_weight)
            self.params.friction_cost_weight = yaml_defaults.get(
                'friction_cost_weight',
                rospy.get_param("costs/friction_cost_weight", 5000.0))
            rospy.set_param("costs/friction_cost_weight",
                            self.params.friction_cost_weight)
            self.params.lateral_jerk_cost_weight = yaml_defaults.get(
                'lateral_jerk_cost_weight',
                rospy.get_param("costs/lateral_jerk_cost_weight", 0.0))
            rospy.set_param("costs/lateral_jerk_cost_weight",
                            self.params.lateral_jerk_cost_weight)

            # Overtaking weight parameters
            self.params.raceline_cost_weight_overtaking = yaml_defaults.get(
                'raceline_cost_weight_overtaking',
                rospy.get_param("overtaking_weights/raceline_cost_weight_overtaking", 2.0))
            rospy.set_param("overtaking_weights/raceline_cost_weight_overtaking",
                            self.params.raceline_cost_weight_overtaking)
            self.params.velocity_cost_weight_overtaking = yaml_defaults.get(
                'velocity_cost_weight_overtaking',
                rospy.get_param("overtaking_weights/velocity_cost_weight_overtaking", 8.0))
            rospy.set_param("overtaking_weights/velocity_cost_weight_overtaking",
                            self.params.velocity_cost_weight_overtaking)
            self.params.lateral_jerk_cost_weight_overtaking = yaml_defaults.get(
                'lateral_jerk_cost_weight_overtaking',
                rospy.get_param("overtaking_weights/lateral_jerk_cost_weight_overtaking", 0.0))
            rospy.set_param("overtaking_weights/lateral_jerk_cost_weight_overtaking",
                            self.params.lateral_jerk_cost_weight_overtaking)

            # Prediction and collision cost parameters
            self.params.prediction_cost_weight = yaml_defaults.get(
                'prediction_cost_weight',
                rospy.get_param("costs/prediction_cost_weight", 100000.0))
            rospy.set_param("costs/prediction_cost_weight",
                            self.params.prediction_cost_weight)
            self.params.additional_absolute_sample_cost = yaml_defaults.get(
                'additional_absolute_sample_cost',
                rospy.get_param("costs/additional_absolute_sample_cost", 50.0))
            rospy.set_param("costs/additional_absolute_sample_cost",
                            self.params.additional_absolute_sample_cost)
            self.params.collision_cost_weight = yaml_defaults.get(
                'collision_cost_weight',
                rospy.get_param("costs/collision_cost_weight", 100000000.0))
            rospy.set_param("costs/collision_cost_weight",
                            self.params.collision_cost_weight)

            # Behavior parameters
            self.params.horizon = yaml_defaults.get(
                'planning_horizon',
                rospy.get_param("behavior/horizon", 4.0))
            rospy.set_param("behavior/horizon", self.params.horizon)
            self.params.max_deceleration_on_target_change = yaml_defaults.get(
                'max_deceleration_on_target_change',
                rospy.get_param("behavior/max_deceleration_on_target_change", 5.0))
            rospy.set_param("behavior/max_deceleration_on_target_change",
                            self.params.max_deceleration_on_target_change)
            self.params.collision_check_horizon_s = yaml_defaults.get(
                'collision_check_horizon_s',
                rospy.get_param("behavior/collision_check_horizon_s", 1.5))
            rospy.set_param("behavior/collision_check_horizon_s",
                            self.params.collision_check_horizon_s)
            self.params.tube_width = yaml_defaults.get(
                'tube_width',
                rospy.get_param("behavior/tube_width", 1.0))
            rospy.set_param("behavior/tube_width", self.params.tube_width)

            # Prediction factor parameters
            self.params.prediction_s_factor_min_size = yaml_defaults.get(
                'prediction_s_factor_min_size',
                rospy.get_param("costs/prediction_s_factor_min_size", 0.03))
            rospy.set_param("costs/prediction_s_factor_min_size",
                            self.params.prediction_s_factor_min_size)
            self.params.prediction_s_factor_max_size = yaml_defaults.get(
                'prediction_s_factor_max_size',
                rospy.get_param("costs/prediction_s_factor_max_size", 0.012))
            rospy.set_param("costs/prediction_s_factor_max_size",
                            self.params.prediction_s_factor_max_size)
            self.params.prediction_s_asym_scaling = yaml_defaults.get(
                'prediction_s_asym_scaling',
                rospy.get_param("costs/prediction_s_asym_scaling", 1.5))
            rospy.set_param("costs/prediction_s_asym_scaling",
                            self.params.prediction_s_asym_scaling)
            self.params.prediction_n_factor = yaml_defaults.get(
                'prediction_n_factor',
                rospy.get_param("costs/prediction_n_factor", 0.2))
            rospy.set_param("costs/prediction_n_factor",
                            self.params.prediction_n_factor)
            self.params.prediction_s_factor_defender = yaml_defaults.get(
                'prediction_s_factor_defender',
                rospy.get_param("costs/prediction_s_factor_defender", 0.03))
            rospy.set_param("costs/prediction_s_factor_defender",
                            self.params.prediction_s_factor_defender)
            self.params.prediction_n_factor_defender = yaml_defaults.get(
                'prediction_n_factor_defender',
                rospy.get_param("costs/prediction_n_factor_defender", 0.2))
            rospy.set_param("costs/prediction_n_factor_defender",
                            self.params.prediction_n_factor_defender)
            self.params.prediction_s_factor_static = yaml_defaults.get(
                'prediction_s_factor_static',
                rospy.get_param("costs/prediction_s_factor_static", 0.05))
            rospy.set_param("costs/prediction_s_factor_static",
                            self.params.prediction_s_factor_static)
            self.params.prediction_n_factor_static = yaml_defaults.get(
                'prediction_n_factor_static',
                rospy.get_param("costs/prediction_n_factor_static", 0.35))
            rospy.set_param("costs/prediction_n_factor_static",
                            self.params.prediction_n_factor_static)
            self.params.prediction_uncertainty_weight = yaml_defaults.get(
                'prediction_uncertainty_weight',
                rospy.get_param("costs/prediction_uncertainty_weight", 4.0))
            rospy.set_param("costs/prediction_uncertainty_weight",
                            self.params.prediction_uncertainty_weight)

            # Additional cost parameters
            self.params.increasing_rl_cost = yaml_defaults.get(
                'increasing_rl_cost',
                rospy.get_param("costs/increasing_rl_cost", True))
            rospy.set_param("costs/increasing_rl_cost",
                            self.params.increasing_rl_cost)
            self.params.velocity_excess_cost_multiplier = yaml_defaults.get(
                'velocity_excess_cost_multiplier',
                rospy.get_param("costs/velocity_excess_cost_multiplier", 2.0))
            rospy.set_param("costs/velocity_excess_cost_multiplier",
                            self.params.velocity_excess_cost_multiplier)
            self.params.V_diff_max_costs = yaml_defaults.get(
                'V_diff_max_costs',
                rospy.get_param("costs/V_diff_max_costs", 15.0))
            rospy.set_param("costs/V_diff_max_costs",
                            self.params.V_diff_max_costs)

            # Safety distance parameters
            self.params.safety_distance_vehicles = yaml_defaults.get(
                'safety_distance_vehicles',
                rospy.get_param("safety_distances/safety_distance_vehicles", 0.0))
            rospy.set_param("safety_distances/safety_distance_vehicles",
                            self.params.safety_distance_vehicles)
            self.initialized_params = True
        else:
            # Load cost parameters
            self.params.curvature_cost_weight = rospy.get_param(
                "costs/curvature_cost_weight",
                self.params.curvature_cost_weight)
            self.params.curvature_cost_threshold = rospy.get_param(
                "costs/curvature_cost_threshold",
                self.params.curvature_cost_threshold)
            self.params.raceline_cost_weight = rospy.get_param(
                "costs/raceline_cost_weight",
                self.params.raceline_cost_weight)
            self.params.velocity_cost_weight = rospy.get_param(
                "costs/velocity_cost_weight",
                self.params.velocity_cost_weight)
            self.params.friction_cost_weight = rospy.get_param(
                "costs/friction_cost_weight",
                self.params.friction_cost_weight)
            self.params.lateral_jerk_cost_weight = rospy.get_param(
                "costs/lateral_jerk_cost_weight",
                self.params.lateral_jerk_cost_weight)

            # Overtaking weight parameters
            self.params.raceline_cost_weight_overtaking = rospy.get_param(
                "overtaking_weights/raceline_cost_weight_overtaking",
                self.params.raceline_cost_weight_overtaking)
            self.params.velocity_cost_weight_overtaking = rospy.get_param(
                "overtaking_weights/velocity_cost_weight_overtaking",
                self.params.velocity_cost_weight_overtaking)
            self.params.lateral_jerk_cost_weight_overtaking = rospy.get_param(
                "overtaking_weights/lateral_jerk_cost_weight_overtaking",
                self.params.lateral_jerk_cost_weight_overtaking)

            # Prediction and collision cost parameters
            self.params.prediction_cost_weight = rospy.get_param(
                "costs/prediction_cost_weight",
                self.params.prediction_cost_weight)
            self.params.additional_absolute_sample_cost = rospy.get_param(
                "costs/additional_absolute_sample_cost",
                self.params.additional_absolute_sample_cost)
            self.params.collision_cost_weight = rospy.get_param(
                "costs/collision_cost_weight",
                self.params.collision_cost_weight)

            # Behavior parameters
            self.params.horizon = rospy.get_param(
                "behavior/horizon", self.params.horizon)
            self.params.max_deceleration_on_target_change = rospy.get_param(
                "behavior/max_deceleration_on_target_change",
                self.params.max_deceleration_on_target_change)
            self.params.collision_check_horizon_s = rospy.get_param(
                "behavior/collision_check_horizon_s",
                self.params.collision_check_horizon_s)
            self.params.tube_width = rospy.get_param(
                "behavior/tube_width", self.params.tube_width)

            # Prediction factor parameters
            self.params.prediction_s_factor_min_size = rospy.get_param(
                "costs/prediction_s_factor_min_size",
                self.params.prediction_s_factor_min_size)
            self.params.prediction_s_factor_max_size = rospy.get_param(
                "costs/prediction_s_factor_max_size",
                self.params.prediction_s_factor_max_size)
            self.params.prediction_s_asym_scaling = rospy.get_param(
                "costs/prediction_s_asym_scaling",
                self.params.prediction_s_asym_scaling)
            self.params.prediction_n_factor = rospy.get_param(
                "costs/prediction_n_factor",
                self.params.prediction_n_factor)
            self.params.prediction_s_factor_defender = rospy.get_param(
                "costs/prediction_s_factor_defender",
                self.params.prediction_s_factor_defender)
            self.params.prediction_n_factor_defender = rospy.get_param(
                "costs/prediction_n_factor_defender",
                self.params.prediction_n_factor_defender)
            self.params.prediction_s_factor_static = rospy.get_param(
                "costs/prediction_s_factor_static",
                self.params.prediction_s_factor_static)
            self.params.prediction_n_factor_static = rospy.get_param(
                "costs/prediction_n_factor_static",
                self.params.prediction_n_factor_static)
            self.params.prediction_uncertainty_weight = rospy.get_param(
                "costs/prediction_uncertainty_weight",
                self.params.prediction_uncertainty_weight)

            # Additional cost parameters
            self.params.increasing_rl_cost = rospy.get_param(
                "costs/increasing_rl_cost",
                self.params.increasing_rl_cost)
            self.params.velocity_excess_cost_multiplier = rospy.get_param(
                "costs/velocity_excess_cost_multiplier",
                self.params.velocity_excess_cost_multiplier)
            self.params.V_diff_max_costs = rospy.get_param(
                "costs/V_diff_max_costs",
                self.params.V_diff_max_costs)

            # Safety distance parameters
            self.params.safety_distance_vehicles = rospy.get_param(
                "safety_distances/safety_distance_vehicles",
                self.params.safety_distance_vehicles)

    def set_action_space_parameters(self, parameters):
        self.params.curvature_cost_weight = parameters.curvature_cost_weight
        self.params.raceline_cost_weight = parameters.raceline_cost_weight
        self.params.velocity_cost_weight = parameters.velocity_cost_weight
        self.params.friction_cost_weight = parameters.friction_cost_weight
        self.params.collision_cost_weight = parameters.collision_cost_weight
        # print("new parameters set")

    def debug_params(self):
        print(
            "*****************************************************************************")
        print(f"Curvature Cost Weight: {self.params.curvature_cost_weight}")
        print(f"Raceline Cost Weight: {self.params.raceline_cost_weight}")
        print(f"Velocity Cost Weight: {self.params.velocity_cost_weight}")
        print(f"Friction Cost Weight: {self.params.friction_cost_weight}")
        print(f"Collision Cost Weight: {self.params.collision_cost_weight}")
        print(
            "*****************************************************************************")

    def sort_trajectories_by_cost(self,
                                  valid_array,
                                  cost_array):
        # get sorted indices of costs, from lowest to highest
        sorted_idx = np.argsort(cost_array[valid_array])

        return sorted_idx

    def calc_costs(
            self,
            valid_array: np.ndarray,
            rel_long_sampling_array: np.ndarray,
            track_handler: GlobalWaypointsTrackHandler,
            s_array: np.ndarray,
            n_array: np.ndarray,
            t_array: np.ndarray,
            V_array: np.ndarray,
            Omega_z_array: np.ndarray,
            ay_array: np.ndarray,
            raceline: dict,
            prediction: dict,
            V_target: float,
            planning_requests: dict,
            tire_util_array: np.ndarray,
            pitlane_mode: bool,
            vehicle_ahead: bool,
            emergency_brake: bool,
            vehicle_params: dict,
    ) -> int:

        skip_update = getattr(self, '_skip_param_updates', False)
        self.declare_and_update_parameters(skip_update=skip_update)

        curvature_cost_array = np.zeros_like(valid_array, dtype=float)
        lat_jerk_cost_array = np.zeros_like(valid_array, dtype=float)
        velocity_cost_array = np.zeros_like(valid_array, dtype=float)
        collision_cost_array = np.zeros_like(valid_array, dtype=float)
        raceline_cost_array = np.zeros_like(valid_array, dtype=float)
        friction_cost_array = np.zeros_like(valid_array, dtype=float)
        prediction_cost_array = np.zeros_like(valid_array, dtype=float)

        # store raw cost terms for debugging
        if self.debugging:
            curvature_cost_array_raw = np.zeros(
                (len(valid_array), len(t_array[0])-1))
            lat_jerk_cost_array_raw = np.zeros(
                (len(valid_array), len(t_array[0])-1))
            velocity_cost_array_raw = np.zeros(
                (len(valid_array), len(t_array[0])-1))
            collision_cost_array_raw = np.zeros(
                (len(valid_array), len(t_array[0])-1))
            raceline_cost_array_raw = np.zeros(
                (len(valid_array), len(t_array[0])-1))
            friction_cost_array_raw = np.zeros(
                (len(valid_array), len(t_array[0])-1))
            prediction_cost_array_raw = np.zeros(
                (len(valid_array), len(t_array[0])-1))

        # general expressions
        increasing_time_factor = np.minimum(
            t_array[valid_array] / self.params.horizon, 1.0)
        diff_time_array = np.diff(t_array[valid_array], axis=1)
        V_fading_factor = np.ones_like(s_array[valid_array])

        # Extract raceline data from global waypoints format
        if "wpnts" in raceline:
            # New global waypoints format
            raceline_s = np.array([wp["s_m"] for wp in raceline["wpnts"]])
            raceline_v = np.array([wp.get("vx_mps", 0.0)
                                  for wp in raceline["wpnts"]])
        else:
            # Fallback to old format (postprocessed_raceline from tam_sampling_core)
            raceline_s = raceline.get("s_post", [])
            # lowercase 'v' for consistency
            raceline_v = raceline.get("v_post", [])

        V_raceline = interpolate_with_period(s_array[valid_array], raceline_s,
                                             raceline_v, track_handler.get_track_length())

        # set costs according to role and if overtaking is allowed
        # 0 = no flag, 1 = defender, 2 = attacker

        # Init cost weight with default params
        velocity_cost_weight = self.params.velocity_cost_weight
        raceline_cost_weight = self.params.raceline_cost_weight
        lateral_jerk_cost_weight = self.params.lateral_jerk_cost_weight

        # assign attacker role cost weights
        if planning_requests["role"] == 2 and planning_requests["overtaking_allowed"]:
            velocity_cost_weight = self.params.velocity_cost_weight_overtaking
            raceline_cost_weight = self.params.raceline_cost_weight_overtaking
            lateral_jerk_cost_weight = self.params.lateral_jerk_cost_weight_overtaking

        # ------------------------------------------------------------------------------------------------------------------
        # CURVATURE COST
        # ------------------------------------------------------------------------------------------------------------------
        # set curvature cost to zero when below preset speed
        curvature_rl = interpolate_with_period(
            s_array[valid_array], track_handler.s_coord(
            ), track_handler.omega_z(), track_handler.get_track_length()
        )
        curvature_diff_array = np.where(
            V_array[valid_array, :-1] < self.params.curvature_cost_threshold,
            0.0,
            (np.abs(Omega_z_array[valid_array, :-1]) -
             np.abs(curvature_rl[:, :-1])),
        )

        curvature_cost = curvature_diff_array**2

        curvature_cost_array[valid_array] = self.params.curvature_cost_weight * np.add.reduce(
            curvature_cost * diff_time_array * np.sqrt(V_array[valid_array, :-1]), axis=1
        )

        # ------------------------------------------------------------------------------------------------------------------
        # LATERAL JERK COST
        # ------------------------------------------------------------------------------------------------------------------
        lat_jerk_array = np.abs(np.diff(ay_array[valid_array]))
        lat_jerk_cost_array[valid_array] = lateral_jerk_cost_weight * np.add.reduce(
            lat_jerk_array * diff_time_array, axis=1
        )

        # ------------------------------------------------------------------------------------------------------------------
        # VELOCITY COST
        # ------------------------------------------------------------------------------------------------------------------
        # set target speed to zero if all edges collide and behind an opponent vehicle
        if emergency_brake and vehicle_ahead:
            V_target = 0.0
            V_diff_array = np.abs(np.minimum(
                V_raceline[:, :-1], V_target) - V_array[valid_array, :-1])

        # reduce deceleration when hard braking is not necessary
        # V_target + 1.0 is used to avoid numerical errors
        # Deceleration required
        elif (V_target + 1.0) < V_array[0][0] and not self.params.max_deceleration_on_target_change <= 0:
            v_start = V_array[0][0]  # current velocity
            t_end = self.params.horizon
            v_end = (
                v_start -
                max(self.params.max_deceleration_on_target_change, 2.0) * t_end
            )  # avoid user error setting to low acceleration
            V_target_array = np.maximum(np.interp(t_array[valid_array], [
                                        0, t_end], [v_start, v_end]), V_target)
            V_diff_array = V_array[valid_array, :-1] - V_target_array[:, :-1]
        # cap maximum delta speed when accelerating
        else:
            # normalize cost term
            V_diff_array_equal = np.minimum(
                V_raceline[:, :-1], V_target) - V_array[valid_array, :-1]
            # Mutiply all negative values in V_diff x velocity_excess_cost_multiplier
            V_diff_array_unequal = np.where(
                V_diff_array_equal >= 0.0, 1.0, self.params.velocity_excess_cost_multiplier) * V_diff_array_equal
            V_diff_array_unsaturated = np.abs(V_diff_array_unequal)

            # Safety check: Handle empty array case when no valid trajectories
            if V_diff_array_unsaturated.size > 0:
                V_diff_max_cur = np.max(V_diff_array_unsaturated)
                V_diff_array = V_diff_array_unsaturated / V_diff_max_cur * \
                    np.minimum(self.params.V_diff_max_costs, V_diff_max_cur)
            else:
                V_diff_array = V_diff_array_unsaturated  # Empty array

        velocity_cost = V_diff_array ** 2

        velocity_cost_array[valid_array] = velocity_cost_weight * np.add.reduce(
            velocity_cost * diff_time_array, axis=1
        )

        # ------------------------------------------------------------------------------------------------------------------
        # RACELINE COST
        # ------------------------------------------------------------------------------------------------------------------
        # either equally weighted or linearly increasing over horizon
        # Extract raceline lateral positions from global waypoints format
        if "wpnts" in raceline:
            # New global waypoints format - use d_m for lateral offset
            raceline_n = np.array([wp.get("d_m", 0.0)
                                  for wp in raceline["wpnts"]])
        else:
            # Fallback to old format for backward compatibility
            raceline_n = raceline.get("n_post", [])

        raceline_deviation = np.interp(
            s_array[valid_array], raceline_s, raceline_n) - n_array[valid_array]

        raceline_cost = (((1.0 + abs(raceline_deviation[:, :-1]))**2) - 1.0) * increasing_time_factor[:,
                                                                                                      :-1]**2 if self.params.increasing_rl_cost else np.abs(raceline_deviation[:, :-1])

        raceline_cost_array[valid_array] = raceline_cost_weight * np.add.reduce(
            raceline_cost * diff_time_array, axis=1)

        # ------------------------------------------------------------------------------------------------------------------
        # FRICTION COST
        # ------------------------------------------------------------------------------------------------------------------
        # add costs where tire limits are slightly violated
        friction_violation_array = np.maximum(
            0.0, (tire_util_array[valid_array][:, :-1] - 1)) ** 3

        # TODO: remove SPAX factor for cost function weight
        friction_cost_array[valid_array] = 50000 * self.params.friction_cost_weight * np.add.reduce(
            np.abs(friction_violation_array) * diff_time_array,
            axis=1
        )

        # ------------------------------------------------------------------------------------------------------------------
        # PREDICTION COST
        # ------------------------------------------------------------------------------------------------------------------

        # if no prediction reveived
        weighted_prediction_costs = np.zeros(
            (len(valid_array), len(t_array[0])-1))[valid_array]

        # reduce safety zone size when in defender role
        if planning_requests["role"] == 1:
            prediction_s_factor = self.params.prediction_s_factor_defender
            prediction_n_factor = self.params.prediction_n_factor_defender

        else:
            # adjust longitudinal safety ellipse size to current velocity in atacker mode
            prediction_s_factor = np.interp(V_array[valid_array], [15.0, 50.0], [
                                            self.params.prediction_s_factor_min_size, self.params.prediction_s_factor_max_size])
            prediction_n_factor = self.params.prediction_n_factor

        for pred_idx, prediction_id in enumerate(prediction):
            prediction_cur = prediction[prediction_id]

            # check if prediction is considered at all
            if not prediction_cur["valid"]:
                continue

            # Convert global waypoints format to time-series format if needed
            if "wpnts" in prediction_cur:
                pred_data = self._convert_global_waypoints_to_prediction(
                    prediction_cur)
            else:
                pred_data = prediction_cur

            # use smaller ellipse for static objects
            if prediction_cur["prediction_type"] == "static":
                prediction_s_factor = self.params.prediction_s_factor_static
                prediction_n_factor = self.params.prediction_n_factor_static

            # time factor reduced prediction costs that is further in the future
            time_uncertain = self.params.horizon
            time_pred_uncertainty = t_array[valid_array] + \
                prediction_cur["time_offset"]
            time_factor = np.minimum(
                time_pred_uncertainty / time_uncertain, 1.0)

            # get distances to predicted vehicle and handle start-finish line
            s_prediction_cur = interpolate_with_period(
                t_array[valid_array],
                pred_data["time_w_offset"],
                pred_data["s"],
                track_handler.get_track_length(),
            )
            n_prediction_cur = np.interp(
                t_array[valid_array], pred_data["time_w_offset"], pred_data["n"])

            track_length = track_handler.get_track_length()
            # positive if ego is ahead, negative if behind
            s_dist_raw = s_array[valid_array] - s_prediction_cur

            # handle start-finish line for s distance
            s_dist_sign = np.sign(s_dist_raw)
            s_dist = np.where(abs(s_dist_raw) < track_length / 2.0,
                              s_dist_raw, s_dist_raw + (-s_dist_sign) * track_length)

            n_dist = np.abs(n_array[valid_array] - n_prediction_cur)

            # calculate tendency if s_dist becomes bigger or smaller
            delta_s = np.diff(s_dist, axis=1)
            delta_s_dot = np.zeros_like(s_dist)
            delta_s_dot[:, :-1] = delta_s / diff_time_array

            # Calculate factor based on delta_s_dot
            delta_s_dot_factor = np.interp(
                delta_s_dot, [0, 10], [1.0, self.params.prediction_s_asym_scaling])

            # ASYMMETRIC ELLIPSE: assign smaller ellipse size when in front of other vehicle (positive s_dist)
            prediction_s_factor = np.where(
                s_dist > 0.0, self.params.prediction_s_asym_scaling * prediction_s_factor, prediction_s_factor)

            # Apply delta_s_dot_factor to prediction_s_factor
            prediction_s_factor *= delta_s_dot_factor

            # ASYMMETRIC ELLIPSE: assign smaller ellipse size when in front of other vehicle (positive s_dist)
            prediction_s_factor = np.where(
                s_dist > 0.0, self.params.prediction_s_asym_scaling * prediction_s_factor, prediction_s_factor)

            # less weight to prediction points further into the future
            uncertainty_discount = np.exp(
                -self.params.prediction_uncertainty_weight * time_factor**2)

            raw_prediction_costs = np.exp(-prediction_s_factor *
                                          (s_dist) ** 2 - prediction_n_factor * (n_dist) ** 2)
            weighted_prediction_costs = raw_prediction_costs[:,
                                                             :-1] * uncertainty_discount[:, :-1]
            prediction_cost_array[valid_array] += self.params.prediction_cost_weight * np.add.reduce(
                weighted_prediction_costs * diff_time_array, axis=1
            )

        # ------------------------------------------------------------------------------------------------------------------
        # COLLISION COST
        # ------------------------------------------------------------------------------------------------------------------

        # Only check collisions if there are valid trajectories to check
        if np.any(valid_array):
            for pred_idx, prediction_id in enumerate(prediction):
                prediction_cur = prediction[prediction_id]

                if prediction_cur["valid"]:
                    # Convert global waypoints format to time-series format if needed
                    if "wpnts" in prediction_cur:
                        pred_data = self._convert_global_waypoints_to_prediction(
                            prediction_cur)
                    else:
                        pred_data = prediction_cur

                    # get time array of equal distance steps
                    t_equal_steps = np.linspace(
                        0, self.params.collision_check_horizon_s, 51)
                    t_array_equal_steps = np.ones_like(
                        t_array[valid_array]) * t_equal_steps

                    s_prediction_cur = np.interp(
                        t_array_equal_steps, pred_data["time_w_offset"], pred_data["s"]
                    )
                    n_prediction_cur = np.interp(
                        t_array_equal_steps, pred_data["time_w_offset"], pred_data["n"]
                    )

                    # evaluate candidate trajectories on equal time step array
                    s_traj_check = np.array([np.interp(t_array_equal_steps[i], t_array[valid_array]
                                            [i], s_array[valid_array][i]) for i in range(len(s_array[valid_array]))])
                    n_traj_check = np.array([np.interp(t_array_equal_steps[i], t_array[valid_array]
                                            [i], n_array[valid_array][i]) for i in range(len(n_array[valid_array]))])

                    # check for collisions
                    s_diff_tmp = np.abs(s_prediction_cur - s_traj_check)
                    n_diff = np.abs(n_prediction_cur - n_traj_check)

                    # handle start finish line
                    track_length = track_handler.get_track_length()
                    s_diff = np.where(s_diff_tmp < track_length /
                                      2.0, s_diff_tmp, track_length - s_diff_tmp)

                    # handle start-finish line

                    # check for collisions in s and n
                    s_collision = s_diff < (
                        vehicle_params["total_length"] + self.params.safety_distance_vehicles)
                    n_collision = n_diff < (
                        vehicle_params["total_width"] + self.params.tube_width + self.params.safety_distance_vehicles)

                    # get earliest time stamp for a collision for each trajectory
                    collision = (s_collision) & (n_collision)

                    # store where collision occurs
                    collision_mask = np.any(collision, axis=1)

                    # handle empty arrays
                    if collision.size == 0:
                        collision_cost_array = np.zeros_like(valid_array)
                    else:
                        # some ugly lines to distinguish between collision in first step and no collision
                        earliest_idx = np.argmax(collision, axis=1)
                        earliest_idx = np.where(
                            collision_mask, earliest_idx, -1)
                        time_to_collision = t_array_equal_steps[0][earliest_idx]

                        # velocity difference on projected impact
                        if prediction_cur["prediction_type"] != "static":
                            vel_prediction_cur = np.interp(
                                t_array_equal_steps, pred_data["time_w_offset"], pred_data["vel"]
                            )
                            delta_vel_on_collision = np.where(earliest_idx == -1, 0.0, np.abs(
                                vel_prediction_cur[0][earliest_idx] - V_array[valid_array, earliest_idx]))
                        else:  # handle static predictions
                            delta_vel_on_collision = np.where(
                                earliest_idx == -1, 0.0, np.abs(V_array[valid_array, earliest_idx]))

                        # prevent division by zero
                        time_to_collision = np.maximum(time_to_collision, 0.01)

                        collision_cost = delta_vel_on_collision / \
                            (time_to_collision ** 2)

                        collision_cost_array[valid_array] += self.params.collision_cost_weight * collision_cost

        # ------------------------------------------------------------------------------------------------------------------
        # PUNISHMENT FOR USING ABSOLUTE SAMPLES
        # ------------------------------------------------------------------------------------------------------------------
        velocity_cost_array[valid_array] = np.where(rel_long_sampling_array[valid_array], velocity_cost_array[valid_array],
                                                    self.params.additional_absolute_sample_cost + velocity_cost_array[valid_array])
        # ------------------------------------------------------------------------------------------------------------------

        # store raw cost terms over time for debugging
        if self.debugging:
            curvature_cost_array_raw[valid_array] = curvature_cost
            lat_jerk_cost_array_raw[valid_array] = lat_jerk_array
            velocity_cost_array_raw[valid_array] = velocity_cost
            # collision_cost_array_raw[valid_array] = collision_cost
            raceline_cost_array_raw[valid_array] = raceline_cost
            friction_cost_array_raw[valid_array] = friction_violation_array
            prediction_cost_array_raw[valid_array] = weighted_prediction_costs

        # OVERALL COSTS
        cost_array = curvature_cost_array + velocity_cost_array + raceline_cost_array + \
            prediction_cost_array + lat_jerk_cost_array + \
            friction_cost_array + collision_cost_array

        if self.debugging:
            cost_extensive_array = [lat_jerk_cost_array_raw, velocity_cost_array_raw, raceline_cost_array_raw,
                                    prediction_cost_array_raw, lat_jerk_cost_array_raw, friction_cost_array_raw, None]
        else:
            cost_extensive_array = None

        return cost_array, [curvature_cost_array, velocity_cost_array, raceline_cost_array, prediction_cost_array, lat_jerk_cost_array, friction_cost_array, collision_cost_array], cost_extensive_array
