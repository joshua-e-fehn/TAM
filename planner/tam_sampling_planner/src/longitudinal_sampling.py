#!/usr/bin/env python3
"""
TAM Longitudinal Sampling Module - Simplified Version
Velocity profile generation using kinematic constraints only

SIMPLIFIED VERSION:
- GGGV imports and physics-based acceleration limits commented out
- Only kinematic sampling methods available: calc_samples() and calc_samples_s_based()
- Physics-based method calc_samples_s_based_forward_backward() commented out
- This provides basic trajectory generation without complex vehicle dynamics

Ported from tam_race_stack/mod_planning/sampling_planner/sampling_planner/longitudinal_sampling.py
"""

from track_handler_global_waypoints import GlobalWaypointsTrackHandler as Track
import numpy as np

# GGGV imports commented out - using only kinematic sampling methods
# try:
#     from planning_common.track.gggvManager import GGGVManager, Grip_Map
#     REAL_GGGV_AVAILABLE = True
#     rospy.loginfo("Using real GGGV manager from planning_common")
# except ImportError:
#     from simple_gggv_manager import SimpleGGGVManager as GGGVManager, GripMap as Grip_Map
#     REAL_GGGV_AVAILABLE = False
#     rospy.logwarn("Real GGGV manager not available, using SimpleGGGVManager fallback")

from dataclasses import dataclass
import rospy

from simple_helper_utils import create_trim_mask, create_trim_mask_2d, find_nearest_s_and_idx, interpolate_with_period
from pacejka_tire_model import PacejkaTireModel


@dataclass(init=False)
class LongSamplingParams():
    s_dot_end_min: float
    relative_s_dot_min_percentage: float
    s_dot_max_positive_delta: float
    s_dot_discretization: float
    s_dot_dense_min: float
    s_dot_dense_max: float
    s_dot_dense_samples: int
    n_samples: int
    n_dense_samples: int
    num_samples: int
    horizon: float
    v_sampling_scale: float
    forward_backward_velocities: bool
    samples_forward_backward: float
    forward_backward_min_scale: float
    forward_backward_max_scale: float
    forward_backward_max_v_to_rl_delta: float
    tracjectory_length: float
    max_velocity: float
    max_acceleration: float
    max_lateral_acceleration: float


class LongitudinalSampling:
    def __init__(self, debugging=False):
        self.params = LongSamplingParams()
        self.initalized_parameters = False
        self.declare_and_update_parameters()

        # Initialize Pacejka tire model for physics-based acceleration limits
        try:
            self.tire_model = PacejkaTireModel()
            self.use_tire_model = True
            rospy.loginfo(
                "LongitudinalSampling initialized with Pacejka tire model")
        except Exception as e:
            rospy.logwarn(f"Failed to initialize Pacejka tire model: {e}")
            rospy.logwarn("Falling back to fixed acceleration limits")
            self.tire_model = None
            self.use_tire_model = False

        rospy.loginfo("LongitudinalSampling initialized in SIMPLIFIED mode")
        rospy.loginfo(
            "Available methods: calc_samples() and calc_samples_s_based()")
        rospy.loginfo(
            "Physics-based method calc_samples_s_based_forward_backward() uses Pacejka tire limits")

    def get_available_methods(self):
        """
        Return list of available sampling methods in this simplified version.

        Returns:
            list: Available method names
        """
        return [
            "calc_samples",           # Time-based kinematic sampling
            "calc_samples_s_based"    # Space-based kinematic sampling
        ]

    def get_disabled_methods(self):
        """
        Return list of methods that are disabled in this simplified version.

        Returns:
            list: Disabled method names and reasons
        """
        return [
            {
                "method": "calc_samples_s_based_forward_backward",
                "reason": "Requires GGGV diagrams for physics-based acceleration limits"
            }
        ]

    def convert_global_waypoints_to_raceline_format(self, global_waypoints: dict) -> dict:
        """
        Convert global waypoints format to postprocessed_raceline format for compatibility.

        Global waypoints format:
        {
            "wpnts": [
                {
                    "id": 1,
                    "s_m": 1.0,        # Arc length position [m]
                    "d_m": 0.0,        # Lateral offset from centerline [m]
                    "x_m": 11.0,       # X coordinate [m]
                    "y_m": 5.1,        # Y coordinate [m]
                    "vx_mps": 8.7,     # Velocity [m/s]
                    "kappa_radpm": 0.05, # Curvature [rad/m]
                    "d_left": 2.5,     # Left track boundary [m]
                    "d_right": -2.5    # Right track boundary [m]
                }
            ]
        }

        Args:
            global_waypoints: Dictionary with 'wpnts' key containing list of waypoint dicts

        Returns:
            dict: Postprocessed raceline format with t_post, s_post, s_dot_post, s_ddot_post

        Raises:
            ValueError: If global_waypoints format is invalid
        """
        if not isinstance(global_waypoints, dict):
            raise ValueError("global_waypoints must be a dictionary")

        if 'wpnts' not in global_waypoints:
            raise ValueError("global_waypoints must contain 'wpnts' key")

        waypoints = global_waypoints['wpnts']
        if not isinstance(waypoints, list):
            raise ValueError("waypoints must be a list")

        if len(waypoints) == 0:
            raise ValueError("waypoints list cannot be empty")

        # Extract arrays from waypoints with validation
        s_post = []
        s_dot_post = []

        for i, wp in enumerate(waypoints):
            if not isinstance(wp, dict):
                raise ValueError(f"Waypoint {i} must be a dictionary")

            # Extract s_m with validation
            if 's_m' not in wp:
                raise ValueError(f"Waypoint {i} missing required field 's_m'")
            s_post.append(float(wp['s_m']))

            # Extract vx_mps with fallback
            vx = wp.get('vx_mps', 1.0)  # Default to 1.0 m/s if missing
            if vx <= 0:
                rospy.logwarn(
                    f"Waypoint {i} has non-positive velocity {vx}, using 1.0 m/s")
                vx = 1.0
            s_dot_post.append(float(vx))

        # Convert to numpy arrays
        s_post = np.array(s_post)
        s_dot_post = np.array(s_dot_post)

        # Validate monotonic s coordinates
        if not np.all(np.diff(s_post) >= 0):
            rospy.logwarn(
                "Arc length coordinates (s_m) are not monotonically increasing")

        # Calculate time array from velocity and arc length
        t_post = self._calculate_time_from_velocity(s_post, s_dot_post)

        # Calculate acceleration array from velocity gradient
        s_ddot_post = self._calculate_acceleration_from_velocity(
            s_post, s_dot_post)

        rospy.loginfo(
            f"Converted {len(waypoints)} global waypoints to raceline format")
        rospy.loginfo(
            f"Arc length range: {s_post[0]:.2f} to {s_post[-1]:.2f} m")
        rospy.loginfo(
            f"Velocity range: {np.min(s_dot_post):.2f} to {np.max(s_dot_post):.2f} m/s")
        rospy.loginfo(f"Time range: {t_post[0]:.2f} to {t_post[-1]:.2f} s")

        return {
            't_post': t_post,
            's_post': s_post,
            's_dot_post': s_dot_post,
            's_ddot_post': s_ddot_post
        }

    def _calculate_time_from_velocity(self, s_array: np.ndarray, vx_array: np.ndarray) -> np.ndarray:
        """
        Approximate time array from velocity profile and arc length.

        Args:
            s_array: Arc length array [m]
            vx_array: Velocity array [m/s]

        Returns:
            np.ndarray: Time array [s]
        """
        if len(s_array) < 2:
            return np.array([0.0])

        # Calculate distance increments
        ds = np.diff(s_array)

        # Use average velocity between points
        v_mean = (vx_array[:-1] + vx_array[1:]) / 2.0

        # Ensure no zero velocities to avoid division by zero
        v_mean = np.maximum(v_mean, 0.1)  # Minimum 0.1 m/s

        # Calculate time increments
        dt = ds / v_mean

        # Cumulative time starting from 0
        t_array = np.concatenate(([0.0], np.cumsum(dt)))

        return t_array

    def _calculate_acceleration_from_velocity(self, s_array: np.ndarray, vx_array: np.ndarray) -> np.ndarray:
        """
        Approximate acceleration array from velocity gradient.

        Args:
            s_array: Arc length array [m]
            vx_array: Velocity array [m/s]

        Returns:
            np.ndarray: Acceleration array [m/s²]
        """
        if len(s_array) < 2:
            return np.array([0.0])

        # Calculate velocity gradient with respect to arc length: d(s_dot)/ds
        ds_dot_ds = np.gradient(vx_array, s_array)

        # Transform to acceleration: s_ddot = d(s_dot)/dt = d(s_dot)/ds * ds/dt = d(s_dot)/ds * s_dot
        s_ddot = ds_dot_ds * vx_array

        # Apply smoothing to reduce numerical noise
        if len(s_ddot) >= 5:
            # Simple moving average smoothing
            kernel_size = min(5, len(s_ddot))
            kernel = np.ones(kernel_size) / kernel_size
            s_ddot = np.convolve(s_ddot, kernel, mode='same')

        return s_ddot

    def validate_converted_raceline(self, postprocessed_raceline: dict) -> bool:
        """
        Validate the converted raceline data for potential issues.

        Args:
            postprocessed_raceline: Converted raceline data

        Returns:
            bool: True if validation passes, False otherwise
        """
        try:
            required_keys = ['t_post', 's_post', 's_dot_post', 's_ddot_post']
            for key in required_keys:
                if key not in postprocessed_raceline:
                    rospy.logerr(f"Missing required key: {key}")
                    return False

                if not isinstance(postprocessed_raceline[key], np.ndarray):
                    rospy.logerr(f"Key {key} must be numpy array")
                    return False

            # Check array lengths match
            lengths = [len(postprocessed_raceline[key])
                       for key in required_keys]
            if not all(l == lengths[0] for l in lengths):
                rospy.logerr(f"Array length mismatch: {lengths}")
                return False

            # Check for NaN or infinite values
            for key in required_keys:
                arr = postprocessed_raceline[key]
                if np.any(np.isnan(arr)) or np.any(np.isinf(arr)):
                    rospy.logerr(f"Invalid values (NaN/Inf) in {key}")
                    return False

            # Check monotonic time
            t_post = postprocessed_raceline['t_post']
            if not np.all(np.diff(t_post) >= 0):
                rospy.logerr("Time array is not monotonically increasing")
                return False

            # Check positive velocities
            s_dot_post = postprocessed_raceline['s_dot_post']
            if np.any(s_dot_post <= 0):
                rospy.logwarn("Some velocities are non-positive")

            # Check reasonable acceleration limits
            s_ddot_post = postprocessed_raceline['s_ddot_post']
            max_accel = np.max(np.abs(s_ddot_post))
            if max_accel > 20.0:  # 20 m/s² is quite high for racing
                rospy.logwarn(
                    f"Very high acceleration detected: {max_accel:.2f} m/s²")

            # rospy.loginfo("Raceline validation passed")
            return True

        except Exception as e:
            rospy.logerr(f"Raceline validation failed: {e}")
            return False

    @staticmethod
    def create_example_global_waypoints(num_points: int = 10, track_length: float = 100.0) -> dict:
        """
        Create example global waypoints for testing purposes.

        Args:
            num_points: Number of waypoints to generate
            track_length: Total track length [m]

        Returns:
            dict: Example global waypoints in the correct format
        """
        waypoints = []

        for i in range(num_points):
            # Create simple straight track with varying velocity
            s = (i / (num_points - 1)) * track_length

            # Simple velocity profile: fast in middle, slower at ends
            v_base = 10.0  # Base velocity
            v_var = 5.0    # Velocity variation
            t_norm = 2 * i / (num_points - 1) - 1  # Normalized to [-1, 1]
            # Parabolic velocity profile
            velocity = v_base + v_var * (1 - t_norm**2)

            waypoint = {
                "id": i + 1,
                "s_m": s,
                "d_m": 0.0,  # On centerline
                "x_m": s * 0.9,  # Slightly angled track
                "y_m": s * 0.1,
                "vx_mps": velocity,
                # Slight curvature variation
                "kappa_radpm": 0.01 * np.sin(s / 10.0),
                "d_left": 2.5,
                "d_right": -2.5
            }
            waypoints.append(waypoint)

        return {"wpnts": waypoints}

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
                f"LongitudinalSampling: Could not load YAML defaults: {e}")
            return {}

    def declare_and_update_parameters(self):
        if not self.initalized_parameters:
            # Initialization: Load from YAML with ROS param fallback, then set ROS params
            yaml_defaults = self._load_yaml_defaults()

            # Load parameters: YAML defaults with ROS param fallback
            self.params.s_dot_end_min = yaml_defaults.get(
                's_dot_end_min', rospy.get_param("discretization/s_dot_end_min", 1.0))
            self.params.relative_s_dot_min_percentage = yaml_defaults.get(
                'relative_s_dot_min_percentage', rospy.get_param("behavior/relative_s_dot_min_percentage", 0.7))
            self.params.s_dot_max_positive_delta = yaml_defaults.get(
                's_dot_max_positive_delta', rospy.get_param("behavior/s_dot_max_positive_delta", 20.0))
            self.params.s_dot_discretization = yaml_defaults.get(
                's_dot_discretization', rospy.get_param("discretization/s_dot_discretization", 2.0))
            self.params.s_dot_dense_min = yaml_defaults.get(
                's_dot_dense_min', rospy.get_param("discretization/s_dot_dense_min", -4.0))
            self.params.s_dot_dense_max = yaml_defaults.get(
                's_dot_dense_max', rospy.get_param("discretization/s_dot_dense_max", 1.0))
            self.params.s_dot_dense_samples = yaml_defaults.get(
                's_dot_dense_samples', rospy.get_param("discretization/s_dot_dense_samples", 10))
            self.params.n_samples = yaml_defaults.get(
                'lateral_samples', rospy.get_param("discretization/n_samples", 20))
            self.params.n_dense_samples = yaml_defaults.get(
                'n_dense_samples', rospy.get_param("discretization/n_dense_samples", 5))
            self.params.num_samples = yaml_defaults.get(
                'num_samples', rospy.get_param("discretization/num_samples", 51))
            self.params.horizon = yaml_defaults.get(
                'planning_horizon', rospy.get_param("behavior/horizon", 4.0))
            self.params.v_sampling_scale = yaml_defaults.get(
                'v_sampling_scale', rospy.get_param("behavior/v_sampling_scale", 1.1))
            self.params.forward_backward_velocities = yaml_defaults.get(
                'forward_backward_velocities', rospy.get_param("behavior/forward_backward_velocities", True))
            self.params.samples_forward_backward = int(yaml_defaults.get(
                'samples_forward_backward', rospy.get_param("behavior/samples_forward_backward", 3)))
            self.params.forward_backward_min_scale = yaml_defaults.get(
                'forward_backward_min_scale', rospy.get_param("behavior/forward_backward_min_scale", 0.85))
            self.params.forward_backward_max_scale = yaml_defaults.get(
                'forward_backward_max_scale', rospy.get_param("behavior/forward_backward_max_scale", 0.95))
            self.params.forward_backward_max_v_to_rl_delta = yaml_defaults.get(
                'forward_backward_max_v_to_rl_delta', rospy.get_param("behavior/forward_backward_max_v_to_rl_delta", 0.0))
            self.params.tracjectory_length = yaml_defaults.get(
                'trajectory_length', rospy.get_param("behavior/track_length", 10.0))
            self.params.max_velocity = yaml_defaults.get(
                'max_speed', rospy.get_param("behavior/max_velocity", 10.0))
            self.params.max_acceleration = yaml_defaults.get(
                'max_accel', rospy.get_param("behavior/max_acceleration", 3.0))
            self.params.max_lateral_acceleration = yaml_defaults.get(
                'max_lateral_accel', rospy.get_param("behavior/max_lateral_acceleration", 3.0))

            # Set ROS params to loaded values
            rospy.set_param("discretization/s_dot_end_min",
                            self.params.s_dot_end_min)
            rospy.set_param("behavior/relative_s_dot_min_percentage",
                            self.params.relative_s_dot_min_percentage)
            rospy.set_param("behavior/s_dot_max_positive_delta",
                            self.params.s_dot_max_positive_delta)
            rospy.set_param("discretization/s_dot_discretization",
                            self.params.s_dot_discretization)
            rospy.set_param("discretization/s_dot_dense_min",
                            self.params.s_dot_dense_min)
            rospy.set_param("discretization/s_dot_dense_max",
                            self.params.s_dot_dense_max)
            rospy.set_param("discretization/s_dot_dense_samples",
                            self.params.s_dot_dense_samples)
            rospy.set_param("discretization/n_samples", self.params.n_samples)
            rospy.set_param("discretization/n_dense_samples",
                            self.params.n_dense_samples)
            rospy.set_param("discretization/num_samples",
                            self.params.num_samples)
            rospy.set_param("behavior/horizon", self.params.horizon)
            rospy.set_param("behavior/v_sampling_scale",
                            self.params.v_sampling_scale)
            rospy.set_param("behavior/forward_backward_velocities",
                            self.params.forward_backward_velocities)
            rospy.set_param("behavior/samples_forward_backward",
                            self.params.samples_forward_backward)
            rospy.set_param("behavior/forward_backward_min_scale",
                            self.params.forward_backward_min_scale)
            rospy.set_param("behavior/forward_backward_max_scale",
                            self.params.forward_backward_max_scale)
            rospy.set_param("behavior/forward_backward_max_v_to_rl_delta",
                            self.params.forward_backward_max_v_to_rl_delta)
            rospy.set_param("behavior/track_length",
                            self.params.tracjectory_length)
            rospy.set_param("behavior/max_velocity", self.params.max_velocity)
            rospy.set_param("behavior/max_acceleration",
                            self.params.max_acceleration)
            rospy.set_param("behavior/max_lateral_acceleration",
                            self.params.max_lateral_acceleration)

            self.initalized_parameters = True
        else:
            # Update: Use ROS params with current self.params values as fallback
            self.params.s_dot_end_min = rospy.get_param(
                "discretization/s_dot_end_min", self.params.s_dot_end_min)
            self.params.relative_s_dot_min_percentage = rospy.get_param(
                "behavior/relative_s_dot_min_percentage", self.params.relative_s_dot_min_percentage)
            self.params.s_dot_max_positive_delta = rospy.get_param(
                "behavior/s_dot_max_positive_delta", self.params.s_dot_max_positive_delta)
            self.params.s_dot_discretization = rospy.get_param(
                "discretization/s_dot_discretization", self.params.s_dot_discretization)
            self.params.s_dot_dense_min = rospy.get_param(
                "discretization/s_dot_dense_min", self.params.s_dot_dense_min)
            self.params.s_dot_dense_max = rospy.get_param(
                "discretization/s_dot_dense_max", self.params.s_dot_dense_max)
            self.params.s_dot_dense_samples = rospy.get_param(
                "discretization/s_dot_dense_samples", self.params.s_dot_dense_samples)
            self.params.n_samples = rospy.get_param(
                "discretization/n_samples", self.params.n_samples)
            self.params.n_dense_samples = rospy.get_param(
                "discretization/n_dense_samples", self.params.n_dense_samples)
            self.params.num_samples = rospy.get_param(
                "discretization/num_samples", self.params.num_samples)
            self.params.horizon = rospy.get_param(
                "behavior/horizon", self.params.horizon)
            self.params.v_sampling_scale = rospy.get_param(
                "behavior/v_sampling_scale", self.params.v_sampling_scale)
            self.params.forward_backward_velocities = rospy.get_param(
                "behavior/forward_backward_velocities", self.params.forward_backward_velocities)
            self.params.samples_forward_backward = int(rospy.get_param(
                "behavior/samples_forward_backward", self.params.samples_forward_backward))
            self.params.forward_backward_min_scale = rospy.get_param(
                "behavior/forward_backward_min_scale", self.params.forward_backward_min_scale)
            self.params.forward_backward_max_scale = rospy.get_param(
                "behavior/forward_backward_max_scale", self.params.forward_backward_max_scale)
            self.params.forward_backward_max_v_to_rl_delta = rospy.get_param(
                "behavior/forward_backward_max_v_to_rl_delta", self.params.forward_backward_max_v_to_rl_delta)
            self.params.tracjectory_length = rospy.get_param(
                "behavior/track_length", self.params.tracjectory_length)
            self.params.max_velocity = rospy.get_param(
                "behavior/max_velocity", self.params.max_velocity)
            self.params.max_acceleration = rospy.get_param(
                "behavior/max_acceleration", self.params.max_acceleration)
            self.params.max_lateral_acceleration = rospy.get_param(
                "behavior/max_lateral_acceleration", self.params.max_lateral_acceleration)

    def calc_samples(
            self,
            # initial condition
            s_start: float,
            s_dot_start: float,
            s_ddot_start: float,
            # end condition
            V_target: float,
            V_max: float,
            # Postprocessed Raceline (from postprocess_raceline method)
            postprocessed_raceline: dict,
            # Track
            track_handler: Track,
            raceline_tendency: bool
    ):  # -> TrajectorySamples

        self.declare_and_update_parameters()

        # Validate the postprocessed raceline data
        if not self.validate_converted_raceline(postprocessed_raceline):
            rospy.logerr(
                "Raceline validation failed in calc_samples - trajectory sampling may produce poor results")

        # raceline end conditions
        s_dot_end_rl = np.interp(
            self.params.horizon, postprocessed_raceline["t_post"], postprocessed_raceline["s_dot_post"])
        s_ddot_end_rl = np.interp(
            self.params.horizon, postprocessed_raceline["t_post"], postprocessed_raceline["s_ddot_post"])
        if raceline_tendency:  # use s_ddot of race line for boundary conditions
            # F1TENTH NOTE: NumPy 1.18.1 doesn't support 'period' parameter in unwrap (added in 1.21.0)
            # Manually implement periodic unwrapping for compatibility
            s_post = postprocessed_raceline["s_post"]
            period = track_handler.get_track_length()
            discont = period / 2

            # Normalize to [-period/2, period/2] range before unwrapping
            s_normalized = np.mod(s_post + period / 2, period) - period / 2
            # Unwrap without period parameter (standard unwrap with pi as default discontinuity)
            # Scale to use standard unwrap, then scale back
            s_continuous = np.unwrap(
                s_normalized * (np.pi / discont)) * (discont / np.pi)
        else:
            s_ddot_end = 0.0

        # set number of total n_samples
        n_samples = self.params.n_samples + self.params.n_dense_samples

        # sampled s_dot end conditions and do not let s_dot_max decrease below threshold
        s_dot_max = min(max(s_dot_start, min(s_dot_start + self.params.s_dot_max_positive_delta,
                        V_target, s_dot_end_rl)) * self.params.v_sampling_scale, V_max)
        s_dot_max = np.maximum(s_dot_max, 5.0)

        # only generate relative samples that are close to raceline
        if raceline_tendency:
            # TODO: implement parameter
            s_dot_min = self.params.relative_s_dot_min_percentage * s_dot_max
            # Dense sampling around raceline
            s_dot_dense_end_values = np.linspace(
                self.params.s_dot_dense_min + s_dot_end_rl, self.params.s_dot_dense_max + s_dot_end_rl, self.params.s_dot_dense_samples)
            s_dot_coarse_end_values = np.arange(
                s_dot_min, s_dot_max, self.params.s_dot_discretization)
            s_dot_end_values = np.concatenate(
                (s_dot_dense_end_values, s_dot_coarse_end_values))
        else:
            # always sample raceline and V_target
            s_dot_end_values = np.arange(
                self.params.s_dot_end_min, s_dot_max, self.params.s_dot_discretization)

        # always sample raceline s_dot end and target speed
        s_dot_end_values = np.concatenate(
            (s_dot_end_values, [max(V_target, 1.0), s_dot_end_rl]))

        # assign array sizes depending on number of s_dot end values
        s_array = np.zeros(
            (s_dot_end_values.shape[0] * n_samples, self.params.num_samples))
        s_dot_array = np.zeros(
            (s_dot_end_values.shape[0] * n_samples, self.params.num_samples))
        s_ddot_array = np.zeros(
            (s_dot_end_values.shape[0] * n_samples, self.params.num_samples))
        # save which samples are absolute
        rel_long_sampling_array = np.zeros(
            (s_dot_end_values.shape[0] * n_samples))

        # construct t array
        t_vector = np.linspace(0.0, self.params.horizon,
                               self.params.num_samples)
        t_array = np.tile(t_vector, (s_array.shape[0], 1))

        # end values of s and s_dot (needed for lateral curves)
        s_end_values = np.zeros_like(s_dot_end_values)

        for i, (s_dot_end) in enumerate(s_dot_end_values):
            t_end = t_vector[-1]

            """if raceline_tendency:
                # set end acceleration between 0 and raceline acceleration dependent on sampled velocity
                s_ddot_end_tmp = np.interp(
                    s_dot_end, [0.0, s_dot_end_rl], [0.0, s_ddot_end_rl])
                # only adhere to end acceleration of raceline when start velocity is also near to raceline velocity
                s_ddot_end = np.interp(s_dot_start, [0.0, postprocessed_raceline['s_dot_post'][0]], [0.0, s_ddot_end_tmp])"""

            if raceline_tendency:
                s_ddot_end = np.interp(s_dot_end, [0.0, s_dot_end_rl], [
                                       0.0, s_ddot_end_rl])

                # formulate linear system of equations
            a = np.array(
                [
                    [1, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0],
                    [0, 0, 2, 0, 0],
                    [0, 1, 2 * t_end, 3 * t_end**2, 4 * t_end**3],
                    [0, 0, 2, 6 * t_end, 12 * t_end**2],
                ]
            )
            if raceline_tendency:  # sample curves relative to raceline
                b = np.array(
                    [
                        s_start - postprocessed_raceline["s_post"][0],
                        s_dot_start - postprocessed_raceline["s_dot_post"][0],
                        s_ddot_start -
                        postprocessed_raceline["s_ddot_post"][0],
                        s_dot_end - s_dot_end_rl,
                        s_ddot_end - s_ddot_end_rl,
                    ]
                )
            else:  # sample curves absolute
                b = np.array(
                    [s_start, s_dot_start, s_ddot_start, s_dot_end, s_ddot_end])

            # calculate coefficients of quartic polynomial
            c = np.linalg.solve(a=a, b=b)

            # sampled s curve
            s_sample = c[0] + c[1] * t_vector + c[2] * t_vector ** 2 + \
                c[3] * t_vector ** 3 + c[4] * t_vector ** 4
            s_dot_sample = c[1] + 2 * c[2] * t_vector + 3 * \
                c[3] * t_vector ** 2 + 4 * c[4] * t_vector ** 3
            s_ddot_sample = 2 * c[2] + 6 * c[3] * \
                t_vector + 12 * c[4] * t_vector ** 2

            if raceline_tendency:
                # evaluate raceline s data at t_array points
                s_rl_eval = np.mod(np.interp(
                    t_vector, postprocessed_raceline['t_post'], s_continuous), track_handler.get_track_length())
                s_dot_rl_eval = np.interp(
                    t_vector, postprocessed_raceline['t_post'], postprocessed_raceline['s_dot_post'])
                s_ddot_rl_eval = np.interp(
                    t_vector, postprocessed_raceline['t_post'], postprocessed_raceline['s_ddot_post'])

                # add raceline s data to sampled relative s curve
                s = s_sample + s_rl_eval
                s_dot = s_dot_sample + s_dot_rl_eval
                s_ddot = s_ddot_sample + s_ddot_rl_eval
            else:
                s = s_sample
                s_dot = s_dot_sample
                s_ddot = s_ddot_sample

            # consider track length
            s = np.mod(s, track_handler.get_track_length())

            # save last values
            s_end_values[i] = s[-1]

            s_array[i * n_samples: (i + 1) * n_samples,
                    :] = np.tile(s, (n_samples, 1))
            s_dot_array[i * n_samples: (i + 1) * n_samples,
                        :] = np.tile(s_dot, (n_samples, 1))
            s_ddot_array[i * n_samples: (i + 1) * n_samples,
                         :] = np.tile(s_ddot, (n_samples, 1))

            # store type of sample
            if raceline_tendency:
                rel_long_sampling_array[i *
                                        n_samples: (i + 1) * n_samples] = True
            else:
                rel_long_sampling_array[i *
                                        n_samples: (i + 1) * n_samples] = False

        return s_array, s_dot_array, s_ddot_array, s_dot_end_values, s_end_values, rel_long_sampling_array, t_array

    def calc_samples_s_based(
            self,
            s_start: float,
            s_dot_start: float,
            s_ddot_start: float,
            V_target: float,
            postprocessed_raceline: dict,
            track_handler: Track,
            raceline_tendency: bool
    ):

        self.declare_and_update_parameters()

        # trajectory horizon in meters
        s_glob_end = np.mod(s_start + self.params.tracjectory_length,
                            track_handler.get_track_length())

        # construct local s vector so it uses only s values from the postprocessed_raceline
        _, idx_end = find_nearest_s_and_idx(
            postprocessed_raceline["s_post"], s_glob_end, track_handler)

        s_glob_interval = postprocessed_raceline["s_post"][:idx_end]

        # add s_start in pitlane mode
        if s_start not in postprocessed_raceline["s_post"]:
            s_glob_interval = np.insert(s_glob_interval, 0, s_start)

        s_loc_vector = (s_glob_interval -
                        s_start) % track_handler.get_track_length()

        # raceline end conditions
        s_dot_end_rl = np.interp(
            s_glob_end, postprocessed_raceline["s_post"], postprocessed_raceline["s_dot_post"])
        s_ddot_end_rl = np.interp(
            s_glob_end, postprocessed_raceline["s_post"], postprocessed_raceline["s_ddot_post"])

        # set number of total n_samples
        n_samples = self.params.n_samples + self.params.n_dense_samples

        # set end conditions
        if raceline_tendency:
            s_dot_end_max = s_dot_end_rl * self.params.v_sampling_scale
            s_dot_end_min = self.params.relative_s_dot_min_percentage * s_dot_end_max

            # Adaptive dense sampling - use percentages relative to raceline velocity
            dense_deviation_min_percentage = -0.30  # 30% below raceline
            dense_deviation_max_percentage = +0.10  # 10% above raceline

            dense_deviation_min = dense_deviation_min_percentage * s_dot_end_rl
            dense_deviation_max = dense_deviation_max_percentage * s_dot_end_rl

            # Dense sampling around raceline (now adaptive)
            s_dot_dense_end_values = np.linspace(
                dense_deviation_min + s_dot_end_rl,
                dense_deviation_max + s_dot_end_rl,
                self.params.s_dot_dense_samples)
            s_dot_coarse_end_values = np.arange(
                s_dot_end_min, s_dot_end_max, self.params.s_dot_discretization)
            s_dot_end_values_tmp = np.concatenate(
                (s_dot_dense_end_values, s_dot_coarse_end_values))
        else:
            s_dot_end_max = s_dot_start + self.params.s_dot_max_positive_delta
            s_dot_end_values_tmp = np.arange(
                self.params.s_dot_end_min, s_dot_end_max, self.params.s_dot_discretization)

        # always sample raceline s_dot end and target speed
        s_dot_end_values = np.concatenate(
            (s_dot_end_values_tmp, [max(V_target, 1.0), s_dot_end_rl]))

        # Clamp end values to vehicle limits with slack (1.5x for velocity)
        max_velocity_with_slack = self.params.max_velocity * 1.5
        s_dot_end_values = np.minimum(
            s_dot_end_values, max_velocity_with_slack)
        s_dot_end_values = np.maximum(
            s_dot_end_values, 0.0)  # No negative velocities

        # Remove duplicate end values to keep distinctive values only
        s_dot_end_values = np.unique(s_dot_end_values)

        # assign array sizes depending on number of s_dot end values
        target_shape = (
            s_dot_end_values.shape[0] * n_samples, self.params.num_samples)

        s_array = np.zeros(target_shape)
        s_dot_array = np.zeros(target_shape)
        s_ddot_array = np.zeros(target_shape)
        # save which samples are absolute
        rel_long_sampling_array = np.zeros(target_shape[0])
        t_array = np.zeros_like(s_array)

        # end values of s and s_dot (needed for lateral curves)
        s_end_values = np.zeros_like(s_dot_end_values)

        # Pre-compute minimum raceline velocity for deviation clamping
        v_rl_min_in_trajectory = None
        if raceline_tendency:
            s_dot_rl_eval_preview = interpolate_with_period(
                s_glob_interval, postprocessed_raceline['s_post'],
                postprocessed_raceline['s_dot_post'],
                period=track_handler.get_track_length())
            v_rl_min_in_trajectory = np.min(s_dot_rl_eval_preview)

        # Track which samples are valid (will skip samples with negative velocities)
        # Store (index, s_vals, s_dot, s_ddot, t, s_end) tuples
        valid_sample_data = []
        samples_skipped = 0

        # Slack: 1.5x for velocity/accel, 2.0x for decel
        rejection_stats = {
            'negative_velocity': 0,
            'over_max_velocity': 0,
            'over_max_accel': 0,
            'over_max_decel': 0
        }
        max_velocity_with_slack = self.params.max_velocity * 1.5
        max_accel_with_slack = self.params.max_acceleration * 1.5
        max_decel_with_slack = self.params.max_acceleration * 2.0

        for i, (s_dot_end) in enumerate(s_dot_end_values):
            s_end_loc = s_loc_vector[-1]

            # set end acceleration between 0 and raceline acceleration dependent on sampled velocity
            s_ddot_end = np.interp(s_dot_end, [0.0, s_dot_end_rl], [
                                   0.0, s_ddot_end_rl])

            # transform s_ddot(t) to s_ddot(s)
            s_pprime_start = s_ddot_start / s_dot_start
            s_pprime_end = s_ddot_end / s_dot_end

            # do the same for the raceline
            s_pprime_start_rl = postprocessed_raceline["s_ddot_post"][0] / \
                postprocessed_raceline["s_dot_post"][0]
            s_pprime_end_rl = s_ddot_end_rl / s_dot_end_rl

            # formulate linear system of equations
            A = np.array(
                [
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [1, s_end_loc, s_end_loc**2, s_end_loc**3],
                    [0, 1, 2 * s_end_loc, 3 * s_end_loc**2],
                ]
            )

            # Initialize for diagnostic output
            delta_v_start = None
            delta_v_end = None

            if raceline_tendency:  # sample curves relative to raceline
                # Calculate raw deviations
                delta_v_start_raw = s_dot_start - \
                    postprocessed_raceline["s_dot_post"][0]
                delta_v_end_raw = s_dot_end - s_dot_end_rl

                # Clamp deviations to prevent negative velocities (50% safety margin)
                max_safe_negative_deviation = -0.50 * v_rl_min_in_trajectory
                delta_v_start = max(delta_v_start_raw,
                                    max_safe_negative_deviation)
                delta_v_end = max(delta_v_end_raw, max_safe_negative_deviation)

                b = np.array(
                    [
                        delta_v_start,
                        s_pprime_start - s_pprime_start_rl,
                        delta_v_end,
                        s_pprime_end - s_pprime_end_rl,
                    ]
                )
            else:  # sample curves absolute
                b = np.array([s_dot_start,
                              s_pprime_start,
                              s_dot_end,
                              0.0])

            # calculate coefficients of quartic polynomial
            c = np.linalg.solve(a=A, b=b)

            # sampled s dot curve with fixed spatial horizon
            s_dot_sample = c[0] + c[1] * s_loc_vector + c[2] * \
                s_loc_vector ** 2 + c[3] * s_loc_vector ** 3
            s_pprime_sample = c[1] + 2 * c[2] * \
                s_loc_vector + 3 * c[3] * s_loc_vector ** 2

            if raceline_tendency:
                s_dot_rl_eval = interpolate_with_period(
                    s_glob_interval, postprocessed_raceline['s_post'], postprocessed_raceline['s_dot_post'], period=track_handler.get_track_length())
                s_pprime_rl_eval = interpolate_with_period(
                    s_glob_interval, postprocessed_raceline['s_post'], postprocessed_raceline['s_ddot_post'], period=track_handler.get_track_length()) / s_dot_rl_eval

                # add raceline s data to sampled relative s curve
                s_dot = s_dot_sample + s_dot_rl_eval
                s_ddot = s_pprime_sample + s_pprime_rl_eval
            else:
                s_dot = s_dot_sample
                s_ddot = s_pprime_sample

            #    Check for all vehicle limit violations and skip invalid samples
            min_s_dot = np.min(s_dot)
            max_s_dot = np.max(s_dot)
            min_s_ddot = np.min(s_ddot)
            max_s_ddot = np.max(s_ddot)

            # Check each violation type
            violation_detected = False
            violation_reasons = []

            if min_s_dot < 0:
                rejection_stats['negative_velocity'] += 1
                violation_detected = True
                violation_reasons.append(
                    f"negative velocity {min_s_dot:.3f} m/s")

            if max_s_dot > max_velocity_with_slack:
                rejection_stats['over_max_velocity'] += 1
                violation_detected = True
                violation_reasons.append(
                    f"over max_velocity {max_s_dot:.3f} > {max_velocity_with_slack:.3f} m/s")

            if min_s_ddot < -max_decel_with_slack:
                rejection_stats['over_max_decel'] += 1
                violation_detected = True
                violation_reasons.append(
                    f"over max_decel {min_s_ddot:.3f} < {-max_decel_with_slack:.3f} m/s²")

            if max_s_ddot > max_accel_with_slack:
                rejection_stats['over_max_accel'] += 1
                violation_detected = True
                violation_reasons.append(
                    f"over max_accel {max_s_ddot:.3f} > {max_accel_with_slack:.3f} m/s²")

            if violation_detected:
                samples_skipped += 1
                continue

            # consider track length
            s_vals = s_loc_vector + s_start

            # postprocessing 1: omit everything that is not in the horizon
            # add value at horizon to s_array

            t = self.calc_time_vector(
                track_handler, s_glob_interval, np.zeros_like(s_glob_interval), s_dot, raceline_tendency)

            # transform s_ddot(s) back to s_ddot(t)
            s_ddot = s_ddot * s_dot

            # Cut trajectories at spatial horizon (distance-based, not time-based)
            # Use s_glob_end which is s_start + trajectory_length
            s_horizon = s_glob_end

            # Find the corresponding velocity and acceleration at this spatial horizon
            # Interpolate using s_vals (spatial coordinates) instead of t (time)
            s_dot_horizon = np.interp(s_horizon, s_vals, s_dot)
            s_ddot_horizon = np.interp(s_horizon, s_vals, s_ddot)

            # adjust s values and s value at horizon by track length
            s_vals = np.mod(s_vals, track_handler.get_track_length())
            s_horizon = np.mod(s_horizon, track_handler.get_track_length())

            _, s_idx_horizon = find_nearest_s_and_idx(
                s_vals, s_horizon, track_handler)
            s_vals = np.insert(s_vals, s_idx_horizon, s_horizon)
            s_dot = np.insert(s_dot, s_idx_horizon, s_dot_horizon)
            s_ddot = np.insert(s_ddot, s_idx_horizon, s_ddot_horizon)

            s_vals = s_vals[:s_idx_horizon + 1]
            s_dot = s_dot[:s_idx_horizon + 1]
            s_ddot = s_ddot[:s_idx_horizon + 1]

            # postprocessing 2: cut length to horizon
            trim_mask = create_trim_mask(s_vals, self.params.num_samples)
            s_vals = s_vals[trim_mask]
            s_dot = s_dot[trim_mask]
            s_ddot = s_ddot[trim_mask]

            # postprocessing 3: interpolate to get the correct number of samples
            if len(s_vals) < self.params.num_samples:
                # insert points into s_vals between the current data points

                s_vals_enriched = np.linspace(
                    s_vals[0], s_vals[-1], self.params.num_samples)
                s_dot = np.interp(s_vals_enriched, s_vals, s_dot)
                s_ddot = np.interp(s_vals_enriched, s_vals, s_ddot)
                s_vals = s_vals_enriched

            # recalc shorter time vector
            t = self.calc_time_vector(
                track_handler, s_vals, np.zeros_like(s_vals), s_dot, raceline_tendency)

            # Store valid sample data
            valid_sample_data.append({
                'index': i,
                's_vals': s_vals,
                's_dot': s_dot,
                's_ddot': s_ddot,
                't': t,
                's_end': s_vals[-1],
                'raceline_tendency': raceline_tendency
            })

        # Build output arrays only from valid samples
        num_valid_samples = len(valid_sample_data)
        if num_valid_samples == 0:
            rospy.logerr(
                "[calc_samples_s_based] All samples rejected - returning empty arrays")
            return (np.zeros((0, self.params.num_samples)),
                    np.zeros((0, self.params.num_samples)),
                    np.zeros((0, self.params.num_samples)),
                    np.array([]), np.array([]), np.array([]),
                    np.zeros((0, self.params.num_samples)))

        # Resize arrays to actual valid sample count
        valid_shape = (num_valid_samples * n_samples, self.params.num_samples)
        s_array = np.zeros(valid_shape)
        s_dot_array = np.zeros(valid_shape)
        s_ddot_array = np.zeros(valid_shape)
        rel_long_sampling_array = np.zeros(valid_shape[0])
        t_array = np.zeros(valid_shape)
        s_end_values = np.zeros(num_valid_samples)

        # Fill arrays with valid samples
        for j, sample_data in enumerate(valid_sample_data):
            s_array[j * n_samples: (j + 1) * n_samples,
                    :] = np.tile(sample_data['s_vals'], (n_samples, 1))
            s_dot_array[j * n_samples: (j + 1) * n_samples,
                        :] = np.tile(sample_data['s_dot'], (n_samples, 1))
            s_ddot_array[j * n_samples: (j + 1) * n_samples,
                         :] = np.tile(sample_data['s_ddot'], (n_samples, 1))
            t_array[j * n_samples: (j + 1) * n_samples,
                    :] = np.tile(sample_data['t'], (n_samples, 1))
            s_end_values[j] = sample_data['s_end']

            if sample_data['raceline_tendency']:
                rel_long_sampling_array[j *
                                        n_samples: (j + 1) * n_samples] = True
            else:
                rel_long_sampling_array[j *
                                        n_samples: (j + 1) * n_samples] = False

        # Check final trajectories for boundary violations
        max_velocity_with_slack_check = self.params.max_velocity * 1.5
        max_accel_with_slack_check = self.params.max_acceleration * 1.5
        max_decel_with_slack_check = self.params.max_acceleration * 2.0

        all_neg_vel = np.sum(s_dot_array < 0)
        all_over_max_vel = np.sum(s_dot_array > max_velocity_with_slack_check)
        all_below_min_accel = np.sum(
            s_ddot_array < -max_decel_with_slack_check)
        all_over_max_accel = np.sum(s_ddot_array > max_accel_with_slack_check)

        # Only log critical violations (negative velocities should never occur)
        if all_neg_vel > 0:
            neg_vel_mask = s_dot_array < 0
            min_neg_vel = np.min(s_dot_array[neg_vel_mask])
            rospy.logerr(
                f"[calc_samples_s_based] CRITICAL: {all_neg_vel} negative velocity points detected, "
                f"worst: {min_neg_vel:.3f} m/s")

        return s_array, s_dot_array, s_ddot_array, s_dot_end_values, s_end_values, rel_long_sampling_array, t_array

    def calc_samples_s_based_forward_backward(
        self,
        s_start: float,
        s_dot_start: float,
        s_ddot_start: float,
        n_start: float,
        V_target: float,
        postprocessed_raceline: dict,
        track_handler: Track,
        gggv_handler=None,  # F1TENTH: Not used, kept for compatibility
        pitlane_mode: bool = False,
        raceline_tendency: bool = False
    ):
        """
        F1TENTH VERSION: Forward-backward velocity sampling with Pacejka tire model.

        Uses Pacejka tire model to calculate physics-based acceleration limits that
        account for lateral acceleration from cornering (friction circle model).

        Key features:
        - Velocity-dependent limits based on track curvature
        - Combined slip modeling (lateral + longitudinal forces)
        - Weight transfer considerations (front/rear axle loads)
        - Falls back to fixed limits if tire model unavailable

        Suitable for F1TENTH planar racing with realistic tire dynamics.
        """

        # only generate forward integration samples when below target speed
        if (s_dot_start < V_target):  # or True
            num_long_profiles = 2 * self.params.samples_forward_backward
        else:
            num_long_profiles = self.params.samples_forward_backward

        n_samples = self.params.n_samples + self.params.n_dense_samples

        # themporary s array with raceline locs, get pruned at the t_horizon
        s_arr_ref = np.copy(postprocessed_raceline["s_post"])
        s_dot_arr_ref = np.copy(postprocessed_raceline["s_dot_post"])
        # make s_arr_ref monotonically increasing
        if s_arr_ref[0] > s_arr_ref[-1]:
            multiplier = np.abs(
                np.floor((2*s_arr_ref[0]) / track_handler.get_track_length())-1)
            s_arr_ref = s_arr_ref + multiplier * track_handler.get_track_length()

        # add s_start to s_arr_ref if not in raceline
        if s_start < s_arr_ref[0]:
            s_arr_ref = np.insert(s_arr_ref, 0, s_start)
            s_dot_arr_ref = np.insert(s_dot_arr_ref, 0, s_dot_start)

        # extend s_arr_ref if to short !!!! DONT COPY INVESTIGATE WHY REQUIRED
        if np.shape(s_arr_ref)[0] < self.params.num_samples + 10:
            # extension assumes static step size, but end of horizon so ok even if real rl uneven
            step_size = s_arr_ref[1] - s_arr_ref[0]
            s_arr_ref = np.append(s_arr_ref, s_arr_ref[-1] + step_size + np.linspace(
                0, self.params.num_samples * 4 * step_size, 4 * self.params.num_samples))

        # extend s_dot_arr_ref if to short !!!! DONT COPY INVESTIGATE WHY REQUIRED
        if np.shape(s_dot_arr_ref)[0] < self.params.num_samples + 10:
            s_dot_arr_ref = np.append(s_dot_arr_ref, np.ones(
                self.params.num_samples * 4)*s_dot_arr_ref[-1])

        s_arr_temp = np.copy(s_arr_ref)
        # assign array sizes depending on number of s_dot end values
        target_shape = (num_long_profiles * n_samples, self.params.num_samples)

        s_array = np.zeros(target_shape)
        s_dot_array = np.zeros(target_shape)
        s_ddot_array = np.zeros(target_shape)
        # save which samples are absolute
        rel_long_sampling_array = np.zeros(target_shape[0])
        t_array = np.zeros(target_shape)
        s_dot_end_values = np.zeros(num_long_profiles)
        s_end_values = np.zeros(num_long_profiles)

        # Fill out put arrays with scaled profiles
        scales = np.linspace(self.params.forward_backward_min_scale,
                             self.params.forward_backward_max_scale, self.params.samples_forward_backward)

        # get fastest accelerating profile
        ##############################################################################################################
        if s_dot_start < V_target:
            for j in range(self.params.samples_forward_backward, self.params.samples_forward_backward*2):
                t_cumulative = 0.0
                s_dot_current = max(s_dot_start, self.params.s_dot_end_min)

                s_vec_local = np.zeros_like(s_arr_temp)
                s_dot_vec_local = np.zeros_like(s_arr_temp)
                s_ddot_vec_local = np.zeros_like(s_arr_temp)
                t_vec_local = np.zeros_like(s_arr_temp)

                for i in range(np.shape(s_arr_ref)[0]):

                    s_vec_local[i] = s_arr_ref[i]
                    s_dot_vec_local[i] = s_dot_current
                    t_vec_local[i] = t_cumulative

                    # Calculate s_step with proper wraparound handling
                    # Use modulo arithmetic to always get positive forward distance
                    track_length = track_handler.get_track_length()
                    s_step = (s_arr_ref[1+i] - s_arr_ref[i]) % track_length

                    # Handle the case where modulo gives us nearly track_length (backward wrap)
                    # We want the shorter distance, so if > half track, it's actually backward
                    if s_step > 0.5 * track_length:
                        s_step = s_step - track_length
                        if not wraparound_detected:
                            wraparound_detected = True
                            wraparound_step = i

                    # Safety check: s_step should always be positive and non-zero
                    if s_step <= 1e-6:
                        # Use minimum step size based on typical raceline resolution
                        s_step = 0.1

                    # F1TENTH: Use Pacejka tire model for acceleration limits
                    _, ax_avail_max_tilde = self.__calc_ax_avail_tire_limits(
                        s_arr_ref[i+1], s_dot_current, track_handler)
                    ax_avail_scaled = ax_avail_max_tilde * \
                        scales[j-self.params.samples_forward_backward]
                    # s_dot_next = s_dot_current + ax_avail_max_tilde * scales[j-self.params.samples_forward_backward] * (s_step/s_dot_current)  #constant vel approx not constant ax
                    s_dot_next = np.sqrt(
                        s_dot_current**2 + 2 * ax_avail_scaled * s_step)
                    # t_cumulative += (s_step) / s_dot_current #constant vel approx not constant ax

                    max_s_dot = min(max(V_target, self.params.s_dot_end_min), (s_dot_arr_ref[i] + (
                        self.params.forward_backward_max_v_to_rl_delta)) * scales[j-self.params.samples_forward_backward])
                    if s_dot_next > max_s_dot:

                        # s_ddot_vec_local[i] = (s_dot_next - s_dot_current) / ((s_vec_local[i] - s_vec_local[i-1])/s_dot_current) #const vel expression not const ax
                        s_ddot_vec_local[i] = (
                            max_s_dot**2 - s_dot_current**2) / (2*s_step)
                        # forward acceleration breaks when decreasing vel so at slowest keep const
                        if s_ddot_vec_local[i] < 0.0:
                            s_ddot_vec_local[i] = 0.0
                            s_dot_next = s_dot_current
                        else:
                            s_dot_next = max_s_dot

                    else:
                        # (s_dot_next - s_dot_current) / (t_cumulative-t_vec_forward[i])
                        s_ddot_vec_local[i] = ax_avail_scaled

                    # Time increment calculation
                    if np.abs(s_ddot_vec_local[i]) > 1e-5:
                        t_cumulative += (s_dot_next -
                                         s_dot_current) / s_ddot_vec_local[i]
                    else:
                        if s_dot_current > 1e-6:
                            t_cumulative += s_step / s_dot_current

                    s_dot_current = s_dot_next

                    # Distance-based cutoff for acceleration profiles
                    # Calculate total distance traveled with wraparound handling
                    s_distance = s_vec_local[i] - s_vec_local[0]
                    if s_distance < 0:
                        s_distance += track_handler.get_track_length()

                    # Break when we've traveled trajectory_length distance AND have enough samples
                    if s_distance >= self.params.tracjectory_length and i >= (self.params.num_samples-1):
                        break
                    # Also break if we run out of reference data
                    if i >= (np.shape(s_arr_ref)[0]-2):
                        break

                rel_long_sampling_array[j *
                                        n_samples:  (j+1) * n_samples] = False

                trim_mask = np.zeros_like(s_vec_local, dtype=bool)
                trim_mask[:(i+1)] = create_trim_mask(s_vec_local[:(i+1)],
                                                     self.params.num_samples)

                s_array[j*n_samples:  (j+1) * n_samples,
                        :] = np.tile(s_vec_local[trim_mask], (n_samples, 1))
                s_dot_array[j*n_samples:  (j+1) * n_samples, :] = np.tile(
                    s_dot_vec_local[trim_mask], (n_samples, 1))
                s_ddot_array[j*n_samples:  (j+1) * n_samples, :] = np.tile(
                    s_ddot_vec_local[trim_mask], (n_samples, 1))
                t_array[j*n_samples:  (j+1) * n_samples,
                        :] = np.tile(t_vec_local[trim_mask], (n_samples, 1))

        ##############################################################################################################

        # get fastest decelerating profile
        ##############################################################################################################
        if True:
            for j in range(self.params.samples_forward_backward):
                t_cumulative = 0.0
                s_dot_current = s_dot_start

                s_vec_local = np.zeros_like(s_arr_temp)
                s_dot_vec_local = np.zeros_like(s_arr_temp)
                s_ddot_vec_local = np.zeros_like(s_arr_temp)
                t_vec_local = np.zeros_like(s_arr_temp)

                for i in range(np.shape(s_arr_ref)[0]):
                    s_vec_local[i] = s_arr_ref[i]
                    s_dot_vec_local[i] = s_dot_current
                    t_vec_local[i] = t_cumulative

                    # Calculate s_step with proper wraparound handling
                    # Use modulo arithmetic to always get positive forward distance
                    track_length = track_handler.get_track_length()
                    s_step = (s_arr_ref[1+i] - s_arr_ref[i]) % track_length

                    # Handle the case where modulo gives us nearly track_length (backward wrap)
                    # We want the shorter distance, so if > half track, it's actually backward
                    if s_step > 0.5 * track_length:
                        s_step = s_step - track_length
                        if not wraparound_detected:
                            wraparound_detected = True
                            wraparound_step = i

                    # Safety check: s_step should always be positive and non-zero
                    if s_step <= 1e-6:
                        # Use minimum step size based on typical raceline resolution
                        s_step = 0.1

                    # F1TENTH: Use Pacejka tire model for deceleration limits
                    ax_avail_min_tilde, _ = self.__calc_ax_avail_tire_limits(
                        s_arr_ref[i+1], s_dot_current, track_handler)
                    ax_avail_scaled = -np.abs(ax_avail_min_tilde) * scales[j]
                    # s_dot_next = s_dot_current -np.abs(ax_avail_min_tilde) * scales[j] * (s_step/s_dot_current)  #constant vel approx not constant ax
                    discriminant = s_dot_current**2 + 2 * ax_avail_scaled * s_step
                    if discriminant > 0:
                        s_dot_next = np.sqrt(discriminant)
                    else:
                        s_dot_next = 0.0

                    # below the threshold speed to stop without asymptotic behavior
                    s_dot_min = self.params.s_dot_end_min*0.3
                    if s_dot_next < s_dot_min:

                        # s_ddot_vec_local[i] = (s_dot_next - s_dot_current) / ((s_vec_local[i] - s_vec_local[i-1])/s_dot_current) #const vel expression not const ax
                        s_ddot_vec_local[i] = (
                            s_dot_min**2 - s_dot_current**2) / (2*s_step)
                        s_dot_next = s_dot_min

                    else:
                        # (s_dot_next - s_dot_current) / (t_cumulative-t_vec_forward[i])
                        s_ddot_vec_local[i] = ax_avail_scaled

                    # Time increment calculation
                    if np.abs(s_ddot_vec_local[i]) > 1e-5:
                        t_cumulative += (s_dot_next -
                                         s_dot_current) / s_ddot_vec_local[i]
                    else:
                        if s_dot_current > 1e-6:
                            t_cumulative += s_step / s_dot_current

                    s_dot_current = s_dot_next

                    # Distance-based cutoff for deceleration profiles
                    # Calculate total distance traveled with wraparound handling
                    s_distance = s_vec_local[i] - s_vec_local[0]
                    if s_distance < 0:
                        s_distance += track_handler.get_track_length()

                    # Break when we've traveled trajectory_length distance AND have enough samples
                    if s_distance >= self.params.tracjectory_length and i >= (self.params.num_samples-1):
                        break
                    # Also break if we run out of reference data
                    if i >= (np.shape(s_arr_ref)[0]-2):
                        break

                rel_long_sampling_array[j *
                                        n_samples:  (j+1) * n_samples] = False

                trim_mask = np.zeros_like(s_vec_local, dtype=bool)
                trim_mask[:(i+1)] = create_trim_mask(s_vec_local[:(i+1)],
                                                     self.params.num_samples)

                s_array[j*n_samples:  (j+1) * n_samples,
                        :] = np.tile(s_vec_local[trim_mask], (n_samples, 1))
                s_dot_array[j*n_samples:  (j+1) * n_samples, :] = np.tile(
                    s_dot_vec_local[trim_mask], (n_samples, 1))
                s_ddot_array[j*n_samples:  (j+1) * n_samples, :] = np.tile(
                    s_ddot_vec_local[trim_mask], (n_samples, 1))
                t_array[j*n_samples:  (j+1) * n_samples,
                        :] = np.tile(t_vec_local[trim_mask], (n_samples, 1))

        ##############################################################################################################
        s_dot_end_values = s_dot_array[0::n_samples, -1]
        s_end_values = s_array[0::n_samples, -1]

        return s_array, s_dot_array, s_ddot_array, s_dot_end_values, s_end_values, rel_long_sampling_array, t_array

    def calc_time_vector(
            self,
            track_handler,
            s: np.ndarray,
            n: np.ndarray,
            V: np.ndarray,
            raceline_tendency: bool = False

    ):
        """
        Calculate time vector from arc length and velocity arrays.

        CRITICAL: Uses arc length differences (not Cartesian distance) to handle
        lap wraparound correctly. Clamps velocities to prevent division by zero.

        Args:
            track_handler: Track handler for wraparound detection
            s: Arc length array [m]
            n: Lateral offset array [m] (not used in distance calculation)
            V: Velocity array [m/s]

        Returns:
            t_vector: Time array [s] with t[0] = 0
        """
        if len(s) < 2:
            return np.array([0.0])

        # Use arc length differences instead of Cartesian distance
        # This correctly handles lap wraparound
        ds = np.diff(s)

        # Handle wraparound: if ds is very negative, we wrapped around the track
        track_length = track_handler.get_track_length()
        wraparound_mask = ds < -track_length / 2.0
        if np.any(wraparound_mask):
            ds[wraparound_mask] += track_length

        # Calculate mean velocity between points
        V_mean = (V[:-1] + V[1:]) / 2.0

        # # CRITICAL: Clamp velocities to prevent division by zero or negative values
        # # If polynomial fitting produced bad velocities, this prevents catastrophic failure
        # V_mean_safe = np.maximum(V_mean, 0.1)  # Minimum 0.1 m/s

        # # Check if we had to clamp any velocities (indicates upstream problem)
        # if np.any(V_mean < 0.1):
        #     num_clamped = np.sum(V_mean < 0.1)
        #     min_v = np.min(V_mean)
        #     rospy.logerr(f"LongitudinalSampling.calc_time_vector: Clamped {num_clamped} velocity values "
        #                  f"(min={min_v:.3f} m/s). This indicates polynomial fitting produced invalid velocities.")

        # Calculate time increments: dt = ds / v_mean
        dt = ds / V_mean

        # Cumulative sum to get absolute time array
        t_vector = np.concatenate(([0.0], np.cumsum(dt)))

        # Validate monotonicity (should always be true with positive ds and V)
        if not np.all(np.diff(t_vector) >= 0):
            rospy.logerr(
                "LongitudinalSampling.calc_time_vector: Time array not monotonically increasing!")
            rospy.logerr(
                f"  Time jumps backward at indices: {np.where(np.diff(t_vector) < 0)[0]}")
            rospy.logerr(f"  ds range: [{np.min(ds):.3f}, {np.max(ds):.3f}]")
            rospy.logerr(
                f"  V_mean range: [{np.min(V_mean):.3f}, {np.max(V_mean):.3f}]")
            rospy.logerr(f"  raceline_tendency: {raceline_tendency}")

        return t_vector

    def __calc_ax_avail_tire_limits(self, s: float, V: float, track_handler: Track) -> tuple:
        """
        Calculate acceleration limits using Pacejka tire model with friction circle.

        Args:
            s: Arc length position [m]
            V: Current velocity [m/s]
            track_handler: Track handler for curvature information

        Returns:
            (ax_avail_min, ax_avail_max): Available acceleration limits [m/s²]
        """
        if not self.use_tire_model or self.tire_model is None:
            # Fallback to fixed limits if tire model unavailable
            return -self.params.max_acceleration, self.params.max_acceleration

        try:
            # Calculate lateral acceleration from track curvature
            # ay = V² * κ (centripetal acceleration)
            kappa = track_handler.omega_z(s)  # Curvature [1/m]
            ay_current = V**2 * kappa  # [m/s²]

            # Get normal loads considering static weight distribution
            # (simplified - no pitch dynamics in F1TENTH)
            g = 9.81
            Fz_front_static = self.tire_model.mass * g * self.tire_model.l_r / \
                (self.tire_model.l_f + self.tire_model.l_r)
            Fz_rear_static = self.tire_model.mass * g * self.tire_model.l_f / \
                (self.tire_model.l_f + self.tire_model.l_r)

            # Calculate available longitudinal force considering lateral usage
            Fx_available, _ = self.tire_model.calc_combined_limits(
                Fz_front_static, Fz_rear_static, ay_current
            )

            # Convert force to acceleration
            ax_available = Fx_available / self.tire_model.mass

            # Return symmetric limits (braking and acceleration)
            # In reality, braking is often stronger, but simplified here
            ax_avail_max = ax_available
            ax_avail_min = -ax_available

            # Clamp to reasonable bounds to prevent numerical issues
            ax_avail_max = min(
                ax_avail_max, self.params.max_acceleration * 2.0)
            ax_avail_min = max(
                ax_avail_min, -self.params.max_acceleration * 2.0)

            return ax_avail_min, ax_avail_max

        except Exception as e:
            rospy.logwarn_throttle(
                5.0, f"Error calculating tire limits: {e}, using fallback")
            return -self.params.max_acceleration, self.params.max_acceleration

    # def __calc_ax_avail(self, s, n, chi, V, Omega_z, track_handler, gggv_handler, pitlane_mode):
    #     ay_hat = V**2 * track_handler.omega_z(s)
    #     _, ay_tilde, g_tilde = track_handler.calc_apparent_acceleration(
    #         s,
    #         n,
    #         chi,
    #         0.0,  # ax_hat not required for ay_tilde and g_tilde
    #         ay_hat,
    #         V,
    #     )

    #     _, ax_min_tilde, ax_max_tilde, ay_max_tilde, _ = gggv_handler.acc_interpolator(
    #         np.array(V), np.array(g_tilde), np.array(
    #             s), np.array(n), not pitlane_mode, False
    #     )
    #     ax_avail_min_tilde = -np.abs(ax_min_tilde) * np.power(
    #         max((1.0 - np.power(min(np.abs(ay_tilde) / (ay_max_tilde), 1.0),
    #             gggv_handler.gg_exponent_ax_neg)), 1e-4), 1.0 / gggv_handler.gg_exponent_ax_neg
    #     )

    #     ax_avail_max_tilde = np.abs(ax_max_tilde) * np.power(
    #         max((1.0 - np.power(min(np.abs(ay_tilde) / (ay_max_tilde), 1.0),
    #             gggv_handler.gg_exponent_ax_neg)), 1e-4), 1.0 / gggv_handler.gg_exponent_ax_neg
    #     )

    #     # DEBUG -- redeclairation of the linspace -> slow
    #     ax_machine_lim = np.interp(V, np.linspace(
    #         0.0, 90.0, 10), gggv_handler.ax_machine_limits)
    #     ax_avail_max_tilde = np.minimum(ax_machine_lim, ax_avail_max_tilde)
    #     # print('ax_max_tilde', ax_avail_max_tilde)
    #     return ax_avail_min_tilde, ax_avail_max_tilde, ay_tilde, g_tilde
