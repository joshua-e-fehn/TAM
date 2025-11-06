#!/usr/bin/env python3
"""
TAM Lateral Sampling Module
Quintic polynomial lateral trajectory generation following original TAM approach

Ported from tam_race_stack/mod_planning/sampling_planner/sampling_planner/lateral_sampling.py
"""

from track_handler_global_waypoints import GlobalWaypointsTrackHandler as Track
import numpy as np
from dataclasses import dataclass
import rospy
from simple_helper_utils import interpolate_with_period


@dataclass(init=False)
class LatSamplingParams():
    """
    Data class containing all lateral sampling parameters.

    No constructor parameters - all fields are populated by declare_and_update_parameters().
    """
    n_samples: int                    # Number of lateral samples to generate
    # Minimum dense sampling offset relative to raceline [m]
    n_dense_min: float
    # Maximum dense sampling offset relative to raceline [m]
    n_dense_max: float
    n_dense_samples: int             # Number of dense samples around raceline
    # Safety margin from left track boundary [m]
    safety_distance_track_left: float
    # Safety margin from right track boundary [m]
    safety_distance_track_right: float
    tube_width: float                # Tube width for trajectory planning [m]
    # Use geometric lateral end velocity (position-dependent) vs zero velocity
    use_geometric_lateral_end_velocity: bool


class LateralSampling:
    def __init__(self, debugging=False):
        """
        Initialize the LateralSampling class.

        No input parameters required - all configuration comes from ROS parameter server.
        """
        self.params = LatSamplingParams()
        self.initialized_params = False
        self.declare_and_update_parameters()

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
                f"LateralSampling: Could not load YAML defaults: {e}")
            return {}

    def declare_and_update_parameters(self):
        """
        Update parameters from ROS parameter server with YAML defaults.

        No input parameters - reads from ROS parameter server using rospy.get_param().
        Updates self.params with latest parameter values.
        """
        if not self.initialized_params:
            yaml_defaults = self._load_yaml_defaults()

            self.params.n_samples = yaml_defaults.get(
                'lateral_samples', rospy.get_param("discretization/n_samples", 20))
            rospy.set_param("discretization/n_samples", self.params.n_samples)
            self.params.n_dense_min = yaml_defaults.get(
                'n_dense_min', rospy.get_param("discretization/n_dense_min", -0.5))
            rospy.set_param("discretization/n_dense_min",
                            self.params.n_dense_min)
            self.params.n_dense_max = yaml_defaults.get(
                'n_dense_max', rospy.get_param("discretization/n_dense_max", 0.5))
            rospy.set_param("discretization/n_dense_max",
                            self.params.n_dense_max)
            self.params.n_dense_samples = yaml_defaults.get(
                'n_dense_samples', rospy.get_param("discretization/n_dense_samples", 5))
            rospy.set_param("discretization/n_dense_samples",
                            self.params.n_dense_samples)
            self.params.safety_distance_track_left = yaml_defaults.get(
                'safety_distance_track_left',
                rospy.get_param("safety_distances/safety_distance_track_left", 0.5))
            rospy.set_param("safety_distances/safety_distance_track_left",
                            self.params.safety_distance_track_left)
            self.params.safety_distance_track_right = yaml_defaults.get(
                'safety_distance_track_right',
                rospy.get_param("safety_distances/safety_distance_track_right", 0.5))
            rospy.set_param("safety_distances/safety_distance_track_right",
                            self.params.safety_distance_track_right)
            self.params.tube_width = yaml_defaults.get(
                'tube_width', rospy.get_param("behavior/tube_width", 1.0))
            rospy.set_param("behavior/tube_width", self.params.tube_width)
            self.params.use_geometric_lateral_end_velocity = yaml_defaults.get(
                'use_geometric_lateral_end_velocity',
                rospy.get_param("behavior/use_geometric_lateral_end_velocity", False))
            rospy.set_param("behavior/use_geometric_lateral_end_velocity",
                            self.params.use_geometric_lateral_end_velocity)
            self.initialized_params = True
        else:
            self.params.n_samples = rospy.get_param(
                "discretization/n_samples", self.params.n_samples)
            self.params.n_dense_min = rospy.get_param(
                "discretization/n_dense_min", self.params.n_dense_min)
            self.params.n_dense_max = rospy.get_param(
                "discretization/n_dense_max", self.params.n_dense_max)
            self.params.n_dense_samples = rospy.get_param(
                "discretization/n_dense_samples", self.params.n_dense_samples)
            self.params.safety_distance_track_left = rospy.get_param(
                "safety_distances/safety_distance_track_left",
                self.params.safety_distance_track_left)
            self.params.safety_distance_track_right = rospy.get_param(
                "safety_distances/safety_distance_track_right",
                self.params.safety_distance_track_right)
            self.params.tube_width = rospy.get_param(
                "behavior/tube_width", self.params.tube_width)
            self.params.use_geometric_lateral_end_velocity = rospy.get_param(
                "behavior/use_geometric_lateral_end_velocity",
                self.params.use_geometric_lateral_end_velocity)

    def _compute_improved_derivatives(self, waypoints, s_coords, vx_coords, d_coords, kappa_coords):
        """
        Compute improved derivative approximations from waypoint data.

        Args:
            waypoints: List of waypoint dictionaries
            s_coords: Arc length coordinates [m]
            vx_coords: Velocity coordinates [m/s] 
            d_coords: Lateral offset coordinates [m]
            kappa_coords: Curvature coordinates [rad/m]

        Returns:
            dict: Dictionary containing improved derivative approximations
        """
        derivatives = {}

        # Improved velocity derivative for s_ddot
        derivatives['vx_grad'] = np.gradient(vx_coords, s_coords)  # dv/ds

        # Lateral derivatives from d_m data
        derivatives['d_grad'] = np.gradient(d_coords, s_coords)    # dn/ds
        derivatives['d_grad2'] = np.gradient(
            derivatives['d_grad'], s_coords)  # d²n/ds²

        # Try to compute improved chi from geometry
        try:
            x_coords = np.array([wp.get("x_m", 0) for wp in waypoints])
            y_coords = np.array([wp.get("y_m", 0) for wp in waypoints])
            if np.any(x_coords != 0) or np.any(y_coords != 0):
                derivatives['heading_angles'] = np.arctan2(
                    np.gradient(y_coords, s_coords),
                    np.gradient(x_coords, s_coords)
                )
                derivatives['use_geometric_chi'] = True
            else:
                derivatives['use_geometric_chi'] = False
        except (KeyError, ValueError):
            derivatives['use_geometric_chi'] = False

        return derivatives

    def _extend_raceline_for_wraparound(self, postprocessed_raceline, track_length):
        """
        Extend raceline data to handle wraparound interpolation at track boundaries.

        Creates extended arrays with data prepended (s - track_length) and 
        appended (s + track_length) to allow smooth interpolation across 
        the track start/end boundary without discontinuities.

        Args:
            postprocessed_raceline: Dictionary with '_post' suffixed raceline data
            track_length: Total track length [m]

        Returns:
            Dictionary with extended arrays for wraparound-safe interpolation
        """
        extended = {}

        # Extend s coordinates: [s-L, s, s+L]
        s_coords = postprocessed_raceline['s_post']
        s_before = s_coords - track_length  # Data for wraparound from end
        s_after = s_coords + track_length   # Data for wraparound to start
        extended['s_post'] = np.concatenate([s_before, s_coords, s_after])

        # Extend all other arrays by triplicating (periodic data)
        # These arrays represent physical quantities that repeat each lap
        for key in ['v_post', 's_dot_post', 's_ddot_post', 'n_post',
                    'n_dot_post', 'n_ddot_post', 'chi_post']:
            if key in postprocessed_raceline:
                arr = postprocessed_raceline[key]
                extended[key] = np.concatenate([arr, arr, arr])

        return extended

    def calc_samples(
            self,
            # Initial arc length position along the track centerline [m]
            s_start: float,
            # Initial longitudinal velocity (ds/dt) [m/s]
            s_dot_start: float,
            # Array of arc length trajectories for each sample [m] - shape: (n_samples, n_points)
            s_array: np.array,
            # Array of longitudinal velocity trajectories [m/s] - shape: (n_samples, n_points)
            s_dot_array: np.array,
            # Array of longitudinal acceleration trajectories [m/s²] - shape: (n_samples, n_points)
            s_ddot_array: np.array,
            # Target end velocities for each sample [m/s] - shape: (n_samples,)
            s_dot_end_values: np.array,
            # Target end arc length positions for each sample [m] - shape: (n_samples,)
            s_end_values: np.array,
            # Initial lateral offset from track centerline [m] (positive = left)
            n_start: float,
            # Initial lateral velocity (dn/dt) [m/s]
            n_dot_start: float,
            # Initial lateral acceleration (d²n/dt²) [m/s²]
            n_ddot_start: float,
            # Time arrays for each sample trajectory [s] - shape: (n_samples, n_points)
            t_array: np.ndarray,
            # Postprocessed raceline (from postprocess_raceline method)
            # Dictionary with '_post' suffixed keys: 's_post', 'n_post', 's_dot_post',
            # 's_ddot_post', 'n_dot_post', 'n_ddot_post', 'v_post', 'chi_post',
            # 'ax_post', 'ay_post', 't_post', 'kappa_post', 'x_post', 'y_post'
            postprocessed_raceline: dict,
            # If True, sample relative to raceline; if False, sample absolute trajectories
            raceline_tendency: bool,
            # Track
            # Track handler object providing track geometry and boundaries
            track_handler: Track,
            # Vehicle
            # Vehicle parameters dict containing 'total_width' and other vehicle dimensions
            vehicle_params: dict,
    ):

        self.declare_and_update_parameters()

        n_array = np.zeros_like(s_array)
        n_dot_array = np.zeros_like(s_array)
        n_ddot_array = np.zeros_like(s_array)

        i = 0
        for s_dot_end, s_end in zip(s_dot_end_values, s_end_values):
            # t_array evaluation
            # precompute the polynomial values of the time
            ones = np.ones_like(t_array[i])
            t_quad = t_array[i] ** 2
            t_cubi = t_array[i] * t_quad
            t_quar = t_array[i] * t_cubi
            t_quin = t_array[i] * t_quar
            t_mat = np.column_stack(
                (ones, t_array[i], t_quad, t_cubi, t_quar, t_quin))
            t_mat_dot = np.column_stack(
                (ones, 2*t_array[i], 3*t_quad, 4*t_cubi, 5*t_quar))
            t_mat_ddot = np.column_stack(
                (2*ones, 6*t_array[i], 12*t_quad, 20*t_cubi))

            # formulate matrix a of linear system of equations
            t_end = t_array[i][-1]

            # Check for invalid or degenerate time horizon
            # Negative time means the input trajectory data is corrupted or invalid
            # Very small positive time makes the polynomial problem ill-conditioned
            if t_end < 0.0:
                rospy.logerr(
                    f"LateralSampling: INVALID negative time horizon ({t_end:.4f}s) detected! "
                    f"t_array[{i}] range: [{t_array[i][0]:.4f}, {t_end:.4f}]. "
                    "This indicates corrupted input data. Skipping this trajectory."
                )
                # Skip this invalid trajectory by not incrementing stored results
                continue
            elif t_end < 0.01:  # Less than 10ms
                rospy.logwarn_throttle(
                    1.0,
                    f"LateralSampling: Very short time horizon ({t_end:.4f}s) detected. "
                    "Trajectories may be numerically unstable."
                )

            a = np.array([[1, 0, 0, 0, 0, 0],
                          [0, 1, 0, 0, 0, 0],
                          [0, 0, 2, 0, 0, 0],
                          [1, t_end, t_end ** 2, t_end **
                              3, t_end ** 4, t_end ** 5],
                          [0, 1, 2 * t_end, 3 * t_end ** 2,
                              4 * t_end ** 3, 5 * t_end ** 4],
                          [0, 0, 2, 6 * t_end, 12 * t_end ** 2, 20 * t_end ** 3]])

            # calculate the inverse of a for solving the linear equation system in for loop just with a matrix multiplication
            a_inverse = np.linalg.inv(a)

            # evaluate raceline at specific s points
            s_dot_rl = interpolate_with_period(
                s_array[i],
                postprocessed_raceline["s_post"],
                postprocessed_raceline["s_dot_post"],
                period=track_handler.get_track_length(),
            )
            s_ddot_rl = interpolate_with_period(
                s_array[i],
                postprocessed_raceline["s_post"],
                postprocessed_raceline["s_ddot_post"],
                period=track_handler.get_track_length(),
            )
            n_rl = interpolate_with_period(
                s_array[i],
                postprocessed_raceline["s_post"],
                postprocessed_raceline["n_post"],
                period=track_handler.get_track_length(),
            )
            n_dot_rl = interpolate_with_period(
                s_array[i],
                postprocessed_raceline["s_post"],
                postprocessed_raceline["n_dot_post"],
                period=track_handler.get_track_length(),
            )
            n_ddot_rl = interpolate_with_period(
                s_array[i],
                postprocessed_raceline["s_post"],
                postprocessed_raceline["n_ddot_post"],
                period=track_handler.get_track_length(),
            )

            chi_rl = interpolate_with_period(
                s_array[i],
                postprocessed_raceline["s_post"],
                postprocessed_raceline["chi_post"],
                period=track_handler.get_track_length(),
            )

            # Clamp velocities to prevent division by zero/near-zero
            # This prevents numerical instabilities when raceline velocity is very small
            # Minimum threshold: 0.1 m/s (equivalent to ~0.36 km/h)
            s_dot_rl_safe = np.maximum(s_dot_rl, 0.1)
            s_dot_array_safe = np.maximum(s_dot_array[i], 0.1)

            n_rl_eval = n_rl
            n_dot_rl_eval = n_dot_rl / s_dot_rl_safe * s_dot_array_safe
            n_ddot_rl_eval = (
                n_ddot_rl / (s_dot_rl_safe**2) * (s_dot_array_safe ** 2)
                - n_dot_rl / (s_dot_rl_safe**3) * s_ddot_rl *
                (s_dot_array_safe ** 2)
                + n_dot_rl / s_dot_rl_safe * s_ddot_array[i]
            )

            n_dense_end_values = np.linspace(
                n_rl_eval[-1] + self.params.n_dense_min, n_rl_eval[-1] + self.params.n_dense_max, self.params.n_dense_samples)

            # sampled n end conditions(relative to raceline)
            # In Frenet coordinates: positive n = left, negative n = right
            # Right boundary (negative n): -trackwidth_right + margins
            trackwidth_right_raw = track_handler.trackwidth_right(s_end)
            trackwidth_left_raw = track_handler.trackwidth_left(s_end)

            n_min_track = -(trackwidth_right_raw - vehicle_params['total_width'] /
                            2.0 - self.params.tube_width - self.params.safety_distance_track_right)
            # Left boundary (positive n): +trackwidth_left - margins
            n_max_track = trackwidth_left_raw - \
                vehicle_params['total_width'] / 2.0 - self.params.tube_width - \
                self.params.safety_distance_track_left

            n_end_min = n_min_track
            n_end_max = n_max_track
            n_end_values = np.concatenate((np.linspace(n_end_min, n_end_max, self.params.n_samples - 1), [
                                          n_rl_eval[-1]], n_dense_end_values))  # always sample straight driving and raceline

            # Calculate geometric lateral end velocities for track boundaries if enabled
            # This provides position-dependent end conditions based on track geometry
            if self.params.use_geometric_lateral_end_velocity:
                # Calculate track boundary headings at left and right bounds
                xy_left = track_handler.sn2cartesian(
                    s_end, track_handler.trackwidth_left(s_end))
                xy_left_prev = track_handler.sn2cartesian(
                    s_end - 1.0, track_handler.trackwidth_left(s_end - 1.0))
                psi_left = np.arctan2(
                    (xy_left[1] - xy_left_prev[1]), (xy_left[0] - xy_left_prev[0]))

                xy_right = track_handler.sn2cartesian(
                    s_end, -track_handler.trackwidth_right(s_end))
                xy_right_prev = track_handler.sn2cartesian(
                    s_end - 1.0, -track_handler.trackwidth_right(s_end - 1.0))
                psi_right = np.arctan2(
                    (xy_right[1] - xy_right_prev[1]), (xy_right[0] - xy_right_prev[0]))

                chi_left = track_handler.calc_chi_from_2d_heading(
                    s_end, psi_left)
                chi_right = track_handler.calc_chi_from_2d_heading(
                    s_end, psi_right)

                V_end_left = (
                    1.0 - n_max_track * track_handler.omega_z(s_end)) * s_dot_end / np.cos(chi_left)
                V_end_right = (
                    1.0 - n_min_track * track_handler.omega_z(s_end)) * s_dot_end / np.cos(chi_right)

                n_dot_end_left = V_end_left * np.sin(chi_left)
                n_dot_end_right = V_end_right * np.sin(chi_right)

            # n_prime_end = n_dot_end / s_dot_end
            for n_end in n_end_values:
                # Calculate lateral end velocity based on mode
                if self.params.use_geometric_lateral_end_velocity:
                    # Position-dependent: interpolate between boundary velocities
                    # Note: In Frenet coordinates, negative n = right, positive n = left
                    n_dot_end = np.interp(n_end, [n_min_track, 0.0, n_max_track],
                                          [n_dot_end_right, 0.0, n_dot_end_left])
                else:
                    # Simple: zero lateral velocity (perpendicular arrival to centerline)
                    n_dot_end = 0.0

                if raceline_tendency:  # sample curves relative to raceline
                    b = np.array(
                        [
                            n_start - n_rl_eval[0],
                            n_dot_start - n_dot_rl_eval[0],
                            n_ddot_start - n_ddot_rl_eval[0],
                            n_end - n_rl_eval[-1],
                            n_dot_end - n_dot_rl_eval[-1],
                            0.0,
                        ]
                    )
                else:  # sample curves absolute
                    b = np.array(
                        [n_start, n_dot_start, n_ddot_start, n_end, n_dot_end, 0.0])

                # calculate coefficients of quintic polynomial(now as matrix multiplication)
                c = a_inverse @ b

                # calculate sampled n curve (now as matrix multiplication)
                n_sample = t_mat @ c
                n_dot_sample = t_mat_dot @ c[1:]
                n_ddot_sample = t_mat_ddot @ c[2:]

                if raceline_tendency:
                    # add raceline n data to sampled relative n curve
                    n = n_sample + n_rl_eval
                    n_dot = n_dot_sample + n_dot_rl_eval
                    n_ddot = n_ddot_sample + n_ddot_rl_eval
                else:
                    n = n_sample
                    n_dot = n_dot_sample
                    n_ddot = n_ddot_sample

                # Sanity check: detect and reject extreme outlier trajectories
                # Check for unreasonable lateral offsets (>100m suggests numerical instability)
                max_n_abs = np.max(np.abs(n))
                if max_n_abs > 100.0:
                    rospy.logwarn_throttle(
                        5.0,
                        f"LateralSampling: Extreme lateral trajectory detected (max |n|={max_n_abs:.1f}m). "
                        f"n_start={n_start:.2f}, n_end={n_end:.2f}, t_end={t_end:.3f}s. "
                        "Clamping to track bounds."
                    )
                    # Clamp to reasonable bounds (±50m should cover any realistic track)
                    n = np.clip(n, -50.0, 50.0)
                    n_dot = np.clip(n_dot, -50.0, 50.0)
                    n_ddot = np.clip(n_ddot, -100.0, 100.0)

                n_array[i, :] = n
                n_dot_array[i, :] = n_dot
                n_ddot_array[i, :] = n_ddot
                i += 1

        return n_array, n_dot_array, n_ddot_array

    def calc_s_based_lat_samples(
            self,
            # Initial arc length position along the track centerline [m]
            s_start: float,
            # Initial longitudinal velocity (ds/dt) [m/s]
            s_dot_start: float,
            # Array of arc length trajectories for each sample [m] - shape: (n_samples, n_points)
            s_array: np.array,
            # Array of longitudinal velocity trajectories [m/s] - shape: (n_samples, n_points)
            s_dot_array: np.array,
            # Array of longitudinal acceleration trajectories [m/s²] - shape: (n_samples, n_points)
            s_ddot_array: np.array,
            # Target end velocities for each sample [m/s] - shape: (n_samples,)
            s_dot_end_values: np.array,
            # Target end arc length positions for each sample [m] - shape: (n_samples,)
            s_end_values: np.array,
            # Initial lateral offset from track centerline [m] (positive = left)
            n_start: float,
            # Initial lateral velocity (dn/dt) [m/s]
            n_dot_start: float,
            # Initial lateral acceleration (d²n/dt²) [m/s²]
            n_ddot_start: float,
            # Time arrays for each sample trajectory [s] - shape: (n_samples, n_points)
            t_array: np.ndarray,
            # Postprocessed raceline (from postprocess_raceline method)
            # Dictionary with '_post' suffixed keys: 's_post', 'n_post', 's_dot_post',
            # 's_ddot_post', 'n_dot_post', 'n_ddot_post', 'v_post', 'chi_post',
            # 'ax_post', 'ay_post', 't_post', 'kappa_post', 'x_post', 'y_post'
            postprocessed_raceline: dict,
            # If True, sample relative to raceline; if False, sample absolute trajectories
            raceline_tendency: bool,
            # Track
            # Track handler object providing track geometry and boundaries
            track_handler: Track,
            # Vehicle
            # Vehicle parameters dict containing 'total_width' and other vehicle dimensions
            vehicle_params: dict,
    ):

        self.declare_and_update_parameters()

        n_array = np.zeros_like(s_array)
        n_dot_array = np.zeros_like(s_array)
        n_ddot_array = np.zeros_like(s_array)

        # Extract data from postprocessed raceline (already computed by postprocess_raceline)
        s_coords = postprocessed_raceline['s_post']
        # Use v_post instead of vx_mps
        vx_coords = postprocessed_raceline['v_post']
        # Use n_post (lateral offset)
        d_coords = postprocessed_raceline['n_post']
        kappa_coords = postprocessed_raceline['kappa_post']

        # Pre-computed derivatives from postprocess_raceline
        s_dot_coords = postprocessed_raceline['s_dot_post']
        s_ddot_coords = postprocessed_raceline['s_ddot_post']
        n_dot_coords = postprocessed_raceline['n_dot_post']
        n_ddot_coords = postprocessed_raceline['n_ddot_post']
        chi_coords = postprocessed_raceline['chi_post']

        # get track length for periodic interpolation
        track_length = track_handler.get_track_length()

        # Extend raceline data to handle wraparound interpolation
        # This allows smooth interpolation when trajectories cross the track start/end boundary
        extended_raceline = self._extend_raceline_for_wraparound(
            postprocessed_raceline, track_length)

        # Use extended arrays for all subsequent interpolations
        s_coords_ext = extended_raceline['s_post']
        s_dot_coords_ext = extended_raceline['s_dot_post']
        s_ddot_coords_ext = extended_raceline['s_ddot_post']
        n_coords_ext = extended_raceline['n_post']
        n_dot_coords_ext = extended_raceline['n_dot_post']
        n_ddot_coords_ext = extended_raceline['n_ddot_post']
        chi_coords_ext = extended_raceline['chi_post']

        # define starting condition
        n_prime_start = n_dot_start / s_dot_start
        # n_dot_end = 0.0
        n_prime_end = 0.0  # n_dot_end / s_dot_end

        i = 0
        # iterate over s end valules
        for s_dot_end, s_end in zip(s_dot_end_values, s_end_values):

            # get the s values for the current s_end
            s_end = s_array[i][-1]
            s_loc_vector = (
                s_array[i] - s_start) % track_handler.get_track_length()

            # get start and end conditions for n curve
            # Clamp velocities to prevent division by zero/near-zero
            s_dot_start_safe = max(s_dot_array[i][0], 0.1)
            n_pprime_start = (n_ddot_start - n_prime_start *
                              s_ddot_array[i][0]) / (s_dot_start_safe ** 2)

            # construct matrix
            ones = np.ones_like(s_loc_vector)
            s_quad = s_loc_vector ** 2
            s_cubi = s_loc_vector * s_quad
            s_quar = s_loc_vector * s_cubi
            s_quin = s_loc_vector * s_quar
            s_mat = np.column_stack(
                (ones, s_loc_vector, s_quad, s_cubi, s_quar, s_quin))
            s_mat_dot = np.column_stack(
                (ones, 2*s_loc_vector, 3*s_quad, 4*s_cubi, 5*s_quar))
            s_mat_ddot = np.column_stack(
                (2*ones, 6*s_loc_vector, 12*s_quad, 20*s_cubi))

            # formulate matrix a of linear system of equations
            s_end_loc = s_loc_vector[-1]

            # Check for invalid or degenerate spatial horizon
            # Negative distance means the input trajectory data is corrupted or wraps incorrectly
            # Very small positive distance makes the polynomial problem ill-conditioned
            if s_end_loc < 0.0:
                rospy.logerr_throttle(
                    1.0,
                    f"LateralSampling (s-based): INVALID negative spatial horizon ({s_end_loc:.4f}m) detected! "
                    f"s_start={s_start:.2f}m, s_end={s_end:.2f}m, track_length={track_handler.get_track_length():.2f}m. "
                    "This indicates corrupted input data or wraparound issue. Skipping this trajectory."
                )
                # Skip this invalid trajectory
                continue
            elif s_end_loc < 0.1:  # Less than 10cm
                rospy.logwarn_throttle(
                    1.0,
                    f"LateralSampling (s-based): Very short spatial horizon ({s_end_loc:.4f}m) detected. "
                    "Trajectories may be numerically unstable."
                )

            a = np.array([[1, 0, 0, 0, 0, 0],
                          [0, 1, 0, 0, 0, 0],
                          [0, 0, 2, 0, 0, 0],
                          [1, s_end_loc, s_end_loc ** 2, s_end_loc **
                              3, s_end_loc ** 4, s_end_loc ** 5],
                          [0, 1, 2 * s_end_loc, 3 * s_end_loc ** 2,
                              4 * s_end_loc ** 3, 5 * s_end_loc ** 4],
                          [0, 0, 2, 6 * s_end_loc, 12 * s_end_loc ** 2, 20 * s_end_loc ** 3]])

            # calculate the inverse of a for solving the linear equation system in for loop just with a matrix multiplication
            a_inverse = np.linalg.inv(a)

            # Interpolate raceline data at specific s points using extended raceline
            # Extended arrays handle wraparound smoothly without discontinuities
            # All derivatives and transformations already computed by postprocess_raceline
            s_dot_rl = np.interp(
                s_array[i],
                s_coords_ext,
                s_dot_coords_ext,
            )
            s_ddot_rl = np.interp(
                s_array[i],
                s_coords_ext,
                s_ddot_coords_ext,
            )
            n_rl = np.interp(
                s_array[i],
                s_coords_ext,
                n_coords_ext,
            )
            n_dot_rl = np.interp(
                s_array[i],
                s_coords_ext,
                n_dot_coords_ext,
            )
            n_ddot_rl = np.interp(
                s_array[i],
                s_coords_ext,
                n_ddot_coords_ext,
            )
            chi_rl = np.interp(
                s_array[i],
                s_coords_ext,
                chi_coords_ext,
            )

            # Calculate n_prime and n_pprime for quintic polynomial generation
            # n_prime = dn/ds, n_pprime = d²n/ds²
            # Clamp velocities to prevent division by zero/near-zero
            s_dot_rl_safe = np.maximum(s_dot_rl, 0.1)
            s_dot_array_safe = np.maximum(s_dot_array[i], 0.1)

            n_prime_rl = n_dot_rl / s_dot_rl_safe
            n_pprime_rl = (n_ddot_rl - n_prime_rl *
                           s_ddot_rl) / (s_dot_rl_safe**2)

            n_rl_eval = n_rl
            n_dot_rl_eval = n_dot_rl / s_dot_rl_safe * s_dot_array_safe
            n_ddot_rl_eval = (
                n_ddot_rl / (s_dot_rl_safe**2) * (s_dot_array_safe ** 2)
                - n_dot_rl / (s_dot_rl_safe**3) * s_ddot_rl *
                (s_dot_array_safe ** 2)
                + n_dot_rl / s_dot_rl_safe * s_ddot_array[i]
            )

            n_dense_end_values = np.linspace(
                n_rl_eval[-1] + self.params.n_dense_min, n_rl_eval[-1] + self.params.n_dense_max, self.params.n_dense_samples)

            # sampled n end conditions (relative to raceline)
            # In Frenet coordinates: positive n = left, negative n = right
            # Right boundary (negative n): -trackwidth_right + margins
            n_min_track = -(track_handler.trackwidth_right(
                s_end) - vehicle_params['total_width'] / 2.0 - self.params.tube_width - self.params.safety_distance_track_right)
            # Left boundary (positive n): +trackwidth_left - margins
            n_max_track = track_handler.trackwidth_left(
                s_end) - vehicle_params['total_width'] / 2.0 - self.params.tube_width - self.params.safety_distance_track_left
            n_end_min = n_min_track
            n_end_max = n_max_track
            n_end_values = np.concatenate((np.linspace(n_end_min, n_end_max, self.params.n_samples - 1), [
                                          n_rl_eval[-1]], n_dense_end_values))  # always sample straight driving and raceline

            # conditions for racing line
            # Clamp velocities to prevent division by zero
            s_dot_rl_start_safe = max(s_dot_rl[0], 0.1)
            s_dot_rl_end_safe = max(s_dot_rl[-1], 0.1)

            n_prime_start_rl = n_dot_rl_eval[0] / s_dot_rl_start_safe
            n_prime_end_rl = n_dot_rl_eval[-1] / s_dot_rl_end_safe
            n_pprime_start_rl = (
                n_ddot_rl_eval[0] - n_prime_start_rl * s_ddot_rl[0]) / (s_dot_rl_start_safe ** 2)

            # Calculate geometric lateral velocities if using position-dependent mode
            if self.params.use_geometric_lateral_end_velocity:
                # Calculate track boundary headings and velocities
                xy_left = track_handler.sn2cartesian(
                    s_end, track_handler.trackwidth_left(s_end))
                xy_left_prev = track_handler.sn2cartesian(
                    s_end - 1.0, track_handler.trackwidth_left(s_end - 1.0))
                psi_left = np.arctan2(
                    (xy_left[1] - xy_left_prev[1]), (xy_left[0] - xy_left_prev[0]))

                xy_right = track_handler.sn2cartesian(
                    s_end, -track_handler.trackwidth_right(s_end))
                xy_right_prev = track_handler.sn2cartesian(
                    s_end - 1.0, -track_handler.trackwidth_right(s_end - 1.0))
                psi_right = np.arctan2(
                    (xy_right[1] - xy_right_prev[1]), (xy_right[0] - xy_right_prev[0]))

                chi_left = track_handler.calc_chi_from_2d_heading(
                    s_end, psi_left)
                chi_right = track_handler.calc_chi_from_2d_heading(
                    s_end, psi_right)

                V_end_left = (
                    1.0 - n_max_track * track_handler.omega_z(s_end)) * s_dot_end / np.cos(chi_left)
                V_end_right = (
                    1.0 - n_min_track * track_handler.omega_z(s_end)) * s_dot_end / np.cos(chi_right)

                n_dot_end_left = V_end_left * np.sin(chi_left)
                n_dot_end_right = V_end_right * np.sin(chi_right)

            for n_end in n_end_values:
                # Calculate lateral end velocity based on mode
                if self.params.use_geometric_lateral_end_velocity:
                    # Position-dependent: interpolate between boundary velocities
                    n_dot_end = np.interp(n_end, [n_min_track, 0.0, n_max_track],
                                          [n_dot_end_right, 0.0, n_dot_end_left])
                else:
                    # Simple: zero lateral velocity (perpendicular arrival)
                    n_dot_end = 0.0

                n_prime_end = n_dot_end / s_dot_end
                # n_pprime_end = (n_ddot_end - n_prime_end * s_ddot_array[i]) / (s_dot_array[i] ** 2)

                if raceline_tendency:  # sample curves relative to raceline
                    b = np.array(
                        [
                            n_start - n_rl_eval[0],
                            n_prime_start - n_prime_start_rl,
                            n_pprime_start - n_pprime_start_rl,
                            n_end - n_rl_eval[-1],
                            n_prime_end - n_prime_end_rl,
                            0.0,
                        ]
                    )
                else:  # sample curves absolute
                    b = np.array(
                        [n_start, n_prime_start, n_pprime_start, n_end, n_prime_end, 0.0])

                # calculate coefficients of quintic polynomial(now as matrix multiplication)
                c = a_inverse @ b

                # calculate sampled n curve (now as matrix multiplication)
                n_sample = s_mat @ c
                n_prime_sample = s_mat_dot @ c[1:]
                n_pprime_sample = s_mat_ddot @ c[2:]

                if raceline_tendency:
                    n_prime_rl_eval = n_dot_rl_eval / s_dot_rl_safe
                    n_pprime_rl_eval = (
                        n_ddot_rl_eval - n_prime_rl_eval * s_ddot_rl) / (s_dot_rl_safe ** 2)
                    # add raceline n data to sampled relative n curve
                    n = n_sample + n_rl_eval
                    n_prime = n_prime_sample + n_prime_rl_eval
                    n_pprime = n_pprime_sample + n_pprime_rl_eval
                else:
                    n = n_sample
                    n_prime = n_prime_sample
                    n_pprime = n_pprime_sample

                # transform back to n_dot and n_ddot
                n_dot = n_prime * s_dot_array_safe
                n_ddot = n_pprime * \
                    (s_dot_array_safe ** 2) + n_prime * s_ddot_array[i]

                # Sanity check: detect and reject extreme outlier trajectories
                max_n_abs = np.max(np.abs(n))
                if max_n_abs > 100.0:
                    rospy.logwarn_throttle(
                        5.0,
                        f"LateralSampling (s-based): Extreme lateral trajectory detected (max |n|={max_n_abs:.1f}m). "
                        f"n_start={n_start:.2f}, n_end={n_end:.2f}, s_end_loc={s_end_loc:.3f}m. "
                        "Clamping to track bounds."
                    )
                    # Clamp to reasonable bounds
                    n = np.clip(n, -50.0, 50.0)
                    n_dot = np.clip(n_dot, -50.0, 50.0)
                    n_ddot = np.clip(n_ddot, -100.0, 100.0)

                n_array[i, :] = n
                n_dot_array[i, :] = n_dot
                n_ddot_array[i, :] = n_ddot
                i += 1

        return n_array, n_dot_array, n_ddot_array
