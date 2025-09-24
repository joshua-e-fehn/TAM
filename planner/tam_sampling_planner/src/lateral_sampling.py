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


class LateralSampling:
    def __init__(self):
        """
        Initialize the LateralSampling class.

        No input parameters required - all configuration comes from ROS parameter server.
        """
        self.params = LatSamplingParams()
        self.declare_and_update_parameters()

    def declare_and_update_parameters(self):
        """
        Update parameters from ROS parameter server.

        No input parameters - reads from ROS parameter server using rospy.get_param().
        Updates self.params with latest parameter values.
        """
        self.params.n_samples = rospy.get_param(
            "discretization/n_samples", 20)
        self.params.n_dense_min = rospy.get_param(
            "discretization/n_dense_min", -0.5)
        self.params.n_dense_max = rospy.get_param(
            "discretization/n_dense_max", 0.5)
        self.params.n_dense_samples = rospy.get_param(
            "discretization/n_dense_samples", 5)
        self.params.safety_distance_track_left = rospy.get_param(
            "safety_distances/safety_distance_track_left", 0.5)
        self.params.safety_distance_track_right = rospy.get_param(
            "safety_distances/safety_distance_track_right", 0.5)
        self.params.tube_width = rospy.get_param("behavior/tube_width", 1.0)

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
            # Global waypoints (serves as raceline)
            # Dictionary with 'wpnts' key containing list of waypoint dicts.
            global_waypoints: dict,
            # Each waypoint dict must have: 's_m' (arc length), 'vx_mps' (velocity),
            # 'd_m' (lateral offset), 'kappa_radpm' (curvature), plus optional fields
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

        # Create track handler from global waypoints or use existing one
        if not isinstance(track_handler, Track):
            # Create new track handler from global waypoints
            track_handler = Track(global_waypoints)
        elif not track_handler.is_initialized():
            # Update existing track handler with new waypoints
            track_handler.update_waypoints(global_waypoints)

        # extract data from global waypoints once
        waypoints = global_waypoints["wpnts"]
        s_coords = np.array([wp["s_m"] for wp in waypoints])
        vx_coords = np.array([wp["vx_mps"] for wp in waypoints])
        # lateral offset (n coordinate)
        d_coords = np.array([wp["d_m"] for wp in waypoints])
        kappa_coords = np.array([wp["kappa_radpm"] for wp in waypoints])

        # compute improved derivatives for better approximations
        derivatives = self._compute_improved_derivatives(
            waypoints, s_coords, vx_coords, d_coords, kappa_coords)

        # get track length for periodic interpolation
        track_length = track_handler.get_track_length()

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

            # evaluate raceline at specific s points using global waypoints
            s_dot_rl = np.interp(
                s_array[i],
                s_coords,
                vx_coords,
                period=track_length,
            )
            # improved s_ddot approximation using pre-computed gradient
            vx_grad = derivatives['vx_grad']
            s_ddot_rl = np.interp(
                s_array[i],
                s_coords,
                vx_grad * vx_coords,  # dv/dt = dv/ds * ds/dt = dv/ds * v
                period=track_length,
            )
            n_rl = np.interp(
                s_array[i],
                s_coords,
                d_coords,
                period=track_length,
            )
            # improved lateral derivatives from d_m (lateral offset) data
            n_prime_rl = np.interp(
                s_array[i], s_coords, derivatives['d_grad'], period=track_length)
            n_pprime_rl = np.interp(
                s_array[i], s_coords, derivatives['d_grad2'], period=track_length)

            # transform to time derivatives: n_dot = n_prime * s_dot, n_ddot = n_pprime * s_dot² + n_prime * s_ddot
            n_dot_rl = n_prime_rl * s_dot_rl
            n_ddot_rl = n_pprime_rl * (s_dot_rl**2) + n_prime_rl * s_ddot_rl

            # improved chi calculation
            if derivatives['use_geometric_chi']:
                chi_rl = np.interp(s_array[i], s_coords,
                                   derivatives['heading_angles'] +
                                   kappa_coords * d_coords,
                                   period=track_length)
            else:
                # fallback to curvature-based approximation
                chi_rl = np.interp(
                    s_array[i],
                    s_coords,
                    # fallback chi from curvature
                    np.arctan(kappa_coords * d_coords),
                    period=track_length,
                )

            n_rl_eval = n_rl
            n_dot_rl_eval = n_dot_rl / s_dot_rl * s_dot_array[i]
            n_ddot_rl_eval = (
                n_ddot_rl / (s_dot_rl**2) * (s_dot_array[i] ** 2)
                - n_dot_rl / (s_dot_rl**3) * s_ddot_rl * (s_dot_array[i] ** 2)
                + n_dot_rl / s_dot_rl * s_ddot_array[i]
            )

            n_dense_end_values = np.linspace(
                n_rl_eval[-1] + self.params.n_dense_min, n_rl_eval[-1] + self.params.n_dense_max, self.params.n_dense_samples)

            # sampled n end conditions (relative to raceline)
            # no safety distances for sampling (soft constraints)
            n_min_track = track_handler.trackwidth_right(
                s_end) + vehicle_params['total_width'] / 2.0 + self.params.tube_width + self.params.safety_distance_track_right
            # no safety distances for sampling (soft constraints)
            n_max_track = track_handler.trackwidth_left(
                s_end) - vehicle_params['total_width'] / 2.0 - self.params.tube_width - self.params.safety_distance_track_left
            n_end_min = n_min_track
            n_end_max = n_max_track
            n_end_values = np.concatenate((np.linspace(n_end_min, n_end_max, self.params.n_samples - 1), [
                                          n_rl_eval[-1]], n_dense_end_values))  # always sample straight driving and raceline

            n_dot_end = s_dot_array[i][-1] * np.sin(chi_rl[-1])
            # n_prime_end = n_dot_end / s_dot_end
            for n_end in n_end_values:
                # n_dot_end = 0.0

                # assume the same chi for left and right trackbound?
                """xy_left = track_handler.sn2cartesian(s_end, track_handler.trackwidth_left(s_end))
                xy_left_prev = track_handler.sn2cartesian(s_end - 1.0, track_handler.trackwidth_left(s_end - 1.0))
                psi_left = np.arctan2((xy_left[1] - xy_left_prev[1]), (xy_left[0] - xy_left_prev[0]))

                xy_right = track_handler.sn2cartesian(s_end, track_handler.trackwidth_right(s_end))
                xy_right_prev = track_handler.sn2cartesian(s_end - 1.0, track_handler.trackwidth_right(s_end - 1.0))
                psi_right = np.arctan2((xy_right[1] - xy_right_prev[1]), (xy_right[0] - xy_right_prev[0]))

                chi_left = track_handler.calc_chi_from_2d_heading(s_end, psi_left)
                chi_right = track_handler.calc_chi_from_2d_heading(s_end, psi_right)

                V_end_left = (1.0 - n_max_track * track_handler.omega_z(s_end)) * s_dot_end / np.cos(chi_left)
                V_end_right = (1.0 - n_min_track * track_handler.omega_z(s_end)) * s_dot_end / np.cos(chi_right)

                n_dot_end_left = V_end_left * np.sin(chi_left)
                n_dot_end_right = V_end_right * np.sin(chi_right)

                n_dot_end = np.interp(n_end, [n_min_track, 0.0, n_max_track], [n_dot_end_right, 0.0, n_dot_end_left])"""

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
            # Global waypoints (serves as raceline)
            # Dictionary with 'wpnts' key containing list of waypoint dicts.
            global_waypoints: dict,
        # Each waypoint dict must have: 's_m' (arc length), 'vx_mps' (velocity),
        # 'd_m' (lateral offset), 'kappa_radpm' (curvature), plus optional fields
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

        # Create track handler from global waypoints or use existing one
        if not isinstance(track_handler, Track):
            # Create new track handler from global waypoints
            track_handler = Track(global_waypoints)
        elif not track_handler.is_initialized():
            # Update existing track handler with new waypoints
            track_handler.update_waypoints(global_waypoints)

        # extract data from global waypoints once
        waypoints = global_waypoints["wpnts"]
        s_coords = np.array([wp["s_m"] for wp in waypoints])
        vx_coords = np.array([wp["vx_mps"] for wp in waypoints])
        # lateral offset (n coordinate)
        d_coords = np.array([wp["d_m"] for wp in waypoints])
        kappa_coords = np.array([wp["kappa_radpm"] for wp in waypoints])

        # compute improved derivatives for better approximations
        derivatives = self._compute_improved_derivatives(
            waypoints, s_coords, vx_coords, d_coords, kappa_coords)

        # get track length for periodic interpolation
        track_length = track_handler.get_track_length()

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
            n_pprime_start = (n_ddot_start - n_prime_start *
                              s_ddot_array[i][0]) / (s_dot_array[i][0] ** 2)

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

            # evaluate raceline at specific s points using global waypoints
            s_dot_rl = np.interp(
                s_array[i],
                s_coords,
                vx_coords,
                period=track_length,
            )
            # improved s_ddot approximation using pre-computed gradient
            s_ddot_rl = np.interp(
                s_array[i],
                s_coords,
                # dv/dt = dv/ds * ds/dt = dv/ds * v
                derivatives['vx_grad'] * vx_coords,
                period=track_length,
            )
            n_rl = np.interp(
                s_array[i],
                s_coords,
                d_coords,
                period=track_length,
            )
            # improved lateral derivatives from d_m (lateral offset) data
            n_prime_rl = np.interp(
                s_array[i], s_coords, derivatives['d_grad'], period=track_length)
            n_pprime_rl = np.interp(
                s_array[i], s_coords, derivatives['d_grad2'], period=track_length)

            # transform to time derivatives: n_dot = n_prime * s_dot, n_ddot = n_pprime * s_dot² + n_prime * s_ddot
            n_dot_rl = n_prime_rl * s_dot_rl
            n_ddot_rl = n_pprime_rl * (s_dot_rl**2) + n_prime_rl * s_ddot_rl

            # improved chi calculation
            if derivatives['use_geometric_chi']:
                chi_rl = np.interp(s_array[i], s_coords,
                                   derivatives['heading_angles'] +
                                   kappa_coords * d_coords,
                                   period=track_length)
            else:
                # fallback to curvature-based approximation
                chi_rl = np.interp(
                    s_array[i],
                    s_coords,
                    # fallback chi from curvature
                    np.arctan(kappa_coords * d_coords),
                    period=track_length,
                )

            n_rl_eval = n_rl
            n_dot_rl_eval = n_dot_rl / s_dot_rl * s_dot_array[i]
            n_ddot_rl_eval = (
                n_ddot_rl / (s_dot_rl**2) * (s_dot_array[i] ** 2)
                - n_dot_rl / (s_dot_rl**3) * s_ddot_rl * (s_dot_array[i] ** 2)
                + n_dot_rl / s_dot_rl * s_ddot_array[i]
            )

            n_dense_end_values = np.linspace(
                n_rl_eval[-1] + self.params.n_dense_min, n_rl_eval[-1] + self.params.n_dense_max, self.params.n_dense_samples)

            # sampled n end conditions (relative to raceline)
            # no safety distances for sampling (soft constraints)
            n_min_track = track_handler.trackwidth_right(
                s_end) + vehicle_params['total_width'] / 2.0 + self.params.tube_width + self.params.safety_distance_track_right
            # no safety distances for sampling (soft constraints)
            n_max_track = track_handler.trackwidth_left(
                s_end) - vehicle_params['total_width'] / 2.0 - self.params.tube_width - self.params.safety_distance_track_left
            n_end_min = n_min_track
            n_end_max = n_max_track
            n_end_values = np.concatenate((np.linspace(n_end_min, n_end_max, self.params.n_samples - 1), [
                                          n_rl_eval[-1]], n_dense_end_values))  # always sample straight driving and raceline

            # conditions for racing line
            n_prime_start_rl = n_dot_rl_eval[0] / s_dot_rl[0]
            n_prime_end_rl = n_dot_rl_eval[-1] / s_dot_rl[-1]
            n_pprime_start_rl = (
                n_ddot_rl_eval[0] - n_prime_start_rl * s_ddot_rl[0]) / (s_dot_rl[0] ** 2)

            # transform n_dot and n_ddot to n_prime and n_pprime
            n_dot_end = s_dot_array[i][-1] * np.sin(chi_rl[-1])
            n_prime_end = n_dot_end / s_dot_end

            for n_end in n_end_values:

                # assume the same chi for left and right trackbound?
                """xy_left = track_handler.sn2cartesian(s_end, track_handler.trackwidth_left(s_end))
                xy_left_prev = track_handler.sn2cartesian(s_end - 1.0, track_handler.trackwidth_left(s_end - 1.0))
                psi_left = np.arctan2((xy_left[1] - xy_left_prev[1]), (xy_left[0] - xy_left_prev[0]))

                xy_right = track_handler.sn2cartesian(s_end, track_handler.trackwidth_right(s_end))
                xy_right_prev = track_handler.sn2cartesian(s_end - 1.0, track_handler.trackwidth_right(s_end - 1.0))
                psi_right = np.arctan2((xy_right[1] - xy_right_prev[1]), (xy_right[0] - xy_right_prev[0]))

                chi_left = track_handler.calc_chi_from_2d_heading(s_end, psi_left)
                chi_right = track_handler.calc_chi_from_2d_heading(s_end, psi_right)

                V_end_left = (1.0 - n_max_track * track_handler.omega_z(s_end)) * s_dot_end / np.cos(chi_left)
                V_end_right = (1.0 - n_min_track * track_handler.omega_z(s_end)) * s_dot_end / np.cos(chi_right)

                n_dot_end_left = V_end_left * np.sin(chi_left)
                n_dot_end_right = V_end_right * np.sin(chi_right)

                n_dot_end = np.interp(n_end, [n_min_track, 0.0, n_max_track], [n_dot_end_right, 0.0, n_dot_end_left])"""

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
                    n_prime_rl_eval = n_dot_rl_eval / s_dot_rl
                    n_pprime_rl_eval = (
                        n_ddot_rl_eval - n_prime_rl_eval * s_ddot_rl) / (s_dot_rl ** 2)
                    # add raceline n data to sampled relative n curve
                    n = n_sample + n_rl_eval
                    n_prime = n_prime_sample + n_prime_rl_eval
                    n_pprime = n_pprime_sample + n_pprime_rl_eval
                else:
                    n = n_sample
                    n_prime = n_prime_sample
                    n_pprime = n_pprime_sample

                # transform back to n_dot and n_ddot
                n_dot = n_prime * s_dot_array[i]
                n_ddot = n_pprime * \
                    (s_dot_array[i] ** 2) + n_prime * s_ddot_array[i]

                n_array[i, :] = n
                n_dot_array[i, :] = n_dot
                n_ddot_array[i, :] = n_ddot
                i += 1

        return n_array, n_dot_array, n_ddot_array
