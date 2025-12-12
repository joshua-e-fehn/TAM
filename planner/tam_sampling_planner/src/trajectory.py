"""Trajectory generation and emergency braking module for TAM Sampling Planner.

This module provides trajectory extension and emergency braking trajectory
generation using physics-based tire models. It calculates braking limits
using either Pacejka tire model (preferred) or simplified friction circle
fallback.

Key Features:
    - Emergency braking trajectory generation with optimal deceleration
    - Performance trajectory extension to meet minimum length requirements
    - Pacejka tire model integration for accurate force limits
    - Track-aware calculations (banking, elevation effects)
"""

from track_handler_global_waypoints import GlobalWaypointsTrackHandler as Track
from pacejka_tire_model import PacejkaTireModel
import numpy as np
from dataclasses import dataclass
import copy
import rospy


@dataclass
class TrajectoryParams():
    """Configuration parameters for trajectory generation.

    Attributes:
        tube_width: Safety margin width around trajectory [m]. Default 0.6m
            (reduced from 1.0m for tighter racing, car width ~0.5m).
        num_samples: Number of discrete samples in trajectory. Default 51.
        min_trajectory_length: Minimum required trajectory length [m]. Default 10.0m.
        extension_emergency_time_offset: Time offset for emergency extension start [s].
        extension_n_samples: Number of extension length samples to try.
        extension_point_distance: Distance between extension points [m].
        extension_min_resolution: Minimum number of extension samples.
        extension_max_s_sample: Maximum arc-length for extension sampling [m].
        additional_const_time_emergency: Additional constant time for emergency [s].
        const_trajectory_time: Constant trajectory time period [s].
        add_emergency_safety_distance_left: Extra left safety margin for emergency [m].
        add_emergency_safety_distance_right: Extra right safety margin for emergency [m].
        max_braking_deceleration_g: Fallback max braking in g's (when GGGV unavailable).
        max_lateral_acceleration_g: Fallback max lateral acceleration in g's.
    """
    # Reduced from 1.0 to 0.6m for tighter racing (car width ~0.5m)
    tube_width: float = 0.6
    num_samples: int = 51
    min_trajectory_length: float = 10.0
    extension_emergency_time_offset: float = 1.0
    extension_n_samples: int = 10
    extension_point_distance: float = 2.0
    extension_min_resolution: int = 10
    extension_max_s_sample: int = 300
    additional_const_time_emergency: float = 0.5
    const_trajectory_time: float = 0.3
    add_emergency_safety_distance_left: float = 0.0
    add_emergency_safety_distance_right: float = 0.0
    # Fallback tire limits (used when GGGV data is unavailable)
    max_braking_deceleration_g: float = 1.0  # Maximum braking in multiples of g
    # Maximum lateral acceleration in multiples of g
    max_lateral_acceleration_g: float = 1.2


class Trajectory():
    """Trajectory generation and extension handler for TAM planner.

    Provides emergency braking trajectory calculation and performance trajectory
    extension. Uses Pacejka tire model for accurate tire force limits when available,
    with simplified physics fallback.

    Attributes:
        params (TrajectoryParams): Configuration parameters.
        initialized_params (bool): Whether parameters have been initialized from YAML.
        debugging (bool): Enable debug output.
        pacejka_model (PacejkaTireModel or None): Tire model for force calculations.
        use_pacejka (bool): Whether Pacejka model is available and active.
    """

    def __init__(self, debugging):
        """Initialize trajectory handler.

        Args:
            debugging: Enable debug logging and output.
        """
        self.params = TrajectoryParams()
        self.initialized_params = False
        self.declare_and_update_parameters()
        self.debugging = debugging

        # Initialize Pacejka tire model for accurate force limits
        try:
            self.pacejka_model = PacejkaTireModel()
            self.use_pacejka = True
            rospy.loginfo(
                "Trajectory: Using Pacejka tire model for force limits")
        except Exception as e:
            rospy.logwarn(
                f"Trajectory: Failed to initialize Pacejka model ({e}), using fallback")
            self.pacejka_model = None
            self.use_pacejka = False

    def _load_yaml_defaults(self):
        """Load default parameters from tam_sampling_params.yaml.

        Returns:
            dict: Parameter dictionary from YAML file, or empty dict on failure.
                Keys match ROS parameter names (e.g., 'tube_width', 'num_samples').
        """
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
                f"Trajectory: Could not load YAML defaults: {e}")
            return {}

    def declare_and_update_parameters(self, skip_update=False):
        """Load parameters from ROS parameter server with YAML defaults as fallback."""
        # Check if parameter updates should be skipped (e.g., during race)
        if skip_update:
            return

        if not self.initialized_params:
            yaml_defaults = self._load_yaml_defaults()

            self.params.tube_width = yaml_defaults.get(
                'tube_width', rospy.get_param('behavior/tube_width', 0.6))
            rospy.set_param('behavior/tube_width', self.params.tube_width)
            self.params.num_samples = yaml_defaults.get(
                'num_samples', rospy.get_param('discretization/num_samples', 51))
            rospy.set_param('discretization/num_samples',
                            self.params.num_samples)
            self.params.min_trajectory_length = yaml_defaults.get(
                'min_trajectory_length', rospy.get_param('behavior/min_trajectory_length', 8.0))
            rospy.set_param('behavior/min_trajectory_length',
                            self.params.min_trajectory_length)
            self.params.extension_emergency_time_offset = yaml_defaults.get(
                'extension_emergency_time_offset', rospy.get_param('behavior/extension_emergency_time_offset', 1.0))
            rospy.set_param('behavior/extension_emergency_time_offset',
                            self.params.extension_emergency_time_offset)
            self.params.extension_n_samples = yaml_defaults.get(
                'extension_n_samples', rospy.get_param('behavior/extension_n_samples', 10))
            rospy.set_param('behavior/extension_n_samples',
                            self.params.extension_n_samples)
            self.params.extension_point_distance = yaml_defaults.get(
                'extension_point_distance', rospy.get_param('behavior/extension_point_distance', 2.0))
            rospy.set_param('behavior/extension_point_distance',
                            self.params.extension_point_distance)
            self.params.extension_min_resolution = yaml_defaults.get(
                'extension_min_resolution', rospy.get_param('behavior/extension_min_resolution', 10))
            rospy.set_param('behavior/extension_min_resolution',
                            self.params.extension_min_resolution)
            self.params.extension_max_s_sample = yaml_defaults.get(
                'extension_max_s_sample', rospy.get_param('behavior/extension_max_s_sample', 300))
            rospy.set_param('behavior/extension_max_s_sample',
                            self.params.extension_max_s_sample)
            self.params.add_emergency_safety_distance_left = yaml_defaults.get(
                'add_emergency_safety_distance_left', rospy.get_param('safety_distances/add_emergency_safety_distance_left', 0.3))
            rospy.set_param('safety_distances/add_emergency_safety_distance_left',
                            self.params.add_emergency_safety_distance_left)
            self.params.add_emergency_safety_distance_right = yaml_defaults.get(
                'add_emergency_safety_distance_right', rospy.get_param('safety_distances/add_emergency_safety_distance_right', 0.3))
            rospy.set_param('safety_distances/add_emergency_safety_distance_right',
                            self.params.add_emergency_safety_distance_right)
            self.params.additional_const_time_emergency = yaml_defaults.get(
                'additional_const_time_emergency', rospy.get_param('behavior/additional_const_time_emergency', 0.5))
            rospy.set_param('behavior/additional_const_time_emergency',
                            self.params.additional_const_time_emergency)
            self.params.const_trajectory_time = yaml_defaults.get(
                'const_trajectory_time', rospy.get_param('behavior/const_trajectory_time', 0.0))
            rospy.set_param('behavior/const_trajectory_time',
                            self.params.const_trajectory_time)

            # Fallback tire limits for emergency braking (when GGGV data unavailable)
            # These should be tuned based on your Pacejka tire model parameters
            # Conservative defaults: racing tires typically 1.2-1.5g braking, 1.5-2.0g lateral
            self.params.max_braking_deceleration_g = yaml_defaults.get(
                'max_braking_deceleration_g', rospy.get_param('behavior/max_braking_deceleration_g', 1.0))
            rospy.set_param('behavior/max_braking_deceleration_g',
                            self.params.max_braking_deceleration_g)
            self.params.max_lateral_acceleration_g = yaml_defaults.get(
                'max_lateral_acceleration_g', rospy.get_param('behavior/max_lateral_acceleration_g', 1.2))
            rospy.set_param('behavior/max_lateral_acceleration_g',
                            self.params.max_lateral_acceleration_g)

            self.initialized_params = True
        else:
            self.params.tube_width = rospy.get_param(
                'behavior/tube_width', self.params.tube_width)
            self.params.num_samples = rospy.get_param(
                'discretization/num_samples', self.params.num_samples)
            self.params.min_trajectory_length = rospy.get_param(
                'behavior/min_trajectory_length', self.params.min_trajectory_length)
            self.params.extension_emergency_time_offset = rospy.get_param(
                'behavior/extension_emergency_time_offset', self.params.extension_emergency_time_offset)
            self.params.extension_n_samples = rospy.get_param(
                'behavior/extension_n_samples', self.params.extension_n_samples)
            self.params.extension_point_distance = rospy.get_param(
                'behavior/extension_point_distance', self.params.extension_point_distance)
            self.params.extension_min_resolution = rospy.get_param(
                'behavior/extension_min_resolution', self.params.extension_min_resolution)
            self.params.extension_max_s_sample = rospy.get_param(
                'behavior/extension_max_s_sample', self.params.extension_max_s_sample)
            self.params.add_emergency_safety_distance_left = rospy.get_param(
                'safety_distances/add_emergency_safety_distance_left', self.params.add_emergency_safety_distance_left)
            self.params.add_emergency_safety_distance_right = rospy.get_param(
                'safety_distances/add_emergency_safety_distance_right', self.params.add_emergency_safety_distance_right)
            self.params.additional_const_time_emergency = rospy.get_param(
                'behavior/additional_const_time_emergency', self.params.additional_const_time_emergency)
            self.params.const_trajectory_time = rospy.get_param(
                'behavior/const_trajectory_time', self.params.const_trajectory_time)

            # Fallback tire limits for emergency braking (when GGGV data unavailable)
            # These should be tuned based on your Pacejka tire model parameters
            # Conservative defaults: racing tires typically 1.2-1.5g braking, 1.5-2.0g lateral
            self.params.max_braking_deceleration_g = rospy.get_param(
                'behavior/max_braking_deceleration_g', self.params.max_braking_deceleration_g)
            self.params.max_lateral_acceleration_g = rospy.get_param(
                'behavior/max_lateral_acceleration_g', self.params.max_lateral_acceleration_g)

    def __calc_ax_avail(self, s, n, chi, V, Omega_z, track_handler, gggv_handler, pitlane_mode):
        """Calculate available braking acceleration considering current lateral load.

        Uses friction ellipse to determine available longitudinal (braking) acceleration
        given current lateral acceleration from cornering. Prefers Pacejka tire model
        for accurate calculations, falls back to simplified friction circle.

        NOTE: gggv_handler parameter is kept for API compatibility but not used.
        GGGV diagram functionality is commented out as Pacejka tire model is used instead.

        Args:
            s: Arc-length position on track [m].
            n: Lateral offset from centerline [m].
            chi: Heading angle relative to track tangent [rad].
            V: Vehicle velocity [m/s].
            Omega_z: Yaw rate / curvature [rad/m]. Note: Named Omega_z but represents
                path curvature (kappa), not vehicle yaw rate.
            track_handler: Track geometry handler for coordinate transforms.
            gggv_handler: GGGV handler (NOT USED - kept for compatibility).
            pitlane_mode: Whether in pitlane mode (NOT USED with current implementation).

        Returns:
            tuple: (ax_avail_tilde, ay_tilde, g_tilde)
                - ax_avail_tilde: Available braking acceleration in apparent frame [m/s²]
                  (negative value indicating deceleration)
                - ay_tilde: Lateral acceleration in apparent frame [m/s²]
                - g_tilde: Apparent gravity accounting for track geometry [m/s²]
        """
        ay_hat = V**2 * Omega_z
        _, ay_tilde, g_tilde = track_handler.calc_apparent_acceleration(
            s,
            n,
            chi,
            0.0,  # ax_hat not required for ay_tilde and g_tilde
            ay_hat,
            V,
        )

        # ============================================================================
        # GGGV DIAGRAM USAGE - COMMENTED OUT (No GGGV data available)
        # ============================================================================
        # Original GGGV-based code:
        # _, ax_min_tilde, _, ay_max_tilde, ym_max = gggv_handler.acc_interpolator(
        #     np.array(V), np.array(g_tilde), np.array(
        #         s), np.array(n), not pitlane_mode, self.debugging
        # )
        # ax_avail_tilde = -np.abs(ax_min_tilde) * np.power(
        #     max((1.0 - np.power(min(np.abs(ay_tilde) / (ay_max_tilde), 1.0),
        #         gggv_handler.gg_exponent_ax_neg)), 1e-4), 1.0 / gggv_handler.gg_exponent_ax_neg
        # )
        # ============================================================================

        # ============================================================================
        # TIRE LIMIT CALCULATION - Two implementations available:
        # 1. Pacejka tire model (accurate, physics-based)
        # 2. Simplified fallback (configurable parameters)
        # ============================================================================

        if self.use_pacejka and self.pacejka_model is not None:
            # PACEJKA MODEL: Accurate tire force calculation
            # Considers:
            # - Load-dependent tire forces (normal load from weight transfer)
            # - Combined slip (friction ellipse with proper exponent)
            # - Front/rear axle force distribution
            # - Magic Formula coefficients from vehicle parameters

            # Calculate available braking considering current lateral acceleration
            # The Pacejka model internally handles:
            # - Fx_max(Fz) from longitudinal magic formula
            # - Fy_max(Fz) from lateral magic formula
            # - Combined slip: sqrt((Fx/Fx_max)^n + (Fy/Fy_max)^n) <= 1
            ax_avail_tilde = self.pacejka_model.calc_max_braking_acceleration(
                ay_current=ay_tilde,
                ax_current=0.0  # Conservative: assume no current longitudinal acceleration
            )

            # Scale by local gravity (track banking/elevation effects)
            # g_tilde accounts for apparent gravity due to track geometry
            ax_avail_tilde = ax_avail_tilde * (g_tilde / 9.81)

        else:
            # FALLBACK: Simplified physics-based braking limits
            # Uses configurable parameters from ROS parameter server
            # This is used if Pacejka model fails to initialize

            rospy.logwarn_throttle(
                10.0, "Using simplified tire model fallback")

            # Maximum braking acceleration (from ROS parameters)
            ax_max_available = -self.params.max_braking_deceleration_g * g_tilde

            # Maximum lateral acceleration (from ROS parameters)
            ay_max_available = self.params.max_lateral_acceleration_g * g_tilde

            # Friction circle: reduce available braking when cornering
            # sqrt(ax^2 + ay^2) <= mu*g
            lateral_usage = min(abs(ay_tilde) / ay_max_available, 1.0)
            longitudinal_capacity = np.sqrt(max(1.0 - lateral_usage**2, 0.0))

            ax_avail_tilde = ax_max_available * longitudinal_capacity

        return ax_avail_tilde, ay_tilde, g_tilde

    def __extend_emergency_trajectory(
        self,
        trajectory: dict,
        track_handler: Track,
        s_range: np.ndarray,
        vehicle_params,
        msgs_logger

    ):
        """Extend emergency trajectory with smooth polynomial lateral profile.

        Generates a 5th-order polynomial extension that smoothly brings the vehicle
        back to the centerline (n=0) while maintaining C2 continuity at the junction.
        Tries multiple extension lengths from s_range until one passes track bounds.

        Args:
            trajectory: Trajectory dict to extend IN-PLACE. Must contain keys:
                V, s_dot, s, n, chi, Omega_z, n_dot, t, s_loc, and acceleration fields.
            track_handler: Track geometry for bounds checking and curvature.
            s_range: Array of extension lengths to try [m], from longest to shortest.
            vehicle_params: Dict with 'total_width' for collision checking.
            msgs_logger: Logger instance (currently unused in this method).

        Returns:
            bool: True if extension succeeded and trajectory was modified,
                  False if no valid extension found within track bounds.

        Notes:
            - Modifies trajectory dict in-place by appending extension arrays.
            - Uses boundary conditions: start matches trajectory end (n, n', n''),
              end targets centerline (n=0, n'=0, n''=0).
            - Extension includes tube_width and emergency safety distances in bounds.
        """
        # Safety check: ensure trajectory has sufficient points
        if len(trajectory.get("V", [])) == 0:
            rospy.logerr("Emergency trajectory extension: trajectory is empty")
            return False

        if len(trajectory.get("s_dot", [])) == 0 or trajectory["s_dot"][-1] == 0:
            rospy.logerr("Emergency trajectory extension: invalid s_dot")
            return False

        points_to_reach_min_traj_points = self.params.num_samples - \
            len(trajectory)
        ds = self.params.extension_point_distance

        # End values of current emergency trajectory
        V = trajectory["V"][-1]
        n = trajectory["n"][-1]
        s = trajectory["s"][-1]
        s_dot = trajectory["s_dot"][-1]
        chi = trajectory["chi"][-1]
        omega_z_vf = trajectory["Omega_z"][-1]
        omega_z_reference_frame_start = track_handler.omega_z(s)
        d_omega_z_reference_frame_start = track_handler.d_omega_z(s)

        # Boundary conditions of lateral extension sample (sampled regarding to s not to t) for the start
        n_prime_start = trajectory["n_dot"][-1] / trajectory["s_dot"][-1]

        n_pprime_start = -(
            d_omega_z_reference_frame_start * n +
            omega_z_reference_frame_start * n_prime_start
        ) * np.tan(chi) + (1 - omega_z_reference_frame_start * n) / np.cos(chi) ** 2 * (
            omega_z_vf * (1 - omega_z_reference_frame_start * n) /
            np.cos(chi) - omega_z_reference_frame_start
        )

        for length_emergency in s_range:
            num_samples = max(int(length_emergency / ds), max(
                self.params.extension_min_resolution, points_to_reach_min_traj_points))
            ds = length_emergency / num_samples
            # Calculate the extention of time, velocity, s_global and s_local
            trajectory_extension = {}
            trajectory_extension["t"] = trajectory["t"][-1] + \
                np.cumsum(ds / V * np.ones(num_samples))
            trajectory_extension["V"] = V * np.ones(num_samples)
            trajectory_extension["s_dot"] = s_dot * np.ones(num_samples)
            trajectory_extension["s"] = (
                s + np.cumsum(ds * np.ones(num_samples))) % track_handler.s_coord()[-1]
            trajectory_extension["s_loc"] = trajectory["s_loc"][-1] + \
                np.cumsum(ds * np.ones(num_samples))

            # Linaer euqation systems for calculating extention polynomial
            s_loc_loc = trajectory_extension["s_loc"] - \
                trajectory_extension["s_loc"][0] + ds
            s_end = s_loc_loc[-1]

            # Safety check: ensure s_end is large enough to avoid singular matrix
            if s_end < 0.01:  # Less than 1cm extension is too small
                rospy.logwarn(
                    f"Emergency trajectory extension too short (s_end={s_end:.6f}m), skipping")
                continue

            a = np.array(
                [
                    [1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0],
                    [1, s_end, s_end**2, s_end**3, s_end**4, s_end**5],
                    [0, 1, 2 * s_end, 3 * s_end**2, 4 * s_end**3, 5 * s_end**4],
                    [0, 0, 2, 6 * s_end, 12 * s_end**2, 20 * s_end**3],
                ]
            )

            # Boundary conditions of lateral extension sample (sampled regarding to s not to t) for the end
            n_rl_end = 0.0
            n_prime_end = 0.0
            n_pprime_end = 0.0

            b = np.array([n, n_prime_start, n_pprime_start,
                         n_rl_end, n_prime_end, n_pprime_end])

            # Calculating coefficients of sample
            try:
                c = np.linalg.solve(a=a, b=b)
            except np.linalg.LinAlgError as e:
                rospy.logwarn(
                    f"Emergency trajectory: Singular matrix with s_end={s_end:.6f}m, skipping this extension")
                continue

            # Calculate n and derivatives regarding to s
            s_quad = s_loc_loc * s_loc_loc
            s_cubi = s_loc_loc * s_quad
            s_quar = s_loc_loc * s_cubi
            s_quin = s_loc_loc * s_quar

            s_matrix = np.column_stack(
                (np.ones_like(s_loc_loc), s_loc_loc, s_quad, s_cubi, s_quar, s_quin))

            trajectory_extension["n"] = s_matrix @ c
            dn_ds = (s_matrix[:, :-1] * np.array([1, 2, 3, 4, 5])) @ c[1:]
            dn_ds2 = (s_matrix[:, :-2] * np.array([2, 6, 12, 20])) @ c[2:]

            # Interpolate the angle velocity of the road frame (reference frame)
            omega_z_reference_frame = track_handler.omega_z(
                trajectory_extension["s"]).squeeze()

            # Calculate the angle acceleration of the road frame (reference frame)
            d_omega_z_reference_frame = track_handler.d_omega_z(
                trajectory_extension["s"]).squeeze()

            # Calculate heading and s_dot
            trajectory_extension["chi"] = np.arctan(
                dn_ds / (1 - omega_z_reference_frame * trajectory_extension["n"]))

            # Calculate angular velocity regarding to velocity frame
            trajectory_extension["Omega_z"] = (
                (
                    (
                        dn_ds2
                        + (d_omega_z_reference_frame *
                           trajectory_extension["n"] + omega_z_reference_frame * dn_ds)
                        * np.tan(trajectory_extension["chi"])
                    )
                    * np.cos(trajectory_extension["chi"]) ** 2
                    / (1 - omega_z_reference_frame * trajectory_extension["n"])
                    + omega_z_reference_frame
                )
                * np.cos(trajectory_extension["chi"])
                / (1 - omega_z_reference_frame * trajectory_extension["n"])
            )

            # Calculate accelerations (vertical velocity w of reference frame is not considered regrading a_y)
            trajectory_extension["ay"] = (
                trajectory_extension["V"] ** 2) * trajectory_extension["Omega_z"]
            trajectory_extension["ax"] = np.zeros(num_samples)
            trajectory_extension["ax_tilde"], trajectory_extension["ay_tilde"], trajectory_extension["g_tilde"] = (
                track_handler.calc_apparent_acceleration(
                    trajectory_extension["s"],
                    trajectory_extension["n"],
                    trajectory_extension["chi"],
                    trajectory_extension["ax"],
                    trajectory_extension["ay"],
                    trajectory_extension["V"],
                )
            )
            trajectory_extension["ax_tilde"] = np.squeeze(
                trajectory_extension["ax_tilde"])
            trajectory_extension["ay_tilde"] = np.squeeze(
                trajectory_extension["ay_tilde"])
            trajectory_extension["g_tilde"] = np.squeeze(
                trajectory_extension["g_tilde"])
            trajectory_extension["tire_util"] = np.zeros(num_samples)

            # Check if trajectory extension is within track:
            left_bound = (
                track_handler.trackwidth_left(
                    trajectory_extension["s"]).squeeze()
                - vehicle_params["total_width"] / 2.0
                - (self.params.tube_width)
                - self.params.add_emergency_safety_distance_left
            )
            right_bound = (
                track_handler.trackwidth_right(
                    trajectory_extension["s"]).squeeze()
                + vehicle_params["total_width"] / 2.0
                + (self.params.tube_width)
                + self.params.add_emergency_safety_distance_right
            )

            if not np.all((trajectory_extension["n"] < (left_bound)) & (trajectory_extension["n"] > (right_bound))):
                continue
            else:
                # Add trajectory extention to original trajectory
                for key in trajectory_extension:
                    if isinstance(trajectory[key], np.ndarray):
                        trajectory[key] = np.concatenate(
                            (trajectory[key], trajectory_extension[key]))
                return True
        rospy.logwarn(
            "No trajectory extension passed the checks. Trajectory is not extended!")
        return False

    def calc_emergency_trajectory(
        self,
        track_handler: Track,
        performance_trajectory,
        # NOTE: Not used in current implementation (GGGV data unavailable)
        gggv_handler,
        pitlane_mode,
        vehicle_params,
        msgs_logger
    ):
        """
        Calculate physics-based emergency braking trajectory.

        This generates optimal braking trajectories using Pacejka tire model and physics.
        Used as emergency fallback when TAM sampling fails to find valid trajectories.

        NOTE: gggv_handler parameter is kept for compatibility but not actively used.
        Braking limits are calculated using Pacejka tire model or simplified physics fallback.
        """

        skip_update = getattr(self, '_skip_param_updates', False)
        self.declare_and_update_parameters(skip_update=skip_update)

        # Safety check: ensure performance trajectory is valid
        if not performance_trajectory or len(performance_trajectory.get("V", [])) < 2:
            rospy.logerr(
                "Emergency trajectory: performance trajectory is invalid or too short")
            return None

        emergency_trajectory = copy.deepcopy(performance_trajectory)
        emergency_trajectory["emergency"] = True

        start_idx = 0
        extension_count = 0
        while not (emergency_trajectory["V"][-1] <= 1e-3 and emergency_trajectory["V"][-2] <= 1e-3):
            # Forward solver
            for i, (s, n, chi, Omega_z) in enumerate(
                zip(
                    emergency_trajectory["s"][start_idx:-1],
                    emergency_trajectory["n"][start_idx:-1],
                    emergency_trajectory["chi"][start_idx:-1],
                    emergency_trajectory["Omega_z"][start_idx:-1],
                ),
                start_idx,
            ):
                if emergency_trajectory["t"][i] < (self.params.const_trajectory_time + self.params.additional_const_time_emergency):
                    continue

                V = emergency_trajectory["V"][i]
                ax_avail_tilde, ay_tilde, g_tilde = self.__calc_ax_avail(
                    s=s, n=n, chi=chi, Omega_z=Omega_z, V=V, track_handler=track_handler, gggv_handler=gggv_handler, pitlane_mode=pitlane_mode)
                ax_avail_hat, ay_hat = track_handler.calc_acceleration(
                    s, chi, ax_avail_tilde, ay_tilde)
                emergency_trajectory["ax"][i] = ax_avail_hat
                emergency_trajectory["ay"][i] = ay_hat
                emergency_trajectory["ax_tilde"][i] = ax_avail_tilde
                emergency_trajectory["ay_tilde"][i] = ay_tilde
                emergency_trajectory["g_tilde"][i] = g_tilde

                ds = (emergency_trajectory["s"][i + 1] -
                      s) % track_handler.s_coord()[-1]
                emergency_trajectory["V"][i + 1] = (
                    np.sqrt(V**2 + 2 * ax_avail_tilde * ds) if V ** 2 +
                    2 * ax_avail_tilde * ds > 0.0 else 1e-4
                )
                emergency_trajectory["t"][i + 1] = emergency_trajectory["t"][i] + 2 * ds / (
                    V + emergency_trajectory["V"][i + 1]
                )

                if emergency_trajectory["V"][i + 1] < 1e-3:
                    dt = emergency_trajectory["t"][i +
                                                   1] - emergency_trajectory["t"][i]
                    emergency_trajectory["ax_tilde"][i] = 2 * \
                        (ds - V * dt) / dt**2

            # set accelerations for last point that is not inside the for loop:
            ax_avail_tilde, ay_tilde, g_tilde = self.__calc_ax_avail(
                s=emergency_trajectory["s"][i + 1],
                n=emergency_trajectory["n"][i + 1],
                chi=emergency_trajectory["chi"][i + 1],
                Omega_z=emergency_trajectory["Omega_z"][i + 1],
                V=emergency_trajectory["V"][i + 1],
                track_handler=track_handler,
                gggv_handler=gggv_handler,
                pitlane_mode=pitlane_mode
            )
            ax_avail_hat, ay_hat = track_handler.calc_acceleration(
                emergency_trajectory["s"][i +
                                          1], emergency_trajectory["chi"][i + 1], ax_avail_tilde, ay_tilde
            )
            emergency_trajectory["ax"][i + 1] = ax_avail_hat
            emergency_trajectory["ay"][i + 1] = ay_hat
            emergency_trajectory["ax_tilde"][i + 1] = ax_avail_tilde
            emergency_trajectory["ay_tilde"][i + 1] = ay_tilde
            emergency_trajectory["g_tilde"][i + 1] = g_tilde

            if emergency_trajectory["V"][i + 1] < 1e-3:
                emergency_trajectory["ax_tilde"][i + 1] = 0.0

            if not (emergency_trajectory["V"][-1] <= 1e-3 and emergency_trajectory["V"][-2] <= 1e-3):
                if extension_count == 0:
                    t_emergency_start = performance_trajectory["t"][-1] - \
                        self.params.extension_emergency_time_offset
                    mask = performance_trajectory["t"] <= t_emergency_start

                    start_idx = np.min(
                        np.argpartition(
                            np.abs(performance_trajectory["t"] - t_emergency_start), 2)[:2]
                    )

                    for key in emergency_trajectory:
                        try:
                            emergency_trajectory[key] = emergency_trajectory[key][mask]
                        except:
                            pass

                    s_min = performance_trajectory["s_loc"][-1] - \
                        emergency_trajectory["s_loc"][-1]
                else:
                    t_emergency_start = emergency_trajectory["t"][-1]
                    s_min = 5.0

                s_max = self.params.extension_max_s_sample
                s_range = np.linspace(
                    s_max, s_min, num=self.params.extension_n_samples)
                solution_within_trackbounds = self.__extend_emergency_trajectory(
                    trajectory=emergency_trajectory,
                    track_handler=track_handler,
                    s_range=s_range,
                    vehicle_params=vehicle_params,
                    msgs_logger=msgs_logger
                )
                extension_count = extension_count + 1
                if not solution_within_trackbounds:
                    return

        # set tire utilization to 1.0 while V > 0.0 + threshold
        emergency_trajectory["tire_util"] = np.where(
            emergency_trajectory["V"] > 1e-3, 1.0, 0.0)

        # Compute s_dot
        Omega_z = np.interp(
            emergency_trajectory["s"], track_handler.s_coord(), track_handler.omega_z())
        emergency_trajectory['s_dot'] = emergency_trajectory['V'] * np.cos(
            emergency_trajectory['chi']) / (1.0 - emergency_trajectory['n'] * Omega_z)

        return emergency_trajectory

    def extend_performance_trajectory(
        self,
        trajectory: dict,
        track_handler: Track,
    ):
        """Extend performance trajectory to meet minimum length requirement.

        If the trajectory is shorter than min_trajectory_length, extends it by
        appending constant-velocity, constant-lateral-offset samples. The extension
        maintains the final velocity and lateral offset of the original trajectory.

        Args:
            trajectory: Trajectory dict to extend IN-PLACE. Must contain keys:
                s_loc, V, n, s, t, and kinematic state fields.
            track_handler: Track geometry for curvature and coordinate transforms.

        Returns:
            dict: The extended trajectory (same object, modified in-place).

        Notes:
            - Only extends if current length < min_trajectory_length (with 1μm tolerance).
            - Extension follows track centerline offset by final 'n' value.
            - Acceleration fields (ax, jx, jy) are set to zero in extension.
            - Lateral dynamics (n_dot, n_ddot, chi) are set to zero (straight tracking).
        """
        # Reload parameters in case they've been updated
        skip_update = getattr(self, '_skip_param_updates', False)
        self.declare_and_update_parameters(skip_update=skip_update)

        # Calculate current trajectory length in s-coordinates
        current_s_length = trajectory["s_loc"][-1] - trajectory["s_loc"][0]

        # Check if extension is needed (with small tolerance for floating point errors)
        tolerance = 1e-6  # 1 micrometer tolerance
        if current_s_length >= (self.params.min_trajectory_length - tolerance):
            # No extension needed (within tolerance)
            return trajectory

        # Calculate how much length we need to add
        s_length_to_add = self.params.min_trajectory_length - current_s_length

        # Calculate number of samples needed for extension
        ds = self.params.min_trajectory_length / self.params.num_samples
        num_extension_samples = max(1, int(np.ceil(s_length_to_add / ds)))

        # Ensure we don't add more than needed - recalculate ds for exact length
        ds = s_length_to_add / num_extension_samples

        V = trajectory["V"][-1]
        n = trajectory["n"][-1]
        trajectory_extension = {}

        trajectory_extension["t"] = trajectory["t"][-1] + \
            np.cumsum(ds / V * np.ones(num_extension_samples))
        trajectory_extension["V"] = V * np.ones(num_extension_samples)
        trajectory_extension["n"] = n * np.ones(num_extension_samples)
        trajectory_extension["s"] = (
            trajectory["s"][-1] +
            np.cumsum(ds * np.ones(num_extension_samples))
        ) % track_handler.s_coord()[-1]
        trajectory_extension["s_loc"] = trajectory["s_loc"][-1] + \
            np.cumsum(ds * np.ones(num_extension_samples))

        # Note: NumPy < 1.21 doesn't support 'period' parameter in np.interp
        # For periodic interpolation, we rely on modulo wrapping of s coordinates above
        trajectory_extension["Omega_x"] = np.interp(
            trajectory_extension["s"],
            track_handler.s_coord(),
            track_handler.omega_x(),
        )
        trajectory_extension["Omega_y"] = np.interp(
            trajectory_extension["s"],
            track_handler.s_coord(),
            track_handler.omega_y(),
        )
        Omega_z_tmp = np.interp(
            trajectory_extension["s"],
            track_handler.s_coord(),
            track_handler.omega_z(),
        )

        # prevent Omega_z from becoming zero
        Omega_z_tmp_no_zeros = np.where(Omega_z_tmp == 0, 1e-7, Omega_z_tmp)

        trajectory_extension["Omega_z"] = 1.0 / \
            (1.0 / (Omega_z_tmp_no_zeros) - n)
        trajectory_extension["s_dot"] = V / (
            1.0 - n *
            np.interp(
                trajectory_extension["s"], track_handler.s_coord(), track_handler.omega_z())
        )
        trajectory_extension["ax"] = np.zeros(num_extension_samples)
        trajectory_extension["jx"] = np.zeros(num_extension_samples)
        trajectory_extension["jy"] = np.zeros(num_extension_samples)
        trajectory_extension["epsilon_rho"] = np.zeros(num_extension_samples)
        trajectory_extension["epsilon_V"] = np.zeros(num_extension_samples)
        trajectory_extension["ay"] = trajectory_extension["Omega_z"] * \
            trajectory_extension["V"] ** 2
        trajectory_extension["chi"] = np.zeros(num_extension_samples)
        trajectory_extension["ax_tilde"], trajectory_extension["ay_tilde"], trajectory_extension["g_tilde"] = (
            track_handler.calc_apparent_acceleration(
                trajectory_extension["s"],
                trajectory_extension["n"],
                trajectory_extension["chi"],
                trajectory_extension["ax"],
                trajectory_extension["ay"],
                trajectory_extension["V"],
            )
        )

        # Ensure arrays maintain at least 1D shape (don't squeeze to scalars)
        trajectory_extension["ax_tilde"] = np.atleast_1d(np.squeeze(
            trajectory_extension["ax_tilde"]))
        trajectory_extension["ay_tilde"] = np.atleast_1d(np.squeeze(
            trajectory_extension["ay_tilde"]))
        trajectory_extension["g_tilde"] = np.atleast_1d(np.squeeze(
            trajectory_extension["g_tilde"]))

        trajectory_extension["s_ddot"] = np.zeros(num_extension_samples)
        trajectory_extension["n_dot"] = np.zeros(num_extension_samples)
        trajectory_extension["n_ddot"] = np.zeros(num_extension_samples)

        trajectory_extension["tire_util"] = np.zeros(num_extension_samples)

        for key in trajectory:
            if isinstance(trajectory[key], np.ndarray):
                # Ensure both arrays are 1D before concatenation
                if key in trajectory_extension:
                    trajectory[key] = np.concatenate(
                        (trajectory[key], np.atleast_1d(trajectory_extension[key])))

        return trajectory
