#!/usr/bin/env python3
"""
TAM Coordinate Transformation Module
Frenet to Cartesian conversion and velocity frame transformations following original TAM approach

Ported from tam_race_stack/mod_planning/sampling_planner/sampling_planner/coordinate_transformation.py

F1Tenth Integration:
- Simplified convert_trajectory_to_wpnt_array() for F1Tenth message conversion
- Uses rospy params instead of param_management_py
- Full TAM functionality commented out (not needed for F1Tenth simulation)
"""

# F1Tenth essential imports
import numpy as np
import rospy

# F1Tenth track handler (uses global waypoints format)
# Note: Requires track_handler_global_waypoints.py, not original track_handler_py
try:
    from track_handler_global_waypoints import GlobalWaypointsTrackHandler as Track
except ImportError:
    print("WARNING: Using fallback Track import")
    from track_handler_py import Track

# F1Tenth message types
try:
    from f110_msgs.msg import Wpnt, WpntArray
    F110_MSGS_AVAILABLE = True
except ImportError:
    F110_MSGS_AVAILABLE = False
    print("WARNING: f110_msgs not available. F1Tenth conversion disabled.")

# ============================================================================
# FULL TAM IMPORTS (COMMENTED OUT - NOT NEEDED FOR F1TENTH)
# ============================================================================
# from planning_common.track.gggvManager import GGGVManager
# from tum_helpers_py.rotations import vector_to_2d, Eigen3D
# from tum_helpers_py import get_curvature_from_heading, get_curvature_from_points
# import param_management_py as pmg
# import time
# from dataclasses import dataclass
# import tum_type_conversions_ros_py._cpp_binding
# from tum_types_py._common_binding import EulerYPR


class CoordinateTransformation:
    """
    F1Tenth simplified coordinate transformation.
    Converts trajectory dictionaries to WpntArray messages.
    """

    def __init__(self, use_f1tenth_mode=True):
        """
        Initialize coordinate transformation for F1Tenth.

        Args:
            use_f1tenth_mode: If True, only F1Tenth functionality is enabled (default)
        """
        self.use_f1tenth_mode = use_f1tenth_mode

        # Load parameters from ROS parameter server
        self.load_ros_parameters()

    def load_ros_parameters(self):
        """Load parameters from ROS parameter server (replaces param_management_py)"""
        # F1Tenth doesn't need most of these, but keeping for compatibility
        self.tube_width = rospy.get_param('tube_width', 1.0)
        self.trajectory_len_controller = rospy.get_param(
            'trajectory_len_controller', 50)

        rospy.loginfo(f"F1Tenth CoordinateTransformation initialized:")
        rospy.loginfo(f"  - tube_width: {self.tube_width}")
        rospy.loginfo(
            f"  - trajectory_len_controller: {self.trajectory_len_controller}")

    def transform_to_velocity_frame(
        self,
        track_handler: Track,
        s_array: np.ndarray,
        s_dot_array: np.ndarray,
        s_ddot_array: np.ndarray,
        n_array: np.ndarray,
        n_dot_array: np.ndarray,
        n_ddot_array: np.ndarray,
        postprocessed_raceline: dict
    ):
        """
        Simplified F1TENTH velocity frame transformation for 2D planar motion.

        Transforms Frenet frame trajectory samples (s, n, derivatives) to velocity frame
        quantities (V, chi, accelerations, yaw rate) for trajectory validation and cost
        calculation.

        F1TENTH Simplifications vs Full TAM:
        - No 3D track geometry (banking/pitch angles = 0)
        - No quaternion calculations
        - No complex reference frame rotations
        - Direct 2D kinematic calculations

        Args:
            track_handler: Track handler with omega_z() for curvature
            s_array: Longitudinal position samples [m] - shape (n_samples, n_points)
            s_dot_array: Longitudinal velocity samples [m/s] - shape (n_samples, n_points)
            s_ddot_array: Longitudinal acceleration samples [m/s²] - shape (n_samples, n_points)
            n_array: Lateral position samples [m] - shape (n_samples, n_points)
            n_dot_array: Lateral velocity samples [m/s] - shape (n_samples, n_points)
            n_ddot_array: Lateral acceleration samples [m/s²] - shape (n_samples, n_points)
            postprocessed_raceline: Raceline data (not used in simplified version)

        Returns:
            Tuple of (V_array, chi_array, ax_vf_array, ay_vf_array, Omega_z_vf_array, kappa_vf_array):
            - V_array: Total velocity magnitude [m/s] - shape (n_samples, n_points)
            - chi_array: Heading angle in Frenet frame [rad] - shape (n_samples, n_points)
            - ax_vf_array: Longitudinal acceleration in velocity frame [m/s²]
            - ay_vf_array: Lateral acceleration in velocity frame [m/s²]
            - Omega_z_vf_array: Yaw rate [rad/s] - shape (n_samples, n_points)
            - kappa_vf_array: Path curvature [rad/m] - shape (n_samples, n_points)
        """

        # Get track curvature (omega_z) at each s position
        # omega_z = dψ/ds where ψ is track heading
        # Handle both single trajectories and arrays of trajectories
        if s_array.ndim == 1:
            # Single trajectory
            omega_z_rf_array = track_handler.omega_z(s_array).flatten()
        else:
            # Multiple trajectories - interpolate for each
            track_s = track_handler.s_coord()
            track_omega_z = track_handler.omega_z(track_s).flatten()
            track_length = track_s[-1]

            omega_z_rf_array = np.zeros_like(s_array)
            for i in range(s_array.shape[0]):
                omega_z_rf_array[i] = np.interp(
                    s_array[i],
                    track_s,
                    track_omega_z,
                    period=track_length
                )

        # Calculate chi (heading angle relative to track tangent)
        # chi = arctan(n_dot / s_dot)
        # Handle division by zero with small epsilon
        epsilon = 1e-6
        s_dot_safe = np.where(np.abs(s_dot_array) >
                              epsilon, s_dot_array, epsilon)
        chi_array = np.arctan2(n_dot_array, s_dot_safe)

        # Calculate total velocity magnitude
        # V = sqrt(s_dot² + n_dot²)
        V_array = np.sqrt(s_dot_array**2 + n_dot_array**2)

        # Calculate yaw rate in velocity frame
        # Omega_z_vf = omega_z * s_dot * (1 - n * omega_z) + chi_dot
        # For F1TENTH with small lateral deviations, (1 - n * omega_z) ≈ 1
        # chi_dot = d(chi)/dt = (n_ddot * s_dot - n_dot * s_ddot) / (s_dot² + n_dot²)

        # Calculate chi_dot (time derivative of chi)
        velocity_squared = s_dot_array**2 + n_dot_array**2
        velocity_squared_safe = np.where(
            velocity_squared > epsilon**2, velocity_squared, epsilon**2)
        chi_dot = (n_ddot_array * s_dot_array - n_dot_array *
                   s_ddot_array) / velocity_squared_safe

        # Calculate Omega_z in velocity frame (yaw rate in rad/s)
        # For F1TENTH 2D kinematics: Omega_z_vf = omega_z * s_dot + chi_dot
        Omega_z_vf_array = omega_z_rf_array * s_dot_array + chi_dot

        # Calculate trajectory curvature (kappa in rad/m)
        # kappa = Omega_z / V (path curvature, not yaw rate)
        # Use safe division to avoid numerical issues at low velocities
        V_safe = np.maximum(V_array, epsilon)
        kappa_vf_array = Omega_z_vf_array / V_safe

        # Calculate accelerations in velocity frame
        # For planar motion without banking/pitch:
        # ax_vf = s_ddot * cos(chi) + n_ddot * sin(chi) - n_dot * omega_z * s_dot
        # ay_vf = -s_ddot * sin(chi) + n_ddot * cos(chi) + s_dot * omega_z * s_dot + s_dot * chi_dot

        cos_chi = np.cos(chi_array)
        sin_chi = np.sin(chi_array)

        # Longitudinal acceleration in velocity frame
        # Includes projection of Frenet accelerations and centripetal effects
        ax_vf_array = (
            s_ddot_array * cos_chi +
            n_ddot_array * sin_chi -
            n_dot_array * omega_z_rf_array * s_dot_array
        )

        # Lateral acceleration in velocity frame
        # Includes projection of Frenet accelerations and centripetal/Coriolis effects
        ay_vf_array = (
            -s_ddot_array * sin_chi +
            n_ddot_array * cos_chi +
            s_dot_array * omega_z_rf_array * s_dot_array * (1.0 - n_array * omega_z_rf_array) +
            s_dot_array * chi_dot
        )

        # For very small velocities, set accelerations to reasonable values
        # to avoid numerical issues
        mask_low_velocity = V_array < 0.01  # Less than 1 cm/s
        if np.any(mask_low_velocity):
            ax_vf_array = np.where(
                mask_low_velocity, s_ddot_array, ax_vf_array)
            ay_vf_array = np.where(mask_low_velocity, 0.0, ay_vf_array)
            Omega_z_vf_array = np.where(
                mask_low_velocity, 0.0, Omega_z_vf_array)
            kappa_vf_array = np.where(
                mask_low_velocity, 0.0, kappa_vf_array)

        return V_array, chi_array, ax_vf_array, ay_vf_array, Omega_z_vf_array, kappa_vf_array

    def __cut_trajectory_at_zero(self, trajectory_N):

        # get indices where emergency trajectory is zero
        zero_idxs = np.where(trajectory_N["v"] <= 0.001)[0]

        # cut trajectory after reaching zero (plus buffer)
        if zero_idxs.size:  # case: trajectory reaches zero
            zero_buffer = 10
            first_zero_idx = zero_idxs[0]
            cut_idx = first_zero_idx + zero_buffer

            # cut trajectory at zero index + buffer
            if cut_idx < trajectory_N["v"].size:
                for key in list(trajectory_N.keys()):
                    try:
                        trajectory_N[key] = trajectory_N[key][:cut_idx]
                    except:
                        pass
        else:  # case: trajectory does not reach zero
            print("WARNING: Emergency trajectory does not reach zero!")

    def __recalc_ax_profile(self, trajectory_N):
        # recalc ax profile
        trajectory_N["ax"] = np.zeros_like(trajectory_N["v"])

        # Calculate position differences
        ds = np.diff(trajectory_N["s_loc"])

        # Avoid division by zero: only calculate where ds > epsilon
        epsilon = 1e-6  # Minimum distance threshold (1 micron)
        valid_mask = np.abs(ds) > epsilon

        # Calculate acceleration only for valid segments
        if np.any(valid_mask):
            v_diff_sq = np.power(
                trajectory_N["v"][1:], 2) - np.power(trajectory_N["v"][:-1], 2)
            trajectory_N["ax"][:-1] = np.where(
                valid_mask,
                v_diff_sq / (2 * ds),
                0.0  # Zero acceleration for stationary segments
            )

        trajectory_N["ax"][-1] = trajectory_N["ax"][-2]

        # set ax to zero when velocity is zero
        trajectory_N["ax"] = np.where(
            trajectory_N["v"] <= 0.0001, 0.0, trajectory_N["ax"])

    def __create_trim_mask(self, array_in, len_array_out):
        """
        Create mask for downsampling trajectory.
        NOTE: This method is from full TAM but kept for compatibility.
        F1Tenth typically doesn't need trajectory trimming.
        """
        remove_mask = np.ones(array_in.size, dtype=np.bool_)
        num_remove = array_in.size - len_array_out  # number of elements to be removed

        remove_count = 0  # removed
        iter = 1
        i = array_in.size - iter - 1

        # start_time = time.time()
        while remove_count < num_remove:
            if i > 0:
                if remove_mask[i + iter]:
                    remove_mask[i] = False
                    remove_count += 1
                i -= iter
            else:
                iter = iter * 2
                i = array_in.size - iter - 1

        return remove_mask

    def convert_trajectory_to_wpnt_array(
        self,
        trajectory: dict,
        track_handler: Track,
        traj_cnt: int = 0
    ):
        """
        Simplified conversion for F1Tenth: Trajectory dict -> WpntArray message

        This method performs minimal postprocessing for F1Tenth controllers:
        1. Converts Frenet (s, n) to Cartesian (x, y)
        2. Calculates 2D heading (psi) from chi
        3. Packages into WpntArray message format

        Unlike calc_values_for_controller(), this does NOT:
        - Transform to 3D coordinates
        - Calculate quaternions
        - Compute angular velocities (Omega_x, Omega_y, Omega_z)
        - Apply GGGV acceleration limits
        - Calculate tube widths
        - Perform trim mask downsampling (controller handles length)

        Postprocessing steps performed:
        - Cut trajectory at zero velocity (for emergency trajectories)
        - Recalculate ax profile from velocity differences
        - Interpolate track properties (phi, mu, kappa)

        Args:
            trajectory: Dict with keys s, n, v (or V), chi, ax, ay
            track_handler: Track geometry handler
            traj_cnt: Trajectory counter/ID

        Returns:
            WpntArray message if f110_msgs available, else dict
        """

        if not F110_MSGS_AVAILABLE:
            rospy.logerr("f110_msgs not available for WpntArray conversion")
            return None

        # Make a working copy
        traj = {}
        for key in trajectory:
            if isinstance(trajectory[key], np.ndarray):
                traj[key] = trajectory[key].copy()
            else:
                traj[key] = trajectory[key]

        # Handle velocity field name (V or v)
        if "V" in traj and "v" not in traj:
            traj["v"] = traj["V"]
        elif "v" not in traj and "V" not in traj:
            raise ValueError("Trajectory must contain 'v' or 'V' field")

        # Postprocessing Step 1: Cut emergency trajectories at zero velocity
        if traj.get("emergency", False):
            self.__cut_trajectory_at_zero(traj)

        # Postprocessing Step 2: Recalculate ax profile from velocity
        # This ensures consistency between velocity and acceleration
        if "s_loc" in traj:
            self.__recalc_ax_profile(traj)

        # Get trajectory length
        n_points = len(traj["s"])

        # Convert Frenet to Cartesian
        xyz_array = track_handler.sn2cartesian(traj["s"], traj["n"])
        x_array = xyz_array[:, 0]
        y_array = xyz_array[:, 1]

        # Calculate 2D heading from chi
        psi_array = track_handler.calc_2d_heading_from_chi(
            traj["s"], traj["chi"]
        )

        # Interpolate track curvature
        kappa_array = np.interp(
            traj["s"],
            track_handler.s_coord(),
            track_handler.kappa(),
            period=track_handler.s_coord()[-1]
        )

        # Get track boundaries
        d_left_array = track_handler.trackwidth_left(
            traj["s"]).flatten() - traj["n"]
        d_right_array = np.abs(track_handler.trackwidth_right(
            traj["s"]).flatten() - traj["n"])

        # Create WpntArray message
        wpnt_array_msg = WpntArray()
        wpnt_array_msg.wpnts = []

        # Fill waypoints
        for i in range(n_points):
            wpnt = Wpnt()
            wpnt.id = i
            wpnt.s_m = float(traj["s"][i])
            wpnt.d_m = float(traj["n"][i])
            wpnt.x_m = float(x_array[i])
            wpnt.y_m = float(y_array[i])
            wpnt.d_left = float(d_left_array[i])
            wpnt.d_right = float(d_right_array[i])
            wpnt.psi_rad = float(psi_array[i])
            wpnt.kappa_radpm = float(kappa_array[i])
            wpnt.vx_mps = float(traj["v"][i])

            # Handle ax field (may be ax_tilde or ax depending on trajectory type)
            if "ax" in traj:
                wpnt.ax_mps2 = float(traj["ax"][i])
            elif "ax_tilde" in traj:
                wpnt.ax_mps2 = float(traj["ax_tilde"][i])
            else:
                wpnt.ax_mps2 = 0.0

            wpnt_array_msg.wpnts.append(wpnt)

        return wpnt_array_msg

    # def calc_values_for_controller(
    #         self,
    #         trajectory: dict,
    #         track_handler: Track,
    #         traj_cnt: int,
    #         gggv_handler: GGGVManager,
    #         pitlane_mode: bool,
    #         vehicle_params: dict

    # ):

    #     phi_array = np.interp(trajectory['s'], track_handler.s_coord(
    #     ), track_handler.phi(), period=track_handler.s_coord()[-1])
    #     mu_array = np.interp(trajectory['s'], track_handler.s_coord(
    #     ), track_handler.mu(), period=track_handler.s_coord()[-1])

    #     # Values that are the same for the performance and emergency trajectory
    #     s_glob_array = trajectory["s"]
    #     xyz_array = track_handler.sn2cartesian(
    #         trajectory["s"], trajectory["n"])
    #     x_array = xyz_array[:, 0]
    #     y_array = xyz_array[:, 1]
    #     chi_array = trajectory["chi"]
    #     psi_array = track_handler.calc_2d_heading_from_chi(
    #         trajectory["s"],
    #         chi_array,
    #     )

    #     # Calculate psi, mu and phi from inertial to velocity frame
    #     quaternions_x, quaternions_y, quaternions_z, quaternions_w = [], [], [], []
    #     euler_angles = np.array((track_handler.angles_to_velocity_frame(
    #         trajectory["s"],
    #         chi_array,
    #     )))

    #     # Convert euler angles to quaternions
    #     for row in euler_angles:
    #         input = EulerYPR(row[0], row[1], row[2])
    #         quaternion = (tum_type_conversions_ros_py._cpp_binding._bound_euler_type_to_quaternion_msg(
    #             input
    #         ))

    #         quaternions_x.append(quaternion[0])
    #         quaternions_y.append(quaternion[1])
    #         quaternions_z.append(quaternion[2])
    #         quaternions_w.append(quaternion[3])

    #     # Add angular velocity calculations
    #     # Compute Omega_xyz at each trajectory point
    #     Omega_x_array = track_handler.omega_x(trajectory["s"]).flatten()
    #     Omega_y_array = track_handler.omega_y(trajectory["s"]).flatten()
    #     Omega_z_array = track_handler.omega_z(trajectory["s"]).flatten()

    #     # Compute time derivative of chi (dot_chi)
    #     dot_chi_array = np.gradient(
    #         trajectory["chi"], trajectory["t"]).flatten()

    #     # Compute hat_omega_z
    #     hat_omega_z_array = Omega_z_array * trajectory["s_dot"] + dot_chi_array

    #     # Compute cos and sin of chi
    #     cos_chi = np.cos(trajectory["chi"])
    #     sin_chi = np.sin(trajectory["chi"])

    #     # Compute hat_omega_x and hat_omega_y
    #     hat_omega_x_array = (Omega_x_array * cos_chi +
    #                          Omega_y_array * sin_chi) * trajectory["s_dot"]
    #     hat_omega_y_array = (Omega_y_array * cos_chi -
    #                          Omega_x_array * sin_chi) * trajectory["s_dot"]

    #     # Store computed angular velocities in the trajectory dictionary
    #     trajectory["hat_omega_x"] = hat_omega_x_array
    #     trajectory["hat_omega_y"] = hat_omega_y_array
    #     trajectory["hat_omega_z"] = hat_omega_z_array

    #     # Values that are the same for both trajectories
    #     trajectory["traj_cnt"] = traj_cnt
    #     trajectory["s_glob"] = s_glob_array
    #     trajectory["x"] = x_array
    #     trajectory["y"] = y_array
    #     trajectory["psi"] = psi_array
    #     trajectory["mu"] = mu_array
    #     trajectory["bank"] = phi_array

    #     if np.any(np.diff(trajectory["s_glob"])) <= 0:
    #         print("ERROR: Trajectory is not monotonically increasing in s_glob")
    #         print(trajectory["s_glob"])

    #     # Quaternions from inertial to velocity frame
    #     trajectory["quaternion_x"] = np.array(quaternions_x)
    #     trajectory["quaternion_y"] = np.array(quaternions_y)
    #     trajectory["quaternion_z"] = np.array(quaternions_z)
    #     trajectory["quaternion_w"] = np.array(quaternions_w)

    #     # Over s, widht_left, width right
    #     if pitlane_mode and np.all(trajectory["V"] < 20.0):
    #         dist_left_bound = track_handler.trackwidth_left(
    #             trajectory["s"]).flatten() - trajectory["n"]
    #         dist_right_bound = np.abs(track_handler.trackwidth_right(
    #             trajectory["s"]).flatten() - trajectory["n"])

    #         safety_margin_tube_hardcoded = 0.4
    #         tube_width_min_hardcoded = 0.5

    #         trajectory["tube_l"] = np.maximum(np.minimum(np.ones_like(trajectory["s"]) * self.params.tube_width, dist_left_bound -
    #                                           0.5 * vehicle_params["total_width"] - safety_margin_tube_hardcoded), tube_width_min_hardcoded)
    #         trajectory["tube_r"] = np.maximum(np.minimum(np.ones_like(trajectory["s"]) * self.params.tube_width, dist_right_bound -
    #                                           0.5 * vehicle_params["total_width"] - safety_margin_tube_hardcoded), tube_width_min_hardcoded)
    #     else:
    #         trajectory["tube_l"] = np.ones_like(
    #             trajectory["s"]) * self.params.tube_width
    #         trajectory["tube_r"] = np.ones_like(
    #             trajectory["s"]) * self.params.tube_width

    #     # receive acceleration limts in tilde frame
    #     _, ax_tilde_min, ax_tilde_max, ay_tilde_max, ym_max = gggv_handler.acc_interpolator(
    #         trajectory['V'], trajectory['g_tilde'], trajectory["s_glob"], trajectory["n"], not pitlane_mode, self.debugging
    #     )

    #     # ONLY TOUCH WHEN 100% MENTALLY PREPARED
    #     # increase limits of all trajectory points if tire limits are slightly violated, limit by maximum allowed tire limit violation
    #     # 1.0 as smallest value
    #     max_tire_util = max(np.max(trajectory["tire_util"]), 1.0)
    #     # value in range [1.0, tire_util_max_check]
    #     limit_scaling = min(max_tire_util, self.params.tire_util_max_check)
    #     ax_tilde_min *= limit_scaling
    #     ax_tilde_max *= limit_scaling
    #     ay_tilde_max *= limit_scaling

    #     # empty entries in limits
    #     zero_values = np.zeros_like(ax_tilde_min)

    #     # trajectory values for controller in 3D
    #     if self.params.send_trajectory_in_3d:
    #         # assign 2D coordinates of acc limit values
    #         trajectory["ax_min_x"] = ax_tilde_min
    #         trajectory["ax_min_y"] = zero_values

    #         trajectory["ax_max_x"] = ax_tilde_max
    #         trajectory["ax_max_y"] = zero_values

    #         trajectory["ay_min_x"] = zero_values
    #         # for symmetry, take the negative max value of ay
    #         trajectory["ay_min_y"] = -ay_tilde_max

    #         trajectory["ay_max_x"] = zero_values
    #         trajectory["ay_max_y"] = ay_tilde_max

    #         trajectory["v"] = trajectory["V"]
    #         trajectory["ax"] = trajectory["ax_tilde"]
    #         trajectory["ay"] = trajectory["ay_tilde"]
    #         trajectory["az"] = trajectory["g_tilde"]

    #         # set accelerations to zero if velocity is zero
    #         trajectory["ax"] = np.where(
    #             trajectory["V"] <= 0.0001, 0.0, trajectory["ax"])

    #         trajectory["kappa"] = trajectory["Omega_z"]

    #     # trajectory values for controller in 2D
    #     else:

    #         # create Eigen3D struct for pybind method
    #         rotation = Eigen3D()
    #         rotation.x = phi_array  # banking
    #         rotation.y = mu_array  # pitch
    #         rotation.z = np.zeros_like(phi_array)

    #         ax_min_2D_tilde_eigen = vector_to_2d(
    #             Eigen3D(ax_tilde_min, zero_values,
    #                     trajectory["g_tilde"]), mu_array, phi_array
    #         )
    #         ax_max_2D_tilde_eigen = vector_to_2d(
    #             Eigen3D(ax_tilde_max, zero_values,
    #                     trajectory["g_tilde"]), mu_array, phi_array
    #         )
    #         ay_min_2D_tilde_eigen = vector_to_2d(
    #             Eigen3D(zero_values, -ay_tilde_max,
    #                     trajectory["g_tilde"]), mu_array, phi_array
    #         )
    #         ay_max_2D_tilde_eigen = vector_to_2d(
    #             Eigen3D(zero_values, ay_tilde_max,
    #                     trajectory["g_tilde"]), mu_array, phi_array
    #         )

    #         # assign 2D coordinates of acc limit values
    #         trajectory["ax_min_x"] = ax_min_2D_tilde_eigen.x
    #         trajectory["ax_min_y"] = ax_min_2D_tilde_eigen.y

    #         trajectory["ax_max_x"] = ax_max_2D_tilde_eigen.x
    #         trajectory["ax_max_y"] = ax_max_2D_tilde_eigen.y

    #         trajectory["ay_min_x"] = ay_min_2D_tilde_eigen.x
    #         trajectory["ay_min_y"] = ay_min_2D_tilde_eigen.y

    #         trajectory["ay_max_x"] = ay_max_2D_tilde_eigen.x
    #         trajectory["ay_max_y"] = ay_max_2D_tilde_eigen.y

    #         # calc 2D velocity and accelerations
    #         v_trajectory = vector_to_2d(
    #             Eigen3D(trajectory["V"], zero_values, zero_values), mu_array, phi_array)
    #         a_trajectory = vector_to_2d(
    #             Eigen3D(trajectory["ax_tilde"], trajectory["ay_tilde"],
    #                     trajectory["g_tilde"]), mu_array, phi_array
    #         )

    #         # assign 2D trajectory velocity
    #         trajectory["v"] = v_trajectory.x

    #         # assign 2D trajectory accelerations
    #         trajectory["ax"] = a_trajectory.x
    #         trajectory["ay"] = a_trajectory.y
    #         trajectory["az"] = a_trajectory.z

    #         # recalculate 2D kappa
    #         kappa_from_heading = get_curvature_from_heading(
    #             trajectory["psi"], trajectory["s_loc"])
    #         kappa_from_trafo = trajectory["ay"] / (trajectory["v"] ** 2)
    #         trajectory["kappa"] = np.where(
    #             trajectory["v"] > 3.0, kappa_from_trafo, kappa_from_heading)

    #         # set accelerations to zero if velocity is zero
    #         trajectory["ax"] = np.where(
    #             trajectory["V"] <= 0.0001, 0.0, trajectory["ax"])

    #     # initialize trajectory for controller
    #     trajectory_N = {}

    #     trajectory_N["pitlane_mode"] = trajectory["pitlane_mode"]
    #     trajectory_N["traj_cnt"] = trajectory["traj_cnt"]

    #     if (len(trajectory["t"]) > self.params.trajectory_len_controller) and not trajectory["emergency"]:
    #         trim_mask = self.__create_trim_mask(
    #             array_in=trajectory["s_loc"], len_array_out=self.params.trajectory_len_controller)
    #         for key in trajectory:
    #             try:
    #                 if isinstance(trajectory[key], np.ndarray):
    #                     trajectory_N[key] = trajectory[key][trim_mask]
    #                 elif isinstance(trajectory[key], float):
    #                     trajectory_N[key] = trajectory[key]
    #             except:
    #                 pass

    #     return trajectory_N, trajectory
