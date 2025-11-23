#!/usr/bin/env python3
"""
TAM Trajectory Checks Module
Safety and feasibility validation following original TAM approach

Ported from tam_race_stack/mod_planning/sampling_planner/sampling_planner/trajectory_checks.py

MODIFICATIONS:
- Uses ROS parameter server instead of param_manager
- Uses postprocessed_raceline format (processed from global waypoints)
- GGGV-dependent functionality replaced with Pacejka tire model
- Simplified for use with Pacejka tire model parameters
- NodeMonitor dependencies simplified to basic print statements

ACTIVE CHECKS:
- Curvature limits (kappa_thr parameter)
- Vehicle capability limits (max_speed, max_accel)
- Path collision with track boundaries
- Physics-based friction limits using Pacejka tire model
- Combined slip modeling (friction circle)

DISABLED CHECKS (commented out due to missing GGGV diagrams):
- Speed rule compliance (no rules defined)
- Apparent acceleration transformations (flat track assumption used instead)
- Yaw moment calculations (quasi-transient limits)
- GGGV interpolation-based friction limits (replaced by Pacejka)

PACEJKA FRICTION CHECKING:
- Uses PacejkaTireModel for physics-based tire force limits
- Implements friction circle model: (|ax|/ax_max)^n + (|ay|/ay_max)^n <= 1
- Considers combined longitudinal and lateral tire forces
- Accounts for velocity-dependent limits and track curvature
- Falls back to accepting all trajectories if tire model unavailable

Required ROS Parameters (with defaults):
- behavior/tube_width: 1.15
- behavior/tire_util_max_check: 1.1
- behavior/tire_util_relaxation: 1.0 (friction check relaxation: 1.0=default, >1.0=relaxed, <1.0=strict)
- behavior/kappa_thr: 0.1
- behavior/max_speed: 6.0 (m/s)
- behavior/max_accel: 2.0 (m/s²) - longitudinal acceleration limit
- behavior/max_lateral_accel: 2.0 (m/s²) - DEPRECATED: not used (lateral forces checked by Pacejka friction model)
- safety_distances/safety_distance_track_left: 0.0
- safety_distances/safety_distance_track_right: 0.0
- safety_distances/safety_distance_pitlane_left: 0.0
- safety_distances/safety_distance_pitlane_right: 0.0
- safety_distances/soft_safety_distance_left_m: 0.0
- safety_distances/soft_safety_distance_right_m: 0.0
"""
from track_handler_global_waypoints import GlobalWaypointsTrackHandler as Track
from simple_helper_utils import interpolate_with_period
import numpy as np
# GGGV-dependent imports commented out - no GGGV diagrams available, using Pacejka model instead
# from planning_common.track.gggvManager import GGGVManager, Grip_Map
# from planning_common.helper.utils import calc_Omega_z_dot_tilde
from dataclasses import dataclass
# from tum_types_py.common import ErrorLvl  # Commented out - simplified monitoring
# from ros2_watchdog_py.node_monitor import NodeMonitor  # Commented out - simplified monitoring
import rospy
# from planning_common.helper.utils import calc_raceline_tire_util


@dataclass(init=False)
class TrajectoryChecksParams():
    tube_width: float
    tire_util_max_check: float
    # Friction check relaxation factor (1.0=default, >1.0=relaxed, <1.0=strict)
    tire_util_relaxation: float
    kappa_thr: float
    safety_distance_track_left: float
    safety_distance_track_right: float
    safety_distance_pitlane_left: float
    safety_distance_pitlane_right: float
    soft_safety_distance_left_m: float
    soft_safety_distance_right_m: float
    # Vehicle capability limits
    max_speed: float  # Maximum vehicle speed (m/s)
    max_accel: float  # Maximum longitudinal acceleration (m/s²)
    max_lateral_accel: float  # Maximum lateral acceleration (m/s²)


class TrajectoryChecks():
    def __init__(self, debugging):
        """
        Initialize TrajectoryChecks module.

        Args:
            debugging: Enable debug logging and trajectory failure tracking

        Note: param_manager parameter removed - now uses ROS parameter server directly
        """
        self.params = TrajectoryChecksParams()
        self.initialized_params = False
        self.declare_and_update_parameters()
        self.debugging = debugging

        # Initialize Pacejka tire model for physics-based friction checking
        try:
            from pacejka_tire_model import PacejkaTireModel
            self.tire_model = PacejkaTireModel()
            self.use_tire_model = True
            rospy.loginfo(
                "TrajectoryChecks: Pacejka tire model initialized for friction checking")
        except Exception as e:
            self.tire_model = None
            self.use_tire_model = False
            rospy.logwarn(
                f"TrajectoryChecks: Could not initialize tire model: {e}")
            rospy.logwarn(
                "TrajectoryChecks: Friction checks will accept all trajectories")

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
                f"TrajectoryChecks: Could not load YAML defaults: {e}")
            return {}

    def declare_and_update_parameters(self):
        """Load parameters from ROS parameter server with YAML defaults."""
        if not self.initialized_params:
            yaml_defaults = self._load_yaml_defaults()

            self.params.tube_width = yaml_defaults.get(
                'tube_width', rospy.get_param("behavior/tube_width", 1.15))
            rospy.set_param("behavior/tube_width", self.params.tube_width)
            self.params.tire_util_max_check = yaml_defaults.get(
                'tire_util_max_check', rospy.get_param("behavior/tire_util_max_check", 1.1))
            rospy.set_param("behavior/tire_util_max_check",
                            self.params.tire_util_max_check)
            self.params.tire_util_relaxation = yaml_defaults.get(
                'tire_util_relaxation', rospy.get_param("behavior/tire_util_relaxation", 1.0))
            rospy.set_param("behavior/tire_util_relaxation",
                            self.params.tire_util_relaxation)
            self.params.kappa_thr = yaml_defaults.get(
                'kappa_thr', rospy.get_param("behavior/kappa_thr", 0.1))
            rospy.set_param("behavior/kappa_thr", self.params.kappa_thr)
            self.params.safety_distance_track_left = yaml_defaults.get(
                'safety_distance_track_left',
                rospy.get_param("safety_distances/safety_distance_track_left", 0.0))
            rospy.set_param("safety_distances/safety_distance_track_left",
                            self.params.safety_distance_track_left)
            self.params.safety_distance_track_right = yaml_defaults.get(
                'safety_distance_track_right',
                rospy.get_param("safety_distances/safety_distance_track_right", 0.0))
            rospy.set_param("safety_distances/safety_distance_track_right",
                            self.params.safety_distance_track_right)
            self.params.safety_distance_pitlane_left = yaml_defaults.get(
                'safety_distance_pitlane_left',
                rospy.get_param("safety_distances/safety_distance_pitlane_left", 0.0))
            rospy.set_param("safety_distances/safety_distance_pitlane_left",
                            self.params.safety_distance_pitlane_left)
            self.params.safety_distance_pitlane_right = yaml_defaults.get(
                'safety_distance_pitlane_right',
                rospy.get_param("safety_distances/safety_distance_pitlane_right", 0.0))
            rospy.set_param("safety_distances/safety_distance_pitlane_right",
                            self.params.safety_distance_pitlane_right)
            self.params.soft_safety_distance_left_m = yaml_defaults.get(
                'soft_safety_distance_left_m',
                rospy.get_param("safety_distances/soft_safety_distance_left_m", 0.0))
            rospy.set_param("safety_distances/soft_safety_distance_left_m",
                            self.params.soft_safety_distance_left_m)
            self.params.soft_safety_distance_right_m = yaml_defaults.get(
                'soft_safety_distance_right_m',
                rospy.get_param("safety_distances/soft_safety_distance_right_m", 0.0))
            # Vehicle capability limits
            self.params.max_speed = yaml_defaults.get(
                'max_speed', rospy.get_param("behavior/max_speed", 6.0))
            rospy.set_param("behavior/max_speed", self.params.max_speed)
            self.params.max_accel = yaml_defaults.get(
                'max_accel', rospy.get_param("behavior/max_accel", 2.0))
            rospy.set_param("behavior/max_accel", self.params.max_accel)
            self.params.max_lateral_accel = yaml_defaults.get(
                'max_lateral_accel', rospy.get_param("behavior/max_lateral_accel", 2.0))
            rospy.set_param("behavior/max_lateral_accel",
                            self.params.max_lateral_accel)

            self.initialized_params = True
        else:
            self.params.tube_width = rospy.get_param(
                "behavior/tube_width", self.params.tube_width)
            self.params.tire_util_max_check = rospy.get_param(
                "behavior/tire_util_max_check", self.params.tire_util_max_check)
            self.params.tire_util_relaxation = rospy.get_param(
                "behavior/tire_util_relaxation", self.params.tire_util_relaxation)
            self.params.kappa_thr = rospy.get_param(
                "behavior/kappa_thr", self.params.kappa_thr)
            self.params.safety_distance_track_left = rospy.get_param(
                "safety_distances/safety_distance_track_left",
                self.params.safety_distance_track_left)
            self.params.safety_distance_track_right = rospy.get_param(
                "safety_distances/safety_distance_track_right",
                self.params.safety_distance_track_right)
            self.params.safety_distance_pitlane_left = rospy.get_param(
                "safety_distances/safety_distance_pitlane_left",
                self.params.safety_distance_pitlane_left)
            self.params.safety_distance_pitlane_right = rospy.get_param(
                "safety_distances/safety_distance_pitlane_right",
                self.params.safety_distance_pitlane_right)
            self.params.soft_safety_distance_left_m = rospy.get_param(
                "safety_distances/soft_safety_distance_left_m",
                self.params.soft_safety_distance_left_m)
            self.params.soft_safety_distance_right_m = rospy.get_param(
                "safety_distances/soft_safety_distance_right_m",
                self.params.soft_safety_distance_right_m)
            # Vehicle capability limits
            self.params.max_speed = rospy.get_param(
                "behavior/max_speed", self.params.max_speed)
            self.params.max_accel = rospy.get_param(
                "behavior/max_accel", self.params.max_accel)
            self.params.max_lateral_accel = rospy.get_param(
                "behavior/max_lateral_accel", self.params.max_lateral_accel)

    def check_curvature(
        self,
        valid_array: np.ndarray,
        # Actually kappa (curvature in rad/m), not Omega_z (yaw rate in rad/s)
        Omega_z: np.ndarray,
        invalid_array_info: np.ndarray,
    ):
        valid_tmp = np.all(
            np.abs(Omega_z[valid_array]) <= self.params.kappa_thr, axis=1)

        valid_array[valid_array] = valid_tmp

    def __check_vehicle_capabilities(
        self,
        valid_array: np.ndarray,
        V_array: np.ndarray,
        ax_array: np.ndarray,
        ay_array: np.ndarray,
        invalid_array_info: np.ndarray,
        traj_cnt: int,
    ):
        """
        Check if trajectories exceed vehicle capabilities (max speed, max longitudinal accel).

        Args:
            valid_array: Boolean mask of currently valid trajectories
            V_array: Velocity array (m/s) - shape (n_trajectories, n_points)
            ax_array: Longitudinal acceleration in velocity frame (m/s²)
            ay_array: Lateral acceleration in velocity frame (m/s²) - NOT CHECKED (unused)
            invalid_array_info: Array for storing failure reasons
            traj_cnt: Trajectory counter for logging

        Note:
            - Checks max_speed and max_accel (longitudinal only)
            - Lateral acceleration NOT checked here - handled by Pacejka friction model
            - ay_array parameter kept for API compatibility but not used
        """
        # Small tolerance for numerical stability
        tolerance_speed = 2.0  # m/s
        tolerance_accel = 1.0  # m/s²

        # Check maximum speed
        valid_speed = np.all(
            V_array[valid_array] <= self.params.max_speed + tolerance_speed,
            axis=1
        )

        # Check maximum longitudinal acceleration (both positive and negative)
        valid_ax = np.all(
            np.abs(ax_array[valid_array]
                   ) <= self.params.max_accel + tolerance_accel,
            axis=1
        )

        # Note: Lateral acceleration check removed - handled by Pacejka friction check
        # which provides physics-based tire limits rather than guessed values

        # Combine all capability checks
        valid_tmp = valid_speed & valid_ax

        # Store trajectories that failed this check (only if not already marked)
        # Also track which specific capability was violated for detailed reporting
        if self.debugging:
            combined_mask = valid_array.copy()
            combined_mask[valid_array] = ~valid_tmp
            # Only mark trajectories that haven't failed a previous check
            invalid_array_info[combined_mask & (
                invalid_array_info == "")] = "vehicle_capability"

        valid_array[valid_array] = valid_tmp

        # Return detailed breakdown for summary (no valid_ay since lateral accel check removed)
        return valid_speed, valid_ax

    def __check_path_collision(
        self,
        track_handler: Track,
        valid_array: np.ndarray,
        s_array: np.ndarray,
        n_array: np.ndarray,
        pitlane_mode: bool,
        vehicle_params: dict,
        invalid_array_info: np.ndarray,
    ):
        # if pitlane_mode:
        #     safety_distance_left = self.params.safety_distance_pitlane_left
        #     safety_distance_right = self.params.safety_distance_pitlane_right
        # else:
        safety_distance_left = self.params.safety_distance_track_left
        safety_distance_right = self.params.safety_distance_track_right

        # Left boundary: positive value, reduce by margins to get usable left limit
        left_bound = (
            interpolate_with_period(
                s_array[valid_array],
                track_handler.s_coord(),
                track_handler.trackwidth_left(),
                track_handler.get_track_length(),
            )
            - vehicle_params["total_width"] / 2.0
            - (safety_distance_left + self.params.tube_width)
        )

        # Right boundary: trackwidth_right() returns POSITIVE width (distance from centerline)
        # We need NEGATIVE boundary (right side of track), so negate and ADD margins inward
        right_bound = (
            -interpolate_with_period(
                s_array[valid_array],
                track_handler.s_coord(),
                track_handler.trackwidth_right(),
                track_handler.get_track_length(),
            )
            + vehicle_params["total_width"] / 2.0
            + (safety_distance_right + self.params.tube_width)
        )

        valid_tmp = np.all((n_array[valid_array] < left_bound) & (
            n_array[valid_array] > right_bound), axis=1)

        # Check for impossible constraints due to excessive safety margins
        has_negative_left = np.any(left_bound < 0)
        has_positive_right = np.any(right_bound > 0)

        if has_negative_left or has_positive_right:
            rospy.logerr("\n" + "="*80)
            rospy.logerr(
                "⚠️⚠️⚠️  CRITICAL WARNING: SAFETY MARGINS TOO LARGE  ⚠️⚠️⚠️")
            rospy.logerr("="*80)
            if has_negative_left:
                min_left = np.min(left_bound)
                rospy.logerr(
                    f"❌ LEFT BOUND has NEGATIVE values (min: {min_left:.3f} m)")
                rospy.logerr(
                    f"   This means safety margins exceed available left track width!")
            if has_positive_right:
                max_right = np.max(right_bound)
                rospy.logerr(
                    f"❌ RIGHT BOUND has POSITIVE values (max: {max_right:.3f} m)")
                rospy.logerr(
                    f"   This means safety margins exceed available right track width!")
            rospy.logerr(f"\n🔧 ACTION REQUIRED: Reduce safety parameters:")
            rospy.logerr(
                f"   - Current tube_width: {self.params.tube_width:.3f} m")
            rospy.logerr(
                f"   - Current safety_distance_track_left: {safety_distance_left:.3f} m")
            rospy.logerr(
                f"   - Current safety_distance_track_right: {safety_distance_right:.3f} m")
            rospy.logerr(
                f"   - Current vehicle_width/2: {vehicle_params['total_width']/2.0:.3f} m")
            rospy.logerr(
                f"   - Total margin per side: ~{vehicle_params['total_width']/2.0 + safety_distance_left + self.params.tube_width:.3f} m")
            rospy.logerr(
                f"\n💡 These margins are NOT compatible with this raceline/map!")
            rospy.logerr(f"   Adjust parameters in tam_sampling_params.yaml")
            rospy.logerr("="*80 + "\n")

        valid_array[valid_array] = valid_tmp

        return left_bound, right_bound

    # def __check_rules(
    #     self,
    #     valid_array: np.ndarray,
    #     V_array: np.ndarray,
    #     V_target_rules: float,
    #     invalid_array_info: np.ndarray,
    # ):
    #     # RULES CHECK DISABLED - No speed rules to enforce
    #     # Original functionality commented out:

    #     # # allow slight violation
    #     # tolerance_mps = 3.0
    #     #
    #     # # In case all entries are false the next condition will crash otherwise
    #     # if np.sum(valid_array) > 0:
    #     #     # handle case when target velocity is reduced to a value below ego velocity
    #     #     if V_array[valid_array][0][0] > V_target_rules:
    #     #         V_target_rules = V_array[valid_array][0][0]
    #     #
    #     # # check if every trajectory point is below maximum allowed velocity
    #     # valid_tmp = np.all(V_array[valid_array] <
    #     #                    V_target_rules + tolerance_mps, axis=1)
    #     #
    #     # # store trajectories that failed this check for visualizer
    #     # if self.debugging:
    #     #     combined_mask = valid_array.copy()
    #     #     combined_mask[valid_array] = ~valid_tmp
    #     #     invalid_array_info[combined_mask] = "rules"
    #     #
    #     # # if no valid trajectory is found, allow all
    #     # if np.sum(valid_tmp) < 1:
    #     #     # self.msgs_logger.warning(f'Trajectory ID {self.traj_cnt}: All trajectories failed the rule check - no rule check applied in this step.')
    #     #     return
    #     #
    #     # valid_array[valid_array] = valid_tmp

    #     # No rules to check - accept all trajectories
    #     # if self.debugging:
    #     #     print("Rules check disabled - no speed rules to enforce")

    #     # All trajectories remain valid (no modification to valid_array needed)

    def __check_friction_limits(
            self,
            valid_array: np.ndarray,
            track_handler: Track,
            s_array: np.ndarray,
            V_array: np.ndarray,
            n_array: np.ndarray,
            chi_array: np.ndarray,
            ax_array: np.ndarray,
            ay_array: np.ndarray,
            t_array: np.ndarray,
            ggv_mode: str,
            gggv_handler,  # GGGVManager - commented out due to no GGGV diagrams
            traj_cnt: int,
            msgs_logger,  # NodeMonitor - simplified
            pitlane_mode: bool,
            invalid_array_info: np.ndarray,
            postprocessed_raceline: dict,
    ):
        ax_tilde = np.zeros_like(s_array)
        ay_tilde = np.zeros_like(s_array)
        g_tilde = np.zeros_like(s_array)
        tire_util_array = np.zeros_like(s_array)

        # Avoid crash due to empty array
        if np.sum(valid_array) < 1:
            return ax_tilde, ay_tilde, g_tilde, tire_util_array

        # GGGV-dependent friction limit checks commented out - no GGGV diagrams available
        # Using simplified approach without full physics-based tire limits
        # For Pacejka model integration, implement separate tire limit checks

        # Commented out: calc_apparent_acceleration requires GGGV functionality
        # ax_tilde[valid_array], ay_tilde[valid_array], g_tilde[valid_array] = track_handler.calc_apparent_acceleration(
        #     s_array[valid_array],
        #     n_array[valid_array],
        #     chi_array[valid_array],
        #     ax_array[valid_array],
        #     ay_array[valid_array],
        #     V_array[valid_array],
        # )

        # Simplified apparent acceleration (flat track assumption)
        ax_tilde[valid_array] = ax_array[valid_array]
        ay_tilde[valid_array] = ay_array[valid_array]
        g_tilde[valid_array] = 9.81  # Standard gravity

        # Pacejka-based friction limit checking
        if self.use_tire_model and self.tire_model is not None:
            try:
                # Calculate static normal loads (no pitch dynamics for F1TENTH)
                g = 9.81
                l_wb = self.tire_model.l_f + self.tire_model.l_r
                Fz_front_static = self.tire_model.mass * g * self.tire_model.l_r / l_wb
                Fz_rear_static = self.tire_model.mass * g * self.tire_model.l_f / l_wb

                # Get friction circle exponent
                n = self.tire_model.combined_slip_exponent

                # Process each valid trajectory
                valid_indices = np.where(valid_array)[0]
                for traj_idx in valid_indices:
                    # Process all points in this trajectory
                    for point_idx in range(s_array.shape[1]):
                        s = s_array[traj_idx, point_idx]
                        V = V_array[traj_idx, point_idx]
                        ax_actual = ax_tilde[traj_idx, point_idx]
                        ay_actual = ay_tilde[traj_idx, point_idx]

                        # Get track curvature at this point
                        try:
                            kappa = track_handler.omega_z(s)
                        except:
                            # Fallback if omega_z fails
                            kappa = 0.0

                        # Use actual lateral acceleration from trajectory for combined slip calculation
                        # This accounts for the friction circle: lateral acc reduces available longitudinal acc
                        ay_for_limits = np.abs(ay_actual)

                        # Get available forces from Pacejka model considering combined slip
                        Fx_available, Fy_max = self.tire_model.calc_combined_limits(
                            Fz_front_static, Fz_rear_static, ay_for_limits
                        )

                        # Convert forces to accelerations
                        ax_max = Fx_available / self.tire_model.mass
                        ay_max = Fy_max / self.tire_model.mass

                        # Avoid division by zero (minimum 0.1 m/s² limits)
                        ax_max = max(ax_max, 0.1)
                        ay_max = max(ay_max, 0.1)

                        # Calculate tire utilization using friction circle model
                        # Formula: (|ax|/ax_max)^n + (|ay|/ay_max)^n
                        # Note: ax_max already accounts for lateral usage via calc_combined_limits
                        tire_util = (np.abs(ax_actual) / ax_max)**n + \
                            (np.abs(ay_actual) / ay_max)**n
                        tire_util_array[traj_idx, point_idx] = tire_util

                # Check which trajectories are valid (all points must be within limit)
                # Apply relaxation factor: >1.0 relaxes (allows higher tire util), <1.0 makes stricter
                effective_threshold = self.params.tire_util_max_check * \
                    self.params.tire_util_relaxation
                valid_tmp = np.all(
                    tire_util_array[valid_array] <= effective_threshold,
                    axis=1
                )

                # Update valid array
                valid_array[valid_array] = valid_tmp

            except Exception as e:
                rospy.logwarn_throttle(
                    5.0,
                    f"Trajectory ID {traj_cnt}: Error in Pacejka friction check: {e}. "
                    "Accepting all trajectories."
                )
                # Fallback: accept all trajectories
                valid_tmp = np.ones(np.sum(valid_array), dtype=bool)
                valid_array[valid_array] = valid_tmp
        else:
            # No tire model available - accept all trajectories
            rospy.logwarn_throttle(
                10.0,
                f"Trajectory ID {traj_cnt}: Tire model unavailable, skipping friction check"
            )
            valid_tmp = np.ones(np.sum(valid_array), dtype=bool)
            valid_array[valid_array] = valid_tmp

        return ax_tilde, ay_tilde, g_tilde, tire_util_array

    def mandatory_checks_trajectory(self,
                                    # NOTE: Actually trajectory curvature (kappa) in rad/m, not yaw rate!
                                    Omega_z_vf_array: np.ndarray,
                                    track_handler: Track,
                                    s_array: np.ndarray,
                                    n_array: np.ndarray,
                                    t_array: np.ndarray,
                                    V_array: np.ndarray,
                                    V_target_rules: float,
                                    chi_array: np.ndarray,
                                    ax_vf_array: np.ndarray,
                                    ay_vf_array: np.ndarray,
                                    node_monitor,  # NodeMonitor - simplified
                                    msgs_logger,
                                    traj_cnt: int,
                                    pitlane_mode: bool,
                                    vehicle_params: dict,
                                    ggv_mode: str,
                                    gggv_handler,  # GGGVManager - commented out due to no GGGV diagrams
                                    postprocessed_raceline: dict,
                                    ):

        self.declare_and_update_parameters()

        # initially all valid
        valid_array = np.ones(s_array.shape[0], dtype=bool)

        # info for invalid arrays in visualizer
        invalid_array_info = np.array([""] * s_array.shape[0], dtype='<U20')

        # Initialize return variables with safe defaults (in case all trajectories fail early checks)
        ax_tilde = np.zeros_like(s_array)
        ay_tilde = np.zeros_like(s_array)
        g_tilde = np.ones_like(s_array) * 9.81  # Standard gravity
        tire_util_array = np.zeros_like(s_array)
        left_bound = np.array([])
        right_bound = np.array([])

        # Track valid count after each check for failure analysis
        total_trajectories = s_array.shape[0]
        valid_after_curvature = total_trajectories
        valid_after_collision = total_trajectories
        valid_after_friction = total_trajectories

        # Curvature Check
        self.check_curvature(
            valid_array=valid_array,
            Omega_z=Omega_z_vf_array,
            invalid_array_info=invalid_array_info,
        )
        valid_after_curvature = np.sum(valid_array)

        left_bound, right_bound = self.__check_path_collision(
            track_handler=track_handler,
            valid_array=valid_array,
            s_array=s_array,
            n_array=n_array,
            pitlane_mode=pitlane_mode,
            vehicle_params=vehicle_params,
            invalid_array_info=invalid_array_info,
        )
        valid_after_collision = np.sum(valid_array)

        # Friction Check
        ax_tilde, ay_tilde, g_tilde, tire_util_array = self.__check_friction_limits(
            valid_array=valid_array,
            track_handler=track_handler,
            s_array=s_array,
            V_array=V_array,
            n_array=n_array,
            chi_array=chi_array,
            ax_array=ax_vf_array,
            ay_array=ay_vf_array,
            t_array=t_array,
            ggv_mode=ggv_mode,
            gggv_handler=gggv_handler,
            traj_cnt=traj_cnt,
            msgs_logger=msgs_logger,
            pitlane_mode=pitlane_mode,
            invalid_array_info=invalid_array_info,
            postprocessed_raceline=postprocessed_raceline,
        )
        valid_sum_tmp = np.sum(valid_array)
        valid_after_friction = valid_sum_tmp

        if not valid_sum_tmp:
            # Calculate percentage reduction after each check
            curvature_reduction_pct = ((total_trajectories - valid_after_curvature) /
                                       total_trajectories * 100) if total_trajectories > 0 else 0
            collision_reduction_pct = ((valid_after_curvature - valid_after_collision) /
                                       total_trajectories * 100) if total_trajectories > 0 else 0
            friction_reduction_pct = ((valid_after_collision - valid_after_friction) /
                                      total_trajectories * 100) if total_trajectories > 0 else 0

            rospy.logerr(f"[Traj {traj_cnt}] ❌ NO VALID TRAJECTORIES | Total: {total_trajectories} | Curvature: -{curvature_reduction_pct:.1f}% | Collision: -{collision_reduction_pct:.1f}% | Friction: -{friction_reduction_pct:.1f}%")

        return valid_array, ax_tilde, ay_tilde, g_tilde, tire_util_array, invalid_array_info, (left_bound, right_bound)
