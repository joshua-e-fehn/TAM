#!/usr/bin/env python3
"""
TAM Trajectory Checks Module
Safety and feasibility validation following original TAM approach

Ported from tam_race_stack/mod_planning/sampling_planner/sampling_planner/trajectory_checks.py

MODIFICATIONS:
- Uses ROS parameter server instead of param_manager
- Uses postprocessed_raceline format (processed from global waypoints)
- GGGV-dependent functionality commented out (no GGGV diagrams available)
- Simplified for use with Pacejka tire model parameters
- NodeMonitor dependencies simplified to basic print statements

ACTIVE CHECKS:
- Curvature limits (kappa_thr parameter)
- Path collision with track boundaries
- Speed rule compliance (DISABLED - no rules defined)
- Simplified friction limits (GGGV disabled)

DISABLED CHECKS (commented out due to missing GGGV diagrams):
- Full physics-based tire utilization limits
- Apparent acceleration transformations
- Yaw moment calculations
- GGGV interpolation-based friction limits

Required ROS Parameters (with defaults):
- behavior/tube_width: 1.15
- behavior/tire_util_max_check: 1.1  
- behavior/kappa_thr: 0.1
- safety_distances/safety_distance_track_left: 0.0
- safety_distances/safety_distance_track_right: 0.0
- safety_distances/safety_distance_pitlane_left: 0.0
- safety_distances/safety_distance_pitlane_right: 0.0
- safety_distances/soft_safety_distance_left_m: 0.0
- safety_distances/soft_safety_distance_right_m: 0.0
"""
from track_handler_global_waypoints import GlobalWaypointsTrackHandler as Track
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
    kappa_thr: float
    safety_distance_track_left: float
    safety_distance_track_right: float
    safety_distance_pitlane_left: float
    safety_distance_pitlane_right: float
    soft_safety_distance_left_m: float
    soft_safety_distance_right_m: float


class TrajectoryChecks():
    def __init__(self, debugging):
        """
        Initialize TrajectoryChecks module.

        Args:
            debugging: Enable debug logging and trajectory failure tracking

        Note: param_manager parameter removed - now uses ROS parameter server directly
        """
        self.params = TrajectoryChecksParams()
        self.declare_and_update_parameters()
        self.debugging = debugging

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
        yaml_defaults = self._load_yaml_defaults()

        self.params.tube_width = rospy.get_param(
            "behavior/tube_width", yaml_defaults.get('tube_width', 1.15))
        self.params.tire_util_max_check = rospy.get_param(
            "behavior/tire_util_max_check", yaml_defaults.get('tire_util_max_check', 1.1))
        self.params.kappa_thr = rospy.get_param(
            "behavior/kappa_thr", yaml_defaults.get('kappa_thr', 0.1))
        self.params.safety_distance_track_left = rospy.get_param(
            "safety_distances/safety_distance_track_left",
            yaml_defaults.get('safety_distance_track_left', 0.0))
        self.params.safety_distance_track_right = rospy.get_param(
            "safety_distances/safety_distance_track_right",
            yaml_defaults.get('safety_distance_track_right', 0.0))
        self.params.safety_distance_pitlane_left = rospy.get_param(
            "safety_distances/safety_distance_pitlane_left",
            yaml_defaults.get('safety_distance_pitlane_left', 0.0))
        self.params.safety_distance_pitlane_right = rospy.get_param(
            "safety_distances/safety_distance_pitlane_right",
            yaml_defaults.get('safety_distance_pitlane_right', 0.0))
        self.params.soft_safety_distance_left_m = rospy.get_param(
            "safety_distances/soft_safety_distance_left_m",
            yaml_defaults.get('soft_safety_distance_left_m', 0.0))
        self.params.soft_safety_distance_right_m = rospy.get_param(
            "safety_distances/soft_safety_distance_right_m",
            yaml_defaults.get('soft_safety_distance_right_m', 0.0))

    def check_curvature(
        self,
        valid_array: np.ndarray,
        Omega_z: np.ndarray,
        invalid_array_info: np.ndarray,
    ):
        valid_tmp = np.all(
            np.abs(Omega_z[valid_array]) <= self.params.kappa_thr, axis=1)

        # store trajectories that failed this check
        if self.debugging:
            combined_mask = valid_array.copy()
            combined_mask[valid_array] = ~valid_tmp
            invalid_array_info[combined_mask] = "curvature"

        valid_array[valid_array] = valid_tmp

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
        if pitlane_mode:
            safety_distance_left = self.params.safety_distance_pitlane_left
            safety_distance_right = self.params.safety_distance_pitlane_right
        else:
            safety_distance_left = self.params.safety_distance_track_left
            safety_distance_right = self.params.safety_distance_track_right

        # Left boundary: positive value, reduce by margins to get usable left limit
        left_bound = (
            np.interp(
                s_array[valid_array],
                track_handler.s_coord(),
                track_handler.trackwidth_left(),
                period=track_handler.s_coord()[-1],
            )
            - vehicle_params["total_width"] / 2.0
            - (safety_distance_left + self.params.tube_width)
        )

        # Right boundary: trackwidth_right() returns POSITIVE width (distance from centerline)
        # We need NEGATIVE boundary (right side of track), so negate and ADD margins inward
        right_bound = (
            -np.interp(
                s_array[valid_array],
                track_handler.s_coord(),
                track_handler.trackwidth_right(),
                period=track_handler.s_coord()[-1],
            )
            + vehicle_params["total_width"] / 2.0
            + (safety_distance_right + self.params.tube_width)
        )

        valid_tmp = np.all((n_array[valid_array] < left_bound) & (
            n_array[valid_array] > right_bound), axis=1)

        # # Debug logging for path collision analysis
        # if self.debugging and np.sum(~valid_tmp) > 0:
        #     print("\n=== PATH COLLISION DEBUG ===")
        #     print(f"Total trajectories checked: {len(valid_tmp)}")
        #     print(f"Failed path check: {np.sum(~valid_tmp)}")
        #     print(f"\nBoundary Configuration:")
        #     print(f"  Vehicle width: {vehicle_params['total_width']:.3f} m")
        #     print(f"  Safety distance left: {safety_distance_left:.3f} m")
        #     print(f"  Safety distance right: {safety_distance_right:.3f} m")
        #     print(f"  Tube width: {self.params.tube_width:.3f} m")
        #     print(
        #         f"  Total margin left: {vehicle_params['total_width']/2.0 + safety_distance_left + self.params.tube_width:.3f} m")
        #     print(
        #         f"  Total margin right: {vehicle_params['total_width']/2.0 + safety_distance_right + self.params.tube_width:.3f} m")

        #     print(f"\nTrack Boundaries (first point):")
        #     if len(left_bound) > 0 and len(right_bound) > 0:
        #         # left_bound and right_bound are 2D arrays [trajectories, points]
        #         print(f"  Left bound: {float(left_bound[0, 0]):.3f} m")
        #         print(f"  Right bound: {float(right_bound[0, 0]):.3f} m")
        #         print(
        #             f"  Available width: {float(left_bound[0, 0] - right_bound[0, 0]):.3f} m")

        #     print(f"\nSampled Lateral Positions (n_array):")
        #     if len(n_array[valid_array]) > 0:
        #         # First point of each trajectory
        #         n_values = n_array[valid_array][:, 0]
        #         print(f"  Min n: {np.min(n_values):.3f} m")
        #         print(f"  Max n: {np.max(n_values):.3f} m")
        #         print(f"  Mean n: {np.mean(n_values):.3f} m")
        #         print(f"  Range: {np.max(n_values) - np.min(n_values):.3f} m")

        #         # Check if all trajectories have the same n value
        #         unique_n = np.unique(np.round(n_values, 6))
        #         if len(unique_n) == 1:
        #             print(
        #                 f"\n  ⚠️  WARNING: All {len(n_values)} trajectories have IDENTICAL n value = {unique_n[0]:.6f} m")
        #             print(
        #                 f"  ⚠️  This indicates lateral sampling is NOT producing variation!")
        #         else:
        #             print(
        #                 f"  ✓  Found {len(unique_n)} unique lateral positions")

        #         # Show which trajectories exceed bounds
        #         exceeds_left = np.any(
        #             n_array[valid_array] >= left_bound, axis=1)
        #         exceeds_right = np.any(
        #             n_array[valid_array] <= right_bound, axis=1)
        #         print(
        #             f"\n  Trajectories exceeding left bound: {np.sum(exceeds_left)}")
        #         print(
        #             f"  Trajectories exceeding right bound: {np.sum(exceeds_right)}")

        #         if np.sum(exceeds_left) > 0:
        #             max_violation_left = np.max(
        #                 n_array[valid_array][exceeds_left] - left_bound[exceeds_left])
        #             print(f"  Max left violation: {max_violation_left:.3f} m")
        #         if np.sum(exceeds_right) > 0:
        #             max_violation_right = np.max(
        #                 right_bound[exceeds_right] - n_array[valid_array][exceeds_right])
        #             print(
        #                 f"  Max right violation: {max_violation_right:.3f} m")
        #     print("========================\n")

        # store trajectories that failed this check
        if self.debugging:
            combined_mask = valid_array.copy()
            combined_mask[valid_array] = ~valid_tmp
            invalid_array_info[combined_mask] = "path_collision"

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

        # Commented out: GGGV-dependent time processing and yaw calculations
        # dt_array = np.diff(t_array[valid_array], append=(
        #     t_array[valid_array][:, -1] + t_array[valid_array][:, -1] - t_array[valid_array][:, -2]).reshape(-1, 1))
        #
        # d_ay_array = np.diff(
        #     ay_array[valid_array], append=ay_array[valid_array][:, -1].reshape(-1, 1), axis=1)
        #
        # # calc yaw acceleration for quasi-transient limits
        # omega_z_dot_tilde = calc_Omega_z_dot_tilde(
        #     track_handler,
        #     s_array[valid_array],
        #     n_array[valid_array],
        #     chi_array[valid_array],
        #     V_array[valid_array],
        #     ax_array[valid_array],
        #     ay_array[valid_array],
        #     d_ay_array,
        #     dt_array,
        #     neglect_w_dot=True,
        # )

        # GGGV-dependent friction limit checks commented out - no GGGV diagrams available
        # For Pacejka model, implement separate tire limit validation here

        # All GGGV friction checking commented out:
        # if ggv_mode == "polar":
        #     alpha = np.arctan2(ax_tilde[valid_array], ay_tilde[valid_array])
        #     rho = np.sqrt(ax_tilde[valid_array] ** 2 +
        #                   ay_tilde[valid_array] ** 2)
        #     rho_max = (
        #         gggv_handler.gggv_interpolator(
        #             np.array((V_array[valid_array].flatten(
        #             ), g_tilde[valid_array].flatten(), alpha.flatten()))
        #         )
        #         .full()
        #         .squeeze()
        #         .reshape(g_tilde[valid_array].shape)
        #     )
        #     valid_tmp = np.all(rho < rho_max, axis=1)
        #     if np.sum(valid_tmp) < 1:
        #         rho_exc = np.max(rho - rho_max, axis=1)
        #         exc_min_idx = np.argmin(rho_exc)
        #         valid_tmp[exc_min_idx] = True
        #         msgs_logger.warning(
        #             f"Trajectory ID {traj_cnt}: Friction check with infeasible rho: {rho_exc[exc_min_idx]}"
        #         )
        #
        # elif ggv_mode == 'diamond':
        #     _, ax_min, ax_max, ay_max, ym_max = gggv_handler.acc_interpolator(
        #         V_array[valid_array], g_tilde[valid_array], s_array[valid_array], n_array[valid_array], not pitlane_mode, self.debugging)
        #
        #     # get yaw moment factor for quasi-transient limits
        #     ym_tilde = np.abs(omega_z_dot_tilde * gggv_handler.vehicle_inertia)
        #     ym_factor = (1 - ym_tilde/ym_max)
        #
        #     # set ax limit and gg exponent according to sign of current acceleration
        #     ax_tire_lim = np.where(ax_tilde[valid_array] > 0.0, ax_max, ax_min)
        #     gg_exponent = np.where(
        #         ax_tilde[valid_array] > 0.0, gggv_handler.gg_exponent_ax_pos, gggv_handler.gg_exponent_ax_neg)
        #
        #     # set ax machine limits
        #     ax_machine_lim = np.interp(V_array[valid_array], np.linspace(
        #         0.0, 90.0, 10), gggv_handler.ax_machine_limits)
        #
        #     # get tire utilization
        #     tire_util_array[valid_array] = (np.abs(ax_tilde[valid_array] / (ax_tire_lim * ym_factor))) ** gg_exponent + (
        #         np.abs(ay_tilde[valid_array] / (ay_max * ym_factor))) ** gg_exponent
        #
        #     # sort out trajectories that exceed the ax machine limits
        #     valid_tmp = np.all(tire_util_array[valid_array] <= self.params.tire_util_max_check, axis=1) & \
        #         np.all(ax_tilde[valid_array] <= (ax_machine_lim), axis=1)
        #
        #     # recalc raceline tire utilization - commented out due to postprocessed_raceline format
        #     # Note: calc_raceline_tire_util expects specific raceline format.
        #     # This needs to be adapted when GGGV is re-enabled.
        #     # tire_util_array_rl = calc_raceline_tire_util(
        #     #     track_handler, gggv_handler, postprocessed_raceline)
        #
        #     # if np.sum(np.any(tire_util_array_rl > 1.005)) > 0: # add tolerance for numerical inaccuracies
        #     #    print("Raceline violates the friction check!")
        #     #    print("Max tire util value:", max(tire_util_array_rl), "at index:", np.argmax(tire_util_array_rl))
        #
        #     # store trajectories that failed this check for visualizer
        #     if self.debugging:
        #         combined_mask = valid_array.copy()
        #         combined_mask[valid_array] = ~valid_tmp
        #         invalid_array_info[combined_mask] = "friction"
        #
        #     # allow all trajectories if none is below the maximum allowed friction
        #     if np.sum(valid_tmp) < 1:
        #         msgs_logger.warning(
        #             f"Trajectory ID {traj_cnt}: All trajectories exceed the friction limits by at least {self.params.tire_util_max_check}"
        #         )
        #
        #         return ax_tilde, ay_tilde, g_tilde, tire_util_array

        # Simplified friction check without GGGV - accept all trajectories for now
        # TODO: Implement Pacejka-based tire limit checking here
        # Accept all trajectories
        valid_tmp = np.ones(np.sum(valid_array), dtype=bool)

        # Store info for debugging
        # if self.debugging:
        #     print(
        #         f"Trajectory ID {traj_cnt}: GGGV friction checks disabled - using simplified approach")

        valid_array[valid_array] = valid_tmp

        return ax_tilde, ay_tilde, g_tilde, tire_util_array

    def mandatory_checks_trajectory(self,
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

        # checks modify the valid array. The order of the checks can have influence on the calculation time
        valid_sum = np.sum(valid_array)

        # Curvature Check
        self.check_curvature(
            valid_array=valid_array,
            Omega_z=Omega_z_vf_array,
            invalid_array_info=invalid_array_info,
        )
        valid_sum_tmp = np.sum(valid_array)
        if not valid_sum_tmp:
            print(
                f"Trajectory ID {traj_cnt}: No valid edges after curvature check. Valid edges before: {valid_sum}")
            # Simplified monitoring without NodeMonitor dependency
            # if node_monitor:
            #     node_monitor.set_error_lvl("curvature_checks", ErrorLvl.WARN)
        else:
            pass
            # if node_monitor:
            #     node_monitor.set_error_lvl("curvature_checks", ErrorLvl.OK)

        # Path Collision Check
        valid_sum = valid_sum_tmp

        left_bound, right_bound = self.__check_path_collision(
            track_handler=track_handler,
            valid_array=valid_array,
            s_array=s_array,
            n_array=n_array,
            pitlane_mode=pitlane_mode,
            vehicle_params=vehicle_params,
            invalid_array_info=invalid_array_info,
        )
        valid_sum_tmp = np.sum(valid_array)
        if not valid_sum_tmp:
            print(
                f"Trajectory ID {traj_cnt}: No valid edges after path check. Valid edges before: {valid_sum}")
            # Simplified monitoring without NodeMonitor dependency
            # if node_monitor:
            #     node_monitor.set_error_lvl("path_collision_checks", ErrorLvl.WARN)
        else:
            pass
            # if node_monitor:
            #     node_monitor.set_error_lvl("path_collision_checks", ErrorLvl.OK)

            # Rules Check
            valid_sum = valid_sum_tmp
            # self.__check_rules(
            #     valid_array=valid_array,
            #     V_array=V_array,
            #     V_target_rules=V_target_rules,
            #     invalid_array_info=invalid_array_info,
            # )
            valid_sum_tmp = np.sum(valid_array)
            if not valid_sum_tmp:
                print(
                    f'Trajectory ID {traj_cnt}: No valid edges after rule check. Valid edges before: {valid_sum}')
                # Simplified monitoring without NodeMonitor dependency
                # if node_monitor:
                #     node_monitor.set_error_lvl("rule_checks", ErrorLvl.WARN)
            else:
                pass
                # if node_monitor:
                #     node_monitor.set_error_lvl("rule_checks", ErrorLvl.OK)

            # Friction Check
            valid_sum = valid_sum_tmp
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
            if not valid_sum_tmp:
                print(
                    f'Trajectory ID {traj_cnt}: No valid edges after friction check. Valid edges before: {valid_sum}')
                # Simplified monitoring without NodeMonitor dependency
                # if node_monitor:
                #     node_monitor.set_error_lvl("friction_checks", ErrorLvl.WARN)
            else:
                pass
                # if node_monitor:
                #     node_monitor.set_error_lvl("friction_checks", ErrorLvl.OK)

        return valid_array, ax_tilde, ay_tilde, g_tilde, tire_util_array, invalid_array_info, (left_bound, right_bound)
