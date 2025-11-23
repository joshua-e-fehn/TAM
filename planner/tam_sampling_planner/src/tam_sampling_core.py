import os
# import sys
import traceback
import yaml
import copy
from scipy.spatial.transform import Rotation as R

from lateral_sampling import LateralSampling
from longitudinal_sampling import LongitudinalSampling
from coordinate_transformation import CoordinateTransformation
from trajectory_checks import TrajectoryChecks
from calculation_costs import CalculationCosts
from trajectory import Trajectory

import numpy as np
from simple_helper_utils import interpolate_with_period, unwrap_periodic_coordinates, slice_trajectory_dict
from typing import Tuple
import rospy

from track_handler_global_waypoints import GlobalWaypointsTrackHandler


class LocalSamplingPlanner:

    def __init__(
        self,
        node_monitor=False,
        load_from_params: bool = True,
        debugging: bool = False
    ):

        # Load parameters from ROS parameter server
        self.initialized_params = False
        self.declare_and_update_parameters()

        self.longitudinal_sampling = LongitudinalSampling(debugging=debugging)
        self.lateral_sampling = LateralSampling(debugging=debugging)
        self.coordinate_transformation = CoordinateTransformation(
            use_f1tenth_mode=True)
        self.trajectory_checks = TrajectoryChecks(debugging=debugging)
        self.calculation_costs = CalculationCosts(debugging=debugging)
        self.trajectory = Trajectory(debugging=debugging)

        self.debugging = debugging
        self.logging = self.logging_sp

        self.status_dict = {
            "startup": 10,
            "handshake": 20,
            "handshake_finished": 21,
            "idle": 22,
            "stillstand": 30,
            "driving": 31,
            "stopping": 40,
            "soft_emergency": 50,
        }

        self.vehicle_params = {
            "ax_machine_limits": self.ax_machine_limit_overwrites if self.overwrite_vehicle_params else [8.0, 7.0, 7.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0],
            "ax_max_scale": self.ax_max_scale if self.overwrite_vehicle_params else 1.0,
            "ax_min_scale": self.ax_min_scale if self.overwrite_vehicle_params else 1.0,
            "ay_scale": self.ay_scale if self.overwrite_vehicle_params else 1.0,
            "gg_exponent_ax_pos": self.gg_exponent_ax_pos if self.overwrite_vehicle_params else 1.0,
            "gg_exponent_ax_neg": self.gg_exponent_ax_neg if self.overwrite_vehicle_params else 1.0,
            "Iz": self.Iz if self.overwrite_vehicle_params else 1.0,
            "total_width": self.total_width if self.overwrite_vehicle_params else 0.20,
            "total_length": self.total_length if self.overwrite_vehicle_params else 0.50,
        }

        self.status = self.status_dict["stillstand"]
        self.performance_trajectory = {}
        self.emergency_trajectory = {}
        self.traj_cnt = 0
        self.pitlane_mode = False

        self.overtaking_allowed = False
        self.following_vel = np.inf

        # for detection of emergency brake maneuvers
        self.emergency_brake = False
        self.vehicle_ahead = False

        # Store raw sampled trajectories for visualization (populated by calc_trajectory)
        self.last_s_array = None
        self.last_n_array = None
        self.last_valid_array = None

        # Initialize track handler (will be updated when waypoints received)
        self.track_handler = GlobalWaypointsTrackHandler()

        # Initialize missing class members required by trajectory methods
        # NOTE: These are placeholder initializations - actual implementations needed
        # GGGV diagram handler (not used in F1TENTH mode)
        self.gggv_handler = None
        # Message logger for debugging (not used in F1TENTH)
        self.msgs_logger = None
        # Lap counter for endurance racing (not used in F1TENTH sprint racing)
        self.lap_counter = None
        self.node_monitor = node_monitor  # Node monitoring object

        rospy.loginfo("Sampling planner initialized.")

    def postprocess_raceline(self, raw_raceline: dict, s_start: float, horizon: float, track_handler) -> dict:
        """
        Extract distance-bounded raceline segment for local planning.

        Purpose: Get raceline waypoints from current position forward for 2×trajectory_length meters,
                 handling track wraparound (e.g., s: 74, 75, 76, 0, 1, 2...).

        Args:
            raw_raceline: Global waypoints with 'wpnts' list or legacy list format
            s_start: Current arc length position on track [m]
            horizon: Planning time horizon [s] (unused, kept for API compatibility)
            track_handler: Track handler with get_track_length()

        Returns:
            Dictionary with extracted segment containing s_post, x_post, y_post, v_post, etc.
        """
        # Extract waypoints from input format
        if isinstance(raw_raceline, dict) and 'wpnts' in raw_raceline:
            waypoints = raw_raceline['wpnts']
        elif isinstance(raw_raceline, list):
            waypoints = raw_raceline
        else:
            rospy.logwarn("Empty raceline in postprocess_raceline")
            return self._empty_raceline_dict()

        # Extract arrays from waypoints (handle both ROS messages and dicts)
        def get_attr(wp, attr, default=0.0):
            if hasattr(wp, attr):
                return getattr(wp, attr)
            elif isinstance(wp, dict):
                return wp.get(attr, default)
            else:
                return default

        s_rl = np.array([get_attr(wp, 's_m') for wp in waypoints])
        x_rl = np.array([get_attr(wp, 'x_m') for wp in waypoints])
        y_rl = np.array([get_attr(wp, 'y_m') for wp in waypoints])
        v_rl = np.array([get_attr(wp, 'vx_mps') for wp in waypoints])
        kappa_rl = np.array([get_attr(wp, 'kappa_radpm') for wp in waypoints])
        # Handle track wraparound - create extended segment
        n_rl = np.array([get_attr(wp, 'd_m', 0.0) for wp in waypoints])
        track_length = track_handler.get_track_length()
        s_start_norm = s_start % track_length
        idx_start = np.searchsorted(s_rl, s_start_norm, side='left')
        if idx_start >= len(s_rl):
            idx_start = 0

        # Concatenate: [from start_idx to end] + [full track for wraparound]
        # s values naturally wrap: [..., 74, 75, 76, 0, 1, 2, ...]
        s_rl = np.concatenate([s_rl[idx_start:], s_rl])
        x_rl = np.concatenate([x_rl[idx_start:], x_rl])
        y_rl = np.concatenate([y_rl[idx_start:], y_rl])
        v_rl = np.concatenate([v_rl[idx_start:], v_rl])
        kappa_rl = np.concatenate([kappa_rl[idx_start:], kappa_rl])
        n_rl = np.concatenate([n_rl[idx_start:], n_rl])

        # Trim to distance limit (2×trajectory_length from current position)
        s_limit = 2.0 * self.trajectory_length
        s_distances = np.zeros(len(s_rl))
        for i in range(1, len(s_rl)):
            ds = s_rl[i] - s_rl[i-1]
            if ds < -track_length / 2:  # Handle wraparound
                ds += track_length
            s_distances[i] = s_distances[i-1] + ds

        # Find index where distance exceeds limit
        idx_limit = np.searchsorted(s_distances, s_limit, side='right')
        if idx_limit < len(s_rl):
            s_rl = s_rl[:idx_limit]
            x_rl = x_rl[:idx_limit]
            y_rl = y_rl[:idx_limit]
            v_rl = v_rl[:idx_limit]
            kappa_rl = kappa_rl[:idx_limit]
            n_rl = n_rl[:idx_limit]

        # Calculate derived quantities
        chi_rl = np.arctan2(np.gradient(y_rl), np.gradient(x_rl))

        # Calculate time stamps by integrating t = ∫(ds/v)
        t_rl = np.zeros_like(s_rl)
        if len(s_rl) > 1:
            for i in range(1, len(s_rl)):
                ds = s_rl[i] - s_rl[i-1]
                if ds < -track_length / 2:  # Handle wraparound
                    ds += track_length
                v_avg = 0.5 * (v_rl[i-1] + v_rl[i])
                if v_avg > 1e-6:
                    t_rl[i] = t_rl[i-1] + ds / v_avg
                else:
                    t_rl[i] = t_rl[i-1] + ds / 1e-6

        # Calculate longitudinal acceleration from velocity gradient
        # s_ddot = dv/dt = d(s_dot)/dt, approximate using finite differences
        s_ddot_rl = np.zeros_like(v_rl)
        if len(t_rl) > 1:
            dt_arr = np.diff(t_rl)
            dv_arr = np.diff(v_rl)
            # Avoid division by zero
            dt_safe = np.where(dt_arr > 1e-6, dt_arr, 1e-6)
            s_ddot_interior = dv_arr / dt_safe
            # Use forward difference for first point, backward for last
            s_ddot_rl[0] = s_ddot_interior[0] if len(
                s_ddot_interior) > 0 else 0.0
            s_ddot_rl[1:-1] = 0.5 * (s_ddot_interior[:-1] +
                                     s_ddot_interior[1:]) if len(s_ddot_interior) > 1 else 0.0
            s_ddot_rl[-1] = s_ddot_interior[-1] if len(
                s_ddot_interior) > 0 else 0.0

        # Return processed segment
        return {
            's_post': s_rl,
            'x_post': x_rl,
            'y_post': y_rl,
            'n_post': n_rl,
            'v_post': v_rl,
            # Longitudinal velocity (same as v for raceline)
            's_dot_post': v_rl,
            'kappa_post': kappa_rl,
            'chi_post': chi_rl,
            't_post': t_rl,  # Time stamps from velocity integration
            # DEPRECATED: use s_ddot_post (kept for backward compatibility)
            'ax_post': s_ddot_rl,
            'ay_post': v_rl**2 * kappa_rl,  # Centripetal acceleration
            # Longitudinal acceleration (braking/accelerating)
            's_ddot_post': s_ddot_rl,
            # Zero (no lateral motion on raceline, n=const=0)
            'n_dot_post': np.zeros_like(v_rl),
            # Zero (no lateral acceleration in raceline frame)
            'n_ddot_post': np.zeros_like(v_rl)
        }

    def _empty_raceline_dict(self):
        """Return empty raceline dictionary for error cases."""
        empty = np.array([])
        return {
            's_post': empty, 'x_post': empty, 'y_post': empty, 'n_post': empty,
            'v_post': empty, 's_dot_post': empty, 'kappa_post': empty,
            'chi_post': empty, 't_post': empty, 'ax_post': empty, 'ay_post': empty,
            's_ddot_post': empty, 'n_dot_post': empty, 'n_ddot_post': empty
        }

    def postprocess_prediction(self, raw_prediction: dict, state_estimate: dict, track_handler,
                               perception_offset_threshold: float, t_offset_state_estimate_to_start: float,
                               following_distance_target: float, following_distance_factor_pos: float,
                               following_distance_factor_neg: float, planning_requests: dict,
                               following_vel: float, traj_cnt: int, vehicle_ahead: bool,
                               node_monitor, msgs_logger, debugging: bool) -> Tuple[dict, float, bool]:
        """
        Postprocess prediction data from tam_prediction_node for obstacle avoidance.

        Processes OpponentTrajectory message from tam_prediction_node which provides
        opponent predicted trajectory as a list of OppWpnt waypoints in Frenet coordinates.

        Args:
            raw_prediction: Raw prediction from tam_prediction_node with 'oppwpnts' list
                           or legacy dict format for backward compatibility
            state_estimate: Current vehicle state with 's', 'n', 'vel_current'
            track_handler: Track handler object
            perception_offset_threshold: Threshold for perception offset
            t_offset_state_estimate_to_start: Time offset from state estimate to planning start
            following_distance_target: Target following distance
            following_distance_factor_pos: Positive following distance factor
            following_distance_factor_neg: Negative following distance factor
            planning_requests: Planning requests dictionary
            following_vel: Current following velocity
            traj_cnt: Trajectory counter
            vehicle_ahead: Flag indicating vehicle ahead
            node_monitor: Node monitor object
            msgs_logger: Message logger
            debugging: Debug flag

        Returns:
            Tuple of (postprocessed_prediction, following_vel, vehicle_ahead)
        """
        # Initialize output
        postprocessed_prediction = {}
        new_following_vel = np.inf
        new_vehicle_ahead = False

        # If no prediction data, return defaults
        if not raw_prediction or len(raw_prediction) == 0:
            return postprocessed_prediction, new_following_vel, new_vehicle_ahead

        # Get track length for wraparound calculations
        track_length = track_handler.s_coord(
        )[-1] if hasattr(track_handler, 's_coord') else 100.0

        # Check if this is tam_prediction_node format (has 'oppwpnts')
        if 'oppwpnts' in raw_prediction:
            # TAM prediction node format - process opponent trajectory waypoints
            oppwpnts = raw_prediction['oppwpnts']

            if len(oppwpnts) == 0:
                return postprocessed_prediction, new_following_vel, new_vehicle_ahead

            # Extract ego position for segment extraction
            ego_s = state_estimate.get('s', 0.0)

            # Extract all waypoint data (already in Frenet coordinates)
            s_all = np.array([wp.s_m if hasattr(wp, 's_m')
                             else wp['s_m'] for wp in oppwpnts])
            n_all = np.array([wp.d_m if hasattr(wp, 'd_m')
                             else wp['d_m'] for wp in oppwpnts])
            x_all = np.array([wp.x_m if hasattr(wp, 'x_m')
                             else wp['x_m'] for wp in oppwpnts])
            y_all = np.array([wp.y_m if hasattr(wp, 'y_m')
                             else wp['y_m'] for wp in oppwpnts])
            vel_s_all = np.array([wp.proj_vs_mps if hasattr(
                wp, 'proj_vs_mps') else wp['proj_vs_mps'] for wp in oppwpnts])
            vel_n_all = np.array([wp.vd_mps if hasattr(
                wp, 'vd_mps') else wp.get('vd_mps', 0.0) for wp in oppwpnts])

            # Find first waypoint ahead of ego position (not just closest)
            s_diff = s_all - ego_s
            # Adjust for wraparound
            s_diff = np.where(s_diff > track_length / 2.0,
                              s_diff - track_length, s_diff)
            s_diff = np.where(s_diff < -track_length / 2.0,
                              s_diff + track_length, s_diff)

            # Find first waypoint ahead (s_diff >= 0) or closest if all behind
            ahead_mask = s_diff >= 0
            if np.any(ahead_mask):
                # Use first waypoint ahead of ego
                start_idx = np.argmax(ahead_mask)
            else:
                # All waypoints behind - use closest (should rarely happen)
                start_idx = np.argmin(np.abs(s_diff))
                rospy.logwarn_throttle(
                    5.0, f"[Prediction] All opponent waypoints behind ego (s_diff_min={np.min(s_diff):.2f}m)")

            # Extract relevant segment: from start_idx forward
            # Prediction node now provides pre-trimmed trajectory, but double-check limits
            time_horizon = 2.0 * self.horizon  # 2x planning horizon for safety margin
            distance_horizon = 2.0 * self.trajectory_length

            # Find end index based on time/distance (whichever comes first)
            # Calculate accumulated time and distance from start
            t_accumulated = 0.0
            d_accumulated = 0.0
            end_idx = start_idx

            for i in range(start_idx + 1, len(s_all)):
                # Calculate distance increment
                ds = s_all[i] - s_all[i-1]
                if ds < 0:  # Handle wraparound
                    ds += track_length
                d_accumulated += abs(ds)

                # Calculate time increment
                v_avg = 0.5 * (vel_s_all[i-1] + vel_s_all[i])
                if v_avg > 0.1:
                    dt = ds / v_avg
                else:
                    dt = 0.0
                t_accumulated += dt

                # Stop if we exceed either limit
                if t_accumulated > time_horizon or d_accumulated > distance_horizon:
                    end_idx = i + 1
                    break
                end_idx = i

            # Ensure we have at least some points
            if end_idx <= start_idx:
                end_idx = min(start_idx + 10, len(s_all))

            # Extract segment
            s_segment = s_all[start_idx:end_idx]
            n_segment = n_all[start_idx:end_idx]
            x_segment = x_all[start_idx:end_idx]
            y_segment = y_all[start_idx:end_idx]
            vel_s_segment = vel_s_all[start_idx:end_idx]
            vel_n_segment = vel_n_all[start_idx:end_idx]

            N = len(s_segment)

            if N == 0:
                return postprocessed_prediction, new_following_vel, new_vehicle_ahead

            # Calculate time array from distance and velocity
            t_segment = np.zeros(N)
            if N > 1:
                for i in range(1, N):
                    ds = s_segment[i] - s_segment[i-1]
                    if ds < 0:  # Handle wraparound
                        ds += track_length
                    v_avg = 0.5 * (vel_s_segment[i-1] + vel_s_segment[i])
                    if v_avg > 1e-6:
                        t_segment[i] = t_segment[i-1] + ds / v_avg
                    else:
                        t_segment[i] = t_segment[i-1] + ds / 1e-6

            # Calculate global longitudinal distance with wraparound
            dist_at_t0 = s_segment[0] - ego_s
            abs_dist = abs(dist_at_t0)

            if abs_dist > track_length * 0.5:
                # Adjust for wraparound
                sign_val = -1.0 if dist_at_t0 > 0.0 else 1.0
                s_glob_dist = (track_length - abs_dist) * sign_val
            else:
                s_glob_dist = dist_at_t0

            # Calculate total velocity magnitude
            vel_total = np.sqrt(vel_s_segment**2 + vel_n_segment**2)

            # Determine following behavior - check if opponent is ahead and slower
            if s_glob_dist > 0:  # Opponent ahead
                # Check multiple points in trajectory for closest approach within following distance
                for i in range(N):
                    # Calculate distance at this point (with wraparound)
                    s_dist_i = s_segment[i] - ego_s
                    if abs(s_dist_i) > track_length * 0.5:
                        sign_i = -1.0 if s_dist_i > 0.0 else 1.0
                        s_dist_i = (track_length - abs(s_dist_i)) * sign_i

                    # If within following distance and ahead
                    if 0 < s_dist_i < following_distance_target:
                        opponent_vel = vel_total[i]
                        if opponent_vel < new_following_vel:
                            new_following_vel = opponent_vel
                            new_vehicle_ahead = True

            # Structure output in expected format
            postprocessed_prediction['opponent_0'] = {
                's': s_segment,
                'n': n_segment,
                'x': x_segment,
                'y': y_segment,
                't': t_segment,
                'time_w_offset': t_segment,  # Alias for calculation_costs compatibility
                'vel': vel_total,
                'vel_s': vel_s_segment,
                'vel_n': vel_n_segment,
                's_glob_dist': s_glob_dist,
                'valid': True,
                'prediction_type': 'dynamic',
                'time_offset': 0.0  # Time offset from state estimate to prediction start
            }

            # if debugging:
            #     rospy.loginfo_throttle(5.0,
            #                            f"[Traj {traj_cnt}] Processed prediction: {N} waypoints, "
            #                            f"s_glob_dist={s_glob_dist:.2f}m, "
            #                            f"following_vel={new_following_vel:.2f}m/s, "
            #                            f"vehicle_ahead={new_vehicle_ahead}")

        else:
            # Legacy format for backward compatibility
            rospy.logwarn_throttle(
                10.0, "Using legacy prediction format - consider updating to tam_prediction_node format")

            # Get overtaking settings
            overtaking_allowed = planning_requests.get(
                'overtaking_allowed', False)

            # Process each predicted object (old format)
            for obj_id, obj_data in raw_prediction.items():
                # Skip invalid objects
                if not obj_data.get('valid', True):
                    postprocessed_prediction[obj_id] = {
                        'valid': False,
                        'vel': [0.0],
                        's_glob_dist': np.inf,
                        'time_w_offset': [],
                        's': [],
                        'n': []
                    }
                    continue

                proc_obj = copy.deepcopy(obj_data)

                # Keep existing legacy processing logic...
                # (This path is for backward compatibility only)
                postprocessed_prediction[obj_id] = proc_obj

        return postprocessed_prediction, new_following_vel, new_vehicle_ahead

    def match_and_hold_constant(self, performance_trajectory: dict, track_handler,
                                state_estimate: dict, const_trajectory_time: float,
                                s_dot_min: float) -> Tuple[dict, float, float, float, float, float, float, dict, float, float]:
        """
        Match previous trajectory to current state and hold initial portion constant.

        This enables smooth trajectory warm-starting:
        1. Match: Find current vehicle position on previous trajectory
        2. Trim: Remove trajectory points that are in the past
        3. Hold: Keep first const_trajectory_time seconds as control output
        4. Extract: Use end of constant segment as initial conditions for replanning
        5. Return: Constant segment (for execution) + live segment (for replanning)

        Args:
            performance_trajectory: Previous performance trajectory
            track_handler: Track handler object
            state_estimate: Current vehicle state estimate
            const_trajectory_time: Time to hold trajectory constant [s]
            s_dot_min: Minimum longitudinal velocity [m/s]

        Returns:
            Tuple of (aligned_trajectory, s_start, s_dot_start, s_ddot_start, 
                     n_start, n_dot_start, n_ddot_start, const_part_trajectory, t_start, s_loc_start)
        """
        track_length = track_handler.get_track_length()

        # Initialize default return values (used when no previous trajectory exists)
        default_state = {
            's_start': state_estimate.get('s', 0.0),
            's_dot_start': max(s_dot_min, state_estimate.get('vel_current', s_dot_min)),
            's_ddot_start': 0.0,
            'n_start': state_estimate.get('n', 0.0),
            'n_dot_start': 0.0,
            'n_ddot_start': 0.0,
            't_start': 0.0,
            's_loc_start': 0.0
        }

        # Case 1: No previous trajectory - return current state estimates
        if not performance_trajectory or 's' not in performance_trajectory or len(performance_trajectory['s']) == 0:
            return ({}, default_state['s_start'], default_state['s_dot_start'], default_state['s_ddot_start'],
                    default_state['n_start'], default_state['n_dot_start'], default_state['n_ddot_start'],
                    {}, default_state['t_start'], default_state['s_loc_start'])

        # Case 2: Previous trajectory exists - perform matching and splitting
        aligned_traj = copy.deepcopy(performance_trajectory)
        N = len(aligned_traj['s'])

        # Step 1: Unwrap s-coordinates to handle track wraparound
        s_unwrapped = unwrap_periodic_coordinates(
            aligned_traj['s'], track_length)

        # Align unwrapped coordinates to current vehicle position
        if state_estimate.get('s', 0.0) < s_unwrapped[0]:
            s_unwrapped = s_unwrapped - track_length

        # Step 2: Find match point (where vehicle currently is on previous trajectory)
        match_idx = np.searchsorted(
            s_unwrapped, state_estimate.get('s', 0.0), side='right') - 1
        match_idx = max(match_idx, 0)  # Clamp to valid range

        # Step 3: Trim trajectory - remove everything before match point
        if match_idx > 0:
            t_match = aligned_traj.get('t', np.zeros(N))[match_idx]
            s_loc_match = aligned_traj.get('s_loc', np.zeros(N))[match_idx]

            # Slice all trajectory arrays
            aligned_traj = slice_trajectory_dict(
                aligned_traj, start_idx=match_idx)

            # Re-zero time and s_local (match point becomes t=0)
            if 't' in aligned_traj and len(aligned_traj['t']) > 0:
                aligned_traj['t'] = aligned_traj['t'] - t_match
            if 's_loc' in aligned_traj and len(aligned_traj['s_loc']) > 0:
                aligned_traj['s_loc'] = np.mod(
                    aligned_traj['s_loc'] - s_loc_match, track_length)

        # Step 4: Split into constant segment (held) and live segment (for replanning)
        new_N = len(aligned_traj.get('s', []))
        const_end_idx = 0

        if new_N > 0 and 't' in aligned_traj:
            # Find where constant segment ends
            const_end_idx = np.searchsorted(
                aligned_traj['t'], const_trajectory_time, side='right')
            const_end_idx = min(max(const_end_idx - 1, 0), new_N - 1)

        # Step 5: Extract constant part (this will be executed by controller)
        const_part_traj = {}
        if const_end_idx > 0:
            const_part_traj = slice_trajectory_dict(
                aligned_traj, end_idx=const_end_idx)

        # Step 6: Extract initial conditions from trajectory
        if new_N > 0:
            # When const_trajectory_time == 0, const_end_idx is 0, so extract from first point
            # When const_trajectory_time > 0, extract from end of constant segment
            extract_idx = max(const_end_idx, 0)

            # Extract state from trajectory
            s_start = aligned_traj['s'][extract_idx]
            s_dot_start = max(s_dot_min, aligned_traj.get(
                's_dot', [s_dot_min] * new_N)[extract_idx])
            s_ddot_start = aligned_traj.get(
                's_ddot', [0.0] * new_N)[extract_idx]
            n_start = aligned_traj.get('n', [0.0] * new_N)[extract_idx]
            n_dot_start = aligned_traj.get(
                'n_dot', [0.0] * new_N)[extract_idx]
            n_ddot_start = aligned_traj.get(
                'n_ddot', [0.0] * new_N)[extract_idx]
            t_start = aligned_traj.get('t', [0.0] * new_N)[extract_idx]
            s_loc_start = aligned_traj.get(
                's_loc', [0.0] * new_N)[extract_idx]
        else:
            # No warm-starting: Use current state estimate
            s_start = default_state['s_start']
            s_dot_start = default_state['s_dot_start']
            s_ddot_start = default_state['s_ddot_start']
            n_start = default_state['n_start']
            n_dot_start = default_state['n_dot_start']
            n_ddot_start = default_state['n_ddot_start']
            t_start = default_state['t_start']
            s_loc_start = default_state['s_loc_start']

        # Step 7: Remove constant part from aligned trajectory (keep only "live" part for replanning)
        if const_end_idx > 0 and const_end_idx < new_N:
            aligned_traj = slice_trajectory_dict(
                aligned_traj, start_idx=const_end_idx)

            # Re-zero time and s_local so live part starts at 0
            if len(aligned_traj.get('t', [])) > 0:
                aligned_traj['t'] = aligned_traj['t'] - aligned_traj['t'][0]
            if len(aligned_traj.get('s_loc', [])) > 0:
                s0 = aligned_traj['s_loc'][0]
                aligned_traj['s_loc'] = np.mod(
                    aligned_traj['s_loc'] - s0, track_length)

        return aligned_traj, s_start, s_dot_start, s_ddot_start, n_start, n_dot_start, n_ddot_start, const_part_traj, t_start, s_loc_start

    def _load_yaml_defaults(self):
        """Load default parameter values from YAML configuration file"""
        try:
            # Get the path to the config file
            import rospkg
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('tam_sampling_planner')
            config_file = os.path.join(
                pkg_path, 'config', 'tam_sampling_params.yaml')

            # Load YAML file
            with open(config_file, 'r') as f:
                yaml_config = yaml.safe_load(f)

            rospy.loginfo(f"Loaded default parameters from: {config_file}")
            return yaml_config if yaml_config is not None else {}

        except Exception as e:
            rospy.logwarn(
                f"Could not load YAML defaults: {e}. Using hardcoded defaults.")
            return {}

    def declare_and_update_parameters(self):
        """Load required parameters from ROS parameter server with defaults from YAML"""

        if not self.initialized_params:
            # Load default values from YAML configuration file
            yaml_defaults = self._load_yaml_defaults()

            # Core behavior parameters
            self.hybrid_long_sampling = yaml_defaults.get(
                'hybrid_long_sampling', rospy.get_param('hybrid_long_sampling', True))
            rospy.set_param('hybrid_long_sampling', self.hybrid_long_sampling)
            self.sampling_mode = yaml_defaults.get(
                'sampling_mode', rospy.get_param('sampling_mode', "spatial"))
            rospy.set_param('sampling_mode', self.sampling_mode)
            self.horizon = yaml_defaults.get(
                'planning_horizon', rospy.get_param('horizon', 4.0))
            rospy.set_param('horizon', self.horizon)
            self.trajectory_length = yaml_defaults.get(
                'trajectory_length', rospy.get_param('trajectory_length', 10.0))
            rospy.set_param('trajectory_length', self.trajectory_length)
            self.const_trajectory_time = yaml_defaults.get(
                'const_trajectory_time', rospy.get_param('const_trajectory_time', 0.3))
            rospy.set_param('const_trajectory_time',
                            self.const_trajectory_time)
            self.min_trajectory_length = yaml_defaults.get(
                'min_trajectory_length', rospy.get_param('min_trajectory_length', 8.0))
            rospy.set_param('min_trajectory_length',
                            self.min_trajectory_length)
            self.s_dot_min = yaml_defaults.get(
                's_dot_end_min', rospy.get_param('s_dot_min', 1.0))
            rospy.set_param('s_dot_min', self.s_dot_min)
            self.V_thr_stillstand = yaml_defaults.get(
                'V_thr_stillstand', rospy.get_param('V_thr_stillstand', 2.0))
            rospy.set_param('V_thr_stillstand', self.V_thr_stillstand)
            self.perception_offset_threshold = yaml_defaults.get(
                'perception_offset_threshold', rospy.get_param('perception_offset_threshold', 2.0))
            rospy.set_param('perception_offset_threshold',
                            self.perception_offset_threshold)
            self.relative_long_sampling_threshold = yaml_defaults.get(
                'relative_long_sampling_threshold', rospy.get_param('relative_long_sampling_threshold', 0.7))
            rospy.set_param('relative_long_sampling_threshold',
                            self.relative_long_sampling_threshold)
            self.following_distance_factor_pos = yaml_defaults.get(
                'following_distance_factor_pos', rospy.get_param('following_distance_factor_pos', 100.0))
            rospy.set_param('following_distance_factor_pos',
                            self.following_distance_factor_pos)
            self.following_distance_factor_neg = yaml_defaults.get(
                'following_distance_factor_neg', rospy.get_param('following_distance_factor_neg', 40.0))
            rospy.set_param('following_distance_factor_neg',
                            self.following_distance_factor_neg)

            # Vehicle and track configuration
            self.vehicle_name = yaml_defaults.get(
                'vehicle_name', rospy.get_param('vehicle_name', 'f1tenth'))
            rospy.set_param('vehicle_name', self.vehicle_name)
            self.track_name = yaml_defaults.get(
                'track_name', rospy.get_param('track_name', 'levine'))
            rospy.set_param('track_name', self.track_name)
            self.pitlane_name = yaml_defaults.get(
                'pitlane_name', rospy.get_param('pitlane_name', 'levine_pitlane'))
            rospy.set_param('pitlane_name', self.pitlane_name)
            self.gggv_mode = yaml_defaults.get(
                'gggv_mode', rospy.get_param('gggv_mode', 'diamond'))
            rospy.set_param('gggv_mode', self.gggv_mode)
            self.gg_scale = yaml_defaults.get(
                'gg_scale', rospy.get_param('gg_scale', 0.9))
            rospy.set_param('gg_scale', self.gg_scale)

            # Debug and Logging Parameters
            self.logging_sp = yaml_defaults.get(
                'logging_sp', rospy.get_param('logging_sp', False))
            rospy.set_param('logging_sp', self.logging_sp)
            self.log_console_level = yaml_defaults.get(
                'log_console_level', rospy.get_param('log_console_level', "INFO"))
            rospy.set_param('log_console_level', self.log_console_level)
            self.log_file_level = yaml_defaults.get(
                'log_file_level', rospy.get_param('log_file_level', "DEBUG"))
            rospy.set_param('log_file_level', self.log_file_level)

            # Vehicle parameter overwrites
            self.overwrite_vehicle_params = yaml_defaults.get(
                'overwrite_vehicle_params', rospy.get_param('overwrite_vehicle_params', True))
            rospy.set_param('overwrite_vehicle_params',
                            self.overwrite_vehicle_params)
            self.ax_machine_limit_overwrites = yaml_defaults.get(
                'ax_machine_limit_overwrites',
                rospy.get_param('ax_machine_limit_overwrites', [8.0, 7.0, 7.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0]))
            rospy.set_param('ax_machine_limit_overwrites',
                            self.ax_machine_limit_overwrites)
            self.gg_exponent_ax_pos = yaml_defaults.get(
                'gg_exponent_ax_pos', rospy.get_param('gg_exponent_ax_pos', 1.0))
            rospy.set_param('gg_exponent_ax_pos',
                            self.gg_exponent_ax_pos)
            self.gg_exponent_ax_neg = yaml_defaults.get(
                'gg_exponent_ax_neg', rospy.get_param('gg_exponent_ax_neg', 1.0))
            rospy.set_param('gg_exponent_ax_neg',
                            self.gg_exponent_ax_neg)
            self.ax_min_scale = yaml_defaults.get(
                'ax_min_scale', rospy.get_param('ax_min_scale', 1.0))
            rospy.set_param('ax_min_scale', self.ax_min_scale)
            self.ax_max_scale = yaml_defaults.get(
                'ax_max_scale', rospy.get_param('ax_max_scale', 1.0))
            rospy.set_param('ax_max_scale', self.ax_max_scale)
            self.ay_scale = yaml_defaults.get(
                'ay_scale', rospy.get_param('ay_scale', 1.0))
            rospy.set_param('ay_scale', self.ay_scale)
            self.Iz = yaml_defaults.get(
                'Iz', rospy.get_param('Iz', 1.0))
            rospy.set_param('Iz', self.Iz)
            self.total_width = yaml_defaults.get(
                'width', rospy.get_param('width', 0.20))
            rospy.set_param('width', self.total_width)
            self.total_length = yaml_defaults.get(
                'length', rospy.get_param('length', 0.50))
            rospy.set_param('length', self.total_length)
            # F1TENTH NOTE: Forward-backward sampling now available with simplified kinematics (no GGGV required)
            self.add_forward_backward_samples = yaml_defaults.get(
                'forward_backward_velocities', rospy.get_param('add_forward_backward_samples', False))
            rospy.set_param('add_forward_backward_samples',
                            self.add_forward_backward_samples)

            rospy.loginfo(
                "Loaded TAM sampling planner parameters from parameter server")
            if self.add_forward_backward_samples:
                rospy.loginfo(
                    "F1TENTH mode: Forward-backward sampling ENABLED (using fixed acceleration limits)")
            else:
                rospy.loginfo(
                    "F1TENTH mode: Forward-backward sampling DISABLED")

            self.initialized_params = True
        else:
            # Core behavior parameters
            self.hybrid_long_sampling = rospy.get_param(
                'hybrid_long_sampling', self.hybrid_long_sampling)
            self.sampling_mode = rospy.get_param(
                'sampling_mode', self.sampling_mode)
            self.horizon = rospy.get_param(
                'horizon', self.horizon)
            self.trajectory_length = rospy.get_param(
                'trajectory_length', self.trajectory_length)
            self.const_trajectory_time = rospy.get_param(
                'const_trajectory_time', self.const_trajectory_time)
            self.min_trajectory_length = rospy.get_param(
                'min_trajectory_length', self.min_trajectory_length)
            self.s_dot_min = rospy.get_param(
                's_dot_min', self.s_dot_min)
            self.V_thr_stillstand = rospy.get_param(
                'V_thr_stillstand', self.V_thr_stillstand)
            self.perception_offset_threshold = rospy.get_param(
                'perception_offset_threshold', self.perception_offset_threshold)
            self.relative_long_sampling_threshold = rospy.get_param(
                'relative_long_sampling_threshold', self.relative_long_sampling_threshold)
            self.following_distance_factor_pos = rospy.get_param(
                'following_distance_factor_pos', self.following_distance_factor_pos)
            self.following_distance_factor_neg = rospy.get_param(
                'following_distance_factor_neg', self.following_distance_factor_neg)

            # Vehicle and track configuration
            self.vehicle_name = rospy.get_param(
                'vehicle_name', self.vehicle_name)
            self.track_name = rospy.get_param(
                'track_name', self.track_name)
            self.pitlane_name = rospy.get_param(
                'pitlane_name', self.pitlane_name)
            self.gggv_mode = rospy.get_param(
                'gggv_mode', self.gggv_mode)
            self.gg_scale = rospy.get_param(
                'gg_scale', self.gg_scale)

            # Debug and Logging Parameters
            self.logging_sp = rospy.get_param(
                'logging_sp', self.logging_sp)
            self.log_console_level = rospy.get_param(
                'log_console_level', self.log_console_level)
            self.log_file_level = rospy.get_param(
                'log_file_level', self.log_file_level)

            # Vehicle parameter overwrites
            self.overwrite_vehicle_params = rospy.get_param(
                'overwrite_vehicle_params', self.overwrite_vehicle_params)
            self.ax_machine_limit_overwrites = rospy.get_param(
                'ax_machine_limit_overwrites',
                self.ax_machine_limit_overwrites)
            self.gg_exponent_ax_pos = rospy.get_param(
                'gg_exponent_ax_pos', self.gg_exponent_ax_pos)
            self.gg_exponent_ax_neg = rospy.get_param(
                'gg_exponent_ax_neg', self.gg_exponent_ax_neg)
            self.ax_min_scale = rospy.get_param(
                'ax_min_scale', self.ax_min_scale)
            self.ax_max_scale = rospy.get_param(
                'ax_max_scale', self.ax_max_scale)
            self.ay_scale = rospy.get_param(
                'ay_scale', self.ay_scale)
            self.Iz = rospy.get_param(
                'Iz', self.Iz)
            self.total_width = rospy.get_param(
                'width', self.total_width)
            self.total_length = rospy.get_param(
                'length', self.total_length)
            # F1TENTH NOTE: Forward-backward sampling now available with simplified kinematics (no GGGV required)
            self.add_forward_backward_samples = rospy.get_param(
                'add_forward_backward_samples', self.add_forward_backward_samples)

    def calc_trajectory(
            self,
            state_estimate: dict,
            raceline: dict,
            prediction: dict,
            planning_requests: dict,
    ):
        self.declare_and_update_parameters()

        following_distance_target = planning_requests["following_distance"]

        # Initialize return values to safe defaults in case of error
        s_start = state_estimate.get('s', 0.0)
        n_start = state_estimate.get('n', 0.0)
        V_target = planning_requests.get('V_max', 10.0)
        t_start = 0.0
        s_loc_start = 0.0

        try:
            state_estimate_sn = self.track_handler.project_2d_point_on_track_global(
                state_estimate["x_current"], state_estimate["y_current"], state_estimate["z_current"], 6.0)
            state_estimate['s'] = state_estimate_sn[0]
            state_estimate['n'] = state_estimate_sn[1]
            state_estimate['chi'] = self.track_handler.calc_chi_from_2d_heading(
                state_estimate['s'],
                state_estimate["psi_current"],
            )

            # delete previous trajectory to plan from current state estimate in stillstand mode
            if self.status == self.status_dict["stillstand"]:
                self.performance_trajectory.clear()

            # match on previous trajectory (if existing)
            (self.performance_trajectory,
                s_start,
                s_dot_start,
                s_ddot_start,
                n_start,
                n_dot_start,
                n_ddot_start,
                constant_part_trajectory,
                t_start,
                s_loc_start,
             ) = self.match_and_hold_constant(
                performance_trajectory=self.performance_trajectory,
                track_handler=self.track_handler,
                state_estimate=state_estimate,
                const_trajectory_time=self.const_trajectory_time,
                s_dot_min=self.s_dot_min,
            )

            self.handle_state_transitions(
                planning_requests=planning_requests,
                state_estimate=state_estimate,
                V_thr_stillstand=self.V_thr_stillstand,
            )

            postprocessed_prediction, self.following_vel, self.vehicle_ahead = self.postprocess_prediction(
                raw_prediction=prediction,
                state_estimate=state_estimate,
                track_handler=self.track_handler,
                perception_offset_threshold=self.perception_offset_threshold,
                t_offset_state_estimate_to_start=t_start,
                following_distance_target=following_distance_target,
                following_distance_factor_pos=self.following_distance_factor_pos,
                following_distance_factor_neg=self.following_distance_factor_neg,
                planning_requests=planning_requests,
                following_vel=self.following_vel,
                traj_cnt=self.traj_cnt,
                vehicle_ahead=self.vehicle_ahead,
                node_monitor=self.node_monitor,
                msgs_logger=self.msgs_logger,
                debugging=self.debugging
            )

            # set target speed
            # F1TENTH NOTE: No GGGV handler, use planning_requests["V_max"] as absolute max
            V_target = min(
                planning_requests["V_max"], self.following_vel)

            # for ruling out trajectories that exceed the allowed maximum velocity
            V_target_rules = planning_requests["V_max"]

            # postprocess raceline
            postprocessed_raceline = self.postprocess_raceline(
                raw_raceline=raceline, s_start=s_start, horizon=self.horizon, track_handler=self.track_handler,
            )

            # raceline velocity to determine if relative sampling required
            s_dot_raceline_cur = postprocessed_raceline["s_dot_post"][0]

            # enable relative sampling when not in stillstand and when minimum velocity is reached
            enable_relative_sampling = (
                (self.status != self.status_dict['stillstand'] and
                 s_dot_start > self.relative_long_sampling_threshold * s_dot_raceline_cur) or
                self.status != self.status_dict['stopping']
            )

            # generate frenet curves
            s_array, s_dot_array, s_ddot_array, n_array, n_dot_array, n_ddot_array, rel_long_sampling_array, t_array = self.perform_trajectory_sampling(
                track_handler=self.track_handler,
                s_start=s_start,
                s_dot_start=s_dot_start,
                s_ddot_start=s_ddot_start,
                n_start=n_start,
                n_dot_start=n_dot_start,
                n_ddot_start=n_ddot_start,
                V_target=V_target,
                enable_relative_sampling=enable_relative_sampling,
                sampling_mode=self.sampling_mode,
                postprocessed_raceline=postprocessed_raceline,
                hybrid_long_sampling=self.hybrid_long_sampling,
            )

            # transform frenet curves to velocity frame
            V_array, chi_array, ax_vf_array, ay_vf_array, Omega_z_vf_array, kappa_vf_array = self.coordinate_transformation.transform_to_velocity_frame(
                track_handler=self.track_handler,
                s_array=s_array,
                s_dot_array=s_dot_array,
                s_ddot_array=s_ddot_array,
                n_array=n_array,
                n_dot_array=n_dot_array,
                n_ddot_array=n_ddot_array,
                postprocessed_raceline=postprocessed_raceline,
            )

            if self.status == self.status_dict["stillstand"]:
                V_array[:] = 0.0001
                ax_vf_array[:] = 0.0

            # perform all trajectory checks
            # Note: kappa_vf_array contains path curvature (rad/m), not yaw rate
            valid_array, ax_tilde, ay_tilde, g_tilde, tire_util_array, invalid_array_info, track_bound = self.trajectory_checks.mandatory_checks_trajectory(
                Omega_z_vf_array=kappa_vf_array,
                track_handler=self.track_handler,
                s_array=s_array,
                n_array=n_array,
                t_array=t_array,
                V_array=V_array,
                V_target_rules=V_target_rules,
                chi_array=chi_array,
                ax_vf_array=ax_vf_array,
                ay_vf_array=ay_vf_array,
                node_monitor=self.node_monitor,
                msgs_logger=self.msgs_logger,
                traj_cnt=self.traj_cnt,
                pitlane_mode=self.pitlane_mode,
                vehicle_params=self.vehicle_params,
                # F1TENTH: Not used (gggv_handler is None)
                ggv_mode=self.gggv_mode,
                gggv_handler=self.gggv_handler,
                postprocessed_raceline=postprocessed_raceline,
            )

            # Store raw sampled trajectories for visualization (before filtering)
            self.last_s_array = s_array.copy() if s_array is not None else None
            self.last_n_array = n_array.copy() if n_array is not None else None
            self.last_valid_array = valid_array.copy() if valid_array is not None else None

            # Check if any trajectories are valid
            if not np.any(valid_array):
                rospy.logerr(
                    "[Sampling Core] ⚠️ ALL TRAJECTORIES INVALID! Planning will fail.")
                rospy.logerr(
                    f"[Sampling Core] Invalid reasons: {invalid_array_info}")
                rospy.logerr(
                    f"[Sampling Core] Total trajectories sampled: {len(valid_array)}")

            # for debugging
            V_upper = V_target + 1.0

            # choose best trajectory
            cost_array, cost_terms, cost_extensive_array = self.calculation_costs.calc_costs(
                valid_array=valid_array,
                rel_long_sampling_array=rel_long_sampling_array,
                track_handler=self.track_handler,
                s_array=s_array,
                n_array=n_array,
                t_array=t_array,
                V_array=V_array,
                ay_array=ay_tilde,
                Omega_z_array=Omega_z_vf_array,
                raceline=postprocessed_raceline,
                prediction=postprocessed_prediction,
                V_target=V_target,
                planning_requests=planning_requests,
                tire_util_array=tire_util_array,
                pitlane_mode=self.pitlane_mode,
                vehicle_ahead=self.vehicle_ahead,
                emergency_brake=self.emergency_brake,
                vehicle_params=self.vehicle_params,
            )

            sorted_idx = self.calculation_costs.sort_trajectories_by_cost(
                valid_array=valid_array, cost_array=cost_array)

            if np.sum(valid_array):
                # set index of best trajectory
                optimal_idx = np.arange(s_array.shape[0])[
                    valid_array][sorted_idx][0]

                # correct values if in stillstand
                if self.status == self.status_dict["stillstand"]:
                    V_array[:] = 0.0001
                    ax_tilde[:] = 0.0

                # frenet values
                self.performance_trajectory["pitlane_mode"] = self.pitlane_mode
                self.performance_trajectory["emergency"] = False
                self.performance_trajectory["t"] = t_array[optimal_idx] + t_start
                self.performance_trajectory["s"] = s_array[optimal_idx]
                self.performance_trajectory["s_dot"] = s_dot_array[optimal_idx]
                self.performance_trajectory["s_ddot"] = s_ddot_array[optimal_idx]
                self.performance_trajectory["n"] = n_array[optimal_idx]
                self.performance_trajectory["n_dot"] = n_dot_array[optimal_idx]
                self.performance_trajectory["n_ddot"] = n_ddot_array[optimal_idx]
                self.performance_trajectory["V"] = V_array[optimal_idx]
                self.performance_trajectory["chi"] = chi_array[optimal_idx]
                self.performance_trajectory["Omega_z"] = Omega_z_vf_array[optimal_idx]
                self.performance_trajectory["ax"] = ax_vf_array[optimal_idx]
                self.performance_trajectory["ay"] = ay_vf_array[optimal_idx]
                self.performance_trajectory["ax_tilde"] = ax_tilde[optimal_idx]
                self.performance_trajectory["ay_tilde"] = ay_tilde[optimal_idx]
                self.performance_trajectory["g_tilde"] = g_tilde[optimal_idx]
                self.performance_trajectory["tire_util"] = tire_util_array[optimal_idx]

                # Unwrap s_loc (compatible with NumPy < 1.21 which lacks 'period' parameter)
                # TODO: Necessary?
                s_traj = self.performance_trajectory["s"]
                track_length = self.track_handler.get_track_length()
                s_unwrapped = s_traj.copy()
                for i in range(1, len(s_unwrapped)):
                    diff = s_unwrapped[i] - s_unwrapped[i-1]
                    if diff > track_length / 2.0:
                        s_unwrapped[i:] -= track_length
                    elif diff < -track_length / 2.0:
                        s_unwrapped[i:] += track_length
                self.performance_trajectory["s_loc"] = s_unwrapped - \
                    s_unwrapped[0] + s_loc_start

                # throw warning if selected trajectory exceeds 100 % of tire utilization
                # include tolerance and only throw warning when friction violation occurs on first 30 points of trajectory
                last_check_idx = min(30, self.trajectory.params.num_samples)
                if np.max(self.performance_trajectory["tire_util"][:last_check_idx] > 1.01):
                    rospy.logwarn(
                        f'[Sampling Core] Trajectory ID {self.traj_cnt}: Max. tire utilization at index {np.argmax(self.performance_trajectory["tire_util"])} is {np.max(self.performance_trajectory["tire_util"])}')

                # extend trajectory if necessary
                # checks are omitted since this succeeds the planning (time) horizon
                if self.performance_trajectory["s_loc"][-1] < self.min_trajectory_length:
                    # rospy.logerr(
                    #     f"[Sampling Core] Extending trajectory because it is {self.performance_trajectory['s_loc'][-1]} instead of {self.min_trajectory_length} m long.")

                    self.performance_trajectory = self.trajectory.extend_performance_trajectory(
                        trajectory=self.performance_trajectory,
                        track_handler=self.track_handler
                    )

                self.emergency_trajectory = self.trajectory.calc_emergency_trajectory(
                    track_handler=self.track_handler,
                    performance_trajectory=self.performance_trajectory,
                    gggv_handler=self.gggv_handler,
                    pitlane_mode=self.pitlane_mode,
                    vehicle_params=self.vehicle_params,
                    msgs_logger=self.msgs_logger
                )

                if constant_part_trajectory:
                    for trajectory in [self.performance_trajectory, self.emergency_trajectory]:
                        for key in trajectory:
                            if isinstance(trajectory[key], np.ndarray):
                                trajectory[key] = np.concatenate(
                                    (constant_part_trajectory[key], trajectory[key]))

                # F1TENTH: Add Cartesian coordinates for controller compatibility
                # The trajectories already have Frenet (s, n, chi) and velocity frame (V, ax, ay) fields
                # We just need to add global Cartesian fields (x, y, psi, kappa) for Wpnt messages
                for trajectory in [self.performance_trajectory, self.emergency_trajectory]:
                    # Convert Frenet to Cartesian using track handler
                    xyz_array = self.track_handler.sn2cartesian(
                        trajectory["s"], trajectory["n"])
                    trajectory["x"] = xyz_array[:, 0]
                    trajectory["y"] = xyz_array[:, 1]

                    # Calculate global heading (psi) from Frenet heading (chi)
                    trajectory["psi"] = self.track_handler.calc_2d_heading_from_chi(
                        trajectory["s"],
                        trajectory["chi"],
                    )

                    # Get curvature from track (already available in track handler)
                    # Use periodic interpolation for smooth wraparound handling
                    trajectory["kappa"] = interpolate_with_period(
                        trajectory['s'],
                        self.track_handler.s_coord(),
                        self.track_handler.omega_z(),
                        self.track_handler.get_track_length()
                    )

                    # Add trajectory counter for debugging
                    trajectory["traj_cnt"] = self.traj_cnt

                # Note: For F1TENTH, we don't need trajectory_N or complex GGGV calculations
                # The tam_sampling_node.py will convert these trajectories to OTWpntArray directly

        except Exception as e:
            rospy.logerr("="*80)
            rospy.logerr("⚠️⚠️⚠️ CRITICAL ERROR in calc_trajectory ⚠️⚠️⚠️")
            rospy.logerr(f"Exception type: {type(e).__name__}")
            rospy.logerr(f"Exception message: {str(e)}")
            rospy.logerr("Full traceback:")
            rospy.logerr(traceback.format_exc())
            rospy.logerr("="*80)
            raise  # Re-raise to ensure error is not silently swallowed

        # rospy.loginfo(
        #     f"[Sampling Core]🚗 calc_trajectory: Finished trajectory calculation")

        if not self.debugging:
            return self.performance_trajectory, self.emergency_trajectory, s_start, n_start, V_target,
        else:
            return (
                self.performance_trajectory,
                self.emergency_trajectory,
                s_start,
                n_start,
                V_target,
                t_start,
                valid_array,
                invalid_array_info,
                t_array,
                s_array,
                s_dot_array,
                s_ddot_array,
                n_array,
                n_dot_array,
                n_ddot_array,
                V_array,
                chi_array,
                ax_vf_array,
                ay_vf_array,
                Omega_z_vf_array,
                tire_util_array,
                V_upper,
                track_bound,
                postprocessed_raceline,
                postprocessed_prediction,
                cost_array,
                cost_terms,
                cost_extensive_array,
            )

    def handle_state_transitions(
        self,
        planning_requests: dict,
        state_estimate: dict,
        V_thr_stillstand: float,
    ):
        # handle state transitions
        if self.status == self.status_dict["stillstand"]:
            # case stillstand
            if planning_requests["V_max"] > 0.5:
                # if start request and not stop request leave stillstand mode
                rospy.loginfo(
                    "Exiting STILLSTAND mode, entering DRIVING mode.")
                self.status = self.status_dict["driving"]

        elif self.status == self.status_dict["driving"]:
            # case driving
            if planning_requests["V_max"] == 0.0:
                # switch to stopping mode
                rospy.loginfo(
                    "Exiting DRIVING mode, entering STOPPING mode.")
                self.status = self.status_dict["stopping"]

        elif self.status == self.status_dict["stopping"]:
            # case stopping
            if planning_requests["V_max"] > 0.5:
                # leave stopping mode if start request
                rospy.loginfo(
                    "Exiting STOPPING mode, entering DRIVING mode.")
                self.status = self.status_dict["driving"]
            elif state_estimate["vel_current"] < V_thr_stillstand:
                # enter stillstand mode if velocity falls below threshold
                rospy.loginfo(
                    "Exiting STOPPING mode, entering STILLSTAND mode.")
                self.status = self.status_dict["stillstand"]

    def change_track(self, track_handler):
        """
        Update trajectories when track changes.

        Args:
            track_handler: New track handler with updated waypoints
        """
        # Update internal track handler
        self.track_handler = track_handler

        # match old trajectory on new map
        performance_trajectory_sn = self.track_handler.project_2d_point_on_track_global(
            self.performance_trajectory["x"], self.performance_trajectory["y"], self.performance_trajectory["z"], 6.0
        )
        self.performance_trajectory["s"] = performance_trajectory_sn[:, 0]
        self.performance_trajectory["n"] = performance_trajectory_sn[:, 1]
        self.performance_trajectory["chi"] = self.track_handler.calc_chi_from_2d_heading(
            self.performance_trajectory["s"],
            self.performance_trajectory["psi"],
        )
        # recalculate s_dot, s_ddot, n_dot, n_ddot
        self.performance_trajectory["s_dot"] = (
            self.performance_trajectory["V"] *
            np.cos(self.performance_trajectory["chi"])
        ) / (
            1.0
            - self.performance_trajectory["n"]
            * interpolate_with_period(
                self.performance_trajectory["s"],
                self.track_handler.s_coord(),
                self.track_handler.omega_z(),
                self.track_handler.get_track_length()
            )
        )
        self.performance_trajectory["s_ddot"] = np.zeros_like(
            self.performance_trajectory["s"])  # TODO
        self.performance_trajectory["n_dot"] = self.performance_trajectory["V"] * np.sin(
            self.performance_trajectory["chi"]
        )
        self.performance_trajectory["n_ddot"] = np.zeros_like(
            self.performance_trajectory["s"])  # TODO
        self.performance_trajectory["pitlane_mode"] = self.pitlane_mode

    def perform_trajectory_sampling(
            self,
            track_handler,
            s_start: float,
            s_dot_start: float,
            s_ddot_start: float,
            n_start: float,
            n_dot_start: float,
            n_ddot_start: float,
            V_target: float,
            enable_relative_sampling: bool,
            sampling_mode: bool,
            postprocessed_raceline: dict,
            hybrid_long_sampling: bool,
    ):
        # V_target = 90.0  # DEBUG TEST WHY NOT FULL ACCELL FROM 0
        s_array = np.array([])
        s_dot_array = np.array([])
        s_ddot_array = np.array([])
        s_dot_end_values = np.array([])
        s_end_values = np.array([])
        rel_long_sampling_array = np.array([])

        # determine sampling strategy
        if hybrid_long_sampling and enable_relative_sampling:
            relative_sampling = [True, False]
        else:
            relative_sampling = [False]

        for idx, tendency in enumerate(relative_sampling):
            s_part, s_dot_part, s_ddot_part, s_dot_end_part, s_end_part, rel_long_sampling_part, t_array_part = self.longitudinal_sampling.calc_samples_s_based(
                s_start=s_start,
                s_dot_start=s_dot_start,
                s_ddot_start=s_ddot_start,
                V_target=V_target,
                postprocessed_raceline=postprocessed_raceline,
                track_handler=track_handler,
                raceline_tendency=tendency
            )

            # Concatenate longitudinal samples (first iteration uses assignment, rest use concatenate)
            if idx == 0:
                # First iteration: direct assignment
                s_array = s_part
                s_dot_array = s_dot_part
                s_ddot_array = s_ddot_part
                s_dot_end_values = s_dot_end_part
                s_end_values = s_end_part
                rel_long_sampling_array = rel_long_sampling_part
                t_array = t_array_part
            else:
                # Subsequent iterations: concatenate along axis 0 (trajectory dimension)
                s_array = np.concatenate((s_array, s_part), axis=0)
                s_dot_array = np.concatenate((s_dot_array, s_dot_part), axis=0)
                s_ddot_array = np.concatenate(
                    (s_ddot_array, s_ddot_part), axis=0)
                s_dot_end_values = np.concatenate(
                    (s_dot_end_values, s_dot_end_part), axis=0)
                s_end_values = np.concatenate(
                    (s_end_values, s_end_part), axis=0)
                rel_long_sampling_array = np.concatenate(
                    (rel_long_sampling_array, rel_long_sampling_part), axis=0)
                t_array = np.concatenate((t_array, t_array_part), axis=0)

        # Forward backward integrated velocity profiles
        if self.add_forward_backward_samples:
            # rospy.logerr(f"\n--- Forward-Backward Sampling ---")
            # rospy.logerr(
            #     f"  add_forward_backward_samples=True, starting forward-backward integration")
            s_part, s_dot_part, s_ddot_part, s_dot_end_part, s_end_part, rel_long_sampling_part, t_array_part = self.longitudinal_sampling.calc_samples_s_based_forward_backward(
                s_start=s_start,
                s_dot_start=s_dot_start,
                s_ddot_start=s_ddot_start,
                n_start=n_start,
                V_target=V_target,
                postprocessed_raceline=postprocessed_raceline,
                track_handler=track_handler,
                gggv_handler=self.gggv_handler,
                pitlane_mode=self.pitlane_mode,
                raceline_tendency=True
            )

            s_array = np.concatenate((s_array, s_part))
            s_dot_array = np.concatenate((s_dot_array, s_dot_part))
            s_ddot_array = np.concatenate((s_ddot_array, s_ddot_part))
            s_dot_end_values = np.concatenate(
                (s_dot_end_values, s_dot_end_part))
            s_end_values = np.concatenate((s_end_values, s_end_part))
            rel_long_sampling_array = np.concatenate(
                (rel_long_sampling_array, rel_long_sampling_part))
            t_array = np.concatenate((t_array, t_array_part))
        else:
            pass

        n_array, n_dot_array, n_ddot_array = self.lateral_sampling.calc_samples(
            s_start=s_start,
            s_dot_start=s_dot_start,
            s_array=s_array,
            s_dot_array=s_dot_array,
            s_ddot_array=s_ddot_array,
            s_dot_end_values=s_dot_end_values,
            s_end_values=s_end_values,
            n_start=n_start,
            n_dot_start=n_dot_start,
            n_ddot_start=n_ddot_start,
            t_array=t_array,
            postprocessed_raceline=postprocessed_raceline,
            raceline_tendency=True,
            track_handler=track_handler,
            vehicle_params=self.vehicle_params,
        )

        return s_array, s_dot_array, s_ddot_array, n_array, n_dot_array, n_ddot_array, rel_long_sampling_array, t_array
