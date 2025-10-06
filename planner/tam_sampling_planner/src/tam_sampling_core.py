#!/usr/bin/env python3
"""
TAM Sampling Core - F1Tenth-compatible implementation
Simplified for F1Tenth simulation with GlobalWaypointsTrackHandler

This module integrates the F1Tenth-compatible TAM components:
- Lateral sampling (quintic polynomials)
- Longitudinal sampling (velocity profiles) 
- Coordinate transformations (Frenet-Cartesian, F1Tenth WpntArray)
- Trajectory validation (2D safety checks)
- Cost calculations (multi-objective optimization)
"""
import numpy as np
from typing import List, Tuple, Dict, Optional
import time
import rospy

# Import F1Tenth-compatible TAM modular components
from lateral_sampling import LateralSampling
from longitudinal_sampling import LongitudinalSampling
from coordinate_transformation import CoordinateTransformation
from trajectory_checks import TrajectoryChecks
from calculation_costs import CalculationCosts
from trajectory import Trajectory

# F1Tenth track handler
try:
    from track_handler_global_waypoints import GlobalWaypointsTrackHandler
except ImportError:
    rospy.logwarn("GlobalWaypointsTrackHandler not found")
    GlobalWaypointsTrackHandler = None


class FrenetTrajectory:
    """Complete Frenet trajectory representation (TAM-compatible)"""

    def __init__(self):
        # Time array
        self.t = []

        # Frenet coordinates
        self.s = []      # longitudinal position
        self.s_dot = []  # longitudinal velocity
        self.s_ddot = []  # longitudinal acceleration
        self.n = []      # lateral position
        self.n_dot = []  # lateral velocity
        self.n_ddot = []  # lateral acceleration

        # Cartesian coordinates
        self.x = []      # global x position
        self.y = []      # global y position
        self.heading = []  # heading angle

        # Vehicle state
        self.V = []      # velocity magnitude
        self.v_x = []    # velocity x component
        self.v_y = []    # velocity y component

        # Accelerations in velocity frame
        self.ax_vf = []  # longitudinal acceleration (velocity frame)
        self.ay_vf = []  # lateral acceleration (velocity frame)
        self.Omega_z = []  # angular velocity

        # Trajectory properties
        self.cost = float('inf')
        self.valid = False
        self.sample_info = {}


class TAMSamplingCore:
    """
    F1Tenth TAM Sampling Planner Core
    Integrates all F1Tenth-compatible TAM modules
    """

    def __init__(self, use_f1tenth_mode=True):
        """Initialize TAM sampling core with F1Tenth-compatible modules"""

        rospy.loginfo("[TAMCore] Initializing F1Tenth TAM Sampling Core...")

        # Load parameters from ROS parameter server
        self.load_parameters()

        # Initialize F1Tenth-compatible TAM modules
        try:
            self.lateral_sampling = LateralSampling(
                use_f1tenth_mode=use_f1tenth_mode)
            self.longitudinal_sampling = LongitudinalSampling(
                use_f1tenth_mode=use_f1tenth_mode)
            self.coordinate_transformation = CoordinateTransformation(
                use_f1tenth_mode=use_f1tenth_mode)
            self.trajectory_checks = TrajectoryChecks(
                use_f1tenth_mode=use_f1tenth_mode)
            self.calculation_costs = CalculationCosts(
                use_f1tenth_mode=use_f1tenth_mode)
            self.trajectory = Trajectory(debugging=False)
            rospy.loginfo("[TAMCore] ✓ All modules initialized successfully")
        except Exception as e:
            rospy.logerr(f"[TAMCore] Failed to initialize modules: {e}")
            raise

        # Performance tracking
        self.last_computation_time = 0.0
        self.trajectory_count = 0
        self.traj_cnt = 0
        self.performance_trajectory = {}

    def load_parameters(self):
        """Load F1Tenth parameters from ROS parameter server"""
        # Core sampling parameters
        self.lateral_samples = rospy.get_param('lateral_samples', 15)
        self.longitudinal_samples = rospy.get_param('longitudinal_samples', 8)
        self.planning_horizon = rospy.get_param('planning_horizon', 4.0)
        self.dt = rospy.get_param('dt', 0.1)
        self.n_dense_samples = rospy.get_param('n_dense_samples', 5)

        # Vehicle constraints (from NUC2 car model)
        self.max_speed = rospy.get_param('max_speed', 10.0)
        self.max_accel = rospy.get_param('max_accel', 8.29)
        self.max_lateral_accel = rospy.get_param('max_lateral_accel', 12.0)
        self.max_trajectories = rospy.get_param('max_trajectories', 100)

        # Safety parameters
        self.safety_distance_track_left = rospy.get_param(
            'safety_distance_track_left', 0.5)
        self.safety_distance_track_right = rospy.get_param(
            'safety_distance_track_right', 0.5)
        self.safety_margin_static = rospy.get_param(
            'safety_margin_static', 0.5)
        self.safety_margin_dynamic = rospy.get_param(
            'safety_margin_dynamic', 1.0)

        # Cost function weights
        self.raceline_cost_weight = rospy.get_param(
            'raceline_cost_weight', 3.5)
        self.velocity_cost_weight = rospy.get_param(
            'velocity_cost_weight', 3.0)
        self.friction_cost_weight = rospy.get_param(
            'friction_cost_weight', 5000.0)
        self.curvature_cost_weight = rospy.get_param(
            'curvature_cost_weight', 500000.0)
        self.lateral_jerk_cost_weight = rospy.get_param(
            'lateral_jerk_cost_weight', 0.5)
        self.prediction_cost_weight = rospy.get_param(
            'prediction_cost_weight', 100000.0)
        self.collision_cost_weight = rospy.get_param(
            'collision_cost_weight', 100000000.0)

        # Trajectory validation
        self.kappa_thr = rospy.get_param('kappa_thr', 0.1)
        self.curvature_cost_threshold = rospy.get_param(
            'curvature_cost_threshold', 30.0)
        self.increasing_rl_cost = rospy.get_param('increasing_rl_cost', True)

        # Longitudinal sampling
        self.s_dot_discretization = rospy.get_param(
            's_dot_discretization', 2.0)
        self.v_sampling_scale = rospy.get_param('v_sampling_scale', 1.1)

        # Lateral sampling
        self.n_dense_min = rospy.get_param('n_dense_min', -0.5)
        self.n_dense_max = rospy.get_param('n_dense_max', 0.5)
        self.track_width = rospy.get_param('track_width', 3.0)

        # Trajectory generation
        self.tube_width = rospy.get_param('tube_width', 1.0)
        self.trajectory_len_controller = rospy.get_param(
            'trajectory_len_controller', 50)

        rospy.loginfo(f"[TAMCore] Parameters loaded: lat_samples={self.lateral_samples}, "
                      f"long_samples={self.longitudinal_samples}, horizon={self.planning_horizon}s, "
                      f"max_speed={self.max_speed}m/s, max_accel={self.max_accel}m/s²")

    def calc_trajectory(self, current_state: Dict, track_handler,
                        obstacles: List[Dict] = None) -> Optional[Dict]:
        """
        Main trajectory planning method for F1Tenth (simplified TAM implementation)

        This follows the original TAM calc_trajectory() flow from samplingplanner.py
        but adapted for F1Tenth requirements (no GGGV, no postprocessed_raceline)

        Args:
            current_state: Dict with keys: s, n, s_dot, n_dot, s_ddot, n_ddot, x, y, psi
            track_handler: GlobalWaypointsTrackHandler instance
            obstacles: List of obstacle dicts (optional)

        Returns:
            Best trajectory dict with keys: s, n, v, chi, ax, ay, t, s_loc, emergency, cost
            Ready for convert_trajectory_to_wpnt_array()
        """

        start_time = time.time()
        self.traj_cnt += 1

        try:
            # Extract starting conditions from current state
            s_start = current_state.get('s', 0.0)
            n_start = current_state.get('n', 0.0)
            s_dot_start = max(current_state.get('s_dot', 0.1),
                              0.1)  # Avoid zero velocity
            n_dot_start = current_state.get('n_dot', 0.0)
            s_ddot_start = current_state.get('s_ddot', 0.0)
            n_ddot_start = current_state.get('n_ddot', 0.0)

            rospy.logdebug(
                f"[TAMCore] Planning from s={s_start:.2f}, n={n_start:.2f}, v={s_dot_start:.2f}")

            # STEP 1: Generate lateral trajectory samples (quintic polynomials)
            lateral_samples = self.lateral_sampling.calc_frenet_trajectories(
                n_start=n_start,
                n_dot_start=n_dot_start,
                n_ddot_start=n_ddot_start,
                planning_horizon=self.planning_horizon
            )

            if not lateral_samples:
                rospy.logwarn(
                    "[TAMCore] No lateral samples generated, using emergency trajectory")
                return self.generate_emergency_trajectory(current_state, track_handler)

            # STEP 2: Generate longitudinal profiles and combine with lateral samples
            full_trajectories = []
            for lat_sample in lateral_samples:
                long_samples = self.longitudinal_sampling.calc_velocity_profiles(
                    s_start=s_start,
                    s_dot_start=s_dot_start,
                    s_ddot_start=s_ddot_start,
                    v_target=min(self.max_speed, s_dot_start * 1.2),
                    planning_horizon=self.planning_horizon
                )

                # Combine lateral and longitudinal samples
                for long_sample in long_samples:
                    traj = {
                        's': long_sample['s'],  # Array of s positions
                        'n': lat_sample['n'],   # Array of n positions
                        's_dot': long_sample['s_dot'],
                        'n_dot': lat_sample['n_dot'],
                        's_ddot': long_sample.get('s_ddot', np.zeros_like(long_sample['s'])),
                        'n_ddot': lat_sample.get('n_ddot', np.zeros_like(lat_sample['n'])),
                        't': long_sample['t'],
                        # Local s coordinate
                        's_loc': long_sample['s'] - s_start,
                        # F1Tenth uses lowercase 'v'
                        'v': long_sample['s_dot'],
                        'emergency': False
                    }

                    # STEP 3: Calculate chi (velocity angle in Frenet frame)
                    # chi = arctan(n_dot / s_dot) - matches original TAM
                    traj['chi'] = np.arctan2(traj['n_dot'], traj['s_dot'])

                    full_trajectories.append(traj)

            rospy.logdebug(
                f"[TAMCore] Generated {len(full_trajectories)} trajectory samples")

            # STEP 4: Validate trajectories (collision checks, track bounds)
            valid_trajectories = []
            for traj in full_trajectories:
                if self.trajectory_checks.check_single_trajectory(traj, track_handler, obstacles):
                    # Add accelerations (needed for visualization and controller)
                    traj['ax'] = np.gradient(traj['v'], traj['t'])
                    traj['ay'] = traj['v']**2 * \
                        np.gradient(traj['chi'], traj['s'])
                    valid_trajectories.append(traj)

            if not valid_trajectories:
                rospy.logwarn(
                    f"[TAMCore] No valid trajectories out of {len(full_trajectories)} samples")
                return self.generate_emergency_trajectory(current_state, track_handler)

            # STEP 5: Calculate costs for valid trajectories
            for traj in valid_trajectories:
                traj['cost'] = self.calculation_costs.calculate_trajectory_cost(
                    traj, track_handler, obstacles
                )

            # STEP 6: Select best trajectory (minimum cost)
            best_trajectory = min(valid_trajectories, key=lambda t: t['cost'])

            # Update performance statistics
            self.last_computation_time = time.time() - start_time
            self.trajectory_count = len(valid_trajectories)
            self.performance_trajectory = best_trajectory

            rospy.logdebug(f"[TAMCore] Planning complete: {self.last_computation_time*1000:.1f}ms, "
                           f"{len(valid_trajectories)} valid, cost={best_trajectory['cost']:.3f}")

            return best_trajectory

        except Exception as e:
            rospy.logerr(f"[TAMCore] Error in calc_trajectory: {e}")
            import traceback
            traceback.print_exc()
            return self.generate_emergency_trajectory(current_state, track_handler)

    # ========== LEGACY METHODS - COMMENTED OUT FOR F1TENTH ==========
    # These methods are from the original TAM implementation and are not used in F1Tenth
    # Use calc_trajectory() instead

    # def generate_trajectory_samples(self, current_state: Dict, track_data: Dict = None,
    #                                 obstacles: List = None) -> List[FrenetTrajectory]:
    #     """
    #     LEGACY METHOD - kept for backwards compatibility
    #     Use calc_trajectory() instead for F1Tenth
    #
    #     Generate complete TAM trajectory samples
    #
    #     Args:
    #         current_state: Current vehicle state
    #         track_data: Track geometry information
    #         obstacles: List of obstacles
    #
    #     Returns:
    #         List of valid trajectory samples
    #     """
    #     rospy.logwarn("[TAMCore] Using legacy generate_trajectory_samples(), prefer calc_trajectory()")
    #
    #     # Call new F1Tenth method and convert output
    #     traj_dict = self.calc_trajectory(current_state, track_data, obstacles)
    #
    #     if traj_dict is None:
    #         return []
    #
    #     # Convert dict to FrenetTrajectory for backwards compatibility
    #     traj_obj = FrenetTrajectory()
    #     traj_obj.t = traj_dict.get('t', [])
    #     traj_obj.s = traj_dict.get('s', [])
    #     traj_obj.s_dot = traj_dict.get('s_dot', [])
    #     traj_obj.s_ddot = traj_dict.get('s_ddot', [])
    #     traj_obj.n = traj_dict.get('n', [])
    #     traj_obj.n_dot = traj_dict.get('n_dot', [])
    #     traj_obj.n_ddot = traj_dict.get('n_ddot', [])
    #     traj_obj.cost = traj_dict.get('cost', float('inf'))
    #     traj_obj.valid = not traj_dict.get('emergency', False)
    #
    #     return [traj_obj]

    # def find_best_trajectory(self, trajectories: List[FrenetTrajectory]) -> Optional[FrenetTrajectory]:
    #     """
    #     Find best trajectory from samples using cost function
    #
    #     Args:
    #         trajectories: List of trajectory samples
    #
    #     Returns:
    #         Best trajectory or None if no valid trajectories
    #     """
    #
    #     if not trajectories:
    #         return None
    #
    #     # Filter valid trajectories
    #     valid_trajectories = [t for t in trajectories if t.valid]
    #
    #     if not valid_trajectories:
    #         return None
    #
    #     # Find minimum cost trajectory
    #     best_trajectory = min(valid_trajectories, key=lambda t: t.cost)
    #
    #     return best_trajectory

    # def plan_trajectory(self, current_state: Dict, track_data: Dict = None,
    #                     obstacles: List = None) -> Optional[FrenetTrajectory]:
    #     """
    #     Complete TAM trajectory planning pipeline
    #
    #     Args:
    #         current_state: Current vehicle state
    #         track_data: Track geometry information
    #         obstacles: List of obstacles
    #
    #     Returns:
    #         Best planned trajectory or None
    #     """
    #
    #     # Generate trajectory samples
    #     trajectories = self.generate_trajectory_samples(
    #         current_state, track_data, obstacles)
    #
    #     # Find best trajectory
    #     best_trajectory = self.find_best_trajectory(trajectories)
    #
    #     return best_trajectory

    def update_parameters(self, new_params: Dict = None):
        """
        Update TAM parameters dynamically from ROS parameter server

        Args:
            new_params: Dictionary of updated parameters (optional, uses ROS params if None)
        """

        if new_params is None:
            # Reload from ROS parameter server
            self.load_parameters()
            rospy.loginfo(
                "[TAMCore] Parameters reloaded from ROS parameter server")
        else:
            # Update specific parameters
            for key, value in new_params.items():
                if hasattr(self, key):
                    setattr(self, key, value)
                    rospy.logdebug(
                        f"[TAMCore] Updated parameter {key}={value}")

            # Note: F1Tenth modules don't have update_parameters method
            # They read directly from rospy.get_param() when needed

    def get_computation_stats(self) -> Dict:
        """Get performance statistics"""
        return {
            'last_computation_time': self.last_computation_time,
            'trajectory_count': self.trajectory_count,
            'planning_horizon': self.planning_horizon,
            'max_trajectories': self.max_trajectories
        }

    # def validate_single_trajectory(self, trajectory: FrenetTrajectory,
    #                                track_data: Dict = None, obstacles: List = None) -> bool:
    #     """
    #     LEGACY METHOD - not used in F1Tenth
    #     Validate and score single trajectory
    #
    #     Args:
    #         trajectory: Trajectory to validate
    #         track_data: Track geometry information
    #         obstacles: List of obstacles
    #
    #     Returns:
    #         Boolean indicating if trajectory is valid
    #     """
    #
    #     # Validate trajectory
    #     is_valid = self.trajectory_checks.validate_trajectory(trajectory)
    #     trajectory.valid = is_valid
    #
    #     if is_valid:
    #         # Calculate cost
    #         cost = self.calculation_costs.calculate_trajectory_cost(
    #             trajectory, track_data, obstacles
    #         )
    #         trajectory.cost = cost
    #     else:
    #         trajectory.cost = float('inf')
    #
    #     return is_valid

    def get_lateral_bounds(self) -> Tuple[float, float]:
        """Get current lateral sampling bounds from ROS params"""
        try:
            if hasattr(self.lateral_sampling, 'get_lateral_bounds'):
                return self.lateral_sampling.get_lateral_bounds()
            else:
                # Fallback to ROS params
                n_min = rospy.get_param('~lateral_bound_min', -2.0)
                n_max = rospy.get_param('~lateral_bound_max', 2.0)
                return (n_min, n_max)
        except Exception as e:
            rospy.logwarn(f"[TAMCore] Error getting lateral bounds: {e}")
            return (-2.0, 2.0)

    def get_velocity_bounds(self) -> Tuple[float, float]:
        """Get current velocity sampling bounds from ROS params"""
        try:
            if hasattr(self.longitudinal_sampling, 'get_velocity_bounds'):
                return self.longitudinal_sampling.get_velocity_bounds()
            else:
                # Fallback to ROS params
                v_min = rospy.get_param('~velocity_min', 0.5)
                v_max = rospy.get_param('~velocity_max', 10.0)
                return (v_min, v_max)
        except Exception as e:
            rospy.logwarn(f"[TAMCore] Error getting velocity bounds: {e}")
            return (0.5, 10.0)

    def set_safety_margins(self, left_margin: float, right_margin: float):
        """Update safety margins for trajectory validation"""
        safety_params = {
            'safety_distance_track_left': left_margin,
            'safety_distance_track_right': right_margin
        }
        self.update_parameters(safety_params)

    def generate_emergency_trajectory(self, current_state: Dict, track_handler) -> Optional[Dict]:
        """
        Generate emergency braking trajectory using Pacejka tire model

        Args:
            current_state: Current vehicle state dict
            track_handler: GlobalWaypointsTrackHandler for track bounds

        Returns:
            Emergency trajectory dict or None
        """
        try:
            rospy.logwarn("[TAMCore] Generating emergency braking trajectory")

            # Use Trajectory module's emergency trajectory generation
            emergency_traj = self.trajectory.calc_emergency_trajectory(
                track_handler=track_handler,
                s_start=current_state.get('s', 0.0),
                n_start=current_state.get('n', 0.0),
                s_dot_start=max(current_state.get('s_dot', 0.1), 0.1),
                n_dot_start=current_state.get('n_dot', 0.0),
                s_ddot_start=current_state.get('s_ddot', 0.0),
                n_ddot_start=current_state.get('n_ddot', 0.0)
            )

            if emergency_traj:
                emergency_traj['emergency'] = True
                emergency_traj['cost'] = 0.0  # Highest priority
                rospy.loginfo(
                    "[TAMCore] Emergency trajectory generated successfully")
                return emergency_traj
            else:
                rospy.logerr(
                    "[TAMCore] Failed to generate emergency trajectory")
                return None

        except Exception as e:
            rospy.logerr(
                f"[TAMCore] Emergency trajectory generation failed: {e}")
            import traceback
            traceback.print_exc()
            return None

    # def optimize_trajectory_set(self, trajectories: List[FrenetTrajectory],
    #                             max_set_size: int = 10) -> List[FrenetTrajectory]:
    #     """
    #     LEGACY METHOD - not used in F1Tenth
    #     Optimize trajectory set using diversity and cost criteria
    #
    #     Args:
    #         trajectories: Input trajectory set
    #         max_set_size: Maximum number of trajectories to keep
    #
    #     Returns:
    #         Optimized trajectory set
    #     """
    #
    #     if len(trajectories) <= max_set_size:
    #         return trajectories
    #
    #     # Sort by cost
    #     sorted_trajectories = sorted(trajectories, key=lambda t: t.cost)
    #
    #     # Keep best trajectories with diversity consideration
    #     optimized_set = []
    #
    #     for trajectory in sorted_trajectories:
    #         if len(optimized_set) >= max_set_size:
    #             break
    #
    #         # Check diversity (simplified - could use more sophisticated metrics)
    #         is_diverse = True
    #         for existing_traj in optimized_set:
    #             if self._trajectories_similar(trajectory, existing_traj):
    #                 is_diverse = False
    #                 break
    #
    #         if is_diverse or len(optimized_set) == 0:
    #             optimized_set.append(trajectory)
    #
    #     return optimized_set

    # def _trajectories_similar(self, traj1: FrenetTrajectory, traj2: FrenetTrajectory,
    #                           threshold: float = 0.5) -> bool:
    #     """Check if two trajectories are similar"""
    #
    #     if len(traj1.n) == 0 or len(traj2.n) == 0:
    #         return False
    #
    #     # Compare final lateral positions
    #     n1_final = traj1.n[-1]
    #     n2_final = traj2.n[-1]
    #
    #     if abs(n1_final - n2_final) < threshold:
    #         return True
    #
    #     return False

    # def get_trajectory_diversity_metrics(self, trajectories: List[FrenetTrajectory]) -> Dict:
    #     """
    #     LEGACY METHOD - not used in F1Tenth
    #     Calculate diversity metrics for trajectory set
    #     """
    #
    #     if len(trajectories) < 2:
    #         return {'diversity_score': 0.0, 'lateral_spread': 0.0, 'velocity_spread': 0.0}
    #
    #     # Lateral diversity
    #     final_lateral_positions = [
    #         t.n[-1] if len(t.n) > 0 else 0.0 for t in trajectories]
    #     lateral_spread = max(final_lateral_positions) - \
    #         min(final_lateral_positions)
    #
    #     # Velocity diversity
    #     final_velocities = [t.s_dot[-1]
    #                         if len(t.s_dot) > 0 else 0.0 for t in trajectories]
    #     velocity_spread = max(final_velocities) - min(final_velocities)
    #
    #     # Overall diversity score
    #     diversity_score = lateral_spread + 0.1 * velocity_spread
    #
    #     return {
    #         'diversity_score': diversity_score,
    #         'lateral_spread': lateral_spread,
    #         'velocity_spread': velocity_spread,
    #         'trajectory_count': len(trajectories)
    #     }
