#!/usr/bin/env python3
"""
Complete TAM Sampling Planner Core Implementation
Ported from original TAM ROS2 sampling planner modules for ROS1 multi-car racing

This module contains the comprehensive TAM sampling algorithms:
- LateralSampling: Quintic polynomial lateral trajectory generation
- LongitudinalSampling: Velocity profile generation with constraints
- CoordinateTransformation: Frenet to Cartesian conversion
- TrajectoryChecks: Safety and feasibility validation
- CalculationCosts: Multi-objective cost function evaluation
- TAMSamplingCore: Main planning interface
"""
import numpy as np
from typing import List, Tuple, Dict, Optional
import time
from dataclasses import dataclass


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
        self.x = []      # global x
        self.y = []      # global y
        self.psi = []    # absolute heading

        # Vehicle dynamics
        self.V = []      # velocity magnitude
        self.chi = []    # heading relative to track
        self.ax_vf = []  # longitudinal acceleration (velocity frame)
        self.ay_vf = []  # lateral acceleration (velocity frame)
        self.Omega_z = []  # angular velocity

        # Trajectory properties
        self.cost = float('inf')
        self.valid = False
        self.sample_info = {}


class LateralSampling:
    """TAM Lateral Sampling - Quintic polynomial generation"""

    def __init__(self, params: Dict):
        self.params = params
        self.n_samples = params.get('lateral_samples', 15)
        self.n_dense_samples = params.get('n_dense_samples', 5)
        self.n_dense_min = params.get('n_dense_min', -0.5)
        self.n_dense_max = params.get('n_dense_max', 0.5)
        self.track_width = params.get('track_width', 3.0)
        self.safety_distance_left = params.get(
            'safety_distance_track_left', 0.5)
        self.safety_distance_right = params.get(
            'safety_distance_track_right', 0.5)
        self.tube_width = params.get('tube_width', 1.0)

    def calc_samples(self,
                     s_start: float, s_dot_start: float, s_array: np.ndarray,
                     s_dot_array: np.ndarray, s_ddot_array: np.ndarray,
                     s_end_values: np.ndarray, s_dot_end_values: np.ndarray,
                     n_start: float, n_dot_start: float, n_ddot_start: float,
                     t_array: np.ndarray, raceline_data: dict, track_data: dict):
        """
        Generate lateral trajectory samples using quintic polynomials
        Following exact TAM lateral sampling approach
        """

        # Initialize output arrays
        n_samples_total = len(s_end_values)
        n_array_all = np.zeros((n_samples_total, len(t_array[0])))
        n_dot_array_all = np.zeros_like(n_array_all)
        n_ddot_array_all = np.zeros_like(n_array_all)

        # Generate lateral target positions
        track_left_bound = self.track_width/2 - \
            self.safety_distance_left - self.tube_width
        track_right_bound = -self.track_width/2 + \
            self.safety_distance_right + self.tube_width

        # Regular sampling across track width
        n_targets_regular = np.linspace(track_right_bound, track_left_bound,
                                        self.n_samples - self.n_dense_samples)

        # Dense sampling around raceline
        raceline_n = raceline_data.get('n', 0.0)
        n_targets_dense = np.linspace(raceline_n + self.n_dense_min,
                                      raceline_n + self.n_dense_max,
                                      self.n_dense_samples)

        # Combine targets
        n_targets = np.concatenate(
            [n_targets_regular, n_targets_dense, [raceline_n]])

        sample_idx = 0
        for i, (s_end, s_dot_end) in enumerate(zip(s_end_values, s_dot_end_values)):
            t_end = t_array[i][-1]

            # Precompute polynomial time matrices for efficiency
            ones = np.ones_like(t_array[i])
            t_quad = t_array[i] ** 2
            t_cubi = t_array[i] * t_quad
            t_quar = t_array[i] * t_cubi
            t_quin = t_array[i] * t_quar

            t_mat = np.column_stack(
                [ones, t_array[i], t_quad, t_cubi, t_quar, t_quin])
            t_mat_dot = np.column_stack(
                [np.zeros_like(ones), ones, 2*t_array[i], 3*t_quad, 4*t_cubi, 5*t_quar])
            t_mat_ddot = np.column_stack([np.zeros_like(ones), np.zeros_like(
                ones), 2*ones, 6*t_array[i], 12*t_quad, 20*t_cubi])

            # Boundary condition matrix for quintic polynomial
            A = np.array([
                [1, 0, 0, 0, 0, 0],                           # n(0) = n_start
                # n'(0) = n_dot_start
                [0, 1, 0, 0, 0, 0],
                # n''(0) = n_ddot_start
                [0, 0, 2, 0, 0, 0],
                [1, t_end, t_end**2, t_end**3, t_end **
                    4, t_end**5],  # n(T) = n_target
                [0, 1, 2*t_end, 3*t_end**2, 4*t_end **
                    3, 5*t_end**4],  # n'(T) = 0
                [0, 0, 2, 6*t_end, 12*t_end**2, 20 *
                    t_end**3]         # n''(T) = 0
            ])
            A_inv = np.linalg.inv(A)

            for n_target in n_targets:
                if sample_idx >= n_samples_total:
                    break

                # Boundary conditions vector
                b = np.array(
                    [n_start, n_dot_start, n_ddot_start, n_target, 0.0, 0.0])

                # Solve for polynomial coefficients
                coeffs = A_inv @ b

                # Evaluate polynomial
                n_array_all[sample_idx] = t_mat @ coeffs
                n_dot_array_all[sample_idx] = t_mat_dot @ coeffs
                n_ddot_array_all[sample_idx] = t_mat_ddot @ coeffs

                sample_idx += 1

        return n_array_all[:sample_idx], n_dot_array_all[:sample_idx], n_ddot_array_all[:sample_idx]


class LongitudinalSampling:
    """TAM Longitudinal Sampling - Velocity profile generation"""

    def __init__(self, params: Dict):
        self.params = params
        self.s_dot_end_min = params.get('s_dot_end_min', 1.0)
        self.s_dot_discretization = params.get('s_dot_discretization', 2.0)
        self.s_dot_max_positive_delta = params.get(
            's_dot_max_positive_delta', 20.0)
        self.v_sampling_scale = params.get('v_sampling_scale', 1.1)
        self.relative_s_dot_min_percentage = params.get(
            'relative_s_dot_min_percentage', 0.5)
        self.horizon = params.get('planning_horizon', 4.0)
        self.max_speed = params.get('max_speed', 20.0)
        self.max_accel = params.get('max_accel', 8.0)

    def calc_samples(self, s_start: float, s_dot_start: float, s_ddot_start: float,
                     V_target: float, V_max: float, raceline_data: dict, raceline_tendency: bool):
        """
        Generate longitudinal velocity profiles following TAM approach
        """

        # Extract raceline velocity at horizon
        if 't_post' in raceline_data and 's_dot_post' in raceline_data:
            s_dot_end_rl = np.interp(
                self.horizon, raceline_data['t_post'], raceline_data['s_dot_post'])
            s_ddot_end_rl = 0.0
        else:
            s_dot_end_rl = raceline_data.get('s_dot', V_target)
            s_ddot_end_rl = 0.0

        # Calculate velocity sampling bounds
        s_dot_max = min(
            max(s_dot_start, min(s_dot_start + self.s_dot_max_positive_delta,
                V_target, s_dot_end_rl)) * self.v_sampling_scale,
            V_max
        )
        s_dot_max = max(s_dot_max, 5.0)  # Minimum velocity threshold

        # Generate velocity end targets
        if raceline_tendency:
            s_dot_min = self.relative_s_dot_min_percentage * s_dot_max
            s_dot_end_values = np.arange(
                s_dot_min, s_dot_max, self.s_dot_discretization)
        else:
            s_dot_end_values = np.arange(
                self.s_dot_end_min, s_dot_max, self.s_dot_discretization)

        # Always include raceline and target speeds
        s_dot_end_values = np.concatenate(
            [s_dot_end_values, [max(V_target, 1.0), s_dot_end_rl]])
        s_dot_end_values = np.unique(s_dot_end_values)

        # Generate time arrays and trajectories
        time_arrays = []
        s_arrays = []
        s_dot_arrays = []
        s_ddot_arrays = []
        s_end_values = []

        for s_dot_end in s_dot_end_values:
            # Generate time-optimal trajectory to target velocity
            t_traj, s_traj, s_dot_traj, s_ddot_traj = self._generate_velocity_profile(
                s_start, s_dot_start, s_ddot_start, s_dot_end, s_ddot_end_rl
            )

            if len(t_traj) > 0:
                time_arrays.append(t_traj)
                s_arrays.append(s_traj)
                s_dot_arrays.append(s_dot_traj)
                s_ddot_arrays.append(s_ddot_traj)
                s_end_values.append(s_traj[-1])

        return (time_arrays, s_arrays, s_dot_arrays, s_ddot_arrays,
                np.array(s_end_values), s_dot_end_values[:len(s_end_values)])

    def _generate_velocity_profile(self, s_start: float, s_dot_start: float, s_ddot_start: float,
                                   s_dot_end: float, s_ddot_end: float):
        """Generate acceleration-limited velocity profile"""

        dt = 0.1
        t_max = self.horizon
        num_points = int(t_max / dt) + 1

        t_traj = np.linspace(0, t_max, num_points)
        s_traj = np.zeros(num_points)
        s_dot_traj = np.zeros(num_points)
        s_ddot_traj = np.zeros(num_points)

        # Initial conditions
        s_traj[0] = s_start
        s_dot_traj[0] = s_dot_start
        s_ddot_traj[0] = s_ddot_start

        # Simple velocity control towards target
        for i in range(1, num_points):
            # Velocity error and control
            v_error = s_dot_end - s_dot_traj[i-1]
            accel = np.clip(
                v_error / (t_max - t_traj[i-1] + 1e-6), -self.max_accel, self.max_accel)

            # Integration
            s_ddot_traj[i] = accel
            s_dot_traj[i] = s_dot_traj[i-1] + accel * dt
            s_dot_traj[i] = max(
                0, min(s_dot_traj[i], self.max_speed))  # Clamp velocity
            s_traj[i] = s_traj[i-1] + s_dot_traj[i] * dt

        return t_traj, s_traj, s_dot_traj, s_ddot_traj


class CoordinateTransformation:
    """TAM Coordinate Transformation - Frenet to Cartesian conversion"""

    def __init__(self, params: Dict):
        self.params = params

    def transform_to_velocity_frame(self, track_data: dict, s_array: np.ndarray,
                                    s_dot_array: np.ndarray, s_ddot_array: np.ndarray,
                                    n_array: np.ndarray, n_dot_array: np.ndarray,
                                    n_ddot_array: np.ndarray):
        """Transform Frenet trajectories to velocity frame (TAM method)"""

        # Track curvature (omega_z) interpolation
        if 'omega_z' in track_data and 's_coord' in track_data:
            Omega_z_rf = np.interp(
                s_array, track_data['s_coord'], track_data['omega_z'])
            dOmega_z_rf = np.interp(s_array, track_data['s_coord'], track_data.get(
                'd_omega_z', np.zeros_like(track_data['omega_z'])))
        else:
            # Simplified curvature estimation
            Omega_z_rf = np.zeros_like(s_array)
            dOmega_z_rf = np.zeros_like(s_array)

        # Vehicle heading relative to track
        chi_array = np.arctan(
            n_dot_array / (s_dot_array * (1.0 - Omega_z_rf * n_array) + 1e-6))

        # Absolute velocity magnitude
        V_array = np.sqrt((1.0 - Omega_z_rf * n_array)**2 *
                          s_dot_array**2 + n_dot_array**2)

        # Longitudinal acceleration in velocity frame
        ax_vf_array = (1 / np.sqrt(s_dot_array**2 * (1.0 - Omega_z_rf * n_array)**2 + n_dot_array**2) *
                       (s_dot_array * s_ddot_array * (1.0 - Omega_z_rf * n_array)**2 -
                       s_dot_array**2 * (1.0 - Omega_z_rf * n_array) *
                       (dOmega_z_rf * s_dot_array * n_array + Omega_z_rf * n_dot_array) +
                       n_dot_array * n_ddot_array))

        # Angular velocity in velocity frame
        Omega_z_vf = ((n_ddot_array + dOmega_z_rf * n_array * s_dot_array + Omega_z_rf * n_dot_array) * 
                     np.cos(chi_array) + Omega_z_rf * (1.0 - Omega_z_rf * n_array) * np.sin(chi_array)) / (V_array + 1e-6)
        
        # Lateral acceleration in velocity frame
        ay_vf_array = V_array * Omega_z_vf
        
        return chi_array, V_array, ax_vf_array, ay_vf_array, Omega_z_vf

    def frenet_to_cartesian(self, s_array: np.ndarray, n_array: np.ndarray, track_data: dict):
        """Convert Frenet coordinates to Cartesian"""

        if 'centerline' not in track_data:
            # Fallback: simple coordinate transformation
            x_array = s_array * 0.1  # Simple scaling
            y_array = n_array
            psi_array = np.zeros_like(s_array)
            return x_array, y_array, psi_array

        centerline = track_data['centerline']
        s_coord = track_data.get('s_coord', np.arange(len(centerline)))
        headings = track_data.get('headings', np.zeros(len(centerline)))

        # Interpolate centerline position and heading
        x_center = np.interp(s_array, s_coord, centerline[:, 0], period=s_coord[-1])
        y_center = np.interp(s_array, s_coord, centerline[:, 1], period=s_coord[-1])
        psi_track = np.interp(s_array, s_coord, headings, period=s_coord[-1])

        # Apply lateral offset
        x_array = x_center - n_array * np.sin(psi_track)
        y_array = y_center + n_array * np.cos(psi_track)
        psi_array = psi_track  # Simplified heading

        return x_array, y_array, psi_array


class TrajectoryChecks:
    """TAM Trajectory Validation - Safety and feasibility checks"""

    def __init__(self, params: Dict):
        self.params = params
        self.max_speed = params.get('max_speed', 20.0)
        self.max_accel = params.get('max_accel', 8.0)
        self.max_lateral_accel = params.get('max_lateral_accel', 12.0)
        self.track_width = params.get('track_width', 3.0)
        self.safety_distance_left = params.get('safety_distance_track_left', 0.5)
        self.safety_distance_right = params.get('safety_distance_track_right', 0.5)
        self.kappa_threshold = params.get('kappa_thr', 0.1)

    def validate_trajectory(self, trajectory: FrenetTrajectory, track_data: dict, obstacles: List) -> bool:
        """Comprehensive trajectory validation following TAM checks"""

        # Check velocity limits
        if np.any(np.array(trajectory.V) > self.max_speed):
            return False

        # Check acceleration limits
        if len(trajectory.s_ddot) > 0 and np.any(np.abs(trajectory.s_ddot) > self.max_accel):
            return False

        # Check lateral acceleration limits
        if len(trajectory.ay_vf) > 0 and np.any(np.abs(trajectory.ay_vf) > self.max_lateral_accel):
            return False

        # Check track boundaries
        track_left = self.track_width / 2 - self.safety_distance_left
        track_right = -self.track_width / 2 + self.safety_distance_right

        if np.any(np.array(trajectory.n) > track_left) or np.any(np.array(trajectory.n) < track_right):
            return False

        # Check curvature limits
        if len(trajectory.Omega_z) > 0 and np.any(np.abs(trajectory.Omega_z) > self.kappa_threshold):
            return False

        # TODO: Add obstacle collision checking
        # This would be integrated with the existing obstacle detection system

        return True


class CalculationCosts:
    """TAM Cost Function - Multi-objective trajectory evaluation"""

    def __init__(self, params: Dict):
        self.params = params

        # Cost weights (from TAM parameters)
        self.raceline_cost_weight = params.get('raceline_cost_weight', 3.5)
        self.velocity_cost_weight = params.get('velocity_cost_weight', 3.0)
        self.friction_cost_weight = params.get('friction_cost_weight', 5000.0)
        self.curvature_cost_weight = params.get(
            'curvature_cost_weight', 500000.0)
        self.lateral_jerk_cost_weight = params.get(
            'lateral_jerk_cost_weight', 0.5)
        self.prediction_cost_weight = params.get(
            'prediction_cost_weight', 100000.0)
        self.collision_cost_weight = params.get(
            'collision_cost_weight', 100000000.0)

        # Additional parameters
        self.curvature_cost_threshold = params.get(
            'curvature_cost_threshold', 30.0)
        self.horizon = params.get('planning_horizon', 4.0)
        self.increasing_rl_cost = params.get('increasing_rl_cost', True)

    def calculate_cost(self, trajectory: FrenetTrajectory, raceline_data: dict,
                       track_data: dict, obstacles: List = None) -> float:
        """Calculate trajectory cost using TAM multi-objective function"""

        if not trajectory.valid:
            return float('inf')

        total_cost = 0.0

        # 1. Raceline deviation cost
        raceline_n = raceline_data.get('n', 0.0)
        if isinstance(raceline_n, (list, np.ndarray)):
            raceline_n_interp = np.interp(
                trajectory.t, raceline_data.get('t_post', trajectory.t), raceline_n)
        else:
            raceline_n_interp = np.full(len(trajectory.n), raceline_n)

        n_deviation = np.abs(np.array(trajectory.n) -
                             raceline_n_interp[:len(trajectory.n)])

        if self.increasing_rl_cost:
            # Increasing cost with time (encourage early raceline following)
            time_weight = 1 + np.arange(len(n_deviation)) / len(n_deviation)
            raceline_cost = self.raceline_cost_weight * \
                np.sum(n_deviation * time_weight)
        else:
            raceline_cost = self.raceline_cost_weight * np.sum(n_deviation)

        total_cost += raceline_cost

        # 2. Velocity cost (encourage higher speeds)
        raceline_v = raceline_data.get('s_dot', raceline_data.get('V', 10.0))
        if isinstance(raceline_v, (list, np.ndarray)):
            raceline_v_interp = np.interp(
                trajectory.t, raceline_data.get('t_post', trajectory.t), raceline_v)
        else:
            raceline_v_interp = np.full(len(trajectory.V), raceline_v)

        velocity_deficit = np.maximum(
            raceline_v_interp[:len(trajectory.V)] - np.array(trajectory.V), 0)
        velocity_cost = self.velocity_cost_weight * np.sum(velocity_deficit**2)
        total_cost += velocity_cost

        # 3. Lateral jerk cost (comfort)
        if len(trajectory.n_ddot) > 1:
            lateral_jerk = np.diff(trajectory.n_ddot)
            jerk_cost = self.lateral_jerk_cost_weight * np.sum(lateral_jerk**2)
            total_cost += jerk_cost

        # 4. Curvature cost (avoid high curvatures)
        if len(trajectory.Omega_z) > 0:
            high_curvature_mask = np.abs(
                trajectory.Omega_z) > self.curvature_cost_threshold
            if np.any(high_curvature_mask):
                curvature_cost = self.curvature_cost_weight * \
                    np.sum(np.abs(trajectory.Omega_z)[high_curvature_mask])
                total_cost += curvature_cost

        # 5. Friction cost (vehicle dynamics)
        if len(trajectory.ax_vf) > 0 and len(trajectory.ay_vf) > 0:
            # Simplified friction circle constraint
            friction_utilization = np.sqrt(
                np.array(trajectory.ax_vf)**2 + np.array(trajectory.ay_vf)**2) / 9.81
            friction_violation = np.maximum(friction_utilization - 1.0, 0)
            friction_cost = self.friction_cost_weight * \
                np.sum(friction_violation**2)
            total_cost += friction_cost

        # 6. Obstacle avoidance cost
        if obstacles is not None and len(obstacles) > 0:
            obstacle_cost = self._calculate_obstacle_cost(
                trajectory, obstacles)
            total_cost += obstacle_cost

        return total_cost

    def _calculate_obstacle_cost(self, trajectory: FrenetTrajectory, obstacles: List) -> float:
        """Calculate cost for obstacle avoidance"""

        if not hasattr(trajectory, 'x') or len(trajectory.x) == 0:
            return 0.0

        obstacle_cost = 0.0
        safety_margin = self.params.get('safety_margin_dynamic', 1.0)

        for obstacle in obstacles:
            # Simple distance-based cost
            obs_x = obstacle.get('x', 0.0)
            obs_y = obstacle.get('y', 0.0)
            obs_radius = obstacle.get('radius', 1.0) + safety_margin

            distances = np.sqrt((np.array(trajectory.x) - obs_x)**2 +
                                (np.array(trajectory.y) - obs_y)**2)

            # Heavy penalty for collision risk
            collision_risk = np.maximum(obs_radius - distances, 0)
            if np.any(collision_risk > 0):
                obstacle_cost += self.collision_cost_weight * \
                    np.sum(collision_risk**2)

            # Proximity cost
            proximity_cost = np.sum(np.exp(-distances / obs_radius))
            obstacle_cost += self.prediction_cost_weight * proximity_cost

        return obstacle_cost


class TAMSamplingCore:
    """
    Complete TAM Sampling Planner Core
    Integrates all TAM modules for comprehensive trajectory planning
    """

    def __init__(self, params: Dict):
        """Initialize TAM sampling core with all modules"""
        self.params = params

        # Initialize TAM modules
        self.lateral_sampling = LateralSampling(params)
        self.longitudinal_sampling = LongitudinalSampling(params)
        self.coordinate_transformation = CoordinateTransformation(params)
        self.trajectory_checks = TrajectoryChecks(params)
        self.calculation_costs = CalculationCosts(params)

        # Core parameters
        self.planning_horizon = params.get('planning_horizon', 4.0)
        self.dt = params.get('dt', 0.1)
        self.num_points = int(self.planning_horizon / self.dt) + 1
        self.time_points = np.linspace(
            0, self.planning_horizon, self.num_points)

    def plan_trajectory(self, current_state: Dict, raceline_data: Dict,
                        track_data: Dict, obstacles: List = None) -> Optional[FrenetTrajectory]:
        """
        Main trajectory planning function - Complete TAM pipeline

        Args:
            current_state: Dict with 's', 'n', 's_dot', 'n_dot', 's_ddot', 'n_ddot'
            raceline_data: Dict with raceline information
            track_data: Dict with track geometry
            obstacles: List of obstacle information

        Returns:
            Optimal FrenetTrajectory or None if no valid trajectory found
        """

        if obstacles is None:
            obstacles = []

        try:
            # Extract current state
            s_start = current_state['s']
            n_start = current_state['n']
            s_dot_start = current_state['s_dot']
            n_dot_start = current_state['n_dot']
            s_ddot_start = current_state.get('s_ddot', 0.0)
            n_ddot_start = current_state.get('n_ddot', 0.0)

            # Generate longitudinal samples first
            V_target = raceline_data.get(
                'V_target', raceline_data.get('s_dot', 10.0))
            V_max = self.params.get('max_speed', 20.0)

            (time_arrays, s_arrays, s_dot_arrays, s_ddot_arrays,
             s_end_values, s_dot_end_values) = self.longitudinal_sampling.calc_samples(
                s_start, s_dot_start, s_ddot_start, V_target, V_max, raceline_data, True)

            if len(s_arrays) == 0:
                return None

            # Generate lateral samples
            n_arrays, n_dot_arrays, n_ddot_arrays = self.lateral_sampling.calc_samples(
                s_start, s_dot_start, s_arrays, s_dot_arrays, s_ddot_arrays,
                s_end_values, s_dot_end_values, n_start, n_dot_start, n_ddot_start,
                time_arrays, raceline_data, track_data)

            # Generate and evaluate trajectory candidates
            candidate_trajectories = []

            for i in range(len(s_arrays)):
                for j in range(len(n_arrays)):
                    trajectory = self._create_trajectory(
                        time_arrays[i], s_arrays[i], s_dot_arrays[i], s_ddot_arrays[i],
                        n_arrays[j], n_dot_arrays[j], n_ddot_arrays[j], track_data)

                    # Validate trajectory
                    trajectory.valid = self.trajectory_checks.validate_trajectory(
                        trajectory, track_data, obstacles)

                    if trajectory.valid:
                        # Calculate cost
                        trajectory.cost = self.calculation_costs.calculate_cost(
                            trajectory, raceline_data, track_data, obstacles)
                        candidate_trajectories.append(trajectory)

            # Select best trajectory
            if not candidate_trajectories:
                return None

            best_trajectory = min(candidate_trajectories, key=lambda t: t.cost)
            return best_trajectory

        except Exception as e:
            print(f"TAM sampling planning failed: {e}")
            return None

    def _create_trajectory(self, t_array: np.ndarray, s_array: np.ndarray,
                           s_dot_array: np.ndarray, s_ddot_array: np.ndarray,
                           n_array: np.ndarray, n_dot_array: np.ndarray,
                           n_ddot_array: np.ndarray, track_data: Dict) -> FrenetTrajectory:
        """Create complete trajectory from samples"""

        trajectory = FrenetTrajectory()

        # Time and Frenet coordinates
        trajectory.t = t_array.tolist()
        trajectory.s = s_array.tolist()
        trajectory.s_dot = s_dot_array.tolist()
        trajectory.s_ddot = s_ddot_array.tolist()
        trajectory.n = n_array.tolist()
        trajectory.n_dot = n_dot_array.tolist()
        trajectory.n_ddot = n_ddot_array.tolist()

        # Transform to velocity frame
        chi_array, V_array, ax_vf_array, ay_vf_array, Omega_z_array = \
            self.coordinate_transformation.transform_to_velocity_frame(
                track_data, s_array, s_dot_array, s_ddot_array,
                n_array, n_dot_array, n_ddot_array)

        trajectory.chi = chi_array.tolist()
        trajectory.V = V_array.tolist()
        trajectory.ax_vf = ax_vf_array.tolist()
        trajectory.ay_vf = ay_vf_array.tolist()
        trajectory.Omega_z = Omega_z_array.tolist()

        # Transform to Cartesian coordinates
        x_array, y_array, psi_array = self.coordinate_transformation.frenet_to_cartesian(
            s_array, n_array, track_data)

        trajectory.x = x_array.tolist()
        trajectory.y = y_array.tolist()
        trajectory.psi = psi_array.tolist()

        return trajectory


# Utility functions for ROS1 integration
class TAMSamplingUtils:
    """Utility functions for coordinate transformations and data processing"""

    @staticmethod
    def ros_odom_to_frenet_state(odom_msg, track_centerline: np.ndarray) -> Dict:
        """Convert ROS Odometry message to Frenet state"""

        # Extract Cartesian state
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y

        # Simple Frenet conversion (would need proper track geometry)
        if len(track_centerline) > 0:
            distances = np.sqrt(
                (track_centerline[:, 0] - x)**2 + (track_centerline[:, 1] - y)**2)
            closest_idx = np.argmin(distances)

            s = float(closest_idx)  # Simplified
            n = distances[closest_idx] * np.sign(y)  # Simplified
            s_dot = np.sqrt(vx**2 + vy**2)  # Simplified
            n_dot = 0.0  # Simplified
        else:
            s, n, s_dot, n_dot = 0.0, 0.0, 0.0, 0.0

        return {
            's': s,
            'n': n,
            's_dot': s_dot,
            'n_dot': n_dot,
            's_ddot': 0.0,
            'n_ddot': 0.0,
            'x': x,
            'y': y
        }

    @staticmethod
    def waypoints_to_raceline_data(waypoints_msg) -> Dict:
        """Convert ROS waypoints to raceline data"""

        if len(waypoints_msg.wpnts) == 0:
            return {'n': 0.0, 's_dot': 10.0, 'V': 10.0}

        # Extract waypoint data
        s_coords = []
        n_coords = []
        velocities = []
        times = []

        for i, wpnt in enumerate(waypoints_msg.wpnts):
            s_coords.append(i * 1.0)  # Simplified s coordinate
            n_coords.append(0.0)  # Assume raceline at center
            # Use velocity field if available (scaled global waypoints usually carry v_mps)
            v = getattr(wpnt, 'v_mps', None)
            if v is None or (isinstance(v, float) and np.isnan(v)):
                # Fallback: use a reasonable default
                v = 10.0
            velocities.append(float(v))
            times.append(i * 0.1)  # Simplified time

        return {
            's_post': np.array(s_coords),
            'n_post': np.array(n_coords),
            's_dot_post': np.array(velocities),
            't_post': np.array(times),
            'n': 0.0,  # Raceline at centerline
            's_dot': np.mean(velocities),
            'V': np.mean(velocities),
            'V_target': max(velocities) if velocities else 10.0
        }

    @staticmethod
    def obstacles_to_tam_format(obstacles_msg) -> List[Dict]:
        """Convert ROS obstacles to TAM format"""

        obstacles = []
        for obs in obstacles_msg.obstacles:
            obstacles.append({
                'x': obs.pose.position.x,
                'y': obs.pose.position.y,
                'radius': max(obs.scale.x, obs.scale.y) / 2.0,
                'velocity': np.sqrt(obs.twist.linear.x**2 + obs.twist.linear.y**2)
            })

        return obstacles
