#!/usr/bin/env python3
"""
TAM Sampling Planner Core Algorithms
Ported from ROS2 TAM implementation to ROS1 for multi-car racing

This module contains the core sampling-based trajectory planning algorithms
adapted for the existing ROS1 multi-car racing architecture.
"""

import numpy as np
import scipy.interpolate as interp
from typing import List, Tuple, Dict, Optional
import time


class FrenetTrajectory:
    """Represents a trajectory in Frenet coordinates"""
    
    def __init__(self):
        self.s = []      # longitudinal position
        self.s_dot = []  # longitudinal velocity
        self.s_ddot = [] # longitudinal acceleration
        self.n = []      # lateral position
        self.n_dot = []  # lateral velocity
        self.n_ddot = [] # lateral acceleration
        self.t = []      # time
        self.x = []      # global x
        self.y = []      # global y
        self.V = []      # velocity magnitude
        self.chi = []    # heading relative to track
        self.ax = []     # x acceleration
        self.ay = []     # y acceleration
        self.cost = float('inf')
        self.valid = False


class TAMSamplingCore:
    """
    Core TAM Sampling Planner implementation
    Adapted from the ROS2 TAM sampling planner for ROS1 compatibility
    """
    
    def __init__(self, params: Dict):
        """Initialize the TAM sampling core with parameters"""
        self.params = params
        
        # Sampling parameters (adapted from TAM defaults)
        self.lateral_samples = params.get('lateral_samples', 15)
        self.longitudinal_samples = params.get('longitudinal_samples', 8)
        self.planning_horizon = params.get('planning_horizon', 4.0)  # seconds
        self.dt = params.get('dt', 0.1)  # time step
        
        # Vehicle constraints
        self.max_speed = params.get('max_speed', 20.0)  # m/s
        self.max_accel = params.get('max_accel', 8.0)   # m/s²
        self.max_lateral_accel = params.get('max_lateral_accel', 12.0)  # m/s²
        self.track_width = params.get('track_width', 3.0)  # m
        
        # Cost weights (adapted from TAM cost function)
        self.w_raceline = params.get('w_raceline', 3.5)
        self.w_velocity = params.get('w_velocity', 3.0)
        self.w_smoothness = params.get('w_smoothness', 1.0)
        self.w_obstacle = params.get('w_obstacle', 10000.0)
        self.w_lateral_jerk = params.get('w_lateral_jerk', 0.5)
        
        # Safety margins
        self.safety_margin_static = params.get('safety_margin_static', 0.5)  # m
        self.safety_margin_dynamic = params.get('safety_margin_dynamic', 1.0)  # m
        
        # Time discretization
        self.num_points = int(self.planning_horizon / self.dt) + 1
        self.time_points = np.linspace(0, self.planning_horizon, self.num_points)
    
    def generate_lateral_samples(self, current_n: float, current_n_dot: float, 
                                target_n: float = 0.0) -> List[np.ndarray]:
        """
        Generate lateral trajectory samples using quintic polynomials
        Following TAM's lateral sampling approach
        """
        lateral_trajectories = []
        
        # Generate lateral target positions around the raceline
        n_targets = np.linspace(-self.track_width/2 + 0.5, 
                               self.track_width/2 - 0.5, 
                               self.lateral_samples)
        
        for n_target in n_targets:
            try:
                # Quintic polynomial boundary conditions
                # Start: current_n, current_n_dot, 0 (assume zero lateral acceleration)
                # End: n_target, 0, 0 (target position with zero velocity and acceleration)
                
                # Solve quintic polynomial: n(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
                T = self.planning_horizon
                
                # Matrix for quintic polynomial constraints
                A = np.array([
                    [1, 0, 0, 0, 0, 0],           # n(0) = current_n
                    [0, 1, 0, 0, 0, 0],           # n'(0) = current_n_dot
                    [0, 0, 2, 0, 0, 0],           # n''(0) = 0
                    [1, T, T**2, T**3, T**4, T**5],      # n(T) = n_target
                    [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4], # n'(T) = 0
                    [0, 0, 2, 6*T, 12*T**2, 20*T**3]     # n''(T) = 0
                ])
                
                b = np.array([current_n, current_n_dot, 0, n_target, 0, 0])
                coeffs = np.linalg.solve(A, b)
                
                # Evaluate polynomial at time points
                n_traj = np.polyval(coeffs[::-1], self.time_points)
                n_dot_traj = np.polyval(np.polyder(coeffs[::-1]), self.time_points)
                n_ddot_traj = np.polyval(np.polyder(coeffs[::-1], 2), self.time_points)
                
                lateral_trajectories.append({
                    'n': n_traj,
                    'n_dot': n_dot_traj,
                    'n_ddot': n_ddot_traj,
                    'target_n': n_target
                })
                
            except np.linalg.LinAlgError:
                # Skip invalid polynomial solutions
                continue
        
        return lateral_trajectories
    
    def generate_longitudinal_samples(self, current_s: float, current_s_dot: float,
                                    raceline_velocity: np.ndarray) -> List[np.ndarray]:
        """
        Generate longitudinal velocity profiles
        Following TAM's longitudinal sampling with velocity targets
        """
        longitudinal_trajectories = []
        
        # Generate different velocity targets (conservative to aggressive)
        velocity_factors = np.linspace(0.7, 1.0, self.longitudinal_samples)
        
        for v_factor in velocity_factors:
            try:
                # Target velocity profile (scaled raceline velocity)
                v_target = raceline_velocity * v_factor
                v_target = np.clip(v_target, 0, self.max_speed)
                
                # Simple acceleration-limited velocity profile
                s_traj = [current_s]
                s_dot_traj = [current_s_dot]
                s_ddot_traj = []
                
                for i in range(1, self.num_points):
                    dt = self.dt
                    current_v = s_dot_traj[-1]
                    target_v = v_target[min(i, len(v_target)-1)]
                    
                    # Simple acceleration control towards target velocity
                    v_error = target_v - current_v
                    accel = np.clip(v_error / dt, -self.max_accel, self.max_accel)
                    
                    # Integrate kinematics
                    new_v = current_v + accel * dt
                    new_s = s_traj[-1] + new_v * dt
                    
                    s_traj.append(new_s)
                    s_dot_traj.append(new_v)
                    s_ddot_traj.append(accel)
                
                # Add final acceleration point
                s_ddot_traj.append(0.0)
                
                longitudinal_trajectories.append({
                    's': np.array(s_traj),
                    's_dot': np.array(s_dot_traj),
                    's_ddot': np.array(s_ddot_traj),
                    'v_factor': v_factor
                })
                
            except Exception as e:
                # Skip invalid longitudinal profiles
                continue
        
        return longitudinal_trajectories
    
    def combine_trajectories(self, lateral_samples: List, longitudinal_samples: List) -> List[FrenetTrajectory]:
        """
        Combine lateral and longitudinal samples into complete Frenet trajectories
        """
        combined_trajectories = []
        
        for lat_sample in lateral_samples:
            for lon_sample in longitudinal_samples:
                traj = FrenetTrajectory()
                
                # Combine samples
                traj.t = self.time_points.copy()
                traj.s = lon_sample['s'].copy()
                traj.s_dot = lon_sample['s_dot'].copy()
                traj.s_ddot = lon_sample['s_ddot'].copy()
                traj.n = lat_sample['n'].copy()
                traj.n_dot = lat_sample['n_dot'].copy()
                traj.n_ddot = lat_sample['n_ddot'].copy()
                
                # Calculate velocity magnitude
                traj.V = np.sqrt(traj.s_dot**2 + traj.n_dot**2)
                
                combined_trajectories.append(traj)
        
        return combined_trajectories
    
    def validate_trajectory(self, traj: FrenetTrajectory, obstacles: List) -> bool:
        """
        Validate trajectory against constraints (adapted from TAM checks)
        """
        # Check velocity limits
        if np.any(traj.V > self.max_speed):
            return False
        
        # Check acceleration limits  
        if np.any(np.abs(traj.s_ddot) > self.max_accel):
            return False
        
        # Check lateral acceleration limits
        lateral_accel = np.abs(traj.n_ddot)
        if np.any(lateral_accel > self.max_lateral_accel):
            return False
        
        # Check track boundaries (simplified)
        if np.any(np.abs(traj.n) > self.track_width/2):
            return False
        
        # TODO: Add obstacle collision checking
        # This would require obstacle prediction and collision detection
        
        return True
    
    def calculate_cost(self, traj: FrenetTrajectory, raceline_n: np.ndarray,
                      raceline_v: np.ndarray, obstacles: List) -> float:
        """
        Calculate trajectory cost using TAM-inspired multi-objective function
        """
        cost = 0.0
        
        # Raceline deviation cost
        n_deviation = np.abs(traj.n - raceline_n[:len(traj.n)])
        cost += self.w_raceline * np.sum(n_deviation)
        
        # Velocity cost (encourage higher speeds)
        velocity_diff = raceline_v[:len(traj.V)] - traj.V
        cost += self.w_velocity * np.sum(np.maximum(velocity_diff, 0)**2)
        
        # Smoothness cost (lateral jerk)
        if len(traj.n_ddot) > 1:
            lateral_jerk = np.diff(traj.n_ddot)
            cost += self.w_lateral_jerk * np.sum(lateral_jerk**2)
        
        # Longitudinal smoothness
        if len(traj.s_ddot) > 1:
            longitudinal_jerk = np.diff(traj.s_ddot)
            cost += self.w_smoothness * np.sum(longitudinal_jerk**2)
        
        # TODO: Add obstacle avoidance costs
        # This would penalize trajectories that come close to obstacles
        
        return cost
    
    def plan_trajectory(self, current_state: Dict, raceline: Dict, 
                       obstacles: List = None) -> Optional[FrenetTrajectory]:
        """
        Main trajectory planning function
        
        Args:
            current_state: Dict with 's', 'n', 's_dot', 'n_dot'
            raceline: Dict with 'n' and 'v' arrays for reference
            obstacles: List of obstacle information
        
        Returns:
            Optimal FrenetTrajectory or None if no valid trajectory found
        """
        if obstacles is None:
            obstacles = []
        
        # Extract current state
        current_s = current_state['s']
        current_n = current_state['n']
        current_s_dot = current_state['s_dot']
        current_n_dot = current_state['n_dot']
        
        # Generate samples
        lateral_samples = self.generate_lateral_samples(current_n, current_n_dot)
        
        # Create raceline velocity profile for this horizon
        raceline_v_profile = np.full(self.num_points, raceline.get('v', 10.0))
        if isinstance(raceline.get('v'), (list, np.ndarray)):
            raceline_v_profile = np.interp(self.time_points, 
                                         np.linspace(0, self.planning_horizon, len(raceline['v'])),
                                         raceline['v'])
        
        longitudinal_samples = self.generate_longitudinal_samples(
            current_s, current_s_dot, raceline_v_profile)
        
        # Combine into complete trajectories
        candidate_trajectories = self.combine_trajectories(lateral_samples, longitudinal_samples)
        
        # Validate and evaluate trajectories
        valid_trajectories = []
        for traj in candidate_trajectories:
            if self.validate_trajectory(traj, obstacles):
                traj.valid = True
                
                # Get raceline reference for cost calculation
                raceline_n_ref = np.full(len(traj.n), raceline.get('n', 0.0))
                if isinstance(raceline.get('n'), (list, np.ndarray)):
                    raceline_n_ref = np.interp(self.time_points[:len(traj.n)], 
                                             np.linspace(0, self.planning_horizon, len(raceline['n'])),
                                             raceline['n'])
                
                raceline_v_ref = raceline_v_profile[:len(traj.V)]
                traj.cost = self.calculate_cost(traj, raceline_n_ref, raceline_v_ref, obstacles)
                valid_trajectories.append(traj)
        
        # Select best trajectory
        if not valid_trajectories:
            return None
        
        best_trajectory = min(valid_trajectories, key=lambda t: t.cost)
        return best_trajectory


class TAMSamplingUtils:
    """Utility functions for coordinate transformations and data processing"""
    
    @staticmethod
    def frenet_to_cartesian(s: float, n: float, track_centerline: np.ndarray,
                           track_headings: np.ndarray) -> Tuple[float, float]:
        """Convert Frenet coordinates to Cartesian coordinates"""
        # Simplified conversion - in practice would use proper track geometry
        # This is a placeholder that assumes track_centerline is [(x,y), ...]
        if len(track_centerline) == 0:
            return 0.0, 0.0
        
        # Find closest point on centerline (simplified)
        idx = min(int(s) % len(track_centerline), len(track_centerline)-1)
        center_x, center_y = track_centerline[idx]
        heading = track_headings[idx] if idx < len(track_headings) else 0.0
        
        # Apply lateral offset
        x = center_x - n * np.sin(heading)
        y = center_y + n * np.cos(heading)
        
        return x, y
    
    @staticmethod
    def cartesian_to_frenet(x: float, y: float, track_centerline: np.ndarray) -> Tuple[float, float]:
        """Convert Cartesian coordinates to Frenet coordinates"""
        # Simplified conversion - find closest point and calculate offset
        if len(track_centerline) == 0:
            return 0.0, 0.0
        
        distances = np.sqrt((track_centerline[:, 0] - x)**2 + (track_centerline[:, 1] - y)**2)
        closest_idx = np.argmin(distances)
        
        s = float(closest_idx)  # Simplified s coordinate
        n = distances[closest_idx]  # Simplified lateral distance
        
        return s, n
