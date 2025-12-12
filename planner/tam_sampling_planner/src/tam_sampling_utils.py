#!/usr/bin/env python3
"""
TAM Sampling Utils Module
Utility functions for coordinate transformations and data processing following original TAM approach

Ported from tam_race_stack/mod_planning/sampling_planner/
"""

import numpy as np
from typing import Dict, List
import math


class TAMSamplingUtils:
    """
    Utility functions for coordinate transformations and data processing.

    Provides static methods for:
    - Converting ROS messages to internal data formats
    - Frenet-Cartesian coordinate transformations
    - Trajectory visualization (ROS Path and MarkerArray)
    - Trajectory interpolation and validation

    All methods are static and do not require class instantiation.
    """

    @staticmethod
    def ros_odom_to_frenet_state(odom_msg, track_centerline: np.ndarray) -> Dict:
        """
        Convert ROS Odometry message to Frenet state.

        Performs simplified Frenet conversion by finding the closest point
        on the track centerline. For accurate conversion, use FrenetConverter.

        Args:
            odom_msg: ROS nav_msgs/Odometry message with pose and twist
            track_centerline: Nx2 array of (x, y) centerline coordinates

        Returns:
            dict: Frenet state with keys:
                - s: Arc length position [m] (simplified as index)
                - n: Lateral offset [m]
                - s_dot: Longitudinal velocity [m/s]
                - n_dot: Lateral velocity [m/s] (simplified to 0)
                - s_ddot, n_ddot: Accelerations (set to 0)
                - x, y: Original Cartesian position
        """

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
        """
        Convert ROS WpntArray message to postprocessed raceline format.

        Extracts velocity and position data from waypoints and creates
        arrays compatible with the longitudinal/lateral sampling modules.

        Args:
            waypoints_msg: ROS WpntArray message with wpnts list

        Returns:
            dict: Raceline data with keys:
                - s_post: Arc length array [m]
                - n_post: Lateral offset array [m] (zeros for centerline)
                - s_dot_post: Velocity array [m/s]
                - t_post: Time array [s]
                - n: Scalar lateral offset (0.0)
                - s_dot, V: Mean velocity [m/s]
                - V_target: Maximum velocity [m/s]
        """

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
        """
        Convert ROS ObstacleArray message to TAM internal format.

        Transforms F1Tenth Frenet-based obstacle data into dictionaries
        compatible with the TAM prediction and collision checking modules.

        Args:
            obstacles_msg: ROS ObstacleArray message with Frenet coordinates

        Returns:
            list[dict]: List of obstacle dictionaries with keys:
                - s: Longitudinal position along track [m]
                - d: Lateral offset from centerline [m]
                - radius: Obstacle radius [m]
                - velocity_s: Longitudinal velocity [m/s]
                - velocity_d: Lateral velocity [m/s]
                - is_static: Boolean flag for static obstacles
                - is_visible: Boolean flag for visibility
        """

        obstacles = []
        for obs in obstacles_msg.obstacles:
            # F1Tenth obstacles use Frenet coordinates (s, d) not Cartesian (x, y)
            obstacles.append({
                's': obs.s_center,  # Longitudinal position along track
                'd': obs.d_center,  # Lateral offset from centerline
                'radius': obs.size / 2.0,  # Obstacle size
                'velocity_s': obs.vs,  # Velocity in longitudinal direction
                'velocity_d': obs.vd,  # Velocity in lateral direction
                'is_static': obs.is_static,
                'is_visible': obs.is_visible
            })

        return obstacles

    @staticmethod
    def frenet_trajectory_to_ros_path(trajectory, frame_id: str = "map"):
        """
        Convert FrenetTrajectory to ROS Path message for visualization.

        Creates a nav_msgs/Path message from trajectory Cartesian coordinates.
        Heading is converted to quaternion orientation.

        Args:
            trajectory: Trajectory object with x, y, heading arrays
            frame_id: TF frame ID for the path header

        Returns:
            nav_msgs/Path: ROS Path message with PoseStamped waypoints
        """
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped

        path = Path()
        path.header.frame_id = frame_id

        if hasattr(trajectory, 'x') and hasattr(trajectory, 'y') and hasattr(trajectory, 'heading'):
            for i in range(len(trajectory.x)):
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = frame_id
                pose_stamped.pose.position.x = trajectory.x[i]
                pose_stamped.pose.position.y = trajectory.y[i]
                pose_stamped.pose.position.z = 0.0

                # Convert heading to quaternion
                if i < len(trajectory.heading):
                    heading = trajectory.heading[i]
                    pose_stamped.pose.orientation.z = math.sin(heading / 2.0)
                    pose_stamped.pose.orientation.w = math.cos(heading / 2.0)

                path.poses.append(pose_stamped)

        return path

    @staticmethod
    def frenet_trajectory_to_marker_array(trajectories: List, frame_id: str = "map", namespace: str = "tam_trajectories"):
        """
        Convert list of FrenetTrajectories to ROS MarkerArray for visualization.

        Creates LINE_STRIP markers for each trajectory with color coding:
        - Valid trajectories: Green (low cost) to Red (high cost)
        - Invalid trajectories: Transparent red

        Args:
            trajectories: List of trajectory objects with x, y arrays
            frame_id: TF frame ID for marker headers
            namespace: ROS marker namespace for grouping

        Returns:
            visualization_msgs/MarkerArray: Array of LINE_STRIP markers
        """
        from visualization_msgs.msg import MarkerArray, Marker
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        import rospy

        marker_array = MarkerArray()

        for i, trajectory in enumerate(trajectories):
            if not hasattr(trajectory, 'x') or len(trajectory.x) == 0:
                continue

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Set scale
            marker.scale.x = 0.05  # Line width

            # Set color based on trajectory validity and cost
            if hasattr(trajectory, 'valid') and trajectory.valid:
                if hasattr(trajectory, 'cost'):
                    # Color based on cost (green=low, red=high)
                    normalized_cost = min(1.0, trajectory.cost / 100.0)
                    marker.color.r = normalized_cost
                    marker.color.g = 1.0 - normalized_cost
                    marker.color.b = 0.0
                    marker.color.a = 0.8
                else:
                    # Default valid color (green)
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8
            else:
                # Invalid trajectory (red)
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.3

            # Add points
            for j in range(len(trajectory.x)):
                point = Point()
                point.x = trajectory.x[j]
                point.y = trajectory.y[j]
                point.z = 0.0
                marker.points.append(point)

            marker_array.markers.append(marker)

        return marker_array

    @staticmethod
    def frenet_state_to_cartesian(frenet_state: Dict, track_centerline: np.ndarray) -> Dict:
        """
        Convert Frenet state to Cartesian coordinates.

        Performs simplified Frenet-to-Cartesian conversion using the track
        centerline. Uses linear interpolation for position and velocities.

        Args:
            frenet_state: Dict with s, n, s_dot, n_dot keys
            track_centerline: Nx2 array of (x, y) centerline coordinates

        Returns:
            dict: Cartesian state with keys:
                - x, y: Position [m]
                - heading: Heading angle [rad]
                - vx, vy: Velocity components [m/s]
        """

        if len(track_centerline) == 0:
            return {
                'x': frenet_state.get('s', 0.0),
                'y': frenet_state.get('n', 0.0),
                'heading': 0.0,
                'vx': frenet_state.get('s_dot', 0.0),
                'vy': frenet_state.get('n_dot', 0.0)
            }

        # Simple conversion (would use spline interpolation in full implementation)
        s = frenet_state.get('s', 0.0)
        n = frenet_state.get('n', 0.0)

        # Find closest point on centerline
        s_idx = int(min(max(s, 0), len(track_centerline) - 1))

        if s_idx < len(track_centerline):
            center_x = track_centerline[s_idx, 0]
            center_y = track_centerline[s_idx, 1]

            # Calculate normal vector (simplified)
            if s_idx < len(track_centerline) - 1:
                dx = track_centerline[s_idx + 1, 0] - center_x
                dy = track_centerline[s_idx + 1, 1] - center_y
                normal_x = -dy
                normal_y = dx
                norm = math.sqrt(normal_x**2 + normal_y**2)
                if norm > 0:
                    normal_x /= norm
                    normal_y /= norm
            else:
                normal_x, normal_y = 0.0, 1.0

            # Apply lateral offset
            x = center_x + n * normal_x
            y = center_y + n * normal_y

            # Calculate heading (tangent to centerline)
            heading = math.atan2(dy, dx) if s_idx < len(
                track_centerline) - 1 else 0.0
        else:
            x, y, heading = 0.0, 0.0, 0.0

        # Convert velocities
        s_dot = frenet_state.get('s_dot', 0.0)
        n_dot = frenet_state.get('n_dot', 0.0)

        vx = s_dot * math.cos(heading) - n_dot * math.sin(heading)
        vy = s_dot * math.sin(heading) + n_dot * math.cos(heading)

        return {
            'x': x,
            'y': y,
            'heading': heading,
            'vx': vx,
            'vy': vy
        }

    @staticmethod
    def validate_trajectory_data(trajectory) -> bool:
        """
        Validate trajectory data for consistency.

        Checks that the trajectory has a time array and that all state
        arrays (s, n, s_dot, n_dot) have matching lengths.

        Args:
            trajectory: Trajectory object with t, s, n, s_dot, n_dot attributes

        Returns:
            bool: True if trajectory data is valid, False otherwise
        """

        if not hasattr(trajectory, 't') or len(trajectory.t) == 0:
            return False

        length = len(trajectory.t)

        # Check that all arrays have consistent lengths
        arrays_to_check = ['s', 'n', 's_dot', 'n_dot']
        for array_name in arrays_to_check:
            if hasattr(trajectory, array_name):
                array = getattr(trajectory, array_name)
                if len(array) != length:
                    return False

        return True

    @staticmethod
    def interpolate_trajectory_at_time(trajectory, target_time: float) -> Dict:
        """
        Interpolate trajectory state at a specific time.

        Performs linear interpolation between trajectory points to estimate
        the state at an arbitrary time. Handles boundary conditions by
        clamping to first/last point.

        Args:
            trajectory: Trajectory object with t, s, n, s_dot, n_dot arrays
            target_time: Time at which to interpolate [s]

        Returns:
            dict: Interpolated state with keys t, s, n, s_dot, n_dot, s_ddot, n_ddot.
                  Returns empty dict if trajectory validation fails.
        """

        if not TAMSamplingUtils.validate_trajectory_data(trajectory):
            return {}

        t_array = np.array(trajectory.t)

        # Check bounds
        if target_time <= t_array[0]:
            idx = 0
        elif target_time >= t_array[-1]:
            idx = len(t_array) - 1
        else:
            # Find interpolation indices
            idx = np.searchsorted(t_array, target_time)
            if idx > 0:
                idx -= 1

        # Interpolate or use exact value
        if idx < len(t_array) - 1 and target_time != t_array[idx]:
            # Linear interpolation
            t1, t2 = t_array[idx], t_array[idx + 1]
            alpha = (target_time - t1) / (t2 - t1)

            state = {'t': target_time}
            arrays_to_interpolate = [
                's', 'n', 's_dot', 'n_dot', 's_ddot', 'n_ddot']

            for array_name in arrays_to_interpolate:
                if hasattr(trajectory, array_name):
                    array = getattr(trajectory, array_name)
                    if len(array) > idx + 1:
                        value = array[idx] * \
                            (1 - alpha) + array[idx + 1] * alpha
                        state[array_name] = value
        else:
            # Use exact value
            state = {'t': target_time}
            arrays_to_copy = ['s', 'n', 's_dot', 'n_dot', 's_ddot', 'n_ddot']

            for array_name in arrays_to_copy:
                if hasattr(trajectory, array_name):
                    array = getattr(trajectory, array_name)
                    if len(array) > idx:
                        state[array_name] = array[idx]

        return state
