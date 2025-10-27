#!/usr/bin/env python3
"""
TAM Sampling Planner Node for ROS1 Multi-Car Racing
Complete integration of TAM algorithms with proper namespaced topics

This node integrates the complete TAM sampling algorithms into the existing 
multi-car racing architecture with full namespace support.
"""
import sys
import os

# Add the package src directory to Python path FIRST before any local imports
# This must be done before importing local modules
if __name__ == '__main__':
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)
else:
    # When executed as a module (e.g., through catkin relay)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)

import rospy
import numpy as np
from typing import List, Dict, Optional
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from dynamic_reconfigure.msg import Config
from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray, OpponentTrajectory

# Import TAM modules (must be after sys.path setup)
try:
    from tam_sampling_utils import TAMSamplingUtils
    from tam_sampling_core import LocalSamplingPlanner  # Actual class name in core
except ImportError as e:
    print(f"ERROR: Failed to import TAM modules: {e}")
    print(f"Current directory: {os.path.dirname(os.path.abspath(__file__))}")
    print(f"sys.path: {sys.path}")
    raise


# ROS message imports

# F110 custom message imports

# Import complete TAM sampling core


class TAMSamplingPlannerNode:
    """
    Complete TAM Sampling Planner ROS1 Node with Namespaced Topics

    Subscribes to (all topics automatically namespaced):
        - global_waypoints: Global racing line waypoints
        - global_waypoints_scaled: Speed-scaled global waypoints  
        - car_state/odom_frenet: Vehicle state in Frenet coordinates
        - perception/obstacles: Detected obstacles
        - dynamic_tam_sampling_tuner_node/parameter_updates: Dynamic reconfigure

    Publishes to (all topics automatically namespaced):
        - planner/avoidance/otwpnts: Planned trajectory for state machine
        - planner/avoidance/markers: Visualization markers
        - planner/avoidance/latency: Planning computation time
    """

    def __init__(self):
        """Initialize the complete TAM Sampling Planner node"""

        # Initialize node with automatic namespace handling
        self.name = "tam_sampling_planner_node"
        rospy.init_node(self.name)

        # Get car namespace for logging and topic namespacing
        self.car_namespace = rospy.get_namespace().strip('/')
        if self.car_namespace:
            self.log_name = f"[{self.car_namespace}_{self.name}]"
        else:
            self.log_name = f"[{self.name}]"

        # State variables
        self.obs = ObstacleArray()
        self.opponent_predictions = OpponentTrajectory()
        self.global_waypoints = WpntArray()
        self.global_waypoints_scaled = WpntArray()
        self.current_state = {
            's': 0.0, 'n': 0.0, 's_dot': 0.0, 'n_dot': 0.0,
            's_ddot': 0.0, 'n_ddot': 0.0, 'x': 0.0, 'y': 0.0, 'heading': 0.0
        }

        # Complete TAM planning parameters (from original TAM implementation)
        self.planning_params = {
            # Core sampling parameters
            'lateral_samples': 15,
            'longitudinal_samples': 8,
            'n_dense_samples': 5,
            'n_dense_min': -0.5,
            'n_dense_max': 0.5,
            'planning_horizon': 4.0,
            'dt': 0.1,

            # Vehicle constraints
            'max_speed': 20.0,
            'max_accel': 8.0,
            'max_lateral_accel': 12.0,
            'track_width': 3.0,

            # Longitudinal sampling parameters
            's_dot_end_min': 1.0,
            's_dot_discretization': 2.0,
            's_dot_max_positive_delta': 20.0,
            'v_sampling_scale': 1.1,
            'relative_s_dot_min_percentage': 0.5,

            # Cost function weights (TAM defaults)
            'raceline_cost_weight': 3.5,
            'velocity_cost_weight': 3.0,
            'friction_cost_weight': 5000.0,
            'curvature_cost_weight': 500000.0,
            'lateral_jerk_cost_weight': 0.5,
            'prediction_cost_weight': 100000.0,
            'collision_cost_weight': 100000000.0,

            # Safety parameters
            'safety_distance_track_left': 0.5,
            'safety_distance_track_right': 0.5,
            'safety_margin_static': 0.5,
            'safety_margin_dynamic': 1.0,
            'tube_width': 1.0,

            # Trajectory validation
            'kappa_thr': 0.1,
            'curvature_cost_threshold': 30.0,
            'increasing_rl_cost': True
        }

        # Update parameters from ROS parameter server and dynamic reconfigure
        self.update_dynamic_params()

        # Initialize F1Tenth-compatible TAM sampling core
        # NOTE: LocalSamplingPlanner uses rospy.get_param() internally
        # Enable debugging to see detailed path collision analysis
        self.tam_planner = LocalSamplingPlanner(
            node_monitor=False,
            load_from_params=True,
            debugging=True  # F1TENTH: Enable debugging for trajectory analysis
        )

        # Import CoordinateTransformation for WpntArray conversion
        from coordinate_transformation import CoordinateTransformation
        self.coordinate_transformation = CoordinateTransformation(
            use_f1tenth_mode=True)

        # Track handler for F1Tenth (initialized when global_waypoints received)
        self.track_handler = None

        # ROS parameters
        self.from_bag = rospy.get_param("/from_bag", False)
        self.measuring = rospy.get_param("/measure", False)
        self.lookahead = rospy.get_param("~lookahead", 15.0)

        # Track and raceline data cache
        self.track_data = {
            'centerline': np.array([]),
            'headings': np.array([]),
            's_coord': np.array([]),
            'omega_z': np.array([]),
            'd_omega_z': np.array([])
        }
        self.raceline_data = {}

        # Performance monitoring
        self.last_planning_time = 0.0
        self.planning_rate = rospy.Rate(20)  # 20 Hz matching other planners
        self.planning_count = 0

        # Initialize ROS interface with namespaced topics
        self.setup_ros_interface()

        rospy.loginfo(
            f"{self.log_name} Complete TAM Sampling Planner initialized with namespace: {self.car_namespace}")

    def setup_ros_interface(self):
        """Setup ROS subscribers and publishers with proper namespacing"""

        # Subscribers (topics are automatically namespaced by ROS)
        self.global_wp_sub = rospy.Subscriber(
            "global_waypoints", WpntArray, self.global_waypoints_callback, queue_size=1)
        self.global_wp_scaled_sub = rospy.Subscriber(
            "global_waypoints_scaled", WpntArray, self.global_waypoints_scaled_callback, queue_size=1)
        self.state_sub = rospy.Subscriber(
            "car_state/odom_frenet", Odometry, self.state_callback, queue_size=1)
        self.obstacles_sub = rospy.Subscriber(
            "perception/obstacles", ObstacleArray, self.obstacles_callback, queue_size=1)

        # TAM Custom Prediction Subscriber
        self.opponent_prediction_sub = rospy.Subscriber(
            "prediction/opponent_waypoints", OpponentTrajectory, self.opponent_prediction_callback, queue_size=1)

        # Dynamic reconfigure subscriber (only if not from bag)
        if not self.from_bag:
            self.dyn_reconfig_sub = rospy.Subscriber(
                "dynamic_tam_sampling_tuner_node/parameter_updates",
                Config, self.dynamic_params_callback, queue_size=1)

        # Publishers (topics are automatically namespaced by ROS)
        self.trajectory_pub = rospy.Publisher(
            "planner/avoidance/otwpnts", OTWpntArray, queue_size=1)
        self.markers_pub = rospy.Publisher(
            "planner/avoidance/markers", MarkerArray, queue_size=1)

        # Optional latency publisher for performance measurement
        if self.measuring:
            self.latency_pub = rospy.Publisher(
                "planner/avoidance/latency", Float32, queue_size=1)

        rospy.loginfo(
            f"{self.log_name} ROS interface setup complete with namespaced topics")

    def update_dynamic_params(self):
        """Update parameters from ROS parameter server (complete TAM parameters)"""

        # Core sampling parameters
        self.planning_params['lateral_samples'] = rospy.get_param(
            "~lateral_samples", 15)
        self.planning_params['longitudinal_samples'] = rospy.get_param(
            "~longitudinal_samples", 8)
        self.planning_params['planning_horizon'] = rospy.get_param(
            "~planning_horizon", 4.0)
        self.planning_params['n_dense_samples'] = rospy.get_param(
            "~n_dense_samples", 5)

        # Vehicle constraints
        self.planning_params['max_speed'] = rospy.get_param("~max_speed", 20.0)
        self.planning_params['max_accel'] = rospy.get_param("~max_accel", 8.0)
        self.planning_params['max_lateral_accel'] = rospy.get_param(
            "~max_lateral_accel", 12.0)

        # Longitudinal sampling parameters
        self.planning_params['s_dot_discretization'] = rospy.get_param(
            "~s_dot_discretization", 2.0)
        self.planning_params['v_sampling_scale'] = rospy.get_param(
            "~v_sampling_scale", 1.1)

        # Cost weights (TAM naming convention)
        self.planning_params['raceline_cost_weight'] = rospy.get_param(
            "~raceline_cost_weight", 3.5)
        self.planning_params['velocity_cost_weight'] = rospy.get_param(
            "~velocity_cost_weight", 3.0)
        self.planning_params['friction_cost_weight'] = rospy.get_param(
            "~friction_cost_weight", 5000.0)
        self.planning_params['curvature_cost_weight'] = rospy.get_param(
            "~curvature_cost_weight", 500000.0)
        self.planning_params['lateral_jerk_cost_weight'] = rospy.get_param(
            "~lateral_jerk_cost_weight", 0.5)
        self.planning_params['prediction_cost_weight'] = rospy.get_param(
            "~prediction_cost_weight", 100000.0)
        self.planning_params['collision_cost_weight'] = rospy.get_param(
            "~collision_cost_weight", 100000000.0)

        # Safety parameters
        self.planning_params['safety_distance_track_left'] = rospy.get_param(
            "~safety_distance_track_left", 0.5)
        self.planning_params['safety_distance_track_right'] = rospy.get_param(
            "~safety_distance_track_right", 0.5)
        self.planning_params['safety_margin_static'] = rospy.get_param(
            "~safety_margin_static", 0.5)
        self.planning_params['safety_margin_dynamic'] = rospy.get_param(
            "~safety_margin_dynamic", 1.0)

        # Trajectory validation parameters
        self.planning_params['kappa_thr'] = rospy.get_param("~kappa_thr", 0.1)
        self.planning_params['curvature_cost_threshold'] = rospy.get_param(
            "~curvature_cost_threshold", 30.0)

    def global_waypoints_callback(self, msg: WpntArray):
        """Process global waypoints and initialize F1Tenth track handler"""
        self.global_waypoints = msg

        if len(msg.wpnts) > 0:
            # Convert WpntArray to dictionary format for GlobalWaypointsTrackHandler
            try:
                from track_handler_global_waypoints import GlobalWaypointsTrackHandler

                # Convert ROS message to dictionary format
                waypoints_dict = {
                    'wpnts': [
                        {
                            's_m': wpnt.s_m,
                            'x_m': wpnt.x_m,
                            'y_m': wpnt.y_m,
                            'd_m': wpnt.d_m if hasattr(wpnt, 'd_m') else 0.0,
                            'd_left': wpnt.d_left if hasattr(wpnt, 'd_left') else 1.5,
                            'd_right': wpnt.d_right if hasattr(wpnt, 'd_right') else 1.5,
                            'psi_rad': wpnt.psi_rad,
                            'kappa_radpm': wpnt.kappa_radpm,
                            'vx_mps': wpnt.vx_mps if hasattr(wpnt, 'vx_mps') else 5.0
                        }
                        for wpnt in msg.wpnts
                    ]
                }

                self.track_handler = GlobalWaypointsTrackHandler(
                    waypoints_dict)

                # Update the track handler in the TAM planner core
                self.tam_planner.track_handler = self.track_handler

                rospy.loginfo(
                    f"{self.log_name} ✓ Track handler initialized with {len(msg.wpnts)} waypoints")
            except Exception as e:
                rospy.logerr(
                    f"{self.log_name} Failed to initialize track handler: {e}")
                self.track_handler = None

            # Legacy track data extraction (kept for compatibility)
            centerline = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in msg.wpnts])
            headings = np.array([wpnt.psi_rad for wpnt in msg.wpnts])
            s_coords = np.array([wpnt.s_m if hasattr(
                wpnt, 's_m') else i*1.0 for i, wpnt in enumerate(msg.wpnts)])

            # Calculate track curvature (omega_z)
            omega_z = np.zeros(len(headings))
            d_omega_z = np.zeros(len(headings))

            if len(headings) > 2:
                # Simple curvature calculation
                for i in range(1, len(headings)-1):
                    ds = s_coords[i+1] - s_coords[i-1]
                    if ds > 0:
                        omega_z[i] = (headings[i+1] - headings[i-1]) / ds

                # Calculate derivative of curvature
                for i in range(1, len(omega_z)-1):
                    ds = s_coords[i+1] - s_coords[i-1]
                    if ds > 0:
                        d_omega_z[i] = (omega_z[i+1] - omega_z[i-1]) / ds

            # Update track data cache (legacy)
            self.track_data = {
                'centerline': centerline,
                'headings': headings,
                's_coord': s_coords,
                'omega_z': omega_z,
                'd_omega_z': d_omega_z
            }

            rospy.loginfo_throttle(
                5, f"{self.log_name} Track data updated: {len(msg.wpnts)} waypoints")

    def global_waypoints_scaled_callback(self, msg: WpntArray):
        """Process scaled global waypoints and extract raceline data (LEGACY - not used in F1Tenth)"""
        self.global_waypoints_scaled = msg

        # COMMENTED OUT - F1Tenth uses GlobalWaypointsTrackHandler instead of raceline_data
        # self.raceline_data = TAMSamplingUtils.waypoints_to_raceline_data(msg)
        # rospy.loginfo_throttle(
        #     5, f"{self.log_name} Raceline data updated with {len(msg.wpnts)} points")

    def state_callback(self, msg: Odometry):
        """Process vehicle state in Frenet coordinates"""

        # Primary state from Frenet odometry
        self.current_state['s'] = msg.pose.pose.position.x
        self.current_state['n'] = msg.pose.pose.position.y
        self.current_state['s_dot'] = msg.twist.twist.linear.x
        self.current_state['n_dot'] = msg.twist.twist.linear.y

        # Additional state information (if available)
        if hasattr(msg.pose, 'covariance') and len(msg.pose.covariance) > 0:
            self.current_state['s_ddot'] = msg.pose.covariance[0] if msg.pose.covariance[0] != 0 else 0.0
            self.current_state['n_ddot'] = msg.pose.covariance[1] if msg.pose.covariance[1] != 0 else 0.0

        # Cartesian fields are not directly provided in Frenet odom; keep previous values
        self.current_state['x'] = self.current_state.get('x', 0.0)
        self.current_state['y'] = self.current_state.get('y', 0.0)

        # Extract heading from quaternion
        try:
            from tf.transformations import euler_from_quaternion
            orientation = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])
            self.current_state['heading'] = yaw
        except:
            self.current_state['heading'] = 0.0

    def obstacles_callback(self, msg: ObstacleArray):
        """Process detected obstacles"""
        self.obs = msg
        rospy.loginfo_throttle(
            10, f"{self.log_name} Obstacles updated: {len(msg.obstacles)} detected")

    def opponent_prediction_callback(self, msg: OpponentTrajectory):
        """Process opponent trajectory predictions from TAM custom predictor"""
        self.opponent_predictions = msg
        rospy.loginfo_throttle(
            5, f"{self.log_name} Predictions updated: {len(msg.oppwpnts)} predicted opponent waypoints")

    def dynamic_params_callback(self, msg: Config):
        """Handle dynamic reconfigure parameter updates (LEGACY - F1Tenth uses rospy params directly)"""

        # COMMENTED OUT - F1Tenth reads parameters directly from rospy.get_param()
        # For dynamic reconfiguration, restart the node or use rosparam set

        # # Update parameters from dynamic reconfigure
        # for param in msg.doubles:
        #     # Get parameter name without namespace
        #     param_name = param.name.split('.')[-1]
        #     if param_name in self.planning_params:
        #         old_value = self.planning_params[param_name]
        #         self.planning_params[param_name] = param.value
        #         rospy.loginfo(
        #             f"{self.log_name} Updated {param_name}: {old_value} -> {param.value}")
        #
        # for param in msg.ints:
        #     param_name = param.name.split('.')[-1]
        #     if param_name in self.planning_params:
        #         old_value = self.planning_params[param_name]
        #         self.planning_params[param_name] = param.value
        #         rospy.loginfo(
        #             f"{self.log_name} Updated {param_name}: {old_value} -> {param.value}")
        #
        # # Reinitialize planner with new parameters
        # self.tam_planner = TAMSamplingCore(self.planning_params)
        # rospy.loginfo(
        #     f"{self.log_name} TAM planner reinitialized with new parameters")

        rospy.logwarn_throttle(
            10, f"{self.log_name} Dynamic reconfigure not supported in F1Tenth mode. Use rosparam set and restart node.")

    def process_obstacles(self) -> List[Dict]:
        """Convert ROS obstacles to TAM format, including predictions"""

        # Process static and dynamic obstacles
        obstacles = TAMSamplingUtils.obstacles_to_tam_format(self.obs)

        # Process predictions if available
        if len(self.opponent_predictions.oppwpnts) > 0:
            predicted_obstacles = self.process_predictions()
            obstacles.extend(predicted_obstacles)

        return obstacles

    def process_predictions(self) -> List[Dict]:
        """Convert opponent predictions to TAM obstacle format"""

        predicted_obstacles = []

        try:
            # Group prediction waypoints by time/distance
            prediction_groups = self.group_predictions_by_time()

            for time_step, waypoints in prediction_groups.items():
                for waypoint in waypoints:
                    # Convert predicted waypoint to TAM obstacle format
                    predicted_obstacle = {
                        's': waypoint.s_m,
                        'd': waypoint.d_m,
                        'velocity_s': waypoint.proj_vs_mps,
                        'velocity_d': 0.0,  # Lateral velocity not provided
                        'radius': 0.25,  # Default vehicle radius
                        'is_static': False,
                        'is_visible': True,
                        'is_predicted': True,
                        'prediction_time': time_step
                    }
                    predicted_obstacles.append(predicted_obstacle)

        except Exception as e:
            rospy.logwarn_throttle(
                5.0, f"{self.log_name} Error processing predictions: {e}")

        return predicted_obstacles

    def group_predictions_by_time(self) -> Dict:
        """Group prediction waypoints by time steps for trajectory sampling"""

        prediction_groups = {}

        # Simple grouping - could be enhanced with more sophisticated time binning
        dt = 0.1  # 100ms time steps

        for i, waypoint in enumerate(self.opponent_predictions.oppwpnts):
            time_step = i * dt
            if time_step not in prediction_groups:
                prediction_groups[time_step] = []
            prediction_groups[time_step].append(waypoint)

        return prediction_groups

    def map_current_state_to_state_estimate(self) -> Dict:
        """
        Map F1TENTH current_state format to TAM state_estimate format.

        Converts from:
            current_state: {s, n, s_dot, n_dot, s_ddot, n_ddot, x, y, heading}
        To:
            state_estimate: {x_current, y_current, z_current, psi_current, vel_current, s, n}
        """
        # Calculate total velocity from Frenet velocities
        vel_current = np.sqrt(self.current_state['s_dot']**2 +
                              self.current_state['n_dot']**2)

        # Convert current_state to state_estimate format
        state_estimate = {
            'x_current': self.current_state['x'],
            'y_current': self.current_state['y'],
            'z_current': 0.0,  # F1TENTH is 2D planar
            'psi_current': self.current_state['heading'],
            'vel_current': vel_current,
            's': self.current_state['s'],
            'n': self.current_state['n']
        }

        return state_estimate

    def get_raceline_from_global_waypoints(self) -> Dict:
        """
        Extract raceline dictionary from global waypoints for calc_trajectory.

        Returns:
            Dictionary with 'wpnts' key containing global waypoints
        """
        # Return waypoints in expected format for postprocess_raceline
        return {'wpnts': self.global_waypoints.wpnts}

    def create_planning_requests(self) -> Dict:
        """
        Create planning_requests dictionary with required parameters.

        Returns:
            Dictionary with planning request parameters
        """
        planning_requests = {
            'V_max': rospy.get_param('max_speed', self.planning_params.get('max_speed', 10.0)),
            'following_distance': rospy.get_param('following_distance', 10.0),
            'overtaking_allowed': rospy.get_param('overtaking_allowed', True),
            'role': rospy.get_param('role', 0),  # 0 = normal racing
            'gg_scaling': rospy.get_param('gg_scaling', 1.0)
        }

        return planning_requests

    # ========== LEGACY VISUALIZATION METHODS - COMMENTED OUT FOR F1TENTH ==========
    # These methods work with FrenetTrajectory objects from old TAM
    # Use create_f1tenth_visualization_markers() instead

    # def create_trajectory_message(self, trajectory: FrenetTrajectory) -> OTWpntArray:
    #     """Convert TAM trajectory to ROS message format (LEGACY - not used in F1Tenth)"""
    #     msg = OTWpntArray()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.header.frame_id = "map"
    #
    #     # Convert trajectory points to waypoints
    #     min_length = min(len(trajectory.t), len(trajectory.x),
    #                      len(trajectory.y), len(trajectory.V))
    #
    #     for i in range(min_length):
    #         wpnt = Wpnt()
    #         wpnt.x_m = trajectory.x[i]
    #         wpnt.y_m = trajectory.y[i]
    #         wpnt.psi_rad = trajectory.psi[i] if i < len(
    #             trajectory.psi) else 0.0
    #         wpnt.s_m = trajectory.s[i] if i < len(
    #             trajectory.s) else trajectory.V[i] * 0.1  # Fallback
    #         wpnt.d_m = trajectory.n[i] if i < len(trajectory.n) else 0.0
    #         wpnt.v_mps = trajectory.V[i]
    #
    #         # Additional information
    #         if i < len(trajectory.ax_vf):
    #             wpnt.ax_mps2 = trajectory.ax_vf[i]
    #         if i < len(trajectory.ay_vf):
    #             wpnt.ay_mps2 = trajectory.ay_vf[i]
    #
    #         msg.wpnts.append(wpnt)
    #
    #     rospy.loginfo_throttle(
    #         1, f"{self.log_name} Created trajectory message with {len(msg.wpnts)} points")
    #     return msg

    # def create_visualization_markers(self, trajectory: FrenetTrajectory) -> MarkerArray:
    #     """Create comprehensive visualization markers for the planned trajectory (LEGACY)"""
    #     marker_array = MarkerArray()
    #
    #     if len(trajectory.x) == 0:
    #         return marker_array
    #
    #     # Main trajectory line
    #     trajectory_marker = Marker()
    #     trajectory_marker.header.frame_id = "map"
    #     trajectory_marker.header.stamp = rospy.Time.now()
    #     trajectory_marker.ns = f"{self.car_namespace}_tam_trajectory" if self.car_namespace else "tam_trajectory"
    #     trajectory_marker.id = 0
    #     trajectory_marker.type = Marker.LINE_STRIP
    #     trajectory_marker.action = Marker.ADD
    #     trajectory_marker.scale.x = 0.15  # Line width
    #     trajectory_marker.color.a = 1.0
    #     trajectory_marker.color.r = 0.0
    #     trajectory_marker.color.g = 1.0
    #     trajectory_marker.color.b = 0.0
    #
    #     # Add trajectory points
    #     for i in range(len(trajectory.x)):
    #         point = Point()
    #         point.x = trajectory.x[i]
    #         point.y = trajectory.y[i]
    #         point.z = 0.1
    #         trajectory_marker.points.append(point)
    #
    #     marker_array.markers.append(trajectory_marker)
    #
    #     # Velocity visualization
    #     if len(trajectory.V) > 0:
    #         velocity_marker = Marker()
    #         velocity_marker.header.frame_id = "map"
    #         velocity_marker.header.stamp = rospy.Time.now()
    #         velocity_marker.ns = f"{self.car_namespace}_tam_velocity" if self.car_namespace else "tam_velocity"
    #         velocity_marker.id = 1
    #         velocity_marker.type = Marker.TEXT_VIEW_FACING
    #         velocity_marker.action = Marker.ADD
    #         velocity_marker.scale.z = 0.5
    #         velocity_marker.color.a = 1.0
    #         velocity_marker.color.r = 1.0
    #         velocity_marker.color.g = 1.0
    #         velocity_marker.color.b = 0.0
    #
    #         # Show velocity at trajectory start
    #         if len(trajectory.x) > 0:
    #             velocity_marker.pose.position.x = trajectory.x[0]
    #             velocity_marker.pose.position.y = trajectory.y[0]
    #             velocity_marker.pose.position.z = 1.0
    #             velocity_marker.text = f"TAM: {trajectory.V[0]:.1f} m/s\nCost: {trajectory.cost:.1f}"
    #             marker_array.markers.append(velocity_marker)
    #
    #     return marker_array

    def create_f1tenth_visualization_markers(self, trajectory_dict: Dict, wpnt_array: WpntArray) -> MarkerArray:
        """Create F1Tenth visualization markers from trajectory dict and WpntArray"""
        marker_array = MarkerArray()

        # Extract Cartesian coordinates from trajectory_dict using track_handler
        try:
            # Get x, y from WpntArray (already converted)
            if len(wpnt_array.wpnts) == 0:
                return marker_array

            # Main trajectory line
            trajectory_marker = Marker()
            trajectory_marker.header.frame_id = "map"
            trajectory_marker.header.stamp = rospy.Time.now()
            trajectory_marker.ns = f"{self.car_namespace}_tam_trajectory" if self.car_namespace else "tam_trajectory"
            trajectory_marker.id = 0
            trajectory_marker.type = Marker.LINE_STRIP
            trajectory_marker.action = Marker.ADD
            trajectory_marker.scale.x = 0.15

            # Color: green for normal, red for emergency
            if trajectory_dict.get('emergency', False):
                trajectory_marker.color.r = 1.0
                trajectory_marker.color.g = 0.0
                trajectory_marker.color.b = 0.0
            else:
                trajectory_marker.color.r = 0.0
                trajectory_marker.color.g = 1.0
                trajectory_marker.color.b = 0.0
            trajectory_marker.color.a = 1.0

            # Add trajectory points
            for wpnt in wpnt_array.wpnts:
                point = Point()
                point.x = wpnt.x_m
                point.y = wpnt.y_m
                point.z = 0.1
                trajectory_marker.points.append(point)

            marker_array.markers.append(trajectory_marker)

            # Velocity text marker
            if len(wpnt_array.wpnts) > 0:
                velocity_marker = Marker()
                velocity_marker.header.frame_id = "map"
                velocity_marker.header.stamp = rospy.Time.now()
                velocity_marker.ns = f"{self.car_namespace}_tam_velocity" if self.car_namespace else "tam_velocity"
                velocity_marker.id = 1
                velocity_marker.type = Marker.TEXT_VIEW_FACING
                velocity_marker.action = Marker.ADD
                velocity_marker.scale.z = 0.5
                velocity_marker.color.a = 1.0
                velocity_marker.color.r = 1.0
                velocity_marker.color.g = 1.0
                velocity_marker.color.b = 0.0

                velocity_marker.pose.position.x = wpnt_array.wpnts[0].x_m
                velocity_marker.pose.position.y = wpnt_array.wpnts[0].y_m
                velocity_marker.pose.position.z = 1.0

                cost = trajectory_dict.get('cost', 0.0)
                v_start = wpnt_array.wpnts[0].vx_mps
                status = "EMERGENCY" if trajectory_dict.get(
                    'emergency', False) else f"Cost: {cost:.2f}"
                velocity_marker.text = f"TAM: {v_start:.1f} m/s\n{status}"

                marker_array.markers.append(velocity_marker)

        except Exception as e:
            rospy.logwarn_throttle(
                5, f"{self.log_name} Visualization error: {e}")

        return marker_array

    def run_planning_cycle(self):
        """Execute one complete F1Tenth TAM planning cycle"""

        planning_start_time = time.time()

        # Check if we have required data for F1Tenth
        if (len(self.global_waypoints.wpnts) == 0 or
                self.track_handler is None):
            rospy.logwarn_throttle(
                5, f"{self.log_name} Missing required data for planning (track_handler or waypoints)")
            return

        # Process obstacles - convert to prediction format if needed
        # For now, use empty prediction dict (obstacles integration can be added later)
        prediction_dict = {}

        # If we have opponent predictions, convert them
        if len(self.opponent_predictions.oppwpnts) > 0:
            prediction_dict = {'oppwpnts': self.opponent_predictions.oppwpnts}

        # Run F1Tenth TAM sampling planner
        try:
            # STEP 1: Map state and parameters to TAM format
            state_estimate = self.map_current_state_to_state_estimate()
            raceline = self.get_raceline_from_global_waypoints()
            planning_requests = self.create_planning_requests()

            # STEP 2: Plan trajectory using TAM calc_trajectory()
            # NOTE: calc_trajectory returns TUPLE of (perf_traj, emerg_traj, s_start, n_start, V_target)
            result = self.tam_planner.calc_trajectory(
                state_estimate=state_estimate,
                raceline=raceline,
                prediction=prediction_dict,
                planning_requests=planning_requests
            )

            # STEP 3: Unpack return values (handle tuple return)
            if result is not None:
                # calc_trajectory returns tuple: (perf_traj, emerg_traj, s_start, n_start, V_target)
                if isinstance(result, tuple) and len(result) >= 2:
                    performance_traj = result[0]
                    emergency_traj = result[1]
                    trajectory_dict = performance_traj  # Use performance trajectory
                else:
                    # Fallback if unexpected format
                    trajectory_dict = result

                # Verify we have a valid trajectory
                if trajectory_dict and 's' in trajectory_dict and len(trajectory_dict['s']) > 0:
                    # STEP 4: Convert trajectory dict to WpntArray
                    wpnt_array = self.coordinate_transformation.convert_trajectory_to_wpnt_array(
                        trajectory=trajectory_dict,
                        track_handler=self.track_handler,
                        traj_cnt=self.planning_count
                    )

                    # STEP 5: Wrap in OTWpntArray for state machine compatibility
                    ot_msg = OTWpntArray()
                    ot_msg.wpnts = wpnt_array.wpnts  # Extract list of Wpnt objects
                    ot_msg.header.stamp = rospy.Time.now()
                    ot_msg.header.frame_id = "map"

                    # STEP 6: Publish trajectory
                    self.trajectory_pub.publish(ot_msg)

                    # STEP 7: Publish visualization markers (optional)
                    try:
                        markers_msg = self.create_f1tenth_visualization_markers(
                            trajectory_dict, wpnt_array)
                        self.markers_pub.publish(markers_msg)
                    except Exception as viz_e:
                        rospy.logdebug(
                            f"{self.log_name} Visualization error: {viz_e}")

                    self.planning_count += 1

                    # Log planning stats
                    is_emergency = trajectory_dict.get('emergency', False)
                    cost = trajectory_dict.get('cost', 0.0)
                    status = "EMERGENCY" if is_emergency else f"cost={cost:.2f}"

                    rospy.loginfo_throttle(
                        2, f"{self.log_name} Published trajectory #{self.planning_count}: {status}")
                else:
                    rospy.logwarn_throttle(
                        5, f"{self.log_name} Empty trajectory received from TAM planner")

            else:
                rospy.logwarn_throttle(
                    1, f"{self.log_name} No valid trajectory found - planning returned None")

        except Exception as e:
            rospy.logerr(f"{self.log_name} Planning failed: {str(e)}")
            import traceback
            traceback.print_exc()

        # Publish timing information if measuring
        if self.measuring:
            planning_time = time.time() - planning_start_time
            latency_msg = Float32()
            latency_msg.data = planning_time * 1000  # Convert to milliseconds
            self.latency_pub.publish(latency_msg)

            if planning_time > 0.05:  # Warn if planning takes more than 50ms
                rospy.logwarn(
                    f"{self.log_name} Slow planning cycle: {planning_time*1000:.1f}ms")

    def loop(self):
        """Main planning loop with proper error handling"""

        # Wait for critical messages
        rospy.loginfo(f"{self.log_name} Waiting for required messages...")
        rospy.loginfo(f"{self.log_name} Listening on topics:")
        rospy.loginfo(f"{self.log_name}   - global_waypoints")
        rospy.loginfo(f"{self.log_name}   - global_waypoints_scaled")
        rospy.loginfo(f"{self.log_name}   - car_state/odom_frenet")

        # Wait for messages with longer timeout and better feedback
        timeout_duration = 60.0  # Extended to 60 seconds for simulator startup

        try:
            rospy.loginfo(f"{self.log_name} Waiting for global_waypoints...")
            rospy.wait_for_message(
                "global_waypoints", WpntArray, timeout=timeout_duration)
            rospy.loginfo(f"{self.log_name} ✓ Received global_waypoints")

            rospy.loginfo(
                f"{self.log_name} Waiting for global_waypoints_scaled...")
            rospy.wait_for_message(
                "global_waypoints_scaled", WpntArray, timeout=timeout_duration)
            rospy.loginfo(
                f"{self.log_name} ✓ Received global_waypoints_scaled")

            rospy.loginfo(
                f"{self.log_name} Waiting for car_state/odom_frenet...")
            rospy.wait_for_message(
                "car_state/odom_frenet", Odometry, timeout=timeout_duration)
            rospy.loginfo(f"{self.log_name} ✓ Received car_state/odom_frenet")

        except rospy.ROSException as e:
            rospy.logerr(
                f"{self.log_name} Timeout waiting for required messages after {timeout_duration}s: {e}")
            rospy.logerr(f"{self.log_name} Please ensure:")
            rospy.logerr(f"{self.log_name}   1. F1Tenth simulator is running")
            rospy.logerr(
                f"{self.log_name}   2. Map/track waypoints are being published")
            rospy.logerr(
                f"{self.log_name}   3. Frenet conversion node is active")
            return

        rospy.loginfo(
            f"{self.log_name} ✓✓✓ All required messages received. Starting TAM planning loop.")

        # Main planning loop
        while not rospy.is_shutdown():
            try:
                self.run_planning_cycle()
                self.planning_rate.sleep()

            except rospy.ROSInterruptException:
                rospy.loginfo(f"{self.log_name} Planning loop interrupted")
                break
            except Exception as e:
                rospy.logerr(f"{self.log_name} Planning loop error: {str(e)}")
                rospy.sleep(0.1)  # Brief pause before retry


def main():
    """Main function with comprehensive error handling"""
    try:
        planner_node = TAMSamplingPlannerNode()
        planner_node.loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("TAM Sampling Planner node interrupted")
    except Exception as e:
        rospy.logerr(f"TAM Sampling Planner node failed: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
