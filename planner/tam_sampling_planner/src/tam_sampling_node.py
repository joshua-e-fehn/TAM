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
from simple_helper_utils import interpolate_with_period
from typing import List, Dict, Optional
import time
import threading
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String, Bool
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
        - prediction/opponent_waypoints: Opponent trajectory predictions
        - state_machine: State machine state for race coordination

    Publishes to (all topics automatically namespaced):
        - planner/avoidance/otwpnts: Planned trajectory for state machine
        - planner/avoidance/markers: Visualization markers
        - planner/avoidance/all_samples: All sampled trajectories visualization
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

        # State machine state tracking
        self.state_machine_state = "GB_TRACK"  # Default to racing mode
        # Track if race has started (optimization: skip param updates during race)
        self.race_started = False

        # Predictive Sampler Mode: Conditional planning (only plan when needed, like SQP)
        # When ot_planner == "predictive_sampler", TAM only plans when obstacles nearby + in OT sector
        # When ot_planner == "tam_sampling", TAM plans continuously (original behavior)
        self.ot_planner = rospy.get_param(
            'state_machine/ot_planner', 'tam_sampling')
        self.conditional_planning_mode = (
            self.ot_planner == "predictive_sampler")

        # Conditional planning state (mirroring SQP behavior)
        self.ot_section_check = False  # Are we in overtaking sector?
        self.lookahead = 15.0  # meters - same as SQP
        self.obs_traj_thresh = 2.0  # meters - same as SQP
        self.track_length = 0.0  # Will be set from global waypoints

        if self.conditional_planning_mode:
            rospy.loginfo(
                f"{self.log_name} Running in PREDICTIVE SAMPLER mode - conditional planning enabled (plans only when obstacles nearby + in OT sector)")
        else:
            rospy.loginfo(
                f"{self.log_name} Running in TAM SAMPLING mode - continuous planning (always plans)")

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
        self.initialized_params = False
        self.declare_and_update_parameters()

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
        self.planning_rate = rospy.Rate(15)
        self.planning_count = 0

        # Planning cycle protection
        self.planning_in_progress = False
        self.planning_lock = threading.Lock()

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

        # State machine state subscriber for race start coordination
        self.state_machine_sub = rospy.Subscriber(
            "state_machine", String, self.state_machine_callback, queue_size=1)

        # Conditional planning: Subscribe to ot_section_check (only used in predictive_sampler mode)
        if self.conditional_planning_mode:
            rospy.Subscriber("ot_section_check", Bool,
                             self.ot_section_check_cb, queue_size=1)
            rospy.loginfo(
                f"{self.log_name} Subscribed to ot_section_check for conditional planning")

        # Publishers (topics are automatically namespaced by ROS)
        self.trajectory_pub = rospy.Publisher(
            "planner/avoidance/otwpnts", OTWpntArray, queue_size=1)
        self.markers_pub = rospy.Publisher(
            "planner/avoidance/markers", MarkerArray, queue_size=1)

        # Publisher for ALL sampled trajectories (for visualization/debugging)
        self.all_samples_pub = rospy.Publisher(
            "planner/avoidance/all_samples", MarkerArray, queue_size=1)

        # Optional latency publisher for performance measurement
        if self.measuring:
            self.latency_pub = rospy.Publisher(
                "planner/avoidance/latency", Float32, queue_size=1)

        rospy.loginfo(
            f"{self.log_name} ROS interface setup complete with namespaced topics")

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
                f"TAMSamplingNode: Could not load YAML defaults: {e}")
            return {}

    def declare_and_update_parameters(self):
        """Update parameters from ROS parameter server with YAML defaults as fallback"""
        # Performance optimization: Skip parameter updates after race has started
        if self.race_started:
            return

        if not self.initialized_params:
            yaml_defaults = self._load_yaml_defaults()

            # Core sampling parameters
            self.planning_params['lateral_samples'] = yaml_defaults.get(
                'lateral_samples', rospy.get_param("lateral_samples", 15))
            rospy.set_param("lateral_samples",
                            self.planning_params['lateral_samples'])
            self.planning_params['longitudinal_samples'] = yaml_defaults.get(
                'longitudinal_samples', rospy.get_param("longitudinal_samples", 8))
            rospy.set_param("longitudinal_samples",
                            self.planning_params['longitudinal_samples'])
            self.planning_params['planning_horizon'] = yaml_defaults.get(
                'planning_horizon', rospy.get_param("planning_horizon", 4.0))
            rospy.set_param("planning_horizon",
                            self.planning_params['planning_horizon'])
            self.planning_params['n_dense_samples'] = yaml_defaults.get(
                'n_dense_samples', rospy.get_param("n_dense_samples", 5))
            rospy.set_param("n_dense_samples",
                            self.planning_params['n_dense_samples'])

            # Vehicle constraints
            self.planning_params['max_speed'] = yaml_defaults.get(
                'max_speed', rospy.get_param("max_speed", 10.0))
            rospy.set_param("max_speed", self.planning_params['max_speed'])
            self.planning_params['max_accel'] = yaml_defaults.get(
                'max_accel', rospy.get_param("max_accel", 3.0))
            rospy.set_param("max_accel", self.planning_params['max_accel'])
            self.planning_params['max_lateral_accel'] = yaml_defaults.get(
                'max_lateral_accel', rospy.get_param("max_lateral_accel", 12.0))
            rospy.set_param("max_lateral_accel",
                            self.planning_params['max_lateral_accel'])

            # Longitudinal sampling parameters
            self.planning_params['s_dot_discretization'] = yaml_defaults.get(
                's_dot_discretization', rospy.get_param("s_dot_discretization", 2.0))
            rospy.set_param("s_dot_discretization",
                            self.planning_params['s_dot_discretization'])
            self.planning_params['v_sampling_scale'] = yaml_defaults.get(
                'v_sampling_scale', rospy.get_param("v_sampling_scale", 1.1))
            rospy.set_param("v_sampling_scale",
                            self.planning_params['v_sampling_scale'])

            # Cost weights (TAM naming convention)
            self.planning_params['raceline_cost_weight'] = yaml_defaults.get(
                'raceline_cost_weight', rospy.get_param("raceline_cost_weight", 3.5))
            rospy.set_param("raceline_cost_weight",
                            self.planning_params['raceline_cost_weight'])
            self.planning_params['velocity_cost_weight'] = yaml_defaults.get(
                'velocity_cost_weight', rospy.get_param("velocity_cost_weight", 3.0))
            rospy.set_param("velocity_cost_weight",
                            self.planning_params['velocity_cost_weight'])
            self.planning_params['friction_cost_weight'] = yaml_defaults.get(
                'friction_cost_weight', rospy.get_param("friction_cost_weight", 5000.0))
            rospy.set_param("friction_cost_weight",
                            self.planning_params['friction_cost_weight'])
            self.planning_params['curvature_cost_weight'] = yaml_defaults.get(
                'curvature_cost_weight', rospy.get_param("curvature_cost_weight", 500000.0))
            rospy.set_param("curvature_cost_weight",
                            self.planning_params['curvature_cost_weight'])
            self.planning_params['lateral_jerk_cost_weight'] = yaml_defaults.get(
                'lateral_jerk_cost_weight', rospy.get_param("lateral_jerk_cost_weight", 0.5))
            rospy.set_param("lateral_jerk_cost_weight",
                            self.planning_params['lateral_jerk_cost_weight'])
            self.planning_params['prediction_cost_weight'] = yaml_defaults.get(
                'prediction_cost_weight', rospy.get_param("prediction_cost_weight", 100000.0))
            rospy.set_param("prediction_cost_weight",
                            self.planning_params['prediction_cost_weight'])
            self.planning_params['collision_cost_weight'] = yaml_defaults.get(
                'collision_cost_weight', rospy.get_param("collision_cost_weight", 100000000.0))
            rospy.set_param("collision_cost_weight",
                            self.planning_params['collision_cost_weight'])

            # Safety parameters
            self.planning_params['safety_distance_track_left'] = yaml_defaults.get(
                'safety_distance_track_left', rospy.get_param("safety_distance_track_left", 0.02))
            rospy.set_param("safety_distance_track_left",
                            self.planning_params['safety_distance_track_left'])
            self.planning_params['safety_distance_track_right'] = yaml_defaults.get(
                'safety_distance_track_right', rospy.get_param("safety_distance_track_right", 0.02))
            rospy.set_param("safety_distance_track_right",
                            self.planning_params['safety_distance_track_right'])
            self.planning_params['safety_margin_static'] = yaml_defaults.get(
                'safety_margin_static', rospy.get_param("safety_margin_static", 0.2))
            rospy.set_param("safety_margin_static",
                            self.planning_params['safety_margin_static'])
            self.planning_params['safety_margin_dynamic'] = yaml_defaults.get(
                'safety_margin_dynamic', rospy.get_param("safety_margin_dynamic", 0.5))
            rospy.set_param("safety_margin_dynamic",
                            self.planning_params['safety_margin_dynamic'])

            # Trajectory validation parameters
            self.planning_params['kappa_thr'] = yaml_defaults.get(
                'kappa_thr', rospy.get_param("kappa_thr", 1.5))
            rospy.set_param("kappa_thr", self.planning_params['kappa_thr'])
            self.planning_params['curvature_cost_threshold'] = yaml_defaults.get(
                'curvature_cost_threshold', rospy.get_param("curvature_cost_threshold", 30.0))
            rospy.set_param("curvature_cost_threshold",
                            self.planning_params['curvature_cost_threshold'])

            # ROS parameters
            self.from_bag = rospy.get_param("from_bag", False)
            rospy.set_param("from_bag", self.from_bag)
            self.measuring = rospy.get_param("measure", True)
            rospy.set_param("measure", self.measuring)
            self.lookahead = yaml_defaults.get(
                'lookahead', rospy.get_param("lookahead", 15.0))
            rospy.set_param("lookahead", self.lookahead)

            self.following_distance = yaml_defaults.get(
                'following_distance', rospy.get_param('following_distance', 10.0))
            rospy.set_param('sfollowing_distance', self.following_distance)
            self.role = yaml_defaults.get(
                'role', rospy.get_param('role', 0))
            rospy.set_param('role', self.role)
            self.gg_scaling = yaml_defaults.get(
                'gg_scaling', rospy.get_param('gg_scaling', 1.0))
            rospy.set_param('gg_scaling', self.gg_scaling)
            self.overtaking_allowed = yaml_defaults.get(
                'overtaking_allowed', rospy.get_param('overtaking_allowed', True))
            rospy.set_param('overtaking_allowed', self.overtaking_allowed)

            self.initialized_params = True
        else:
            # Core sampling parameters
            self.planning_params['lateral_samples'] = rospy.get_param(
                "lateral_samples", self.planning_params['lateral_samples'])
            self.planning_params['longitudinal_samples'] = rospy.get_param(
                "longitudinal_samples", self.planning_params['longitudinal_samples'])
            self.planning_params['planning_horizon'] = rospy.get_param(
                "planning_horizon", self.planning_params['planning_horizon'])
            self.planning_params['n_dense_samples'] = rospy.get_param(
                "n_dense_samples", self.planning_params['n_dense_samples'])

            # Vehicle constraints
            self.planning_params['max_speed'] = rospy.get_param(
                "max_speed", self.planning_params['max_speed'])
            self.planning_params['max_accel'] = rospy.get_param(
                "max_accel", self.planning_params['max_accel'])
            self.planning_params['max_lateral_accel'] = rospy.get_param(
                "max_lateral_accel", self.planning_params['max_lateral_accel'])

            # Longitudinal sampling parameters
            self.planning_params['s_dot_discretization'] = rospy.get_param(
                "s_dot_discretization", self.planning_params['s_dot_discretization'])
            self.planning_params['v_sampling_scale'] = rospy.get_param(
                "v_sampling_scale", self.planning_params['v_sampling_scale'])

            # Cost weights (TAM naming convention)
            self.planning_params['raceline_cost_weight'] = rospy.get_param(
                "raceline_cost_weight", self.planning_params['raceline_cost_weight'])
            self.planning_params['velocity_cost_weight'] = rospy.get_param(
                "velocity_cost_weight", self.planning_params['velocity_cost_weight'])
            self.planning_params['friction_cost_weight'] = rospy.get_param(
                "friction_cost_weight", self.planning_params['friction_cost_weight'])
            self.planning_params['curvature_cost_weight'] = rospy.get_param(
                "curvature_cost_weight", self.planning_params['curvature_cost_weight'])
            self.planning_params['lateral_jerk_cost_weight'] = rospy.get_param(
                "lateral_jerk_cost_weight", self.planning_params['lateral_jerk_cost_weight'])
            self.planning_params['prediction_cost_weight'] = rospy.get_param(
                "prediction_cost_weight", self.planning_params['prediction_cost_weight'])
            self.planning_params['collision_cost_weight'] = rospy.get_param(
                "collision_cost_weight", self.planning_params['collision_cost_weight'])

            # Safety parameters
            self.planning_params['safety_distance_track_left'] = rospy.get_param(
                "safety_distance_track_left", self.planning_params['safety_distance_track_left'])
            self.planning_params['safety_distance_track_right'] = rospy.get_param(
                "safety_distance_track_right", self.planning_params['safety_distance_track_right'])
            self.planning_params['safety_margin_static'] = rospy.get_param(
                "safety_margin_static", self.planning_params['safety_margin_static'])
            self.planning_params['safety_margin_dynamic'] = rospy.get_param(
                "safety_margin_dynamic", self.planning_params['safety_margin_dynamic'])

            # Trajectory validation parameters
            self.planning_params['kappa_thr'] = rospy.get_param(
                "kappa_thr", self.planning_params['kappa_thr'])
            self.planning_params['curvature_cost_threshold'] = rospy.get_param(
                "curvature_cost_threshold", self.planning_params['curvature_cost_threshold'])

            # ROS parameters
            self.from_bag = rospy.get_param("from_bag", self.from_bag)
            self.measuring = rospy.get_param("measure", self.measuring)
            self.lookahead = rospy.get_param(
                "lookahead", self.lookahead)

            self.following_distance = rospy.get_param(
                'following_distance', self.following_distance)
            self.role = rospy.get_param('role',  self.role)
            self.gg_scaling = rospy.get_param('gg_scaling', self.gg_scaling)
            self.overtaking_allowed = rospy.get_param(
                'overtaking_allowed', self.overtaking_allowed)

    def global_waypoints_callback(self, msg: WpntArray):
        """Process global waypoints and initialize F1Tenth track handler"""
        self.global_waypoints = msg

        if len(msg.wpnts) > 0:
            # Store track length for conditional planning
            if len(msg.wpnts) > 0:
                self.track_length = msg.wpnts[-1].s_m
                if self.conditional_planning_mode:
                    rospy.loginfo_once(
                        f"{self.log_name} Track length set to {self.track_length:.2f}m for conditional planning")

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

                # rospy.loginfo(
                #     f"{self.log_name} ✓ Track handler initialized with {len(msg.wpnts)} waypoints")
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

            # rospy.loginfo_throttle(
            #     5, f"{self.log_name} Track data updated: {len(msg.wpnts)} waypoints")

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

        # Convert Frenet coordinates to Cartesian using track_handler
        if self.track_handler is not None:
            try:
                # Use sn2cartesian to convert (s, n) -> (x, y)
                # sn2cartesian expects scalar floats, returns (x, y) tuple
                x, y = self.track_handler.sn2cartesian(
                    float(self.current_state['s']),
                    float(self.current_state['n'])
                )
                self.current_state['x'] = x
                self.current_state['y'] = y
            except Exception as e:
                rospy.logerr_throttle(
                    2.0, f"{self.log_name} Failed to convert Frenet to Cartesian: {e}")
                import traceback
                rospy.logerr_throttle(2.0, traceback.format_exc())
                # Fallback: keep previous values
                self.current_state['x'] = self.current_state.get('x', 0.0)
                self.current_state['y'] = self.current_state.get('y', 0.0)
        else:
            # Track handler not yet initialized, keep previous values
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

        # Debug logging to verify state updates
        # rospy.logwarn_throttle(1.0,
        #                        f"{self.log_name} State: s={self.current_state['s']:.2f}, n={self.current_state['n']:.3f}, "
        #                        f"x={self.current_state['x']:.2f}, y={self.current_state['y']:.2f}, "
        #                        f"v={self.current_state['s_dot']:.2f} m/s")

    def obstacles_callback(self, msg: ObstacleArray):
        """Process detected obstacles"""
        self.obs = msg
        # rospy.loginfo_throttle(
        #     10, f"{self.log_name} Obstacles updated: {len(msg.obstacles)} detected")

    def opponent_prediction_callback(self, msg: OpponentTrajectory):
        """Process opponent trajectory predictions from TAM custom predictor"""
        self.opponent_predictions = msg
        # rospy.loginfo_throttle(
        #     5, f"{self.log_name} Predictions updated: {len(msg.oppwpnts)} predicted opponent waypoints")

    def state_machine_callback(self, msg: String):
        """Callback for state machine state - used to coordinate race start"""
        prev_state = self.state_machine_state
        self.state_machine_state = msg.data

        # Detect race start transition (READY -> any other state)
        if prev_state == "READY" and msg.data != "READY" and not self.race_started:
            self.race_started = True
            rospy.loginfo(
                f"{self.log_name} 🏁 Race started! Disabling parameter updates for performance.")

        # Log state transitions for debugging
        # rospy.loginfo_throttle(
        #     2.0, f"{self.log_name} State machine state: {self.state_machine_state}")

    def ot_section_check_cb(self, msg: Bool):
        """Callback for overtaking section check (used in predictive_sampler mode)"""
        self.ot_section_check = msg.data

    def _check_should_plan(self) -> bool:
        """
        Determine if TAM should execute planning cycle.

        In predictive_sampler mode: Only plan when obstacles nearby + in OT sector (mimics SQP behavior)
        In tam_sampling mode: Always plan (original TAM behavior)

        Returns:
            True if planning should execute, False otherwise
        """
        # If not in conditional planning mode, always plan
        if not self.conditional_planning_mode:
            return True

        # In predictive_sampler mode: Check conditions (same as SQP)
        # 1. Must be in overtaking sector
        if not self.ot_section_check:
            return False

        # 2. Must have obstacles within range
        if len(self.obs.obstacles) == 0:
            return False

        # 3. Filter obstacles by distance and trajectory proximity (same as SQP logic)
        cur_s = self.current_state['s']
        considered_obs = []

        for obs in self.obs.obstacles:
            # Calculate distance to obstacle (handling wraparound)
            dist_to_obs = (obs.s_start - cur_s) % self.track_length

            # Check if obstacle is within lookahead distance
            within_lookahead = dist_to_obs < self.lookahead

            # Check if obstacle is close to trajectory (lateral distance)
            traj_dist = abs(obs.d_center)
            within_traj_thresh = traj_dist < self.obs_traj_thresh

            if within_lookahead and within_traj_thresh:
                considered_obs.append(obs)

        # Only plan if we have obstacles that meet the criteria
        should_plan = len(considered_obs) > 0

        if not should_plan:
            rospy.loginfo_throttle(
                2.0, f"{self.log_name} Skipping planning: ot_sector={self.ot_section_check}, "
                f"obstacles_total={len(self.obs.obstacles)}, obstacles_considered={len(considered_obs)}")

        return should_plan

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

        Integrates with state machine to force stillstand when in READY state,
        ensuring the planner doesn't advance trajectories before race start.

        Returns:
            Dictionary with planning request parameters
        """
        # Check if state machine is in READY state - if so, force stopping/stillstand
        if self.state_machine_state == "READY":
            # Force V_max = 0 to trigger stopping -> stillstand transition in TAM planner
            planning_requests = {
                'V_max': 0.0,  # Forces TAM planner into stopping -> stillstand mode
                'following_distance': self.following_distance,
                'overtaking_allowed': False,  # No overtaking before race start
                'role': self.role,
                'gg_scaling': self.gg_scaling
            }
            rospy.loginfo_throttle(
                5.0, f"{self.log_name} State READY: Forcing V_max=0 (stillstand mode)")
        else:
            # Normal racing mode - use configured maximum speed
            planning_requests = {
                'V_max': self.planning_params['max_speed'],
                'following_distance': self.following_distance,
                'overtaking_allowed': self.overtaking_allowed,
                'role': self.role,  # 0 = normal racing
                'gg_scaling': self.gg_scaling
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

    def interpolate_trajectory_to_controller_spacing(
        self,
        wpnt_array: WpntArray,
        target_spacing: float = 0.1
    ) -> WpntArray:
        """
        Interpolate trajectory to dense controller spacing (0.1m default).

        This method follows the predictive spliner's proven approach:
        1. Calculate number of points needed based on target spacing
        2. Create dense s-array using np.linspace
        3. Interpolate all fields (d, vx, ax, etc.) to new spacing
        4. Convert (s,d) back to (x,y) using track geometry
        5. Recalculate heading, curvature, and track boundaries

        This is the LAST processing step before publishing to state machine.

        Args:
            wpnt_array: Sparse waypoint array from TAM planner (~10m spacing)
            target_spacing: Target spacing in meters (default 0.1m for controller)

        Returns:
            Dense waypoint array with target_spacing between points
        """

        # Check if input is valid
        if not wpnt_array or not wpnt_array.wpnts or len(wpnt_array.wpnts) < 2:
            rospy.logwarn(
                f"{self.log_name} Cannot interpolate: empty or single-point trajectory")
            return wpnt_array

        # Extract arrays from input waypoints
        s_sparse = np.array([w.s_m for w in wpnt_array.wpnts])
        d_sparse = np.array([w.d_m for w in wpnt_array.wpnts])
        vx_sparse = np.array([w.vx_mps for w in wpnt_array.wpnts])
        ax_sparse = np.array([w.ax_mps2 for w in wpnt_array.wpnts])

        # Get track length for wrap-around handling
        max_s = self.track_handler.s_coord()[-1]

        # Convert to continuous coordinates (unwrap s-values at wrap-around)
        s_continuous = s_sparse.copy()
        for i in range(1, len(s_continuous)):
            # If s decreases significantly (wrap-around detected)
            if s_continuous[i] < s_continuous[i-1] - max_s/2:
                # Unwrap by adding max_s to this and all following points
                s_continuous[i:] += max_s

        # Calculate trajectory length and number of points needed
        start_s_continuous = s_continuous[0]
        end_s_continuous = s_continuous[-1]
        trajectory_length = end_s_continuous - start_s_continuous

        if trajectory_length < 0:
            rospy.logwarn(
                f"{self.log_name} Negative trajectory length after unwrapping: {trajectory_length:.2f}m")
            return wpnt_array

        # Calculate number of interpolated points
        n_dense_points = max(2, int(trajectory_length / target_spacing))

        # Create dense s-array in continuous coordinates
        s_dense_continuous = np.linspace(
            start_s_continuous, end_s_continuous, n_dense_points)

        # Interpolate lateral offset, velocity, and acceleration using CONTINUOUS coordinates
        d_dense = np.interp(s_dense_continuous, s_continuous, d_sparse)
        vx_dense = np.interp(s_dense_continuous, s_continuous, vx_sparse)
        ax_dense = np.interp(s_dense_continuous, s_continuous, ax_sparse)

        # Convert back to modulo coordinates for track geometry lookup
        s_dense_mod = np.mod(s_dense_continuous, max_s)

        # Convert Frenet (s,d) to Cartesian (x,y) using track geometry
        xyz_array = self.track_handler.sn2cartesian(s_dense_mod, d_dense)
        x_dense = xyz_array[:, 0]
        y_dense = xyz_array[:, 1]

        # Calculate 2D heading from track geometry
        # Note: chi = 0 for trajectory following centerline, so psi ≈ track heading
        # Assume following track for interpolated points
        chi_dense = np.zeros_like(s_dense_continuous)
        psi_dense = self.track_handler.calc_2d_heading_from_chi(
            s_dense_mod, chi_dense)

        # Interpolate track curvature
        kappa_dense = interpolate_with_period(
            s_dense_mod,
            self.track_handler.s_coord(),
            self.track_handler.kappa(),
            max_s
        )

        # Calculate track boundaries at interpolated points
        d_left_dense = self.track_handler.trackwidth_left(
            s_dense_mod).flatten() - d_dense
        d_right_dense = np.abs(self.track_handler.trackwidth_right(
            s_dense_mod).flatten() - d_dense)

        # Create new dense WpntArray
        dense_wpnt_array = WpntArray()
        dense_wpnt_array.wpnts = []

        for i in range(n_dense_points):
            wpnt = Wpnt()
            wpnt.id = i
            wpnt.s_m = float(s_dense_mod[i])
            wpnt.d_m = float(d_dense[i])
            wpnt.x_m = float(x_dense[i])
            wpnt.y_m = float(y_dense[i])
            wpnt.d_left = float(d_left_dense[i])
            wpnt.d_right = float(d_right_dense[i])
            wpnt.psi_rad = float(psi_dense[i])
            wpnt.kappa_radpm = float(kappa_dense[i])
            wpnt.vx_mps = float(vx_dense[i])
            wpnt.ax_mps2 = float(ax_dense[i])

            dense_wpnt_array.wpnts.append(wpnt)

        # Log interpolation result
        wrap_around = (s_continuous[-1] - s_continuous[0]) > max_s * 0.5
        wrap_info = " (wrap-around)" if wrap_around else ""
        # rospy.loginfo_throttle(
        #     5.0,
        #     f"{self.log_name} Interpolated trajectory{wrap_info}: {len(wpnt_array.wpnts)} points ({trajectory_length:.1f}m) → "
        #     f"{n_dense_points} points (spacing ~{target_spacing:.2f}m)"
        # )

        return dense_wpnt_array

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

    def create_all_samples_markers(self, s_array, n_array, valid_array, track_handler) -> MarkerArray:
        """
        Create visualization markers for ALL sampled trajectories from perform_trajectory_sampling

        Args:
            s_array: (N_traj, N_points) array of s-coordinates for all trajectories
            n_array: (N_traj, N_points) array of n-coordinates for all trajectories
            valid_array: (N_traj,) boolean array indicating valid (True) vs invalid (False) trajectories
            track_handler: Track handler object for Frenet->Cartesian conversion

        Returns:
            MarkerArray with one LINE_STRIP marker per trajectory (blue=valid, red=invalid)
        """
        marker_array = MarkerArray()

        # rospy.loginfo_throttle(
        #     2, f"{self.log_name} create_all_samples_markers called")

        if s_array is None or n_array is None or track_handler is None:
            rospy.logwarn_throttle(
                2, f"{self.log_name} create_all_samples_markers: missing inputs - s_array={s_array is not None}, n_array={n_array is not None}, track_handler={track_handler is not None}")
            return marker_array

        if len(s_array) == 0 or len(n_array) == 0:
            rospy.logwarn_throttle(
                2, f"{self.log_name} create_all_samples_markers: empty arrays - s_array.shape={s_array.shape}, n_array.shape={n_array.shape}")
            return marker_array

        try:
            num_trajectories = s_array.shape[0]
            # rospy.loginfo_throttle(
            #     2, f"{self.log_name} Processing {num_trajectories} trajectories for visualization")

            # Limit visualization to avoid performance issues (e.g., max 500 trajectories)
            max_viz_trajectories = 600
            if num_trajectories > max_viz_trajectories:
                # rospy.logwarn_throttle(10,
                #                        f"{self.log_name} Too many trajectories ({num_trajectories}), visualizing only first {max_viz_trajectories}")
                num_trajectories = max_viz_trajectories

            markers_created = 0
            points_created = 0

            for i in range(num_trajectories):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = f"{self.car_namespace}_tam_samples" if self.car_namespace else "tam_samples"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.02  # Thin lines

                # Color based on validity: blue for valid, red for invalid
                is_valid = valid_array[i] if valid_array is not None and i < len(
                    valid_array) else True
                if is_valid:
                    # Valid trajectory: semi-transparent blue
                    marker.color.r = 0.3
                    marker.color.g = 0.3
                    marker.color.b = 1.0
                    marker.color.a = 0.3
                else:
                    # Invalid trajectory: semi-transparent red
                    marker.color.r = 1.0
                    marker.color.g = 0.3
                    marker.color.b = 0.3
                    marker.color.a = 0.3

                # Convert Frenet (s, n) to Cartesian (x, y) for this trajectory
                s_traj = s_array[i, :]
                n_traj = n_array[i, :]

                # CRITICAL FIX: Wrap s-coordinates to [0, track_length) for sn2cartesian
                # sn2cartesian expects s in [0, track_length) and uses interpolate_with_period internally
                track_length = track_handler.s_coord()[-1]
                s_traj_wrapped = np.mod(s_traj, track_length)

                # Use track_handler to convert Frenet to global coordinates
                for j in range(len(s_traj_wrapped)):
                    try:
                        # Get x, y from track handler at (s, n)
                        # sn2cartesian uses interpolate_with_period which handles periodic wrapping
                        x, y = track_handler.sn2cartesian(
                            s_traj_wrapped[j], n_traj[j])

                        point = Point()
                        point.x = x
                        point.y = y
                        point.z = 0.05  # Slightly above ground
                        marker.points.append(point)
                        points_created += 1
                    except Exception as e:
                        # Skip invalid points (log only first few errors)
                        if i < 3 and j < 3:
                            rospy.logwarn_throttle(
                                10, f"{self.log_name} Frenet conversion error at traj {i}, point {j}: {e}")
                        continue

                # Only add marker if it has points
                if len(marker.points) > 0:
                    marker_array.markers.append(marker)
                    markers_created += 1
                elif i < 5:  # Log first few empty markers
                    rospy.logwarn_throttle(
                        5, f"{self.log_name} Marker {i} has no valid points (s range: [{s_traj[0]:.2f}, {s_traj[-1]:.2f}], n range: [{n_traj[0]:.2f}, {n_traj[-1]:.2f}])")

            # rospy.loginfo_throttle(
            #     2, f"{self.log_name} Created {markers_created} markers with {points_created} total points")

        except Exception as e:
            rospy.logerr_throttle(
                5, f"{self.log_name} Error creating all-samples markers: {e}")
            import traceback
            rospy.logerr_throttle(5, traceback.format_exc())

        return marker_array

    def _clear_visualization_markers(self):
        """
        Clear all visualization markers when not in active planning states.

        Publishes DELETE actions for trajectory and sample markers to remove them from RViz.
        This is called when state_machine_state is not in ['OVERTAKE', 'TAM_PLANNING'].
        """
        try:
            # Create marker array with DELETE actions
            marker_array = MarkerArray()

            # Delete main trajectory marker
            trajectory_marker = Marker()
            trajectory_marker.header.frame_id = "map"
            trajectory_marker.header.stamp = rospy.Time.now()
            trajectory_marker.ns = f"{self.car_namespace}_tam_trajectory" if self.car_namespace else "tam_trajectory"
            trajectory_marker.id = 0
            trajectory_marker.action = Marker.DELETE
            marker_array.markers.append(trajectory_marker)

            # Delete velocity text marker
            velocity_marker = Marker()
            velocity_marker.header.frame_id = "map"
            velocity_marker.header.stamp = rospy.Time.now()
            velocity_marker.ns = f"{self.car_namespace}_tam_velocity" if self.car_namespace else "tam_velocity"
            velocity_marker.id = 1
            velocity_marker.action = Marker.DELETE
            marker_array.markers.append(velocity_marker)

            # Publish deletion markers
            self.markers_pub.publish(marker_array)

            # Also clear all sample trajectory markers (up to max_viz_trajectories)
            samples_marker_array = MarkerArray()
            max_viz_trajectories = 600

            for i in range(max_viz_trajectories):
                sample_marker = Marker()
                sample_marker.header.frame_id = "map"
                sample_marker.header.stamp = rospy.Time.now()
                sample_marker.ns = f"{self.car_namespace}_tam_samples" if self.car_namespace else "tam_samples"
                sample_marker.id = i
                sample_marker.action = Marker.DELETE
                samples_marker_array.markers.append(sample_marker)

            self.all_samples_pub.publish(samples_marker_array)

        except Exception as e:
            rospy.logwarn_throttle(
                5.0, f"{self.log_name} Failed to clear visualization markers: {e}")

    def publish_stop_trajectory(self):
        """Publish a stop trajectory when TAM planning fails"""
        try:
            # Create a minimal stop trajectory at current position
            ot_msg = OTWpntArray()
            ot_msg.header.stamp = rospy.Time.now()
            ot_msg.header.frame_id = "map"

            # Create a single waypoint at current position with zero velocity
            stop_wpnt = Wpnt()
            stop_wpnt.x_m = self.current_state.get('x', 0.0)
            stop_wpnt.y_m = self.current_state.get('y', 0.0)
            stop_wpnt.psi_rad = self.current_state.get('heading', 0.0)
            stop_wpnt.s_m = self.current_state.get('s', 0.0)
            stop_wpnt.d_m = self.current_state.get('n', 0.0)
            stop_wpnt.vx_mps = 0.0  # Zero velocity = stop
            stop_wpnt.ax_mps2 = -3.0  # Light braking
            stop_wpnt.ay_mps2 = 0.0
            stop_wpnt.kappa_radpm = 0.0

            # Add multiple identical waypoints to create a stable stop trajectory
            for _ in range(10):
                ot_msg.wpnts.append(stop_wpnt)

            # Publish stop trajectory
            self.trajectory_pub.publish(ot_msg)

            rospy.logwarn_throttle(
                2, f"{self.log_name} Published STOP trajectory - TAM planning failed")

        except Exception as e:
            rospy.logerr(
                f"{self.log_name} Failed to publish stop trajectory: {e}")

    def run_planning_cycle(self):
        """Execute one complete F1Tenth TAM planning cycle"""

        # Check if a planning cycle is already running
        if self.planning_in_progress:
            rospy.logwarn_throttle(
                2.0, f"{self.log_name} ⚠️ Skipping planning cycle - previous cycle still in progress")
            return

        # PREDICTIVE SAMPLER MODE: Check if we should plan (only when obstacles + in OT sector)
        if not self._check_should_plan():
            self._clear_visualization_markers()
            # Publish empty trajectory (same as SQP does when conditions not met)
            empty_msg = OTWpntArray()
            empty_msg.header.stamp = rospy.Time.now()
            empty_msg.header.frame_id = "map"
            empty_msg.wpnts = []
            self.trajectory_pub.publish(empty_msg)
            return

        # Acquire lock to prevent concurrent planning
        with self.planning_lock:
            self.planning_in_progress = True
            try:
                self._execute_planning_cycle()
            finally:
                self.planning_in_progress = False

    def _execute_planning_cycle(self):
        """Internal method that does the actual planning work"""

        # Propagate race_started flag to child modules for parameter update optimization
        self.tam_planner._skip_param_updates = self.race_started
        if hasattr(self.tam_planner, 'trajectory_checks'):
            self.tam_planner.trajectory_checks._skip_param_updates = self.race_started

        self.declare_and_update_parameters()

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
            t1 = time.time()
            state_estimate = self.map_current_state_to_state_estimate()
            raceline = self.get_raceline_from_global_waypoints()
            planning_requests = self.create_planning_requests()
            t2 = time.time()

            # STEP 2: Plan trajectory using TAM calc_trajectory()
            # NOTE: calc_trajectory returns TUPLE of (perf_traj, emerg_traj, s_start, n_start, V_target)
            result = self.tam_planner.calc_trajectory(
                state_estimate=state_estimate,
                raceline=raceline,
                prediction=prediction_dict,
                planning_requests=planning_requests
            )
            t3 = time.time()

            # # STEP 2.5: Publish ALL sampled trajectories for visualization (before filtering)
            # # Access the stored raw arrays from the planner
            # if (hasattr(self.tam_planner, 'last_s_array') and
            #     self.tam_planner.last_s_array is not None and
            #     hasattr(self.tam_planner, 'last_n_array') and
            #     self.tam_planner.last_n_array is not None and
            #     hasattr(self.tam_planner, 'last_valid_array') and
            #         self.tam_planner.last_valid_array is not None):

            #     try:
            #         all_samples_markers = self.create_all_samples_markers(
            #             self.tam_planner.last_s_array,
            #             self.tam_planner.last_n_array,
            #             self.tam_planner.last_valid_array,
            #             self.track_handler
            #         )

            #         self.all_samples_pub.publish(all_samples_markers)

            #         # Log info about sampled trajectories
            #         num_total = self.tam_planner.last_s_array.shape[0]
            #         num_valid = np.sum(
            #             self.tam_planner.last_valid_array) if self.tam_planner.last_valid_array is not None else 0

            #     except Exception as viz_e:
            #         rospy.logerr_throttle(
            #             2, f"{self.log_name} All-samples visualization error: {viz_e}")
            #         import traceback
            #         rospy.logerr_throttle(2, traceback.format_exc())
            # else:
            #     rospy.logwarn_throttle(
            #         5, f"{self.log_name} Cannot publish all_samples: last_s_array={self.tam_planner.last_s_array is not None if hasattr(self.tam_planner, 'last_s_array') else 'N/A'}, last_n_array={self.tam_planner.last_n_array is not None if hasattr(self.tam_planner, 'last_n_array') else 'N/A'}")

            # STEP 3: Unpack return values (handle tuple return)
            t3_unpack_start = time.time()
            if result is not None:
                # calc_trajectory returns tuple: (perf_traj, emerg_traj, s_start, n_start, V_target)
                if isinstance(result, tuple) and len(result) >= 2:
                    performance_traj = result[0]
                    emergency_traj = result[1]
                    trajectory_dict = performance_traj  # Use performance trajectory
                else:
                    # Fallback if unexpected format
                    trajectory_dict = result

                # Verify we have a valid trajectory with sufficient points
                if trajectory_dict and 's' in trajectory_dict and len(trajectory_dict['s']) >= 2:
                    t3_unpack = time.time()

                    # STEP 4: Convert trajectory dict to WpntArray
                    wpnt_array = self.coordinate_transformation.convert_trajectory_to_wpnt_array(
                        trajectory=trajectory_dict,
                        track_handler=self.track_handler,
                        traj_cnt=self.planning_count
                    )
                    t4_convert = time.time()

                    # Check if conversion was successful
                    if wpnt_array is None:
                        # Publish empty trajectory and continue planning
                        empty_msg = OTWpntArray()
                        empty_msg.header.stamp = rospy.Time.now()
                        empty_msg.header.frame_id = "map"
                        self.trajectory_pub.publish(empty_msg)
                        return  # Exit this cycle, will retry in next cycle

                    # STEP 4.5: Interpolate to dense controller spacing (0.1m)
                    # This follows the predictive spliner's proven approach
                    wpnt_array = self.interpolate_trajectory_to_controller_spacing(
                        wpnt_array=wpnt_array,
                        target_spacing=0.1
                    )
                    t4_interpolate = time.time()

                    # Check if interpolation was successful
                    if wpnt_array is None:
                        # Publish empty trajectory and continue planning
                        empty_msg = OTWpntArray()
                        empty_msg.header.stamp = rospy.Time.now()
                        empty_msg.header.frame_id = "map"
                        self.trajectory_pub.publish(empty_msg)
                        return  # Exit this cycle, will retry in next cycle

                    # STEP 5: Wrap in OTWpntArray for state machine compatibility
                    ot_msg = OTWpntArray()
                    ot_msg.wpnts = wpnt_array.wpnts  # Extract list of Wpnt objects
                    ot_msg.header.stamp = rospy.Time.now()
                    ot_msg.header.frame_id = "map"
                    t5_wrap = time.time()

                    # DEBUG: Log first waypoint position vs current position
                    if len(ot_msg.wpnts) > 0:
                        first_wpnt = ot_msg.wpnts[0]
                        dx = first_wpnt.x_m - self.current_state['x']
                        dy = first_wpnt.y_m - self.current_state['y']
                        dist = np.sqrt(dx**2 + dy**2)

                    # STEP 6: Publish trajectory
                    self.trajectory_pub.publish(ot_msg)
                    t6_publish = time.time()

                    # STEP 7: Publish visualization markers
                    # try:
                    #     markers_msg = self.create_f1tenth_visualization_markers(
                    #         trajectory_dict, wpnt_array)
                    #     self.markers_pub.publish(markers_msg)
                    # except Exception as viz_e:
                    #     rospy.logdebug(
                    #         f"{self.log_name} Visualization error: {viz_e}")
                    t7_visualize = time.time()

                    self.planning_count += 1

                    # === TIMING: Log detailed postprocessing breakdown ===
                    t4 = time.time()
                    rospy.loginfo(
                        f"{self.log_name} _execute_planning_cycle timing breakdown:\n"
                        f"  State mapping:      {(t2-t1)*1000:.1f}ms\n"
                        f"  calc_trajectory:    {(t3-t2)*1000:.1f}ms\n"
                        f"  Visualize all samples: {(t3_unpack_start - t3)*1000:.1f}ms\n"
                        f"  Postprocessing breakdown:\n"
                        f"    Unpack result:    {(t3_unpack-t3_unpack_start)*1000:.1f}ms\n"
                        f"    Convert WpntArr:  {(t4_convert-t3_unpack)*1000:.1f}ms\n"
                        f"    Interpolate:      {(t4_interpolate-t4_convert)*1000:.1f}ms\n"
                        f"    Wrap OTWpntArr:   {(t5_wrap-t4_interpolate)*1000:.1f}ms\n"
                        f"    Publish traj:     {(t6_publish-t5_wrap)*1000:.1f}ms\n"
                        f"    Visualize:        {(t7_visualize-t6_publish)*1000:.1f}ms\n"
                        f"  Total postproc:     {(t4-t3)*1000:.1f}ms\n"
                        f"  CYCLE TOTAL:        {(t4-t1)*1000:.1f}ms"
                    )

                    # Log planning stats
                    is_emergency = trajectory_dict.get('emergency', False)
                    cost = trajectory_dict.get('cost', 0.0)
                else:
                    # Publish empty trajectory to signal state machine, then continue planning
                    empty_msg = OTWpntArray()
                    empty_msg.header.stamp = rospy.Time.now()
                    empty_msg.header.frame_id = "map"
                    self.trajectory_pub.publish(empty_msg)
                    # Don't raise exception - just continue to next cycle

            else:
                # Publish empty trajectory to signal state machine, then continue planning
                empty_msg = OTWpntArray()
                empty_msg.header.stamp = rospy.Time.now()
                empty_msg.header.frame_id = "map"
                self.trajectory_pub.publish(empty_msg)
                # Don't raise exception - just continue to next cycle

        except Exception as e:
            import traceback
            rospy.logerr_throttle(5.0, traceback.format_exc())

            # Publish empty trajectory to signal state machine, then continue planning
            empty_msg = OTWpntArray()
            empty_msg.header.stamp = rospy.Time.now()
            empty_msg.header.frame_id = "map"
            self.trajectory_pub.publish(empty_msg)

        # Always measure and log planning time
        planning_time = time.time() - planning_start_time

        rospy.loginfo(
            f"{self.log_name} ⏱️ Time planning cycle: {planning_time*1000:.1f}ms")

        # Publish timing information if measuring
        if self.measuring:
            latency_msg = Float32()
            latency_msg.data = planning_time * 1000  # Convert to milliseconds
            self.latency_pub.publish(latency_msg)

    def loop(self):
        """Main planning loop with proper error handling"""

        # Wait for messages with longer timeout and better feedback
        timeout_duration = 60.0  # Extended to 60 seconds for simulator startup

        try:
            rospy.wait_for_message(
                "global_waypoints", WpntArray, timeout=timeout_duration)
            rospy.wait_for_message(
                "global_waypoints_scaled", WpntArray, timeout=timeout_duration)
            rospy.wait_for_message(
                "car_state/odom_frenet", Odometry, timeout=timeout_duration)

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
