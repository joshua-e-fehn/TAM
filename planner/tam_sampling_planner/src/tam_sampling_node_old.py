#!/usr/bin/env python3
"""
TAM Sampling Planner Node for ROS1 Multi-Car Racing
Complete integration of TAM algorithms with proper namespaced topics

This node integrates the complete TAM sampling algorithms into the existing 
multi-car racing architecture with full namespace support.
"""
import rospy
import numpy as np
from typing import List, Dict, Optional
import time

# ROS message imports
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from dynamic_reconfigure.msg import Config

# F110 custom message imports
from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray

# Import complete TAM sampling core
from tam_sampling_core import TAMSamplingCore, FrenetTrajectory, TAMSamplingUtils


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
        
        # Initialize complete TAM sampling core
        self.tam_planner = TAMSamplingCore(self.planning_params)
        
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
        
        rospy.loginfo(f"{self.log_name} Complete TAM Sampling Planner initialized with namespace: {self.car_namespace}")
    
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
        
        rospy.loginfo(f"{self.log_name} ROS interface setup complete with namespaced topics")
    
    def update_dynamic_params(self):
        """Update parameters from ROS parameter server (complete TAM parameters)"""
        
        # Core sampling parameters
        self.planning_params['lateral_samples'] = rospy.get_param("~lateral_samples", 15)
        self.planning_params['longitudinal_samples'] = rospy.get_param("~longitudinal_samples", 8)
        self.planning_params['planning_horizon'] = rospy.get_param("~planning_horizon", 4.0)
        self.planning_params['n_dense_samples'] = rospy.get_param("~n_dense_samples", 5)
        
        # Vehicle constraints
        self.planning_params['max_speed'] = rospy.get_param("~max_speed", 20.0)
        self.planning_params['max_accel'] = rospy.get_param("~max_accel", 8.0)
        self.planning_params['max_lateral_accel'] = rospy.get_param("~max_lateral_accel", 12.0)
        
        # Longitudinal sampling parameters
        self.planning_params['s_dot_discretization'] = rospy.get_param("~s_dot_discretization", 2.0)
        self.planning_params['v_sampling_scale'] = rospy.get_param("~v_sampling_scale", 1.1)
        
        # Cost weights (TAM naming convention)
        self.planning_params['raceline_cost_weight'] = rospy.get_param("~raceline_cost_weight", 3.5)
        self.planning_params['velocity_cost_weight'] = rospy.get_param("~velocity_cost_weight", 3.0)
        self.planning_params['friction_cost_weight'] = rospy.get_param("~friction_cost_weight", 5000.0)
        self.planning_params['curvature_cost_weight'] = rospy.get_param("~curvature_cost_weight", 500000.0)
        self.planning_params['lateral_jerk_cost_weight'] = rospy.get_param("~lateral_jerk_cost_weight", 0.5)
        self.planning_params['prediction_cost_weight'] = rospy.get_param("~prediction_cost_weight", 100000.0)
        self.planning_params['collision_cost_weight'] = rospy.get_param("~collision_cost_weight", 100000000.0)
        
        # Safety parameters
        self.planning_params['safety_distance_track_left'] = rospy.get_param("~safety_distance_track_left", 0.5)
        self.planning_params['safety_distance_track_right'] = rospy.get_param("~safety_distance_track_right", 0.5)
        self.planning_params['safety_margin_static'] = rospy.get_param("~safety_margin_static", 0.5)
        self.planning_params['safety_margin_dynamic'] = rospy.get_param("~safety_margin_dynamic", 1.0)
        
        # Trajectory validation parameters
        self.planning_params['kappa_thr'] = rospy.get_param("~kappa_thr", 0.1)
        self.planning_params['curvature_cost_threshold'] = rospy.get_param("~curvature_cost_threshold", 30.0)
    
    def global_waypoints_callback(self, msg: WpntArray):
        """Process global waypoints and extract track data"""
        self.global_waypoints = msg
        
        if len(msg.wpnts) > 0:
            # Extract track centerline and geometry
            centerline = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in msg.wpnts])
            headings = np.array([wpnt.psi_rad for wpnt in msg.wpnts])
            s_coords = np.array([wpnt.s_m if hasattr(wpnt, 's_m') else i*1.0 for i, wpnt in enumerate(msg.wpnts)])
            
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
            
            # Update track data cache
            self.track_data = {
                'centerline': centerline,
                'headings': headings,
                's_coord': s_coords,
                'omega_z': omega_z,
                'd_omega_z': d_omega_z
            }
            
            rospy.loginfo_throttle(5, f"{self.log_name} Track data updated: {len(msg.wpnts)} waypoints")
    
    def global_waypoints_scaled_callback(self, msg: WpntArray):
        """Process scaled global waypoints and extract raceline data"""
        self.global_waypoints_scaled = msg
        
        # Convert to TAM raceline format
        self.raceline_data = TAMSamplingUtils.waypoints_to_raceline_data(msg)
        rospy.loginfo_throttle(5, f"{self.log_name} Raceline data updated with {len(msg.wpnts)} points")
    
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
        
        # Store Cartesian information for reference
        self.current_state['x'] = msg.pose.pose.position.z  # Sometimes stored in z
        self.current_state['y'] = msg.pose.pose.orientation.z  # Check message definition
        
        # Extract heading from quaternion
        from tf.transformations import euler_from_quaternion
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_state['heading'] = yaw
    
    def obstacles_callback(self, msg: ObstacleArray):
        """Process detected obstacles"""
        self.obs = msg
        rospy.loginfo_throttle(10, f"{self.log_name} Obstacles updated: {len(msg.obstacles)} detected")
    
    def dynamic_params_callback(self, msg: Config):
        """Handle dynamic reconfigure parameter updates"""
        
        # Update parameters from dynamic reconfigure
        for param in msg.doubles:
            param_name = param.name.split('.')[-1]  # Get parameter name without namespace
            if param_name in self.planning_params:
                old_value = self.planning_params[param_name]
                self.planning_params[param_name] = param.value
                rospy.loginfo(f"{self.log_name} Updated {param_name}: {old_value} -> {param.value}")
        
        for param in msg.ints:
            param_name = param.name.split('.')[-1]
            if param_name in self.planning_params:
                old_value = self.planning_params[param_name]
                self.planning_params[param_name] = param.value
                rospy.loginfo(f"{self.log_name} Updated {param_name}: {old_value} -> {param.value}")
        
        # Reinitialize planner with new parameters
        self.tam_planner = TAMSamplingCore(self.planning_params)
        rospy.loginfo(f"{self.log_name} TAM planner reinitialized with new parameters")
    
    def process_obstacles(self) -> List[Dict]:
        """Convert ROS obstacles to TAM format"""
        return TAMSamplingUtils.obstacles_to_tam_format(self.obs)
    
    def create_trajectory_message(self, trajectory: FrenetTrajectory) -> OTWpntArray:
        """Convert TAM trajectory to ROS message format"""
        msg = OTWpntArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        # Convert trajectory points to waypoints
        min_length = min(len(trajectory.t), len(trajectory.x), len(trajectory.y), len(trajectory.V))
        
        for i in range(min_length):
            wpnt = Wpnt()
            wpnt.x_m = trajectory.x[i]
            wpnt.y_m = trajectory.y[i]
            wpnt.psi_rad = trajectory.psi[i] if i < len(trajectory.psi) else 0.0
            wpnt.s_m = trajectory.s[i] if i < len(trajectory.s) else trajectory.V[i] * 0.1  # Fallback
            wpnt.d_m = trajectory.n[i] if i < len(trajectory.n) else 0.0
            wpnt.v_mps = trajectory.V[i]
            
            # Additional information
            if i < len(trajectory.ax_vf):
                wpnt.ax_mps2 = trajectory.ax_vf[i]
            if i < len(trajectory.ay_vf):
                wpnt.ay_mps2 = trajectory.ay_vf[i]
            
            msg.wpnts.append(wpnt)
        
        rospy.loginfo_throttle(1, f"{self.log_name} Created trajectory message with {len(msg.wpnts)} points")
        return msg
    
    def create_visualization_markers(self, trajectory: FrenetTrajectory) -> MarkerArray:
        """Create comprehensive visualization markers for the planned trajectory"""
        marker_array = MarkerArray()
        
        if len(trajectory.x) == 0:
            return marker_array
        
        # Main trajectory line
        trajectory_marker = Marker()
        trajectory_marker.header.frame_id = "map"
        trajectory_marker.header.stamp = rospy.Time.now()
        trajectory_marker.ns = f"{self.car_namespace}_tam_trajectory" if self.car_namespace else "tam_trajectory"
        trajectory_marker.id = 0
        trajectory_marker.type = Marker.LINE_STRIP
        trajectory_marker.action = Marker.ADD
        trajectory_marker.scale.x = 0.15  # Line width
        trajectory_marker.color.a = 1.0
        trajectory_marker.color.r = 0.0
        trajectory_marker.color.g = 1.0
        trajectory_marker.color.b = 0.0
        
        # Add trajectory points
        for i in range(len(trajectory.x)):
            point = Point()
            point.x = trajectory.x[i]
            point.y = trajectory.y[i]
            point.z = 0.1
            trajectory_marker.points.append(point)
        
        marker_array.markers.append(trajectory_marker)
        
        # Velocity visualization
        if len(trajectory.V) > 0:
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
            
            # Show velocity at trajectory start
            if len(trajectory.x) > 0:
                velocity_marker.pose.position.x = trajectory.x[0]
                velocity_marker.pose.position.y = trajectory.y[0]
                velocity_marker.pose.position.z = 1.0
                velocity_marker.text = f"TAM: {trajectory.V[0]:.1f} m/s\nCost: {trajectory.cost:.1f}"
                marker_array.markers.append(velocity_marker)
        
        return marker_array
    
    def run_planning_cycle(self):
        """Execute one complete TAM planning cycle"""
        
        planning_start_time = time.time()
        
        # Check if we have required data
        if (len(self.global_waypoints.wpnts) == 0 or 
            len(self.track_data['centerline']) == 0 or
            not self.raceline_data):
            rospy.logwarn_throttle(5, f"{self.log_name} Missing required data for planning")
            return
        
        # Process obstacles
        obstacles = self.process_obstacles()
        
        # Run complete TAM sampling planner
        try:
            optimal_trajectory = self.tam_planner.plan_trajectory(
                current_state=self.current_state,
                raceline_data=self.raceline_data,
                track_data=self.track_data,
                obstacles=obstacles
            )
            
            if optimal_trajectory is not None:
                # Publish trajectory
                trajectory_msg = self.create_trajectory_message(optimal_trajectory)
                self.trajectory_pub.publish(trajectory_msg)
                
                # Publish visualization markers
                markers_msg = self.create_visualization_markers(optimal_trajectory)
                self.markers_pub.publish(markers_msg)
                
                self.planning_count += 1
                rospy.loginfo_throttle(2, f"{self.log_name} Published trajectory {self.planning_count}, cost: {optimal_trajectory.cost:.1f}")
                
            else:
                rospy.logwarn_throttle(1, f"{self.log_name} No valid trajectory found!")
                
        except Exception as e:
            rospy.logerr(f"{self.log_name} Planning failed: {str(e)}")
        
        # Publish timing information if measuring
        if self.measuring:
            planning_time = time.time() - planning_start_time
            latency_msg = Float32()
            latency_msg.data = planning_time * 1000  # Convert to milliseconds
            self.latency_pub.publish(latency_msg)
            
            if planning_time > 0.05:  # Warn if planning takes more than 50ms
                rospy.logwarn(f"{self.log_name} Slow planning cycle: {planning_time*1000:.1f}ms")
    
    def loop(self):
        """Main planning loop with proper error handling"""
        
        # Wait for critical messages
        rospy.loginfo(f"{self.log_name} Waiting for required messages...")
        
        try:
            rospy.wait_for_message("global_waypoints", WpntArray, timeout=10.0)
            rospy.wait_for_message("global_waypoints_scaled", WpntArray, timeout=10.0)
            rospy.wait_for_message("car_state/odom_frenet", Odometry, timeout=10.0)
        except rospy.ROSException as e:
            rospy.logerr(f"{self.log_name} Timeout waiting for required messages: {e}")
            return
        
        rospy.loginfo(f"{self.log_name} All required messages received. Starting TAM planning loop.")
        
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

import time
import rospy
import numpy as np
from typing import List, Dict, Optional

# ROS message imports
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from dynamic_reconfigure.msg import Config

# F110 custom message imports
from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray

# Import TAM sampling core
from tam_sampling_core import TAMSamplingCore, FrenetTrajectory, TAMSamplingUtils


class TAMSamplingPlannerNode:
    """
    TAM Sampling Planner ROS1 Node
    
    Subscribes to:
        - global_waypoints: Global racing line waypoints
        - global_waypoints_scaled: Speed-scaled global waypoints  
        - car_state/odom_frenet: Vehicle state in Frenet coordinates
        - perception/obstacles: Detected obstacles
        - dynamic_tam_sampling_tuner_node/parameter_updates: Dynamic reconfigure
    
    Publishes:
        - planner/avoidance/otwpnts: Planned trajectory for state machine
        - planner/avoidance/markers: Visualization markers
        - planner/avoidance/latency: Planning computation time (if measuring enabled)
    """
    
    def __init__(self):
        """Initialize the TAM Sampling Planner node"""
        # Initialize node
        self.name = "tam_sampling_planner_node"
        rospy.init_node(self.name)
        
        # Get car namespace for logging
        self.car_namespace = rospy.get_namespace().strip('/')
        if self.car_namespace:
            self.log_name = f"[{self.car_namespace}_{self.name}]"
        else:
            self.log_name = f"[{self.name}]"
        
        # State variables
        self.obs = ObstacleArray()
        self.global_waypoints = WpntArray()
        self.global_waypoints_scaled = WpntArray()
        self.current_state = {
            's': 0.0,
            'n': 0.0,
            's_dot': 0.0,
            'n_dot': 0.0,
            'x': 0.0,
            'y': 0.0,
            'heading': 0.0
        }
        
        # Planning parameters (following TAM defaults)
        self.planning_params = {
            'lateral_samples': 15,
            'longitudinal_samples': 8,
            'planning_horizon': 4.0,  # seconds
            'dt': 0.1,
            'max_speed': 20.0,
            'max_accel': 8.0,
            'max_lateral_accel': 12.0,
            'track_width': 3.0,
            'w_raceline': 3.5,
            'w_velocity': 3.0,
            'w_smoothness': 1.0,
            'w_obstacle': 10000.0,
            'w_lateral_jerk': 0.5,
            'safety_margin_static': 0.5,
            'safety_margin_dynamic': 1.0
        }
        
        # Dynamic reconfigure parameters (match TAM parameter structure)
        self.update_dynamic_params()
        
        # Initialize TAM sampling core
        self.tam_planner = TAMSamplingCore(self.planning_params)
        
        # ROS parameters
        self.from_bag = rospy.get_param("/from_bag", False)
        self.measuring = rospy.get_param("/measure", False)
        self.lookahead = rospy.get_param("~lookahead", 15.0)  # meters
        
        # Track information cache
        self.track_centerline = np.array([])
        self.track_headings = np.array([])
        self.raceline_data = {'n': [], 'v': []}
        
        # Timing and performance
        self.last_planning_time = 0.0
        self.planning_rate = rospy.Rate(20)  # 20 Hz like other planners
        
        # Initialize ROS interface
        self.setup_ros_interface()
        
        rospy.loginfo(f"{self.log_name} TAM Sampling Planner initialized")
    
    def setup_ros_interface(self):
        """Setup ROS subscribers and publishers"""
        
        # Subscribers (using relative topics for namespace support)
        rospy.Subscriber("global_waypoints", WpntArray, self.global_waypoints_callback)
        rospy.Subscriber("global_waypoints_scaled", WpntArray, self.global_waypoints_scaled_callback)
        rospy.Subscriber("car_state/odom_frenet", Odometry, self.state_callback)
        rospy.Subscriber("perception/obstacles", ObstacleArray, self.obstacles_callback)
        
        # Dynamic reconfigure subscriber (only if not from bag)
        if not self.from_bag:
            rospy.Subscriber("dynamic_tam_sampling_tuner_node/parameter_updates", 
                           Config, self.dynamic_params_callback)
        
        # Publishers (using relative topics for namespace support)
        self.trajectory_pub = rospy.Publisher("planner/avoidance/otwpnts", OTWpntArray, queue_size=1)
        self.markers_pub = rospy.Publisher("planner/avoidance/markers", MarkerArray, queue_size=1)
        
        # Optional latency publisher for performance measurement
        if self.measuring:
            self.latency_pub = rospy.Publisher("planner/avoidance/latency", Float32, queue_size=1)
    
    def update_dynamic_params(self):
        """Update parameters from ROS parameter server (TAM-style parameters)"""
        # Core sampling parameters
        self.planning_params['lateral_samples'] = rospy.get_param("~lateral_samples", 15)
        self.planning_params['longitudinal_samples'] = rospy.get_param("~longitudinal_samples", 8)
        self.planning_params['planning_horizon'] = rospy.get_param("~planning_horizon", 4.0)
        
        # Vehicle constraints
        self.planning_params['max_speed'] = rospy.get_param("~max_speed", 20.0)
        self.planning_params['max_accel'] = rospy.get_param("~max_accel", 8.0)
        self.planning_params['max_lateral_accel'] = rospy.get_param("~max_lateral_accel", 12.0)
        
        # Cost weights (following TAM naming convention)
        self.planning_params['w_raceline'] = rospy.get_param("~raceline_cost_weight", 3.5)
        self.planning_params['w_velocity'] = rospy.get_param("~velocity_cost_weight", 3.0)
        self.planning_params['w_smoothness'] = rospy.get_param("~smoothness_cost_weight", 1.0)
        self.planning_params['w_obstacle'] = rospy.get_param("~obstacle_cost_weight", 10000.0)
        self.planning_params['w_lateral_jerk'] = rospy.get_param("~lateral_jerk_cost_weight", 0.5)
        
        # Safety parameters
        self.planning_params['safety_margin_static'] = rospy.get_param("~safety_margin_static", 0.5)
        self.planning_params['safety_margin_dynamic'] = rospy.get_param("~safety_margin_dynamic", 1.0)
    
    def global_waypoints_callback(self, msg: WpntArray):
        """Process global waypoints (racing line)"""
        self.global_waypoints = msg
        
        # Extract track centerline and headings for coordinate conversion
        if len(msg.wpnts) > 0:
            self.track_centerline = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in msg.wpnts])
            self.track_headings = np.array([wpnt.psi_rad for wpnt in msg.wpnts])
            
            # Extract raceline data for planning
            self.raceline_data['n'] = [0.0] * len(msg.wpnts)  # Assume raceline is centerline
            self.raceline_data['v'] = [wpnt.vx_mps for wpnt in msg.wpnts]
    
    def global_waypoints_scaled_callback(self, msg: WpntArray):
        """Process scaled global waypoints"""
        self.global_waypoints_scaled = msg
    
    def state_callback(self, msg: Odometry):
        """Process vehicle state in Frenet coordinates"""
        self.current_state['s'] = msg.pose.pose.position.x
        self.current_state['n'] = msg.pose.pose.position.y
        self.current_state['s_dot'] = msg.twist.twist.linear.x
        self.current_state['n_dot'] = msg.twist.twist.linear.y
        
        # Store Cartesian position for debugging
        self.current_state['x'] = msg.pose.pose.position.z  # Sometimes stored in z
        self.current_state['y'] = msg.pose.pose.orientation.z  # Check message definition
    
    def obstacles_callback(self, msg: ObstacleArray):
        """Process detected obstacles"""
        self.obs = msg
    
    def dynamic_params_callback(self, msg: Config):
        """Handle dynamic reconfigure parameter updates"""
        # Update parameters from dynamic reconfigure
        for param in msg.doubles:
            param_name = param.name.split('/')[-1]  # Get parameter name without namespace
            
            # Map parameter names to internal structure
            if param_name == "lateral_samples":
                self.planning_params['lateral_samples'] = int(param.value)
            elif param_name == "longitudinal_samples":
                self.planning_params['longitudinal_samples'] = int(param.value)
            elif param_name == "planning_horizon":
                self.planning_params['planning_horizon'] = param.value
            elif param_name == "raceline_cost_weight":
                self.planning_params['w_raceline'] = param.value
            elif param_name == "velocity_cost_weight":
                self.planning_params['w_velocity'] = param.value
            elif param_name == "obstacle_cost_weight":
                self.planning_params['w_obstacle'] = param.value
            elif param_name == "safety_margin_static":
                self.planning_params['safety_margin_static'] = param.value
            elif param_name == "safety_margin_dynamic":
                self.planning_params['safety_margin_dynamic'] = param.value
        
        # Reinitialize planner with new parameters
        self.tam_planner = TAMSamplingCore(self.planning_params)
        
        rospy.loginfo(f"{self.log_name} Dynamic parameters updated")
    
    def process_obstacles(self) -> List[Dict]:
        """Convert ROS obstacles to internal format"""
        obstacles = []
        
        for obs in self.obs.obstacles:
            # Convert obstacle to internal format
            obstacle = {
                'x': obs.x,
                'y': obs.y,
                'radius': max(obs.radius, 0.1),  # Minimum radius
                'velocity': obs.velocity,
                'heading': getattr(obs, 'heading', 0.0),
                'static': obs.velocity < 0.1  # Consider stationary if very slow
            }
            obstacles.append(obstacle)
        
        return obstacles
    
    def create_trajectory_message(self, trajectory: FrenetTrajectory) -> OTWpntArray:
        """Convert TAM trajectory to ROS message format"""
        msg = OTWpntArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        # Convert trajectory points to waypoints
        for i in range(len(trajectory.t)):
            if i >= len(trajectory.s) or i >= len(trajectory.n):
                break
                
            wpnt = Wpnt()
            wpnt.id = i
            
            # Convert Frenet to Cartesian coordinates
            x, y = TAMSamplingUtils.frenet_to_cartesian(
                trajectory.s[i], trajectory.n[i], 
                self.track_centerline, self.track_headings
            )
            
            wpnt.x_m = x
            wpnt.y_m = y
            wpnt.s_m = trajectory.s[i]
            wpnt.d_m = trajectory.n[i]
            wpnt.vx_mps = trajectory.V[i] if i < len(trajectory.V) else 0.0
            wpnt.psi_rad = getattr(trajectory, 'chi', [0.0] * len(trajectory.t))[i]
            
            # Add acceleration information if available
            if hasattr(trajectory, 'ax') and i < len(trajectory.ax):
                wpnt.ax_mps2 = trajectory.ax[i]
            if hasattr(trajectory, 'ay') and i < len(trajectory.ay):
                wpnt.ay_mps2 = trajectory.ay[i]
            
            msg.wpnts.append(wpnt)
        
        return msg
    
    def create_visualization_markers(self, trajectory: FrenetTrajectory) -> MarkerArray:
        """Create visualization markers for the planned trajectory"""
        marker_array = MarkerArray()
        
        # Create trajectory line marker
        trajectory_marker = Marker()
        trajectory_marker.header.frame_id = "map"
        trajectory_marker.header.stamp = rospy.Time.now()
        trajectory_marker.ns = "tam_sampling_trajectory"
        trajectory_marker.id = 0
        trajectory_marker.type = Marker.LINE_STRIP
        trajectory_marker.action = Marker.ADD
        trajectory_marker.scale.x = 0.1  # Line width
        trajectory_marker.color.a = 1.0
        trajectory_marker.color.r = 0.0
        trajectory_marker.color.g = 1.0
        trajectory_marker.color.b = 0.0
        
        # Add trajectory points
        for i in range(len(trajectory.s)):
            if i >= len(trajectory.n):
                break
                
            x, y = TAMSamplingUtils.frenet_to_cartesian(
                trajectory.s[i], trajectory.n[i],
                self.track_centerline, self.track_headings
            )
            
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.1
            trajectory_marker.points.append(point)
        
        marker_array.markers.append(trajectory_marker)
        return marker_array
    
    def run_planning_cycle(self):
        """Execute one planning cycle"""
        if self.measuring:
            start_time = time.perf_counter()
        
        # Check if we have required data
        if (len(self.global_waypoints.wpnts) == 0 or 
            len(self.track_centerline) == 0):
            return
        
        # Process obstacles
        obstacles = self.process_obstacles()
        
        # Run TAM sampling planner
        try:
            optimal_trajectory = self.tam_planner.plan_trajectory(
                self.current_state,
                self.raceline_data,
                obstacles
            )
            
            if optimal_trajectory is not None:
                # Publish trajectory
                trajectory_msg = self.create_trajectory_message(optimal_trajectory)
                self.trajectory_pub.publish(trajectory_msg)
                
                # Publish visualization
                markers = self.create_visualization_markers(optimal_trajectory)
                self.markers_pub.publish(markers)
                
                rospy.logdebug(f"{self.log_name} Published trajectory with cost: {optimal_trajectory.cost:.2f}")
            else:
                rospy.logwarn(f"{self.log_name} No valid trajectory found!")
                
        except Exception as e:
            rospy.logerr(f"{self.log_name} Planning failed: {str(e)}")
        
        # Publish timing information if measuring
        if self.measuring:
            planning_time = time.perf_counter() - start_time
            self.latency_pub.publish(Float32(data=planning_time))
    
    def loop(self):
        """Main planning loop"""
        # Wait for critical messages
        rospy.loginfo(f"{self.log_name} Waiting for messages...")
        rospy.wait_for_message("global_waypoints", WpntArray)
        rospy.wait_for_message("global_waypoints_scaled", WpntArray)
        rospy.wait_for_message("car_state/odom_frenet", Odometry)
        
        rospy.loginfo(f"{self.log_name} Ready! Starting planning loop.")
        
        while not rospy.is_shutdown():
            self.run_planning_cycle()
            self.planning_rate.sleep()


def main():
    """Main function"""
    try:
        planner_node = TAMSamplingPlannerNode()
        planner_node.loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("TAM Sampling Planner node interrupted")
    except Exception as e:
        rospy.logerr(f"TAM Sampling Planner node failed: {str(e)}")


if __name__ == '__main__':
    main()
