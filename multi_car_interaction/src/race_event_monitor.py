#!/usr/bin/env python3

"""
Race Event Monitor Node

This node monitors races and:
1. Detects race completion conditions
2. Logs race events (collisions, near-misses, overtakes, crashes, state transitions)
3. Sets /race_test/simulation_complete when race ends
4. Provides event data for post-race analysis via CSV logs

Modes:
- multi_car: Two cars racing (default)
- single_car_no_obstacle: Single car time trial
- single_car_obstacle: Single car vs dummy obstacle

Completion Conditions (mode-dependent):

Multi-car:
  - Car2 finishes target laps while car1 is behind
  - Collision between cars (max_car_collisions reached)
  - Car1 crashes with track boundary (max_boundary_collisions reached, if enabled)
  - Car1 overtakes car2 by overtake_lead_distance
  - Car2 overlaps car1 by 2+ laps
  - Ego car (car1) stalled (not moving 5m in 15 seconds)

Single car (no obstacle):
  - Car completes target laps
  - Car crashes with track boundary (max_boundary_collisions reached, if enabled)
  - Ego car stalled (not moving 5m in 15 seconds)

Single car (with obstacle):
  - Car completes target laps
  - Collision with obstacle (max_obstacle_collisions reached)
  - Car crashes with track boundary (max_boundary_collisions reached, if enabled)
  - Car overtakes obstacle by overtake_lead_distance (max_overtakes reached)
  - Obstacle overlaps car by 2+ laps
  - Ego car stalled (not moving 5m in 15 seconds)

Note: Obstacle boundary collisions are NOT monitored (only car boundaries are checked).

Author: Atlas  
Date: December 2025
"""

import rospy
import math
import csv
import os
import numpy as np
from datetime import datetime
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Int32
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from f110_msgs.msg import WpntArray


class RaceEventMonitor:
    def __init__(self):
        rospy.init_node('race_event_monitor', anonymous=True)

        # Race mode configuration
        self.race_mode = rospy.get_param('~race_mode', 'multi_car')
        # Options: 'multi_car', 'single_car_no_obstacle', 'single_car_obstacle'

        # Car configuration
        if self.race_mode.startswith('single_car'):
            # Single car mode - only one car
            self.car_names = ['car']  # No namespace in single_car mode
            self.monitor_obstacle = (self.race_mode == 'single_car_obstacle')
        else:
            # Multi-car mode - get from parameter
            car_names_param = rospy.get_param('~car_names', 'car1,car2')
            if isinstance(car_names_param, str):
                self.car_names = [name.strip()
                                  for name in car_names_param.split(',')]
            else:
                self.car_names = car_names_param
            self.monitor_obstacle = False

        # Monitoring parameters
        self.check_rate = rospy.get_param('/race_test/check_rate', 80.0)  # Hz

        # Collision detection parameters
        self.warning_distance = rospy.get_param(
            '/race_test/warning_distance', 1.5)  # meters
        self.critical_distance = rospy.get_param(
            '/race_test/critical_distance', 0.8)  # meters
        self.collision_distance = rospy.get_param(
            '/race_test/collision_distance', 0.4)  # meters
        # Bounding box collision margin (added to car dimensions for safety)
        self.bbox_collision_margin = rospy.get_param(
            '/race_test/bbox_collision_margin', 0.05)  # meters

        # Race completion parameters
        self.target_laps = rospy.get_param('/race_test/target_laps', 3)
        self.overtake_lead_distance = rospy.get_param(
            '/race_test/overtake_lead_distance', 5.0)  # meters
        self.boundary_safety_margin = rospy.get_param(
            '/race_test/boundary_safety_margin', 0.0)  # meters - additional safety margin
        self.boundary_violation_tolerance = rospy.get_param(
            '/race_test/boundary_violation_tolerance', 0.05)  # meters - tolerance for minor boundary scratches

        # Track parameters
        try:
            self.track_length = rospy.get_param(
                '/global_republisher/track_length')
        except:
            self.track_length = rospy.get_param('~track_length', 76.48)

        # Load global waypoints for track boundary checking
        self.global_waypoints = WpntArray()
        # Note: Don't reset track_length here - keep the value loaded from params above
        # It will be updated when global_waypoints_callback receives waypoints
        self.track_boundaries = {'left': [], 'right': []}

        # Car model and dimensions
        self.car_model = rospy.get_param('/race_test/car_model', 'NUC2')
        self.car_length = rospy.get_param(
            '/race_test/car_length', 0.48)  # meters
        self.car_width = rospy.get_param(
            '/race_test/car_width', 0.31)    # meters

        # Storage for car data
        self.car_positions = {}  # Cartesian positions
        self.car_frenet = {}     # Frenet coordinates (s, d)
        self.car_laps = {}       # Lap counts
        self.car_previous_s = {}  # Previous s for lap detection

        # Storage for obstacle data (single_car_obstacle mode)
        self.obstacle_position = None
        self.obstacle_frenet = None
        self.obstacle_laps = 0
        self.obstacle_previous_s = None
        # Track obstacle state (READY, GB_TRACK, etc.)
        self.obstacle_state = None

        # Race state
        self.race_complete = False
        self.race_complete_reason = None
        self.race_start_time = rospy.Time.now()
        self.race_started = False  # Track if race has actually started

        # Event logging
        self.events = []
        self.log_file_path = self.setup_event_logging()

        # Publishers for collision/race status
        self.collision_publishers = {}
        self.warning_publishers = {}
        self.status_publisher = rospy.Publisher(
            '/multi_car/race_status', String, queue_size=10)
        self.viz_publisher = rospy.Publisher(
            '/multi_car/race_visualization', MarkerArray, queue_size=10)

        # Publisher for opponent lap complete flag (used by collision_prediction)
        # Latched so collision_prediction gets the latest value when it starts
        self.opponent_lap_complete_pub = rospy.Publisher(
            '/opponent_lap_complete', Bool, queue_size=10, latch=True)
        # Initialize to False - opponent hasn't completed a lap yet
        self.opponent_lap_complete_pub.publish(Bool(data=False))

        # Publisher for force_trailing flag (used by collision_prediction)
        # Topic is mode-dependent: /car1/collision_prediction/force_trailing for multi_car
        #                          /collision_prediction/force_trailing for single_car
        if self.race_mode == 'multi_car':
            force_trailing_topic = '/car1/collision_prediction/force_trailing'
        else:
            force_trailing_topic = '/collision_prediction/force_trailing'
        self.force_trailing_pub = rospy.Publisher(
            force_trailing_topic, Bool, queue_size=10, latch=True)
        # Initialize to True - force trailing until opponent completes a lap
        self.force_trailing_pub.publish(Bool(data=True))
        rospy.loginfo(
            f"[Race Monitor] Publishing force_trailing to {force_trailing_topic}")

        # Initialize subscribers and publishers
        self.setup_global_topics()  # Subscribe to global waypoints
        self.setup_car_topics()

        # Setup obstacle monitoring if needed
        if self.monitor_obstacle:
            self.setup_obstacle_topics()

        # Main monitoring timer
        self.timer = rospy.Timer(rospy.Duration(
            1.0/self.check_rate), self.monitor_race)

        rospy.loginfo(
            f"[Race Monitor] Initialized in {self.race_mode} mode")
        rospy.loginfo(
            f"[Race Monitor] Cars: {self.car_names}, Monitor obstacle: {self.monitor_obstacle}")
        rospy.loginfo(
            f"[Race Monitor] Target laps: {self.target_laps}, Event log: {self.log_file_path}")

    def setup_global_topics(self):
        """Setup global subscribers and publishers"""
        rospy.Subscriber("global_waypoints", WpntArray,
                         self.global_waypoints_callback, queue_size=1)

        # Subscribe to race start signals
        if self.race_mode.startswith('single_car'):
            rospy.Subscriber("/state_machine_cmd", String,
                             self.race_start_callback, queue_size=1)
        else:
            rospy.Subscriber("/car1/state_machine_cmd", String,
                             self.race_start_callback, queue_size=1)

    def race_start_callback(self, msg):
        """Detect race start signal and log the exact timestamp"""
        if not self.race_started and msg.data == "GB_TRACK":
            self.race_started = True
            # Update race start time to the actual start moment
            self.race_start_time = rospy.Time.now()

            # Log race start event with timestamp 0.000 (start of race)
            self.log_event('race_start',
                           car1_name=self.car_names[0] if self.car_names else None,
                           details=f"Race started - GO signal received")
            rospy.loginfo("[Race Monitor] 🏁 RACE STARTED - Logging timestamp")

    def global_waypoints_callback(self, msg: WpntArray):
        """Process global waypoints and setup Frenet converter"""
        self.global_waypoints = msg

        if len(msg.wpnts) > 0:
            self.track_length = msg.wpnts[-1].s_m

            # Extract track boundaries
            self.track_boundaries['left'] = [wpnt.d_left for wpnt in msg.wpnts]
            self.track_boundaries['right'] = [
                wpnt.d_right for wpnt in msg.wpnts]

    def find_closest_waypoint_index(self, s_pos):
        """Find the closest waypoint index for a given s position"""

        if len(self.global_waypoints.wpnts) == 0:
            return 0

        # Simple linear search (could be optimized with binary search)
        s_coords = [wpnt.s_m for wpnt in self.global_waypoints.wpnts]
        differences = [abs(s - s_pos) for s in s_coords]
        return differences.index(min(differences))

    def get_track_boundaries_at_s(self, s_coord):
        """
        Get track boundaries at a given s-coordinate

        Args:
            s_coord: Current s-coordinate of the car

        Returns:
            tuple: (d_left, d_right) boundaries at this position, or None if waypoints unavailable
        """
        # Find closest waypoint index for boundary lookup
        waypoint_idx = self.find_closest_waypoint_index(s_coord)

        # Get track boundaries at this position
        d_left = self.track_boundaries['left'][waypoint_idx]
        d_right = self.track_boundaries['right'][waypoint_idx]
        return (d_left, d_right)

    def setup_event_logging(self):
        """Setup CSV event logging"""
        # Get batch number and simulation ID
        batch_number = rospy.get_param('/race_test/batch_number', '')
        simulation_id = rospy.get_param('/race_test/simulation_id', 0)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Base log directory - use logs folder in catkin workspace
        base_log_dir = rospy.get_param('/race_test/log_directory',
                                       os.path.join(os.path.expanduser('~'), 'catkin_ws', 'src', 'race_stack', 'test_simulation', 'logs'))

        # Create mode-specific subdirectory
        mode_dir = os.path.join(base_log_dir, self.race_mode)

        # Batch number should always be set by the test framework
        # If not set, generate one (fallback - should not happen in normal operation)
        if not batch_number or str(batch_number) == '0':
            batch_number = datetime.now().strftime("%Y%m%d%H%M%S")
            rospy.logwarn(
                f"[Race Monitor] No batch_number set, generated: {batch_number}")

        # Create batch-specific subdirectory within mode folder
        batch_dir = os.path.join(mode_dir, f"batch_{batch_number}")
        log_filename = f"race_events_sim{simulation_id}_{timestamp}.csv"

        # Create batch directory if it doesn't exist
        os.makedirs(batch_dir, exist_ok=True)

        log_file = os.path.join(batch_dir, log_filename)

        # Write CSV header
        with open(log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'event_type', 'car1_name', 'car2_name',
                'distance', 'car1_s', 'car2_s', 'car1_d', 'car2_d',
                'car1_lap', 'car2_lap', 'details'
            ])
        return log_file

    def log_event(self, event_type, car1_name=None, car2_name=None, distance=None, details=''):
        """Log race event to CSV and memory"""
        timestamp = (rospy.Time.now() - self.race_start_time).to_sec()

        # Get car data if available
        car1_s = self.car_frenet.get(car1_name, {}).get(
            's', 0.0) if car1_name else 0.0
        car2_s = self.car_frenet.get(car2_name, {}).get(
            's', 0.0) if car2_name else 0.0
        car1_d = self.car_frenet.get(car1_name, {}).get(
            'd', 0.0) if car1_name else 0.0
        car2_d = self.car_frenet.get(car2_name, {}).get(
            'd', 0.0) if car2_name else 0.0
        car1_lap = self.car_laps.get(car1_name, 0) if car1_name else 0
        car2_lap = self.car_laps.get(car2_name, 0) if car2_name else 0

        # Store in memory
        event = {
            'timestamp': timestamp,
            'event_type': event_type,
            'car1_name': car1_name or '',
            'car2_name': car2_name or '',
            'distance': distance or 0.0,
            'car1_s': car1_s,
            'car2_s': car2_s,
            'car1_d': car1_d,
            'car2_d': car2_d,
            'car1_lap': car1_lap,
            'car2_lap': car2_lap,
            'details': details
        }
        self.events.append(event)

        # Write to CSV
        try:
            with open(self.log_file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    f"{timestamp:.3f}", event_type, car1_name or '', car2_name or '',
                    f"{distance:.3f}" if distance else '',
                    f"{car1_s:.2f}", f"{car2_s:.2f}",
                    f"{car1_d:.2f}", f"{car2_d:.2f}",
                    car1_lap, car2_lap, details
                ])
        except Exception as e:
            rospy.logwarn(f"Failed to write event to CSV: {e}")

    def setup_car_topics(self):
        """Setup subscribers and publishers for each car"""
        for car_name in self.car_names:
            # In single-car mode, topics have no namespace prefix
            # In multi-car mode, topics are namespaced: /car1/..., /car2/...
            if self.race_mode.startswith('single_car'):
                odom_topic = "/car_state/odom"
                frenet_topic = "/car_state/odom_frenet"
                state_transition_topic = "/state_transition"
            else:
                odom_topic = f"/{car_name}/car_state/odom"
                frenet_topic = f"/{car_name}/car_state/odom_frenet"
                state_transition_topic = f"/{car_name}/state_transition"

            # Subscribe to car odometry (Cartesian)
            rospy.Subscriber(odom_topic, Odometry,
                             lambda msg, name=car_name: self.car_odom_callback(msg, name))

            # Subscribe to Frenet odometry (if available)
            rospy.Subscriber(frenet_topic, Odometry,
                             lambda msg, name=car_name: self.car_frenet_callback(msg, name))

            # Subscribe to state transitions (for spliner/predictive_spliner OVERTAKE logging)
            rospy.Subscriber(state_transition_topic, String,
                             lambda msg, name=car_name: self.state_transition_callback(msg, name))

            # Publishers for collision warnings per car
            if self.race_mode.startswith('single_car'):
                collision_topic = "/collision_detected"
                warning_topic = "/collision_warning"
            else:
                collision_topic = f"/{car_name}/collision_detected"
                warning_topic = f"/{car_name}/collision_warning"

            self.collision_publishers[car_name] = rospy.Publisher(
                collision_topic, Bool, queue_size=10)
            self.warning_publishers[car_name] = rospy.Publisher(
                warning_topic, Bool, queue_size=10)

            # Initialize tracking
            self.car_laps[car_name] = 0
            self.car_previous_s[car_name] = None

        # Track boundary collision tracking (for cooldown)
        # {car_name: {'s': s_coord, 'timestamp': rospy.Time}}
        self.last_boundary_collision = {}

        # Event counters for single_car_obstacle mode
        self.successful_overtakes_count = 0
        self.obstacle_collision_count = 0
        self.boundary_collision_count = 0
        self.max_overtakes = rospy.get_param('/race_test/max_overtakes', 2)
        self.max_obstacle_collisions = rospy.get_param(
            '/race_test/max_obstacle_collisions', 2)
        self.max_boundary_collisions = rospy.get_param(
            '/race_test/max_boundary_collisions', 5)

        # Collision tracking with separation distance requirement
        # Track if currently in collision with obstacle
        self.in_collision_with_obstacle = False
        # Track if car1 and car2 are currently in collision
        self.in_collision_car1_car2 = False
        self.car_collision_count = 0  # Count collisions between cars in multi-car mode
        # Maximum car-to-car collisions before ending race
        self.max_car_collisions = rospy.get_param(
            '/race_test/max_car_collisions', 2)
        # meters - must separate by this much before counting another collision
        self.min_separation_distance = 2.0

        # Boundary collision behavior switch
        self.end_race_on_boundary_collision = rospy.get_param(
            '/race_test/end_race_on_boundary_collision', False)

        # Overtake state tracking
        # Track if car is locally behind obstacle (within overtake range)
        self.car_is_locally_behind = True
        self.car_ahead_distance = 0.0  # How far ahead the car is

        # Multi-car overtake state tracking
        self.car1_is_locally_behind = True  # Track if car1 is locally behind car2

        # Ego car stall detection (for all modes)
        # List of (timestamp, s_position) tuples
        self.ego_car_movement_history = []
        self.stall_check_duration = 15.0  # seconds
        self.stall_distance_threshold = 5.0  # meters

    def setup_obstacle_topics(self):
        """Setup subscribers for obstacle monitoring (single_car_obstacle mode)"""
        # Subscribe to obstacle odometry (Cartesian)
        rospy.Subscriber("/obstacle/odom", Odometry,
                         self.obstacle_odom_callback)

        # Subscribe to obstacle Frenet odometry
        rospy.Subscriber("/obstacle/odom_frenet", Odometry,
                         self.obstacle_frenet_callback)

        # Subscribe to obstacle state commands to know when it's ready
        rospy.Subscriber("/state_machine_cmd", String,
                         self.obstacle_state_callback)

        rospy.logwarn("[Race Monitor] ✓ Obstacle monitoring ENABLED")
        rospy.logwarn("[Race Monitor] ✓ Subscribed to /obstacle/odom_frenet")
        rospy.logwarn("[Race Monitor] ✓ Overtake detection: Car must be {:.1f}m ahead".format(
            self.overtake_lead_distance))

    def car_odom_callback(self, msg, car_name):
        """Store car Cartesian position from odometry"""
        self.car_positions[car_name] = {
            'pose': msg.pose.pose,
            'twist': msg.twist.twist,
            'timestamp': msg.header.stamp,
            'frame_id': msg.header.frame_id
        }

    def car_frenet_callback(self, msg, car_name):
        """Store car Frenet coordinates from frenet odometry"""
        # Frenet odom stores s in pose.position.x and d in pose.position.y
        s = msg.pose.pose.position.x
        d = msg.pose.pose.position.y

        self.car_frenet[car_name] = {
            's': s,
            'd': d,
            'timestamp': msg.header.stamp
        }

        # Update lap tracking
        self.update_lap_tracking(car_name, s)

    def obstacle_odom_callback(self, msg):
        """Store obstacle Cartesian position and velocity from odometry"""
        self.obstacle_position = {
            'pose': msg.pose.pose,
            'twist': msg.twist.twist,
            'timestamp': msg.header.stamp
        }

        # Calculate Frenet coordinates from Cartesian if we have a converter
        # For now, we'll get Frenet from the obstacle publisher directly via perception/obstacles
        # But store position for collision detection

    def obstacle_frenet_callback(self, msg):
        """Store obstacle Frenet coordinates from frenet odometry"""
        # Frenet odom stores s in pose.position.x and d in pose.position.y
        s = msg.pose.pose.position.x
        d = msg.pose.pose.position.y

        self.obstacle_frenet = {
            's': s,
            'd': d,
            'timestamp': msg.header.stamp
        }

        # Update lap tracking for obstacle
        self.update_obstacle_lap_tracking(s)

    def obstacle_state_callback(self, msg):
        """Track obstacle state changes"""
        self.obstacle_state = msg.data

    def update_lap_tracking(self, car_name, current_s):
        """Update lap tracking for a car based on s-coordinate"""
        previous_s = self.car_previous_s.get(car_name)

        if previous_s is not None:
            # Detect lap completion: large negative jump in s-coordinate
            if previous_s > (self.track_length * 0.8) and current_s < (self.track_length * 0.2):
                self.car_laps[car_name] += 1
                lap_num = self.car_laps[car_name]
                rospy.loginfo(
                    f"[Race Monitor] 🏁 {car_name} completed lap, now on lap {lap_num}/{self.target_laps}")

                # Log lap completion event
                self.log_event('lap_complete', car1_name=car_name,
                               details=f"{car_name} finished lap {lap_num}/{self.target_laps}")

                # Publish lap count as parameter for test framework
                rospy.set_param(f'/race_test/{car_name}/current_lap', lap_num)

                # Publish opponent lap complete flag when car2 completes first lap
                # This is used by collision_prediction to disable force_trailing
                if car_name == 'car2' and lap_num >= 1:
                    self.opponent_lap_complete_pub.publish(Bool(data=True))
                    rospy.loginfo(
                        f"[Race Monitor] Published opponent_lap_complete=True (force_trailing kept True for testing) for {car_name}")

        self.car_previous_s[car_name] = current_s

    def update_obstacle_lap_tracking(self, current_s):
        """Update lap tracking for obstacle based on s-coordinate"""
        previous_s = self.obstacle_previous_s

        if previous_s is not None:
            # Detect lap completion: large negative jump in s-coordinate
            if previous_s > (self.track_length * 0.8) and current_s < (self.track_length * 0.2):
                self.obstacle_laps += 1
                lap_num = self.obstacle_laps
                rospy.loginfo(
                    f"[Race Monitor] 🏁 obstacle completed lap, now on lap {lap_num}/{self.target_laps}")

                # Log lap completion event
                self.log_event('lap_complete', car1_name='obstacle',
                               details=f"obstacle finished lap {lap_num}/{self.target_laps}")

                # Publish opponent lap complete flag when obstacle completes first lap
                # This is used by collision_prediction to disable force_trailing
                if lap_num >= 1:
                    self.opponent_lap_complete_pub.publish(Bool(data=True))
                    rospy.loginfo(
                        f"[Race Monitor] Published opponent_lap_complete=True (force_trailing kept True for testing) for obstacle")

        self.obstacle_previous_s = current_s

    def state_transition_callback(self, msg, car_name):
        """Log state transitions to/from OVERTAKE for spliner and predictive_spliner"""
        transition = msg.data  # Format: "FROM_STATE -> TO_STATE"

        # Parse the transition
        if " -> " in transition:
            from_state, to_state = transition.split(" -> ")

            # Only log transitions involving OVERTAKE state
            # if "OVERTAKE" in transition:
            details = f"{car_name} state transition: {transition}"
            self.log_event('state_transition', car1_name=car_name,
                           details=details)
            rospy.loginfo(f"[Race Monitor] {details}")

    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)

    def quaternion_to_yaw(self, orientation):
        """Extract yaw angle from quaternion orientation"""
        # Convert quaternion to yaw using the formula:
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_bounding_box_corners(self, pose, length, width, margin=0.0):
        """
        Get the 4 corners of an oriented bounding box in world coordinates.

        Args:
            pose: geometry_msgs/Pose with position and orientation
            length: Length of the bounding box (along car's forward direction)
            width: Width of the bounding box (perpendicular to forward)
            margin: Additional margin to add to dimensions

        Returns:
            numpy array of shape (4, 2) with corner coordinates
        """
        # Get position and yaw
        cx = pose.position.x
        cy = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)

        # Half dimensions with margin
        half_length = (length + margin) / 2.0
        half_width = (width + margin) / 2.0

        # Rotation matrix
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Local corner coordinates (relative to center)
        # Front-left, Front-right, Rear-right, Rear-left
        local_corners = np.array([
            [half_length, half_width],
            [half_length, -half_width],
            [-half_length, -half_width],
            [-half_length, half_width]
        ])

        # Rotate and translate to world coordinates
        rotation_matrix = np.array([
            [cos_yaw, -sin_yaw],
            [sin_yaw, cos_yaw]
        ])

        world_corners = np.dot(
            local_corners, rotation_matrix.T) + np.array([cx, cy])
        return world_corners

    def project_polygon_onto_axis(self, corners, axis):
        """
        Project a polygon's corners onto an axis and return min/max projections.

        Args:
            corners: numpy array of shape (N, 2) with corner coordinates
            axis: numpy array of shape (2,) representing the axis direction

        Returns:
            tuple (min_projection, max_projection)
        """
        projections = np.dot(corners, axis)
        return np.min(projections), np.max(projections)

    def check_bounding_box_collision(self, pose1, pose2, length1, width1, length2, width2, margin=0.0):
        """
        Check if two oriented bounding boxes overlap using the Separating Axis Theorem (SAT).

        Args:
            pose1, pose2: geometry_msgs/Pose for each object
            length1, width1: Dimensions of first bounding box
            length2, width2: Dimensions of second bounding box
            margin: Additional collision margin

        Returns:
            tuple (is_colliding, overlap_distance) where overlap_distance is the minimum
            penetration depth (negative means separation distance)
        """
        # Get corners for both bounding boxes
        corners1 = self.get_bounding_box_corners(
            pose1, length1, width1, margin)
        corners2 = self.get_bounding_box_corners(
            pose2, length2, width2, margin)

        # Get the 4 axes to test (2 from each box - their edge normals)
        def get_axes(corners):
            axes = []
            for i in range(len(corners)):
                edge = corners[(i + 1) % len(corners)] - corners[i]
                # Normal to edge (perpendicular)
                normal = np.array([-edge[1], edge[0]])
                # Normalize
                norm = np.linalg.norm(normal)
                if norm > 1e-6:
                    axes.append(normal / norm)
            return axes

        axes = get_axes(corners1) + get_axes(corners2)

        min_overlap = float('inf')

        # Test each axis
        for axis in axes:
            min1, max1 = self.project_polygon_onto_axis(corners1, axis)
            min2, max2 = self.project_polygon_onto_axis(corners2, axis)

            # Check for overlap
            overlap = min(max1, max2) - max(min1, min2)

            if overlap < 0:
                # Separating axis found - no collision
                return False, -abs(overlap)

            min_overlap = min(min_overlap, overlap)

        # All axes have overlap - collision detected
        return True, min_overlap

    def check_bounding_box_warning(self, pose1, pose2, length1, width1, length2, width2,
                                   warning_margin, critical_margin):
        """
        Check warning/critical proximity using expanded bounding boxes.

        Returns:
            tuple (is_warning, is_critical) indicating proximity level
        """
        # Check critical first (smaller margin)
        is_critical, _ = self.check_bounding_box_collision(
            pose1, pose2, length1, width1, length2, width2, critical_margin)

        if is_critical:
            return True, True

        # Check warning (larger margin)
        is_warning, _ = self.check_bounding_box_collision(
            pose1, pose2, length1, width1, length2, width2, warning_margin)

        return is_warning, False

    def check_condition_lap_completion(self):
        """Condition 1: Car2 finished target laps while car1 is behind"""
        if self.car_laps.get('car2', 0) >= self.target_laps:
            if self.car_laps.get('car1', 0) < self.target_laps:
                reason = f"car2_finished_laps (car2: {self.car_laps['car2']} laps, car1: {self.car_laps['car1']} laps)"
                self.log_event('race_complete', car1_name='car1',
                               car2_name='car2', details=reason)
                self.set_race_complete(reason)
                return True
        return False

    def check_condition_collision(self):
        """Condition 2: Collision between cars using oriented bounding boxes.

        Uses Separating Axis Theorem (SAT) for accurate oriented bounding box collision.
        Tracks collision count with separation distance requirement - cars must separate
        by min_separation_distance before a new collision can be counted.
        Race ends when max_car_collisions is reached.
        """
        if len(self.car_names) < 2:
            return False

        for i, car1_name in enumerate(self.car_names):
            if car1_name not in self.car_positions:
                continue

            for j, car2_name in enumerate(self.car_names):
                if j <= i or car2_name not in self.car_positions:
                    continue

                pose1 = self.car_positions[car1_name]['pose']
                pose2 = self.car_positions[car2_name]['pose']

                # Calculate center-to-center distance for separation check
                distance = self.calculate_distance(pose1, pose2)

                # Check bounding box collision
                is_colliding, overlap = self.check_bounding_box_collision(
                    pose1, pose2,
                    self.car_length, self.car_width,
                    self.car_length, self.car_width,
                    self.bbox_collision_margin
                )

                if is_colliding:
                    # Currently in collision (bounding boxes overlap)
                    if not self.in_collision_car1_car2:
                        # NEW collision - cars were previously separated
                        self.car_collision_count += 1
                        self.in_collision_car1_car2 = True

                        reason = f"bbox_collision #{self.car_collision_count} (overlap: {overlap:.3f}m, distance: {distance:.2f}m between {car1_name} and {car2_name})"
                        self.log_event('collision', car1_name=car1_name, car2_name=car2_name,
                                       distance=distance, details=reason)

                        rospy.logwarn(
                            f"[Race Monitor] Car collision {self.car_collision_count}/{self.max_car_collisions} (bbox overlap: {overlap:.3f}m)")

                        # Check if max collisions reached
                        if self.car_collision_count >= self.max_car_collisions:
                            reason = f"max_car_collisions ({self.car_collision_count} bbox collisions between {car1_name} and {car2_name})"
                            self.set_race_complete(reason)
                            return True
                else:
                    # No bounding box overlap
                    if distance > self.min_separation_distance:
                        # Cars have separated enough - reset collision flag
                        self.in_collision_car1_car2 = False

        return False

    def check_condition_track_boundary(self):
        """Condition 3: Car crashed with track boundary (d-coordinate out of bounds)

        In multi_car mode, only checks car1 (ego car) boundary collisions.
        In single_car modes, checks the single car.
        """

        # Check if we have waypoints loaded
        if len(self.track_boundaries['left']) == 0:
            # No waypoints yet, skip boundary checking
            return False

        race_ending_collision = False

        # In multi_car mode, only check car1 (ego car) boundary collisions
        if self.race_mode == 'multi_car':
            cars_to_check = ['car1']
        else:
            cars_to_check = self.car_names

        for car_name in cars_to_check:
            if car_name not in self.car_frenet:
                continue

            s = self.car_frenet[car_name]['s']
            d = self.car_frenet[car_name]['d']

            # Get Cartesian coordinates for logging
            x = self.car_positions.get(car_name, {}).get('pose', None)
            if x:
                x_coord = x.position.x
                y_coord = x.position.y
            else:
                x_coord = 0.0
                y_coord = 0.0

            # Try to get actual track boundaries from global waypoints
            d_left, d_right = self.get_track_boundaries_at_s(s)

            # Add safety margin based on car width
            # d_left is positive (left boundary), d_right is negative (right boundary)
            d_left_limit = d_left - \
                (self.car_width / 2.0) - self.boundary_safety_margin
            d_right_limit = -abs(d_right) + \
                (self.car_width / 2.0) + self.boundary_safety_margin

            # Log boundary check info (at reduced rate - once per second per car)
            if not hasattr(self, '_last_boundary_log'):
                self._last_boundary_log = {}

            last_log = self._last_boundary_log.get(car_name, rospy.Time(0))
            if (rospy.Time.now() - last_log).to_sec() > 1.0:
                self._last_boundary_log[car_name] = rospy.Time.now()

            # Check if car is outside boundaries (with tolerance for minor scratches)
            # Car is off-track if: d > d_left_limit + tolerance (too far left) OR d < d_right_limit - tolerance (too far right)
            is_out_of_bounds = False
            violation_side = ""

            if d > d_left_limit + self.boundary_violation_tolerance:
                is_out_of_bounds = True
                violation_side = "LEFT"
            elif d < d_right_limit - self.boundary_violation_tolerance:
                is_out_of_bounds = True
                violation_side = "RIGHT"

            if is_out_of_bounds:
                # Check cooldown distance - only log if 10m away from last collision
                should_log = True
                last_collision = self.last_boundary_collision.get(car_name)

                if last_collision is not None:
                    # Calculate distance from last collision
                    s_diff = abs(s - last_collision['s'])
                    # Handle wrap-around at track end
                    if s_diff > self.track_length / 2:
                        s_diff = self.track_length - s_diff

                    if s_diff < 10.0:  # Within 10m cooldown
                        should_log = False

                if should_log:
                    # Update last collision tracking
                    self.last_boundary_collision[car_name] = {
                        's': s,
                        'timestamp': rospy.Time.now()
                    }

                    # Log the boundary collision
                    reason = f"track_boundary_collision ({car_name} off-track {violation_side}: x={x_coord:.2f}m, y={y_coord:.2f}m, s={s:.2f}m, d={d:.2f}m)"
                    self.log_event('track_crash', car1_name=car_name,
                                   details=reason)

                    # Increment boundary collision counter
                    self.boundary_collision_count += 1
                    rospy.logwarn(
                        f"[Race Monitor] ⚠️  Total boundary collisions: {self.boundary_collision_count}/{self.max_boundary_collisions}")

                    # Check if we've reached the maximum number of boundary collisions
                    if self.boundary_collision_count >= self.max_boundary_collisions:
                        if self.end_race_on_boundary_collision:
                            self.set_race_complete(
                                f"Track boundary collision limit reached ({self.boundary_collision_count})")
                            race_ending_collision = True
                            break
                        else:
                            rospy.logwarn(
                                f"[Race Monitor] ⚠️  Boundary collision limit reached but continuing race (end_race_on_boundary_collision=False)")
            else:
                # Car is within acceptable bounds (including tolerance)
                pass

        return race_ending_collision

    def check_condition_enemy_overlapped(self):
        """Check if car2 has overlapped car1 (2+ laps ahead)"""
        car1_lap = self.car_laps.get('car1', 0)
        car2_lap = self.car_laps.get('car2', 0)

        if car2_lap >= car1_lap + 2:
            reason = f"car2_overlapped_car1 (car2_lap={car2_lap}, car1_lap={car1_lap}, car2 is {car2_lap - car1_lap} laps ahead)"
            self.log_event('enemy_overlapped', car1_name='car1', car2_name='car2',
                           details=reason)
            self.set_race_complete(reason)
            return True
        return False

    def check_condition_ego_car_stalled(self):
        """Check if ego car (car1/car) has not moved more than stall_distance_threshold in stall_check_duration.

        Default thresholds: 5m movement required within 15 seconds.
        Applies to all race modes.
        """
        # Determine ego car name based on mode
        if self.race_mode.startswith('single_car'):
            ego_car_name = 'car'
        else:
            ego_car_name = 'car1'

        # Check if we have Frenet data for ego car
        if ego_car_name not in self.car_frenet:
            return False

        current_time = rospy.Time.now()
        current_s = self.car_frenet[ego_car_name]['s']
        current_lap = self.car_laps.get(ego_car_name, 0)

        # Calculate absolute s position (accounting for laps)
        current_absolute_s = current_lap * self.track_length + current_s

        # Add current position to history
        self.ego_car_movement_history.append(
            (current_time, current_absolute_s))

        # Remove entries older than stall_check_duration
        cutoff_time = current_time - rospy.Duration(self.stall_check_duration)
        self.ego_car_movement_history = [
            (t, s) for t, s in self.ego_car_movement_history if t > cutoff_time
        ]

        # Check if we have enough history (at least 1 second of data)
        if len(self.ego_car_movement_history) < 2:
            return False

        # Get oldest position in the window
        oldest_time, oldest_s = self.ego_car_movement_history[0]
        time_span = (current_time - oldest_time).to_sec()

        # Only check if we have at least 15 seconds of history
        if time_span < self.stall_check_duration:
            return False

        # Calculate distance traveled
        distance_traveled = abs(current_absolute_s - oldest_s)

        # Check if car hasn't moved enough
        if distance_traveled < self.stall_distance_threshold:
            reason = f"ego_car_stalled ({ego_car_name} moved only {distance_traveled:.2f}m in {time_span:.1f}s, threshold: {self.stall_distance_threshold}m in {self.stall_check_duration}s)"
            self.log_event('ego_stalled', car1_name=ego_car_name,
                           details=reason)
            self.set_race_complete(reason)
            return True

        return False

    def check_condition_overtake_lead(self):
        """Condition 4: Car1 overtook car2 and is 5+ meters ahead (using absolute s-coordinates)"""
        if 'car1' not in self.car_frenet or 'car2' not in self.car_frenet:
            return False

        # Get current positions (local s-coordinates)
        car1_s = self.car_frenet['car1']['s']
        car2_s = self.car_frenet['car2']['s']
        car1_lap = self.car_laps.get('car1', 0)
        car2_lap = self.car_laps.get('car2', 0)

        # Calculate absolute s positions (accounting for laps)
        car1_absolute_s = car1_lap * self.track_length + car1_s
        car2_absolute_s = car2_lap * self.track_length + car2_s

        # Use absolute difference for reliable detection
        absolute_diff = car1_absolute_s - car2_absolute_s

        # Check for successful overtake: car1 must be ahead by overtake_lead_distance
        if absolute_diff >= self.overtake_lead_distance:
            reason = f"car1_overtake_lead (car1 ahead by {absolute_diff:.2f}m, car1_abs={car1_absolute_s:.2f}m, car2_abs={car2_absolute_s:.2f}m, threshold: {self.overtake_lead_distance}m, car1_lap={car1_lap}, car2_lap={car2_lap})"
            self.log_event('overtake_lead', car1_name='car1', car2_name='car2',
                           distance=absolute_diff, details=reason)
            self.set_race_complete(reason)
            return True

        return False

    def check_near_miss(self):
        """Detect and log near-miss events using bounding box proximity"""
        if len(self.car_names) < 2:
            return

        for i, car1_name in enumerate(self.car_names):
            if car1_name not in self.car_positions:
                continue

            for j, car2_name in enumerate(self.car_names):
                if j <= i or car2_name not in self.car_positions:
                    continue

                pose1 = self.car_positions[car1_name]['pose']
                pose2 = self.car_positions[car2_name]['pose']
                distance = self.calculate_distance(pose1, pose2)

                # Check bounding box warning/critical using expanded margins
                # Warning margin: how much to expand bbox to detect warning zone
                warning_margin = self.warning_distance - \
                    (self.car_length / 2 + self.car_width / 2)
                critical_margin = self.critical_distance - \
                    (self.car_length / 2 + self.car_width / 2)

                is_warning, is_critical = self.check_bounding_box_warning(
                    pose1, pose2,
                    self.car_length, self.car_width,
                    self.car_length, self.car_width,
                    max(0, warning_margin), max(0, critical_margin)
                )

                # Check for actual collision (no extra margin beyond bbox_collision_margin)
                is_collision, _ = self.check_bounding_box_collision(
                    pose1, pose2,
                    self.car_length, self.car_width,
                    self.car_length, self.car_width,
                    self.bbox_collision_margin
                )

                # Near-miss: critical proximity but not actual collision
                if is_critical and not is_collision:
                    # Log at reduced rate (once per second per car pair)
                    if not hasattr(self, '_last_near_miss_log'):
                        self._last_near_miss_log = {}

                    key = f"{car1_name}_{car2_name}"
                    last_log = self._last_near_miss_log.get(key, rospy.Time(0))

                    if (rospy.Time.now() - last_log).to_sec() > 1.0:
                        self.log_event('near_miss', car1_name=car1_name, car2_name=car2_name,
                                       distance=distance,
                                       details=f"Critical bbox proximity (center dist: {distance:.2f}m)")
                        self._last_near_miss_log[key] = rospy.Time.now()

                # Publish warning/collision flags
                self.collision_publishers[car1_name].publish(
                    Bool(is_collision))
                self.collision_publishers[car2_name].publish(
                    Bool(is_collision))
                self.warning_publishers[car1_name].publish(Bool(is_warning))
                self.warning_publishers[car2_name].publish(Bool(is_warning))

    # ============================================
    # Single-Car Mode Condition Checks
    # ============================================

    def check_condition_single_car_lap_completion(self):
        """Single-car mode: Car completed target laps"""
        car_name = self.car_names[0]  # Single car

        if self.car_laps.get(car_name, 0) >= self.target_laps:
            reason = f"single_car_lap_completion ({car_name} completed {self.target_laps} laps)"
            self.log_event('race_complete', car1_name=car_name, details=reason)
            self.set_race_complete(reason)
            return True
        return False

    def check_condition_car_obstacle_collision(self):
        """Single-car obstacle mode: Collision between car and obstacle using oriented bounding boxes.

        Uses SAT for accurate collision detection. Tracks collision count with separation
        distance requirement - car and obstacle must separate by min_separation_distance
        before a new collision can be counted. Collision resets any ongoing overtake attempt.
        Race ends when max_obstacle_collisions is reached.
        """
        if not self.monitor_obstacle or self.obstacle_position is None:
            return False

        car_name = self.car_names[0]
        if car_name not in self.car_positions:
            return False

        car_pose = self.car_positions[car_name]['pose']
        obstacle_pose = self.obstacle_position['pose']

        # Calculate center-to-center distance for separation check
        distance = self.calculate_distance(car_pose, obstacle_pose)

        # Check bounding box collision (obstacle uses same dimensions as car)
        is_colliding, overlap = self.check_bounding_box_collision(
            car_pose, obstacle_pose,
            self.car_length, self.car_width,
            self.car_length, self.car_width,
            self.bbox_collision_margin
        )

        if is_colliding:
            # Currently in collision (bounding boxes overlap)
            if not self.in_collision_with_obstacle:
                # NEW collision - car and obstacle were previously separated
                self.obstacle_collision_count += 1
                self.in_collision_with_obstacle = True

                # Reset overtake state - collision invalidates any ongoing overtake attempt
                self.car_ahead_distance = 0.0

                reason = f"car_obstacle_bbox_collision (overlap: {overlap:.3f}m, distance: {distance:.2f}m, collision #{self.obstacle_collision_count})"
                self.log_event('collision', car1_name=car_name, car2_name='obstacle',
                               distance=distance, details=reason)

                rospy.logwarn(
                    f"[Race Monitor] Obstacle collision {self.obstacle_collision_count}/{self.max_obstacle_collisions} (bbox overlap: {overlap:.3f}m)")

                if self.obstacle_collision_count >= self.max_obstacle_collisions:
                    reason = f"max_obstacle_collisions ({self.obstacle_collision_count} bbox collisions)"
                    self.set_race_complete(reason)
                    return True
        else:
            # No bounding box overlap
            if distance > self.min_separation_distance:
                # Car and obstacle have separated enough - reset collision flag
                self.in_collision_with_obstacle = False

        return False

    def check_condition_obstacle_overlapped(self):
        """Check if obstacle has overlapped car (2+ laps ahead)"""
        car_name = self.car_names[0]
        car_lap = self.car_laps.get(car_name, 0)
        obstacle_lap = self.obstacle_laps

        if obstacle_lap >= car_lap + 2:
            reason = f"obstacle_overlapped_car (obstacle_lap={obstacle_lap}, car_lap={car_lap}, obstacle is {obstacle_lap - car_lap} laps ahead)"
            self.log_event('enemy_overlapped', car1_name=car_name, car2_name='obstacle',
                           details=reason)
            self.set_race_complete(reason)
            return True
        return False

    def check_condition_car_overtake_obstacle(self):
        """Single-car obstacle mode: Car overtook obstacle by overtake_lead_distance.

        Uses absolute s-coordinates (accounting for lap count) for reliable detection.
        Race ends when max_overtakes is reached.
        """
        # Check if obstacle monitoring is enabled
        if not self.monitor_obstacle:
            return False

        # Check if obstacle Frenet data is available
        if self.obstacle_frenet is None:
            return False

        car_name = self.car_names[0]
        if car_name not in self.car_frenet:
            return False

        # Get current positions (local s-coordinates)
        car_s = self.car_frenet[car_name]['s']
        obstacle_s = self.obstacle_frenet['s']
        car_lap = self.car_laps.get(car_name, 0)
        obstacle_lap = self.obstacle_laps

        # Calculate absolute s positions first (accounting for laps)
        car_absolute_s = car_lap * self.track_length + car_s
        obstacle_absolute_s = obstacle_lap * self.track_length + obstacle_s

        # Use absolute difference for more reliable detection
        absolute_diff = car_absolute_s - obstacle_absolute_s
        print(
            f"{absolute_diff=:.2f}m (car_abs={car_absolute_s:.2f}m, obs_abs={obstacle_absolute_s:.2f}m) (car_lap={car_lap}, obs_lap={obstacle_lap})", flush=True)

        if absolute_diff >= self.overtake_lead_distance:
            self.successful_overtakes_count += 1

            reason = f"successful_overtake #{self.successful_overtakes_count} ({car_name} ahead by {absolute_diff:.2f}m locally, car_abs={car_absolute_s:.2f}m, obs_abs={obstacle_absolute_s:.2f}m, car_lap={car_lap}, obstacle_lap={obstacle_lap})"
            self.log_event('overtake_lead', car1_name=car_name, car2_name='obstacle',
                           distance=absolute_diff, details=reason)

            # Check if max overtakes reached
            if self.successful_overtakes_count >= self.max_overtakes:
                reason = f"max_successful_overtakes ({self.successful_overtakes_count} overtakes)"
                self.set_race_complete(reason)
                return True
        return False

    def set_race_complete(self, reason):
        """Set race completion flag and reason"""
        if not self.race_complete:  # Only log first completion
            self.race_complete = True
            self.race_complete_reason = reason

            rospy.set_param('/race_test/simulation_complete', True)
            rospy.set_param('/race_test/race_complete_reason', reason)

            race_duration = (rospy.Time.now() - self.race_start_time).to_sec()
            rospy.logwarn(
                f"🏁 RACE COMPLETE after {race_duration:.1f}s: {reason}")

    def monitor_race(self, event):
        """Main monitoring function - called by timer"""
        if self.race_complete:
            return  # Race already finished

        current_time = rospy.Time.now()

        # Check ego car stall condition FIRST (applies to all modes)
        if self.check_condition_ego_car_stalled():
            return

        # Check race completion conditions based on mode
        if self.race_mode == 'multi_car':
            # Multi-car specific conditions
            if self.check_condition_lap_completion():
                return
            if self.check_condition_collision():
                return
            if self.check_condition_enemy_overlapped():
                return
            if self.check_condition_overtake_lead():
                return
            # Track boundary check is common to all modes
            if self.check_condition_track_boundary():
                return
            # Check for events that don't end the race
            self.check_near_miss()

        elif self.race_mode == 'single_car_no_obstacle':
            # Single car time trial conditions
            if self.check_condition_single_car_lap_completion():
                return
            if self.check_condition_track_boundary():
                return

        elif self.race_mode == 'single_car_obstacle':
            # Single car vs obstacle conditions
            # Check overtakes FIRST (before lap completion) to ensure they're detected
            if self.check_condition_obstacle_overlapped():
                return
            if self.check_condition_car_obstacle_collision():
                return
            if self.check_condition_car_overtake_obstacle():
                return
            if self.check_condition_track_boundary():
                return
            if self.check_condition_single_car_lap_completion():
                return

        # Publish status periodically
        self.publish_race_status()

    def publish_race_status(self):
        """Publish current race status"""
        status_parts = []

        # Add lap information
        for car_name in self.car_names:
            lap = self.car_laps.get(car_name, 0)
            status_parts.append(f"{car_name}: Lap {lap}/{self.target_laps}")

        # Add relative position if Frenet data available
        if 'car1' in self.car_frenet and 'car2' in self.car_frenet:
            s_diff = self.car_frenet['car1']['s'] - \
                self.car_frenet['car2']['s']
            if s_diff > self.track_length / 2:
                s_diff -= self.track_length
            elif s_diff < -self.track_length / 2:
                s_diff += self.track_length

            if s_diff > 0:
                status_parts.append(f"car1 ahead by {s_diff:.1f}m")
            else:
                status_parts.append(f"car2 ahead by {abs(s_diff):.1f}m")

        status_msg = " | ".join(status_parts)
        self.status_publisher.publish(String(status_msg))


def main():
    try:
        monitor = RaceEventMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Race Event Monitor terminated")


if __name__ == '__main__':
    main()
