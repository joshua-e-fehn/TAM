#!/usr/bin/env python3

"""
Race Event Monitor Node

This node monitors races and:
1. Detects race completion conditions
2. Logs race events (collisions, near-misses, overtakes, crashes)
3. Sets /simulation_complete when race ends
4. Provides event data for post-race analysis

Modes:
- multi_car: Two cars racing (default)
- single_car_no_obstacle: Single car time trial
- single_car_obstacle: Single car vs dummy obstacle

Completion Conditions (mode-dependent):
Multi-car:
  - Car2 finishes target laps while car1 is behind
  - Collision between cars
  - Car crashes with track boundary
  - Car1 overtakes car2 by lead distance

Single car (no obstacle):
  - Car completes target laps
  - Car crashes with track boundary

Single car (with obstacle):
  - Car completes target laps
  - Collision with obstacle
  - Car or obstacle crashes with track boundary
  - Car overtakes obstacle by lead distance

Author: Atlas  
Date: October 2025
"""

import rospy
import math
import csv
import os
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
            # rospy.loginfo(
            #     f"[Race Monitor] Using track length: {self.track_length:.2f}m")
        except:
            self.track_length = rospy.get_param('~track_length', 76.48)
            # rospy.logwarn(
            # f"[Race Monitor] Using default track length: {self.track_length:.2f}m")

        # Load global waypoints for track boundary checking
        self.global_waypoints = WpntArray()
        self.track_length = 0.0
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

            # rospy.loginfo(
            #     f"[Race Monitor] Loaded {len(msg.wpnts)} global waypoints for boundary checking")
            # rospy.loginfo(
            #     f"[Race Monitor] Track length from waypoints: {self.track_length:.2f}m")

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

        # rospy.loginfo(f"[Race Monitor] Event log created: {log_file}")
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

        # Also log to rosout for debugging
        # rospy.loginfo(f"[Event] {event_type}: {details}")

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
        """Condition 2: Collision between cars with separation distance requirement"""
        if len(self.car_names) < 2:
            return False

        for i, car1_name in enumerate(self.car_names):
            if car1_name not in self.car_positions:
                continue

            for j, car2_name in enumerate(self.car_names):
                if j <= i or car2_name not in self.car_positions:
                    continue

                distance = self.calculate_distance(
                    self.car_positions[car1_name]['pose'],
                    self.car_positions[car2_name]['pose']
                )

                if distance <= self.collision_distance:
                    # Currently in collision zone
                    if not self.in_collision_car1_car2:
                        # NEW collision - cars were previously separated
                        self.car_collision_count += 1
                        self.in_collision_car1_car2 = True

                        reason = f"collision #{self.car_collision_count} (distance: {distance:.2f}m between {car1_name} and {car2_name})"
                        self.log_event('collision', car1_name=car1_name, car2_name=car2_name,
                                       distance=distance, details=reason)

                        rospy.logwarn(
                            f"[Race Monitor] Car collision {self.car_collision_count}/{self.max_car_collisions}")

                        # Check if max collisions reached
                        if self.car_collision_count >= self.max_car_collisions:
                            reason = f"max_car_collisions ({self.car_collision_count} collisions between {car1_name} and {car2_name})"
                            self.set_race_complete(reason)
                            return True
                else:
                    # Outside collision zone
                    if distance > self.min_separation_distance:
                        # Cars have separated enough - reset collision flag
                        self.in_collision_car1_car2 = False

        return False

    def check_condition_track_boundary(self):
        """Condition 3: Car crashed with track boundary (d-coordinate out of bounds)"""

        # Check if we have waypoints loaded
        if len(self.track_boundaries['left']) == 0:
            # No waypoints yet, skip boundary checking
            return False

        race_ending_collision = False

        for car_name in self.car_names:
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
                # rospy.loginfo(
                #     f"[Boundary Check] {car_name}: s={s:.2f}m, d={d:.2f}m, left_limit={d_left_limit:.2f}m, right_limit={d_right_limit:.2f}m")
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
        """Check if ego car (car1/car) has not moved more than 3m in last 15 seconds"""
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
        """Condition 4: Car1 overtook car2 and is 5+ meters ahead"""
        if 'car1' not in self.car_frenet or 'car2' not in self.car_frenet:
            return False

        # Get current positions (local s-coordinates)
        car1_s = self.car_frenet['car1']['s']
        car2_s = self.car_frenet['car2']['s']
        car1_lap = self.car_laps.get('car1', 0)
        car2_lap = self.car_laps.get('car2', 0)

        # Calculate local s-coordinate difference (handling wrap-around)
        local_s_diff = car1_s - car2_s

        # Normalize to shortest path around circular track
        # If the difference is more than half the track length,
        # the shorter path is in the opposite direction
        if local_s_diff > self.track_length / 2:
            # Car1 is actually behind (shorter to go backwards)
            local_s_diff = local_s_diff - self.track_length
        elif local_s_diff < -self.track_length / 2:
            # Car1 is actually ahead (shorter to go forwards)
            local_s_diff = local_s_diff + self.track_length

        # Now local_s_diff is in range [-track_length/2, +track_length/2]
        # Positive means car1 is ahead, negative means car1 is behind

        # Check if car1 comes close to car2 again (within 10m behind)
        # This resets the locally_behind flag
        if local_s_diff > -10.0 and local_s_diff < 0.0:
            if not self.car1_is_locally_behind:
                self.car1_is_locally_behind = True
                self.log_event('overtake_state_change', car1_name='car1', car2_name='car2',
                               distance=local_s_diff,
                               details=f"Car1 is now locally behind car2 (local_diff={local_s_diff:.2f}m, car1_lap={car1_lap}, car2_lap={car2_lap})")

        # Check for successful overtake:
        # Car1 must be locally behind AND local position must be 10m+ ahead
        # AND absolute position must be ahead (to ensure proper lap accounting)
        car1_absolute_s = car1_lap * self.track_length + car1_s
        car2_absolute_s = car2_lap * self.track_length + car2_s

        if self.car1_is_locally_behind and local_s_diff >= self.overtake_lead_distance and car1_absolute_s > car2_absolute_s:
            # Overtake detected! Update state and end race
            self.car1_is_locally_behind = False

            reason = f"car1_overtake_lead (car1 ahead by {local_s_diff:.2f}m locally, car1_abs={car1_absolute_s:.2f}m, car2_abs={car2_absolute_s:.2f}m, threshold: {self.overtake_lead_distance}m, car1_lap={car1_lap}, car2_lap={car2_lap})"
            self.log_event('overtake_lead', car1_name='car1', car2_name='car2',
                           distance=local_s_diff, details=reason)
            self.set_race_complete(reason)
            return True

        return False

    def check_near_miss(self):
        """Detect and log near-miss events (close calls without collision)"""
        if len(self.car_names) < 2:
            return

        for i, car1_name in enumerate(self.car_names):
            if car1_name not in self.car_positions:
                continue

            for j, car2_name in enumerate(self.car_names):
                if j <= i or car2_name not in self.car_positions:
                    continue

                distance = self.calculate_distance(
                    self.car_positions[car1_name]['pose'],
                    self.car_positions[car2_name]['pose']
                )

                # Near-miss: between collision and critical distance
                if self.collision_distance < distance <= self.critical_distance:
                    # Log at reduced rate (once per second per car pair)
                    if not hasattr(self, '_last_near_miss_log'):
                        self._last_near_miss_log = {}

                    key = f"{car1_name}_{car2_name}"
                    last_log = self._last_near_miss_log.get(key, rospy.Time(0))

                    if (rospy.Time.now() - last_log).to_sec() > 1.0:
                        self.log_event('near_miss', car1_name=car1_name, car2_name=car2_name,
                                       distance=distance,
                                       details=f"Close encounter: {distance:.2f}m")
                        self._last_near_miss_log[key] = rospy.Time.now()

                # Publish warning/collision flags
                warning = distance <= self.warning_distance
                collision = distance <= self.collision_distance

                self.collision_publishers[car1_name].publish(Bool(collision))
                self.collision_publishers[car2_name].publish(Bool(collision))
                self.warning_publishers[car1_name].publish(Bool(warning))
                self.warning_publishers[car2_name].publish(Bool(warning))

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
        """Single-car obstacle mode: Collision between car and obstacle with separation distance requirement"""
        if not self.monitor_obstacle or self.obstacle_position is None:
            return False

        car_name = self.car_names[0]
        if car_name not in self.car_positions:
            return False

        distance = self.calculate_distance(
            self.car_positions[car_name]['pose'],
            self.obstacle_position['pose']
        )

        if distance <= self.collision_distance:
            # Currently in collision zone
            if not self.in_collision_with_obstacle:
                # NEW collision - car and obstacle were previously separated
                self.obstacle_collision_count += 1
                self.in_collision_with_obstacle = True

                # Reset overtake state - collision invalidates any ongoing overtake attempt
                self.car_ahead_distance = 0.0

                reason = f"car_obstacle_collision (distance: {distance:.2f}m, collision #{self.obstacle_collision_count})"
                self.log_event('collision', car1_name=car_name, car2_name='obstacle',
                               distance=distance, details=reason)

                rospy.logwarn(
                    f"[Race Monitor] Obstacle collision {self.obstacle_collision_count}/{self.max_obstacle_collisions}")

                if self.obstacle_collision_count >= self.max_obstacle_collisions:
                    reason = f"max_obstacle_collisions ({self.obstacle_collision_count} collisions)"
                    self.set_race_complete(reason)
                    return True
        else:
            # Outside collision zone
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
        """Single-car obstacle mode: Car overtook obstacle by lead distance"""
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

        # Calculate local s-coordinate difference (handling wrap-around)
        local_s_diff = car_s - obstacle_s

        # Normalize to shortest path around circular track
        # If the difference is more than half the track length,
        # the shorter path is in the opposite direction
        if local_s_diff > self.track_length / 2:
            # Car is actually behind (shorter to go backwards)
            local_s_diff = local_s_diff - self.track_length
        elif local_s_diff < -self.track_length / 2:
            # Car is actually ahead (shorter to go forwards)
            local_s_diff = local_s_diff + self.track_length

        # Now local_s_diff is in range [-track_length/2, +track_length/2]
        # Positive means car is ahead, negative means car is behind

        # Store distance for logging
        self.car_ahead_distance = local_s_diff

        # Check if car comes close to obstacle again (within 10m behind)
        # This resets the locally_behind flag
        if local_s_diff > -10.0 and local_s_diff < 0.0:
            if not self.car_is_locally_behind:
                self.car_is_locally_behind = True
                self.log_event('overtake_state_change', car1_name=car_name, car2_name='obstacle',
                               distance=local_s_diff,
                               details=f"Car is now locally behind obstacle (local_diff={local_s_diff:.2f}m, car_lap={car_lap}, obs_lap={obstacle_lap})")

        # Check for successful overtake:
        # Car must be locally behind AND local position must be 10m+ ahead
        # AND absolute position must be ahead (to ensure proper lap accounting)
        car_absolute_s = car_lap * self.track_length + car_s
        obstacle_absolute_s = obstacle_lap * self.track_length + obstacle_s

        if self.car_is_locally_behind and local_s_diff > self.overtake_lead_distance and car_absolute_s > obstacle_absolute_s:
            # Overtake detected! Update state immediately
            self.car_is_locally_behind = False
            self.successful_overtakes_count += 1

            reason = f"successful_overtake #{self.successful_overtakes_count} ({car_name} ahead by {local_s_diff:.2f}m locally, car_abs={car_absolute_s:.2f}m, obs_abs={obstacle_absolute_s:.2f}m, car_lap={car_lap}, obstacle_lap={obstacle_lap})"
            self.log_event('overtake_lead', car1_name=car_name, car2_name='obstacle',
                           distance=local_s_diff, details=reason)

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
