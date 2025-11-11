#!/usr/bin/env python3

"""
Race Event Monitor Node

This node monitors multi-car races and:
1. Detects race completion conditions
2. Logs race events (collisions, near-misses, overtakes, crashes)
3. Sets /simulation_complete when race ends
4. Provides event data for post-race analysis

Completion Conditions:
- Car2 finishes target laps while car1 is behind
- Collision between cars
- Car crashes with track boundary
- Car1 overtakes car2 by lead distance

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

        # Car configuration
        car_names_param = rospy.get_param('~car_names', 'car1,car2')
        if isinstance(car_names_param, str):
            self.car_names = [name.strip()
                              for name in car_names_param.split(',')]
        else:
            self.car_names = car_names_param

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
            '/race_test/overtake_lead_distance', 10.0)  # meters
        self.boundary_safety_margin = rospy.get_param(
            '/race_test/boundary_safety_margin', 0.0)  # meters - additional safety margin

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
            '/race_test/car_length', 0.58)  # meters
        self.car_width = rospy.get_param(
            '/race_test/car_width', 0.31)    # meters

        # Storage for car data
        self.car_positions = {}  # Cartesian positions
        self.car_frenet = {}     # Frenet coordinates (s, d)
        self.car_laps = {}       # Lap counts
        self.car_previous_s = {}  # Previous s for lap detection

        # Race state
        self.race_complete = False
        self.race_complete_reason = None
        self.race_start_time = rospy.Time.now()

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

        # Main monitoring timer
        self.timer = rospy.Timer(rospy.Duration(
            1.0/self.check_rate), self.monitor_race)

        # rospy.loginfo(
        #     f"Race Event Monitor initialized for cars: {self.car_names}")
        # rospy.loginfo(
        #     f"Target laps: {self.target_laps}, Overtake lead: {self.overtake_lead_distance}m")
        # rospy.loginfo(f"Event log: {self.log_file_path}")

    def setup_global_topics(self):
        """Setup global subscribers and publishers"""
        rospy.Subscriber("global_waypoints", WpntArray,
                         self.global_waypoints_callback, queue_size=1)

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
                                       os.path.join(os.path.expanduser('~'), 'catkin_ws', 'testSimulation', 'logs'))

        # Create batch-specific subdirectory
        # batch_number can be string or int, handle both
        if batch_number and str(batch_number) != '0':
            batch_dir = os.path.join(base_log_dir, f"batch_{batch_number}")
            log_filename = f"race_events_sim{simulation_id}_{timestamp}.csv"
        else:
            # Fallback: use 'unbatched' directory if batch_number not set
            batch_dir = os.path.join(base_log_dir, "unbatched")
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
            # Subscribe to car odometry (Cartesian)
            odom_topic = f"/{car_name}/car_state/odom"
            rospy.Subscriber(odom_topic, Odometry,
                             lambda msg, name=car_name: self.car_odom_callback(msg, name))

            # Subscribe to Frenet odometry (if available)
            frenet_topic = f"/{car_name}/car_state/odom_frenet"
            rospy.Subscriber(frenet_topic, Odometry,
                             lambda msg, name=car_name: self.car_frenet_callback(msg, name))

            # Subscribe to state transitions (for spliner/predictive_spliner OVERTAKE logging)
            state_transition_topic = f"/{car_name}/state_transition"
            rospy.Subscriber(state_transition_topic, String,
                             lambda msg, name=car_name: self.state_transition_callback(msg, name))

            # Publishers for collision warnings per car
            self.collision_publishers[car_name] = rospy.Publisher(
                f"/{car_name}/collision_detected", Bool, queue_size=10)
            self.warning_publishers[car_name] = rospy.Publisher(
                f"/{car_name}/collision_warning", Bool, queue_size=10)

            # Initialize tracking
            self.car_laps[car_name] = 0
            self.car_previous_s[car_name] = None

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

    def update_lap_tracking(self, car_name, current_s):
        """Update lap tracking for a car based on s-coordinate"""
        previous_s = self.car_previous_s.get(car_name)

        if previous_s is not None:
            # Detect lap completion: large negative jump in s-coordinate
            if previous_s > (self.track_length * 0.8) and current_s < (self.track_length * 0.2):
                self.car_laps[car_name] += 1
                lap_num = self.car_laps[car_name]
                rospy.loginfo(
                    f"[Race Monitor] 🏁 {car_name} completed lap, now on lap {lap_num}")

                # Log lap completion event
                self.log_event('lap_complete', car1_name=car_name,
                               details=f"{car_name} finished lap {lap_num}")

                # Publish lap count as parameter for test framework
                rospy.set_param(f'/race_test/{car_name}/current_lap', lap_num)

        self.car_previous_s[car_name] = current_s

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
        """Condition 2: Collision between cars"""
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
                    reason = f"collision (distance: {distance:.2f}m between {car1_name} and {car2_name})"
                    self.log_event('collision', car1_name=car1_name, car2_name=car2_name,
                                   distance=distance, details=reason)
                    self.set_race_complete(reason)
                    return True

        return False

    def check_condition_track_boundary(self):
        """Condition 3: Car crashed with track boundary (d-coordinate out of bounds)"""

        # Check if we have waypoints loaded
        if len(self.track_boundaries['left']) == 0:
            # No waypoints yet, skip boundary checking
            return False

        for car_name in self.car_names:
            if car_name not in self.car_frenet:
                continue

            s = self.car_frenet[car_name]['s']
            d = self.car_frenet[car_name]['d']

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

            # Check if car is outside boundaries
            # Car is off-track if: d > d_left_limit (too far left) OR d < d_right_limit (too far right)
            if d > d_left_limit:
                reason = f"track_boundary_crash ({car_name} off-track LEFT: d={d:.2f}m, limit={d_left_limit:.2f}m, s={s:.2f}m)"
                self.log_event('track_crash', car1_name=car_name,
                               details=reason)
                self.set_race_complete(reason)
                return True
            elif d < d_right_limit:
                reason = f"track_boundary_crash ({car_name} off-track RIGHT: d={d:.2f}m, limit={d_right_limit:.2f}m, s={s:.2f}m)"
                self.log_event('track_crash', car1_name=car_name,
                               details=reason)
                self.set_race_complete(reason)
                return True

        return False

    def check_condition_overtake_lead(self):
        """Condition 4: Car1 overtook car2 and is 10+ meters ahead"""
        if 'car1' not in self.car_frenet or 'car2' not in self.car_frenet:
            return False

        car1_s = self.car_frenet['car1']['s']
        car2_s = self.car_frenet['car2']['s']
        car1_lap = self.car_laps.get('car1', 0)
        car2_lap = self.car_laps.get('car2', 0)

        # Calculate absolute position including laps
        car1_absolute_s = car1_lap * self.track_length + car1_s
        car2_absolute_s = car2_lap * self.track_length + car2_s

        # Calculate actual distance difference
        s_diff = car1_absolute_s - car2_absolute_s

        # Check if car1 is ahead by the lead distance
        # Only trigger if car1 is actually ahead (positive difference)
        if s_diff >= self.overtake_lead_distance:
            reason = f"car1_overtake_lead (car1 ahead by {s_diff:.2f}m, threshold: {self.overtake_lead_distance}m, car1_lap={car1_lap}, car2_lap={car2_lap})"
            self.log_event('overtake_lead', car1_name='car1', car2_name='car2',
                           distance=s_diff, details=reason)
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

        # Check all race completion conditions
        if self.check_condition_lap_completion():
            return

        if self.check_condition_collision():
            return

        if self.check_condition_track_boundary():
            return

        if self.check_condition_overtake_lead():
            return

        # Check for events that don't end the race (near-misses, etc.)
        self.check_near_miss()

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
