#!/usr/bin/env python3

"""
Multi-Car Obstacle Publisher Node

This node creates a system for multi-car interaction by:
1. Subscribing to all car positions  
2. Converting car positions to Frenet coordinates
3. Publishing other cars as f110_msgs/Obstacle messages (same as dummy obstacles)
4. Enabling collision avoidance through existing perception/planning pipeline

Author: Atlas
Date: September 2025
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
import math
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray, OpponentTrajectory, OppWpnt
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from frenet_conversion.srv import Glob2FrenetArr, Frenet2GlobArr
from tf2_geometry_msgs import do_transform_pose


class MultiCarObstaclePublisher:
    def __init__(self):
        rospy.init_node('multi_car_obstacle_publisher', anonymous=True)

        # Car configuration
        car_names_param = rospy.get_param('~car_names', 'car1,car2')
        if isinstance(car_names_param, str):
            self.car_names = [name.strip()
                              for name in car_names_param.split(',')]
        else:
            self.car_names = car_names_param
        self.publish_rate = rospy.get_param(
            '~publish_rate', 50)  # Hz - Updated to 50Hz

        # Car model and dimensions
        self.car_model = rospy.get_param('~car_model', 'NUC2')

        # Try to get dimensions from car model, fallback to launch file params, then defaults
        self.car_length = self.get_car_dimension('car_length', 0.58)  # meters
        self.car_width = self.get_car_dimension('car_width', 0.31)    # meters
        self.safety_margin = rospy.get_param('~safety_margin', 0.2)   # meters

        # Detection range (only consider cars within this distance)
        self.max_detection_range = rospy.get_param(
            '~max_detection_range', 20.0)  # meters

        # Track length for handling wraparound in relative coordinate calculation
        # Try to get actual track length from global parameter, fallback to default
        try:
            self.track_length = rospy.get_param(
                '/global_republisher/track_length')
            rospy.loginfo(
                f"[Multi-Car Publisher] Using actual track length: {self.track_length:.2f}m")
        except Exception:
            # meters - Fixed default to match actual track
            self.track_length = rospy.get_param('~track_length', 76.48)
            rospy.logwarn(
                f"[Multi-Car Publisher] Using default track length: {self.track_length:.2f}m")

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Storage for car positions and velocities
        self.car_positions = {}
        self.car_subscribers = {}

        # Lap tracking for each car
        self.car_laps = {}  # Track current lap number for each car
        self.car_previous_s = {}  # Track previous s-coordinate to detect lap completion

        # Publishers for obstacles (per car) - Using f110_msgs/ObstacleArray
        self.obstacle_publishers = {}
        self.marker_publishers = {}
        self.opponent_waypoints_publishers = {}

        # Frenet Conversion Services (per car)
        self.frenet_converters = {}
        self.setup_frenet_services()

        # Initialize subscribers and publishers for each car
        self.setup_car_topics()

        # Initialize lap tracking for all cars
        for car_name in self.car_names:
            self.car_laps[car_name] = 0
            self.car_previous_s[car_name] = None

        # Main publishing timer
        self.timer = rospy.Timer(rospy.Duration(
            1.0/self.publish_rate), self.publish_obstacles)

        rospy.loginfo(
            f"Multi-Car Obstacle Publisher initialized for cars: {self.car_names}")

    def get_car_dimension(self, dimension_name, default_value):
        """Get car dimension from model parameters with fallback hierarchy"""
        # 1. Try to get from launch file parameters (highest priority)
        launch_param = rospy.get_param(f'~{dimension_name}', None)
        if launch_param is not None:
            rospy.loginfo(
                f"[Multi-Car Publisher] Using {dimension_name}={launch_param} from launch file")
            return launch_param

        # 2. Try to calculate from car model parameters
        try:
            if dimension_name == 'car_length':
                # Calculate car length from wheelbase (wheelbase + front/rear overhangs ≈ 1.9x wheelbase)
                wheelbase = rospy.get_param(
                    '/car_model_params/wheelbase', None)
                if wheelbase is not None:
                    # Based on F1TENTH proportions (0.307 -> 0.58)
                    calculated_length = wheelbase * 1.89
                    rospy.loginfo(
                        f"[Multi-Car Publisher] Calculated car_length={calculated_length:.3f} from wheelbase={wheelbase} ({self.car_model})")
                    return calculated_length
            elif dimension_name == 'car_width':
                # Use standard F1TENTH width (fairly constant across models)
                standard_width = 0.31  # F1TENTH standard width
                rospy.loginfo(
                    f"[Multi-Car Publisher] Using standard F1TENTH car_width={standard_width} ({self.car_model})")
                return standard_width
        except Exception as e:
            rospy.logwarn(
                f"[Multi-Car Publisher] Could not calculate {dimension_name} from car model {self.car_model}: {e}")

        # 3. Fallback to default value
        rospy.loginfo(
            f"[Multi-Car Publisher] Using default {dimension_name}={default_value}")
        return default_value

    def setup_frenet_services(self):
        """Setup connection to Frenet conversion services for each car"""
        for car_name in self.car_names:
            try:
                service_name = f"/{car_name}/convert_glob2frenetarr_service"
                rospy.loginfo(
                    f"Waiting for frenet conversion service: {service_name}")
                rospy.wait_for_service(service_name, timeout=5.0)
                self.frenet_converters[car_name] = rospy.ServiceProxy(
                    service_name, Glob2FrenetArr)
                rospy.loginfo(
                    f"Connected to frenet conversion service for {car_name}")
            except rospy.ROSException:
                rospy.logwarn(
                    f"Frenet conversion service not available for {car_name}. Using direct coordinate conversion.")
                self.frenet_converters[car_name] = None

    def setup_car_topics(self):
        """Setup subscribers and publishers for each car"""
        for car_name in self.car_names:
            # Subscribe to car odometry
            odom_topic = f"/{car_name}/car_state/odom"
            self.car_subscribers[car_name] = rospy.Subscriber(
                odom_topic, Odometry,
                lambda msg, name=car_name: self.car_odom_callback(msg, name)
            )

            # Publisher for obstacles seen by this car - Using standard perception topic
            obstacle_topic = f"/{car_name}/perception/multi_car_obstacles"
            self.obstacle_publishers[car_name] = rospy.Publisher(
                obstacle_topic, ObstacleArray, queue_size=10
            )

            # Publisher for visualization
            viz_topic = f"/{car_name}/car_obstacles_viz"
            self.marker_publishers[car_name] = rospy.Publisher(
                viz_topic, MarkerArray, queue_size=10
            )

            # Publisher for opponent waypoints (full trajectory)
            opponent_waypoints_topic = f"/{car_name}/opponent_waypoints"
            self.opponent_waypoints_publishers[car_name] = rospy.Publisher(
                opponent_waypoints_topic, OpponentTrajectory, queue_size=10
            )

            rospy.loginfo(
                f"[Multi-Car Publisher] Setup topics for {car_name}: {odom_topic} -> {obstacle_topic}")

    def car_odom_callback(self, msg, car_name):
        """Store car position from odometry"""
        self.car_positions[car_name] = {
            'pose': msg.pose.pose,
            'twist': msg.twist.twist,
            'timestamp': msg.header.stamp,
            'frame_id': msg.header.frame_id
        }

        # Initialize lap tracking for new cars
        if car_name not in self.car_laps:
            self.car_laps[car_name] = 0
            self.car_previous_s[car_name] = None

    def update_lap_tracking(self, car_name, current_s):
        """Update lap tracking for a car based on s-coordinate"""
        if car_name not in self.car_laps:
            self.car_laps[car_name] = 0
            self.car_previous_s[car_name] = current_s
            return

        previous_s = self.car_previous_s[car_name]
        if previous_s is not None:
            # Detect lap completion: large negative jump in s-coordinate
            # (from near track_length back to near 0)
            if previous_s > (self.track_length * 0.8) and current_s < (self.track_length * 0.2):
                self.car_laps[car_name] += 1
                rospy.loginfo(
                    f"[Multi-Car Publisher] {car_name} completed lap {self.car_laps[car_name]}")

        self.car_previous_s[car_name] = current_s

    def get_car_lap_info(self, car_name, current_s):
        """Get lap information for a car"""
        self.update_lap_tracking(car_name, current_s)
        return self.car_laps.get(car_name, 0)

    def create_visualization_marker(self, car_name, other_car_name, other_car_data, marker_id):
        """Create a visualization marker for RViz"""
        marker = Marker()
        marker.header.frame_id = "map"  # Use global map frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"multi_car_obstacles_{car_name}"
        marker.id = marker_id
        marker.type = Marker.CYLINDER  # Represent car as cylinder
        marker.action = Marker.ADD

        # Position from car pose
        marker.pose.position = other_car_data['pose'].position
        marker.pose.orientation = other_car_data['pose'].orientation

        # Car dimensions (with safety margin)
        marker.scale.x = self.car_length + 2 * self.safety_margin
        marker.scale.y = self.car_width + 2 * self.safety_margin
        marker.scale.z = 0.3  # Height for visualization

        # Color coding (red for obstacles, with car name based color variation)
        color_hash = hash(other_car_name) % 3
        if color_hash == 0:
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.7)  # Red
        elif color_hash == 1:
            marker.color = ColorRGBA(1.0, 0.5, 0.0, 0.7)  # Orange
        else:
            marker.color = ColorRGBA(1.0, 0.0, 0.5, 0.7)  # Pink

        # Lifetime
        # Short lifetime for real-time updates
        marker.lifetime = rospy.Duration(0.2)

        return marker

    def create_car_obstacle(self, car_name, other_car_name, observing_car_data, other_car_data, obstacle_id):
        """Create an f110_msgs/Obstacle representing another car with correct coordinates for both collision detection and predictive planning"""

        # Get other car position in global coordinates
        car_pose = other_car_data['pose']
        car_x = car_pose.position.x
        car_y = car_pose.position.y

        # Get observing car position for relative calculation
        obs_pose = observing_car_data['pose']
        obs_x = obs_pose.position.x
        obs_y = obs_pose.position.y

        # Get car velocity
        car_twist = other_car_data['twist']

        # Convert to Frenet coordinates using car-specific service
        frenet_converter = self.frenet_converters.get(car_name)
        if frenet_converter is not None:
            try:
                # Convert both car positions to Frenet coordinates
                resp_other = frenet_converter([car_x], [car_y])
                resp_obs = frenet_converter([obs_x], [obs_y])

                if (len(resp_other.s) > 0 and len(resp_other.d) > 0 and
                        len(resp_obs.s) > 0 and len(resp_obs.d) > 0):

                    # Get absolute Frenet coordinates
                    other_s = resp_other.s[0]
                    other_d = resp_other.d[0]
                    obs_s = resp_obs.s[0]

                    # Update lap tracking for both cars
                    other_car_lap = self.get_car_lap_info(
                        other_car_name, other_s)
                    observing_car_lap = self.get_car_lap_info(car_name, obs_s)

                    # Calculate relative distance for predictive planning
                    s_diff = other_s - obs_s

                    # Handle track wraparound for relative distance calculation
                    if s_diff > self.track_length / 2:
                        s_diff -= self.track_length
                    elif s_diff < -self.track_length / 2:
                        s_diff += self.track_length

                    # SOLUTION: Use ABSOLUTE coordinates for collision detection (like V1)
                    # Store RELATIVE distance for predictive planning (like V2)
                    s_center = other_s  # Absolute position for emergency braking
                    d_center = other_d  # Absolute lateral position

                    # Store relative distance in s_start for predictive planning
                    # The opponent_trajectory.py looks for relative distance in s_start calculation
                    relative_s_start = obs_s + s_diff - \
                        (self.car_length + self.safety_margin) / 2.0

                    # rospy.loginfo_throttle(2.0,
                    #                        f"[{car_name}] {other_car_name}: abs_s={s_center:.2f}m, rel_s={s_diff:.2f}m, d={d_center:.2f}m")

                else:
                    rospy.logwarn(
                        f"Frenet conversion failed for {other_car_name} observed by {car_name}")
                    return None

            except rospy.ServiceException as e:
                rospy.logwarn(
                    f"Frenet conversion service call failed for {car_name}: {e}")
                return None
        else:
            # Fallback: use global coordinates directly
            rospy.logwarn(
                f"Using global coordinates as fallback for {car_name}")
            s_center = car_x
            d_center = car_y
            relative_s_start = car_x  # Unknown relative distance

        # Create obstacle message
        obstacle = Obstacle()
        obstacle.id = obstacle_id

        # CRITICAL: Use ABSOLUTE Frenet coordinates for collision detection
        obstacle.s_center = s_center  # Absolute s-coordinate for emergency braking
        obstacle.d_center = d_center  # Absolute d-coordinate

        # Obstacle bounds (car dimensions + safety margin)
        half_length = (self.car_length + self.safety_margin) / 2.0
        half_width = (self.car_width + self.safety_margin) / 2.0

        # FIXED: Use ABSOLUTE coordinates for obstacle bounds (not relative)
        obstacle.s_start = s_center - half_length  # Absolute start position
        obstacle.s_end = s_center + half_length    # Absolute end position
        obstacle.d_left = d_center + half_width
        obstacle.d_right = d_center - half_width

        # Velocity in Frenet frame
        obstacle.vs = car_twist.linear.x  # Forward velocity
        obstacle.vd = car_twist.linear.y  # Lateral velocity

        # ENHANCEMENT: Store relative distance information for predictive planning
        # Store relative distance magnitude and direction in variance fields
        obstacle.s_var = abs(s_diff) if 's_diff' in locals(
        ) else 0.1  # Relative distance magnitude
        obstacle.d_var = 1.0 if s_diff > 0 else - \
            1.0 if 's_diff' in locals() and s_diff < 0 else 0.0  # Direction (ahead/behind)

        # Standard obstacle properties
        obstacle.size = max(self.car_length, self.car_width)
        obstacle.is_static = False  # Cars are dynamic
        obstacle.is_visible = False  # Assume cars are always visible
        obstacle.is_actually_a_gap = False  # This is a solid obstacle

        # Velocity uncertainty
        obstacle.vs_var = 0.2  # 0.2 m/s velocity uncertainty
        obstacle.vd_var = 0.2  # 0.2 m/s velocity uncertainty

        return obstacle

    def calculate_distance(self, pose1, pose2):
        """Calculate Cartesian distance between two poses (deprecated, kept for compatibility)"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)

    def calculate_frenet_distance(self, car_name, pose1, pose2):
        """
        Calculate Frenet-based distance between two poses with wraparound handling.

        Returns:
            tuple: (track_distance, lateral_distance, car1_s, car2_s) or None if conversion fails
            - track_distance: Along-track distance (s) with wraparound correction
            - lateral_distance: Lateral distance (d) 
            - car1_s: s-coordinate of first car
            - car2_s: s-coordinate of second car
        """
        frenet_converter = self.frenet_converters.get(car_name)
        if frenet_converter is None:
            return None

        try:
            # Convert both positions to Frenet coordinates
            resp1 = frenet_converter([pose1.position.x], [pose1.position.y])
            resp2 = frenet_converter([pose2.position.x], [pose2.position.y])

            if len(resp1.s) == 0 or len(resp2.s) == 0:
                return None

            s1 = resp1.s[0]
            d1 = resp1.d[0]
            s2 = resp2.s[0]
            d2 = resp2.d[0]

            # Calculate along-track distance with wraparound handling
            s_diff = abs(s2 - s1)

            # Handle track wraparound: if distance is more than half the track,
            # the shorter path is around the other way
            if s_diff > self.track_length / 2.0:
                s_diff = self.track_length - s_diff

            # Calculate lateral distance
            d_diff = abs(d2 - d1)

            return (s_diff, d_diff, s1, s2)

        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0,
                                   f"Frenet conversion failed for distance calculation: {e}")
            return None

    def publish_obstacles(self, event):
        """Main publishing function - called by timer"""
        current_time = rospy.Time.now()

        for car_name in self.car_names:
            if car_name not in self.car_positions:
                continue

            # Create obstacle array for this car
            obstacle_array = ObstacleArray()
            obstacle_array.header.stamp = current_time
            obstacle_array.header.frame_id = "map"  # Global frame

            # Create visualization markers
            markers = MarkerArray()
            marker_id = 0
            obstacle_id = 1000 + hash(car_name) % 1000  # Unique ID per car

            # Find all other cars and create obstacle messages
            for other_car_name in self.car_names:
                if other_car_name == car_name:
                    continue  # Don't include self

                if other_car_name not in self.car_positions:
                    continue  # Skip if position unknown

                other_car_data = self.car_positions[other_car_name]

                # Check if data is recent (within 0.5 seconds)
                age = (current_time - other_car_data['timestamp']).to_sec()
                if age > 0.5:
                    rospy.logwarn(
                        f"Stale data for {other_car_name}: {age:.2f}s old")
                    continue

                # REMOVED: Detection range filtering
                # Like the dummy obstacle publisher, we now publish all other cars
                # regardless of distance. This matches the behavior where the dummy
                # obstacle is always published when in GB_TRACK state.

                # Create obstacle message (f110_msgs/Obstacle)
                obstacle = self.create_car_obstacle(
                    car_name, other_car_name, self.car_positions[car_name], other_car_data, obstacle_id)
                if obstacle is not None:
                    obstacle_array.obstacles.append(obstacle)
                    obstacle_id += 1

                    # Create visualization marker
                    viz_marker = self.create_visualization_marker(
                        car_name, other_car_name, other_car_data, marker_id
                    )
                    markers.markers.append(viz_marker)
                    marker_id += 1

                    # # Log detection for debugging with both distances
                    # if car_name in self.car_positions:
                    #     frenet_dist = self.calculate_frenet_distance(
                    #         car_name,
                    #         self.car_positions[car_name]['pose'],
                    #         other_car_data['pose']
                    #     )
                    #     if frenet_dist is not None:
                    #         track_dist, lateral_dist, car_s, other_s = frenet_dist
                    #         rospy.loginfo_throttle(2.0,
                    #                                f"{car_name} published {other_car_name} as obstacle: "
                    #                                f"track_dist={track_dist:.2f}m, lateral={lateral_dist:.2f}m, "
                    #                                f"s_coords=({car_s:.1f}, {other_s:.1f})")
                    #     else:
                    #         cartesian_dist = self.calculate_distance(
                    #             self.car_positions[car_name]['pose'],
                    #             other_car_data['pose']
                    #         )
                    #         rospy.logdebug(f"{car_name} detects {other_car_name} at {cartesian_dist:.2f}m "
                    #                        f"(s={obstacle.s_center:.2f}, d={obstacle.d_center:.2f})")

            # Publish obstacle array for this car (integrates with existing perception pipeline)
            if obstacle_array.obstacles:
                self.obstacle_publishers[car_name].publish(obstacle_array)
                rospy.logdebug(
                    f"Published {len(obstacle_array.obstacles)} car obstacles for {car_name}")

            # Publish visualization markers
            if markers.markers:
                self.marker_publishers[car_name].publish(markers)

            # Publish opponent waypoints (full trajectory) for each opponent
            # This matches the dummy obstacle behavior of publishing opponent_waypoints
            self.publish_opponent_waypoints(car_name, obstacle_array)

    def publish_opponent_waypoints(self, car_name, obstacle_array):
        """
        Publish opponent trajectory waypoints using the global waypoints.
        This uses the car's global waypoints as the opponent trajectory,
        similar to how dummy obstacle publisher works.
        """
        if not obstacle_array.obstacles:
            return

        # Get global waypoints for this car
        try:
            global_waypoints_topic = f"/{car_name}/global_waypoints"
            global_wpnts_msg = rospy.wait_for_message(
                global_waypoints_topic, WpntArray, timeout=0.1)
        except:
            # If we can't get car-specific waypoints, try global topic
            try:
                global_wpnts_msg = rospy.wait_for_message(
                    "/global_waypoints", WpntArray, timeout=0.1)
            except:
                return  # No waypoints available

        # Create opponent trajectory message
        opponent_traj_msg = OpponentTrajectory()
        opponent_traj_msg.header.stamp = rospy.Time.now()
        opponent_traj_msg.header.frame_id = "map"

        # Use lap count >= 2 to indicate trajectory is available (like dummy obstacle)
        opponent_traj_msg.lap_count = 2

        # Convert global waypoints to opponent waypoints format
        # Exclude last point (because last point == first point in global waypoints)
        for wpnt in global_wpnts_msg.wpnts[:-1]:
            opp_wpnt = OppWpnt()
            opp_wpnt.x_m = wpnt.x_m
            opp_wpnt.y_m = wpnt.y_m
            opp_wpnt.s_m = wpnt.s_m
            opp_wpnt.d_m = 0.0  # Global waypoints are on centerline
            opp_wpnt.proj_vs_mps = wpnt.vx_mps
            opp_wpnt.vd_mps = 0.0
            opp_wpnt.d_var = 0.3  # 30cm lateral uncertainty
            opp_wpnt.vs_var = 0.5  # 0.5 m/s speed uncertainty
            opponent_traj_msg.oppwpnts.append(opp_wpnt)

        # Publish the opponent waypoints
        if opponent_traj_msg.oppwpnts:
            self.opponent_waypoints_publishers[car_name].publish(
                opponent_traj_msg)


def main():
    try:
        publisher = MultiCarObstaclePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Multi-Car Obstacle Publisher node terminated")


if __name__ == '__main__':
    main()
