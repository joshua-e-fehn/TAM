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
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from frenet_conversion.srv import Glob2FrenetArr
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
            '~publish_rate', 50.0)  # Hz - Updated to 50Hz

        # Car model and dimensions
        self.car_model = rospy.get_param('~car_model', 'NUC2')

        # Try to get dimensions from car model, fallback to launch file params, then defaults
        self.car_length = self.get_car_dimension('car_length', 0.58)  # meters
        self.car_width = self.get_car_dimension('car_width', 0.31)    # meters
        self.safety_margin = rospy.get_param('~safety_margin', 0.2)   # meters

        # Detection range (only consider cars within this distance)
        self.max_detection_range = rospy.get_param(
            '~max_detection_range', 15.0)  # meters

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Storage for car positions and velocities
        self.car_positions = {}
        self.car_subscribers = {}

        # Publishers for obstacles (per car) - Using f110_msgs/ObstacleArray
        self.obstacle_publishers = {}
        self.marker_publishers = {}

        # Frenet Conversion Services (per car)
        self.frenet_converters = {}
        self.setup_frenet_services()

        # Initialize subscribers and publishers for each car
        self.setup_car_topics()

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

    def create_car_obstacle(self, car_name, other_car_name, other_car_data, obstacle_id):
        """Create an f110_msgs/Obstacle representing another car"""

        # Get car position in global coordinates
        car_pose = other_car_data['pose']
        car_x = car_pose.position.x
        car_y = car_pose.position.y

        # Get car velocity
        car_twist = other_car_data['twist']

        # Convert to Frenet coordinates using car-specific service
        frenet_converter = self.frenet_converters.get(car_name)
        if frenet_converter is not None:
            try:
                # Call frenet conversion service for the observing car
                resp = frenet_converter([car_x], [car_y])
                if len(resp.s) > 0 and len(resp.d) > 0:
                    s_center = resp.s[0]
                    d_center = resp.d[0]
                else:
                    rospy.logwarn(
                        f"Frenet conversion failed for {other_car_name} observed by {car_name}")
                    return None
            except rospy.ServiceException as e:
                rospy.logwarn(
                    f"Frenet conversion service call failed for {car_name}: {e}")
                return None
        else:
            # Fallback: use global coordinates directly (not ideal)
            rospy.logwarn(
                f"Using global coordinates as fallback for {car_name} - Frenet conversion unavailable")
            s_center = car_x
            d_center = car_y

        # Create obstacle message
        obstacle = Obstacle()
        obstacle.id = obstacle_id

        # Position in Frenet coordinates
        obstacle.s_center = s_center
        obstacle.d_center = d_center

        # Obstacle bounds (car dimensions + safety margin)
        half_length = (self.car_length + self.safety_margin) / 2.0
        half_width = (self.car_width + self.safety_margin) / 2.0

        obstacle.s_start = s_center - half_length
        obstacle.s_end = s_center + half_length
        obstacle.d_left = d_center + half_width
        obstacle.d_right = d_center - half_width

        # Velocity in Frenet frame (approximation)
        # TODO: Proper velocity transformation would require orientation
        obstacle.vs = car_twist.linear.x  # Forward velocity
        obstacle.vd = car_twist.linear.y  # Lateral velocity

        # Obstacle properties
        obstacle.size = max(self.car_length, self.car_width)
        obstacle.is_static = False  # Cars are dynamic
        obstacle.is_visible = True  # Assume cars are always visible
        obstacle.is_actually_a_gap = False  # This is a solid obstacle

        # Variance/uncertainty (set to reasonable defaults)
        obstacle.s_var = 0.1  # 10cm uncertainty in s
        obstacle.d_var = 0.1  # 10cm uncertainty in d
        obstacle.vs_var = 0.2  # 0.2 m/s velocity uncertainty
        obstacle.vd_var = 0.2  # 0.2 m/s velocity uncertainty

        return obstacle

    def calculate_distance(self, pose1, pose2):
        """Calculate distance between two poses"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)

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

                # Check if within detection range
                if car_name in self.car_positions:
                    distance = self.calculate_distance(
                        self.car_positions[car_name]['pose'],
                        other_car_data['pose']
                    )
                    if distance > self.max_detection_range:
                        continue  # Too far away

                # Create obstacle message (f110_msgs/Obstacle)
                obstacle = self.create_car_obstacle(
                    car_name, other_car_name, other_car_data, obstacle_id)
                if obstacle is not None:
                    obstacle_array.obstacles.append(obstacle)
                    obstacle_id += 1

                    # Create visualization marker
                    viz_marker = self.create_visualization_marker(
                        car_name, other_car_name, other_car_data, marker_id
                    )
                    markers.markers.append(viz_marker)
                    marker_id += 1

                    # Log detection for debugging
                    if car_name in self.car_positions:
                        distance = self.calculate_distance(
                            self.car_positions[car_name]['pose'],
                            other_car_data['pose']
                        )
                        rospy.logdebug(f"{car_name} detects {other_car_name} at {distance:.2f}m "
                                       f"(s={obstacle.s_center:.2f}, d={obstacle.d_center:.2f})")

            # Publish obstacle array for this car (integrates with existing perception pipeline)
            if obstacle_array.obstacles:
                self.obstacle_publishers[car_name].publish(obstacle_array)
                rospy.logdebug(
                    f"Published {len(obstacle_array.obstacles)} car obstacles for {car_name}")

            # Publish visualization markers
            if markers.markers:
                self.marker_publishers[car_name].publish(markers)


def main():
    try:
        publisher = MultiCarObstaclePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Multi-Car Obstacle Publisher node terminated")


if __name__ == '__main__':
    main()
