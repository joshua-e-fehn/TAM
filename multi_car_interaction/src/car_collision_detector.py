#!/usr/bin/env python3

"""
Car Collision Detector Node

This node monitors car positions and detects potential collisions:
1. Calculates distances between cars
2. Publishes collision warnings
3. Can trigger emergency stops if needed

Author: Atlas  
Date: September 2025
"""

import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA


class CarCollisionDetector:
    def __init__(self):
        rospy.init_node('car_collision_detector', anonymous=True)

        # Car configuration
        car_names_param = rospy.get_param('~car_names', 'car1,car2')
        if isinstance(car_names_param, str):
            self.car_names = [name.strip()
                              for name in car_names_param.split(',')]
        else:
            self.car_names = car_names_param
        # Hz        # Collision detection parameters
        self.check_rate = rospy.get_param('~check_rate', 50.0)
        self.warning_distance = rospy.get_param(
            '~warning_distance', 1.5)  # meters
        self.critical_distance = rospy.get_param(
            '~critical_distance', 0.8)  # meters
        self.collision_distance = rospy.get_param(
            '~collision_distance', 0.5)  # meters

        # Car model and dimensions
        self.car_model = rospy.get_param('~car_model', 'NUC2')

        # Try to get dimensions from car model, fallback to launch file params, then defaults
        self.car_length = self.get_car_dimension('car_length', 0.48)  # meters
        self.car_width = self.get_car_dimension('car_width', 0.31)    # meters

        # Storage for car positions
        self.car_positions = {}
        self.car_subscribers = {}

        # Publishers for collision status
        self.collision_publishers = {}
        self.warning_publishers = {}
        self.status_publisher = rospy.Publisher(
            '/multi_car/collision_status', String, queue_size=10)
        self.viz_publisher = rospy.Publisher(
            '/multi_car/collision_visualization', MarkerArray, queue_size=10)

        # Initialize subscribers and publishers
        self.setup_car_topics()

        # Main detection timer
        self.timer = rospy.Timer(rospy.Duration(
            1.0/self.check_rate), self.check_collisions)

        rospy.loginfo(
            f"Car Collision Detector initialized for cars: {self.car_names}")

    def get_car_dimension(self, dimension_name, default_value):
        """Get car dimension from model parameters with fallback hierarchy"""
        # 1. Try to get from launch file parameters (highest priority)
        launch_param = rospy.get_param(f'~{dimension_name}', None)
        if launch_param is not None:
            rospy.loginfo(
                f"[Multi-Car Collision] Using {dimension_name}={launch_param} from launch file")
            return launch_param

        # 2. Try to calculate from car model parameters
        try:
            if dimension_name == 'car_length':
                # Calculate car length from wheelbase (wheelbase + front/rear overhangs ≈ 1.9x wheelbase)
                wheelbase = rospy.get_param(
                    '/car_model_params/wheelbase', None)
                if wheelbase is not None:
                    # Based on F1TENTH proportions (0.307 -> 0.48)
                    calculated_length = wheelbase * 1.56
                    rospy.loginfo(
                        f"[Multi-Car Collision] Calculated car_length={calculated_length:.3f} from wheelbase={wheelbase} ({self.car_model})")
                    return calculated_length
            elif dimension_name == 'car_width':
                # Use standard F1TENTH width (fairly constant across models)
                standard_width = 0.31  # F1TENTH standard width
                rospy.loginfo(
                    f"[Multi-Car Collision] Using standard F1TENTH car_width={standard_width} ({self.car_model})")
                return standard_width
        except Exception as e:
            rospy.logwarn(
                f"[Multi-Car Collision] Could not calculate {dimension_name} from car model {self.car_model}: {e}")

        # 3. Fallback to default value
        rospy.loginfo(
            f"[Multi-Car Collision] Using default {dimension_name}={default_value}")
        return default_value

    def setup_car_topics(self):
        """Setup subscribers and publishers for each car"""
        for car_name in self.car_names:
            # Subscribe to car odometry
            odom_topic = f"/{car_name}/car_state/odom"
            self.car_subscribers[car_name] = rospy.Subscriber(
                odom_topic, Odometry,
                lambda msg, name=car_name: self.car_odom_callback(msg, name)
            )

            # Publishers for collision warnings per car
            self.collision_publishers[car_name] = rospy.Publisher(
                f"/{car_name}/collision_detected", Bool, queue_size=10
            )
            self.warning_publishers[car_name] = rospy.Publisher(
                f"/{car_name}/collision_warning", Bool, queue_size=10
            )

    def car_odom_callback(self, msg, car_name):
        """Store car position from odometry"""
        self.car_positions[car_name] = {
            'pose': msg.pose.pose,
            'timestamp': msg.header.stamp,
            'frame_id': msg.header.frame_id,
            'velocity': msg.twist.twist
        }

    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)

    def calculate_relative_velocity(self, vel1, vel2):
        """Calculate relative velocity magnitude"""
        dvx = vel1.linear.x - vel2.linear.x
        dvy = vel1.linear.y - vel2.linear.y
        return math.sqrt(dvx*dvx + dvy*dvy)

    def create_warning_marker(self, car1_name, car2_name, distance, status):
        """Create visualization marker for collision warning"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "collision_warnings"
        marker.id = hash(f"{car1_name}_{car2_name}") % 1000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position between the two cars
        car1_pos = self.car_positions[car1_name]['pose'].position
        car2_pos = self.car_positions[car2_name]['pose'].position

        marker.pose.position.x = (car1_pos.x + car2_pos.x) / 2
        marker.pose.position.y = (car1_pos.y + car2_pos.y) / 2
        marker.pose.position.z = 1.0  # Above cars
        marker.pose.orientation.w = 1.0

        # Text and color based on status
        if status == "COLLISION":
            marker.text = f"COLLISION!\n{distance:.2f}m"
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
        elif status == "CRITICAL":
            marker.text = f"CRITICAL\n{distance:.2f}m"
            marker.color = ColorRGBA(1.0, 0.5, 0.0, 1.0)  # Orange
        elif status == "WARNING":
            marker.text = f"WARNING\n{distance:.2f}m"
            marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)  # Yellow

        marker.scale.z = 0.3  # Text size
        marker.lifetime = rospy.Duration(0.2)  # Short lifetime for updates

        return marker

    def check_collisions(self, event):
        """Main collision checking function"""
        current_time = rospy.Time.now()
        collision_status = "SAFE"
        warnings = []
        markers = MarkerArray()

        # Check all car pairs
        for i, car1_name in enumerate(self.car_names):
            if car1_name not in self.car_positions:
                continue

            car1_data = self.car_positions[car1_name]

            # Check if data is recent
            age1 = (current_time - car1_data['timestamp']).to_sec()
            if age1 > 0.2:  # 200ms threshold
                continue

            for j, car2_name in enumerate(self.car_names):
                if j <= i:  # Avoid duplicate checks and self-check
                    continue

                if car2_name not in self.car_positions:
                    continue

                car2_data = self.car_positions[car2_name]

                # Check if data is recent
                age2 = (current_time - car2_data['timestamp']).to_sec()
                if age2 > 0.2:
                    continue

                # Calculate distance
                distance = self.calculate_distance(
                    car1_data['pose'], car2_data['pose'])

                # Calculate relative velocity
                rel_velocity = self.calculate_relative_velocity(
                    car1_data['velocity'], car2_data['velocity']
                )

                # Determine collision status
                warning_triggered = False
                collision_triggered = False
                status_text = "SAFE"

                if distance <= self.collision_distance:
                    collision_status = "COLLISION"
                    status_text = "COLLISION"
                    collision_triggered = True
                    warning_triggered = True
                    rospy.logwarn_throttle(2.0,
                                           f"COLLISION between {car1_name} and {car2_name}: {distance:.2f}m")

                elif distance <= self.critical_distance:
                    if collision_status != "COLLISION":
                        collision_status = "CRITICAL"
                    status_text = "CRITICAL"
                    warning_triggered = True
                    # rospy.logwarn_throttle(2.0,
                    #                        f"CRITICAL distance between {car1_name} and {car2_name}: {distance:.2f}m")

                elif distance <= self.warning_distance:
                    if collision_status not in ["COLLISION", "CRITICAL"]:
                        collision_status = "WARNING"
                    status_text = "WARNING"
                    warning_triggered = True

                # Publish individual car warnings
                self.collision_publishers[car1_name].publish(
                    Bool(collision_triggered))
                self.collision_publishers[car2_name].publish(
                    Bool(collision_triggered))
                self.warning_publishers[car1_name].publish(
                    Bool(warning_triggered))
                self.warning_publishers[car2_name].publish(
                    Bool(warning_triggered))

                # Create visualization marker if warning/collision
                if warning_triggered:
                    marker = self.create_warning_marker(
                        car1_name, car2_name, distance, status_text)
                    markers.markers.append(marker)

                # Store warning info
                warnings.append({
                    'cars': [car1_name, car2_name],
                    'distance': distance,
                    'relative_velocity': rel_velocity,
                    'status': status_text
                })

        # Publish overall status
        status_msg = f"Status: {collision_status}"
        if warnings:
            status_msg += f" | Warnings: {len(warnings)}"
            for warning in warnings:
                status_msg += f" | {warning['cars'][0]}-{warning['cars'][1]}: {warning['distance']:.2f}m"

        self.status_publisher.publish(String(status_msg))

        # Publish visualization
        if markers.markers:
            self.viz_publisher.publish(markers)

        # Log periodic status (every 5 seconds)
        if hasattr(self, '_last_status_log'):
            if (current_time - self._last_status_log).to_sec() > 5.0:
                # rospy.loginfo(
                #     f"[Multi-Car] Collision Detection Status: {collision_status}, Active warnings: {len(warnings)} (monitoring {len(self.car_names)} cars)")
                self._last_status_log = current_time
        else:
            self._last_status_log = current_time


def main():
    try:
        detector = CarCollisionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Car Collision Detector node terminated")


if __name__ == '__main__':
    main()
