#!/usr/bin/env python3

"""
Speed and Acceleration Monitor for F1/10 Racing
Displays real-time speed and acceleration information in RViz
Works in both single-car and multi-car mode
Also monitors dummy obstacle in single-car mode when active
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point


class SpeedMonitor:
    def __init__(self):
        rospy.init_node('speed_monitor', anonymous=True)

        # Get parameters
        self.car_name = rospy.get_param('~car_name', '')
        self.update_rate = rospy.get_param('~update_rate', 10.0)  # Hz
        self.monitor_obstacle = rospy.get_param(
            '~monitor_obstacle', False)  # Enable obstacle monitoring

        # Display parameters
        self.display_name = self.car_name if self.car_name else "Car"
        self.namespace = self.car_name if self.car_name else "car"

        # State variables - Car
        self.current_speed = 0.0
        self.last_speed = 0.0
        self.last_time = None
        self.current_acceleration = 0.0
        self.max_speed = 0.0
        self.max_accel = 0.0
        self.min_accel = 0.0
        self.current_position = None

        # State variables - Dummy Obstacle
        self.obstacle_speed = 0.0
        self.obstacle_last_speed = 0.0
        self.obstacle_last_time = None
        self.obstacle_acceleration = 0.0
        self.obstacle_max_speed = 0.0
        self.obstacle_max_accel = 0.0
        self.obstacle_min_accel = 0.0
        self.obstacle_position = None
        self.obstacle_active = False

        # Moving average filter for acceleration
        self.accel_window_size = 5
        self.accel_buffer = []
        self.obstacle_accel_buffer = []

        # Subscribe to odometry
        odom_topic = 'car_state/odom' if not self.car_name else f'/{self.car_name}/car_state/odom'
        rospy.Subscriber(odom_topic, Odometry,
                         self.odom_callback, queue_size=1)

        # Subscribe to obstacle odometry if monitoring enabled
        if self.monitor_obstacle or not self.car_name:  # Enable for single-car mode by default
            obstacle_odom_topic = '/obstacle/odom'
            rospy.Subscriber(obstacle_odom_topic, Odometry,
                             self.obstacle_odom_callback, queue_size=1)
            rospy.loginfo(
                f"Obstacle monitoring enabled, subscribing to: {obstacle_odom_topic}")

        # Publisher for visualization markers
        marker_topic = 'speed_monitor/markers' if not self.car_name else f'/{self.car_name}/speed_monitor/markers'
        self.marker_pub = rospy.Publisher(
            marker_topic, MarkerArray, queue_size=1)

        rospy.loginfo(f"Speed Monitor initialized for: {self.display_name}")
        rospy.loginfo(f"Subscribing to: {odom_topic}")
        rospy.loginfo(f"Publishing markers to: {marker_topic}")

    def odom_callback(self, msg):
        """Process odometry message and calculate speed/acceleration"""
        current_time = msg.header.stamp.to_sec()

        # Store current position for marker placement
        self.current_position = msg.pose.pose.position

        # Get velocity from odometry (already in m/s)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = np.sqrt(vx**2 + vy**2)

        # Update max speed
        if self.current_speed > self.max_speed:
            self.max_speed = self.current_speed

        # Calculate acceleration
        if self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                raw_accel = (self.current_speed - self.last_speed) / dt

                # Apply moving average filter
                self.accel_buffer.append(raw_accel)
                if len(self.accel_buffer) > self.accel_window_size:
                    self.accel_buffer.pop(0)

                self.current_acceleration = np.mean(self.accel_buffer)

                # Update max/min acceleration
                if self.current_acceleration > self.max_accel:
                    self.max_accel = self.current_acceleration
                if self.current_acceleration < self.min_accel:
                    self.min_accel = self.current_acceleration

        self.last_speed = self.current_speed
        self.last_time = current_time

    def obstacle_odom_callback(self, msg):
        """Process obstacle odometry message and calculate speed/acceleration"""
        current_time = msg.header.stamp.to_sec()

        # Mark obstacle as active
        self.obstacle_active = True

        # Store current position for marker placement
        self.obstacle_position = msg.pose.pose.position

        # Get velocity from odometry (already in m/s)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.obstacle_speed = np.sqrt(vx**2 + vy**2)

        # Update max speed
        if self.obstacle_speed > self.obstacle_max_speed:
            self.obstacle_max_speed = self.obstacle_speed

        # Calculate acceleration
        if self.obstacle_last_time is not None:
            dt = current_time - self.obstacle_last_time
            if dt > 0:
                raw_accel = (self.obstacle_speed -
                             self.obstacle_last_speed) / dt

                # Apply moving average filter
                self.obstacle_accel_buffer.append(raw_accel)
                if len(self.obstacle_accel_buffer) > self.accel_window_size:
                    self.obstacle_accel_buffer.pop(0)

                self.obstacle_acceleration = np.mean(
                    self.obstacle_accel_buffer)

                # Update max/min acceleration
                if self.obstacle_acceleration > self.obstacle_max_accel:
                    self.obstacle_max_accel = self.obstacle_acceleration
                if self.obstacle_acceleration < self.obstacle_min_accel:
                    self.obstacle_min_accel = self.obstacle_acceleration

        self.obstacle_last_speed = self.obstacle_speed
        self.obstacle_last_time = current_time

    def get_color_for_speed(self, speed):
        """Return color based on speed magnitude"""
        if speed < 2.0:
            return ColorRGBA(1.0, 1.0, 0.0, 1.0)  # Yellow
        elif speed < 7.0:
            return ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
        else:
            return ColorRGBA(0.0, 1.0, 1.0, 1.0)  # Cyan

    def get_color_for_accel(self, accel):
        """Return color based on acceleration (green=accel, red=brake)"""
        if accel > 0.5:
            return ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
        elif accel < -0.5:
            return ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
        else:
            return ColorRGBA(1.0, 1.0, 0.0, 1.0)  # Yellow

    def create_text_marker(self, marker_id, text, position, color, scale=0.3):
        """Create a text marker for RViz"""
        marker = Marker()
        marker.header.frame_id = f"{self.car_name}_base_link" if self.car_name else "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"{self.namespace}_speed_monitor"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0

        marker.scale.z = scale
        marker.color = color
        marker.text = text
        marker.lifetime = rospy.Duration(0.5)

        return marker

    def publish_markers(self):
        """Publish visualization markers to RViz"""
        if self.current_position is None:
            return

        markers = MarkerArray()

        # Base position above the car
        base_z = 1.2

        # Create combined text block for car
        accel_direction = "↑" if self.current_acceleration > 0.1 else "↓" if self.current_acceleration < -0.1 else "→"

        # Combine all info into single text block
        combined_text = (
            f"【{self.display_name}】\n"
            f"Speed: {self.current_speed:.2f} m/s\n"
            f"Accel: {accel_direction} {self.current_acceleration:+.2f} m/s²\n"
            f"Max V: {self.max_speed:.2f} m/s\n"
            f"Max A: {self.max_accel:+.2f} m/s²"
        )

        # Use white color for the car text
        text_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

        markers.markers.append(
            self.create_text_marker(
                0, combined_text, [0, 0, base_z], text_color, scale=0.5)
        )

        # Add obstacle marker if active
        if self.obstacle_active and self.obstacle_position is not None:
            obstacle_accel_direction = "↑" if self.obstacle_acceleration > 0.1 else "↓" if self.obstacle_acceleration < -0.1 else "→"

            obstacle_text = (
                f"【Obstacle】\n"
                f"Speed: {self.obstacle_speed:.2f} m/s\n"
                f"Accel: {obstacle_accel_direction} {self.obstacle_acceleration:+.2f} m/s²\n"
                f"Max V: {self.obstacle_max_speed:.2f} m/s\n"
                f"Max A: {self.obstacle_max_accel:+.2f} m/s²"
            )

            obstacle_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

            # Create marker in map frame at obstacle position
            obstacle_marker = Marker()
            obstacle_marker.header.frame_id = "map"
            obstacle_marker.header.stamp = rospy.Time.now()
            obstacle_marker.ns = f"{self.namespace}_obstacle_monitor"
            obstacle_marker.id = 1
            obstacle_marker.type = Marker.TEXT_VIEW_FACING
            obstacle_marker.action = Marker.ADD

            obstacle_marker.pose.position.x = self.obstacle_position.x
            obstacle_marker.pose.position.y = self.obstacle_position.y
            obstacle_marker.pose.position.z = self.obstacle_position.z + 1.2
            obstacle_marker.pose.orientation.w = 1.0

            obstacle_marker.scale.z = 0.5
            obstacle_marker.color = obstacle_color
            obstacle_marker.text = obstacle_text
            obstacle_marker.lifetime = rospy.Duration(0.5)

            markers.markers.append(obstacle_marker)

        self.marker_pub.publish(markers)

    def display_loop(self):
        """Main display loop - publishes markers instead of terminal output"""
        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            self.publish_markers()
            rate.sleep()


if __name__ == '__main__':
    try:
        monitor = SpeedMonitor()
        monitor.display_loop()
    except rospy.ROSInterruptException:
        pass
