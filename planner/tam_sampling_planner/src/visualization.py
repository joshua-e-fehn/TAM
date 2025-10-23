#!/usr/bin/env python3
"""
Visualization Node for TAM Sampling Planner

This node subscribes to car-specific TAM Sampling Planner visualization markers
and republishes them to global topics with car-specific colors and namespaces
for multi-car visualization in RViz.

Subscribes to:
- /{car_name}/planner/avoidance/markers: TAM planned trajectory markers
- /{car_name}/prediction/opponent_markerarray: Opponent trajectory prediction markers
- /{car_name}/local_waypoints: Controller waypoints (from state machine)

Publishes to:
- /visualization/tam_sampling_planner/{car_name}/planned_trajectory: Per-car planned trajectory markers
- /visualization/tam_sampling_planner/{car_name}/opponent_traj: Per-car opponent trajectory markers
- /visualization/tam_sampling_planner/{car_name}/controller_waypoints: Per-car controller waypoints
"""

import rospy
import math
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from f110_msgs.msg import WpntArray
from geometry_msgs.msg import Point
from copy import deepcopy


class TAMSamplingPlannerVisualization:
    def __init__(self):
        rospy.init_node('tam_sampling_planner_visualization', anonymous=True)

        # Get car name from namespace or parameter
        self.car_name = rospy.get_namespace().strip('/')
        if not self.car_name or self.car_name == '/':
            self.car_name = rospy.get_param('~car_name', 'car1')

        rospy.loginfo(
            f"TAM Sampling Planner Visualization initialized for {self.car_name}")

        # Color schemes for different cars
        # Planned trajectory colors (bright, solid)
        self.planned_colors = {
            'car1': ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Bright Red - Car1 planned
            # Bright Blue - Car2 planned
            'car2': ColorRGBA(0.0, 0.5, 1.0, 1.0),
            # Bright Green - Car3 planned
            'car3': ColorRGBA(0.0, 1.0, 0.0, 1.0),
            # Bright Yellow - Car4 planned
            'car4': ColorRGBA(1.0, 1.0, 0.0, 1.0),
        }

        # Opponent prediction colors (darker, more transparent)
        self.opponent_colors = {
            # Orange - Car1's opponent prediction
            'car1': ColorRGBA(0.8, 0.4, 0.0, 0.6),
            # Purple - Car2's opponent prediction
            'car2': ColorRGBA(0.6, 0.0, 0.8, 0.6),
            # Cyan - Car3's opponent prediction
            'car3': ColorRGBA(0.0, 0.6, 0.6, 0.6),
            # Dark Yellow - Car4's opponent prediction
            'car4': ColorRGBA(0.8, 0.6, 0.0, 0.6),
        }

        # Default colors for unknown cars
        self.default_planned_color = ColorRGBA(0.5, 0.5, 0.5, 1.0)  # Gray
        self.default_opponent_color = ColorRGBA(
            0.3, 0.3, 0.3, 0.6)  # Dark Gray

        # Initialize subscribers for car-specific topics
        self.init_subscribers()

        # Initialize publishers for global visualization topics
        self.init_publishers()

        rospy.loginfo(
            f"TAM Sampling Planner Visualization ready for {self.car_name}")

    def init_subscribers(self):
        """Initialize subscribers to car-specific marker topics"""
        # Opponent prediction markers
        opponent_topic = f'/{self.car_name}/prediction/opponent_markerarray'
        rospy.Subscriber(opponent_topic, MarkerArray, self.opponent_callback)

        # Planned trajectory markers
        planned_topic = f'/{self.car_name}/planner/avoidance/markers'
        rospy.Subscriber(planned_topic, MarkerArray,
                         self.planned_trajectory_callback)

        # Local waypoints - the actual trajectory sent to the controller (from state machine)
        local_wpnts_topic = f'/{self.car_name}/local_waypoints'
        rospy.Subscriber(local_wpnts_topic, WpntArray,
                         self.local_waypoints_callback)

    def init_publishers(self):
        """Initialize publishers to car-specific visualization topics"""
        # Per-car opponent trajectory predictions
        self.opponent_pub = rospy.Publisher(
            f'/visualization/tam_sampling_planner/{self.car_name}/opponent_traj',
            MarkerArray,
            queue_size=1
        )

        # Per-car planned trajectory (TAM sampling output)
        self.planned_pub = rospy.Publisher(
            f'/visualization/tam_sampling_planner/{self.car_name}/planned_trajectory',
            MarkerArray,
            queue_size=1
        )

        # Per-car controller waypoints (what's actually being followed)
        self.controller_pub = rospy.Publisher(
            f'/visualization/tam_sampling_planner/{self.car_name}/controller_waypoints',
            MarkerArray,
            queue_size=1
        )

    def get_car_color(self, marker_type):
        """Get the color for this car based on marker type"""
        if 'opponent' in marker_type:
            return self.opponent_colors.get(self.car_name, self.default_opponent_color)
        else:
            return self.planned_colors.get(self.car_name, self.default_planned_color)

    def modify_marker(self, marker, marker_type, marker_id_offset=0):
        """
        Modify a single marker with car-specific color and namespace

        Args:
            marker: Original marker
            marker_type: Type identifier for namespace
            marker_id_offset: Offset to add to marker ID for uniqueness

        Returns:
            Modified marker
        """
        new_marker = deepcopy(marker)

        # Set car-specific namespace
        new_marker.ns = f"{self.car_name}_{marker_type}"

        # Apply car-specific color based on marker type
        new_marker.color = self.get_car_color(marker_type)

        # Ensure unique IDs across cars by adding car-specific offset
        # Use hash for consistent offset
        car_offset = hash(self.car_name) % 1000
        new_marker.id = marker.id + car_offset + marker_id_offset

        # Ensure frame is global map
        new_marker.header.frame_id = "map"

        # Fix uninitialized quaternion warning - set identity quaternion if not set
        if marker.type in [Marker.LINE_STRIP, Marker.LINE_LIST, Marker.POINTS]:
            # For line/point markers, ensure orientation is identity
            if (new_marker.pose.orientation.x == 0.0 and
                new_marker.pose.orientation.y == 0.0 and
                new_marker.pose.orientation.z == 0.0 and
                    new_marker.pose.orientation.w == 0.0):
                new_marker.pose.orientation.w = 1.0

        # Scale adjustments for better visibility
        if 'opponent' in marker_type:
            # Make opponent predictions slightly smaller
            new_marker.scale.x *= 0.8
            new_marker.scale.y *= 0.8
            new_marker.scale.z *= 0.8

        return new_marker

    def modify_marker_array(self, markers, marker_type):
        """
        Modify all markers in a MarkerArray with car-specific colors and namespaces

        Args:
            markers: Original MarkerArray
            marker_type: Type identifier for namespace

        Returns:
            Modified MarkerArray
        """
        modified = MarkerArray()
        modified.markers = []

        for i, marker in enumerate(markers.markers):
            new_marker = self.modify_marker(marker, marker_type, i * 10)
            modified.markers.append(new_marker)

        return modified

    def opponent_callback(self, msg):
        """Handle opponent trajectory prediction markers"""
        try:
            # Publish immediately - don't cache to avoid lag
            modified = self.modify_marker_array(msg, "opponent_traj")
            if modified.markers:
                self.opponent_pub.publish(modified)
                rospy.logdebug_throttle(5.0,
                                        f"{self.car_name}: Published {len(modified.markers)} opponent trajectory markers")
        except Exception as e:
            rospy.logwarn(
                f"Error processing opponent trajectory markers for {self.car_name}: {e}")

    def planned_trajectory_callback(self, msg):
        """Handle planned trajectory markers from TAM sampling planner"""
        try:
            # Publish immediately - don't cache to avoid lag
            modified = self.modify_marker_array(msg, "planned_traj")
            if modified.markers:
                self.planned_pub.publish(modified)
                rospy.logdebug_throttle(5.0,
                                        f"{self.car_name}: Published {len(modified.markers)} planned trajectory markers")
        except Exception as e:
            rospy.logwarn(
                f"Error processing planned trajectory markers for {self.car_name}: {e}")

    def local_waypoints_callback(self, msg):
        """Handle local waypoints - the actual trajectory sent to the controller from state machine"""
        try:
            # Publish immediately - don't cache to avoid any lag
            markers = self.create_controller_waypoint_markers(msg)
            if markers.markers:
                self.controller_pub.publish(markers)
                rospy.logdebug_throttle(5.0,
                                        f"{self.car_name}: Published {len(markers.markers)} controller waypoint markers")
        except Exception as e:
            rospy.logwarn(
                f"Error processing controller waypoints for {self.car_name}: {e}")

    def create_controller_waypoint_markers(self, waypoints_msg):
        """Create visualization markers for controller waypoints (downsampled, distinct style)"""
        marker_array = MarkerArray()

        if not waypoints_msg.wpnts:
            return marker_array

        car_color = self.get_car_color('controller')
        # Different offset from other markers
        car_offset = hash(self.car_name) % 1000 + 3000

        # Create markers for each waypoint (downsampled - every 3rd waypoint)
        marker_id = 0
        for i, wpnt in enumerate(waypoints_msg.wpnts):
            # Downsample - only show every 3rd waypoint
            if i % 3 != 0:
                continue

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = f"{self.car_name}_controller_wpnts"
            marker.id = car_offset + marker_id
            marker.type = Marker.SPHERE  # Use spheres for distinction from planner trajectories
            marker.action = Marker.ADD
            marker_id += 1

            # Position
            marker.pose.position.x = wpnt.x_m
            marker.pose.position.y = wpnt.y_m
            marker.pose.position.z = 0.0

            # Orientation (identity quaternion)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Scale - spheres
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25

            # Color - use car-specific color, fully opaque for controller waypoints
            marker.color = car_color

            # Lifetime - short lifetime, will be republished
            marker.lifetime = rospy.Duration(0.15)

            marker_array.markers.append(marker)

        return marker_array


def main():
    """Main function"""
    try:
        visualization = TAMSamplingPlannerVisualization()
        rospy.loginfo("TAM Sampling Planner visualization node running")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("TAM Sampling Planner visualization node interrupted")
    except Exception as e:
        rospy.logerr(f"TAM Sampling Planner visualization node failed: {e}")
        raise


if __name__ == '__main__':
    main()
