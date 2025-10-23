#!/usr/bin/env python3
"""
Visualization Node for Spliner Planner

This node subscribes to car-specific Spliner visualization markers
and republishes them to global topics with car-specific colors and namespaces
for multi-car visualization in RViz.

Subscribes to:
- /{car_name}/planner/avoidance/markers: Spliner planned trajectory markers

Publishes to:
- /visualization/spliner/planned_trajectory: Global planned trajectory markers (per car)
"""

import rospy
import math
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from f110_msgs.msg import OTWpntArray, WpntArray
from copy import deepcopy


class SplinerVisualization:
    def __init__(self):
        rospy.init_node('spliner_visualization', anonymous=True)

        # Get car name from namespace or parameter
        self.car_name = rospy.get_namespace().strip('/')
        if not self.car_name or self.car_name == '/':
            self.car_name = rospy.get_param('~car_name', 'car1')

        rospy.loginfo(f"Spliner Visualization initialized for {self.car_name}")

        # Color schemes for different cars - planned trajectories (bright, solid)
        self.planned_colors = {
            'car1': ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Bright Red - Car1 planned
            # Bright Blue - Car2 planned
            'car2': ColorRGBA(0.0, 0.5, 1.0, 1.0),
            # Bright Green - Car3 planned
            'car3': ColorRGBA(0.0, 1.0, 0.0, 1.0),
            # Bright Yellow - Car4 planned
            'car4': ColorRGBA(1.0, 1.0, 0.0, 1.0),
        }

        # Default color for unknown cars
        self.default_planned_color = ColorRGBA(0.5, 0.5, 0.5, 1.0)  # Gray

        # Initialize subscribers for car-specific topics
        self.init_subscribers()

        # Initialize publishers for global visualization topics
        self.init_publishers()

        rospy.loginfo(f"Spliner Visualization ready for {self.car_name}")

    def init_subscribers(self):
        """Initialize subscribers to car-specific marker topics"""
        # Local waypoints - the actual trajectory sent to the controller (from state machine)
        from f110_msgs.msg import WpntArray
        local_wpnts_topic = f'/{self.car_name}/local_waypoints'
        rospy.Subscriber(local_wpnts_topic, WpntArray,
                         self.local_waypoints_callback)

        # Avoidance waypoints - spliner's output (before state machine processing)
        avoidance_wpnts_topic = f'/{self.car_name}/planner/avoidance/otwpnts'
        rospy.Subscriber(avoidance_wpnts_topic, OTWpntArray,
                         self.avoidance_waypoints_callback)

    def init_publishers(self):
        """Initialize publishers to global visualization topics"""
        # Local waypoints - what's actually being sent to the controller (from state machine)
        self.local_wpnts_pub = rospy.Publisher(
            '/visualization/spliner/controller_waypoints',
            MarkerArray,
            queue_size=1
        )

        # Avoidance waypoints - spliner's output (arrows showing avoidance path)
        self.avoidance_wpnts_pub = rospy.Publisher(
            '/visualization/spliner/avoidance_waypoints',
            MarkerArray,
            queue_size=1
        )

    def get_car_color(self):
        """Get the color for this car"""
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

        # Apply car-specific color
        new_marker.color = self.get_car_color()

        # Ensure unique IDs across cars by adding car-specific offset
        # Use hash for consistent offset
        car_offset = hash(self.car_name) % 1000
        new_marker.id = marker.id + car_offset + marker_id_offset

        # Ensure frame is global map
        new_marker.header.frame_id = "map"

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

    def local_waypoints_callback(self, msg):
        """Handle local waypoints - the actual trajectory sent to the controller from state machine"""
        try:
            # Publish immediately - don't cache to avoid any lag
            markers = self.create_wpntarray_markers(msg, "local_wpnts")
            if markers.markers:
                self.local_wpnts_pub.publish(markers)
                rospy.logdebug_throttle(5.0,
                                        f"{self.car_name}: Published {len(markers.markers)} local waypoint markers")
        except Exception as e:
            rospy.logwarn(
                f"Error processing local waypoints for {self.car_name}: {e}")

    def avoidance_waypoints_callback(self, msg):
        """Handle avoidance waypoints - spliner's output before state machine processing"""
        try:
            # Only show if there are actually waypoints (not empty)
            if msg.wpnts and len(msg.wpnts) > 0:
                # Publish immediately - don't cache to avoid lag
                markers = self.create_otwpntarray_markers(
                    msg, "avoidance_wpnts")
                if markers.markers:
                    self.avoidance_wpnts_pub.publish(markers)
                    rospy.logdebug_throttle(5.0,
                                            f"{self.car_name}: Published {len(markers.markers)} avoidance waypoint markers")
            else:
                # Clear avoidance markers when no waypoints
                clear_marker = MarkerArray()
                delete_marker = Marker()
                delete_marker.action = Marker.DELETEALL
                delete_marker.ns = f"{self.car_name}_avoidance_wpnts"
                clear_marker.markers.append(delete_marker)
                self.avoidance_wpnts_pub.publish(clear_marker)
        except Exception as e:
            rospy.logwarn(
                f"Error processing avoidance waypoints for {self.car_name}: {e}")

    def create_wpntarray_markers(self, waypoints_msg, namespace_suffix):
        """Create visualization markers from WpntArray waypoints (local waypoints from state machine)"""
        marker_array = MarkerArray()

        if not waypoints_msg.wpnts:
            return marker_array

        car_color = self.get_car_color()
        car_offset = hash(self.car_name) % 1000

        # Create markers for each waypoint
        for i, wpnt in enumerate(waypoints_msg.wpnts):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = f"{self.car_name}_{namespace_suffix}"
            marker.id = car_offset + i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = wpnt.x_m
            marker.pose.position.y = wpnt.y_m
            marker.pose.position.z = 0.0

            # Orientation from heading (convert to quaternion)
            yaw = wpnt.psi_rad
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)

            # Scale - arrows pointing forward
            marker.scale.x = 0.35  # Length
            marker.scale.y = 0.1  # Width
            marker.scale.z = 0.1  # Height

            # Color - use car-specific color, fully opaque for controller waypoints
            marker.color = car_color

            # Lifetime
            # Short lifetime, will be republished
            marker.lifetime = rospy.Duration(0.15)

            marker_array.markers.append(marker)

        return marker_array

    def create_otwpntarray_markers(self, waypoints_msg, namespace_suffix):
        """Create visualization markers from OTWpntArray waypoints (avoidance waypoints from spliner)"""
        marker_array = MarkerArray()

        if not waypoints_msg.wpnts:
            return marker_array

        car_color = self.get_car_color()
        # Make avoidance waypoints distinct - use cube markers with transparency
        # and slightly different color tint (add some white/yellow tint)
        car_color_avoidance = ColorRGBA(
            min(1.0, car_color.r * 0.8 + 0.4),  # Lighten with white/yellow
            min(1.0, car_color.g * 0.8 + 0.3),
            min(1.0, car_color.b * 0.8),
            0.4  # More transparent
        )
        # Different offset to avoid ID collision
        car_offset = hash(self.car_name) % 1000 + 5000

        # Create markers for each waypoint
        for i, wpnt in enumerate(waypoints_msg.wpnts):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = f"{self.car_name}_{namespace_suffix}"
            marker.id = car_offset + i
            marker.type = Marker.CUBE  # Use cubes instead of arrows for clear distinction
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = wpnt.x_m
            marker.pose.position.y = wpnt.y_m
            marker.pose.position.z = 0.08  # Elevated above controller waypoints

            # Orientation from heading (convert to quaternion)
            yaw = wpnt.psi_rad
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)

            # Scale - small cubes
            marker.scale.x = 0.15  # Length
            marker.scale.y = 0.15  # Width
            marker.scale.z = 0.15  # Height

            # Color - transparent and lighter tint to distinguish from controller waypoints
            marker.color = car_color_avoidance

            # Lifetime - very short since we publish on every update
            marker.lifetime = rospy.Duration(0.1)

            marker_array.markers.append(marker)

        return marker_array


def main():
    """Main function"""
    try:
        visualization = SplinerVisualization()
        rospy.loginfo("Spliner visualization node running")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Spliner visualization node interrupted")
    except Exception as e:
        rospy.logerr(f"Spliner visualization node failed: {e}")
        raise


if __name__ == '__main__':
    main()
