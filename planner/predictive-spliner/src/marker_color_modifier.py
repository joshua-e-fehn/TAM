#!/usr/bin/env python3
"""
Marker Color Modifier Node for Predictive Spliner Visualization

This node subscribes to car-specific predictive spliner visualization markers
and republishes them to global topics with car-specific colors and namespaces
for multi-car visualization in RViz.

Subscribes to:
- /{car_name}/planner/avoidance/markers_sqp: SQP avoidance trajectory markers
- /{car_name}/opponent_traj_markerarray: Opponent trajectory prediction markers  
- /{car_name}/collision_predict/beginn: Collision prediction start markers
- /{car_name}/collision_predict/end: Collision prediction end markers
- /global_waypoints/overtaking/markers: Global overtaking reference line markers

Publishes to:
- /visualization/predictive_spliner/sqp_markers: Global SQP trajectory markers
- /visualization/predictive_spliner/opponent_traj: Global opponent trajectory markers
- /visualization/predictive_spliner/collision_predict: Global collision prediction markers
- /visualization/predictive_spliner/overtaking_reference: Global overtaking reference line
"""

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from copy import deepcopy


class MarkerColorModifier:
    def __init__(self):
        rospy.init_node('marker_color_modifier', anonymous=True)

        # Get car name from namespace or parameter
        self.car_name = rospy.get_namespace().strip('/')
        if not self.car_name or self.car_name == '/':
            self.car_name = rospy.get_param('~car_name', 'car1')

        rospy.loginfo(f"Marker Color Modifier initialized for {self.car_name}")

        # Color schemes for different cars
        self.colors = {
            'car1': ColorRGBA(1.0, 0.2, 0.2, 0.8),  # Red with transparency
            'car2': ColorRGBA(0.2, 0.2, 1.0, 0.8),  # Blue with transparency
            'car3': ColorRGBA(0.2, 1.0, 0.2, 0.8),  # Green with transparency
            'car4': ColorRGBA(1.0, 1.0, 0.2, 0.8),  # Yellow with transparency
        }

        # Default color for unknown cars
        self.default_color = ColorRGBA(0.5, 0.5, 0.5, 0.8)  # Gray

        # Initialize subscribers for car-specific topics
        self.init_subscribers()

        # Initialize publishers for global visualization topics
        self.init_publishers()

        rospy.loginfo(f"Marker Color Modifier ready for {self.car_name}")

    def init_subscribers(self):
        """Initialize subscribers to car-specific marker topics"""
        # SQP avoidance trajectory markers
        rospy.Subscriber(
            f'/{self.car_name}/planner/avoidance/markers_sqp',
            MarkerArray,
            self.sqp_markers_callback
        )

        # Opponent trajectory prediction markers
        rospy.Subscriber(
            f'/{self.car_name}/opponent_traj_markerarray',
            MarkerArray,
            self.opponent_traj_callback
        )

        # Collision prediction markers (start)
        rospy.Subscriber(
            f'/{self.car_name}/collision_predict/beginn',
            Marker,
            self.collision_begin_callback
        )

        # Collision prediction markers (end)
        rospy.Subscriber(
            f'/{self.car_name}/collision_predict/end',
            Marker,
            self.collision_end_callback
        )

        # Global overtaking reference line (only subscribe from car1 to avoid duplication)
        if self.car_name == 'car1':
            rospy.Subscriber(
                '/global_waypoints/overtaking/markers',
                MarkerArray,
                self.overtaking_reference_callback
            )

    def init_publishers(self):
        """Initialize publishers for global visualization topics"""
        # Global SQP trajectory markers
        self.sqp_pub = rospy.Publisher(
            '/visualization/predictive_spliner/sqp_markers',
            MarkerArray,
            queue_size=10
        )

        # Global opponent trajectory markers
        self.opponent_pub = rospy.Publisher(
            '/visualization/predictive_spliner/opponent_traj',
            MarkerArray,
            queue_size=10
        )

        # Global collision prediction markers
        self.collision_pub = rospy.Publisher(
            '/visualization/predictive_spliner/collision_predict',
            MarkerArray,
            queue_size=10
        )

        # Global overtaking reference line
        self.overtaking_ref_pub = rospy.Publisher(
            '/visualization/predictive_spliner/overtaking_reference',
            MarkerArray,
            queue_size=10
        )

    def get_car_color(self):
        """Get the color for this car"""
        return self.colors.get(self.car_name, self.default_color)

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

    def sqp_markers_callback(self, msg):
        """Handle SQP avoidance trajectory markers"""
        try:
            modified = self.modify_marker_array(msg, "sqp_trajectory")
            self.sqp_pub.publish(modified)
        except Exception as e:
            rospy.logwarn(
                f"Error processing SQP markers for {self.car_name}: {e}")

    def opponent_traj_callback(self, msg):
        """Handle opponent trajectory prediction markers"""
        try:
            modified = self.modify_marker_array(msg, "opponent_traj")
            self.opponent_pub.publish(modified)
        except Exception as e:
            rospy.logwarn(
                f"Error processing opponent trajectory markers for {self.car_name}: {e}")

    def collision_begin_callback(self, msg):
        """Handle collision prediction start marker"""
        try:
            # Convert single marker to MarkerArray for consistency
            marker_array = MarkerArray()
            modified_marker = self.modify_marker(msg, "collision_begin", 1000)
            marker_array.markers = [modified_marker]
            self.collision_pub.publish(marker_array)
        except Exception as e:
            rospy.logwarn(
                f"Error processing collision begin marker for {self.car_name}: {e}")

    def collision_end_callback(self, msg):
        """Handle collision prediction end marker"""
        try:
            # Convert single marker to MarkerArray for consistency
            marker_array = MarkerArray()
            modified_marker = self.modify_marker(msg, "collision_end", 2000)
            marker_array.markers = [modified_marker]
            self.collision_pub.publish(marker_array)
        except Exception as e:
            rospy.logwarn(
                f"Error processing collision end marker for {self.car_name}: {e}")

    def overtaking_reference_callback(self, msg):
        """Handle global overtaking reference line markers"""
        try:
            # For the reference line, we don't need car-specific colors
            # Just republish with a consistent namespace
            modified = MarkerArray()
            modified.markers = []

            # Downsample: take every 3rd marker
            for i in range(0, len(msg.markers), 3):
                marker = msg.markers[i]
                new_marker = deepcopy(marker)
                new_marker.ns = "overtaking_reference_line"
                # Set to green for reference line
                new_marker.color.r = 0.0
                new_marker.color.g = 1.0
                new_marker.color.b = 0.0  # Green for reference
                new_marker.color.a = 1  # Semi-transparent
                new_marker.header.frame_id = "map"
                # Update marker ID to maintain uniqueness after downsampling
                new_marker.id = i // 3
                modified.markers.append(new_marker)

            self.overtaking_ref_pub.publish(modified)
        except Exception as e:
            rospy.logwarn(
                f"Error processing overtaking reference markers: {e}")


def main():
    try:
        modifier = MarkerColorModifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Marker Color Modifier node interrupted")
    except Exception as e:
        rospy.logerr(f"Marker Color Modifier node failed: {e}")


if __name__ == '__main__':
    main()
