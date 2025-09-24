#!/usr/bin/env python3
"""
Visualization Node for TAM Sampling Planner

This node subscribes to car-specific TAM Sampling Planner visualization markers
and republishes them to global topics with car-specific colors and namespaces
for multi-car visualization in RViz.

Subscribes to:
# - /{car_name}/planner/avoidance/markers_sqp: SQP avoidance trajectory markers
- /{car_name}/prediction/opponent_markerarray: Opponent trajectory prediction markers  
# - /{car_name}/collision_predict/beginn: Collision prediction start markers
# - /{car_name}/collision_predict/end: Collision prediction end markers
# - /global_waypoints/overtaking/markers: Global overtaking reference line markers

Publishes to:
# - /visualization/tam_sampling_planner/sqp_markers: Global SQP trajectory markers
- /visualization/tam_sampling_planner/opponent_traj: Global opponent trajectory markers
# - /visualization/tam_sampling_planner/collision_predict: Global collision prediction markers
# - /visualization/tam_sampling_planner/overtaking_reference: Global overtaking reference line
"""

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
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
        self.colors = {
            'car1': ColorRGBA(1.0, 0.2, 0.2, 0.8),  # Red with transparency
            'car2': ColorRGBA(0.2, 0.2, 1.0, 0.8),  # Blue with transparency
            'car3': ColorRGBA(0.2, 1.0, 0.2, 0.8),  # Green with transparency
            'car4': ColorRGBA(1.0, 1.0, 0.2, 0.8),  # Yellow with transparency
        }

        # Default color for unknown cars
        self.default_color = ColorRGBA(0.5, 0.5, 0.5, 0.8)  # Gray

        # Smooth visualization parameters
        self.visualization_rate = rospy.get_param(
            '~visualization_rate', 20.0)  # 20 Hz default
        self.last_markers = MarkerArray()  # Cache last markers for smooth republishing
        self.last_update_time = rospy.Time.now()
        self.marker_timeout = rospy.get_param(
            '~marker_timeout', 1.0)  # 1 second timeout

        # Initialize subscribers for car-specific topics
        self.init_subscribers()

        # Initialize publishers for global visualization topics
        self.init_publishers()

        rospy.loginfo(
            f"TAM Sampling Planner Visualization ready for {self.car_name}")

    def init_subscribers(self):
        """Initialize subscribers to car-specific marker topics"""
        opponent_topic = f'/{self.car_name}/prediction/opponent_markerarray'
        rospy.Subscriber(opponent_topic, MarkerArray, self.opponent_callback)

    def init_publishers(self):
        """Initialize publishers to global visualization topics"""
        self.opponent_pub = rospy.Publisher(
            '/visualization/tam_sampling_planner/opponent_traj',
            MarkerArray,
            queue_size=1
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

    def opponent_callback(self, msg):
        """Handle opponent trajectory prediction markers"""
        try:
            modified = self.modify_marker_array(msg, "opponent_traj")
            self.last_markers = modified
            self.last_update_time = rospy.Time.now()
            self.opponent_pub.publish(modified)
        except Exception as e:
            rospy.logwarn(
                f"Error processing opponent trajectory markers for {self.car_name}: {e}")

    def publish_smooth_predictions(self):
        """Publish predictions at fixed rate, using cached data if needed"""
        current_time = rospy.Time.now()

        # Check if we have recent data
        if self.last_markers.markers and (current_time - self.last_update_time).to_sec() < self.marker_timeout:
            # Update timestamps but keep same positions for smooth display
            for marker in self.last_markers.markers:
                marker.header.stamp = current_time
                # Extend lifetime slightly to overlap with next update
                marker.lifetime = rospy.Duration(
                    1.0 / self.visualization_rate + 0.1)

            try:
                self.opponent_pub.publish(self.last_markers)
            except rospy.ROSException:
                pass  # Handle shutdown gracefully


def main():
    """Main function"""
    try:
        visualization = TAMSamplingPlannerVisualization()

        # Use fixed-rate loop instead of rospy.spin() for smoother visualization
        rate = rospy.Rate(visualization.visualization_rate)

        rospy.loginfo(
            f"TAM visualization running at {visualization.visualization_rate} Hz")

        while not rospy.is_shutdown():
            visualization.publish_smooth_predictions()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("TAM visualization node interrupted")
    except Exception as e:
        rospy.logerr(f"TAM visualization node failed: {e}")
        raise


if __name__ == '__main__':
    main()
