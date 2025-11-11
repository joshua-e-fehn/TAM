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
- /global_waypoints: Global track waypoints (for start/finish line)

Publishes to:
- /visualization/tam_sampling_planner/{car_name}/planned_trajectory: Per-car planned trajectory markers
- /visualization/tam_sampling_planner/{car_name}/opponent_traj: Per-car opponent trajectory markers
- /visualization/tam_sampling_planner/{car_name}/controller_waypoints: Per-car controller waypoints
- /visualization/track/start_finish_line: Start/finish line marker
"""

import rospy
import math
import numpy as np
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
            'car1': ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Bright Red - Car1 SQP
            'car2': ColorRGBA(0.0, 0.5, 1.0, 1.0),  # Bright Blue - Car2 SQP
            'car3': ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Bright Green - Car3 SQP
            'car4': ColorRGBA(1.0, 1.0, 0.0, 1.0),  # Bright Yellow - Car4 SQP
        }

        # Opponent prediction colors (darker, more transparent)
        self.opponent_colors = {
            # Red - Car1's opponent prediction
            'car1': ColorRGBA(1.0, 0.0, 0.0, 0.5),
            # Blue - Car2's opponent prediction
            'car2': ColorRGBA(0.0, 0.5, 1.0, 0.5),
            # Green - Car3's opponent prediction
            'car3': ColorRGBA(0.0, 1.0, 0.0, 0.5),
            # Yellow - Car4's opponent prediction
            'car4': ColorRGBA(1.0, 1.0, 0.0, 0.5),
        }

        # Default colors for unknown cars
        self.default_planned_color = ColorRGBA(0.5, 0.5, 0.5, 1.0)  # Gray
        self.default_opponent_color = ColorRGBA(
            0.3, 0.3, 0.3, 0.6)  # Dark Gray

        # Track waypoint source for color coding controller waypoints
        self.waypoint_source = 'unknown'  # 'tam_planner', 'global_fallback', or 'unknown'

        # Track state machine state for conditional visualization
        self.state_machine_state = "GB_TRACK"  # Default state

        # Detect if running as predictive_sampler (for dual publishing)
        self.ot_planner = rospy.get_param(
            'state_machine/ot_planner', 'tam_sampling')
        self.is_predictive_sampler = (self.ot_planner == 'predictive_sampler')

        if self.is_predictive_sampler:
            rospy.loginfo(
                f"TAM Visualization: Running in PREDICTIVE SAMPLER mode - will publish opponent traj to both namespaces")

        # Track data for start/finish line
        self.global_waypoints = None
        # Initialize subscribers for car-specific topics
        self.start_finish_line_published = False
        self.init_subscribers()

        # Initialize publishers for global visualization topics
        self.init_publishers()

        rospy.loginfo(
            f"TAM Sampling Planner Visualization ready for {self.car_name}")

    def init_subscribers(self):
        """Initialize subscribers to car-specific marker topics"""

        # Opponent prediction markers - from simple TAM Sampling prediction
        opponent_tam_sampling_topic = f'/{self.car_name}/prediction/opponent_markerarray'
        rospy.Subscriber(opponent_tam_sampling_topic,
                         MarkerArray, self.opponent_tam_sampling_callback)

        # Opponent prediction markers - from Gaussian Process trajectory prediction
        opponent_predictive_topic = f'/{self.car_name}/opponent_traj_markerarray'
        rospy.Subscriber(opponent_predictive_topic,
                         MarkerArray, self.opponent_predictive_callback)

        # Planned trajectory markers
        planned_topic = f'/{self.car_name}/planner/avoidance/markers'
        rospy.Subscriber(planned_topic, MarkerArray,
                         self.planned_trajectory_callback)

        # Local waypoints - the actual trajectory sent to the controller (from state machine)
        local_wpnts_topic = f'/{self.car_name}/local_waypoints'
        rospy.Subscriber(local_wpnts_topic, WpntArray,
                         self.local_waypoints_callback)

        # TAM waypoint source - for color coding controller waypoints
        from std_msgs.msg import String
        source_topic = f'/{self.car_name}_state_machine/tam_waypoint_source'
        rospy.Subscriber(source_topic, String, self.waypoint_source_callback)

        # State machine state - for conditional visualization logic
        state_machine_topic = f'/{self.car_name}/state_machine'
        rospy.Subscriber(state_machine_topic, String,
                         self.state_machine_callback)

        # Global waypoints - for start/finish line (only subscribe once, not per car)
        if self.car_name == 'car1':  # Only car1 subscribes to avoid duplicate markers
            rospy.Subscriber('/global_waypoints', WpntArray,
                             self.global_waypoints_callback)

    def init_publishers(self):
        """Initialize publishers to car-specific visualization topics"""
        # Per-car opponent trajectory predictions
        self.opponent_pub = rospy.Publisher(
            f'/visualization/tam_sampling_planner/{self.car_name}/opponent_traj',
            MarkerArray,
            queue_size=1
        )

        # PREDICTIVE SAMPLER MODE: Also publish to predictive_spliner namespace for compatibility
        if self.is_predictive_sampler:
            self.opponent_pub_ps = rospy.Publisher(
                f'/visualization/predictive_spliner/{self.car_name}/opponent_traj',
                MarkerArray,
                queue_size=1
            )
            rospy.loginfo(
                f"TAM Visualization: Added predictive_spliner opponent_traj publisher for {self.car_name}")

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

        # Start/finish line publisher (only car1 publishes to avoid duplicates)
        if self.car_name == 'car1':
            self.start_finish_pub = rospy.Publisher(
                '/visualization/track/start_finish_line',
                Marker,
                queue_size=1,
                latch=True  # Latch so it persists
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

    def waypoint_source_callback(self, msg):
        """Handle waypoint source updates for color coding"""
        self.waypoint_source = msg.data
        rospy.logdebug_throttle(5.0,
                                f"{self.car_name}: Waypoint source updated to {self.waypoint_source}")

    def state_machine_callback(self, msg):
        """Handle state machine state updates"""
        self.state_machine_state = msg.data
        rospy.logdebug_throttle(5.0,
                                f"{self.car_name}: State machine state updated to {self.state_machine_state}")

    def global_waypoints_callback(self, msg):
        """Handle global waypoints update and create start/finish line marker"""
        if not self.start_finish_line_published and msg.wpnts:
            self.global_waypoints = msg
            self.publish_start_finish_line()
            self.start_finish_line_published = True
            rospy.loginfo("Start/finish line visualization published")

    def opponent_tam_sampling_callback(self, msg):
        """Handle opponent trajectory prediction markers"""
        if not self.is_predictive_sampler:
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

    def opponent_predictive_callback(self, msg):
        """Handle opponent trajectory prediction markers"""
        if self.is_predictive_sampler:
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
        """Create visualization markers for controller waypoints (downsampled, distinct style, color-coded by source)"""
        marker_array = MarkerArray()

        if not waypoints_msg.wpnts:
            return marker_array

        # Choose color based on waypoint source
        if self.waypoint_source == 'tam_planner' or self.state_machine_state not in ['OVERTAKE', 'TAM_PLANNING']:
            # TAM planner waypoints: Use car-specific color (bright)
            marker_color = self.get_car_color('controller')
        elif self.waypoint_source == 'global_fallback':
            # Global fallback waypoints: Use bright pink warning color
            # Bright Pink - clearly indicates fallback
            marker_color = ColorRGBA(1.0, 0.0, 0.8, 1.0)
        else:
            # Unknown source: Use gray
            marker_color = ColorRGBA(0.5, 0.5, 0.5, 1.0)  # Gray

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

            # Color - based on waypoint source
            marker.color = marker_color

            # Lifetime - short lifetime, will be republished
            marker.lifetime = rospy.Duration(0.15)

            marker_array.markers.append(marker)

        return marker_array

    def publish_start_finish_line(self):
        """Create and publish start/finish line marker from global waypoints"""
        if not self.global_waypoints or not self.global_waypoints.wpnts:
            return

        # Get first waypoint (s=0, start/finish line)
        start_wpnt = self.global_waypoints.wpnts[0]

        # Get track width at start
        track_width = start_wpnt.d_left + start_wpnt.d_right

        # Calculate perpendicular direction to track (90 degrees to heading)
        psi = start_wpnt.psi_rad
        # Perpendicular to track
        perp_angle = psi + np.pi / 2.0

        # Calculate line endpoints (from left to right boundary)
        left_x = start_wpnt.x_m + start_wpnt.d_left * np.cos(perp_angle)
        left_y = start_wpnt.y_m + start_wpnt.d_left * np.sin(perp_angle)

        right_x = start_wpnt.x_m - start_wpnt.d_right * np.cos(perp_angle)
        right_y = start_wpnt.y_m - start_wpnt.d_right * np.sin(perp_angle)

        # Create marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "track_start_finish"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Add points for the line
        p1 = Point()
        p1.x = left_x
        p1.y = left_y
        p1.z = 0.1  # Slightly above ground

        p2 = Point()
        p2.x = right_x
        p2.y = right_y
        p2.z = 0.1

        marker.points = [p1, p2]

        # Checkered flag style - black and white
        marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White

        # Line width
        marker.scale.x = 0.15  # Thick line for visibility

        # Identity orientation
        marker.pose.orientation.w = 1.0

        # Permanent marker
        marker.lifetime = rospy.Duration(0)

        # Publish
        if hasattr(self, 'start_finish_pub'):
            self.start_finish_pub.publish(marker)
            rospy.loginfo(
                f"Start/finish line published at ({start_wpnt.x_m:.2f}, {start_wpnt.y_m:.2f}), "
                f"width={track_width:.2f}m, heading={np.degrees(psi):.1f}°")


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
