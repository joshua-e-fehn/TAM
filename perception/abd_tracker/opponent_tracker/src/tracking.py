#!/usr/bin/env python3
"""
Simplified pass-through opponent tracking node.
Maintains relative (namespaced) topics and private param fallbacks.
Legacy complex tracking removed for stability in multi-car setup.
"""
import math
import time
import rospy
from std_msgs.msg import Float32
from f110_msgs.msg import WpntArray, ObstacleArray, Obstacle
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from dynamic_reconfigure.msg import Config
from tf.transformations import euler_from_quaternion


class StaticDynamic:
    def __init__(self):
        # Use non-anonymous to ensure proper namespacing
        rospy.init_node('tracking', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Get car namespace for logging
        self.car_namespace = rospy.get_namespace().strip('/')
        if self.car_namespace:
            self.log_name = f"[{self.car_namespace}_Perception_Tracking]"
        else:
            self.log_name = "[Perception_Tracking]"

        def _p(pv, gv, default=None):
            return rospy.get_param(pv, rospy.get_param(gv, default))

        self.from_bag = _p('~from_bag', '/from_bag', False)
        self.measuring = _p('~measure', '/measure', False)
        self.rate_hz = _p('~rate', '/tracking/rate', 30)
        self.frame_prefix = _p('~frame_prefix', '/frame_prefix', '') or ''
        self.map_frame = _p('~map_frame', '/map_frame',
                            f"{self.frame_prefix}map")

        self.raw_obstacles = []
        self.multi_car_obstacles = []  # Add multi-car obstacles storage
        self.current_stamp = rospy.Time.now()
        self.waypoints = None
        self.track_length = None
        self.car_s = 0.0
        self.car_position = None
        self.car_orientation = None

        rospy.Subscriber('perception/detection/raw_obstacles',
                         ObstacleArray, self.raw_obs_cb)
        rospy.Subscriber('perception/multi_car_obstacles',
                         ObstacleArray, self.multi_car_obs_cb)  # Add multi-car subscriber
        rospy.Subscriber('global_waypoints', WpntArray, self.wp_cb)
        rospy.Subscriber('car_state/odom_frenet',
                         Odometry, self.odom_frenet_cb)
        rospy.Subscriber('car_state/odom', Odometry, self.odom_cb)
        if not self.from_bag:
            rospy.Subscriber(
                'dynamic_tracker_server/parameter_updates', Config, self.dyn_param_cb)

        self.pub_est = rospy.Publisher(
            'perception/obstacles', ObstacleArray, queue_size=5)
        self.pub_raw = rospy.Publisher(
            'perception/raw_obstacles', ObstacleArray, queue_size=5)
        self.pub_markers = rospy.Publisher(
            'perception/static_dynamic_marker_pub', MarkerArray, queue_size=5)
        if self.measuring:
            self.pub_latency = rospy.Publisher(
                'perception/tracking/latency', Float32, queue_size=5)

        if not rospy.has_param('~rate') and not rospy.has_param('/tracking/rate'):
            rospy.logwarn(
                '[Opponent Tracking] Using default tracking/rate=30 (param not set)')

    # Callbacks
    def raw_obs_cb(self, msg: ObstacleArray):
        self.raw_obstacles = msg.obstacles
        self.current_stamp = msg.header.stamp

    def multi_car_obs_cb(self, msg: ObstacleArray):
        """Handle multi-car obstacles from other cars"""
        self.multi_car_obstacles = msg.obstacles

    def wp_cb(self, msg: WpntArray):
        if self.waypoints is None:
            self.waypoints = msg.wpnts
            if msg.wpnts:
                self.track_length = msg.wpnts[-1].s_m
            rospy.loginfo('[Perception Tracking] received global path')

    def odom_frenet_cb(self, msg: Odometry):
        self.car_s = msg.pose.pose.position.x

    def odom_cb(self, msg: Odometry):
        self.car_position = (msg.pose.pose.position.x,
                             msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.car_orientation = (math.cos(yaw), math.sin(yaw))

    def dyn_param_cb(self, _cfg: Config):
        pass

    # Publishing logic
    def publish(self):
        arr = ObstacleArray()
        arr.header.stamp = self.current_stamp
        arr.header.frame_id = self.map_frame

        # Merge raw obstacles (from perception) and multi-car obstacles
        all_obstacles = list(self.raw_obstacles)  # Copy raw obstacles
        # Add multi-car obstacles
        all_obstacles.extend(self.multi_car_obstacles)

        arr.obstacles = all_obstacles
        self.pub_est.publish(arr)
        self.pub_raw.publish(arr)
        self.pub_markers.publish(MarkerArray())

    def main(self):
        rospy.loginfo(
            f'{self.log_name} Waiting for global waypoints & raw obstacles')
        rospy.wait_for_message('global_waypoints', WpntArray)
        rospy.wait_for_message(
            'perception/detection/raw_obstacles', ObstacleArray)
        rospy.loginfo(f'{self.log_name} Ready (pass-through mode)')
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            start = time.perf_counter() if self.measuring else None
            self.publish()
            if self.measuring:
                self.pub_latency.publish(time.perf_counter() - start)
            rate.sleep()

    def shutdown(self):
        rospy.logwarn('Tracking is shutdown')


if __name__ == '__main__':
    node = StaticDynamic()
    node.main()
