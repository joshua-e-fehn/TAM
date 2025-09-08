#!/usr/bin/env python3
import tf2_ros
import tf.transformations as tft
import rospy
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TransformStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
from typing import List


class CarStateNode:
    def __init__(self, prop_state: bool = False):
        rospy.init_node('carstate_node', anonymous=True)
        # Parameters (private with legacy fallback)
        self.DEBUG = rospy.get_param(
            '~debug', rospy.get_param('/carstate_node/debug', False))
        self.LOCALIZATION = rospy.get_param(
            '~localization', rospy.get_param('/carstate_node/localization', 'slam'))
        self.ODOM_TOPIC = rospy.get_param(
            '~odom_topic', rospy.get_param('/carstate_node/odom_topic', 'odom'))
        self.IMU_TOPIC = rospy.get_param('~imu_topic', rospy.get_param(
            '/carstate_node/imu_topic', 'vesc/sensors/imu/raw'))
        self.prop_state = prop_state

        # Frame prefix support
        self.frame_prefix = rospy.get_param('~frame_prefix', '')
        if self.frame_prefix and not self.frame_prefix.endswith('/'):
            self.frame_prefix += '/'
        self.map_frame = self.frame_prefix + \
            rospy.get_param('~map_frame', 'map')
        self.odom_frame = self.frame_prefix + \
            rospy.get_param('~odom_frame', 'odom')
        self.base_frame = self.frame_prefix + \
            rospy.get_param('~base_frame', 'base_link')
        self.imu_frame = self.frame_prefix + \
            rospy.get_param('~imu_frame', 'imu')

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._wait_for_transforms()

        # State holders
        self.ekf_odom = None
        self.imu_data = None
        self.last_pose = None
        self.acc = np.zeros(5)
        self.vx_buffer = np.zeros(10)
        self.vy_buffer = np.zeros(10)
        self.MAX_PATH_LEN = 500
        self.path_counter = 0
        self.rate_hz = 100
        self.path_msg = Path()

        # Subs / pubs
        rospy.Subscriber(self.ODOM_TOPIC, Odometry,
                         self.ekf_odom_cb, tcp_nodelay=True)
        rospy.Subscriber(self.IMU_TOPIC, Imu, self.imu_cb, tcp_nodelay=True)
        self.pub_acc = rospy.Publisher(
            'acc_estimate', Float64, queue_size=1, tcp_nodelay=True)
        self.pub_state_pose = rospy.Publisher(
            'car_state/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.pub_state_odom = rospy.Publisher(
            'car_state/odom', Odometry, queue_size=1, tcp_nodelay=True)
        self.pub_state_diffodom = rospy.Publisher(
            'car_state/odom_diff', Odometry, queue_size=1, tcp_nodelay=True)
        self.pub_state_path = rospy.Publisher(
            'car_state/path', Path, queue_size=1, tcp_nodelay=True)
        self.pub_state_pitch = rospy.Publisher(
            'car_state/pitch', Float32, queue_size=1, tcp_nodelay=True)

        self.state_loop()

    def _wait_for_transforms(self):
        rospy.loginfo(
            'Waiting for required TF transforms (with prefix if provided)')
        needed = [
            (self.base_frame, self.imu_frame),
            (self.odom_frame, self.base_frame),
            (self.map_frame, self.odom_frame)
        ]
        rate = rospy.Rate(1)
        for parent, child in needed:
            while not rospy.is_shutdown() and not self.tf_buffer.can_transform(parent, child, rospy.Time(), rospy.Duration(1.0)):
                rospy.logwarn(f'Waiting for {parent}->{child} transform')
                rate.sleep()
            rospy.loginfo(f'{parent}->{child} transform OK')

    def ekf_odom_cb(self, msg: Odometry):
        self.ekf_odom = msg
        if self.prop_state:
            self.pub_acc.publish(np.mean(self.acc))
            self.ekf_odom.twist.twist.linear.x += 0.2 * np.mean(self.acc)

    def imu_cb(self, imu: Imu):
        self.acc[1:] = self.acc[:-1]
        self.acc[0] = -imu.linear_acceleration.y
        self.imu_data = imu

    def imu_to_rpy(self, imu: Imu):
        if imu is None:
            return 0.0, 0.0, 0.0
        try:
            b_imu = self.tf_buffer.lookup_transform(
                self.base_frame, self.imu_frame, rospy.Time(), rospy.Duration(0.5))
        except tf2_ros.LookupException:
            return 0.0, 0.0, 0.0
        rot_quat = quaternion_to_list(b_imu.transform.rotation)
        base_quat = tft.quaternion_multiply(
            [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w], rot_quat)
        r, p, y = tft.euler_from_quaternion(base_quat)
        return r, p, y

    def _lookup_pose(self) -> TransformStamped:
        return self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5))

    def state_loop(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo(
            'CarStateNode waiting for first Odometry & IMU messages...')
        rospy.wait_for_message(self.ODOM_TOPIC, Odometry)
        rospy.wait_for_message(self.IMU_TOPIC, Imu)
        rospy.loginfo('CarStateNode received initial Odometry & IMU')
        while not rospy.is_shutdown():
            try:
                trans = self._lookup_pose()
            except tf2_ros.LookupException:
                rate.sleep()
                continue

            pose_msg = PoseStamped()
            pose_msg.header = trans.header
            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = trans.transform.translation.z
            pose_msg.pose.orientation = trans.transform.rotation

            odom_msg = Odometry()
            odom_msg.header = pose_msg.header
            odom_msg.pose.pose = pose_msg.pose
            if self.ekf_odom:
                odom_msg.twist = self.ekf_odom.twist

            _, pitch, _ = self.imu_to_rpy(self.imu_data)
            pitch_msg = Float32()
            pitch_msg.data = pitch

            diff_odom_msg = Odometry()
            diff_odom_msg.header = pose_msg.header
            diff_odom_msg.pose.pose = pose_msg.pose
            if self.last_pose is not None:
                dt = 1.0 / self.rate_hz
                vx_map = (pose_msg.pose.position.x -
                          self.last_pose.pose.position.x) / dt
                vy_map = (pose_msg.pose.position.y -
                          self.last_pose.pose.position.y) / dt
                rot_q = quaternion_to_list(trans.transform.rotation)
                rot_mat = tft.quaternion_matrix(rot_q)
                vel_map = np.array([vx_map, vy_map, 0, 1])
                vel_base = np.dot(rot_mat.T, vel_map)
                self.vx_buffer[1:] = self.vx_buffer[:-1]
                self.vx_buffer[0] = vel_base[0]
                self.vy_buffer[1:] = self.vy_buffer[:-1]
                self.vy_buffer[0] = vel_base[1]
                diff_odom_msg.twist.twist.linear.x = np.mean(self.vx_buffer)
                diff_odom_msg.twist.twist.linear.y = np.mean(self.vy_buffer)
                odom_msg.twist.twist.linear.y = diff_odom_msg.twist.twist.linear.y

            self.pub_state_pose.publish(pose_msg)
            self.pub_state_odom.publish(odom_msg)
            self.pub_state_diffodom.publish(diff_odom_msg)
            self.pub_state_pitch.publish(pitch_msg)
            self.last_pose = pose_msg

            if self.DEBUG and self.ekf_odom is not None:
                self.path_handle(pose_msg, self.ekf_odom)
                self.pub_state_path.publish(self.path_msg)

            rate.sleep()

    def path_handle(self, pose: PoseStamped, odom: Odometry):
        self.path_msg.header = pose.header
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        if np.hypot(vx, vy) > 0.1 and self.path_counter % 8 == 0:
            self.path_msg.poses.append(pose)
            self.path_counter = 0
        else:
            self.path_counter += 1
        if len(self.path_msg.poses) >= self.MAX_PATH_LEN:
            self.path_msg.poses.pop(0)


def quaternion_to_list(q: Quaternion) -> List:
    return [q.x, q.y, q.z, q.w]


def translation_to_list(t: Vector3) -> List:
    return [t.x, t.y, t.z]


if __name__ == '__main__':  # single entry point
    CarStateNode(prop_state=False)
