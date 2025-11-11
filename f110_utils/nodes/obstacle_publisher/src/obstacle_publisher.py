#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray, Wpnt, OpponentTrajectory, OppWpnt
from visualization_msgs.msg import Marker, MarkerArray
from frenet_conversion.srv import Glob2FrenetArr, Frenet2GlobArr
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import dynamic_reconfigure.client


class ObstaclePublisher:
    """Publish a dynamic obstacle in the F110 simulator

    node can be used to puclish a dynamic obstacle with a chosen speed, trajectory and starting 
    parameter "s".
    The described attributes can be set in the launch file.

    Attributes 
    ----------
        speed: float
            defines the speed of the obstacle in meters per second
        trajectory: string
            defines which trajectory is used among "min_curv", "shortest_path", "centerline"
        starting_s: float
            defines the inital starting parameter "s" as according to the Frenet Frame
    """

    def __init__(self):
        looprate = 50
        self.rate = rospy.Rate(looprate)
        self.looptime = 1/looprate

        self.dynamic_obstacle = self.init_dynamic_obstacle()
        self.obj_len = 0.5

        # Parameters (prefer private; fall back to legacy global keys for backward compatibility)
        self.speed_scaler = rospy.get_param(
            '~speed_scaler', rospy.get_param('obstacle_publisher/speed_scaler', 1))
        self.constant = rospy.get_param(
            '~constant_speed', rospy.get_param('obstacle_publisher/constant_speed', False))

        # Sinusoidal path deviation parameters
        self.path_amplitude = rospy.get_param(
            '~path_amplitude', rospy.get_param('obstacle_publisher/path_amplitude', 0.0))
        self.path_frequency = rospy.get_param(
            '~path_frequency', rospy.get_param('obstacle_publisher/path_frequency', 0.1))
        self.path_phase = rospy.get_param(
            '~path_phase', rospy.get_param('obstacle_publisher/path_phase', 0.0))

        # Frames (allow override & multi-car namespacing). Map frame may be prefixed externally.
        self.map_frame = rospy.get_param(
            '~map_frame', rospy.get_param('/map_frame', 'map'))

        # Initialize dynamic reconfigure client
        self.dyn_client = None
        # Wait a bit for the dynamic reconfigure server to start
        rospy.sleep(2.0)

        for attempt in range(3):
            try:
                rospy.loginfo(
                    "Connecting to dynamic reconfigure server (attempt %d/%d)...", attempt+1, 3)
                rospy.wait_for_service(
                    "dynamic_obstacle_server/set_parameters", timeout=10.0)
                self.dyn_client = dynamic_reconfigure.client.Client(
                    "dynamic_obstacle_server", timeout=10.0)
                rospy.loginfo(
                    "Successfully connected to dynamic reconfigure server!")
                break
            except Exception as e:
                rospy.logwarn(
                    f"Attempt {attempt+1} failed to connect to dynamic reconfigure server: {e}")
                if attempt < 2:
                    rospy.sleep(3.0)  # Wait before retry
                else:
                    rospy.logwarn("Using static parameters from launch file")

        # Update dynamic reconfigure server with launch file parameters
        if self.dyn_client is not None:
            try:
                rospy.loginfo(
                    "Updating dynamic reconfigure with launch file parameters...")
                self.dyn_client.update_configuration({
                    'speed_scaler': self.speed_scaler,
                    'path_amplitude': self.path_amplitude,
                    'path_frequency': self.path_frequency,
                    'path_phase': self.path_phase
                })
                rospy.loginfo(
                    f"Dynamic reconfigure updated: speed_scaler={self.speed_scaler}, path_amplitude={self.path_amplitude}")
            except Exception as e:
                rospy.logwarn(f"Failed to update dynamic reconfigure: {e}")

        # choose trajectory
        self.waypoints_type = rospy.get_param(
            '~trajectory', rospy.get_param('obstacle_publisher/trajectory', 'min_curv'))
        if self.waypoints_type == "min_curv":
            self.waypoints_topic = "global_waypoints"
        elif self.waypoints_type == "shortest_path":
            self.waypoints_topic = "global_waypoints/shortest_path"
        elif self.waypoints_type == "centerline":
            self.waypoints_topic = "centerline_waypoints"
        elif self.waypoints_type == "updated":
            self.waypoints_topic = "global_waypoints_updated"
            print("Using updated waypoints")
        elif self.waypoints_type == "min_time":
            raise NotImplementedError(
                "LTO Trajectory is not currently implemented. Choose another trajectory type."
            )
        else:
            raise ValueError(
                f"Waypoints of type {self.waypoints_type} are not supported."
            )

        self.starting_s = rospy.get_param(
            '~start_s', rospy.get_param('obstacle_publisher/start_s', 0))
        rospy.Subscriber('car_state/odom_frenet', Odometry, self.odom_cb)
        self.car_odom = Odometry()

        # State machine support - subscribe to state machine commands
        rospy.Subscriber('/state_machine_cmd', String, self.state_cmd_callback)
        self.current_state = "READY"  # Start in READY state (not moving)
        rospy.loginfo(
            "[Obstacle Publisher] Starting in READY state - waiting for start command")

        self.obstacle_pub = rospy.Publisher(
            'perception/obstacles', ObstacleArray, queue_size=10)
        self.obstacle_mrk_pub = rospy.Publisher(
            'dummy_obstacle_markers', MarkerArray, queue_size=10)
        self.opponent_traj_pub = rospy.Publisher(
            'opponent_waypoints', OpponentTrajectory, queue_size=10)
        self.obstacle_odom_pub = rospy.Publisher(
            'obstacle/odom', Odometry, queue_size=10)

        # Publish obstacle state for test framework monitoring
        self.obstacle_state_pub = rospy.Publisher(
            'obstacle/state', String, queue_size=10, latch=True)

        # Frenet Conversion Service
        rospy.wait_for_service("convert_glob2frenet_service")
        self.glob2frenet = rospy.ServiceProxy(
            "convert_glob2frenetarr_service", Glob2FrenetArr)
        self.frenet2glob = rospy.ServiceProxy(
            "convert_frenet2globarr_service", Frenet2GlobArr)
        self.mincurv_wpnts = None

    def init_dynamic_obstacle(self) -> Obstacle:
        """ Initializes the dynamic obstacles, it could be expanded to multiple obstacles, by changing the id
        """
        dynamic_obstacle = Obstacle()
        dynamic_obstacle.id = 1
        dynamic_obstacle.d_right = -0.1  # needs to be a smaller value than d_left
        dynamic_obstacle.d_left = 0.1
        dynamic_obstacle.is_actually_a_gap = False

        return dynamic_obstacle

    ### CALLBACKS ###

    def wpnts_cb(self, data: WpntArray):
        # exclude last point (because last point == first point)
        wpnts = data.wpnts[:-1]
        max_s = wpnts[-1].s_m
        return wpnts, max_s

    def odom_cb(self, data: Odometry):
        self.car_odom = data

    def state_cmd_callback(self, msg: String):
        """Handle state machine commands from race start controller"""
        new_state = msg.data
        if new_state != self.current_state:
            rospy.loginfo(
                f"[Obstacle Publisher] State change: {self.current_state} -> {new_state}")
            self.current_state = new_state
            # Publish state for monitoring
            self.obstacle_state_pub.publish(String(self.current_state))

    def update_dynamic_parameters(self):
        """Update parameters from dynamic reconfigure server if available"""
        if self.dyn_client is not None:
            try:
                config = self.dyn_client.get_configuration()
                old_speed = self.speed_scaler
                old_amplitude = self.path_amplitude
                old_frequency = self.path_frequency
                old_phase = self.path_phase

                self.speed_scaler = config.get(
                    'speed_scaler', self.speed_scaler)
                self.path_amplitude = config.get(
                    'path_amplitude', self.path_amplitude)
                self.path_frequency = config.get(
                    'path_frequency', self.path_frequency)
                self.path_phase = config.get('path_phase', self.path_phase)

                # Log parameter changes for debugging
                if (old_speed != self.speed_scaler or old_amplitude != self.path_amplitude or
                        old_frequency != self.path_frequency or old_phase != self.path_phase):
                    rospy.loginfo_throttle(1.0, f"[Obstacle Publisher] Updated parameters: "
                                           f"speed={self.speed_scaler:.3f}, "
                                           f"amplitude={self.path_amplitude:.3f}, "
                                           f"frequency={self.path_frequency:.3f}, "
                                           f"phase={self.path_phase:.3f}")

            except Exception as e:
                # If dynamic reconfigure fails, keep using current values
                rospy.logwarn_throttle(
                    5.0, f"[Obstacle Publisher] Failed to get dynamic config: {e}")
        else:
            # Try to reconnect if client is None
            try:
                self.dyn_client = dynamic_reconfigure.client.Client(
                    "dynamic_obstacle_publisher_node", timeout=1.0)
                rospy.loginfo_throttle(
                    10.0, "[Obstacle Publisher] Reconnected to dynamic reconfigure server!")
            except:
                pass

    ### HELPERS ###
    def publish_obstacle_cartesian(self, obstacles):
        """Visualizes obstacles in cartesian frame and publishes odometry for the first obstacle"""
        obs_markers = MarkerArray()
        for i, obs in enumerate(obstacles):
            # Do frenet conversion from (s,d) [frenet wrt min curv] -> (x,y) [cartesian]
            resp = self.frenet2glob([obs.s_center], [obs.d_center])
            x = resp.x[0]
            y = resp.y[0]

            obs_marker = Marker(header=rospy.Header(
                frame_id="map"), id=obs.id, type=Marker.SPHERE)
            obs_marker.scale.x = 0.5
            obs_marker.scale.y = 0.5
            obs_marker.scale.z = 0.5
            obs_marker.color.a = 0.5
            obs_marker.color.b = 0.5
            obs_marker.color.r = 0.5

            obs_marker.pose.position.x = x
            obs_marker.pose.position.y = y
            obs_marker.pose.orientation.w = 1
            obs_markers.markers.append(obs_marker)

            # Publish odometry for the first (main) obstacle
            if i == 0:
                self.publish_obstacle_odom(obs, x, y)

        self.obstacle_mrk_pub.publish(obs_markers)

    def publish_obstacle_odom(self, obstacle, x, y):
        """Publish obstacle odometry for speed monitoring"""
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "obstacle_base_link"

        # Position (in map frame)
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        # Velocity (linear speed from obstacle, assuming movement along track)
        odom_msg.twist.twist.linear.x = obstacle.vs
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        self.obstacle_odom_pub.publish(odom_msg)

    def shutdown(self):
        rospy.loginfo("BEEP BOOP DUMMY OD SHUTDOWN")
        self.obstacle_pub.publish(ObstacleArray())

    ### MAIN ###

    def ros_loop(self):
        """Main loop that moves around the car based on time measurement. It also publishes the 
        `Obstacle` message and the `MarkerArray`.
        """
        # Wait for essential messages to arrive
        rospy.loginfo("Dummy Obstacle Publisher waiting for waypoints...")
        rospy.wait_for_service("convert_frenet2globarr_service")
        rospy.wait_for_service("convert_glob2frenetarr_service")
        # Read in ego waypoints
        if self.waypoints_type == "updated":
            global_wpnts_msg = rospy.wait_for_message(
                "/global_waypoints_updated", WpntArray)
        else:
            global_wpnts_msg = rospy.wait_for_message(
                "/global_waypoints", WpntArray)
        global_wpnts, max_s = self.wpnts_cb(data=global_wpnts_msg)
        s_array = np.array([wpnt.s_m for wpnt in global_wpnts])

        # Read in opponent waypoints
        if self.constant:
            for i in range(len(global_wpnts)):
                # Base constant speed, scaling applied dynamically
                global_wpnts[i].vx_mps = 1.0
        else:
            # Keep original speeds, scaling will be applied dynamically
            pass

        opponent_wpnts_msg = rospy.wait_for_message(
            self.waypoints_topic, WpntArray)
        opponent_wpnts_list, _ = self.wpnts_cb(data=opponent_wpnts_msg)

        # Resmaple opponent waypoints to match ego waypoints

        opponent_xy = self.glob2frenet([wpnt.x_m for wpnt in opponent_wpnts_list], [
                                       wpnt.y_m for wpnt in opponent_wpnts_list])
        opponent_s = opponent_xy.s
        opponent_d = opponent_xy.d
        sorted_indices = sorted(range(len(opponent_s)),
                                key=lambda i: opponent_s[i])
        opponent_s_sorted = [opponent_s[i] for i in sorted_indices]
        opponent_d_sorted = [opponent_d[i] for i in sorted_indices]
        # opponent_vs_sorted= [opponent_wpnts_list[i].vx_mps for i in sorted_indices]
        # opponent_vd_sorted= [opponent_wpnts_list[i].vy_mps for i in sorted_indices]
        resampeld_opponent_d = np.interp(
            s_array, opponent_s_sorted, opponent_d_sorted)
        resampeld_opponent_vs = [wpnt.vx_mps for wpnt in global_wpnts]
        # np.interp(s_array, opponent_s_sorted, opponent_vs_sorted)
        # resampeld_opponent_vd = np.interp(s_array, opponent_s_sorted, opponent_vd_sorted)
        resampled_opponent_xy = self.frenet2glob(s_array, resampeld_opponent_d)

        self.opponent_wpnts = OpponentTrajectory()
        for i in range(len(s_array)):
            wpnt = OppWpnt()
            wpnt.x_m = resampled_opponent_xy.x[i]
            wpnt.y_m = resampled_opponent_xy.y[i]
            wpnt.proj_vs_mps = resampeld_opponent_vs[i]
            # wpnt.vy_mps = resampeld_opponent_vs[i]
            wpnt.s_m = s_array[i]
            wpnt.d_m = resampeld_opponent_d[i]
            self.opponent_wpnts.oppwpnts.append(wpnt)

        start_time = rospy.Time.now()

        rospy.sleep(0.1)

        # Add s offset only once in the beginning

        self.dynamic_obstacle.s_center = self.starting_s

        opponent_s_array = np.array(
            [wpnt.s_m for wpnt in self.opponent_wpnts.oppwpnts])
        rospy.loginfo("Dummy Obstacle Publisher ready.")

        # Publish initial READY state
        rospy.loginfo("[Obstacle Publisher] Publishing initial READY state")
        self.obstacle_state_pub.publish(String(self.current_state))

        counter = 0
        while not rospy.is_shutdown():
            time_tracker = rospy.Time.now()

            # Update dynamic parameters
            self.update_dynamic_parameters()

            # publish obstacle message
            obstacle_msg = ObstacleArray()
            obstacle_msg.header.stamp = rospy.Time.now()
            obstacle_msg.header.frame_id = "frenet"

            s = self.dynamic_obstacle.s_center
            approx_idx = np.abs(opponent_s_array - s).argmin()

            # Apply dynamic speed scaling to the base speed
            base_speed = self.opponent_wpnts.oppwpnts[approx_idx].proj_vs_mps
            self.dyn_obstacle_speed = base_speed * self.speed_scaler

            # Only update obstacle position if in GB_TRACK state (racing)
            # In READY state, obstacle stays at starting position
            if self.current_state == "GB_TRACK":
                self.dynamic_obstacle.s_center = (
                    self.dynamic_obstacle.s_center + self.dyn_obstacle_speed * self.looptime) % max_s
                self.dynamic_obstacle.s_start = (
                    self.dynamic_obstacle.s_center - self.obj_len/2) % max_s
                self.dynamic_obstacle.s_end = (
                    self.dynamic_obstacle.s_center + self.obj_len/2) % max_s
            else:
                # In READY state - obstacle doesn't move, velocity is 0
                self.dyn_obstacle_speed = 0.0

            # Base d position from the raceline
            base_d_center = self.opponent_wpnts.oppwpnts[approx_idx].d_m

            # Add sinusoidal deviation to the lateral position
            sinusoidal_deviation = self.path_amplitude * np.sin(
                self.path_frequency * self.dynamic_obstacle.s_center + self.path_phase
            )

            self.dynamic_obstacle.d_center = base_d_center + sinusoidal_deviation
            self.dynamic_obstacle.d_right = self.dynamic_obstacle.d_center - 0.1
            self.dynamic_obstacle.d_left = self.dynamic_obstacle.d_center + 0.1
            self.dynamic_obstacle.vs = self.dyn_obstacle_speed
            # Build s and d on selected reference line (self.waypoints_topic) by simply incrementing s and d=0
            # self.dynamic_obstacle.s_start = (self.dynamic_obstacle.s_start + self.dyn_obstacle_speed * self.looptime + self.starting_s) % max_s
            # self.dynamic_obstacle.s_end = max((self.dynamic_obstacle.s_start + self.obj_len), 0.1) % max_s
            # self.dynamic_obstacle.s_center = (self.dynamic_obstacle.s_start + self.obj_len / 2) % max_s  # using len to avoid wrap problems
            # self.dynamic_obstacle.d_center = (self.dynamic_obstacle.d_right + self.dynamic_obstacle.d_left) / 2
            # self.dynamic_obstacle.vs = self.dyn_obstacle_speed

            obstacle_msg.obstacles.append(self.dynamic_obstacle)
            self.publish_obstacle_cartesian(obstacle_msg.obstacles)

            self.obstacle_pub.publish(obstacle_msg)

            counter = counter + 1

            if counter > 25:
                # Lap count has to be bigger than 1 to show that the trajectory is updated after one lap
                opponent_traj_msg = OpponentTrajectory(header=rospy.Header(
                    frame_id="map", stamp=rospy.Time.now()), lap_count=2)
                opponent_traj_msg.oppwpnts = self.opponent_wpnts.oppwpnts
                self.opponent_traj_pub.publish(opponent_traj_msg)
                counter = 0

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("obstacle_publisher",
                    anonymous=False, log_level=rospy.INFO)
    obstacle_publisher = ObstaclePublisher()
    rospy.on_shutdown(obstacle_publisher.shutdown)
    obstacle_publisher.ros_loop()
