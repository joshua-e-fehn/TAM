#!/usr/bin/env python3
import time
from typing import List, Any, Tuple

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from scipy.interpolate import InterpolatedUnivariateSpline as Spline

from dynamic_reconfigure.msg import Config
from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray
from frenet_converter.frenet_converter import FrenetConverter


class ObstacleSpliner:
    """
    This class implements a ROS node that performs splining around obstacles.

    It subscribes to the following topics:
        - `/perception/obstacles`: Subscribes to the obstacle array.
        - `/car_state/odom_frenet`: Subscribes to the car state in Frenet coordinates.
        - `/global_waypoints`: Subscribes to global waypoints.
        - `/global_waypoints_scaled`: Subscribes to the scaled global waypoints.

    The node publishes the following topics:
        - `/planner/avoidance/markers`: Publishes spline markers.
        - `/planner/avoidance/otwpnts`: Publishes splined waypoints.
        - `/planner/avoidance/considered_OBS`: Publishes markers for the closest obstacle.
        - `/planner/avoidance/propagated_obs`: Publishes markers for the propagated obstacle.
        - `/planner/avoidance/latency`: Publishes the latency of the spliner node. (only if measuring is enabled)
    """

    def __init__(self):
        """
        Initialize the node, subscribe to topics, and create publishers and service proxies.
        """
        # Initialize the node
        self.name = "obs_spliner_node"
        rospy.init_node(self.name)

        # Get car namespace for logging
        self.car_namespace = rospy.get_namespace().strip('/')
        if self.car_namespace:
            self.log_name = f"[{self.car_namespace}_{self.name}]"
        else:
            self.log_name = f"[{self.name}]"

        # initialize the instance variable
        self.obs = ObstacleArray()
        self.gb_wpnts = WpntArray()
        self.gb_vmax = None
        self.gb_max_idx = None
        self.gb_max_s = None
        self.cur_s = 0
        self.cur_d = 0
        self.cur_vs = 0
        self.gb_scaled_wpnts = WpntArray()
        self.lookahead = 10  # in meters [m]
        self.last_switch_time = rospy.Time.now()
        self.last_ot_side = ""
        self.from_bag = rospy.get_param("/from_bag", False)
        self.measuring = rospy.get_param("/measure", False)

        # Subscribe to the topics (use relative topic names for namespace-aware operation)
        rospy.Subscriber("perception/obstacles", ObstacleArray, self.obs_cb)
        rospy.Subscriber("car_state/odom_frenet", Odometry, self.state_cb)
        rospy.Subscriber("global_waypoints", WpntArray, self.gb_cb)
        rospy.Subscriber("global_waypoints_scaled",
                         WpntArray, self.gb_scaled_cb)
        # dyn params sub (internal representation - pre_apex are negative for positions before apex)
        self.pre_apex_0 = -4
        self.pre_apex_1 = -3
        self.pre_apex_2 = -1.5
        self.post_apex_0 = 4.5
        self.post_apex_1 = 5.0
        self.post_apex_2 = 5.5
        self.evasion_dist = 0.65
        self.obs_traj_tresh = 1.5
        self.spline_bound_mindist = 0.2
        if not self.from_bag:
            rospy.Subscriber(
                "dynamic_spline_tuner_node/parameter_updates", Config, self.dyn_param_cb)

        # Publishers (use relative topic names for namespace-aware operation)
        self.mrks_pub = rospy.Publisher(
            "planner/avoidance/markers", MarkerArray, queue_size=10)
        self.evasion_pub = rospy.Publisher(
            "planner/avoidance/otwpnts", OTWpntArray, queue_size=10)
        self.closest_obs_pub = rospy.Publisher(
            "planner/avoidance/considered_OBS", Marker, queue_size=10)
        self.pub_propagated = rospy.Publisher(
            "planner/avoidance/propagated_obs", Marker, queue_size=10)
        if self.measuring:
            self.latency_pub = rospy.Publisher(
                "planner/avoidance/latency", Float32, queue_size=10)

        self.converter = self.initialize_converter()

        # Set the rate at which the loop runs
        self.rate = rospy.Rate(20)  # Hz

    #############
    # CALLBACKS #
    #############
    # Callback for obstacle topic
    def obs_cb(self, data: ObstacleArray):
        self.obs = data

    def state_cb(self, data: Odometry):
        self.cur_s = data.pose.pose.position.x
        self.cur_d = data.pose.pose.position.y
        self.cur_vs = data.twist.twist.linear.x

    # Callback for global waypoint topic
    def gb_cb(self, data: WpntArray):
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m]
                                  for wpnt in data.wpnts])
        self.gb_wpnts = data
        if self.gb_vmax is None:
            self.gb_vmax = np.max(
                np.array([wpnt.vx_mps for wpnt in data.wpnts]))
            self.gb_max_idx = data.wpnts[-1].id
            self.gb_max_s = data.wpnts[-1].s_m

    # Callback for scaled global waypoint topic
    def gb_scaled_cb(self, data: WpntArray):
        self.gb_scaled_wpnts = data

    # Callback triggered by dynamic spline reconf
    def dyn_param_cb(self, params: Config):
        """
        Notices the change in the parameters and changes spline params
        """
        # Detect if we're in a car namespace (multi-car mode)
        node_name = rospy.get_name()
        car_namespace = None
        if '/car1/' in node_name or node_name.startswith('/car1'):
            car_namespace = '/car1'
        elif '/car2/' in node_name or node_name.startswith('/car2'):
            car_namespace = '/car2'

        def get_param(param_name, dyn_reconf_param, default):
            """Get parameter checking car namespace first if in multi-car mode"""
            if car_namespace:
                return rospy.get_param(f'{car_namespace}/{param_name}', rospy.get_param(param_name, rospy.get_param(dyn_reconf_param, default)))
            else:
                return rospy.get_param(param_name, rospy.get_param(dyn_reconf_param, default))

        # Priority: 1) Car-namespaced (multi-car), 2) Global (single-car), 3) Dynamic reconfigure, 4) Defaults
        self.pre_apex_0 = -1 * get_param(
            "pre_apex_dist0", "dynamic_spline_tuner_node/pre_apex_dist0", 4.0)
        self.pre_apex_1 = -1 * get_param(
            "pre_apex_dist1", "dynamic_spline_tuner_node/pre_apex_dist1", 3.0)
        self.pre_apex_2 = -1 * get_param(
            "pre_apex_dist2", "dynamic_spline_tuner_node/pre_apex_dist2", 2.0) + 0.1
        self.post_apex_0 = get_param(
            "post_apex_dist0", "dynamic_spline_tuner_node/post_apex_dist0", 4.5)
        self.post_apex_1 = get_param(
            "post_apex_dist1", "dynamic_spline_tuner_node/post_apex_dist1", 5.0)
        self.post_apex_2 = get_param(
            "post_apex_dist2", "dynamic_spline_tuner_node/post_apex_dist2", 5.5)

        self.evasion_dist = get_param(
            "evasion_dist", "dynamic_spline_tuner_node/evasion_dist", 0.65)
        self.obs_traj_tresh = get_param(
            "obs_traj_tresh", "dynamic_spline_tuner_node/obs_traj_tresh", 1.5)
        self.spline_bound_mindist = get_param(
            "spline_bound_mindist", "dynamic_spline_tuner_node/spline_bound_mindist", 0.2)

        self.kd_obs_pred = get_param(
            "kd_obs_pred", "dynamic_spline_tuner_node/kd_obs_pred", 1.0)
        self.fixed_pred_time = get_param(
            "fixed_pred_time", "dynamic_spline_tuner_node/fixed_pred_time", 0.15)

        spline_params = [
            self.pre_apex_0,
            self.pre_apex_1,
            self.pre_apex_2,
            0,
            self.post_apex_0,
            self.post_apex_1,
            self.post_apex_2,
        ]

        # Debug output with all parameter values
        rospy.logwarn(
            f"{'='*70}\n"
            f"[{self.name}] 🔧 DYNAMIC PARAMETERS UPDATED\n"
            f"{'='*70}\n"
            f"  Namespace: {car_namespace if car_namespace else 'global'}\n"
            f"  Spline control points [m]: {spline_params}\n"
            f"  ├─ pre_apex_0:  {self.pre_apex_0:7.2f} m (config: {-self.pre_apex_0:.2f})\n"
            f"  ├─ pre_apex_1:  {self.pre_apex_1:7.2f} m (config: {-self.pre_apex_1:.2f})\n"
            f"  ├─ pre_apex_2:  {self.pre_apex_2:7.2f} m (config: {-(self.pre_apex_2-0.1):.2f})\n"
            f"  ├─ post_apex_0: {self.post_apex_0:7.2f} m\n"
            f"  ├─ post_apex_1: {self.post_apex_1:7.2f} m\n"
            f"  └─ post_apex_2: {self.post_apex_2:7.2f} m\n"
            f"  Avoidance parameters:\n"
            f"  ├─ evasion_dist:         {self.evasion_dist:.3f} m\n"
            f"  ├─ obs_traj_tresh:       {self.obs_traj_tresh:.3f} m\n"
            f"  └─ spline_bound_mindist: {self.spline_bound_mindist:.3f} m\n"
            f"  Prediction parameters:\n"
            f"  ├─ kd_obs_pred:          {self.kd_obs_pred:.3f}\n"
            f"  └─ fixed_pred_time:      {self.fixed_pred_time:.3f} s\n"
            f"{'='*70}"
        )

    #############
    # MAIN LOOP #
    #############
    def loop(self):
        # Wait for critical Messages and services (use relative topic names)
        rospy.loginfo(f"[{self.name}] Waiting for messages and services...")
        rospy.wait_for_message("global_waypoints", WpntArray)
        rospy.wait_for_message("global_waypoints_scaled", WpntArray)
        rospy.wait_for_message("car_state/odom", Odometry)
        rospy.wait_for_message(
            "dynamic_spline_tuner_node/parameter_updates", Config)
        rospy.loginfo(f"{self.log_name} Ready!")

        while not rospy.is_shutdown():
            if self.measuring:
                start = time.perf_counter()
            # Sample data
            obs = self.obs
            gb_scaled_wpnts = self.gb_scaled_wpnts.wpnts
            wpnts = OTWpntArray()
            mrks = MarkerArray()

            # If obs then do splining around it
            if len(obs.obstacles) > 0:
                wpnts, mrks = self.do_spline(
                    obstacles=obs, gb_wpnts=gb_scaled_wpnts)
            # Else delete spline markers
            else:
                del_mrk = Marker()
                del_mrk.header.stamp = rospy.Time.now()
                del_mrk.action = Marker.DELETEALL
                mrks.markers.append(del_mrk)

            # Publish wpnts and markers
            if self.measuring:
                end = time.perf_counter()
                self.latency_pub.publish(end - start)
            self.evasion_pub.publish(wpnts)
            self.mrks_pub.publish(mrks)
            self.rate.sleep()

    #########
    # UTILS #
    #########
    def initialize_converter(self) -> FrenetConverter:
        """
        Initialize the FrenetConverter object"""
        rospy.wait_for_message("global_waypoints", WpntArray)

        # Initialize the FrenetConverter object
        converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
        rospy.loginfo(f"[{self.name}] initialized FrenetConverter object")

        return converter

    def _more_space(self, obstacle: Obstacle, gb_wpnts: List[Any], gb_idxs: List[int]) -> Tuple[str, float]:
        left_gap = abs(gb_wpnts[gb_idxs[0]].d_left - obstacle.d_left)
        right_gap = abs(gb_wpnts[gb_idxs[0]].d_right + obstacle.d_right)
        min_space = self.evasion_dist + self.spline_bound_mindist

        if right_gap > min_space and left_gap < min_space:
            # Compute apex distance to the right of the opponent
            d_apex_right = obstacle.d_right - self.evasion_dist
            # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
            if d_apex_right > 0:
                d_apex_right = 0
            return "right", d_apex_right

        elif left_gap > min_space and right_gap < min_space:
            # Compute apex distance to the left of the opponent
            d_apex_left = obstacle.d_left + self.evasion_dist
            # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
            if d_apex_left < 0:
                d_apex_left = 0
            return "left", d_apex_left
        else:
            candidate_d_apex_left = obstacle.d_left + self.evasion_dist
            candidate_d_apex_right = obstacle.d_right - self.evasion_dist

            if abs(candidate_d_apex_left) <= abs(candidate_d_apex_right):
                # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
                if candidate_d_apex_left < 0:
                    candidate_d_apex_left = 0
                return "left", candidate_d_apex_left
            else:
                # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
                if candidate_d_apex_right > 0:
                    candidate_d_apex_right = 0
                return "right", candidate_d_apex_right

    def do_spline(self, obstacles: ObstacleArray, gb_wpnts: WpntArray) -> Tuple[WpntArray, MarkerArray]:
        """
        Creates an evasion trajectory for the closest obstacle by splining between pre- and post-apex points.

        This function takes as input the obstacles to be evaded, and a list of global waypoints that describe a reference raceline.
        It only considers obstacles that are within a threshold of the raceline and generates an evasion trajectory for each of these obstacles.
        The evasion trajectory consists of a spline between pre- and post-apex points that are spaced apart from the obstacle.
        The spatial and velocity components of the spline are calculated using the `Spline` class, and the resulting waypoints and markers are returned.

        Args:
        - obstacles (ObstacleArray): An array of obstacle objects to be evaded.
        - gb_wpnts (WpntArray): A list of global waypoints that describe a reference raceline.
        - state (Odometry): The current state of the car.

        Returns:
        - wpnts (WpntArray): An array of waypoints that describe the evasion trajectory to the closest obstacle.
        - mrks (MarkerArray): An array of markers that represent the waypoints in a visualization format.

        """
        # Return wpnts and markers
        mrks = MarkerArray()
        wpnts = OTWpntArray()

        # Get spacing between wpnts for rough approximations
        wpnt_dist = gb_wpnts[1].s_m - gb_wpnts[0].s_m

        # Only use obstacles that are within a threshold of the raceline, else we don't care about them
        close_obs = self._obs_filtering(obstacles=obstacles)

        # If there are obstacles within the lookahead distance, then we need to generate an evasion trajectory considering the closest one
        if len(close_obs) > 0:
            # Get the closest obstacle handling wraparound
            closest_obs = min(
                close_obs, key=lambda obs: (
                    obs.s_center - self.cur_s) % self.gb_max_s
            )

            # Get Apex for evasion that is further away from the trackbounds
            if closest_obs.s_end < closest_obs.s_start:
                s_apex = (closest_obs.s_end + self.gb_max_s +
                          closest_obs.s_start) / 2
            else:
                s_apex = (closest_obs.s_end + closest_obs.s_start) / 2
            # Approximate next 20 indexes of global wpnts with wrapping => 2m and compute which side is the outside of the raceline
            gb_idxs = [int(s_apex / wpnt_dist + i) %
                       self.gb_max_idx for i in range(20)]
            kappas = np.array(
                [gb_wpnts[gb_idx].kappa_radpm for gb_idx in gb_idxs])
            outside = "left" if np.sum(kappas) < 0 else "right"
            # Choose the correct side and compute the distance to the apex based on left of right of the obstacle
            more_space, d_apex = self._more_space(
                closest_obs, gb_wpnts, gb_idxs)

            # Publish the point around which we are splining
            mrk = self.xy_to_point(
                x=gb_wpnts[gb_idxs[0]].x_m, y=gb_wpnts[gb_idxs[0]].y_m, opponent=False)
            self.closest_obs_pub.publish(mrk)

            # Choose wpnts from global trajectory for splining with velocity
            evasion_points = []
            spline_params = [
                self.pre_apex_0,
                self.pre_apex_1,
                self.pre_apex_2,
                0,
                self.post_apex_0,
                self.post_apex_1,
                self.post_apex_2,
            ]
            for i, dst in enumerate(spline_params):
                # scale dst linearly between 1 and 1.5 depending on the speed normalised to the max speed
                dst = dst * np.clip(1.0 + self.cur_vs / self.gb_vmax, 1, 1.5)
                # If we overtake on the outside, we smoothen the spline
                if outside == more_space:
                    si = s_apex + dst * 1.75  # TODO make parameter
                else:
                    si = s_apex + dst
                di = d_apex if dst == 0 else 0
                evasion_points.append([si, di])
            # Convert to nump
            evasion_points = np.array(evasion_points)

            # Spline spatialy for d with s as base
            # TODO read from ros params to make consistent in case it changes
            spline_resolution = 0.1
            spatial_spline = Spline(
                x=evasion_points[:, 0], y=evasion_points[:, 1])
            evasion_s = np.arange(
                evasion_points[0, 0], evasion_points[-1, 0], spline_resolution)
            # Clipe the d to the apex distance
            if d_apex < 0:
                evasion_d = np.clip(spatial_spline(evasion_s), d_apex, 0)
            else:
                evasion_d = np.clip(spatial_spline(evasion_s), 0, d_apex)

            # Handle Wrapping of s
            evasion_s = evasion_s % self.gb_max_s

            # Do frenet conversion via conversion service for spline and create markers and wpnts
            danger_flag = False
            resp = self.converter.get_cartesian(evasion_s, evasion_d)

            # Check if a side switch is possible
            if not self._check_ot_side_possible(more_space):
                danger_flag = True

            for i in range(evasion_s.shape[0]):
                gb_wpnt_i = int((evasion_s[i] / wpnt_dist) % self.gb_max_idx)
                # Check if wpnt is too close to the trackbounds but only if spline is actually off the raceline
                if abs(evasion_d[i]) > spline_resolution:
                    tb_dist = gb_wpnts[gb_wpnt_i].d_left if more_space == "left" else gb_wpnts[gb_wpnt_i].d_right
                    if abs(evasion_d[i]) > abs(tb_dist) - self.spline_bound_mindist:
                        # rospy.loginfo_throttle_identical(
                        #     2, f"{self.log_name}: Evasion trajectory too close to TRACKBOUNDS, aborting evasion"
                        # )
                        danger_flag = True
                        break
                # Get V from gb wpnts and go slower if we are going through the inside
                # TODO make speed scaling ros param
                vi = gb_wpnts[gb_wpnt_i].vx_mps if outside == more_space else gb_wpnts[gb_wpnt_i].vx_mps * 0.9
                wpnts.wpnts.append(
                    self.xyv_to_wpnts(
                        x=resp[0, i], y=resp[1, i], s=evasion_s[i], d=evasion_d[i], v=vi, wpnts=wpnts)
                )
                mrks.markers.append(self.xyv_to_markers(
                    x=resp[0, i], y=resp[1, i], v=vi, mrks=mrks))

            # Fill the rest of OTWpnts
            wpnts.header.stamp = rospy.Time.now()
            wpnts.header.frame_id = "map"
            if not danger_flag:
                wpnts.ot_side = more_space
                wpnts.ot_line = outside
                wpnts.side_switch = True if self.last_ot_side != more_space else False
                wpnts.last_switch_time = self.last_switch_time

                # Update the last switch time and the last side
                if self.last_ot_side != more_space:
                    self.last_switch_time = rospy.Time.now()
                self.last_ot_side = more_space
            else:
                wpnts.wpnts = []
                mrks.markers = []
                # This fools the statemachine to cool down
                wpnts.side_switch = True
                self.last_switch_time = rospy.Time.now()
        return wpnts, mrks

    def _obs_filtering(self, obstacles: ObstacleArray) -> List[Obstacle]:
        # Only use obstacles that are within a threshold of the raceline, else we don't care about them
        obs_on_traj = [obs for obs in obstacles.obstacles if abs(
            obs.d_center) < self.obs_traj_tresh]

        # Only use obstacles that within self.lookahead in front of the car
        close_obs = []
        for obs in obs_on_traj:
            obs = self._predict_obs_movement(obs)
            # Handle wraparound
            dist_in_front = (obs.s_center - self.cur_s) % self.gb_max_s
            # dist_in_back = abs(dist_in_front % (-self.gb_max_s)) # distance from ego to obstacle in the back
            if dist_in_front < self.lookahead:
                close_obs.append(obs)
                # Not within lookahead
            else:
                pass
        return close_obs

    def _predict_obs_movement(self, obs: Obstacle, mode: str = "constant") -> Obstacle:
        """
        Predicts the movement of an obstacle based on the current state and mode.

        TODO: opponent prediction should be completely isolated for added modularity       

        Args:
            obs (Obstacle): The obstacle to predict the movement for.
            mode (str, optional): The mode for predicting the movement. Defaults to "constant".

        Returns:
            Obstacle: The updated obstacle with the predicted movement.
        """
        # propagate opponent by time dependent on distance
        if (obs.s_center - self.cur_s) % self.gb_max_s < 10:  # TODO make param
            if mode == "adaptive":
                # distance in s coordinate
                cur_s = self.cur_s
                ot_distance = (obs.s_center - cur_s) % self.gb_max_s
                rel_speed = np.clip(self.gb_scaled_wpnts.wpnts[int(
                    cur_s * 10)].vx_mps - obs.vs, 0.1, 10)
                ot_time_distance = np.clip(ot_distance / rel_speed, 0, 5) * 0.5

                delta_s = ot_time_distance * obs.vs
                delta_d = ot_time_distance * obs.vd
                delta_d = -(obs.d_center + delta_d) * \
                    np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            elif mode == "adaptive_velheuristic":
                opponent_scaler = 0.7
                cur_s = self.cur_s
                ego_speed = self.gb_scaled_wpnts.wpnts[int(cur_s * 10)].vx_mps

                # distance in s coordinate
                ot_distance = (obs.s_center - cur_s) % self.gb_max_s
                rel_speed = (1 - opponent_scaler) * ego_speed
                ot_time_distance = np.clip(ot_distance / rel_speed, 0, 5)

                delta_s = ot_time_distance * opponent_scaler * ego_speed
                delta_d = -(obs.d_center) * \
                    np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            # propagate opponent by constant time
            elif mode == "constant":
                delta_s = self.fixed_pred_time * obs.vs
                delta_d = self.fixed_pred_time * obs.vd
                # delta_d = -(obs.d_center+delta_d)*np.exp(-np.abs(self.kd_obs_pred*obs.d_center))

            elif mode == "heuristic":
                # distance in s coordinate
                ot_distance = (obs.s_center - self.cur_s) % self.gb_max_s
                rel_speed = 3
                ot_time_distance = ot_distance / rel_speed

                delta_d = ot_time_distance * obs.vd
                delta_d = -(obs.d_center + delta_d) * \
                    np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            # update
            obs.s_start += delta_s
            obs.s_center += delta_s
            obs.s_end += delta_s
            obs.s_start %= self.gb_max_s
            obs.s_center %= self.gb_max_s
            obs.s_end %= self.gb_max_s

            obs.d_left += delta_d
            obs.d_center += delta_d
            obs.d_right += delta_d

            resp = self.converter.get_cartesian([obs.s_center], [obs.d_center])

            marker = self.xy_to_point(resp[0], resp[1], opponent=True)
            self.pub_propagated.publish(marker)

        return obs

    def _check_ot_side_possible(self, more_space) -> bool:
        # TODO make rosparam for cur_d threshold
        if abs(self.cur_d) > 0.25 and more_space != self.last_ot_side:
            # rospy.loginfo(
            #     f"{self.log_name}: Can't switch sides, because we are not on the raceline")
            return False
        return True

    ######################
    # VIZ + MSG WRAPPING #
    ######################
    def xyv_to_markers(self, x: float, y: float, v: float, mrks: MarkerArray) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = rospy.Time.now()
        mrk.type = mrk.CYLINDER
        mrk.scale.x = 0.1
        mrk.scale.y = 0.1
        mrk.scale.z = v / self.gb_vmax
        mrk.color.a = 1.0
        mrk.color.b = 0.75
        mrk.color.r = 0.75
        if self.from_bag:
            mrk.color.g = 0.75

        mrk.id = len(mrks.markers)
        mrk.pose.position.x = x
        mrk.pose.position.y = y
        mrk.pose.position.z = v / self.gb_vmax / 2
        mrk.pose.orientation.w = 1

        return mrk

    def xy_to_point(self, x: float, y: float, opponent=True) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = rospy.Time.now()
        mrk.type = mrk.SPHERE
        mrk.scale.x = 0.5
        mrk.scale.y = 0.5
        mrk.scale.z = 0.5
        mrk.color.a = 0.8
        mrk.color.b = 0.65
        mrk.color.r = 1 if opponent else 0
        mrk.color.g = 0.65

        mrk.pose.position.x = x
        mrk.pose.position.y = y
        mrk.pose.position.z = 0.01
        mrk.pose.orientation.w = 1

        return mrk

    def xyv_to_wpnts(self, s: float, d: float, x: float, y: float, v: float, wpnts: OTWpntArray) -> Wpnt:
        wpnt = Wpnt()
        wpnt.id = len(wpnts.wpnts)
        wpnt.x_m = x
        wpnt.y_m = y
        wpnt.s_m = s
        wpnt.d_m = d
        wpnt.vx_mps = v
        return wpnt


if __name__ == "__main__":
    spliner = ObstacleSpliner()
    spliner.loop()
