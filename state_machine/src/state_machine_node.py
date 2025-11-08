#!/usr/bin/env python3
import states
import state_transitions
from states_types import StateType
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Float32, Float32MultiArray, Bool
from scipy.interpolate import InterpolatedUnivariateSpline as Spline
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from f110_msgs.msg import ObstacleArray, OTWpntArray, WpntArray, Wpnt
from dynamic_reconfigure.msg import Config
import tf
import rospy
import numpy as np
import threading
import time
import warnings

# Suppress sklearn convergence warnings
warnings.filterwarnings("ignore", category=UserWarning, module="sklearn")


try:  # optional (car only)
    from vesc_msgs.msg import VescStateStamped
except ImportError:  # simulator environment without vesc package
    VescStateStamped = None  # type: ignore


class StateMachine:
    def __init__(self, name) -> None:
        self.name = name
        # Initialize car name for multi-car identification (will be set by main)
        self.car_name = "unknown_car"

        # Track node startup time to reject queued messages from before subscriber connected
        self.node_start_time = rospy.Time.now()

        # Core configuration
        self.rate_hz = rospy.get_param('state_machine/rate')
        self.n_loc_wpnts = rospy.get_param('state_machine/n_loc_wpnts')
        self.local_wpnts = WpntArray()
        self.waypoints_dist = 0.1
        self.lock = threading.Lock()
        self.measuring = rospy.get_param(
            '~measure', rospy.get_param('/measure', False))

        # Dynamic / map parameters (namespaced fallback to global)
        self.racecar_version = rospy.get_param(
            '~racecar_version', rospy.get_param('/racecar_version'))
        self.sectors_params = rospy.get_param(
            'map_params', rospy.get_param('/map_params'))
        self.timetrials_only = bool(rospy.get_param(
            'state_machine/timetrials_only', 'True'))
        self.n_sectors = self.sectors_params.get('n_sectors', 0)
        self.only_ftg_zones = []
        self.ftg_counter = 0
        self.ot_sectors_params = rospy.get_param(
            'ot_map_params', rospy.get_param('/ot_map_params'))
        self.n_ot_sectors = self.ot_sectors_params.get('n_sectors', 0)
        self.overtake_wpnts = None
        self.overtake_zones = []
        self.ot_begin_margin = 0.5
        self.cur_volt = 11.69
        self.volt_threshold = rospy.get_param(
            'state_machine/volt_threshold', 10)

        # Planner selection
        self.ot_planner = rospy.get_param(
            'state_machine/ot_planner', 'spliner')

        # Waypoint bookkeeping
        self.cur_id_ot = 1
        self.max_speed = -1
        self.max_s = 0
        self.current_position = None
        self.glb_wpnts = None
        self.gb_max_idx = None
        self.wpnt_dist = self.waypoints_dist
        self.num_glb_wpnts = 0
        self.num_ot_points = 0
        self.previous_index = 0
        self.gb_ego_width_m = rospy.get_param('state_machine/gb_ego_width_m')
        self.lateral_width_gb_m = rospy.get_param(
            'state_machine/lateral_width_gb_m', 0.75)
        self.gb_horizon_m = rospy.get_param('state_machine/gb_horizon_m')

        # Splines
        self.mincurv_spline_x = None
        self.mincurv_spline_y = None
        self.ot_spline_x = None
        self.ot_spline_y = None
        self.ot_spline_d = None
        self.recompute_ot_spline = True

        # Obstacles / perception
        self.obstacles = []
        self.obstacles_perception = []
        self.obstacles_prediction = []
        self.obstacle_was_here = True
        self.side_by_side_threshold = 0.6
        self.merger = None
        self.force_trailing = False

        # Spliner / predictive parameters
        self.splini_ttl = rospy.get_param(
            'state_machine/splini_ttl', 2.0) if self.ot_planner == 'spliner' else rospy.get_param('state_machine/pred_splini_ttl', 0.2)
        self.splini_ttl_counter = int(self.splini_ttl * self.rate_hz)
        self.avoidance_wpnts = None
        self.last_valid_avoidance_wpnts = None
        self.overtaking_horizon_m = rospy.get_param(
            'state_machine/overtaking_horizon_m', 6.9)
        self.lateral_width_ot_m = rospy.get_param(
            'state_machine/lateral_width_ot_m', 0.3)
        self.splini_hyst_timer_sec = rospy.get_param(
            'state_machine/splini_hyst_timer_sec', 0.75)
        self.emergency_break_horizon = 5.0
        self.emergency_break_d = 0.12

        # Graph Based / Frenet
        self.graph_based_wpts = None
        self.gb_wpnts_arr = None
        self.frenet_wpnts = WpntArray()

        # FTG
        self.ftg_speed_mps = rospy.get_param(
            'state_machine/ftg_speed_mps', 1.0)
        self.ftg_timer_sec = rospy.get_param(
            'state_machine/ftg_timer_sec', 3.0)
        self.ftg_disabled = True

        self.force_gbtrack_state = False

        # Visualization helpers
        self.first_visualization = True
        self.x_viz = 0
        self.y_viz = 0

        # Initial state - conditional based on race start controller
        # Check if race start controller is enabled via parameter
        use_race_start_controller = rospy.get_param('~use_race_start_controller',
                                                    rospy.get_param('/enable_race_start_controller', False))

        if use_race_start_controller:
            self.cur_state = StateType.READY
            rospy.loginfo(
                f"[{self.name}] Race start controller enabled - starting in READY state")
        else:
            self.cur_state = StateType.GB_TRACK
            rospy.loginfo(
                f"[{self.name}] Race start controller disabled - starting in GB_TRACK state")

        self.race_start_received = False
        # Track previous state for transition detection
        self.previous_state = self.cur_state
        rospy.loginfo(
            f"[{self.name}] The default state for the state machine is {self.cur_state}")

        # State transition maps
        if self.ot_planner == "spliner":
            self.state_transitions = {
                StateType.READY: state_transitions.ReadyTransition,
                StateType.GB_TRACK: state_transitions.SpliniGlobalTrackingTransition,
                StateType.TRAILING: state_transitions.SpliniTrailingTransition,
                StateType.OVERTAKE: state_transitions.SpliniOvertakingTransition,
                StateType.FTGONLY: state_transitions.SpliniFTGOnlyTransition,
            }
        elif self.ot_planner == "predictive_spliner":
            self.state_transitions = {
                StateType.READY: state_transitions.PSReadyTransition,
                StateType.GB_TRACK: state_transitions.PSGlobalTrackingTransition,
                StateType.TRAILING: state_transitions.PSTrailingTransition,
                StateType.OVERTAKE: state_transitions.PSOvertakingTransition,
                StateType.FTGONLY: state_transitions.PSFTGOnlyTransition,
            }
        elif self.ot_planner == "predictive_sampler":
            self.state_transitions = {
                StateType.READY: state_transitions.PSAMPReadyTransition,
                StateType.GB_TRACK: state_transitions.PSSAMPGlobalTrackingTransition,
                StateType.TRAILING: state_transitions.PSSAMPTrailingTransition,
                StateType.OVERTAKE: state_transitions.PSSAMPSamplingTransition,
                StateType.FTGONLY: state_transitions.PSSAMPFTGOnlyTransition,
            }
        elif self.ot_planner == "tam_sampling":
            self.state_transitions = {
                StateType.READY: state_transitions.TAMReadyTransition,
                StateType.TAM_PLANNING: state_transitions.TAMPlanningTransition,
            }
        elif self.ot_planner == "graph_based":
            rospy.logwarn(
                "[State Machine] Graph Based Planner is deprecated! Some packages might be missing!")
            self.state_transitions = {
                StateType.READY: state_transitions.GBReadyTransition,
                StateType.GB_TRACK: state_transitions.GBGlobalTrackingTransition,
                StateType.TRAILING: state_transitions.GBTrailingTransition,
                StateType.OVERTAKE: state_transitions.GBOvertakingTransition,
                StateType.FTGONLY: state_transitions.GBFTGOnlyTransition,
            }
        elif self.ot_planner == "frenet":
            self.state_transitions = {
                StateType.READY: state_transitions.FrenetReadyTransition,
                StateType.GB_TRACK: state_transitions.FrenetGlobalTrackingTransition,
                StateType.TRAILING: state_transitions.FrenetTrailingTransition,
                StateType.OVERTAKE: state_transitions.FrenetOvertakingTransition,
                StateType.FTGONLY: state_transitions.FrenetFTGOnlyTransition,
            }
        else:
            raise NotImplementedError(
                f"Planner {self.ot_planner} not supported!")

        self.states = {
            StateType.READY: states.Ready,
            StateType.GB_TRACK: states.GlobalTracking,
            StateType.TRAILING: states.Trailing,
            StateType.OVERTAKE: states.Overtaking,
            StateType.FTGONLY: states.FTGOnly,
            StateType.TAM_PLANNING: states.TAMTracking,
        }

        # SUBSCRIPTIONS (relative topics for namespacing)
        rospy.Subscriber("car_state/pose", PoseStamped, self.pose_cb)
        rospy.wait_for_message("car_state/pose", PoseStamped)
        rospy.Subscriber("global_waypoints_scaled",
                         WpntArray, self.glb_wpnts_cb)
        rospy.Subscriber("global_waypoints/overtaking",
                         WpntArray, self.overtake_cb)
        rospy.wait_for_message("global_waypoints_scaled", WpntArray)
        rospy.wait_for_message("global_waypoints/overtaking", WpntArray)
        rospy.Subscriber("car_state/odom_frenet",
                         Odometry, self.frenet_pose_cb)
        rospy.wait_for_message("car_state/odom_frenet", Odometry)
        rospy.Subscriber("global_waypoints", WpntArray, self.glb_wpnts_og_cb)

        # Race start command subscriber
        rospy.Subscriber("state_machine_cmd", String,
                         self.state_command_callback)

        # dynamic parameters subscriber
        rospy.Subscriber(
            "dynamic_statemachine_server/parameter_updates", Config, self.dyn_param_cb)
        rospy.Subscriber("dyn_sector_server/parameter_updates",
                         Config, self.sector_dyn_param_cb)
        rospy.Subscriber("ot_dyn_sector_server/parameter_updates",
                         Config, self.ot_dyn_param_cb)
        rospy.Subscriber("perception/obstacles", ObstacleArray,
                         self.obstacle_perception_cb)
        rospy.Subscriber("collision_prediction/obstacles",
                         ObstacleArray, self.obstacle_prediction_cb)
        if self.ot_planner in ("spliner", "predictive_spliner", "tam_sampling", "predictive_sampler"):
            rospy.Subscriber("planner/avoidance/otwpnts",
                             OTWpntArray, self.avoidance_cb, queue_size=1)
        if self.ot_planner in ["predictive_spliner", "predictive_sampler"]:
            rospy.Subscriber("planner/avoidance/merger",
                             Float32MultiArray, self.merger_cb)
            rospy.Subscriber("collision_prediction/force_trailing",
                             Bool, self.force_trailing_cb)
        elif self.ot_planner == "graph_based":
            rospy.Subscriber("planner/graph_based_wpnts",
                             Float32MultiArray, self.graphbased_wpts_cb)
        elif self.ot_planner == "frenet":
            rospy.Subscriber("planner/waypoints", WpntArray,
                             self.frenet_planner_cb)
        if not rospy.get_param('~sim', rospy.get_param('/sim', False)) and VescStateStamped is not None:
            rospy.Subscriber("vesc/sensors/core",
                             VescStateStamped, self.vesc_state_cb)

        # Parameters
        self.track_length = rospy.get_param(
            '~global_republisher/track_length', rospy.get_param('/global_republisher/track_length'))

        # PUBLICATIONS (relative)
        self.loc_wpnt_pub = rospy.Publisher(
            "local_waypoints", WpntArray, queue_size=1)
        self.vis_loc_wpnt_pub = rospy.Publisher(
            "local_waypoints/markers", MarkerArray, queue_size=10)
        self.state_pub = rospy.Publisher("state_machine", String, queue_size=1)
        self.state_transition_pub = rospy.Publisher(
            "state_transition", String, queue_size=10)  # For logging state changes
        self.state_mrk = rospy.Publisher("state_marker", Marker, queue_size=10)
        self.emergency_pub = rospy.Publisher(
            "emergency_marker", Marker, queue_size=5)
        self.ot_section_check_pub = rospy.Publisher(
            "ot_section_check", Bool, queue_size=1)
        if self.measuring:
            self.latency_pub = rospy.Publisher(
                "state_machine/latency", Float32, queue_size=10)

    def on_shutdown(self):
        rospy.loginfo(f"[{self.name}] Shutting down state machine")

    # MAIN LOOP start after construction if desired
    def start(self):
        self.loop()

    #############
    # CALLBACKS #
    #############
    def state_command_callback(self, msg):
        """Handle external state commands for race start"""
        if msg.data == "GB_TRACK" and self.cur_state == StateType.READY:
            rospy.loginfo(f"[{self.name}] Received RACE START command!")
            self.race_start_received = True
        elif msg.data == "READY":
            rospy.loginfo(f"[{self.name}] Received RESET to READY command!")
            self.cur_state = StateType.READY
            self.race_start_received = False
        elif msg.data == "EMERGENCY":
            rospy.logwarn(f"[{self.name}] Received EMERGENCY STOP command!")
            self.cur_state = StateType.READY
            self.race_start_received = False

    def vesc_state_cb(self, data):
        """vesc state callback, reads the voltage"""
        self.cur_volt = data.state.voltage_input

    def frenet_planner_cb(self, data: WpntArray):
        """frenet planner waypoints"""
        self.frenet_wpnts = data

    def avoidance_cb(self, data: OTWpntArray):
        """spliniboi waypoints"""
        callback_start = rospy.Time.now()

        # Reject messages published BEFORE this node started (backlog from connection establishment)
        # This prevents processing 100+ queued messages when subscriber first connects
        if data.header.stamp < self.node_start_time:
            # Silently ignore old messages from before node startup
            return

        # Calculate message age
        msg_age = (callback_start - data.header.stamp).to_sec()

        # Reject stale messages (older than 200ms) to prevent using outdated trajectories
        # At 20Hz planning rate, 200ms = 4 planning cycles is reasonable tolerance
        if msg_age > 0.6:
            # rospy.logwarn_throttle(5.0,  # Only log every 5 seconds to avoid spam
            #                        f"[{self.name}] 🕐 REJECTING STALE MESSAGE | "
            #                        f"msg_age={msg_age*1000:.1f}ms | "
            #                        f"wpnts={len(data.wpnts)} | "
            #                        f"Ignoring outdated trajectory")
            return

        # Additional check: Only accept messages newer than the last one we processed
        if not hasattr(self, 'last_trajectory_timestamp'):
            self.last_trajectory_timestamp = rospy.Time(0)

        if data.header.stamp <= self.last_trajectory_timestamp:
            # This message is older than or equal to the last one we processed
            rospy.logdebug(
                f"[{self.name}] Skipping message with duplicate/old timestamp")
            return

        # Update last processed timestamp
        self.last_trajectory_timestamp = data.header.stamp

        if len(data.wpnts) > 0:
            self.splini_ttl_counter = int(self.splini_ttl * self.rate_hz)
            self.avoidance_wpnts = data

            # DEBUG: Log receipt of avoidance waypoints
            rospy.loginfo_throttle(2.0,
                                   f"[{self.name}] ✓ Received {len(data.wpnts)} avoidance waypoints | "
                                   f"s_range=[{data.wpnts[0].s_m:.2f}, {data.wpnts[-1].s_m:.2f}] | "
                                   f"First wpnt: x={data.wpnts[0].x_m:.2f}, y={data.wpnts[0].y_m:.2f} | "
                                   f"msg_age={msg_age*1000:.1f}ms")
        else:
            # Empty trajectory received - planner failed or no valid trajectory
            # Clear avoidance_wpnts to trigger fallback in states.py
            self.avoidance_wpnts = None
            callback_time = (rospy.Time.now() - callback_start).to_sec()
            # rospy.logwarn(
            #     f"[{self.name}] ⚠️ Received EMPTY avoidance waypoints - setting to None | "
            #     f"msg_age={msg_age*1000:.1f}ms | callback_time={callback_time*1000:.1f}ms | "
            #     f"(published at {data.header.stamp.to_sec():.3f}, received at {callback_start.to_sec():.3f})")

    def frenet_pose_cb(self, data: Odometry):
        self.cur_s = data.pose.pose.position.x
        self.cur_d = data.pose.pose.position.y
        self.cur_vs = data.twist.twist.linear.x
        if self.num_ot_points != 0:
            self.cur_id_ot = int(self._find_nearest_ot_s())

    def overtake_cb(self, data):
        """
        Callback function of overtake subscriber.

        Parameters
        ----------
        data
            Data received from overtake topic
        """
        self.overtake_wpnts = data.wpnts
        self.num_ot_points = len(self.overtake_wpnts)

        # compute the OT spline when new spline
        if self.recompute_ot_spline and self.num_ot_points != 0:
            self.ot_splinification()
            self.recompute_ot_spline = False

    def glb_wpnts_cb(self, data: WpntArray):
        """
        Callback function of velocity interpolator subscriber.

        Parameters
        ----------
        data
            Data received from velocity interpolator topic
        """
        self.glb_wpnts = data.wpnts[:-
                                    1]  # exclude last point (because last point == first point)
        self.num_glb_wpnts = len(self.glb_wpnts)
        self.max_s = data.wpnts[-1].s_m
        # Get spacing between wpnts for rough approximations
        self.wpnt_dist = data.wpnts[1].s_m - data.wpnts[0].s_m
        self.gb_max_idx = data.wpnts[-1].id
        if self.ot_planner == "graph_based":
            self.gb_wpnts_arr = np.array([
                [w.s_m, w.d_m, w.x_m, w.y_m, w.d_right, w.d_left, w.psi_rad,
                 w.kappa_radpm, w.vx_mps, w.ax_mps2] for w in data.wpnts
            ])

    def glb_wpnts_og_cb(self, data):
        """
        Callback function of OG global waypoints 100% speed.

        Parameters
        ----------
        data
            Data received from velocity interpolator topic
        """
        if self.max_speed == -1:
            self.max_speed = max([wpnt.vx_mps for wpnt in data.wpnts])
        else:
            pass

    def graphbased_wpts_cb(self, data):
        arr = np.asarray(data.data)
        self.graph_based_wpts = arr.reshape(
            data.layout.dim[0].size, data.layout.dim[1].size)
        self.graph_based_action = data.layout.dim[0].label

    def obstacle_perception_cb(self, data):
        if len(data.obstacles) != 0:
            self.obstacles_perception = data.obstacles
            self.obstacles = data.obstacles
            self.obstacles = self.obstacles + self.obstacles_prediction
        else:
            self.obstacles = []

    def obstacle_prediction_cb(self, data):
        if len(data.obstacles) != 0:
            self.obstacles_prediction = data.obstacles
            self.obstacles = data.obstacles
            self.obstacles = self.obstacles + self.obstacles_perception
        else:
            self.obstacles_prediction = []

    def pose_cb(self, data):
        """
        Callback function of /tracked_pose subscriber.

        Parameters
        ----------
        data
            Data received from /tracked_pose topic
        """
        x = data.pose.position.x
        y = data.pose.position.y
        theta = tf.transformations.euler_from_quaternion(
            [data.pose.orientation.x, data.pose.orientation.y,
                data.pose.orientation.z, data.pose.orientation.w]
        )[2]

        self.current_position = [x, y, theta]

    def dyn_param_cb(self, params: Config):
        """
        Notices the change in the State Machine parameters and sets
        """
        self.lateral_width_gb_m = rospy.get_param(
            "dynamic_statemachine_server/lateral_width_gb_m", 0.75)
        self.lateral_width_ot_m = rospy.get_param(
            "dynamic_statemachine_server/lateral_width_ot_m", 0.3)
        self.splini_ttl = rospy.get_param(
            "dynamic_statemachine_server/splini_ttl") if self.ot_planner == "spliner" else rospy.get_param("dynamic_statemachine_server/pred_splini_ttl")
        # convert seconds to counter
        self.splini_ttl_counter = int(self.splini_ttl * self.rate_hz)
        self.splini_hyst_timer_sec = rospy.get_param(
            "dynamic_statemachine_server/splini_hyst_timer_sec", 0.75)
        self.emergency_break_horizon = rospy.get_param(
            "dynamic_statemachine_server/emergency_break_horizon", 5.0)  # Must be > trailing_gap for safety
        self.ftg_speed_mps = rospy.get_param(
            "dynamic_statemachine_server/ftg_speed_mps", 1.0)
        self.ftg_timer_sec = rospy.get_param(
            "dynamic_statemachine_server/ftg_timer_sec", 3.0)

        self.ftg_disabled = not rospy.get_param(
            "dynamic_statemachine_server/ftg_active", False)
        self.force_gbtrack_state = rospy.get_param(
            "dynamic_statemachine_server/force_GBTRACK", False)

        if self.force_gbtrack_state:
            rospy.logwarn(f"[{self.name}] GBTRACK state force activated!!!")

        rospy.logdebug(
            "[{}] Received new parameters for state machine: lateral_width_gb_m: {}, "
            "lateral_width_ot_m: {}, splini_ttl: {}, splini_hyst_timer_sec: {}, ftg_speed_mps: {}, "
            "ftg_timer_sec: {}, GBTRACK_force: {}".format(
                self.name,
                self.lateral_width_gb_m,
                self.lateral_width_ot_m,
                self.splini_ttl,
                self.splini_hyst_timer_sec,
                self.ftg_speed_mps,
                self.ftg_timer_sec,
                self.force_gbtrack_state
            )
        )

    def sector_dyn_param_cb(self, params: Config):
        """
        Notices the change in the parameters and sets no/only ftg zones
        """
        # reset ftg zones
        self.only_ftg_zones = []
        # update ftg zones
        for i in range(self.n_sectors):
            self.sectors_params[f"Sector{i}"]["only_FTG"] = params.bools[2 * i].value
            if self.sectors_params[f"Sector{i}"]["only_FTG"]:
                self.only_ftg_zones.append(
                    [self.sectors_params[f"Sector{i}"]["start"],
                        self.sectors_params[f"Sector{i}"]["end"]]
                )

    def ot_dyn_param_cb(self, params: Config):
        """
        Notices the change in the parameters and sets overtaking zones
        """
        # reset overtake zones
        self.overtake_zones = []
        # update overtake zones
        try:
            for i in range(self.n_ot_sectors):
                self.ot_sectors_params[f"Overtaking_sector{i}"]["ot_flag"] = params.bools[i].value
                # add start and end index of the sector
                if self.ot_sectors_params[f"Overtaking_sector{i}"]["ot_flag"]:
                    self.overtake_zones.append(
                        [
                            self.ot_sectors_params[f"Overtaking_sector{i}"]["start"],
                            self.ot_sectors_params[f"Overtaking_sector{i}"]["end"],
                        ]
                    )
        except IndexError as e:
            raise IndexError(
                f"[State Machine] Error in overtaking sector numbers. \nTry switching map with the script in stack_master/scripts and re-source in every terminal. \nError thrown: {e}")

        # Choose the dyn ot param value
        self.ot_begin_margin = params.doubles[2].value
        rospy.logwarn(
            f"[{self.name}] Using OT beginning { self.ot_begin_margin}[m] from param: {params.doubles[2].name}")
        # Spline new OT if they exist already
        self.recompute_ot_spline = True

    def merger_cb(self, data):
        self.merger = data.data

    def force_trailing_cb(self, data):
        self.force_trailing = data.data

    ######################################
    # ATTRIBUTES/CONDITIONS CALCULATIONS #
    ######################################
    """ For consistency, all conditions should be calculated in this section, and should all have the same signature:
    def _check_condition(self) -> bool:
    ...
    """

    def _check_only_ftg_zone(self) -> bool:
        ftg_only = False
        # check if the car is in a ftg only zone, but only if there is an only ftg zone
        if len(self.only_ftg_zones) != 0:
            for sector in self.only_ftg_zones:
                if sector[0] <= self.cur_s / self.waypoints_dist <= sector[1]:
                    ftg_only = True
                    # rospy.logwarn(f"[{self.name}] IN FTG ONLY ZONE")
                    break  # cannot be in two ftg zones
        return ftg_only

    def _check_close_to_raceline(self) -> bool:
        return np.abs(self.cur_d) < self.gb_ego_width_m  # [m]

    def _check_ot_sector(self) -> bool:
        for sector in self.overtake_zones:
            if sector[0] <= self.cur_s / self.waypoints_dist <= sector[1]:
                # rospy.loginfo(f"[{self.name}] In overtaking sector!")
                self.ot_section_check_pub.publish(True)
                return True
        self.ot_section_check_pub.publish(False)
        return False

    def _check_ofree(self) -> bool:
        o_free = True

        # Slightly different for spliner and TAM sampling (both use avoidance waypoints)
        if self.ot_planner == "spliner" or self.ot_planner == "predictive_spliner" or self.ot_planner == "tam_sampling" or self.ot_planner == "predictive_sampler":
            if not self.timetrials_only and self.last_valid_avoidance_wpnts is not None:
                # Horizon in front of cur_s [m]
                horizon = self.overtaking_horizon_m

                # Use frenet conversion service to convert (s, d) wrt min curv trajectory to (x, y) in map
                for obs in self.obstacles:
                    if self.ot_planner == "spliner" or (self.ot_planner == "predictive_spliner" and obs.is_static == True) or self.ot_planner == "tam_sampling" or self.ot_planner == "predictive_sampler":
                        obs_s = obs.s_center
                        # Wrapping madness to check if infront
                        dist_to_obj = (obs_s - self.cur_s) % self.max_s
                        if dist_to_obj < horizon and len(self.last_valid_avoidance_wpnts):
                            obs_d = obs.d_center
                            # Get d wrt to mincurv from the overtaking line
                            avoid_wpnt_idx = np.argmin(
                                np.array([abs(avoid_s.s_m - obs_s)
                                         for avoid_s in self.last_valid_avoidance_wpnts])
                            )
                            ot_d = self.last_valid_avoidance_wpnts[avoid_wpnt_idx].d_m
                            ot_obs_dist = ot_d - obs_d
                            if abs(ot_obs_dist) < self.lateral_width_ot_m:
                                o_free = False
                                rospy.loginfo(
                                    "[State Machine] O_FREE False, obs dist to ot lane: {} m".format(ot_obs_dist))
                                break
            else:
                o_free = True
            return o_free

        # Slightly different for frenet
        elif self.ot_planner == "frenet":
            if not self.timetrials_only and self.overtake_wpnts is not None:
                # Horizon in front of cur_s [m]
                horizon = self.overtaking_horizon_m

                # Use frenet conversion service to convert (s, d) wrt min curv trajectory to (x, y) in map
                for obs in self.obstacles:
                    obs_s = obs.s_center
                    # Wrapping madness to check if infront
                    dist_to_obj = (obs_s - self.cur_s) % self.max_s
                    if dist_to_obj < horizon and len(self.frenet_wpnts.wpnts):
                        obs_d = obs.d_center
                        # Get d wrt to mincurv from the overtaking line
                        avoid_wpnt_idx = np.argmin(
                            np.array([abs(avoid_s.s_m - obs_s)
                                     for avoid_s in self.frenet_wpnts.wpnts])
                        )
                        ot_d = self.frenet_wpnts.wpnts[avoid_wpnt_idx].d_m
                        ot_obs_dist = ot_d - obs_d
                        if abs(ot_obs_dist) < self.lateral_width_ot_m:
                            o_free = False
                            rospy.loginfo(
                                "[State Machine] O_FREE False, obs dist to ot lane: {} m".format(ot_obs_dist))
                            break
            else:
                o_free = True
            return o_free

        else:
            rospy.logerr(f"[{self.name}] Unknown overtake planner")
            raise NotImplementedError

    def _check_gbfree(self) -> bool:
        gb_free = True
        # If we are in time trial only mode -> return free overtake i.e. GB_FREE True
        if not self.timetrials_only:
            horizon = self.gb_horizon_m  # Horizon in front of cur_s [m]

            for obs in self.obstacles:
                obs_s = (obs.s_start + obs.s_end) / 2
                obs_s = obs.s_center
                gap = (obs_s - self.cur_s) % self.track_length
                if gap < horizon:
                    obs_d = obs.d_center
                    # Get d wrt to mincurv from the overtaking line
                    if abs(obs_d) < self.lateral_width_gb_m:
                        gb_free = False
                        rospy.loginfo(
                            f"[{self.name}] GB_FREE False, obs dist to ot lane: {obs_d} m")
                        break
        else:
            gb_free = True

        return gb_free

    def _check_prediction_gbfree(self) -> bool:
        if not self.timetrials_only:
            horizon = 10  # Horizon in front of cur_s [m]

            # Use frenet conversion service to convert (s, d) wrt min curv trajectory to (x, y) in map
            for obs in self.obstacles_prediction:
                obs_s = obs.s_start
                gap = (obs_s - self.cur_s) % self.track_length
                if gap < horizon:
                    return False
        return True

    def _check_availability_graph_wpts(self) -> bool:
        if self.graph_based_wpts is None:
            return False
        elif len(self.graph_based_wpts) == 0:
            return False
        else:
            return True

    def _check_enemy_in_front(self) -> bool:
        # If we are in time trial only mode -> return free overtake i.e. GB_FREE True
        if not self.timetrials_only:
            horizon = self.gb_horizon_m  # Horizon in front of cur_s [m]
            for obs in self.obstacles:
                gap = (obs.s_start - self.cur_s) % self.track_length
                if gap < horizon:
                    return True
            return False

    def _check_availability_splini_wpts(self) -> bool:

        if self.avoidance_wpnts is None:
            # rospy.logwarn_throttle(2.0,
            #                        f"[{self.name}] _check_availability: avoidance_wpnts is None -> returning False")
            return False
        elif len(self.avoidance_wpnts.wpnts) == 0:
            # rospy.logwarn(
            #     f"[{self.name}] _check_availability: avoidance_wpnts has 0 waypoints -> returning False")
            return False

        # TAM sampling planner: Skip hysteresis check (doesn't switch between discrete sides)
        # TAM continuously optimizes trajectories, no need for side-switching debouncing
        # Predictive Sampler also uses TAM's continuous optimization
        if self.ot_planner == "tam_sampling" or self.ot_planner == "predictive_sampler":
            self.last_valid_avoidance_wpnts = self.avoidance_wpnts.wpnts.copy()
            rospy.loginfo_throttle(2.0,
                                   f"[{self.name}] ✓✓✓ TAM: Skipping hysteresis, updating last_valid with {len(self.last_valid_avoidance_wpnts)} waypoints | "
                                   f"s_range=[{self.last_valid_avoidance_wpnts[0].s_m:.2f}, {self.last_valid_avoidance_wpnts[-1].s_m:.2f}]")
            return True

        # Spliner/Predictive Spliner: Apply hysteresis check to prevent side-switching oscillation
        # Say no to the ot line if the last switch was less than 0.75 seconds ago
        elif (
            abs((self.avoidance_wpnts.header.stamp -
                self.avoidance_wpnts.last_switch_time).to_sec())
            < self.splini_hyst_timer_sec
        ):
            rospy.logdebug(
                f"[{self.name}]: Still too fresh into the switch...{abs((self.avoidance_wpnts.last_switch_time - rospy.Time.now()).to_sec())}")
            return False
        else:
            # If the splinis are valid update the last valid ones
            self.last_valid_avoidance_wpnts = self.avoidance_wpnts.wpnts.copy()
            rospy.loginfo_throttle(2.0,
                                   f"[{self.name}] ✓✓✓ _check_availability: UPDATING last_valid_avoidance_wpnts with {len(self.last_valid_avoidance_wpnts)} waypoints | "
                                   f"s_range=[{self.last_valid_avoidance_wpnts[0].s_m:.2f}, {self.last_valid_avoidance_wpnts[-1].s_m:.2f}]")
            return True

    def _check_ftg(self) -> bool:
        # If we have been standing still for 3 seconds inside TRAILING -> FTG
        threshold = self.ftg_timer_sec * self.rate_hz
        if self.ftg_disabled:
            return False
        else:
            if self.cur_state == StateType.TRAILING and self.cur_vs < self.ftg_speed_mps:
                self.ftg_counter += 1
                rospy.logwarn(
                    f"[{self.name}] FTG counter: {self.ftg_counter}/{threshold}")
            else:
                self.ftg_counter = 0

            if self.ftg_counter > threshold:
                return True
            else:
                return False

    def _check_emergency_break(self) -> bool:
        emergency_break = False
        if self.ot_planner == "predictive_spliner" or self.ot_planner == "predictive_sampler":
            if not self.timetrials_only:
                obstacles = self.obstacles_perception.copy()
                if obstacles != []:
                    # Horizon in front of cur_s [m]
                    horizon = self.emergency_break_horizon

                    # Get car name for detailed logging
                    car_name = getattr(self, 'car_name', 'unknown_car')

                    for obs in obstacles:
                        # Only use opponent for emergency break
                        # Wrapping madness to check if infront
                        dist_to_obj = (obs.s_start - self.cur_s) % self.max_s
                        # Check if opponent is closer than emegerncy
                        if dist_to_obj < horizon:

                            # Safety check: ensure local waypoints exist
                            if not self.local_wpnts.wpnts:
                                rospy.logwarn_throttle(
                                    5.0, f"[{car_name}] Emergency brake check skipped: no local waypoints available")
                                continue

                            # Get estimated d from local waypoints
                            local_wpnt_idx = np.argmin(
                                np.array([abs(avoid_s.s_m - obs.s_center)
                                         for avoid_s in self.local_wpnts.wpnts])
                            )
                            ot_d = self.local_wpnts.wpnts[local_wpnt_idx].d_m
                            ot_obs_dist = ot_d - obs.d_center

                            # Enhanced logging for emergency brake debugging
                            rospy.loginfo(f"[{car_name}] State Machine - Obstacle check: "
                                          f"obs_id={obs.id}, s_center={obs.s_center:.2f}m, d_center={obs.d_center:.2f}m, "
                                          f"cur_s={self.cur_s:.2f}m, dist_to_obj={dist_to_obj:.2f}m, "
                                          f"ot_d={ot_d:.2f}m, ot_obs_dist={ot_obs_dist:.2f}m, "
                                          f"static={'YES' if obs.is_static else 'NO'}")

                            if abs(ot_obs_dist) < self.emergency_break_d:
                                emergency_break = True
                                obstacle_type = "STATIC" if obs.is_static else "DYNAMIC"
                                # rospy.logwarn_throttle(1.0, f"[{car_name}] STATE MACHINE EMERGENCY BRAKE TRIGGERED! "
                                #                        f"Obstacle {obs.id} ({obstacle_type}): lateral_distance={abs(ot_obs_dist):.3f}m < threshold={self.emergency_break_d:.3f}m, "
                                #                        f"longitudinal_distance={dist_to_obj:.2f}m < horizon={horizon:.2f}m, {self.cur_state}")
            else:
                emergency_break = False
            return emergency_break

    def _check_on_spline(self) -> bool:
        if self.last_valid_avoidance_wpnts is not None:
            # Check if section goes over end of track
            if self.last_valid_avoidance_wpnts[0].s_m > self.last_valid_avoidance_wpnts[-1].s_m:
                if self.cur_s > self.last_valid_avoidance_wpnts[0].s_m and self.cur_s < self.max_s:
                    return True
                elif self.cur_s < self.last_valid_avoidance_wpnts[-1].s_m and self.cur_s > 0:
                    return True
            else:
                if self.cur_s > self.last_valid_avoidance_wpnts[0].s_m and self.cur_s < self.last_valid_avoidance_wpnts[-1].s_m:
                    return True
        return False

    def _check_on_merger(self) -> bool:
        if self.merger is not None:
            if self.merger[0] < self.merger[1]:
                if self.cur_s > self.merger[0] and self.cur_s < self.merger[1]:
                    return True
            elif self.merger[0] > self.merger[1]:
                if self.cur_s > self.merger[0] or self.cur_s < self.merger[1]:
                    return True
            else:
                return False
        return False

    def _check_force_trailing(self) -> bool:
        return self.force_trailing

    ################
    # HELPER FUNCS #
    ################

    def mincurv_splinification(self):
        coords = np.empty((len(self.glb_wpnts), 4))
        for i, wpnt in enumerate(self.glb_wpnts):
            coords[i, 0] = wpnt.s_m
            coords[i, 1] = wpnt.x_m
            coords[i, 2] = wpnt.y_m
            coords[i, 3] = wpnt.vx_mps

        self.mincurv_spline_x = Spline(coords[:, 0], coords[:, 1])
        self.mincurv_spline_y = Spline(coords[:, 0], coords[:, 2])
        self.mincurv_spline_v = Spline(coords[:, 0], coords[:, 3])
        rospy.loginfo(f"[{self.name}] Splinified Min Curve")

    def ot_splinification(self):
        coords = np.empty((len(self.overtake_wpnts), 5))
        for i, wpnt in enumerate(self.overtake_wpnts):
            coords[i, 0] = wpnt.s_m
            coords[i, 1] = wpnt.x_m
            coords[i, 2] = wpnt.y_m
            coords[i, 3] = wpnt.d_m
            coords[i, 4] = wpnt.vx_mps

        # Sort s_m to start splining at 0
        coords = coords[coords[:, 0].argsort()]
        self.ot_spline_x = Spline(coords[:, 0], coords[:, 1])
        self.ot_spline_y = Spline(coords[:, 0], coords[:, 2])
        self.ot_spline_d = Spline(coords[:, 0], coords[:, 3])
        self.ot_spline_v = Spline(coords[:, 0], coords[:, 4])
        rospy.loginfo(f"[{self.name}] Splinified Overtaking Curve")

    def _find_nearest_ot_s(self) -> float:
        half_search_dim = 5

        # create indices
        idxs = [
            i % self.num_ot_points for i in range(self.cur_id_ot - half_search_dim, self.cur_id_ot + half_search_dim)
        ]
        ses = np.array([self.overtake_wpnts[i].s_m for i in idxs])

        dists = np.abs(self.cur_s - ses)
        chose_id = np.argmin(dists)
        s_ot = idxs[chose_id]
        s_ot %= self.num_ot_points

        return s_ot

    def get_splini_wpts(self) -> WpntArray:
        """Obtain the waypoints by fusing those obtained by spliner with the
        global ones.
        """
        # DEBUG: Check status of avoidance waypoints
        avoidance_status = "None" if self.avoidance_wpnts is None else f"{len(self.avoidance_wpnts.wpnts)} wpnts"
        last_valid_status = "None" if self.last_valid_avoidance_wpnts is None else f"{len(self.last_valid_avoidance_wpnts)} wpnts"

        # rospy.loginfo_throttle(2.0,
        #                        f"[{self.name}] get_splini_wpts() called | "
        #                        f"avoidance_wpnts={avoidance_status}, last_valid={last_valid_status}")

        splini_glob = self.glb_wpnts.copy()

        # Handle wrapping
        if self.last_valid_avoidance_wpnts is not None:
            if self.last_valid_avoidance_wpnts[-1].s_m > self.last_valid_avoidance_wpnts[0].s_m:
                splini_idxs = [
                    s
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m /
                            self.waypoints_dist + 0.5),
                        int(self.last_valid_avoidance_wpnts[-1].s_m /
                            self.waypoints_dist + 0.5),
                    )
                ]
            else:
                splini_idxs = [
                    int(s % (self.max_s / self.waypoints_dist) + 0.5)
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m /
                            self.waypoints_dist + 0.5),
                        int((
                            self.max_s + self.last_valid_avoidance_wpnts[-1].s_m) / self.waypoints_dist + 0.5),
                    )
                ]

            with self.lock:  # to avoid crash when the waypoints are updated but we're looping here
                for i, s in enumerate(splini_idxs):
                    # splini_glob[s] = self.last_valid_avoidance_wpnts[i]
                    splini_glob[s] = self.last_valid_avoidance_wpnts[min(
                        i, len(self.last_valid_avoidance_wpnts) - 1)]

        # If the last valid points have been reset, then we just pass the global waypoints
        else:
            rospy.logwarn(
                f"[{self.name}] ⚠️⚠️⚠️ No valid avoidance waypoints (last_valid_avoidance_wpnts=None), passing global waypoints")
            pass

        return splini_glob

    def get_tam_wpts(self) -> WpntArray:
        """
        Obtain waypoints by fusing TAM planner trajectory with global waypoints.

        Uses hybrid Option A + E approach:
        - Linear interpolation for dynamics (velocity, acceleration, lateral offset)
        - Track-assisted geometry for position/heading (using global waypoints as reference)

        This achieves 100% TAM coverage by filling gaps between TAM waypoints
        with interpolated values at 0.1m spacing.

        Returns:
            WpntArray: Fused waypoint array with interpolated TAM trajectory
        """
        import numpy as np

        tam_glob = self.glb_wpnts.copy()

        # Handle wrapping for TAM waypoints
        if self.avoidance_wpnts is not None and len(self.avoidance_wpnts.wpnts) > 0:
            tam_wpnts = self.avoidance_wpnts.wpnts

            if len(tam_wpnts) < 2:
                # Need at least 2 waypoints for interpolation
                rospy.logwarn_throttle(5.0,
                                       f"[{self.name}] TAM trajectory has only {len(tam_wpnts)} waypoint(s), need at least 2 for interpolation")
                return tam_glob

            # Build interpolated waypoint array
            # For each consecutive pair of TAM waypoints, interpolate at 0.1m spacing

            with self.lock:  # Thread safety
                # Process each consecutive pair of TAM waypoints
                for i in range(len(tam_wpnts) - 1):
                    wpnt_start = tam_wpnts[i]
                    wpnt_end = tam_wpnts[i + 1]

                    s_start = wpnt_start.s_m
                    s_end = wpnt_end.s_m

                    # Handle wraparound case (trajectory crosses s=0)
                    if s_end < s_start:
                        # Wraparound - interpolate from s_start to track_length, then 0 to s_end
                        track_length = self.num_glb_wpnts * self.waypoints_dist

                        # First segment: s_start to track_length
                        s_values_1 = np.arange(
                            s_start, track_length, self.waypoints_dist)
                        # Second segment: 0 to s_end
                        s_values_2 = np.arange(
                            0, s_end + self.waypoints_dist, self.waypoints_dist)
                        s_values = np.concatenate([s_values_1, s_values_2])

                        # Adjust s_end for interpolation calculation
                        s_end_adjusted = s_end + track_length
                    else:
                        # Normal case: no wraparound
                        s_values = np.arange(
                            s_start, s_end + self.waypoints_dist, self.waypoints_dist)
                        s_end_adjusted = s_end

                    # For each interpolation point
                    for s_interp in s_values:
                        # Wrap s_interp to track range
                        s_interp_wrapped = s_interp % (
                            self.num_glb_wpnts * self.waypoints_dist)

                        # Calculate interpolation ratio
                        if s_end_adjusted > s_start:
                            alpha = (s_interp - s_start) / \
                                (s_end_adjusted - s_start)
                        else:
                            alpha = 0.0
                        alpha = np.clip(alpha, 0.0, 1.0)

                        # Create interpolated waypoint
                        wpnt_interp = Wpnt()

                        # Linear interpolation for dynamics
                        wpnt_interp.vx_mps = (
                            1 - alpha) * wpnt_start.vx_mps + alpha * wpnt_end.vx_mps
                        wpnt_interp.ax_mps2 = (
                            1 - alpha) * wpnt_start.ax_mps2 + alpha * wpnt_end.ax_mps2
                        wpnt_interp.d_m = (1 - alpha) * \
                            wpnt_start.d_m + alpha * wpnt_end.d_m
                        wpnt_interp.s_m = s_interp_wrapped

                        # Track-assisted geometry: Use global waypoint at this s-position as reference
                        # This ensures positions follow track geometry accurately
                        idx_global = int(
                            s_interp_wrapped / self.waypoints_dist + 0.5) % self.num_glb_wpnts
                        global_wpnt = self.glb_wpnts[idx_global]

                        # Use global waypoint's position and heading as base
                        # Adjust position based on lateral offset (d_m)
                        psi = global_wpnt.psi_rad

                        # Calculate lateral offset direction (perpendicular to track)
                        dx_lateral = -wpnt_interp.d_m * np.sin(psi)
                        dy_lateral = wpnt_interp.d_m * np.cos(psi)

                        wpnt_interp.x_m = global_wpnt.x_m + dx_lateral
                        wpnt_interp.y_m = global_wpnt.y_m + dy_lateral
                        wpnt_interp.psi_rad = global_wpnt.psi_rad
                        wpnt_interp.kappa_radpm = global_wpnt.kappa_radpm

                        # Track boundaries from global waypoint
                        wpnt_interp.d_left = global_wpnt.d_left
                        wpnt_interp.d_right = global_wpnt.d_right

                        # ID for debugging
                        wpnt_interp.id = idx_global

                        # Insert into fused array
                        tam_glob[idx_global] = wpnt_interp

                # Handle the last waypoint (no interpolation needed, just copy)
                last_wpnt = tam_wpnts[-1]
                idx_last = int(last_wpnt.s_m /
                               self.waypoints_dist + 0.5) % self.num_glb_wpnts
                tam_glob[idx_last] = last_wpnt

            rospy.loginfo_throttle(5.0,
                                   f"[{self.name}] TAM interpolation: Processed {len(tam_wpnts)} TAM waypoints, "
                                   f"interpolated at 0.1m spacing from s={tam_wpnts[0].s_m:.2f} to s={tam_wpnts[-1].s_m:.2f}")

        # If no valid TAM waypoints, just return global waypoints
        else:
            pass

        return tam_glob

    #######
    # VIZ #
    #######

    def _pub_local_wpnts(self, wpts: WpntArray):
        loc_markers = MarkerArray()
        loc_wpnts = wpts
        loc_wpnts.header.stamp = rospy.Time.now()
        loc_wpnts.header.frame_id = "map"

        for i, wpnt in enumerate(loc_wpnts.wpnts):
            mrk = Marker()
            mrk.header.frame_id = "map"
            mrk.type = mrk.SPHERE
            mrk.scale.x = 0.15
            mrk.scale.y = 0.15
            mrk.scale.z = 0.15
            mrk.color.a = 1.0
            mrk.color.g = 1.0

            mrk.id = i
            mrk.pose.position.x = wpnt.x_m
            mrk.pose.position.y = wpnt.y_m
            mrk.pose.position.z = wpnt.vx_mps / \
                self.max_speed  # Visualise speed in z dimension
            mrk.pose.orientation.w = 1
            loc_markers.markers.append(mrk)

        if len(loc_wpnts.wpnts) == 0:
            rospy.logwarn(f"[{self.name}] No local waypoints published...")
        else:
            self.loc_wpnt_pub.publish(loc_wpnts)

        self.vis_loc_wpnt_pub.publish(loc_markers)

    def get_graph_based_wpts(self) -> WpntArray:
        waypoint_arr = WpntArray()
        # Fill waypoint and marker array
        for i, coord in enumerate(self.graph_based_wpts[:self.n_loc_wpnts, :]):
            wpnt = Wpnt()
            wpnt.s_m = (coord[0] + self.cur_s) % self.max_s
            wpnt.x_m = coord[1]
            wpnt.y_m = coord[2]
            wpnt.d_m = 0.0
            wpnt.psi_rad = coord[3]
            wpnt.kappa_radpm = coord[4]
            wpnt.vx_mps = coord[5]  # * self._get_speed_scaler()
            wpnt.ax_mps2 = coord[6]
            wpnt.id = i
            # Get index of closest global waypoint in terms of s-coordinate
            idx = np.abs(self.gb_wpnts_arr[:, 0] - wpnt.s_m).argmin()
            # Get left and right distances to track bounds from the global waypoint
            d_left = self.gb_wpnts_arr[idx, 5]
            d_right = self.gb_wpnts_arr[idx, 4]
            # Use this information together with the d coordinate of the local waypoint in order to calculate
            # left and right track bounds distances of the local waypoint
            wpnt.d_left = d_left - wpnt.d_m
            wpnt.d_right = d_right + wpnt.d_m
            waypoint_arr.wpnts.append(wpnt)

        return waypoint_arr

    def visualize_state(self, state: str):
        """
        Function that visualizes the state of the car by displaying a colored cube in RVIZ.

        Parameters
        ----------
        action
            Current state of the car to be displayed
        """
        if self.first_visualization:
            self.first_visualization = False
            x0 = self.glb_wpnts[0].x_m
            y0 = self.glb_wpnts[0].y_m
            x1 = self.glb_wpnts[1].x_m
            y1 = self.glb_wpnts[1].y_m
            # compute normal vector of 125% length of trackboundary but to the left of the trajectory
            xy_norm = (
                -np.array([y1 - y0, x0 - x1]) / np.linalg.norm([y1 -
                                                                y0, x0 - x1]) * 1.25 * self.glb_wpnts[0].d_left
            )

            self.x_viz = x0 + xy_norm[0]
            self.y_viz = y0 + xy_norm[1]

        mrk = Marker()
        mrk.type = mrk.SPHERE
        mrk.id = 1
        mrk.header.frame_id = "map"
        mrk.header.stamp = rospy.Time.now()
        mrk.color.a = 1.0
        mrk.color.g = 1.0
        mrk.pose.position.x = self.x_viz
        mrk.pose.position.y = self.y_viz
        mrk.pose.position.z = 0
        mrk.pose.orientation.w = 1
        mrk.scale.x = 1
        mrk.scale.y = 1
        mrk.scale.z = 1

        # Set color and log info based on the state of the car
        if state == "GB_TRACK":
            mrk.color.g = 1.0
        elif state == "OVERTAKE":
            mrk.color.r = 1.0
            mrk.color.g = 1.0
            mrk.color.b = 1.0
        elif state == "TRAILING":
            mrk.color.r = 0.0
            mrk.color.g = 0.0
            mrk.color.b = 1.0
        elif state == "FTGONLY":
            mrk.color.r = 1.0
            mrk.color.g = 0.0
            mrk.color.b = 0.0
        self.state_mrk.publish(mrk)

    def publish_not_ready_marker(self):
        """Publishes a text marker that warn the user that the car is not ready to run"""
        mrk = Marker()
        mrk.type = mrk.TEXT_VIEW_FACING
        mrk.id = 1
        mrk.header.frame_id = "map"
        mrk.header.stamp = rospy.Time.now()
        mrk.color.a = 1.0
        mrk.color.r = 1.0
        mrk.color.g = 0.0
        mrk.color.b = 0.0
        mrk.pose.position.x = np.mean(
            [wpnt.x_m for wpnt in self.glb_wpnts]
        )  # publish in the center of the track, to avoid not seeing it
        mrk.pose.position.y = np.mean([wpnt.y_m for wpnt in self.glb_wpnts])
        mrk.pose.position.z = 1.0
        mrk.pose.orientation.w = 1
        mrk.scale.x = 4.69
        mrk.scale.y = 4.69
        mrk.scale.z = 4.69
        mrk.text = "BATTERY TOO LOW!!!"
        self.emergency_pub.publish(mrk)

    #############
    # MAIN LOOP #
    #############
    def loop(self):
        """Main loop of the state machine. It is called at a fixed rate by the
        ROS node.
        """

        # safety check
        if self.cur_volt < self.volt_threshold:
            rospy.logerr_throttle_identical(
                1, f"[{self.name}] VOLTS TOO LOW, STOP THE CAR")

            # publishes a marker that warn the user that the car is not ready to run
            self.publish_not_ready_marker()

        # do state transition (unless we want to force it into GB_TRACK via dynamic reconfigure)
        if self.measuring:
            start = time.perf_counter()
        if not self.force_gbtrack_state:
            self.cur_state = self.state_transitions[self.cur_state](self)
        else:
            self.cur_state = StateType.GB_TRACK
            rospy.logwarn(f"[{self.name}] GBTRACK state forced!!!")
        if self.measuring:
            end = time.perf_counter()
            self.latency_pub.publish(end - start)

        # Detect state transitions to/from OVERTAKE for spliner, predictive_spliner, and predictive_sampler
        if self.cur_state != self.previous_state:
            # Only log for spliner, predictive_spliner, and predictive_sampler (not tam_sampling)
            if self.ot_planner in ["spliner", "predictive_spliner", "predictive_sampler"]:
                if self.cur_state == StateType.OVERTAKE or self.previous_state == StateType.OVERTAKE:
                    transition_msg = f"{self.previous_state.value} -> {self.cur_state.value}"
                    self.state_transition_pub.publish(transition_msg)
                    rospy.loginfo(
                        f"[{self.name}] State transition: {transition_msg}")
            self.previous_state = self.cur_state

        self.state_pub.publish(self.cur_state.value)
        self.visualize_state(state=self.cur_state.value)

        rospy.loginfo_throttle(
            5.0, f"[{self.name}] Current Planer: {self.ot_planner}")

        # # Debug logging for Predictive Spliner Trailing Transition conditions
        # if self.ot_planner == "predictive_spliner":
        #     ot_sector = self._check_ot_sector()
        #     valid_spline = self._check_availability_splini_wpts()
        #     emergency_break = self._check_emergency_break()
        #     enemy_in_front = self._check_enemy_in_front()
        #     gb_free = self._check_gbfree()
        #     gb_predict_free = self._check_prediction_gbfree()
        #     o_free = self._check_ofree()
        #     on_avoidance_spline = self._check_on_spline()
        #     on_merger = self._check_on_merger()
        #     force_trailing = self._check_force_trailing()

        #     rospy.logwarn_throttle(
        #         2.0, f"[{self.name}] Trailing Transition Check: ot_sector={ot_sector}, valid_spline={valid_spline}, emergency_break={emergency_break}, enemy_in_front={enemy_in_front}, gb_free={gb_free}, gb_predict_free={gb_predict_free}, o_free={o_free}, on_avoidance_spline={on_avoidance_spline}, on_merger={on_merger}, force_trailing={force_trailing}")
        #     path1_conditions = valid_spline and not emergency_break and o_free and ot_sector and not on_merger
        #     # path2_conditions = not enemy_in_front and on_avoidance_spline and not on_merger

        #     if path1_conditions:
        #         rospy.logwarn_throttle(
        #             2.0, f"[{self.name}] OVERTAKE Path 1 READY: All conditions met for primary overtaking transition!")
        #     # elif path2_conditions:
        #     #     rospy.logwarn_throttle(
        #     #         2.0, f"[{self.name}] OVERTAKE Path 2 READY: All conditions met for alternative overtaking transition!")
        #     else:
        #         # Analyze why transitions are failing
        #         path1_failures = []
        #         # path2_failures = []

        #         # Path 1 analysis: valid_spline and not emergency_break and o_free and ot_sector and not on_merger
        #         if not valid_spline:
        #             path1_failures.append("valid_spline=False")
        #         if emergency_break:
        #             path1_failures.append("emergency_break=True")
        #         if not o_free:
        #             path1_failures.append("o_free=False")
        #         if not ot_sector:
        #             path1_failures.append("ot_sector=False")
        #         if on_merger:
        #             path1_failures.append("on_merger=True")

        #         # # Path 2 analysis: not enemy_in_front and on_avoidance_spline and not on_merger
        #         # if enemy_in_front:
        #         #     path2_failures.append("enemy_in_front=True")
        #         # if not on_avoidance_spline:
        #         #     path2_failures.append("on_avoidance_spline=False")
        #         # if on_merger:
        #         #     path2_failures.append("on_merger=True")

        #         # Report the blocking conditions
        #         path1_reason = ", ".join(
        #             path1_failures) if path1_failures else "ALL_CONDITIONS_MET"
        #         # path2_reason = ", ".join(
        #         #     path2_failures) if path2_failures else "ALL_CONDITIONS_MET"

        #         rospy.logwarn_throttle(
        #             2.0, f"[{self.name}] OVERTAKE BLOCKED - Path1 blocked by: [{path1_reason}]")

        #         avoidance_status = "None" if self.avoidance_wpnts is None else f"{len(self.avoidance_wpnts.wpnts)} waypoints"
        #         obstacles_count = len(self.obstacles)
        #         obstacles_perception_count = len(self.obstacles_perception)
        #         obstacles_prediction_count = len(self.obstacles_prediction)

        #         rospy.logwarn_throttle(
        #             2.0, f"[{self.name}] SQP Debug: avoidance_wpnts={avoidance_status}, "
        #             f"obstacles_total={obstacles_count}, perception={obstacles_perception_count}, prediction={obstacles_prediction_count}, "
        #             f"cur_s={self.cur_s:.2f}, cur_d={self.cur_d:.2f}")

        #         # Check if SQP planner is receiving the right inputs
        #         if hasattr(self, 'last_collision_prediction_time'):
        #             time_since_collision = (
        #                 rospy.Time.now() - self.last_collision_prediction_time).to_sec()
        #             rospy.logwarn_throttle(
        #                 2.0, f"[{self.name}] Time since last collision prediction: {time_since_collision:.2f}s")

        if self.timetrials_only:
            rospy.logdebug_throttle_identical(
                1, f"[{self.name}] Switched to state {self.cur_state} in time trials only mode"
            )
        else:
            rospy.logdebug_throttle_identical(
                1, f"[{self.name}] Switched to state {self.cur_state} in OT mode: {self.ot_planner}"
            )

        # decrease splini ttl counter used to cache the splini waypoints, once 0 it gets overwritten in case of empty avoidance
        if self.ot_planner == "spliner":
            self.splini_ttl_counter -= 1
            # Once ttl has reached 0 we overwrite the avoidance waypoints with the empty waypoints
            if self.splini_ttl_counter <= 0:
                self.last_valid_avoidance_wpnts = None
                self.avoidance_wpnts = WpntArray()
                self.splini_ttl_counter = -1
        elif self.ot_planner in ["predictive_spliner", "predictive_sampler"]:
            self.splini_ttl_counter -= 1
            # Once ttl has reached 0 we overwrite the avoidance waypoints with the empty waypoints
            if self.splini_ttl_counter <= 0:
                if not self._check_on_spline():
                    self.avoidance_wpnts = None
                    self.last_valid_avoidance_wpnts = None
                elif self.splini_ttl_counter <= -int(self.splini_ttl * self.rate_hz)*3:
                    self.avoidance_wpnts = None
                    self.splini_ttl_counter = -10
        else:
            pass

        # get the proper local waypoints based on the new state
        self.local_wpnts.wpnts = self.states[self.cur_state](self)

        # Publish waypoints after source is updated
        self._pub_local_wpnts(self.local_wpnts)

        # Clear FTG counter if not in TRAILING state
        if self.cur_state != StateType.TRAILING:
            self.ftg_counter = 0


if __name__ == "__main__":
    name = "state_machine"
    rospy.init_node(name, anonymous=False, log_level=rospy.WARN)

    # Get car name from namespace for multi-car identification
    namespace = rospy.get_namespace()
    if namespace != "/":
        # Extract car name from namespace (e.g., "/car1/" -> "car1")
        car_name = namespace.strip("/")
        display_name = f"[{car_name}] State Machine"
    else:
        # Single car setup
        car_name = "state_machine"
        display_name = "State Machine"

    # init and run state machine
    state_machine = StateMachine(display_name)
    state_machine.car_name = car_name  # Store car name for emergency brake logging
    rospy.on_shutdown(state_machine.on_shutdown)
    loop_rate = rospy.Rate(state_machine.rate_hz)
    while not rospy.is_shutdown():
        state_machine.loop()
        loop_rate.sleep()
