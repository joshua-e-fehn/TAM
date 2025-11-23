#!/usr/bin/env python3
"""
TAM Custom Predictor Node

This node implements a simple constant offset prediction for opponent vehicles.
The basic principle: the current lateral offset of the opponent to the raceline 
will remain constant in the future. 

If an opponent is currently driving 2 meters left of the raceline, 
it's predicted to continue driving 2 meters left throughout the entire future trajectory.

Includes boundary conditions and track safety constraints.
"""

import rospy
import numpy as np
from scipy.interpolate import interp1d
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray, OpponentTrajectory, OppWpnt
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from frenet_converter.frenet_converter import FrenetConverter


class TAMConstantOffsetPredictor:
    """
    Simple constant offset predictor for TAM sampling planner

    Subscribes to:
        - /perception/obstacles: Current obstacle positions
        - /global_waypoints: Track centerline for Frenet conversion
        - /car_state/odom_frenet: Ego vehicle state (for prediction horizon)

    Publishes to:
        - /prediction/opponent_waypoints: Predicted opponent trajectory (OpponentTrajectory)
        - /prediction/opponent_markers: Visualization markers (MarkerArray)
    """

    def _load_yaml_defaults(self):
        """Load default parameters from tam_sampling_params.yaml"""
        import rospkg
        import yaml
        import os
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('tam_sampling_planner')
            config_file = os.path.join(
                pkg_path, 'config', 'tam_sampling_params.yaml')

            with open(config_file, 'r') as f:
                yaml_params = yaml.safe_load(f)
                return yaml_params if yaml_params else {}
        except Exception as e:
            rospy.logwarn(
                f"TAMConstantOffsetPredictor: Could not load YAML defaults: {e}")
            return {}

    def __init__(self):
        """Initialize the TAM constant offset predictor"""

        # Initialize ROS node
        rospy.init_node('tam_constant_offset_predictor')

        # Get namespace for logging
        self.car_namespace = rospy.get_namespace().strip('/')
        if self.car_namespace:
            self.log_name = f"[{self.car_namespace}] TAM Predictor"
        else:
            self.log_name = "[TAM Predictor]"

        # State variables
        self.global_waypoints = WpntArray()
        self.obstacles = ObstacleArray()
        self.ego_state = {'s': 0.0, 'd': 0.0, 'vs': 0.0, 'vd': 0.0}
        self.track_length = 0.0

        # Frenet converter
        self.converter = None
        self.track_centerline = None
        self.track_boundaries = {'left': [], 'right': []}

        # Caching and buffering for smooth predictions
        self.last_prediction_time = 0.0
        self.cached_predictions = {}
        self.prediction_buffer = {}  # Store recent predictions with timestamps

        # Velocity scale smoothing (exponential moving average)
        self.velocity_scale_history = {}  # obstacle_id -> smoothed scale factor

        # Tracking for existing markers
        self.active_marker_ids = set()

        # Declare and load parameters
        self.initialized_params = False
        self.race_started = False  # Track race state for parameter update optimization
        self.declare_and_update_parameters()

        # State machine tracking
        self.state_machine_state = "UNKNOWN"

        # Setup ROS interface
        self.setup_ros_interface()

        rospy.loginfo(
            f"{self.log_name} Initialized - predictions use raceline velocities and relative lateral offset")

    def declare_and_update_parameters(self, skip_update=False):
        if skip_update:
            return

        if not self.initialized_params:
            # Load YAML defaults
            yaml_defaults = self._load_yaml_defaults()

            self.prediction_dt = yaml_defaults.get(
                'prediction_dt', rospy.get_param("prediction_dt", 0.1))  # time step
            rospy.set_param("prediction_dt", self.prediction_dt)

            # Prediction horizon parameters (match TAM planner)
            self.prediction_horizon = yaml_defaults.get(
                'planning_horizon', rospy.get_param("prediction_horizon", 4.0))  # seconds
            rospy.set_param("prediction_horizon", self.prediction_horizon)

            self.prediction_distance = yaml_defaults.get(
                'trajectory_length', rospy.get_param("prediction_distance", 10.0))  # meters
            rospy.set_param("prediction_distance", self.prediction_distance)

            # Safety parameters
            self.safety_margin = yaml_defaults.get(
                'safety_margin_dynamic', rospy.get_param("safety_margin", 0.5))
            rospy.set_param("safety_margin", self.safety_margin)
            self.smoothing_factor = yaml_defaults.get(
                'smoothing_factor', rospy.get_param("smoothing_factor", 0.8))
            rospy.set_param("smoothing_factor", self.smoothing_factor)

            self.max_buffer_age = yaml_defaults.get(
                'prediction_buffer_age', rospy.get_param("prediction_buffer_age", 1.0))
            rospy.set_param("prediction_buffer_age", self.max_buffer_age)
            self.marker_lifetime = yaml_defaults.get(
                'marker_lifetime', rospy.get_param("marker_lifetime", 0.8))
            rospy.set_param("marker_lifetime", self.marker_lifetime)
            self.initialized_params = True
        else:
            self.prediction_dt = rospy.get_param(
                "prediction_dt", self.prediction_dt)  # time step

            # Prediction horizon parameters (match TAM planner)
            self.prediction_horizon = rospy.get_param(
                "prediction_horizon", self.prediction_horizon)
            self.prediction_distance = rospy.get_param(
                "prediction_distance", self.prediction_distance)

            # Safety parameters
            self.safety_margin = rospy.get_param(
                "safety_margin", self.safety_margin)
            self.smoothing_factor = rospy.get_param(
                "smoothing_factor", self.smoothing_factor)

            self.max_buffer_age = rospy.get_param(
                "prediction_buffer_age", self.max_buffer_age)
            self.marker_lifetime = rospy.get_param(
                "marker_lifetime", self.marker_lifetime)

    def setup_ros_interface(self):
        """Setup ROS subscribers and publishers"""

        # Subscribers
        rospy.Subscriber("perception/multi_car_obstacles", ObstacleArray,
                         self.obstacles_callback, queue_size=1)
        rospy.Subscriber("global_waypoints", WpntArray,
                         self.global_waypoints_callback, queue_size=1)
        rospy.Subscriber("car_state/odom_frenet", Odometry,
                         self.ego_state_callback, queue_size=1)
        # State machine subscriber for race start detection
        self.state_machine_sub = rospy.Subscriber(
            "state_machine", String, self.state_machine_callback, queue_size=1)

        # Publishers - publish to relative namespace topics
        self.waypoints_pub = rospy.Publisher(
            "prediction/opponent_waypoints", OpponentTrajectory, queue_size=1)
        self.markers_pub = rospy.Publisher(
            "prediction/opponent_markerarray", MarkerArray, queue_size=1)

    def state_machine_callback(self, msg):
        """Callback for state machine state - used to coordinate race start"""
        prev_state = self.state_machine_state
        self.state_machine_state = msg.data

        # Detect race start transition (READY -> any other state)
        if prev_state == "READY" and msg.data != "READY" and not self.race_started:
            self.race_started = True
            rospy.loginfo(
                f"{self.log_name} 🏁 Race started! Disabling parameter updates for performance.")

    def global_waypoints_callback(self, msg: WpntArray):
        """Process global waypoints and setup Frenet converter"""
        self.global_waypoints = msg

        if len(msg.wpnts) > 0:
            # Extract track centerline
            self.track_centerline = np.array(
                [[wpnt.x_m, wpnt.y_m] for wpnt in msg.wpnts])
            self.track_length = msg.wpnts[-1].s_m

            # Extract track boundaries
            self.track_boundaries['left'] = [wpnt.d_left for wpnt in msg.wpnts]
            self.track_boundaries['right'] = [
                wpnt.d_right for wpnt in msg.wpnts]

            # Initialize Frenet converter
            if self.converter is None:
                try:
                    self.converter = FrenetConverter(
                        self.track_centerline[:, 0],
                        self.track_centerline[:, 1]
                    )
                    # rospy.loginfo(
                    #     f"{self.log_name} Frenet converter initialized")
                except Exception as e:
                    rospy.logwarn(
                        f"{self.log_name} Failed to initialize Frenet converter: {e}")

    def obstacles_callback(self, msg: ObstacleArray):
        """Process detected obstacles and generate predictions"""
        self.obstacles = msg
        current_time = rospy.Time.now()

        # rospy.loginfo_throttle(
        #     5.0, f"{self.log_name} Received {len(msg.obstacles)} obstacles")

        # Store current obstacle data in buffer
        for i, obstacle in enumerate(msg.obstacles):
            if not obstacle.is_static:
                self.prediction_buffer[i] = {
                    'obstacle': obstacle,
                    'timestamp': current_time,
                    'obstacle_id': i
                }

        # Clean old entries from buffer
        self.clean_prediction_buffer(current_time)

        # Only generate predictions if we have all required data
        if (len(self.global_waypoints.wpnts) == 0 or
            self.converter is None or
                len(msg.obstacles) == 0):
            # rospy.loginfo_throttle(
            #     5.0, f"{self.log_name} Not ready for predictions: waypoints={len(self.global_waypoints.wpnts)}, converter={self.converter is not None}, obstacles={len(msg.obstacles)}")
            return

        # Generate and publish predictions
        # rospy.loginfo_throttle(
        #     5.0, f"{self.log_name} Generating predictions for {len(msg.obstacles)} obstacles")
        self.generate_predictions()

    def ego_state_callback(self, msg: Odometry):
        """Process ego vehicle state"""
        self.ego_state = {
            's': msg.pose.pose.position.x,
            'd': msg.pose.pose.position.y,
            'vs': msg.twist.twist.linear.x,
            'vd': msg.twist.twist.linear.y
        }

    def clean_prediction_buffer(self, current_time):
        """Remove old entries from prediction buffer"""
        expired_keys = []
        for key, entry in self.prediction_buffer.items():
            age = (current_time - entry['timestamp']).to_sec()
            if age > self.max_buffer_age:
                expired_keys.append(key)

        for key in expired_keys:
            del self.prediction_buffer[key]

    def get_active_obstacles(self, current_time):
        """Get all active obstacles including buffered ones"""
        active_obstacles = []

        # Get current obstacles FIRST (they take priority)
        dynamic_obstacles = [
            (i, obs) for i, obs in enumerate(self.obstacles.obstacles)
            if not obs.is_static
        ]

        # # Log current obstacle data
        # for i, obs in dynamic_obstacles:
        #     rospy.logdebug_throttle(
        #         5.0, f"{self.log_name} Current obstacle {i}: s={obs.s_center:.2f}, d={obs.d_center:.2f}, vs={obs.vs:.2f}")

        # Create set of current obstacle IDs for quick lookup
        current_ids = {i for i, _ in dynamic_obstacles}

        # Add any buffered obstacles that are NOT in current obstacles
        for key, entry in self.prediction_buffer.items():
            age = (current_time - entry['timestamp']).to_sec()
            if age <= self.max_buffer_age and entry['obstacle_id'] not in current_ids:
                # Use buffered obstacle with time extrapolation
                extrapolated_obs = self.extrapolate_obstacle(entry, age)
                if extrapolated_obs:
                    active_obstacles.append(
                        (entry['obstacle_id'], extrapolated_obs))
                    # rospy.logdebug_throttle(
                    #     5.0, f"{self.log_name} Using buffered obstacle {entry['obstacle_id']} (age: {age:.2f}s)")

        # Add current obstacles (these take priority)
        active_obstacles.extend(dynamic_obstacles)
        return active_obstacles

    def extrapolate_obstacle(self, buffer_entry, age):
        """Extrapolate obstacle position based on last known state"""
        obstacle = buffer_entry['obstacle']

        # Simple linear extrapolation based on velocity
        vs = obstacle.vs if obstacle.vs > 0.1 else 2.0  # Minimum velocity assumption

        # Create new obstacle with extrapolated position
        extrapolated = Obstacle()
        extrapolated.s_center = obstacle.s_center + vs * age
        extrapolated.d_center = obstacle.d_center  # Constant lateral offset
        extrapolated.vs = vs
        extrapolated.is_static = obstacle.is_static

        return extrapolated

    def generate_predictions(self):
        """Generate constant offset predictions for all dynamic obstacles"""

        self.declare_and_update_parameters(skip_update=self.race_started)

        current_time = rospy.Time.now()

        # Get active obstacles (current + buffered with extrapolation)
        active_obstacles = self.get_active_obstacles(current_time)

        if len(active_obstacles) == 0:
            # Publish empty markers to clear old ones
            self.publish_empty_markers(current_time)
            return

        # Generate predictions for each active obstacle
        all_predictions = []
        markers = MarkerArray()
        current_marker_ids = set()

        for obstacle_id, obstacle in active_obstacles:
            try:
                prediction = self.predict_obstacle_trajectory(
                    obstacle, obstacle_id)
                if prediction is not None:
                    all_predictions.extend(prediction)

                    # Create visualization marker
                    marker = self.create_prediction_marker(
                        prediction, obstacle_id, current_time)
                    if marker:
                        markers.markers.append(marker)
                        current_marker_ids.add(obstacle_id)

            except Exception as e:
                rospy.logwarn_throttle(
                    5.0, f"{self.log_name} Failed to predict obstacle {obstacle_id}: {e}")

        # Add deletion markers for obstacles that are no longer active
        for old_id in self.active_marker_ids - current_marker_ids:
            delete_marker = Marker()
            delete_marker.header.frame_id = "map"
            delete_marker.header.stamp = current_time
            delete_marker.ns = f"{self.car_namespace}_tam_prediction" if self.car_namespace else "tam_prediction"
            delete_marker.id = old_id
            delete_marker.action = Marker.DELETE
            markers.markers.append(delete_marker)

        # Update active marker IDs
        self.active_marker_ids = current_marker_ids

        # Publish combined predictions
        if all_predictions and not rospy.is_shutdown():
            try:
                prediction_msg = self.create_prediction_message(
                    all_predictions, current_time)
                self.waypoints_pub.publish(prediction_msg)
            except rospy.ROSException as e:
                rospy.logwarn_throttle(
                    5.0, f"{self.log_name} Failed to publish predictions: {e}")

        # Publish visualization markers
        if markers.markers and not rospy.is_shutdown():
            try:
                self.markers_pub.publish(markers)
                self.last_prediction_time = current_time.to_sec()
            except rospy.ROSException as e:
                rospy.logwarn_throttle(
                    5.0, f"{self.log_name} Failed to publish markers: {e}")

    def publish_empty_markers(self, current_time):
        """Publish deletion markers when no obstacles are present"""
        if not self.active_marker_ids:
            return

        empty_markers = MarkerArray()
        for marker_id in self.active_marker_ids:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = current_time
            marker.ns = f"{self.car_namespace}_tam_prediction" if self.car_namespace else "tam_prediction"
            marker.id = marker_id
            marker.action = Marker.DELETE
            empty_markers.markers.append(marker)

        if not rospy.is_shutdown():
            try:
                self.markers_pub.publish(empty_markers)
            except rospy.ROSException:
                pass

        self.active_marker_ids.clear()

    def predict_obstacle_trajectory(self, obstacle: Obstacle, obstacle_id: int):
        """
        Generate relative offset prediction for a single obstacle

        The prediction follows the global waypoints track and applies the obstacle's 
        current RELATIVE lateral position to each waypoint position along the track.

        If the obstacle is currently 20% to the right of the track center, it will
        maintain that 20% relative position throughout the prediction, adapting to
        varying track widths.

        Key improvements:
        1. Uses raceline velocity at each waypoint (not constant)
        2. Maintains relative lateral offset (scales with track width)
        3. Starts from opponent's ACTUAL position (not full lap)
        4. Trims to reasonable horizon (time or distance based)

        Args:
            obstacle: Current obstacle state
            obstacle_id: Unique identifier for the obstacle

        Returns:
            List of predicted waypoints (OppWpnt)
        """

        try:
            # Obstacles are already in Frenet coordinates
            s_current = obstacle.s_center
            d_current = obstacle.d_center

            # Calculate the relative position at the obstacle's current location
            # This is the percentage position between track boundaries
            relative_position = self.calculate_relative_position(obstacle)

            # Calculate relative velocity scale factor with smoothing
            # If opponent drives at 60% of raceline speed, maintain 60% in prediction
            velocity_scale = self.calculate_velocity_scale_factor(
                obstacle, obstacle_id)

            # rospy.loginfo_throttle(
            #     2.0, f"{self.log_name} Obstacle {obstacle_id}: s={s_current:.2f}, d={d_current:.2f}, "
            #          f"relative_pos={relative_position:.2%}, vel_scale={velocity_scale:.2%}")

            # Find starting waypoint index (closest to opponent's current position)
            start_idx = self.find_closest_waypoint_index(s_current)

            # Generate prediction points starting from opponent's actual position
            predicted_waypoints = []
            num_waypoints = len(self.global_waypoints.wpnts)

            # Track accumulated distance and time for horizon limits
            distance_accumulated = 0.0
            time_accumulated = 0.0

            # Iterate forward from opponent's position with wraparound
            for i in range(num_waypoints):
                # Wraparound index
                idx = (start_idx + i) % num_waypoints
                waypoint = self.global_waypoints.wpnts[idx]

                # Calculate distance increment
                if i > 0:
                    prev_idx = (start_idx + i - 1) % num_waypoints
                    prev_waypoint = self.global_waypoints.wpnts[prev_idx]
                    ds = waypoint.s_m - prev_waypoint.s_m

                    # Handle wraparound at track start/finish
                    if ds < -self.track_length / 2.0:
                        ds += self.track_length
                    elif ds > self.track_length / 2.0:
                        ds -= self.track_length

                    distance_accumulated += abs(ds)

                    # Use raceline velocity for time calculation
                    v_raceline = waypoint.vx_mps if waypoint.vx_mps > 0.5 else 2.0
                    time_accumulated += abs(ds) / v_raceline

                # Check horizon limits (use 2× for safety margin)
                if distance_accumulated > 2.0 * self.prediction_distance:
                    break
                if time_accumulated > 2.0 * self.prediction_horizon:
                    break

                # Calculate future d offset based on relative position and local track width
                d_future = self.apply_relative_position_at_waypoint(
                    waypoint, relative_position)

                # Convert s and d to Cartesian coordinates using Frenet converter
                try:
                    x_future, y_future = self.converter.get_cartesian(
                        [waypoint.s_m], [d_future])
                    x_future = x_future[0]
                    y_future = y_future[0]
                except Exception as e:
                    rospy.logwarn_throttle(
                        10.0, f"{self.log_name} Error converting Frenet to Cartesian: {e}")
                    # Fallback: use centerline position with offset
                    x_future = waypoint.x_m
                    y_future = waypoint.y_m

                # Use raceline velocity at this waypoint with opponent's velocity scale
                v_raceline_base = waypoint.vx_mps if waypoint.vx_mps > 0.5 else 2.0
                # Scale by opponent's driving style
                v_predicted = v_raceline_base * velocity_scale

                # Create predicted waypoint
                pred_waypoint = OppWpnt()
                pred_waypoint.x_m = x_future
                pred_waypoint.y_m = y_future
                pred_waypoint.s_m = waypoint.s_m
                pred_waypoint.d_m = d_future  # Use calculated offset (FIX #2)
                pred_waypoint.proj_vs_mps = v_predicted  # Scaled raceline velocity
                pred_waypoint.vd_mps = 0.0  # Constant relative offset = zero lateral velocity

                predicted_waypoints.append(pred_waypoint)

            # rospy.loginfo_throttle(
            #     1.0, f"{self.log_name} Generated {len(predicted_waypoints)} prediction waypoints "
            #          f"(relative_pos={relative_position:.2%}, distance={distance_accumulated:.1f}m, time={time_accumulated:.1f}s)")
            return predicted_waypoints

        except Exception as e:
            rospy.logwarn_throttle(
                10.0, f"{self.log_name} Error predicting trajectory for obstacle {obstacle_id}: {e}")
            return None

    def calculate_relative_position(self, obstacle: Obstacle) -> float:
        """
        Calculate the relative position of the obstacle within the track boundaries.

        Returns a value between -1.0 (at right boundary) and +1.0 (at left boundary),
        where 0.0 is the centerline.

        Args:
            obstacle: Current obstacle state with s_center and d_center

        Returns:
            float: Relative position (-1.0 to +1.0)
        """
        try:
            # Find the closest waypoint to get track boundaries
            waypoint_idx = self.find_closest_waypoint_index(obstacle.s_center)
            waypoint = self.global_waypoints.wpnts[waypoint_idx]

            d_left = waypoint.d_left
            d_right = waypoint.d_right
            d_current = obstacle.d_center

            # Calculate total track width (d_right is typically negative)
            track_width = d_left + abs(d_right)

            if track_width < 0.1:  # Avoid division by zero
                return 0.0

            # Calculate relative position
            # If d_current = 0: position = 0.0 (centerline)
            # If d_current = d_left: position = +1.0 (left boundary)
            # If d_current = d_right: position = -1.0 (right boundary)

            if d_current >= 0:
                # Obstacle is on the left side
                relative_position = d_current / d_left if d_left > 0.01 else 0.0
            else:
                # Obstacle is on the right side
                relative_position = d_current / \
                    abs(d_right) if abs(d_right) > 0.01 else 0.0

            # Clamp to [-1, 1] range
            relative_position = max(-1.0, min(1.0, relative_position))

            # rospy.logerr(
            #     f"{self.log_name} Relative position: {relative_position:.2%} (d={d_current:.2f}, left={d_left:.2f}, right={d_right:.2f}, width={track_width:.2f})")

            return relative_position

        except Exception as e:
            rospy.logwarn_throttle(
                10.0, f"{self.log_name} Error calculating relative position: {e}")
            return 0.0  # Default to centerline

    def calculate_velocity_scale_factor(self, obstacle: Obstacle, obstacle_id: int) -> float:
        """
        Calculate the velocity scale factor for the obstacle relative to raceline.

        If opponent is driving at 60% of raceline speed at current position,
        return 0.6 so predictions maintain that conservative driving style.

        Uses exponential moving average for smoothing to avoid jittery predictions
        from noisy velocity measurements.

        Args:
            obstacle: Current obstacle state with s_center and vs
            obstacle_id: Unique obstacle identifier for tracking history

        Returns:
            float: Velocity scale factor (0.4 to 1.2)
                   - 1.0 = matching raceline speed
                   - 0.6 = driving at 60% of raceline (conservative)
                   - 1.1 = driving 10% faster than raceline (aggressive)
        """
        try:
            # Get opponent's current velocity
            v_opponent = obstacle.vs
            if v_opponent < 0.1:  # Avoid division by zero
                v_opponent = 1.0

            # Find raceline velocity at opponent's current position
            waypoint_idx = self.find_closest_waypoint_index(obstacle.s_center)
            waypoint = self.global_waypoints.wpnts[waypoint_idx]
            v_raceline = waypoint.vx_mps if waypoint.vx_mps > 0.5 else 2.0

            # Calculate raw scale factor
            velocity_scale_raw = v_opponent / v_raceline

            # Apply reasonable bounds:
            # - Lower bound: 40% (prevents unrealistically slow predictions)
            # - Upper bound: 120% (prevents unrealistic speeding)
            velocity_scale_raw = max(0.4, min(1.2, velocity_scale_raw))

            # Smooth using exponential moving average (EMA)
            # alpha = 0.3 means: 30% new value, 70% old value
            # This provides good responsiveness while filtering noise
            alpha = 0.3

            if obstacle_id in self.velocity_scale_history:
                # Smooth with previous value
                velocity_scale_smoothed = (alpha * velocity_scale_raw +
                                           (1 - alpha) * self.velocity_scale_history[obstacle_id])
            else:
                # First observation - no smoothing needed
                velocity_scale_smoothed = velocity_scale_raw

            # Store for next iteration
            self.velocity_scale_history[obstacle_id] = velocity_scale_smoothed

            return velocity_scale_smoothed

        except Exception as e:
            rospy.logwarn_throttle(
                10.0, f"{self.log_name} Error calculating velocity scale: {e}")
            return 0.8  # Default to 80% of raceline (conservative fallback)

    def apply_relative_position_at_waypoint(self, waypoint, relative_position: float) -> float:
        """
        Apply the relative position to a specific waypoint, respecting track boundaries.

        Args:
            waypoint: The global waypoint containing boundary information
            relative_position: Relative position from -1.0 (right boundary) to +1.0 (left boundary)

        Returns:
            float: Absolute d offset at this waypoint
        """
        try:
            # Get track boundaries at this waypoint
            d_left = waypoint.d_left
            d_right = waypoint.d_right

            # Apply safety margins
            d_left_safe = d_left - self.safety_margin
            # d_right is positive
            d_right_safe = d_right - self.safety_margin

            # Calculate the absolute d offset based on relative position
            if relative_position >= 0:
                # Obstacle is on the left side
                d_offset = relative_position * d_left_safe
            else:
                # Obstacle is on the right side (relative_position is negative)
                d_offset = relative_position * d_right_safe

            # Final safety check: ensure we're within bounds
            d_offset = max(-d_right_safe, min(d_left_safe, d_offset))

            return d_offset

        except Exception as e:
            rospy.logwarn_throttle(
                10.0, f"{self.log_name} Error applying relative position at waypoint: {e}")
            return 0.0  # Default to centerline

    def apply_boundary_constraints_at_waypoint(self, waypoint, d_offset):
        """
        Apply track boundary constraints at a specific waypoint (LEGACY - for absolute offset)

        Args:
            waypoint: The global waypoint containing boundary information
            d_offset: Desired lateral offset

        Returns:
            Constrained lateral offset
        """

        try:
            # Get track boundaries at this waypoint
            d_left = waypoint.d_left
            d_right = waypoint.d_right

            # rospy.loginfo_throttle(
            #     10.0, f"[TAM Predictor] Current d values: {d_left} and {d_right}")

            # Apply safety margins
            d_left_safe = d_left - self.safety_margin
            d_right_safe = d_right - self.safety_margin

            if d_offset < -d_right_safe:
                return - d_right_safe
            if d_offset > d_left_safe:
                return d_left_safe

            return d_offset

        except Exception as e:
            rospy.logwarn_throttle(
                10.0, f"{self.log_name} Error applying boundary constraints at waypoint: {e}")
            return d_offset  # Return original if constraint fails

    def apply_boundary_constraints(self, s_pos, d_offset):
        """
        Apply track boundary constraints with smooth transitions (legacy method)

        Args:
            s_pos: Current s position along track
            d_offset: Desired lateral offset

        Returns:
            Constrained lateral offset
        """

        try:
            # Find closest waypoint index for boundary lookup
            waypoint_idx = self.find_closest_waypoint_index(s_pos)

            # Get track boundaries at this position
            d_left = self.track_boundaries['left'][waypoint_idx]
            d_right = self.track_boundaries['right'][waypoint_idx]

            # Apply safety margins
            d_left_safe = d_left - self.safety_margin
            d_right_safe = d_right + self.safety_margin  # d_right is typically negative

            # Constrain with smooth transitions
            if d_offset > d_left_safe:
                # Smooth transition when hitting left boundary
                d_offset = d_left_safe
            elif d_offset < d_right_safe:
                # Smooth transition when hitting right boundary
                d_offset = d_right_safe

            return d_offset

        except Exception as e:
            rospy.logwarn_throttle(
                10.0, f"{self.log_name} Error applying boundary constraints: {e}")
            return d_offset  # Return original if constraint fails

    def find_closest_waypoint_index(self, s_pos):
        """Find the closest waypoint index for a given s position"""

        if len(self.global_waypoints.wpnts) == 0:
            return 0

        # Simple linear search (could be optimized with binary search)
        s_coords = [wpnt.s_m for wpnt in self.global_waypoints.wpnts]
        differences = [abs(s - s_pos) for s in s_coords]
        return differences.index(min(differences))

    def create_prediction_message(self, all_predictions, timestamp):
        """Create OpponentTrajectory message from predictions"""

        msg = OpponentTrajectory()
        msg.header.stamp = timestamp
        msg.header.frame_id = "map"
        msg.lap_count = 0  # Not applicable for this simple predictor
        msg.oppwpnts = all_predictions

        return msg

    def create_prediction_marker(self, prediction_waypoints, obstacle_id, timestamp):
        """Create visualization marker for predicted trajectory"""

        if not prediction_waypoints:
            return None

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = timestamp
        marker.ns = f"{self.car_namespace}_tam_prediction" if self.car_namespace else "tam_prediction"
        marker.id = obstacle_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Initialize pose with identity quaternion to avoid RViz warning
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Add configurable lifetime to prevent blinking - marker persists beyond update frequency
        marker.lifetime = rospy.Duration(self.marker_lifetime)

        # Marker appearance
        marker.scale.x = 0.1  # Line width
        marker.color.r = 1.0  # Red for predictions
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8  # Semi-transparent

        # Add prediction points
        marker.points = []
        for waypoint in prediction_waypoints:
            point = Point()
            point.x = waypoint.x_m
            point.y = waypoint.y_m
            point.z = 0.1  # Slightly above ground
            marker.points.append(point)

        return marker

    def run(self):
        """Main prediction loop"""

        # Wait for required data
        # rospy.loginfo(f"{self.log_name} Waiting for required data...")

        self.declare_and_update_parameters(skip_update=self.race_started)

        try:
            rospy.wait_for_message("global_waypoints",
                                   WpntArray, timeout=10.0)
            rospy.wait_for_message(
                "perception/multi_car_obstacles", ObstacleArray, timeout=10.0)
        except rospy.ROSException as e:
            rospy.logwarn(f"{self.log_name} Timeout waiting for messages: {e}")
            return

        # rospy.loginfo(
        #     f"{self.log_name} All required data received, starting prediction loop")

        # Main prediction loop (predictions are triggered by obstacle callbacks)
        rate = rospy.Rate(10)  # 10 Hz for marker cleanup and diagnostics

        while not rospy.is_shutdown():
            # Periodic cleanup and diagnostics
            if rospy.Time.now().to_sec() - self.last_prediction_time > 1.0:
                # Clear old markers if no obstacles detected recently
                empty_markers = MarkerArray()
                for i in range(5):  # Clear up to 5 old markers
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = f"{self.car_namespace}_tam_prediction" if self.car_namespace else "tam_prediction"
                    marker.id = i
                    marker.action = Marker.DELETE
                    empty_markers.markers.append(marker)

                self.markers_pub.publish(empty_markers)

            rate.sleep()


def main():
    """Main function"""
    try:
        predictor = TAMConstantOffsetPredictor()
        predictor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TAM predictor node interrupted")
    except Exception as e:
        rospy.logerr(f"TAM predictor node failed: {e}")
        raise


if __name__ == '__main__':
    main()
