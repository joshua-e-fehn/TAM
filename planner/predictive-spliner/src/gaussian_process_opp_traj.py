#!/usr/bin/env python3

import numpy as np
import rospy
from f110_msgs.msg import ObstacleArray, OpponentTrajectory, OppWpnt, WpntArray, ProjOppTraj
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, Matern, WhiteKernel, ConstantKernel
import time
import os
import pickle
from datetime import datetime
from scipy.optimize import fmin_l_bfgs_b
from frenet_converter.frenet_converter import FrenetConverter

from ccma import CCMA


class GaussianProcessOppTraj(object):
    def __init__(self):
        # Node
        rospy.init_node('gaussian_process_opp_traj', anonymous=True)
        self.rate = rospy.Rate(10)
        self.opp_traj = OpponentTrajectory()
        self.opp_traj_gp = OpponentTrajectory()
        self.opp_positions_in_map_frenet = []  # testing with Rosbag

        # Subscribers
        rospy.Subscriber('proj_opponent_trajectory',
                         ProjOppTraj, self.proj_opp_traj_cb)
        rospy.Subscriber('global_waypoints', WpntArray,
                         self.glb_wpnts_cb)  # global waypoints

        # Publishers
        self.opp_traj_gp_pub = rospy.Publisher(
            'opponent_trajectory', OpponentTrajectory, queue_size=10)
        self.opp_traj_marker_pub = rospy.Publisher(
            'opponent_traj_markerarray', MarkerArray, queue_size=10)

        # Frenet Converter
        rospy.wait_for_message("global_waypoints", WpntArray)
        self.converter = self.initialize_converter()

        # Parameters / switches
        self._use_global_prediction_param = rospy.get_param(
            '~use_global_prediction', False)
        self._global_prediction_speed_scale = rospy.get_param(
            '~global_prediction_speed_scale', 1.0)
        self._global_prediction_d_var = rospy.get_param(
            '~global_prediction_d_variance', 0.05)
        self._global_prediction_vs_var = rospy.get_param(
            '~global_prediction_vs_variance', 0.05)
        self._proj_traj_wait_timeout = rospy.get_param(
            '~proj_traj_wait_timeout', 0.5)
        self._global_prediction_max_speed = rospy.get_param(
            '~global_prediction_max_speed', 10.0)

        # Cached data for global prediction mode
        self._global_prediction_cache = None
        self._global_prediction_marker = None
        self._global_wpnts_dirty = True
        self.lap_count = 0.0
        self._last_prediction_mode = None

        # GP data saving parameters
        self._save_gp_data = rospy.get_param('~save_gp_data', True)
        self._gp_data_save_path = rospy.get_param(
            '~gp_data_save_path', '/tmp/gp_analysis')
        self._gp_data_saved = False
        self._gp_analysis_data = {}  # Store data for saving

        # Use GP for lateral prediction (matches paper: two independent GPs)
        # When True: uses GP with Matérn kernel for d(s) - provides uncertainty
        # When False: uses CCMA smoothing for d(s) - no uncertainty (legacy behavior)
        self._use_gp_for_lateral = rospy.get_param(
            '~use_gp_for_lateral', False)
        # Also check global parameter for runtime switching
        self._use_gp_for_lateral = rospy.get_param(
            '/race_test/use_gp_for_lateral', self._use_gp_for_lateral)
        if self._use_gp_for_lateral:
            rospy.loginfo(
                "[GP Opp Traj] Using GP for lateral prediction (paper mode)")
        else:
            rospy.loginfo(
                "[GP Opp Traj] Using CCMA for lateral prediction (legacy mode)")

    # Callback
    def proj_opp_traj_cb(self, data: ProjOppTraj):
        self.proj_opp_traj = data

    def glb_wpnts_cb(self, data):
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m]
                                  for wpnt in data.wpnts])
        self.glb_wpnts = data
        self.track_length = data.wpnts[-1].s_m
        self._global_wpnts_dirty = True

    # Functions

    def initialize_converter(self) -> bool:
        """Initialize the FrenetConverter object"""

        rospy.wait_for_message("global_waypoints", WpntArray)

        # Initialize the FrenetConverter object
        converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
        rospy.loginfo("[Tracking] initialized FrenetConverter object")

        return converter

    # Main Loop
    def get_gp_opp_traj(self):
        # Define the constant kernels with a lower bound for the constant_value parameter
        constant_kernel1_d = ConstantKernel(
            constant_value=0.5, constant_value_bounds=(1e-6, 1e3))
        constant_kernel2_d = ConstantKernel(
            constant_value=0.2, constant_value_bounds=(1e-6, 1e3))
        constant_kernel1_vs = ConstantKernel(
            constant_value=0.5, constant_value_bounds=(1e-6, 1e3))
        constant_kernel2_vs = ConstantKernel(
            constant_value=0.2, constant_value_bounds=(1e-6, 1e3))

        # Define the Gaussian Process kernel
        self.kernel_vs = constant_kernel1_vs * \
            RBF(length_scale=1.0) + constant_kernel2_vs * \
            WhiteKernel(noise_level=1)
        self.kernel_d = constant_kernel1_d * \
            Matern(length_scale=1.0, nu=3/2) + \
            constant_kernel2_d * WhiteKernel(noise_level=1)
        first_half_lap = True
        self.global_wpnts = rospy.wait_for_message(
            "global_waypoints", WpntArray)
        self.max_velocity = max(
            [wnpt.vx_mps for wnpt in self.global_wpnts.wpnts])
        ego_s_original = [wnpt.s_m for wnpt in self.global_wpnts.wpnts]
        # pop last point of original ego_s since it is double
        ego_s_original.pop()
        ego_s_doublelap = ego_s_original.copy()
        for i in range(len(ego_s_original)):
            ego_s_doublelap.append(ego_s_original[i]+self.track_length)

        # create a oppwpmt lap with velocity 100
        oppwpnts_list = self.make_initial_opponent_trajectory_msg(
            ego_s_original=ego_s_original)
        proj_opp_traj = ProjOppTraj()
        sorted_detection_list = []
        while not rospy.is_shutdown():
            global_mode_enabled = self._global_prediction_enabled()
            self._maybe_log_prediction_mode(global_mode_enabled)

            if global_mode_enabled:
                self._publish_global_waypoint_prediction()
                self.rate.sleep()
                continue

            try:
                proj_opp_traj = rospy.wait_for_message(
                    'proj_opponent_trajectory', ProjOppTraj, timeout=self._proj_traj_wait_timeout)
            except rospy.ROSException:
                continue
            self.lap_count = proj_opp_traj.lapcount
            opp_on_traj = proj_opp_traj.opp_is_on_trajectory
            nr_of_points = proj_opp_traj.nrofpoints
            if opp_on_traj == True and len(proj_opp_traj.detections) != 0:
                if self.lap_count <= 1:
                    sorted_detection_list_first_lap, around_origin = self.create_sorted_detection_list(
                        proj_opponent_detections=proj_opp_traj.detections, sorted_detection_list=[], ego_s_original=ego_s_original)
                    for i in range(len(sorted_detection_list_first_lap)):
                        sorted_detection_list.append(
                            sorted_detection_list_first_lap[i])

                    opponent_s_sorted, opponent_d_sorted, opponent_vs_sorted, opponent_vd_sorted = self.get_sorted_s_d_vs_vd_lists(
                        sorted_detection_list=sorted_detection_list_first_lap)

                    if first_half_lap:
                        first_half_lap = False
                        ego_s_sorted_halflap = []
                        for i in range(len(ego_s_doublelap)):
                            if ego_s_doublelap[i] > opponent_s_sorted[0] and ego_s_doublelap[i] < opponent_s_sorted[0]+(self.track_length/2):
                                ego_s_sorted_halflap.append(ego_s_doublelap[i])
                        last_ego_s = ego_s_sorted_halflap[-1]
                        first_ego_s = ego_s_sorted_halflap[0]

                    else:
                        # make a new ego_s_sorted_halflap with the last_ego_s as a statring point and first_ego_s as an end point
                        ego_s_sorted_halflap = []
                        last_ego_s = last_ego_s % self.track_length
                        if last_ego_s > first_ego_s:
                            first_ego_s = first_ego_s+self.track_length

                        for i in range(len(ego_s_doublelap)):
                            if ego_s_doublelap[i] >= last_ego_s and ego_s_doublelap[i] <= first_ego_s:
                                ego_s_sorted_halflap.append(ego_s_doublelap[i])
                        last_ego_s = ego_s_sorted_halflap[-1]
                        first_ego_s = ego_s_sorted_halflap[0]

                    oppwpnts_list, opp_traj_marker_array = self.get_opponnent_wpnts(whole_lap=False, ego_s_sorted=ego_s_sorted_halflap, opponent_s_sorted=opponent_s_sorted,
                                                                                    opponent_d_sorted=opponent_d_sorted, opponent_vs_sorted=opponent_vs_sorted,
                                                                                    opponent_vd_sorted=opponent_vd_sorted, arond_origin=around_origin, oppwpnts_list=oppwpnts_list)

                    opp_traj_gp_msg = self.make_opponent_trajectory_msg(
                        oppwpnts_list=oppwpnts_list, lap_count=self.lap_count, raw_oppenent_traj_msg=proj_opp_traj)
                    # Publish
                    self.opp_traj_gp_pub.publish(opp_traj_gp_msg)
                    self.opp_traj_marker_pub.publish(opp_traj_marker_array)

                    if around_origin:  # reset s value of sorted_detection_list and sort again
                        for i in range(len(sorted_detection_list)):
                            sorted_detection_list[i].s = sorted_detection_list[i].s % self.track_length
                        sorted_detection_list.sort(key=lambda x: x.s)

                    if self.lap_count == 1:
                        self.lap_count = 1.1
                        sorted_detection_list, around_origin = self.create_sorted_detection_list(
                            proj_opponent_detections=proj_opp_traj.detections, sorted_detection_list=sorted_detection_list, ego_s_original=ego_s_original)
                        opponent_s_sorted, opponent_d_sorted, opponent_vs_sorted, opponent_vd_sorted = self.get_sorted_s_d_vs_vd_lists(
                            sorted_detection_list=sorted_detection_list)
                        oppwpnts_list, opp_traj_marker_array = self.get_opponnent_wpnts(whole_lap=True, ego_s_sorted=ego_s_original, opponent_s_sorted=opponent_s_sorted,
                                                                                        opponent_d_sorted=opponent_d_sorted, opponent_vs_sorted=opponent_vs_sorted,
                                                                                        opponent_vd_sorted=opponent_vd_sorted, arond_origin=False, oppwpnts_list=oppwpnts_list)
                        # Publish
                        self.opp_traj_gp_pub.publish(opp_traj_gp_msg)
                        self.opp_traj_marker_pub.publish(opp_traj_marker_array)

                else:  # adding additional points to the trajectory
                    sorted_detection_list, around_origin = self.create_sorted_detection_list(
                        proj_opponent_detections=proj_opp_traj.detections, sorted_detection_list=sorted_detection_list, ego_s_original=ego_s_original)

                    opponent_s_sorted, opponent_d_sorted, opponent_vs_sorted, opponent_vd_sorted = self.get_sorted_s_d_vs_vd_lists(
                        sorted_detection_list=sorted_detection_list)

                    oppwpnts_list, opp_traj_marker_array = self.get_opponnent_wpnts(whole_lap=True, ego_s_sorted=ego_s_original, opponent_s_sorted=opponent_s_sorted,
                                                                                    opponent_d_sorted=opponent_d_sorted, opponent_vs_sorted=opponent_vs_sorted,
                                                                                    opponent_vd_sorted=opponent_vd_sorted, arond_origin=False, oppwpnts_list=oppwpnts_list)

                    opp_traj_gp_msg = self.make_opponent_trajectory_msg(
                        oppwpnts_list=oppwpnts_list, lap_count=self.lap_count, raw_oppenent_traj_msg=proj_opp_traj)

                    # Publish
                    self.opp_traj_gp_pub.publish(opp_traj_gp_msg)
                    self.opp_traj_marker_pub.publish(opp_traj_marker_array)

    def _global_prediction_enabled(self):
        if rospy.get_param('~use_global_prediction', self._use_global_prediction_param):
            return True

        return rospy.get_param('/race_test/use_global_prediction', False)

    def _get_global_prediction_max_speed(self):
        """Get max speed limit, checking race_test parameter first"""
        return rospy.get_param('/race_test/global_prediction_max_speed',
                               self._global_prediction_max_speed)

    def _publish_global_waypoint_prediction(self):
        if not hasattr(self, 'glb_wpnts'):
            return

        oppwpnts_list, marker_array = self._get_global_waypoint_prediction()
        if oppwpnts_list is None:
            return

        # Use lap_count >= 1.0 for global prediction to allow overtaking immediately
        # (collision prediction checks lap_count < 1 to force trailing)
        opp_traj_gp_msg = self.make_opponent_trajectory_msg(
            oppwpnts_list=oppwpnts_list,
            lap_count=1.0,
            raw_oppenent_traj_msg=None)
        self.opp_traj_gp_pub.publish(opp_traj_gp_msg)
        self.opp_traj_marker_pub.publish(marker_array)

    def _get_global_waypoint_prediction(self):
        if self._global_prediction_cache is None or self._global_wpnts_dirty:
            if not hasattr(self, 'glb_wpnts'):
                return None, None

            # Get current max speed limit (may be updated via race_test param)
            max_speed_limit = self._get_global_prediction_max_speed()

            oppwpnts_list = []
            for wpnt in self.glb_wpnts.wpnts[:-1]:
                oppwpnt = OppWpnt()
                oppwpnt.x_m = wpnt.x_m
                oppwpnt.y_m = wpnt.y_m
                oppwpnt.s_m = wpnt.s_m
                oppwpnt.d_m = 0.0
                # Cap at physical speed limit before scaling (like obstacle publisher)
                capped_speed = min(wpnt.vx_mps, max_speed_limit)
                oppwpnt.proj_vs_mps = max(
                    0.0, capped_speed * self._global_prediction_speed_scale)
                oppwpnt.vd_mps = 0.0
                oppwpnt.d_var = self._global_prediction_d_var
                oppwpnt.vs_var = self._global_prediction_vs_var
                oppwpnts_list.append(oppwpnt)

            self._global_prediction_cache = oppwpnts_list
            self._global_prediction_marker = self._visualize_opponent_wpnts(
                oppwpnts_list=oppwpnts_list)
            self._global_wpnts_dirty = False

        return self._global_prediction_cache, self._global_prediction_marker

    def _maybe_log_prediction_mode(self, using_global_prediction):
        if self._last_prediction_mode == using_global_prediction:
            return

        node_switch = rospy.get_param(
            '~use_global_prediction', self._use_global_prediction_param)
        race_test_switch = rospy.get_param(
            '/race_test/use_global_prediction', False)
        mode_label = "GLOBAL WAYPOINT" if using_global_prediction else "GAUSSIAN PROCESS"
        rospy.loginfo("[GP Opp Traj] Prediction mode: %s (node switch=%s, /race_test switch=%s)",
                      mode_label, node_switch, race_test_switch)
        self._last_prediction_mode = using_global_prediction

    def save_gp_data(self, train_s, train_d, train_vs, train_vd,
                     s_pred, d_pred, vs_pred, sigma_d, sigma_vs,
                     gpr_vs, gpr_d=None, d_pred_ccma=None):
        """
        Save GP regression data for later analysis.

        Saves:
        - Training data (raw opponent detections)
        - Prediction points and results
        - Uncertainty estimates (sigma)
        - Fitted GP models with kernel hyperparameters
        - CCMA smoothed d predictions (if available)
        - Experiment configuration (map, opponent path params, etc.)
        """
        # Read parameters dynamically to allow runtime enabling
        # Check both private namespace and global /race_test/ namespace
        save_enabled = rospy.get_param('~save_gp_data', self._save_gp_data) or \
            rospy.get_param('/race_test/save_gp_data', True)
        save_path = rospy.get_param(
            '~gp_data_save_path', self._gp_data_save_path)
        # Also check global path parameter
        save_path = rospy.get_param('/race_test/gp_data_save_path', save_path)

        if not save_enabled or self._gp_data_saved:
            return

        try:
            # Create save directory if it doesn't exist
            os.makedirs(save_path, exist_ok=True)

            # Read experiment configuration from ROS params
            use_gp_lateral = rospy.get_param(
                '/race_test/use_gp_for_lateral', self._use_gp_for_lateral)
            experiment_config = {
                'map_name': rospy.get_param('/map_name', 'unknown'),
                'path_amplitude': rospy.get_param('/obstacle_publisher/path_amplitude', 0.0),
                'path_frequency': rospy.get_param('/obstacle_publisher/path_frequency', 0.0),
                'path_phase': rospy.get_param('/obstacle_publisher/path_phase', 0.0),
                'obstacle_speed': rospy.get_param('/obstacle_publisher/constant_speed', -1.0),
                'speed_amplitude': rospy.get_param('/obstacle_publisher/speed_amplitude', 0.0),
                'speed_scaler': rospy.get_param('/obstacle_publisher/speed_scaler', 0.8),
                'use_global_prediction': self._use_global_prediction_param,
                # True = paper mode (GP), False = legacy (CCMA)
                'use_gp_for_lateral': use_gp_lateral,
                'lap_count': self.lap_count,
            }

            # Prepare data dictionary
            gp_data = {
                'timestamp': datetime.now().isoformat(),
                'track_length': self.track_length,
                'experiment_config': experiment_config,

                # Training data (raw opponent detections)
                'training_data': {
                    's': train_s.tolist() if isinstance(train_s, np.ndarray) else train_s,
                    'd': train_d.tolist() if isinstance(train_d, np.ndarray) else train_d,
                    'vs': train_vs.tolist() if isinstance(train_vs, np.ndarray) else train_vs,
                    'vd': train_vd.tolist() if isinstance(train_vd, np.ndarray) else train_vd,
                    'n_samples': len(train_s)
                },

                # Prediction points (ego waypoint s positions)
                'prediction_points': {
                    's': s_pred.flatten().tolist() if isinstance(s_pred, np.ndarray) else s_pred,
                    'n_points': len(s_pred)
                },

                # GP predictions
                'predictions': {
                    'd': d_pred.tolist() if isinstance(d_pred, np.ndarray) else d_pred,
                    'vs': vs_pred.tolist() if isinstance(vs_pred, np.ndarray) else vs_pred,
                    'd_ccma': d_pred_ccma.tolist() if d_pred_ccma is not None and isinstance(d_pred_ccma, np.ndarray) else d_pred_ccma,
                },

                # Uncertainty estimates
                'uncertainty': {
                    'sigma_d': sigma_d.tolist() if isinstance(sigma_d, np.ndarray) else sigma_d,
                    'sigma_vs': sigma_vs.tolist() if isinstance(sigma_vs, np.ndarray) else sigma_vs,
                },

                # GP model parameters (velocity)
                'gp_vs_model': {
                    'kernel_initial': str(self.kernel_vs),
                    'kernel_fitted': str(gpr_vs.kernel_),
                    'log_marginal_likelihood': float(gpr_vs.log_marginal_likelihood_value_),
                    'n_features': gpr_vs.n_features_in_,
                },
            }

            # Add d model if available (when not using CCMA for whole lap)
            if gpr_d is not None:
                gp_data['gp_d_model'] = {
                    'kernel_initial': str(self.kernel_d),
                    'kernel_fitted': str(gpr_d.kernel_),
                    'log_marginal_likelihood': float(gpr_d.log_marginal_likelihood_value_),
                    'n_features': gpr_d.n_features_in_,
                }

            # Generate filename with timestamp
            timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
            json_filename = os.path.join(
                save_path, f'gp_analysis_{timestamp_str}.pkl')

            # Save as pickle (preserves numpy arrays and sklearn objects)
            with open(json_filename, 'wb') as f:
                pickle.dump(gp_data, f)

            # Save GP model parameters (not the full models, as they contain unpicklable local functions)
            # We save the fitted kernel parameters and training data which is sufficient for analysis
            models_filename = os.path.join(
                save_path, f'gp_models_{timestamp_str}.pkl')
            models_data = {
                # Save kernel parameters as strings (they contain the fitted hyperparameters)
                'kernel_vs_fitted': str(gpr_vs.kernel_),
                'kernel_d_fitted': str(gpr_d.kernel_) if gpr_d is not None else None,
                'kernel_vs_initial': str(self.kernel_vs),
                'kernel_d_initial': str(self.kernel_d),
                # Save the training data for potential model reconstruction
                'X_train_vs': gpr_vs.X_train_.tolist(),
                'y_train_vs': gpr_vs.y_train_.tolist(),
                'alpha_vs': gpr_vs.alpha_.tolist(),  # Dual coefficients
            }
            # Add d model training data if available
            if gpr_d is not None:
                models_data['X_train_d'] = gpr_d.X_train_.tolist()
                models_data['y_train_d'] = gpr_d.y_train_.tolist()
                models_data['alpha_d'] = gpr_d.alpha_.tolist()

            with open(models_filename, 'wb') as f:
                pickle.dump(models_data, f)

            self._gp_data_saved = True
            rospy.loginfo(
                f"[GP Opp Traj] Saved GP analysis data to {json_filename}")
            rospy.loginfo(
                f"[GP Opp Traj] Saved GP models to {models_filename}")

        except Exception as e:
            rospy.logerr(f"[GP Opp Traj] Failed to save GP data: {e}")

    # Helper Functions
    def create_sorted_detection_list(self, proj_opponent_detections: list, sorted_detection_list: list, ego_s_original: list):
        """Sort the opponent trajectory based on the s position and return the sorted lists"""
        around_origin = False

        if self.lap_count <= 1:
            for i in range(len(proj_opponent_detections)-1):
                if proj_opponent_detections[i].s > proj_opponent_detections[i+1].s:
                    around_origin = True
                if around_origin:
                    proj_opponent_detections[i +
                                             1].s = proj_opponent_detections[i+1].s+self.track_length
        else:
            for i in range(len(proj_opponent_detections)-1):
                proj_opponent_detections[i +
                                         1].s = proj_opponent_detections[i+1].s % self.track_length

        for i in range(len(proj_opponent_detections)):
            sorted_detection_list.append(proj_opponent_detections[i])
        sorted_detection_list.sort(key=lambda x: x.s)
        printed_s_list = [sorted_detection_list[i].s for i in range(
            len(sorted_detection_list))]

        # 2*distance between waypoints
        delta_s = 2*(self.track_length/len(ego_s_original))
        if (len(sorted_detection_list) > 200) and (self.lap_count >= 1):

            # if detections are too close together, remove the older one
            sorted_detection_list_new = []
            last_s = sorted_detection_list[-1].s
            i = 0
            for x in range(int(self.track_length/delta_s)):
                if last_s > (x+1)*delta_s:
                    helper_list = []
                    while (sorted_detection_list[i].s < x*delta_s):
                        helper_list.append(sorted_detection_list[i])
                        i = i+1
                    if (len(helper_list) > 0):
                        helper_list.sort(key=lambda x: x.time)
                        sorted_detection_list_new.append(helper_list[-1])

            sorted_detection_list = sorted_detection_list_new

        return sorted_detection_list, around_origin

    def get_sorted_s_d_vs_vd_lists(self, sorted_detection_list: list):
        """Sort the opponent trajectory based on the s position and return the sorted lists"""

        opponent_s_sorted = [
            detection.s for detection in sorted_detection_list]
        opponent_d_sorted = [
            detection.d for detection in sorted_detection_list]
        opponent_vs_sorted = [
            detection.vs for detection in sorted_detection_list]
        opponent_vd_sorted = [
            detection.vd for detection in sorted_detection_list]

        return opponent_s_sorted, opponent_d_sorted, opponent_vs_sorted, opponent_vd_sorted

    def get_opponnent_wpnts(self, whole_lap: bool, ego_s_sorted: list, opponent_s_sorted: list, opponent_d_sorted: list, opponent_vs_sorted: list, opponent_vd_sorted: list, arond_origin: bool, oppwpnts_list: list):
        """Resample the opponent trajectory based on the ego vehicle's s position and return the resampled opponent trajectory (aso return the resampled opponent trajectory as a marker array)"""
        ego_s_sorted_copy = ego_s_sorted.copy()

        # testing with Rosbag
        # split self.opp_positions_in_map_frenet in a s/s/vs/vd list
        # opp_s = [position[0] for position in self.opp_positions_in_map_frenet]
        # opp_d = [position[1] for position in self.opp_positions_in_map_frenet]
        # opp_vs = [position[2] for position in self.opp_positions_in_map_frenet]
        # opp_vd = [position[3] for position in self.opp_positions_in_map_frenet]
        if whole_lap:

            # Predict the d coordinate with CCMA in case of a whole lap
            # stretch the s and d list to ensure that the CCMA works smoothly around the origin
            opp_s_copy = opponent_s_sorted.copy()
            opp_d_copy = opponent_d_sorted.copy()
            opp_s_copy.insert(0, opp_s_copy[-1])
            opp_d_copy.insert(0, opp_d_copy[-1])
            opp_s_copy.append(opp_s_copy[1])
            opp_d_copy.append(opp_d_copy[1])

            # convert to cartesian
            noisy_xy_points = self.converter.get_cartesian(
                opp_s_copy, opp_d_copy)
            noisy_xy_points = noisy_xy_points.transpose()
            # smooth the trajectory with CCMA
            ccma = CCMA(w_ma=5, w_cc=3)
            smoothed_xy_points = ccma.filter(noisy_xy_points)

            # convert back to frenet
            smoothed_sd_points = self.converter.get_frenet(
                smoothed_xy_points[:, 0], smoothed_xy_points[:, 1])
            # sort the points based on s
            smoothed_s_points = smoothed_sd_points[0]
            smoothed_d_points = smoothed_sd_points[1]

            smoothed_s_points, smoothed_d_points = zip(
                *sorted(zip(smoothed_s_points, smoothed_d_points)))

            # interpolate the smoothed trajectory on the same s points as the ego vehicles trajectory
            d_pred_CCMA = np.interp(
                ego_s_sorted, smoothed_s_points, smoothed_d_points)

            # Preparing the data for the Gaussian Process
            # prepend the last points of the opponent trajectory to the beginning of the list
            n = -1
            nr_of_points_pre = 0
            for i in range(len(opponent_s_sorted)):
                # go 3 m in negative direction
                if abs(opponent_s_sorted[n]-self.track_length) < 3:
                    opponent_s_sorted.insert(
                        0, opponent_s_sorted[n]-self.track_length)
                    n = n-1
                    nr_of_points_pre = nr_of_points_pre+1
            n = -1
            for i in range(nr_of_points_pre):
                opponent_d_sorted.insert(0, opponent_d_sorted[n])
                n = n-1
            n = -1
            for i in range(nr_of_points_pre):
                opponent_vs_sorted.insert(0, opponent_vs_sorted[n])
                n = n-1
            n = -1
            for i in range(nr_of_points_pre):
                opponent_vd_sorted.insert(0, opponent_vd_sorted[n])
                n = n-1
            n = -1
            # prepend last points of the ego_s_sorted to the beginning of the list as a negative value
            nr_of_points_ego_s = 0
            for i in range(len(ego_s_sorted_copy)):
                if abs(ego_s_sorted_copy[n]-self.track_length) < 3:
                    ego_s_sorted_copy.insert(
                        0, ego_s_sorted_copy[n]-self.track_length)
                    n = n-1
                    nr_of_points_ego_s = nr_of_points_ego_s+1
            # append the first points of the opponent trajectory to the end of the list
            n = 0
            nr_of_points_app = 0
            for i in range(len(opponent_s_sorted)):
                if abs(opponent_s_sorted[nr_of_points_pre+n]) < 3:
                    opponent_s_sorted.append(
                        opponent_s_sorted[nr_of_points_pre+n]+self.track_length)
                    n = n+1
                    nr_of_points_app = nr_of_points_app+1
            n = 0
            for i in range(nr_of_points_app):
                opponent_d_sorted.append(opponent_d_sorted[nr_of_points_pre+n])
                n = n+1
            n = 0
            for i in range(nr_of_points_app):
                opponent_vs_sorted.append(
                    opponent_vs_sorted[nr_of_points_pre+n])
                n = n+1
            n = 0
            for i in range(nr_of_points_app):
                opponent_vd_sorted.append(
                    opponent_vd_sorted[nr_of_points_pre+n])
                n = n+1
            n = 0

            # append first 3m of the ego_s_sorted to the end of the list as a value bigger than the track_length
            for i in range(len(ego_s_sorted_copy)):
                if abs(ego_s_sorted_copy[nr_of_points_ego_s+n]) < 3:
                    ego_s_sorted_copy.append(
                        ego_s_sorted_copy[nr_of_points_ego_s+n]+self.track_length)
                    n = n+1
            train_s = np.array([opponent_s_sorted[i]
                               for i in range(len(opponent_s_sorted))])
            train_d = np.array([opponent_d_sorted[i]
                               for i in range(len(opponent_d_sorted))])
            train_vs = np.array([opponent_vs_sorted[i]
                                for i in range(len(opponent_vs_sorted))])
            train_vd = np.array([opponent_vd_sorted[i]
                                for i in range(len(opponent_vd_sorted))])
        else:
            train_s = np.array([opponent_s_sorted[i]
                               for i in range(len(opponent_s_sorted))])
            train_d = np.array([opponent_d_sorted[i]
                               for i in range(len(opponent_d_sorted))])
            train_vs = np.array([opponent_vs_sorted[i]
                                for i in range(len(opponent_vs_sorted))])
            train_vd = np.array([opponent_vd_sorted[i]
                                for i in range(len(opponent_vd_sorted))])

        opponent_s_sorted_reshape = train_s.reshape(-1, 1)
        opponent_d_sorted_reshape = train_d.reshape(-1, 1)
        opponent_vs_sorted_reshape = train_vs.reshape(-1, 1)
        opponent_vd_sorted_reshape = train_vd.reshape(-1, 1)

        # Define a range of s values for prediction
        ego_s_sorted_nparray = np.array(ego_s_sorted_copy)
        s_pred = ego_s_sorted_nparray.reshape(-1, 1)

        # Fit the Gaussian Process Regressor to the data
        def optimizer_wrapper(obj_func, initial_theta, bounds):
            solution, function_value, _ = fmin_l_bfgs_b(
                obj_func, initial_theta, bounds=bounds)
            return solution, function_value

        # Fit Vs (always use GP for velocity as per paper)
        gpr_vs = GaussianProcessRegressor(
            kernel=self.kernel_vs, optimizer=optimizer_wrapper)
        gpr_vs.fit(opponent_s_sorted_reshape, opponent_vs_sorted_reshape)
        vs_pred, sigma_vs = gpr_vs.predict(s_pred, return_std=True)

        # Check if we should use GP for lateral (runtime parameter check)
        use_gp_lateral = rospy.get_param(
            '/race_test/use_gp_for_lateral', self._use_gp_for_lateral)

        # Fit D with GP if:
        # 1. Not whole_lap yet (first half lap - always use GP), OR
        # 2. use_gp_for_lateral is True (paper mode - always use GP)
        if not whole_lap or use_gp_lateral:
            gpr_d = GaussianProcessRegressor(
                kernel=self.kernel_d, optimizer=optimizer_wrapper)
            gpr_d.fit(opponent_s_sorted_reshape, opponent_d_sorted_reshape)
            d_pred_GP, sigma_d = gpr_d.predict(s_pred, return_std=True)

        # shorten the copy lists (that was changed) to the length of the original ego_s
        if whole_lap:
            # Check runtime parameter for GP lateral mode
            use_gp_lateral = rospy.get_param(
                '/race_test/use_gp_for_lateral', self._use_gp_for_lateral)

            n = 0
            for i in range(len(ego_s_sorted_copy)):
                if ego_s_sorted_copy[i-n] >= self.track_length or ego_s_sorted_copy[i-n] < 0:
                    ego_s_sorted_copy.pop(i-n)
                    # Filter GP predictions when using GP for lateral (before whole_lap or paper mode)
                    if use_gp_lateral:
                        d_pred_GP = np.delete(d_pred_GP, i-n)
                        sigma_d = np.delete(sigma_d, i-n)
                    vs_pred = np.delete(vs_pred, i-n)
                    n += 1
        else:
            # Not whole_lap yet - always using GP for lateral
            n = 0
            for i in range(len(ego_s_sorted_copy)):
                if ego_s_sorted_copy[i-n] >= self.track_length or ego_s_sorted_copy[i-n] < 0:
                    ego_s_sorted_copy.pop(i-n)
                    d_pred_GP = np.delete(d_pred_GP, i-n)
                    sigma_d = np.delete(sigma_d, i-n)
                    vs_pred = np.delete(vs_pred, i-n)
                    n += 1

        if whole_lap:
            print("Saving GP data for whole lap prediction...", flush=True)
            # if use_gp_lateral:
            # Paper mode: Use GP for lateral prediction (provides uncertainty)
            resampled_opponent_d = d_pred_GP
            sigma_d_for_save = sigma_d
            gpr_d_for_save = gpr_d
            d_pred_ccma_for_save = None  # No CCMA in paper mode
            # else:
            #     # Legacy mode: Use CCMA for lateral prediction
            #     resampled_opponent_d = d_pred_CCMA
            #     sigma_d_for_save = np.zeros_like(vs_pred)
            #     gpr_d_for_save = None
            #     d_pred_ccma_for_save = d_pred_CCMA

            # Save GP data after first full lap (whole_lap=True means we have complete data)
            self.save_gp_data(
                train_s=train_s,
                train_d=train_d,
                train_vs=train_vs,
                train_vd=train_vd,
                s_pred=s_pred,
                d_pred=resampled_opponent_d,
                vs_pred=vs_pred,
                sigma_d=sigma_d_for_save,
                sigma_vs=sigma_vs,
                gpr_vs=gpr_vs,
                gpr_d=gpr_d_for_save,
                d_pred_ccma=d_pred_ccma_for_save
            )
        else:
            resampled_opponent_d = d_pred_GP
            # Don't save GP data for half-lap predictions - wait for full lap
        resampled_opponent_vs = vs_pred
        resampled_opponent_vd = np.interp(
            ego_s_sorted, opponent_s_sorted, opponent_vd_sorted)

        if arond_origin:
            ego_s = [ego_s_sorted[i] %
                     self.track_length for i in range(len(ego_s_sorted_copy))]
        else:
            ego_s = ego_s_sorted_copy

        resampled_wpnts_xy = self.converter.get_cartesian(
            ego_s, resampled_opponent_d.tolist())

        # replace all the entries where there is a corresponding ego_s with the interpolated values
        i = 0

        for i in range(len(oppwpnts_list)):
            for j in range(len(ego_s)):
                if abs(ego_s[j]-oppwpnts_list[i].s_m) < 1e-8:
                    oppwpnts_list[i].x_m = resampled_wpnts_xy[0][j]
                    oppwpnts_list[i].y_m = resampled_wpnts_xy[1][j]
                    oppwpnts_list[i].d_m = resampled_opponent_d[j]
                    oppwpnts_list[i].proj_vs_mps = resampled_opponent_vs[j]
                    oppwpnts_list[i].vd_mps = resampled_opponent_vd[j]
                    if not whole_lap:
                        oppwpnts_list[i].d_var = sigma_d[j]
                    elif use_gp_lateral:
                        # Paper mode: use GP uncertainty for lateral
                        oppwpnts_list[i].d_var = sigma_d[j]
                    else:
                        # Legacy CCMA mode: no lateral uncertainty
                        oppwpnts_list[i].d_var = 0
                    oppwpnts_list[i].vs_var = sigma_vs[j]
        opp_traj_marker_array = self._visualize_opponent_wpnts(
            oppwpnts_list=oppwpnts_list)

        return oppwpnts_list, opp_traj_marker_array

    def make_initial_opponent_trajectory_msg(self, ego_s_original: list):

        # make trajectory with velocity 100 for the first half lap
        resampled_wpnts_xy_original = self.converter.get_cartesian(
            ego_s_original, np.zeros(len(ego_s_original)).tolist())
        oppwpnts_list = []
        i = 0

        for i in range(len(ego_s_original)):
            oppwpnts = OppWpnt()
            oppwpnts.x_m = resampled_wpnts_xy_original[0][i]
            oppwpnts.y_m = resampled_wpnts_xy_original[1][i]
            oppwpnts.s_m = ego_s_original[i]
            oppwpnts.d_m = 0
            oppwpnts.proj_vs_mps = 100
            oppwpnts.vd_mps = 0
            oppwpnts_list.append(oppwpnts)
        return oppwpnts_list

    def make_opponent_trajectory_msg(self, oppwpnts_list: list, lap_count: int, raw_oppenent_traj_msg: ObstacleArray):
        """Make the opponent trajectory message and return it"""

        opponent_trajectory_msg = OpponentTrajectory()
        opponent_trajectory_msg.header.seq = lap_count
        opponent_trajectory_msg.header.stamp = rospy.Time.now()
        opponent_trajectory_msg.header.frame_id = "opponent_trajectory"
        opponent_trajectory_msg.lap_count = lap_count
        opponent_trajectory_msg.oppwpnts = oppwpnts_list

        return opponent_trajectory_msg

    def _visualize_opponent_wpnts(self, oppwpnts_list: list):
        """Visualize the resampled opponent trajectory as a marker array"""
        opp_traj_marker_array = MarkerArray()

        i = 0
        for i in range(len(oppwpnts_list)):
            marker_height = oppwpnts_list[i].proj_vs_mps/self.max_velocity

            marker = Marker(header=rospy.Header(
                frame_id="map"), id=i, type=Marker.CYLINDER)
            marker.pose.position.x = oppwpnts_list[i].x_m
            marker.pose.position.y = oppwpnts_list[i].y_m
            marker.pose.position.z = marker_height/2
            marker.pose.orientation.w = 1.0
            marker.scale.x = min(max(5 * oppwpnts_list[i].d_var, 0.07), 0.7)
            marker.scale.y = min(max(5 * oppwpnts_list[i].d_var, 0.07), 0.7)
            marker.scale.z = marker_height
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            opp_traj_marker_array.markers.append(marker)  # frenpy

        return opp_traj_marker_array


if __name__ == '__main__':

    node = GaussianProcessOppTraj()
    node.get_gp_opp_traj()
    rospy.spin()
