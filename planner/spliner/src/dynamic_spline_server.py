#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from spliner.cfg import dyn_spliner_tunerConfig


def callback(config, level):
    # Ensuring nice rounding by either 0.05 or 0.5
    config.evasion_dist = round(config.evasion_dist * 20) / 20
    config.obs_traj_tresh = round(config.obs_traj_tresh * 20) / 20
    config.spline_bound_mindist = round(config.spline_bound_mindist * 20) / 20

    config.pre_apex_dist0 = round(config.pre_apex_dist0 * 2) / 2
    # Ensuring that the pre_apex_dist1 is always greater than pre_apex_dist0
    config.pre_apex_dist1 = round(
        min(config.pre_apex_dist0 + 0.5, config.pre_apex_dist1) * 2) / 2
    config.pre_apex_dist2 = round(
        min(config.pre_apex_dist1 + 0.5, config.pre_apex_dist2) * 2) / 2
    config.post_apex_dist0 = round(config.post_apex_dist0 * 2) / 2
    config.post_apex_dist1 = round(
        max(config.post_apex_dist0 + 0.5, config.post_apex_dist1) * 2) / 2
    config.post_apex_dist2 = round(
        max(config.post_apex_dist1 + 0.5, config.post_apex_dist2) * 2) / 2
    config.kd_obs_pred = round(config.kd_obs_pred * 20) / 20
    config.fixed_pred_time = round(config.fixed_pred_time * 100) / 100
    return config


if __name__ == "__main__":
    rospy.init_node("dynamic_spline_tuner_node", anonymous=False)
    print('[Planner] Dynamic Spline Server Launched...')

    # Read initial values from ROS parameters (set by test framework or launch files)
    # This allows test framework to control initial configuration
    # Priority order for single-car: 1) Global params, 2) Node-private, 3) Namespaced, 4) Defaults
    # Priority order for multi-car: 1) Car-namespaced params (/car1/param), 2) Global, 3) Node-private, 4) Namespaced, 5) Defaults

    # Detect if we're in a car namespace (multi-car mode)
    # e.g., /car1/dynamic_spline_tuner_node or /dynamic_spline_tuner_node
    node_name = rospy.get_name()
    car_namespace = None
    if '/car1/' in node_name or node_name.startswith('/car1'):
        car_namespace = '/car1'
    elif '/car2/' in node_name or node_name.startswith('/car2'):
        car_namespace = '/car2'

    def get_param_with_car_namespace(param_name, default):
        """Get parameter checking car namespace first if in multi-car mode"""
        if car_namespace:
            # Multi-car: check /car1/param first
            return rospy.get_param(f'{car_namespace}/{param_name}',
                                   rospy.get_param(f'/{param_name}',
                                                   rospy.get_param(f'~{param_name}',
                                                                   rospy.get_param(f'dynamic_spline_tuner_node/{param_name}', default))))
        else:
            # Single-car: check global /param first
            return rospy.get_param(f'/{param_name}',
                                   rospy.get_param(f'~{param_name}',
                                                   rospy.get_param(f'dynamic_spline_tuner_node/{param_name}', default)))

    initial_config = {
        'evasion_dist': get_param_with_car_namespace('evasion_dist', 0.65),
        'obs_traj_tresh': get_param_with_car_namespace('obs_traj_tresh', 1.5),
        'spline_bound_mindist': get_param_with_car_namespace('spline_bound_mindist', 0.2),
        'pre_apex_dist0': get_param_with_car_namespace('pre_apex_dist0', 4.0),
        'pre_apex_dist1': get_param_with_car_namespace('pre_apex_dist1', 3.0),
        'pre_apex_dist2': get_param_with_car_namespace('pre_apex_dist2', 2.0),
        'post_apex_dist0': get_param_with_car_namespace('post_apex_dist0', 4.5),
        'post_apex_dist1': get_param_with_car_namespace('post_apex_dist1', 5.0),
        'post_apex_dist2': get_param_with_car_namespace('post_apex_dist2', 5.5),
        'kd_obs_pred': get_param_with_car_namespace('kd_obs_pred', 1.0),
        'fixed_pred_time': get_param_with_car_namespace('fixed_pred_time', 0.15),
    }

    srv = Server(dyn_spliner_tunerConfig, callback)

    # Update server with initial configuration from ROS parameters
    srv.update_configuration(initial_config)

    rospy.loginfo(
        '[Planner] Dynamic Spline Server initialized with parameters from ROS param server')
    rospy.spin()
