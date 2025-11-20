#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from predictive_spliner.cfg import dyn_sqp_tunerConfig


def callback(config, level):
    config.evasion_dist = round(config.evasion_dist, 2)
    config.obs_traj_tresh = round(config.obs_traj_tresh, 2)
    config.spline_bound_mindist = round(config.spline_bound_mindist, 3)
    config.lookahead_dist = round(config.lookahead_dist, 2)
    config.avoidance_resolution = round(config.avoidance_resolution)
    config.back_to_raceline_before = round(config.back_to_raceline_before, 2)
    config.back_to_raceline_after = round(config.back_to_raceline_after, 2)

    config.avoid_static_obs = config.avoid_static_obs
    return config


if __name__ == "__main__":
    rospy.init_node("dynamic_sqp_tuner_node", anonymous=False)
    print('[Planner] Dynamic SQP Server Launched...')

    # Read initial values from ROS parameters (set by test framework or launch files)
    # This allows test framework to control initial configuration
    # Priority order for single-car: 1) Global params, 2) Node-private, 3) Namespaced, 4) Defaults
    # Priority order for multi-car: 1) Car-namespaced params (/car1/param), 2) Global, 3) Node-private, 4) Namespaced, 5) Defaults

    # Detect if we're in a car namespace (multi-car mode)
    # e.g., /car1/dynamic_sqp_tuner_node or /dynamic_sqp_tuner_node
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
                                                                   rospy.get_param(f'dynamic_sqp_tuner_node/{param_name}', default))))
        else:
            # Single-car: check global /param first
            return rospy.get_param(f'/{param_name}',
                                   rospy.get_param(f'~{param_name}',
                                                   rospy.get_param(f'dynamic_sqp_tuner_node/{param_name}', default)))

    initial_config = {
        'evasion_dist': get_param_with_car_namespace('evasion_dist', 0.35),
        'obs_traj_tresh': get_param_with_car_namespace('obs_traj_tresh', 1.5),
        'spline_bound_mindist': get_param_with_car_namespace('spline_bound_mindist', 0.20),
        'lookahead_dist': get_param_with_car_namespace('lookahead_dist', 15.0),
        'avoidance_resolution': get_param_with_car_namespace('avoidance_resolution', 30),
        'back_to_raceline_before': get_param_with_car_namespace('back_to_raceline_before', 6.0),
        'back_to_raceline_after': get_param_with_car_namespace('back_to_raceline_after', 8.0),
        'avoid_static_obs': get_param_with_car_namespace('avoid_static_obs', False),
    }

    srv = Server(dyn_sqp_tunerConfig, callback)

    # Update server with initial configuration from ROS parameters
    srv.update_configuration(initial_config)

    rospy.loginfo(
        '[Planner] Dynamic SQP Server initialized with parameters from ROS param server')
    rospy.spin()
