#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from predictive_spliner.cfg import dyn_collision_tunerConfig


def callback(config, level):
    config.n_time_steps = round(config.n_time_steps)
    config.dt = config.dt
    config.save_distance_front = round(config.save_distance_front, 2)
    config.save_distance_back = round(config.save_distance_back, 2)
    config.max_v = round(config.max_v, 2)
    config.min_v = round(config.min_v, 2)
    config.max_a = round(config.max_a, 2)
    config.min_a = round(config.min_a, 2)
    config.max_expire_counter = round(config.max_expire_counter)
    config.update_waypoints = config.update_waypoints
    config.speed_offset = round(config.speed_offset, 3)
    return config


if __name__ == "__main__":
    rospy.init_node("dynamic_collision_tuner_node", anonymous=False)
    print('[Planner] Dynamic Collision Server Launched...')

    # Read initial values from ROS parameters (set by test framework or launch files)
    # This allows test framework to control initial configuration
    # Priority order for single-car: 1) Global params, 2) Node-private, 3) Namespaced, 4) Defaults
    # Priority order for multi-car: 1) Car-namespaced params (/car1/param), 2) Global, 3) Node-private, 4) Namespaced, 5) Defaults

    # Detect if we're in a car namespace (multi-car mode)
    # e.g., /car1/dynamic_collision_tuner_node or /dynamic_collision_tuner_node
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
                                                                   rospy.get_param(f'dynamic_collision_tuner_node/{param_name}', default))))
        else:
            # Single-car: check global /param first
            return rospy.get_param(f'/{param_name}',
                                   rospy.get_param(f'~{param_name}',
                                                   rospy.get_param(f'dynamic_collision_tuner_node/{param_name}', default)))

    initial_config = {
        'n_time_steps': get_param_with_car_namespace('n_time_steps', 400),
        'dt': get_param_with_car_namespace('dt', 0.02),
        'save_distance_front': get_param_with_car_namespace('save_distance_front', 0.6),
        'save_distance_back': get_param_with_car_namespace('save_distance_back', 0.6),
        'max_v': get_param_with_car_namespace('max_v', 10.0),
        'min_v': get_param_with_car_namespace('min_v', 0.0),
        'max_a': get_param_with_car_namespace('max_a', 7.0),
        'min_a': get_param_with_car_namespace('min_a', 5.0),
        'max_expire_counter': get_param_with_car_namespace('max_expire_counter', 10),
        'update_waypoints': get_param_with_car_namespace('update_waypoints', True),
        'speed_offset': get_param_with_car_namespace('speed_offset', 0.0),
    }

    srv = Server(dyn_collision_tunerConfig, callback)

    # Update server with initial configuration from ROS parameters
    srv.update_configuration(initial_config)

    rospy.loginfo(
        '[Planner] Dynamic Collision Server initialized with parameters from ROS param server')
    rospy.spin()
