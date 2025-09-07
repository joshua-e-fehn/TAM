#!/usr/bin/env python3
import rospy
import sys
import os
from dynamic_reconfigure.server import Server

# Add the devel path to PYTHONPATH
if 'ROS_PACKAGE_PATH' in os.environ:
    catkin_ws = None
    for path in os.environ['ROS_PACKAGE_PATH'].split(':'):
        if 'catkin_ws' in path:
            catkin_ws = path.split('/src')[0]
            break

    if catkin_ws:
        devel_path = os.path.join(
            catkin_ws, 'devel', 'lib', 'python3', 'dist-packages')
        if devel_path not in sys.path:
            sys.path.insert(0, devel_path)

# Import the configuration
try:
    from obstacle_publisher.cfg import dyn_obs_publisherConfig
    rospy.loginfo("Successfully imported dynamic reconfigure config")
except ImportError as e:
    rospy.logerr(f"Failed to import dynamic reconfigure config: {e}")
    rospy.logerr("Trying alternative import method...")

    # Alternative: try to import directly
    try:
        import importlib.util
        config_file = "/home/atlas/catkin_ws/devel/lib/python3/dist-packages/obstacle_publisher/cfg/dyn_obs_publisherConfig.py"
        spec = importlib.util.spec_from_file_location(
            "dyn_obs_publisherConfig", config_file)
        dyn_obs_publisherConfig = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(dyn_obs_publisherConfig)
        rospy.loginfo("Successfully imported config via alternative method")
    except Exception as e2:
        rospy.logerr(f"Alternative import also failed: {e2}")
        exit(1)


def callback(config, level):
    config.speed_scaler = config.speed_scaler
    config.ampl_sin1 = config.ampl_sin1
    config.ampl_sin2 = config.ampl_sin2
    config.phase_sin1 = config.phase_sin1
    config.phase_sin2 = config.phase_sin2
    config.path_amplitude = config.path_amplitude
    config.path_frequency = config.path_frequency
    config.path_phase = config.path_phase
    print("[Dynamic Reconfigure] Parameters updated:")
    print(f"  speed_scaler: {config.speed_scaler}")
    print(f"  path_amplitude: {config.path_amplitude}")
    print(f"  path_frequency: {config.path_frequency}")
    print(f"  path_phase: {config.path_phase}")
    return config


if __name__ == "__main__":
    rospy.init_node("dynamic_obstacle_publisher_node", anonymous=False)
    print('[Obs. Publisher] Dynamic Obstacle Publisher Server Launched...')
    srv = Server(dyn_obs_publisherConfig, callback)
    rospy.spin()
