#!/usr/bin/env python3
"""
Dynamic Reconfigure Server for TAM Sampling Planner
Allows real-time parameter tuning following the pattern of other planners
"""

import rospy
from dynamic_reconfigure.server import Server
from tam_sampling_planner.cfg import TAMSamplingTunerConfig


class TAMSamplingDynamicReconfigureServer:
    """Dynamic reconfigure server for TAM sampling planner parameters"""

    def __init__(self):
        rospy.init_node('dynamic_tam_sampling_tuner_node')

        # Initialize the dynamic reconfigure server
        self.server = Server(TAMSamplingTunerConfig, self.reconfigure_callback)

        rospy.loginfo("TAM Sampling Dynamic Reconfigure Server started")
        rospy.spin()

    def reconfigure_callback(self, config, level):
        """Handle parameter updates"""
        rospy.loginfo(f"TAM Sampling parameters updated: {config}")
        return config


if __name__ == "__main__":
    try:
        TAMSamplingDynamicReconfigureServer()
    except rospy.ROSInterruptException:
        pass
