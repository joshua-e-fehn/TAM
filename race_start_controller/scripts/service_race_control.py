#!/usr/bin/env python3
"""
Service-based Race Start Controller
Clean ROS service interface - no terminal formatting issues.

Usage:
    python3 service_race_control.py
    
    # Control with services:
    rosservice call /race_control/start_car1
    rosservice call /race_control/start_car2  
    rosservice call /race_control/start_both
    rosservice call /race_control/reset_cars
    rosservice call /race_control/emergency_stop
"""

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse


class ServiceRaceController:
    def __init__(self):
        rospy.init_node('service_race_controller', anonymous=False)

        # Publishers for each car's state machine
        self.car1_state_pub = rospy.Publisher(
            '/car1/state_machine_cmd', String, queue_size=1)
        self.car2_state_pub = rospy.Publisher(
            '/car2/state_machine_cmd', String, queue_size=1)

        # ROS Services for clean control
        rospy.Service('/race_control/start_car1',
                      Empty, self.start_car1_service)
        rospy.Service('/race_control/start_car2',
                      Empty, self.start_car2_service)
        rospy.Service('/race_control/start_both',
                      Empty, self.start_both_service)
        rospy.Service('/race_control/reset_cars',
                      Empty, self.reset_cars_service)
        rospy.Service('/race_control/emergency_stop',
                      Empty, self.emergency_stop_service)

        rospy.loginfo("Race Control Services Ready!")
        rospy.loginfo(
            "Commands: rosservice call /race_control/{start_car1,start_car2,start_both,reset_cars,emergency_stop}")

    def start_car1_service(self, req):
        rospy.loginfo("Starting Car1")
        self.car1_state_pub.publish(String("GB_TRACK"))
        return EmptyResponse()

    def start_car2_service(self, req):
        rospy.loginfo("Starting Car2")
        self.car2_state_pub.publish(String("GB_TRACK"))
        return EmptyResponse()

    def start_both_service(self, req):
        rospy.loginfo("RACE START - Both Cars")
        self.car1_state_pub.publish(String("GB_TRACK"))
        self.car2_state_pub.publish(String("GB_TRACK"))
        return EmptyResponse()

    def reset_cars_service(self, req):
        rospy.loginfo("Resetting cars to READY")
        self.car1_state_pub.publish(String("READY"))
        self.car2_state_pub.publish(String("READY"))
        return EmptyResponse()

    def emergency_stop_service(self, req):
        rospy.logwarn("EMERGENCY STOP")
        self.car1_state_pub.publish(String("READY"))
        self.car2_state_pub.publish(String("READY"))
        return EmptyResponse()


if __name__ == '__main__':
    try:
        controller = ServiceRaceController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
