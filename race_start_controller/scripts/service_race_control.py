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

        # Get car namespaces from parameters (supports both single and multi-car modes)
        self.car1_namespace = rospy.get_param('~car1_namespace', 'car1')
        self.car2_namespace = rospy.get_param('~car2_namespace', 'car2')

        # Check if we're in single-car mode (no namespace prefix)
        self.single_car_mode = rospy.get_param('~single_car_mode', False)

        # Publishers for each car's state machine
        if self.single_car_mode:
            # Single car mode - publish to root namespace
            self.car1_state_pub = rospy.Publisher(
                '/state_machine_cmd', String, queue_size=1)
            self.car2_state_pub = None  # No second car in single mode
            rospy.loginfo("Race Control initialized in SINGLE-CAR mode")
        else:
            # Multi-car mode - publish to namespaced topics
            self.car1_state_pub = rospy.Publisher(
                f'/{self.car1_namespace}/state_machine_cmd', String, queue_size=1)
            self.car2_state_pub = rospy.Publisher(
                f'/{self.car2_namespace}/state_machine_cmd', String, queue_size=1)
            rospy.loginfo(
                f"Race Control initialized in MULTI-CAR mode ({self.car1_namespace}, {self.car2_namespace})")

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
        if self.single_car_mode:
            rospy.loginfo(
                "Commands: rosservice call /race_control/{start_both,reset_cars,emergency_stop}")
        else:
            rospy.loginfo(
                "Commands: rosservice call /race_control/{start_car1,start_car2,start_both,reset_cars,emergency_stop}")

    def start_car1_service(self, req):
        if self.single_car_mode:
            rospy.loginfo("Starting Car (single-car mode)")
            self.car1_state_pub.publish(String("GB_TRACK"))
        else:
            rospy.loginfo("Starting Car1")
            self.car1_state_pub.publish(String("GB_TRACK"))
        return EmptyResponse()

    def start_car2_service(self, req):
        if self.single_car_mode:
            rospy.logwarn("start_car2 called in single-car mode - ignoring")
        else:
            rospy.loginfo("Starting Car2")
            self.car2_state_pub.publish(String("GB_TRACK"))
        return EmptyResponse()

    def start_both_service(self, req):
        if self.single_car_mode:
            rospy.loginfo("RACE START (single-car mode)")
            self.car1_state_pub.publish(String("GB_TRACK"))
        else:
            rospy.loginfo("RACE START - Both Cars")
            self.car1_state_pub.publish(String("GB_TRACK"))
            self.car2_state_pub.publish(String("GB_TRACK"))
        return EmptyResponse()

    def reset_cars_service(self, req):
        if self.single_car_mode:
            rospy.loginfo("Resetting car to READY")
            self.car1_state_pub.publish(String("READY"))
        else:
            rospy.loginfo("Resetting cars to READY")
            self.car1_state_pub.publish(String("READY"))
            self.car2_state_pub.publish(String("READY"))
        return EmptyResponse()

    def emergency_stop_service(self, req):
        if self.single_car_mode:
            rospy.logwarn("EMERGENCY STOP")
            self.car1_state_pub.publish(String("READY"))
        else:
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
