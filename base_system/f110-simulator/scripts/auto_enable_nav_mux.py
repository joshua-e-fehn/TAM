#!/usr/bin/env python3
"""
Auto-enable navigation mux for single-car mode.
This ensures the mux controller selects the navigation input.
"""

import rospy
from std_msgs.msg import Int32MultiArray
import time

if __name__ == '__main__':
    rospy.init_node('auto_enable_nav_mux', anonymous=False)

    # Wait 2 seconds for all relays to be ready
    rospy.loginfo("Waiting 2 seconds for relays to initialize...")
    time.sleep(2.0)

    # Publisher for mux control
    mux_pub = rospy.Publisher('/mux_controller/mux',
                              Int32MultiArray, queue_size=10)

    # Wait a bit more for publisher to establish connection
    rospy.sleep(0.5)

    # Create mux message: [joy, nav, keyboard, random_walk, brake]
    # Enable navigation (index 1)
    mux_msg = Int32MultiArray()
    mux_msg.data = [0, 1, 0, 0, 0]

    rospy.loginfo("Auto-enabling navigation mux...")

    # Publish at 10 Hz to ensure it stays enabled
    rate = rospy.Rate(10)  # 10 Hz

    # Only log the first time to avoid terminal spam
    first_publish = True

    while not rospy.is_shutdown():
        mux_pub.publish(mux_msg)

        # Log only once for debugging (comment out to disable completely)
        # if first_publish:
        #     rospy.loginfo_throttle(5.0, f"Mux enabled with: {mux_msg.data}")
        #     first_publish = False

        rate.sleep()
