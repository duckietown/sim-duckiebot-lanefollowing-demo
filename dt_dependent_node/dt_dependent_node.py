#!/usr/bin/env python

import rospy
import numpy as np

from duckietown_msgs.msg import WheelsCmdStamped

def continuous_publisher():
    vel_pub = rospy.Publisher('/random/topic/', WheelsCmdStamped, queue_size=1)
    rospy.init_node('continuous_test_publisher')
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        msg = WheelsCmdStamped()
        msg.vel_left = np.random.random()
        msg.vel_right = np.random.random()

        vel_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    continuous_publisher()


