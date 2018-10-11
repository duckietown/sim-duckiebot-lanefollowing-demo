import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
import numpy as np
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped

import gym
import gym_duckietown_agent  # don't remove this line


def main():
    # Create the gym environment
    # env = gym.make("Duckietown-Lf-Lfv-Navv-Silent-v0")

    # Initialize. This is mainly here because it follows the
    # gym convention. There are however few cases where the
    # simulation is already running and you might wanna call
    # this first to make sure the sim is reset.
    # env.reset()

    action_debug_pub = rospy.Publisher('/1', Twist2DStamped, queue_size=10)
    rospy.init_node('ActionServer')
    action = Twist2DStamped()
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        action.v = np.random.random()
        action.omega = np.random.random()
        action.header.stamp = rospy.Time.now()
        
        action_debug_pub.publish(action)
    
        r.sleep()

if __name__ == '__main__':
    main()