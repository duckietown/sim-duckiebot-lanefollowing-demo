import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
import numpy as np

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

    action_debug_pub = rospy.Publisher('/1', String, queue_size=10)
    rospy.init_node('ActionServer')
    while True:
        action = "{} {}".format(np.random.random(), np.random.random())
        action_debug_pub.publish(action)
        # env.step([np.random.random(), np.random.random()])

if __name__ == '__main__':
    main()