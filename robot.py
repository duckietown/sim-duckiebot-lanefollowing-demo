#!/usr/bin/env python

import rospy

from duckietown_slimremote.pc.robot import RemoteRobot

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image


class RemoteRobotRos(RemoteRobot):
    def __init__(self, host):
        super().__init__(host)

        self.action_sub = rospy.Subscriber('/1', String, self._action_cb)
        self.reset_sub = rospy.Subscriber('/2', Bool, self._reset_cb)

        self.cam_pub = rospy.Publisher('/img', Image, queue_size=10)
        rospy.init_node('RemoteRobotRos')


    def query_camera(self):
        img = None #self.cam.get_img_nonblocking()
        if img is not None:
            pass # How do we send the image?
        else:
            pass


    def _action_cb(self, msg):
        action = msg.data.split()
        self.step(action, with_observation=False)


    def _reset_cb(self, msg):
        if msg.data == True:
            self.reset()


if __name__ == '__main__':
    r = RemoteRobotRos(10)
    rate = rospy.Rate(60) #60Hz
    while not rospy.is_shutdown():
        r.query_camera()
        rate.sleep()
