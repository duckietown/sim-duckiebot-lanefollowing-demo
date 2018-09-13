from duckietown_slimremote.networking import make_sub_socket, make_pub_socket, construct_action, recv_array, \
    get_ip, recv_gym
from duckietown_slimremote.helpers import random_id, timer

import time

import zmq

from duckietown_slimremote.helpers import random_id, get_py_version

import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
import numpy as np


class ROSBridge(object):
    def __init__(self):
        self.context = zmq.Context()

        self.image_sub = make_sub_socket(target="gym-duckietown-server", for_images=True)
        self.action_pub = make_pub_socket(context_=self.context)

        self.poller = zmq.Poller()
        self.poller.register(self.image_sub, zmq.POLLIN)

        self.own_id = random_id()
        self.own_ip = get_ip()
        msg = construct_action(self.own_id, self.own_ip)

        time.sleep(1)  # wait for sock init

        print("sending init", msg)

        # init
        self.action_pub.send_string(msg)

        self.action_sub = rospy.Subscriber('/1', String, self._action_cb)
        # self.reset_sub = rospy.Subscriber('/2', Bool, self._reset_cb)

        self.cam_pub = rospy.Publisher('/img', Image, queue_size=10)
        self.action = construct_action(self.own_id, self.own_ip, [0, 0])

        rospy.init_node('RemoteRobotRos')
        self.r = rospy.Rate(60)

    def _action_cb(self, msg):
        action = msg.data.split()
        assert len(action) == 2

        self.action = construct_action(self.own_id, self.own_ip, action)
        

    def _publish_img(self, obs):
        # Hardcoded Implementation of ros_numpy's ImageConverter
        img_msg = Image(encoding='uint8')
        img_msg.height, img_msg.width, _ = obs.shape
        contig = np.ascontiguousarray(obs)
        img_msg.data = contig.tostring()
        img_msg.step = contig.strides[0]
        img_msg.is_bigendian = (
            obs.dtype.byteorder == '>' or
            obs.dtype.byteorder == '=' and sys.byteorder == 'big'
        )
        
        self.cam_pub.publish(img_msg)

    def spin(self):
        while not rospy.is_shutdown():
            self.action_pub.send_string(self.action)
            socks = dict(self.poller.poll(5))
            if self.image_sub in socks and socks[self.image_sub] == zmq.POLLIN:
                img, r , d , _ = recv_gym(self.image_sub)
                self._publish_img(img)

            self.r.sleep()

r = ROSBridge()
r.spin()
