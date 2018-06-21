import tkinter
import numpy as np
from PIL import ImageTk, Image

from duckietown_slimremote.helpers import random_id
from duckietown_slimremote.networking import make_push_socket, construct_action
from duckietown_slimremote.pc.camera import SubCameraMaster
from duckietown_slimremote.pc.robot import RemoteRobot

import rospy
import ros_numpy
import threading
from std_msgs import Bool, String



class RemoteRobotRos(RemoteRobot):
    def __init__(self, host):
        super.__init__(host)

        self.action_sub = rospy.Subscriber('/1', String, self._action_cb)
        self.reset_sub = rospy.Subscriber('/2', Bool, self._reset_cb)

        self.cam_pub = rospy.Publisher('/img', pass, queue_size=10)
        self.camera_running = False


    def _action_cb(self, msg):
        action = msg.data.split()
        self.step(action, with_observation=False)


    def _reset_cb(self, msg):
        if msg.data == True:
            self.reset()


    def _query_camera(self):
        while self.camera_running:
            img = self.cam.get_img_nonblocking(self):
            if img is not None:
                pass # How do we send the image?




    def _camera_setup(self):
        self.camera_running = True
        sub_thread = threading.Thread(target=self.query_camera)
        sub_thread.daemon = True
        sub_thread.start()

        return sub_thread


    def __del__(self):
        self.camera_running = False





class RemoteRobot():
    def __init__(self, host):
        self.host = host

        self.id = random_id()
        self.ping_msg = construct_action(self.id)
        self.robot_sock = make_push_socket(host)
        self.robot_sock.send_string(self.ping_msg)

        self.cam = SubCameraMaster(host)

    def step(self, action, with_observation=True):
        assert len(action) == 2
        msg = construct_action(self.id, action=action)

        # run action on robot
        self.robot_sock.send_string(msg)
        print("sent action:",msg)

        # return last known camera image #FIXME: this must be non-blocking and re-send ping if necessary
        if with_observation:
            return self.cam.get_img_blocking()
        else:
            return None

    def observe(self):
        return self.cam.get_img_blocking()

    def reset(self):
        # This purposefully doesn't do anything on the real robot (other than halt).
        # But in sim this obviously resets the simulation
        return self.step([0, 0])
