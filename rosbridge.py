from duckietown_slimremote.pc.robot import RemoteRobot
import rospy
from std_msgs.msg import Bool, String, Header
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import numpy as np
import os
import cv2


class ROSBridge(object):
    def __init__(self):
        print("Hello!")
        host = os.getenv("DUCKIETOWN_SERVER", "localhost")
        # Create ZMQ connection
        self.sim = RemoteRobot(host, silent=False)
        self.action_sub = rospy.Subscriber('/1', Twist2DStamped, self._action_cb)

        self.cam_pub = rospy.Publisher('/corrected_image/compressed', CompressedImage, queue_size=10)
        self.action = np.array([0, 0])

        rospy.init_node('RemoteRobotRos')
        self.r = rospy.Rate(60)

    def _action_cb(self, msg):
        v = msg.v
        omega = msg.omega
        self.action = np.array([v, omega])
        

    def _publish_img(self, obs):
        img_msg = CompressedImage()

        time = rospy.get_rostime()
        img_msg.header.stamp.secs = time.secs
        img_msg.header.stamp.nsecs = time.nsecs

        img_msg.format = "jpeg"
        contig = np.ascontiguousarray(np.roll(obs, 2, axis=-1))
        img_msg.data = np.array(cv2.imencode('.jpg', contig)[1]).tostring()
  
        self.cam_pub.publish(img_msg)

    def spin(self):
        while not rospy.is_shutdown():
            img, r , d, _ = self.sim.step(self.action)
            self._publish_img(img)
            self.r.sleep()

r = ROSBridge()
r.spin()
