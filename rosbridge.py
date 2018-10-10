from duckietown_slimremote.pc.robot import RemoteRobot
import rospy
from std_msgs.msg import Bool, String, Header
from sensor_msgs.msg import Image
import numpy as np
import os


class ROSBridge(object):
    def __init__(self):
        print("Hello!")
        host = os.getenv("DUCKIETOWN_SERVER", "localhost")
        # Create ZMQ connection
        self.sim = RemoteRobot(host, silent=False)
        self.action_sub = rospy.Subscriber('/1', String, self._action_cb)

        self.cam_pub = rospy.Publisher('/img', Image, queue_size=10)
        self.action_debug_pub = rospy.Publisher('/1', String, queue_size=10)
        self.action = np.array([0, 0])

        rospy.init_node('RemoteRobotRos')
        self.r = rospy.Rate(60)

    def _action_cb(self, msg):
        action = msg.data.split()
        assert len(action) == 2
        # print("Got action {}".format(action))
        self.action = np.array(action)
        

    def _publish_img(self, obs):
        # Hardcoded Implementation of ros_numpy's ImageConverter
        img_msg = Image(encoding='rgb8')
        img_msg.height, img_msg.width, _ = obs.shape
        contig = np.ascontiguousarray(obs)
        img_msg.data = contig.tostring()
        img_msg.step = contig.strides[0]
        img_msg.is_bigendian = (
            obs.dtype.byteorder == '>' or
            obs.dtype.byteorder == '=' and sys.byteorder == 'big'
        )

        time = rospy.get_rostime()
        img_msg.header.stamp.secs = time.secs
        img_msg.header.stamp.nsecs = time.nsecs
  
        self.cam_pub.publish(img_msg)

    def spin(self):
        while not rospy.is_shutdown():
            img, r , d, _ = self.sim.step(self.action)
            self._publish_img(img)
            self.r.sleep()

r = ROSBridge()
r.spin()
