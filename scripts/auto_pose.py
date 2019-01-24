from __future__ import absolute_import, division, print_function
from builtins import *

import rospy
from nav_msgs import msg as nav
import tf


class AutoPose (object):
    def __init__(self, odom_topic):
        self.odom_topic = odom_topic
        self.odom_sub = rospy.Subscriber(odom_topic,
                nav.Odometry, self.odom_callback)
        self.first = None
        self.x = None
        self.y = None
        self.heading = None

    def odom_callback(self, msg):
        pose = msg.pose.pose
        if self.first is None:
            self.first = pose
        self.x = pose.position.x
        self.y = pose.position.y
        o = pose.orientation
        q = [o.x, o.y, o.z, o.w]
        _, _, self.heading = tf.transformations.euler_from_quaternion(q)
