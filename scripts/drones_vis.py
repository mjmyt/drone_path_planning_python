#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped, Quaternion
import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# print working directory
print("Current working directory:", os.getcwd())

if __name__ == "__main__":
    rospy.init_node("drones_vis_on_rb", anonymous=True)

    listener = tf.TransformListener()

    br = tf.TransformBroadcaster()

    pos1 = [1, 0, 1]
    pos2 = pos1.copy()
    pos2[0] = -pos2[0]

    orientation = (0, 0, 0, 1)

    rate = rospy.Rate(20.0)  # hz
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                '/world', '/rigid_body', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        br.sendTransform(pos1, orientation, rospy.Time.now(),
                         "drone1", "rigid_body")

        br.sendTransform(pos2, orientation, rospy.Time.now(),
                         "drone2", "rigid_body")

        rate.sleep()
