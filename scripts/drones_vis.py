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


def main_without_dyn_planning():
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


class Drone_path_vis:
    def __init__(self, id):
        self.id = id
        self.pose_counter = 0

    def receive_path(self, msg):
        print("Received path")
        self.path = msg
        self.path_received = True

    def broadcast_drone_pose(self, index=None):
        if index is None:
            index = self.pose_counter

        drone_pose = self.path.poses[index]
        pos = drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z
        rot = drone_pose.pose.orientation.x, drone_pose.pose.orientation.y, drone_pose.pose.orientation.z, drone_pose.pose.orientation.w

        br.sendTransform(pos, rot, rospy.Time.now(),
                         "drone" + str(self.id), "world")

        if self.pose_counter >= len(self.path.poses)-1:
            self.pose_counter = 0
        else:
            self.pose_counter = index+1


def main_with_dyn_planning():
    drone1 = Drone_path_vis(1)
    drone2 = Drone_path_vis(2)

    drone1_path_sub = rospy.Subscriber(
        "/drone1Path", Path, drone1.receive_path)
    drone2_path_sub = rospy.Subscriber(
        "/drone2Path", Path, drone2.receive_path)

    rate = rospy.Rate(10.0)  # hz
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                '/world', '/rigid_body', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        drone1.broadcast_drone_pose()
        drone2.broadcast_drone_pose()

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("drones_vis_on_rb", anonymous=True)

    listener = tf.TransformListener()

    br = tf.TransformBroadcaster()

    # main_without_dyn_planning()
    main_with_dyn_planning()
