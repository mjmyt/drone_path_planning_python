#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped, Quaternion
import rospy
import numpy as np
from math import pi
import tf
from nav_msgs.msg import Path
import os
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Point
from optimizations import *

# print working directory
print("Current working directory:", os.getcwd())


def callback(path: Path):
    print("Path received...")
    # print(len(path.poses))
    callback.counter += 1
    if callback.counter > 1:
        return 0

    total_duration = 10  # secs
    n = len(path.poses)
    time_step = total_duration / n

    traj_points = []
    for i, pose in enumerate(path.poses):
        x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        q = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        yaw = tf.transformations.euler_from_quaternion(q)[2]

        print(x, y, z, q[2])

        traj_points.append(Point_time(Waypoint(x, y, z, yaw), t=time_step*i))

    calculate_trajectory4D(traj_points)


callback.counter = 0


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('drones_path_listener', anonymous=True)

    # rospy.Subscriber('drone1Path',  Path, callback)
    rospy.Subscriber('drone2Path',  Path, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
