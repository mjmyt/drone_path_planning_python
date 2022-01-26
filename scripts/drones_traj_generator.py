#!/usr/bin/env python3
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Quaternion
import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf
import os
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Point
import math

import tf2_ros
import tf2_geometry_msgs

# print working directory
print("Current working directory:", os.getcwd())


drone_positions = [
    [1, 0, 1],
    [-1, 0, 1]
]


drone_pose = PoseStamped()
drone_pose.header.frame_id = 'rb_path'
drone_pose.pose.position = Point(1, 0, 1)
drone_pose.pose.orientation = Quaternion(0, 0, 0, 1)

drone_pose2 = PoseStamped()
drone_pose2.header.frame_id = 'rb_path'
drone_pose2.pose.position = Point(-1, 0, 1)
drone_pose2.pose.orientation = Quaternion(0, 0, 0, 1)


def get_drone_positions(drone_positions):
    drone_poses = []
    drone_pose = PoseStamped()
    drone_pose.header.frame_id = 'rb_path'
    drone_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    for pos in drone_positions:
        drone_pose.pose.position.x = pos[0]
        drone_pose.pose.position.y = pos[1]
        drone_pose.pose.position.z = pos[2]

        drone_poses.append(drone_pose)
    return drone_poses


trajPub1 = rospy.Publisher('drone1Path',  Path, queue_size=10)
trajPub2 = rospy.Publisher('drone2Path',  Path, queue_size=10)


def transform(path, inverse=False) -> PoseStamped:
    buffer_core = tf2_ros.BufferCore(rospy.Duration(10.0))

    path1 = Path()
    path1.header.frame_id = "world"
    path1.header.stamp = rospy.get_rostime()

    path2 = Path()
    path2.header.frame_id = "world"
    path2.header.stamp = rospy.get_rostime()

    for i, rb_pose in enumerate(path.poses):
        ts2 = TransformStamped()
        ts2.header.stamp = rospy.Time(0)
        ts2.header.frame_id = 'world'
        ts2.child_frame_id = 'rb_path'
        ts2.transform.translation = rb_pose.pose.position
        ts2.transform.rotation = rb_pose.pose.orientation

        buffer_core.set_transform(ts2, "default_authority")

        pose_transformed = tf2_geometry_msgs.do_transform_pose(drone_pose, ts2)
        path1.poses.append(pose_transformed)

        pose_transformed2 = tf2_geometry_msgs.do_transform_pose(drone_pose2, ts2)
        path2.poses.append(pose_transformed2)

        # print("rb pose")
        # print(rb_pose)
        # print("pose_transformed")
        # print(pose_transformed)

    return path1, path2


def callback(path: Path):
    print("Path received...")
    print(len(path.poses))
    drone_path1, drone_path2 = transform(path)

    trajPub1.publish(drone_path1)
    trajPub2.publish(drone_path2)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('rb_path_listener', anonymous=True)

    rospy.Subscriber('rigiBodyPath', Path, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
