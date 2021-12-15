from geometry_msgs.msg import TransformStamped, Quaternion
import math
import rospy
from tf import TransformListener, transformations
import numpy as np
from math import pi
import tf
from geometry_msgs.msg import PoseStamped

import tf2_ros
import tf2_geometry_msgs


def transform(pose_stamped, inverse=False) -> PoseStamped:
    buffer_core = tf2_ros.BufferCore(rospy.Duration(10.0))

    ts2 = TransformStamped()
    ts2.header.stamp = rospy.Time(0)
    ts2.header.frame_id = 'map'
    ts2.child_frame_id = 'ompl_base'
    ts2.transform.translation.x = 0
    ts2.transform.translation.y = 0
    ts2.transform.translation.z = 0

    if inverse:
        quat = tf.transformations.quaternion_from_euler(+math.pi/2, 0, 0)
    else:
        quat = tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0)

    ts2.transform.rotation.x = quat[0]
    ts2.transform.rotation.y = quat[1]
    ts2.transform.rotation.z = quat[2]
    ts2.transform.rotation.w = quat[3]

    buffer_core.set_transform(ts2, "default_authority")

    pose_transformed = tf2_geometry_msgs.do_transform_pose(
        pose_stamped, ts2)

    # print("initilal pose")
    # print(pose_stamped)
    # print("pose_transformed")
    # print(pose_transformed)

    # pose_transformed.pose.orientation.x = pose_stamped.pose.orientation.x
    # pose_transformed.pose.orientation.y = pose_stamped.pose.orientation.y
    # pose_transformed.pose.orientation.z = pose_stamped.pose.orientation.z
    # pose_transformed.pose.orientation.w = pose_stamped.pose.orientation.w
    return pose_transformed
