#!/usr/bin/env python3
# print working directory
from geometry_msgs.msg import TransformStamped, Quaternion
import math
import rospy
from tf import TransformListener, transformations
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from RigidBodyPlanners import *
import tf2_ros
import tf2_geometry_msgs

from ompl import base as ob
from ompl import geometric as og

from catenaries.srv import CatLowestPoint, CatLowestPointResponse


print("Current working directory:", os.getcwd())
DRONES_NUMBER = 5


class MeshMarker(Marker):
    def __init__(self, id, mesh_path, pos=[0, 0, 0], rot=[0, 0, 0, 1]):
        super().__init__()
        self.header.frame_id = "world"
        self.header.stamp = rospy.get_rostime()
        self.ns = "collada_mesh"
        self.id = id
        self.type = Marker.MESH_RESOURCE
        self.mesh_resource = mesh_path
        self.action = 0

        self.updatePose(pos, rot)

        scale_fac = 1
        self.scale.x = scale_fac
        self.scale.y = scale_fac
        self.scale.z = scale_fac

        self.color.r = 0.0
        self.color.g = 1.0
        self.color.b = 0.0
        self.color.a = 1.0

        self.lifetime = rospy.Duration(0)

    def updatePose(self, pos, quatern, frame="world"):
        try:
            self.pose.position.x = pos[0]
            self.pose.position.y = pos[1]
            self.pose.position.z = pos[2]

            self.pose.orientation.x = quatern[0]
            self.pose.orientation.y = quatern[1]
            self.pose.orientation.z = quatern[2]
            self.pose.orientation.w = quatern[3]
        except:
            self.pose.position = pos
            self.pose.orientation = quatern


def getPath(data):
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.get_rostime()
    path.poses = []
    print("Path size:", len(data))
    for i in range(data.shape[0]):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.get_rostime()

        pose.pose.position.x = data[i, 0]
        pose.pose.position.y = data[i, 1]
        pose.pose.position.z = data[i, 2]

        q = tf.quaternion_from_euler(0, 0, data[i, 3])
        pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        # transformed_pose = transform(pose,  inverse=True)
#
        # pose.pose.orientation.x = data[i, 3]
        # pose.pose.orientation.y = data[i, 4]
        # pose.pose.orientation.z = data[i, 5]
        # pose.pose.orientation.w = data[i, 6]

        path.poses.append(pose)

    return path


def calculate_path():
    # planner = RBPlanner()

    planner = PlannerSepCollision()
    start = [2, -4, 0]
    goal = [2, 5, 0]

    start_pose = PoseStamped()
    start_pose.pose.position.x = start[0]
    start_pose.pose.position.y = start[1]
    start_pose.pose.position.z = start[2]
    q = tf.quaternion_from_euler(math.pi/2, 0, 0)
    start_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    goal_pose = PoseStamped()
    goal_pose.pose.position.x = goal[0]
    goal_pose.pose.position.y = goal[1]
    goal_pose.pose.position.z = goal[2]
    q = tf.quaternion_from_euler(math.pi/2, 0, 0)
    goal_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    start_pose_transformed = transform(start_pose)
    goal_pose_transformed = transform(goal_pose)

    print("==============================")
    print("START POSE:")
    print(start_pose_transformed)
    print("==============================")
    print("GOAL POSE:")
    print(goal_pose_transformed)
    print("==============================")
    planner.set_start_goal(start_pose_transformed.pose,
                           goal_pose_transformed.pose)
    planner.set_planner()
    # planner.set_planner(og.FMT)

    planner.solve(timeout=20.0)
    # planner.visualize_path()


def calculate_path_FCL():
    env_mesh_name = "env-scene-hole.stl"
    env_mesh_name = "env-scene-hole-narrow.stl"

    robot_mesh_name = "robot-scene-triangle.stl"
    planner = PlannerSepCollision(
        env_mesh_name, robot_mesh_name, get_cat_lowest_function())

    start = [-2, 5, 0]
    goal = [-2, -3, 0]

    start_pose = PoseStamped()
    start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z = start
    q = tf.quaternion_from_euler(0, 0, 0)
    start_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    goal_pose = PoseStamped()
    goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z = goal
    q = tf.quaternion_from_euler(0, 0, 0)
    goal_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    print("==============================")
    print("START POSE:")
    print(start_pose)
    print("==============================")
    print("GOAL POSE:")
    print(goal_pose)
    print("==============================")
    planner.set_start_goal(start_pose.pose, goal_pose.pose)
    planner.set_planner()
    # planner.set_planner(og.FMT)

    planner.solve(timeout=40.0)
    # planner.visualize_path()


def get_cat_lowest_function():
    print("Waiting for cat_lowest_function service...")
    rospy.wait_for_service('catenary_lowest_point')
    try:
        catenary_lowest = rospy.ServiceProxy(
            'catenary_lowest_point', CatLowestPoint)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    return catenary_lowest


if __name__ == "__main__":

    rospy.init_node("rb_path_planning")
    # transform()

    # robot marker initialization
    mesh = "package://drone_path_planning/resources/collada/robot-scene-triangle.dae"
    rb = MeshMarker(id=0, mesh_path=mesh)

    robPub = rospy.Publisher('rb_robot',  Marker, queue_size=10)

    # Environment marker initialization
    mesh = "package://drone_path_planning/resources/collada/env-scene-hole-narrow.dae"
    env = MeshMarker(id=1, mesh_path=mesh)
    env.color.r = 1
    env.color.g = 0
    env.color.b = 0
    env.updatePose([0, 0, 0], [0, 0, 0, 1])
    envPub = rospy.Publisher('rb_environment',  Marker, queue_size=10)

    # calculate path
    print("Calculating path...")
    # calculate_path()
    calculate_path_FCL()

    # path
    try:
        data = np.loadtxt('path.txt')
    except Exception as e:
        print("Error:", e)
        print("Trying crazyswarm/path.txt...")
        try:
            data = np.loadtxt('crazyswarm/path.txt')
        except Exception as e:
            print("No path file found")
            exit(0)

    path = getPath(data)
    trajPub = rospy.Publisher('rigiBodyPath',  Path, queue_size=10)
    trajPub.publish(path)

    # transform
    br = tf.TransformBroadcaster()

    i = 0
    rate = rospy.Rate(10.0)  # hz
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0), rospy.Time.now(),
                         "world", "ompl")

        if i == data.shape[0]-1:
            i = 0
        else:
            i += 1

        rb.updatePose(path.poses[i].pose.position,
                      path.poses[i].pose.orientation, frame="world")

        # rb_transformed = transform(rb)

        # print("robot pos:", rb_transformed.pose.position.x,
        #   rb_transformed.pose.position.y, rb_transformed.pose.position.z, " orient:", rb_transformed.pose.orientation.x, rb_transformed.pose.orientation.y, rb_transformed.pose.orientation.z, rb_transformed.pose.orientation.w)

        robPub.publish(rb)

        pos = (rb.pose.position.x, rb.pose.position.y, rb.pose.position.z)
        orientation = (rb.pose.orientation.x, rb.pose.orientation.y,
                       rb.pose.orientation.z, rb.pose.orientation.w)

        br.sendTransform(pos, orientation, rospy.Time.now(),
                         "rigid_body", "world")

        envPub.publish(env)
        trajPub.publish(path)

        rate.sleep()
