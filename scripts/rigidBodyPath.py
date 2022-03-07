#!/usr/bin/env python3
# print working directory
from drone_path_planning.msg import rigid_body_dynamic_path
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

from catenary import catenaries

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()


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


def generate_dynamic_path_msg(data):
    dynamic_path_msg = rigid_body_dynamic_path()

    drones_distances = np.zeros((len(data), 1))
    drones_angles = np.zeros((len(data), 1))

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
        path.poses.append(pose)

        drones_distances[i] = data[i, 4]
        # drones_angles[i] = data[i, 5]

    dynamic_path_msg.Path = path
    dynamic_path_msg.drones_distances = drones_distances
    dynamic_path_msg.drones_angles = drones_angles

    return dynamic_path_msg


def calculate_path_FCL():
    env_mesh_name = "env-scene-hole.stl"
    env_mesh_name = "env-scene-hole-narrow.stl"

    robot_mesh_name = "robot-scene-triangle.stl"

    planner = PlannerSepCollision(
        env_mesh_name, robot_mesh_name, catenaries.lowest_point_optimized)

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


def get_cat_lowest_function_service():
    print("Waiting for cat_lowest_function service...")
    rospy.wait_for_service('catenary_lowest_point')
    try:
        catenary_lowest = rospy.ServiceProxy(
            'catenary_lowest_point', CatLowestPoint)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    return catenary_lowest


def load_saved_path(filename='path.txt'):
    try:
        # get the file path of droen_path_planning package
        path_file = rospack.get_path(
            'drone_path_planning')+'/resources/paths/{}'.format(filename)
        data = np.loadtxt(path_file)
    except Exception as e:
        print("Error:", e)
        print("Trying crazyswarm/{}...".format(filename))
        try:
            data = np.loadtxt('crazyswarm/{}'.format(filename))
        except Exception as e:
            print("No path file found")
            exit(0)
    return data


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
    env.color.r, env.color.g, env.color.b = 1, 0, 0
    env.updatePose([0, 0, 0], [0, 0, 0, 1])
    envPub = rospy.Publisher('rb_environment',  Marker, queue_size=10)

    # calculate path
    print("Calculating path...")
    calculate_path_FCL()

    # path
    data = load_saved_path()

    # generate dynamic path msg
    # path = getPath(data)
    dynamic_path = generate_dynamic_path_msg(data)
    print("Loaded and generated dynamic path")

    trajPub = rospy.Publisher('rigiBodyPath',  Path, queue_size=10)
    trajPub.publish(dynamic_path.Path)

    dynamic_path_pub = rospy.Publisher(
        'dynamicRigiBodyPath', rigid_body_dynamic_path, queue_size=10)

    print("Waiting for connections to the  /dynamicRigiBodyPath topic...")
    while dynamic_path_pub.get_num_connections() == 0:
        pass

    print("Publishing dynamic path...")
    dynamic_path_pub.publish(dynamic_path)
    print("Published dynamic path!")

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

        # rb.updatePose(path.poses[i].pose.position,
        #               path.poses[i].pose.orientation, frame="world")

        rb.updatePose(dynamic_path.Path.poses[i].pose.position,
                      dynamic_path.Path.poses[i].pose.orientation, frame="world")

        # rb_transformed = transform(rb )

        # print("robot pos:", rb_transformed.pose.position.x,
        #   rb_transformed.pose.position.y, rb_transformed.pose.position.z, " orient:", rb_transformed.pose.orientation.x, rb_transformed.pose.orientation.y, rb_transformed.pose.orientation.z, rb_transformed.pose.orientation.w)

        robPub.publish(rb)

        pos = (rb.pose.position.x, rb.pose.position.y, rb.pose.position.z)
        orientation = (rb.pose.orientation.x, rb.pose.orientation.y,
                       rb.pose.orientation.z, rb.pose.orientation.w)

        br.sendTransform(pos, orientation, rospy.Time.now(),
                         "rigid_body", "world")

        envPub.publish(env)
        # trajPub.publish(path)
        trajPub.publish(dynamic_path.Path)

        rate.sleep()
