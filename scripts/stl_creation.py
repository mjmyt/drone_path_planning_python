#!/usr/bin/env python3
# print working directory
from nav_msgs.msg import Path
import numpy as np
import rospy

from RigidBodyPlanners import Custom_robot_mesh, Fcl_checker

from catenaries.srv import CatLowestPoint, CatLowestPointResponse
from stl import mesh

import os
print(os.getcwd())


def get_cat_lowest_function():
    rospy.wait_for_service('catenary_lowest_point')
    try:
        catenary_lowest = rospy.ServiceProxy(
            'catenary_lowest_point', CatLowestPoint)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    return catenary_lowest


def client():
    drones_distance = 2
    print("drone_distance: ", drones_distance)
    L = 3
    theta = np.deg2rad(0)

    catenary_lowest = get_cat_lowest_function()
    mesh = Custom_robot_mesh(drones_distance, theta, L,
                             catenary_lowest, mesh_type="fcl")

    return mesh


if __name__ == "__main__":
    rospy.init_node("create_stl", anonymous=True)
    # initialize
    env_mesh_name = "src/drone_path_planning/resources/stl/env-scene-hole.stl"
    robot_mesh_name = "src/drone_path_planning/resources/stl/robot-scene-triangle.stl"

    coll_checker = Fcl_checker(env_mesh_name, robot_mesh_name)

    robot_mesh = client()

    # feed mesh to collision checker
    coll_checker.update_robot(robot_mesh.mesh)

    # check collision
    coll_checker.set_robot_transform([0, 0, 0], [0, 0, 0, 1])
    coll = coll_checker.check_collision()
    print("collision: ", coll)

    # mesh update
    drones_distance = 2
    print("drone_distance: ", drones_distance)
    robot_mesh.update_mesh(drones_distance, 0, 3)

    coll_checker.set_robot_transform([0, 0, 0], [0, 0, 0, 1])

    print(coll_checker.check_collision())
