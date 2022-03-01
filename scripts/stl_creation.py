#!/usr/bin/env python3
# print working directory
from nav_msgs.msg import Path
import numpy as np
import rospy

from RigidBodyPlanners import Custom_robot_mesh

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
    drones_distance = 1
    print("drone_distance: ", drones_distance)
    L = 3
    theta = np.deg2rad(0)

    catenary_lowest = get_cat_lowest_function()
    mesh = Custom_robot_mesh(drones_distance, theta, L,
                             catenary_lowest, mesh_type="stl")

    input("Hit any key to continue...")

    drones_distance = 2
    print("drone_distance: ", drones_distance)
    mesh.update_mesh(drones_distance, theta, L)


if __name__ == "__main__":

    rospy.init_node("create_stl", anonymous=True)
    client()
