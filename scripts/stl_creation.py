#!/usr/bin/env python3
# print working directory
from nav_msgs.msg import Path
import numpy as np
import rospy

from RigidBodyPlanners import drones_formation_2_triangle_points, create_3D_triangle_stl

from catenaries.srv import CatLowestPoint, CatLowestPointResponse
from stl import mesh

import os
print(os.getcwd())


def create_custom_robot(drones_distance, theta, L) -> mesh.Mesh:
    """
    This function generated a 3D rigid trinagle body suitable for path planning of the drone swarm
    theta : represents the angle that is formed between the line connecting the drones and the horizontal plane 
    """
    # Get first 2 points based on drones distance and theta
    p0, p1 = drones_formation_2_triangle_points(drones_distance, theta)
    p0, p1 = [p0[0], p0[1], 0], [p1[0], p1[1], 0]

    # Set the lowest point of the catenary formed by the 2 previous points
    # as the 3rd point of the catenary
    rospy.wait_for_service('catenary_lowest_point')
    try:
        catenary_lowest = rospy.ServiceProxy(
            'catenary_lowest_point', CatLowestPoint)

        lowest_point = catenary_lowest(p0, p1, L).lowest_point

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    lowest_point = [lowest_point[0], lowest_point[2]]

    mesh = create_3D_triangle_stl(p0, p1, lowest_point,
                                  "custom_triangle_robot.stl")

    return mesh


def client():
    drones_distance = 1
    L = 3
    theta = 0
    mesh = create_custom_robot(drones_distance, theta, L)
    print(dir(mesh))
    print(mesh.is_closed())


if __name__ == "__main__":

    rospy.init_node("create_stl", anonymous=True)
    client()
