#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped, Quaternion
import rospy
import numpy as np
from math import pi
import tf
from nav_msgs.msg import Path
import os
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Point

from crazyswarm.msg import TrajectoryPolynomialPieceMarios

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

    pols_coeffs, pc_pols = calculate_trajectory4D(traj_points)

    pol_to_send = TrajectoryPolynomialPieceMarios()
    pol_to_send.cf_id = 0

    n = len(pc_pols[0].pols)
    matrix = np.zeros((n, 8*4+1), dtype=np.float32)  # 8 coeffs per x,y,z,yaw + 1 for time
    for i, pc_pol in enumerate(pc_pols):  # iterate over piecewise polynomials(x,y,z,yaw)
        for j, pol in enumerate(pc_pol.pols):
            coeffs = pol.p  # get coefficients of j-th  polynomial

            matrix[j, 8*i:8*(i+1)] = coeffs.reshape((1, 8))  # add coefficients to matrix

    #  dt column
    t_i_plus_1 = np.array(pc_pols[0].time_setpoints[1:])
    t_i = np.array(pc_pols[0].time_setpoints[:-1])
    time_col = t_i_plus_1 - t_i
    matrix[:, -1] = time_col

    np.savetxt("Pol_matrix.csv", matrix, delimiter=",")

    pol_to_send.poly_x = list(matrix[:, 0:8].flatten())
    pol_to_send.poly_y = list(matrix[:, 8:16].flatten())
    pol_to_send.poly_z = list(matrix[:, 16:24].flatten())
    pol_to_send.poly_yaw = list(matrix[:, 24:32].flatten())
    pol_to_send.durations = list(matrix[:, -1].flatten())

    piece_pols_pub.publish(pol_to_send)
    print("Published polynomial piece...")


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


# create a publisher to publish the trajectory
piece_pols_pub = rospy.Publisher('piece_pol', TrajectoryPolynomialPieceMarios, queue_size=10)

if __name__ == '__main__':
    listener()
