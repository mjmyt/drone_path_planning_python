#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped, Quaternion
import rospy
import numpy as np
from math import pi
import tf
from nav_msgs.msg import Path
import os
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Point

try:
    from execution.msg import TrajectoryPolynomialPieceMarios
except:
    from crazyswarm.msg import TrajectoryPolynomialPieceMarios

from optimizations import *

# print working directory
print("Current working directory:", os.getcwd())


def callback1(path: Path):
    if callback1.counter == 0:
        path_to_pol(path, 1)
        callback1.counter += 1


callback1.counter = 0


def callback2(path: Path):
    if callback2.counter == 0:
        path_to_pol(path, 2)
        callback2.counter += 1


callback2.counter = 0


def path_to_pol(path: Path, cfid: int):
    print("Path received...")
    # print(len(path.poses))

    total_duration = 10  # secs
    n = len(path.poses)
    time_step = total_duration / n

    traj_points = []
    for i, pose in enumerate(path.poses):
        x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        q = [pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w]
        yaw = tf.transformations.euler_from_quaternion(q)[2]

        # TODO: set constant yaw
        traj_points.append(Point_time(Waypoint(x, y, z, yaw), t=time_step*i))

    pols_coeffs, pc_pols = calculate_trajectory4D(traj_points)

    pol_to_send = TrajectoryPolynomialPieceMarios()
    pol_to_send.cf_id = cfid

    n = len(pc_pols[0].pols)
    # 8 coeffs per x,y,z,yaw + 1 for duration
    matrix = np.zeros((n, 8*4+1), dtype=np.float32)
    # iterate over piecewise polynomials(x,y,z,yaw)
    for i, pc_pol in enumerate(pc_pols):
        for j, pol in enumerate(pc_pol.pols):
            coeffs = pol.p  # get coefficients of j-th  polynomial

            # add coefficients to matrix
            matrix[j, 8*i+1:8*(i+1)+1] = coeffs.reshape((1, 8))

    #  dt column

    time_col = np.array(pc_pols[0].time_durations)
    matrix[:, 0] = time_col

    file_prefix = "/home/marios/thesis_ws/src/drone_path_planning/resources/trajectories/"
    np.savetxt(file_prefix+"Pol_matrix_{}.csv".format(cfid),
               matrix, delimiter=",")

    pol_to_send.poly_x = list(matrix[:,     0 + 1: 8 + 1].flatten())
    pol_to_send.poly_y = list(matrix[:,     8 + 1: 16+1].flatten())
    pol_to_send.poly_z = list(matrix[:,     16+1: 24+1].flatten())
    pol_to_send.poly_yaw = list(matrix[:,   24+1: 32+1].flatten())
    pol_to_send.durations = list(matrix[:, 0].flatten())

    piece_pols_pub.publish(pol_to_send)
    print("Published polynomial piece...")


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('drones_path_listener')

    rospy.Subscriber('drone1Path',  Path, callback1)
    rospy.Subscriber('drone2Path',  Path, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# create a publisher to publish the trajectory
piece_pols_pub = rospy.Publisher(
    'piece_pol', TrajectoryPolynomialPieceMarios, queue_size=10)

if __name__ == '__main__':
    listener()
