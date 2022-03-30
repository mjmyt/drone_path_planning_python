#!/usr/bin/env python3
import sys
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
from nav_msgs.msg import Odometry


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
    """
        This functtion converts a path to a polynomial trajectory message and returns both 
        the messaage and the matrix used to create it.
    """

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

    pol_to_send = create_piece_pol_message(cfid, matrix)

    return pol_to_send, matrix


def create_piece_pol_message(cfid, matrix):
    # cfid = 0 goes to leader, cfid = 1 goes to follower
    pol_to_send = TrajectoryPolynomialPieceMarios()
    pol_to_send.cf_id = cfid
    pol_to_send.poly_x = list(matrix[:,    0+1:1+8].flatten())
    pol_to_send.poly_y = list(matrix[:,    8+1:1+16].flatten())
    pol_to_send.poly_z = list(matrix[:,   16+1:1+24].flatten())
    pol_to_send.poly_yaw = list(matrix[:, 24+1:1+32].flatten())
    pol_to_send.durations = list(matrix[:, 0].flatten())
    return pol_to_send


def get_executor_id(cf_name):
    # get id after prefix
    try:
        common_prefix = "demo_crazyflie"
        executor_id = int(cf_name[len(common_prefix):])
    except:
        common_prefix = "crazyflie"
        executor_id = int(cf_name[len(common_prefix):])

    return executor_id


class PolsGenerator:
    def __init__(self):
        self.leader_pos = None
        self.follower_pos = None
        self.paths = []
        self.DRONES_NUM = 2
        self.published_pols = 0
        self.online_planning = True

    def leader_pos_callback(self, msg: Odometry):
        self.leader_pos = msg

    def follower_pos_callback(self, msg: Odometry):
        self.follower_pos = msg

    def path_callback(self, path):
        self.paths.append(path)
        print("Drones pols generator: received path and total # of paths is {}".format(len(self.paths)))
        if len(self.paths) == self.DRONES_NUM:
            self.generate_pols()

    def generate_pols(self):
        paths = self.paths
        # assign trajectory to each drone
        # consider leader as the one that is closest to the start of the 1st path
        path_1_start_pos = [paths[0].poses[0].pose.position.x, paths[0].poses[0].pose.position.y, paths[0].poses[0].pose.position.z]
        try:
            lead_dist = self.leader_distance_from_pos(path_1_start_pos)
            foll_dist = self.follower_distance_from_pos(path_1_start_pos)
            leader_id = 0 if lead_dist < foll_dist else 1
        except:
            leader_id = 0

        follower_id = 1 if leader_id == 0 else 0

        leader_path = paths[leader_id]
        follower_path = paths[follower_id]

        leader_pol, leader_matrix = path_to_pol(leader_path, 0)
        follower_pol, follower_matrix = path_to_pol(follower_path, 1)

        self.save_pols(leader_matrix, follower_matrix)
        self.publish_pols(leader_pol, follower_pol)

    def publish_pols(self, leader_pol, follower_pol):
        if self.published_pols == 1:
            return

        # wait to get connections
        if self.online_planning:
            self.wait_for_executor_connections()

        piece_pols_pub.publish(leader_pol)
        piece_pols_pub.publish(follower_pol)
        print("Drones pols generator: Published pols ")
        self.published_pols = 1

    def wait_for_executor_connections(self):
        print("Drones pols generator: waiting for connections")
        while piece_pols_pub.get_num_connections() < 2:
            rospy.sleep(0.1)
            if rospy.is_shutdown():
                print("Drones pols generator: ros shutdown")
                sys.exit()

    def save_pols(self, leader_matrix, follower_matrix):
        file_prefix = "/home/marios/thesis_ws/src/drone_path_planning/resources/trajectories/"
        # np.savetxt(file_prefix+"Pol_matrix_{}.csv".format(0), leader_matrix, delimiter=",")
        # np.savetxt(file_prefix+"Pol_matrix_{}.csv".format(1), follower_matrix, delimiter=",")
        np.savetxt(file_prefix+"Pol_matrix_leader.csv", leader_matrix, delimiter=",")
        np.savetxt(file_prefix+"Pol_matrix_follower.csv", follower_matrix, delimiter=",")
        print("Drones pols generator: Saved pols ")

    def leader_distance_from_pos(self, position: list):
        l = [self.leader_pos.pose.pose.position.x, self.leader_pos.pose.pose.position.y,
             self.leader_pos.pose.pose.position.z]

        return np.linalg.norm(np.array(l) - np.array(position))

    def follower_distance_from_pos(self, position: list):
        f = [self.follower_pos.pose.pose.position.x, self.follower_pos.pose.pose.position.y,
             self.follower_pos.pose.pose.position.z]

        return np.linalg.norm(np.array(f) - np.array(position))


def listener():
    rospy.init_node('drones_path_listener')

    pols_gen = PolsGenerator()
    try:
        # This code works in case of online planning
        leader_cf_name = rospy.get_param("/cf_follower_name")
        follower_cf_name = rospy.get_param("/cf_leader_name")

        # get id after prefix
        leader_id = get_executor_id(leader_cf_name)
        follower_id = get_executor_id(follower_cf_name)

        # drones positions subscriber
        leader_top = '/pixy/vicon/demo_crazyflie{}/demo_crazyflie{}/odom'.format(leader_id, leader_id)
        follower_top = '/pixy/vicon/demo_crazyflie{}/demo_crazyflie{}/odom'.format(follower_id, follower_id)

        rospy.Subscriber(leader_top, Odometry,   pols_gen.leader_pos_callback)
        rospy.Subscriber(follower_top, Odometry, pols_gen.follower_pos_callback)
    except:
        print("Drones pols generator: Failed to get cf names -->using offline planning")
        pols_gen.online_planning = False

    # path subscribers
    rospy.Subscriber('drone1Path',  Path, pols_gen.path_callback)
    rospy.Subscriber('drone2Path',  Path, pols_gen.path_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# create a publisher to publish the trajectory
piece_pols_pub = rospy.Publisher('piece_pol', TrajectoryPolynomialPieceMarios, queue_size=10)

if __name__ == '__main__':
    listener()
