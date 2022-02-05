#!/usr/bin/env python3

from cgi import test
from concurrent.futures import thread
import rospy
from rospy.client import INFO
import numpy as np
from geometry_msgs.msg import PoseStamped
# from crazyswarm.msg import TrajectoryPolynomialPieceMarios
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import tf
# import uav_trajectory
import sys


def handle_new_trajectory(piece_pol):
    cf_id = piece_pol.cf_id
    print("Received new trajectory with cfid:", cf_id, "...")

    if cf_id == executor_id:
        executor_pos.receive_trajectory(piece_pol)


class TrajectoryExecutor_Position_Controller:
    def __init__(self, ) -> None:
        self.odom = None

    def odometry_callback(self, odom: Odometry):
        self.odom = odom

    def wait_until_get_to_pose(self, x, y, z, yaw, threshold=0.4):  # threshold propably needs tuning
        # Wait until the crazyflie gets to the pose-->nomrm(error) is smaller than threshold value
        # if self.odom is None:
        #     raise Exception("No odometry received yet")

        des_pose = PoseStamped()
        des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z = x, y, z

        error = np.inf
        while error > threshold:
            des = np.array([des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z])
            actual = np.array([self.odom.pose.pose.position.x,
                              self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
            # print("Waiting to go to pose...")
            error = np.linalg.norm(des - actual)

    def receive_trajectory(self, piece_pol):
        print("Crazyflie with id:", executor_id, "received trajectory...")
        cfid = piece_pol.cf_id

        lines = int(len(piece_pol.poly_x)/8)

        print(len(piece_pol.poly_x))

        x = np.array(piece_pol.poly_x).reshape((lines, 8))
        y = np.array(piece_pol.poly_y).reshape((lines, 8))
        z = np.array(piece_pol.poly_z).reshape((lines, 8))
        yaw = np.array(piece_pol.poly_yaw).reshape((lines, 8))
        durations = np.array(piece_pol.durations).reshape((lines, 1))

        print("x:", x.shape)
        print("y:", y.shape)
        print("z:", z.shape)
        print("yaw:", yaw.shape)
        print("durations:", durations.shape)

        matrix = np.zeros((lines, 1+8*4))  # 8 coeffs per x,y,z,yaw + 1 for duration
        matrix[:, 0] = durations.flatten()
        matrix[:, 1:9] = x
        matrix[:, 9:17] = y
        matrix[:, 17:25] = z
        matrix[:, 25:33] = yaw

        self.matrix = matrix

        self.execute_trajectory(matrix)

    def go_to_pose(self, x, y, z, yaw):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q[0], q[1], q[2], q[3]

        print("Publishing new position...")
        pos_pub.publish(p)
        print("Published new position...")

    def get_traj_start_pose(self) -> PoseStamped:
        eval = self.tr.eval(0)

        start_pose = PoseStamped()
        start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z = eval.pos[
            0], eval.pos[1], eval.pos[2]
        start_pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, eval.yaw)

        return start_pose

    def take_off(self, height=1):
        if self.odom == None:
            return False
        x, y, z = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z

        self.go_to_pose(x, y, height, 0)
        self.wait_until_get_to_pose(x, y, height, 0)

    def land(self):
        print("Landing...")
        safety_land_publisher.publish("Land")  # Land string is not necessary, but it is nice to have

    def execute_trajectory(self, matrix):
        file_name = "piecewise_pole_{}.csv".format(executor_id)
        names = ["duration",
                 "x^0", "x^1", "x^2", "x^3", "x^4", "x^5", "x^6", "x^7",
                 "y^0", "y^1", "y^2", "y^3", "y^4", "y^5", "y^6", "y^7",
                 "z^0", "z^1", "z^2", "z^3", "z^4", "z^5", "z^6", "z^7",
                 "yaw^0", "yaw^1", "yaw^2", "yaw^3", "yaw^4", "yaw^5", "yaw^6", "yaw^7"]

        np.savetxt(file_name,  matrix, delimiter=",", fmt='%.6f')
        tr = uav_trajectory.Trajectory()
        tr.loadcsv(file_name)  # TODO:Loaf trajectory without using file
        self.tr = tr
        print("duration:", tr.duration)

        t_space = np.linspace(0, tr.duration, 100)

        # feed waypoints to position controller
        start_pose = self.get_traj_start_pose()

        self.go_to_pose(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z, yaw=0)
        self.wait_until_get_to_pose(start_pose.pose.position.x, start_pose.pose.position.y,
                                    start_pose.pose.position.z, yaw=0)

        # frequency of sending references to the controller in hz
        rate = rospy.Rate(10.0)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time()-t0
            if t > tr.duration:
                break
            evaluation = tr.eval(t)
            pos, yaw = evaluation.pos, evaluation.yaw
            x, y, z = pos[0], pos[1], pos[2]

            print("t:", t, "x:", x, "y:", y, "z:", z, "yaw:", yaw)
            self.go_to_pose(x, y, z, yaw)

            rate.sleep()

        self.land()


def test_system():
    # while executor_pos.odom == None:
    #     pass

    # executor_pos.take_off(height=0.5)  # takes off and waits until it is at the desired height
    # rospy.sleep(2)
    x, y, z = 1.5, 4.5, 1.5
    executor_pos.go_to_pose(x, y, z, yaw=0)  # goes to the desired position
    # executor_pos.wait_until_get_to_pose(x, y, z, yaw=0)  # waits until it is at the desired position
    rospy.sleep(2)

    executor_pos.land()


if __name__ == "__main__":
    rospy.init_node("Traj_Executor_Position_Controller")

    # get command line arguments
    executor_id = int(sys.argv[2])
    print("Executor id:", executor_id)

    safety_land_publisher = rospy.Publisher('safety_land', String, queue_size=10)
    pos_pub = rospy.Publisher('reference', PoseStamped, queue_size=10)

    while pos_pub.get_num_connections() < 1:
        pass

    print("Got reference...")

    executor_pos = TrajectoryExecutor_Position_Controller()
    odometry_sub = rospy.Subscriber('/pixy/vicon/demo_crazyflie4/demo_crazyflie4/odom',
                                    Odometry, executor_pos.odometry_callback)

    # Subscribe to trajectory topic and then execute it after going to the first waypoint
    # rospy.Subscriber('piece_pol', TrajectoryPolynomialPieceMarios, handle_new_trajectory)

    test_system()  # Used to check the functionality of the system
    # p = PoseStamped()
    # p.pose.position.x, p.pose.position.y, p.pose.position.z = 1.5, 4.5, 1.5
    # q = tf.transformations.quaternion_from_euler(0, 0, 0)
    # p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q[0], q[1], q[2], q[3]
    # pos_pub.publish(p)

    rospy.spin()
