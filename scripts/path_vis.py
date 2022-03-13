#!/usr/bin/env python3
# print working directory
from nav_msgs.msg import Path
import numpy as np
import rospy
from trajectory_visualising import get_nav_path_msg
from trajectory_visualising import Trajectory
import os
print(os.getcwd())


if __name__ == "__main__":

    rospy.init_node("trajectory_planning", anonymous=True)
    traj1_file_name = '/home/marios/thesis_ws/src/drone_path_planning/resources/trajectories/Pol_matrix_1.csv'
    traj2_file_name = '/home/marios/thesis_ws/src/drone_path_planning/resources/trajectories/Pol_matrix_2.csv'

    offset1 = [0, 0, -0.5]
    offset2 = [0, 0, -0.5]

    tr = Trajectory()
    tr.loadcsv(traj1_file_name)

    tr2 = Trajectory()
    tr2.loadcsv(traj2_file_name)

    print("Trajectoty time duration:", tr.duration)
    timestep = 0.1

    trajPub1 = rospy.Publisher('path1',  Path, queue_size=10)
    trajPub2 = rospy.Publisher('path2',  Path, queue_size=10)

    rate = rospy.Rate(30.0)  # hz
    while not rospy.is_shutdown():

        msg = get_nav_path_msg(tr, timestep, offset1)
        trajPub1.publish(msg)
        msg2 = get_nav_path_msg(tr2, timestep, offset2)
        trajPub2.publish(msg2)

    rate.sleep()
