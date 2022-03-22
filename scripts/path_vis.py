#!/usr/bin/env python3
# print working directory
from nav_msgs.msg import Path
import numpy as np
import rospy
from trajectory_visualising import get_nav_path_msg
from trajectory_visualising import Trajectory
import os


if __name__ == "__main__":

    rospy.init_node("trajectory_planning", anonymous=True)

    tr = Trajectory()
    tr.loadcsv(
        '/home/marios/crazyswarm/ros_ws/src/drone_path_planning/src/traj.csv')

    print("Trajectoty time duration:", tr.duration)
    timestep = 0.1

    trajPub = rospy.Publisher('traj',  Path, queue_size=10)

    rate = rospy.Rate(30.0)  # hz
    while not rospy.is_shutdown():

        msg = get_nav_path_msg(tr, timestep)
        trajPub.publish(msg)

        rate.sleep()
