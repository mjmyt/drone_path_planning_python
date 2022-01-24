#!/usr/bin/env python

######################################################################
# Rice University Software Distribution License
#
# Copyright (c) 2010, Rice University
# All Rights Reserved.
#
# For a full description see the file named LICENSE.
#
######################################################################

# Author: Mark Moll
# ================Marios================
import os
from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
from functools import partial
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
import math
try:
    from ros_ws.src.drone_path_planning.src.RigidBodyPlanners import frameTransforms
except Exception as e:
    from frameTransforms import transform

#  ================Marios================

import sys
from os.path import abspath, dirname, join

ompl_app_root = dirname(dirname(dirname(abspath(__file__))))

try:
    from ompl import base as ob
    from ompl import app as oa
    from ompl import geometric as og
    from ompl import util as ou
except ImportError:
    sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings'))
    from ompl import base as ob
    from ompl import app as oa

path_filename = "path_hole4.txt"


def printCoords(ax, x, y, z):
    text = str(x) + ', ' + str(y) + ', ' + str(z)
    ax.text(x, y, z, text, zdir=(1, 1, 1))


class RBPlanner():
    def __init__(self) -> None:
        # plan in SE(3)
        self.setup = oa.SE3RigidBodyPlanning()

        self.load_meshes()
        self.set_bounds()
        # self.set_validity_checker()

        # self.set_start_goal()
        # self.set_planner()

    def load_meshes(self):
        print("Working directory: " + os.getcwd())

        # load the robot and the environment
        # try:
        #     self.setup.setRobotMesh('3D/robot-scene.dae')
        #     self.setup.setEnvironmentMesh('3D/empty-scene.dae')
        # except Exception as e:
        #     print("Error loading robot mesh: \n" + str(e))
        robot_mesh_path = 'ros_ws/src/drone_path_planning/resources/collada/robot-scene.dae'
        env_mesh_path = 'ros_ws/src/drone_path_planning/resources/collada/env-scene-hole.dae'

        self.setup.setRobotMesh(robot_mesh_path)
        self.setup.setEnvironmentMesh(env_mesh_path)

    def set_bounds(self):
        # set the bounds for the workspace
        bounds = ob.RealVectorBounds(3)
        bounds.low[0] = -4.09
        bounds.low[1] = -2.2
        bounds.low[2] = -6

        # bounds.low[3] = 0
        # bounds.low[4] = 0
        # bounds.low[5] = 0
        # bounds.low[6] = 1

        bounds.high[0] = 4.09
        bounds.high[1] = 2.2
        bounds.high[2] = 6

        # bounds.high[3] = 0
        # bounds.high[4] = 0
        # bounds.high[5] = 0
        # bounds.high[6] = 1

        self.setup.getGeometricComponentStateSpace().setBounds(bounds)

    def set_start_goal(self, start_pose, goal_pose):
        # define start state
        start = ob.State(self.setup.getSpaceInformation())
        start().setX(start_pose.position.x)
        start().setY(start_pose.position.y)
        start().setZ(start_pose.position.z)
        start().rotation().x = start_pose.orientation.x
        start().rotation().y = start_pose.orientation.y
        start().rotation().z = start_pose.orientation.z
        start().rotation().w = start_pose.orientation.w

        goal = ob.State(self.setup.getSpaceInformation())
        goal().setX(goal_pose.position.x)
        goal().setY(goal_pose.position.y)
        goal().setZ(goal_pose.position.z)
        goal().rotation().x = goal_pose.orientation.x
        goal().rotation().y = goal_pose.orientation.y
        goal().rotation().z = goal_pose.orientation.z
        goal().rotation().w = goal_pose.orientation.w

        # set the start & goal states
        self.setup.setStartAndGoalStates(start, goal)

    def set_planner(self):
        print("Setting planner...")
        # setting collision checking resolution to 1% of the space extent
        # self.setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)

        # we call setup just so print() can show more information
        print("Planner setup")

        # print(self.setup)

    def solve(self, timeout=30.0):
        # solve the problem
        print("Solving...")
        solved = self.setup.solve(10)

        # print the results
        if solved:
            print("Solved")
            self.setup.simplifySolution()
            path = self.setup.getSolutionPath()
            path.interpolate(100)
            self.path = path

            # print("Found path:\n%s" % path)
            print("Planner solved in %.3f sec" %
                  self.setup.getLastPlanComputationTime())
            print("Final epsilon: %.3f" %
                  self.setup.getProblemDefinition().getSolutionDifference())
            # print(path.printAsMatrix())
            self.save_path()
        else:
            print("No solution found")

        return solved

    def save_path(self, path=path_filename):
        # save the path
        print("Saving path to %s" % path)
        text_file = open(path, "w")
        n = text_file.write(self.path.printAsMatrix())
        text_file.close()

    def visualize_path(self, path_file=path_filename):
        data = numpy.loadtxt(path_file)

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(data[:, 3], data[:, 2], data[:, 1], '.-')

        # set axes limits
        ax.set_xlim3d(-5, 7.22)
        ax.set_ylim3d(-3, 3.84)
        ax.set_zlim3d(-0.5, 2)

        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        printCoords(ax, data[0, 1], data[0, 2], data[0, 3])
        printCoords(ax, data[-1, 1], data[-1, 2], data[-1, 3])

        plt.show()

    def set_validity_checker(self):
        # create a simple setup for the validity checker
        self.setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)
        print("Setting validity checker...")

        # self.setup.getSpaceInformation().setStateValidityChecker(ob.StateValidityCheckerFn(
        #     partial(isStateValid, self.setup.getSpaceInformation())))

        # self.ss.setStateValidityChecker(
        #     ob.StateValidityCheckerFn(isStateValid))

        # self.setup.getSpaceInformation().setStateValidityChecker(
        #     ob.StateValidityCheckerFn(is_state_valid)
        # )
        self.setup.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

        self.setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)

        print(self.setup.getSpaceInformation().getStateValidityChecker().getSpecs())


def isStateValid(spaceInformation, state):
    # perform collision checking or check if other constraints are
    # satisfied
    print("checking state validity...")
    pos = [state.getX(), state.getY(), state.getZ()]
    q = [state.rotation().x, state.rotation().y, state.rotation().z, state.rotation().w]
    euler = tf.transformations.euler_from_quaternion(q)

    max_angle = math.degrees(10)

    valid_rotation_arr = []
    # valid_rotation_arr.append(isBetween(euler[0], -max_angle, max_angle))
    valid_rotation_arr.append(isBetween(euler[0], -max_angle, max_angle))
    valid_rotation_arr.append(isBetween(euler[1], -max_angle, max_angle))

    valid_rotation = all(valid_rotation_arr)

    return valid_rotation


def isBetween(x, min, max):
    return x >= min and x <= max


class MyValidStateSampler(ob.ValidStateSampler):
    def __init__(self, si):
        super(MyValidStateSampler, self).__init__(si)
        self.name_ = "my sampler"
        self.rng_ = ou.RNG()

    # Generate a sample in the valid part of the R^3 state space.
    # Valid states satisfy the following constraints:
    # -1<= x,y,z <=1
    # if .25 <= z <= .5, then |x|>.8 and |y|>.8
    def sample(self, state):
        print("sampling state...")

        z = self.rng_.uniformReal(-1, 1)

        if z > .25 and z < .5:
            x = self.rng_.uniformReal(0, 1.8)
            y = self.rng_.uniformReal(0, .2)
            i = self.rng_.uniformInt(0, 3)
            if i == 0:
                state[0] = x-1
                state[1] = y-1
            elif i == 1:
                state[0] = x-.8
                state[1] = y+.8
            elif i == 2:
                state[0] = y-1
                state[1] = x-1
            elif i == 3:
                state[0] = y+.8
                state[1] = x-.8
        else:
            state[0] = self.rng_.uniformReal(-1, 1)
            state[1] = self.rng_.uniformReal(-1, 1)
        state[2] = z
        return True


# return an instance of my sampler
def allocMyValidStateSampler(si):
    return MyValidStateSampler(si)


if __name__ == '__main__':
    planner = RBPlanner()

    start = [2, 0, -2]
    goal = [2, 0, 5]

    start_pose = PoseStamped()
    start_pose.pose.position.x = start[0]
    start_pose.pose.position.y = start[1]
    start_pose.pose.position.z = start[2]
    q = tf.quaternion_from_euler(math.pi/2, 0, 0)
    start_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    goal_pose = PoseStamped()
    goal_pose.pose.position.x = goal[0]
    goal_pose.pose.position.y = goal[1]
    goal_pose.pose.position.z = goal[2]
    q = tf.quaternion_from_euler(0, 0, 0)
    goal_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    # start_pose_transformed = transform(start_pose)
    # goal_pose_transformed = transform(goal_pose)

    start_pose_transformed = start_pose
    goal_pose_transformed = goal_pose

    space = planner.setup.getGeometricComponentStateSpace()
    print("Space Bounds High:", space.getBounds().high[0], space.getBounds().high[1], space.getBounds().high[2])
    print("Space Bounds Low:", space.getBounds().low[0], space.getBounds().low[1], space.getBounds().low[2])

    print("==============================")
    print("START POSE:")
    print(start_pose_transformed)
    print("==============================")
    print("GOAL POSE:")
    print(goal_pose_transformed)
    print("==============================")

    planner.set_start_goal(start_pose_transformed.pose, goal_pose_transformed.pose)

    # use my sampler
    si = planner.setup.getSpaceInformation()
    si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(allocMyValidStateSampler))

    # planner.set_planner()
    planner.setup.setPlanner(og.RRT(si))

    solved = planner.solve(timeout=10.0)

    if solved:
        data = numpy.loadtxt(path_filename)

        planner.visualize_path()
