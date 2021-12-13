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
# ================Marios================

import sys
from os.path import abspath, dirname, join

ompl_app_root = dirname(dirname(dirname(abspath(__file__))))

try:
    from ompl import base as ob
    from ompl import app as oa
except ImportError:
    sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings'))
    from ompl import base as ob
    from ompl import app as oa


def printCoords(ax, x, y, z):
    text = str(x) + ', ' + str(y) + ', ' + str(z)
    ax.text(x, y, z, text, zdir=(1, 1, 1))


class RBPlanner():
    def __init__(self) -> None:
        # plan in SE(3)
        self.setup = oa.SE3RigidBodyPlanning()

        self.load_meshes()
        self.set_bounds()
        self.set_validity_checker()

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

        self.setup.setRobotMesh('3D/robot-scene.dae')
        self.setup.setEnvironmentMesh('3D/empty-scene.dae')

    def set_bounds(self):
        # set the bounds for the workspace
        bounds = ob.RealVectorBounds(3)
        bounds.low[0] = -6.0
        bounds.low[1] = -6.0
        bounds.low[2] = -4

        # bounds.low[3] = 0
        # bounds.low[4] = 0
        # bounds.low[5] = 0
        # bounds.low[6] = 1

        bounds.high[0] = 8
        bounds.high[1] = 8
        bounds.high[2] = 10

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
        self.setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)

        # we call setup just so print() can show more information
        print("Planner setup")
        self.setup.setup()
        # print(self.setup)

    def solve(self, timeout=10.0):
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

    def save_path(self, path="path.txt"):
        # save the path
        print("Saving path to %s" % path)
        text_file = open(path, "w")
        n = text_file.write(self.path.printAsMatrix())
        text_file.close()

    def visualize_path(self, path_file="path.txt"):
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

        self.setup.getSpaceInformation().setStateValidityChecker(ob.StateValidityCheckerFn(
            partial(isStateValid, self.setup.getSpaceInformation())))

        print(self.setup.getSpaceInformation())

        self.setup.getSpaceInformation().setStateValidityChecker(
            ob.StateValidityCheckerFn(is_state_valid)
        )


def isStateValid(spaceInformation, state):
    # perform collision checking or check if other constraints are
    # satisfied
    print("mlkia")
    return spaceInformation.satisfiesBounds(state) and state.getX() > 0.5


def is_state_valid(state):
    # "state" is of type SE2StateInternal, so we don't need to use the "()"
    # operator.
    #
    # Some arbitrary condition on the state (note that thanks to
    # dynamic type checking we can just call getX() and do not need
    # to convert state to an SE2State.)
    print(dir(state))
    input()
    return state.getX() >= 3.0


if __name__ == '__main__':
    planner = RBPlanner()
    start = [3, 0, -2]
    goal = [3, 2, 2]

    start_pose = PoseStamped()
    start_pose.pose.position.x = start[0]
    start_pose.pose.position.y = start[1]
    start_pose.pose.position.z = start[2]
    start_pose.pose.orientation = Quaternion(
        -0.7071067811865475, 0, 0, 0.7071067811865476)

    goal_pose = PoseStamped()
    goal_pose.pose.position.x = goal[0]
    goal_pose.pose.position.y = goal[1]
    goal_pose.pose.position.z = goal[2]
    goal_pose.pose.orientation = Quaternion(
        -0.7071067811865475, 0, 0, 0.7071067811865476)

    planner.set_start_goal(start_pose.pose, goal_pose.pose)

    planner.set_planner()
    planner.solve(timeout=10.0)

    data = numpy.loadtxt("path.txt")
    print(data[:, 0])
    print(data[:, 0] > 0.5)

    # planner.visualize_path()
