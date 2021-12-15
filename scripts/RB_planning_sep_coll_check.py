#!/usr/bin/env python

# Author: Mark Moll
try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(
        0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og


from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from math import pi
import tf
from sep_collision_checking import *


class PlannerSepCollision:
    def __init__(self, coll_checker) -> None:
        self.coll_checker = coll_checker

        self.space = ob.SE3StateSpace()
        # set lower and upper bounds
        self.set_bounds()

        # create a simple setup object
        self.ss = og.SimpleSetup(self.space)
        self.ss.setStateValidityChecker(
            ob.StateValidityCheckerFn(PlannerSepCollision.isStateValid))

        # self.set_start_goal(self.space)

        # choose planner
        # planner = og.PRM(ss.getSpaceInformation())
        # ss.setPlanner(planner)
        # ss.setup()

    def set_bounds(self):
        bounds = ob.RealVectorBounds(3)
        bounds.low[0] = -6.0
        bounds.low[1] = -6.0
        bounds.low[2] = -4

        bounds.high[0] = 8
        bounds.high[1] = 8
        bounds.high[2] = 10

        # bounds.setLow(-10)
        # bounds.setHigh(10)
        self.space.setBounds(bounds)

        return bounds

    def save_path(self, file_name="path.txt"):
        # save the path
        print("Saving path to %s" % file_name)
        text_file = open(file_name, "w")
        n = text_file.write(self.path.printAsMatrix())
        text_file.close()

    def set_start_goal(self, start_pos, goal_pos, transform=False):
        # start
        start_pose = PoseStamped()
        start_pose.pose.position = Point(
            start_pos[0], start_pos[1], start_pos[2])
        start_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        # goal
        goal_pose = PoseStamped()
        goal_pose.pose.position = Point(goal_pos[0], goal_pos[1], goal_pos[2])
        goal_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        if transform:
            start_pose = transform(start_pose)
            goal_pose = transform(goal_pose)

        start_pose = start_pose.pose
        goal_pose = goal_pose.pose

        # define start state
        print("Space:", self.space)
        start = ob.State(self.space)
        start().setX(start_pose.position.x)
        start().setY(start_pose.position.y)
        start().setZ(start_pose.position.z)
        start().rotation().x = start_pose.orientation.x
        start().rotation().y = start_pose.orientation.y
        start().rotation().z = start_pose.orientation.z
        start().rotation().w = start_pose.orientation.w

        goal = ob.State(self.space)
        goal().setX(goal_pose.position.x)
        goal().setY(goal_pose.position.y)
        goal().setZ(goal_pose.position.z)
        goal().rotation().x = goal_pose.orientation.x
        goal().rotation().y = goal_pose.orientation.y
        goal().rotation().z = goal_pose.orientation.z
        goal().rotation().w = goal_pose.orientation.w

        self.ss.setStartAndGoalStates(start, goal)
        # return the start & goal states
        return start, goal

    def plan(self):
        #

        # this will automatically choose a default planner with
        # default parameters

        solved = self.ss.solve(15.0)
        if solved:
            print("Found solution...")
            # try to shorten the path
            self.ss.simplifySolution()
            # print the simplified path
            path = self.ss.getSolutionPath()
            path.interpolate(50)

            self.path = path
            self.save_path()
        else:
            print("No solution found")

    def isStateValid(state):
        # Some arbitrary condition on the state (note that thanks to
        # dynamic type checking we can just call getX() and do not need
        # to convert state to an SE2State.)
        pos = [state.getX(), state.getY(), state.getZ()]
        q = [state.rotation().x, state.rotation().y,
             state.rotation().z, state.rotation().w]

        checker.set_robot_transform(pos, q)

        no_collision = not checker.collision_check()

        euler = tf.euler_from_quaternion(q)

        max_angle = pi/10  # 18 degrees

        valid_rotation = isBetween(euler[0], -max_angle, max_angle)

        return no_collision and valid_rotation

    def visualize_path(self, path_file="path.txt"):
        data = np.loadtxt(path_file)

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

        # printCoords(ax, data[0, 1], data[0, 2], data[0, 3])
        # printCoords(ax, data[-1, 1], data[-1, 2], data[-1, 3])

        plt.show()


def isBetween(x, min, max):
    return x >= min and x <= max


"""
def planTheHardWay():
    # create an SE2 state space
    space = ob.SE2StateSpace()
    # set lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)
    # construct an instance of space information from this state space
    si = ob.SpaceInformation(space)
    # set state validity checking for this space
    si.setStateValidityChecker(ob.StateValidityCheckerFn(PlaisStateValid))
    # create a random start state
    start = ob.State(space)
    start.random()
    # create a random goal state
    goal = ob.State(space)
    goal.random()
    # create a problem instance
    pdef = ob.ProblemDefinition(si)
    # set the start and goal states
    pdef.setStartAndGoalStates(start, goal)
    # create a planner for the defined space
    planner = og.RRTConnect(si)
    # set the problem we are trying to solve for the planner
    planner.setProblemDefinition(pdef)
    # perform setup steps for the planner
    planner.setup()
    # print the settings for this space
    print(si.settings())
    # print the problem settings
    print(pdef)
    # attempt to solve the problem within one second of planning time
    solved = planner.solve(1.0)
    if solved:
        # get the goal representation from the problem definition (not the same as the goal state)
        # and inquire about the found path
        path = pdef.getSolutionPath()
        print("Found solution:\n%s" % path)
    else:
        print("No solution found")
"""

if __name__ == "__main__":
    checker = SepCollisionChecking()
    checker.set_env_transform([0, 0, 0], [0, 0, 0, 1])
    checker.set_robot_transform([4, -2, 2], [0, 0, 0, 1])
    # checker.visualize()

    planner = PlannerSepCollision(checker)

    start = [2, 2, 0]
    goal = [4, -2, 2]
    planner.set_start_goal(start, goal)
    planner.plan()
    # planner.visualize_path()
