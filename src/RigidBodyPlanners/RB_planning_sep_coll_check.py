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
from .sep_collision_checking import *
from stl import mesh
from mpl_toolkits import mplot3d


class PlannerSepCollision:
    def __init__(self) -> None:
        self.coll_checker = checker

        self.space = ob.SE3StateSpace()
        # set lower and upper bounds
        self.set_bounds()

        self.ss = og.SimpleSetup(self.space)
        # set State Validity Checker function
        self.ss.setStateValidityChecker(
            ob.StateValidityCheckerFn(isStateValid))

        self.ss.getSpaceInformation().setStateValidityCheckingResolution(0.001)
        # set problem optimization objective
        self.set_optim_objective()

        print("Space Bounds High:", self.space.getBounds(
        ).high[0], self.space.getBounds().high[1], self.space.getBounds().high[2])
        print("Space Bounds Low:", self.space.getBounds(
        ).low[0], self.space.getBounds().low[1], self.space.getBounds().low[2])

    def set_optim_objective(self, objective_class=ob.MechanicalWorkOptimizationObjective):
        self.ss.setOptimizationObjective(
            objective_class(self.ss.getSpaceInformation()))

    def set_planner(self, planner_class=og.RRT):
        # choose planner
        planner = planner_class(self.ss.getSpaceInformation())

        self.ss.setPlanner(planner)
        self.ss.setup()

    def set_bounds(self):
        bounds = ob.RealVectorBounds(3)
        bounds.low[0] = -4.09
        bounds.low[1] = -2.2
        bounds.low[2] = -6

        bounds.high[0] = 4.09
        bounds.high[1] = 2.2
        bounds.high[2] = 6

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

    def set_start_goal(self, start_pose, goal_pose, transform=False):

        # define start state
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

    def solve(self, timeout=15.0):
        #

        # this will automatically choose a default planner with
        # default parameters
        print(f"Solving with timeout: {timeout} sec...")
        solved = self.ss.solve(timeout)
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

        return solved

    def visualize_path(self, path_file="path.txt"):
        try:
            data = np.loadtxt(path_file)
        except Exception as e:
            print("No path file found")

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(data[:, 3], data[:, 2], data[:, 1], '.-')

        # Load the STL files and add the vectors to the plot
        env_mesh = mesh.Mesh.from_file(
            'ros_ws/src/drone_path_planning/resources/stl/env-scene.stl')

        ax.add_collection3d(mplot3d.art3d.Poly3DCollection(env_mesh.vectors))

        # Auto scale to the mesh size
        scale = env_mesh.points.flatten()
        ax.auto_scale_xyz(scale, scale, scale)

        # set axes limits
        ax.set_xlim3d(-2, 5, 2.5)
        ax.set_ylim3d(-2, 5, 2.5)
        ax.set_zlim3d(-2, 5, 2.5)

        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        # printCoords(ax, data[0, 1], data[0, 2], data[0, 3])
        # printCoords(ax, data[-1, 1], data[-1, 2], data[-1, 3])

        plt.show()


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
    # print("Euler:", euler[0], euler[1], euler[2])

    max_angle = np.deg2rad(10)

    valid_rotation_arr = []
    if isStateValid.counter == 0:
        print("First state:")
        print(np.rad2deg(euler))
        # input()
        isStateValid.counter += 1

    # valid_rotation_arr.append(isBetween(euler[0], -max_angle, max_angle))
    valid_rotation_arr.append(isBetween(euler[0], -max_angle, max_angle))
    valid_rotation_arr.append(isBetween(euler[1], -max_angle, max_angle))

    valid_rotation = all(valid_rotation_arr)
    # print("             Valid Rotation:", valid_rotation)
    return no_collision and valid_rotation


isStateValid.counter = 0


def isBetween(x, min, max):
    return x >= min and x <= max


if __name__ == "__main__":
    # checker.visualize()
    planner = PlannerSepCollision()

    start_pos = [2, 0, -2]
    goal_pos = [4, 2, 2]

    # start
    start_pose = PoseStamped()
    start_pose.pose.position = Point(start_pos[0], start_pos[1], start_pos[2])
    start_pose.pose.orientation = Quaternion(
        -0.7071067811865475, 0, 0, 0.7071067811865476)
    # start_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    # goal
    goal_pose = PoseStamped()
    goal_pose.pose.position = Point(goal_pos[0], goal_pos[1], goal_pos[2])
    # goal_pose.pose.orientation = Quaternion(0, 0, 0, 1)
    goal_pose.pose.orientation = Quaternion(
        -0.7071067811865475, 0, 0, 0.7071067811865476)

    # if transform:
    #     start_pose = transform(start_pose)
    #     goal_pose = transform(goal_pose)

    planner.set_start_goal(start_pose.pose, goal_pose.pose)
    planner.set_planner()
    solved = planner.solve(timeout=80.0)
    if solved:
        planner.visualize_path()
