#!/usr/bin/env python

# Author: Mark Moll
from stl import mesh
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Pose
import tf
import numpy as np
import matplotlib.pyplot as plt
from math import pi
import rospy

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    import sys
    from os.path import abspath, dirname, join
    sys.path.insert(
        0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og

try:
    from .fcl_checker import Fcl_checker
    from .custom_robot_mesh import Custom_robot_mesh
except ImportError:
    from fcl_checker import Fcl_checker
    from custom_robot_mesh import Custom_robot_mesh

import os
print("cwd:", os.getcwd())


class PlannerSepCollision:
    def __init__(self, env_mesh_name, robot_mesh_name, cat_lowest_function) -> None:
        # env_mesh_name and robot_mesh_name are type of "env-scene-hole.stl"
        try:
            env_mesh = "ros_ws/src/drone_path_planning/resources/stl/{}".format(
                env_mesh_name)
            robot_mesh = "ros_ws/src/drone_path_planning/resources/stl/{}".format(
                robot_mesh_name)

            self.checker = Fcl_checker(env_mesh, robot_mesh)
        except:
            print("cwd:", os.getcwd())
            env_mesh = r"/home/marios/thesis_ws/src/drone_path_planning/resources/stl/{}".format(
                env_mesh_name)
            robot_mesh = r"/home/marios/thesis_ws/src/drone_path_planning/resources/stl/{}".format(
                robot_mesh_name)

            self.checker = Fcl_checker(env_mesh, robot_mesh)

        self.states_tried = 0
        self.time_sum = 0
        self.L = 3.0  # rope length #TODO:make this a ROS parameter
        drones_distance = 2  # distance between drones
        theta = 0
        self.custom_robot = Custom_robot_mesh(
            drones_distance, theta, self.L, cat_lowest_function, mesh_type="fcl")

        # x, y, z, yaw , drones_distance, drones_angles
        self.space = ob.RealVectorStateSpace(6)

        # set lower and upper bounds
        self.set_bounds()

        self.ss = og.SimpleSetup(self.space)
        # set State Validity Checker function
        self.ss.setStateValidityChecker(
            ob.StateValidityCheckerFn(self.isStateValid))

        self.ss.getSpaceInformation().setStateValidityCheckingResolution(0.01)

        # set problem optimization objective
        # all available ob optimization objectives (maybe there are some more)
        # I can also make my own
        optimizations = ["MechanicalWorkOptimizationObjective",
                         "MultiOptimizationObjective", "PathLengthOptimizationObjective"]

        # self.set_optim_objective()
        print("Setting default Optimization Objective (Path Length)")
        self.set_optim_objective(ob.PathLengthOptimizationObjective)

        print("Space Bounds High:", self.space.getBounds().high[0], self.space.getBounds().high[1], self.space.getBounds().high[2],
              self.space.getBounds().high[3], self.space.getBounds().high[4])
        print("Space Bounds Low:", self.space.getBounds().low[0], self.space.getBounds().low[1], self.space.getBounds().low[2],
              self.space.getBounds().low[3], self.space.getBounds().low[4])

    def set_optim_objective(self, objective_class=ob.MechanicalWorkOptimizationObjective):
        try:
            self.ss.setOptimizationObjective(
                objective_class(self.ss.getSpaceInformation()))
        except Exception as e:
            print("Error setting optimization objective:", e)
            self.ss.setOptimizationObjective(objective_class)

    def set_planner(self, planner_class=og.RRT):
        # choose planner
        planner = planner_class(self.ss.getSpaceInformation())

        self.ss.setPlanner(planner)
        self.ss.setup()

    def set_bounds(self):
        bounds = ob.RealVectorBounds(6)
        # set bounds for x, y, z , rotation
        bounds.low[0] = -4.09
        bounds.low[1] = -6
        bounds.low[2] = -1.5
        bounds.low[3] = -pi
        bounds.low[4] = self.L * 0.2
        bounds.low[5] = -pi/3

        # set bounds for x, y, z, rotation
        bounds.high[0] = 4.09
        bounds.high[1] = 6
        bounds.high[2] = 2.2
        bounds.high[3] = pi
        bounds.high[4] = self.L * 0.9
        bounds.high[5] = pi/3

        # bounds.setLow(-10)
        # bounds.setHigh(10)
        self.space.setBounds(bounds)

        return bounds

    def save_path(self, file_name="path.txt"):
        # save the path
        file_name = "src/drone_path_planning/resources/paths/{}".format(
            file_name)
        print("Saving path to %s" % file_name)
        text_file = open(file_name, "w")
        n = text_file.write(self.path.printAsMatrix())
        text_file.close()

    def set_start_goal(self, start_pose: Pose, goal_pose: Pose, transform=False):

        # define start state
        start = ob.State(self.space)

        start[0] = start_pose.position.x
        start[1] = start_pose.position.y
        start[2] = start_pose.position.z
        start[3] = tf.transformations.euler_from_quaternion(
            [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w])[2]
        start[4] = self.L * 0.8
        start[5] = 0

        goal = ob.State(self.space)
        goal[0] = goal_pose.position.x
        goal[1] = goal_pose.position.y
        goal[2] = goal_pose.position.z
        goal[3] = tf.transformations.euler_from_quaternion(
            [goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w])[2]
        goal[4] = self.L * 0.5
        goal[5] = 0

        print("============================================================")
        print("START POSE:")
        self.print_state_data(start)
        print("============================================================")
        print("GOAL POSE:")
        self.print_state_data(goal)
        print("============================================================")

        self.ss.setStartAndGoalStates(start, goal)
        # return the start & goal states
        return start, goal

    def print_state_data(self, state):
        print("\t x: %.2f" % state[0])
        print("\t y: %.2f" % state[1])
        print("\t z: %.2f" % state[2])
        print("\t yaw: %.2f" % np.rad2deg(state[3]), "deg")
        print("\t drones distance: %.2f" % state[4])
        print("\t drones angle: %.2f" % np.rad2deg(state[5]), "deg")

    def solve(self, timeout=15.0):
        #

        # this will automatically choose a default planner with
        # default parameters
        print(f"Solving with timeout: {timeout} sec...")
        self.states_tried = 0
        self.time_sum = 0
        solved = self.ss.solve(timeout)
        if solved:
            print("Found solution...")
            # try to shorten the path
            self.ss.simplifySolution()
            # print the simplified path
            path = self.ss.getSolutionPath()
            path.interpolate(30)

            self.path = path
            self.save_path()
            # solution_states = self.path.getStates()

            # print("Checking vaildity of the solution states...")
            # for state in solution_states:
            #     print("[ %.2f %.2f %.2f %.2f %.2f %.2f deg ] :" % (state[0], state[1],
            #           state[2], state[3], state[4], np.rad2deg(state[5])), self.isStateValid(state))
        else:
            print("No solution found")
            return None, 0, 0, 0

        return solved, self.time_sum, self.states_tried, self.time_sum/self.states_tried

    def visualize_path(self, path_file="path.txt"):
        try:
            data = np.loadtxt(path_file)
        except Exception as e:
            print("No path file found")

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(data[:, 3], data[:, 2], data[:, 1], '.-')

        # Load the STL files and add the vectors to the plot
        env_mesh = mesh.Mesh.from_file(env_mesh_name)

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

    def isStateValid(self, state):
        # calculate seconds ellapsed
        t0 = rospy.get_time()
        pos = [state[0], state[1], state[2]]
        q = tf.transformations.quaternion_from_euler(0, 0, state[3])

        drones_distance = state[4]
        theta = state[5]

        self.custom_robot.update_mesh(drones_distance, theta, self.L)
        self.checker.update_robot(self.custom_robot.mesh)

        self.checker.set_robot_transform(pos, q)

        no_collision = not self.checker.check_collision()
        self.states_tried += 1

        dt = rospy.get_time()-t0
        self.time_sum += dt
        if (self.states_tried % 500) == 0:
            print("Tried {} states --> average time: {} msec".format(self.states_tried,
                  self.time_sum / self.states_tried*1000), end="")
            print("\r", end="")

        return no_collision


def isBetween(x, min, max):
    return x >= min and x <= max


if __name__ == "__main__":
    # checker.visualize()
    env_mesh_name = "env-scene-hole.stl"
    robot_mesh_name = "robot-scene-triangle.stl"
    planner = PlannerSepCollision(env_mesh_name, robot_mesh_name)

    start_pos = [-4, -2, 2]
    goal_pos = [4, 2, 2]

    # start
    start_pose = PoseStamped()
    start_pose.pose.position = Point(start_pos[0], start_pos[1], start_pos[2])
    # start_pose.pose.orientation = Quaternion(-0.7071067811865475, 0, 0, 0.7071067811865476)
    start_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    # goal
    goal_pose = PoseStamped()
    goal_pose.pose.position = Point(goal_pos[0], goal_pos[1], goal_pos[2])
    goal_pose.pose.orientation = Quaternion(0, 0, 0, 1)
    # goal_pose.pose.orientation = Quaternion(-0.7071067811865475, 0, 0, 0.7071067811865476)

    # if transform:
    #     start_pose = transform(start_pose)
    #     goal_pose = transform(goal_pose)

    planner.set_start_goal(start_pose.pose, goal_pose.pose)
    planner.set_planner()
    solved = planner.solve(timeout=80.0)
    if solved:
        planner.visualize_path()
