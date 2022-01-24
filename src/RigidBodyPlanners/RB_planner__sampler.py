#!/usr/bin/env python

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
from time import sleep
from math import degrees, fabs, pi, radians

import tf.transformations as tft
import numpy as np

try:
    from .sep_collision_checking import *
except:
    from sep_collision_checking import *

# @cond IGNORE

# This is a problem-specific sampler that automatically generates valid
# states; it doesn't need to call SpaceInformation::isValid. This is an
# example of constrained sampling. If you can explicitly describe the set valid
# states and can draw samples from it, then this is typically much more
# efficient than generating random samples from the entire state space and
# checking for validity.


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
        # print("sampling")
        lows, highs = [0, 0, 0], [0, 0, 0]

        lows[0] = -4.09
        lows[1] = -2.2
        lows[2] = -6

        highs[0] = 4.09
        highs[1] = 2.2
        highs[2] = 6

        x = self.rng_.uniformReal(lows[0], highs[0])
        y = self.rng_.uniformReal(lows[1], highs[1])
        z = self.rng_.uniformReal(lows[2], highs[2])

        roll = 0
        pitch = 0
        yaw = self.rng_.uniformReal(-pi, pi)

        q = tft.quaternion_from_euler(roll, pitch, yaw)

        state[0][0] = x
        state[0][1] = y
        state[0][2] = z

        state[1].x = q[0]
        state[1].y = q[1]
        state[1].z = q[2]
        state[1].w = q[3]

        print("Sampling:", x, y, z)
        # print("Euler sampling:", tft.euler_from_quaternion(q))

        return True

# @endcond

# This function is needed, even when we can write a sampler like the one
# above, because we need to check path segments for validity


def isStateValid(state):
    # Let's pretend that the validity check is computationally relatively
    # expensive to emphasize the benefit of explicitly generating valid
    # samples

    # sleep(.001)

    # Valid states satisfy the following constraints:
    # -1<= x,y,z <=1
    # if .25 <= z <= .5, then |x|>.8 and |y|>.8

    # return fabs(state[0][0]) < 0.5 and fabs(state[0][1]) < 0.5 and fabs(state[0][2]) < 0.5
    pos = [state.getX(), state.getY(), state.getZ()]
    q = [state.rotation().x, state.rotation().y,
         state.rotation().z, state.rotation().w]

    checker.set_robot_transform(pos, q)

    no_collision = not checker.collision_check()

    valid_rotation_arr = []
    max_angle = radians(15)
    euler = tft.euler_from_quaternion(q)

    theta = degrees(euler[0])
    cond_0 = isBetween(theta, -180, -170) or isBetween(theta, -90, -80) or isBetween(theta, 170, 180)

    valid_rotation_arr.append(cond_0)
    # valid_rotation_arr.append(isBetween(euler[1], -max_angle, max_angle))
    valid_rotation_arr.append(isBetween(euler[2], -max_angle, max_angle))

    valid_rotation = all(valid_rotation_arr)
    # print("Validating:", pos[0], pos[1], pos[2])
    # print("Euler validation:", tft.euler_from_quaternion(q))
    # print("State Valid:", no_collision)
    return no_collision and valid_rotation


def isBetween(x, min, max):
    return x >= min and x <= max


# return an instance of my sampler
def allocMyValidStateSampler(si):
    print("allocMyValidStateSampler")
    return MyValidStateSampler(si)


def plan(samplerIndex):
    # construct the state space we are planning in
    # space = ob.RealVectorStateSpace(3)
    space = ob.SE3StateSpace()

    # set the bounds
    bounds = ob.RealVectorBounds(3)

    bounds.low[0] = -1.74
    bounds.low[1] = -2.19
    bounds.low[2] = -4

    bounds.high[0] = 2
    bounds.high[1] = 2.2
    bounds.high[2] = 5

    space.setBounds(bounds)

    # define a simple setup class
    ss = og.SimpleSetup(space)

    # set state validity checking for this space
    valCheckFn = ob.StateValidityCheckerFn(isStateValid)
    ss.setStateValidityChecker(valCheckFn)

    start_coords = [2, 0, -4]
    goal_coords = [2, 0, 5]

    # create a start state
    start = ob.State(space)
    start[0], start[1], start[2] = start_coords

    q = tft.quaternion_from_euler(-pi/2, 0, 0)
    start[3], start[4], start[5], start[6] = q

    # create a goal state
    goal = ob.State(space)
    goal[0], goal[1], goal[2] = goal_coords

    q = tft.quaternion_from_euler(-pi/2, 0, 0)
    goal[3], goal[4], goal[5], goal[6] = q

    # set the start and goal states;
    print("start:", start)
    print("goal:", goal)
    ss.setStartAndGoalStates(start, goal)

    # set sampler (optional; the default is uniform sampling)
    si = ss.getSpaceInformation()
    print(si)
    print("using my sampler")
    si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(allocMyValidStateSampler))

    # create a planner for the defined space
    planner = og.RRT(si)
    ss.setPlanner(planner)

    # attempt to solve the problem within ten seconds of planning time
    print("start planning")
    solved = ss.solve(100.0)
    if solved:
        print("Found solution:")
        # ss.simplifySolution()

        # print the path to screen
        ss.simplifySolution()
        path = ss.getSolutionPath()
        path.interpolate(50)

        # print the path to a file
        file_path = "mlkia_path.txt"

        print("Saving path to %s" % file_path)
        text_file = open(file_path, "w")
        n = text_file.write(path.printAsMatrix())
        text_file.close()

        path = np.array(path.getStates())
        for s in path:
            # print(s[0][0], s[0][1], s[0][2])
            # print(s[1].x, s[1].y, s[1].z, s[1].w)
            rpy = tft.euler_from_quaternion([s[1].x, s[1].y, s[1].z, s[1].w])
            rpy_deg = [degrees(i) for i in rpy]
            print(s[0][0], s[0][1], s[0][2], "Euler:", rpy_deg)

    else:
        print("No solution found")


if __name__ == '__main__':
    plan(2)
