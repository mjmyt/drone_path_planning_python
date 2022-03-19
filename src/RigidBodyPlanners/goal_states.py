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

import numpy as np

"""
-2.21463 -2.96286 -0.00225225 0.701844 0.709669 0.830401 
This state didnt pass--->why?

"""


class CustomGoal(ob.Goal):
    def __init__(self, si):
        super(CustomGoal, self).__init__(si)

    def setGoalPosition(self, goal_pos):
        self.goal_pos = goal_pos

    def setRopeLength(self, L):
        self.L = L

    def isSatisfied(self, state):
        pos_sat = self.pos_check(state)
        yaw_sat = self.yaw_check(state)
        drones_dist_sat = self.drones_dist_check(state)
        drones_angle_sat = self.drones_angle_check(state)

        result = pos_sat and yaw_sat and drones_dist_sat and drones_angle_sat
        result = pos_sat

        if result:
            input("Goal reached")

        return result

    def set_thresholds(self, pos_threshold=0.2, yaw_threshold=np.pi/10, drones_distance_threshold=0.2, drones_angle_threshold=np.pi/10):
        self.pos_threshold = pos_threshold
        self.yaw_threshold = yaw_threshold
        self.drones_distance_threshold = drones_distance_threshold
        self.drones_angle_threshold = drones_angle_threshold

    def pos_check(self, state):
        pos = [state[0], state[1], state[2]]
        dist = np.linalg.norm(np.array(pos)-np.array(self.goal_pos))
        return dist < self.pos_threshold

    def yaw_check(self, state):
        yaw = state[3]
        yaw_err = np.abs(yaw)
        symmetric_yaw_err = np.abs(yaw - np.pi)

        return yaw_err < self.yaw_threshold or symmetric_yaw_err < self.yaw_threshold

    def drones_dist_check(self, state):
        drones_dist = state[4]
        return np.abs(drones_dist-0.5*self.L) < self.drones_distance_threshold

    def drones_angle_check(self, state):
        drones_angle = state[5]
        return np.abs(drones_angle) < self.drones_angle_threshold


class MyGoalRegion(ob.GoalRegion):
    def __init__(self, si):
        super(MyGoalRegion, self).__init__(si)
        self.setThreshold(0.25)
        self.drones_angle_threshold = np.deg2rad(10)
        self.yaw_threshold = np.deg2rad(10)

    def setGoalPosition(self, goal_pos):
        self.goal_pos = goal_pos

    def distanceGoal(self, state):
        pos = [state[0], state[1], state[2]]
        dist = [state[0]-self.goal_pos[0], state[1]-self.goal_pos[1], state[2]-self.goal_pos[2]]

        yaw_dist = self.get_yaw_distance(state[3])

        return np.sqrt(dist[0]**2 + dist[1]**2 + dist[2]**2) + yaw_dist

    # def isSatisfied(self, state):
        # return self.distanceGoal(state) < self.threshold and self.drones_angle_check(state)

    # def isSatisfied(self, state, threshold):
        # return self.distanceGoal(state) < threshold and self.drones_angle_check(state) and self.yaw_check(state)

    def drones_angle_check(self, state):
        drones_angle = state[5]
        return np.abs(drones_angle) < self.drones_angle_thresholds

    def yaw_check(self, state):
        yaw = state[3]
        yaw_err = np.abs(yaw)
        symmetric_yaw_err = np.abs(yaw - np.pi)

        return yaw_err < self.yaw_threshold or symmetric_yaw_err < self.yaw_threshold

    def get_yaw_distance(self, yaw):
        return min(np.abs(yaw), np.abs(yaw - np.pi))


if __name__ == "__main__":
    # space = ob.RealVectorStateSpace(6)

    # goal = CustomGoal(space)

    # print(goal.isSatisfied([0, 0, 0, np.pi/6, 0, 0]))
    # -1.85896 -3.3834 -0.152726 -2.77478 2.23709 0.105001

    pos = [-1.85896, -3.3834, -0.152726]
    goal_pos = [-2, -3, 0]
    dist = np.linalg.norm(np.array(pos)-np.array(goal_pos))

    print("pos: ", pos)
    print("goal_pos: ", goal_pos)
    print("dist: ", dist)
