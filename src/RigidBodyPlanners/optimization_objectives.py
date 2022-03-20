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
from RigidBodyPlanners import Custom_robot_mesh, Custom_robot_mesh_improvement
from RigidBodyPlanners.fcl_checker import Fcl_checker
from catenary import catenaries
import tf.transformations


class CustomObjective(ob.StateCostIntegralObjective):
    def __init__(self, si, rope_length):
        super(CustomObjective, self).__init__(si, True)
        self.si_ = si
        self.L = rope_length

    # The requirement is to keep the drones distance away from 0 and rope length
    # in order to avoid tension of the rope or drones coliision in case of big trajectory following errors
    # So in order to avoid this we need to try to have the drone distance close to the middle of the rope (50%)
    # and set as cost the absolute difference from it
    def stateCost(self, state):
        # state is the state
        x, y, z = state[0], state[1], state[2]
        yaw = state[3]
        drones_distance = state[4]
        theta = state[5]

        cost = np.abs(drones_distance-self.L*0.5)

        return ob.Cost(cost)


def getThresholdPathLengthObj(si, threshold):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(threshold))
    return obj


def getBalancedObjective(si, rope_length, cost_threshold):
    lengthObj = getThresholdPathLengthObj(si, cost_threshold)
    customObj = CustomObjective(si, rope_length)

    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(lengthObj, 5.0)
    opt.addObjective(customObj, 1.0)

    return opt


def getObstacleClearanceObjective(si, rope_legth: float, robot_mesh: Custom_robot_mesh, checker: Fcl_checker, threshold=0.1):
    obj = ObstaclelearanceObjective(si, rope_legth, robot_mesh, checker)
    obj.setCostThreshold(ob.Cost(threshold))
    return obj


class ObstaclelearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si, rope_legth: float, robot_mesh: Custom_robot_mesh, checker: Fcl_checker):
        super(ObstaclelearanceObjective, self).__init__(si, True)
        self.si_ = si

        self.L = rope_legth
        self.rb_mesh = robot_mesh
        self.checker = checker

    # The requirement is to keep the rigid body formation away from the obstacles
    # in order to avoid that the cost is the reciprocal of the distance to the obstacle
    def stateCost(self, state):
        # state is the state
        pos = state[0], state[1], state[2]

        yaw = state[3]
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        drones_distance = state[4]
        theta = state[5]

        self.rb_mesh.update_mesh(drones_distance, theta, self.L)
        self.checker.update_robot(self.rb_mesh.mesh)
        distance = self.checker.get_distance_from_obstacle(pos, q)

        return ob.Cost(1/distance)

    def __str__(self) -> str:
        thres = self.getCostThreshold().value()
        return "ObstaclelearanceObjective with threshold " + str(thres)
