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


class CustomObjective(ob.StateCostIntegralObjective):
    def __init__(self, si, rope_length):
        super(CustomObjective, self).__init__(si, True)
        self.si_ = si
        self.L = rope_length

    # The requirement is to keep the drones distance away from 0 and rope length
    # in order to avoid tension of the rope or drones coliision in case of big trajectory following errors
    # So in order to avoid this we need to try to have the drone distance close to the midele of the rope (50%)
    # and set as cost the absolute difference from it

    def stateCost(self, s):
        # s is the state
        x, y, z = s[0], s[1], s[2]
        yaw = s[3]
        drones_distance = s[4]
        theta = s[5]

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
