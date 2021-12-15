import numpy as np
import tf
from geometry_msgs.msg import TransformStamped, Quaternion
from sep_collision_checking import *


def isBetween(x, min, max):
    return x >= min and x <= max


path_file = "path.txt"
try:
    data = np.loadtxt(path_file)
except Exception as e:
    data = np.loadtxt("crazyswarm/"+path_file)

pos = data[:, :3]
orient = data[:, 3:7]

max_angle = np.deg2rad(5)
print("Max angle in degrees:", np.rad2deg(max_angle))

for i in range(data.shape[0]):
    q = [orient[i, 0], orient[i, 1], orient[i, 2], orient[i, 3]]
    euler = tf.transformations.euler_from_quaternion(q)

    checks = []
    checks.append(isBetween(euler[1], -max_angle, max_angle))
    checks.append(isBetween(euler[2], -max_angle, max_angle))

    checker.set_robot_transform(pos[i, :], q)
    no_collision = not checker.collision_check()

    if not all(checks):
        print("Invalid orientation:", np.rad2deg(euler))

    if not no_collision:
        print("Collision at:", i)
        print(pos[i, :])

    # print(euler)
