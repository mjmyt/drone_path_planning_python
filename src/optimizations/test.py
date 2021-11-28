# Useful link :https://realpython.com/linear-programming-python/
import numpy as np
from uav_trajectory import *


class Waypoint():
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw


class Point_time():
    # Class that combines waypoint and desired time for trajectory generation
    def __init__(self, wp: Waypoint, t: float):
        self.wp = wp
        self.t = t


traj_points = []

traj_points.append(Point_time(Waypoint(-1.0, 0.0,  0.0, 0.0), t=0))
traj_points.append(Point_time(Waypoint(5.0, 5.0,  0.0, 0.0), t=1))
traj_points.append(Point_time(Waypoint(5.0, 10.0, 0.0, 0.0), t=2))
traj_points.append(Point_time(Waypoint(0.0, 10.0, 0.0, 0.0), t=3))
traj_points.append(Point_time(Waypoint(+2.0, 0.0, 0.0, 0.0), t=4))


def calculate_trajectory(waypoints):
    # If m is the number of waypoints, n is the number of polynomials
    m = len(waypoints)
    n = m - 1

    A = np.zeros((8*n, 8*n))
    b = np.zeros((8*n, 1))
    print("A.shape:", A.shape)
    print("b.shape:", b.shape)

    for i, traj_point in enumerate(traj_points):
        wp = traj_point.wp
        t = traj_point.t

        print("i:", i)
        pol = Polynomial([1, 1, 1, 1, 1, 1, 1, 1])

        if (i == 0 or i == n):  # start/end constraints

            for j in range(0, n):
                arr = np.array(pol.pol_coeffs_at_t(t))

                # padding with zeros
                arr = np.pad(arr, (8-len(arr), 0), 'constant')
                if i == 0:
                    A[j, 8*i:8*(i+1)] = arr
                else:
                    A[-(4-j), 8*(i-1):8*(i)] = arr

                pol = pol.derivative()

            tmp = np.array([wp.x, 0, 0, 0]).reshape((4, 1))

            if i == 0:
                b[0:4] = tmp
            else:
                b[-4:] = tmp

        else:  # continuity constraints
            array_to_add = np.zeros((8, 8))
            for j in range(0, 8):
                vec = np.array(pol.pol_coeffs_at_t(t))

                # padding with zeros
                vec = np.pad(vec, (8-len(vec), 0), 'constant')
                array_to_add[j, :] = vec
                pol = pol.derivative()

            startl = 4+(i-1)*8  # start line index
            endl = 4+(i-1)*8 + 6   # end line index
            # conitnuity constraints
            A[startl:endl, 8*(i-1):8*(i)] = array_to_add[1:7, :]
            A[startl:endl, 8*(i):8*(i+1)] = -array_to_add[1:7, :]

            b[startl:endl] = np.zeros((6, 1))

            # waypoints constraints
            A[endl,  8*(i-1):8*(i)] = array_to_add[0, :]
            A[endl+1, 8*(i):8*(i+1)] = array_to_add[0, :]

            b[endl] = wp.x
            b[endl+1] = wp.x

    polynomials_coefficients = np.linalg.solve(a=A, b=b)

    print("n:", n)

    piece_pols = []  # piecewise polynomials
    for i in range(n):
        p = polynomials_coefficients[8*i:8*(i+1)]
        piece_pols.append(Polynomial(p))

    print("polynomials_coefficients.shape:", polynomials_coefficients.shape)
    np.savetxt("A.csv", A, delimiter=",")
    np.savetxt("b.csv", b, delimiter=",")

    return piece_pols


if __name__ == "__main__":
    piece_pols = calculate_trajectory(traj_points)

    for p in piece_pols:
        print(p.p)

    # testing pol values

    t = 2
    p = piece_pols[2].derivative().derivative()

    print(f"vel at t={t} --> {p.eval(t)}")

    t = 2
    p = piece_pols[1].derivative().derivative()
    print(f"vel at t={t} --> {p.eval(t)}")
