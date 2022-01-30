#!/usr/bin/env python

import numpy as np


def normalize(v):
    norm = np.linalg.norm(v)
    assert norm > 0
    return v / norm


class Polynomial:
    def __init__(self, p):
        self.p = p

    # evaluate a polynomial using horner's rule
    def eval(self, t):
        assert t >= 0
        x = 0.0
        for i in range(0, len(self.p)):
            x = x * t + self.p[len(self.p) - 1 - i]
        return x

    # compute and return derivative
    def derivative(self):
        return Polynomial([(i+1) * self.p[i+1] for i in range(0, len(self.p) - 1)])

    def pol_coeffs_at_t(self, t):
        # calculate the coefficients of the polynomial at time t
        assert t >= 0
        coeffs = np.zeros(len(self.p))

        for i in range(0, len(self.p)):
            coeffs[i] = self.p[i] * (t**i)

        return coeffs


class TrajectoryOutput:
    def __init__(self):
        self.pos = None   # position [m]
        self.vel = None   # velocity [m/s]
        self.acc = None   # acceleration [m/s^2]
        self.omega = None  # angular velocity [rad/s]
        self.yaw = None   # yaw angle [rad]


# 4d single polynomial piece for x-y-z-yaw, includes duration.
class Polynomial4D:
    def __init__(self, duration, px, py, pz, pyaw):
        self.duration = duration
        self.px = Polynomial(px)
        self.py = Polynomial(py)
        self.pz = Polynomial(pz)
        self.pyaw = Polynomial(pyaw)

    # compute and return derivative
    def derivative(self):
        return Polynomial4D(
            self.duration,
            self.px.derivative().p,
            self.py.derivative().p,
            self.pz.derivative().p,
            self.pyaw.derivative().p)

    def eval(self, t):
        result = TrajectoryOutput()
        # flat variables
        result.pos = np.array(
            [self.px.eval(t), self.py.eval(t), self.pz.eval(t)])
        result.yaw = self.pyaw.eval(t)

        # 1st derivative
        derivative = self.derivative()
        result.vel = np.array(
            [derivative.px.eval(t), derivative.py.eval(t), derivative.pz.eval(t)])
        dyaw = derivative.pyaw.eval(t)

        # 2nd derivative
        derivative2 = derivative.derivative()
        result.acc = np.array([derivative2.px.eval(
            t), derivative2.py.eval(t), derivative2.pz.eval(t)])

        # 3rd derivative
        derivative3 = derivative2.derivative()
        jerk = np.array([derivative3.px.eval(
            t), derivative3.py.eval(t), derivative3.pz.eval(t)])

        thrust = result.acc + np.array([0, 0, 9.81])  # add gravity

        z_body = normalize(thrust)
        x_world = np.array([np.cos(result.yaw), np.sin(result.yaw), 0])
        y_body = normalize(np.cross(z_body, x_world))
        x_body = np.cross(y_body, z_body)

        jerk_orth_zbody = jerk - (np.dot(jerk, z_body) * z_body)
        h_w = jerk_orth_zbody / np.linalg.norm(thrust)

        result.omega = np.array(
            [-np.dot(h_w, y_body), np.dot(h_w, x_body), z_body[2] * dyaw])
        return result


class Trajectory:
    def __init__(self):
        self.polynomials = None
        self.duration = None

    def n_pieces(self):
        return len(self.polynomials)

    def loadcsv(self, filename):
        data = np.loadtxt(filename, delimiter=",",
                          skiprows=1, usecols=range(33))
        self.polynomials = [Polynomial4D(
            row[0], row[1:9], row[9:17], row[17:25], row[25:33]) for row in data]
        self.duration = np.sum(data[:, 0])

    def eval(self, t):
        assert t >= 0
        assert t <= self.duration

        current_t = 0.0
        for p in self.polynomials:
            if t <= current_t + p.duration:
                return p.eval(t - current_t)
            current_t = current_t + p.duration


class PiecewisePolynomial():
    """
    Piece-wise polynomial.

    THis class represent a piece-wise polynomail where the used polynomial
    to calculate the evaluation depends on time.

    Parameters
    ----------
    pols : list of Polynomial classes
        All the polynomials used.

    time_setpoints: list of floats
        The time points used to decide which polynomial to use

    """

    def __init__(self, pols: list, time_durations: list):
        self.pols = pols
        self.nOfPols = len(pols)

        # self.time_setpoints = np.zeros(self.nOfPols+1)
        self.time_durations = time_durations

    def eval(self, t):
        assert t >= 0
        # print("Durations: ", self.time_durations)
        # Evaluate resylt at time t.
        t_counting = 0
        for i in range(self.nOfPols+1):

            if i >= self.nOfPols:  # t bigger than whole duration
                # print("Calling pol {} with t:{}".format(i, t-sum(self.time_durations[:-1])))
                return self.pols[-1].eval(t-sum(self.time_durations[:-1]))

            if t < t_counting+self.time_durations[i]:
                # print("Calling pol {} with t:{}".format(i, t-t_counting))
                return self.pols[i].eval(t-t_counting)

            t_counting = t_counting + self.time_durations[i]


class Waypoint():
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    # constants
    WP_TYPE_X = 0
    WP_TYPE_Y = 1
    WP_TYPE_Z = 2
    WP_TYPE_YAW = 3

    def getType(self, type):
        if (type == Waypoint.WP_TYPE_X):
            return self.x
        elif (type == Waypoint.WP_TYPE_Y):
            return self.y
        elif (type == Waypoint.WP_TYPE_Z):
            return self.z
        elif (type == Waypoint.WP_TYPE_YAW):
            return self.yaw
        else:
            print("Sorry, invalid type")


class Point_time():
    # Class that combines waypoint and desired time for trajectory generation
    def __init__(self, wp: Waypoint, t: float):
        self.wp = wp
        self.t = t


class Point_time1D():
    # Class that combines waypoint and desired time for trajectory generation
    def __init__(self, wp: float, t: float):
        self.wp = wp
        self.t = t


if __name__ == "__main__":
    # pol = Polynomial([1, 1, 1, 1, 1, 1, 1, 1])
    # print(pol.pol_coeffs_at_t(2))

    pols = []
    pols.append(Polynomial([1, 0, ]))
    pols.append(Polynomial([-1, 1, ]))

    pc = PiecewisePolynomial(pols, time_setpoints=[1, 2])

    print(pc.eval(t=0.8))
