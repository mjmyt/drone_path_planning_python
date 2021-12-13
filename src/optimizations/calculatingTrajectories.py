# Useful link :https://realpython.com/linear-programming-python/
import numpy as np
from numpy.core.function_base import linspace
from uav_trajectory import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def calculate_trajectory1D(waypoints, wp_type=Waypoint.WP_TYPE_X):
    """
    waypoints: list of Point_Time

    wp_type: specifies the type of waypoint (x,y,z or yaw)

    """
    # If m is the number of waypoints, n is the number of polynomials
    m = len(waypoints)
    n = m - 1

    A = np.zeros((8*n, 8*n))
    b = np.zeros((8*n, 1))
    print("A.shape:", A.shape)
    print("b.shape:", b.shape)

    time_points = []
    for i, traj_point in enumerate(waypoints):
        traj_point: Point_time

        wp = traj_point.wp.getType(wp_type)
        t = traj_point.t
        time_points.append(t)

        pol = Polynomial([1, 1, 1, 1, 1, 1, 1, 1])

        if (i == 0 or i == n):  # start/end constraints

            for j in range(0, n):
                arr = np.array(pol.pol_coeffs_at_t(t))

                # padding with zeros
                arr = np.pad(arr, (8-len(arr), 0), 'constant')
                if i == 0:
                    A[j, 8*i:8*(i+1)] = arr
                else:
                    ind = -(4-j)
                    if ind == 0:
                        continue

                    A[ind, 8*(i-1):8*(i)] = arr

                pol = pol.derivative()

            tmp = np.array([wp, 0, 0, 0]).reshape((4, 1))

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

            b[endl] = wp
            b[endl+1] = wp

    polynomials_coefficients = np.linalg.solve(a=A, b=b)

    # print("polynomials_coefficients.shape:", polynomials_coefficients.shape)
    # np.savetxt("A.csv", A, delimiter=",")
    # np.savetxt("b.csv", b, delimiter=",")

    piece_pols = []  # piecewise polynomials
    for i in range(n):
        p = polynomials_coefficients[8*i:8*(i+1)]
        piece_pols.append(Polynomial(p))

    # tests
    for i, wp in enumerate(waypoints):
        t = wp.t
        print("i:", i)
        if i >= len(waypoints)-2:
            continue

        if wp_type != Waypoint.WP_TYPE_X:
            break
        if i == 0:
            print(f"pos at t={t} and pol={i}  -->{piece_pols[i].eval(t)}")
            print(
                f"vel at t={t} and pol={i}-->{piece_pols[i+0].derivative().eval(t)}")
            print(
                f"accel at t={t} and pol={i}-->{piece_pols[i+0].derivative().derivative().eval(t)}")

            t = t+1
            print(f"pos at t={t} and pol={i}  -->{piece_pols[i].eval(t)}")
            print(f"pos at t={t} and pol={i+1}-->{piece_pols[i+1].eval(t)}")

            print(
                f"vel at t={t} and pol={i}-->{piece_pols[i+0].derivative().eval(t)}")
            print(
                f"vel at t={t} and pol={i+1}-->{piece_pols[i+1].derivative().eval(t)}")
            print(
                f"accel at t={t} and pol={i}-->{piece_pols[i+0].derivative().derivative().eval(t)}")
            print(
                f"accel at t={t} and pol={i+1}-->{piece_pols[i+1].derivative().derivative().eval(t)}")

        else:
            t = t+1
            print(f"pos at t={t} and pol={i}  -->{piece_pols[i].eval(t)}")
            print(f"pos at t={t} and pol={i+1}-->{piece_pols[i+1].eval(t)}")

            print(
                f"vel at t={t} and pol={i}-->{piece_pols[i+0].derivative().eval(t)}")
            print(
                f"vel at t={t} and pol={i+1}-->{piece_pols[i+1].derivative().eval(t)}")
            print(
                f"accel at t={t} and pol={i}-->{piece_pols[i+0].derivative().derivative().eval(t)}")
            print(
                f"accel at t={t} and pol={i+1}-->{piece_pols[i+1].derivative().derivative().eval(t)}")

    total_pol = PiecewisePolynomial(piece_pols, time_points)

    return piece_pols, total_pol


def calculate_trajectory4D(waypoints):
    # waypoints:list of Point_time instances

    polx, pc_polx = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_X)
    poly, pc_poly = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_Y)
    polz, pc_polz = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_Z)
    polyaw, pc_polyaw = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_YAW)

    pols_coeffs = [polx, poly, polz, polyaw]
    pc_pols = [pc_polx, pc_poly, pc_polz, pc_polyaw]

    visualize_trajectory3D(pc_pols)


def visualize_trajectory3D(pols):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    N = 100
    time_frame = linspace(0, 10, N)
    x = np.zeros(N)
    y = np.zeros(N)
    z = np.zeros(N)
    yaw = np.zeros(N)

    for i, t in enumerate(time_frame):
        x[i] = pols[0].eval(t)
        y[i] = pols[1].eval(t)
        z[i] = pols[2].eval(t)

    ax.scatter(x, y, z, c='r', marker='o')

    plt.show()


if __name__ == "__main__":
    traj_points = []

    traj_points.append(Point_time(Waypoint(0.0, 0.0,  0.0, 0.0), t=0))
    traj_points.append(Point_time(Waypoint(2.0, 2.2,  0.3, 0.0), t=2))
    traj_points.append(Point_time(Waypoint(4.0, 8.0,  0.8, 0.0), t=4))
    traj_points.append(Point_time(Waypoint(0.0, 2.0, 0.4, 0.0), t=6))
    traj_points.append(Point_time(Waypoint(0.0, 0.0, 0.0, 0.0), t=8))

    calculate_trajectory4D(traj_points)

    # t = np.linspace(0, 5, 100)
    # y = [pol.eval(i) for i in t]

    # plt.figure()
    # plt.plot(t, y)
    # plt.grid(True)

    # plt.show()

    # t = 0
    # p = piece_pols[0]

    # print(f"pos at t={t} --> {p.eval(t)}")

    # t = 2
    # p = piece_pols[0].derivative().derivative()

    # print(f"vel at t={t} --> {p.eval(t)}")

    # t = 2
    # p = piece_pols[1].derivative().derivative()
    # print(f"vel at t={t} --> {p.eval(t)}")
