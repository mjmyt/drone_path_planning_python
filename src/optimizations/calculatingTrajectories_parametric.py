# Useful link :https://realpython.com/linear-programming-python/
import time
from genpy import Duration
import numpy as np
from numpy.core.function_base import linspace
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sympy import primitive

try:
    from uav_trajectory import *
except:
    from .uav_trajectory import *

#################################POL GENERATOR OVERVIEW#######################################
"""
A 7th rank polynomial is used (t^7)

First wp(waipont) conditions:
    x(0) = waypoint(0)
    x'(0) = x''(0)=x'''(0)= 0

i-th waypoint conditions:
    x_i-1(ti)    =  waypoint(i)
    x_i-1'(ti)   =  x_i'(ti)
    x_i-1''(ti)  =  x_i''(ti)
    x_i-1'''(ti) =  x_i'''(ti)
    x_i-1(4)(ti) =  x_i(4)(ti)
    x_i-1(5)(ti) =  x_i(5)(ti)
    x_i-1(6)(ti) =  x_i(6)(ti)
    x_i(ti)      =  waypoint(i)

Last wp(waipont) conditions:
    x(-1) = waypoint(-1)
    x'(-1) = x''(-1)=x'''(-1)= 0
"""
##############################################################################################


POL_RANK = 3
POL_COEFFS = POL_RANK+1


def calculate_trajectory1D(waypoints, wp_type=Waypoint.WP_TYPE_X):
    """
    waypoints: list of Point_Time

    wp_type: specifies the type of waypoint (x,y,z or yaw)

    """
    # If m is the number of waypoints, n is the number of polynomials
    m = len(waypoints)
    n = m - 1
    # print("m:", m, "n:", n)
    A = np.zeros((POL_COEFFS*n, POL_COEFFS*n))
    b = np.zeros((POL_COEFFS*n, 1))
    # print("A.shape:", A.shape)
    # print("b.shape:", b.shape)

    time_points = []
    prev_t = 0
    for i, traj_point in enumerate(waypoints):
        traj_point: Point_time

        wp = traj_point.wp.getType(wp_type)
        t = traj_point.t-prev_t
        if i != 0:
            time_points.append(t)

        # pol = Polynomial([1, 1, 1, 1, 1, 1, 1, 1])
        pol = Polynomial(np.ones(POL_COEFFS))

        if (i == 0 or i == n):  # start/end constraints

            for j in range(0, int(POL_COEFFS/2)):
                arr = np.array(pol.pol_coeffs_at_t(t))

                # padding with zeros
                arr = np.pad(arr, (POL_COEFFS-len(arr), 0), 'constant')
                if i == 0:
                    A[j, POL_COEFFS*i:POL_COEFFS*(i+1)] = arr
                else:
                    ind = -(int(POL_COEFFS/2)-j)
                    if ind >= 0:
                        continue

                    A[ind, POL_COEFFS*(i-1):POL_COEFFS*(i)] = arr
                pol = pol.derivative()

            # tmp = np.array([wp, 0, 0, 0]).reshape((4, 1))

            tmp = np.zeros((int(POL_COEFFS/2), 1))
            tmp[0] = wp

            if i == 0:
                b[0:int(POL_COEFFS/2)] = tmp
            else:
                b[-int(POL_COEFFS/2):] = tmp

        else:  # continuity constraints
            array_to_add_prev = np.zeros((POL_COEFFS, POL_COEFFS))
            for j in range(0, POL_COEFFS):
                vec = np.array(pol.pol_coeffs_at_t(t))

                # padding with zeros
                vec = np.pad(vec, (POL_COEFFS-len(vec), 0), 'constant')
                array_to_add_prev[j, :] = vec
                pol = pol.derivative()

            # TODO: Make this a separate function
            # pol = Polynomial([1, 1, 1, 1, 1, 1, 1, 1])
            pol = Polynomial(np.ones(POL_COEFFS))

            array_to_add_next = np.zeros((POL_COEFFS, POL_COEFFS))
            for j in range(0, POL_COEFFS):
                # t=0 because it is the start of the next polynomial
                vec = np.array(pol.pol_coeffs_at_t(t=0))

                # padding with zeros
                vec = np.pad(vec, (POL_COEFFS-len(vec), 0), 'constant')
                array_to_add_next[j, :] = vec
                pol = pol.derivative()

            # print("array_to_add_prev:", array_to_add_prev)
            # print("array_to_add_next:", array_to_add_next)

            startl = int((POL_COEFFS/2)) + (i-1)*POL_COEFFS  # start line index
            endl = int((POL_COEFFS/2)) + (i-1)*POL_COEFFS + (POL_COEFFS-2)   # end line index
            # conitnuity constraints
            A[startl:endl, POL_COEFFS*(i-1):POL_COEFFS*(i)] = array_to_add_prev[1: POL_COEFFS-1, :]
            A[startl:endl, POL_COEFFS*(i):POL_COEFFS*(i+1)] = -array_to_add_next[1:POL_COEFFS-1, :]

            b[startl:endl] = np.zeros((POL_COEFFS-2, 1))

            # waypoints constraints
            A[endl,  POL_COEFFS*(i-1):POL_COEFFS*(i)] = array_to_add_prev[0, :]
            A[endl+1, POL_COEFFS*(i):POL_COEFFS*(i+1)] = array_to_add_next[0, :]

            b[endl] = wp
            b[endl+1] = wp

        # copy the time
        prev_t = traj_point.t

    if calculate_trajectory1D.counter == 0:
        print("det(A):", np.linalg.det(A))
        np.savetxt("A.csv", A, delimiter=",")
        np.savetxt("b.csv", b, delimiter=",")

    calculate_trajectory1D.counter += 1

    t0 = time.time()
    polynomials_coefficients = np.linalg.solve(a=A, b=b)
    dt = time.time()-t0
    print("Time to solve matrix equation:", dt*1000, "ms")
    # print("polynomials_coefficients.shape:", polynomials_coefficients.shape)

    piece_pols = []  # piecewise polynomials
    for i in range(n):
        p = polynomials_coefficients[POL_COEFFS*i:POL_COEFFS*(i+1)]
        piece_pols.append(Polynomial(p))

    # tests
    DEBUG = 0
    if DEBUG:
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

                t = waypoints[i+1].t
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
                t = waypoints[i+1].t
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
    # t_final = sum(total_pol.time_durations)
    # print("t_final:", t_final)
    # for t in linspace(0, t_final, 100):
    # print(f"t={t} --> {total_pol.eval(t)}")

    return piece_pols, total_pol


calculate_trajectory1D.counter = 0


def calculate_trajectory4D(waypoints):
    # waypoints:list of Point_time instances

    polx, pc_polx = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_X)
    poly, pc_poly = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_Y)
    polz, pc_polz = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_Z)
    polyaw, pc_polyaw = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_YAW)

    pols_coeffs = [polx, poly, polz, polyaw]
    pc_pols = [pc_polx, pc_poly, pc_polz, pc_polyaw]

    # visualize_trajectory3D(pc_pols)

    return pols_coeffs, pc_pols


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


timestep = 2
test_data = [
    [-1.0, 5.0, 1.0, 0.0],
    [-0.9105214656082087, 4.866527813557898, 0.9821609406403813, 0.02039080103534039],
    [-0.8225743363589189, 4.73288554489947, 0.964441662764501, 0.04077309721300639],
    [-0.7361272257466368, 4.5990008095166175, 0.946841139664208, 0.06113820463325185],
    [-0.6511925243577015, 4.464831726680945, 0.9293520786300249, 0.08147761001670759],
    [-0.5677385453806084, 4.3303072795034305, 0.9119680575644483, 0.10178299225536093],
    [-0.4857652247831987, 4.19536539455793, 0.8946893929748612, 0.12204500119949559],
    [-0.40523866600088, 4.05994725771634, 0.8775065705250998, 0.14225598708544368],
    [-0.3261547497876769, 3.923993481128284, 0.8604168009043632, 0.16240701194973708],
    [-0.2484752985307498, 3.78743548705188, 0.84341227981323, 0.1824899882345422],
    [-0.17219220053404993, 3.6502373470789204, 0.8264894614074699, 0.20249698043707148],
    [-0.09725801527295008, 3.51232995673728, 0.8096428181876703, 0.22241799607297275],
    [-0.02365621826047004, 3.37365871553904, 0.7928635266349502, 0.24224499181234838],
    [0.04864606691479989, 3.23417214717458, 0.7761488620638199, 0.2619709834334211],
    [0.11968750758167002, 3.0938169790745595, 0.75949244060815, 0.2815870183901405],
    [0.18949994071968002, 2.95253000758626, 0.7428843320283001, 0.3010839819518382],
    [0.2581249596866899, 2.8102848216266, 0.72632388402931, 0.32045501331803433],
    [0.32560330474396, 2.667021932848, 0.70980157478244, 0.3396910397023212]]

test_data_osc = [
    [0.3284945525231711,     2.8135137700356214,     0.3],
    [0.32265292238486715,    2.845155448796263,      0.3007139556664981],
    [0.30909738716671,       2.927032162051976,      0.30402359711924526],
    [0.2942661908082198,     3.0123615249743874,     0.3076859124739516],
    [0.2906073399503452,     3.0772601461173585,     0.31468395198426724],
    [0.2891399846125392,     3.1043367695181456,     0.3176428945979113],
    [0.2886227334897987,     3.1642368076931255,     0.32559356881757073],
    [0.28579988833538106,    3.7004467238423824,     0.39771167320305206],
    [0.2894276235804048,     3.993910352201953,      0.4398021768450572],
    [0.2984637096077048,     4.570337581374501,      0.5235207956143707],
    [0.30246467531222865,    4.809651806246219,      0.5584457234841543],
    [0.3046483546185627,     4.89040293479041,       0.5706821607402336],
    [0.3066483583487096,     4.941871765475373,      0.5788144021502838],
    [0.32903149050331554,    5.480404360184069,      0.6653526783438337],
    [0.34274059221445585,    5.836034451260831,      0.7226590868575216],
    [0.34560848218353507,    5.907930175708629,      0.7344025879425593],
    [0.3471219313979523,     5.942348314795831,      0.7401340619220861],
    [0.3499999940395355,     6.0,                    0.75]
]

if __name__ == "__main__":
    traj_points = []
    data_to_test = test_data_osc
    # traj_points.append(Point_time(Waypoint(0.0, 0.0,  0.0, 0.0), t=0))
    # traj_points.append(Point_time(Waypoint(2.0, 2.2,  0.3, 0.0), t=1))
    # traj_points.append(Point_time(Waypoint(4.0, 8.0,  0.8, 0.0), t=3))
    # # traj_points.append(Point_time(Waypoint(0.0, 2.0, 0.4, 0.0), t=4))
    # # traj_points.append(Point_time(Waypoint(0.0, 0.0, 0.0, 0.0), t=5))
    print("Waypoints number:", len(data_to_test))

    for i, point in enumerate(data_to_test):
        traj_points.append(Point_time(Waypoint(point[0], point[1], point[2], 0), t=i*timestep))

    t0 = time.time()
    pols_coeffs, pc_pols = calculate_trajectory4D(traj_points)
    dt = time.time() - t0
    print("dt:", dt*1000)

    PLOT_POINTS = 50
    pol_x: PiecewisePolynomial = pc_pols[0]
    dur = sum(pol_x.time_durations)
    dt = dur/PLOT_POINTS

    xs, ys, zs = [], [], []
    vel_xs, vel_ys, vel_zs = [], [], []
    acc_xs, acc_ys, acc_zs = [], [], []
    ts = []
    for i in range(PLOT_POINTS):
        t = i*dt
        ts.append(t)

        xs.append(pc_pols[0].eval(t))
        ys.append(pc_pols[1].eval(t))
        zs.append(pc_pols[2].eval(t))

        vel_xs.append(pc_pols[0].derivative().eval(t))
        vel_ys.append(pc_pols[1].derivative().eval(t))
        vel_zs.append(pc_pols[2].derivative().eval(t))

        acc_xs.append(pc_pols[0].derivative().derivative().eval(t))
        acc_ys.append(pc_pols[1].derivative().derivative().eval(t))
        acc_zs.append(pc_pols[2].derivative().derivative().eval(t))

    # # # 3Dplot them
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')

    # ax.scatter(xs, ys, zs, c='r', marker='o')
    # plt.show()

    plt.figure()

    plt.subplot(3, 4, 1,)
    plt.title("Original")
    plt.scatter([i*timestep for i in range(len(data_to_test))], [point[0] for point in data_to_test], c='r', marker='o')
    plt.grid(True)

    plt.subplot(3, 4, 5)
    plt.scatter([i*timestep for i in range(len(data_to_test))], [point[1] for point in data_to_test], c='r', marker='o')
    plt.grid(True)
    plt.subplot(3, 4, 9)
    plt.scatter([i*timestep for i in range(len(data_to_test))], [point[2] for point in data_to_test], c='r', marker='o')
    plt.grid(True)

    plt.subplot(3, 4, 2)
    plt.title("Position")
    plt.plot(ts, xs)
    plt.scatter([i*timestep for i in range(len(data_to_test))], [point[0] for point in data_to_test], c='r', marker='o')
    plt.grid(True)
    plt.subplot(3, 4, 6)
    plt.plot(ts, ys)
    plt.scatter([i*timestep for i in range(len(data_to_test))], [point[1] for point in data_to_test], c='r', marker='o')
    plt.grid(True)
    plt.subplot(3, 4, 10)
    plt.plot(ts, zs)
    plt.scatter([i*timestep for i in range(len(data_to_test))], [point[2] for point in data_to_test], c='r', marker='o')
    plt.grid(True)

    plt.subplot(3, 4, 3)
    plt.title("Velocity")
    plt.plot(ts, vel_xs)
    plt.scatter([i*timestep for i in range(len(data_to_test))], [point[0]/100 for point in data_to_test], c='r', marker='o')
    plt.grid(True)
    plt.subplot(3, 4, 7)
    plt.plot(ts, vel_ys)
    plt.grid(True)
    plt.subplot(3, 4, 11)
    plt.grid(True)
    plt.plot(ts, vel_zs)

    plt.subplot(3, 4, 4)
    plt.title("Acceleration")
    plt.plot(ts, acc_xs)
    plt.grid(True)
    plt.subplot(3, 4, 8)
    plt.plot(ts, acc_ys)
    plt.grid(True)
    plt.subplot(3, 4, 12)
    plt.plot(ts, acc_zs)
    plt.grid(True)

    # plt.draw()
    # plt.pause(0.001)
    plt.show()
