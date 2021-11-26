import math
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rospy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from .uav_trajectory import *


def visualize_python(tr: Trajectory, timestep: float):
    size = int(tr.duration / timestep+0.5)
    print("size:", size)
    x = np.zeros(size)
    y = np.zeros(size)
    z = np.zeros(size)

    for i, t in enumerate(np.arange(0, tr.duration, timestep)):
        out = tr.eval(t)
        out: TrajectoryOutput
        x[i], y[i], z[i] = out.pos[0], out.pos[1], out.pos[2]
        print(x[i], y[i], z[i])

    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')

    ax.plot(x, y, z)

    plt.show()


def get_nav_path_msg(tr: Trajectory, timestep: float):
    """
    Publish the ROS message containing the waypoints
    """
    msg = Path()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()

    size = int(tr.duration / timestep+0.5)
    print("size:", size)
    x = np.zeros(size)
    y = np.zeros(size)
    z = np.zeros(size)

    for i, t in enumerate(np.arange(0, tr.duration, timestep)):
        out = tr.eval(t)
        out: TrajectoryOutput

        pose = PoseStamped()
        pose.pose.position.x = out.pos[0]
        pose.pose.position.y = out.pos[1]
        pose.pose.position.z = out.pos[2]

        quaternion = tf.transformations.quaternion_from_euler(
            0, 0, -out.yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        msg.poses.append(pose)

    rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))
    return msg


if __name__ == "__main__":
    tr = Trajectory()
    tr.loadcsv(
        '/home/marios/crazyswarm/ros_ws/src/drone_path_planning/src/traj.csv')

    print("Trajectoty time duration:", tr.duration)
    timestep = 0.1

    visualize_python(tr, timestep)
