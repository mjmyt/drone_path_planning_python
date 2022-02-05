#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import time
import numpy
import math
xpos = 0
ypos = 0
zpos = 0
xref = 0
yref = 0
zref = 0
yawref = 0
qx = 0
qy = 0
qz = 0
qw = 0
d_yaw = 0
vx = 0
vy = 0
vz = 0
roll = 0
pitch = 0
yaw = 0


def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [roll, pitch, yaw]

# We read the odometry message from the motion capture system


def callback(data):
    global xpos, ypos, zpos, vx, vy, vz, roll, pitch, yaw, d_yaw
    xpos = data.pose.pose.position.x
    ypos = data.pose.pose.position.y
    zpos = data.pose.pose.position.z
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    d_yaw = data.twist.twist.angular.z
    [roll, pitch, yaw] = quaternion_to_euler(qx, qy, qz, qw)

    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z


def callback_ref(data):
    print("Callback ref")
    global xref, yref, zref, yawref
    xref = data.pose.position.x
    yref = data.pose.position.y
    zref = data.pose.position.z
    yawref = data.pose.orientation.z


def callback_safety(data):
    print("Callback safety")
    global land_flag
    land_flag = 1


def controller():
    pub = rospy.Publisher('demo_crazyflie4/cmd_vel', Twist, queue_size=1)
    rospy.init_node('controller', anonymous=True)
    sub = rospy.Subscriber(
        '/pixy/vicon/demo_crazyflie4/demo_crazyflie4/odom', Odometry, callback)
    sub_safety = rospy.Subscriber('safety_land', String, callback_safety)
    sub_ref = rospy.Subscriber('reference', PoseStamped, callback_ref)

    rate = rospy.Rate(20)  # 20hz
    global xref, yref, zref, integrator, land_flag, yawref
    xref = 0
    yref = 4
    zref = 1
    yawref = 0
    to_thrust = 0.6
    land_flag = 0

    t = 0
    integrator = 0
    # CONTROLLER GAINS
    k_px = 0.5
    k_py = 0.5
    k_pz = 0.5
    k_vx = 0.5
    k_vy = 0.5
    k_vz = 0.5
    k_y = 1
    k_dy = 0.2

    while not rospy.is_shutdown():

        # Apply rotations around the z-axis (yaw angle)

        xpos_body = math.cos(yaw)*xpos + math.sin(yaw)*ypos
        ypos_body = -math.sin(yaw)*xpos + math.cos(yaw)*ypos

        xref_body = math.cos(yaw)*xref + math.sin(yaw)*yref
        yref_body = -math.sin(yaw)*xref + math.cos(yaw)*yref

        # print(xref, yref, zref, yawref)

        # Implement your controller
        integrator = integrator + 0.001*(zref-zpos)
        ang_diff = numpy.mod(yawref - yaw + math.pi, 2*math.pi) - math.pi

        u_p = k_vx*(k_px*(xref_body-xpos_body) - vx)
        u_r = k_vy*(k_py*(yref_body-ypos_body) - vy)
        u_t = to_thrust + integrator + k_vz*(k_pz*(zref-zpos) - vz)
        u_y = k_y*ang_diff - k_dy*d_yaw

        if u_p > 0.25:
            u_p = 0.25

        if u_p < -0.25:
            u_p = -0.25

        if u_r > 0.25:
            u_r = 0.25

        if u_r < -0.25:
            u_r = -0.25

        if u_t > 1:
            u_t = 1

        if u_t < 0:
            u_t = 0

        if land_flag == 1:
            u_t = 0.55

        cmd_vel = Twist()
        cmd_vel.linear.x = u_p
        cmd_vel.linear.y = u_r
        cmd_vel.linear.z = u_t
        cmd_vel.angular.z = u_y
        pub.publish(cmd_vel)
        rate.sleep()

        t = t + 1


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
