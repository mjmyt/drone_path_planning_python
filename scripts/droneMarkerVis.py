#!/usr/bin/env python3

import rospy
from tf import TransformListener, transformations
import numpy as np
import os
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf

DRONES_NUMBER = 5


class DroneMarker(Marker):
    def __init__(self, id, pos=[0, 0, 0], rot=[0, 0, 0, 0]):
        super().__init__()
        self.header.frame_id = "world"
        self.header.stamp = rospy.get_rostime()
        self.ns = "cf"
        self.id = id
        self.type = Marker.MESH_RESOURCE
        self.mesh_resource = "package://3Dvisualization/resources/Quadcopter.stl"
        self.action = 0

        self.updatePose(pos, rot)

        scale_fac = 1/400
        self.scale.x = scale_fac
        self.scale.y = scale_fac
        self.scale.z = scale_fac

        self.color.r = 0.0
        self.color.g = 1.0
        self.color.b = 0.0
        self.color.a = 1.0

        self.lifetime = rospy.Duration(0)

    def updatePose(self, pos, quatern):
        self.pose.position.x = pos[0]
        self.pose.position.y = pos[1]
        self.pose.position.z = pos[2]

        rpy = transformations.euler_from_quaternion(quatern)

        quatern = transformations.quaternion_from_euler(
            rpy[0] + pi/2, rpy[1], rpy[2])

        self.pose.orientation.x = quatern[0]
        self.pose.orientation.y = quatern[1]
        self.pose.orientation.z = quatern[2]
        self.pose.orientation.w = quatern[3]


class DroneMarkersArray(MarkerArray):
    def __init__(self):
        super().__init__()
        self.markers = []
        self.id_index_dict = {}

    def update(self, id, trans, rot):
        if id not in self.id_index_dict.keys():
            self.markers.append(DroneMarker(id, trans, rot))
        else:
            index = self.id_index_dict[id]
            self.markers[index].updatePose(trans, rot)


if __name__ == "__main__":

    rospy.init_node("droneMarkerVis", anonymous=True)
    listener = TransformListener()

    rate = rospy.Rate(30.0)  # hz

    dronesMarkPub = rospy.Publisher(
        'dronesMarkers_array',  MarkerArray, queue_size=10)

    drones = DroneMarkersArray()
    while not rospy.is_shutdown():

        for id in range(DRONES_NUMBER):
            try:
                (trans, rot) = listener.lookupTransform(
                    '/world', "/cf" + str(id), rospy.Time(0))
                # print(f"     Found /cf{id} at position {trans}")

                drones.update(id, trans, rot)

            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        dronesMarkPub.publish(drones)
        rate.sleep()
