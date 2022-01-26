from math import pi
import numpy as np
import trimesh
from trimesh import viewer as trimesh_viewer
from trimesh.scene import Scene
import os
from geometry_msgs.msg import PoseStamped, Quaternion, Point
import tf.transformations

try:
    from .frameTransforms import transform
except:
    from frameTransforms import transform
# print("Current working directory:", os.getcwd())


class SepCollisionChecking:
    def __init__(self) -> None:
        self.load_meshes()
        self.init_manager()

    def init_manager(self):
        self.coll_manager = trimesh.collision.CollisionManager()
        self.coll_manager.add_object('env', self.env_mesh)
        self.coll_manager.add_object('robot', self.robot_mesh)

    def load_meshes(self):
        try:
            robot_mesh = trimesh.load_mesh(
                'ros_ws/src/drone_path_planning/resources/stl/robot-scene-safeScale.stl')
            env_mesh = trimesh.load_mesh(
                'ros_ws/src/drone_path_planning/resources/stl/env-scene-narrow.stl')
        except Exception as e:
            print("Error in loading meshes:", e)
            print("Working directory:", os.getcwd())
            print("Trying different path...")
            robot_mesh = trimesh.load_mesh(
                'crazyswarm/ros_ws/src/drone_path_planning/resources/stl/robot-scene-safeScale.stl')
            env_mesh = trimesh.load_mesh(
                'crazyswarm/ros_ws/src/drone_path_planning/resources/stl/env-scene-narrow.stl')

        self.robot_mesh = robot_mesh
        self.env_mesh = env_mesh

    def set_robot_transform(self, pos: list, orient: list):
        # get transfomration matrix
        mat = tf.transformations.quaternion_matrix(orient)
        mat[:3, 3] = pos[0], pos[1], pos[2]

        self.robot_mesh.apply_transform(mat)
        self.coll_manager.set_transform('robot', mat)

    def set_env_transform(self, pos: list, orient: list):
        # get transfomration matrix
        mat = tf.transformations.quaternion_matrix(orient)
        mat[:3, 3] = pos[0], pos[1], pos[2]

        self.env_mesh.apply_transform(mat)
        self.coll_manager.set_transform('env', mat)

    def collision_check(self):
        is_collision = self.coll_manager.in_collision_internal()
        return is_collision

    def visualize(self):
        scene = Scene()
        scene.add_geometry(self.robot_mesh, 'robot')
        scene.add_geometry(self.env_mesh, 'env')

        viewer = trimesh_viewer.SceneViewer(scene)


# Checker initialization
checker = SepCollisionChecking()
pose = PoseStamped()
pose.pose.position = Point(0, 0, 0)
pose.pose.orientation = Quaternion(0, 0, 0, 1)
transformed = transform(pose)

# q = tf.transformations.quaternion_from_euler(-pi/2, 0, 0)
pos = transformed.pose.position
q = transformed.pose.orientation

# checker.set_env_transform([0, 2.19, 0], [q[0], q[1], q[2], q[3]])
checker.set_env_transform([pos.x, pos.y, pos.z], [q.x, q.y, q.z, q.w])

if __name__ == "__main__":
    checker.set_robot_transform([2, 0, -2], [q.x, q.y, q.z, q.w])

    # coll_manager.set_transform('robot', mat)
    print("No collision:", not checker.collision_check())

    checker.visualize()
