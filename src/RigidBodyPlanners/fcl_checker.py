from math import pi
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import fcl
import numpy as np
from simplejson import load
from stl import Mesh, mesh
import tf.transformations
import os
from stl import mesh
print(os.getcwd())


class Fcl_mesh():
    def __init__(self, filename=None) -> None:
        if filename is not None:
            self.load_stl(filename)
            self.create_indexed_triangles(self.verts, self.vecs)
            self.create_fcl_mesh()

    def load_stl(self, filename):
        env_mesh = mesh.Mesh.from_file(filename)
        verts = np.around(np.unique(env_mesh.vectors.reshape(
            [int(env_mesh.vectors.size/3), 3]), axis=0), 2)
        vecs = np.around(env_mesh.vectors, 2)
        self.verts, self.vecs = verts, vecs

        return verts, vecs

    def create_indexed_triangles(self, vertices, vectors):
        """
        Create an indexed triangle mesh from a list of vertices and a list of vectors.
        """
        # remove duplicates from vertices not sure if this is correct
        #vertices = np.unique(vertices, axis=0)
        # Create indexed triangles
        tris = np.zeros([len(vectors),  3])
        for i, vec in enumerate(vectors):
            for j, p in enumerate(vec):
                index = np.where(np.all(p == vertices, axis=1))
                try:
                    tris[i][j] = index[0]
                except Exception as e:
                    print("index: ", index)
                    print("p: ", p)
                    print("vertices: ", vertices)
                    print(e)
                    tris[i][j] = index[0][0]

        self.tris = tris
        return tris

    def create_fcl_mesh(self):
        m = fcl.BVHModel()
        m.beginModel(len(self.verts), len(self.tris))
        # print(self.verts.shape, self.tris.shape)
        # print(self.verts)
        # print(self.tris)

        m.addSubModel(self.verts, self.tris)
        m.endModel()
        self.m = m

    def create_collision_object(self, mesh=None):
        T = fcl.Transform()
        if mesh is None:
            mesh = self.m

        self.collision_object = fcl.CollisionObject(mesh, T)

    def set_transform(self, T=[0, 0, 0], q=[0, 0, 0, 1]):

        q = [q[3], q[0], q[1], q[2]]  # from XYZW to WXYZ
        tf = fcl.Transform(q, T)

        self.collision_object.setTransform(tf)

    def update_vertices(self, verts):
        print(dir(self.m))
        self.m.beginUpdateModel()

        for vert in verts:
            self.m.updateVertex(vert)

        self.m.endUpdateModel()


def visualize_meshes(filenames):
    # Create a new plot
    figure = plt.figure()
    axes = mplot3d.Axes3D(figure)

    # Load the STL files and add the vectors to the plot
    for filename in filenames:
        your_mesh = mesh.Mesh.from_file(filename)
        axes.add_collection3d(
            mplot3d.art3d.Poly3DCollection(your_mesh.vectors))

    axes.set_xlabel('X')
    axes.set_ylabel('Y')
    axes.set_zlabel('Z')

    axes.set_xlim3d(-2, 2)
    axes.set_ylim3d(-2, 2)
    axes.set_zlim3d(-1, 2)

    # # Auto scale to the mesh size
    # scale = your_mesh.points.flatten(-1)
    # axes.auto_scale_xyz(scale, scale, scale)

    # Show the plot to the screen
    plt.show()


class Fcl_checker():
    def __init__(self, env_mesh_file, robot_mesh_file) -> None:
        self.env = Fcl_mesh(env_mesh_file)
        self.robot = Fcl_mesh(robot_mesh_file)

        # create collision objects
        self.robot.create_collision_object()
        self.env.create_collision_object()

        self.request = fcl.CollisionRequest()
        self.result = fcl.CollisionResult()

    def check_collision(self, T=None, q=[0, 0, 0, 1]):
        if T != None:
            self.robot.set_transform(T, q)

        is_collision = fcl.collide(
            self.robot.collision_object, self.env.collision_object, self.request, self.result)

        return is_collision

    def set_robot_transform(self, T, q=[0, 0, 0, 1]):
        self.robot.set_transform(T, q)

    def update_robot(self, custom_mesh: Fcl_mesh):
        self.robot = custom_mesh
        self.robot.create_collision_object()


def check_collision_detect_test():
    env_mesh_name = "src/drone_path_planning/resources/stl/env-scene-hole.stl"
    robot_mesh_name = "src/drone_path_planning/resources/stl/robot-scene-triangle.stl"
    robot_mesh_name = "src/drone_path_planning/resources/stl/custom_triangle_robot.stl"

    coll_checker = Fcl_checker(env_mesh_name, robot_mesh_name)
    q = tf.transformations.quaternion_from_euler(-pi/2, 0, 0)
    print(q)

    coll_checker.set_robot_transform(
        [-1.21917, -0.441611, -0.0462389], [-0.298798, 0.00548747, 0.0160421, 0.954166])

    print(coll_checker.check_collision())


if __name__ == '__main__':
    print("cwd", os.getcwd())

    env_mesh_name = "src/drone_path_planning/resources/stl/env-scene-hole.stl"
    robot_mesh_name = "src/drone_path_planning/resources/stl/robot-scene-triangle.stl"

    custom_file_name = "custom_mesh.stl"

    custom_robot_file_name = "src/drone_path_planning/resources/stl/custom_triangle_robot.stl"

    # p0 = np.array([-1.68, 1])
    # p1 = np.array([1.68, 1])

    # p0, p1 = drones_formation_2_triangle_points(1.68*2, np.deg2rad(20))
    # print(p0, p1)

    # p2 = np.array([0, -1])

    # create_3D_triangle_stl(p0, p1, p2, custom_file_name)

    # visualize_meshes([custom_robot_file_name])

    robot_V_filename = "src/drone_path_planning/resources/stl/robot-scene-V.stl"
    robot_mesh = Fcl_mesh()
    verts, vecs = robot_mesh.load_stl(robot_V_filename)
    tris = robot_mesh.create_indexed_triangles(verts, vecs)

    print(verts.shape, vecs.shape, tris.shape)
    print("========================== verts ==========================")
    for i, vert in enumerate(verts):
        print(i, vert)
    # print(verts)
    print("========================== vecs ===========================")
    print(vecs)
    print("========================== tris ==========================")
    print(tris)
