import os
import sys
from stl import mesh
import numpy as np
import fcl
import os
from types import SimpleNamespace
from math import pi
try:
    from .fcl_checker import Fcl_mesh
except:
    from fcl_checker import Fcl_mesh


class Custom_robot_mesh():
    def __init__(self, drones_distance, theta, L, catenary_lowest_function, mesh_type=None) -> None:
        self.catenary_lowest_function = catenary_lowest_function
        self.safe_distances = False
        self.L = L

        self.enable_safe_distances(
            drones_distance=0.3, lowest_point_safety_distance=0.3)

        if mesh_type == None:
            print("mesh_type is None")
            sys.exit()
        elif mesh_type.lower() == "stl":
            self.MESH_TYPE = mesh.Mesh
        else:
            self.MESH_TYPE = Fcl_mesh

        self.create_custom_robot(drones_distance, theta, L)

    def enable_safe_distances(self, drones_distance, lowest_point_safety_distance):
        self.safe_distances = True
        self.drones_safety_distance = drones_distance
        self.lowest_point_safety_distance = lowest_point_safety_distance

    def drones_formation_2_triangle_points(self, drones_distance, theta):
        """
            This function gets the distnace between the 2 drones and the angle they form
            with the horizontal plane. It deeems a circle with radius equal to the distance/2
            and calculates the points of the triangle.
        """

        # get the distance between the 2 drones
        r = drones_distance/2
        y_offset = 0
        # get the points of the triangle
        p0 = np.array([r * np.cos(theta),  y_offset + r * np.sin(theta)])
        p1 = np.array([-r * np.cos(theta), y_offset + -r * np.sin(theta)])

        # print(np.linalg.norm(p0 - p1))
        if self != None:
            self.p0, self.p1 = p0, p1

        return p0, p1

    def get_V_3D_points(p0, p1, lower, upper):
        # Created a matrix with all the vertices needed for the 3D triangle
        thickness = 0.2  # thickness of the triangle ,maybe should be a parameter
        offset = 0  # TODO: makes this 0 (used for comapring with thhe old one)

        # print("p0: ", p0)
        # print("p1: ", p1)
        # print("p2: ", p2)
        verts = np.zeros((8, 3))

        verts[0, :] = [p0[0],  offset - thickness/2, p0[1]]
        verts[1, :] = [p0[0],  offset + thickness/2, p0[1]]

        verts[2, :] = [lower[0],  offset - thickness/2, lower[1]]
        verts[3, :] = [upper[0],  offset - thickness/2, upper[1]]

        verts[4, :] = [lower[0],  offset + thickness/2, lower[1]]
        verts[5, :] = [upper[0],  offset + thickness/2, upper[1]]

        verts[6, :] = [p1[0],  offset - thickness/2, p1[1]]
        verts[7, :] = [p1[0],  offset + thickness/2, p1[1]]

        # print("verts: ", verts)
        return verts

    def get_tris():
        # manual triangluation of the rigid body
        tris = np.zeros((8, 3), dtype=int)
        tris[0, :] = [0, 1, 2]
        tris[1, :] = [3, 4, 5]
        tris[2, :] = [0, 1, 4]
        tris[3, :] = [0, 3, 4]
        tris[4, :] = [0, 2, 3]
        tris[5, :] = [2, 3, 5]
        tris[6, :] = [1, 2, 4]
        tris[7, :] = [2, 4, 5]

        return tris

    def create_3D_triangle_stl(p0, p1, p2, custom_filename):
        # create mesh of stl.mesh.Mesh type
        verts = Custom_robot_mesh.get_triangle_3D_points(p0, p1, p2)
        tris = Custom_robot_mesh.get_tris()

        num_triangles = tris.shape[0]
        data = np.zeros(num_triangles, dtype=mesh.Mesh.dtype)

        for i, tr in enumerate(tris):
            data["vectors"][i] = np.array(
                [verts[tr[0]], verts[tr[1]], verts[tr[2]]])

        m = mesh.Mesh(data)
        m.save(custom_filename)
        print("Saved mesh to: ", custom_filename)
        return m

    def create_3D_triangle_fcl_mesh(p0, p1, p2):
        # create mesh of Fcl_mesh type
        verts = Custom_robot_mesh.get_triangle_3D_points(p0, p1, p2)
        tris = Custom_robot_mesh.get_tris()

        m = Fcl_mesh()
        m.verts = verts
        m.tris = tris
        m.create_fcl_mesh()

        # print("Created Fcl_Mesh ")
        return m

    def get_V_2D_points(self, drones_distance, theta, L) -> mesh.Mesh:
        """
        This function generated a 3D rigid trinagle body suitable for path planning of the drone swarm
        theta : represents the angle that is formed between the line connecting the drones and the horizontal plane
        """
        # Get first 2 points based on drones distance and theta
        p0, p1 = self.drones_formation_2_triangle_points(
            drones_distance, theta)

        p0, p1 = [p0[0], p0[1], 0], [p1[0], p1[1], 0]

        # print("p0: ", p0)
        # print("p1: ", p1)
        # Set the lowest point of the catenary formed by the 2 previous points
        # as the 3rd point of the catenary

        # lowest_point = self.catenary_lowest_function(p0, p1, L).lowest_point
        lowest_point = self.catenary_lowest_function(p0, p1, L)
        lowest_point = [lowest_point[0], lowest_point[2]]

        # safe distances calculations
        p0[0] += self.drones_safety_distance
        p1[0] += -self.drones_safety_distance

        upper_lowest_point = [lowest_point[0],
                              lowest_point[1] + self.lowest_point_safety_distance]
        lower_lowest_point = [lowest_point[0],
                              lowest_point[1] - self.lowest_point_safety_distance]

        return p0, p1, upper_lowest_point, lower_lowest_point

    def create_custom_robot(self, drones_distance, theta, L) -> mesh.Mesh:
        p0, p1, lowest_point = self.get_triangle_2D_points(
            drones_distance, theta, L)

        if self.MESH_TYPE == mesh.Mesh:
            filename = "src/drone_path_planning/resources/stl/custom_triangle_robot.stl"
            self.mesh = Custom_robot_mesh.create_3D_triangle_stl(
                p0, p1, lowest_point, filename)

        elif self.MESH_TYPE == Fcl_mesh:
            self.mesh = Custom_robot_mesh.create_3D_triangle_fcl_mesh(
                p0, p1, lowest_point)
        else:
            print("mesh_type is not one of the expected ones...")
            sys.exit()

        return self.mesh

    def update_verts(self, drones_distance, theta, L):
        p0, p1, lowest_point = self.get_triangle_2D_points(
            drones_distance, theta, L)

        verts = Custom_robot_mesh.get_triangle_3D_points(p0, p1, lowest_point)

        return verts

    def update_mesh(self, drones_distance, theta, L):
        if self.MESH_TYPE == mesh.Mesh:
            self.update_mesh_stl_mesh(drones_distance, theta, L)
        elif self.MESH_TYPE == Fcl_mesh:
            self.update_mesh_fcl_mesh(drones_distance, theta, L)
        else:
            print("mesh_type is not one of the expected ones...")
            sys.exit()

    def update_mesh_fcl_mesh(self, drones_distance, theta, L):
        # verts = self.update_verts(drones_distance, theta, L)

        # this one is not working on python-fcl (doesn't have the .begin_update function)
        # self.mesh.update_vertices(verts)

        # so I end up making a new mesh
        self.mesh = self.create_custom_robot(
            drones_distance, theta, L)

    def update_mesh_stl_mesh(self, drones_distance, theta, L):
        self.mesh = self.create_custom_robot(
            drones_distance, theta, L)  # TODO: should try not to  making a new mesh


class Custom_robot_mesh_improvement():
    """
    This class is used to improve the planning of the drone formation
    It doesn't assume that the drone formation is a triangle, but it can be a combination of 2 triangles.
    Check resources/explanations/custom_robot_mesh_improvement.png for more details
    """

    def __init__(self, drones_distance, theta, L, catenary_lowest_function, mesh_type=None) -> None:
        self.catenary_lowest_function = catenary_lowest_function
        self.safe_distances = False
        self.L = L

        self.enable_safe_distances(
            drones_distance=0.3, lowest_point_safety_distance=0.3)

        if mesh_type == None:
            print("mesh_type is None")
            sys.exit()
        elif mesh_type.lower() == "stl":
            self.MESH_TYPE = mesh.Mesh
        else:
            self.MESH_TYPE = Fcl_mesh

        self.create_custom_robot(drones_distance, theta, L)

    def enable_safe_distances(self, drones_distance, lowest_point_safety_distance):
        self.safe_distances = True
        self.drones_safety_distance = drones_distance
        self.lowest_point_safety_distance = lowest_point_safety_distance

    def drones_formation_2_triangle_points(self, drones_distance, theta):
        """
            This function gets the distnace between the 2 drones and the angle they form
            with the horizontal plane. It deeems a circle with radius equal to the distance/2
            and calculates the points of the triangle.
        """

        # get the distance between the 2 drones
        r = drones_distance/2
        y_offset = 0
        # get the points of the triangle
        p0 = np.array([r * np.cos(theta),  y_offset + r * np.sin(theta)])
        p1 = np.array([-r * np.cos(theta), y_offset + -r * np.sin(theta)])

        # print(np.linalg.norm(p0 - p1))
        if self != None:
            self.p0, self.p1 = p0, p1

        return p0, p1

    def get_triangle_3D_points(p0, p1, p2):
        # Created a matrix with all the vertices needed for the 3D triangle
        thickness = 0.2  # thickness of the triangle ,maybe should be a parameter
        offset = 0  # TODO: makes this 0 (used for comapring with thhe old one)

        # print("p0: ", p0)
        # print("p1: ", p1)
        # print("p2: ", p2)
        verts = np.zeros((6, 3))

        verts[0, :] = [p0[0],  offset + thickness/2, p0[1]]
        verts[1, :] = [p1[0],  offset + thickness/2, p1[1]]
        verts[2, :] = [p2[0],  offset + thickness/2, p2[1]]
        verts[3, :] = [p0[0],  offset + -thickness/2, p0[1]]
        verts[4, :] = [p1[0],  offset + -thickness/2, p1[1]]
        verts[5, :] = [p2[0],  offset + -thickness/2, p2[1]]

        # print("verts: ", verts)
        return verts

    def get_tris():
        # manual triangluation of the rigid body
        tris = [[7, 5, 3, ],
                [7, 3, 6, ],
                [2, 6, 3, ],
                [2, 3, 0, ],
                [0, 3, 5, ],
                [0, 5, 1, ],
                [1, 4, 2, ],
                [1, 2, 0, ],
                [4, 7, 6, ],
                [4, 6, 2, ],
                [1, 5, 4, ],
                [5, 7, 4, ]]
        tris = np.array(tris, dtype=int)

        return tris

    def create_3D_triangle_stl(p0, p1, p2, custom_filename):
        # create mesh of stl.mesh.Mesh type
        verts = Custom_robot_mesh.get_triangle_3D_points(p0, p1, p2)
        tris = Custom_robot_mesh.get_tris()

        num_triangles = tris.shape[0]
        data = np.zeros(num_triangles, dtype=mesh.Mesh.dtype)

        for i, tr in enumerate(tris):
            data["vectors"][i] = np.array(
                [verts[tr[0]], verts[tr[1]], verts[tr[2]]])

        m = mesh.Mesh(data)
        m.save(custom_filename)
        print("Saved mesh to: ", custom_filename)
        return m

    def create_3D_triangle_fcl_mesh(p0, p1, p2):
        # create mesh of Fcl_mesh type
        verts = Custom_robot_mesh.get_triangle_3D_points(p0, p1, p2)
        tris = Custom_robot_mesh.get_tris()

        m = Fcl_mesh()
        m.verts = verts
        m.tris = tris
        m.create_fcl_mesh()

        # print("Created Fcl_Mesh ")
        return m

    def get_triangle_2D_points(self, drones_distance, theta, L) -> mesh.Mesh:
        """
        This function generated a 3D rigid trinagle body suitable for path planning of the drone swarm
        theta : represents the angle that is formed between the line connecting the drones and the horizontal plane
        """
        # Get first 2 points based on drones distance and theta
        p0, p1 = self.drones_formation_2_triangle_points(
            drones_distance, theta)

        p0, p1 = [p0[0], p0[1], 0], [p1[0], p1[1], 0]

        # print("p0: ", p0)
        # print("p1: ", p1)
        # Set the lowest point of the catenary formed by the 2 previous points
        # as the 3rd point of the catenary

        # lowest_point = self.catenary_lowest_function(p0, p1, L).lowest_point
        lowest_point = self.catenary_lowest_function(p0, p1, L)
        lowest_point = [lowest_point[0], lowest_point[2]]

        # safe distances calculations
        if self.enable_safe_distances:
            p0[0] += self.drones_safety_distance
            p1[0] += -self.drones_safety_distance
            lowest_point[1] -= self.lowest_point_safety_distance

        return p0, p1, lowest_point

    def create_custom_robot(self, drones_distance, theta, L) -> mesh.Mesh:
        p0, p1, lower, upper = self.get_V_2D_points(
            drones_distance, theta, L)

        if self.MESH_TYPE == mesh.Mesh:
            filename = "src/drone_path_planning/resources/stl/custom_triangle_robot.stl"
            self.mesh = Custom_robot_mesh.create_3D_triangle_stl(
                p0, p1, lowest_point, filename)

        elif self.MESH_TYPE == Fcl_mesh:
            self.mesh = Custom_robot_mesh.create_3D_triangle_fcl_mesh(
                p0, p1, lowest_point)
        else:
            print("mesh_type is not one of the expected ones...")
            sys.exit()

        return self.mesh

    def update_verts(self, drones_distance, theta, L):
        p0, p1, lowest_point = self.get_triangle_2D_points(
            drones_distance, theta, L)

        verts = Custom_robot_mesh.get_triangle_3D_points(p0, p1, lowest_point)

        return verts

    def update_mesh(self, drones_distance, theta, L):
        if self.MESH_TYPE == mesh.Mesh:
            self.update_mesh_stl_mesh(drones_distance, theta, L)
        elif self.MESH_TYPE == Fcl_mesh:
            self.update_mesh_fcl_mesh(drones_distance, theta, L)
        else:
            print("mesh_type is not one of the expected ones...")
            sys.exit()

    def update_mesh_fcl_mesh(self, drones_distance, theta, L):
        # verts = self.update_verts(drones_distance, theta, L)

        # this one is not working on python-fcl (doesn't have the .begin_update function)
        # self.mesh.update_vertices(verts)

        # so I end up making a new mesh
        self.mesh = self.create_custom_robot(
            drones_distance, theta, L)

    def update_mesh_stl_mesh(self, drones_distance, theta, L):
        self.mesh = self.create_custom_robot(
            drones_distance, theta, L)  # TODO: should try not to  making a new mesh


def test_cat_lowest_function(p0, p1, L):
    return [0, 0, -0.5]


if __name__ == "__main__":
    print("cwd: ", os.getcwd())
    drones_distance = 1
    theta = 0
    L = 2

    mesh = Custom_robot_mesh(drones_distance, theta, L,
                             test_cat_lowest_function, mesh_type="stl")
