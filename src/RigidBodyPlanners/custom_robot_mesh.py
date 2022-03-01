from .fcl_checker import Fcl_mesh
from stl import mesh
import numpy as np


class Custom_robot_mesh():
    def __init__(self, drones_distance, theta, L, catenary_lowest_function) -> None:
        self.create_custom_robot(
            drones_distance, theta, L, catenary_lowest_function)

    def drones_formation_2_triangle_points(self, drones_distance, theta):
        """
            This function gets the distnace between the 2 drones and the angle they form
            with the horizontal plane. It deeems a circle with radius equal to the distance/2
            and calculates the points of the triangle. 
        """

        # get the distance between the 2 drones
        r = drones_distance/2
        y_offset = 1
        # get the points of the triangle
        self.p0 = np.array([r * np.cos(theta),  y_offset + r * np.sin(theta)])
        self.p1 = np.array([-r * np.cos(theta), y_offset + -r * np.sin(theta)])

        # print(np.linalg.norm(p0 - p1))

        return self.p0, self.p1

    def create_3D_triangle_stl(p0, p1, p2, custom_filename):
        robot = Fcl_mesh()
        # create 3d triangle mesh

        thickness = 0.3
        offset = 1

        robot.verts = []
        robot.verts.append([p0[0],  offset + thickness/2, p0[1]])
        robot.verts.append([p1[0],  offset + thickness/2, p1[1]])
        robot.verts.append([p2[0],  offset + thickness/2, p2[1]])
        robot.verts.append([p0[0],  offset + -thickness/2, p0[1]])
        robot.verts.append([p1[0],  offset + -thickness/2, p1[1]])
        robot.verts.append([p2[0],  offset + -thickness/2, p2[1]])

        robot.tris = []
        robot.tris.append([0, 1, 2])
        robot.tris.append([3, 4, 5])

        robot.tris.append([0, 1, 4])
        robot.tris.append([0, 3, 4])

        robot.tris.append([0, 2, 3])
        robot.tris.append([2, 3, 5])

        robot.tris.append([1, 2, 4])
        robot.tris.append([2, 4, 5])

        robot.verts = np.array(robot.verts)
        robot.tris = np.array(robot.tris)

        num_triangles = len(robot.tris)
        verts = robot.verts
        data = np.zeros(num_triangles, dtype=mesh.Mesh.dtype)

        for i, tr in enumerate(robot.tris):
            data["vectors"][i] = np.array(
                [verts[tr[0]], verts[tr[1]], verts[tr[2]]])

        m = mesh.Mesh(data)
        m.save(custom_filename)

        return m

    def create_custom_robot(self, drones_distance, theta, L, catenary_lowest_function) -> mesh.Mesh:
        """
        This function generated a 3D rigid trinagle body suitable for path planning of the drone swarm
        theta : represents the angle that is formed between the line connecting the drones and the horizontal plane 
        """
        # Get first 2 points based on drones distance and theta
        p0, p1 = self.drones_formation_2_triangle_points(
            drones_distance, theta)
        p0, p1 = [p0[0], p0[1], 0], [p1[0], p1[1], 0]

        # Set the lowest point of the catenary formed by the 2 previous points
        # as the 3rd point of the catenary

        lowest_point = catenary_lowest_function(p0, p1, L).lowest_point
        lowest_point = [lowest_point[0], lowest_point[2]]

        self.mesh = Custom_robot_mesh.create_3D_triangle_stl(p0, p1, lowest_point,
                                                             "custom_triangle_robot.stl")

        return self.mesh


if __name__ == "__main__":
    pass
