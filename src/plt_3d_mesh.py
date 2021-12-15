from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot

# Create a new plot
figure = pyplot.figure()
axes = mplot3d.Axes3D(figure)

# Load the STL files and add the vectors to the plot
env_mesh = mesh.Mesh.from_file(
    'ros_ws/src/drone_path_planning/resources/stl/env-scene.stl')

axes.add_collection3d(mplot3d.art3d.Poly3DCollection(env_mesh.vectors))

# Auto scale to the mesh size
scale = env_mesh.points.flatten()
axes.auto_scale_xyz(scale, scale, scale)

# Show the plot to the screen
pyplot.show()
