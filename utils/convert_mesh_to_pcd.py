import open3d as o3d
import numpy as np

# Load mesh
mesh = o3d.io.read_triangle_mesh("tree.ply")

# Sample points from the mesh surface
pcd = mesh.sample_points_uniformly(number_of_points=1000)  # Adjust number of points

#Move each point in y direction by -.9 m
pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) + [0, -0.9, 0])
# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])

# Save the point cloud
o3d.io.write_point_cloud("tree.pcd", pcd)

