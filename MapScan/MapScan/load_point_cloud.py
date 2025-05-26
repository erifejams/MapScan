##to make 3D reconsturction more visible

##code from here : https://orbi.uliege.be/bitstream/2268/254933/1/TDS_generate_3D_meshes_with_python.pdf

import matplotlib
from matplotlib import pyplot as plt
#matplotlib.use('Agg') ##opens the image windows, but produces errors
from PIL import Image as PILImage
import torch
from transformers import GLPNImageProcessor, GLPNForDepthEstimation
import numpy as np
import open3d as o3d
import cv2




pcd_loaded = o3d.io.read_point_cloud("scanRoom/flower_output_point_cloud.ply")
point_cloud = np.asarray(pcd_loaded.points)

# had to change it over here - check the difference, something to do with how the cloud was saved
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
colors_data = np.asarray(pcd.colors)
pcd_normal = np.asarray(pcd.normals)


# had to change it over here - check the difference, something to do with how the cloud was saved
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])

# Estimate normals for the point cloud
# Search radius for KNN
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

# Optionally, orient the normals consistently
pcd.orient_normals_consistent_tangent_plane(k=30)


colors_data = np.asarray(pcd.colors) # This will be empty unless you add colors to the point cloud
pcd_normal = np.asarray(pcd.normals) # Now this will contain the estimated normals


o3d.visualization.draw_geometries([pcd])


distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist

#To get results with Poisson
bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector([radius, radius * 2]))


dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)

dec_mesh.remove_degenerate_triangles()
dec_mesh.remove_duplicated_triangles()
dec_mesh.remove_duplicated_vertices()
dec_mesh.remove_non_manifold_edges()

# Visualize the resulting mesh
o3d.visualization.draw_geometries([dec_mesh])