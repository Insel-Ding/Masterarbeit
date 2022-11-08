# -*- coding: utf-8 -*-
"""
Created on Wed Jan 19 21:51:19 2022

@author: liang
"""

import open3d as o3d
import os
model_path = r'C:\Users\dings\Documents\Masterarbeit\models'

mesh = o3d.io.read_triangle_mesh(os.path.join(model_path,"ogabgabe_0_05_Mesh.stl"))

pcl = o3d.geometry.PointCloud(mesh.vertices)

distance_threshold = 0.2 # Abstand zwischen Inlier und Outlier   
ransac_n = 3            #Random Punkte am Anfang  
num_iterations = 1000   #Anzahl Iteration

plane_model, inliers = pcl.segment_plane(distance_threshold, ransac_n, num_iterations)

#Funktion der Trennebene
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = pcl.select_by_index(inliers)
inlier_cloud.paint_uniform_color([0, 0, 1])
print(inlier_cloud)
#o3d.io.write_point_cloud("ransac_02.ply",inlier_cloud)

outlier_cloud = pcl.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([1, 0, 0])
print(outlier_cloud)
o3d.io.write_point_cloud("boden.ply",inlier_cloud)

o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
