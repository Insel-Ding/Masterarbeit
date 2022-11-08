# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 21:19:30 2022

@author: liang
"""

import open3d as o3d


# Boden trennen von Standard mesh Datei
mesh = o3d.io.read_triangle_mesh("Obergesenk.stl")
pcl = o3d.geometry.PointCloud(mesh.vertices)

distance_threshold = 0.2 # Abstand zwischen Inlier und Outlier   
ransac_n = 3            #Random Punkte am Anfang  
num_iterations = 1000   #Anzahl Iteration

plane_model, inliers = pcl.segment_plane(distance_threshold, ransac_n, num_iterations)


outlier_cloud = pcl.select_by_index(inliers,invert=True)
o3d.io.write_point_cloud("obergesenk_ohne_boden.ply",outlier_cloud)


# Hier manuell abschneiden
o3d.visualization.draw_geometries_with_editing([outlier_cloud])
