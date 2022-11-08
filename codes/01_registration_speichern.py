# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 21:13:59 2022

@author: liang
"""

import open3d as o3d
import numpy as np


#Lade .stl Datei als mesh und .ply Datei als Punktwolke

mesh = o3d.io.read_triangle_mesh("obergesenk.stl")
original = o3d.geometry.PointCloud(mesh.vertices)
verschleiss = o3d.io.read_point_cloud("test.ply")


#Geben die beiden Punktwolken zwei Farben
original.paint_uniform_color([1, 0, 0])    
verschleiss.paint_uniform_color([0, 1, 0])


#Registration beider Punktwolken mit ICP Algorithmen
threshold = 40.0  
trans_init = np.asarray([[1,0,0,0],   
                         [0,1,0,0],   
                         [0,0,1,0],   
                         [0,0,0,1]])

reg_p2p = o3d.pipelines.registration.registration_icp(
        verschleiss, original, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())



verschleiss.transform(reg_p2p.transformation)

o3d.io.write_point_cloud("test_ausgerichtet.ply", verschleiss)
o3d.visualization.draw_geometries([original, verschleiss])