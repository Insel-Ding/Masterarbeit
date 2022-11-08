# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 14:03:23 2022

@author: liang
"""

import open3d as o3d
import numpy as np


#Lade .stl Datei als mesh und .ply Datei als Punktwolke

mesh = o3d.io.read_triangle_mesh("obergesenk.stl")
original = mesh.sample_points_uniformly(100000)
verschleiss = o3d.io.read_point_cloud("test.ply")

#Geben die beiden Punktwolken zwei Farben
original.paint_uniform_color([1, 0.706, 0])    #original in gelb
verschleiss.paint_uniform_color([0, 0.651, 0.929])#verschleiss in blau

#Rauschen eliminieren für gescannte Datei bzw. verschleiss
processed_verschleiss, outlier_index = o3d.geometry.PointCloud.remove_radius_outlier(verschleiss,
                                              nb_points=100,
                                              radius=5)

#Registration beider Punktwolken mit ICP Algorithmen
threshold = 40.0  
trans_init = np.asarray([[1,0,0,0],   
                         [0,1,0,0],   
                         [0,0,1,0],   
                         [0,0,0,1]])

reg_p2p = o3d.pipelines.registration.registration_icp(
        processed_verschleiss, original, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())



processed_verschleiss.transform(reg_p2p.transformation)

o3d.visualization.draw_geometries([original, processed_verschleiss])

