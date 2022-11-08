# -*- coding: utf-8 -*-
"""
Created on Thu Jan 20 21:41:38 2022

@author: liang
"""

import open3d as o3d
import numpy as np


#Lade .stl Datei als mesh und .ply Datei als Punktwolke
mesh = o3d.io.read_triangle_mesh("obergesenk.stl")
original = o3d.geometry.PointCloud(mesh.vertices)
verschleiss = o3d.io.read_point_cloud("test.ply")
boden = o3d.io.read_point_cloud("boden.ply")

#Geben die drei beiden Punktwolken Farben
original.paint_uniform_color([1, 0, 0])    
verschleiss.paint_uniform_color([0, 1, 0])
boden.paint_uniform_color([0, 0, 1]) #Vergleichsebene

#Rauschen eliminieren für gescannte Datei bzw. verschleiss
processed_verschleiss, outlier_index = o3d.geometry.PointCloud.remove_radius_outlier(verschleiss,
                                              nb_points=100,
                                              radius=5)

#Registration der Punktwolken mit ICP Algorithmen
threshold = 50.0  
trans_init = np.asarray([[1,0,0,0],   
                         [0,1,0,0],   
                         [0,0,1,0],   
                         [0,0,0,1]])

reg_p2p = o3d.pipelines.registration.registration_icp(
        processed_verschleiss, original, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())


processed_verschleiss.transform(reg_p2p.transformation)

#Abstand berechenen zwischen gescannte Punktwolken und Vergleichsebene "Boden"
print("Berechne den Abstand zwischen Verschleiss und Boden")
dists = processed_verschleiss.compute_point_cloud_distance(boden)
dists = np.asarray(dists)


#Punkte mit bestimmten Abstand (z.B. 36,5mm) anzeigen und visualizieren

unter = np.where(dists < 37.2)[0] # +0,7mm wegen Rauschen
unterteil = processed_verschleiss.select_by_index(unter)

dists_neu = unterteil.compute_point_cloud_distance(boden)
dists_neu = np.asarray(dists_neu)

oben = np.where(dists_neu > 35.8)[0] # -0,7mm wegen Rauschen

auswahl = unterteil.select_by_index(oben)

o3d.io.write_point_cloud("region_1.ply", auswahl)
o3d.visualization.draw_geometries([auswahl])
