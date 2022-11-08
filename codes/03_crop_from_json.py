# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 21:30:00 2022

@author: liang
"""

import open3d as o3d


pcd = o3d.io.read_point_cloud("test_ausgerichtet.ply")

#Zeige originale Geometrie in Rot
pcd.paint_uniform_color([1, 0, 0])
o3d.visualization.draw_geometries([pcd])

#Lade json Datei
vol = o3d.visualization.read_selection_polygon_volume("auswahl.json")

#Zeige ausgewählte Geometrie in Grün
auswahl = vol.crop_point_cloud(pcd) 
auswahl.paint_uniform_color([0, 1, 0]) 
o3d.visualization.draw_geometries([auswahl])
