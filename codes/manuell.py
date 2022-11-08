# -*- coding: utf-8 -*-
"""
Created on Fri Jan 14 11:04:04 2022

@author: liang
"""

import open3d as o3d
ply_path = r'C:\Users\dings\Desktop\Masterarbeit\models\test1.ply'

pc = o3d.io.read_point_cloud(ply_path)

#Strl + K Ansicht fixieren und ROI manuell auswählen mit gedrückte Strl-Taste
#Strl + C gewähltes Bereich speichern

o3d.visualization.draw_geometries_with_editing([pc])