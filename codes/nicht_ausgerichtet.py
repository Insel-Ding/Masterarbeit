# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 13:46:01 2022

@author: liang
"""

import open3d as o3d


mesh = o3d.io.read_triangle_mesh("obergesenk.stl")
pc_from_mesh = mesh.sample_points_uniformly(100000)
pc = o3d.io.read_point_cloud("test.ply")

pc.paint_uniform_color([1, 0.706, 0])
pc_from_mesh.paint_uniform_color([0, 0.651, 0.929])

o3d.visualization.draw_geometries([pc_from_mesh, pc])

