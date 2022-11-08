# -*- coding: utf-8 -*-
"""
Created on Mon Jan 24 09:04:54 2022

@author: liang
"""

import open3d as o3d
import numpy as np


if __name__ == "__main__":
    
    # PointCloud aufladen
    mesh = o3d.io.read_triangle_mesh("obergesenk.stl")
    
    source = o3d.io.read_point_cloud("test.ply")
    target = o3d.geometry.PointCloud(mesh.vertices)


    #ICP Registration
    flip_transform = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    source.transform(flip_transform)
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Visualizer', width=1800, height=1200)
    source.paint_uniform_color([1, 0, 0])  # in rot
    target.paint_uniform_color([0, 1, 0])  # in grün
    vis.add_geometry(source)  
    vis.add_geometry(target)  
    threshold = 40          
    icp_iteration = 50       
    save_image = False        
    
    #Registration Visualizieren
    for i in range(icp_iteration):
        reg_p2l = o3d.pipelines.registration.registration_icp(
            source, target, threshold, flip_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))
        source.transform(reg_p2l.transformation)
        vis.update_geometry(source)
        
        vis.poll_events()            
        vis.update_renderer()        
        if save_image:
            vis.capture_screen_image("temp_%04d.jpg" % i)
    vis.run()
    vis.destroy_window()             

