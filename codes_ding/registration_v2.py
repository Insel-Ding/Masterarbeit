import open3d as o3d
import numpy as np
import os

#path
model_path = r'C:\Users\dings\Documents\Masterarbeit\models'

ply_referez_path1 = os.path.join(model_path,'obergesenk_rand_draw.ply') 
ply_referez_path2 = os.path.join(model_path,'obergesenk_rand_scann.ply') 
stl_model_path = os.path.join(model_path,'obergesenk.stl')
ply_model_path = os.path.join(model_path,'obergesenk_gescannt.ply')

ply_pc = o3d.io.read_point_cloud(ply_model_path)    
mesh = o3d.io.read_triangle_mesh(stl_model_path)
stl_pc = o3d.geometry.PointCloud(mesh.vertices)

if __name__ == __main__:

    
    referenz1 = o3d.io.read_point_cloud(ply_referez_path1)

    referenz2 = o3d.io.read_point_cloud(ply_referez_path2)

    vis = o3d.visualization.Visualizer()

    vis.create_window(window_name='Visualizer', width = 1600, height = 1200)
    referenz1.paint_uniform_color([1,0,0])#rot
    referenz2.paint_uniform_color([0,1,0])#grün

    icp_iteration = 40
    threshold = 30

    vis.add_geometry(referenz1)
    vis.add_geometry(referenz2)

    