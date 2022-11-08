import open3d as o3d
import numpy as np
import os
import sys
import copy
import ICP_utilities as uti
import math
import json

#parameters
model_path = r'C:\Users\dings\Documents\Masterarbeit\models'
stl_path= os.path.join(model_path,'obergesenk.stl')
#compare_path = os.path.join(model_path,'registed_both.ply')
ply_path= os.path.join(model_path,'obergesenk_gescannt.ply') 
source = o3d.io.read_point_cloud(ply_path)
mesh = o3d.io.read_triangle_mesh(stl_path)
target = o3d.geometry.PointCloud(mesh.vertices)
trans_init=[[1,0,0,0],
            [0,1,0,0],    
            [0,0,1,0],            
            [0,0,0,1],]

num_iteration = 50
threshold = 30
threshold_calib = 0.01


example = os.path.join(model_path,'reg_source.ply') 
example1 = o3d.io.read_point_cloud(example)
## y-> k -> draw -> c
#Rechtecke(außer)
o3d.visualization.draw_geometries_with_editing([example1])
#Rechtecke(inner)
