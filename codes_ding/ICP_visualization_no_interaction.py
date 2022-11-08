import open3d as o3d
import numpy as np
import os
import ICP_utilities as uti


#path
model_path = r'C:\Users\dings\Documents\Masterarbeit\models'
stl_path= os.path.join(model_path,'obergesenk.stl')
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

if __name__ == "__main__":
    low_noise_source = uti.rauchen_eliminieren(source,nb_points=5,radius=3)
    down_sample_source = source.uniform_down_sample(every_k_points=3)
    uti.icp_algo_step_by_step(source=down_sample_source, target=target, threshold=threshold, trans_init=trans_init, num_iteration=num_iteration)