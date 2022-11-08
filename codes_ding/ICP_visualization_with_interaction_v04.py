import open3d as o3d
import numpy as np
import os
import copy
import ICP_utilities as uti

#parameters
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
threshold_calib = 0.01

if __name__ == "__main__":
    #low_noise_source = uti.rauchen_eliminieren(source,nb_points=25,radius=3)
    down_sample_source = source.uniform_down_sample(every_k_points=2)
    #----step by step----:
    #registed_source,target = uti.icp_algo_step_by_step(source=down_sample_source, target=target, threshold=threshold, trans_init=trans_init, num_iteration=num_iteration)
    #----quick show----:
    registed_source,target = uti.icp_algo(source=down_sample_source, target=target, threshold=threshold, trans_init=trans_init)
    
    #----calibration----:
    uti.calibration_after_rough_reg(source=registed_source,target=target,threshold=threshold_calib,num_samples=5)
    #o3d.visualization.draw_geometries([registed_source,target])
    o3d.io.write_point_cloud(os.path.join(model_path,"reg_source.ply"),registed_source)
    o3d.io.write_point_cloud(os.path.join(model_path,"reg_target.ply"),target)
    o3d.io.write_point_cloud(os.path.join(model_path,"registed_both.ply"),registed_source+target)
    #uti.compute_dists(source=registed_source,target=target,abstandgrenz=0.4)
    
    #----segmentation----:
    print("segmentieren...")
    o3d.visualization.draw_geometries_with_editing([registed_source])
    uti.crop_from_json(point_cloud_path=os.path.join(model_path,"reg_source.ply"), json_path=os.path.join(model_path,"croped_1.json"))