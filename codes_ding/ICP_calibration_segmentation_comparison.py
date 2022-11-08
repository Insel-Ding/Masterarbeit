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
    down_sample_source = source.uniform_down_sample(every_k_points=3)
    registed_source,target = uti.icp_algo(source=source, target=target, threshold=threshold, trans_init=trans_init,max_iteration=num_iteration)
    
    #----calibration----:
    registed_source,target = uti.calibration_after_rough_reg(source=registed_source,target=target,threshold=threshold_calib,num_samples=5)
    o3d.io.write_point_cloud(os.path.join(model_path,"reg_source.ply"),registed_source)
    o3d.io.write_point_cloud(os.path.join(model_path,"reg_target.ply"),target)
    o3d.io.write_point_cloud(os.path.join(model_path,"registed_both.ply"),registed_source+target)
    #uti.compute_dists(source=registed_source,target=target,tolerance=0.4)
    print("3 point clouds saved in path: %s"%(model_path))
    
    print("segmentieren...")
    o3d.visualization.draw_geometries_with_editing([registed_source])
    print("cropped file saved in path:%s"%(model_path))
    
    #Set cropped reference json coodinates
    reference_json_cood = os.path.join(model_path,"cropped_1.json")
    uti.segments_compare(registed_source,target,reference_json_cood,tolerance=0.4)