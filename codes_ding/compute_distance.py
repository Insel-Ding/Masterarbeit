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



if __name__ == "__main__":
    #initial
    #low_noise_source = uti.rauchen_eliminieren(source, nb_points=5, radius=3)
    dists_limit= 30
    threshold = 50
    trans_init=[[1,0,0,0],
                [0,1,0,0],    
                [0,0,1,0],
                [0,0,0,1],]
    transformed_source, target = uti.icp_algo(source=source, target=target, threshold=threshold, trans_init=trans_init,max_iteration=50)
    uti.compute_dists(source=transformed_source,target=target,tolerance=dists_limit)