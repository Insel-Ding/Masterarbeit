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

if __name__ == "__main__":

    
    # read point cloud (gescannt)
    
    referenz1 = o3d.io.read_point_cloud(ply_referez_path1)

    referenz2 = o3d.io.read_point_cloud(ply_referez_path2)

    

    # ICP Registration Algo http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.registration_icp.html#open3d.pipelines.registration.registration_icp
    # Einheitsmatrix
    transform_matrix = [[1,0,0,0],
                        [0,1,0,0],    
                        [0,0,1,0],
                        [0,0,0,1],]
    #referenz1.transform(transform_matrix) # http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.transform
                                       # return open3d.geometry.Geometry3D
                            
    #referenz2.transform(transform_matrix)
    #ply_pc.transform(transform_matrix)
    #stl_pc.transform(transform_matrix)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Visualizer', width = 1600, height = 1200)
    referenz1.paint_uniform_color([1,0,0])#rot
    referenz2.paint_uniform_color([0,1,0])#grün
    ply_pc.paint_uniform_color([0,0,1])#blau
    stl_pc.paint_uniform_color([1,1,0])#gelb
    
  
    
    icp_iteration = 40
    threshold = 30


    # 1. reference p2p----------------------------------------
    
    vis.add_geometry(referenz1)
    vis.add_geometry(referenz2)

    for i in range(icp_iteration):
        reg_p2p = o3d.pipelines.registration.registration_icp(
            referenz1,referenz2,threshold,transform_matrix,o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1) 
        )

        
        referenz1.transform(reg_p2p.transformation) #http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.RegistrationResult.html?highlight=registrationresult#open3d.pipelines.registration.RegistrationResult.transformation
        

        vis.update_geometry(referenz1)
        
        vis.poll_events()
        vis.update_renderer()
    vis.run()
    vis.destroy_window()
    #o3d.io.write_point_cloud(os.path.join(model_path,'two_reg.ply'),referenz1+referenz2)
    #two_ref_path = os.path.join(model_path,'two_reg.ply') 
    #two_ref_path_pc = o3d.io.read_point_cloud(two_ref_path)


    
    
    # 2. stl - reference p2p -----------------------------------
    
    '''
    vis.add_geometry(stl_pc)
    for i in range(icp_iteration):
        reg_stl_p2p = o3d.pipelines.registration.registration_icp(
            two_ref_path_pc,stl_pc,threshold,transform_matrix,o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1) 
        )

        
        two_ref_path_pc.transform(reg_stl_p2p.transformation) #http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.RegistrationResult.html?highlight=registrationresult#open3d.pipelines.registration.RegistrationResult.transformation
        
        vis.update_geometry(two_ref_path_pc)
        
        vis.poll_events()
        vis.update_renderer()
    vis.run()
    vis.destroy_window()
   
    # 3. ply - reference p2p -----------------------------------
        
    for i in range(icp_iteration):
        reg_ply_p2p = o3d.pipelines.registration.registration_icp(
            ply_pc,referenz2,threshold,transform_matrix,o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1) 
        )

        

        #return open3d.pipelines.registration.RegistrationResult
        # reg_p2p(RegistrationResult) --> property)
        ply_pc.transform(reg_ply_p2p.transformation) #http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.RegistrationResult.html?highlight=registrationresult#open3d.pipelines.registration.RegistrationResult.transformation
        

        vis.update_geometry(ply_pc)
        
        vis.poll_events()
        vis.update_renderer()
    '''  
    
    #--------------------------------------------
