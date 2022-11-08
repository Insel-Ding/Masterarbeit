import open3d as o3d
import numpy as np
import os

#path
model_path = r'C:\Users\dings\Documents\Masterarbeit\models'
stl_path= os.path.join(model_path,'obergesenk.stl')
ply_path= os.path.join(model_path,'obergesenk_gescannt.ply') 
ply_referez_path = os.path.join(model_path,'obergesenk_rand_scann.ply') 


if __name__ == "__main__":

    # 3d-mesh datei aufladen
    mesh = o3d.io.read_triangle_mesh(stl_path)
   
    target = o3d.geometry.PointCloud(mesh.vertices)
    # read point cloud (gescannt)
    source = o3d.io.read_point_cloud(ply_path) #return open3d.geometry.PointCloud 
    referenz = o3d.io.read_point_cloud(ply_referez_path)

   
    transform_matrix = [[1,0,0,0],
                        [0,1,0,0],    
                        [0,0,1,0],
                        [0,0,0,1],]
    source.transform(transform_matrix) # http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.transform
                                       # return open3d.geometry.Geometry3D
                            
    referenz.transform(transform_matrix)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Visualizer', width = 1600, height = 1200)
    source.paint_uniform_color([1,0,0])#rot
    referenz.paint_uniform_color([0,1,1])#blau
    #target.paint_uniform_color([0,1,1])#Blau


    vis.add_geometry(source)
    vis.add_geometry(referenz)
    #vis.add_geometry(target)
    
    #Registion Visualizieren 
    
    icp_iteration = 40
    threshold = 30
    #p2p----------------------------------------
    

    for i in range(icp_iteration):
        reg_p2p = o3d.pipelines.registration.registration_icp(
            referenz,source,threshold,transform_matrix,o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1) 
        )

        

        #return open3d.pipelines.registration.RegistrationResult
        # reg_p2p(RegistrationResult) --> property)
        source.transform(reg_p2p.transformation) #http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.RegistrationResult.html?highlight=registrationresult#open3d.pipelines.registration.RegistrationResult.transformation
        

        vis.update_geometry(source)
        
        vis.poll_events()
        vis.update_renderer()
    vis.run()
    vis.destroy_window()
    #--------------------------------------------
