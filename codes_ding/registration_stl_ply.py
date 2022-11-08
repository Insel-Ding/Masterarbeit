import open3d as o3d
import numpy as np
import os


#Lade .stl Datei als mesh und .ply Datei als Punktwolke
model_path = r'C:\Users\dings\Documents\Masterarbeit\models'
stl_path= os.path.join(model_path,'obergesenk.stl')
ply_path= os.path.join(model_path,'obergesenk_gescannt.ply') 


mesh = o3d.io.read_triangle_mesh(stl_path)
original = o3d.geometry.PointCloud(mesh.vertices)
verschleiss = o3d.io.read_point_cloud(ply_path)

#Geben die beiden Punktwolken zwei Farben
original.paint_uniform_color([1, 0.7, 0])    #original in gelb
verschleiss.paint_uniform_color([0, 0.6, 0.9])#verschleiss in blau

#Rauschen eliminieren für gescannte Datei bzw. verschleiss
processed_verschleiss, outlier_index = o3d.geometry.PointCloud.remove_radius_outlier(verschleiss,
                                              nb_points=100,
                                              radius=5)

#Registration beider Punktwolken mit ICP Algorithmen (point to point)
threshold = 50.0  
trans_init = np.asarray([[1,0,0,0],   
                         [0,1,0,0],   
                         [0,0,1,0],   
                         [0,0,0,1]])

reg_p2p = o3d.pipelines.registration.registration_icp(
        processed_verschleiss, original, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
'''
#Registration beider Punktwolken mit ICP Algorithmen (point to plane)
threshold = 50.0  
trans_init = np.asarray([[1,0,0,0],   
                         [0,1,0,0],   
                         [0,0,1,0],   
                         [0,0,0,1]])

reg_p2plane = o3d.pipelines.registration.registration_icp(
        processed_verschleiss, original, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())

'''
processed_verschleiss.transform(reg_p2p.transformation)

#Abstand berechenen zwischen beiden Punktwolken
print("Berechne den Abstand zwischen beiden Punktwolken...")
dists = processed_verschleiss.compute_point_cloud_distance(original)
dists = np.asarray(dists)
print(">Die Ergebnisse von den ersten 50 Punkten:")
print(dists[:50])

#Punkte mit größeren Abständen anzeigen und visualizieren
print("Punkte die mehr als 0,3mm Abstand haben:")
ind = np.where(dists > 1)[0]
pcd = processed_verschleiss.select_by_index(ind)
print(pcd)
o3d.visualization.draw_geometries([pcd])