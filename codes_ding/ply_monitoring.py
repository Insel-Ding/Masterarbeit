
import open3d as o3d
import os

model_path = r'C:\Users\dings\Documents\Masterarbeit\models'
stl_path= os.path.join(model_path,'obergesenk.stl')
ply_path= os.path.join(model_path,'test2.ply') 
test_path= r'C:\Users\dings\Documents\Masterarbeit\two_reg.ply'

mesh = o3d.io.read_triangle_mesh(stl_path)
pc_mesh = o3d.geometry.PointCloud(mesh.vertices)
ply_pc = o3d.io.read_point_cloud(ply_path)
test = o3d.io.read_point_cloud(test_path)

o3d.visualization.draw_geometries([test])