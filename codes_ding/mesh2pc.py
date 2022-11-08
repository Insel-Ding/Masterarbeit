import open3d as o3d
import os

model_path = r'C:\Users\dings\Documents\Masterarbeit\models'

mesh_path = os.path.join(model_path,'ogabgabe_0_25_Mesh.stl')
save_path = os.path.join(model_path,'ogabgabe_0_25_Mesh.ply')
mesh = o3d.io.read_triangle_mesh(mesh_path)
pc = o3d.geometry.PointCloud(mesh.vertices)
o3d.visualization.draw_geometries([pc])
o3d.io.write_point_cloud(save_path, pc)