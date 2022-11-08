

import open3d as o3d
import os

model_path = r'C:\Users\dings\Documents\Masterarbeit\models'
stl_path= os.path.join(model_path,'obergesenk.stl')
ply_path= os.path.join(model_path,'obergesenk_gescannt.ply') 

mesh = o3d.io.read_triangle_mesh(stl_path)
pc_mesh = o3d.geometry.PointCloud(mesh.vertices)
ply_pc = o3d.io.read_point_cloud(ply_path)

#http://www.open3d.org/docs/0.9.0/tutorial/Advanced/interactive_visualization.html#interactive-visualization
# Hier manuell abschneiden
# K Ansicht fixieren und ROI manuell auswählen mit gedrückte Strl-Taste
# C gewähltes Bereich speichern

#o3d.visualization.draw_geometries_with_editing([pc_mesh])
o3d.visualization.draw_geometries_with_editing([ply_pc])

