import open3d as o3d
import os
import ICP_utilities as uti

model_path = r'C:\Users\dings\Documents\Masterarbeit\models'
#pc = os.path.join(model_path,'ogabgabe_0_25_Mesh.ply')
#pc = os.path.join(model_path,'registed_both.ply')
pc = os.path.join(model_path,'registed_both_005_025.ply')
json_path = os.path.join(model_path,'nr1.json')

pointscloud = o3d.io.read_point_cloud(pc)
o3d.visualization.draw_geometries_with_editing([pointscloud])

#cropped =uti.crop_from_json(pointscloud, json_path)
#o3d.visualization.draw_geometries_with_editing([cropped])

