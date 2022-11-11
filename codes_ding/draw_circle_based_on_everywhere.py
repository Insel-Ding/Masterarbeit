import open3d as o3d
import numpy as np
import os
import sys
import copy
import ICP_utilities as uti
import math
import json

#parameters
model_path = r'C:\Users\dings\Documents\Masterarbeit\models'
stl_path= os.path.join(model_path,'obergesenk.stl')
#compare_path = os.path.join(model_path,'registed_both.ply')
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
########################################################################
#registed_both= 'registed_both_nr1'
example = os.path.join(model_path,'registed_both.ply') 
example = os.path.join(model_path,'registed_both_005_025.ply') 
example1 = o3d.io.read_point_cloud(example)

picked = uti.pick_points(example1)

xyz_load = np.asarray(example1.points)


x0 = round(xyz_load[picked[0]][0],2)
y0 = round(xyz_load[picked[0]][1],2)

x_edge = round(xyz_load[picked[1]][0],2)
y_edge = round(xyz_load[picked[1]][1],2)

r = math.sqrt((x0-x_edge)**2 + (y0-y_edge)**2)

r = round(r,2)

coord=[]
for i in range(0,365,2):
    
    coord.append(round(r*math.cos(math.radians(i))+x0,2))
    coord.append(round(r*math.sin(math.radians(i))+y0,2))
print(coord)

for j in range(2,len(coord)+183,3):
    coord.insert(j,0)

    j+=1

arr = np.array(coord)
new = np.reshape(arr, (-1,3))
anew = np.float64(new)

#json:
nr = 'nr4'
js = nr+'.json'
ply = nr+'.ply'
pc = os.path.join(model_path,ply)
json_path = os.path.join(model_path,"cropped_1.json")
json_path_out = os.path.join(model_path,js)

dict1={}
lis= anew.tolist()
dict1 = uti.get_json_data(json_path, circle_coord=lis)
uti.write_json_dict(json_path_out,dict1)

cropped = uti.crop_from_json(example1, json_path_out)

o3d.visualization.draw_geometries([cropped])
o3d.io.write_point_cloud(pc, cropped)