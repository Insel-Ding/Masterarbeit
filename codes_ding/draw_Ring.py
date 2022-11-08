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


example = ply_path= os.path.join(model_path,'reg_source.ply') 
example1 = o3d.io.read_point_cloud(example)
#center = example1.get_center()
picked = uti.pick_points(example1)
#print(center)
print(picked)

xyz_load = np.asarray(example1.points)
#xyz=xyz_load[picked[0],picked[1]]
print(xyz_load[picked[0]])
print(xyz_load[picked[1]])
print(xyz_load[picked[2]])

x0 = round(xyz_load[picked[0]][0],2)
y0 = round(xyz_load[picked[0]][1],2)

x_edge = round(xyz_load[picked[1]][0],2)
y_edge = round(xyz_load[picked[1]][1],2)

x_outer = round(xyz_load[picked[2]][0],2)
y_outer = round(xyz_load[picked[2]][1],2)

r = math.sqrt((x0-x_edge)**2 + (y0-y_edge)**2)
r_outer = math.sqrt((x0-x_outer)**2 + (y0-y_outer)**2)
print(r)
print(r_outer)
r = round(r,2)
r_outer = round(r_outer,2)
coord=[]
coord_outer=[]
#####################
for i in range(0,365,2):
    
    coord.append(round(r*math.cos(math.radians(i))+x0,2))
    coord.append(round(r*math.sin(math.radians(i))+y0,2))

print(coord)    

for j in range(2,len(coord)+183,3):
    coord.insert(j,0)

    j+=1
print(coord)
#####################
for i in range(0,365,2):
    
    coord_outer.append(round(r_outer*math.cos(math.radians(i))+x0,2))
    coord_outer.append(round(r_outer*math.sin(math.radians(i))+y0,2))

print(coord)    

for j in range(2,len(coord_outer)+183,3):
    coord_outer.insert(j,0)

    j+=1
print(coord_outer)
#####################

arr = np.array(coord)
print(arr)
arr_outer =np.array(coord_outer)
print(arr_outer)

new = np.reshape(arr, (-1,3))
anew = np.float64(new)
print(anew)

new_outer = np.reshape(arr_outer, (-1,3))
anew_outer = np.float64(new_outer)
print(anew_outer)


#gg=o3d.geometry.crop_point_cloud(example1,anew)
#o3d.visualization.draw_geometries(gg)

#json:
json_path = os.path.join(model_path,"cropped_1.json")
json_path_out = os.path.join(model_path,"ccc.json")
json_path_out_outer = os.path.join(model_path,"ccc_outer.json")

dict1={}
dict1_outer={}

lis= anew.tolist()
lis_outer = anew_outer.tolist()

dict1 = uti.get_json_data(json_path, circle_coord=lis)
dict1_outer = uti.get_json_data(json_path, circle_coord=lis_outer)

uti.write_json_dict(json_path_out,dict1)
uti.write_json_dict(json_path_out_outer,dict1_outer)

cropped = uti.crop_from_json(example1, json_path_out)
cropped_outer = uti.crop_from_json(example1, json_path_out_outer)

dis = cropped_outer.compute_point_cloud_distance(cropped)
dis = np.asarray(dis)
ind = np.where(dis>0.01)[0]
ring = cropped_outer.select_by_index(ind)

o3d.visualization.draw_geometries([ring])
