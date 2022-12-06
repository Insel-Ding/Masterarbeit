import open3d as o3d
import numpy as np
import os
import copy
import json
import sys

def compute_dists(source,target,tolerance):
    dists = source.compute_point_cloud_distance(target)
    dists = np.asarray(dists)

    ind = np.where(dists>tolerance)[0]
    difference = source.select_by_index(ind)
    
    o3d.visualization.draw_geometries([difference])
#rauchen eliminieren für gescannte PLY

def rauchen_eliminieren(source,nb_points,radius):
    low_noise_source, outlier = o3d.geometry.PointCloud.remove_radius_outlier(source,
                                              nb_points,
                                              radius)
    return low_noise_source

def icp_algo(source,target,threshold,trans_init,max_iteration):

    reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration))                

    transformed_source=source.transform(reg_p2p.transformation)
    o3d.visualization.draw_geometries([transformed_source, target])
    return transformed_source,target



def icp_algo_step_by_step(source,target,threshold,trans_init,num_iteration):
    vis = o3d.visualization.Visualizer()
    source.paint_uniform_color([1, 0.706, 0])  
    target.paint_uniform_color([0, 0.651, 0.929])  
    vis.create_window()
    vis.add_geometry(source)
    vis.add_geometry(target)

    for i in range(num_iteration):
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source,target,threshold,trans_init,o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))

        source.transform(reg_p2p.transformation) 

        vis.update_geometry(source)
        vis.poll_events()
        vis.update_renderer()
    vis.run()
    vis.destroy_window()
    return source,target

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
    


def pick_points(pcd):
    
    print(
        "1) Please pick points using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    
    return vis.get_picked_points()




def manual_registration(source,target,threshold):
    print("manual ICP")
    print("Visualization of two point clouds before manual alignment")
    draw_registration_result(source, target, np.identity(4))

    # pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    print("picked_id_source = %s"%(picked_id_source))
    picked_id_target = pick_points(target)
    print("picked_id_source = %s"%(picked_id_target))
    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    print("corr = %s"%(corr))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target
    print("corr1 = %s"%(corr))


    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    draw_registration_result(source, target, reg_p2p.transformation)
    print("done")

def calibration_after_rough_reg(source,target,threshold,num_samples):
    #print("Finished -- finetune registration after the rough registration")
    #print("Finished -- Visualization of two rough registed point clouds before finetune")
    #print("press 'q' to continue\n")
    draw_registration_result(source, target, np.identity(4))
    corr = np.zeros((num_samples, 2)) 

    for j in range(num_samples):
        print("chose No. %d coordinate of %d total in source and press 'q' to continue\n"%(j+1,num_samples) )
        picked_id_source = pick_points(source)
        print("chose No. %d coordinate of %d total in target and press 'q' to continue\n"%(j+1,num_samples))
        picked_id_target = pick_points(target)
        corr[j:j+1,0] = picked_id_source
        corr[j:j+1,1] = picked_id_target
    

    print("corr1 = %s", corr)    

    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))

    
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    draw_registration_result(source, target, reg_p2p.transformation)
    return source ,target
    print("done")

def crop_from_json(point_cloud,json_path):
    
    vol  = o3d.visualization.read_selection_polygon_volume(json_path)
    cropped = vol.crop_point_cloud(point_cloud)
    cropped.paint_uniform_color([1, 0.651, 1]) 
    return cropped

def segments_compare(registed_source,target,reference_json_cood,tolerance):
    cropped_source = crop_from_json(registed_source,json_path=reference_json_cood)
    cropped_target = crop_from_json(target,json_path=reference_json_cood)
    o3d.visualization.draw_geometries([cropped_source+cropped_target])
    compute_dists(cropped_source, cropped_target, tolerance)

def get_json_data(json_path,circle_coord):
    dict1={}
    with open(json_path,'rb')as f:
        param = json.load(f)
        param["bounding_polygon"] = circle_coord
        dict1=param
    f.close()
    return dict1

def write_json_dict(json_path,dict):
    with open(json_path,'w') as r:
    
        json.dump(dict,r)
        
    r.close()

def draw_circle(pc_path,json_path,json_path_out):
    pc = o3d.io.read_point_cloud(pc_path)
    #center = example1.get_center()
    picked = uti.pick_points(pc)
    #print(center)
    print(picked)

    xyz_load = np.asarray(pc.points)
    #xyz=xyz_load[picked[0],picked[1]]
    print(xyz_load[picked[0]])
    print(xyz_load[picked[1]])

    x0 = round(xyz_load[picked[0]][0],2)
    y0 = round(xyz_load[picked[0]][1],2)

    x_edge = round(xyz_load[picked[1]][0],2)
    y_edge = round(xyz_load[picked[1]][1],2)

    r = math.sqrt((x0-x_edge)**2 + (y0-y_edge)**2)
    print(r)
    r = round(r,2)
    coord=[]
    for i in range(0,365,2):
        
        coord.append(round(r*math.cos(math.radians(i)),2))
        coord.append(round(r*math.sin(math.radians(i)),2))
    print(coord)    

    for j in range(2,len(coord)+183,3):
        coord.insert(j,0)

        j+=1
    print(coord)

    arr = np.array(coord)
    print(arr)

    new = np.reshape(arr, (-1,3))
    print(new)
    anew = np.float64(new)
    print(anew)
    #gg=o3d.geometry.crop_point_cloud(example1,anew)
    #o3d.visualization.draw_geometries(gg)

    #json:
    dict1={}
    lis= anew.tolist()
    dict1 = get_json_data(json_path, circle_coord=lis)
    write_json_dict(json_path_out,dict1)
    cropped = crop_from_json(pc, json_path_out)
    o3d.visualization.draw_geometries([cropped])
        