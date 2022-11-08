"""
Created on OCT 4 21:41:38 2022

@author: Ding
"""
import open3d as o3d
import tkinter as tk
import numpy as np
from tkinter import filedialog
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from sympy import *
import ICP_utilities as uti
import os
import copy
import json
import sys
import math


#Funktion Menu "Datei -- Mesh Datei visualisieren"
def open_mesh():
    tk.messagebox.showinfo(message="Wählen Sie eine Mesh Datei...")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.paint_uniform_color([1,0.5,0])
    print(mesh)
    o3d.visualization.draw_geometries([mesh],mesh_show_wireframe = True, 
                                      mesh_show_back_face = False)

#Funktion Menu "Datei -- Punktwolke Datei visualisieren"
def open_pc():
    tk.messagebox.showinfo(message="Wählen Sie eine Punktwolke Datei...")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    pcd = o3d.io.read_point_cloud(pcd_path)
    print(pcd)
    v = tk.messagebox.askyesno(message="Punktwolke einfarbig darstellen?")
    if v == True:
        pcd.paint_uniform_color([0,0,1])
    else:
        pass
    o3d.visualization.draw_geometries([pcd])

#Funktion Menu "Datei -- Mesh in Punktwolke konvertieren und speichern"
def mesh2pc():
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    pc = o3d.geometry.PointCloud(mesh.vertices)
    o3d.visualization.draw_geometries([pc])
    
    #Punktwolke speichern
    v = tk.messagebox.askyesno(message="Punktwolke speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, pc)
#Funktion Menu "Vorverarbeitung - Reduktion der Datenmenge - Mit Voxel-Grid-Downsamping
def voxel_down():
    #Lade Punktwolke Datei
    tk.messagebox.showinfo(message="Punktwolke-Datei öffnen")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    pcd = o3d.io.read_point_cloud(pcd_path)

    #Eingabe Parametern
    voxel_size = tk.simpledialog.askfloat("Voxel Grid Downsampling",
                               "Größe des Voxels bzw. Länge des Gitters:\n\
    Typischer Wert 0,01 bis 0,5")
   
    #Downsampling mit Voxel-Grid Verfahren
    down_pcd = pcd.voxel_down_sample(voxel_size)
    
    #Visualisierung
    down_pcd.paint_uniform_color([0, 0, 1]) # in blau
    o3d.visualization.draw_geometries([down_pcd])
    
    #Ergebnisse speichern
    v = tk.messagebox.askyesno(message="Ergebnis speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply"),("xyz Datei","*.xyz"),("pcd Datei","*.pcd")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, down_pcd)
#Funktion Menu "Vorverarbeitung - Rauschenunterdrückung - Mit Radius Outlier Removal"
def radius_out():
    #Lade Punktwolke Datei
    tk.messagebox.showinfo(message="Punktwolke-Datei öffnen")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    pcd = o3d.io.read_point_cloud(pcd_path)

    #Eingabe Parametern
    radius = tk.simpledialog.askfloat("Radius Outlier Removal",
                               "Größe des Radius:\n\
    bei genauen Aufnahmen ab 0,05 bis 0,2")
    nb_points = tk.simpledialog.askinteger("Radius Outlier Removal", 
                                            "Kleinste Anzahl der Nachbarpunkte in diesem Bereich:\n\
    bei genauen Aufnahmen ab 50 bis 200 mit dem Radius = 0,05")
    #Rauschenunterdrückung mit radius outlier removal
    pcd_processed, ind = pcd.remove_radius_outlier(nb_points, radius)
    noise = pcd.select_by_index(ind, invert = True)
    
    #Visualisierung
    pcd_processed.paint_uniform_color([0, 0, 1]) # in blau
    noise.paint_uniform_color([1, 0, 0]) # in rot
    o3d.visualization.draw_geometries([pcd_processed, noise])
    
    #Ergebnisse speichern
    v = tk.messagebox.askyesno(message="Punktwolke in blau als Ergebnis speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply"),("xyz Datei","*.xyz"),("pcd Datei","*.pcd")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, pcd_processed)

def registration_icp_po2po():
    tk.messagebox.showinfo(message="Wählen Sie den Model Pfad ")
    model_path = filedialog.askdirectory()
    tk.messagebox.showinfo(message="Wählen Sie eine stl-mesh Datei ")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    target = o3d.geometry.PointCloud(mesh.vertices)
    tk.messagebox.showinfo(message="Wählen Sie eine ply-punktwolke Datei ")
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    source = o3d.io.read_point_cloud(source_path)
    
    trans_init=[[1,0,0,0],
                [0,1,0,0],    
                [0,0,1,0],            
                [0,0,0,1],]

    num_iteration = 50
    threshold = 30
    threshold_calib = 0.01


    down_sample_source = source.uniform_down_sample(every_k_points=3)
    registed_source,target = uti.icp_algo(source=source, target=target, threshold=threshold, trans_init=trans_init,max_iteration=num_iteration)
    v = tk.messagebox.askyesno(message="(Ring) Punktwolke speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, registed_source)
def registration_icp_po2po_steps():
    tk.messagebox.showinfo(message="Wählen Sie den Model Pfad ")
    model_path = filedialog.askdirectory()
    tk.messagebox.showinfo(message="Wählen Sie eine stl-mesh Datei ")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    target = o3d.geometry.PointCloud(mesh.vertices)
    tk.messagebox.showinfo(message="Wählen Sie eine ply-punktwolke Datei ")
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    source = o3d.io.read_point_cloud(source_path)
    
    trans_init=[[1,0,0,0],
                [0,1,0,0],    
                [0,0,1,0],            
                [0,0,0,1],]

    num_iteration = 50
    threshold = 30
    threshold_calib = 0.01


    down_sample_source = source.uniform_down_sample(every_k_points=3)
    registed_source,target = uti.icp_algo_step_by_step(source=source, target=target, threshold=threshold, trans_init=trans_init,num_iteration=num_iteration)
    v = tk.messagebox.askyesno(message="(Ring) Punktwolke speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, registed_source)
#Funktion Menu "Registrierung -- Point-to-Point" 
def registration_icp_po2po_steps_calibration():
    tk.messagebox.showinfo(message="Wählen Sie den Model Pfad ")
    model_path = filedialog.askdirectory()
    tk.messagebox.showinfo(message="Wählen Sie eine stl-mesh Datei ")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    target = o3d.geometry.PointCloud(mesh.vertices)
    tk.messagebox.showinfo(message="Wählen Sie eine ply-punktwolke Datei ")
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    source = o3d.io.read_point_cloud(source_path)
    
    trans_init=[[1,0,0,0],
                [0,1,0,0],    
                [0,0,1,0],            
                [0,0,0,1],]

    num_iteration = 50
    threshold = 30
    threshold_calib = 0.01


    down_sample_source = source.uniform_down_sample(every_k_points=3)
    registed_source,target = uti.icp_algo(source=source, target=target, threshold=threshold, trans_init=trans_init,max_iteration=num_iteration)
    
    #----calibration----:
    registed_source,target = uti.calibration_after_rough_reg(source=registed_source,target=target,threshold=threshold_calib,num_samples=5)
    o3d.io.write_point_cloud(os.path.join(model_path,"reg_source.ply"),registed_source)
    o3d.io.write_point_cloud(os.path.join(model_path,"reg_target.ply"),target)
    o3d.io.write_point_cloud(os.path.join(model_path,"registed_both.ply"),registed_source+target)
   
    print("3 point clouds saved in path: %s"%(model_path))
    print('2 registed clouds named as : registed_both.ply')
    
    '''print("segmentieren...")
    o3d.visualization.draw_geometries_with_editing([registed_source])
    print("cropped file saved in path:%s"%(model_path))
    
    #Set cropped reference json coodinates
    reference_json_cood = os.path.join(model_path,"cropped_1.json")
    segments_compare(registed_source,target,reference_json_cood,tolerance=0.4)

    '''
def ICP_calibration_segmentation_comparision():
    tk.messagebox.showinfo(message="Wählen Sie den Model Pfad ")
    model_path = filedialog.askdirectory()
    tk.messagebox.showinfo(message="Wählen Sie eine stl-mesh Datei ")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    target = o3d.geometry.PointCloud(mesh.vertices)
    tk.messagebox.showinfo(message="Wählen Sie eine ply-punktwolke Datei ")
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    source = o3d.io.read_point_cloud(source_path)
    
    trans_init=[[1,0,0,0],
                [0,1,0,0],    
                [0,0,1,0],            
                [0,0,0,1],]

    num_iteration = 50
    threshold = 30
    threshold_calib = 0.01


    down_sample_source = source.uniform_down_sample(every_k_points=3)
    registed_source,target = uti.icp_algo(source=source, target=target, threshold=threshold, trans_init=trans_init,max_iteration=num_iteration)
    
    #----calibration----:
    registed_source,target = uti.calibration_after_rough_reg(source=registed_source,target=target,threshold=threshold_calib,num_samples=5)
    o3d.io.write_point_cloud(os.path.join(model_path,"reg_source.ply"),registed_source)
    o3d.io.write_point_cloud(os.path.join(model_path,"reg_target.ply"),target)
    o3d.io.write_point_cloud(os.path.join(model_path,"registed_both.ply"),registed_source+target)
   
    print("3 point clouds saved in path: %s"%(model_path))
def ICP_calibration_segmentation_comparision_steps():
    tk.messagebox.showinfo(message="Wählen Sie den Model Pfad ")
    model_path = filedialog.askdirectory()
    tk.messagebox.showinfo(message="Wählen Sie eine stl-mesh Datei ")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    target = o3d.geometry.PointCloud(mesh.vertices)
    tk.messagebox.showinfo(message="Wählen Sie eine ply-punktwolke Datei ")
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    source = o3d.io.read_point_cloud(source_path)
    
    trans_init=[[1,0,0,0],
                [0,1,0,0],    
                [0,0,1,0],            
                [0,0,0,1],]

    num_iteration = 50
    threshold = 30
    threshold_calib = 0.01


    down_sample_source = source.uniform_down_sample(every_k_points=3)
    registed_source,target = uti.icp_algo_step_by_step(source=source, target=target, threshold=threshold, trans_init=trans_init,num_iteration=num_iteration)
    
    #----calibration----:
    registed_source,target = uti.calibration_after_rough_reg(source=registed_source,target=target,threshold=threshold_calib,num_samples=5)
    o3d.io.write_point_cloud(os.path.join(model_path,"reg_source.ply"),registed_source)
    o3d.io.write_point_cloud(os.path.join(model_path,"reg_target.ply"),target)
    o3d.io.write_point_cloud(os.path.join(model_path,"registed_both.ply"),registed_source+target)
   
    print("3 point clouds saved in path: %s"%(model_path))

def segmentation_kreis():
    tk.messagebox.showinfo(message="Wählen Sie den Model Pfad ")
    model_path = filedialog.askdirectory()
    ####
    tk.messagebox.showinfo(message="Wählen Sie eine ply-punktwolke Datei ")
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    tk.messagebox.showinfo(message="Wählen Sie zuerst Mittelpunkt der Kreises und dann Radius (shift + left mouseclick)")

    example1 = o3d.io.read_point_cloud(source_path)
    ####
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

    for j in range(2,len(coord)+183,3):
        coord.insert(j,0)

        j+=1

    arr = np.array(coord)
    new = np.reshape(arr, (-1,3))
    anew = np.float64(new)

    #json:
    json_path = os.path.join(model_path,"muster.json")
    json_path_out = os.path.join(model_path,"circle.json")
    dict1={}
    lis= anew.tolist()
    dict1 = uti.get_json_data(json_path, circle_coord=lis)
    uti.write_json_dict(json_path_out,dict1)

    cropped = uti.crop_from_json(example1, json_path_out)

    o3d.visualization.draw_geometries([cropped])
    #Punktwolke speichern
    v = tk.messagebox.askyesno(message="(Kreis) Punktwolke speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, cropped)

def segmentation_ring():
    tk.messagebox.showinfo(message="Wählen Sie den Model Pfad ")
    model_path = filedialog.askdirectory()
    tk.messagebox.showinfo(message="Wählen Sie eine ply-punktwolke Datei ")
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    tk.messagebox.showinfo(message="1, Mittelpunkt der Kreises auswählen \n 2, Inneren und außeren Radius auswählen (shift + left mouseclick)")

    example1 = o3d.io.read_point_cloud(source_path)
    
    picked = uti.pick_points(example1)
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
    v = tk.messagebox.askyesno(message="(Ring) Punktwolke speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, ring)
def segmentation_rechtecke():
    tk.messagebox.showinfo(message="Wählen Sie eine ply-punktwolke Datei ")
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    example1 = o3d.io.read_point_cloud(source_path)
    tk.messagebox.showinfo(message="y -> k -> Draw -> c(speichern)")
    print('y -> k -> Draw -> c(speichern)')
    o3d.visualization.draw_geometries_with_editing([example1])

def segmentation_rahmen():

    tk.messagebox.showinfo(message="Wählen Sie eine ply-punktwolke Datei ")
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    example1 = o3d.io.read_point_cloud(source_path)

    ## y -> k -> draw -> c
    #Rechtecke(außer) gespeichert als cropped_rect1
    tk.messagebox.showinfo(message="Wählen Sie einen Bereich für Rechtecke(außer) -- y -> k -> Draw -> c")
    o3d.visualization.draw_geometries_with_editing([example1])
    #Rechtecke(inner) gespeichert als cropped_rect2
    tk.messagebox.showinfo(message="Wählen Sie einen Bereich für Rechtecke(inner) -- y -> k -> Draw -> c")
    o3d.visualization.draw_geometries_with_editing([example1])

    tk.messagebox.showinfo(message="Wählen Sie gerade gespeicherte ply Datei -- y -> k -> Draw -> c")

    rect1 = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    rect2 = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    
    ex1 = o3d.io.read_point_cloud(rect1)
    ex2 = o3d.io.read_point_cloud(rect2)

    dis = ex1.compute_point_cloud_distance(ex2)
    dis = np.asarray(dis)
    ind = np.where(dis>0.01)[0]
    rahmen = ex1.select_by_index(ind)

    o3d.visualization.draw_geometries([rahmen])
    v = tk.messagebox.askyesno(message="(Rahmen) Punktwolke speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, rahmen)

if __name__ == '__main__':
    #Hauptfenster initialisieren
    root = tk.Tk()
    root.title('Prognose der Reststandmenge mittels 3D-Bildverarbeitung und Kraft-Analyse')
    root.geometry('1300x700+200+100')
    root.minsize(700, 300)
    #Menu im Hauptfenster generieren
    menubar = tk.Menu(root)
    filemenu = tk.Menu(menubar)
    ##############
    finetune_menu = tk.Menu(menubar)
    
    #Menu zu Hauptfenster hinzufügen
    menubar.add_cascade(label='Datei', menu=filemenu)

    menubar.add_cascade(label='Registrierung finetune',menu=finetune_menu)


    #Menu mit den entsprechenden Funktionen versehen
    filemenu.add_command(label='Mesh-Datei visualisieren',font=("Arial",13), command=open_mesh)
    filemenu.add_command(label='Punktwolke-Datei visualisieren',font=("Arial",13), command=open_pc)
    filemenu.add_command(label='Mesh in Punktwolke konvertieren und speichern',font=("Arial",13), command=mesh2pc)
    
    finetune_menu.add_command(label='icp_ani',font=("Arial",13),command=registration_icp_po2po_steps)
    finetune_menu.add_command(label='icp_ani(Animation)',font=("Arial",13),command=registration_icp_po2po_steps)
    finetune_menu.add_command(label='Finetune Kalibration',font=("Arial",13),command=ICP_calibration_segmentation_comparision)
    finetune_menu.add_command(label='Finetune Kalibration(Animation)',font=("Arial",13),command=ICP_calibration_segmentation_comparision_steps)
    finetune_menu.add_command(label='Manuell Segmentation(Kreis) ',font=("Arial",13),command=segmentation_kreis)
    finetune_menu.add_command(label='Manuell Segmentation(Ring) ',font=("Arial",13),command=segmentation_ring)
    finetune_menu.add_command(label='Manuell Segmentation(Rechtecke) ',font=("Arial",13),command=segmentation_rechtecke)
    finetune_menu.add_command(label='Manuell Segmentation(Rahmen) ',font=("Arial",13),command=segmentation_rahmen)

    #Funktion "Produktionsmenge berechnen mit Peripheriewinkelsatz" direkt in Hauptfenster initialisieren
    #Label erzeugen
    titel = tk.Label(root, text= 'Reststandmenge berechnen mit quadratischer Funktion ',font=('Arial',12,'bold'))
    titel1 = tk.Label(root,text='Produktionsmenge in Stück')
    titel2 = tk.Label(root,text='Verschleiß in mm')
    num1 = tk.Label(root,text='Stichprobe 1')
    num2 = tk.Label(root,text='Stichprobe 2')
    num3 = tk.Label(root,text='Stichprobe 3')
    tol = tk.Label(root,text='Verschleißtoleranz in mm')
        
    #Eingabenfelde erzeugen
    pro1 = tk.Entry(root)
    ver1 = tk.Entry(root)
    pro2 = tk.Entry(root)
    ver2 = tk.Entry(root)
    pro3 = tk.Entry(root)
    ver3 = tk.Entry(root)
    tole = tk.Entry(root)
    p_menge = tk.Entry(root)
    r_menge = tk.Entry(root)
        
    #Label und Eingabenfelder plazieren
    titel.grid(row=1,column=1)
    titel1.grid(row=2,column=1)
    titel2.grid(row=2,column=2)
    num1.grid(row=3,column=0)
    num2.grid(row=4,column=0)
    num3.grid(row=5,column=0)
    tol.grid(row=6,column=1)
    pro1.grid(row=3,column=1)
    ver1.grid(row=3,column=2)
    pro2.grid(row=4,column=1)
    ver2.grid(row=4,column=2)
    pro3.grid(row=5,column=1)
    ver3.grid(row=5,column=2)
    tole.grid(row=6,column=2)
    p_menge.grid(row=7,column=1)
    r_menge.grid(row=8,column=1)

    

    root.config(menu=menubar)
    root.mainloop()