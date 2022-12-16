import tkinter as tk
import open3d as o3d
from plyfile import PlyData
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
import pandas as pd



# Erstelle das Hauptfenster
window = tk.Tk()
window.title("GUI")

# Setze die Fenstergröße 
window.geometry("1800x1000")
#window.configure(background="white")
# Setze die Dateipfad an 
model_path = None
mesh_path =None
source_path = None
registed_source = None
registed_target = None
finetuned_source = None
finetuned_target = None
finetuned_registed__both = None
n = None
pc = []
ex = []

# Definiere die Methode, die beim Klicken auf den Button ausgeführt wird
def pfad_auswahl():
    global model_path
    model_path = filedialog.askdirectory()
    print("Pfad wurde ausgewält")
    

def stl_auswahl():
    global mesh_path
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    print("stl datei wurde ausgewält")
    

def ply_auswahl():
    global source_path
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    print("ply datei wurde ausgewält")

def ICP_steps():
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    target = o3d.geometry.PointCloud(mesh.vertices)
    
    source = o3d.io.read_point_cloud(source_path)
    
    trans_init=[[1,0,0,0],
                [0,1,0,0],    
                [0,0,1,0],            
                [0,0,0,1],]

    num_iteration = 50
    threshold = 30
    threshold_calib = 0.01


    #down_sample_source = source.uniform_down_sample(every_k_points=3)
    registed_source,registed_target = uti.icp_algo_step_by_step(source=source, target=target, threshold=threshold, trans_init=trans_init,num_iteration=num_iteration)
       


def ICP_mit_finetuning():
    global registed_source
    global registed_target
    global finetuned_source
    global finetuned_target
    global finetuned_registed__both
    # target
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    target = o3d.geometry.PointCloud(mesh.vertices)
    # source
    source = o3d.io.read_point_cloud(source_path)

    trans_init=[[1,0,0,0],
                [0,1,0,0],    
                [0,0,1,0],            
                [0,0,0,1],]

    num_iteration = 50
    threshold = 30
    threshold_calib = 0.01
    registed_source,registed_target = uti.icp_algo(source=source, target=target, threshold=threshold, trans_init=trans_init,max_iteration=num_iteration)
    v = tk.messagebox.askyesno(message="Wollen Sie Punktwolke speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(os.path.join(model_path,"registed_source.ply"),registed_source)
        o3d.io.write_point_cloud(os.path.join(model_path,"registed_target.ply"),registed_target)
        o3d.io.write_point_cloud(os.path.join(model_path,"registed__both.ply"),registed_source+registed_target)
        

    # finetuning:
    finetuning_process = tk.messagebox.askyesno(message="Führen Sie den Finetuning durch?")
    if finetuning_process == True:
        finetuned_source,finetuned_target = uti.calibration_after_rough_reg(source=registed_source,target=registed_target,threshold=threshold_calib,num_samples=int(input1.get()))
        o3d.io.write_point_cloud(os.path.join(model_path,"finetuned_source.ply"),finetuned_source)
        o3d.io.write_point_cloud(os.path.join(model_path,"finetuned_target.ply"),finetuned_target)
        finetuned_registed__both = finetuned_source+finetuned_target
        o3d.io.write_point_cloud(os.path.join(model_path,"finetuned_registed__both.ply"),finetuned_registed__both)
        print("finetuned Datein wurden in %s gespeichert"%(model_path))

def segmentation_kreis():
    
    print("Wählen Sie zuerst Mittelpunkt der Kreises und dann Radius (shift + left mouseclick)")
    example1 = o3d.io.read_point_cloud(os.path.join(model_path,"finetuned_registed__both.ply"))
    picked = uti.pick_points(example1)
    points_load = np.asarray(example1.points)
    x0 = round(points_load[picked[0]][0],2)
    y0 = round(points_load[picked[0]][1],2)
    x_edge = round(points_load[picked[1]][0],2)
    y_edge = round(points_load[picked[1]][1],2)
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
    print("Wählen Sie zuerst Mittelpunkt der Kreises und dann innere/außere-Radius (shift + left mouseclick)")
    example1 = o3d.io.read_point_cloud(os.path.join(model_path,"finetuned_registed__both.ply"))
    
    picked = uti.pick_points(example1)
    print(picked)

    points_load = np.asarray(example1.points)
    #points=points_load[picked[0],picked[1]]
    print(points_load[picked[0]])
    print(points_load[picked[1]])
    print(points_load[picked[2]])

    x0 = round(points_load[picked[0]][0],2)
    y0 = round(points_load[picked[0]][1],2)

    x_edge = round(points_load[picked[1]][0],2)
    y_edge = round(points_load[picked[1]][1],2)

    x_outer = round(points_load[picked[2]][0],2)
    y_outer = round(points_load[picked[2]][1],2)

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

def flaeche_auswaehlen(n=None):
    global pc 
    if n is None:
        n = int(input2.get())
    for i in range(n):
        pc_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
        readed = o3d.io.read_point_cloud(pc_path)
        pc.append(readed)

def kraftdatei_auswaehlen(n=None):
    global ex 
    if n is None:
        n = int(input2.get())
    for i in range(n):
        excel_path = filedialog.askopenfilename(filetypes=[("xlsx Datei","*.xlsx"),("Alle Datei","*.*")])
        readExcel = pd.read_excel(excel_path)
        print(readExcel)
        ex.append(readExcel)
'''
def flaeche_und_figure_anzeigen(n=None):
    global ex 
    a = []
    x = []
    y = []
    if n is None:
        n = int(input2.get())
        f = Figure(figsize=(5,4), dpi=150)
    for j in range(n):
        print('Toleranzfläche %d'%(j+1))
        o3d.visualization.draw_geometries([pc[j]])
        print('Kraftdaten %d'%(j+1)) 
        print(ex[j])   
        y_name = 'SENS%d_FZ'%(j+1)
        x = list(ex[j]['Time'])      
        y = list(ex[j][y_name])
        a =f.add_subplot(221)
        a.scatter(x,y,c="blue")
        plt.title(y_name)
        
        #plt.title(y_name)
        #plt.show()
    canvas  = FigureCanvasTkAgg(f, master=window)
    canvas.draw()
    canvas.get_tk_widget().grid(row=20,column=4, columnspan=3)  
'''
def flaeche_und_figure_anzeigen(n=None):
    global ex 
    x = []
    y = []
    if n is None:
        n = int(input2.get())
        f = Figure(figsize=(5,4), dpi=150)
        a = f.add_subplot()
        w,h=176,n
        X_arr = [[0 for x in range(w)] for y in range(h)] 
        Y_arr = [[0 for x in range(w)] for y in range(h)] 

        for j in range(n):
            o3d.visualization.draw_geometries([pc[j]])

            X_arr[j]=list(ex[j]['Time'])
            Y_arr[j]=list(ex[j]['SENS%d_FZ'%(j+1)])
        
        print(X_arr,Y_arr)
        if n ==1:
            a.plot(X_arr[0],Y_arr[0],'x')
        if n ==2:
            a.plot(X_arr[0],Y_arr[0],'x')
            a.plot(X_arr[1],Y_arr[1],'.')
        if n ==3:
            a.plot(X_arr[0],Y_arr[0],'x')
            a.plot(X_arr[1],Y_arr[1],'.')
            a.plot(X_arr[2],Y_arr[2],'r')
        if n ==4:
            a.plot(X_arr[0],Y_arr[0],'x')
            a.plot(X_arr[1],Y_arr[1],'.')
            a.plot(X_arr[2],Y_arr[2],'r')
            a.plot(X_arr[3],Y_arr[3],'*')

        canvas = FigureCanvasTkAgg(f, master=window)
        canvas.draw()
        canvas.get_tk_widget().grid(row=20,column=4, columnspan=3)
        '''
        X_polyfit = np.array(X[0])
        Y_polyfit =np.array(Y[0])
        f1 = np.polyfit(X_polyfit,Y_polyfit,2)

        a.plot(Y,X,'x')
        
        a.set_xlabel('Produktionsmenge in Stück')
        a.set_ylabel('Verschleiß in mm') 

        
'''
# Erstelle die Labeln
label1 = tk.Label(window, text="Anzahl des Referenzpunkts:",font=("Arial",12))
label2 = tk.Label(window, text="/",font=("Arial",12))
label3 = tk.Label(window, text="/",font=("Arial",12))
label4 = tk.Label(window, text="Anzahl der Segmentiertefläche:",font=("Arial",12))

# Erstelle die Eingabefelder
input1 = tk.Entry(window)
input2 = tk.Entry(window)

# Erstelle die Schaltflächen
button1 = tk.Button(window, text="1.Pfad_auswählen", command=pfad_auswahl,font=("Arial",12))
button2 = tk.Button(window, text="2.stl_auswählen", command=stl_auswahl,font=("Arial",12))
button3 = tk.Button(window, text="3.ply_auswählen", command=ply_auswahl,font=("Arial",12))
button4 = tk.Button(window, text="4.ICP_mit_finetuning", command=ICP_mit_finetuning,font=("Arial",12))
button5 = tk.Button(window, text="ICP_steps", command=ICP_steps,font=("Arial",12))
button6 = tk.Button(window, text="5.Segmentierung_kreis", command=segmentation_kreis,font=("Arial",12))
button7 = tk.Button(window, text="Segmentierung_ring", command=segmentation_ring,font=("Arial",12))
button8 = tk.Button(window, text="6.Segmentiertefläche einladen", command=flaeche_auswaehlen,font=("Arial",12))
button9 = tk.Button(window, text="7.Kraftwert einladen", command=kraftdatei_auswaehlen,font=("Arial",12))
button10 = tk.Button(window, text="8.Flaeche und Figure anzeigen", command=flaeche_und_figure_anzeigen,font=("Arial",12))


# Platziere die Schaltflächen und Eingabefelder und labeln im Fenster
label1.place(relx=0.1, rely=0.25, anchor="c")
label2.place(relx=0.28, rely=0.28, anchor="c")
label3.place(relx=0.28, rely=0.33, anchor="c")
label4.place(relx=0.1, rely=0.37, anchor="c")

input1.place(relx=0.22, rely=0.25, anchor="c",width=50)
input2.place(relx=0.22, rely=0.37, anchor="c",width=50)

button1.place(relx=0.2, rely=0.1, anchor="c")
button2.place(relx=0.2, rely=0.15, anchor="c")
button3.place(relx=0.2, rely=0.2, anchor="c")
button4.place(relx=0.2, rely=0.28, anchor="c")
button5.place(relx=0.33, rely=0.28, anchor="c")
button6.place(relx=0.2, rely=0.33, anchor="c")
button7.place(relx=0.36, rely=0.33, anchor="c")
button8.place(relx=0.23, rely=0.40, anchor="c")
button9.place(relx=0.23, rely=0.44, anchor="c")
button10.place(relx=0.23, rely=0.48, anchor="c")
# Betreten die Meldungsschleife

window.mainloop()
