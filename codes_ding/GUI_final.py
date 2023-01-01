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
finetuned_registed_both = None
anzahl_seg = 0
anzahl_prob = 0
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
    print("stl datei wurde ausgewält, 'q' drücken, um fortzufahren.")
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    pc_mesh = o3d.geometry.PointCloud(mesh.vertices)
    o3d.visualization.draw_geometries([pc_mesh])
    
    

def ply_auswahl():
    global source_path
    source_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    print("ply datei wurde ausgewält, 'q' drücken, um fortzufahren.")
    pc_source = o3d.io.read_point_cloud(source_path)
    o3d.visualization.draw_geometries([pc_source])

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
    global finetuned_registed_both
    
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
    v = tk.messagebox.askyesno(message="Wollen Sie Punktwolke (ohne Finetuning) speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(os.path.join(model_path,"registed_source_no_finetune.ply"),registed_source)
        o3d.io.write_point_cloud(os.path.join(model_path,"registed_target_no_finetune.ply"),registed_target)
        o3d.io.write_point_cloud(os.path.join(model_path,"registed_both_no_finetune.ply"),registed_source+registed_target)
        

    # finetuning:
    finetuning_process = tk.messagebox.askyesno(message="Führen Sie den Finetuning durch?")
    answer = tk.simpledialog.askinteger("Input", "Anzahl der Referenzpunkt:", parent=window)
    if finetuning_process == True:
        finetuned_source,finetuned_target = uti.calibration_after_rough_reg(source=registed_source,target=registed_target,threshold=threshold_calib,num_samples=answer)
        #finetuned_source,finetuned_target = uti.calibration_after_rough_reg(source=registed_source,target=registed_target,threshold=threshold_calib,num_samples=int(input1.get()))
        o3d.io.write_point_cloud(os.path.join(model_path,"finetuned_source.ply"),finetuned_source)
        o3d.io.write_point_cloud(os.path.join(model_path,"finetuned_target.ply"),finetuned_target)
        finetuned_registed_both = finetuned_source+finetuned_target
        o3d.io.write_point_cloud(os.path.join(model_path,"finetuned_registed_both.ply"),finetuned_registed_both)
        print("finetuned Datein(finetuned_source.ply; finetuned_target.ply; finetuned_registed_both.ply) wurden in %s gespeichert"%(model_path))

def segmentation_kreis():
    global anzahl_seg
    anzahl_seg = tk.simpledialog.askinteger("Input", "Anzahl der Segmentation:", parent=window)
    for k in range(anzahl_seg):
        print("Wählen Sie zuerst Mittelpunkt der Kreises und dann Radius (shift + left mouseclick)")
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        pc_mesh = o3d.geometry.PointCloud(mesh.vertices)
        example1 = pc_mesh
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
        json_path_out = os.path.join(model_path,"cropped_%d.json"%(k+1))
        dict1={}
        lis= anew.tolist()
        dict1 = uti.get_json_data(json_path, circle_coord=lis)
        uti.write_json_dict(json_path_out,dict1)

        cropped = uti.crop_from_json(example1, json_path_out)

        o3d.visualization.draw_geometries([cropped])
        #Punktwolke speichern
        v = tk.messagebox.askyesno(message="%d.kreisformige Punktwolke speichern?"%(k+1))
        if v == True:
            #save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                    #defaultextension = ".ply")
            save_path = os.path.join(model_path,"cropped_%d.ply"%(k+1))                                        
            o3d.io.write_point_cloud(save_path, cropped)  

def segmentation_ring():
    print("Wählen Sie zuerst Mittelpunkt der Kreises und dann innere/außere-Radius (shift + left mouseclick)")
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    pc_mesh = o3d.geometry.PointCloud(mesh.vertices)
    example1 = pc_mesh
    
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

def tolerenzflächen_anzeigen():
    for l in range(anzahl_seg):
        to_be_cropped = os.path.join(model_path,"finetuned_registed_both.ply")
        pc_cropped = o3d.io.read_point_cloud(to_be_cropped)
        json_path = os.path.join(model_path,"cropped_%d.json"%(l+1))
        vol  = o3d.visualization.read_selection_polygon_volume(json_path)
        cropped = vol.crop_point_cloud(pc_cropped)
        cropped.paint_uniform_color([1, 0.651, 1]) 
        o3d.visualization.draw_geometries([cropped]) 
        


def flaeche_auswaehlen(n=None):
    global pc 
    if n is None:
        n = int(input2.get())
    for i in range(n):
        pc_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
        readed = o3d.io.read_point_cloud(pc_path)
        pc.append(readed)

def kraftdatei_auswaehlen_diag_anzeigen(n=None):
    global ex 
    if n is None:
        n = int(anzahl_seg)
    for i in range(n):
        excel_path = filedialog.askopenfilename(filetypes=[("xlsx Datei","*.xlsx"),("Alle Datei","*.*")])
        readExcel = pd.read_excel(excel_path)
        print(readExcel)
        ex.append(readExcel)
    
    x = []
    y = []
    
   
    f = Figure(figsize=(5,4), dpi=150)
    a = f.add_subplot()
    w,h=176,n
    X_arr = [[0 for x in range(w)] for y in range(h)] 
    Y_arr = [[0 for x in range(w)] for y in range(h)] 

    for j in range(n):
        #o3d.visualization.draw_geometries([pc[j]])

        X_arr[j]=list(ex[j]['Time'])
        Y_arr[j]=list(ex[j]['SENS%d_FZ'%(j+1)])
    
    print(X_arr,Y_arr)
    if n ==1:
        a.plot(X_arr[0],Y_arr[0],'x',label='Sensor 1')
        a.set_xlabel('Zeit')
        a.set_ylabel('kraftwert')
        
    if n ==2:
        a.plot(X_arr[0],Y_arr[0],'x',label='Sensor 1')
        a.plot(X_arr[1],Y_arr[1],'.',label='Sensor 2')
        a.set_xlabel('Zeit')
        a.set_ylabel('kraftwert')
    if n ==3:
        a.plot(X_arr[0],Y_arr[0],'x',label='Sensor 1')
        a.plot(X_arr[1],Y_arr[1],'.',label='Sensor 2')
        a.plot(X_arr[2],Y_arr[2],'r',label='Sensor 3')
        a.set_xlabel('Zeit')
        a.set_ylabel('kraftwert')
    if n ==4:
        a.plot(X_arr[0],Y_arr[0],'x',label='Sensor 1')
        a.plot(X_arr[1],Y_arr[1],'.',label='Sensor 2')
        a.plot(X_arr[2],Y_arr[2],'r',label='Sensor 3')
        a.plot(X_arr[3],Y_arr[3],'*',label='Sensor 4')
        a.set_xlabel('Zeit')
        a.set_ylabel('kraftwert')
    a.legend()
    canvas = FigureCanvasTkAgg(f, master=window)
    canvas.draw()
    canvas.get_tk_widget().grid(row=11,column=1, columnspan=3)

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
            #o3d.visualization.draw_geometries([pc[j]])

            X_arr[j]=list(ex[j]['Time'])
            Y_arr[j]=list(ex[j]['SENS%d_FZ'%(j+1)])
        
        print(X_arr,Y_arr)
        if n ==1:
            a.plot(X_arr[0],Y_arr[0],'x',label='Sensor 1')
            a.set_xlabel('Zeit')
            a.set_ylabel('kraftwert')
            
        if n ==2:
            a.plot(X_arr[0],Y_arr[0],'x',label='Sensor 1')
            a.plot(X_arr[1],Y_arr[1],'.',label='Sensor 2')
            a.set_xlabel('Zeit')
            a.set_ylabel('kraftwert')
        if n ==3:
            a.plot(X_arr[0],Y_arr[0],'x',label='Sensor 1')
            a.plot(X_arr[1],Y_arr[1],'.',label='Sensor 2')
            a.plot(X_arr[2],Y_arr[2],'r',label='Sensor 3')
            a.set_xlabel('Zeit')
            a.set_ylabel('kraftwert')
        if n ==4:
            a.plot(X_arr[0],Y_arr[0],'x',label='Sensor 1')
            a.plot(X_arr[1],Y_arr[1],'.',label='Sensor 2')
            a.plot(X_arr[2],Y_arr[2],'r',label='Sensor 3')
            a.plot(X_arr[3],Y_arr[3],'*',label='Sensor 4')
            a.set_xlabel('Zeit')
            a.set_ylabel('kraftwert')
        a.legend()
        canvas = FigureCanvasTkAgg(f, master=window)
        canvas.draw()
        canvas.get_tk_widget().grid(row=11,column=1, columnspan=3)

def bestaetigen():
    global anzahl_prob
    anzahl_prob = int(input1.get())
    print("%d Stichproben bestätigt! "%(anzahl_prob))
    #titel = tk.Label(window,text="Reststandmenge schätzen mit Ausgleichsrechnung",font=('Arial',12,'bold'))
    titel1 = tk.Label(window,text='Produktionsmenge in Stück')
    titel2 = tk.Label(window,text='Verschleiß in mm')
    #titel.grid(row=0,column=5)
    titel1.grid(row=2,column=7)
    titel2.grid(row=2,column=8)
    for i in range(anzahl_prob):
        (tk.Label(window,text='Stichprobe %d'%(i+1)).grid(row=i+3,column=6))
    tol = tk.Label(window,text='Verschleißtoleranz in mm:')
    tol.grid(row=anzahl_prob+2,column=9)
    input_tol= tk.Entry(window)
    input_tol.grid(row=anzahl_prob+2,column=10)
    names = locals()
    for m in range(anzahl_prob):
        names['prob' + str(m+1)] = tk.Entry(window)
        (names['prob' + str(m+1)]).grid(row=m+3,column=7)
        names['versc' + str(m+1)] = tk.Entry(window)
        (names['versc' + str(m+1)]).grid(row=m+3,column=8)
    
    def quadratisch():
        X_poly=[] 
        Y_poly=[] 
        for n in range(anzahl_prob):
            X_poly.append(float((names['versc' + str(n+1)]).get()))
            Y_poly.append(int((names['prob' + str(n+1)]).get()))
        #Curve Fitting mit numpy.polyfit
        X_polyfit = np.array(X_poly)
        Y_polyfit =np.array(Y_poly)
        f1 = np.polyfit(X_polyfit,Y_polyfit,2)
        p = np.poly1d(f1)
        t = np.linspace(0, float(input_tol.get()),250)
        
        diag = Figure(figsize=(5,4), dpi=150)
        a_diag = diag.add_subplot()
        a_diag.scatter(p(t)[-1],t[-1],color='red')
        a_diag.plot([p(t)[-1],p(t)[-1]],[0,t[-1]],c='b',linestyle='--')#Senkrecht
        a_diag.plot([0,p(t)[-1]],[t[-1],t[-1]],c='b',linestyle='--')#Paralle
        a_diag.text(p(t)[-1],t[-1]-0.02,'Prognose\n(%d)'%(p(t)[-1]))
        a_diag.plot(Y_poly,X_poly,'o',p(t),t,"-")#,p(t)[-1],t[-1],'x')
        a_diag.set_xlabel('Productionsmenge')
        a_diag.set_ylabel('Verschleiß')

        canvas = FigureCanvasTkAgg(diag, master=window)
        canvas.draw()
        canvas.get_tk_widget().grid(row=anzahl_prob+4,column=7, columnspan=3)

    button10 = tk.Button(window, text="Prognose", command=quadratisch,font=("Arial",12))   
    button10.grid(row=anzahl_prob+3,column=7)   

# Erstelle die Labeln (Haupt Programm)

label_Hauptprogram = tk.Label(window, text="Haupt-programm",font=("Arial",18))
label0 = tk.Label(window, text="|\n",font=("Arial",12))
label1 = tk.Label(window, text="|\n",font=("Arial",12))
label2 = tk.Label(window, text="/",font=("Arial",12))
label3 = tk.Label(window, text="/",font=("Arial",12))
label4 = tk.Label(window, text="--------------------------------",font=("Arial",12))
label5 = tk.Label(window, text="--------------------------------",font=("Arial",12))
label6 = tk.Label(window, text="--------------------------------",font=("Arial",12))
label7 = tk.Label(window, text="--------------------------------",font=("Arial",12))
label8 = tk.Label(window, text="---------",font=("Arial",12))
label9 = tk.Label(window, text="|\n",font=("Arial",12))
label10 = tk.Label(window, text="|\n",font=("Arial",12))
label11 = tk.Label(window, text="|\n",font=("Arial",12))
label12 = tk.Label(window, text="|\n",font=("Arial",12))
label_Verhersage = tk.Label(window, text="Verhersage-funktion",font=("Arial",18))
label13 = tk.Label(window, text="Anzahl der Stichproben:",font=("Arial",12))
# Erstelle die Eingabefelder (Verhersage Funktion)
input1 = tk.Entry(window)


# Erstelle die Schaltflächen
button1 = tk.Button(window, text="1.Model_Pfad_auswählen", command=pfad_auswahl,font=("Arial",12))
button2 = tk.Button(window, text="2.stl_auswählen", command=stl_auswahl,font=("Arial",12))
button3 = tk.Button(window, text="3.Segmentierung_kreis", command=segmentation_kreis,font=("Arial",12))
button4 = tk.Button(window, text="4.Segmentierung_ring(optimal)", command=segmentation_ring,font=("Arial",12))
button5 = tk.Button(window, text="5.ply_auswählen", command=ply_auswahl,font=("Arial",12))
button6 = tk.Button(window, text="6.ICP_mit_finetuning", command=ICP_mit_finetuning,font=("Arial",12))
button7 = tk.Button(window, text="7.ICP_steps(optimal)", command=ICP_steps,font=("Arial",12))
button8 = tk.Button(window, text="8.Tolerenzflächen_anzeigen", command=tolerenzflächen_anzeigen ,font=("Arial",12))
button9 = tk.Button(window, text="9.Kraftwert anzeigen", command=kraftdatei_auswaehlen_diag_anzeigen,font=("Arial",12))
button10 = tk.Button(window, text="Bestätigen", command=bestaetigen,font=("Arial",12))
'''
button8 = tk.Button(window, text="6.Segmentiertefläche einladen", command=flaeche_auswaehlen,font=("Arial",12))
button9 = tk.Button(window, text="7.Kraftwert einladen", command=kraftdatei_auswaehlen_diag_anzeigen,font=("Arial",12))
button10 = tk.Button(window, text="8.Flaeche und Figure anzeigen", command=flaeche_und_figure_anzeigen,font=("Arial",12))
'''


# Platziere die Schaltflächen und Eingabefelder und labeln im Fenster
label_Hauptprogram.grid(row=0,column=2)
label0.grid(row=2,column=5)
label1.grid(row=3,column=5)
label2.grid(row=3,column=3)
label3.grid(row=6,column=3)
label4.grid(row=4,column=1)
label5.grid(row=4,column=2)
label6.grid(row=4,column=3)
label7.grid(row=4,column=4)
label8.grid(row=4,column=5)
label9.grid(row=5,column=5)
label10.grid(row=6,column=5)
label11.grid(row=7,column=5)
label12.grid(row=8,column=5)
label_Verhersage.grid(row=0,column=7)
label13.grid(row=1,column=7)

input1.grid(row=1,column=8)


button1.grid(row=1,column=2)
button2.grid(row=2,column=2)
button3.grid(row=3,column=2)
button4.grid(row=3,column=4)
button5.grid(row=5,column=2)
button6.grid(row=6,column=2)
button7.grid(row=6,column=4)
button8.grid(row=7,column=2)
button9.grid(row=8,column=2)
button10.grid(row=1,column=9)

window.mainloop()
