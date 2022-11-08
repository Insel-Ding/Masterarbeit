# -*- coding: utf-8 -*-
"""
Created on Wed Jan 26 14:31:02 2022

@author: liang
"""

import open3d as o3d
import tkinter as tk
import numpy as np
from tkinter import filedialog
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from sympy import *

#Hauptfenster initialisieren
window = tk.Tk()
window.title('Prognose der Reststandmenge mittels 3D-Bildverarbeitung')
window.geometry('2200x1300+50+50')
window.minsize(1600,1000)

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
        
#Funktion Menu "Vorverarbeitung - Rauschenunterdrückung - Mit Statistical Outlier Removal"
def sta_out():
    #Lade Punktwolke Datei
    tk.messagebox.showinfo(message="Punktwolke-Datei öffnen")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    pcd = o3d.io.read_point_cloud(pcd_path)

    #Eingabe Parametern
    std_ratio = tk.simpledialog.askfloat("Statistical Outlier Removal",
                               "Standardabweichung der durchschnittlichen Abstand zwischen den Nachbarpunkten\n\
    Typischer Wert 0,1 bis 0,5, je kleiner der Wert, desto mehr Punkte werden gefiltert")
    nb_neighbors = tk.simpledialog.askinteger("Statistical Outlier Removal", 
                                            "Anzahl der Nachbarpunkte:\n\
    Typischer Wert 20 bis 50")
    #Rauschenunterdrückung mit radius outlier removal
    pcd_processed, ind = pcd.remove_radius_outlier(nb_neighbors, std_ratio)
    noise = pcd.select_by_index(ind, invert=True)
    
    #Visualisierung
    pcd_processed.paint_uniform_color([0, 0, 1]) # in blau
    noise.paint_uniform_color([1, 0, 0]) # in rot
    o3d.visualization.draw_geometries([pcd_processed, noise])
    
    #Ergebnisse speichern
    v = tk.messagebox.askyesno(message="Ergebnis speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply"),("xyz Datei","*.xyz"),("pcd Datei","*.pcd")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, pcd_processed)
        
#Funktion Menu "Registrierung -- Point-to-Point" 
def registration_icp_po2po():
    #Datei laden
    tk.messagebox.showinfo(message="Mesh-Datei öffnen")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    tk.messagebox.showinfo(message="Punktwolke-Datei öffnen")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    source = o3d.io.read_point_cloud(pcd_path)
    target = o3d.geometry.PointCloud(mesh.vertices)
    
    #Registrierung mit ICP
    trans_init = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]     
    threshold = 40  #Max. Abstand zwischen Mesh und Punktwolken
    reg_po2po = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())  
    source.transform(reg_po2po.transformation)
    print(reg_po2po)
    print(reg_po2po.transformation)
    
    #Ergebnisse visualisieren
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Visualizer', width=1600, height=1200)
    source.paint_uniform_color([1, 0, 0])  # rot
    target.paint_uniform_color([0, 1, 0])  # grün
    vis.add_geometry(source)
    vis.add_geometry(target)
    vis.run()
    vis.destroy_window()
    
    #Datei speichern
    s = tk.messagebox.askyesno(message="Registrierte Punktwolke speichern?")
    if s == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply"),("xyz Datei","*.xyz")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, source)
        
#Funktion Menu "Registrierung -- Point-to-Plane" 
def registration_icp_po2pl():
    #Datei laden
    tk.messagebox.showinfo(message="Mesh-Datei öffnen")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    tk.messagebox.showinfo(message="Punktwolke-Datei öffnen")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    source = o3d.io.read_point_cloud(pcd_path)
    target = o3d.geometry.PointCloud(mesh.vertices)
    
    #Oberflächennormale berechnen
    source.estimate_normals()
    target.estimate_normals()
    
    #Registrierung mit ICP
    trans_init = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]     
    threshold = 40  #Max. Abstand zwischen Mesh und Punktwolken
    reg_po2pl = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPlane())  
    source.transform(reg_po2pl.transformation)
    print(reg_po2pl)
    print(reg_po2pl.transformation)
    
    #Ergebnisse visualisieren
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Visualizer', width=1600, height=1200)
    source.paint_uniform_color([1, 0, 0])  # rot
    target.paint_uniform_color([0, 1, 0])  # grün
    vis.add_geometry(source)
    vis.add_geometry(target)
    vis.run()
    vis.destroy_window()
    
    #Datei speichern
    s = tk.messagebox.askyesno(message="Registrierte Punktwolke speichern?")
    if s == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply"),("xyz Datei","*.xyz")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, source)
        
#Funktion Menu "Manuelle Segmentierung -- Bereich in Mesh-Datei auswählen"
def segmentation_manu_crop():
    #Original Mesh Datei laden
    tk.messagebox.showinfo(message="Referenzdatei (z.B. *.stl) öffnen")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl"),("Alle Datei","*.*")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    pc = o3d.geometry.PointCloud(mesh.vertices)
    
    #Bereich manuell auswählen und .json Datei generieren
    tk.messagebox.showinfo(message="Bereich manuell auswählen:\n\
    1.Ansicht auswählen mit Strl+X, Strl+Y oder Strl+Z, \n\
    2.Ansicht fixieren mit Strl+K, \n\
    3.Bereich manuell auswählen mit gedrückte Strl-Taste,\n\
    4.Taste C drücken, um ausgewähltes Bereich zu speichern\n\
    5.Die koordinaten des Bereichs werden in .json Datei gespeichert.")
    o3d.visualization.draw_geometries_with_editing([pc])

#Funktion Menu "Manuelle Segmentierung -- Punktwolke mit Muster segmentieren"
def segmentation_manu_save():
    #Ziel Datei laden    
    tk.messagebox.showinfo(message="Quellpunktwolke öffnen")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    pcd = o3d.io.read_point_cloud(pcd_path)
    
    #.json Datei laden
    tk.messagebox.showinfo(message=".json Datei auswählen")
    crop_path = filedialog.askopenfilename(filetypes=[("json Datei","*.json")])
    crop = o3d.visualization.read_selection_polygon_volume(crop_path)
    auswahl = crop.crop_point_cloud(pcd) 
    
    #Ausgewählte Punktwolke visualiezieren 
    s = tk.messagebox.askyesno(message="Segmentierte Punktwolke anzeigen?")
    if s == True:
        auswahl.paint_uniform_color([0, 1, 0])
        o3d.visualization.draw_geometries([auswahl])
    #Punktwolke speichern
    v = tk.messagebox.askyesno(message="Segmentierte Punktwolke speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply"),("xyz Datei","*.xyz")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, auswahl)

#Funktion Menu "Automatische Segmentierung -- mit RANSAC"
def segmentation_auto_ran():
    #Original Mesh Datei laden
    tk.messagebox.showinfo(message="Datei öffnen")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl"),("Alle Datei","*.*")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    pc = o3d.geometry.PointCloud(mesh.vertices)
    
    #Eingabe Parametern
    dis = tk.simpledialog.askfloat("Segmentieren mit RANSAC-Algorithmus",
                               "Abstand zwischen Inlier und Outlier eingeben:\n\
    Typischer Wert 0,2 bis 2,0")
    ransac_n = 3            #Anzahl Punkte am Anfang  
    num_iterations = 2000   #Anzahl Iteration
    
    #Segmentierung mit RANSAC
    plane_model, inliers = pc.segment_plane(dis, ransac_n, num_iterations)
    inlier_cloud = pc.select_by_index(inliers)
    outlier_cloud = pc.select_by_index(inliers, invert=True)
    
    
    #Visualisierung
    inlier_cloud.paint_uniform_color([0, 0, 1]) #Inlier in blau
    outlier_cloud.paint_uniform_color([1, 0, 0]) #Outliner in rot
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    
    #Segmentierte Punktwolke speichern
    v = tk.messagebox.askyesno(message="Punktwolke in blau speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, inlier_cloud)
    k = tk.messagebox.askyesno(message="Punktwolke in rot speichern?")
    if k == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                     defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, outlier_cloud)
    
#Funktion Menu "Automatische Segmentierung -- mit DBSCAN"
def segmentation_auto_dbscan():
    #Lade Punktwolke Datei
    tk.messagebox.showinfo(message="Punktwolke-Datei öffnen")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    pcd = o3d.io.read_point_cloud(pcd_path)

    #Eingabe Parametern
    eps = tk.simpledialog.askfloat("Segmentieren mit DBSCAN Algorithmus",
                               "Maximale Abstand zwischen Punkten in der selben Gruppe:\n\
    Typischer Wert 4 bis 10")
    min_points = tk.simpledialog.askinteger("Segmentieren mit DBSCAN Algorithmus", 
                                            "Kleineste Anzahl Punkte in einer Gruppe\n\
    Typischer Wert 100 bis 300")
    #Segmentierung mit DBScan
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps, min_points, print_progress=True))
    max_label = labels.max() 
    
    #Visualisierung
    tk.messagebox.showinfo(message=" Ergebnis der Segmentierung wird gleich präsentiert,  \n\
    Rauschpunkte werden in schwarz angezeigt.")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0  
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd])
    
    #Ergebnisse speichern
    v = tk.messagebox.askyesno(message="Punktwolke in %d Gruppen geteilt,Ergebnisse speichern?"
                               % (max_label+1))
    if v == True:

        for i in range(max_label+1):
            ind = np.where(labels == i)[0]
            clusters_cloud = pcd.select_by_index(ind)
            file_name = "dbscan_cluster" + str(i+1) + ".ply"
            o3d.io.write_point_cloud(file_name, clusters_cloud)

#Funktion Menu "Automatische Segmentierung -- Kombiniertes Verfahren mit Bauteilabmessung"
def segmentation_auto_mass():
    #Lade Vergleichsebene
    tk.messagebox.showinfo(message="Basisebene öffnen wie z.B. Boden")
    basis_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply")])
    basis = o3d.io.read_point_cloud(basis_path)
    
    #Lade Punktwolke
    tk.messagebox.showinfo(message="Registrierte Punktwolke zu segmentieren öffnen")
    pc_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("Alle Datei","*.*")])
    pc = o3d.io.read_point_cloud(pc_path)
    
    #Ober-/Untergrenze eingeben
    obergrenze = tk.simpledialog.askfloat("Segmentieren mit Bauteilabmessung",
                                        "Abstand der Obergrenze zur Basisebene in mm:")
    untergrenze = tk.simpledialog.askfloat("Segmentieren mit Maßeingaben",
                                        "Abstand der Untergrenze zur Basisebene in mm:")
    
    #Abstand zu Boden berechnen und in Array speichern
    dists = pc.compute_point_cloud_distance(basis)
    dists = np.asarray(dists) 
    
    #Bereich unterhalb der Obergrenze segmentieren
    unter_dist = np.where(dists <= (obergrenze+0.1))[0] #0,1 als Puffer für Obergrenze
    unter = pc.select_by_index(unter_dist)
    
    #Neuberechnung des Unterteils, Ergebnis wieder als Array speichern
    dists_neu = unter.compute_point_cloud_distance(basis)
    dists_neu = np.asarray(dists_neu)
    
    #Bereich über der Untergrenze segmentieren als Zielbereich
    ueber_dist = np.where(dists_neu >= untergrenze)[0]
    auswahl = unter.select_by_index(ueber_dist)
    
    #Visualisierung und speichern
    auswahl.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([auswahl]) 
    v = tk.messagebox.askyesno(message="Ergebnisse speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, auswahl) 

#Funktion Menu "Verschleiß berechnen"
def compute_distance():
    #Nummer der Stichprobe erfragen
    #num_s = tk.simpledialog.askinteger("Nummer der Stichprobe eingeben",
    #                                  "Geben Sie die Nummer der Stichprobe ein(1,2 oder 3):")
    #Lade Referenzfläche
    tk.messagebox.showinfo(message="Basisebene öffnen")
    refe_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("xyz Datei","*.xyz*")])
    refe = o3d.io.read_point_cloud(refe_path)
    
    #Lade Original-Datei
    tk.messagebox.showinfo(message="Originale Punktwolke-Datei öffnen")
    ori_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("xyz Datei","*.xyz*")])
    ori = o3d.io.read_point_cloud(ori_path)
    
    #Lade Verschleiss-Datei
    tk.messagebox.showinfo(message="Verschleiß Punktwolke-Datei öffnen")
    vers_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply"),("xyz Datei","*.xyz*")])
    vers = o3d.io.read_point_cloud(vers_path)
    
    #Abstand berechnen durch Vergleich mit der Referenzfläche 
    distance_ori2refe = ori.compute_point_cloud_distance(refe)
    distance_vers2refe = vers.compute_point_cloud_distance(refe)
    dists_ori2refe = np.asarray(distance_ori2refe)
    dists_vers2refe = np.asarray(distance_vers2refe)
    
    #Abstand zur Referenzfläche im Durchschnitt
    abstand_ori2refe = np.mean(dists_ori2refe) 
    abstand_vers2refe = np.mean(dists_vers2refe)
    
    #Differenz zwischen den Abständen zur Referenzfläche ist der Verschleißwert  
    abstand_mean = abstand_ori2refe - abstand_vers2refe 
    
    v = tk.messagebox.askyesno(message="Verschleiß beträgt im Durchschnitt %.3f mm \n\
 Ergebnis direkt ins Hauptfenster unter \"Reststandmenge berechnen mit quadratischer Funktion\" eintragen?" 
     %abstand_mean)
    
    #Eintragen das Ergebnis im Eingabenfeld
    if v == True:
        if num_s == 1:
            ver1.delete(0,'end')
            ver1.insert(0,'%.4g' %abstand_mean)
        if num_s == 2:
            ver2.delete(0,'end')
            ver2.insert(0,'%.4g' %abstand_mean)
        if num_s == 3:
            ver3.delete(0,'end')
            ver3.insert(0,'%.4g' %abstand_mean)
        else:
            pass
        
#Funktion Button "Produktionsmenge berechnen (mit quadratischer Funktion)"
def produktionsmenge_peri():

    #Hier X - Produktionsmenge, Y - Verschleiß, t - Verschleißtoleranz
    X = [int(pro1.get()), int(pro2.get()), int(pro3.get())]
    Y = [float(ver1.get()), float(ver2.get()), float(ver3.get())]
    t = float(tole.get())
    
    x = Symbol('x')
    solved_value=solve([(t -Y[0])/(x-X[0]) - (t-Y[1])/(x-X[1]) 
                    - (Y[2]-Y[0])/(X[2]-X[0]) + (Y[2]-Y[1])/(X[2]-X[1])] , [x])
    
    ergeb = int(solved_value[1][0]) #Nur das positive Ergebnis 
    p_menge.delete(0, 'end')
    p_menge.insert(0,'%d' %ergeb)

#Funktion Button "Reststandmenge" (mit quadratischer Funktion) 
def reststand_peri():
    
    menge_gesamt = int(p_menge.get())
    menge_pro = int(pro3.get())
    menge_rest = menge_gesamt - menge_pro
    
    r_menge.delete(0, 'end')
    r_menge.insert(0,'%d' %menge_rest)
    
#Funktion Menu "Produktionsmenge berechnen mit alternativen Methoden"
def alternativ():

    #Anzahl der Exemplare eingeben
    anzahl = tk.simpledialog.askinteger("Anzahl der Stichproben",
                               "Anzahl der Stichproben (max. 15):")
    
    #Label erzeugen und platzieren
    titel = tk.Label(window,text="Reststandmenge schätzen mit Ausgleichsrechnung",font=('Arial',12,'bold'))
    titel1 = tk.Label(window,text='Produktionsmenge in Stück')
    titel2 = tk.Label(window,text='Verschleiß in mm')
    titel.grid(row=0,column=5)
    titel1.grid(row=1,column=5)
    titel2.grid(row=1,column=6)
    for i in range(anzahl):
        (tk.Label(window,text='Stichprobe %d'%(i+1)).grid(row=i+2,column=4))
    tol = tk.Label(window,text='Verschleißtoleranz in mm')
    tol.grid(row=anzahl+2,column=5)
    
    #Eingabenfelde erzeugen und platzieren
    names = locals()
    for i in range(anzahl):
        names['pro' + str(i+1)] = tk.Entry(window)
        (names['pro' + str(i+1)]).grid(row=i+2,column=5)
        names['ver' + str(i+1)] = tk.Entry(window)
        (names['ver' + str(i+1)]).grid(row=i+2,column=6)
    tole = tk.Entry(window)
    tole.grid(row=anzahl+2,column=6)
    ergeb_q = tk.Entry(window)
    ergeb_q.grid(row=anzahl+3,column=5)
    r_menge = tk.Entry(window)
    r_menge.grid(row=anzahl+4,column=5)
    
    #Mit Modell Quadratische Funktion die Produktionsmenge prognostizieren
    def quadratisch():
        #Bei der Berechnung X:Verschleiß, Y:Produktionsmenge, da Reststandmenge gesucht wird
        X_poly=[] 
        Y_poly=[] 
        for i in range(anzahl):
            X_poly.append(float((names['ver' + str(i+1)]).get()))
            Y_poly.append(int((names['pro' + str(i+1)]).get()))
            
        #Curve Fitting mit numpy.polyfit
        X_polyfit = np.array(X_poly)
        Y_polyfit =np.array(Y_poly)
        f1 = np.polyfit(X_polyfit,Y_polyfit,2)
        
        #Ergebnis schätzen mit der Funktion
        pmenge = np.polyval(f1, float(tole.get()))  
        ergeb_q.delete(0,'end')
        ergeb_q.insert(0,'%d' %pmenge)
        
    #Matplotlib Plot in Tkinter Interface einbetten bzw. Funktion zum Button "Ergebnis plotten"
    def einbetten_alternativ():
        #Figure initialisieren
        f = Figure(figsize=(5,4), dpi=150)
        a = f.add_subplot()
        
        #Daten erfassen, bei der Berechnung X:Verschleiß, Y:Produktionsmenge, da Reststandmenge gesucht wird
        X_poly=[] 
        Y_poly=[] 
        for i in range(anzahl):
            Y_poly.append(int((names['pro' + str(i+1)]).get()))
            X_poly.append(float((names['ver' + str(i+1)]).get()))
            
        #Curvefitting mit numpy.polyfit
        X_polyfit = np.array(X_poly)
        Y_polyfit =np.array(Y_poly)
        f1 = np.polyfit(X_polyfit,Y_polyfit,2)
        p1 = np.poly1d(f1)
        
        Y_erg = int(ergeb_q.get())
        X_erg = float(tole.get())
        
        #Plotten und in Tkinter einbetten, bei plotten werden umgetauscht X:Produktionsmenge, Y:Verschleiß
        a.plot(Y_poly,X_poly,'x')
        Yvals = p1(X_poly)
        a.plot(Yvals, X_poly,'r')
        a.plot(Y_erg, X_erg, '*')
        a.set_xlabel('Produktionsmenge in Stück')
        a.set_ylabel('Verschleiß in mm')
        canvas = FigureCanvasTkAgg(f, master=window)
        canvas.draw()
        canvas.get_tk_widget().grid(row=20,column=4, columnspan=3)
    
    #Funktion Button "Reststandmenge" (mit alternativen Methoden) 
    def reststand_curfit():
        
        menge_gesamt = int(ergeb_q.get())
        
        menge_pro = int((names['pro' + str(anzahl)]).get())
        menge_rest = menge_gesamt - menge_pro
        
        r_menge.delete(0, 'end')
        r_menge.insert(0,'%d' %menge_rest) 
        
    #Button zum Ergebnis berechnen initialisieren und platzieren
    button1 = tk.Button(window, text='Erwartete Produktionsmenge',command=quadratisch)
    button1.grid(row=anzahl+3,column=4)
    button2 = tk.Button(window, text='Reststandmenge',command=reststand_curfit)
    button2.grid(row=anzahl+4,column=4)
    button_plot = tk.Button(window, text='Ergebnis plotten',command=einbetten_alternativ)
    button_plot.grid(row=anzahl+5, column=4)

#Matplotlib Plot in Tkinter Interface einbetten bzw. Funktion zum Button "Ergebnis plotten"
def einbetten():
    #Figure initialisieren
    f = Figure(figsize=(5,4), dpi=150)
    a = f.add_subplot()
    
    #Daten erfassen X:Produktionsmenge, Y:Verschleiß
    X = [int(pro1.get()), int(pro2.get()), int(pro3.get())]
    Y = [float(ver1.get()), float(ver2.get()), float(ver3.get())]
    X_p = [int(p_menge.get())]
    Y_p = [float(tole.get())]
    
    #Plotten und in Tkinter einbetten
    a.scatter(X,Y,color='blue')
    a.scatter(X_p,Y_p,color='red')
    a.set_xlabel('Produktionsmenge in Stück')
    a.set_ylabel('Verschleiß in mm')
    canvas = FigureCanvasTkAgg(f, master=window)
    canvas.draw()
    canvas.get_tk_widget().grid(row=20,column=0, columnspan=3)
    
#Menu im Hauptfenster generieren
menubar = tk.Menu(window)
filemenu = tk.Menu(menubar)
vorv_menu = tk.Menu(menubar)
outl_menu = tk.Menu(menubar)
down_menu = tk.Menu(menubar)
regi_menu = tk.Menu(menubar)
regi_po2po_menu = tk.Menu(menubar)
regi_po2pl_menu = tk.Menu(menubar)
seg_manu_menu = tk.Menu(menubar)
seg_auto_menu = tk.Menu(menubar)
verschleiss_menu = tk.Menu(menubar)
alternativ_menu = tk.Menu(menubar)

#Menu zu Hauptfenster hinzufügen
menubar.add_cascade(label='Datei', menu=filemenu)
menubar.add_cascade(label='Vorverarbeitung', menu=vorv_menu)
vorv_menu.add_cascade(label='Rauschenunterdrückung',font=("Arial",16), menu=outl_menu)
vorv_menu.add_cascade(label='Reduktion der Datenmenge',font=("Arial",16), menu=down_menu)
menubar.add_cascade(label='Registrierung', menu=regi_menu)
menubar.add_cascade(label='Manuelle Segmentierung',menu=seg_manu_menu)
menubar.add_cascade(label='Automatische Segmentierung',menu=seg_auto_menu)
menubar.add_cascade(label='Ermittlung des Verschleißes',menu=verschleiss_menu)
menubar.add_cascade(label='Reststandmenge schätzen',menu=alternativ_menu)

#Menu mit den entsprechenden Funktionen versehen
filemenu.add_command(label='Mesh-Datei visualisieren',font=("Arial",16), command=open_mesh)
filemenu.add_command(label='Punktwolke-Datei visualisieren',font=("Arial",16), command=open_pc)
filemenu.add_command(label='Mesh in Punktwolke konvertieren und speichern',font=("Arial",16), command=mesh2pc)
outl_menu.add_command(label='Mit Radius Outlier Removal',font=("Arial",16), command=radius_out)
outl_menu.add_command(label='Mit Statistical Outlier Removal',font=("Arial",16), command=sta_out)
down_menu.add_command(label='Mit Voxel Grid Downsampling',font=("Arial",16), command=voxel_down)
regi_menu.add_command(label='Registrierung mit ICP Algorithmus Point-to-Point',font=("Arial",16),command=registration_icp_po2po)
regi_menu.add_command(label='Registrierung mit ICP Algorithmus Point-to-Plane',font=("Arial",16),command=registration_icp_po2pl)
seg_manu_menu.add_command(label='Bereich in Mesh-Datei auswählen',font=("Arial",16),command=segmentation_manu_crop)
seg_manu_menu.add_command(label='Punktwolke mit Muster segmentieren',font=("Arial",16), command=segmentation_manu_save)
seg_auto_menu.add_command(label='Segmentierung mit RANSAC-Algorithmus',font=("Arial",16), command=segmentation_auto_ran)
seg_auto_menu.add_command(label='Segmentierung mit DBSCAN-Algorithmus', font=("Arial",16),command=segmentation_auto_dbscan)
seg_auto_menu.add_command(label='Segmentierung mit Bauteilabmessung', font=("Arial",16),command=segmentation_auto_mass)
verschleiss_menu.add_command(label='Verschleiß berechnen',font=("Arial",16),command=compute_distance)
alternativ_menu.add_command(label='Mit Ausgleichsrechnung (Curve Fitting)',font=("Arial",16),command=alternativ)

#Funktion "Produktionsmenge berechnen mit Peripheriewinkelsatz" direkt in Hauptfenster initialisieren
#Label erzeugen
titel = tk.Label(window, text= 'Reststandmenge berechnen mit quadratischer Funktion ',font=('Arial',12,'bold'))
titel1 = tk.Label(window,text='Produktionsmenge in Stück')
titel2 = tk.Label(window,text='Verschleiß in mm')
num1 = tk.Label(window,text='Stichprobe 1')
num2 = tk.Label(window,text='Stichprobe 2')
num3 = tk.Label(window,text='Stichprobe 3')
tol = tk.Label(window,text='Verschleißtoleranz in mm')
    
#Eingabenfelde erzeugen
pro1 = tk.Entry(window)
ver1 = tk.Entry(window)
pro2 = tk.Entry(window)
ver2 = tk.Entry(window)
pro3 = tk.Entry(window)
ver3 = tk.Entry(window)
tole = tk.Entry(window)
p_menge = tk.Entry(window)
r_menge = tk.Entry(window)
    
#Label und Eingabenfelder plazieren
titel.grid(row=0,column=1)
titel1.grid(row=1,column=1)
titel2.grid(row=1,column=2)
num1.grid(row=2,column=0)
num2.grid(row=3,column=0)
num3.grid(row=4,column=0)
tol.grid(row=5,column=1)
pro1.grid(row=2,column=1)
ver1.grid(row=2,column=2)
pro2.grid(row=3,column=1)
ver2.grid(row=3,column=2)
pro3.grid(row=4,column=1)
ver3.grid(row=4,column=2)
tole.grid(row=5,column=2)
p_menge.grid(row=6,column=1)
r_menge.grid(row=7,column=1)

#Button zum Ergebnis berechnen, initialisieren und platzieren
button = tk.Button(window, text='Erwartete Produktionsmenge',command=produktionsmenge_peri)
button.grid(row=6,column=0)
button_re = tk.Button(window, text='Reststandmenge',command=reststand_peri)
button_re.grid(row=7,column=0)
button_plot = tk.Button(window, text='Ergebnis plotten',command=einbetten)
button_plot.grid(row=8, column=0)

window.config(menu=menubar)
window.mainloop()