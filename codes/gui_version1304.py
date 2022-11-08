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
window.geometry('1800x1200+50+50')
window.minsize(1700,1000)

#Funktion Menu "Datei -- Mesh Datei visualisieren"
def open_mesh():
    tk.messagebox.showinfo(message="Wählen Sie eine Mesh Datei...")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.paint_uniform_color([1,0.5,0])
    o3d.visualization.draw_geometries([mesh],mesh_show_wireframe = True, 
                                      mesh_show_back_face = False)

#Funktion Menu "Datei -- Punktwolke Datei visualisieren"
def open_pc():
    tk.messagebox.showinfo(message="Wählen Sie eine Punktwolke Datei...")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply")])
    pcd = o3d.io.read_point_cloud(pcd_path)
    pcd.paint_uniform_color([0,0,1])
    o3d.visualization.draw_geometries([pcd])
    
#Funktion Menu "Datei -- Mesh in Punktwolke umwandeln und speichern"
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
        
#Funktion Menu "Punktwolke registrieren"   
def registration():
    #Datei laden
    tk.messagebox.showinfo(message="Mesh-Datei öffnen")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    tk.messagebox.showinfo(message="Punktwolke-Datei öffnen")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    source = o3d.io.read_point_cloud(pcd_path)
    target = o3d.geometry.PointCloud(mesh.vertices)
    
    #ICP Registration
    trans_init = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]     
    threshold = 40  #Max. Abstand zwischen Mesh und Punktwolken
    reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())  
    source.transform(reg_p2p.transformation)

    #Ergebnisse Visualisieren
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Visualizer', width=1600, height=1200)
    source.paint_uniform_color([1, 0, 0])  # rot
    target.paint_uniform_color([0, 1, 0])  # grün
    vis.add_geometry(source)
    vis.add_geometry(target)
    vis.run()
    vis.destroy_window()
    
    #Datei Speichern
    s = tk.messagebox.askyesno(message="Registriete Punktwolke speichern?")
    if s == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, source)

#Funktion Menu "Punktwolke segmentieren manuell -- Bereich auswählen"
def segmentation_manu_crop():
    #Original Mesh Datei laden
    tk.messagebox.showinfo(message="Original Mesh Datei(*.stl) öffnen")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    pc = o3d.geometry.PointCloud(mesh.vertices)
    
    #Bereich manuell auswählen und .json Datei generieren
    tk.messagebox.showinfo(message="Bereich manuell auswählen:\n\
    1.Ansicht auswählen mit Strl+X, Strl+Y oder Strl+Z \n\
    2.Ansicht fixieren mit Strl+K \n\
    3.Bereich manuell auswählen mit gedrückte Strl-Taste\n\
    4.Koordinaten des Bereichs speichern als .json Datei")
    o3d.visualization.draw_geometries_with_editing([pc])

#Funktion Menu "Punktwolke segmentieren manuell -- Ziel Datei segmentieren"
def segmentation_manu_save():
    #Ziel Datei laden    
    tk.messagebox.showinfo(message="Punktwolke-Datei öffnen")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply")])
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
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, auswahl)

#Funktion Menu "Punktwolke segmentieren automatisch -- mit RANSAC"
def segmentation_auto_ran():
    #Original Mesh Datei laden
    tk.messagebox.showinfo(message="Original Mesh Datei(*.stl) öffnen")
    mesh_path = filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    pc = o3d.geometry.PointCloud(mesh.vertices)
    
    #Eingabe Parametern
    dis = tk.simpledialog.askfloat("Segmentieren mit RANSAC Algorithmus",
                               "Abstand zwischen Inlier und Outliner eingeben:")
    ransac_n = 3            #Random Punkte am Anfang  
    num_iterations = 1000   #Anzahl Iteration
    
    #Segmentieren mit RANSAC
    plane_model, inliers = pc.segment_plane(dis, ransac_n, num_iterations)
    inlier_cloud = pc.select_by_index(inliers)
    outlier_cloud = pc.select_by_index(inliers, invert=True)
    
    
    #Visualisieren
    inlier_cloud.paint_uniform_color([0, 0, 1]) #Inlier in blau
    outlier_cloud.paint_uniform_color([1, 0, 0]) #Outliner in rot
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    
    #Segmentierte Punktwolke speichern
    v = tk.messagebox.askyesno(message="Punktwolke in blau speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, inlier_cloud)
    
#Funktion Menu "Punktwolke segmentieren automatisch -- mit DBScan"
def segmentation_auto_dbscan():
    #Lade Punktwolke Datei
    tk.messagebox.showinfo(message="Punktwolke-Datei öffnen")
    pcd_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply")])
    pcd = o3d.io.read_point_cloud(pcd_path)

    #Eingabe Parametern
    eps = tk.simpledialog.askfloat("Segmentieren mit DBScan Algorithmus",
                               "Maximale Abstand zwischen Punkten in der selben Gruppe:")
    min_points = tk.simpledialog.askinteger("Segmentieren mit DBScan Algorithmus", 
                                            "Kleineste Anzahl Punkte in einer Gruppe")
    #Segmentieren mit DBScan
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps, min_points, print_progress=True))
    max_label = labels.max() 
    
    #Visualisieren
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0  
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd])
    
    #Ergebnisse Speichern
    v = tk.messagebox.askyesno(message="Punktwolke in %d Gruppen geteilt, Ergebnisse speichern?"
                               % (max_label+1))
    if v == True:
        for i in range(max_label+1):
            ind = np.where(labels == i)[0]
            clusters_cloud = pcd.select_by_index(ind)
            file_name = "dbscan_cluster" + str(i+1) + ".ply"
            o3d.io.write_point_cloud(file_name, clusters_cloud)

#Funktion Menu "Punktwolke segmentieren automatisch -- mit Maßeingaben"
def segmentation_auto_mass():
    #Lade Vergleichsebene
    tk.messagebox.showinfo(message="Vergleichsebene Datei öffnen wie z.B. Boden")
    basis_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply")])
    basis = o3d.io.read_point_cloud(basis_path)
    
    #Lade Punktwolke
    tk.messagebox.showinfo(message="Punktwolke zu segmentieren öffnen")
    pc_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply")])
    pc = o3d.io.read_point_cloud(pc_path)
    
    #Ober-/Untergrenze eingeben
    obergrenze = tk.simpledialog.askfloat("Segmentieren mit Maßeingaben",
                                        "Abstand der Obergrenze zur Vergleichsebene in mm:")
    untergrenze = tk.simpledialog.askfloat("Segmentieren mit Maßeingaben",
                                        "Abstand der Untergrenze zur Vergleichsebene in mm:")
    
    dists = pc.compute_point_cloud_distance(basis)
    dists = np.asarray(dists) 
    
    #+/-0.5mm als Toleranz sowie erwartete Verschleiß 
    unter = np.where(dists < (obergrenze + 0.5))[0] 
    unterteil = pc.select_by_index(unter)
    dists_neu = unterteil.compute_point_cloud_distance(basis)
    dists_neu = np.asarray(dists_neu)
    oben = np.where(dists_neu > (untergrenze - 0.5))[0]
    auswahl = unterteil.select_by_index(oben)
    
    #Visualisieren und Speichern
    auswahl.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([auswahl]) 
    v = tk.messagebox.askyesno(message="Ergebnisse speichern?")
    if v == True:
        save_path = filedialog.asksaveasfilename(filetypes=[("ply Datei","*.ply")],
                                                 defaultextension = ".ply")
        o3d.io.write_point_cloud(save_path, auswahl) 

#Funktion Menu "Produktionsmenge prognostizieren -- Verschleiß berechnen"
def compute_distance():
    #Nummer der Stichprobe erfragen
    num_s = tk.simpledialog.askinteger("Nummer der Stichprobe eingeben",
                                       "Geben Sie die Nummer der Stichprobe ein(1,2 oder 3):")
    #Lade Original Datei
    tk.messagebox.showinfo(message="Basis Punktwolke-Datei öffnen")
    ori_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply")])
    ori = o3d.io.read_point_cloud(ori_path)
    
    #Lade Verschleiss Datei
    tk.messagebox.showinfo(message="Verschleiss Punktwolke-Datei öffnen")
    vers_path = filedialog.askopenfilename(filetypes=[("ply Datei","*.ply")])
    vers = o3d.io.read_point_cloud(vers_path)
    
    #Abstand berechnen
    distance = vers.compute_point_cloud_distance(ori)
    dists = np.asarray(distance)
    abstand_mean = np.mean(dists) #Abstand in Durchschnitt
    abstand_median = np.median(dists) #Abstand im Median
    v = tk.messagebox.askyesno(message="Verschleiss beträgt im Durchschnitt %.3f mm, im Median %.3f mm \
    Ergebnis direkt in der Tabelle eintragen?" %(abstand_mean, abstand_median))
    
    #Eintragen des Ergebnis im Eingabenfeld
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
        
#Funktion Button "Erwartete Produktionsmenge berechnen (mit Peripheriewinkelsatz)"
def produktionsmenge_peri():

    #X - Produktionsmenge, Y - Verschleiß, t - Verschleißtoleranz
    X = [int(pro1.get()), int(pro2.get()), int(pro3.get())]
    Y = [float(ver1.get()), float(ver2.get()), float(ver3.get())]
    t = float(tole.get())
    
    x = Symbol('x')
    solved_value=solve([(t -Y[0])/(x-X[0]) - (t-Y[1])/(x-X[1]) 
                    - (Y[2]-Y[0])/(X[2]-X[0]) + (Y[2]-Y[1])/(X[2]-X[1])] , [x])
    
    ergeb = int(solved_value[1][0]) #Nur das positive Ergebnis 
    p_menge.delete(0, 'end')
    p_menge.insert(0,'%d' %ergeb)

#Funktion Button "Reststandmenge" (mit Peripheriewinkelsatz) 
def reststand_peri():
    
    menge_gesamt = int(p_menge.get())
    menge_pro = int(pro3.get())
    menge_rest = menge_gesamt - menge_pro
    
    r_menge.delete(0, 'end')
    r_menge.insert(0,'%d' %menge_rest)
    
#Funktion Menu "Produktionsmenge berechnen mit alternativen Modellen"
def alternativ():

    #Anzahl der Exemplare eingeben
    anzahl = tk.simpledialog.askinteger("Anzahl der Stichproben",
                               "Anzahl der Stichproben:")
    
    #Label erzeugen und platzieren
    titel = tk.Label(window,text="Produktionsmenge berechnen mit Curve Fitting",font=('Arial',12,'bold'))
    titel1 = tk.Label(window,text='Produktionsmenge in Stück')
    titel2 = tk.Label(window,text='Verschleiß in mm')
    titel.grid(row=0,column=4)
    titel1.grid(row=1,column=4)
    titel2.grid(row=1,column=5)
    for i in range(anzahl):
        (tk.Label(window,text='Stichprobe %d'%(i+1)).grid(row=i+2,column=3))
    tol = tk.Label(window,text='Verschleißtoleranz in mm')
    tol.grid(row=anzahl+2,column=4)
    
    #Eingabenfelde erzeugen und platzieren
    names = locals()
    for i in range(anzahl):
        names['pro' + str(i+1)] = tk.Entry(window)
        (names['pro' + str(i+1)]).grid(row=i+2,column=4)
        names['ver' + str(i+1)] = tk.Entry(window)
        (names['ver' + str(i+1)]).grid(row=i+2,column=5)
    tole = tk.Entry(window)
    tole.grid(row=anzahl+2,column=5)
    ergeb_q = tk.Entry(window)
    ergeb_q.grid(row=anzahl+3,column=4)
    #ergeb_e = tk.Entry(window)
    #ergeb_e.grid(row=anzahl+4,column=4)
    
    #Mit Modell Quadratische Funktion die Produktionsmenge prognostizieren
    def quadratisch():
        X_poly=[] # X:Produktionsmenge
        Y_poly=[] # Y:Verschleiß
        for i in range(anzahl):
            X_poly.append(int((names['pro' + str(i+1)]).get()))
            Y_poly.append(float((names['ver' + str(i+1)]).get()))
        
        #Curve Fitting mit numpy.polyfit
        X_polyfit = np.array(X_poly)
        Y_polyfit =np.array(Y_poly)
        f1 = np.polyfit(X_polyfit,Y_polyfit,2)
        p1 = np.poly1d(f1)
        x = Symbol('x')
        menge = solve([f1[0]*x**2 + f1[1]*x + f1[2] - float(tole.get())],[x])
        quadra = int(menge[1][0])
        ergeb_q.delete(0,'end')
        ergeb_q.insert(0,'%d' %quadra)
    
    #Mit Modell Kubische Funktion die Produktionsmenge prognostizieren
    def expo():
        X_poly=[] # X:Produktionsmenge
        Y_poly=[] # Y:Verschleiß
        for i in range(anzahl):
            X_poly.append(int((names['pro' + str(i+1)]).get()))
            Y_poly.append(float((names['ver' + str(i+1)]).get()))
        
        #Curve Fitting mit Exponentialfunktion
        X_polyfit = np.array(X_poly)
        Y_polyfit =np.array(Y_poly)
        f2 = np.polyfit(X_polyfit,np.log(Y_polyfit),1)
        
    #Matplotlib Plot in Tkinter Interface einbetten bzw. Funktion zum Button "Ergebnis plotten"
    def einbetten_alternativ():
        #Figure initialisieren
        f = Figure(figsize=(6,5), dpi=150)
        a = f.add_subplot()
        
        #Daten erfassen
        X_poly=[] # X:Produktionsmenge
        Y_poly=[] # Y:Verschleiß
        for i in range(anzahl):
            X_poly.append(int((names['pro' + str(i+1)]).get()))
            Y_poly.append(float((names['ver' + str(i+1)]).get()))
            
        #Curvefitting mit numpy.polyfit
        X_polyfit = np.array(X_poly)
        Y_polyfit =np.array(Y_poly)
        f1 = np.polyfit(X_polyfit,Y_polyfit,2)
        p1 = np.poly1d(f1)
        
        X_erg = int(ergeb_q.get())
        Y_erg = float(tole.get())
        
        #Plotten und in Tkinter einbetten
        a.plot(X_poly,Y_poly,'x')
        Yvals = p1(X_poly)
        a.plot(X_poly,Yvals,'r')
        a.plot(X_erg, Y_erg, '*')
        a.set_xlabel('Produktionsmenge in Stück')
        a.set_ylabel('Verschleiß in mm')
        canvas = FigureCanvasTkAgg(f, master=window)
        canvas.draw()
        canvas.get_tk_widget().grid(row=anzahl+6,column=3, columnspan=3)
        
    #Button zum Ergebnis berechnen initialisieren und platzieren
    button1 = tk.Button(window, text='Modell Quadratische Funktion',command=quadratisch)
    button1.grid(row=anzahl+3,column=3)
    #button2 = tk.Button(window, text='Modell Exponentialfunktion',command=expo)
    #button2.grid(row=anzahl+4,column=3)
    button_plot = tk.Button(window, text='Ergebnis plotten',command=einbetten_alternativ)
    button_plot.grid(row=anzahl+5, column=3)

#Matplotlib Plot in Tkinter Interface einbetten bzw. Funktion zum Button "Ergebnis plotten"
def einbetten():
    #Figure initialisieren
    f = Figure(figsize=(6,5), dpi=150)
    a = f.add_subplot()
    
    #Daten erfassen
    X = [int(pro1.get()), int(pro2.get()), int(pro3.get()), int(p_menge.get())]
    Y = [float(ver1.get()), float(ver2.get()), float(ver3.get()), float(tole.get())]

    #Plotten und in Tkinter einbetten
    a.scatter(X,Y,color='red')
    a.set_xlabel('Produktionsmenge in Stück')
    a.set_ylabel('Verschleiß in mm')
    canvas = FigureCanvasTkAgg(f, master=window)
    canvas.draw()
    canvas.get_tk_widget().grid(row=8,column=0, columnspan=3)
    
#Menu in Hauptfenster generieren
menubar = tk.Menu(window)
filemenu = tk.Menu(menubar)
icp_menu = tk.Menu(menubar)
seg_manu_menu = tk.Menu(menubar)
seg_auto_menu = tk.Menu(menubar)
verschleiss_menu = tk.Menu(menubar)
alternativ_menu = tk.Menu(menubar)

#Menu zum Hauptfenster hinzufügen
menubar.add_cascade(label='Datei', menu=filemenu)
menubar.add_cascade(label='Punktwolke registrieren', menu=icp_menu)
menubar.add_cascade(label='Punktwolke segmentieren manuell',menu=seg_manu_menu)
menubar.add_cascade(label='Punktwolke segmentieren automatisch',menu=seg_auto_menu)
menubar.add_cascade(label='Verschleiß berechnen',menu=verschleiss_menu)
menubar.add_cascade(label='Produktionsmenge berechnen mit alternativen Methoden',menu=alternativ_menu)

#Menu mit den entsprechenden Funktionen versehen
filemenu.add_command(label='Mesh-Datei visualisieren',font=("Arial",16), command=open_mesh)
filemenu.add_command(label='Punktwolke-Datei visualisieren',font=("Arial",16), command=open_pc)
filemenu.add_command(label='Mesh in Punktwolke umwandeln und speichern',font=("Arial",16), command=mesh2pc)
icp_menu.add_command(label='Punktwolke registrieren und speichern',font=("Arial",16),command=registration)
seg_manu_menu.add_command(label='Bereich in Mesh Datei auswählen',font=("Arial",16),command=segmentation_manu_crop)
seg_manu_menu.add_command(label='Ziel Datei segmentieren',font=("Arial",16), command=segmentation_manu_save)
seg_auto_menu.add_command(label='Segmentieren mit RANSAC Algorithmus',font=("Arial",16), command=segmentation_auto_ran)
seg_auto_menu.add_command(label='Segmentieren mit DBScan Algorithmus', font=("Arial",16),command=segmentation_auto_dbscan)
seg_auto_menu.add_command(label='Segmentieren mit Maßeingaben', font=("Arial",16),command=segmentation_auto_mass)
verschleiss_menu.add_command(label='Verschleiß berechnen',font=("Arial",16),command=compute_distance)
alternativ_menu.add_command(label='Mit Curve Fitting',font=("Arial",16),command=alternativ)

#Funktion "Produktionsmenge berechnen mit Peripheriewinkelsatz" direkt in Hauptfenster initialisieren
#Label erzeugen
titel = tk.Label(window, text= 'Produktionsmenge berechnen mit Peripheriesatz',font=('Arial',12,'bold'))
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
    
#Label und Eingabenfelder in Toplevel-Fenster plazieren
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

#Button zum Ergebnis berechnen initialisieren und platzieren
button = tk.Button(window, text='Erwartete Produktionsmenge',command=produktionsmenge_peri)
button.grid(row=6,column=0)
button_re = tk.Button(window, text='Reststandmenge',command=reststand_peri)
button_re.grid(row=7,column=0)
button_plot = tk.Button(window, text='Ergebnis plotten',command=einbetten)
button_plot.grid(row=8, column=0)

window.config(menu=menubar)
window.mainloop()