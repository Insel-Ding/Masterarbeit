import tkinter as tk
import open3d as o3d









# Hauptfenster initialisieren

window = tk.Tk() 
window.title('prognose der Reststandmenge mittels 3D-Bildverarbeitung')
window.geometry('2000x1300+50+50')
window.minsize(1600,1000)


#Funktion Menu "Datei -- Mesh Datei visualisieren" : --------3D creo 
def open_mesh():
    tk.messagebox.showinfo(message="Wählen Sie eine Mesh Datei...")
    tk.filedialog.askopenfilename(filetypes=[("stl Datei","*.stl")])
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.paint_uniform_color([1,0.5,0])
    print(mesh)
    o3d.visualization.draw_geometries([mesh], mesh_show_wireframe = True, mesh_show_back_face= False)


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
    source = o3d.io.read_point_cloud(pcd_path) #ply datei
    target = o3d.geometry.PointCloud(mesh.vertices) #stl datei
    
    #Registrierung mit ICP
    trans_init = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]     
    threshold = 40  #Max. Abstand zwischen Mesh und Punktwolken
    reg_po2po = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,o3d.pipelines.registration.TransformationEstimationPointToPoint())  
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
filemenu.add_command(label='Mesh-Datei visualisieren',font=("Arial",12), command=open_mesh)
filemenu.add_command(label='Punktwolke-Datei visualisieren',font=("Arial",12), command=open_pc)
filemenu.add_command(label='Mesh in Punktwolke konvertieren und speichern',font=("Arial",12), command=mesh2pc)
outl_menu.add_command(label='Mit Radius Outlier Removal',font=("Arial",12), command=radius_out)
outl_menu.add_command(label='Mit Statistical Outlier Removal',font=("Arial",12), command=sta_out)
down_menu.add_command(label='Mit Voxel Grid Downsampling',font=("Arial",12), command=voxel_down)
regi_menu.add_command(label='Registrierung mit ICP Algorithmus Point-to-Point',font=("Arial",12),command=registration_icp_po2po)
regi_menu.add_command(label='Registrierung mit ICP Algorithmus Point-to-Plane',font=("Arial",12),command=registration_icp_po2pl)
seg_manu_menu.add_command(label='Bereich in Mesh-Datei auswählen',font=("Arial",12),command=segmentation_manu_crop)
seg_manu_menu.add_command(label='Punktwolke mit Muster segmentieren',font=("Arial",12), command=segmentation_manu_save)
seg_auto_menu.add_command(label='Segmentierung mit RANSAC-Algorithmus',font=("Arial",12), command=segmentation_auto_ran)
seg_auto_menu.add_command(label='Segmentierung mit DBSCAN-Algorithmus', font=("Arial",12),command=segmentation_auto_dbscan)
seg_auto_menu.add_command(label='Segmentierung mit Bauteilabmessung', font=("Arial",12),command=segmentation_auto_mass)
verschleiss_menu.add_command(label='Verschleiß berechnen',font=("Arial",12),command=compute_distance)
alternativ_menu.add_command(label='Mit Ausgleichsrechnung (Curve Fitting)',font=("Arial",12),command=alternativ)

window.config(menu=menubar)
window.mainloop()