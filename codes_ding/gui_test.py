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

window.title('Prognose der Reststandmenge mittels 3D-Bildverarbeitung')
# Setze die Fenstergröße 
window.geometry("1800x1000")
#window.configure(background="white")
# Setze die Dateipfad an 
model_path = None #source
mesh_path =None #target
source_path = None
registed_source = None
registed_target = None
finetuned_source = None
finetuned_target = None
finetuned_registed_both = None
anzahl_seg = 1
anzahl_prob = 0
n = None
produktionsmenge = 0
all_toleranz = 0
pc = []
ex = []
toleranz = []

#verschleisswert_max = []
Meta = []
def graph_zeichnen():
    X = []
    Y = []
    anzahl_seg=3
    diag = Figure(figsize=(5,4), dpi=150,constrained_layout=True)
    for k in range(anzahl_seg):
        print('draw Seg_%s'%(k+1))
        names = locals()
        names['Seg_' + str(k+1)] = diag.add_subplot(anzahl_seg,1,k+1)
        
        X_array = [1000,2000,3000,4000]
        Y_array = [0.3,0.4,0.6,0.8]  
       
        f1 = np.polyfit(Y_array,X_array, 2)
        p = np.poly1d(f1)
        t = np.linspace(0, 1,250)
        names['Seg_' + str(k+1)].scatter(t[-1],p(t)[-1],color='red')
        #names['Seg_' + str(k+1)].plot([0,t[-1]],[p(t)[-1],p(t)[-1]],c='b',linestyle='--')#Senkrecht
        #names['Seg_' + str(k+1)].plot([t[-1],t[-1]],[0,p(t)[-1]],c='b',linestyle='--')#Paralle
        names['Seg_' + str(k+1)].text(t[-1]-0.02,p(t)[-1],'Prognose\n(%d)'%(p(t)[-1]))
        #names['Seg_' + str(k+1)].plot(Y_array,X_array,'o',p(t),t,"-")#,p(t)[-1],t[-1],'x')
        names['Seg_' + str(k+1)].plot(Y_array,X_array,c='b',linestyle='--')#Paralle
        names['Seg_' + str(k+1)].set_xlabel('Verschleiß')
        names['Seg_' + str(k+1)].set_ylabel('pro-menge')   
        names['Seg_' + str(k+1)].set_title('Seg_%d'%(k+1))
        
        X = []
        Y = []
    canvas = FigureCanvasTkAgg(diag, master=window)
    canvas.draw()
    canvas.get_tk_widget().grid(row=9,column=5, columnspan=3)

button1 = tk.Button(window, text="test", command=graph_zeichnen,font=("Arial",12))
button1.grid(row=1,column=2)
window.mainloop()
