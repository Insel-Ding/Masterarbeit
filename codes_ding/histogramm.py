import open3d as o3d
import numpy as np
import os
import ICP_utilities as uti
import matplotlib.pyplot as plt
import numpy as np
import scipy.optimize as opt;


##__NR.1__ #################################

#x : Probe
x1 =np.array( [500,1000, 1400, 1700, 1900, 2100,2200])
#y : Verschleiß
y1 = np.array( [0.05,0.1, 0.15, 0.2, 0.25,0.3,0.35])

##__NR.2__ #################################

#x : Probe
x2 = np.array( [500,1000, 1400, 1700, 1900, 2100,2200])
#y : Verschleiß
y2 = np.array( [0.05,0.06, 0.07, 0.09, 0.1,0.12,0.14])

##__NR.3__ #################################

#x : Probe
x3 = [500,1000, 1400, 1700, 1900, 2100,2200]
#y : Verschleiß
y3 = [0.05,0.05, 0.05, 0.055, 0.06,0.065,0.07]

##__NR.4__ #################################

#x : Probe
x4 = [500,1000, 1400, 1700, 1900, 2100,2200]
#y : Verschleiß
y4 = [0.05,0.05, 0.05, 0.055, 0.055,0.06,0.06]

def func(x, a, b, c):
     return a * np.exp(-b * x) + c

plt.figure(1)


optimizedParameters1, pcov1 = opt.curve_fit(func, x1, y1)
a, b, c = optimizedParameters1

l1,=plt.plot(x1,y1)

l2,=plt.plot(x2,y2)
l3,=plt.plot(x3,y3)
l4,=plt.plot(x4,y4)
plt.legend(handles=[l1, l2,l3,l4],labels=['Pos_Nr_1','Pos_Nr_2','Pos_Nr_3','Pos_Nr_4'])


plt.xlabel('Probestück')
plt.ylabel('Verschleißwert')

plt.show()