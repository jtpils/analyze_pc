#!/usr/bin/python
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

occl_frac = 0.445188
result = [1-occl_frac, occl_frac, 0]

#Reading res_data.txt
mnfa = [0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9] #min NN factor array
nna = [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,24,26,28,30] #NN Aray

fraction_data=[[[0.0 for i in range(len(mnfa))] for j in range(len(nna))] for k in range(3)]

df = open("res_data.txt", "r")
while(True):
    try:
        x = df.readline()
        y = map(float,x.split())
        if y==[]:
            break
        mnfa_index = mnfa.index(y[0])
        nna_index = nna.index(y[1])
        df.readline() #Get no. of points in each
        df.readline() #Get blown up fractions
        frdata = map(float,df.readline().split()) #get actual fractions
        assert(len(frdata) == 3)
        for i in range(3):
            fraction_data[i][nna_index][mnfa_index] = frdata[i]
    except(IOError):
        print "What?"
        break
print "Read all data from file, plotting stuff..."

fig = plt.figure()

for i in range(3):
    ax = fig.add_subplot(1,3,i, projection='3d')
    X = mnfa
    xlen = len(X)
    Y = nna
    ylen = len(Y)
    X, Y = np.meshgrid(X, Y)
    Z = fraction_data[i]
    colortuple = ('r', 'b')
    colors = np.empty(X.shape, dtype=str)
    for y in range(ylen):
        for x in range(xlen):
            colors[y, x] = colortuple[(x+y) % len(colortuple)]
    ax.plot_surface(X,Y,Z,rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0)
plt.show()
