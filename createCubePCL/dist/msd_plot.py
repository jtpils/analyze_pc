#!/usr/bin/python
import matplotlib.pyplot as plt

mcd_array=[]
graphs=[]
#Graphs=[B,G,Q]
# B = [means, stdvs]

for i in range(3):
    graphs.append([[],[]])
data = open("data.txt","r")
while True:
    try:
        x = data.readline()
        if x=="":
            break
        mcd_array.append(float(x))
        for i in range(3):
            x = map(float, data.readline().split())
            graphs[i][0].append(x[0])
            graphs[i][1].append(x[1])
    except(IOError):
        print "What?"
gn=[0,3,1,4,2,5]
count = 0
for x in graphs:
    for y in x:
        plt.subplot(2,3,gn[count]+1)
        plt.plot(mcd_array, y)
        plt.title("Plot: "+str(count))
        count+=1
plt.show()
