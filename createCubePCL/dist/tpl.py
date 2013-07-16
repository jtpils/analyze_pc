#!/usr/bin/python
import matplotlib.pyplot as plt

mcd_array=[]
graphs=[]
#Graphs=[B,G,Q]
# B = [means, stdvs]

for i in range(6):
    graphs.append([])
data = open("fraction_data.txt","r")
while True:
    try:
        x = data.readline()
        if x=="":
            break
        mcd_array.append(float(x))
        x = map(float, data.readline().split())
        for i in range(len(x)):
            graphs[i].append(x[i])
    except(IOError):
        print "What?"
for x in  graphs:
    print x
gn=[0,3,1,4,2,5]
count = 0
for x in graphs:
    plt.subplot(2,3,gn[count]+1)
    plt.plot(mcd_array, x)
    plt.title("Plot: "+str(count))
    count+=1
plt.show()
