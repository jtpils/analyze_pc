#!/usr/bin/python

import matplotlib.pyplot as plt
import roslib
import rospy
max_correspondence_distance = 0.01
max_correspondence_distance = rospy.get_param("/coverage_pc/max_correspondence_distance")
sample=4
fin=[]
data = []
titles=["both","gt","qd"]
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 26}
plt.rc('font', **font)
data_out = open("data.txt","a")
data_out.write(str(max_correspondence_distance)+"\n")
for i in range(3):
    plt.subplot(1,3,i+1)
    fin.append(open(str(sample)+"/"+str(i)+"_data.txt","r"))
    data.append(map(int,fin[i].read().split()))
    #Now drawing a histogram of the data
    plt.hist(data[i],bins=max(data[i])-min(data[i]))
    plt.title("Histogram of NN :"+titles[i])
    plt.xlabel("Value")
    plt.ylabel("Frequency")
    #print data[i]
    fin[i].close()
    mean = 1.0*sum(data[i])/len(data[i])
    sq_diff = 0
    for x in data[i]:
        sq_diff += (x-mean)**2
    var = sq_diff/len(data[i])
    print titles[i], ":Mean", mean , ":Std dev", var**0.5
    data_out.write(str(mean)+" "+str(var**0.5)+"\n")
plt.show()
data_out.close()
print "Ending program"
