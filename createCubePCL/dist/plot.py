#!/usr/bin/python

import matplotlib.pyplot as plt
import roslib
import rospy
from std_srvs.srv import *
import os

mcd_array=[0.01*i for i in range(1,11)]
mcd_array.append(0.2)
set_param_call = rospy.ServiceProxy('/coverage_pc/set_parameters', Empty)
for tn in range(len(mcd_array)):
    max_correspondence_distance = mcd_array[tn]
    rospy.set_param("/coverage_pc/max_correspondence_distance", max_correspondence_distance)
    rospy.set_param("/coverage_pc/test_number", tn)
    sample=tn
    fin=[]
    data = []
    titles=["both","gt","qd"]
    font = {'family' : 'normal',
            'weight' : 'bold',
            'size'   : 26}
    plt.rc('font', **font)
    data_out = open("data.txt","a")
    data_out.write(str(max_correspondence_distance)+"\n")
    mypath = "./"+str(sample)+"/"
    if not os.path.isdir(mypath):
           os.makedirs(mypath)
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
    plt.suptitle("With MCD :"+str(max_correspondence_distance))
    mypath = "./hist/"
    if not os.path.isdir(mypath):
           os.makedirs(mypath)
    plt.savefig("hist/"+str(tn)+"_plot.png")
    data_out.close()
print "Ending program"
