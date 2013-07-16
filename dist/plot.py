#!/usr/bin/python

import matplotlib.pyplot as plt
import roslib
import rospy
from std_srvs.srv import *
import os,sys
import time

mcd_array=[0.01*i for i in range(1,5)]
mcd_array += [0.04 + 0.002*i for i in range(1,10)]
mcd_array += [0.01*i for i in range(6,21)]
print mcd_array

#mcd_array.append(0.2)
set_param_call = rospy.ServiceProxy('/coverage_pc/set_parameters', Empty)
if len(sys.argv)>1:
    start = int(sys.argv[1])
else:
    start = 0
for tn in range(start,len(mcd_array)):
    max_correspondence_distance = mcd_array[tn]
    rospy.set_param("/coverage_pc/max_correspondence_distance", max_correspondence_distance)
    rospy.set_param("/coverage_pc/test_number", tn)
    set_param_call()
    print "Test case :",tn,"MCD =",max_correspondence_distance
    time.sleep(1)
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
        got = False
        while not got:
            try:
                x = open(str(sample)+"/"+str(i)+"_data.txt","r")
                fin.append(x)
                got = True
            except(IOError):
                print "IOError:",str(sample)+"/"+str(i)+"_data.txt"
                time.sleep(1)
        data.append(map(int,fin[i].read().split()))
        #Now drawing a histogram of the data
        #print len(data[i])," "
        if data[i]==[]:
            plt.hist([0],bins=1)
        else:
            plt.hist(data[i],bins=max([1,max(data[i])-min(data[i])]))
        #plt.title("Histogram of NN :"+titles[i])
        plt.title("HNN :"+titles[i])
        plt.xlabel("Value")
        plt.ylabel("Frequency")
        #print data[i]
        fin[i].close()
        if not data[i]==[]:
            mean = 1.0*sum(data[i])/len(data[i])
            sq_diff = 0
            for x in data[i]:
                sq_diff += (x-mean)**2
            var = sq_diff/len(data[i])
            #print titles[i], ":Mean", mean , ":Std dev", var**0.5
        else:
            mean = 0
            var = 0
        data_out.write(str(mean)+" "+str(var**0.5)+"\n")
    plt.suptitle("With MCD :"+str(max_correspondence_distance))
    mypath = "./hist/"
    if not os.path.isdir(mypath):
           os.makedirs(mypath)
    figure = plt.gcf() # get current figure
    figure.set_size_inches(16, 8)
    plt.savefig("hist/"+str(tn)+"_plot.png", bbox_inches=0, dpi=100)
    plt.clf()
    data_out.close()
print "Ending program"

plt.clf()

