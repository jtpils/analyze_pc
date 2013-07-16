#!/usr/bin/python
import roslib
import rospy
from std_srvs.srv import *
import os,sys
import time

set_param_call = rospy.ServiceProxy('/coverage_pc/set_parameters', Empty)
mnfa = [0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9] #min NN factor array
nna = [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,24,26,28,30] #NN Aray
print len(mnfa), len(nna)
count = 0
print "Testing",len(mnfa)*len(nna),"cases"
for T in nna:
    rospy.set_param("/coverage_pc/min_nn", T)
    for n in mnfa:
        print "Test case :",count
        rospy.set_param("/coverage_pc/min_nn_factor", n)
        set_param_call() ##will call the set param thing in the main code and print stuff
        count+=1




