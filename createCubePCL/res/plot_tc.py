#!/usr/bin/python
import matplotlib.pyplot as plt

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
