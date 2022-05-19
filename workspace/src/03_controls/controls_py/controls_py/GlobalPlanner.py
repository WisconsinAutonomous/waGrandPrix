from cProfile import label
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
from scipy import interpolate
import math
import time

# CONSTANT  


class GlobalPlanner:

    def __init__(self, csv_filename) -> None:
        file = open(csv_filename)
        data = np.loadtxt(file,delimiter=",")
        self.x_ref = data[:,0]
        self.y_ref = data[:,1]

        self.pos_x = None
        self.pos_y = None

    def SearchMinDistance(self):
        ### find minimum distance between current car's position and reference and give the index of that point in the reference trajectory
        dis = np.zeros(len(self.x_ref))
        min_dis = 100000
        index = -1
        for i in range(len(dis)-1):
            dis[i] = (self.pos_x-self.x_ref[i])**2+(self.pos_y-self.y_ref[i])**2
            if dis[i]<min_dis:
                min_dis = dis[i]
                index = i
        return index

    def SearchTargetPoint(self):
        ind = self.SearchMinDistance()
        index = ind - 5 # lookahead = 5 
        # minus sign is due to arrangement of dataset order and the counter-clockwise direction of race track.
        x_target = self.x_ref[index]
        y_target = self.y_ref[index]
        return x_target,y_target,0.0


# x_cur = -86.944173
# y_cur = 40.437549

# x_cur = -86.944085
# y_cur = 40.437999

# [x_target,y_target] = SearchTargetPoint(x_cur,y_cur,lookahead=3)
# plt.plot(x_cur,y_cur,'*b')
# plt.plot(x_target,y_target,'*r')
# plt.plot(x_ref,y_ref)
# plt.show()