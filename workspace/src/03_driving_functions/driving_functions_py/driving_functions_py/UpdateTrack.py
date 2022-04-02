from turtle import xcor
import numpy as np
import matplotlib.pyplot as plt
import time

# ellipsoid function x^2/a^2+y^2/b^2=1
a = 4
b = 5
c = 5
d = 100
x_cor = np.linspace(-4,4,1000)
y1_cor = np.zeros(1000)
y1_cor = a*x_cor**3+b*x_cor**2+c
y2_cor = a*x_cor**3+b*x_cor**2+d

def extractData(x,y1,y2,start_ind,num):
    x_out = x[start_ind:start_ind+num]
    y1_out = y1[start_ind:start_ind+num]
    y2_out = y2[start_ind:start_ind+num]
    return [x_out,y1_out,y2_out]

def centerline(xcor,y1cor,y2cor):
    pre_length = xcor.size
    center_x = np.zeros(pre_length)
    center_y = np.zeros(pre_length)
    for i in np.arange(1,pre_length+1):
        center_y[i-1] = (y1cor[i-1]+y2cor[i-1])/2
    center_x = xcor
    return [center_x,center_y]


pre_horizon = 5
plt.plot(x_cor,y1_cor,'b',x_cor,y2_cor,'b')



start_ind = 0
while start_ind < len(x_cor):
    # Extract next "pre_horizon" many data points ahead
    x_update = extractData(x_cor,y1_cor,y2_cor,start_ind,pre_horizon)[0]
    y1_update = extractData(x_cor,y1_cor,y2_cor,start_ind,pre_horizon)[1]
    y2_update = extractData(x_cor,y1_cor,y2_cor,start_ind,pre_horizon)[2]
    plt.plot(x_update,y1_update,'r',x_update,y2_update,'r')
    # doing simple centerline path planning based on local data at this time step
    xcenter = centerline(x_update,y1_update,y2_update)[0]
    ycenter = centerline(x_update,y1_update,y2_update)[1]
    plt.plot(xcenter,ycenter,'b--')
    start_ind = start_ind + pre_horizon

    plt.pause(0.25)
plt.show()   
