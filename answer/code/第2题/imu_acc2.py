import os
import math
import numpy as np

def xf(ax, ay, az):
    x = [[ax, 0, 0, ay, az, 0, 0, 0, 0, 1, 0, 0],
        [0, ay, 0, 0, 0, ax, az, 0, 0, 0, 1, 0],
        [0, 0, az, 0, 0, 0, 0, ax, ay, 0, 0, 1]]
    return np.mat(x)

def yf(data):
    y = np.mat(np.mean(data, axis=0))
    y = y.T
    return np.mat(y)

if __name__ == '__main__':
    directory = '/home/zsp/Desktop/imu/data2/'
    acc_path = directory + 'accel-0.csv'
    acc_data = np.genfromtxt(acc_path, delimiter=',', skip_header=1)
    g = 9.79484197226504

#1
    ax = 0.0
    ay = 0.0
    az = -1.0 * g
    y = yf(acc_data[0:5000,:])
    x = xf(ax, ay, az)
#2
    ax = 0.0
    ay = -1.0 * g
    az = 0.0
    y = np.append(y, yf(acc_data[8000:13000,:]), axis=0)
    x = np.append(x, xf(ax, ay, az), axis=0)
#3
    ax = 0.0
    ay = 0.0
    az = 1.0 * g
    y = np.append(y, yf(acc_data[16000:21000,:]), axis=0)
    x = np.append(x, xf(ax, ay, az), axis=0)
#4
    ax = 0.0
    ay = 1.0 * g
    az = 0.0
    y = np.append(y, yf(acc_data[24000:29000,:]), axis=0)
    x = np.append(x, xf(ax, ay, az), axis=0)
#5
    ax = 0.0
    ay = 0.0
    az = -1.0 * g
    y = np.append(y, yf(acc_data[32000:37000,:]), axis=0)
    x = np.append(x, xf(ax, ay, az), axis=0)
#6
    ax = 1.0 * g
    ay = 0.0
    az = 0.0
    y = np.append(y, yf(acc_data[40000:45000,:]), axis=0)
    x = np.append(x, xf(ax, ay, az), axis=0)
#7
    ax = 0.0
    ay = 0.0
    az = 1.0 * g
    y = np.append(y, yf(acc_data[48000:53000,:]), axis=0)
    x = np.append(x, xf(ax, ay, az), axis=0)
#8
    ax = -1.0 * g
    ay = 0.0
    az = 0.0
    y = np.append(y, yf(acc_data[56000:61000,:]), axis=0)
    x = np.append(x, xf(ax, ay, az), axis=0)
#9
    ax = 0.0
    ay = 0.0
    az = -1.0 * g
    y = np.append(y, yf(acc_data[64000:69000,:]), axis=0)
    x = np.append(x, xf(ax, ay, az), axis=0)

    theta = (x.T*x).I * x.T * y
    ks = np.mat([[theta[0, 0], theta[3, 0], theta[4, 0]],
                 [theta[5, 0], theta[1, 0], theta[6, 0]],
                 [theta[7, 0], theta[8, 0], theta[2, 0]]])
    delta = np.mat([[theta[9, 0]], [theta[10, 0]], [theta[11, 0]]])
    print 'ks ='
    print ks
    print 'delta ='
    print delta