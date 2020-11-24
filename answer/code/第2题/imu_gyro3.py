import os
import math
import numpy as np

dt = 0.01


def mf(data):
    y = np.mat(np.sum(data, axis=0) * dt)
    y = y.T
    return np.mat(y)


if __name__ == '__main__':
    directory = '/home/zsp/Desktop/imu/data3/'
    gyro_path = directory + 'gyro-0.csv'
    gyro_data = np.genfromtxt(gyro_path, delimiter=',', skip_header=1)
    g = 9.79484197226504
    D2R = math.pi/180
    w = 90
    t = 20
    w = w * t
#
    m1 = mf(gyro_data[500:2500, :])
    m2 = mf(gyro_data[4500:6500, :])
    sgxz = (m1[0, 0] - m2[0, 0]) / 2 / w
    sgyz = (m1[1, 0] - m2[1, 0]) / 2 / w
    kgz = (m1[2, 0] - m2[2, 0]) / 2 / w
#
    m1 = mf(gyro_data[8500:10500, :])
    m2 = mf(gyro_data[12500:14500, :])
    sgxy = (m1[0, 0] - m2[0, 0]) / 2 / w
    kgy = (m1[1, 0] - m2[1, 0]) / 2 / w
    sgzy = (m1[2, 0] - m2[2, 0]) / 2 / w
#
    m1 = mf(gyro_data[16500:18500, :])
    m2 = mf(gyro_data[20500:22500, :])
    kgx = (m1[0, 0] - m2[0, 0]) / 2 / w
    sgyx = (m1[1, 0] - m2[1, 0]) / 2 / w
    sgzx = (m1[2, 0] - m2[2, 0]) / 2 / w

    ks = np.mat([[kgx, sgxy, sgxz],
                 [sgyx, kgy, sgyz],
                 [sgzx, sgzy, kgz]])
#
    m1 = mf(gyro_data[24500:26500, :])
    m2 = mf(gyro_data[29500:31500, :])
    epsilonx = (m1[0,0] + m2[0, 0]) / 2 / 20 * D2R
    epsilony = (m1[1,0] + m2[1, 0]) / 2 / 20 * D2R
#
    m2 = mf(gyro_data[37500:39500, :])
    epsilonz = (m1[2,0] + m2[2, 0]) / 2 / 20 * D2R

    epsilon = np.mat([[epsilonx], [epsilony], [epsilonz]])

    print 'ks ='
    print ks
    print 'epsilon ='
    print epsilon

