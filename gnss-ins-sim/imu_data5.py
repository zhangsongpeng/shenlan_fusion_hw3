# -*- coding: utf-8 -*-
# Filename: imu_data.py

import os
import math
import numpy as np
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# globals
D2R = math.pi/180

motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 10000.0          # IMU sample frequency
fs_gps = 10000.0       # GPS sample frequency
fs_mag = fs         # magnetometer sample frequency, not used for now

def test_path_gen():
    '''
    test only path generation in Sim.
    '''
    #### imu_err
    imu_err = {'gyro_b': np.array([1e-2, 2e-2, 3e-2]) / D2R * 3600 * 0,
               'gyro_arw': np.array([1e-5, 1e-5, 1e-5]) / D2R * 60,
               'gyro_b_stability': np.array([5e-6, 5e-6, 5e-6]) / D2R * 3600,
               'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
               'accel_b': np.array([2.0e-3, 1.0e-3, 5.0e-3]) * 0,
               'accel_vrw': np.array([1e-4, 1e-4, 1e-4]) * 60.0,
               'accel_b_stability': np.array([1e-5, 1e-5, 1e-5]) * 1.0,
               'accel_b_corr': np.array([200.0, 200.0, 200.0]),
            #    'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0
              }
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=True)

    #### start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      motion_def_path+"//imu_def6.csv",
                      ref_frame=1,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=None)
    sim.run(1)
    # save simulation data to files
    sim.results('/home/zsp/Desktop/imu/data5')
    # plot data, 3d plot of reference positoin, 2d plots of gyro and accel
    # sim.plot(['gyro', 'accel'])
    sim.plot(['ref_pos', 'gyro', 'gps_visibility'], opt={'ref_pos': '3d'})

if __name__ == '__main__':
    test_path_gen()
