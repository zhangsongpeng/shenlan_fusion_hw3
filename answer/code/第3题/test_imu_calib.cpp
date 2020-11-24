#include <iostream>

#include "imu_tk/io_utils.h"
#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"

using namespace std;
using namespace imu_tk;
using namespace Eigen;

#define D2R 0.017453292519943295
bool ReadData(const std::vector<std::string> &path,
              std::vector<TriadData> &accs,
              std::vector<TriadData> &gyros)
{
    accs.clear();
    gyros.clear();
    std::vector<std::ifstream> reads;
    for (int i = 0; i < 3; ++i)
    {
        reads.push_back(std::ifstream(path[i]));
    }
    bool init = false;
    while (true)
    {
        if (!init)
        {
            init = true;
            for (int i = 0; i < 3; ++i)
            {
                std::string strs;
                std::getline(reads[i], strs);
            }
        }
        else
        {
            double time;
            {
                std::string strs;
                if (std::getline(reads[0], strs))
                {
                    time = std::stod(strs);
                }
                else
                {
                    break;
                }
            }

            {
                std::string strs;
                std::string temp;

                strs = "";
                std::getline(reads[1], strs);
                temp = "";
                std::vector<double> acc;
                for (int i = 0; i < strs.size(); ++i)
                {
                    if (strs[i] == ',')
                    {
                        acc.push_back(std::stod(temp));
                        temp = "";
                    }
                    else
                    {
                        temp = temp + strs[i];
                    }
                }
                acc.push_back(std::stod(temp));

                strs = "";
                std::getline(reads[2], strs);
                temp = "";
                std::vector<double> gyro;
                for (int i = 0; i < strs.size(); ++i)
                {
                    if (strs[i] == ',')
                    {
                        gyro.push_back(std::stod(temp));
                        temp = "";
                    }
                    else
                    {
                        temp = temp + strs[i];
                    }
                }
                gyro.push_back(std::stod(temp));

                // std::cout << time << std::endl;
                // for (int i = 0; i < gyro.size(); ++i)
                // {
                //     std::cout << gyro[i] << std::endl;
                // }
                // for (int i = 0; i < acc.size(); ++i)
                // {
                //     std::cout << acc[i] << std::endl;
                // }
                // throw;
                accs.push_back(TriadData(time, acc[0], acc[1], acc[2]));
                gyros.push_back(TriadData(time, gyro[0] * D2R, gyro[1] * D2R, gyro[2] * D2R));
            }
        }
    }
}

int main(int argc, char **argv)
{

    vector<TriadData> acc_data, gyro_data;

    std::vector<std::string> path;

    path.push_back("/home/zsp/Desktop/imu/data4/time.csv");
    path.push_back("/home/zsp/Desktop/imu/data4/accel-0.csv");
    path.push_back("/home/zsp/Desktop/imu/data4/gyro-0.csv");
    ReadData(path, acc_data, gyro_data);

      CalibratedTriad init_acc_calib, init_gyro_calib;
      init_acc_calib.setBias( Vector3d(0, 0, 0) );
      init_acc_calib.setScale( Vector3d(1.0, 1.0, 1.0) );
      init_gyro_calib.setBias( Vector3d(0, 0, 0) );
      init_gyro_calib.setScale( Vector3d(1.0, 1.0, 1.0) );
    // CalibratedTriad init_acc_calib(0, 0, 0, 0.03, -0.04, 0.05, 0.95, 1.03, 1.05, 0.02, 0.01, -0.02);
    // CalibratedTriad init_gyro_calib(0.03, -0.02, 0.03, -0.02, 0.05, -0.03, 1.03, 0.97, 1.01, -0.01, 0.03, 0.02);

    MultiPosCalibration mp_calib;

    mp_calib.setInitStaticIntervalDuration(50.0);
    mp_calib.setInitAccCalibration(init_acc_calib);
    mp_calib.setInitGyroCalibration(init_gyro_calib);
    mp_calib.setGravityMagnitude(9.79484197226504);
    mp_calib.enableVerboseOutput(true);
    mp_calib.enableAccUseMeans(false);
    //mp_calib.setGyroDataPeriod(0.01);
    mp_calib.calibrateAccGyro(acc_data, gyro_data);
    mp_calib.getAccCalib().save("test_imu_acc.calib");
    mp_calib.getGyroCalib().save("test_imu_gyro.calib");

    //   for( int i = 0; i < acc_data.size(); i++)
    //   {
    //     cout<<acc_data[i].timestamp()<<" "
    //         <<acc_data[i].x()<<" "<<acc_data[i].y()<<" "<<acc_data[i].z()<<" "
    //         <<gyro_data[i].x()<<" "<<gyro_data[i].y()<<" "<<gyro_data[i].z()<<endl;
    //   }
    //   cout<<"Read "<<acc_data.size()<<" tuples"<<endl;

    return 0;
}