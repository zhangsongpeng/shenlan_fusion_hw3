#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#define D2R 0.017453292519943295
bool ReadData(const std::vector<std::string> &path,
              std::vector<double> &stamps,
              std::vector<Eigen::Vector3d> &accs,
              std::vector<Eigen::Vector3d> &gyros,
              std::vector<Eigen::Vector3d> &gpses,
              std::vector<Eigen::Vector3d> &ref_poses,
              std::vector<Eigen::Quaterniond> &ref_att_quats)
{
    stamps.clear();
    accs.clear();
    gyros.clear();
    gpses.clear();
    ref_poses.clear();
    ref_att_quats.clear();
    std::vector<std::ifstream> reads;
    // int count = 0;
    for (int i = 0; i < 6; ++i)
    {
        reads.push_back(std::ifstream(path[i]));
    }
    bool init = false;
    while (true)
    {
        if (!init)
        {
            init = true;
            for (int i = 0; i < 6; ++i)
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
                    // count++;
                    // count = count % denom;
                    // if (count != 0)
                    // {
                    //     for (int i = 1; i < 6; i++)
                    //     {
                    //         std::getline(reads[i], strs);
                    //     }
                    //     continue;
                    // }
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
                strs = "";
                std::getline(reads[3], strs);
                temp = "";
                std::vector<double> gps;
                for (int i = 0; i < strs.size(); ++i)
                {
                    if (strs[i] == ',')
                    {
                        gps.push_back(std::stod(temp));
                        temp = "";
                    }
                    else
                    {
                        temp = temp + strs[i];
                    }
                }
                gps.push_back(std::stod(temp));
                strs = "";
                std::getline(reads[4], strs);
                temp = "";
                std::vector<double> ref_pos;
                for (int i = 0; i < strs.size(); ++i)
                {
                    if (strs[i] == ',')
                    {
                        ref_pos.push_back(std::stod(temp));
                        temp = "";
                    }
                    else
                    {
                        temp = temp + strs[i];
                    }
                }
                ref_pos.push_back(std::stod(temp));
                strs = "";
                std::getline(reads[5], strs);
                temp = "";
                std::vector<double> ref_att_quat;
                for (int i = 0; i < strs.size(); ++i)
                {
                    if (strs[i] == ',')
                    {
                        ref_att_quat.push_back(std::stod(temp));
                        temp = "";
                    }
                    else
                    {
                        temp = temp + strs[i];
                    }
                }
                ref_att_quat.push_back(std::stod(temp));

                // std::cout << time << std::endl;
                // std::cout << (Eigen::Vector3d(acc[0], acc[1], acc[2])).transpose() << std::endl;
                // std::cout << (Eigen::Vector3d(gyro[0], gyro[1], gyro[2])).transpose() << std::endl;
                // std::cout << (Eigen::Vector3d(gps[0], gps[1], gps[2])).transpose() << std::endl;
                // std::cout << (Eigen::Vector3d(ref_pos[0], ref_pos[1], ref_pos[2])).transpose() << std::endl;
                // std::cout << (Eigen::Quaterniond(ref_att_quat[0], ref_att_quat[1], ref_att_quat[2], ref_att_quat[3])).coeffs().transpose() << std::endl;
                // throw;
                stamps.push_back(time);
                accs.push_back(Eigen::Vector3d(acc[0], acc[1], acc[2]));
                gyros.push_back(Eigen::Vector3d(gyro[0] * D2R, gyro[1] * D2R, gyro[2] * D2R));
                Eigen::Quaterniond q = Eigen::AngleAxisd(90 * D2R, Eigen::Vector3d::UnitZ()) *
                                       Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(180 * D2R, Eigen::Vector3d::UnitX());
                q = q.inverse();
                gpses.push_back(Eigen::Vector3d(gps[0], gps[1], gps[2]));
                // std::cout << Eigen::Vector3d(ref_pos[0], ref_pos[1], ref_pos[2]) << std::endl;
                // std::cout << q * Eigen::Vector3d(ref_pos[0], ref_pos[1], ref_pos[2]) << std::endl;
                // throw;
                ref_poses.push_back(q * Eigen::Vector3d(ref_pos[0], ref_pos[1], ref_pos[2]));
                ref_att_quats.push_back(q * Eigen::Quaterniond(ref_att_quat[0], ref_att_quat[1], ref_att_quat[2], ref_att_quat[3]));
            }
        }
    }
}

int main(int argc, char **argv)
{
    int method = std::atoi(argv[1]);
    std::vector<double> stamps;
    std::vector<Eigen::Vector3d> accs;
    std::vector<Eigen::Vector3d> gyros;
    std::vector<Eigen::Vector3d> gpses;
    std::vector<Eigen::Vector3d> ref_poses;
    std::vector<Eigen::Quaterniond> ref_att_quats;

    std::vector<std::string> path;

    path.push_back("/home/zsp/Desktop/imu/data5/time.csv");
    path.push_back("/home/zsp/Desktop/imu/data5/accel-0.csv");
    path.push_back("/home/zsp/Desktop/imu/data5/gyro-0.csv");
    path.push_back("/home/zsp/Desktop/imu/data5/gps-0.csv");
    path.push_back("/home/zsp/Desktop/imu/data5/ref_pos.csv");
    path.push_back("/home/zsp/Desktop/imu/data5/ref_att_quat.csv");
    ReadData(path, stamps, accs, gyros, gpses, ref_poses, ref_att_quats);
    // std::cout << stamps.size() << ", " << accs.size() << std::endl;
    std::ofstream gt;
    gt.open("/home/zsp/Desktop/imu/data5/gt.txt", std::fstream::out);
    std::ofstream pose;
    pose.open("/home/zsp/Desktop/imu/data5/pose.txt", std::fstream::out);
    std::ofstream pose_gt;
    pose_gt.open("/home/zsp/Desktop/imu/data5/pose_gt.txt", std::fstream::out);
    Eigen::Vector3d g = Eigen::Vector3d(0, 0, -9.79484197226504);

    Eigen::Quaterniond c_nm_bm(1, 0, 0, 0);
    Eigen::Quaterniond c_e_nm(1, 0, 0, 0);
    Eigen::Quaterniond c_e_nm_1(1, 0, 0, 0);
    Eigen::Quaterniond c_i_bm_1(1, 0, 0, 0);
    Eigen::Quaterniond c_i_bm(1, 0, 0, 0);
    Eigen::Quaterniond c_i_nm_1(1, 0, 0, 0);
    Eigen::Quaterniond c_i_nm(1, 0, 0, 0);
    Eigen::Quaterniond c_e_bm_1(1, 0, 0, 0);
    Eigen::Quaterniond c_e_bm(1, 0, 0, 0);
    Eigen::Vector3d p_e_nm_e(0, 0, 0);
    Eigen::Vector3d p_e_nm_1_e(0, 0, 0);
    Eigen::Vector3d v_e_nm_e(0, 0, 0);
    Eigen::Vector3d v_e_nm_1_e(0, 0, 0);
    Eigen::Vector3d v_e_nm_nm(0, 0, 0);
    double w_ie = 360.0 / 24.0 / 3600.0 * D2R;
    double rm = 6353346.18315;
    double rn = 6384140.52699;
    double L = 32 * D2R;
    double h = 0;
    int denom = std::atoi(argv[2]);
    for (int i = 1; i < stamps.size() / denom; ++i)
    {
        double dt = stamps[i * denom] - stamps[(i - 1) * denom];
        Eigen::Vector3d w_ie_n = Eigen::Vector3d(0, w_ie * std::cos(L), w_ie * std::sin(L));
        Eigen::Vector3d w_en_n = Eigen::Vector3d(-v_e_nm_nm[1] / (rm + h), v_e_nm_nm[0] / (rn + h), v_e_nm_nm[0] / (rn + h) * std::tan(L));
        Eigen::Vector3d phi(0, 0, 0);
        double angle;
        Eigen::Vector3d direction;
        Eigen::Quaterniond c_bm_1_bm;
        if (method == 0)
        {
            for (int j = 1; j < denom; j++)
            {
                double deltat = stamps[(i - 1) * denom + j] - stamps[(i - 1) * denom + j - 1];
                phi = phi + 0.5 * deltat * (gyros[(i - 1) * denom + j - 1] + gyros[(i - 1) * denom + j]);
            }
            angle = phi.norm();
            if (angle == 0)
            {
                direction = Eigen::Vector3d(0, 0, 0);
            }
            else
            {
                direction = phi / angle;
                direction = direction * std::sin(angle / 2.0);
            }
            c_bm_1_bm = Eigen::Quaterniond(std::cos(angle / 2.0), direction[0], direction[1], direction[2]);
        }
        else
        {
            // for (int j = 1; j < denom; j++)
            // {
            //     double deltat = stamps[(i - 1) * denom + j] - stamps[(i - 1) * denom + j - 1];
            //     phi = phi + 0.5 * deltat * (gyros[(i - 1) * denom + j - 1] + gyros[(i - 1) * denom + j]);
            // }
            Eigen::Vector3d phi1(0, 0, 0);
            Eigen::Vector3d phi2(0, 0, 0);
            for (int j = 1; j < denom / 2; j++)
            {
                double deltat = stamps[(i - 1) * denom + j] - stamps[(i - 1) * denom + j - 1];
                phi1 = phi1 + 0.5 * deltat * (gyros[(i - 1) * denom + j - 1] + gyros[(i - 1) * denom + j]);
            }
            for (int j = denom / 2; j < denom; j++)
            {
                double deltat = stamps[(i - 1) * denom + j] - stamps[(i - 1) * denom + j - 1];
                phi2 = phi2 + 0.5 * deltat * (gyros[(i - 1) * denom + j - 1] + gyros[(i - 1) * denom + j]);
            }
            phi = phi1 + phi2 + 2.0 / 3.0 * phi1.cross(phi2);
            // phi = phi1 + phi2 + 2.0 / 3.0 * phi1.cross(phi2) - phi;
            // std::cout << phi.transpose() << std::endl;
            // std::cout << phi1.transpose() << ",  " << phi1.cross(phi2).transpose() << std::endl;
            angle = phi.norm();
            if (angle == 0)
            {
                direction = Eigen::Vector3d(0, 0, 0);
            }
            else
            {
                direction = phi / angle;
                direction = direction * std::sin(angle / 2.0);
            }
            c_bm_1_bm = Eigen::Quaterniond(std::cos(angle / 2.0), direction[0], direction[1], direction[2]);
        }

        c_i_bm = c_i_bm_1 * c_bm_1_bm;

        phi = (w_ie_n + w_en_n) * dt;
        angle = phi.norm();
        if (angle == 0)
        {
            direction = Eigen::Vector3d(0, 0, 0);
        }
        else
        {
            direction = phi / angle;
            direction = direction * std::sin(angle / 2.0);
        }
        Eigen::Quaterniond c_nm_1_nm(std::cos(angle / 2.0), direction[0], direction[1], direction[2]);
        c_i_nm = c_i_nm_1 * c_nm_1_nm;

        c_nm_bm = c_i_nm.inverse() * c_i_bm;

        phi = w_en_n * dt;
        angle = phi.norm();
        if (angle == 0)
        {
            direction = Eigen::Vector3d(0, 0, 0);
        }
        else
        {
            direction = phi / angle;
            direction = direction * std::sin(angle / 2.0);
        }
        Eigen::Quaterniond temp(std::cos(angle / 2.0), direction[0], direction[1], direction[2]);
        c_e_nm = c_e_nm_1 * temp;

        c_e_bm = c_e_nm * c_nm_bm;

        v_e_nm_e = v_e_nm_1_e + dt * (0.5 * (c_e_bm_1 * accs[(i - 1) * denom] + c_e_bm * accs[i * denom]) + g);
        // std::cout << v_e_nm_e.transpose() << std::endl;
        v_e_nm_nm = c_e_nm.inverse() * v_e_nm_e;
        p_e_nm_e = p_e_nm_1_e + 0.5 * dt * (v_e_nm_1_e + v_e_nm_e);

        c_i_bm_1 = c_i_bm;
        c_i_nm_1 = c_i_nm;
        c_e_nm_1 = c_e_nm;
        c_e_bm_1 = c_e_bm;
        v_e_nm_1_e = v_e_nm_e;
        p_e_nm_1_e = p_e_nm_e;

        Eigen::Quaterniond qtemp = ref_att_quats[i * denom];
        angle = std::acos(qtemp.w()) * 2;
        double sin_angle = std::sin(angle / 2);
        Eigen::Vector3d dir(0, 0, 0);
        if (sin_angle != 0)
        {
            dir(0) = qtemp.x() / sin_angle;
            dir(1) = qtemp.y() / sin_angle;
            dir(2) = qtemp.z() / sin_angle;
            dir = dir * angle;
        }

        gt << std::fixed << stamps[i * denom] << " "
           << dir(0) << " " << dir(1) << " "
           << dir(2) << std::endl;

        qtemp = c_nm_bm;
        angle = std::acos(qtemp.w()) * 2;
        sin_angle = std::sin(angle / 2);
        dir = Eigen::Vector3d(0, 0, 0);
        if (sin_angle != 0)
        {
            dir(0) = qtemp.x() / sin_angle;
            dir(1) = qtemp.y() / sin_angle;
            dir(2) = qtemp.z() / sin_angle;
            dir = dir * angle;
        }

        pose << std::fixed << stamps[i * denom] << " "
             << dir(0) << " " << dir(1) << " "
             << dir(2) << std::endl;


        qtemp = ref_att_quats[i * denom].inverse() * c_nm_bm;
        if (qtemp.w() < 0)
        {
            qtemp.coeffs() = -qtemp.coeffs();
        }
        angle = std::acos(qtemp.w()) * 2;
        sin_angle = std::sin(angle / 2);
        dir = Eigen::Vector3d(0, 0, 0);
        if (sin_angle != 0)
        {
            dir(0) = qtemp.x() / sin_angle;
            dir(1) = qtemp.y() / sin_angle;
            dir(2) = qtemp.z() / sin_angle;
            dir = dir * angle;
        }

        pose_gt << std::fixed << stamps[i * denom] << " "
             << dir(0) << " " << dir(1) << " "
             << dir(2) << " " << angle << std::endl;
    }

    return 0;
}