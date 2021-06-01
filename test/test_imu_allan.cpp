#include <iostream>
#include <mutex>
#include <queue>
// #include <code_utils/ros_utils.h>
// #include <geometry_msgs/Vector3Stamped.h>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "acc_lib/allan_acc.h"
#include "acc_lib/fitallan_acc.h"
#include "gyr_lib/allan_gyr.h"
#include "gyr_lib/fitallan_gyr.h"
#include "FileSystemTools.h"
#include "YamlFileIO.h"



bool b_start_rec_data = true;
// start receiving data time
double start_t;
bool b_end_rec_data = false;
const double k_tms_unit = 1000.0;

bool loadRawImuData(const std::string &imu_filepath, imu::AllanGyr *gyr_x_ptr, imu::AllanGyr *gyr_y_ptr, imu::AllanGyr *gyr_z_ptr, 
                    imu::AllanAcc *acc_x_ptr, imu::AllanAcc *acc_y_ptr, imu::AllanAcc *acc_z_ptr, int max_time_min){
    std::ifstream ifs(imu_filepath);
    if (!ifs.is_open())
    {
        LOG(FATAL) << "Failed to open " << imu_filepath;
        return false;
    }

    std::string line_data;
    std::stringstream ss_data;
    while (!ifs.eof() && b_end_rec_data == false)
    {
        ss_data.clear();
        getline(ifs, line_data);
        if (line_data.empty())
            continue;

        ss_data.str(line_data);
        long double timestamp;
        double acc_x, acc_y, acc_z;
        double gyro_x, gyro_y, gyro_z;
        ss_data >> timestamp >> acc_x >> acc_y >> acc_z >> gyro_x >> gyro_y >> gyro_z;
        // std::cout << "timestamp: " << timestamp << ", " << acc_x << ", " << acc_y << ", " << acc_z << ", " << gyro_x << ", " << gyro_y << ", " << gyro_z << "\n";
        long double time = timestamp / k_tms_unit;
        // std::cout << "timestamp : " << time << "\n";
        gyr_x_ptr->pushRadPerSec(gyro_x, time);
        gyr_y_ptr->pushRadPerSec(gyro_y, time);
        gyr_z_ptr->pushRadPerSec(gyro_z, time);
        acc_x_ptr->pushMPerSec2(acc_x, time);
        acc_y_ptr->pushMPerSec2(acc_y, time);
        acc_z_ptr->pushMPerSec2(acc_z, time);

        if (b_start_rec_data)
        {
            start_t = time;
            b_start_rec_data = false;
            LOG(INFO) << "[imu_allan] Start recieve imu data\n";
        }
        else
        {
            double time_min = (time - start_t) / 60;
            if (time_min > max_time_min)
            {
                b_end_rec_data = true;
                LOG(INFO) << "[imu_allan] imu data duration is greater than 2  hours, start calibration!\n";
            }
        }
    }

    return true;
}

void writeData3(const std::string &output_folder,
                const std::string &sensor_name,
                const std::vector<double> &gyro_ts_x,
                const std::vector<double> &gyro_d_x,
                const std::vector<double> &gyro_d_y,
                const std::vector<double> &gyro_d_z)
{
    std::string tms_filepath = common::concatenateFolderAndFileName(output_folder, "data_" + sensor_name + "_t.txt");
    std::string x_std_filepath = common::concatenateFolderAndFileName(output_folder, "data_" + sensor_name + "_x.txt");
    std::string y_std_filepath = common::concatenateFolderAndFileName(output_folder, "data_" + sensor_name + "_y.txt");
    std::string z_std_filepath = common::concatenateFolderAndFileName(output_folder, "data_" + sensor_name + "_z.txt");
    std::ofstream out_t;
    std::ofstream out_x;
    std::ofstream out_y;
    std::ofstream out_z;
    out_t.open(tms_filepath, std::ios::trunc);
    out_x.open(x_std_filepath, std::ios::trunc);
    out_x.open(y_std_filepath, std::ios::trunc);
    out_x.open(z_std_filepath, std::ios::trunc);
    out_t << std::setprecision(10);
    out_x << std::setprecision(10);
    out_y << std::setprecision(10);
    out_z << std::setprecision(10);

    for (int index = 0; index < gyro_ts_x.size(); ++index)
    {
        out_t << gyro_ts_x[index] << '\n';
        out_x << gyro_d_x[index] << '\n';
        out_y << gyro_d_y[index] << '\n';
        out_z << gyro_d_z[index] << '\n';
    }

    out_t.close();
    out_x.close();
    out_y.close();
    out_z.close();
}

void writeYAML(const std::string &output_folder,
               const std::string sensor_name,
               const imu::FitAllanGyr &gyr_x,
               const imu::FitAllanGyr &gyr_y,
               const imu::FitAllanGyr &gyr_z,
               const imu::FitAllanAcc &acc_x,
               const imu::FitAllanAcc &acc_y,
               const imu::FitAllanAcc &acc_z)
{
    std::string save_filepath = common::concatenateFolderAndFileName(output_folder, sensor_name + "_imu_param.yaml");
    cv::FileStorage fs(save_filepath, cv::FileStorage::WRITE);

    fs << "type"
       << "IMU";

    fs << "name" << sensor_name;

    fs << "Gyr";
    fs << "{";
    fs << "unit"
       << " rad/s";

    fs << "avg-axis";
    fs << "{";
    fs << std::string("gyr_n")
       << (gyr_x.getWhiteNoise() + gyr_y.getWhiteNoise() + gyr_z.getWhiteNoise()) / 3;
    fs << std::string("gyr_w")
       << (gyr_x.getBiasInstability() + gyr_y.getBiasInstability() + gyr_z.getBiasInstability()) / 3;

    fs << "}";

    fs << "x-axis";
    fs << "{";
    fs << std::string("gyr_n") << gyr_x.getWhiteNoise();
    fs << std::string("gyr_w") << gyr_x.getBiasInstability();
    fs << "}";

    fs << "y-axis";
    fs << "{";
    fs << std::string("gyr_n") << gyr_y.getWhiteNoise();
    fs << std::string("gyr_w") << gyr_y.getBiasInstability();
    fs << "}";

    fs << "z-axis";
    fs << "{";
    fs << std::string("gyr_n") << gyr_z.getWhiteNoise();
    fs << std::string("gyr_w") << gyr_z.getBiasInstability();
    fs << "}";

    fs << "}";

    fs << "Acc";
    fs << "{";
    fs << "unit"
       << " m/s^2";

    fs << "avg-axis";
    fs << "{";
    fs << std::string("acc_n")
       << (acc_x.getWhiteNoise() + acc_y.getWhiteNoise() + acc_z.getWhiteNoise()) / 3;
    fs << std::string("acc_w")
       << (acc_x.getBiasInstability() + acc_y.getBiasInstability() + acc_z.getBiasInstability()) / 3;
    fs << "}";

    fs << "x-axis";
    fs << "{";
    fs << std::string("acc_n") << acc_x.getWhiteNoise();
    fs << std::string("acc_w") << acc_x.getBiasInstability();
    fs << "}";

    fs << "y-axis";
    fs << "{";
    fs << std::string("acc_n") << acc_y.getWhiteNoise();
    fs << std::string("acc_w") << acc_y.getBiasInstability();
    fs << "}";

    fs << "z-axis";
    fs << "{";
    fs << std::string("acc_n") << acc_z.getWhiteNoise();
    fs << std::string("acc_w") << acc_z.getBiasInstability();
    fs << "}";

    fs << "}";

    fs.release();
}

bool saveImuChainFile(const std::string &save_filepath, 
                    const imu::FitAllanGyr &fit_gyr_x, 
                    const imu::FitAllanGyr &fit_gyr_y, 
                    const imu::FitAllanGyr &fit_gyr_z, 
                    const imu::FitAllanAcc &fit_acc_x, 
                    const imu::FitAllanAcc &fit_acc_y, 
                    const imu::FitAllanAcc &fit_acc_z,
                    double frequency){
    std::ofstream yaml_out_file(save_filepath);
    if (!yaml_out_file.is_open()) {
        LOG(FATAL) << " Fail to open yaml file! \n";
        return false;
    }
    yaml_out_file << std::setprecision(10);

    YAML::Node root_node;
    double gyro_noise_density = (fit_gyr_x.getWhiteNoise() + fit_gyr_y.getWhiteNoise() + fit_gyr_z.getWhiteNoise()) / 3;
    double acc_noise_density = (fit_acc_x.getWhiteNoise() + fit_acc_y.getWhiteNoise() + fit_acc_z.getWhiteNoise()) / 3;
    double gyro_bias_noise = (fit_gyr_x.getBiasInstability() + fit_gyr_y.getBiasInstability() + fit_gyr_z.getBiasInstability()) / 3;
    double acc_bias_noise = (fit_acc_x.getBiasInstability() + fit_acc_y.getBiasInstability() + fit_acc_z.getBiasInstability()) /3;
    root_node["accelerometer_noise_density"] = acc_noise_density;
    root_node["accelerometer_random_walk"] = acc_bias_noise;
    root_node["gyroscope_noise_density"] = gyro_noise_density;
    root_node["gyroscope_random_walk"] = gyro_bias_noise;
    root_node["rostopic"] = "/imu0";
    root_node["update_rate"] = int(frequency);

    yaml_out_file << root_node;
    yaml_out_file.close();
    return true;
}

int main(int argc, char **argv)
{
    if (argc < 6){
        LOG(FATAL) << "Usage: test_imu_allan [imu_file] [imu_name] [max_time_min] [max_cluster] [output_folder] ";
        return -1;
    }

    std::string imu_filepath = argv[1];
    std::string IMU_NAME = argv[2];
    int max_time_min = std::atoi(argv[3]);
    int max_cluster = std::atoi(argv[4]);
    std::string output_folder = argv[5];

    if (!common::fileExists(imu_filepath)){
        LOG(FATAL) << "File does not exist :" << imu_filepath;
        return -1;
    }
    if (!common::pathExists(output_folder)){
        if (!common::createPath(output_folder)){
            LOG(FATAL) << "Couldnot create output folder: " << output_folder;
            return -1;
        }
    }

    imu::AllanGyr* gyr_x = new imu::AllanGyr("gyr x", max_cluster);
    imu::AllanGyr* gyr_y = new imu::AllanGyr("gyr y", max_cluster);
    imu::AllanGyr* gyr_z = new imu::AllanGyr("gyr z", max_cluster);
    imu::AllanAcc* acc_x = new imu::AllanAcc("acc x", max_cluster);
    imu::AllanAcc* acc_y = new imu::AllanAcc("acc y", max_cluster);
    imu::AllanAcc* acc_z = new imu::AllanAcc("acc z", max_cluster);
    LOG(INFO) << "Reading imu data....";

    // load raw imu data
    bool sts = loadRawImuData(imu_filepath, gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z, max_time_min);
    if (!sts) {
        return -1;
    }
    ///
    gyr_x->calc();
    std::vector<double> gyro_v_x = gyr_x->getVariance();
    std::vector<double> gyro_d_x = gyr_x->getDeviation();
    std::vector<double> gyro_ts_x = gyr_x->getTimes();

    gyr_y->calc();
    std::vector<double> gyro_v_y = gyr_y->getVariance();
    std::vector<double> gyro_d_y = gyr_y->getDeviation();
    std::vector<double> gyro_ts_y = gyr_y->getTimes();

    gyr_z->calc();
    std::vector<double> gyro_v_z = gyr_z->getVariance();
    std::vector<double> gyro_d_z = gyr_z->getDeviation();
    std::vector<double> gyro_ts_z = gyr_z->getTimes();

    std::cout << "Gyro X " << std::endl;
    imu::FitAllanGyr fit_gyr_x(gyro_v_x, gyro_ts_x, gyr_x->getFreq());
    std::cout << "  bias " << gyr_x->getAvgValue() / 3600 << " degree/s" << std::endl;
    std::cout << "-------------------" << std::endl;

    std::cout << "Gyro y " << std::endl;
    imu::FitAllanGyr fit_gyr_y(gyro_v_y, gyro_ts_y, gyr_y->getFreq());
    std::cout << "  bias " << gyr_y->getAvgValue() / 3600 << " degree/s" << std::endl;
    std::cout << "-------------------" << std::endl;

    std::cout << "Gyro z " << std::endl;
    imu::FitAllanGyr fit_gyr_z(gyro_v_z, gyro_ts_z, gyr_z->getFreq());
    std::cout << "  bias " << gyr_z->getAvgValue() / 3600 << " degree/s" << std::endl;
    std::cout << "-------------------" << std::endl;

    std::vector<double> gyro_sim_d_x = fit_gyr_x.calcSimDeviation(gyro_ts_x);
    std::vector<double> gyro_sim_d_y = fit_gyr_y.calcSimDeviation(gyro_ts_y);
    std::vector<double> gyro_sim_d_z = fit_gyr_z.calcSimDeviation(gyro_ts_z);

    writeData3(output_folder, IMU_NAME + "_sim_gyr", gyro_ts_x, gyro_sim_d_x, gyro_sim_d_y, gyro_sim_d_z);
    writeData3(output_folder, IMU_NAME + "_gyr", gyro_ts_x, gyro_d_x, gyro_d_y, gyro_d_z);

    std::cout << "==============================================" << std::endl;
    std::cout << "==============================================" << std::endl;

    acc_x->calc();
    std::vector<double> acc_v_x = acc_x->getVariance();
    std::vector<double> acc_d_x = acc_x->getDeviation();
    std::vector<double> acc_ts_x = acc_x->getTimes();

    acc_y->calc();
    std::vector<double> acc_v_y = acc_y->getVariance();
    std::vector<double> acc_d_y = acc_y->getDeviation();
    std::vector<double> acc_ts_y = acc_y->getTimes();

    acc_z->calc();
    std::vector<double> acc_v_z = acc_z->getVariance();
    std::vector<double> acc_d_z = acc_z->getDeviation();
    std::vector<double> acc_ts_z = acc_z->getTimes();

    std::cout << "acc X " << std::endl;
    imu::FitAllanAcc fit_acc_x(acc_v_x, acc_ts_x, acc_x->getFreq());
    std::cout << "-------------------" << std::endl;

    std::cout << "acc y " << std::endl;
    imu::FitAllanAcc fit_acc_y(acc_v_y, acc_ts_y, acc_y->getFreq());
    std::cout << "-------------------" << std::endl;

    std::cout << "acc z " << std::endl;
    imu::FitAllanAcc fit_acc_z(acc_v_z, acc_ts_z, acc_z->getFreq());
    std::cout << "-------------------" << std::endl;

    std::vector<double> acc_sim_d_x = fit_acc_x.calcSimDeviation(acc_ts_x);
    std::vector<double> acc_sim_d_y = fit_acc_y.calcSimDeviation(acc_ts_x);
    std::vector<double> acc_sim_d_z = fit_acc_z.calcSimDeviation(acc_ts_x);

    writeData3(output_folder, IMU_NAME + "_sim_acc", acc_ts_x, acc_sim_d_x, acc_sim_d_y, acc_sim_d_z);
    writeData3(output_folder, IMU_NAME + "_acc", acc_ts_x, acc_d_x, acc_d_y, acc_d_z);

    writeYAML(output_folder, IMU_NAME, fit_gyr_x, fit_gyr_y, fit_gyr_z, fit_acc_x, fit_acc_y, fit_acc_z);

    std::string save_filepath = common::concatenateFolderAndFileName(output_folder, "backpack_imu.yaml");
    saveImuChainFile(save_filepath, fit_gyr_x, fit_gyr_y, fit_gyr_z, fit_acc_x, fit_acc_y, fit_acc_z, gyr_x->getFreq());

    return 0;
}
