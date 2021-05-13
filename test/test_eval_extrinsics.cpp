#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <numeric>

#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "YamlFileIO.h"

Eigen::Matrix4d loadCameraPose(const std::string &file)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    std::ifstream fs(file);
    if (!fs.is_open())
    {
        std::cerr << "Fail to open camera pose file " << file << std::endl;
        return T;
    }

    std::string line;
    int i = 0;
    while (!fs.eof())
    {
        line.clear();
        getline(fs, line);
        if (line.empty())
            continue;

        std::stringstream ss(line);
        std::string dummy_str;
        if (i < 4)
        {
            ss >> T(i, 0) >> dummy_str >> T(i, 1) >> dummy_str >> T(i, 2) >> dummy_str >> T(i, 3);
            i++;
        }
        else
        {
            std::cerr << "Invalid camera pose format!\n";
            break;
        }
    }
    fs.close();

    return T;
}


void evaluateBackpackExt(const std::vector<Eigen::Matrix4d> &v_extrinsics, bool b_eval_cam2cam = true){
    Eigen::Matrix4d T_c0_c1_gt = Eigen::Matrix4d::Identity();
    T_c0_c1_gt  << 0.0,  0, -1.0, -0.05647, 
                    0,  1.0, 0,   0, 
                   1.0, 0, 0.0, -0.05647,
                   0, 0, 0, 1;
    Eigen::Matrix4d T_c0_c2_gt = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_c0_c3_gt = Eigen::Matrix4d::Identity();

    Eigen::Matrix3d R_c0_c1 = T_c0_c1_gt.block<3, 3>(0, 0);
    Eigen::Vector3d v_c0_c1_angles = R_c0_c1.eulerAngles(2,1,0) * 180.0/M_PI;
    LOG(INFO) << "C0-C1 Euler angles: " << v_c0_c1_angles.transpose() << "\n";


    // Eigen::Matrix3d R_c0_c2 = T_c0_c2_gt.block<3, 3>(0, 0);
    // Eigen::Vector3d v_c0_c2_angles = R_c0_c2.eulerAngles(2,1,0) * 180.0/M_PI;
    // LOG(INFO) << "C0-C2 Euler angles: " << v_c0_c2_angles.transpose() << "\n";

    // Eigen::Matrix3d R_c0_c3 = T_c0_c3_gt.block<3, 3>(0, 0);
    // Eigen::Vector3d v_c0_c3_angles = R_c0_c3.eulerAngles(2,1,0) * 180.0/M_PI;
    // LOG(INFO) << "C0-C3 Euler angles: " << v_c0_c3_angles.transpose() << "\n";

    Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
    if (b_eval_cam2cam)
        T_gt = T_c0_c1_gt;
    for(auto &T_ext : v_extrinsics){
        Eigen::Matrix4d T_error = T_gt.inverse() * T_ext;

        Eigen::Matrix3d R_c0_c1 = T_ext.block<3, 3>(0, 0);
        Eigen::Vector3d v_c0_c1_angles = R_c0_c1.eulerAngles(2,1,0) * 180.0/M_PI;
        LOG(INFO) << "C0-C1 Euler angles: " << v_c0_c1_angles.transpose() << "\n";

        Eigen::Vector3d t_c0_c1 = T_ext.block<3,1>(0, 3);
        LOG(INFO) << "C0-C1 trans vector: " << t_c0_c1.transpose() << "\n";
        Eigen::Matrix3d err_R = T_error.block<3, 3>(0, 0);
        Eigen::Vector3d err_t = T_error.block<3, 1>(0, 3);
        Eigen::AngleAxisd delta_R_vec(err_R);
        LOG(INFO) << " Rotation difference angle "
                    << delta_R_vec.angle() * 180.0 / M_PI << "\n";
        LOG(INFO) << " Translation difference norm is "
                    << err_t.norm() << "\n";
    }
}

void evaluateAntmanExt(const std::vector<Eigen::Matrix4d> &v_extrinsics, bool b_eval_cam2cam = true){
    Eigen::Matrix4d T_c0_c1_gt = Eigen::Matrix4d::Identity();
    T_c0_c1_gt  << 0, 0, 1.0, -0.05647,
                   0, 1.0, 0,  0, 
                   -1.0, 0, 0, -0.05647,
                   0, 0, 0, 1;
    Eigen::Matrix4d T_c0_c2_gt = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_c0_c3_gt = Eigen::Matrix4d::Identity();

    Eigen::Matrix3d R_c0_c1 = T_c0_c1_gt.block<3, 3>(0, 0);
    Eigen::Vector3d v_c0_c1_angles = R_c0_c1.eulerAngles(2,1,0) * 180.0/M_PI;
    LOG(INFO) << "C0-C1 Euler angles: " << v_c0_c1_angles.transpose() << "\n";

    // Eigen::Matrix3d R_c0_c2 = T_c0_c2_gt.block<3, 3>(0, 0);
    // Eigen::Vector3d v_c0_c2_angles = R_c0_c2.eulerAngles(2,1,0) * 180.0/M_PI;
    // LOG(INFO) << "C0-C2 Euler angles: " << v_c0_c2_angles.transpose() << "\n";

    // Eigen::Matrix3d R_c0_c3 = T_c0_c3_gt.block<3, 3>(0, 0);
    // Eigen::Vector3d v_c0_c3_angles = R_c0_c3.eulerAngles(2,1,0) * 180.0/M_PI;
    // LOG(INFO) << "C0-C3 Euler angles: " << v_c0_c3_angles.transpose() << "\n";
    Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
    if (b_eval_cam2cam)
        T_gt = T_c0_c1_gt;
    for(auto &T_ext : v_extrinsics){
        Eigen::Matrix4d T_error = T_gt.inverse() * T_ext;

        Eigen::Matrix3d R_c0_c1 = T_ext.block<3, 3>(0, 0);
        Eigen::Vector3d v_c0_c1_angles = R_c0_c1.eulerAngles(2,1,0) * 180.0/M_PI;
        LOG(INFO) << "C0-C1 Euler angles: " << v_c0_c1_angles.transpose() << "\n";

        Eigen::Vector3d t_c0_c1 = T_ext.block<3,1>(0, 3);
        LOG(INFO) << "C0-C1 trans vector: " << t_c0_c1.transpose() << "\n";
        // Eigen::Matrix3d err_R = T_error.block<3, 3>(0, 0);
        // Eigen::Vector3d err_t = T_error.block<3, 1>(0, 3);
        // Eigen::AngleAxisd delta_R_vec(err_R);
        // LOG(INFO) << "[evaluateKaleidoExt] Rotation difference angle "
        //             << delta_R_vec.angle() * 180.0 / M_PI << "\n";
        // LOG(INFO) << "[evaluateKaleidoExt] Translation difference norm is "
        //             << err_t.norm() << "\n";
    }
}

void evaluateKaleidoExt(const std::vector<Eigen::Matrix4d> &v_extrinsics, bool b_eval_cam2cam = true){
    Eigen::Matrix4d T_c0_c1_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c0_c1_vec = Eigen::AngleAxisd(28*M_PI/180.0, Eigen::Vector3d(1,0,0));
    Eigen::Vector3d t_c0_c1(0, 0.028, 0);
    T_c0_c1_gt.block<3, 3>(0, 0) = c0_c1_vec.matrix();
    T_c0_c1_gt.block<3, 1>(0, 3) = t_c0_c1;

    // Eigen::Matrix3d R_c0_c1 = T_c0_c1_gt.block<3, 3>(0, 0);
    // Eigen::Vector3d v_c0_c1_angles = R_c0_c1.eulerAngles(2,1,0) * 180.0/M_PI;
    // LOG(INFO) << "C0-C1 Euler angles: " << v_c0_c1_angles.transpose() << "\n";

    Eigen::Matrix4d T_c0_l0_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c0_l0_vec = Eigen::AngleAxisd(-14*M_PI/180.0, Eigen::Vector3d(1,0,0));
    Eigen::Vector3d t_c0_l0(0, -0.014, 0.020);
    T_c0_l0_gt.block<3, 3>(0, 0) = c0_l0_vec.matrix();
    T_c0_l0_gt.block<3, 1>(0, 3) = t_c0_l0;
    // Eigen::Matrix3d R_c0_l0 = T_c0_l0_gt.block<3, 3>(0, 0);
    // Eigen::Vector3d v_c0_l0_angles = R_c0_l0.eulerAngles(2,1,0) * 180.0/M_PI;
    // LOG(INFO) << "C0-L0 Euler angles: " << v_c0_l0_angles.transpose() << "\n";

    Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
    if (b_eval_cam2cam)
        T_gt = T_c0_c1_gt;
    else
        T_gt = T_c0_l0_gt;

    double avg_roll = 0.0, avg_pitch = 0.0, avg_yaw = 0.0;
    double avg_tx = 0, avg_ty = 0, avg_tz = 0.0;
    for(auto &T_ext : v_extrinsics){
        Eigen::Matrix4d T_error = T_gt.inverse() * T_ext;

        Eigen::Matrix3d R = T_ext.block<3, 3>(0, 0);
        Eigen::Vector3d v_angles = R.eulerAngles(2,1,0) * 180.0/M_PI;
        avg_yaw += (std::abs(v_angles[0]) > 90) ? (180 - std::abs(v_angles[0])) : std::abs(v_angles[0]);
        avg_pitch += (std::abs(v_angles[1]) > 90) ? (180 - std::abs(v_angles[1])) : std::abs(v_angles[1]);
        avg_roll += (std::abs(v_angles[2]) > 90) ? (180 - std::abs(v_angles[2])) : std::abs(v_angles[2]);
        LOG(INFO) << "Euler angles: " << v_angles.transpose() << "\n";

        Eigen::Vector3d t = T_ext.block<3,1>(0, 3);
        avg_tx += t[0];
        avg_ty += t[1];
        avg_tz += t[2];
        LOG(INFO) << "trans vector: " << t.transpose() << "\n";
        Eigen::Matrix3d err_R = T_error.block<3, 3>(0, 0);
        Eigen::Vector3d err_t = T_error.block<3, 1>(0, 3);
        Eigen::AngleAxisd delta_R_vec(err_R);
        LOG(INFO) << " Rotation difference angle "
                    << delta_R_vec.angle() * 180.0 / M_PI << "\n";
        LOG(INFO) << " Translation difference norm is "
                    << err_t.norm() << "\n";
    }
    LOG(INFO) << "Avg euler angles: (" << avg_roll/v_extrinsics.size() << ", " << avg_pitch/v_extrinsics.size() << ", " << avg_yaw/v_extrinsics.size() << ")";
    LOG(INFO) << "Avg trans: (" << avg_tx/v_extrinsics.size() << ", " << avg_ty/v_extrinsics.size() << ", " << avg_tz/v_extrinsics.size() << ")";
}

// compare several camera poses or extrinsic params, to evaluate params consistency
int main(int argc, char** argv){
    
    // This exe is used to analyze the exytrinsic calibration repeatability
    if(argc < 2){
        std::cerr <<"Usage: ./test_eval_extrinsics [input_folder] [file_prefix]\n";
        std::cout << "[input_folder]: contains many extrinsic calibration result.\n";
        std::cout << "[file_prefix]: extrinsic file prefix, different sensor configuration results in different extrinsic file.\n";
        return -1;
    }

    std::string dataset_folder(argv[1]);
    std::string file_prefix(argv[2]);

    std::vector<Eigen::Matrix4d> v_extrinsics;

    bool b_eval_backpack = false, b_eval_antman = false, b_eval_kaleido = false;
    if (dataset_folder.find("backpack") != std::string::npos)
        b_eval_backpack = true;
    if (dataset_folder.find("antman") != std::string::npos)
        b_eval_antman = true;
    if (dataset_folder.find("kaleido") != std::string::npos)
        b_eval_kaleido = true;
        
    for (const auto & entry : boost::filesystem::directory_iterator(dataset_folder)){
        if(!boost::filesystem::is_directory(entry))
            continue;

        std::string extrin_filepath = entry.path().string() + "/" +file_prefix + ".yml";
        Eigen::Matrix4d T_ext = Eigen::Matrix4d::Identity();
        common::loadExtFileOpencv(extrin_filepath, T_ext);
        v_extrinsics.emplace_back(T_ext);
    }

    bool b_eval_cam2cam = false, b_eval_lidar2lidar = false;
    if (file_prefix == "camera0_to_camera1" || file_prefix == "camera0_to_camera2" || file_prefix == "camera0_to_camera3")
        b_eval_cam2cam = true;
    if (file_prefix == "lidar0_to_lidar1")
        b_eval_lidar2lidar = true;

    if (b_eval_backpack){
    evaluateBackpackExt(v_extrinsics, b_eval_cam2cam);
    }else{
        if (b_eval_antman){
            evaluateAntmanExt(v_extrinsics, b_eval_cam2cam);
        }else{
            evaluateKaleidoExt(v_extrinsics, b_eval_cam2cam);
        }
    }
    return 0;
}