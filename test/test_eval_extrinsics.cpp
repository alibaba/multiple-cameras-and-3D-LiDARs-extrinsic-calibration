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


void evaluateBackpackExt(const std::vector<Eigen::Matrix4d> &v_extrinsics, const std::string &eval_pattern){
    Eigen::Matrix4d T_c0_c1_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c0_c1_vec = Eigen::AngleAxisd(-90*M_PI/180.0, Eigen::Vector3d(0,-1,0));
    Eigen::Vector3d t_c0_c1(-0.03543, 0, -0.03543);
    T_c0_c1_gt.block<3, 3>(0, 0) = c0_c1_vec.matrix();
    T_c0_c1_gt.block<3, 1>(0, 3) = t_c0_c1;
    Eigen::Matrix4d T_c0_c2_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c0_c2_vec = Eigen::AngleAxisd(-180*M_PI/180.0, Eigen::Vector3d(0,-1,0));
    Eigen::Vector3d t_c0_c2(0, 0, -0.07086);
    T_c0_c2_gt.block<3, 3>(0, 0) = c0_c2_vec.matrix();
    T_c0_c2_gt.block<3, 1>(0, 3) = t_c0_c2;
    Eigen::Matrix4d T_c0_c3_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c0_c3_vec = Eigen::AngleAxisd(90*M_PI/180.0, Eigen::Vector3d(0,-1,0));
    Eigen::Vector3d t_c0_c3(0.03543, 0, -0.03543);
    T_c0_c3_gt.block<3, 3>(0, 0) = c0_c3_vec.matrix();
    T_c0_c3_gt.block<3, 1>(0, 3) = t_c0_c3;

    Eigen::Matrix4d T_l0_c0_gt = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_l1_c0_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd l1_c0_vec1 = Eigen::AngleAxisd(-15*M_PI/180.0, Eigen::Vector3d(0,1,0));
    Eigen::AngleAxisd l1_c0_vec2 = Eigen::AngleAxisd(90*M_PI/180.0, Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d l1_c0_vec =  l1_c0_vec1.matrix()*l1_c0_vec2.matrix();
    Eigen::Vector3d t_l1_c0(0.58192, 0.0, -0.1954);
    T_l1_c0_gt.block<3,3>(0, 0) = l1_c0_vec;
    T_l1_c0_gt.block<3, 1>(0, 3) = l1_c0_vec* t_l1_c0;
    std::cout << "T_l1_c0_gt:\n" << T_l1_c0_gt << "\n";

    Eigen::Matrix4d T_l0_l1_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd l0_l1_vec1 = Eigen::AngleAxisd(-30*M_PI/180.0, Eigen::Vector3d(0,0,1));
    Eigen::AngleAxisd l0_l1_vec2 = Eigen::AngleAxisd(-75*M_PI/180.0, Eigen::Vector3d(0,1,0));
    Eigen::Matrix3d l0_l1_vec =  l0_l1_vec1.matrix()*l0_l1_vec2.matrix();
    Eigen::Vector3d t_l0_l1(-0.22555, 0.13022, -0.34922);
    T_l0_l1_gt.block<3,3>(0, 0) = l0_l1_vec;
    T_l0_l1_gt.block<3, 1>(0, 3) = t_l0_l1;

    Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
    if (eval_pattern == "camera0_to_camera1")
        T_gt = T_c0_c1_gt;
    if (eval_pattern == "camera0_to_camera2")
        T_gt = T_c0_c2_gt;
    if (eval_pattern == "camera0_to_camera3")
        T_gt = T_c0_c3_gt;
    if (eval_pattern == "lidar0_to_camera0")
        T_gt = T_l0_c0_gt;
    if (eval_pattern == "lidar1_to_camera0")
        T_gt = T_l1_c0_gt;
    if (eval_pattern == "lidar0_to_lidar1")
        T_gt = T_l0_l1_gt;

    double avg_roll = 0.0, avg_pitch = 0.0, avg_yaw = 0.0;
    double avg_tx = 0, avg_ty = 0, avg_tz = 0.0;
    for(auto &T_ext : v_extrinsics){
        Eigen::Matrix4d T_error = T_gt.inverse() * T_ext;

        std::cout << "T_ext:\n" << T_ext << "\n";
        Eigen::Matrix3d R = T_ext.block<3, 3>(0, 0);
        Eigen::Vector3d v_angles = R.eulerAngles(2,1,0) * 180.0/M_PI;
        avg_yaw += (std::abs(v_angles[0]) > 90) ? (180 - std::abs(v_angles[0])) : std::abs(v_angles[0]);
        avg_pitch += (std::abs(v_angles[1]) > 90) ? (180 - std::abs(v_angles[1])) : std::abs(v_angles[1]);
        avg_roll += (std::abs(v_angles[2]) > 90) ? (180 - std::abs(v_angles[2])) : std::abs(v_angles[2]);
        LOG(INFO) << eval_pattern << " euler angles(ZYX): " << v_angles.transpose() << "\n";

        Eigen::Vector3d t = T_ext.block<3,1>(0, 3);
        avg_tx += t[0]; avg_ty += t[1]; avg_tz += t[2];
        LOG(INFO) << eval_pattern << " trans vector: " << t.transpose() << "\n";

        Eigen::Matrix3d err_R = T_error.block<3, 3>(0, 0);
        Eigen::Vector3d err_t = T_error.block<3, 1>(0, 3);
        Eigen::AngleAxisd delta_R_vec(err_R);
        LOG(INFO) << " Rotation difference angle "
                    << delta_R_vec.angle() * 180.0 / M_PI << "\n";
        LOG(INFO) << " Translation difference norm is "
                    << err_t.norm() << "\n";
    }
    int size = v_extrinsics.size();
    LOG(INFO) << "Evaluate " << size  << " extrinsic parameters.";
    LOG(INFO) << "Avg euler angles: (" << avg_roll/size << ", " << avg_pitch/size << ", " << avg_yaw/size << ")";
    LOG(INFO) << "Avg trans: (" << avg_tx/size << ", " << avg_ty/size << ", " << avg_tz/size << ")";
}

void evaluateAntmanExt(const std::vector<Eigen::Matrix4d> &v_extrinsics, const std::string &eval_pattern){
    Eigen::Matrix4d T_c0_c1_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c0_c1_vec = Eigen::AngleAxisd(-90*M_PI/180.0, Eigen::Vector3d(0,-1,0));
    Eigen::Vector3d t_c0_c1(0.05647, 0, -0.05647);
    T_c0_c1_gt.block<3, 3>(0, 0) = c0_c1_vec.matrix();
    T_c0_c1_gt.block<3, 1>(0, 3) = t_c0_c1;
    Eigen::Matrix4d T_c0_c2_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c0_c2_vec = Eigen::AngleAxisd(180*M_PI/180.0, Eigen::Vector3d(0,1,0));
    Eigen::Vector3d t_c0_c2(0, 0, -0.11294);
    T_c0_c2_gt.block<3, 3>(0, 0) = c0_c2_vec.matrix();
    T_c0_c2_gt.block<3, 1>(0, 3) = t_c0_c2;
    Eigen::Matrix4d T_c0_c3_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c0_c3_vec = Eigen::AngleAxisd(-90*M_PI/180.0, Eigen::Vector3d(0,1,0));
    Eigen::Vector3d t_c0_c3(-0.05647, 0, -0.05647);
    T_c0_c3_gt.block<3, 3>(0, 0) = c0_c3_vec.matrix();
    T_c0_c3_gt.block<3, 1>(0, 3) = t_c0_c3;

    Eigen::Matrix4d T_c3_l0_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c3_l0_vec1 = Eigen::AngleAxisd(-M_PI/4.0, -Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd c3_l0_vec2 = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d c3_l0_vec = c3_l0_vec2.matrix() * c3_l0_vec1.matrix();
    Eigen::Vector3d t_c3_l0(-0.03887, -0.03887, 0.0593);
    T_c3_l0_gt.block<3,3>(0, 0) = c3_l0_vec;
    T_c3_l0_gt.block<3,1>(0, 3) = t_c3_l0;

    Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
    if (eval_pattern == "camera0_to_camera1")
        T_gt = T_c0_c1_gt;
    if (eval_pattern == "camera0_to_camera2")
        T_gt = T_c0_c2_gt;
    if (eval_pattern == "camera0_to_camera3")
        T_gt = T_c0_c3_gt;
    // it is lidar0-camera0 actually
    if (eval_pattern == "camera0_to_lidar0")
        T_gt = T_c3_l0_gt;

    Eigen::Matrix3d R_gt = T_gt.block<3, 3>(0, 0);
    Eigen::Vector3d v_gt_angles = R_gt.eulerAngles(2,1,0) * 180.0/M_PI;
    LOG(INFO) << "GT Euler angles(ZYX): " << v_gt_angles.transpose() << "\n";

    double avg_roll = 0.0, avg_pitch = 0.0, avg_yaw = 0.0;
    double avg_tx = 0, avg_ty = 0, avg_tz = 0.0;
    for(size_t i = 0; i < v_extrinsics.size(); i++){
        Eigen::Matrix4d T_ext = v_extrinsics[i];

        Eigen::Matrix4d T_error = T_gt.inverse() * T_ext;

        Eigen::Matrix3d R = T_ext.block<3, 3>(0, 0);
        Eigen::Vector3d v_angles = R.eulerAngles(2,1,0) * 180.0/M_PI;
        avg_yaw += (std::abs(v_angles[0]) > 90) ? (180 - std::abs(v_angles[0])) : std::abs(v_angles[0]);
        avg_pitch += (std::abs(v_angles[1]) > 90) ? (180 - std::abs(v_angles[1])) : std::abs(v_angles[1]);
        avg_roll += (std::abs(v_angles[2]) > 90) ? (180 - std::abs(v_angles[2])) : std::abs(v_angles[2]);
        LOG(INFO) << eval_pattern << " euler angles(ZYX): " << v_angles.transpose() << "\n";

        Eigen::Vector3d t = T_ext.block<3,1>(0, 3);
        avg_tx += t[0]; avg_ty += t[1]; avg_tz += t[2];
        LOG(INFO) << eval_pattern << " trans vector: " << t.transpose() << "\n";

        Eigen::Matrix3d err_R = T_error.block<3, 3>(0, 0);
        Eigen::Vector3d err_t = T_error.block<3, 1>(0, 3);
        Eigen::AngleAxisd delta_R_vec(err_R);
        LOG(INFO) << " Rotation difference angle "
                    << delta_R_vec.angle() * 180.0 / M_PI << "\n";
        LOG(INFO) << " Translation difference norm is "
                    << err_t.norm() << "\n";
    }

    int size = v_extrinsics.size();
    LOG(INFO) << "Evaluate " << size  << " extrinsic parameters.";
    LOG(INFO) << "Avg euler angles: (" << avg_roll/size << ", " << avg_pitch/size << ", " << avg_yaw/size << ")";
    LOG(INFO) << "Avg trans: (" << avg_tx/size << ", " << avg_ty/size << ", " << avg_tz/size << ")";
}

void evaluateKaleidoExt(const std::vector<Eigen::Matrix4d> &v_extrinsics, const std::string &eval_pattern){
    Eigen::Matrix4d T_c0_c1_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c0_c1_vec = Eigen::AngleAxisd(28*M_PI/180.0, Eigen::Vector3d(1,0,0));
    Eigen::Vector3d t_c0_c1(0, 0.028, 0);
    T_c0_c1_gt.block<3, 3>(0, 0) = c0_c1_vec.matrix();
    T_c0_c1_gt.block<3, 1>(0, 3) = t_c0_c1;

    Eigen::Matrix4d T_c0_l0_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd c0_l0_vec = Eigen::AngleAxisd(-14*M_PI/180.0, Eigen::Vector3d(1,0,0));
    Eigen::Vector3d t_c0_l0(0, -0.014, 0.020);
    T_c0_l0_gt.block<3, 3>(0, 0) = c0_l0_vec.matrix();
    T_c0_l0_gt.block<3, 1>(0, 3) = t_c0_l0;

    Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
    if (eval_pattern == "camera0_to_camera1")
        T_gt = T_c0_c1_gt;
    if (eval_pattern == "camera0_to_lidar0")
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
        LOG(INFO) << eval_pattern << " euler angles: " << v_angles.transpose() << "\n";

        Eigen::Vector3d t = T_ext.block<3,1>(0, 3);
        avg_tx += t[0]; avg_ty += t[1]; avg_tz += t[2];
        LOG(INFO) << eval_pattern << " trans vector: " << t.transpose() << "\n";
        Eigen::Matrix3d err_R = T_error.block<3, 3>(0, 0);
        Eigen::Vector3d err_t = T_error.block<3, 1>(0, 3);
        Eigen::AngleAxisd delta_R_vec(err_R);
        LOG(INFO) << " Rotation difference angle "
                    << delta_R_vec.angle() * 180.0 / M_PI << "\n";
        LOG(INFO) << " Translation difference norm is "
                    << err_t.norm() << "\n";
    }
    int size = v_extrinsics.size();
    LOG(INFO) << "Evaluate " << size  << " extrinsic parameters.";
    LOG(INFO) << "Avg euler angles: (" << avg_roll/size << ", " << avg_pitch/size << ", " << avg_yaw/size << ")";
    LOG(INFO) << "Avg trans: (" << avg_tx/size << ", " << avg_ty/size << ", " << avg_tz/size << ")";
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
        if (common::loadExtFileOpencv(extrin_filepath, T_ext)){
            LOG(INFO) << "Read extrinsic parameters from " << extrin_filepath;
        }else{
            LOG(ERROR) << "Fail to load " << extrin_filepath;
            continue;
        }
        v_extrinsics.emplace_back(T_ext);
    }

    if (b_eval_backpack){
    evaluateBackpackExt(v_extrinsics, file_prefix);
    }else{
        if (b_eval_antman){
            evaluateAntmanExt(v_extrinsics, file_prefix);
        }else{
            evaluateKaleidoExt(v_extrinsics, file_prefix);
        }
    }
    return 0;
}