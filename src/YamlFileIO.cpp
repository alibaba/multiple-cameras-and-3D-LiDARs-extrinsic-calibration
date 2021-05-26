#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <yaml-cpp/yaml.h>

#include "YamlFileIO.h"


namespace common{


bool loadExtFileOpencv(const std::string &file, Eigen::Matrix4d &T)
{
    cv::FileStorage fs;
    try
    {
        fs.open(file, cv::FileStorage::READ);

    }catch(cv::Exception &ex){
        std::cout << std::string(ex.what());
        return false;
    }catch(...){
        std::cout << "Fail to read " << file << std::endl;
        return false;
    }

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    cv::Mat rvec, tvec;
    fs["extrinsic_rotation"] >> rvec;
    fs["extrinsic_translation"] >> tvec;
    cv::cv2eigen(rvec, R);
    cv::cv2eigen(tvec, t);
    fs.release();

    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return true;
}

void saveExtFileOpencv(const std::string &file, const Eigen::Matrix4d &T)
{
    cv::FileStorage fs(file, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        std::cout << "Fail to write " << file << std::endl;
        return;
    }

    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    cv::Mat rvec, tvec;
    cv::eigen2cv(R, rvec);
    cv::eigen2cv(t, tvec);
    fs << "extrinsic_rotation" << rvec;
    fs << "extrinsic_translation" << tvec;

    fs.release();
}

bool loadIntrinFileKalibr(const std::string &file, 
                        cv::Mat &K, 
                        cv::Mat &D, 
                        std::string &distort_type, 
                        int &img_width, 
                        int &img_height)
{
    if (file.empty())
    {
        std::cout << "[loadIntrinFileKalibr] Empty input file name!\n";
        return false;
    }

    YAML::Node root_node;
    YAML::Node cam0_node;
    try
    {
        root_node = YAML::LoadFile(file);
    }
    catch (YAML::BadFile &e)
    {
        std::cout << "[loadIntrinFileKalibr] Could not open file: " << file << "\n";
        return false;
    }
    catch (YAML::ParserException &e)
    {
        std::cout << "[loadIntrinFileKalibr] Invalid file format: " << file << "\n";
        return false;
    }
    if (root_node.IsNull())
    {
        std::cout << "[loadIntrinFileKalibr] Could not open file: " << file << "\n";
        return false;
    }

    cam0_node = root_node["cam0"];
    std::string distortion_type = cam0_node["distortion_model"].as<std::string>();;

    double cam0_distortion[5] = {0.0};
    if(distortion_type == "equidistant")
    {
        cam0_distortion[0] = cam0_node["distortion_coeffs"][0].as<double>();
        cam0_distortion[1] = cam0_node["distortion_coeffs"][1].as<double>();
        cam0_distortion[2] = cam0_node["distortion_coeffs"][2].as<double>();
        cam0_distortion[3] = cam0_node["distortion_coeffs"][3].as<double>();
        D = (cv::Mat_<double>(1, 4) << cam0_distortion[0], cam0_distortion[1],
                cam0_distortion[2], cam0_distortion[3]);
        // revise letter spelling
        distort_type = "equi-distant";
    }
    if (distortion_type == "radtan")
    {
        cam0_distortion[0] = cam0_node["distortion_coeffs"][0].as<double>();
        cam0_distortion[1] = cam0_node["distortion_coeffs"][1].as<double>();
        cam0_distortion[2] = cam0_node["distortion_coeffs"][2].as<double>();
        cam0_distortion[3] = cam0_node["distortion_coeffs"][3].as<double>();
        cam0_distortion[4] = cam0_node["distortion_coeffs"][4].as<double>();
        D = (cv::Mat_<double>(1, 5) << cam0_distortion[0], cam0_distortion[1],
                cam0_distortion[2], cam0_distortion[3], cam0_distortion[4]);
        
        distort_type = "radtan";
    }
    Eigen::Matrix3d cam0_intrinsic = Eigen::Matrix3d::Identity();
    cam0_intrinsic(0, 0) = cam0_node["intrinsics"][0].as<double>();
    cam0_intrinsic(1, 1) = cam0_node["intrinsics"][1].as<double>();
    cam0_intrinsic(0, 2) = cam0_node["intrinsics"][2].as<double>();
    cam0_intrinsic(1, 2) = cam0_node["intrinsics"][3].as<double>();

    img_width = cam0_node["resolution"][0].as<int>();
    img_height = cam0_node["resolution"][1].as<int>();
    cv::eigen2cv(cam0_intrinsic, K);

    std::cout << "[loadIntrinFileKalibr] distortion type: " << distort_type << std::endl;
    std::cout << "[loadIntrinFileKalibr] cam0 intrinsic : \n"
              << cam0_intrinsic << "\n";
    std::cout << "[loadIntrinFileKalibr] cam0 distrotion : " << cam0_distortion[0] << ", " << cam0_distortion[3] << "\n";
    return true;
}

bool loadIntrinFileOpencv(const std::string &file_name, 
                        cv::Mat &K, 
                        cv::Mat &D, 
                        std::string &distort_type, 
                        int &img_width, 
                        int &img_height)
{
    if (file_name.empty())
    {
        std::cout << "[loadIntrinFileOpencv] Empty input file name " << file_name << "\n";
        return false;
    }

    cv::FileStorage fsCamera;
    try
    {
        fsCamera.open(file_name, cv::FileStorage::READ);

    }catch(cv::Exception &ex){
        std::cout << std::string(ex.what());
        return false;
    }catch(...){
        std::cout << "invalid camera calibration file: " << file_name << "\n";
        return false;
    }

    img_width = (int)fsCamera["image_width"];
    img_height = (int)fsCamera["image_height"];
    std::string distortion_type = (std::string)fsCamera["camera_model"];
    if (distortion_type == "equidistant")
        distort_type = "equi-distant";
    if (distortion_type == "radtan")
        distort_type = "radtan";
        
    fsCamera["camera_matrix"] >> K;
    fsCamera["distortion_coefficients"] >> D;
    std::cout << "camera_matrix : \n"
              << K << "\n";
    std::cout << "distortion_coefficients : \n"
              << D << "\n";

    fsCamera.release();

    return true;
}

bool saveIntrinFileOpencv(const std::string &file_name, 
                        const cv::Mat &K, 
                        const cv::Mat &D, 
                        const std::string &distort_type, 
                        const int img_width, 
                        const int img_height)
{
    if (file_name.empty())
    {
        std::cout << "[saveIntrinFileOpencv] Empty file name!\n";
        return false;
    }

    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        std::cout << "[saveIntrinFileOpencv] Fail to open " << file_name << "\n";
        return false;
    }

    fs << "camera_model" << distort_type;
    fs << "camera_matrix" << K;
    fs << "distortion_coefficients" << D;

    fs << "avg_reprojection_error" << 1.2799437834241592e-01;
    fs << "image_width" << img_width;
    fs << "image_height" << img_height;
    fs << "serial_num" <<  1;
    fs.release();
    return true;
}

bool loadCCTagResultFile(const std::string &filepath,
                    std::vector<std::string> &v_img_path,
                    std::vector<std::vector<cv::Point2d>> &vv_kpts)
{
    if (filepath.empty())
    {
        std::cout << " Empty file name!\n";
        return false;
    }

    YAML::Node frm_node;
    try
    {
        frm_node = YAML::LoadFile(filepath);
    }
    catch (YAML::BadFile &e)
    {
        std::cout << " Could not open file: " << filepath << "\n";
        return false;
    }
    catch (YAML::ParserException &e)
    {
        std::cout << " Invalid file format: " << filepath << "\n";
        return false;
    }
    if (frm_node.IsNull())
    {
        std::cout << " Could not open file: " << filepath << "\n";
        return false;
    }

    v_img_path.clear();
    vv_kpts.clear();

    // parse FramePoses
    for (YAML::const_iterator it = frm_node.begin(); it != frm_node.end(); ++it)
    {
        int f = it->first.as<int>();
        std::string id = std::to_string(f);

        // image path
        std::string img_path = frm_node[id]["image_file"].as<std::string>();
        std::vector<cv::Point2d> v_keypoints;
        for (size_t k = 0; k < frm_node[id]["keypoints"].size(); ++k)
        {
            cv::Point2d keypoint;
            keypoint.x = frm_node[id]["keypoints"][k][0].as<double>();
            keypoint.y = frm_node[id]["keypoints"][k][1].as<double>();
            v_keypoints.emplace_back(keypoint);
        }

        std::cout<< " Frame " << id << " detect " << v_keypoints.size()
                  << " cctag keypoints.\n";
        v_img_path.emplace_back(img_path);
        vv_kpts.emplace_back(v_keypoints);
    }
    std::cout<< " read " << v_img_path.size() << " frames! \n";

    return true;
}
}
