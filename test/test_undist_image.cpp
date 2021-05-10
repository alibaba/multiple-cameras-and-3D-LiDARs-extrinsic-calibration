#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <boost/filesystem.hpp>
#include <glog/logging.h>

#include "FileSystemTools.h"
#include "YamlFileIO.h"

struct CameraParam
{
    int image_width;
    int image_height;

    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
};

bool readIntrinsic(const std::string &file_name, CameraParam &cam_param)
{
    if (file_name.empty())
    {
        std::cout << "[readFileStorage] Empty input file name " << file_name << "\n";
        return false;
    }

    cv::FileStorage fsCamera(file_name, cv::FileStorage::READ);
    if (!fsCamera.isOpened())
    {
        std::cout << "invalid camera calibration file: " << file_name << "\n";
        return false;
    }

    cam_param.image_width = (int)fsCamera["image_width"];
    cam_param.image_height = (int)fsCamera["image_height"];

    fsCamera["camera_matrix"] >> cam_param.camera_matrix;
    fsCamera["distortion_coefficients"] >> cam_param.distortion_coefficients;
    std::cout << "camera_matrix : \n"
              << cam_param.camera_matrix << "\n";
    std::cout << "distortion_coefficients : \n"
              << cam_param.distortion_coefficients << "\n";

    fsCamera.release();

    return true;
}


int main(int argc, char **argv)
{
    if (argc < 4)
    {
        std::cerr << "Useage : test_undist_img <raw_img_folder> <intrinsic.yaml> <output_folder> <focal_scale>\n";
        return -1;
    }

    //! image path relative parameters
    std::string raw_img_folder(argv[1]);
    std::string intrin_filepath(argv[2]);
    std::string output_folder(argv[3]);
    double f_scale = 0.72;
    if (argc >= 5)
    {
        f_scale = std::stod(argv[4]);
    }

    if (!common::pathExists(raw_img_folder))
    {
        LOG(FATAL) << " Image folder " << raw_img_folder << " does not exit, quit...\n";
        return -1;
    }
    //! check and create undistorted_img image folder
    if (!common::pathExists(output_folder))
    {
        if(!common::createPath(output_folder))
        {
            LOG(FATAL) << "Fail to create " << output_folder << "\n";
            return -1;
        }
    }

    std::vector<std::string> v_img_paths;
    std::vector<std::string> paths;
    paths.push_back(raw_img_folder);
    common::getFileLists(paths, true, "jpg", &v_img_paths);

    //! read intrisnic file
    std::string distort_type;
    cv::Mat K, D;
    int img_width = -1, img_height = -1;
    // bool ret = common::loadIntrinFileKalibr(intrin_filepath, K, D, distort_type, img_width, img_height);
    bool ret = common::loadIntrinFileOpencv(intrin_filepath, K, D, distort_type, img_width, img_height);
    if (!ret)
    {
        return -1;
    }
    LOG(INFO) << " Total distorted image number is " << v_img_paths.size() << "\n";

    cv::Mat new_K = cv::Mat_<double>::eye(3,3);
    cv::Mat new_D = cv::Mat_<double>::zeros(1, 4);
    for (size_t i = 0; i < v_img_paths.size(); i++)
    {
        std::string img_path = v_img_paths[i];
        cv::Mat image = cv::imread(img_path, cv::IMREAD_UNCHANGED);
        if (image.empty())
        {
            LOG(FATAL) << "Fail to load the image: " << img_path << "\n";
            return -1;
        }

        cv::Mat undistorted_img;

        if (distort_type == "equi-distant")
        {
            cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1), map1, map2;
            // new_K.at<double>(0,0) = K.at<double>(0,0) * f_scale;
            // new_K.at<double>(1,1) = K.at<double>(0,0) * f_scale;
            // new_K.at<double>(0,2) = img_width / 2;
            // new_K.at<double>(1,2) = img_height / 2;
            new_K = K;
            cv::fisheye::initUndistortRectifyMap(K, D, R, new_K, cv::Size(img_width, img_height), CV_32FC1, map1, map2);
            cv::remap(image, undistorted_img, map1, map2, cv::InterpolationFlags::INTER_CUBIC);
        }
        if (distort_type == "radtan")
        {
            cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1), map1, map2;
            new_K = K;
            cv::initUndistortRectifyMap(K, D, R, new_K, cv::Size(img_width, img_height), CV_32FC1, map1, map2);
            cv::remap(image, undistorted_img, map1, map2, cv::InterpolationFlags::INTER_CUBIC);
        }

        std::string filepath, img_name;
        common::splitPathAndFilename(img_path, &filepath, &img_name);
        std::string undist_img_filepath = common::concatenateFolderAndFileName(output_folder, img_name); 
        LOG(INFO) << " Save undistorted_img image " << undist_img_filepath << "\n";

        // std::vector<int> compression_params;
        // compression_params.push_back(cv::ImwriteFlags::IMWRITE_JPEG_QUALITY);
        // compression_params.push_back(100);
        // cv::imwrite(undist_img_filepath, undistorted_img, compression_params);
        cv::imwrite(undist_img_filepath, undistorted_img);
    }

    // save new intrinsic file
    std::string intrin_file_name, filepath;
    common::splitPathAndFilename(intrin_filepath, &filepath, &intrin_file_name);
    std::string save_new_intrin_filepath = common::concatenateFolderAndFileName(output_folder, intrin_file_name);
    ret = common::saveIntrinFileOpencv(save_new_intrin_filepath, new_K, new_D, distort_type, img_width, img_height);
    if (!ret)
    {
        return -1;
    }
    return 0;
}
