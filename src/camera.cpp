#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cassert>
#include <limits>

#include "camera.h"
#include "common.h"
#include "YamlFileIO.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace camera
{
Camera::Camera(const std::string &file_name,
               const setup_type_t setup_type,
               const model_type_t model_type) : cam_name_(file_name),
                                                setup_type_(setup_type),
                                                model_type_(model_type)
{
    readIntrinsicFile(file_name);
}

bool Camera::readIntrinsicFile(const std::string &file_name)
{
    if (file_name.empty())
    {
        ERROR_STREAM("[readIntrinsicFile] Empty file name!");
        return false;
    }

    intrinsic_ = Eigen::Matrix3d::Identity();
    distortion_.resize(4);

    cv::Mat intrin_mat, distortion_mat;
    int img_width, img_height;
    std::string distortion_type;
    if (!common::loadIntrinFileOpencv(file_name, intrin_mat, distortion_mat, distortion_type, img_width, img_height)){
        if(!common::loadIntrinFileKalibr(file_name, intrin_mat, distortion_mat, distortion_type, img_width, img_height)){
            return false;
        }
    }
    
    cv::cv2eigen(intrin_mat, intrinsic_);
    distortion_(0) = distortion_mat.at<double>(0, 0);
    distortion_(1) = distortion_mat.at<double>(0, 1);
    distortion_(2) = distortion_mat.at<double>(0, 2);
    distortion_(3) = distortion_mat.at<double>(0, 3);
    img_width_ = img_width;
    img_height_ = img_height;

    // cv::FileStorage fs(file_name, cv::FileStorage::READ);
    // if (!fs.isOpened())
    // {
    //     ERROR_STREAM("[readIntrinsicFile] Fail to open " << file_name);
    //     return false;
    // }

    // img_width_ = (int)fs["image_width"];
    // img_height_ = (int)fs["image_height"];

    // cv::Mat intrin_mat;
    // cv::Mat distortion_mat;
    // fs["camera_matrix"] >> intrin_mat;
    // fs["distortion_coefficients"] >> distortion_mat;

    // // intrinsic_num_ = std::max(distortion_mat.rows, distortion_mat.cols);
    // cv::cv2eigen(intrin_mat, intrinsic_);
    // distortion_(0) = distortion_mat.at<double>(0, 0);
    // distortion_(1) = distortion_mat.at<double>(0, 1);
    // distortion_(2) = distortion_mat.at<double>(0, 2);
    // distortion_(3) = distortion_mat.at<double>(0, 3);

    // fs.release();

    return true;
}

bool Camera::writeIntrinsicFile(const std::string &file_name)
{
    if (file_name.empty())
    {
        ERROR_STREAM("[writeIntrinsicFile] Empty file name!");
        return false;
    }

    cv::Mat intrin_mat, distortion_mat;
    cv::eigen2cv(intrinsic_, intrin_mat);
    cv::eigen2cv(distortion_, distortion_mat);

    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        ERROR_STREAM("[writeIntrinsicFile] Fail to open " << file_name);
        return false;
    }

    fs << "image_width" << img_width_;
    fs << "image_height" << img_height_;

    fs << "camera_matrix" << intrin_mat;
    fs << "distortion_coefficients" << distortion_mat;
    fs.release();

    return true;
}

std::vector<cv::Point2d> Camera::filteKeypoints(const std::vector<cv::Point2d> &input, unsigned int img_bound)
{
    if (input.empty())
    {
        ERROR_STREAM("[Camera::filteKeypoints] Empty input keypoints vector!");
        return std::vector<cv::Point2d>();
    }

    const unsigned int width_bound = img_width_ - img_bound;
    const unsigned int height_bound = img_height_ - img_bound;

    std::vector<cv::Point2d> output;

    for (size_t i = 0; i < input.size(); ++i)
    {
        cv::Point2d kpt = input[i];

        if (img_bound < kpt.x && kpt.x < width_bound && img_bound < kpt.y && kpt.y < height_bound)
        {
            output.emplace_back(kpt);
        }
    }

    if (output.empty())
    {
        ERROR_STREAM("[Camera::filteKeypoints] Empty output keypoints vector!");
    }

    return output;
}

double Camera::reprojectionError(const std::vector<Eigen::Vector3d> &v_obj_points,
                                 const std::vector<cv::Point2d> &v_img_kpts,
                                 const Eigen::Matrix3d &R,
                                 const Eigen::Vector3d &t)
{
    if (v_obj_points.empty() || v_img_kpts.empty())
    {
        ERROR_STREAM("[Camera::reprojectionError] Empty input !");
        return std::numeric_limits<double>::max();
    }

    assert(v_obj_points.size() == v_img_kpts.size());
    if (R.hasNaN() || t.hasNaN())
    {
        ERROR_STREAM("[Camera::reprojectionError] Invalid input rotation and translation!");
        return std::numeric_limits<double>::max();
    }

    const double eps = std::numeric_limits<double>::epsilon();
    double error_sum = 0.0;
    for (size_t i = 0; i < v_img_kpts.size(); i++)
    {
        const Eigen::Vector3d &obj_point = v_obj_points[i];
        const cv::Point2d &img_kpt = v_img_kpts[i];
        Eigen::Vector3d projected_point = R * obj_point + t;
        if (projected_point[2] < eps)
            continue;

        projected_point = intrinsic_ * projected_point;
        double x_err = img_kpt.x - projected_point[0] / projected_point[2];
        double y_err = img_kpt.y - projected_point[1] / projected_point[2];
        error_sum += std::sqrt(x_err * x_err + y_err * y_err);
    }
    // DEBUG_STREAM("[Camera::reprojectionError] Reprojection error sum: " << error_sum);

    return (error_sum / v_img_kpts.size());
}
} // namespace camera