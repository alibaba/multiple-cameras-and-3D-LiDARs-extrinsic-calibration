#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "YamlFileIO.h"


namespace common{


Eigen::Matrix4d loadExtFileOpencv(const std::string &file)
{
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "Fail to read " << file << std::endl;
        return Eigen::Matrix4d::Zero();
    }

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    cv::Mat rvec, tvec;
    fs["extrinsic_rotation"] >> rvec;
    fs["extrinsic_translation"] >> tvec;
    cv::cv2eigen(rvec, R);
    cv::cv2eigen(tvec, t);
    fs.release();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
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

}
