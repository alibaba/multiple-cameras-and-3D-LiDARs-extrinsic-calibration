#ifndef __STEREO_FRAME_H
#define __STEREO_FRAME_H

#include <vector>
#include <string>
#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

// stereo frame composed of up and low frame
struct StereoFrame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // stereo frame index
    int id;
    // image file path
    std::array<std::string, 2> v_img_file_path;

    // 0 is left camera pose, 1 is right camera pose
    std::array<Eigen::Matrix4d, 2> v_T_wc;

    // extrinsic, from low camera to up camera
    Eigen::Matrix4d T_low_up;

    // // inlier keypoints in 2 cameras after pnp
    // std::array<std::vector<cv::Point2d>, 2> v_cams_keypoints;
    // // inlier landmarks observed in 2 cameras after pnp
    // std::array<std::vector<Eigen::Vector3d>, 2> v_cams_landmarks;

    //
    double v_mean_reprojection_error[2];
    // rotation axis in global frame
    Eigen::Vector3d global_rotation_axis;

    StereoFrame(const std::string &left_img_file,
                const std::string &right_img_file)
    {
        v_img_file_path[0] = left_img_file;
        v_img_file_path[1] = right_img_file;
        id = std::stoi(left_img_file.substr(left_img_file.find_last_of("_") + 1));
    }

    StereoFrame(const std::string &left_img_file,
                const std::string &right_img_file,
                int index) : id(index)
    {
        v_img_file_path[0] = left_img_file;
        v_img_file_path[1] = right_img_file;
    }

    StereoFrame(int index) : id(index)
    {
    }

    // raw cctag detection result
    std::vector<cv::Point2d> v_left_keypoints;
    std::vector<cv::Point2d> v_right_keypoints;

    // // forward compatible
    // bool setCameraPose(double *cameraPose)
    // {
    //     if (!cameraPose)
    //     {
    //         std::cout << "Empty camera pose data in  array!\n";
    //         return false;
    //     }

    //     Eigen::AngleAxisd rot_vec;
    //     Eigen::Vector3d &rot_vec_axis = rot_vec.axis();
    //     rot_vec_axis[0] = cameraPose[0];
    //     rot_vec_axis[1] = cameraPose[1];
    //     rot_vec_axis[2] = cameraPose[2];
    //     rot_vec.angle() = cameraPose[3];

    //     v_T_wc[0].block<3, 3>(0, 0) = rot_vec.matrix();
    //     v_T_wc[0](0, 3) = cameraPose[4];
    //     v_T_wc[0](1, 3) = cameraPose[5];
    //     v_T_wc[0](2, 3) = cameraPose[6];

    //     return true;
    // }
};

#endif