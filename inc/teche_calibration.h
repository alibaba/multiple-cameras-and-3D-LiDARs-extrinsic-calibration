/**
 * @file teche_calibration.h
 * @author fangchuan.fc (fangchuan.fc@alibaba-inc.com)
 * @brief This file is the header file of calibration for TECHE 360 ANYWHERE panorama camera.
 * We build the TECHE 360 panorama camera as the concentric axis rotated camera, which is formed by 4 cameras.
 * @date 2020-0618
 *
 * @copyright Copyright (c) Alibaba Inc 2020. All rights reserved.
 *
 */
#ifndef __TECHE_360_ANYWHERE_CALIBRATION_H__
#define __TECHE_360_ANYWHERE_CALIBRATION_H__

#include <vector>
#include <limits.h>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "relocalizer.h"
#include "camera.h"
#include "mono_calibration.h"

class TecheCalibrator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TecheCalibrator() = delete;

    /**
   * @brief Stereo camera calibrator, made up of the left camera and right camera.
   * @param camModelType is camera model type.
   * @param targetModelType is target board type.
   * @param leftIntrinsicFileName is left camera intrinsic file name.
   * @param rightIntrinsicFileName is right camera intrinsic file name.
   * @param targetFileName is target board config file name.
   * @param camera_num is the camera number(rotation angles).
   */
    explicit TecheCalibrator(camera::model_type_t camModelType,
                             target::target_type_t targetModelType,
                             const std::vector<std::string> &camIntrinsicFileNames,
                             const std::string &targetFileName,
                             const unsigned int camera_num);

    /**
   * @brief Add calibration data, calibration pattern is CCTAG.
   * @param imgFilePaths is calibration image file path, imgFilePath.size() == 4.
   * @param centers is extracted cctag centers, centers.size() == 4.
   * @return .
   */
    bool addCctagData(const std::vector<std::string> &imgFilePaths, const std::vector<std::vector<cv::Point2d>> &centers);

    /**
   * @brief Get map points.
   * @return vector of map points.
   */
    const std::vector<Eigen::Vector3d> objectPoints() const;

    /**
   * @brief Get camera pose in w2c if calibration successfully.
   * @return vector of camera pose.
   */
    std::vector<Eigen::Matrix4d> validCameraPoses() const;

    /**
   * @brief Get camera pose in c2r (camera to rotation axis) if calibration successfully.
   * @return vector of camera pose.
   */
    std::vector<Eigen::Matrix4d> validCameraPosesInRotationAxisFrame() const;

    /**
   * @brief Calibrate extrinsic between 4 fisheye camera .
   * @return .
   */
    bool calibrate();

    /**
   * @brief Calculate rotation axis center.
   * @param axis_center Ow camera center.
   * @return .
   */
    bool calcRotationAxisCenter(const std::vector<MonoCalibrator::Ptr> &v_calib_ptrs, Eigen::Vector3d &axis_center);

    /**
   * @brief Calculate camera pose in c2r.
   * @param global_rotation_axis Ow global rotation axis.
   * @return .
   */
    bool calcCameraPoser2c(const Eigen::Vector3d &global_rotation_axis);

private:
    int readImageWidth(const std::string &filename);

    int readImageHeight(const std::string &filename);

    //
    const unsigned int cam_num_;
    // calibrator pointer for 4 cameras
    std::vector<MonoCalibrator::Ptr> v_cam_calibrator_ptrs_;

    // target board pointer
    target::TargetBoard::Ptr target_ptr_;

    // scene map points
    std::vector<Eigen::Vector3d> v_object_points_;

    // rotation axis in world frame
    Eigen::Vector3d global_rotation_axis_;

    // camera pose  in r2c form( rotation axis 2 camera )
    // transform points of rotation axis frame into camera frame
    std::vector<Eigen::Matrix4d> v_cam_pose_r_c_;
};

#endif // __TECHE_360_ANYWHERE_CALIBR