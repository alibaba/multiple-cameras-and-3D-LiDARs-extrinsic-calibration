#include <chrono>
#include <memory>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "common.h"
#include "stereo_calibration.h"
#include "target_board.h"
#include "relocalizer.h"
#include "params_config.h"
#include "cost_function.h"

StereoCalibrator::StereoCalibrator(camera::model_type_t camModelType,
                                   target::target_type_t targetModelType,
                                   const std::string &leftIntrinsicFileName,
                                   const std::string &rightIntrinsicFileName,
                                   const std::string &targetFileName,
                                   const bool b_homogeneous) : b_homo_multicamera_(b_homogeneous)
{
    target_ptr_ = std::make_shared<target::TargetBoard>(targetFileName, targetModelType);
    v_object_points_ = target_ptr_->objectPoints();

    if (b_homo_multicamera_)
    {
        int img_width = readImageWidth(leftIntrinsicFileName);
        int img_height = readImageHeight(leftIntrinsicFileName);
        std::shared_ptr<Relocalizer> relocalizer = std::make_shared<Relocalizer>(v_object_points_,
                                                                                 ParamsConfig::GetPnPMinInlierNum(),
                                                                                 ParamsConfig::GetPnPMinMatchesNum(),
                                                                                 img_width, img_height);
        left_calibrator_ptr_ = std::make_shared<MonoCalibrator>(camModelType, leftIntrinsicFileName, v_object_points_, relocalizer);
        right_calibrator_ptr_ = std::make_shared<MonoCalibrator>(camModelType, rightIntrinsicFileName, v_object_points_, relocalizer);
    }
    else
    {
        left_calibrator_ptr_ = std::make_shared<MonoCalibrator>(camModelType, targetModelType, leftIntrinsicFileName, targetFileName);
        right_calibrator_ptr_ = std::make_shared<MonoCalibrator>(camModelType, targetModelType, rightIntrinsicFileName, targetFileName);
    }
}

bool StereoCalibrator::addCctagData(const std::vector<cv::Point2d> &v_left_centers, const std::vector<cv::Point2d> &v_right_centers)
{
    bool sts = left_calibrator_ptr_->addCctagData(v_left_centers);
    if (!sts)
    {
        ERROR_STREAM("[StereoCalibrator::addCctagData] Fail to add CCTAG centers in  left camera!");
        return false;
    }
    sts = right_calibrator_ptr_->addCctagData(v_right_centers);
    if (!sts)
    {
        ERROR_STREAM("[StereoCalibrator::addCctagData] Fail to add CCTAG centers in right camera!");
        return false;
    }

    return true;
}

bool StereoCalibrator::addCctagData(const std::string &leftImgFilePath, const std::vector<cv::Point2d> &v_left_centers,
                                    const std::string &rightImgFilePath, const std::vector<cv::Point2d> &v_right_centers)
{
    bool sts = left_calibrator_ptr_->addCctagData(leftImgFilePath, v_left_centers);
    if (!sts)
    {
        ERROR_STREAM("[StereoCalibrator::addCctagData] Failed to add cctag data in left camera!");
        return false;
    }
    sts = right_calibrator_ptr_->addCctagData(rightImgFilePath, v_right_centers);
    if (!sts)
    {
        ERROR_STREAM("[StereoCalibrator::addCctagData] Failed to add cctag data in right camera!");
        return false;
    }

    return true;
}

camera::Camera::Ptr &StereoCalibrator::leftCamera(void)
{
    return left_calibrator_ptr_->camera();
}

camera::Camera::Ptr &StereoCalibrator::rightCamera(void)
{
    return right_calibrator_ptr_->camera();
}

int StereoCalibrator::readImageWidth(const std::string &filename)
{
    if (filename.empty())
    {
        ERROR_STREAM("[StereoCalibrator::readImageWidth] Empty file name!");
        return -1;
    }

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        ERROR_STREAM("[StereoCalibrator::readImageWidth] Fail to open file: " + filename);
        return -1;
    }

    int width = (int)fs["image_width"];
    fs.release();
    return width;
}

int StereoCalibrator::readImageHeight(const std::string &filename)
{
    if (filename.empty())
    {
        ERROR_STREAM("[StereoCalibrator::readImageHeight] Empty file name!");
        return -1;
    }

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        ERROR_STREAM("[StereoCalibrator::readImageHeight] Fail to open file: " + filename);
        return -1;
    }

    int height = (int)fs["image_height"];
    fs.release();
    return height;
}

int StereoCalibrator::sampleCount(void) const
{
    return samples_num_;
}

const std::vector<Eigen::Vector3d> StereoCalibrator::objectPoints() const
{
    return v_object_points_;
}

std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> StereoCalibrator::validCameraPosePairs(void) const
{
    std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> v_cam_poses_pair;
    for (int i = 0; i < samples_num_; ++i)
    {
        if (!left_calibrator_ptr_->isValid(i) || !right_calibrator_ptr_->isValid(i))
            continue;

        Eigen::Matrix4d left_pose = left_calibrator_ptr_->cameraPose(i);
        Eigen::Matrix4d right_pose = right_calibrator_ptr_->cameraPose(i);

        v_cam_poses_pair.emplace_back(std::make_pair(left_pose, right_pose));
    }
    return v_cam_poses_pair;
}

bool StereoCalibrator::writeExtrinsicParams(const std::string &directory) const
{
    if (directory.empty())
    {
        ERROR_STREAM("[StereoCalibrator::writeExtrinsicParams] Empty directory");
        return false;
    }

    std::string file_stem;
    if (b_homo_multicamera_)
        file_stem = "stereo0_to_stereo1.yml";
    else
        file_stem = "stereo0_to_teche0.yml";

    std::string file_name;
    if (directory.back() == '/')
        file_name = directory + file_stem;
    else
        file_name = directory + "/" + file_stem;

    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        ERROR_STREAM("[StereoCalibrator::writeExtrinsicParams] Fail to open " << file_name);
        return false;
    }
    cv::Mat rotation, translation;
    Eigen::Matrix3d R = T_r_l_.block<3, 3>(0, 0);
    Eigen::Vector3d t = T_r_l_.block<3, 1>(0, 3);
    cv::eigen2cv(R, rotation);
    cv::eigen2cv(t, translation);

    fs << "extrinsic_rotation" << rotation;
    fs << "extrinsic_translation" << translation;
    fs.release();

    return true;
}

Eigen::Matrix4d StereoCalibrator::validExtrinsics() const
{
    return T_r_l_;
}

bool StereoCalibrator::calibrate(void)
{
    DEBUG_STREAM("[StereoCalibrator::calibrate] ---------------- calibrate left camera ------------------");
    if (!left_calibrator_ptr_->calibrate())
    {
        ERROR_STREAM("[StereoCalibrator::calibrate] Fail to calibrate left camera!");
        return false;
    }

    DEBUG_STREAM("[StereoCalibrator::calibrate] ---------------- calibrate right camera ------------------");
    if (!right_calibrator_ptr_->calibrate())
    {
        ERROR_STREAM("[StereoCalibrator::calibrate] Fail to calibrate right camera!");
        return false;
    }

    size_t left_img_num = left_calibrator_ptr_->imagePoints().size();
    size_t right_img_num = right_calibrator_ptr_->imagePoints().size();
    assert(left_img_num == right_img_num && "Left and right image number is inconsistent!");

    samples_num_ = left_img_num;

    // find best estimate of extrinsic between left and right camera
    double min_reprojection_err = std::numeric_limits<double>::max();
    for (size_t i = 0; i < static_cast<size_t>(samples_num_); ++i)
    {
        if (!left_calibrator_ptr_->isValid(i) || !right_calibrator_ptr_->isValid(i))
        {
            ERROR_STREAM("[StereoCalibrator::calibrate] Stereo Frame " << i << " has invalid camera pose!");
            continue;
        }
        Eigen::Matrix4d T_l_wc = left_calibrator_ptr_->cameraPose(i);
        Eigen::Matrix4d T_r_wc = right_calibrator_ptr_->cameraPose(i);
        Eigen::Matrix4d T_r_l = T_r_wc.inverse() * T_l_wc;

        double reproj_err = 0.0;
        unsigned int reproj_cnt = 0;
        for (size_t j = 0; j < right_img_num; ++j)
        {
            if (!left_calibrator_ptr_->isValid(j) || !right_calibrator_ptr_->isValid(j))
            {
                ERROR_STREAM("[StereoCalibrator::calibrate] Stereo Frame " << j << " has invalid camera pose!");
                continue;
            }
            T_l_wc = left_calibrator_ptr_->cameraPose(j);
            Eigen::Matrix4d T_r_cw_tmp = T_r_l * T_l_wc.inverse();
            Eigen::Matrix3d R_r_cw = T_r_cw_tmp.block<3, 3>(0, 0);
            Eigen::Vector3d t_r_cw = T_r_cw_tmp.block<3, 1>(0, 3);

            std::vector<Eigen::Vector3d> &v_scene_points = right_calibrator_ptr_->inlierObjectPoints(j);
            std::vector<cv::Point2d> &v_img_points = right_calibrator_ptr_->inlierImagePoints(j);
            reproj_err += rightCamera()->reprojectionError(v_scene_points,
                                                           v_img_points,
                                                           R_r_cw, t_r_cw);
            reproj_cnt++;
        }

        reproj_err /= reproj_cnt;
        if (reproj_err < min_reprojection_err)
        {
            min_reprojection_err = reproj_err;
            T_r_l_ = T_r_l;
            DEBUG_STREAM("[StereoCalibrator::calibrate] mininal average reprojection error : " << reproj_err);
        }
    }
    //! optimized variables

    // camera pose of rig base
    // here we choose the left camera as rig base
    double opt_transform_base[samples_num_ * 6] = {0.0};
    // camera pose of left camera, reltive to rig
    double opt_transform_cam0[6] = {0.0};
    // camera pose of right camera, reltive to rig
    double opt_transform_cam1[6] = {0.0};

    // print initial reprojection error
    {
        Eigen::Matrix3d rotation = T_r_l_.block<3, 3>(0, 0);
        Eigen::Vector3d eulers = rotation.eulerAngles(2, 0, 1);
        Eigen::Vector3d trans = T_r_l_.block<3, 1>(0, 3);
        DEBUG_STREAM("[StereoCalibrator::calibrate] Initial extrinsics: "
                     << "r: " << eulers(0) << "  p: " << eulers(1) << "  yaw: " << eulers(2)
                     << " x: " << trans(0) << "  y: " << trans(1) << "  z: " << trans(2));

        // initialize extrinsic params
        ceres::RotationMatrixToAngleAxis(rotation.data(), opt_transform_cam1);
        opt_transform_cam1[3] = trans[0];
        opt_transform_cam1[4] = trans[1];
        opt_transform_cam1[5] = trans[2];

        unsigned int valid_frame_count = 0;
        double left_cam_error = 0.0, right_cam_error = 0.0;
        for (size_t i = 0; i < static_cast<size_t>(samples_num_); ++i)
        {
            if (!left_calibrator_ptr_->isValid(i) || !right_calibrator_ptr_->isValid(i))
                continue;

            Eigen::Matrix4d T_l_wc = left_calibrator_ptr_->cameraPose(i);
            Eigen::Matrix4d T_l_cw = T_l_wc.inverse();
            Eigen::Matrix3d R_l_cw = T_l_cw.block<3, 3>(0, 0);
            Eigen::Vector3d t_l_cw = T_l_cw.block<3, 1>(0, 3);

            // initialize left camera pose(c2w)
            ceres::RotationMatrixToAngleAxis(R_l_cw.data(), opt_transform_base + 6 * i);
            opt_transform_base[6 * i + 3] = t_l_cw[0];
            opt_transform_base[6 * i + 4] = t_l_cw[1];
            opt_transform_base[6 * i + 5] = t_l_cw[2];

            std::vector<Eigen::Vector3d> &v_left_scene_points = left_calibrator_ptr_->inlierObjectPoints(i);
            std::vector<cv::Point2d> &v_left_img_points = left_calibrator_ptr_->inlierImagePoints(i);
            left_cam_error += leftCamera()->reprojectionError(v_left_scene_points,
                                                              v_left_img_points,
                                                              R_l_cw, t_l_cw);
            Eigen::Matrix4d T_r_cw_tmp = T_r_l_ * T_l_cw;
            Eigen::Matrix3d R_r_cw = T_r_cw_tmp.block<3, 3>(0, 0);
            Eigen::Vector3d t_r_cw = T_r_cw_tmp.block<3, 1>(0, 3);

            std::vector<Eigen::Vector3d> &v_right_scene_points = right_calibrator_ptr_->inlierObjectPoints(i);
            std::vector<cv::Point2d> &v_right_img_points = right_calibrator_ptr_->inlierImagePoints(i);
            right_cam_error += rightCamera()->reprojectionError(v_right_scene_points,
                                                                v_right_img_points,
                                                                R_r_cw, t_r_cw);
            valid_frame_count++;
        }
        DEBUG_STREAM("[StereoCalibrator::calibrate] Left camera Initial reprojection error: " << left_cam_error / valid_frame_count << " pixels");
        DEBUG_STREAM("[StereoCalibrator::calibrate] Right camera Initial reprojection error: " << right_cam_error / valid_frame_count << " pixels");
    }

    ceres::Problem problem;

    for (size_t i = 0; i < static_cast<size_t>(samples_num_); ++i)
    {
        if (!left_calibrator_ptr_->isValid(i) || !right_calibrator_ptr_->isValid(i))
            continue;

        // optimize left camera
        {
            const std::vector<cv::Point2d> &v_obs_kpts = left_calibrator_ptr_->inlierImagePoints(i);
            const std::vector<Eigen::Vector3d> &v_world_points = left_calibrator_ptr_->inlierObjectPoints(i);
            const Eigen::Matrix3d &intrinsic = leftCamera()->intrinsic();
            for (size_t j = 0; j < v_obs_kpts.size(); ++j)
            {
                ceres::CostFunction *cost_function = StereoReprojectionError2::Create(v_world_points[j], v_obs_kpts[j], intrinsic);
                problem.AddResidualBlock(cost_function, NULL, opt_transform_base + 6 * i, opt_transform_cam0);
            }

            problem.SetParameterBlockConstant(opt_transform_cam0);
        }
        // optimize right camera
        {
            const std::vector<cv::Point2d> &v_obs_kpts = right_calibrator_ptr_->inlierImagePoints(i);
            const std::vector<Eigen::Vector3d> &v_world_points = right_calibrator_ptr_->inlierObjectPoints(i);
            const Eigen::Matrix3d &intrinsic = rightCamera()->intrinsic();
            for (size_t j = 0; j < v_obs_kpts.size(); ++j)
            {
                ceres::CostFunction *cost_function = StereoReprojectionError2::Create(v_world_points[j], v_obs_kpts[j], intrinsic);
                problem.AddResidualBlock(cost_function, NULL, opt_transform_base + 6 * i, opt_transform_cam1);
            }
        }
    }

    bool sts = ceresOptimize(problem, 1e-8);
    if (!sts)
    {
        ERROR_STREAM("[StereoCalibrator::calibrate] Fail to optimize");
        return false;
    }

    // optimized extrinsic
    Eigen::Matrix3d R_r_l = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_r_l = Eigen::Vector3d::Zero();
    ceres::AngleAxisToRotationMatrix(opt_transform_cam1, R_r_l.data());
    t_r_l[0] = opt_transform_cam1[3];
    t_r_l[1] = opt_transform_cam1[4];
    t_r_l[2] = opt_transform_cam1[5];
    DEBUG_STREAM("[StereoCalibrator::calibrate] Extrinsic R : \n"
                 << R_r_l);
    DEBUG_STREAM("[StereoCalibrator::calibrate] Extrinsic t : " << t_r_l.transpose());
    T_r_l_.block<3, 3>(0, 0) = R_r_l;
    T_r_l_.block<3, 1>(0, 3) = t_r_l;

    for (size_t i = 0; i < static_cast<size_t>(samples_num_); ++i)
    {
        if (!left_calibrator_ptr_->isValid(i) || !right_calibrator_ptr_->isValid(i))
            continue;

        Eigen::Matrix3d rotation_base = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_base = Eigen::Vector3d::Zero();
        unsigned int tmp_inlier_num;

        ceres::AngleAxisToRotationMatrix(opt_transform_base + 6 * i, rotation_base.data());
        t_base[0] = opt_transform_base[6 * i + 3];
        t_base[1] = opt_transform_base[6 * i + 4];
        t_base[2] = opt_transform_base[6 * i + 5];

        Eigen::Matrix3d left_intrinsic = leftCamera()->intrinsic();
        Eigen::Matrix3d right_intrinsic = rightCamera()->intrinsic();
        // check left camera optimized result
        {
            Eigen::Matrix3d R_cam0 = Eigen::Matrix3d::Identity();
            Eigen::Vector3d t_cam0 = Eigen::Vector3d::Zero();
            ceres::AngleAxisToRotationMatrix(opt_transform_cam0, R_cam0.data());
            t_cam0[0] = opt_transform_cam0[3];
            t_cam0[1] = opt_transform_cam0[4];
            t_cam0[2] = opt_transform_cam0[5];
            t_cam0 = R_cam0 * t_base + t_cam0;
            R_cam0 = R_cam0 * rotation_base;
            // DEBUG_STREAM("[StereoCalibrator::calibrate] Camera 0 optimized R : \n"
            //              << R_cam0);

            double mean_reproject_error = 0;
            double origin_mean_reproject_error = left_calibrator_ptr_->meanReprojectionError(i);
            // check inliers after optimization
            const std::vector<Eigen::Vector3d> &v_world_points = left_calibrator_ptr_->inlierObjectPoints(i);
            const std::vector<cv::Point2d> &v_obs_keypoints = left_calibrator_ptr_->inlierImagePoints(i);

            tmp_inlier_num = MonoCalibrator::checkInliers(v_world_points, v_obs_keypoints, left_intrinsic, R_cam0, t_cam0, mean_reproject_error);

            unsigned int origin_inlier_num = v_world_points.size();

            if (tmp_inlier_num * 10 > origin_inlier_num * 7 || mean_reproject_error < origin_mean_reproject_error)
            {
                sts = true;
                Eigen::Matrix4d T_cw_opt = Eigen::Matrix4d::Identity();
                T_cw_opt.block<3, 3>(0, 0) = R_cam0;
                T_cw_opt.block<3, 1>(0, 3) = t_cam0;

                left_calibrator_ptr_->cameraPose(i) = T_cw_opt.inverse();
                left_calibrator_ptr_->meanReprojectionError(i) = mean_reproject_error;
            }
            else
            {
                sts = false;
                ERROR_STREAM("[MonoCalibrator::optimize] Optimize frame " << i << " left camera failed!");
            }
            ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " left camera original inlier num: " << origin_inlier_num);
            ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " left camera Optimized inlier num: " << tmp_inlier_num);
            ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " left camera Original mean reprojection error : " << origin_mean_reproject_error);
            ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " left camera Optimized mean reprojection error : " << mean_reproject_error);
        }
        // check right camera optimized result
        {
            Eigen::Matrix3d R_cam1 = Eigen::Matrix3d::Identity();
            Eigen::Vector3d t_cam1 = t_r_l;
            t_cam1 = R_r_l * t_base + t_cam1;
            R_cam1 = R_r_l * rotation_base;
            // DEBUG_STREAM("[StereoCalibrator::calibrate] Camera 1 optimized R : \n"
            //              << R_cam1);

            double mean_reproject_error = 0;
            double origin_mean_reproject_error = right_calibrator_ptr_->meanReprojectionError(i);
            // check inliers after optimization
            const std::vector<Eigen::Vector3d> &v_world_points = right_calibrator_ptr_->inlierObjectPoints(i);
            const std::vector<cv::Point2d> &v_obs_keypoints = right_calibrator_ptr_->inlierImagePoints(i);

            tmp_inlier_num = MonoCalibrator::checkInliers(v_world_points, v_obs_keypoints, right_intrinsic, R_cam1, t_cam1, mean_reproject_error);

            unsigned int origin_inlier_num = v_world_points.size();

            if (tmp_inlier_num * 10 > origin_inlier_num * 7 || mean_reproject_error < origin_mean_reproject_error)
            {
                sts = true;
                Eigen::Matrix4d T_cw_opt = Eigen::Matrix4d::Identity();
                T_cw_opt.block<3, 3>(0, 0) = R_cam1;
                T_cw_opt.block<3, 1>(0, 3) = t_cam1;

                right_calibrator_ptr_->cameraPose(i) = T_cw_opt.inverse();
                right_calibrator_ptr_->meanReprojectionError(i) = mean_reproject_error;
            }
            else
            {
                sts = false;
                ERROR_STREAM("[MonoCalibrator::optimize] Optimize frame " << i << " right camera failed!");
            }
            ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " right camera original inlier num: " << origin_inlier_num);
            ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " right camera optimized inlier num: " << tmp_inlier_num);
            ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " right camera original mean reprojection error : " << origin_mean_reproject_error);
            ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " right camera optimized mean reprojection error : " << mean_reproject_error);
        }
    }

    return sts;
}
