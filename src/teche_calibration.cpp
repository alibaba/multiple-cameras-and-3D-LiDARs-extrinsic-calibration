#include "cost_function.h"
#include "params_config.h"
#include "YamlFileIO.h"
#include "teche_calibration.h"

TecheCalibrator::TecheCalibrator(camera::model_type_t camModelType,
                                 target::target_type_t targetModelType,
                                 const std::vector<std::string> &camIntrinsicFileNames,
                                 const std::string &targetFileName,
                                 const unsigned int camera_num) : cam_num_(camera_num)
{
    assert(camIntrinsicFileNames.size() == camera_num);

    target_ptr_ = std::make_shared<target::TargetBoard>(targetFileName, targetModelType);
    v_object_points_ = target_ptr_->objectPoints();

    int img_width = readImageWidth(camIntrinsicFileNames[0]);
    int img_height = readImageHeight(camIntrinsicFileNames[0]);
    std::shared_ptr<Relocalizer> relocalizer = std::make_shared<Relocalizer>(v_object_points_,
                                                                             ParamsConfig::GetPnPMinInlierNum(),
                                                                             ParamsConfig::GetPnPMinMatchesNum(),
                                                                             img_width, img_height);
    v_cam_calibrator_ptrs_.resize(cam_num_);
    for (size_t i = 0; i < cam_num_; ++i)
    {
        v_cam_calibrator_ptrs_[i] = std::make_shared<MonoCalibrator>(camModelType, camIntrinsicFileNames[i], v_object_points_, relocalizer);
    }
}

int TecheCalibrator::readImageWidth(const std::string &filename)
{
    if (filename.empty())
    {
        ERROR_STREAM("[TecheCalibrator::readImageWidth] Empty file name!");
        return -1;
    }

    cv::Mat intrin_mat, distortion_mat;
    int width, height;
    std::string distortion_type;
    if (!common::loadIntrinFileOpencv(filename, intrin_mat, distortion_mat, distortion_type, width, height)){
        if(!common::loadIntrinFileKalibr(filename, intrin_mat, distortion_mat, distortion_type, width, height)){
            return false;
        }
    }

    // cv::FileStorage fs(filename, cv::FileStorage::READ);
    // if (!fs.isOpened())
    // {
    //     ERROR_STREAM("[TecheCalibrator::readImageWidth] Fail to open file: " + filename);
    //     return -1;
    // }

    // int width = (int)fs["image_width"];
    // fs.release();
    return width;
}

int TecheCalibrator::readImageHeight(const std::string &filename)
{
    if (filename.empty())
    {
        ERROR_STREAM("[TecheCalibrator::readImageHeight] Empty file name!");
        return -1;
    }

    cv::Mat intrin_mat, distortion_mat;
    int width, height;
    std::string distortion_type;
    if (!common::loadIntrinFileOpencv(filename, intrin_mat, distortion_mat, distortion_type, width, height)){
        if(!common::loadIntrinFileKalibr(filename, intrin_mat, distortion_mat, distortion_type, width, height)){
            return false;
        }
    }

    // cv::FileStorage fs(filename, cv::FileStorage::READ);
    // if (!fs.isOpened())
    // {
    //     ERROR_STREAM("[TecheCalibrator::readImageHeight] Fail to open file: " + filename);
    //     return -1;
    // }

    // int height = (int)fs["image_height"];
    // fs.release();
    return height;
}

bool TecheCalibrator::addCctagData(const std::vector<std::string> &imgFilePaths, const std::vector<std::vector<cv::Point2d>> &centers)
{
    assert(imgFilePaths.size() == centers.size());
    assert(imgFilePaths.size() == cam_num_);

    bool sts = false;

    for (size_t i = 0; i < imgFilePaths.size(); ++i)
    {
        sts = v_cam_calibrator_ptrs_[i]->addCctagData(imgFilePaths[i], centers[i]);
        if (!sts)
        {
            ERROR_STREAM("[TecheCalibrator::addCctagData] Failed to add cctag data in left camera!");
            return false;
        }
    }

    return true;
}

const std::vector<Eigen::Vector3d> TecheCalibrator::objectPoints() const
{
    return v_object_points_;
}

std::vector<Eigen::Matrix4d> TecheCalibrator::validCameraPoses() const
{
    std::vector<Eigen::Matrix4d> v_results(v_cam_calibrator_ptrs_.size(), Eigen::Matrix4d::Identity());
    for (size_t i = 0; i < v_cam_calibrator_ptrs_.size(); ++i)
    {
        if (!v_cam_calibrator_ptrs_[i]->isValid(0))
            continue;

        v_results[i] = v_cam_calibrator_ptrs_[i]->cameraPose(0);
    }
    return v_results;
}

std::vector<Eigen::Matrix4d> TecheCalibrator::validCameraPosesInRotationAxisFrame() const
{
    DEBUG_STREAM("[TecheCalibrator::validCameraPosesInRotationAxisFrame] Get " << v_cam_pose_r_c_.size()
                                                                               << " camera poses relative to rotation axis");
    std::vector<Eigen::Matrix4d> v_results(v_cam_pose_r_c_.size(), Eigen::Matrix4d::Identity());
    for (size_t i = 0; i < v_cam_pose_r_c_.size(); ++i)
    {
        if (v_cam_pose_r_c_[i].hasNaN())
            continue;

        v_results[i] = v_cam_pose_r_c_[i];
    }
    return v_results;
}

bool TecheCalibrator::calcRotationAxisCenter(const std::vector<MonoCalibrator::Ptr> &v_calib_ptrs, Eigen::Vector3d &axis_center)
{
    if (v_calib_ptrs.empty())
    {
        std::cout << "[calcRotationAxisCenter] Empty input stereo frames\n";
        axis_center = Eigen::Vector3d::Zero();
        return false;
    }
    for (size_t i = 0; i < v_calib_ptrs.size(); ++i)
    {
        if (v_calib_ptrs[i]->cameraPose(0).hasNaN())
        {
            std::cout << "[calcRotationAxisCenter] Invalid camera pose!\n";
            axis_center = Eigen::Vector3d::Zero();
            return false;
        }
        Eigen::Vector3d cam_center = v_calib_ptrs[i]->cameraPose(0).block<3, 1>(0, 3);

        axis_center[0] += cam_center[0];
        axis_center[1] += cam_center[1];
        axis_center[2] += cam_center[2];
    }

    axis_center[0] /= (double)(v_calib_ptrs.size());
    axis_center[1] /= (double)(v_calib_ptrs.size());
    axis_center[2] /= (double)(v_calib_ptrs.size());
    std::cout << "[calcRotationAxisCenter] axis_center : " << axis_center.transpose() << std::endl;
    return true;
}

bool TecheCalibrator::calibrate()
{

    for (size_t i = 0; i < cam_num_; ++i)
    {
        DEBUG_STREAM("[TecheCalibrator::calibrate] ---------------- calibrate camera " << i << "------------------");
        if (!v_cam_calibrator_ptrs_[i]->calibrate())
        {
            ERROR_STREAM("[TecheCalibrator::calibrate] Fail to calibrate camera " << i);
            return false;
        }
    }

    // pose of camera 0 first image frame
    Eigen::Matrix4d T_wc_cam0 = v_cam_calibrator_ptrs_[0]->cameraPose(0);
    Eigen::Matrix4d T_cw_cam0 = T_wc_cam0.inverse();
    Eigen::Matrix3d R_wc_cam0 = T_wc_cam0.block<3, 3>(0, 0);
    Eigen::Vector3d t_wc_cam0 = T_wc_cam0.block<3, 1>(0, 3);
    Eigen::Matrix3d R_cw_cam0 = T_cw_cam0.block<3, 3>(0, 0);
    Eigen::Vector3d t_cw_cam0 = T_cw_cam0.block<3, 1>(0, 3);

    // use c2w transformation initialize motor rotation axis
    Eigen::Matrix4d T_wc_cam1 = v_cam_calibrator_ptrs_[1]->cameraPose(0);
    Eigen::Matrix4d T_cam1_cam0 = T_wc_cam1.inverse() * T_wc_cam0;
    Eigen::AngleAxisd rot_axis_1_0(T_cam1_cam0.block<3, 3>(0, 0));

    //! optimzied variables
    // motor axis vector
    double opt_motor_axis[3] = {rot_axis_1_0.axis()[0], rot_axis_1_0.axis()[1], rot_axis_1_0.axis()[2]};

    // rotation angle of each camera in angleaxis frame
    double opt_rotation_angles[cam_num_];

    // delta_0 vector
    double opt_delta_0[3] = {0};

    // initialize camera0 optimized variables
    double opt_cam0_transform[6];
    ceres::RotationMatrixToAngleAxis(R_cw_cam0.data(), opt_cam0_transform);
    opt_cam0_transform[3] = t_cw_cam0[0];
    opt_cam0_transform[4] = t_cw_cam0[1];
    opt_cam0_transform[5] = t_cw_cam0[2];

    // initialize rotation angles
    for (size_t i = 0; i < v_cam_calibrator_ptrs_.size(); ++i)
    {
        Eigen::Matrix4d T_i_0 = (v_cam_calibrator_ptrs_[i]->cameraPose(0)).inverse() * T_wc_cam0;
        Eigen::AngleAxisd rot_i_0_vec(T_i_0.block<3, 3>(0, 0));

        Eigen::Vector3d rot_i_axis = rot_i_0_vec.axis();
        double sign_direction = rot_i_axis.dot(rot_axis_1_0.axis());
        if (sign_direction > 0)
        {
            opt_rotation_angles[i] = rot_i_0_vec.angle();
        }
        else
        {
            opt_rotation_angles[i] = -rot_i_0_vec.angle();
        }

        DEBUG_STREAM("[TecheCalibrator::calibrate] rotation axis : " << rot_i_0_vec.axis().transpose());
        DEBUG_STREAM("[TecheCalibrator::calibrate] rotation angle : " << opt_rotation_angles[i] * RAD_2_DEGREE);
    }

    ceres::Problem problem;
    for (size_t i = 0; i < v_cam_calibrator_ptrs_.size(); ++i)
    {
        // TODO: only support 1 sample image so far
        const Eigen::Matrix3d &intrinsic = v_cam_calibrator_ptrs_[i]->camera()->intrinsic();
        // size_t sample_img_num = v_cam_calibrator_ptrs_[i]->imagePoints().size();
        for (size_t j = 0; j < 1; ++j)
        {
            // only use inliner matches to optimize camera pose
            const std::vector<cv::Point2d> &v_obs_kpts = v_cam_calibrator_ptrs_[i]->inlierImagePoints(j);
            const std::vector<Eigen::Vector3d> &v_world_points = v_cam_calibrator_ptrs_[i]->inlierObjectPoints(j);
            for (size_t p = 0; p < v_world_points.size(); ++p)
            {
                const Eigen::Vector3d &map_pt = v_world_points[p];
                const cv::Point2d &obs_kpt = v_obs_kpts[p];

                ceres::CostFunction *cost_function = ReprojectionErrorWithConcentricConstrain::Create(obs_kpt,
                                                                                                      map_pt,
                                                                                                      intrinsic);

                problem.AddResidualBlock(cost_function, NULL,
                                         opt_motor_axis,
                                         opt_rotation_angles + i,
                                         opt_delta_0,
                                         opt_cam0_transform);
            }
        }
    }
    // Solve the non-linear optimization
    bool sts = ceresOptimize(problem, 1e-6);
    if (!sts)
    {
        ERROR_STREAM("[TecheCalibrator::calibrate] Fail to optimize");
        return false;
    }

    Eigen::Vector3d delta_0_vector(opt_delta_0[0], opt_delta_0[1], opt_delta_0[2]);

    Eigen::Vector3d t_cam0_vector(opt_cam0_transform[3], opt_cam0_transform[4], opt_cam0_transform[5]);
    Eigen::Matrix3d R_cw_cam0_opt = Eigen::Matrix3d::Identity();
    ceres::AngleAxisToRotationMatrix(opt_cam0_transform, R_cw_cam0_opt.data());

    Eigen::Vector3d motor_axis_vector(opt_motor_axis[0], opt_motor_axis[1], opt_motor_axis[2]);
    global_rotation_axis_ = -R_cw_cam0_opt.transpose() * motor_axis_vector;

    // check optimized result
    for (size_t i = 0; i < v_cam_calibrator_ptrs_.size(); ++i)
    {
        MonoCalibrator::Ptr cam_calibrator = v_cam_calibrator_ptrs_[i];
        Eigen::Matrix3d intrinsic = cam_calibrator->camera()->intrinsic();

        Eigen::Matrix3d rotation_cw_tmp = Eigen::Matrix3d::Identity();
        unsigned int tmp_inlier_num;

        // motor rotation vector
        Eigen::AngleAxisd motor_rot_vec(0, motor_axis_vector);
        motor_rot_vec.angle() = opt_rotation_angles[i];
        rotation_cw_tmp = motor_rot_vec.matrix();
        // global rotation vector
        Eigen::AngleAxisd global_rot_vec(0, global_rotation_axis_);
        global_rot_vec.angle() = opt_rotation_angles[i];

        // R_cw_i
        Eigen::Matrix3d R_cw_i = rotation_cw_tmp * R_cw_cam0_opt;

        // delta_i
        Eigen::Vector3d delta_i_vec = global_rot_vec.matrix() * delta_0_vector;
        // t_cw_i
        Eigen::Vector3d t_cw_i = rotation_cw_tmp * (R_cw_cam0_opt * (delta_i_vec - delta_0_vector) + t_cam0_vector);
        Eigen::Matrix3d R_wc_i = R_cw_i.transpose();
        Eigen::Vector3d t_wc_i = -R_wc_i * t_cw_i;

        // check inliers after optimization
        {
            // size_t sample_img_num = cam_calibrator->imagePoints().size();
            for (size_t j = 0; j < 1; ++j)
            {
                double mean_reproject_error = 0;
                double origin_mean_reproject_error = cam_calibrator->meanReprojectionError(j);
                std::vector<Eigen::Vector3d> &v_world_points = cam_calibrator->inlierObjectPoints(j);
                std::vector<cv::Point2d> &v_obs_keypoints = cam_calibrator->inlierImagePoints(j);

                tmp_inlier_num = MonoCalibrator::checkInliers(v_world_points, v_obs_keypoints, intrinsic, R_cw_i, t_cw_i, mean_reproject_error);

                unsigned int origin_inlier_num = v_world_points.size();

                if (tmp_inlier_num * 10 > origin_inlier_num * 7 || mean_reproject_error < origin_mean_reproject_error)
                {
                    sts = true;
                    Eigen::Matrix4d &cam_pose = cam_calibrator->cameraPose(j);
                    cam_pose.block<3, 3>(0, 0) = R_wc_i;
                    cam_pose.block<3, 1>(0, 3) = t_wc_i;
                    cam_calibrator->meanReprojectionError(j) = mean_reproject_error;
                }
                else
                {
                    sts = false;
                    ERROR_STREAM("[TecheCalibrator::calibrate] Optimize camera " << i << " failed!");
                }

                if (i <= cam_num_ / 2)
                {
                    ERROR_STREAM("[TecheCalibrator::calibrate] Camera " << i << " rotation angle " << motor_rot_vec.angle() * RAD_2_DEGREE);
                }
                else
                {
                    ERROR_STREAM("[TecheCalibrator::calibrate] Camera " << i << " rotation angle " << (motor_rot_vec.angle() + 2 * M_PI) * RAD_2_DEGREE);
                }

                ERROR_STREAM("[TecheCalibrator::calibrate] Frame " << i << " Original inlier num: " << origin_inlier_num);
                ERROR_STREAM("[TecheCalibrator::calibrate] Frame " << i << " Optimized inlier num: " << tmp_inlier_num);
                ERROR_STREAM("[TecheCalibrator::calibrate] Frame " << i << " Original mean reprojection error : " << origin_mean_reproject_error);
                ERROR_STREAM("[TecheCalibrator::calibrate] Frame " << i << " Optimized mean reprojection error : " << mean_reproject_error);
            }
        }

        double relative_rotation_angle = 0;
        if (0 == i)
        {
            relative_rotation_angle = opt_rotation_angles[i];
        }
        else
        {
            relative_rotation_angle = opt_rotation_angles[i] - opt_rotation_angles[i - 1];
        }
        if (relative_rotation_angle > M_PI)
            relative_rotation_angle -= 2 * M_PI;
        if (relative_rotation_angle < -M_PI)
            relative_rotation_angle += 2 * M_PI;
        ERROR_STREAM("[TecheCalibrator::calibrate] relative rotation angle " << relative_rotation_angle * RAD_2_DEGREE);
    }

    DEBUG_STREAM("[TecheCalibrator::calibrate] motor axis: " << motor_axis_vector.transpose());
    DEBUG_STREAM("[TecheCalibrator::calibrate] global axis: " << global_rotation_axis_.transpose());
    DEBUG_STREAM("[TecheCalibrator::calibrate] delta vector to axis center : " << delta_0_vector.transpose() << ", norm : " << delta_0_vector.norm());

    return calcCameraPoser2c(global_rotation_axis_);
}

bool TecheCalibrator::calcCameraPoser2c(const Eigen::Vector3d &global_rotation_axis)
{
    if (global_rotation_axis.hasNaN())
    {
        ERROR_STREAM("[TecheCalibrator::calCameraPoser2c] INvalid input!");
        return false;
    }
    // // debug camera pose (wrt calibration room)
    // {
    //     Eigen::Matrix4d cam0_w2c = v_cam_calibrator_ptrs_[0]->cameraPose(0);
    //     Eigen::Matrix4d cam1_w2c = v_cam_calibrator_ptrs_[1]->cameraPose(0);
    //     Eigen::Matrix4d cam2_w2c = v_cam_calibrator_ptrs_[2]->cameraPose(0);
    //     Eigen::Matrix4d cam3_w2c = v_cam_calibrator_ptrs_[3]->cameraPose(0);

    //     Eigen::Matrix4d T_c1_c0 = cam1_w2c.inverse() * cam0_w2c;
    //     Eigen::Matrix4d T_c2_c1 = cam2_w2c.inverse() * cam1_w2c;
    //     Eigen::Matrix4d T_c3_c2 = cam3_w2c.inverse() * cam2_w2c;

    //     Eigen::AngleAxisd rot_vec(T_c1_c0.block<3, 3>(0, 0));
    //     Eigen::Vector3d t_c1_c0 = T_c1_c0.block<3, 1>(0, 3);
    //     std::cout << "[compareCameraPoses] C1 2 C0 axis : " << rot_vec.axis().transpose() << "\n";
    //     std::cout << "[compareCameraPoses] C1 2 C0 Rotation angle: " << rot_vec.angle() * 180.0 / M_PI << "\n";
    //     std::cout << "[compareCameraPoses] C1 2 C0 translation : " << t_c1_c0.norm() << "\n";

    //     rot_vec = T_c2_c1.block<3, 3>(0, 0);
    //     Eigen::Vector3d t_c2_c1 = T_c2_c1.block<3, 1>(0, 3);
    //     std::cout << "[compareCameraPoses] C2 2 C1 axis : " << rot_vec.axis().transpose() << "\n";
    //     std::cout << "[compareCameraPoses] C2 2 C1 Rotation angle: " << rot_vec.angle() * 180.0 / M_PI << "\n";
    //     std::cout << "[compareCameraPoses] C2 2 C1 translation : " << t_c2_c1.norm() << "\n";

    //     rot_vec = T_c3_c2.block<3, 3>(0, 0);
    //     Eigen::Vector3d t_c3_c2 = T_c3_c2.block<3, 1>(0, 3);
    //     std::cout << "[compareCameraPoses] C3 2 C2 axis : " << rot_vec.axis().transpose() << "\n";
    //     std::cout << "[compareCameraPoses] C3 2 C2 Rotation angle: " << rot_vec.angle() * 180.0 / M_PI << "\n";
    //     std::cout << "[compareCameraPoses] C3 2 C2 translation : " << t_c3_c2.norm() << "\n";
    // }

    // calculate relative pose between 4 cameras
    {
        // calculate axis center
        Eigen::Vector3d global_axis_center = Eigen::Vector3d::Zero();
        bool sts = calcRotationAxisCenter(v_cam_calibrator_ptrs_, global_axis_center);
        if (!sts)
        {
            ERROR_STREAM("[TecheCalibrator::calCameraPoser2c] Fail to calculate global_rotation_center");
            return false;
        }

        // construct global coordinate frame( w.r.t calibration room )
        Eigen::Matrix4d T_w_rotaionaxis = Eigen::Matrix4d::Identity();
        {
            // tranform points from rotation axis frame to world frame
            // global rotation axis is Y axis
            Eigen::Vector3d axis_y = global_rotation_axis;
            axis_y.normalize();
            //  Z axis
            Eigen::Vector3d cam_0_center = v_cam_calibrator_ptrs_[0]->cameraPose(0).block<3, 1>(0, 3);
            Eigen::Vector3d axis_z_tmp;
            axis_z_tmp[0] = cam_0_center[0] - global_axis_center[0];
            axis_z_tmp[1] = cam_0_center[1] - global_axis_center[1];
            axis_z_tmp[2] = cam_0_center[2] - global_axis_center[2];
            axis_z_tmp.normalize();

            Eigen::Vector3d axis_x = axis_y.cross(axis_z_tmp);
            axis_x.normalize();
            Eigen::Vector3d axis_z = axis_x.cross(axis_y);
            axis_z.normalize();

            DEBUG_STREAM("[TecheCalibrator::calCameraPoser2c] axis_x : " << axis_x.transpose());
            DEBUG_STREAM("[TecheCalibrator::calCameraPoser2c] axis_y : " << axis_y.transpose());
            DEBUG_STREAM("[TecheCalibrator::calCameraPoser2c] axis_z : " << axis_z.transpose());

            T_w_rotaionaxis.block<3, 1>(0, 0) = -axis_x;
            T_w_rotaionaxis.block<3, 1>(0, 1) = -axis_y;
            T_w_rotaionaxis.block<3, 1>(0, 2) = axis_z;
            T_w_rotaionaxis.block<3, 1>(0, 3) = global_axis_center;
        }

        // calculate poses relative to global rotation axis
        for (size_t i = 0; i < v_cam_calibrator_ptrs_.size(); ++i)
        {
            Eigen::Matrix4d T_ci_w = v_cam_calibrator_ptrs_[i]->cameraPose(0).inverse();
            Eigen::Matrix4d T_r_ci = (T_ci_w * T_w_rotaionaxis).inverse();
            v_cam_pose_r_c_.emplace_back(T_r_ci);
        }
        // // debug camera pose (wrt rotation axis)
        // {
        //     Eigen::Matrix4d T_r_c0 = v_cam_pose_r_c_[0];
        //     Eigen::Matrix4d T_r_c1 = v_cam_pose_r_c_[1];
        //     Eigen::Matrix4d T_r_c2 = v_cam_pose_r_c_[2];
        //     Eigen::Matrix4d T_r_c3 = v_cam_pose_r_c_[3];

        //     Eigen::Matrix4d T_c1_c0 = T_r_c1.inverse() * T_r_c0;
        //     Eigen::Matrix4d T_c2_c1 = T_r_c2.inverse() * T_r_c1;
        //     Eigen::Matrix4d T_c3_c2 = T_r_c3.inverse() * T_r_c2;

        //     Eigen::AngleAxisd rot_vec(T_c1_c0.block<3, 3>(0, 0));
        //     Eigen::Vector3d t_c1_c0 = T_c1_c0.block<3, 1>(0, 3);
        //     std::cout << "[compareCameraPoses] C1 2 C0 axis : " << rot_vec.axis().transpose() << "\n";
        //     std::cout << "[compareCameraPoses] C1 2 C0 Rotation angle: " << rot_vec.angle() * 180.0 / M_PI << "\n";
        //     std::cout << "[compareCameraPoses] C1 2 C0 translation : " << t_c1_c0.norm() << "\n";

        //     rot_vec = T_c2_c1.block<3, 3>(0, 0);
        //     Eigen::Vector3d t_c2_c1 = T_c2_c1.block<3, 1>(0, 3);
        //     std::cout << "[compareCameraPoses] C2 2 C1 axis : " << rot_vec.axis().transpose() << "\n";
        //     std::cout << "[compareCameraPoses] C2 2 C1 Rotation angle: " << rot_vec.angle() * 180.0 / M_PI << "\n";
        //     std::cout << "[compareCameraPoses] C2 2 C1 translation : " << t_c2_c1.norm() << "\n";

        //     rot_vec = T_c3_c2.block<3, 3>(0, 0);
        //     Eigen::Vector3d t_c3_c2 = T_c3_c2.block<3, 1>(0, 3);
        //     std::cout << "[compareCameraPoses] C3 2 C2 axis : " << rot_vec.axis().transpose() << "\n";
        //     std::cout << "[compareCameraPoses] C3 2 C2 Rotation angle: " << rot_vec.angle() * 180.0 / M_PI << "\n";
        //     std::cout << "[compareCameraPoses] C3 2 C2 translation : " << t_c3_c2.norm() << "\n";
        // }
    }

    return true;
}
