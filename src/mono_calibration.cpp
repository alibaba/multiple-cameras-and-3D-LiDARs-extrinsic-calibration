
#include "cost_function.h"
#include "mono_calibration.h"
#include "params_config.h"

const double MonoCalibrator::reproject_err_thre_ = ParamsConfig::GetMinReprojectionErrThreshold();

MonoCalibrator::MonoCalibrator(camera::model_type_t camModelType,
                               target::target_type_t targetModelType,
                               const std::string &intrinsicFileName,
                               const std::string &targetFileName)
{
    camera_ptr_ = std::make_shared<camera::Camera>(intrinsicFileName, camera::setup_type_t::Monocular, camModelType);
    target_ptr_ = std::make_shared<target::TargetBoard>(targetFileName, targetModelType);

    v_object_points_ = target_ptr_->objectPoints();
    map_bound_box_.setMapPoints(v_object_points_);

    // min_inliers = 4, min_pnp_mathces = 3
    relocalizer_ptr_ = std::make_shared<Relocalizer>(v_object_points_,
                                                     ParamsConfig::GetPnPMinInlierNum(),
                                                     ParamsConfig::GetPnPMinMatchesNum(),
                                                     camera_ptr_->imageWidth(),
                                                     camera_ptr_->imageHeight());
}

MonoCalibrator::MonoCalibrator(camera::model_type_t camModelType,
                               const std::string &intrinsicFileName,
                               const std::vector<Eigen::Vector3d> &vGlobalMap,
                               std::shared_ptr<Relocalizer> &relocalizerPtr)
{
    camera_ptr_ = std::make_shared<camera::Camera>(intrinsicFileName, camera::setup_type_t::Monocular, camModelType);
    v_object_points_ = vGlobalMap;
    map_bound_box_.setMapPoints(vGlobalMap);

    relocalizer_ptr_ = relocalizerPtr;
}

void MonoCalibrator::clear()
{
    vv_img_points_.clear();
    sample_img_pathes_.clear();
    v_object_points_.clear();
}

bool MonoCalibrator::addCctagData(const std::vector<cv::Point2d> &v_centers)
{
    if (v_centers.empty())
    {
        ERROR_STREAM("[MonoCalibrator::addCctagData] Empty input cctag result!");
        return false;
    }

    unsigned int min_pnp_matches_nb = ParamsConfig::GetPnPMinMatchesNum();
    std::vector<cv::Point2d> v_valid_kpts = camera_ptr_->filteKeypoints(v_centers, ParamsConfig::GetImageBound());
    if (v_valid_kpts.size() < min_pnp_matches_nb)
    {
        ERROR_STREAM("[MonoCalibrator::addCctagData] Valid keypoints on up image is less than " << min_pnp_matches_nb);
        return false;
    }
    vv_img_points_.emplace_back(v_valid_kpts);

    return true;
}

bool MonoCalibrator::addCctagData(const std::string &imgFilePath, const std::vector<cv::Point2d> &v_centers)
{
    if (v_centers.empty())
    {
        ERROR_STREAM("[MonoCalibrator::addCctagData] Empty input cctag result!");
        return false;
    }

    unsigned int min_pnp_matches_nb = ParamsConfig::GetPnPMinMatchesNum();
    std::vector<cv::Point2d> v_valid_kpts = camera_ptr_->filteKeypoints(v_centers, ParamsConfig::GetImageBound());
    if (v_valid_kpts.size() < min_pnp_matches_nb)
    {
        ERROR_STREAM("[MonoCalibrator::addCctagData] Frame" << imgFilePath << " Valid keypoints on up image is less than " << min_pnp_matches_nb);
        return false;
    }
    sample_img_pathes_.push_back(imgFilePath);
    vv_img_points_.emplace_back(v_valid_kpts);

    return true;
}

std::vector<cv::Point2d> &MonoCalibrator::inlierImagePoints(unsigned int imgIdx)
{
    return vv_inlier_imgPoints_[imgIdx];
}

const std::vector<cv::Point2d> MonoCalibrator::inlierImagePoints(unsigned int imgIdx) const
{
    return vv_inlier_imgPoints_[imgIdx];
}

std::vector<cv::Point2d> &MonoCalibrator::imagePoints(unsigned int imgIdx)
{
    return vv_img_points_[imgIdx];
}

const std::vector<cv::Point2d> MonoCalibrator::imagePoints(unsigned int imgIdx) const
{
    return vv_img_points_[imgIdx];
}

std::vector<std::vector<cv::Point2d>> &MonoCalibrator::imagePoints()
{
    return vv_img_points_;
}

const std::vector<std::vector<cv::Point2d>> MonoCalibrator::imagePoints() const
{
    return vv_img_points_;
}

std::vector<Eigen::Vector3d> &MonoCalibrator::inlierObjectPoints(unsigned int imgIdx)
{
    return vv_inlier_objPoints_[imgIdx];
}

const std::vector<Eigen::Vector3d> MonoCalibrator::inlierObjectPoints(unsigned int imgIdx) const
{
    return vv_inlier_objPoints_[imgIdx];
}

const std::vector<Eigen::Vector3d> MonoCalibrator::objectPoints() const
{
    return v_object_points_;
}

std::map<int, int> &MonoCalibrator::inlierMatchesIdx(unsigned int imgIdx)
{
    if (imgIdx >= vv_img_points_.size())
    {
        ERROR_STREAM("[MonoCalibrator::isValid] Invalid image index: " << imgIdx);
        // return v_mean_reprojection_error_[imgIdx];
    }
    return vv_inlier_matches_[imgIdx];
}

std::map<int, int> MonoCalibrator::inlierMatchesIdx(unsigned int imgIdx) const
{
    if (imgIdx >= vv_img_points_.size())
    {
        ERROR_STREAM("[MonoCalibrator::isValid] Invalid image index: " << imgIdx);
        // return v_mean_reprojection_error_[imgIdx];
    }
    return vv_inlier_matches_[imgIdx];
}

double &MonoCalibrator::meanReprojectionError(unsigned int imgIdx)
{
    if (imgIdx >= vv_img_points_.size())
    {
        ERROR_STREAM("[MonoCalibrator::isValid] Invalid image index: " << imgIdx);
        // return v_mean_reprojection_error_[imgIdx];
    }
    return v_mean_reprojection_error_[imgIdx];
}

camera::Camera::Ptr &MonoCalibrator::camera(void)
{
    return camera_ptr_;
}

bool MonoCalibrator::isValid(unsigned int imgIdx) const
{
    if (imgIdx >= vv_img_points_.size())
    {
        ERROR_STREAM("[MonoCalibrator::isValid] Invalid image index: " << imgIdx);
        return false;
    }
    return v_relocalized_flag_[imgIdx];
}

Eigen::Matrix4d &MonoCalibrator::cameraPose(unsigned int imgIdx)
{
    if (imgIdx >= vv_img_points_.size())
    {
        ERROR_STREAM("[MonoCalibrator::isValid] Invalid image index: " << imgIdx);
        // return Eigen::Matrix4d::Identity();
    }
    return v_T_wc_[imgIdx];
}

std::vector<Eigen::Matrix4d> MonoCalibrator::validCameraPoses() const
{
    std::vector<Eigen::Matrix4d> v_results(v_T_wc_.size(), Eigen::Matrix4d::Identity());
    for (size_t i = 0; i < v_T_wc_.size(); ++i)
    {
        if (!v_relocalized_flag_[i])
            continue;

        v_results[i] = v_T_wc_[i];
    }
    return v_results;
}

bool MonoCalibrator::extractValidIndices(const std::map<int, int> &inlierMatches,
                                         std::vector<unsigned int> &kpt_inlier_indices,
                                         std::vector<unsigned int> &map_inlier_indices)
{
    kpt_inlier_indices.clear();
    map_inlier_indices.clear();

    for (std::map<int, int>::const_iterator it = inlierMatches.begin(); it != inlierMatches.end(); ++it)
    {
        if (it->second < 0)
            continue;

        map_inlier_indices.push_back(it->first);
        kpt_inlier_indices.push_back(it->second);
    }
    if (map_inlier_indices.empty() || kpt_inlier_indices.empty())
    {
        ERROR_STREAM("[StereoCameraCalibrator::extractValidIndices] Empty inlier after relocalization!");
        return false;
    }

    return true;
}

bool MonoCalibrator::calibrate()
{
    if (v_object_points_.empty())
    {
        ERROR_STREAM("[MonoCalibrator::calibrate] Object points is empty!");
        return false;
    }
    if (vv_img_points_.size() < ParamsConfig::GetMinSampleImageCounts())
    {
        ERROR_STREAM("[MonoCalibrator::calibrate] Sample image is less than " << ParamsConfig::GetMinSampleImageCounts());
        return false;
    }

    v_T_wc_.assign(vv_img_points_.size(), Eigen::Matrix4d::Identity());
    v_relocalized_flag_.assign(vv_img_points_.size(), false);
    v_mean_reprojection_error_.resize(vv_img_points_.size(), std::numeric_limits<double>::max());
    vv_inlier_imgPoints_.resize(vv_img_points_.size(), std::vector<cv::Point2d>());
    vv_inlier_objPoints_.resize(vv_img_points_.size(), std::vector<Eigen::Vector3d>());
    vv_inlier_matches_.resize(vv_img_points_.size(), std::map<int, int>());

    bool sts = true;
    for (size_t i = 0; i < vv_img_points_.size(); ++i)
    {
        DEBUG_STREAM("[MonoCalibrator::calibrateStereoFrame] Localize " << sample_img_pathes_[i]);

        // 3D-2D inlier_matches
        std::map<int, int> inlier_matches;
        double mean_reprojection_error = 0;
        Eigen::Matrix4d T_wc;
        sts = relocalizer_ptr_->relocalizeByFrame(sample_img_pathes_[i],
                                                  vv_img_points_[i],
                                                  camera_ptr_->intrinsic(),
                                                  T_wc,
                                                  inlier_matches,
                                                  mean_reprojection_error);
        if (!sts)
        {
            ERROR_STREAM("[MonoCalibrator::calibrate] Fail to relocalize the up camera in frame " << sample_img_pathes_[i]);
            continue;
        }

        // verify camera pose
        if (!map_bound_box_.beInBoundingBox(T_wc))
        {
            ERROR_STREAM("[MonoCalibrator::calibrateMonoFrame] Invalid camera pose! This must be twist!");
            continue;
        }

        v_relocalized_flag_.at(i) = true;
        // extract inlier 3D-2D matches
        std::vector<unsigned int> map_inlier_indices, kpt_inlier_indices;
        if (!extractValidIndices(inlier_matches, kpt_inlier_indices, map_inlier_indices))
        {
            ERROR_STREAM("[MonoCalibrator::calibrate] Invalid 3D-2D matches indices!");
            return false;
        }
        std::vector<cv::Point2d> v_inlier_kpts = resampleByIndices(kpt_inlier_indices, vv_img_points_[i]);
        std::vector<Eigen::Vector3d> v_inlier_maps = resampleByIndices(map_inlier_indices, v_object_points_);
        vv_inlier_imgPoints_[i] = std::move(v_inlier_kpts);
        vv_inlier_objPoints_[i] = std::move(v_inlier_maps);
        vv_inlier_matches_[i] = std::move(inlier_matches);
        v_mean_reprojection_error_[i] = mean_reprojection_error;
        v_T_wc_[i] = T_wc;
    }

    sts = optimize();
    if (!sts)
    {
        ERROR_STREAM("[MonoCalibrator::calibrate] failed to optimize!");
        return false;
    }

    return true;
}

unsigned int MonoCalibrator::checkInliers(const std::vector<Eigen::Vector3d> &v_map_points,
                                          const std::vector<cv::Point2d> &v_keypoints,
                                          const Eigen::Matrix3d &cam_intrinsic,
                                          const Eigen::Matrix3d &rotation,
                                          const Eigen::Vector3d &translation,
                                          double &mean_reprojection_error)
{
    if (v_map_points.empty() || v_keypoints.empty())
    {
        ERROR_STREAM("[MonoCalibrator::checkInliers] Empty input!");
        return 0;
    }
    if (cam_intrinsic.hasNaN() || rotation.hasNaN() || translation.hasNaN())
    {
        ERROR_STREAM("[MonoCalibrator::checkInliers] Invalid camera pose!");
        return 0;
    }

    unsigned int inlier_num = 0;
    mean_reprojection_error = 0;
    const double eps = std::numeric_limits<float>::epsilon();
    for (size_t i = 0; i < v_map_points.size(); ++i)
    {

        Eigen::Vector3d map_point = v_map_points[i];

        Eigen::Vector3d proj_point = rotation * map_point + translation;
        if (proj_point[2] <= eps)
        {
            continue;
        }

        Eigen::Vector3d img_point = cam_intrinsic * proj_point;
        double img_x = img_point[0] / img_point[2];
        double img_y = img_point[1] / img_point[2];

        double x_err = (img_x - v_keypoints[i].x);
        double y_err = (img_y - v_keypoints[i].y);
        double dist = std::sqrt(x_err * x_err + y_err * y_err);
        if (dist < reproject_err_thre_)
        {
            inlier_num++;
            mean_reprojection_error += dist;
        }
    }

    if (1 > inlier_num)
    {
        mean_reprojection_error = std::numeric_limits<double>::max();
        ERROR_STREAM("[MonoCalibrator::checkInliers] Empty inlier found!");
        return 0;
    }

    mean_reprojection_error /= inlier_num;

    return inlier_num;
}

bool MonoCalibrator::optimize(void)
{
    if (v_relocalized_flag_.empty() || v_T_wc_.empty())
    {
        ERROR_STREAM("[MonoCalibrator::optimize] Empty input camera pose!");
        return false;
    }

    assert(v_relocalized_flag_.size() == v_T_wc_.size());

    if (std::count(v_relocalized_flag_.begin(), v_relocalized_flag_.end(), true) == 0)
    {
        ERROR_STREAM("[MonoCalibrator::optimize] Non valid camera pose in current sample images");
        return false;
    }

    //! optimzied variables
    // camera poses
    double v_transform_opt[v_T_wc_.size() * 6] = {0.0};
    // intrinsic : fx, fy, cx, cy
    Eigen::Matrix3d &intrinsic = camera_ptr_->intrinsic();
    double intrinsic_opt[4] = {intrinsic(0, 0), intrinsic(1, 1),
                               intrinsic(0, 2), intrinsic(1, 2)};

    assert(v_T_wc_.size() == vv_inlier_objPoints_.size());
    assert(v_T_wc_.size() == vv_inlier_imgPoints_.size());

    ceres::Problem problem;
    for (size_t j = 0; j < v_T_wc_.size(); ++j)
    {
        if (!v_relocalized_flag_[j])
            continue;

        // prepare initial value for optimized variables
        Eigen::Matrix4d T_cw = v_T_wc_[j].inverse();
        Eigen::Matrix3d rotation = T_cw.block<3, 3>(0, 0);
        Eigen::Vector3d translation = T_cw.block<3, 1>(0, 3);
        ceres::RotationMatrixToAngleAxis(rotation.data(), v_transform_opt + 6 * j);
        v_transform_opt[6 * j + 3] = translation[0];
        v_transform_opt[6 * j + 4] = translation[1];
        v_transform_opt[6 * j + 5] = translation[2];

        // only use inliner matches to optimize camera pose
        assert(vv_inlier_objPoints_[j].size() == vv_inlier_imgPoints_[j].size());

        std::vector<Eigen::Vector3d> &v_world_points = vv_inlier_objPoints_[j];
        std::vector<cv::Point2d> &v_obs_keypoints = vv_inlier_imgPoints_[j];
        for (size_t i = 0; i < v_world_points.size(); ++i)
        {
            Eigen::Vector3d &map_pt = v_world_points[i];
            cv::Point2d &obs_kpt = v_obs_keypoints[i];

            ceres::CostFunction *cost_function = ReprojectionError1::Create(obs_kpt,
                                                                            map_pt,
                                                                            intrinsic);
            // ceres::LossFunction *lossFunction = new ceres::CauchyLoss(1.0);
            problem.AddResidualBlock(cost_function, NULL, v_transform_opt + 6 * j, intrinsic_opt);
        }
    }
    // Solve the non-linear optimization
    bool sts = ceresOptimize(problem, 1e-4);
    if (!sts)
    {
        ERROR_STREAM("[MonoCalibrator::optirmize] Fail to optimize");
        return false;
    }

    for (size_t i = 0; i < v_T_wc_.size(); ++i)
    {
        if (!v_relocalized_flag_[i])
            continue;

        Eigen::Matrix3d rotation_tmp = Eigen::Matrix3d::Identity();
        Eigen::Vector3d trans_tmp = Eigen::Vector3d::Zero();
        unsigned int tmp_inlier_num;

        ceres::AngleAxisToRotationMatrix(v_transform_opt + 6 * i, rotation_tmp.data());
        trans_tmp[0] = v_transform_opt[6 * i + 3];
        trans_tmp[1] = v_transform_opt[6 * i + 4];
        trans_tmp[2] = v_transform_opt[6 * i + 5];

        double mean_reproject_error = 0;
        double origin_mean_reproject_error = v_mean_reprojection_error_[i];
        // check inliers after optimization
        std::vector<Eigen::Vector3d> &v_world_points = vv_inlier_objPoints_[i];
        std::vector<cv::Point2d> &v_obs_keypoints = vv_inlier_imgPoints_[i];
#if CERES_OPTIMZE_INTRINSIC
        Eigen::Matrix3d opt_intrinsic = Eigen::Matrix3d::Identity();
        opt_intrinsic(0, 0) = intrinsic_opt[0];
        opt_intrinsic(1, 1) = intrinsic_opt[1];
        opt_intrinsic(0, 2) = intrinsic_opt[2];
        opt_intrinsic(1, 2) = intrinsic_opt[3];
        tmp_inlier_num = checkInliers(v_world_points, v_obs_keypoints, opt_intrinsic, rotation_tmp, trans_tmp, mean_reproject_error);
#else
        tmp_inlier_num = checkInliers(v_world_points, v_obs_keypoints, intrinsic, rotation_tmp, trans_tmp, mean_reproject_error);
#endif
        unsigned int origin_inlier_num = v_world_points.size();

        if (tmp_inlier_num * 10 > origin_inlier_num * 7 || mean_reproject_error < origin_mean_reproject_error)
        {
            sts = true;
            Eigen::Matrix4d T_cw_opt = Eigen::Matrix4d::Identity();
            T_cw_opt.block<3, 3>(0, 0) = rotation_tmp;
            T_cw_opt.block<3, 1>(0, 3) = trans_tmp;

            v_T_wc_[i] = T_cw_opt.inverse();
            v_mean_reprojection_error_[i] = mean_reproject_error;
        }
        else
        {
            sts = false;
            ERROR_STREAM("[MonoCalibrator::optimize] Optimize frame " << i << " failed!");
        }
        ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " Original inlier num: " << origin_inlier_num);
        ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " Optimized inlier num: " << tmp_inlier_num);
        ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " Original mean reprojection error : " << origin_mean_reproject_error);
        ERROR_STREAM("[MonoCalibrator::optimize] Frame " << i << " Optimized mean reprojection error : " << mean_reproject_error);
#if CERES_OPTIMZE_INTRINSIC
        ERROR_STREAM("[MonoCalibrator::optimize] Optimized intrinsic matrix : " << opt_intrinsic(0, 0) << ", "
                                                                                << opt_intrinsic(1, 1) << ", "
                                                                                << opt_intrinsic(0, 2) << ", "
                                                                                << opt_intrinsic(1, 2));
        intrinsic(0, 0) = opt_intrinsic(0, 0);
        intrinsic(1, 1) = opt_intrinsic(1, 1);
        intrinsic(0, 2) = opt_intrinsic(0, 2);
        intrinsic(1, 2) = opt_intrinsic(1, 2);
#endif
    }

    return sts;
}