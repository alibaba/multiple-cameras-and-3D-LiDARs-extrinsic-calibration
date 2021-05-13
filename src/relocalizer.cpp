#include <chrono>

#include "common.h"
#include "relocalizer.h"
#include "delaunator.hpp"

Relocalizer::Relocalizer(const std::vector<Eigen::Vector3d> &vMapPoints,
                         unsigned int minValidMatchesThre,
                         unsigned int minPnPMatchesNb,
                         int img_width,
                         int img_height) : min_valid_matches_nb_(minValidMatchesThre),
                                           min_pnp_matches_nb_(minPnPMatchesNb),
                                           img_width_(img_width),
                                           img_height_(img_height)
{
    if (vMapPoints.empty())
    {
        ERROR_STREAM("[Relocalizer] Empty Input Parameters!");
    }

    if (vMapPoints.size() < min_pnp_matches_nb_)
    {
        ERROR_STREAM("[Relocalizer] Map points size is less than " << min_pnp_matches_nb_ << ", check it!");
    }

    total_map_points_nb_ = vMapPoints.size();
    for (size_t i = 0; i < total_map_points_nb_; ++i)
    {
        v_map_points_.emplace_back(vMapPoints[i]);
    }

    //! calculate total valid combination on 3D space
    v_map_point_combinations_ = calcFullCombination(total_map_points_nb_, min_pnp_matches_nb_);
}

Relocalizer::~Relocalizer()
{
}

std::vector<std::vector<unsigned int>> Relocalizer::calcFullCombination(unsigned int map_points_num, unsigned int sample_num)
{
    DEBUG_STREAM("[calcFullCombination] Start calculate full combination of " << std::to_string(map_points_num) << " ...");

    bool vbVisit[map_points_num] = {false};
    unsigned int input_array[map_points_num];
    std::iota(input_array, input_array + map_points_num, 0u);

    std::vector<std::vector<unsigned int>> full_combs;

    FullCombination(0u, 0u, map_points_num, sample_num, input_array, vbVisit, full_combs);

    DEBUG_STREAM("[calcFullCombination] Total combination vector size is " << full_combs.size());

    // check distance constraint
    std::vector<std::vector<unsigned int>> final_output;
    for (size_t p = 0; p < full_combs.size(); ++p)
    {
        const std::vector<unsigned int> &comb = full_combs[p];

        if (checkSampleMappoints(comb, v_map_points_))
        {
            final_output.emplace_back(comb);
        }
    }

    if (final_output.empty())
    {
        ERROR_STREAM("[calcFullCombination] Empty output!");
    }

    DEBUG_STREAM("[calcFullCombination] Final combination vector size is " << final_output.size());
    return final_output;
}

std::vector<std::vector<unsigned int>> Relocalizer::calcFullPermutation(const std::vector<unsigned int> &input_array)
{
    if (input_array.size() < min_pnp_matches_nb_)
    {
        ERROR_STREAM("[computeFullPermutation] Input array size is less than " << min_pnp_matches_nb_);
        return std::vector<std::vector<unsigned int>>();
    }

    unsigned int size = input_array.size();
    unsigned int input[size];
    std::copy(input_array.begin(), input_array.end(), input);

    std::vector<std::vector<unsigned int>> permuted_indices;
    FullPermutation(0u, size, input, permuted_indices);

    return permuted_indices;
}

void Relocalizer::convertKeypoint2Bearing(const std::vector<cv::Point2d> &keypts,
                                          const Eigen::Matrix3d &intrinsic,
                                          std::vector<Eigen::Vector3d> &bearings) const
{
    bearings.resize(keypts.size());

    double cx = intrinsic(0, 2);
    double cy = intrinsic(1, 2);
    double fx = intrinsic(0, 0);
    double fy = intrinsic(1, 1);
    for (unsigned long idx = 0; idx < keypts.size(); ++idx)
    {
        const auto x_normalized = (keypts.at(idx).x - cx) / fx;
        const auto y_normalized = (keypts.at(idx).y - cy) / fy;
        const auto l2_norm = std::sqrt(x_normalized * x_normalized + y_normalized * y_normalized + 1.0);
        bearings.at(idx) = Eigen::Vector3d(x_normalized / l2_norm, y_normalized / l2_norm, 1.0 / l2_norm);
    }
}

bool Relocalizer::checkSampleKpts(const std::vector<unsigned int> &sample_kpt_indices, const std::vector<cv::Point2d> &v_keypoints)
{
    if (sample_kpt_indices.empty() || v_keypoints.empty())
    {
        ERROR_STREAM("[Relocalizer::checkSampleKpts] Empty sample_kpt_indices!");
        return false;
    }

    if (sample_kpt_indices.size() != 3 || v_keypoints.size() < 3)
    {
        ERROR_STREAM("[Relocalizer::checkSampleKpts] Wrong size of sample_kpt_indices!");
        return false;
    }

    // check the colinear conditions
    const cv::Point2d kpt_1 = v_keypoints[0];
    const cv::Point2d kpt_2 = v_keypoints[1];
    const cv::Point2d kpt_3 = v_keypoints[2];
    double tempy1 = (kpt_2.y - kpt_1.y);
    double tempx1 = (kpt_2.x - kpt_1.x);
    double tempy2 = (kpt_3.y - kpt_1.y);
    double tempx2 = (kpt_3.x - kpt_1.x);
    double xp = tempy1 * tempx2;
    double yp = tempy2 * tempx1;

    if (std::abs(xp - yp) <= kpt_colinear_angle_thre_)
    {
        WARN_STREAM("[Relocalizer::checkSampleKpts] Sampled keypoints are colinear!");
        return false;
    }
    else
        return true;
}

bool Relocalizer::checkSampleMappoints(const std::vector<unsigned int> &sample_map_indices, const std::vector<Eigen::Vector3d> &v_mappoints)
{
    if (sample_map_indices.empty() || v_mappoints.empty())
    {
        ERROR_STREAM("[Relocalizer::checkSampleKpts] Empty sample_map_indices!");
        return false;
    }

    if (sample_map_indices.size() != 3 || v_mappoints.size() < 3)
    {
        ERROR_STREAM("[Relocalizer::checkSampleKpts] Wrong size of sample_map_indices!");
        return false;
    }

    const unsigned int map_indice_1 = sample_map_indices[0];
    const unsigned int map_indice_2 = sample_map_indices[1];
    const unsigned int map_indice_3 = sample_map_indices[2];
    if (map_indice_1 >= v_mappoints.size() || map_indice_2 >= v_mappoints.size() || map_indice_3 >= v_mappoints.size())
    {
        ERROR_STREAM("[Relocalizer::checkSampleKpts] Invalid sample_map_indices!");
        return false;
    }

    const Eigen::Vector3d map_point_1 = v_mappoints[map_indice_1];
    const Eigen::Vector3d map_point_2 = v_mappoints[map_indice_2];
    const Eigen::Vector3d map_point_3 = v_mappoints[map_indice_3];

    double dist_1_2 = (map_point_1 - map_point_2).norm();
    double dist_1_3 = (map_point_1 - map_point_3).norm();
    double dist_2_3 = (map_point_2 - map_point_3).norm();

    if (dist_1_2 > triangle_edge_thre_ || dist_1_3 > triangle_edge_thre_ || dist_2_3 > triangle_edge_thre_)
    {
        // WARN_STREAM("[Relocalizer::checkSampleMappoints] Edge of sampled map points is greater than " << triangle_edge_thre_);
        return false;
    }

    return true;
}

// sollution is w2c
bool Relocalizer::checkCameraPose(const Eigen::Matrix4d &solution, const Eigen::Matrix4d &T_wc_gt)
{
    if (solution.hasNaN() || T_wc_gt.hasNaN())
    {
        ERROR_STREAM("[checkCameraPose] Invalid solution!");
        return false;
    }

    // evaluate the relocalize accurracy
    Eigen::Matrix4d error = solution * T_wc_gt.inverse();
    Eigen::Matrix3d error_R = error.block<3, 3>(0, 0);
    Eigen::Vector3d error_t = error.block<3, 1>(0, 3);
    Eigen::AngleAxisd err_vec(error_R);
    const double err_angle = err_vec.angle() * RAD_2_DEGREE;
    const double err_trans = error_t.norm();
    DEBUG_STREAM("[checkCameraPose] Rotation Error angle: " << err_angle);
    DEBUG_STREAM("[checkCameraPose] Translation Error: " << err_trans);

    if (err_angle < 5.0 && err_trans < 0.1)
    {
        DEBUG_STREAM("[checkPose] current pose is valid!");
        return true;
    }

    WARN_STREAM("[checkPose] current pose is invalid!");
    return false;
}

std::vector<std::vector<unsigned int>> Relocalizer::calc2DTriangles(const std::vector<cv::Point2d> &v_keypoints)
{
    if (v_keypoints.empty())
    {
        ERROR_STREAM("[Relocalizer::calc2DTriangles] Empty input keypoints!");
        return std::vector<std::vector<unsigned int>>();
    }

    std::vector<std::vector<unsigned int>> inter_vec;
    constexpr double angle_threshold = 20.0;
    constexpr double edge_ratio_threshold = 5;
    constexpr double edge_ratio_threshold_inv = 1.0 / edge_ratio_threshold;
    const double eps = std::numeric_limits<float>::epsilon();

    if (0)
    {
        size_t kpts_len = v_keypoints.size();

        for (size_t i = 0; i < kpts_len - 1; ++i)
        {
            const cv::Point2d &kpt_1 = v_keypoints[i];

            double best_min_dist = std::numeric_limits<double>::max();
            double secon_min_dist = std::numeric_limits<double>::max();
            int best_close_id = -1, secon_close_id = -1;

            for (size_t j = i + 1; j < kpts_len; ++j)
            {
                const cv::Point2d &kpt_2 = v_keypoints[j];
                double dist = kptsDistance(kpt_1, kpt_2);

                if (dist < best_min_dist)
                {
                    secon_min_dist = best_min_dist;
                    best_min_dist = dist;
                    best_close_id = j;
                }
                else if (dist < secon_min_dist)
                {
                    secon_min_dist = dist;
                    secon_close_id = j;
                }
            }

            if (best_close_id > 0 && secon_close_id > 0)
            {
                // check angle between 2 points
                const cv::Point2d kpt_2 = v_keypoints[best_close_id];
                const cv::Point2d kpt_3 = v_keypoints[secon_close_id];

                double angle_1_2 = getAngle(kpt_1, kpt_2, kpt_3);
                double angle_1_3 = getAngle(kpt_1, kpt_3, kpt_2);
                double angle_3_2 = getAngle(kpt_3, kpt_2, kpt_1);

                // angle check
                if (angle_1_2 <= angle_threshold || angle_1_3 <= angle_threshold || angle_3_2 <= angle_threshold)
                {
                    // std::cout << "angle CBA : " << angle_1_2 << "\n";
                    // std::cout << "angle ACB : " << angle_1_3 << "\n";
                    // std::cout << "angle BAC : " << angle_3_2 << "\n";
                    continue;
                }

                // edge check
                // const double edge_threshold = 300;
                // double edge_1 = kptsDistance(kpt_1, kpt_2);
                // double edge_2 = kptsDistance(kpt_1, kpt_3);
                // double edge_3 = kptsDistance(kpt_3, kpt_2);
                // if (edge_1 > edge_threshold || edge_2 > edge_threshold || edge_3 > edge_threshold)
                // {
                //     continue;
                // }

                std::vector<unsigned int> triangle_indices = {static_cast<unsigned int>(i),
                                                              static_cast<unsigned int>(best_close_id),
                                                              static_cast<unsigned int>(secon_close_id)};

                inter_vec.emplace_back(triangle_indices);
            }
        }
    }
    else
    {
        std::vector<double> v_coords;
        for (size_t i = 0; i < v_keypoints.size(); ++i)
        {
            v_coords.push_back(v_keypoints[i].x);
            v_coords.push_back(v_keypoints[i].y);
        }
        delaunator::Delaunator dKey(v_coords);

        const std::vector<size_t> &triangle_indices = dKey.triangles;
        for (size_t t = 0; t < triangle_indices.size(); t += 3)
        {
            unsigned int i1 = triangle_indices[t];
            unsigned int i2 = triangle_indices[t + 1];
            unsigned int i3 = triangle_indices[t + 2];

            const cv::Point2d &kpt_1 = v_keypoints[i1];
            const cv::Point2d &kpt_2 = v_keypoints[i2];
            const cv::Point2d &kpt_3 = v_keypoints[i3];

            // edge check
            double edge_1 = kptsDistance(kpt_1, kpt_2);
            double edge_2 = kptsDistance(kpt_1, kpt_3);
            double edge_3 = kptsDistance(kpt_3, kpt_2);
            if (edge_1 < eps || edge_2 < eps || edge_3 < eps)
            {
                continue;
            }

            // angle check
            double angle_1_2 = getAngle(kpt_1, kpt_2, kpt_3);
            double angle_1_3 = getAngle(kpt_1, kpt_3, kpt_2);
            double angle_3_2 = getAngle(kpt_3, kpt_2, kpt_1);
            if (angle_1_2 <= angle_threshold || angle_1_3 <= angle_threshold || angle_3_2 <= angle_threshold)
            {
                // std::cout << "angle CBA : " << angle_1_2 << "\n";
                // std::cout << "angle ACB : " << angle_1_3 << "\n";
                // std::cout << "angle BAC : " << angle_3_2 << "\n";
                continue;
            }

            // edge ratio check
            double ratio_1 = edge_1 / edge_2;
            double ratio_2 = edge_1 / edge_3;
            double ratio_3 = edge_2 / edge_3;
            if (ratio_1 <= edge_ratio_threshold_inv || ratio_1 >= edge_ratio_threshold ||
                ratio_2 <= edge_ratio_threshold_inv || ratio_2 >= edge_ratio_threshold ||
                ratio_3 <= edge_ratio_threshold_inv || ratio_3 >= edge_ratio_threshold)
            {
                continue;
            }

            std::vector<unsigned int> triangle_indices = {i1, i2, i3};

            inter_vec.emplace_back(triangle_indices);
        }
    }

    if (inter_vec.empty())
    {
        ERROR_STREAM("[Relocalizer::calc2DTriangles] Empty output!");
        return std::vector<std::vector<unsigned int>>();
    }

    return inter_vec;
}

bool Relocalizer::relocalize(const std::string &frameFileName,
                             const std::vector<cv::Point2d> &vUpKeyPoints,
                             const std::vector<cv::Point2d> &vLowKeyPoints,
                             const Eigen::Matrix3d &upCamIntrinsic,
                             const Eigen::Matrix3d &lowCamIntrinsic,
                             Eigen::Matrix4d &T_wc_up_cam,
                             Eigen::Matrix4d &extrinsic)
{
    if (vUpKeyPoints.size() < min_pnp_matches_nb_)
    {
        ERROR_STREAM("[relocalize] Input keypoints in up camera is less than " << min_pnp_matches_nb_);
        return false;
    }
    if (vLowKeyPoints.size() < min_pnp_matches_nb_)
    {
        ERROR_STREAM("[relocalize] Input keypoints in low camera is less than " << min_pnp_matches_nb_);
        return false;
    }
    if (frameFileName.empty())
    {
        ERROR_STREAM("[relocalize] Input frame image file name is empty!");
        return false;
    }
    if (upCamIntrinsic.hasNaN() || lowCamIntrinsic.hasNaN())
    {
        ERROR_STREAM("[Relocalizer] Invalid Input Intrinsic Matrix!");
        return false;
    }

    std::vector<cv::Point2d> v_up_valid_kpts;

    const int width_bound = img_width_ - img_bound_;
    const int height_bound = img_height_ - img_bound_;
    for (size_t i = 0; i < vUpKeyPoints.size(); ++i)
    {
        cv::Point2d kpt = vUpKeyPoints[i];
        if (img_bound_ < kpt.x && kpt.x < width_bound && img_bound_ < kpt.y && kpt.y < height_bound)
        {
            v_up_valid_kpts.emplace_back(kpt);
        }
    }
    if (v_up_valid_kpts.size() < min_pnp_matches_nb_)
    {
        ERROR_STREAM("[relocalize] Frame " << frameFileName << " Valid keypoints on up image is less than " << min_pnp_matches_nb_);
        return false;
    }

    std::map<int, int> up_camera_inlier_matches;
    double mean_reprojection_error = 0;
    bool sts = relocalizeByFrame(frameFileName, v_up_valid_kpts, upCamIntrinsic, T_wc_up_cam, up_camera_inlier_matches, mean_reprojection_error);
    if (!sts)
    {
        ERROR_STREAM("[relocalize] Fail to relocalize the up camera in frame " << frameFileName);
        return false;
    }

    // Eigen::Matrix4d T_wc_low_cam = Eigen::Matrix4d::Identity();
    // sts = relocalizeByFrame(frameIndex, vLowKeyPoints, lowCamIntrinsic, T_wc_low_cam);
    // if (!sts)
    // {
    //     ERROR_STREAM("[relocalize] Fail to relocalize the low camera in frame " << frameIndex);
    //     return false;
    // }

    // extrinsic = T_wc_low_cam.inverse() * T_wc_up_cam;

    return true;
}

bool Relocalizer::relocalizeByFrame(const std::string &frameFileName,
                                    const std::vector<cv::Point2d> &vValidPoints,
                                    const Eigen::Matrix3d &cameraIntrinsic,
                                    Eigen::Matrix4d &cameraPose,
                                    std::map<int, int> &inlierMatches,
                                    double &mean_reprojection_error)
{
    const auto tp_1 = std::chrono::steady_clock::now();
    // filte valid keypoints in image bound
    std::vector<Eigen::Vector3d> valid_bearings;

    const std::vector<std::vector<unsigned int>> v_sample_kpts_indices = calc2DTriangles(vValidPoints);
    if (v_sample_kpts_indices.empty())
    {
        ERROR_STREAM("[relocalize] Empty valid triangles in image!");
        return false;
    }

// debug the sampled keypoints and delauny result
#if DEBUG_2D_DELAUNY_RESULT
    {
        cv::Mat img_mat = cv::imread(frameFileName);

        // draw original keypoints
        for (size_t k = 0; k < vValidPoints.size(); ++k)
        {
            const cv::Point2d &kpt = vValidPoints[k];

            cv::line(img_mat, cv::Point(kpt.x - 20, kpt.y), cv::Point(kpt.x + 20, kpt.y),
                     cv::Scalar(255, 0, 0), 3);
            cv::line(img_mat, cv::Point(kpt.x, kpt.y + 20), cv::Point(kpt.x, kpt.y - 20),
                     cv::Scalar(255, 0, 0), 3);
        }

        // draw keypoints in 2d triangles
        for (size_t i = 0; i < v_sample_kpts_indices.size(); ++i)
        {

            cv::Scalar color;
            if (i % 2 == 0)
                color = cv::Scalar(0, randomPick(255), 0);
            else
                color = cv::Scalar(0, 0, randomPick(255));

            for (size_t j = 0; j < v_sample_kpts_indices[i].size(); ++j)
            {
                const cv::Point2d &sample_kpt = vValidPoints[v_sample_kpts_indices[i][j]];

                cv::line(img_mat, cv::Point(sample_kpt.x - 20, sample_kpt.y - 20), cv::Point(sample_kpt.x + 20, sample_kpt.y + 20),
                         color, 3);
                cv::line(img_mat, cv::Point(sample_kpt.x - 20, sample_kpt.y + 20), cv::Point(sample_kpt.x + 20, sample_kpt.y - 20),
                         color, 3);
            }
            // DEBUG_STREAM("[relocalizeByFrame] Sampled keypoints : " << v_sample_kpts_indices[i][0] << ", " << v_sample_kpts_indices[i][1] << ", " << v_sample_kpts_indices[i][2]);

            const cv::Point2d &sample_kpt_1 = vValidPoints[v_sample_kpts_indices[i][0]];
            const cv::Point2d &sample_kpt_2 = vValidPoints[v_sample_kpts_indices[i][1]];
            const cv::Point2d &sample_kpt_3 = vValidPoints[v_sample_kpts_indices[i][2]];
            cv::line(img_mat, sample_kpt_1, sample_kpt_2, cv::Scalar(0, 0, 255), 5);
            cv::line(img_mat, sample_kpt_1, sample_kpt_3, cv::Scalar(0, 0, 255), 5);
            cv::line(img_mat, sample_kpt_3, sample_kpt_2, cv::Scalar(0, 0, 255), 5);
        }

        std::string out_img_path = frameFileName.substr(0, frameFileName.find(".jpg")) + "_sample.jpg";
        cv::imwrite(out_img_path, img_mat);
    }
#endif

    // convert 2d keypoints to 3d bearings representation
    convertKeypoint2Bearing(vValidPoints, cameraIntrinsic, valid_bearings);

    // launch P3PSolver in a ransac
    int inlier_num = 0, best_inlier_num = 0;
    double mean_repro_err = std::numeric_limits<double>::max();
    double best_mean_repro_err;
    // map points' inlier flag
    std::map<int, int> inlier_matches, best_inlier_matches;
    Eigen::Matrix4d best_T_cw;

    fast_p3p::P3PSolver pnp_solver(valid_bearings, vValidPoints, v_map_points_, cameraIntrinsic, img_width_, img_height_);

    std::vector<unsigned int> valid_sample_kpts_indice;
    std::vector<unsigned int> valid_sample_map_indice;

    size_t max_iter_num = 1000000;
    double inlier_probability = 0.0;
    bool b_early_exit = false;
    // for (size_t k = 0; k < v_sample_kpts_indices.size(); ++k)
    for (size_t itr = 0; itr < max_iter_num; ++itr)
    {       
        DEBUG_STREAM("[relocalize] itr num : " << itr);
		size_t k = randomPick(v_sample_kpts_indices.size());

        if (b_early_exit)
            break;

        const std::vector<unsigned int> &sample_kpts_indice = v_sample_kpts_indices[k];

        for (size_t i = 0; i < v_map_point_combinations_.size(); ++i)
        {
            if (b_early_exit)
                break;

            const std::vector<unsigned int> &map_point_com = v_map_point_combinations_[i];
            std::vector<std::vector<unsigned int>> map_point_full_perm = calcFullPermutation(map_point_com);

            bool sts = true;
            for (size_t m = 0; m < map_point_full_perm.size(); m++)
            {
                if (b_early_exit)
                    break;

                const std::vector<unsigned int> &sample_mappoint_indices = map_point_full_perm[m];
                assert(sample_mappoint_indices.size() == sample_kpts_indice.size());

                // DEBUG_STREAM("[relocalize] Brute-Force traverse " << i << " : keypoints: "
                //                                                   << sample_kpts_indice[0] << " "
                //                                                   << sample_kpts_indice[1] << " "
                //                                                   << sample_kpts_indice[2] << " "
                //                                                   << " map points: " << sample_mappoint_indices[0] << " "
                //                                                   << sample_mappoint_indices[1] << " "
                //                                                   << sample_mappoint_indices[2]);

                sts = pnp_solver.calcTransform(sample_kpts_indice, sample_mappoint_indices);
                if (sts)
                {
                    inlier_num = pnp_solver.validateData(inlier_matches);
                    mean_repro_err = pnp_solver.getMinReprojectionError();

                    if (inlier_num > best_inlier_num)
                    {
                        best_inlier_num = inlier_num;
                        best_mean_repro_err = mean_repro_err;
                        best_inlier_matches = inlier_matches;

                        best_T_cw = pnp_solver.getTransformMatrix();

                        valid_sample_kpts_indice = sample_kpts_indice;
                        valid_sample_map_indice = sample_mappoint_indices;

                        // DEBUG_STREAM("[relocalize] Inlier number : " << best_inlier_num);
                        // DEBUG_STREAM("[relocalize] Mean reprojection error : " << best_mean_repro_err);
                        {
                            inlier_probability = (double)best_inlier_num / vValidPoints.size();
                            max_iter_num = itr + computeIterationsNumberAdaptively(inlier_probability, 3);
                        }
                        // DEBUG_STREAM("[relocalize] Max iteration number : " << max_iter_num);
                    }
                }
                else
                {
                    // WARN_STREAM("[relocalize] Fail to compute camera pose!");
                    continue;
                }

                if (best_inlier_num >= 0.95 * vValidPoints.size())
                {
                    DEBUG_STREAM("[relocalize] Exit Brute-force pnp early!");
                    b_early_exit = true;
                }
            }
        }
    }

    if (static_cast<unsigned int>(best_inlier_num) < min_valid_matches_nb_)
    {
        DEBUG_STREAM("[relocalize] Best Inlier num is less than " << min_valid_matches_nb_);
        for (std::map<int, int>::iterator it = best_inlier_matches.begin(); it != best_inlier_matches.end(); ++it)
        {
            if (it->second < 0)
                continue;
            // if (it->scond == 24 || it->second == 34 || it->second == 46)
            //     DEBUG_STREAM("[relocalize] map: " << it->first << " kpt: " << it->second);
            DEBUG_STREAM("[relocalize] map: " << it->first << " kpt: " << it->second);
        }
        return false;
    }
    else
    {
        cameraPose = best_T_cw.inverse();
        inlierMatches = best_inlier_matches;

        DEBUG_STREAM("[relocalize] Final Camera Pose : \n"
                     << cameraPose);
        DEBUG_STREAM("[relocalize] Final inlier num : " << best_inlier_num);
        mean_reprojection_error = best_mean_repro_err;
        DEBUG_STREAM("[relocalize] Best mean reprojection error : " << best_mean_repro_err);
        DEBUG_STREAM("[relocalize] Relocalization successfully!");

        // for (size_t i = 0; i < valid_sample_kpts_indice.size(); ++i)
        // {
        //     DEBUG_STREAM("[relocalize] valid map-kpt match: " << valid_sample_map_indice[i] << " -- " << valid_sample_kpts_indice[i]);
        // }
    }
    const auto tp_2 = std::chrono::steady_clock::now();
    const auto loc_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    TIMER_STREAM("[relocalize] camera localization time is " << loc_time << " s");

    return true;
}
