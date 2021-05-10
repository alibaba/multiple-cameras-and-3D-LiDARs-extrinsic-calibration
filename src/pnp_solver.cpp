#include <iostream>

#include <Eigen/Geometry>

#include "pnp_solver.h"
#include "params_config.h"

namespace fast_p3p
{

P3PSolver::P3PSolver(const std::vector<Eigen::Vector3d> &v_bearings,
                     const std::vector<cv::Point2d> &v_keypoints,
                     const std::vector<Eigen::Vector3d> &v_map_points,
                     const Eigen::Matrix3d &intrinsicMatrix,
                     const int img_width,
                     const int img_height) : best_inlier_nb_(0),
                                             best_reprojection_error_(std::numeric_limits<double>::max()),
                                             img_width_(img_width),
                                             img_height_(img_height)
{
    // assert(v_bearing_points.size() == v_map_points.size() && v_bearing_points.size() > 3);

    assert(v_keypoints.size() == v_bearings.size() && v_bearings.size() > 3);

    total_correspondeces_nb_ = v_bearings.size();

    // std::cout << "2D keypoints size : " << v_keypoints.size() << "\n";
    // std::cout << "Map points size : " << v_map_points.size() << "\n";
    // std::cout << "image width : " << img_width_ << "\n";
    // std::cout << "image height : " << img_height_ << "\n";

    v_bearing_set_ = v_bearings;
    v_keypoint_set_ = v_keypoints;
    v_map_point_set_ = v_map_points;

    if (intrinsicMatrix.hasNaN())
    {
        ERROR_STREAM("[P3PSolver] Invalid intrinsic Matrix!");
    }
    else
    {
        cam_intrinsic_ = intrinsicMatrix;
    }

    // debug the projected keypoints
    // std::string img_path = "/home/ziqianbai/Projects/vlab/3d_marker_localizer/build/20200402/undist_low/undist_20200402-224626-551d194e_i0_0.jpg";
    // debug_img_mat_ = cv::imread(img_path);
}

P3PSolver::~P3PSolver()
{
    // std::string out_img_path = "/home/ziqianbai/Projects/vlab/3d_marker_localizer/build/20200402/undist_low/undist_20200402-224626-551d194e_i0_0_projected.jpg";
    // cv::imwrite(out_img_path, debug_img_mat_);
}

void P3PSolver::solveQuarticPolynomial(const std::array<double, 5> &coeffs,
                                       std::array<double, 4> &real_roots)
{
    const double a = coeffs[0];
    const double b = coeffs[1] / a;
    const double c = coeffs[2] / a;
    const double d = coeffs[3] / a;
    const double e = coeffs[4] / a;

    const std::complex<double> Q1 = c * c - 3. * b * d + 12. * e;
    const std::complex<double> Q2 = 2. * c * c * c - 9. * b * c * d + 27. * d * d + 27. * b * b * e - 72. * c * e;
    const std::complex<double> Q3 = 8. * b * c - 16. * d - 2. * b * b * b;
    const std::complex<double> Q4 = 3. * b * b - 8. * c;

    auto complex_cbrt = [](const std::complex<double> &z) -> std::complex<double> { return pow(z, 1. / 3.); };
    const std::complex<double> Q5 = complex_cbrt(Q2 / 2. + sqrt(Q2 * Q2 / 4. - Q1 * Q1 * Q1));
    const std::complex<double> Q6 = (Q1 / Q5 + Q5) / 3.;
    const std::complex<double> Q7 = 2. * sqrt(Q4 / 12. + Q6);

    real_roots = {{(-b - Q7 - sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)).real() / 4.,
                   (-b - Q7 + sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)).real() / 4.,
                   (-b + Q7 - sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)).real() / 4.,
                   (-b + Q7 + sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)).real() / 4.}};
}

void P3PSolver::polishQuarticPolynomialRoots(const std::array<double, 5> &coeffs,
                                             std::array<double, 4> &roots,
                                             const int iterations)
{
    for (int i = 0; i < iterations; ++i)
    {
        for (auto &root : roots)
        {
            const double error =
                coeffs[4] + root * (coeffs[3] + root * (coeffs[2] + root * (coeffs[1] + root * coeffs[0])));

            const double derivative =
                coeffs[3] + root * (2 * coeffs[2] + root * ((4 * coeffs[0] * root + 3 * coeffs[1])));

            root -= error / derivative;
        }
    }
}

void P3PSolver::solveAP3P(const Eigen::MatrixXd &bearing_vectors,
                          const Eigen::MatrixXd &world_points,
                          std::vector<Eigen::Matrix4d> &solutions)
{
    assert(bearing_vectors.cols() >= 3 && world_points.cols() >= 3);
    const Eigen::Vector3d w1 = world_points.col(0);
    const Eigen::Vector3d w2 = world_points.col(1);
    const Eigen::Vector3d w3 = world_points.col(2);

    const Eigen::Vector3d b1 = bearing_vectors.col(0);
    const Eigen::Vector3d b2 = bearing_vectors.col(1);
    const Eigen::Vector3d b3 = bearing_vectors.col(2);

    // calculate k1, k2, k3
    const Eigen::Vector3d u0 = w1 - w2;
    const double nu0 = u0.norm();
    const Eigen::Vector3d k1 = u0.normalized();
    Eigen::Vector3d k3 = b1.cross(b2);
    const double nk3 = k3.norm();
    k3 = k3.normalized();

    const Eigen::Vector3d tz = b1.cross(k3);
    // ui, vi
    const Eigen::Vector3d v1 = b1.cross(b3);
    const Eigen::Vector3d v2 = b2.cross(b3);

    const Eigen::Vector3d u1 = w1 - w3;
    // coefficients related terms
    const double u1k1 = u1.dot(k1);
    const double k3b3 = k3.dot(b3);
    // f1i
    double f11 = k3b3;
    double f13 = k3.dot(v1);
    const double f15 = -u1k1 * f11;
    // delta
    const Eigen::Vector3d nl = u1.cross(k1).normalized();
    const double delta = u1.cross(k1).norm();
    f11 *= delta;
    f13 *= delta;
    // f2i
    const double u2k1 = u1k1 - nu0;
    double f21 = tz.dot(v2);
    double f22 = nk3 * k3b3;
    double f23 = k3.dot(v2);
    const double f24 = u2k1 * f22;
    const double f25 = -u2k1 * f21;
    f21 *= delta;
    f22 *= delta;
    f23 *= delta;
    const double g1 = f13 * f22;
    const double g2 = f13 * f25 - f15 * f23;
    const double g3 = f11 * f23 - f13 * f21;
    const double g4 = -f13 * f24;
    const double g5 = f11 * f22;
    const double g6 = f11 * f25 - f15 * f21;
    const double g7 = -f15 * f24;
    const std::array<double, 5> coeffs = {{g5 * g5 + g1 * g1 + g3 * g3, 2 * (g5 * g6 + g1 * g2 + g3 * g4),
                                           g6 * g6 + 2 * g5 * g7 + g2 * g2 + g4 * g4 - g1 * g1 - g3 * g3,
                                           2 * (g6 * g7 - g1 * g2 - g3 * g4), g7 * g7 - g2 * g2 - g4 * g4}};
    std::array<double, 4> s;
    solveQuarticPolynomial(coeffs, s);
    polishQuarticPolynomialRoots(coeffs, s);

    const Eigen::Vector3d temp = k1.cross(nl);

    Eigen::Matrix3d Ck1nl;
    Ck1nl << k1, nl, temp;

    Eigen::Matrix3d Cb1k3tzT;
    Cb1k3tzT << b1.transpose(), k3.transpose(), tz.transpose();

    const Eigen::Vector3d b3p = b3 * (delta / k3b3);

    for (const auto ctheta1p : s)
    {
        if (std::abs(ctheta1p) > 1)
            continue;
        const double stheta1p = ((k3b3 > 0) ? 1 : -1) * sqrt(1 - ctheta1p * ctheta1p);
        const double ntheta3 = stheta1p / ((g5 * ctheta1p + g6) * ctheta1p + g7);
        const double ctheta3 = (g1 * ctheta1p + g2) * ntheta3;
        const double stheta3 = (g3 * ctheta1p + g4) * ntheta3;

        Eigen::Matrix3d C13;
        C13 << ctheta3, 0.0, -stheta3, stheta1p * stheta3, ctheta1p, stheta1p * ctheta3, ctheta1p * stheta3, -stheta1p,
            ctheta1p * ctheta3;

        const Eigen::Matrix3d R = (Ck1nl * C13) * Cb1k3tzT;
        // R' * p3
        const Eigen::Vector3d rp3 = R.transpose() * w3;
        Eigen::Matrix4d T;
        T.setIdentity();
        T.block<3, 3>(0, 0) = R.transpose();
        Eigen::Vector3d t = (b3p * stheta1p) - rp3;
        T.block<3, 1>(0, 3) = t;
        solutions.emplace_back(T);
    }
}

// still use pixel error as reprojection error to evcaluate solution
int P3PSolver::evaluateSolutions(const Eigen::Matrix4d &solution,
                                 std::map<int, int> &valid_matches,
                                 double &mean_reprojection_error)
{
    int valid_measurement = initialMeasurement();

    valid_matches.clear();
    for (size_t i = 0; i < v_map_point_set_.size(); ++i)
    {
        valid_matches[i] = -1;
    }

    mean_reprojection_error = 0.0;

    double eps = std::numeric_limits<double>::epsilon();
    const double img_left_bound = ParamsConfig::GetImageBound();
    const double img_right_bound = img_width_ - img_left_bound;
    const double img_up_bound = ParamsConfig::GetImageBound();
    const double img_bottom_bound = img_height_ - img_up_bound;

    for (size_t i = 0; i < v_map_point_set_.size(); ++i)
    {

        Eigen::Vector3d map_point = v_map_point_set_[i];

        Eigen::Vector3d proj_point = (solution * map_point.homogeneous()).hnormalized();
        if (proj_point[2] <= eps)
        {
            continue;
        }

        Eigen::Vector3d img_point = cam_intrinsic_ * proj_point;
        double img_x = img_point[0] / img_point[2];
        double img_y = img_point[1] / img_point[2];

        if (img_left_bound < img_x && img_x < img_right_bound &&
            img_up_bound < img_y && img_y < img_bottom_bound)
        {
            for (size_t j = 0; j < v_keypoint_set_.size(); ++j)
            {
                double x_err = (img_x - v_keypoint_set_[j].x);
                double y_err = (img_y - v_keypoint_set_[j].y);
                double dist = std::sqrt(x_err * x_err + y_err * y_err);
                if (dist < reproject_err_thre_)
                {
                    valid_matches[i] = j;
                    valid_measurement += 1;
                    mean_reprojection_error += dist;

                    // {
                    //     const cv::Scalar color(0, 0, 255);
                    //     cv::line(debug_img_mat_, cv::Point(img_x - 20, img_y), cv::Point(img_x + 20, img_y),
                    //              color, 1);
                    //     cv::line(debug_img_mat_, cv::Point(img_x, img_y - 20), cv::Point(img_x, img_y + 20),
                    //              color, 1);
                    // }
                }
            }
        }
    }

    // valid_measurement must fall into [0, v_keypoint_set_.size()]
    if (valid_measurement == 0 || valid_measurement > static_cast<int>(v_keypoint_set_.size()))
    {
        valid_measurement = 0;
        mean_reprojection_error = std::numeric_limits<double>::max();
    }
    else
    {
        mean_reprojection_error /= valid_measurement;
    }

    return valid_measurement;
}

bool P3PSolver::calcTransform(const std::vector<unsigned int> &sample_kpt_indices,
                              const std::vector<unsigned int> &sample_map_indices)
{
    assert(sample_kpt_indices.size() > 2 && sample_kpt_indices.size() == sample_map_indices.size());

    Eigen::MatrixXd bearing_mat(3, 3);
    Eigen::MatrixXd world_mat(3, 3);
    for (int i = 0; i < 3; ++i)
    {
        if (sample_kpt_indices[i] >= v_bearing_set_.size() || sample_map_indices[i] >= v_map_point_set_.size())
        {
            ERROR_STREAM("[calcTransform] Invalid sample indices!");
            return false;
        }

        bearing_mat.col(i) = v_bearing_set_[sample_kpt_indices[i]];
        world_mat.col(i) = v_map_point_set_[sample_map_indices[i]];
    }

    // although AP3P is more robust to amgubility
    // we still judge degenerate conditions:
    // 1. colinear
    // 2. 2 points
    {
        const double eps = std::numeric_limits<double>::epsilon();
        double len01 = (world_mat.col(0) - world_mat.col(1)).norm();
        double len12 = (world_mat.col(1) - world_mat.col(2)).norm();
        double len02 = (world_mat.col(2) - world_mat.col(0)).norm();
        // 2 point
        if (len01 <= eps || len02 <= eps || len12 <= eps)
        {
            ERROR_STREAM("[P3PSolver::calcTransform] Invalid world map points!");
            return false;
        }
        // colinear, using heron's formula
        double p = 0.5 * (len01 + len02 + len12);
        double area = std::sqrt(p * (p - len01) * (p - len02) * (p - len12));
        if (area <= eps)
        {
            ERROR_STREAM("[P3PSolver::calcTransform] Invalid world map points' area!");
            return false;
        }
    }

    std::vector<Eigen::Matrix4d> solutions;
    solveAP3P(bearing_mat, world_mat, solutions);
    if (solutions.empty())
    {
        // ERROR_STREAM("[P3PSolver::calcTransform] Fail to solve relative pose from P3P!");
        return false;
    }

    best_inlier_nb_ = 0;
    // check each solutions to avoid amgubility
    for (size_t i = 0; i < solutions.size(); ++i)
    {
        // evaluate each solution on the whole input matches
        Eigen::Matrix4d T = solutions[i];
        if (T.hasNaN())
            continue;

        std::map<int, int> inlier_matches;
        double mean_reproject_error = 0.0;
        int measure = evaluateSolutions(T, inlier_matches, mean_reproject_error);
        if (measure < minimalDataNumber())
            continue;

        if (measure > best_inlier_nb_ || (measure == best_inlier_nb_ && mean_reproject_error < best_reprojection_error_))
        {
            T_cw_ = T;
            best_inlier_nb_ = measure;
            best_reprojection_error_ = mean_reproject_error;
            best_valid_matches_.swap(inlier_matches);
            // DEBUG_STREAM("[calcTransform] best_inlier_num: " << best_inlier_nb_);
            // DEBUG_STREAM("[calcTransform] best_mean_reprojection_error: " << best_reprojection_error_);
        }
    }

    return true;
}

int P3PSolver::validateData(std::map<int, int> &inlier_matches)
{
    if (best_inlier_nb_ > 0)
    {
        inlier_matches.swap(best_valid_matches_);
        return best_inlier_nb_;
    }

    for (size_t i = 0; i < v_map_point_set_.size(); ++i)
    {
        inlier_matches[i] = -1;
    }
    return worstMeasurement();
}

} // namespace fast_p3p
