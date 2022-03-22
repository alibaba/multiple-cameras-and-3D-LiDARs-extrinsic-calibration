#ifndef __COST_FUCNTIONS_H__
#define __COST_FUCNTIONS_H__
#include <vector>
#include <limits.h>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "common.h"

/**
   * @brief Reprojection error item : set map points and features fixed, optimize transform parameters, intrinsic parameters are optional.
   *  transform * world_point = reproject_kpt
   *  Minimize the distance between obs_kpt and reproject_kpt after transform.
   */
struct ReprojectionError1
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ReprojectionError1(const cv::Point2d &obs_kpt,
                       const Eigen::Vector3d &obs_lms,
                       const Eigen::Matrix3d &intrinsic) : obs_kpt_(obs_kpt),
                                                           obs_lms_(obs_lms),
                                                           cam_intrinsic_(intrinsic)
    {
    }

    static ceres::CostFunction *Create(const cv::Point2d &obs_kpt,
                                       const Eigen::Vector3d &obs_lms,
                                       const Eigen::Matrix3d &intrinsic)
    {
        // 2 is the dimension of residual,
        // 6 is dimension of tranform
        // 4 i s the dimension of intrinsic parameters
        return (new ceres::AutoDiffCostFunction<ReprojectionError1, 2, 6, 4>(
            new ReprojectionError1(obs_kpt, obs_lms, intrinsic)));
    }

    // transform: 1x6 [alpha, beta, gamma, x, y, z], residuals: 1x3
    // intrinsic: 1x4 [fx,fy,cx,cy]
    template <typename T>
    bool operator()(const T *const transform, const T *const intrinsic, T *residuals) const
    {
        // world map point
        T world_point[3] = {T(obs_lms_(0)), T(obs_lms_(1)), T(obs_lms_(2))};
        // reprojected point in camera frame
        T reproject_point[3];
        // src_pt = src_pt * R
        ceres::AngleAxisRotatePoint(transform, world_point, reproject_point);

        // src_pt = src_pt + t
        reproject_point[0] += transform[3];
        reproject_point[1] += transform[4];
        reproject_point[2] += transform[5];

        //
        Eigen::Map<Eigen::Matrix<T, 3, 1>> point_c(reproject_point);
#if CERES_OPTIMZE_INTRINSIC
        T reproject_pixel[2];
        reproject_pixel[0] = point_c[0] * intrinsic[0] / point_c[2] + intrinsic[2];
        reproject_pixel[1] = point_c[1] * intrinsic[1] / point_c[2] + intrinsic[3];

        residuals[0] = T(obs_kpt_.x) - reproject_pixel[0];
        residuals[1] = T(obs_kpt_.y) - reproject_pixel[1];
#else
        Eigen::Matrix<T, 3, 1> reproject_pixel = cam_intrinsic_.template cast<T>() * point_c;
        // residual error is pixel distance
        residuals[0] = T(obs_kpt_.x) - (reproject_pixel(0) / reproject_pixel(2));
        residuals[1] = T(obs_kpt_.y) - (reproject_pixel(1) / reproject_pixel(2));
#endif

        return true;
    }

private:
    // observe 2D keypoints
    cv::Point2d obs_kpt_;
    // observe 3D landmarks
    Eigen::Vector3d obs_lms_;
    //
    Eigen::Matrix3d cam_intrinsic_;
};

/**
   * @brief Reprojection error item : used in concentric rotated stereo camera.
   * Add concenteric canstrain : optimize motor_rotation axis, R_cw 、 t_cw and intrinsic of camera 0, \
   * roation_angle in 6 camera pose, 
   *  transform * world_point = reproject_kpt
   *  Minimize the distance between obs_kpt and reproject_kpt after transform.
   */
struct ReprojectionErrorWithConcentricConstrain
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ReprojectionErrorWithConcentricConstrain(const cv::Point2d &obs_kpt,
                                             const Eigen::Vector3d &obs_lms,
                                             const Eigen::Matrix3d &intrinsic) : obs_kpt_(obs_kpt),
                                                                                 obs_lms_(obs_lms),
                                                                                 cam_intrinsic_(intrinsic)
    {
    }

    static ceres::CostFunction *Create(const cv::Point2d &obs_kpt,
                                       const Eigen::Vector3d &obs_lms,
                                       const Eigen::Matrix3d &intrinsic)
    {
        // 3 is the dimension of residual,
        // 3 is dimension of motor rotation axis
        // 1 is dimension of rotation angle
        // 3 is the dimension of rotaion delta vector
        // 6 is the dimension of R_cw_0 and t_cw_0
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorWithConcentricConstrain, 3, 3, 1, 3, 6>(
            new ReprojectionErrorWithConcentricConstrain(obs_kpt, obs_lms, intrinsic)));
    }

    // motor_axis: 1x3, Input is ^c1_T_c0
    // rotation_angle: radian, rotation respect to motor axis.
    // delta_0 : [x, y, z] is delta vector from motor axis center to camera center
    // transform : 1x6 camera pose of cam0
    // residuals: 1x3
    template <typename T>
    bool operator()(const T *const motor_axis,
                    const T *const rotation_angle,
                    const T *const delta_0,
                    const T *const transform,
                    T *residuals) const
    {
        // motor rotation : angle axis
        T motor_rotation[3] = {motor_axis[0] * rotation_angle[0], motor_axis[1] * rotation_angle[0], motor_axis[2] * rotation_angle[0]};
        Eigen::Matrix<T, 3, 3> Phi_i = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::AngleAxisToRotationMatrix(motor_rotation, Phi_i.data());

        Eigen::Matrix<T, 3, 3> R_cw_0 = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::AngleAxisToRotationMatrix(transform, R_cw_0.data());

        // global rotation : angle axis
        T motor_axis_inv[3] = {-motor_axis[0], -motor_axis[1], -motor_axis[2]};
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> tmp_vec(motor_axis_inv);
        Eigen::Matrix<T, 3, 1> global_axis_vec = R_cw_0.transpose().template cast<T>() * tmp_vec;
        T global_rotation[3] = {global_axis_vec[0] * rotation_angle[0],
                                global_axis_vec[1] * rotation_angle[0],
                                global_axis_vec[2] * rotation_angle[0]};

        Eigen::Matrix<T, 3, 3> A_i = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::AngleAxisToRotationMatrix(global_rotation, A_i.data());

        // R_cw_i
        Eigen::Matrix<T, 3, 3> R_cw_i = Phi_i * R_cw_0.template cast<T>();

        T rotation[3];
        ceres::RotationMatrixToAngleAxis(R_cw_i.data(), rotation);

        // delta vector
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> delta_vec(delta_0);
        // t_0 vector
        T t_0[3] = {transform[3], transform[4], transform[5]};
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_0_vec(t_0);

        // delta_i vector
        Eigen::Matrix<T, 3, 1> delta_i_vec = A_i * delta_vec;
        // t_cw_i
        Eigen::Matrix<T, 3, 1> trans_vec = Phi_i * (R_cw_0.template cast<T>() * (delta_i_vec - delta_vec) + t_0_vec);

        // world map point
        T world_point[3] = {T(obs_lms_(0)), T(obs_lms_(1)), T(obs_lms_(2))};
        // reprojected point in camera frame
        T reproject_point[3];
        // src_pt = src_pt * R
        ceres::AngleAxisRotatePoint(rotation, world_point, reproject_point);

        // src_pt = src_pt + t
        reproject_point[0] += trans_vec(0);
        reproject_point[1] += trans_vec(1);
        reproject_point[2] += trans_vec(2);

        //
        Eigen::Map<Eigen::Matrix<T, 3, 1>> point_c(reproject_point);

        Eigen::Matrix<T, 3, 1> reproject_pixel = cam_intrinsic_.template cast<T>() * point_c;
        reproject_pixel[0] /= reproject_pixel[2];
        reproject_pixel[1] /= reproject_pixel[2];

        // residual error is pixel distance
        residuals[0] = T(obs_kpt_.x) - reproject_pixel[0];
        residuals[1] = T(obs_kpt_.y) - reproject_pixel[1];
        // add constrain between global axis  and delta_0
        T perpendicular_err = global_axis_vec.transpose() * delta_vec;
        residuals[2] = perpendicular_err;

        return true;
    }

private:
    // observe 2D keypoints
    cv::Point2d obs_kpt_;
    // observe 3D landmarks
    Eigen::Vector3d obs_lms_;
    //
    Eigen::Matrix3d cam_intrinsic_;
};

/**
   * @brief Reprojection error item : used in jointly optimize concentric rotated stereo camera.
   * Add concenteric canstrain : optimize motor_rotation axis, R_cw 、 t_cw and intrinsic of camera 0, 
   * roation_angle in 6 camera pose, 
   *  transform * world_point = reproject_kpt
   *  Minimize the distance between obs_kpt and reproject_kpt after transform.
   */
struct ReprojectionErrorWithConcentricConstrain2
{

    ReprojectionErrorWithConcentricConstrain2(const cv::Point2d &obs_kpt,
                                              const Eigen::Vector3d &obs_lms,
                                              const Eigen::Matrix3d &intrinsic) : obs_kpt_(obs_kpt),
                                                                                  obs_lms_(obs_lms),
                                                                                  cam_intrinsic_(intrinsic)
    {
    }

    static ceres::CostFunction *Create(const cv::Point2d &obs_kpt,
                                       const Eigen::Vector3d &obs_lms,
                                       const Eigen::Matrix3d &intrinsic)
    {
        // 3 is the dimension of residual,
        // 3 is dimension of global rotation axis
        // 1 is dimension of rotation angle
        // 3 is the dimension of delta_O
        // 6 is the dimension of R_cw_0 and t_cw_0
        // 4 is the dimension of intrinsic parameters
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorWithConcentricConstrain2, 3, 3, 1, 3, 6, 4>(
            new ReprojectionErrorWithConcentricConstrain2(obs_kpt, obs_lms, intrinsic)));
    }

    // global_axis: 1x3, rotation axis in global world frame.
    // rotation_angle: 1x6, radian : rotation respect to motor axis.
    // delta_0 : [x, y, z] is delta vector from camera center to motor axis center.
    // transform : 1x6 [alpha, beta, gamma, x, y, z], camera pose of cam0.
    // intrinsic : 1x4 [fx,fy,cx,cy]
    // residuals: 1x3
    template <typename T>
    bool operator()(const T *const global_axis,
                    const T *const rotation_angle,
                    const T *const delta_0,
                    const T *const transform,
                    const T *const intrinsic,
                    T *residuals) const
    {
        Eigen::Matrix<T, 3, 3> R_cw_0 = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::AngleAxisToRotationMatrix(transform, R_cw_0.data());

        // global rotation : angle axis
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> global_axis_vec(global_axis);
        T global_rotation[3] = {global_axis_vec[0] * rotation_angle[0],
                                global_axis_vec[1] * rotation_angle[0],
                                global_axis_vec[2] * rotation_angle[0]};

        // motor rotation : angle axis
        Eigen::Matrix<T, 3, 1> motor_axis = -R_cw_0 * global_axis_vec;
        T motor_rotation[3] = {motor_axis[0] * rotation_angle[0], motor_axis[1] * rotation_angle[0], motor_axis[2] * rotation_angle[0]};
        Eigen::Matrix<T, 3, 3> Phi_i = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::AngleAxisToRotationMatrix(motor_rotation, Phi_i.data());

        Eigen::Matrix<T, 3, 3> A_i = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::AngleAxisToRotationMatrix(global_rotation, A_i.data());

        // R_cw_i
        Eigen::Matrix<T, 3, 3> R_cw_i = Phi_i * R_cw_0.template cast<T>();

        T rotation[3];
        ceres::RotationMatrixToAngleAxis(R_cw_i.data(), rotation);

        // delta vector
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> delta_vec(delta_0);
        // t_0 vector
        T t_0[3] = {transform[3], transform[4], transform[5]};
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_0_vec(t_0);

        // delta_i vector
        Eigen::Matrix<T, 3, 1> delta_i_vec = A_i * delta_vec;
        // t_cw_i
        Eigen::Matrix<T, 3, 1> trans_vec = Phi_i * (R_cw_0.template cast<T>() * (delta_i_vec - delta_vec) + t_0_vec);

        // world map point
        T world_point[3] = {T(obs_lms_(0)), T(obs_lms_(1)), T(obs_lms_(2))};
        // reprojected point in camera frame
        T reproject_point[3];
        // src_pt = src_pt * R
        ceres::AngleAxisRotatePoint(rotation, world_point, reproject_point);

        // src_pt = src_pt + t
        reproject_point[0] += trans_vec(0);
        reproject_point[1] += trans_vec(1);
        reproject_point[2] += trans_vec(2);

        //
        Eigen::Map<Eigen::Matrix<T, 3, 1>> point_c(reproject_point);
#if CERES_OPTIMZE_INTRINSIC
        T reproject_pixel[2];
        reproject_pixel[0] = point_c[0] * intrinsic[0] / point_c[2] + intrinsic[2];
        reproject_pixel[1] = point_c[1] * intrinsic[1] / point_c[2] + intrinsic[3];
#else
        Eigen::Matrix<T, 3, 1> reproject_pixel = cam_intrinsic_.template cast<T>() * point_c;
        reproject_pixel[0] /= reproject_pixel[2];
        reproject_pixel[1] /= reproject_pixel[2];
#endif
        // residual error is pixel distance
        residuals[0] = T(obs_kpt_.x) - reproject_pixel[0];
        residuals[1] = T(obs_kpt_.y) - reproject_pixel[1];
        // add constrain between global axis  and delta_0
        T perpendicular_err = global_axis_vec.transpose() * delta_vec;
        residuals[2] = perpendicular_err;

        return true;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // observe 2D keypoints
    cv::Point2d obs_kpt_;
    // observe 3D landmarks
    Eigen::Vector3d obs_lms_;
    //
    Eigen::Matrix3d cam_intrinsic_;
};

// cost function of stereo camera who has large overlapping
struct StereoReprojectionError1
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StereoReprojectionError1(const Eigen::Vector3d &observed_P,
                             const cv::Point2d &observed_p_l,
                             const cv::Point2d &observed_p_r,
                             const Eigen::Matrix3d &intrinsic_l,
                             const Eigen::Matrix3d &intrinsic_r)
        : object_point_(observed_P),
          image_point_l_(observed_p_l),
          image_point_r_(observed_p_r),
          left_cam_intrinsic_(intrinsic_l),
          right_cam_intrinsic_(intrinsic_r)
    {
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &observed_P,
                                       const cv::Point2d &observed_p_l,
                                       const cv::Point2d &observed_p_r,
                                       const Eigen::Matrix3d &intrinsic_l,
                                       const Eigen::Matrix3d &intrinsic_r)
    {
        // 4 is the dimension of residual,
        // 6 is the dimension of left camera pose
        // 6 is the dimension of  extrinsic parameters of T_r_l(w2c)
        return (new ceres::AutoDiffCostFunction<StereoReprojectionError1, 4, 6, 6>(
            new StereoReprojectionError1(observed_P, observed_p_l, observed_p_r, intrinsic_l, intrinsic_r)));
    }
    template <typename T>
    bool operator()(const T *const transform_l,
                    const T *const transform_extrinsic,
                    T *residuals) const
    {
        T P[3] = {T(object_point_(0)), T(object_point_(1)), T(object_point_(2))};

        T predicted_p_l[3];
        ceres::AngleAxisRotatePoint(transform_l, P, predicted_p_l);
        predicted_p_l[0] += transform_l[3];
        predicted_p_l[1] += transform_l[4];
        predicted_p_l[2] += transform_l[5];
        predicted_p_l[0] /= predicted_p_l[2];
        predicted_p_l[1] /= predicted_p_l[2];

        predicted_p_l[0] = T(left_cam_intrinsic_(0, 0)) * predicted_p_l[0] + T(left_cam_intrinsic_(0, 2));
        predicted_p_l[1] = T(left_cam_intrinsic_(1, 1)) * predicted_p_l[1] + T(left_cam_intrinsic_(1, 2));

        Eigen::Matrix<T, 3, 3> R_l = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::AngleAxisToRotationMatrix(transform_l, R_l.data());

        Eigen::Matrix<T, 3, 3> R_l_r = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::AngleAxisToRotationMatrix(transform_extrinsic, R_l_r.data());

        Eigen::Matrix<T, 3, 3> R_r = R_l_r * R_l;
        Eigen::Matrix<T, 3, 1> t_r;
        t_r(0) = transform_l[3];
        t_r(1) = transform_l[4];
        t_r(2) = transform_l[5];
        t_r = R_l_r * t_r;
        t_r(0) += transform_extrinsic[3];
        t_r(1) += transform_extrinsic[4];
        t_r(2) += transform_extrinsic[5];

        T predicted_p_r[3];
        T rotation[3];
        ceres::RotationMatrixToAngleAxis(R_r.data(), rotation);
        ceres::AngleAxisRotatePoint(rotation, P, predicted_p_r);
        predicted_p_r[0] += t_r[0];
        predicted_p_r[1] += t_r[1];
        predicted_p_r[2] += t_r[2];
        predicted_p_r[0] /= predicted_p_r[2];
        predicted_p_r[1] /= predicted_p_r[2];

        predicted_p_r[0] = T(right_cam_intrinsic_(0, 0)) * predicted_p_r[0] + T(right_cam_intrinsic_(0, 2));
        predicted_p_r[1] = T(right_cam_intrinsic_(1, 1)) * predicted_p_r[1] + T(right_cam_intrinsic_(1, 2));

        // residual error is pixel distance
        residuals[0] = T(image_point_l_.x) - predicted_p_l[0];
        residuals[1] = T(image_point_l_.y) - predicted_p_l[1];
        residuals[2] = T(image_point_r_.x) - predicted_p_r[0];
        residuals[3] = T(image_point_r_.y) - predicted_p_r[1];
        return true;
    }

private:
    // observed 3D point
    Eigen::Vector3d object_point_;

    // observed 2D point
    cv::Point2d image_point_l_;
    cv::Point2d image_point_r_;
    // left camera intrinsics
    Eigen::Matrix3d left_cam_intrinsic_;
    // left camera intrinsics
    Eigen::Matrix3d right_cam_intrinsic_;
};

// cost function of stereo camera who has no overlapping
struct StereoReprojectionError2
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StereoReprojectionError2(const Eigen::Vector3d &observed_P,
                             const cv::Point2d &observed_p,
                             const Eigen::Matrix3d &intrinsic)
        : object_point_(observed_P),
          image_point_(observed_p),
          cam_intrinsic_(intrinsic)
    {
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &observed_P,
                                       const cv::Point2d &observed_p,
                                       const Eigen::Matrix3d &intrinsic)
    {
        // 2 is the dimension of residual,
        // 6 is the dimension of rig pose
        // 6 is the dimension of  extrinsic parameters of camera relative to rig
        return (new ceres::AutoDiffCostFunction<StereoReprojectionError2, 2, 6, 6>(
            new StereoReprojectionError2(observed_P, observed_p, intrinsic)));
    }
    template <typename T>
    bool operator()(const T *const transform_base,
                    const T *const transform_extrinsic,
                    T *residuals) const
    {
        Eigen::Matrix<T, 3, 3> R_base = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::AngleAxisToRotationMatrix(transform_base, R_base.data());

        Eigen::Matrix<T, 3, 3> R_c_b = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::AngleAxisToRotationMatrix(transform_extrinsic, R_c_b.data());

        Eigen::Matrix<T, 3, 3> R_camera = R_c_b * R_base;
        Eigen::Matrix<T, 3, 1> t_camera;
        t_camera(0) = transform_base[3];
        t_camera(1) = transform_base[4];
        t_camera(2) = transform_base[5];
        t_camera = R_c_b * t_camera;
        t_camera(0) += transform_extrinsic[3];
        t_camera(1) += transform_extrinsic[4];
        t_camera(2) += transform_extrinsic[5];

        Eigen::Matrix<T, 3, 1> predicted_p = R_camera * object_point_.template cast<T>() + t_camera;
        predicted_p[0] /= predicted_p[2];
        predicted_p[1] /= predicted_p[2];

        predicted_p[0] = T(cam_intrinsic_(0, 0)) * predicted_p[0] + T(cam_intrinsic_(0, 2));
        predicted_p[1] = T(cam_intrinsic_(1, 1)) * predicted_p[1] + T(cam_intrinsic_(1, 2));

        // residual error is pixel distance
        residuals[0] = T(image_point_.x) - predicted_p[0];
        residuals[1] = T(image_point_.y) - predicted_p[1];
        return true;
    }

private:
    // observed 3D point
    Eigen::Vector3d object_point_;

    // observed 2D point
    cv::Point2d image_point_;
    // left camera intrinsics
    Eigen::Matrix3d cam_intrinsic_;
};

/**
   * @brief Config ceres optimzior and solve non-linear least-square optimzation problem.
   * @param problem is ceres optimization problem.
   * @return .
   */
static bool ceresOptimize(ceres::Problem &problem, const double gradient_tolerance)
{
    ceres::Solver::Options options;
    //default configuration: use a dense representation
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.preconditioner_type = ceres::JACOBI;
    //if sparse linear solver is available, use sparse configuration
    if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
    {
        options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
    }
    else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
    {
        options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
    }
    else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
    {
        options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
    }
    options.num_threads = 10;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = true;
    options.function_tolerance = 1e-6;
    options.gradient_tolerance = gradient_tolerance;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    DEBUG_STREAM(summary.FullReport());
    if (summary.termination_type != ceres::CONVERGENCE)
    {
        ERROR_STREAM("[Ceres Optimization]  NOT CONVERGENCE.");
        return false;
    }

    return true;
}
#endif