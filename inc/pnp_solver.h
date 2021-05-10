#ifndef __PNP_SOLVER_H
#define __PNP_SOLVER_H

#include <vector>
#include <array>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "common.h"

namespace fast_p3p
{

class P3PSolver
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief .
   * @param v_bearings is the observations on sphere.
   * @param v_map_points is the 3d points in world frame.
   */
  explicit P3PSolver(const std::vector<Eigen::Vector3d> &v_bearing_points,
                     const std::vector<cv::Point2d> &v_keypoints,
                     const std::vector<Eigen::Vector3d> &v_map_points,
                     const Eigen::Matrix3d &intrinsicMatrix,
                     const int img_width = 1280,
                     const int img_height = 800);

  ~P3PSolver();

  static void solveQuarticPolynomial(const std::array<double, 5> &coeffs,
                                     std::array<double, 4> &real_roots);

  static void polishQuarticPolynomialRoots(const std::array<double, 5> &coeffs,
                                           std::array<double, 4> &roots,
                                           const int iterations = 2);

  static void solveAP3P(const Eigen::MatrixXd &bearing_vectors,
                        const Eigen::MatrixXd &world_points,
                        std::vector<Eigen::Matrix4d> &solutions);

  inline int worstMeasurement() const { return 0; }
  inline int initialMeasurement() const { return 0; }
  /**
   * @brief Minimal reuqired matches number for ap3p
   * @return .
   */
  inline int minimalDataNumber() const { return 3; }

  /**
   * @brief Get camera pose.
   * @return .
   */
  inline Eigen::Matrix4d const getTransformMatrix() { return T_cw_; }

  inline double getMinReprojectionError() const { return best_reprojection_error_; }

  /**
   * @brief Get inlier 3D-2D matches.
   * @param inlier_matches is the inlier 3D-2D matches.
   * @return inlier number.
   */
  int validateData(std::map<int, int> &inlier_matches);

  /**
   * @brief Calculate camera pose use given 2D - 3D pairs
   * @param sample_kpt_indices is 3 kepypoints indices.
   * @param sample_map_indices is 3 map points indices.
   * @return True/False if 3-point pose estimation succeeded/failed
   */
  bool calcTransform(const std::vector<unsigned int> &sample_kpt_indices,
                     const std::vector<unsigned int> &sample_map_indices);

private:
  /**
     * @brief Evaluate solution calculated by ap3p.
     * @param solution is the camera pose calculated by ap3p.
     * @param valid_matches is 3D-2D inlier matches passed evaluation.
     * @param mean_reprojection_error is mean reprojection error of current solution.
     * @return inlier numebr of current solution.
     */
  int evaluateSolutions(const Eigen::Matrix4d &solution,
                        std::map<int, int> &valid_matches,
                        double &mean_reprojection_error);

  // 3D Points
  std::vector<Eigen::Vector3d> v_map_point_set_;

  // 3d bearings
  std::vector<Eigen::Vector3d> v_bearing_set_;

  // 2d keypoints
  std::vector<cv::Point2d> v_keypoint_set_;

  //
  unsigned int total_correspondeces_nb_;
  // transform matrix,c2w
  Eigen::Matrix4d T_cw_;
  // intrinsic matrix
  Eigen::Matrix3d cam_intrinsic_;
  // radian threshold is the squared reprojection error
  const double reproject_err_thre_ = 5.0;
  //
  // std::vector<bool> best_inlier_flags_;
  std::map<int, int> best_valid_matches_;
  // the inliers num of best model
  int best_inlier_nb_;
  // the reprojection error of best model
  double best_reprojection_error_;

  double img_width_;
  double img_height_;

  cv::Mat debug_img_mat_;
};

} // namespace fast_p3p

#endif