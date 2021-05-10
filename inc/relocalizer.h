#ifndef __RELOCALIZER_H
#define __RELOCALIZER_H

#include <vector>
#include <memory>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "pnp_solver.h"

class Relocalizer
{

public:
  using Ptr = std::shared_ptr<Relocalizer>;

  /**
   * @brief Relocalize camera in given map points.
   * Actually, it should be called Localizer.
   * @param vMapPoints is the ground truth map points.
   * @param min_valid_matches_thre is the minimal inlier 2D-3D matches after RANSAC.
   * @param min_pnp_matches_nb is the minimal 2D-3D matches using in pnp solver.
   * @param img_width is the width of input image used in calibration.
   * @param img_height is the height of input image used in calibration.
   */
  explicit Relocalizer(const std::vector<Eigen::Vector3d> &vMapPoints,
                       unsigned int min_valid_matches_thre,
                       unsigned int min_pnp_matches_nb = 3,
                       int img_width = 1280,
                       int img_height = 800);
  ~Relocalizer();

  /**
  * @brief relocalize upper camera  and low camera, and calcuate the extrinsic between them.
  * @param vUpKeyPoints is cctag keypoints detected in upper camera.
  * @param vLowKeyPoints is cctag keypoints detected in low camera.
  * @param upCamIntrinsic is the camera intrinsic matrix of upper camera.
  * @param lowCamIntrinsic is the camera intrinsic matrix of low camera.
  * @param T_wc_up_cam is the camera pose of up camera.
  * @param extrinsic is .
  * @return .
  */
  bool relocalize(const std::string &frameFileName,
                  const std::vector<cv::Point2d> &vUpKeyPoints,
                  const std::vector<cv::Point2d> &vLowKeyPoints,
                  const Eigen::Matrix3d &upCamIntrinsic,
                  const Eigen::Matrix3d &lowCamIntrinsic,
                  Eigen::Matrix4d &T_wc_up_cam,
                  Eigen::Matrix4d &extrinsic);

  /**
  * @brief relocalize camera in given map.
  * @param frameFileName is image file name.
  * @param vValidPoints is cctag keypoints detected by camera.
  * @param cameraIntrinsic is the camera intrinsic matrix.
  * @param cameraPose is the returned camera pose.
  * @param inlierMatches is the returned inlier 3D-2D matches.
  * @param mean_reprojection_error is the ultimately mean reprojection error of the camera pose.
  * @return .
  */
  bool relocalizeByFrame(const std::string &frameFileName,
                         const std::vector<cv::Point2d> &vValidPoints,
                         const Eigen::Matrix3d &cameraIntrinsic,
                         Eigen::Matrix4d &cameraPose,
                         std::map<int, int> &inlierMatches,
                         double &mean_reprojection_error);

  /**
  * @brief Convert 2d keypoint to bearing vector.
  * @param keypts is vector of detected cctag keypoints.
  * @param bearings is vector of bearing.
  * @return .
  */
  void convertKeypoint2Bearing(const std::vector<cv::Point2d> &keypts,
                               const Eigen::Matrix3d &intrinsic,
                               std::vector<Eigen::Vector3d> &bearings) const;

private:
  /**
  * @brief get angle ACB, point C is the center point.
  * @param pointA .
  * @param pointB .
  * @param pointC .
  * @return .
  */
  inline double getAngle(const cv::Point2d &pointA, const cv::Point2d &pointB, const cv::Point2d &pointC)
  {
    double theta = std::atan2(pointA.x - pointC.x, pointA.y - pointC.y) - atan2(pointB.x - pointC.x, pointB.y - pointC.y);
    if (theta > M_PI)
      theta -= 2 * M_PI;
    if (theta < -M_PI)
      theta += 2 * M_PI;

    theta = std::abs(theta * RAD_2_DEGREE);
    return theta;
  }

  /**
  * @brief 2d keypoints' distance.
  * @param .
  * @param .
  * @return .
  */
  inline double kptsDistance(const cv::Point2d &a, const cv::Point2d &b)
  {
    const double x = a.x - b.x;
    const double y = a.y - b.y;
    return std::sqrt(x * x + y * y);
  }

  /**
  * @brief Calculate full combination of map points, the filte each combination by distance.
  * @param map_points_num is the number of map points.
  * @param sample_num is the combination number.
  * @return 2d array of valid combinations.
  */
  std::vector<std::vector<unsigned int>> calcFullCombination(unsigned int map_points_num, unsigned int sample_num);
  /**
  * @brief Calculate full permutation of input array.
  * @param input_array is input data.
  * @return 2d array of full permutation of input.
  */
  std::vector<std::vector<unsigned int>> calcFullPermutation(const std::vector<unsigned int> &input_array);

  bool checkSampleKpts(const std::vector<unsigned int> &sample_kpt_indices, const std::vector<cv::Point2d> &v_keypoints);

  /**
  * @brief Check sampled map points by distance.
  * @param sample_map_indices .
  * @param v_mappoints.
  * @return valid map points or not.
  */
  bool checkSampleMappoints(const std::vector<unsigned int> &sample_map_indices, const std::vector<Eigen::Vector3d> &v_mappoints);

  /**
  * @brief Check camera pose by ground_truth.
  * @param solution is w2c camera pose .
  * @param T_wc_gt is w2c ground truth.
  * @return valid camera pose or not.
  */
  bool checkCameraPose(const Eigen::Matrix4d &solution, const Eigen::Matrix4d &T_wc_gt);

  /**
  * @brief Enumerate 2D triangles, using delaunay method or native method.
  * @param v_keypoints is total keypoints.
  * @return vector of valid triangles' indices.
  */
  std::vector<std::vector<unsigned int>> calc2DTriangles(const std::vector<cv::Point2d> &v_keypoints);

  // global map points
  std::vector<Eigen::Vector3d> v_map_points_;

  std::vector<std::vector<unsigned int>> v_map_point_combinations_;
  //
  unsigned long total_map_points_nb_;
  //! minimum threshold of the number of valid (= inlier after pose optimization) matches
  const unsigned int min_valid_matches_nb_;
  // minimum 2D-3D matches feed into pnp
  const size_t min_pnp_matches_nb_;

  int img_width_;
  int img_height_;
  // image bound
  const int img_bound_ = 40;

  // used to jude if keypoints are colinear
  const double kpt_colinear_angle_thre_ = std::tan(10.0);
  // edge distance threshold of triangle that is made up of map points
  const double triangle_edge_thre_ = 0.5;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif