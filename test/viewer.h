/**
 * @file viewer.h
 * @author 
 * @brief Use pangolin for visualization.
 * @date 2020-07-20
 *
 * @copyright Copyright (c) Alibaba Inc 2020. All rights reserved.
 *
 */
#ifndef WALLE_LOCALIZATION_VIEWER_H_
#define WALLE_LOCALIZATION_VIEWER_H_

#include <memory>
#include <unordered_map>
#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <unistd.h>

#include <pangolin/pangolin.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


enum class KeyFrameColor {
  BLACK_KEYFRAME = 0,
  RED_KEYFRAME = 1,
  BLUE_KEYFRAME = 2,
  GREEN_KEYFRAME = 3
};

class Viewer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Viewer>;

  /**
   * @brief Explicit constructor.
   */
  Viewer();
  Viewer(const std::vector<Eigen::Matrix4d> &v_cam_poses);
  Viewer(const Viewer &) = delete;
  Viewer &operator=(const Viewer &) = delete;

  /**
   * @brief Main thread function. Draw points, keyframes, the current camera
   * pose and the last processed frame. Drawing is refreshed according to the
   * camera fps.
   * @return .
   */
  void run();

  /**
   * @brief Finish visualization thread and pangolin window.
   * Called by user.
   * @return .
   */
  void requestFinish();

  /**
   * @brief Stop visualization thread and pangolin window refresh.
   * Called by user.
   * @return .
   */
  void requestStop();

  void setCurrentCameraPose(const Eigen::Matrix4d &Twc);
  void setTargetCameraPose(const Eigen::Matrix4d &Twc);

  void setCurrentMapPoints(const std::vector<Eigen::Vector3d> &v_map_points);

  void setPnPPoints(const std::vector<Eigen::Vector3d> &v_map_points);

 private:
  /**
   * @brief Check finish flag is set.
   * @return .
   */
  bool isFinished();

  /**
   * @brief Check stop flag is set.
   * @return .
   */
  bool isStopped();

  /**
   * @brief Resume state from stop to activated.
   * @return .
   */
  void release();

  /**
   * @brief Check finish flag and stop flag is set.
   * @return .
   */
  bool stop();

  void getCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
  void drawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
  void drawMapPoints();
  void drawPnPPoints();
  void drawCurrentCamera(pangolin::OpenGlMatrix &Twc);

  /**
   * @brief Draw current camera pose.
   * @param kyfrm_color is the color of drawing keyframe.
   * @param twc is camera pose in w2c.
   * @return .
   */
  void drawTargetCamera(const KeyFrameColor &kyfrm_color,
                        const Eigen::Matrix4f &twc);
  bool checkFinish();
  void setFinish();

  bool b_finish_request_;
  bool b_finished_;
  std::mutex mutex_finish_;

  bool b_stopped_;
  bool b_stopp_request_;
  std::mutex mutex_stop_;
  // view point position
  float viewpoint_x_, viewpoint_y_, viewpoint_z_;
  // viewpoint fu, fv
  float viewpoint_f_;
  //
  float point_size_;
  // visualized keyframe size, define the keyframe width, height and depth
  float keyframe_size_;
  // line width used in keyframe visualization
  float keyframe_line_width_;
  //
  float graph_line_width_;
  //
  float camera_size_;
  //
  float camera_line_width_;

  std::mutex mutex_camera_;
  Eigen::Matrix4d T_curr_pose_w_c_;
  Eigen::Matrix4d T_target_pose_w_c_;
  std::vector<Eigen::Matrix4d> v_T_poses_w_c_;

  std::vector<Eigen::Vector3d> v_map_points_;
  std::vector<Eigen::Vector3d> v_pnp_points_;
  std::mutex mutex_map_points_;
};


#endif  // VIEWER_H