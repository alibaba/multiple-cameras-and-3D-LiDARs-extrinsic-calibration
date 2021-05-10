#ifndef __MONO_CALIBRATION_H__
#define __MONO_CALIBRATION_H__

#include <vector>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "camera.h"
#include "target_board.h"
#include "relocalizer.h"

class MonoCalibrator
{

public:
    using Ptr = std::shared_ptr<MonoCalibrator>;

    MonoCalibrator() = delete;

    /**
   * @brief Constructor for monocular camera calibration.
   * @param camModelType is camera model type.
   * @param targetModelType is type of target board.
   * @param intrinsicFileName is the file name of intrinsic file for camera.
   * @param targetFileName is file name of traget board.
   */
    MonoCalibrator(camera::model_type_t camModelType,
                   target::target_type_t targetModelType,
                   const std::string &intrinsicFileName,
                   const std::string &targetFileName);

    /**
   * @brief Constructor for monocular camera calibration.
   * @param camModelType is camera model type.
   * @param intrinsicFileName is the file name of intrinsic file for camera.
   * @param vGlobalMap is the global map points.
   * @param relocalizerPtr is the pointer to the relocalizor.
   * @return .
   */
    MonoCalibrator(camera::model_type_t camModelType,
                   const std::string &intrinsicFileName,
                   const std::vector<Eigen::Vector3d> &vGlobalMap,
                   std::shared_ptr<Relocalizer> &relocalizerPtr);

    void clear(void);

    bool addChessboardData(const std::vector<cv::Point2f> &v_corners);
    bool addApriltagboardData(const std::vector<cv::Point2f> &v_corners);
    bool addCctagData(const std::vector<cv::Point2d> &v_centers);
    bool addCctagData(const std::string &imgFilePath, const std::vector<cv::Point2d> &v_centers);

    bool calibrate(void);

    camera::Camera::Ptr &camera();
    /**
   * @brief Get total count of sample images.
   * @return number of sample images.
   */
    int sampleCount(void) const;

    /**
   * @brief Get 2d keypoints on image plane.
   * @return 2d array of detected keypoints.
   */
    std::vector<cv::Point2d> &imagePoints(unsigned int imgIdx);
    const std::vector<cv::Point2d> imagePoints(unsigned int imgIdx) const;

    std::vector<std::vector<cv::Point2d>> &imagePoints();
    const std::vector<std::vector<cv::Point2d>> imagePoints() const;

    /**
   * @brief Get 3d map points in world frame.
   * @return array of map points.
   */
    const std::vector<Eigen::Vector3d> objectPoints() const;
    /**
   * @brief Get 2d keypoints on image plane.
   * @return 2d array of detected keypoints.
   */
    std::vector<cv::Point2d> &inlierImagePoints(unsigned int imgIdx);
    const std::vector<cv::Point2d> inlierImagePoints(unsigned int imgIdx) const;

    /**
   * @brief Get total 3d object points in scene/object frame.
   * @return array of object points.
   */
    // std::vector<Eigen::Vector3d> &MonoCalibrator::objectPoints(void);
    // const std::vector<Eigen::Vector3d> MonoCalibrator::objectPoints(void) const;
    /**
   * @brief Get inlier 3d object points of each frame in scene/object frame.
   * @return array of object points.
   */
    std::vector<Eigen::Vector3d> &inlierObjectPoints(unsigned int imgIdx);
    const std::vector<Eigen::Vector3d> inlierObjectPoints(unsigned int imgIdx) const;
    /**
   * @brief Get 3d-2d inlier matches of each image frame.
   * @return 3d-2d indices map.
   */
    std::map<int, int> &inlierMatchesIdx(unsigned int imgIdx);
    std::map<int, int> inlierMatchesIdx(unsigned int imgIdx) const;

    double &meanReprojectionError(unsigned int imgIdx);

    bool isValid(unsigned int imgIdx) const;

    Eigen::Matrix4d &cameraPose(unsigned int imgIdx);
    std::vector<Eigen::Matrix4d> validCameraPoses() const;

    /**
   * @brief Extract valid indice after localization.
   * @param inlierMatches is the map of inlier map points' indices and inlier 2d keypoints' indices.
   * @param kpt_inlier_indices is the inlier 2d keypoints indices.
   * @param map_inlier_indices is is the inlier map points indices
   * @return valid 2d keypoints.
   */
    bool extractValidIndices(const std::map<int, int> &inlierMatches,
                             std::vector<unsigned int> &kpt_inlier_indices,
                             std::vector<unsigned int> &map_inlier_indices);

    void drawResults(std::vector<cv::Mat> &images) const;

    /**
   * @brief Check inliers of optimized camera pose.
   * @param v_map_points .
   * @param v_keypoints .
   * @param cam_intrinsic .
   * @param rotation .
   * @param translation .
   * @param mean_reprojection_error .
   * @return inlier numebr.
   */
    static unsigned int checkInliers(const std::vector<Eigen::Vector3d> &v_map_points,
                                     const std::vector<cv::Point2d> &v_keypoints,
                                     const Eigen::Matrix3d &cam_intrinsic,
                                     const Eigen::Matrix3d &rotation,
                                     const Eigen::Vector3d &translation,
                                     double &mean_reprojection_error);

private:
    bool optimize(void);

    // camera pointer
    camera::Camera::Ptr camera_ptr_;
    // w2c camera pose od each sample image
    std::vector<Eigen::Matrix4d> v_T_wc_;

    // target board pointer
    target::TargetBoard::Ptr target_ptr_;

    // relocalizer pointer
    Relocalizer::Ptr relocalizer_ptr_;

    // 2d keypoints on image plane
    std::vector<std::vector<cv::Point2d>> vv_img_points_;
    // sample image frames file path
    std::vector<std::string> sample_img_pathes_;
    // 3d points in object space
    std::vector<Eigen::Vector3d> v_object_points_;
    // flag whether image frame was relocalized successfully
    std::vector<bool> v_relocalized_flag_;
    // inliers in image sequence
    std::vector<std::vector<cv::Point2d>> vv_inlier_imgPoints_;
    std::vector<std::vector<Eigen::Vector3d>> vv_inlier_objPoints_;
    std::vector<std::map<int, int>> vv_inlier_matches_;
    // mean reprojection error of each relocalized image frame
    std::vector<double> v_mean_reprojection_error_;
    // reprojection error threshold
    static const double reproject_err_thre_;

    // the map bounding box
    MapBoundingBox map_bound_box_;
};
#endif /* __MONO_CALIBRATION_H__ */