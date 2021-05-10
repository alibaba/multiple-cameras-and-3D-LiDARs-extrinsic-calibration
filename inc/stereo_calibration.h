#ifndef __STEREO_CALIBRATION_H_
#define __STEREO_CALIBRATION_H_

#include <vector>
#include <limits.h>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "relocalizer.h"
#include "camera.h"
#include "mono_calibration.h"

class StereoCalibrator
{
    // calibrator state
    enum class calibrator_state_t
    {
        //
        NotInitialized,
        // calibrate first frame
        Initializing,
        // calibrate sequencial frames on one camera
        CalibratingSeperately,
        // calibrate stereo camera jointly
        CalibratingJointly,
        // fatal error
        Error
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StereoCalibrator() = delete;
    /**
   * @brief Stereo camera calibrator, made up of the left camera and right camera.
   * @param camModelType is camera model type.
   * @param targetModelType is target board type.
   * @param leftIntrinsicFileName is left camera intrinsic file name.
   * @param rightIntrinsicFileName is right camera intrinsic file name.
   * @param targetFileName is target board config file name.
   * @param b_homogeneous indicate the left camera and right camera are homogeneous or not.
   */
    StereoCalibrator(camera::model_type_t camModelType,
                     target::target_type_t targetModelType,
                     const std::string &leftIntrinsicFileName,
                     const std::string &rightIntrinsicFileName,
                     const std::string &targetFileName,
                     const bool b_homogeneous = true);

    void clear(void);

    bool addChessboardData(const std::vector<cv::Point2f> &cornersLeft,
                           const std::vector<cv::Point2f> &cornersRight);
    bool addApriltagboardData(const std::vector<cv::Point2f> &corners);
    bool addCctagData(const std::vector<cv::Point2d> &leftCenters, const std::vector<cv::Point2d> &rightCenters);
    bool addCctagData(const std::string &leftImgFile, const std::vector<cv::Point2d> &leftCenters,
                      const std::string &rightImgFile, const std::vector<cv::Point2d> &rightCenters);

    /**
   * @brief calibrate stereo image from stereo camera.
   * @param upImgFileName is the image file name of upper camera.
   * @param lowImgFileName is the image file name of lower camera.
   * @param vUpKeyPoints is the detected keypoints of upper camera.
   * @param vLowKeyPoints is the detected keypoints of low camera.
   * @return .
   */
    bool calibrate(void);

    int sampleCount(void) const;

    /**
   * @brief Get 3d map points in world frame.
   * @return array of map points.
   */
    const std::vector<Eigen::Vector3d> objectPoints() const;

    camera::Camera::Ptr &leftCamera(void);

    camera::Camera::Ptr &rightCamera(void);

    /**
   * @brief Get valid stereo camera pose pair in w2c form.
   * @return array of camera pose pair, first is left camera pose, second is right camera pose.
   */
    std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> validCameraPosePairs(void) const;

    bool writeExtrinsicParams(const std::string &directory) const;

private:
    int readImageWidth(const std::string &filename);
    int readImageHeight(const std::string &filename);

    // indicate whether the multi-camera system is homogeneous of heterogeneous
    bool b_homo_multicamera_;

    // target board pointer
    target::TargetBoard::Ptr target_ptr_;

    MonoCalibrator::Ptr left_calibrator_ptr_;
    MonoCalibrator::Ptr right_calibrator_ptr_;

    // scene map points
    std::vector<Eigen::Vector3d> v_object_points_;

    // transform from left camera to right camera, in w2c
    Eigen::Matrix4d T_r_l_;

    // sampled stereo image number
    int samples_num_;
};

#endif /* __STEREO_CALIBRATION_H_*/
