#ifndef _3D_MARKER_LOCALIZER_CAMERA_H
#define _3D_MARKER_LOCALIZER_CAMERA_H

#include <string>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace camera
{

struct image_bounds
{
    //! Default constructor
    image_bounds() = default;

    //! Constructor with initialization
    template <typename T, typename U>
    image_bounds(const T min_x, const U max_x, const T min_y, const U max_y)
        : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y) {}

    float min_x_ = 0.0;
    float max_x_ = 0.0;
    float min_y_ = 0.0;
    float max_y_ = 0.0;
};

enum class setup_type_t
{
    Monocular = 0,
    Stereo = 1,
    RGBD = 2
};

enum class stereo_type_t
{
    // ordinary stereo camera
    Standard = 0,
    // stereo camera with common camera center
    Rotation_concentric = 1,
    // stereo camera without common camera center, but common rotation axis
    Rotation_unconcentric = 2
};

// currently only support pinhole and fisheye model
enum class model_type_t
{
    Pinhole = 0,
    Fisheye = 1,
    Equirectangular = 2
};

class Camera
{

public:
    using Ptr = std::shared_ptr<Camera>;

    /**
     * @brief Constructor.
     * @param file_name is the input intrinsic file_name.
     * @param setup_type is the enum element of camera setup type.
     * @param model_type is the enum element of camera model type.
     * @param cols is imgae width.
     * @param rows is image height.
     * @return .
     */
    explicit Camera(const std::string &file_name, const setup_type_t setup_type, const model_type_t model_type);

    ~Camera(){};

    model_type_t modelType(void) const
    {
        return model_type_;
    }
    const std::string &cameraName(void) const
    {
        return cam_name_;
    }
    int imageWidth(void) const
    {
        return img_width_;
    }
    int imageHeight(void) const
    {
        return img_height_;
    }

    Eigen::Matrix3d &intrinsic(void)
    {
        return intrinsic_;
    }

    Eigen::VectorXd distortion(void) const
    {
        return distortion_;
    }

    int nIntrinsics(void) const
    {
        return intrinsic_num_;
    }

    virtual bool readIntrinsicFile(const std::string &file_name);
    virtual bool writeIntrinsicFile(const std::string &file_name);

    double reprojectionError(const std::vector<Eigen::Vector3d> &v_obj_points,
                             const std::vector<cv::Point2d> &v_img_kpts,
                             const Eigen::Matrix3d &R,
                             const Eigen::Vector3d &t);

    /**
   * @brief Use image bound to to filte keypoints.
   * @param inputs is original keypoints vector.
   * @param img_bound is the image bound in pixel unit.
   * @return .
   */
    std::vector<cv::Point2d> filteKeypoints(const std::vector<cv::Point2d> &input, unsigned int img_bound);

    std::string cam_name_;
    // camera type
    setup_type_t setup_type_;
    // camera model type
    model_type_t model_type_;
    // stereo camera type
    stereo_type_t stereo_type_;
    // [fx,fy,cx,cy]
    Eigen::Matrix3d intrinsic_;
    // [k1,k2,k3,k4] or [k1,k2,p1,p2,k3]
    Eigen::VectorXd distortion_;
    // intrinsic parameters number
    int intrinsic_num_;

    //
    int img_width_;
    int img_height_;
};

} // namespace camera
#endif
