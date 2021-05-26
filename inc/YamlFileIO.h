#ifndef COMMON_UTIL_YAML_FILE_IO_TOOLS_H_
#define COMMON_UTIL_YAML_FILE_IO_TOOLS_H_

#include <string>
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>


namespace common {
    /**
     * @brief Load extrisnic file(Opencv format).
     * @param file is the file name of extrinsics.
     * @param T  is the extrinsic parameters.
     * @return .
     */
    bool loadExtFileOpencv(const std::string &file, Eigen::Matrix4d &T);    

    /**
     * @brief Save extrisnic file(Opencv format).
     * @param file is the file name of extrinsics.
     * @param T  is the extrinsic parameters.
     * @return .
     */
    void saveExtFileOpencv(const std::string &file, const Eigen::Matrix4d &T);

    /**
     * @brief Load intrinsic file(Kalibr format).
     * @param file is the file name of intrinsics.
     * @param K is intrinsic matrix.
     * @param D is distortion vector.
     * @param distort_type is distortion type.
     * @param img_width is width.
     * @param img_height is height.
     * @return .
     */
    bool loadIntrinFileKalibr(const std::string &file, cv::Mat &K, cv::Mat &D, 
                                std::string &distort_type, int &img_width, int &img_height);

    /**
     * @brief Load intrinsic file(Opencv format).
     * @param file is the file name of intrinsics.
     * @param K is intrinsic matrix.
     * @param D is distortion vector.
     * @param distort_type is distortion type.
     * @param img_width is width.
     * @param img_height is height.
     * @return .
     */
    bool loadIntrinFileOpencv(const std::string &file_name, cv::Mat &K, cv::Mat &D, 
                        std::string &distort_type, int &img_width, int &img_height);

    /**
     * @brief Save intrinsic file(Opencv format).
     * @param file is the file name of intrinsics.
     * @param K is intrinsic matrix.
     * @param D is distortion vector.
     * @param distort_type is distortion type.
     * @param img_width is width.
     * @param img_height is height.
     * @return .
     */
    bool saveIntrinFileOpencv(const std::string &file_name, const cv::Mat &K, const cv::Mat &D, 
                            const std::string &distort_type, const int img_width, const int img_height);
    
    /**
     * @brief read cctag result file of a single camera.
     * @param filepath is the file name of ccatg detection results of the camera.
     * @param v_img_path is the image pathes.
     * @param vv_kpts is 2D vector of detected keypoints.
     * @return .
     */
    bool loadCCTagResultFile(const std::string &filepath,
                        std::vector<std::string> &v_img_path,
                        std::vector<std::vector<cv::Point2d>> &vv_kpts);

}

#endif 