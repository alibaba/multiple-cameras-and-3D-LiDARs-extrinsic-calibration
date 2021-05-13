#ifndef COMMON_UTIL_YAML_FILE_IO_TOOLS_H_
#define COMMON_UTIL_YAML_FILE_IO_TOOLS_H_

#include <string>
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>


namespace common {

    bool loadExtFileOpencv(const std::string &file, Eigen::Matrix4d &T);    

    void saveExtFileOpencv(const std::string &file, const Eigen::Matrix4d &T);

    bool loadIntrinFileKalibr(const std::string &file, cv::Mat &K, cv::Mat &D, 
                                std::string &distort_type, int &img_width, int &img_height);
    
    bool loadIntrinFileOpencv(const std::string &file_name, cv::Mat &K, cv::Mat &D, 
                        std::string &distort_type, int &img_width, int &img_height);

    bool saveIntrinFileOpencv(const std::string &file_name, const cv::Mat &K, const cv::Mat &D, 
                            const std::string &distort_type, const int img_width, const int img_height);
    
    /**
     * @brief read cctag result file of stereo camera image.
     * @param left_fn is the file name of ccatg detection results of up camera.
     * @param total_frames is stereo frames vector.
     * @return .
     */
    bool loadCCTagResultFile(const std::string &filepath,
                        std::vector<std::string> &v_img_path,
                        std::vector<std::vector<cv::Point2d>> &vv_kpts);

}

#endif 