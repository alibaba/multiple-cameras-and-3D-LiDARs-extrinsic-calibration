#ifndef COMMON_UTIL_YAML_FILE_IO_TOOLS_H_
#define COMMON_UTIL_YAML_FILE_IO_TOOLS_H_

#include <string>
#include <vector>
#include <Eigen/Core>

namespace common {

    bool loadExtFileOpencv(const std::string &file, Eigen::Matrix4d &T);    

    void saveExtFileOpencv(const std::string &file, const Eigen::Matrix4d &T);

    bool loadIntrinFileKalibr(const std::string &file, cv::Mat &K, cv::Mat &D, 
                                std::string &distort_type, int &img_width, int &img_height);
    
    bool loadIntrinFileOpencv(const std::string &file_name, cv::Mat &K, cv::Mat &D, 
                        std::string &distort_type, int &img_width, int &img_height);

    bool saveIntrinFileOpencv(const std::string &file_name, const cv::Mat &K, const cv::Mat &D, 
                            const std::string &distort_type, const int img_width, const int img_height);


}

#endif 