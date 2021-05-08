#ifndef COMMON_UTIL_YAML_FILE_IO_TOOLS_H_
#define COMMON_UTIL_YAML_FILE_IO_TOOLS_H_

#include <string>
#include <vector>
#include <Eigen/Core>

namespace common {

    Eigen::Matrix4d loadExtFileOpencv(const std::string &file);    

    void saveExtFileOpencv(const std::string &file, const Eigen::Matrix4d &T);

}

#endif 