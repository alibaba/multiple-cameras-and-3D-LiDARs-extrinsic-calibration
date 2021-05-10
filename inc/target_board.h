#ifndef __TARGET_BOARD_H_
#define __TARGET_BOARD_H_

#include <memory>

#include "common.h"

namespace target
{
enum class target_type_t
{
    AprilTag = 0,
    Chessboard = 1,
    CCTAG = 2
};

class TargetBoard
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<TargetBoard>;

    TargetBoard(const std::string &file_name, target_type_t modelType);

    ~TargetBoard(){};

    std::vector<Eigen::Vector3d> objectPoints() const;

private:
    bool loadCCTAGObjPoints(const std::string &file_name);
    bool loadAprilTagObjPoints(const std::string &file_name);
    bool loadChessboardObjPoints(const std::string &file_name);

    std::vector<Eigen::Vector3d> v_object_points_;
};

} // namespace target

#endif /* __TARGET_BOARD_H*/