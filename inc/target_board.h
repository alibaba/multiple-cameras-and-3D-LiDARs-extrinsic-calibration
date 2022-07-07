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


typedef struct AprilTag
{
   int tag_id;
   std::vector<Eigen::Vector3d> corners;
}AprilTag;

class TargetBoard
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<TargetBoard>;

    TargetBoard(const std::string &file_name, target_type_t modelType);

    ~TargetBoard(){};

    std::vector<Eigen::Vector3d> objectPoints() const;
    std::vector<AprilTag> objectAprilTags() const;
    target_type_t objectTargetType() const;
private:
    bool loadCCTAGObjPoints(const std::string &file_name);
    bool loadAprilTagObjPoints(const std::string &file_name);
    bool loadChessboardObjPoints(const std::string &file_name);
    
    target_type_t target_type_;
    std::vector<Eigen::Vector3d> v_object_points_;
    std::vector<AprilTag> v_obj_points_apriltag_;
};

} // namespace target

#endif /* __TARGET_BOARD_H*/