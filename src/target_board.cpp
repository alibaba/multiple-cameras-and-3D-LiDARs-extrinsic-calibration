#include <chrono>

#include <yaml-cpp/yaml.h>

#include "target_board.h"

namespace target
{

TargetBoard::TargetBoard(const std::string &file_name, target_type_t modelType)
{
    if (target_type_t::CCTAG == modelType)
    {
        if (!loadCCTAGObjPoints(file_name))
        {
            ERROR_STREAM("[TargetBoard] Fail to load CCTAG object points!");
        }
    }
    else
    {
        if (target_type_t::AprilTag == modelType)
        {
            if (!loadAprilTagObjPoints(file_name))
            {
                ERROR_STREAM("[TargetBoard] Fail to load Apriltag object points!");
            }
        }
        else
        {
            if (target_type_t::Chessboard == modelType)
            {
                if (!loadChessboardObjPoints(file_name))
                {
                    ERROR_STREAM("[TargetBoard] Fail to load chessboard object points!");
                }
            }
        }
    }
}

bool TargetBoard::loadAprilTagObjPoints(const std::string &file_name)
{
    if (file_name.empty())
    {
        ERROR_STREAM("[TargetBoard::loadAprTagObjPoints] Empty file name!");
        return false;
    }

    //TODO

    return true;
}

bool TargetBoard::loadChessboardObjPoints(const std::string &file_name)
{
    if (file_name.empty())
    {
        ERROR_STREAM("[TargetBoard::loadAprTagObjPoints] Empty file name!");
        return false;
    }

    // TODO

    return true;
}

bool TargetBoard::loadCCTAGObjPoints(const std::string &fileName)
{
    if (fileName.empty())
    {
        ERROR_STREAM("[TargetBoard::loadAprTagObjPoints] Empty file name!");
        return false;
    }

    const auto tp_1 = std::chrono::steady_clock::now();

    YAML::Node root_node;
    try
    {
        root_node = YAML::LoadFile(fileName);
    }
    catch (YAML::BadFile &e)
    {
        ERROR_STREAM("[loadCCTAGObjPoints] Could not open file: " << fileName);
        return false;
    }
    catch (YAML::ParserException &e)
    {
        ERROR_STREAM("[loadCCTAGObjPoints] Invalid file format: " << fileName);
        return false;
    }
    if (root_node.IsNull())
    {
        ERROR_STREAM("[loadCCTAGObjPoints] Could not open file: " << fileName);
        return false;
    }

    DEBUG_STREAM("[loadCCTAGObjPoints] reading MapPoints and FramePoses...\n ");

    v_object_points_.clear();

    YAML::Node mapPointNode = root_node["MapPoints"];

    // parse MapPoints
    for (YAML::const_iterator it = mapPointNode.begin(); it != mapPointNode.end(); ++it)
    {
        int map_point_id = it->first.as<int>();
        std::string id = std::to_string(map_point_id);

        Eigen::Vector3d map_point;
        map_point[0] = mapPointNode[id]["position"][0].as<double>();
        map_point[1] = mapPointNode[id]["position"][1].as<double>();
        map_point[2] = mapPointNode[id]["position"][2].as<double>();

        v_object_points_.emplace_back(map_point);
    }
    DEBUG_STREAM("[loadCCTAGObjPoints] read " << v_object_points_.size() << " MapPoints! \n");

    return true;
}

std::vector<Eigen::Vector3d> TargetBoard::objectPoints() const
{
    return v_object_points_;
}

} // namespace target
