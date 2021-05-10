#include <iostream>
#include <chrono>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

#include "stereo_frame.h"
#include "track.h"
#include "camera.h"
#include "stereo_calibration.h"

#ifdef VISUALIZE_TRAJECTORY
#include "viewer.h"
#endif

/**
  * @brief parse result yaml file of 3d_marker_slam.
  * @param fileName is name of result file.
  * @param globalMapPoints is global map points vector.
  * @param totalFrames is keyframes vector.
  * @return .
  */
bool parseResultFile(const std::string &fileName,
                     std::vector<Eigen::Vector3d> &globalMapPoints)
{
    if (fileName.empty())
    {
        std::cout << "[parseResultFile] Empty file name!\n";
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
        std::cout << "[parseResultFile] Could not open file: " << fileName << "\n";
        return false;
    }
    catch (YAML::ParserException &e)
    {
        std::cout << "[parseResultFile] Invalid file format: " << fileName << "\n";
        return false;
    }
    if (root_node.IsNull())
    {
        std::cout << "[parseResultFile] Could not open file: " << fileName << "\n";
        return false;
    }

    std::cout << "[parseResultFile] reading MapPoints and FramePoses...\n ";

    globalMapPoints.clear();
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

        globalMapPoints.emplace_back(map_point);
    }
    std::cout << "[parseResultFile] read " << mapPointNode.size() << " MapPoints! \n";

    const auto tp_2 = std::chrono::steady_clock::now();
    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    std::cout << "[parseResultFile] Reading yaml file uses " << track_time << " s\n";

    return true;
}

/**
  * @brief read cctag result file of stereo camera image.
  * @param fileName is name of result file.
  * @param left_fn is the file name of ccatg detection results of up camera.
  * @param right_fn is the file name of ccatg detection results of low camera.
  * @param total_frames is stereo frames vector.
  * @return .
  */
bool parseResultFile(const std::string &left_fn,
                     const std::string &right_fn,
                     std::vector<StereoFrame> &total_frames)
{
    if (left_fn.empty() || right_fn.empty())
    {
        std::cout << "[parseResultFile] Empty file name!\n";
        return false;
    }

    const auto tp_1 = std::chrono::steady_clock::now();

    YAML::Node left_frm_node;
    YAML::Node right_frm_node;
    try
    {
        left_frm_node = YAML::LoadFile(left_fn);
        right_frm_node = YAML::LoadFile(right_fn);
    }
    catch (YAML::BadFile &e)
    {
        std::cout << "[parseResultFile] Could not open file: " << left_fn << "\n";
        return false;
    }
    catch (YAML::ParserException &e)
    {
        std::cout << "[parseResultFile] Invalid file format: " << left_fn << "\n";
        return false;
    }
    if (left_frm_node.IsNull() || right_frm_node.IsNull())
    {
        std::cout << "[parseResultFile] Could not open file: " << left_fn << "\n";
        return false;
    }

    assert(left_frm_node.size() == right_frm_node.size());
    total_frames.clear();

    // parse FramePoses
    for (YAML::const_iterator it = left_frm_node.begin(); it != left_frm_node.end(); ++it)
    {
        int f = it->first.as<int>();
        std::string id = std::to_string(f);

        StereoFrame frame(f);

        // left camera
        frame.v_img_file_path[0] = left_frm_node[id]["image_file"].as<std::string>();
        for (size_t k = 0; k < left_frm_node[id]["keypoints"].size(); ++k)
        {
            cv::Point2d keypoint;
            keypoint.x = left_frm_node[id]["keypoints"][k][0].as<double>();
            keypoint.y = left_frm_node[id]["keypoints"][k][1].as<double>();
            frame.v_left_keypoints.emplace_back(keypoint);
        }

        // right camera
        frame.v_img_file_path[1] = right_frm_node[id]["image_file"].as<std::string>();
        for (size_t j = 0; j < right_frm_node[id]["keypoints"].size(); ++j)
        {
            cv::Point2d keypoint;
            keypoint.x = right_frm_node[id]["keypoints"][j][0].as<double>();
            keypoint.y = right_frm_node[id]["keypoints"][j][1].as<double>();
            frame.v_right_keypoints.emplace_back(keypoint);
        }
        std::cout << "[parseResultFile] Frame " << id << " low camera get " << frame.v_left_keypoints.size()
                  << " cctag keypoints.\n";
        total_frames.emplace_back(frame);
    }
    std::cout << "[parseResultFile] read " << left_frm_node.size() << " frames! \n";

    const auto tp_2 = std::chrono::steady_clock::now();
    const auto yaml_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    std::cout << "[parseResultFile] Reading yaml file uses " << yaml_time << " s\n";

    return true;
}

/**
  * @brief save final camera pose in w2c form.
  * @param folder_name is name of file folder.
  * @param cam_id is the id of each camera.
  * @param angle_id is the id of each angle position camera rotated.
  * @param cam_pose is the camera pose in w2c.
  * @return .
  */
bool writeCameraPoses(const std::string &folder_name, const int cam_id, const int angle_id, const Eigen::Matrix4d &cam_pose)
{
    if (folder_name.empty())
    {
        ERROR_STREAM("[writeCameraPoses] Empty output folder_name!");
        return false;
    }

    // create folder
    if (NULL == opendir(folder_name.c_str()))
    {
        if (mkdir(folder_name.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO) != 0)
        {
            ERROR_STREAM("[writeCameraPoses] Fail to create folder " << folder_name);
            return false;
        }
    }

    std::string camera_pose_fn_base = folder_name + "/w2c_pose_" + std::to_string(cam_id);

    std::string camera_pose_file_name = camera_pose_fn_base + "_" +
                                        std::to_string(angle_id) + ".txt";
    std::ofstream output_file(camera_pose_file_name);
    if (!output_file.is_open())
    {
        std::cout << "[writeCameraPoses] Fail to open " << camera_pose_file_name << "\n";
    }
    std::cout << "[writeCameraPoses] Write " << camera_pose_file_name << "\n";
    for (size_t i = 0; i < 4; ++i)
    {
        for (size_t j = 0; j < 4; ++j)

        {
            if (j != 3)
                output_file << cam_pose(i, j) << ", ";
            else
                output_file << cam_pose(i, j) << "\n";
        }
    }

    output_file.close();

    return true;
}

int main(int argc, char **argv)
{

    if (argc < 8)
    {
        std::cerr << "Usage : test_stereocalibrator /path/to/global_map.yaml\n \
                         /path/to/left_cctag_result.yaml\n \
                         /path/to/right_cctag_result.yaml\n \
                         /path/to/left_camera_calibration_file \n \
                         /path/to/right_camera_calibration_file \n \
                         flag_calibrate_hik_teche \n \
                         /path/to/output_folder";
        return -1;
    }

    std::string global_map_fn(argv[1]);
    std::string left_cam_cctag_fn(argv[2]);
    std::string right_cam_cctag_fn(argv[3]);
    // calibrate extrinsic between hik and TECHE camera or hik and hik camera
    bool b_homo_hik_camera = (std::stoi(argv[6]) == 1) ? false : true;
    std::string output_folder(argv[7]);

    // stereo frames data of calibration process
    std::vector<StereoFrame> v_calib_frames;

    bool stat = false;
    //! read cctag result of images taken in calibration process
    stat = parseResultFile(left_cam_cctag_fn, right_cam_cctag_fn, v_calib_frames);
    if (!stat)
    {
        std::cout << "[test_stereocalibrator] Fail to parse test_result.yaml, evaluate failed!\n";
        return -1;
    }

    //! load 2 camera seperately
    std::string left_cam_intrinsic_fn(argv[4]);
    std::string right_cam_intrinsic_fn(argv[5]);

    const auto tp_1 = std::chrono::steady_clock::now();
    StereoCalibrator stereo_calibrator(camera::model_type_t::Fisheye, target::target_type_t::CCTAG,
                                       left_cam_intrinsic_fn, right_cam_intrinsic_fn,
                                       global_map_fn,
                                       b_homo_hik_camera);
    const auto tp_2 = std::chrono::steady_clock::now();
    const auto initialize_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    TIMER_STREAM("[test_stereocalibrator] Initialize Calibrator time is " << initialize_time << " s");

    for (size_t frm_id = 0; frm_id < v_calib_frames.size(); frm_id++)
    {

        stat = stereo_calibrator.addCctagData(v_calib_frames[frm_id].v_img_file_path[0], v_calib_frames[frm_id].v_left_keypoints,
                                              v_calib_frames[frm_id].v_img_file_path[1], v_calib_frames[frm_id].v_right_keypoints);
        if (!stat)
        {
            ERROR_STREAM("[test_stereocalibrator] Fail to add calibration data in frame " << frm_id);
            continue;
        }
    }

    const auto tp_3 = std::chrono::steady_clock::now();
    stat = stereo_calibrator.calibrate();
    if (!stat)
    {
        ERROR_STREAM("[test_stereocalibrator] Fail to calibrate multicamera system!");
        return -1;
    }
    const auto tp_4 = std::chrono::steady_clock::now();
    const auto calib_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_4 - tp_3).count();
    TIMER_STREAM("[test_stereocalibrator] Stereo caliration time is " << calib_time << " s");

    // save calibration result
    stat = stereo_calibrator.writeExtrinsicParams(output_folder);
    if (!stat)
    {
        ERROR_STREAM("[test_stereocalibrator] Failed to save stereo extrinsic parameteres " << output_folder);
        return -1;
    }

    //  visualize calibration result
    std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> v_cam_pose_pairs = stereo_calibrator.validCameraPosePairs();
    std::vector<StereoFrame> v_visual_frames;
    for (size_t i = 0; i < v_cam_pose_pairs.size(); ++i)
    {
        StereoFrame sf(i);
        sf.v_T_wc[0] = v_cam_pose_pairs[i].first;
        sf.v_T_wc[1] = v_cam_pose_pairs[i].second;
        v_visual_frames.emplace_back(sf);
    }

#ifdef VISUALIZE_TRAJECTORY
    Viewer viewer(v_visual_frames);
    std::vector<Eigen::Vector3d> v_map_points = stereo_calibrator.objectPoints();
    viewer.setCurrentMapPoints(v_map_points);
    viewer.run();
#endif

    return 0;
}
