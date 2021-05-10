#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <chrono>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "stereo_frame.h"
#include "mono_calibration.h"
#ifdef VISUALIZE_TRAJECTORY
#include "viewer.h"
#endif

/**
  * @brief read cctag result file of stereo camera image.
  * @param left_fn is the file name of ccatg detection results of up camera.
  * @param total_frames is stereo frames vector.
  * @return .
  */
bool parseResultFile(const std::string &left_fn,
                     std::vector<StereoFrame> &total_frames)
{
    if (left_fn.empty())
    {
        std::cout << "[parseResultFile] Empty file name!\n";
        return false;
    }

    const auto tp_1 = std::chrono::steady_clock::now();

    YAML::Node left_frm_node;
    try
    {
        left_frm_node = YAML::LoadFile(left_fn);
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
    if (left_frm_node.IsNull())
    {
        std::cout << "[parseResultFile] Could not open file: " << left_fn << "\n";
        return false;
    }

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

        std::cout << "[parseResultFile] Frame " << id << " low camera get " << frame.v_left_keypoints.size()
                  << " cctag keypoints.\n";
        total_frames.emplace_back(frame);
    }
    std::cout << "[parseResultFile] read " << total_frames.size() << " frames! \n";

    const auto tp_2 = std::chrono::steady_clock::now();
    const auto yaml_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    std::cout << "[parseResultFile] Reading yaml file uses " << yaml_time << " s\n";

    return true;
}

/**
  * @brief save monocular camera pose in w2c form.
  * @param folder_name is name of file folder.
  * @param cam_id is the id of each camera.
  * @param cam_pose is the camera pose in w2c.
  * @return .
  */
bool writeCameraPoses(const std::string &folder_name, const int cam_id, const Eigen::Matrix4d &cam_pose)
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

    std::string camera_pose_file_name = folder_name + "/w2c_pose_" + std::to_string(cam_id) + ".txt";

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
    if (argc < 4)
    {
        std::cerr << "Usage : test_monocalibrator /path/to/global_map.yaml\n \
                         /path/to/left_cctag_result.yaml\n \
                         /path/to/left_camera_calibration_file\n \
                         /path/to/output_folder \n";
        return -1;
    }

    std::string gt_map_file_name(argv[1]);
    std::string cctag_file_name(argv[2]);
    std::string cam_calibration_file_name(argv[3]);
    std::string output_folder(argv[4]);

    std::vector<StereoFrame> v_calib_frames;

    bool sts = parseResultFile(cctag_file_name, v_calib_frames);
    if (!sts)
    {
        ERROR_STREAM("[test_monocalibrator] Failed to parse " << cctag_file_name);
        return -1;
    }

    MonoCalibrator mono_calibrator(camera::model_type_t::Fisheye, target::target_type_t::CCTAG, cam_calibration_file_name, gt_map_file_name);

    for (size_t i = 0; i < v_calib_frames.size(); ++i)
    {
        sts = mono_calibrator.addCctagData(v_calib_frames[i].v_img_file_path[0], v_calib_frames[i].v_left_keypoints);
        if (!sts)
        {
            ERROR_STREAM("[test_monocalibrator] Fail to add cctag data from " << v_calib_frames[i].v_img_file_path[0]);
            return -1;
        }
    }

    const auto tp_1 = std::chrono::steady_clock::now();
    sts = mono_calibrator.calibrate();
    if (!sts)
    {
        ERROR_STREAM("[test_monocalibrator] Fail to calibrate mono camera");
        return -1;
    }
    const auto tp_2 = std::chrono::steady_clock::now();
    const auto calib_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    TIMER_STREAM("[test_monocalibrator] caliration time is " << calib_time << " s");

    std::vector<Eigen::Matrix4d> v_cam_poses = mono_calibrator.validCameraPoses();

    std::vector<StereoFrame> v_visual_frames;
    for (size_t i = 0; i < v_cam_poses.size(); ++i)
    {
        sts = writeCameraPoses(output_folder, i, v_cam_poses[i]);
        if (!sts)
        {
            ERROR_STREAM("[test_monocalibrator] Fail to write frame " << i);
            return -1;
        }

        StereoFrame sf(i);
        sf.v_T_wc[0] = v_cam_poses[i];
        sf.v_T_wc[1] = v_cam_poses[i];
        v_visual_frames.emplace_back(sf);
    }

#ifdef VISUALIZE_TRAJECTORY
    Viewer viewer(v_visual_frames);
    std::vector<Eigen::Vector3d> v_map_points = mono_calibrator.objectPoints();
    viewer.setCurrentMapPoints(v_map_points);
    viewer.run();
#endif

    return 0;
}
