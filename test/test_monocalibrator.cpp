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
#include <glog/logging.h>

#include "FileSystemTools.h"
#include "YamlFileIO.h"
#include "mono_calibration.h"
#ifdef VISUALIZE_TRAJECTORY
#include "viewer.h"
#endif



/**
  * @brief save monocular camera pose in w2c form.
  * @param folder_name is name of file folder.
  * @param cam_id is the id of each camera.
  * @param cam_pose is the camera pose in w2c.
  * @return .
  */
bool saveCamPoseTxt(const std::string &filepath, const Eigen::Matrix4d &cam_pose)
{
    std::ofstream output_file(filepath);
    if (!output_file.is_open())
    {
        LOG(FATAL) << " Fail to open " << filepath << "\n";
        return false;
    }
    LOG(INFO) << " Write " << filepath << "\n";
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
        std::cerr << "Usage : test_monocalibrator [global_map.yaml] [cctag_result.yaml] [cam_intrin_file] [output_folder] \n";
        return -1;
    }

    std::string gt_map_file_name(argv[1]);
    std::string cctag_file_name(argv[2]);
    std::string cam_calibration_file_name(argv[3]);
    std::string output_folder(argv[4]);
    // start timer
    const auto tp_1 = std::chrono::steady_clock::now();

    std::vector<std::string> v_calib_img_paths;
    std::vector<std::vector<cv::Point2d>> v_cctag_centers;
    bool sts = common::loadCCTagResultFile(cctag_file_name, v_calib_img_paths, v_cctag_centers);
    if (!sts)
    {
        LOG(ERROR) << " Failed to parse " << cctag_file_name;
        return -1;
    }

    MonoCalibrator mono_calibrator(camera::model_type_t::Fisheye, target::target_type_t::CCTAG, cam_calibration_file_name, gt_map_file_name);

    for (size_t i = 0; i < v_calib_img_paths.size(); ++i)
    {
        sts = mono_calibrator.addCctagData(v_calib_img_paths[i], v_cctag_centers[i]);
        if (!sts)
        {
            LOG(ERROR) << " Fail to add cctag data from " << v_calib_img_paths[i];
            return -1;
        }
    }

    sts = mono_calibrator.calibrate();
    if (!sts)
    {
        LOG(ERROR) << " Fail to calibrate mono camera";
        return -1;
    }

    std::vector<Eigen::Matrix4d> v_cam_poses = mono_calibrator.validCameraPoses();
    for (size_t i = 0; i < v_cam_poses.size(); ++i)
    {
        std::string txt_filepath = output_folder + "/cam" + std::to_string(i) + "_w2c.txt";
        sts = saveCamPoseTxt(txt_filepath, v_cam_poses[i]);
        if (!sts)
        {
            LOG(ERROR) << " Fail to write frame " << i << " into " << txt_filepath;
            return -1;
        }
    }

    const auto tp_2 = std::chrono::steady_clock::now();
    const auto calib_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    LOG(INFO) << " calibration time is " << calib_time << " s";
#ifdef VISUALIZE_TRAJECTORY
    Viewer viewer(v_visual_frames);
    std::vector<Eigen::Vector3d> v_map_points = mono_calibrator.objectPoints();
    viewer.setCurrentMapPoints(v_map_points);
    viewer.run();
#endif

    return 0;
}
