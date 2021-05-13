#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <chrono>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>
#include <Open3D/Open3D.h>
#include <glog/logging.h>

#include "teche_calibration.h"
#include "FileSystemTools.h"
#include "YamlFileIO.h"
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

std::shared_ptr<open3d::geometry::PointCloud> convertPointClouds(const std::vector<Eigen::Vector3d> &v_pointclouds)
{
    assert(v_pointclouds.size());

    int pts_num = v_pointclouds.size();
    std::shared_ptr<open3d::geometry::PointCloud> pcl_ptr(new open3d::geometry::PointCloud);
    for(int i = 0; i < pts_num; ++i)
    {
        pcl_ptr->points_.push_back(v_pointclouds[i]);
    }

    return pcl_ptr;
}

int main(int argc, char **argv)
{
    if (argc < 8)
    {
        std::cerr << "Usage : test_techecalibrator [global_map.yaml] [cctag_result.yaml]\n \
        [cam0_intrin_file] [cam1_intrin_file] [cam2_intrin_file] [cam3_intrin_file]\n \
        [output_folder] \n";
        return -1;
    }
    // start timer
    // const auto tp_1 = std::chrono::steady_clock::now();

    // save w2c camera pose
    bool b_save_camera_pose_w_c = true;
    // save c2r camera pose
    bool b_save_camera_pose_c_r = true;

    std::string gt_map_file_name(argv[1]);
    std::string cctag_file_name(argv[2]);
    std::string cam0_calibration_file_name(argv[3]);
    std::string cam1_calibration_file_name(argv[4]);
    std::string cam2_calibration_file_name(argv[5]);
    std::string cam3_calibration_file_name(argv[6]);
    std::string output_folder(argv[7]);

    if (!common::fileExists(gt_map_file_name))
    {
        LOG(ERROR) << " FIle " << gt_map_file_name << " doesnot exist!";
        return -1;
    }
    if (!common::pathExists(output_folder))
    {
        if (!common::createPath(output_folder))
        {
            LOG(ERROR) << " Fail to create " << output_folder;
            return -1;
        }
    }

    std::vector<std::string> v_intrinsic_fns = {cam0_calibration_file_name, cam1_calibration_file_name,
                                                cam2_calibration_file_name, cam3_calibration_file_name};
    std::vector<std::string> v_calib_img_paths;
    std::vector<std::vector<cv::Point2d>> v_cctag_centers;
    bool sts = common::loadCCTagResultFile(cctag_file_name, v_calib_img_paths, v_cctag_centers);
    if (!sts)
    {
        LOG(ERROR) << " Failed to parse " << cctag_file_name;
        return -1;
    }

    TecheCalibrator teche_calibrator(camera::model_type_t::Fisheye, target::target_type_t::CCTAG, v_intrinsic_fns, gt_map_file_name, 4);
    sts = teche_calibrator.addCctagData(v_calib_img_paths, v_cctag_centers);
    if (!sts)
    {
        LOG(ERROR) << " Feail to add cctag data!";
        return -1;
    }

    sts = teche_calibrator.calibrate();
    if (!sts)
    {
        LOG(ERROR) << " Fail to calibrate teche 360 anywhere camera";
        return -1;
    }
    // const auto tp_2 = std::chrono::steady_clock::now();
    // const auto calib_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    // TIMER_STREAM("[test_techecalibrator] caliration time is " << calib_time << " s");

    std::vector<Eigen::Matrix4d> v_cam_poses_w2c = teche_calibrator.validCameraPoses();
    // rotation axis 2 camera
    std::vector<Eigen::Matrix4d> v_cam_poses_r2c = teche_calibrator.validCameraPosesInRotationAxisFrame();

    // save camera pose and relative pose
    for (size_t i = 0; i < v_cam_poses_w2c.size(); ++i)
    {
        if (b_save_camera_pose_w_c)
        {
            std::string txt_filepath = output_folder + "/cam" + std::to_string(i) + "_w2c.txt";
            saveCamPoseTxt(txt_filepath, v_cam_poses_w2c[i]);
        }
        if (i == 0)
            continue;

        Eigen::Matrix4d T_cam0_cami = v_cam_poses_w2c[0].inverse() * v_cam_poses_w2c[i];
        std::string yml_filepath = output_folder + "/camera0"+"_to_camera"+ std::to_string(i) +".yml";
        common::saveExtFileOpencv(yml_filepath, T_cam0_cami);
    }

    // for (size_t i = 0; i < v_cam_poses_r2c.size(); ++i)
    // {
    //     if (b_save_camera_pose_c_r)
    //     {
    //         sts = writeCameraPosesInYml(output_folder, i, v_cam_poses_r2c[i]);
    //         if (!sts)
    //         {
    //             ERROR_STREAM("[test_techecalibrator] Fail to write frame " << i);
    //             return -1;
    //         }
    //     }
    // }

#ifdef VISUALIZE_TRAJECTORY
    Viewer viewer(v_visual_frames);
    std::vector<Eigen::Vector3d> v_map_points = teche_calibrator.objectPoints();
    // std::shared_ptr<open3d::geometry::PointCloud> targtboard_pcl_ptr = convertPointClouds(v_map_points);
    // open3d::io::WritePointCloudToPLY(output_folder + "/calibration_room.ply", *targtboard_pcl_ptr, true);
    viewer.setCurrentMapPoints(v_map_points);
    viewer.run();
#endif

    return 0;
}
