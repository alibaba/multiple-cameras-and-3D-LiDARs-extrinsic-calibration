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
#include <glog/logging.h>

#include "camera.h"
#include "stereo_calibration.h"
#include "FileSystemTools.h"
#include "YamlFileIO.h"

#ifdef VISUALIZE_TRAJECTORY
#include "viewer.h"
#endif



int main(int argc, char **argv)
{

    if (argc < 8)
    {
        std::cerr << "Usage : test_stereocalibrator [global_map.yaml] [cam0_cctag_result.yaml] [cam1_cctag_result.yaml]\n \
                         [cam0_intrin_file] [cam1_intrin_file] [flag_calibrate_hik_teche] [output_folder]\n";
        return -1;
    }

    std::string global_map_fn(argv[1]);
    std::string left_cam_cctag_fn(argv[2]);
    std::string right_cam_cctag_fn(argv[3]);
    // calibrate extrinsic between hik and TECHE camera or hik and hik camera
    bool b_homo_hik_camera = (std::stoi(argv[6]) == 1) ? false : true;
    std::string output_folder(argv[7]);
    
    if (!common::pathExists(output_folder))
    {
        if (!common::createPath(output_folder))
        {
            LOG(FATAL) << " Failed to create " << output_folder;
            return -1;
        }
    }
    // start timer
    const auto tp_1 = std::chrono::steady_clock::now();

    // stereo frames data of calibration process
    std::vector<std::string> v_cam0_img_paths, v_cam1_img_paths;
    std::vector<std::vector<cv::Point2d>> v_cam0_cctag_centers, v_cam1_cctag_centers;
    bool sts = common::loadCCTagResultFile(left_cam_cctag_fn, v_cam0_img_paths, v_cam0_cctag_centers);
    if (!sts)
    {
        LOG(ERROR) << " Failed to parse " << left_cam_cctag_fn;
        return -1;
    }
    sts = common::loadCCTagResultFile(right_cam_cctag_fn, v_cam1_img_paths, v_cam1_cctag_centers);
    if (!sts)
    {
        LOG(ERROR) << " Failed to parse " << right_cam_cctag_fn;
        return -1;
    }
    assert(v_cam0_img_paths.size() == v_cam1_img_paths.size());
    assert(v_cam0_cctag_centers.size() == v_cam1_cctag_centers.size());

    //! load 2 camera seperately
    std::string cam0_intrinsic_fn(argv[4]);
    std::string cam1_intrinsic_fn(argv[5]);

    StereoCalibrator stereo_calibrator(camera::model_type_t::Fisheye, target::target_type_t::CCTAG,
                                       cam0_intrinsic_fn, cam1_intrinsic_fn,
                                       global_map_fn,
                                       b_homo_hik_camera);

    for (size_t frm_id = 0; frm_id < v_cam0_img_paths.size(); frm_id++)
    {

        sts = stereo_calibrator.addCctagData(v_cam0_img_paths[frm_id], v_cam0_cctag_centers[frm_id],
                                            v_cam1_img_paths[frm_id], v_cam1_cctag_centers[frm_id]);
        if (!sts)
        {
            LOG(ERROR) << " Fail to add calibration data in frame " << frm_id;
            continue;
        }
    }

    sts = stereo_calibrator.calibrate();
    if (!sts)
    {
        LOG(ERROR) << " Fail to calibrate multicamera system!";
        return -1;
    }
    const auto tp_4 = std::chrono::steady_clock::now();
    const auto calib_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_4 - tp_1).count();
    LOG(INFO) << " Stereo caliration time is " << calib_time << " s";

    // save calibration result
    std::string cam0_img_name, cam1_img_name, path;
    common::splitPathAndFilename(v_cam0_img_paths[0], &path, &cam0_img_name);
    common::splitPathAndFilename(v_cam1_img_paths[0], &path, &cam1_img_name);
    int cam0_idx = std::atoi(&cam0_img_name[0]);
    int cam1_idx = std::atoi(&cam1_img_name[0]);
    Eigen::Matrix4d T_cam0_cam1 = stereo_calibrator.validExtrinsics();
    std::string yml_filepath = output_folder + "/camera"+ std::to_string(cam0_idx) + "_to_camera"+ std::to_string(cam1_idx) +".yml";
    common::saveExtFileOpencv(yml_filepath, T_cam0_cam1);

    std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> v_cam_poses_w2c = stereo_calibrator.validCameraPosePairs();

#ifdef VISUALIZE_TRAJECTORY
    std::vector<Eigen::Matrix4d> v_visual_frames = {v_cam_poses_w2c[0].first, v_cam_poses_w2c[0].second};
    Viewer viewer(v_visual_frames);
    std::vector<Eigen::Vector3d> v_map_points = stereo_calibrator.objectPoints();
    viewer.setCurrentMapPoints(v_map_points);
    viewer.run();
#endif

    return 0;
}
