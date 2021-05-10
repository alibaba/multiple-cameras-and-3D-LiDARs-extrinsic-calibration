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

#include "stereo_frame.h"
#include "teche_calibration.h"
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
bool writeCameraPosesInTxT(const std::string &folder_name, const int cam_id, const Eigen::Matrix4d &cam_pose)
{
    if (folder_name.empty())
    {
        ERROR_STREAM("[writeCameraPosesInTxT] Empty output folder_name!");
        return false;
    }

    // create folder
    if (NULL == opendir(folder_name.c_str()))
    {
        if (mkdir(folder_name.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO) != 0)
        {
            ERROR_STREAM("[writeCameraPosesInTxT] Fail to create folder " << folder_name);
            return false;
        }
    }

    std::string camera_pose_file_name = folder_name + "/w2c_pose_" + std::to_string(cam_id) + ".txt";

    std::ofstream output_file(camera_pose_file_name);
    if (!output_file.is_open())
    {
        std::cout << "[writeCameraPosesInTxT] Fail to open " << camera_pose_file_name << "\n";
    }
    std::cout << "[writeCameraPosesInTxT] Write " << camera_pose_file_name << "\n";
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

/**
  * @brief save monocular camera pose in r2c form( frame transformation: rotation axis 2 camera).
  * @param folder_name is name of file folder.
  * @param cam_id is the id of each camera.
  * @param cam_pose is the camera pose in w2c.
  * @return .
  */
bool writeCameraPosesInYml(const std::string &folder_name, const int cam_id, const Eigen::Matrix4d &cam_pose)
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

    std::string camera_pose_file_name = folder_name + "/r2c_pose_teche" + std::to_string(cam_id) + ".yml";

    cv::FileStorage fs(camera_pose_file_name, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        std::cout << "[writeCameraPosesInYml] Fail to open " << camera_pose_file_name << "\n";
    }
    std::cout << "[writeCameraPosesInYml] Write " << camera_pose_file_name << "\n";

    Eigen::Matrix3d R_matrix = cam_pose.block<3, 3>(0, 0);
    Eigen::Vector3d t_matrix = cam_pose.block<3, 1>(0, 3);

    cv::Mat R_mat, t_mat;
    cv::eigen2cv(R_matrix, R_mat);
    cv::eigen2cv(t_matrix, t_mat);
    fs << "extrinsic_rotation" << R_mat;
    fs << "extrinsic_translation" << t_mat;
    fs.release();

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
        std::cerr << "Usage : test_techecalibrator /path/to/global_map.yaml\n \
                         /path/to/cctag_result.yaml\n \
                         /path/to/cam0_calibration_file\n \
                         /path/to/cam1_calibration_file\n \
                         /path/to/cam2_calibration_file\n \
                         /path/to/cam3_calibration_file\n \
                         /path/to/output_folder \n";
        return -1;
    }

    // save w2c camera pose
    bool b_save_camera_pose_w_c = false;
    // save c2r camera pose
    bool b_save_camera_pose_c_r = true;

    std::string gt_map_file_name(argv[1]);
    std::string cctag_file_name(argv[2]);
    std::string cam0_calibration_file_name(argv[3]);
    std::string cam1_calibration_file_name(argv[4]);
    std::string cam2_calibration_file_name(argv[5]);
    std::string cam3_calibration_file_name(argv[6]);
    std::string output_folder(argv[7]);

    std::vector<StereoFrame> v_calib_frames;
    std::vector<std::string> v_intrinsic_fns = {cam0_calibration_file_name, cam1_calibration_file_name,
                                                cam2_calibration_file_name, cam3_calibration_file_name};
    std::vector<std::string> v_calib_img_paths;
    std::vector<std::vector<cv::Point2d>> v_cctag_centers;

    bool sts = parseResultFile(cctag_file_name, v_calib_frames);
    if (!sts)
    {
        ERROR_STREAM("[test_techecalibrator] Failed to parse " << cctag_file_name);
        return -1;
    }
    for (size_t i = 0; i < v_calib_frames.size(); ++i)
    {
        v_calib_img_paths.emplace_back(v_calib_frames[i].v_img_file_path[0]);
        v_cctag_centers.emplace_back(v_calib_frames[i].v_left_keypoints);
    }

    TecheCalibrator teche_calibrator(camera::model_type_t::Fisheye, target::target_type_t::CCTAG, v_intrinsic_fns, gt_map_file_name, 4);

    sts = teche_calibrator.addCctagData(v_calib_img_paths, v_cctag_centers);
    if (!sts)
    {
        ERROR_STREAM("[test_techecalibrator] Fail to add cctag data!");
        return -1;
    }

    const auto tp_1 = std::chrono::steady_clock::now();
    sts = teche_calibrator.calibrate();
    if (!sts)
    {
        ERROR_STREAM("[test_techecalibrator] Fail to calibrate teche 360 anywhere camera");
        return -1;
    }
    const auto tp_2 = std::chrono::steady_clock::now();
    const auto calib_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    TIMER_STREAM("[test_techecalibrator] caliration time is " << calib_time << " s");

    std::vector<Eigen::Matrix4d> v_cam_poses_w2c = teche_calibrator.validCameraPoses();
    // rotation axis 2 camera
    std::vector<Eigen::Matrix4d> v_cam_poses_r2c = teche_calibrator.validCameraPosesInRotationAxisFrame();

    std::vector<StereoFrame> v_visual_frames;
    for (size_t i = 1; i < v_cam_poses_w2c.size(); ++i)
    {
        // if (b_save_camera_pose_w_c)
        // {
        //     sts = writeCameraPosesInTxT(output_folder, i, v_cam_poses_w2c[i]);
        //     if (!sts)
        //     {
        //         ERROR_STREAM("[test_techecalibrator] Fail to write frame " << i);
        //         return -1;
        //     }
        // }

        StereoFrame sf(i);
        sf.v_T_wc[0] = v_cam_poses_w2c[i];
        sf.v_T_wc[1] = v_cam_poses_w2c[i];
        v_visual_frames.emplace_back(sf);
        Eigen::Matrix4d T_cam0_cam1 = v_cam_poses_w2c[0].inverse() * v_cam_poses_w2c[i];
        std::string cam_pose_filename = output_folder + "/teche0"+"_teche"+ std::to_string(i) +".yaml";
        cv::FileStorage fs(cam_pose_filename, cv::FileStorage::WRITE);
        if (!fs.isOpened())
        {
            std::cout << "[writeCameraPosesInYml] Fail to open " << cam_pose_filename << "\n";
        }
        std::cout << "[writeCameraPosesInYml] Write " << cam_pose_filename << "\n";

        Eigen::Matrix3d R_matrix = T_cam0_cam1.block<3, 3>(0, 0);
        Eigen::Vector3d t_matrix = T_cam0_cam1.block<3, 1>(0, 3);

        cv::Mat R_mat, t_mat;
        cv::eigen2cv(R_matrix, R_mat);
        cv::eigen2cv(t_matrix, t_mat);
        fs << "extrinsic_rotation" << R_mat;
        fs << "extrinsic_translation" << t_mat;
        fs.release();
    }

    for (size_t i = 0; i < v_cam_poses_r2c.size(); ++i)
    {
        if (b_save_camera_pose_c_r)
        {
            sts = writeCameraPosesInYml(output_folder, i, v_cam_poses_r2c[i]);
            if (!sts)
            {
                ERROR_STREAM("[test_techecalibrator] Fail to write frame " << i);
                return -1;
            }
        }

        StereoFrame sf(i + v_cam_poses_w2c.size());
        sf.v_T_wc[0] = v_cam_poses_r2c[i];
        sf.v_T_wc[1] = v_cam_poses_r2c[i];
        v_visual_frames.emplace_back(sf);
    }

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
