#include <iostream>
#include <sstream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

#include "localization/IOUtils.h"
#include "datastruct/Lidar3DStruct.h"
#include "param_config/ParamConfig.h"
#include "sensor_geometry/MultiRayLidar.h"
#include "sensor_geometry/PanoAxis.h"
#include "sensor_geometry/PinholeCamera.h"
#include "sensor_geometry/SensorFactory.h"
#include "util/FileSystemTools.h"
#include "util/LabelmeUtils.h"
#include "Utils.h"


bool projectLaserScan2Camera(const common::PinholeCameraPtr &cam_ptr,
                             const sensorhandler::PointCloudPtr &laser_scan_ptr,
                             const Eigen::Matrix4d &T_c_l, cv::Mat &out_img) {
  if (cam_ptr == nullptr || laser_scan_ptr == nullptr) {
    LOG(ERROR) << "Invalid input pointer!";
    return false;
  }
  if (T_c_l.hasNaN()) {
    LOG(ERROR) << "Invalid transformation matrix!";
    return false;
  }

  int img_width = cam_ptr->resolution().width;
  int img_height = cam_ptr->resolution().height;
  int point_num = laser_scan_ptr->points.size();
  // LOG(INFO) << "Process " << point_num << " laser points";

  cv::Scalar color_start(0, 255, 0);
  cv::Scalar color_end(0,0,255);
  const double max_distant = 10.0;
  double weight_b = (color_end[0] - color_start[0]) / max_distant;
  double weight_g = (color_end[1] - color_start[1]) / max_distant;
  double weight_r = (color_end[2] - color_start[2]) / max_distant;

  int cnt = 0;
  for (int i = 0; i < point_num; ++i) {
    Eigen::Vector3d p_l(laser_scan_ptr->points[i].x,
                        laser_scan_ptr->points[i].y,
                        laser_scan_ptr->points[i].z);

    Eigen::Vector3d p_c = (T_c_l * p_l.homogeneous()).hnormalized();
    if (p_c[2] < 0)
        continue;
    
    Eigen::Vector2d p_img;
    cam_ptr->project(p_c, &p_img);
    if (p_img[0] < 0 || p_img[0] >= img_width || p_img[1] < 0 || p_img[1] >= img_height)
      continue;

    // draw color square as projected laser point
    {
      double color_scale = p_c[2];
      cv::Scalar color(0, 255 + weight_g*color_scale , weight_r * color_scale);
      cv::Point p(p_img[0], p_img[1]);
      cv::circle(out_img, p, 1, color, -1);
      cnt ++;
    }
  }

  LOG(INFO) << "Project " << cnt << " laser points into camera.";
  return true;
}

// this exe visualize lidar2cam extrinsic by backprojecting laser point cloud into image plane


int main(int argc, char **argv) {
  if (argc < 3) {
    LOG(ERROR)
        << "Usage: ./test_lidar2tehce_evaluation [config.yaml] [data_folder] \n \
    \t [config.yaml] is backpack configuration file. \n \
    \t [data_folder] is dataset fodler for evaluation.  \n";
    return -1;
  }

  std::string config_file(argv[1]);
  std::string data_folder(argv[2]);
  std::string lidar_scan_folder = common::concatenateFolderAndFileName(data_folder, "scans");
  std::string teche_image_folder = common::concatenateFolderAndFileName(data_folder, "undist_pano_images");

  if (!common::fileExists(config_file)) {
    LOG(ERROR) << " Configuration file does not exit, quit...\n";
    return -1;
  }
  if (!common::pathExists(lidar_scan_folder)) {
    LOG(ERROR) << " Lidar folder does not exit, quit...\n";
    return -1;
  }
  if (!common::pathExists(teche_image_folder)) {
    LOG(ERROR) << " Teche image folder does not exit, quit...\n";
    return -1;
  }

  common::ParamConfig::setParameterFile(config_file);

  // teche cameras
  std::vector<common::PinholeCameraPtr> v_teche_cameras_ptr;
  for (size_t i = 0; i < 4; ++i) {
    common::PinholeCameraPtr cam_ptr = std::make_shared<common::PinholeCamera>("teche_" + std::to_string(i));
    if (cam_ptr != nullptr) {
      v_teche_cameras_ptr.push_back(cam_ptr);
    } else {
      LOG(ERROR) << "FAIL to construct teche camera from configuration file!";
      return -1;
    }
  }

  common::MultiRayLidarPtr vertical_lidar_ptr = common::SensorFactory::createMultiRayLidar("vertical_lidar");
  common::MultiRayLidarPtr horizontal_lidar_ptr = common::SensorFactory::createMultiRayLidar("horizon_lidar");

  // pano
  common::PanoAxisPtr pano_axis_ptr = common::SensorFactory::createPanoAxis("teche_pano_axis");

  Eigen::Matrix4d T_base_l0 = horizontal_lidar_ptr->extrinsics();
  Eigen::Matrix4d T_base_l1 = vertical_lidar_ptr->extrinsics();
  Eigen::Matrix4d T_base_pano_axis = pano_axis_ptr->extrinsics();
  std::string horizon_pc_filepath = common::concatenateFolderAndFileName(lidar_scan_folder, "0_0.ply");
  std::string vertical_pc_filepath = common::concatenateFolderAndFileName(lidar_scan_folder, "1_0.ply");

  PointCloudPtr vertical_sweep_cloud_ptr(new PointCloud);
  PointCloudPtr horizon_sweep_cloud_ptr(new PointCloud);

  if (!common::fileExists(vertical_pc_filepath)) {
    LOG(ERROR) << "cannot find pc file: " << vertical_pc_filepath;
    return -1;
  }
  if (!common::fileExists(horizon_pc_filepath)) {
    LOG(ERROR) << "cannot find pc file: " << vertical_pc_filepath;
    return -1;
  }

  pcl::io::loadPLYFile(vertical_pc_filepath, *vertical_sweep_cloud_ptr);
  pcl::io::loadPLYFile(horizon_pc_filepath, *horizon_sweep_cloud_ptr);

  double perpendicular_angle = 0.0;
  Eigen::Vector3d lidar0_corner_point = calcCornerPoint(lidar_scan_folder, "lidar0", perpendicular_angle);
  Eigen::Vector3d lidar1_corner_point = calcCornerPoint(lidar_scan_folder, "lidar1", perpendicular_angle);
  if(lidar0_corner_point.norm() > 1e3 && lidar1_corner_point.norm() > 1e3){
    LOG(ERROR) << "Invalid corner points! Please check input plane cloud";
    return -1;
  }

  std::vector<std::string> v_img_paths;
  std::vector<std::string> paths;
  paths.push_back(teche_image_folder);
  common::getFileLists(paths, true, "jpg", &v_img_paths);

  int img_num = v_img_paths.size();
  for (int i = 0; i < img_num; ++i) {
    std::string &img_path = v_img_paths[i];
    std::string img_folder, img_name;
    common::splitPathAndFilename(img_path, &img_folder, &img_name);

    int cam_index = img_name[0] - 48;
    if (cam_index < 1 || cam_index > 4) {
      LOG(ERROR) << "Invalid teche images!";
      continue;
    }

    // read corner coordinates on image
    std::string corner_file_name = img_name.substr(0, img_name.size()-4) + ".json";
    std::string corner_file_path = common::concatenateFolderAndFileName(teche_image_folder, corner_file_name);
    // skip images that corner is unsivisable
    if (!common::fileExists(corner_file_path)) {
      LOG(WARNING) << "cannot find labelme file: " << corner_file_path;
      continue;
    }
    std::vector<Eigen::Vector2d> v_corners;
    common::readLabelMeResult(corner_file_path, v_corners);

    cv::Mat out_img = cv::imread(img_path, cv::IMREAD_UNCHANGED);
    if(out_img.empty()){
        LOG(ERROR) << "Fail to read image  " << img_path;
        return -1;
    }

    common::PinholeCameraPtr teche_cam_ptr = v_teche_cameras_ptr[cam_index - 1];
    Eigen::Matrix4d T_pano_axis_c = teche_cam_ptr->extrinsics();
    Eigen::Matrix3d intrinsic = teche_cam_ptr->cameraMatrix();

    // project horizontal sweeps into camera
    // Eigen::Matrix4d T_l0_c = (T_base_l0.inverse()) * T_base_pano_axis * T_pano_axis_c;
    Eigen::Matrix4d T_l0_c = (T_base_l0.inverse()) * T_pano_axis_c;
    std::cout << "T_l0_c" << cam_index-1 << " : " << T_l0_c << "\n";
    Eigen::Matrix4d T_c_l0 = T_l0_c.inverse();
    bool sts = projectLaserScan2Camera(teche_cam_ptr, horizon_sweep_cloud_ptr,
                                       T_c_l0, out_img);
    if (!sts) {
      LOG(ERROR) << "Fail to project laser point into image!";
      return -1;
    }

    double reproject_err = calcCornerPointReprojectError(lidar0_corner_point, T_c_l0, intrinsic, v_corners[0], out_img);
    LOG(INFO) << "Reproject error of Corner extract from lidar0 sweeps : " << reproject_err;
    std::string save_img_path = common::concatenateFolderAndFileName(img_folder, "eva_lidar0_" + img_name);
    cv::imwrite(save_img_path, out_img);

    if(lidar1_corner_point.norm() > 1e6){
      LOG(WARNING) << " Cannot find corner point in Lidara1";
      continue;
    }
    out_img = cv::imread(img_path, cv::IMREAD_UNCHANGED);
    // project vertical sweeps into camera
    // Eigen::Matrix4d T_l1_c = (T_base_l1.inverse()) * T_base_pano_axis * T_pano_axis_c;
    Eigen::Matrix4d T_l1_c = (T_base_l1.inverse()) * T_pano_axis_c;
    // T_l1_c << -0.305399684008826,  -0.951924037185201,  0.0239094214973372,   0.640044537317772,
    // -0.0215547405557131, -0.0181916807166895,  -0.999602148813356,  -0.075209496121445,
    //   0.951980265639412,  -0.305793541759355, -0.0149627421078072,  -0.039993200341565,
    //   0, 0, 0, 1;
    // T_l1_c <<  0.298707809405327,  -0.953858732908545, -0.0304493391451621,   0.662864589002141,
    // 0.0389234600194619, -0.0197027160624481,   0.999047930401678,  0.0639108552539182,
    // -0.953550527691314,  -0.299608612416047,  0.0312421271002314, -0.0762731696475622,
    // 0, 0, 0, 1;
    std::cout << "T_l1_c" << cam_index-1 << " : " << T_l1_c << "\n";
    Eigen::Matrix4d T_c_l1 = T_l1_c.inverse();
    sts = projectLaserScan2Camera(teche_cam_ptr, vertical_sweep_cloud_ptr,
                                  T_c_l1, out_img);
    if (!sts) {
      LOG(ERROR) << "Fail to project laser point into image!";
      return -1;
    }

    reproject_err = calcCornerPointReprojectError(lidar1_corner_point, T_c_l1, intrinsic, v_corners[0], out_img);
    LOG(INFO) << "Reproject error of Corner extract from lidar1 sweeps : " << reproject_err;
    save_img_path = common::concatenateFolderAndFileName(img_folder, "eva_lidar1_" + img_name);
    cv::imwrite(save_img_path, out_img);

    Eigen::Matrix4d T_l0_l1 = Eigen::Matrix4d::Identity();
    T_l0_l1 = T_l0_c * T_l1_c.inverse();
    std::cout << T_l0_l1(0,0) << ", " << T_l0_l1(0,1) << ", " << T_l0_l1(0,2) << ", " << T_l0_l1(0,3) << "\n";
    std::cout << T_l0_l1(1,0) << ", " << T_l0_l1(1,1) << ", " << T_l0_l1(1,2) << ", " << T_l0_l1(1,3) << "\n";
    std::cout << T_l0_l1(2,0) << ", " << T_l0_l1(2,1) << ", " << T_l0_l1(2,2) << ", " << T_l0_l1(2,3) << "\n";
    std::cout << T_l0_l1(3,0) << ", " << T_l0_l1(3,1) << ", " << T_l0_l1(3,2) << ", " << T_l0_l1(3,3) << "\n";
  }

  return 0;
}
