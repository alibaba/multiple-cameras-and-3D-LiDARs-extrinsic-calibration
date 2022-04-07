#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>

#include "YamlFileIO.h"
#include "FileSystemTools.h"

#define BACKPACK_1 1
#define BACKPACK_2 2
#define BACKPACK_3 3
#define BACKPACK_4 4
#define BACKPACK_5 5

namespace YAML {
template <>
struct convert<Eigen::Matrix3d> {
  static Node encode(const Eigen::Matrix3d& rhs) {
    Node node;

    for (int i = 0; i < rhs.rows(); ++i) {
      for (int j = 0; j < rhs.cols(); ++j) {
        node.push_back(rhs(i, j));
      }
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix3d& rhs) {
    if (!node.IsSequence() || node.size() != 9) {
      return false;
    }

    rhs(0, 0) = node[0].as<double>();
    rhs(0, 1) = node[1].as<double>();
    rhs(0, 2) = node[2].as<double>();
    rhs(1, 0) = node[3].as<double>();
    rhs(1, 1) = node[4].as<double>();
    rhs(1, 2) = node[5].as<double>();
    rhs(2, 0) = node[6].as<double>();
    rhs(2, 1) = node[7].as<double>();
    rhs(2, 2) = node[8].as<double>();
    return true;
  }
};

template <>
struct convert<Eigen::Vector3d> {
  static Node encode(const Eigen::Vector3d& rhs) {
    Node node;

    for (int i = 0; i < rhs.rows(); ++i) {
      for (int j = 0; j < rhs.cols(); ++j) {
        node.push_back(rhs(i, j));
      }
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::Vector3d& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs(0, 0) = node[0].as<double>();
    rhs(1, 0) = node[1].as<double>();
    rhs(2, 0) = node[2].as<double>();
    return true;
  }
};

template <>
struct convert<Eigen::Vector4d> {
  static Node encode(const Eigen::Vector4d& rhs) {
    Node node;

    for (int i = 0; i < rhs.rows(); ++i) {
      for (int j = 0; j < rhs.cols(); ++j) {
        node.push_back(rhs(i, j));
      }
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::Vector4d& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs(0, 0) = node[0].as<double>();
    rhs(1, 0) = node[1].as<double>();
    rhs(2, 0) = node[2].as<double>();
    rhs(3, 0) = node[3].as<double>();
    return true;
  }
};

template <>
struct convert<Eigen::Matrix4d> {
  static Node encode(const Eigen::Matrix4d& rhs) {
    Node node;

    for (int i = 0; i < rhs.rows(); ++i) {
      for (int j = 0; j < rhs.cols(); ++j) {
        node.push_back(rhs(i, j));
      }
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix4d& rhs) {
    if (!node.IsSequence() || node.size() != 16) {
      return false;
    }

    rhs(0, 0) = node[0].as<double>();
    rhs(0, 1) = node[1].as<double>();
    rhs(0, 2) = node[2].as<double>();
    rhs(0, 3) = node[3].as<double>();
    rhs(1, 0) = node[4].as<double>();
    rhs(1, 1) = node[5].as<double>();
    rhs(1, 2) = node[6].as<double>();
    rhs(1, 3) = node[7].as<double>();
    rhs(2, 0) = node[8].as<double>();
    rhs(2, 1) = node[9].as<double>();
    rhs(2, 2) = node[10].as<double>();
    rhs(2, 3) = node[11].as<double>();
    rhs(3, 0) = node[12].as<double>();
    rhs(3, 1) = node[13].as<double>();
    rhs(3, 2) = node[14].as<double>();
    rhs(3, 3) = node[15].as<double>();
    return true;
  }
};
}  // namespace YAML

/**
 * @brief Stereo camera parameters
 *
 */
struct StereoParam {
  // extrinsic matrix, transform right camera frame into left camera frame
  Eigen::Matrix4d T_rl;
  // left camera intrinsic
  Eigen::Matrix3d cam0_intrinsic;
  // left camera distortion_coeffs
  double cam0_distortions[4];

  // right camera intrinsic
  Eigen::Matrix3d cam1_intrinsic;
  // right camera distortion_coeffs
  double cam1_distortions[4];

  int img_width;
  int img_height;
  // camera model
  std::string camera_type;
  // assume cam0 and cam1 use identical distortion type
  std::string distortion_type;

  StereoParam()
      : T_rl(Eigen::Matrix4d::Identity()),
        cam0_intrinsic(Eigen::Matrix3d::Identity()),
        cam1_intrinsic(Eigen::Matrix3d::Identity()),
        img_width(2592),
        img_height(2048) {
    memset(cam0_distortions, 0, 4 * sizeof(double));
    memset(cam1_distortions, 0, 4 * sizeof(double));
    camera_type = "pinhole";
    distortion_type = "equidistant";
  }

  void getExtrinsic(cv::Mat& R_rl, cv::Mat& t_rl) {
    Eigen::Matrix3d R = T_rl.block<3, 3>(0, 0);
    Eigen::Vector3d t = T_rl.block<3, 1>(0, 3);

    cv::eigen2cv(R, R_rl);
    cv::eigen2cv(t, t_rl);
  }

  void getCam0Param(cv::Mat& K, cv::Mat& D) {
    cv::eigen2cv(cam0_intrinsic, K);

    cv::Mat distortion =
        (cv::Mat_<double>(1, 4) << cam0_distortions[0], cam0_distortions[1],
         cam0_distortions[2], cam0_distortions[3]);
    D = distortion.clone();
  }

  void getCam1Param(cv::Mat& K, cv::Mat& D) {
    cv::eigen2cv(cam1_intrinsic, K);

    cv::Mat distortion =
        (cv::Mat_<double>(1, 4) << cam1_distortions[0], cam1_distortions[1],
         cam1_distortions[2], cam1_distortions[3]);
    D = distortion.clone();
  }
};

/**
 * @brief Stereo camera parameters
 *
 */
struct MonoCamParam {
  // extrinsic: from reference camera to this camera
  Eigen::Matrix4d T_ref_cam_;
  // camera intrinsic
  Eigen::Matrix3d intrinsic_;
  // camera distortion_coeffs
  Eigen::Vector4d distortions_;

  int img_width_;
  int img_height_;
  // camera model
  std::string camera_type;
  // lens distortion type
  std::string distortion_type;

  MonoCamParam()
      : T_ref_cam_(Eigen::Matrix4d::Identity()),
        intrinsic_(Eigen::Matrix3d::Identity()),
        distortions_(Eigen::Vector4d::Zero()),
        img_width_(2592),
        img_height_(2048) {    
    camera_type = "pinhole";
    distortion_type = "equidistant";
  }

  void getExtrinsic(cv::Mat& R_ref_cam, cv::Mat& t_ref_cam) {
    Eigen::Matrix3d R = T_ref_cam_.block<3, 3>(0, 0);
    Eigen::Vector3d t = T_ref_cam_.block<3, 1>(0, 3);

    cv::eigen2cv(R, R_ref_cam);
    cv::eigen2cv(t, t_ref_cam);
  }

  void getCamParam(cv::Mat& K, cv::Mat& D) {
    cv::eigen2cv(intrinsic_, K);

    cv::Mat distortion;
    cv::eigen2cv(distortions_, distortion);
    D = distortion.clone();
  }

};


/**
 * @brief Mono-IMU parameters
 *
 */
struct MonoIMUParam {
  // cam0 2 imu
  Eigen::Matrix4d T_cam0_imu;

  // acc bias(m/s^2)
  Eigen::Vector3d acc_bias;
  // gyro (rad/s)
  Eigen::Vector3d gyro_bias;

  // noise rms
  double avg_acc_noise_density;
  double avg_gyro_noise_density;
  // random walk noise rms
  double avg_acc_bias_noise_density;
  double avg_gyro_bias_noise_density;

  MonoIMUParam()
      : T_cam0_imu(Eigen::Matrix4d::Identity()),
        acc_bias(Eigen::Vector3d::Zero()),
        gyro_bias(Eigen::Vector3d::Zero()),
        avg_acc_noise_density(0.0),
        avg_gyro_noise_density(0.0),
        avg_acc_bias_noise_density(0.0),
        avg_gyro_bias_noise_density(0.0) {}

  void getExtrinsic(cv::Mat& R_cam0_imu, cv::Mat& t_cam0_imu) {
    Eigen::Matrix3d R = T_cam0_imu.block<3, 3>(0, 0);
    Eigen::Vector3d t = T_cam0_imu.block<3, 1>(0, 3);

    cv::eigen2cv(R, R_cam0_imu);
    cv::eigen2cv(t, t_cam0_imu);
  }

  void getAccBias(cv::Mat& acc_bias_mat) {
    cv::eigen2cv(acc_bias, acc_bias_mat);
  }

  void getGyroBias(cv::Mat& gyro_bias_mat) {
    cv::eigen2cv(gyro_bias, gyro_bias_mat);
  }

  void getNoiseDensity(double& acc_nd, double& gyro_nd) {
    acc_nd = avg_acc_noise_density;
    gyro_nd = avg_gyro_noise_density;
  }

  void getBiasNoiseDensity(double& acc_bias_nd, double& gyro_bias_nd) {
    acc_bias_nd = avg_acc_bias_noise_density;
    gyro_bias_nd = avg_gyro_bias_noise_density;
  }
};

/**
 * @brief Stereo-IMU parameters
 *
 */
struct LidarParam {
  //
  double max_dist;
  //
  double min_dist;
  //
  double starting_offset;
  //
  int num_rays;
  double frequency;
  // transform w.r.t lidar0
  Eigen::Matrix4d T;

  LidarParam()
      : max_dist(30.0),
        min_dist(0.1),
        starting_offset(-6.0),
        num_rays(16),
        frequency(10),
        T(Eigen::Matrix4d::Identity()) {}
};

/**
 * @brief Teche parameters
 * Contain 4 camera intrinsics and extrinsics
 */
struct TecheParam {
  // intrinsic parameters
  Eigen::Matrix3d cam0_intrinsic;
  Eigen::Matrix3d cam1_intrinsic;
  Eigen::Matrix3d cam2_intrinsic;
  Eigen::Matrix3d cam3_intrinsic;
  // distortion parameters
  Eigen::Vector4d cam0_distortion;
  Eigen::Vector4d cam1_distortion;
  Eigen::Vector4d cam2_distortion;
  Eigen::Vector4d cam3_distortion;
  // image size
  int img_width;
  int img_height;
  // camera model
  std::string camera_type;
  // distortion type
  std::string distortion_type;
  // extrinsic: rotation axis 2 camera
  Eigen::Matrix4d T_r_c0;
  Eigen::Matrix4d T_r_c1;
  Eigen::Matrix4d T_r_c2;
  Eigen::Matrix4d T_r_c3;

  // extrinsic: lidar0 2 rotation axis
  Eigen::Matrix4d T_lidar0_r;
};

/**
 * @brief parse kalibr yaml file support: pinhole + equidistant
 *
 * @param kalib_intrin_file
 * @param camera Camera Model param
 */
bool readKalibrStereoOutputFile(const std::string& kalib_result_file,
                                StereoParam& stereo_param) {
  if (kalib_result_file.empty()) {
    std::cout << "[readKalibrOutputFile] Empty input file name!\n";
    return false;
  }

  std::ifstream fs(kalib_result_file);
  if (!fs.is_open()) {
    std::cout << "[readKalibrOutputFile] Error read " << kalib_result_file
              << "\n";
    return false;
  }

  YAML::Node root_node;
  YAML::Node cam0_node;
  YAML::Node cam1_node;

  try {
    root_node = YAML::LoadFile(kalib_result_file);
  } catch (YAML::BadFile& e) {
    std::cout << "[readKalibrOutputFile] Could not open file: "
              << kalib_result_file << "\n";
    return false;
  } catch (YAML::ParserException& e) {
    std::cout << "[readKalibrOutputFile] Invalid file format: "
              << kalib_result_file << "\n";
    return false;
  }
  if (root_node.IsNull()) {
    std::cout << "[loadKalibrResult] Could not open file: " << kalib_result_file
              << "\n";
    return false;
  }

  std::cout << "[readKalibrOutputFile] read " << kalib_result_file << ":\n";
  int img_width = 0, img_height = 0;

  cam0_node = root_node["cam0"];
  double cam0_distortion[4] = {0.0};
  cam0_distortion[0] = cam0_node["distortion_coeffs"][0].as<double>();
  cam0_distortion[1] = cam0_node["distortion_coeffs"][1].as<double>();
  cam0_distortion[2] = cam0_node["distortion_coeffs"][2].as<double>();
  cam0_distortion[3] = cam0_node["distortion_coeffs"][3].as<double>();

  Eigen::Matrix3d cam0_intrinsic = Eigen::Matrix3d::Identity();
  cam0_intrinsic(0, 0) = cam0_node["intrinsics"][0].as<double>();
  cam0_intrinsic(1, 1) = cam0_node["intrinsics"][1].as<double>();
  cam0_intrinsic(0, 2) = cam0_node["intrinsics"][2].as<double>();
  cam0_intrinsic(1, 2) = cam0_node["intrinsics"][3].as<double>();

  cam1_node = root_node["cam1"];
  double cam1_distortion[4] = {0.0};
  cam1_distortion[0] = cam1_node["distortion_coeffs"][0].as<double>();
  cam1_distortion[1] = cam1_node["distortion_coeffs"][1].as<double>();
  cam1_distortion[2] = cam1_node["distortion_coeffs"][2].as<double>();
  cam1_distortion[3] = cam1_node["distortion_coeffs"][3].as<double>();

  Eigen::Matrix3d cam1_intrinsic = Eigen::Matrix3d::Identity();
  cam1_intrinsic(0, 0) = cam1_node["intrinsics"][0].as<double>();
  cam1_intrinsic(1, 1) = cam1_node["intrinsics"][1].as<double>();
  cam1_intrinsic(0, 2) = cam1_node["intrinsics"][2].as<double>();
  cam1_intrinsic(1, 2) = cam1_node["intrinsics"][3].as<double>();

  img_width = cam1_node["resolution"][0].as<int>();
  img_height = cam1_node["resolution"][1].as<int>();

  std::string distortion_type = cam0_node["distortion_model"].as<std::string>();
  std::string camera_type = cam0_node["camera_model"].as<std::string>();

  Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
  for (size_t i = 0; i < cam1_node["T_cn_cnm1"].size(); ++i) {
    extrinsic(i, 0) = cam1_node["T_cn_cnm1"][i][0].as<double>();
    extrinsic(i, 1) = cam1_node["T_cn_cnm1"][i][1].as<double>();
    extrinsic(i, 2) = cam1_node["T_cn_cnm1"][i][2].as<double>();
    extrinsic(i, 3) = cam1_node["T_cn_cnm1"][i][3].as<double>();
  }

  stereo_param.T_rl = extrinsic;
  std::cout << "[readKalibrOutputFile] T_rl : \n" << stereo_param.T_rl << "\n";
  Eigen::Vector3d baseline = stereo_param.T_rl.block<3, 1>(0, 3);
  std::cout << "[readKalibrOutputFile] baseline norm : " << baseline.norm()
            << "\n";

  std::cout << "[loadKalibrResult] baseline * fx = "
            << (baseline.norm() * cam0_intrinsic(0, 0)) << "\n";

  stereo_param.cam0_intrinsic = cam0_intrinsic;
  std::cout << "[loadKalibrResult] cam0 intrinsic : \n"
            << stereo_param.cam0_intrinsic << "\n";
  memcpy(stereo_param.cam0_distortions, cam0_distortion, 4 * sizeof(double));
  std::cout << "[loadKalibrResult] cam0 distrotion : "
            << stereo_param.cam0_distortions[0] << ", "
            << stereo_param.cam0_distortions[3] << "\n";
  stereo_param.cam1_intrinsic = cam1_intrinsic;
  std::cout << "[loadKalibrResult] cam1 intrinsic : \n"
            << stereo_param.cam1_intrinsic << "\n";
  memcpy(stereo_param.cam1_distortions, cam1_distortion, 4 * sizeof(double));
  std::cout << "[loadKalibrResult] cam1 distrotion : "
            << stereo_param.cam1_distortions[0] << ", "
            << stereo_param.cam1_distortions[3] << "\n";
  stereo_param.img_width = img_width;
  stereo_param.img_height = img_height;
  stereo_param.distortion_type = distortion_type;
  stereo_param.camera_type = camera_type;

  return true;
}

bool loadMonoIMUExtFileKalibr(const std::string& kalib_result_file,
                                   MonoIMUParam& monocam_imu_param) {
  if (!common::fileExists(kalib_result_file)) {
    LOG(FATAL) << " File doesnt exist: " << kalib_result_file;
    return false;
  }

  std::ifstream fs(kalib_result_file);
  if (!fs.is_open()) {
    LOG(INFO) << " Error read " << kalib_result_file << "\n";
    return false;
  }

  YAML::Node root_node;
  try {
    root_node = YAML::LoadFile(kalib_result_file);
  } catch (YAML::BadFile& e) {
    LOG(INFO) << " Could not open file: " << kalib_result_file << "\n";
    return false;
  } catch (YAML::ParserException& e) {
    LOG(INFO) << " Invalid file format: " << kalib_result_file << "\n";
    return false;
  }
  if (root_node.IsNull()) {
    LOG(INFO) << " Could not open file: " << kalib_result_file << "\n";
    return false;
  }


  YAML::Node cam0_node = root_node["cam0"];
  Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
  for (size_t i = 0; i < cam0_node["T_cam_imu"].size(); ++i) {
    extrinsic(i, 0) = cam0_node["T_cam_imu"][i][0].as<double>();
    extrinsic(i, 1) = cam0_node["T_cam_imu"][i][1].as<double>();
    extrinsic(i, 2) = cam0_node["T_cam_imu"][i][2].as<double>();
    extrinsic(i, 3) = cam0_node["T_cam_imu"][i][3].as<double>();
  }

  monocam_imu_param.T_cam0_imu = extrinsic;
  std::cout << "[loadMonoIMUExtFileKalibr] camera-IMU : \n"
            << extrinsic << "\n";
  return true;
}

bool readOpencvOutputFile(const std::string& file_name, Eigen::Matrix4d& T) {
  if (file_name.empty()) {
    std::cerr << "Empty file name!\n";
    return false;
  }

  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Fail to open " << file_name << "\n";
    return false;
  }

  cv::Mat rotation, trans;
  fs["extrinsic_rotation"] >> rotation;
  fs["extrinsic_translation"] >> trans;
  fs.release();

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  cv::cv2eigen(rotation, R);
  cv::cv2eigen(trans, t);

  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;

  return true;
}

bool readOpencvOutputFile(const std::string& cam0_intrinsic_fn,
                          const std::string& cam1_intrinsic_fn,
                          const std::string& cam2_intrinsic_fn,
                          const std::string& cam3_intrinsic_fn,
                          TecheParam& teche_param) {
  if (cam0_intrinsic_fn.empty() || cam1_intrinsic_fn.empty() ||
      cam2_intrinsic_fn.empty() || cam3_intrinsic_fn.empty()) {
    std::cerr << "[readTecheOutputFile] Empty file name!\n";
    return false;
  }

  teche_param.camera_type = "pinhole";
  {
    cv::FileStorage fs(cam0_intrinsic_fn, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "[readTecheOutputFile] Fail to open " << cam0_intrinsic_fn
                << "\n";
      return false;
    }

    int img_width, img_height;
    cv::Mat intrinsic, distortion;
    Eigen::Matrix3d intrinsic_matrix;
    Eigen::Vector4d distortion_vector;
    std::string distortion_type;

    img_width = (int)fs["image_width"];
    img_height = (int)fs["image_height"];

    distortion_type = (std::string)fs["camera_model"];

    fs["camera_matrix"] >> intrinsic;
    fs["distortion_coefficients"] >> distortion;
    cv::cv2eigen(intrinsic, intrinsic_matrix);
    distortion_vector[0] = distortion.at<double>(0, 0);
    distortion_vector[1] = distortion.at<double>(0, 1);
    distortion_vector[2] = distortion.at<double>(0, 2);
    distortion_vector[3] = distortion.at<double>(0, 3);
    fs.release();

    teche_param.img_width = img_width;
    teche_param.img_height = img_height;

    teche_param.cam0_intrinsic = intrinsic_matrix;
    teche_param.cam0_distortion = distortion_vector;
    teche_param.distortion_type = distortion_type;

    std::cout << "Load teche0 intrinsic parameters\n";
  }

  {
    cv::FileStorage fs(cam1_intrinsic_fn, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "[readTecheOutputFile] Fail to open " << cam1_intrinsic_fn
                << "\n";
      return false;
    }

    cv::Mat intrinsic, distortion;
    Eigen::Matrix3d intrinsic_matrix;
    Eigen::Vector4d distortion_vector;

    fs["camera_matrix"] >> intrinsic;
    fs["distortion_coefficients"] >> distortion;
    cv::cv2eigen(intrinsic, intrinsic_matrix);
    distortion_vector[0] = distortion.at<double>(0, 0);
    distortion_vector[1] = distortion.at<double>(0, 1);
    distortion_vector[2] = distortion.at<double>(0, 2);
    distortion_vector[3] = distortion.at<double>(0, 3);
    fs.release();

    teche_param.cam1_intrinsic = intrinsic_matrix;
    teche_param.cam1_distortion = distortion_vector;
  }

  {
    cv::FileStorage fs(cam2_intrinsic_fn, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "[readTecheOutputFile] Fail to open " << cam2_intrinsic_fn
                << "\n";
      return false;
    }

    cv::Mat intrinsic, distortion;
    Eigen::Matrix3d intrinsic_matrix;
    Eigen::Vector4d distortion_vector;

    fs["camera_matrix"] >> intrinsic;
    fs["distortion_coefficients"] >> distortion;
    cv::cv2eigen(intrinsic, intrinsic_matrix);
    distortion_vector[0] = distortion.at<double>(0, 0);
    distortion_vector[1] = distortion.at<double>(0, 1);
    distortion_vector[2] = distortion.at<double>(0, 2);
    distortion_vector[3] = distortion.at<double>(0, 3);
    fs.release();

    teche_param.cam2_intrinsic = intrinsic_matrix;
    teche_param.cam2_distortion = distortion_vector;
  }

  {
    cv::FileStorage fs(cam3_intrinsic_fn, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "[readTecheOutputFile] Fail to open " << cam3_intrinsic_fn
                << "\n";
      return false;
    }

    cv::Mat intrinsic, distortion;
    Eigen::Matrix3d intrinsic_matrix;
    Eigen::Vector4d distortion_vector;

    fs["camera_matrix"] >> intrinsic;
    fs["distortion_coefficients"] >> distortion;
    cv::cv2eigen(intrinsic, intrinsic_matrix);
    distortion_vector[0] = distortion.at<double>(0, 0);
    distortion_vector[1] = distortion.at<double>(0, 1);
    distortion_vector[2] = distortion.at<double>(0, 2);
    distortion_vector[3] = distortion.at<double>(0, 3);
    fs.release();

    teche_param.cam3_intrinsic = intrinsic_matrix;
    teche_param.cam3_distortion = distortion_vector;
  }

  return true;
}

bool writeYAMLOutputFile(const std::string& file_name, const MonoCamParam& cam0, const MonoCamParam& cam1, 
                         const MonoCamParam& cam2, const MonoCamParam& cam3, const MonoCamParam& cam4,
                         const MonoIMUParam& mono_imu,
                         const Eigen::Matrix4d& T_lidar0_lidar1,
                         const Eigen::Matrix4d& T_lidar1_cam0, int device_id) {
    if (file_name.empty()) {
        std::cerr << "Empty file name!\n";
        return false;
    }

    std::ofstream yaml_out_file(file_name);
    if (!yaml_out_file.is_open()) {
        LOG(FATAL) << " Fail to open yaml file! \n";
        return false;
    }

    YAML::Node root_node;
    YAML::Node lidar0_node;
    YAML::Node lidar1_node;
    YAML::Node camera0_node;
    YAML::Node camera1_node;
    YAML::Node camera2_node;
    YAML::Node camera3_node;
    YAML::Node camera4_node;
    YAML::Node imu_node;

    // serial nunmber
    root_node["serial_num"] = device_id;

    // sensor bucket
    std::vector<std::vector<std::string> > v_sensor_buckets = {
    {"camera_0", "pinhole"}, {"camera_1", "pinhole"}, {"camera_2", "pinhole"}, {"camera_3", "pinhole"},{"camera_4", "pinhole"},
    {"horizon_lidar", "multi"},  {"vertical_lidar", "multi"}, {"imu_config"}};

    for (size_t i = 0; i < v_sensor_buckets.size(); ++i) {
        root_node["sensor_bucket"].push_back(v_sensor_buckets[i]);
        root_node["sensor_bucket"][i].SetStyle(YAML::EmitterStyle::Flow);
    }

    time_t tt;
    time(&tt);
    struct tm* t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);
    root_node["calibration_time"] = buf;

    Eigen::Matrix4d T_cam0_cam1 = cam1.T_ref_cam_;
    Eigen::Matrix4d T_cam0_cam2 = cam2.T_ref_cam_;
    Eigen::Matrix4d T_cam0_cam3 = cam3.T_ref_cam_;
    Eigen::Matrix4d T_cam0_cam4 = cam4.T_ref_cam_;
    Eigen::Matrix4d T_cam0_base = mono_imu.T_cam0_imu;
    Eigen::Matrix4d T_base_cam0 = T_cam0_base.inverse();
    Eigen::Matrix4d T_base_cam1 = T_base_cam0 * T_cam0_cam1;
    Eigen::Matrix4d T_base_cam2 = T_base_cam0 * T_cam0_cam2;
    Eigen::Matrix4d T_base_cam3 = T_base_cam0 * T_cam0_cam3;
    Eigen::Matrix4d T_base_cam4 = T_base_cam0 * T_cam0_cam4;
    Eigen::Matrix4d T_base_l1 = T_base_cam0 * T_lidar1_cam0.inverse();
    Eigen::Matrix4d T_base_l0 = T_base_l1 * T_lidar0_lidar1.inverse();
    Eigen::Matrix4d T_base_imu = Eigen::Matrix4d::Identity();
    // cameras
    {
        {
            camera0_node["type"] = cam0.camera_type;
            camera0_node["distort_type"] = cam0.distortion_type;
            camera0_node["image_height"] = cam0.img_height_;
            camera0_node["image_width"] = cam0.img_width_;
            camera0_node["intrinsics"]["cols"] = cam0.intrinsic_.cols();
            camera0_node["intrinsics"]["rows"] = cam0.intrinsic_.rows();
            camera0_node["intrinsics"]["data"] = cam0.intrinsic_;
            camera0_node["intrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);

            camera0_node["distort_coefficient"]["cols"] = cam0.distortions_.cols();
            camera0_node["distort_coefficient"]["rows"] = cam0.distortions_.rows();
            camera0_node["distort_coefficient"]["data"] = cam0.distortions_;
            camera0_node["distort_coefficient"]["data"].SetStyle(YAML::EmitterStyle::Flow);
            camera0_node["extrinsics"]["cols"] = T_base_cam0.cols();
            camera0_node["extrinsics"]["rows"] = T_base_cam0.rows();
            camera0_node["extrinsics"]["data"] = T_base_cam0;
            camera0_node["extrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);
        }
        {
            camera1_node["type"] = cam1.camera_type;
            camera1_node["distort_type"] = cam1.distortion_type;
            camera1_node["image_height"] = cam1.img_height_;
            camera1_node["image_width"] = cam1.img_width_;
            camera1_node["intrinsics"]["cols"] = cam1.intrinsic_.cols();
            camera1_node["intrinsics"]["rows"] = cam1.intrinsic_.rows();
            camera1_node["intrinsics"]["data"] = cam1.intrinsic_;
            camera1_node["intrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);

            camera1_node["distort_coefficient"]["cols"] = cam1.distortions_.cols();
            camera1_node["distort_coefficient"]["rows"] = cam1.distortions_.rows();
            camera1_node["distort_coefficient"]["data"] = cam1.distortions_;
            camera1_node["distort_coefficient"]["data"].SetStyle(YAML::EmitterStyle::Flow);
            camera1_node["extrinsics"]["cols"] = T_base_cam1.cols();
            camera1_node["extrinsics"]["rows"] = T_base_cam1.rows();
            camera1_node["extrinsics"]["data"] = T_base_cam1;
            camera1_node["extrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);
        }
        {
            camera2_node["type"] = cam2.camera_type;
            camera2_node["distort_type"] = cam2.distortion_type;
            camera2_node["image_height"] = cam2.img_height_;
            camera2_node["image_width"] = cam2.img_width_;
            camera2_node["intrinsics"]["cols"] = cam2.intrinsic_.cols();
            camera2_node["intrinsics"]["rows"] = cam2.intrinsic_.rows();
            camera2_node["intrinsics"]["data"] = cam2.intrinsic_;
            camera2_node["intrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);

            camera2_node["distort_coefficient"]["cols"] = cam2.distortions_.cols();
            camera2_node["distort_coefficient"]["rows"] = cam2.distortions_.rows();
            camera2_node["distort_coefficient"]["data"] = cam2.distortions_;
            camera2_node["distort_coefficient"]["data"].SetStyle(YAML::EmitterStyle::Flow);
            camera2_node["extrinsics"]["cols"] = T_base_cam2.cols();
            camera2_node["extrinsics"]["rows"] = T_base_cam2.rows();
            camera2_node["extrinsics"]["data"] = T_base_cam2;
            camera2_node["extrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);
        }
        {
            camera3_node["type"] = cam3.camera_type;
            camera3_node["distort_type"] = cam3.distortion_type;
            camera3_node["image_height"] = cam3.img_height_;
            camera3_node["image_width"] = cam3.img_width_;
            camera3_node["intrinsics"]["cols"] = cam3.intrinsic_.cols();
            camera3_node["intrinsics"]["rows"] = cam3.intrinsic_.rows();
            camera3_node["intrinsics"]["data"] = cam3.intrinsic_;
            camera3_node["intrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);

            camera3_node["distort_coefficient"]["cols"] = cam3.distortions_.cols();
            camera3_node["distort_coefficient"]["rows"] = cam3.distortions_.rows();
            camera3_node["distort_coefficient"]["data"] = cam3.distortions_;
            camera3_node["distort_coefficient"]["data"].SetStyle(YAML::EmitterStyle::Flow);
            camera3_node["extrinsics"]["cols"] = T_base_cam3.cols();
            camera3_node["extrinsics"]["rows"] = T_base_cam3.rows();
            camera3_node["extrinsics"]["data"] = T_base_cam3;
            camera3_node["extrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);
        }
        {
            camera4_node["type"] = cam4.camera_type;
            camera4_node["distort_type"] = cam4.distortion_type;
            camera4_node["image_height"] = cam4.img_height_;
            camera4_node["image_width"] = cam4.img_width_;
            camera4_node["intrinsics"]["cols"] = cam4.intrinsic_.cols();
            camera4_node["intrinsics"]["rows"] = cam4.intrinsic_.rows();
            camera4_node["intrinsics"]["data"] = cam4.intrinsic_;
            camera4_node["intrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);

            camera4_node["distort_coefficient"]["cols"] = cam4.distortions_.cols();
            camera4_node["distort_coefficient"]["rows"] = cam4.distortions_.rows();
            camera4_node["distort_coefficient"]["data"] = cam4.distortions_;
            camera4_node["distort_coefficient"]["data"].SetStyle(YAML::EmitterStyle::Flow);
            camera4_node["extrinsics"]["cols"] = T_base_cam4.cols();
            camera4_node["extrinsics"]["rows"] = T_base_cam4.rows();
            camera4_node["extrinsics"]["data"] = T_base_cam4;
            camera4_node["extrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);
        }
    }

    // lidar 0
    {
        lidar0_node["max_dist"] = 30.0;
        lidar0_node["min_dist"] = 0.1;
        lidar0_node["starting_offset"] = -6.0;
        lidar0_node["num_rays"] = 16;
        lidar0_node["frequency"] = 10;
        lidar0_node["extrinsics"]["cols"] = T_base_l0.cols();
        lidar0_node["extrinsics"]["rows"] = T_base_l0.rows();
        lidar0_node["extrinsics"]["data"] = T_base_l0;
        lidar0_node["extrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);
    }

    // lidar 1
    {
        lidar1_node["max_dist"] = 30.0;
        lidar1_node["min_dist"] = 0.1;
        lidar1_node["starting_offset"] = -6.0;
        lidar1_node["num_rays"] = 16;
        lidar1_node["frequency"] = 10;
        lidar1_node["extrinsics"]["cols"] = T_base_l1.cols();
        lidar1_node["extrinsics"]["rows"] = T_base_l1.rows();
        lidar1_node["extrinsics"]["data"] = T_base_l1;
        lidar1_node["extrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);
    }

    // imu
    {
        imu_node["name"] = "xsens";
        imu_node["gyro_meas_noise"] = mono_imu.avg_gyro_noise_density;
        imu_node["gyro_bias_noise"] = mono_imu.avg_gyro_bias_noise_density;
        imu_node["acc_meas_noise"] = mono_imu.avg_acc_noise_density;
        imu_node["acc_bias_noise"] = mono_imu.avg_acc_bias_noise_density;
        imu_node["acc_bias"]["cols"] = mono_imu.acc_bias.cols();
        imu_node["acc_bias"]["rows"] = mono_imu.acc_bias.rows();
        imu_node["acc_bias"]["data"] = mono_imu.acc_bias;
        imu_node["acc_bias"]["data"].SetStyle(YAML::EmitterStyle::Flow);
        imu_node["gyro_bias"]["cols"] = mono_imu.gyro_bias.cols();
        imu_node["gyro_bias"]["rows"] = mono_imu.gyro_bias.rows();
        imu_node["gyro_bias"]["data"] = mono_imu.gyro_bias;
        imu_node["gyro_bias"]["data"].SetStyle(YAML::EmitterStyle::Flow);

        imu_node["extrinsics"]["cols"] = T_base_imu.cols();
        imu_node["extrinsics"]["rows"] = T_base_imu.rows();
        imu_node["extrinsics"]["data"] = T_base_imu;
        imu_node["extrinsics"]["data"].SetStyle(YAML::EmitterStyle::Flow);
    }

    root_node["camera_0"] = camera0_node;
    root_node["camera_1"] = camera1_node;
    root_node["camera_2"] = camera2_node;
    root_node["camera_3"] = camera3_node;
    root_node["horizon_lidar"] = lidar0_node;
    root_node["vertical_lidar"] = lidar1_node;
    root_node["imu_config"] = imu_node;

    yaml_out_file << root_node;
    yaml_out_file.close();

    return true;
}

bool readOutputFileTest(const std::string& file_name) {
  if (file_name.empty()) {
    std::cerr << "[readOutputFileTest] Empty input file name";
    return false;
  }

  std::ifstream fs(file_name);
  if (!fs.is_open()) {
    std::cout << "[readOutputFileTest] Error read " << file_name << "\n";
    return false;
  }

  YAML::Node root_node;

  try {
    root_node = YAML::LoadFile(file_name);
  } catch (YAML::BadFile& e) {
    std::cout << "[readOutputFileTest] Could not open file: " << file_name
              << "\n";
    return false;
  } catch (YAML::ParserException& e) {
    std::cout << "[readOutputFileTest] Invalid file format: " << file_name
              << "\n";
    return false;
  }
  if (root_node.IsNull()) {
    std::cout << "[readOutputFileTest] Could not open file: " << file_name
              << "\n";
    return false;
  }

  std::cout << "[readOutputFileTest] read " << file_name << "\n";

  YAML::Node left_stereo_node = root_node["left_stereo"];

  Eigen::Matrix3d stereo0_cam0_intrinsic =
      left_stereo_node["cam0"]["intrinsics"]["data"].as<Eigen::Matrix3d>();
  std::cout << "[readOutputFileTest] left_stereo-cam0-intrinsics:\n"
            << stereo0_cam0_intrinsic << "\n";

  Eigen::Vector4d stereo0_cam0_distortion =
      left_stereo_node["cam0"]["distort_coefficient"]["data"]
          .as<Eigen::Vector4d>();
  std::cout << "[readOutputFileTest] left_stereo-cam0-distort:\n"
            << stereo0_cam0_distortion << "\n";

  Eigen::Matrix4d stereo0_extrinsics =
      left_stereo_node["cam_extrinsics"]["data"].as<Eigen::Matrix4d>();
  std::cout << "[readOutputFileTest] left_stereo cam-extrinsic:\n"
            << stereo0_extrinsics << "\n";

  return true;
}

bool loadMonoCamParam(const std::string &cam_intrin_filepath, MonoCamParam &cam, std::string cam_ext_filepath){
    if (!common::fileExists(cam_intrin_filepath)){
        LOG(FATAL) << "file " << cam_intrin_filepath << " doesn't exist!";
        return false;
    }

    cv::Mat K, D;
    std::string dist_type;
    int width, height;
    if( !common::loadIntrinFileOpencv(cam_intrin_filepath, K, D, dist_type, width, height)){
        if (!common::loadIntrinFileKalibr(cam_intrin_filepath, K, D, dist_type, width, height))
            return false;
    }
    
    cv::cv2eigen(K, cam.intrinsic_);
    // D is type of (1, 4) matrix, while distortions_ is (4,1) matrix
    cv::cv2eigen(D.t(), cam.distortions_);
    
    cam.img_width_ = width;
    cam.img_height_ = height;
    cam.distortion_type = dist_type;

    LOG(INFO) << "camera distortion type: " << dist_type;
    if (cam_ext_filepath.empty()){
        cam.T_ref_cam_ = Eigen::Matrix4d::Identity();
    }else{
        if (!common::loadExtFileOpencv(cam_ext_filepath, cam.T_ref_cam_))
            return false;
    }

    return true;
}

bool loadIMUIntrinFileKalibr(const std::string &imu_intrin_filepath, MonoIMUParam &cam0_imu){
    if (!common::fileExists(imu_intrin_filepath)){
        LOG(FATAL) << "File doesnt exist : " << imu_intrin_filepath;
        return false;
    }

    std::ifstream fs(imu_intrin_filepath);
    if (!fs.is_open()) {
    LOG(INFO) << " Error read " << imu_intrin_filepath << "\n";
    return false;
    }

    YAML::Node root_node;
    try {
    root_node = YAML::LoadFile(imu_intrin_filepath);
    } catch (YAML::BadFile& e) {
    LOG(INFO) << " Could not open file: " << imu_intrin_filepath << "\n";
    return false;
    } catch (YAML::ParserException& e) {
    LOG(INFO) << " Invalid file format: " << imu_intrin_filepath << "\n";
    return false;
    }
    if (root_node.IsNull()) {
    LOG(INFO) << " Could not open file: " << imu_intrin_filepath << "\n";
    return false;
    }


    double acc_noise = root_node["accelerometer_noise_density"].as<double>();
    double gyro_noise = root_node["gyroscope_noise_density"].as<double>();
    double acc_bias_noise = root_node["accelerometer_random_walk"].as<double>();
    double gyro_bias_noise = root_node["gyroscope_random_walk"].as<double>();
    LOG(INFO) << "acc_noise: " << acc_noise;
    LOG(INFO) << "gyro_noise: " << gyro_noise;
    LOG(INFO) << "acc_bias_noise: " << acc_bias_noise;
    LOG(INFO) << "gyro_bias_noise: " << gyro_bias_noise;
    cam0_imu.avg_acc_noise_density = acc_noise;
    cam0_imu.avg_acc_bias_noise_density = acc_bias_noise;
    cam0_imu.avg_gyro_noise_density = gyro_noise;
    cam0_imu.avg_gyro_bias_noise_density = gyro_bias_noise;
    cam0_imu.acc_bias[0] = root_node["acc_bias"][0].as<double>();
    cam0_imu.acc_bias[1] = root_node["acc_bias"][1].as<double>();
    cam0_imu.acc_bias[2] = root_node["acc_bias"][2].as<double>();
    cam0_imu.gyro_bias[0] = root_node["gyro_bias"][0].as<double>();
    cam0_imu.gyro_bias[1] = root_node["gyro_bias"][1].as<double>();
    cam0_imu.gyro_bias[2] = root_node["gyro_bias"][2].as<double>();
    LOG(INFO) << "acc_bias : " << cam0_imu.acc_bias.transpose();
    LOG(INFO) << "gyro_bias : " << cam0_imu.gyro_bias.transpose();

    return true;
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << " Usage : test_gen_backpack_yaml [calib_dataset_folder] [output_folder] [device_id] [cam_num]\n";
        return -1;
    }

    // dataset folder must contains  extrinsics : camera0_to_camera1.yml camera0_to_camera2.yml camera0_to_camera3.yml lidar0_to_lidar1.yml lidar1_to_camera0.yml camera0_to_imu.yaml
    // intrinsics: cam0.yml cam1.yml cam2.yml cam3.yml 
    std::string input_folder = argv[1];
    std::string output_folder = argv[2];
    std::string device_id = argv[3];
    std::string backpack_device_name = "backpack"+device_id;
    int cam_num = 4;
    if (argc == 5)
      cam_num = std::stoi(argv[4]);

    // extrinsic files
    std::string lidar0_2_lidar1_filepath = common::concatenateFolderAndFileName(input_folder, "lidar0_to_lidar1.yml");
    std::string lidar1_2_cam0_filepath = common::concatenateFolderAndFileName(input_folder, "lidar1_to_camera0.yml");
    std::string cam0_2_cam1_filepath = common::concatenateFolderAndFileName(input_folder, "camera0_to_camera1.yml");
    std::string cam0_2_cam2_filepath = common::concatenateFolderAndFileName(input_folder, "camera0_to_camera2.yml");
    std::string cam0_2_cam3_filepath = common::concatenateFolderAndFileName(input_folder, "camera0_to_camera3.yml");
    std::string cam0_2_cam4_filepath = common::concatenateFolderAndFileName(input_folder, "camera0_to_camera4.yml");
    std::string cam0_2_imu_filepath = common::concatenateFolderAndFileName(input_folder, "camera0_to_imu.yaml");
    // intrinsic files
    std::string cam0_intrin_filepath = common::concatenateFolderAndFileName(input_folder, "cam0.yml");
    std::string cam1_intrin_filepath = common::concatenateFolderAndFileName(input_folder, "cam1.yml");
    std::string cam2_intrin_filepath = common::concatenateFolderAndFileName(input_folder, "cam2.yml");
    std::string cam3_intrin_filepath = common::concatenateFolderAndFileName(input_folder, "cam3.yml");
    std::string cam4_intrin_filepath = common::concatenateFolderAndFileName(input_folder, "cam4.yml");
    std::string imu_intrin_filepath = common::concatenateFolderAndFileName(input_folder, backpack_device_name+"_imu.yaml");

    std::string output_filepath =  common::concatenateFolderAndFileName(output_folder, "raw_" + backpack_device_name +".yaml");

    MonoCamParam cam0, cam1, cam2, cam3, cam4;
    MonoIMUParam cam0_imu;
    bool sts = loadMonoCamParam(cam0_intrin_filepath, cam0, "");
    if (!sts){
        LOG(ERROR) << "Fail to parse cam0 intrinsic file and extrinsic file " << cam0_intrin_filepath << ", ";
        return -1;
    }
    sts = loadMonoCamParam(cam1_intrin_filepath, cam1, cam0_2_cam1_filepath);
    if (!sts){
        LOG(ERROR) << "Fail to parse cam1 intrinsic file and extrinsic file " << cam1_intrin_filepath << ", " << cam0_2_cam1_filepath;
        return -1;
    }
    sts = loadMonoCamParam(cam2_intrin_filepath, cam2, cam0_2_cam2_filepath);
    if (!sts){
        LOG(ERROR) << "Fail to parse cam2 intrinsic file and extrinsic file " << cam2_intrin_filepath << ", " << cam0_2_cam2_filepath;
        return -1;
    }
    sts = loadMonoCamParam(cam3_intrin_filepath, cam3, cam0_2_cam3_filepath);
    if (!sts){
        LOG(ERROR) << "Fail to parse cam3 intrinsic file and extrinsic file " << cam3_intrin_filepath << ", " << cam0_2_cam3_filepath;
        return -1;
    }
    if (cam_num == 5) {
        sts = loadMonoCamParam(cam4_intrin_filepath, cam4, cam0_2_cam4_filepath);
        if (!sts){
            LOG(ERROR) << "Fail to parse cam4 intrinsic file and extrinsic file " << cam4_intrin_filepath << ", " << cam0_2_cam4_filepath;
            return -1;
        }
    }
    // load cam2imu extrinsic
    // sts = loadMonoIMUExtFileKalibr(cam0_2_imu_filepath, cam0_imu);
    sts = common::loadExtFileOpencv(cam0_2_imu_filepath, cam0_imu.T_cam0_imu);
    if (!sts) {
          LOG(ERROR) << " Fail to read "<< cam0_2_imu_filepath;
          return -1;
    }
    // load imu intrinsic parameters
    sts = loadIMUIntrinFileKalibr(imu_intrin_filepath, cam0_imu);
    if (!sts) {
        LOG(ERROR) << " Fail to read "<< imu_intrin_filepath;
        return -1;
    }

    // lidar0 2 lidar1
    Eigen::Matrix4d T_lidar0_lidar1 = Eigen::Matrix4d::Identity();
    sts = common::loadExtFileOpencv(lidar0_2_lidar1_filepath, T_lidar0_lidar1);
    if (!sts) {
        LOG(FATAL) << " Fail to read " << lidar0_2_lidar1_filepath << "\n";
        return -1;
    }

    // lidar1 2 cam0
    Eigen::Matrix4d T_lidar1_cam0 = Eigen::Matrix4d::Identity();
    sts = common::loadExtFileOpencv(lidar1_2_cam0_filepath, T_lidar1_cam0);
    if (!sts) {
        LOG(FATAL) << " Fail to read " << lidar1_2_cam0_filepath << "\n";
        return -1;
    }
    // define imu frame as body frame
    sts = writeYAMLOutputFile(output_filepath, cam0, cam1, cam2, cam3, cam4,
                            cam0_imu, T_lidar0_lidar1, T_lidar1_cam0, std::stoi(device_id));
    if (!sts) {
        LOG(ERROR) << " Fail to write " << output_filepath << "\n";
        return -1;
    }

    //    sts = readOutputFileTest(output_fn);

    return 0;
}
