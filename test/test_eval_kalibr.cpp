#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>


/**
 * @brief Stereo camera parameters
 *
 */
struct StereoParam {
  // extrinsic matrix, transform right camera frame into left camera frame
  Eigen::Matrix4d T_r_l;
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
      : T_r_l(Eigen::Matrix4d::Identity()),
        cam0_intrinsic(Eigen::Matrix3d::Identity()),
        cam1_intrinsic(Eigen::Matrix3d::Identity()),
        img_width(2592),
        img_height(2048) {
    memset(cam0_distortions, 0, 4 * sizeof(double));
    memset(cam1_distortions, 0, 4 * sizeof(double));
    camera_type = "pinhole";
    distortion_type = "equi-distant";
  }

  void getExtrinsic(cv::Mat &R_rl, cv::Mat &t_rl)
  {
      Eigen::Matrix3d R = T_r_l.block<3, 3>(0, 0);
      Eigen::Vector3d t = T_r_l.block<3, 1>(0, 3);

      cv::eigen2cv(R, R_rl);
      cv::eigen2cv(t, t_rl);
  }

  void getCam0Param(cv::Mat &K, cv::Mat &D)
  {
      cv::eigen2cv(cam0_intrinsic, K);
      cv::Mat distortion = (cv::Mat_<double>(1, 4) << cam0_distortions[0], cam0_distortions[1],
              cam0_distortions[2], cam0_distortions[3]);
      D = distortion.clone();
  }

  void getCam1Param(cv::Mat &K, cv::Mat &D)
  {
      cv::eigen2cv(cam1_intrinsic, K);
      cv::Mat distortion = (cv::Mat_<double>(1, 4) << cam1_distortions[0], cam1_distortions[1],
              cam1_distortions[2], cam1_distortions[3]);
      D = distortion.clone();
  }
};

/**
 * @brief Monocular camera parameters
 *
 */
struct MonocularParam {
  // camera intrinsic
  Eigen::Matrix3d cam_intrinsic;
  // camera distortion_coeffs
  double cam_distortions[4];

  int img_width;
  int img_height;

  std::string distortion_type;
  // camera model
  std::string camera_type;

  MonocularParam()
      : cam_intrinsic(Eigen::Matrix3d::Identity()),
        img_width(2592),
        img_height(2048),
        distortion_type("equi-distant"),
        camera_type("pinhole") {
    memset(cam_distortions, 0, 4 * sizeof(double));
  }

  void getCamParam(cv::Mat& K, cv::Mat& D) {
    cv::eigen2cv(cam_intrinsic, K);
    cv::Mat distortion =
        (cv::Mat_<double>(1, 4) << cam_distortions[0], cam_distortions[1],
         cam_distortions[2], cam_distortions[3]);
    D = distortion.clone();
  }
};

/**
 * @brief Stereo-IMU parameters
 *
 */
struct StereoIMUParam {
  // cam0 2 imu
  Eigen::Matrix4d T_cam0_imu;
  // cam1 2 imu
  Eigen::Matrix4d T_cam1_imu;

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

  StereoIMUParam()
      : T_cam0_imu(Eigen::Matrix4d::Identity()),
        T_cam1_imu(Eigen::Matrix4d::Identity()),
        acc_bias(Eigen::Vector3d::Zero()),
        gyro_bias(Eigen::Vector3d::Zero()),
        avg_acc_noise_density(0.0),
        avg_gyro_noise_density(0.0),
        avg_acc_bias_noise_density(0.0),
        avg_gyro_bias_noise_density(0.0) {}
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

  //    std::cout << "[readKalibrOutputFile] read "  << kalib_result_file <<
  //    ":\n";
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

  stereo_param.T_r_l = extrinsic;
  //    std::cout << "[readKalibrOutputFile] T_r_l : \n" << stereo_param.T_r_l
  //    << "\n";
  Eigen::Vector3d baseline = stereo_param.T_r_l.block<3, 1>(0, 3);
  //    std::cout << "[readKalibrOutputFile] baseline norm : " <<
  //    baseline.norm() << "\n";

  //    std::cout << "[loadKalibrResult] baseline * fx = " << (baseline.norm() *
  //    cam0_intrinsic(0, 0)) << "\n";

  stereo_param.cam0_intrinsic = cam0_intrinsic;
  //    std::cout << "[loadKalibrResult] cam0 intrinsic : \n" <<
  //    stereo_param.cam0_intrinsic << "\n";
  memcpy(stereo_param.cam0_distortions, cam0_distortion, 4 * sizeof(double));
  //    std::cout << "[loadKalibrResult] cam0 distrotion : " <<
  //    stereo_param.cam0_distortions[0] << ", " <<
  //    stereo_param.cam0_distortions[3] << "\n";
  stereo_param.cam1_intrinsic = cam1_intrinsic;
  //    std::cout << "[loadKalibrResult] cam1 intrinsic : \n" <<
  //    stereo_param.cam1_intrinsic << "\n";
  memcpy(stereo_param.cam1_distortions, cam1_distortion, 4 * sizeof(double));
  //    std::cout << "[loadKalibrResult] cam1 distrotion : " <<
  //    stereo_param.cam1_distortions[0] << ", " <<
  //    stereo_param.cam1_distortions[3] << "\n";
  stereo_param.img_width = img_width;
  stereo_param.img_height = img_height;
  stereo_param.distortion_type = distortion_type;
  stereo_param.camera_type = camera_type;

  return true;
}

bool readKalibrMonoOutputFile(const std::string& kalib_result_file,
                              MonocularParam& mono_param) {
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

  //    std::cout << "[readKalibrOutputFile] read "  << kalib_result_file <<
  //    ":\n";
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

  img_width = cam0_node["resolution"][0].as<int>();
  img_height = cam0_node["resolution"][1].as<int>();

  std::string distortion_type = cam0_node["distortion_model"].as<std::string>();
  std::string camera_type = cam0_node["camera_model"].as<std::string>();

  mono_param.cam_intrinsic = cam0_intrinsic;
  //    std::cout << "[loadKalibrResult] cam0 intrinsic : \n" <<
  //    mono_param.cam0_intrinsic << "\n";
  memcpy(mono_param.cam_distortions, cam0_distortion, 4 * sizeof(double));
  //    std::cout << "[loadKalibrResult] cam0 distrotion : " <<
  //    mono_param.cam0_distortions[0] << ", " << mono_param.cam0_distortions[3]
  //    << "\n";
  mono_param.img_width = img_width;
  mono_param.img_height = img_height;
  mono_param.distortion_type = distortion_type;
  mono_param.camera_type = camera_type;

  return true;
}

bool readKalibrStereoIMUOutputFile(const std::string& kalib_result_file,
                                   StereoIMUParam& stereo_imu_param) {
  if (kalib_result_file.empty()) {
    std::cout << "[readKalibrStereoIMUOutputFile] Empty input file name!\n";
    return false;
  }

  std::ifstream fs(kalib_result_file);
  if (!fs.is_open()) {
    std::cout << "[readKalibrStereoIMUOutputFile] Error read "
              << kalib_result_file << "\n";
    return false;
  }

  YAML::Node root_node;

  try {
    root_node = YAML::LoadFile(kalib_result_file);
  } catch (YAML::BadFile& e) {
    std::cout << "[readKalibrStereoIMUOutputFile] Could not open file: "
              << kalib_result_file << "\n";
    return false;
  } catch (YAML::ParserException& e) {
    std::cout << "[readKalibrStereoIMUOutputFile] Invalid file format: "
              << kalib_result_file << "\n";
    return false;
  }
  if (root_node.IsNull()) {
    std::cout << "[readKalibrStereoIMUOutputFile] Could not open file: "
              << kalib_result_file << "\n";
    return false;
  }

  std::cout << "[readKalibrStereoIMUOutputFile] read " << kalib_result_file
            << "\n";

  YAML::Node cam0_node = root_node["cam0"];
  Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
  for (size_t i = 0; i < cam0_node["T_cam_imu"].size(); ++i) {
    extrinsic(i, 0) = cam0_node["T_cam_imu"][i][0].as<double>();
    extrinsic(i, 1) = cam0_node["T_cam_imu"][i][1].as<double>();
    extrinsic(i, 2) = cam0_node["T_cam_imu"][i][2].as<double>();
    extrinsic(i, 3) = cam0_node["T_cam_imu"][i][3].as<double>();
  }

  stereo_imu_param.T_cam0_imu = extrinsic;

  YAML::Node cam1_node = root_node["cam1"];
  for (size_t i = 0; i < cam1_node["T_cam_imu"].size(); ++i) {
    extrinsic(i, 0) = cam1_node["T_cam_imu"][i][0].as<double>();
    extrinsic(i, 1) = cam1_node["T_cam_imu"][i][1].as<double>();
    extrinsic(i, 2) = cam1_node["T_cam_imu"][i][2].as<double>();
    extrinsic(i, 3) = cam1_node["T_cam_imu"][i][3].as<double>();
  }

  stereo_imu_param.T_cam1_imu = extrinsic;

  std::cout << "[readKalibrStereoIMUOutputFile] camera0-IMU : \n"
            << stereo_imu_param.T_cam0_imu << "\n";
  std::cout << "[readKalibrStereoIMUOutputFile] camera1-IMU : \n"
            << stereo_imu_param.T_cam1_imu << "\n";
  return true;
}

/**
 * @brief Write mono-camera parameters
 *
 * @param file_name is the output yaml file name.
 * @param mono_param is the monocular camera parameters.
 */
bool writeCamParam(const std::string& file_name, MonocularParam& mono_param) {
  if (file_name.empty()) {
    std::cout << "[writeCamParam] Empty file name!\n";
    return false;
  }

  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cout << "[writeCamParam] Fail to open " << file_name << "\n";
    return false;
  }

  fs << "camera_model" << mono_param.distortion_type;

  cv::Mat K, D;
  //
  {
    mono_param.getCamParam(K, D);
    fs << "camera_matrix" << K;
    fs << "distortion_coefficients" << D;
  }

  fs << "avg_reprojection_error" << 1.2799437834241592e-01;
  fs << "image_width" << mono_param.img_width;
  fs << "image_height" << mono_param.img_height;
  fs << "serial_num" << 1;
  fs.release();
  return true;
}

/**
 * @brief Write stereo-camera parameters
 *
 * @param file_name is the output yaml file name.
 * @param cam_index is .
 * @param mono_param is the monocular camera parameters.
 */
bool writeCamParam(const std::string &file_name, const int cam_index, StereoParam &stereo_param)
{
    if (file_name.empty())
    {
        std::cout << "[writeCamParam] Empty file name!\n";
        return false;
    }

    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        std::cout << "[writeCamParam] Fail to open " << file_name << "\n";
        return false;
    }

    fs << "image_width" << stereo_param.img_width;
    fs << "image_height" << stereo_param.img_height;
    fs << "distortion_type" << stereo_param.distortion_type;

    cv::Mat K, D;
    //
    if (0 == cam_index)
    {
        stereo_param.getCam0Param(K, D);
        fs << "camera_matrix" << K;
        fs << "distortion_coefficients" << D;
    }
    if (1 == cam_index)
    {
        stereo_param.getCam1Param(K, D);
        fs << "camera_matrix" << K;
        fs << "distortion_coefficients" << D;
    }

    fs.release();
    return true;
}

bool analyzeKalibrOutput(const std::vector<StereoParam>& v_stereo_params,
                         const std::string& output_fn,
                         const std::string& result_fn) {
  if (v_stereo_params.empty()) {
    std::cerr << "[analyKalibrOutput] Input is empty!\n";
    return false;
  }
  if (output_fn.empty()) {
    std::cerr << "[analyzeKalibrOutput] output file name is empty!\n";
    return false;
  }

  std::ofstream ofs(output_fn);
  if (!ofs.is_open()) {
    std::cerr << "[analyzeKalibrOutput] Fail to open " << output_fn << "\n";
    return false;
  }

  // fx,fy,cx,cy
  double avg_cam0_intrinsic_params[4] = {0.0};
  double avg_cam0_distortion_params[4] = {0.0};
  // k1,k2,k3,k4
  double avg_cam1_intrinsic_params[4] = {0.0};
  double avg_cam1_distortion_params[4] = {0.0};

  // calculate mean
  for (size_t i = 0; i < v_stereo_params.size(); ++i) {
    // cam0
    avg_cam0_intrinsic_params[0] += v_stereo_params[i].cam0_intrinsic(0, 0);
    avg_cam0_intrinsic_params[1] += v_stereo_params[i].cam0_intrinsic(1, 1);
    avg_cam0_intrinsic_params[2] += v_stereo_params[i].cam0_intrinsic(0, 2);
    avg_cam0_intrinsic_params[3] += v_stereo_params[i].cam0_intrinsic(1, 2);
    avg_cam0_distortion_params[0] += v_stereo_params[i].cam0_distortions[0];
    avg_cam0_distortion_params[1] += v_stereo_params[i].cam0_distortions[1];
    avg_cam0_distortion_params[2] += v_stereo_params[i].cam0_distortions[2];
    avg_cam0_distortion_params[3] += v_stereo_params[i].cam0_distortions[3];

    // cam1
    avg_cam1_intrinsic_params[0] += v_stereo_params[i].cam1_intrinsic(0, 0);
    avg_cam1_intrinsic_params[1] += v_stereo_params[i].cam1_intrinsic(1, 1);
    avg_cam1_intrinsic_params[2] += v_stereo_params[i].cam1_intrinsic(0, 2);
    avg_cam1_intrinsic_params[3] += v_stereo_params[i].cam1_intrinsic(1, 2);
    avg_cam1_distortion_params[0] += v_stereo_params[i].cam1_distortions[0];
    avg_cam1_distortion_params[1] += v_stereo_params[i].cam1_distortions[1];
    avg_cam1_distortion_params[2] += v_stereo_params[i].cam1_distortions[2];
    avg_cam1_distortion_params[3] += v_stereo_params[i].cam1_distortions[3];
  }

  int num = v_stereo_params.size();

  for (int i = 0; i < 4; ++i) {
    avg_cam0_intrinsic_params[i] /= num;
    avg_cam0_distortion_params[i] /= num;
    avg_cam1_intrinsic_params[i] /= num;
    avg_cam1_distortion_params[i] /= num;
  }

  std::cout << "[analyzeKalibrOutput] cam0 Average fx : "
            << avg_cam0_intrinsic_params[0] << "\n";
  ofs << "cam0 Average fx : " << avg_cam0_intrinsic_params[0] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average fy : "
            << avg_cam0_intrinsic_params[1] << "\n";
  ofs << "cam0 Average fy : " << avg_cam0_intrinsic_params[1] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average cx : "
            << avg_cam0_intrinsic_params[2] << "\n";
  ofs << "cam0 Average cx : " << avg_cam0_intrinsic_params[2] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average cy : "
            << avg_cam0_intrinsic_params[3] << "\n";
  ofs << "cam0 Average cy : " << avg_cam0_intrinsic_params[3] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average k1 : "
            << avg_cam0_distortion_params[0] << "\n";
  ofs << "cam0 Average k1 : " << avg_cam0_distortion_params[0] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average k2 : "
            << avg_cam0_distortion_params[1] << "\n";
  ofs << "cam0 Average k2 : " << avg_cam0_distortion_params[1] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average k3 : "
            << avg_cam0_distortion_params[2] << "\n";
  ofs << "cam0 Average k3 : " << avg_cam0_distortion_params[2] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average k4 : "
            << avg_cam0_distortion_params[3] << "\n";
  ofs << "cam0 Average k4 : " << avg_cam0_distortion_params[3] << "\n";

  std::cout << "[analyzeKalibrOutput] cam1 Average fx : "
            << avg_cam1_intrinsic_params[0] << "\n";
  ofs << "cam1 Average fx : " << avg_cam1_intrinsic_params[0] << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 Average fy : "
            << avg_cam1_intrinsic_params[1] << "\n";
  ofs << "cam1 Average fy : " << avg_cam1_intrinsic_params[1] << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 Average cx : "
            << avg_cam1_intrinsic_params[2] << "\n";
  ofs << "cam1 Average cx : " << avg_cam1_intrinsic_params[2] << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 Average cy : "
            << avg_cam1_intrinsic_params[3] << "\n";
  ofs << "cam1 Average cy : " << avg_cam1_intrinsic_params[3] << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 Average k1 : "
            << avg_cam1_distortion_params[0] << "\n";
  ofs << "cam1 Average k1 : " << avg_cam1_distortion_params[0] << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 Average k2 : "
            << avg_cam1_distortion_params[1] << "\n";
  ofs << "cam1 Average k2 : " << avg_cam1_distortion_params[1] << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 Average k3 : "
            << avg_cam1_distortion_params[2] << "\n";
  ofs << "cam1 Average k3 : " << avg_cam1_distortion_params[2] << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 Average k4 : "
            << avg_cam1_distortion_params[3] << "\n";
  ofs << "cam1 Average k4 : " << avg_cam1_distortion_params[3] << "\n";

  // calculate RMS
  double fx_rms[2] = {0}, fy_rms[2] = {0}, cx_rms[2] = {0}, cy_rms[2] = {0};
  double k1_rms[2] = {0}, k2_rms[2] = {0}, k3_rms[2] = {0}, k4_rms[2] = {0};
  for (size_t i = 0; i < v_stereo_params.size(); ++i) {
    // cam0
    double err_fx =
        v_stereo_params[i].cam0_intrinsic(0, 0) - avg_cam0_intrinsic_params[0];
    double err_fy =
        v_stereo_params[i].cam0_intrinsic(1, 1) - avg_cam0_intrinsic_params[1];
    double err_cx =
        v_stereo_params[i].cam0_intrinsic(0, 2) - avg_cam0_intrinsic_params[2];
    double err_cy =
        v_stereo_params[i].cam0_intrinsic(1, 2) - avg_cam0_intrinsic_params[3];

    fx_rms[0] += err_fx * err_fx;
    fy_rms[0] += err_fy * err_fy;
    cx_rms[0] += err_cx * err_cx;
    cy_rms[0] += err_cy * err_cy;

    double err_k1 =
        v_stereo_params[i].cam0_distortions[0] - avg_cam0_distortion_params[0];
    double err_k2 =
        v_stereo_params[i].cam0_distortions[1] - avg_cam0_distortion_params[1];
    double err_k3 =
        v_stereo_params[i].cam0_distortions[2] - avg_cam0_distortion_params[2];
    double err_k4 =
        v_stereo_params[i].cam0_distortions[3] - avg_cam0_distortion_params[3];

    k1_rms[0] += err_k1 * err_k1;
    k2_rms[0] += err_k2 * err_k2;
    k3_rms[0] += err_k3 * err_k3;
    k4_rms[0] += err_k4 * err_k4;

    // cam1
    err_fx =
        v_stereo_params[i].cam1_intrinsic(0, 0) - avg_cam1_intrinsic_params[0];
    err_fy =
        v_stereo_params[i].cam1_intrinsic(1, 1) - avg_cam1_intrinsic_params[1];
    err_cx =
        v_stereo_params[i].cam1_intrinsic(0, 2) - avg_cam1_intrinsic_params[2];
    err_cy =
        v_stereo_params[i].cam1_intrinsic(1, 2) - avg_cam1_intrinsic_params[3];

    fx_rms[1] += err_fx * err_fx;
    fy_rms[1] += err_fy * err_fy;
    cx_rms[1] += err_cx * err_cx;
    cy_rms[1] += err_cy * err_cy;

    err_k1 =
        v_stereo_params[i].cam1_distortions[0] - avg_cam1_distortion_params[0];
    err_k2 =
        v_stereo_params[i].cam1_distortions[1] - avg_cam1_distortion_params[1];
    err_k3 =
        v_stereo_params[i].cam1_distortions[2] - avg_cam1_distortion_params[2];
    err_k4 =
        v_stereo_params[i].cam1_distortions[3] - avg_cam1_distortion_params[3];

    k1_rms[1] += err_k1 * err_k1;
    k2_rms[1] += err_k2 * err_k2;
    k3_rms[1] += err_k3 * err_k3;
    k4_rms[1] += err_k4 * err_k4;
  }

  fx_rms[0] /= num;
  fx_rms[1] /= num;
  fy_rms[0] /= num;
  fy_rms[1] /= num;
  cx_rms[0] /= num;
  cx_rms[1] /= num;
  cy_rms[0] /= num;
  cy_rms[1] /= num;
  k1_rms[0] /= num;
  k1_rms[1] /= num;
  k2_rms[0] /= num;
  k2_rms[1] /= num;
  k3_rms[0] /= num;
  k3_rms[1] /= num;
  k4_rms[0] /= num;
  k4_rms[1] /= num;

  std::cout << "[analyzeKalibrOutput] cam0 fx rms: " << std::sqrt(fx_rms[0])
            << "\n";
  ofs << "cam0 fx rms: " << std::sqrt(fx_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 fy rms: " << std::sqrt(fy_rms[0])
            << "\n";
  ofs << "cam0 fy rms: " << std::sqrt(fy_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 cx rms: " << std::sqrt(cx_rms[0])
            << "\n";
  ofs << "cam0 cx rms: " << std::sqrt(cx_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 cy rms: " << std::sqrt(cy_rms[0])
            << "\n";
  ofs << "cam0 cy rms: " << std::sqrt(cy_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 k1 rms: " << std::sqrt(k1_rms[0])
            << "\n";
  ofs << "cam0 k1 rms: " << std::sqrt(k1_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 k2 rms: " << std::sqrt(k2_rms[0])
            << "\n";
  ofs << "cam0 k2 rms: " << std::sqrt(k2_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 k3 rms: " << std::sqrt(k3_rms[0])
            << "\n";
  ofs << "cam0 k3 rms: " << std::sqrt(k3_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 k4 rms: " << std::sqrt(k4_rms[0])
            << "\n";
  ofs << "cam0 k4 rms: " << std::sqrt(k4_rms[0]) << "\n";

  std::cout << "[analyzeKalibrOutput] cam1 fx rms: " << std::sqrt(fx_rms[1])
            << "\n";
  ofs << "cam1 fx rms: " << std::sqrt(fx_rms[1]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 fy rms: " << std::sqrt(fy_rms[1])
            << "\n";
  ofs << "cam1 fy rms: " << std::sqrt(fy_rms[1]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 cx rms: " << std::sqrt(cx_rms[1])
            << "\n";
  ofs << "cam1 cx rms: " << std::sqrt(cx_rms[1]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 cy rms: " << std::sqrt(cy_rms[1])
            << "\n";
  ofs << "cam1 cy rms: " << std::sqrt(cy_rms[1]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 k1 rms: " << std::sqrt(k1_rms[1])
            << "\n";
  ofs << "cam1 k1 rms: " << std::sqrt(k1_rms[1]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 k2 rms: " << std::sqrt(k2_rms[1])
            << "\n";
  ofs << "cam1 k2 rms: " << std::sqrt(k2_rms[1]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 k3 rms: " << std::sqrt(k3_rms[1])
            << "\n";
  ofs << "cam1 k3 rms: " << std::sqrt(k3_rms[1]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam1 k4 rms: " << std::sqrt(k4_rms[1])
            << "\n";
  ofs << "cam1 k4 rms: " << std::sqrt(k4_rms[1]) << "\n";

  assert(v_stereo_params.size() > 1);
  // avg delta R
  double avg_extrinsic_delta_rotation = 0.0;
  // avh delta t
  double avg_extrinsic_delta_t = 0.0;

  Eigen::Matrix4d reference_T_r_l_inv = v_stereo_params[0].T_r_l.inverse();
  for (int i = 0; i < num; ++i) {
    Eigen::Matrix4d T_r_l = v_stereo_params[i].T_r_l;
    Eigen::Matrix4d delta_T = T_r_l * reference_T_r_l_inv;
    Eigen::Matrix3d delta_R = delta_T.block<3, 3>(0, 0);
    Eigen::Vector3d delta_t = delta_T.block<3, 1>(0, 3);

    Eigen::AngleAxisd R_vec(delta_R);
    avg_extrinsic_delta_rotation += R_vec.angle() * 180.0 / M_PI;
    avg_extrinsic_delta_t += delta_t.norm();
  }

  std::cout << "[annalyzeKalibrOutput] Average T_r_l rotation error : "
            << avg_extrinsic_delta_rotation / num << "\n";
  ofs << "Average T_r_l rotation error : " << avg_extrinsic_delta_rotation / num
      << "\n";
  std::cout << "[annalyzeKalibrOutput] Average T_r_l translation error : "
            << avg_extrinsic_delta_t / num << "\n";
  ofs << "Average T_r_l translation error : " << avg_extrinsic_delta_t / num
      << "\n";

  ofs.close();

//  StereoParam avg_stereo_param;
//  avg_stereo_param.T_r_l = v_stereo_params[0].T_r_l;
//  avg_stereo_param.img_width = v_stereo_params[0].img_width;
//  avg_stereo_param.img_height = v_stereo_params[0].img_height;
//  avg_stereo_param.camera_type = v_stereo_params[0].camera_type;
//  avg_stereo_param.distortion_type = v_stereo_params[0].distortion_type;
//  avg_stereo_param.cam0_intrinsic(0, 0) = avg_cam0_intrinsic_params[0];
//  avg_stereo_param.cam0_intrinsic(1, 1) = avg_cam0_intrinsic_params[1];
//  avg_stereo_param.cam0_intrinsic(0, 2) = avg_cam0_intrinsic_params[2];
//  avg_stereo_param.cam0_intrinsic(1, 2) = avg_cam0_intrinsic_params[3];
//  avg_stereo_param.cam0_distortions[0] = avg_cam0_distortion_params[0];
//  avg_stereo_param.cam0_distortions[1] = avg_cam0_distortion_params[1];
//  avg_stereo_param.cam0_distortions[2] = avg_cam0_distortion_params[2];
//  avg_stereo_param.cam0_distortions[3] = avg_cam0_distortion_params[3];
//  avg_stereo_param.cam1_intrinsic(0, 0) = avg_cam1_intrinsic_params[0];
//  avg_stereo_param.cam1_intrinsic(1, 1) = avg_cam1_intrinsic_params[1];
//  avg_stereo_param.cam1_intrinsic(0, 2) = avg_cam1_intrinsic_params[2];
//  avg_stereo_param.cam1_intrinsic(1, 2) = avg_cam1_intrinsic_params[3];
//  avg_stereo_param.cam1_distortions[0] = avg_cam1_distortion_params[0];
//  avg_stereo_param.cam1_distortions[1] = avg_cam1_distortion_params[1];
//  avg_stereo_param.cam1_distortions[2] = avg_cam1_distortion_params[2];
//  avg_stereo_param.cam1_distortions[3] = avg_cam1_distortion_params[3];
//  writeCamParam (result_fn, avg_stereo_param);
  return true;
}

bool analyzeKalibrOutput(const std::vector<MonocularParam>& v_mono_params,
                         const std::string& output_fn,
                         const std::string& result_fn) {
  if (v_mono_params.empty()) {
    std::cerr << "[analyKalibrOutput] Input is empty!\n";
    return false;
  }
  if (output_fn.empty()) {
    std::cerr << "[analyzeKalibrOutput] output file name is empty!\n";
    return false;
  }

  std::ofstream ofs(output_fn);
  if (!ofs.is_open()) {
    std::cerr << "[analyzeKalibrOutput] Fail to open " << output_fn << "\n";
    return false;
  }

  // fx,fy,cx,cy
  double avg_cam0_intrinsic_params[4] = {0.0};
  // k1,k2,k3,k4
  double avg_cam0_distortion_params[4] = {0.0};

  // calculate mean
  for (size_t i = 0; i < v_mono_params.size(); ++i) {
    // cam0
    avg_cam0_intrinsic_params[0] += v_mono_params[i].cam_intrinsic(0, 0);
    avg_cam0_intrinsic_params[1] += v_mono_params[i].cam_intrinsic(1, 1);
    avg_cam0_intrinsic_params[2] += v_mono_params[i].cam_intrinsic(0, 2);
    avg_cam0_intrinsic_params[3] += v_mono_params[i].cam_intrinsic(1, 2);
    avg_cam0_distortion_params[0] += v_mono_params[i].cam_distortions[0];
    avg_cam0_distortion_params[1] += v_mono_params[i].cam_distortions[1];
    avg_cam0_distortion_params[2] += v_mono_params[i].cam_distortions[2];
    avg_cam0_distortion_params[3] += v_mono_params[i].cam_distortions[3];
  }

  int num = v_mono_params.size();

  for (int i = 0; i < 4; ++i) {
    avg_cam0_intrinsic_params[i] /= num;
    avg_cam0_distortion_params[i] /= num;
  }

  MonocularParam avg_cam_param;
  avg_cam_param.img_height = v_mono_params[0].img_height;
  avg_cam_param.img_width = v_mono_params[0].img_width;
  avg_cam_param.cam_intrinsic(0, 0) = avg_cam0_intrinsic_params[0];
  avg_cam_param.cam_intrinsic(1, 1) = avg_cam0_intrinsic_params[1];
  avg_cam_param.cam_intrinsic(0, 2) = avg_cam0_intrinsic_params[2];
  avg_cam_param.cam_intrinsic(1, 2) = avg_cam0_intrinsic_params[3];
  avg_cam_param.camera_type = v_mono_params[0].camera_type;
  avg_cam_param.distortion_type = v_mono_params[0].distortion_type;
  avg_cam_param.cam_distortions[0] = avg_cam0_distortion_params[0];
  avg_cam_param.cam_distortions[1] = avg_cam0_distortion_params[1];
  avg_cam_param.cam_distortions[2] = avg_cam0_distortion_params[2];
  avg_cam_param.cam_distortions[3] = avg_cam0_distortion_params[3];
  writeCamParam(result_fn, avg_cam_param);

  std::cout << "[analyzeKalibrOutput] cam0 Average fx : "
            << avg_cam0_intrinsic_params[0] << "\n";
  ofs << "cam0 Average fx : " << avg_cam0_intrinsic_params[0] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average fy : "
            << avg_cam0_intrinsic_params[1] << "\n";
  ofs << "cam0 Average fy : " << avg_cam0_intrinsic_params[1] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average cx : "
            << avg_cam0_intrinsic_params[2] << "\n";
  ofs << "cam0 Average cx : " << avg_cam0_intrinsic_params[2] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average cy : "
            << avg_cam0_intrinsic_params[3] << "\n";
  ofs << "cam0 Average cy : " << avg_cam0_intrinsic_params[3] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average k1 : "
            << avg_cam0_distortion_params[0] << "\n";
  ofs << "cam0 Average k1 : " << avg_cam0_distortion_params[0] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average k2 : "
            << avg_cam0_distortion_params[1] << "\n";
  ofs << "cam0 Average k2 : " << avg_cam0_distortion_params[1] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average k3 : "
            << avg_cam0_distortion_params[2] << "\n";
  ofs << "cam0 Average k3 : " << avg_cam0_distortion_params[2] << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 Average k4 : "
            << avg_cam0_distortion_params[3] << "\n";
  ofs << "cam0 Average k4 : " << avg_cam0_distortion_params[3] << "\n";

  // calculate RMS
  double fx_rms[2] = {0}, fy_rms[2] = {0}, cx_rms[2] = {0}, cy_rms[2] = {0};
  double k1_rms[2] = {0}, k2_rms[2] = {0}, k3_rms[2] = {0}, k4_rms[2] = {0};
  for (size_t i = 0; i < v_mono_params.size(); ++i) {
    // cam0
    double err_fx =
        v_mono_params[i].cam_intrinsic(0, 0) - avg_cam0_intrinsic_params[0];
    double err_fy =
        v_mono_params[i].cam_intrinsic(1, 1) - avg_cam0_intrinsic_params[1];
    double err_cx =
        v_mono_params[i].cam_intrinsic(0, 2) - avg_cam0_intrinsic_params[2];
    double err_cy =
        v_mono_params[i].cam_intrinsic(1, 2) - avg_cam0_intrinsic_params[3];

    fx_rms[0] += err_fx * err_fx;
    fy_rms[0] += err_fy * err_fy;
    cx_rms[0] += err_cx * err_cx;
    cy_rms[0] += err_cy * err_cy;

    double err_k1 =
        v_mono_params[i].cam_distortions[0] - avg_cam0_distortion_params[0];
    double err_k2 =
        v_mono_params[i].cam_distortions[1] - avg_cam0_distortion_params[1];
    double err_k3 =
        v_mono_params[i].cam_distortions[2] - avg_cam0_distortion_params[2];
    double err_k4 =
        v_mono_params[i].cam_distortions[3] - avg_cam0_distortion_params[3];

    k1_rms[0] += err_k1 * err_k1;
    k2_rms[0] += err_k2 * err_k2;
    k3_rms[0] += err_k3 * err_k3;
    k4_rms[0] += err_k4 * err_k4;
  }

  fx_rms[0] /= num;
  fy_rms[0] /= num;
  cx_rms[0] /= num;
  cy_rms[0] /= num;
  k1_rms[0] /= num;
  k2_rms[0] /= num;
  k3_rms[0] /= num;
  k4_rms[0] /= num;

  std::cout << "[analyzeKalibrOutput] cam0 fx rms: " << std::sqrt(fx_rms[0])
            << "\n";
  ofs << "cam0 fx rms: " << std::sqrt(fx_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 fy rms: " << std::sqrt(fy_rms[0])
            << "\n";
  ofs << "cam0 fy rms: " << std::sqrt(fy_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 cx rms: " << std::sqrt(cx_rms[0])
            << "\n";
  ofs << "cam0 cx rms: " << std::sqrt(cx_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 cy rms: " << std::sqrt(cy_rms[0])
            << "\n";
  ofs << "cam0 cy rms: " << std::sqrt(cy_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 k1 rms: " << std::sqrt(k1_rms[0])
            << "\n";
  ofs << "cam0 k1 rms: " << std::sqrt(k1_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 k2 rms: " << std::sqrt(k2_rms[0])
            << "\n";
  ofs << "cam0 k2 rms: " << std::sqrt(k2_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 k3 rms: " << std::sqrt(k3_rms[0])
            << "\n";
  ofs << "cam0 k3 rms: " << std::sqrt(k3_rms[0]) << "\n";
  std::cout << "[analyzeKalibrOutput] cam0 k4 rms: " << std::sqrt(k4_rms[0])
            << "\n";
  ofs << "cam0 k4 rms: " << std::sqrt(k4_rms[0]) << "\n";

  ofs.close();
  return true;
}

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: test_eval_kalibr camera_model file_num "
                 "kalibr_file1 kalibr_file2 kalibr_file3 "
                 "analysis_file_name avg_result_file_name...\n";
    std::cerr << "\t camera_model : mono, stereo\n";
    return -1;
  }

  std::string camera_model(argv[1]);
  int file_num = std::stoi(argv[2]);
  std::vector<std::string> v_file_names(file_num);
  for (int i = 0; i < file_num; ++i) {
    v_file_names[i] = argv[i + 3];
  }
  std::string analysis_file_name(argv[file_num + 3]);
  std::string avg_result_file_name(argv[file_num + 4]);

  if (camera_model == "stereo") {
    std::vector<StereoParam> v_stereo_params;
    for (int i = 0; i < file_num; ++i) {
      StereoParam stereo_param;
      bool sts = readKalibrStereoOutputFile(v_file_names[i], stereo_param);
      if (!sts) {
        std::cerr << "[AnalyzeKalibrOutput] Fail to read " << v_file_names[i]
                  << "\n";
        return -1;
      }
      v_stereo_params.emplace_back(stereo_param);
    }

    bool sts = analyzeKalibrOutput(v_stereo_params, analysis_file_name,
                                   avg_result_file_name);
    if (!sts) {
      std::cerr << "[AnalyzeKalibrOutput] Fail to analyze statistic data from "
                   "kalibr stereo output files.\n";
      return -1;
    }
  } else if (camera_model == "mono") {
    std::vector<MonocularParam> v_mono_params;
    for (int i = 0; i < file_num; ++i) {
      MonocularParam mono_param;
      bool sts = readKalibrMonoOutputFile(v_file_names[i], mono_param);
      if (!sts) {
        std::cerr << "[AnalyzeKalibrOutput] Fail to read " << v_file_names[i]
                  << "\n";
        return -1;
      }
      v_mono_params.emplace_back(mono_param);
    }

    bool sts = analyzeKalibrOutput(v_mono_params, analysis_file_name,
                                   avg_result_file_name);
    if (!sts) {
      std::cerr << "[AnalyzeKalibrOutput] Fail to analyze statistic data from "
                   "kalibr monocular output files.\n";
      return -1;
    }
  }
  return 0;
}
