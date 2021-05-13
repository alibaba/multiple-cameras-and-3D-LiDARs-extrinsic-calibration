#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cassert>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

/**
 * @brief Stereo camera parameters
 *
 */
struct StereoParam
{

    // extrinsic matrix, transform right camera frame into left camera frame
    Eigen::Matrix4d T_rl;
    // left camera intrinsic
    Eigen::Matrix3d cam0_intrinsic;
    // left camera distortion_coeffs
    double cam0_distortions[5];

    // right camera intrinsic
    Eigen::Matrix3d cam1_intrinsic;
    // right camera distortion_coeffs
    double cam1_distortions[5];

    int img_width;
    int img_height;
    // assume cam0 and cam1 use identical distortion type
    std::string distortion_type;

    StereoParam() : T_rl(Eigen::Matrix4d::Identity()),
                    cam0_intrinsic(Eigen::Matrix3d::Identity()),
                    cam1_intrinsic(Eigen::Matrix3d::Identity()),
                    img_width(2592),
                    img_height(2048)
    {
        memset(cam0_distortions, 0, 5 * sizeof(double));
        memset(cam1_distortions, 0, 5 * sizeof(double));
        distortion_type = "equidistant";
    }

    void getExtrinsic(cv::Mat &R_rl, cv::Mat &t_rl)
    {
        Eigen::Matrix3d R = T_rl.block<3, 3>(0, 0);
        Eigen::Vector3d t = T_rl.block<3, 1>(0, 3);

        cv::eigen2cv(R, R_rl);
        cv::eigen2cv(t, t_rl);
    }

    void getCam0Param(cv::Mat &K, cv::Mat &D)
    {
        cv::eigen2cv(cam0_intrinsic, K);
//        cv::Mat distortion = (cv::Mat_<double>(1, 5) << cam0_distortions[0], cam0_distortions[1],
//                cam0_distortions[2], cam0_distortions[3], cam0_distortions[4]);
        cv::Mat distortion = (cv::Mat_<double>(1, 4) << cam0_distortions[0], cam0_distortions[1],
                cam0_distortions[2], cam0_distortions[3]);
        D = distortion.clone();
    }

    void getCam1Param(cv::Mat &K, cv::Mat &D)
    {
        cv::eigen2cv(cam1_intrinsic, K);
//        cv::Mat distortion = (cv::Mat_<double>(1, 5) << cam1_distortions[0], cam1_distortions[1],
//                cam1_distortions[2], cam1_distortions[3], cam1_distortions[4]);
        cv::Mat distortion = (cv::Mat_<double>(1, 4) << cam1_distortions[0], cam1_distortions[1],
                cam1_distortions[2], cam1_distortions[3]);
        D = distortion.clone();
    }
};

/**
 * @brief Monocular camera parameters
 *
 */
struct MonocularParam{
    // camera intrinsic
    Eigen::Matrix3d cam_intrinsic;
    // camera distortion_coeffs
    double cam_distortions[5];

    int img_width;
    int img_height;

    std::string distortion_type;

    MonocularParam() :cam_intrinsic(Eigen::Matrix3d::Identity()),
                    img_width(2592),
                    img_height(2048)
    {
        memset(cam_distortions, 0, 5 * sizeof(double));
        distortion_type = "equidistant";
    }

    void getCamParam(cv::Mat &K, cv::Mat &D)
    {
        cv::eigen2cv(cam_intrinsic, K);
//        cv::Mat distortion = (cv::Mat_<double>(1, 5) << cam_distortions[0], cam_distortions[1],
//                cam_distortions[2], cam_distortions[3], cam_distortions[4]);
        cv::Mat distortion = (cv::Mat_<double>(1, 4) << cam_distortions[0], cam_distortions[1],
                cam_distortions[2], cam_distortions[3]);
        D = distortion.clone();
    }


};

/**
 * @brief parse kalibr yaml file support: pinhole + equidistant
 *
 * @param kalib_intrin_file
 * @param camera Camera Model param
 */
bool loadKalibrResult(const std::string &kalib_result_file, StereoParam &stereo_param)
{
    if (kalib_result_file.empty())
    {
        std::cout << "[loadKalibrResult] Empty input file name!\n";
        return false;
    }

    std::ifstream fs(kalib_result_file);
    if (!fs.is_open())
    {
        std::cout << "[loadKalibrResult] Error read " << kalib_result_file << "\n";
        return false;
    }

    YAML::Node root_node;
    YAML::Node cam0_node;
    YAML::Node cam1_node;

    try
    {
        root_node = YAML::LoadFile(kalib_result_file);
    }
    catch (YAML::BadFile &e)
    {
        std::cout << "[loadKalibrResult] Could not open file: " << kalib_result_file << "\n";
        return false;
    }
    catch (YAML::ParserException &e)
    {
        std::cout << "[loadKalibrResult] Invalid file format: " << kalib_result_file << "\n";
        return false;
    }
    if (root_node.IsNull())
    {
        std::cout << "[loadKalibrResult] Could not open file: " << kalib_result_file << "\n";
        return false;
    }

    int img_width = 0, img_height = 0;
    std::string distortion_type = "equidistant";

    cam0_node = root_node["cam0"];
    double cam0_distortion[5] = {0.0};
    cam0_distortion[0] = cam0_node["distortion_coeffs"][0].as<double>();
    cam0_distortion[1] = cam0_node["distortion_coeffs"][1].as<double>();
    cam0_distortion[2] = cam0_node["distortion_coeffs"][2].as<double>();
    cam0_distortion[3] = cam0_node["distortion_coeffs"][3].as<double>();
    // cam0_distortion[0] = cam0_node["distortion_coeffs"][0].as<double>();
    Eigen::Matrix3d cam0_intrinsic = Eigen::Matrix3d::Identity();
    cam0_intrinsic(0, 0) = cam0_node["intrinsics"][0].as<double>();
    cam0_intrinsic(1, 1) = cam0_node["intrinsics"][1].as<double>();
    cam0_intrinsic(0, 2) = cam0_node["intrinsics"][2].as<double>();
    cam0_intrinsic(1, 2) = cam0_node["intrinsics"][3].as<double>();

    cam1_node = root_node["cam1"];
    double cam1_distortion[5] = {0.0};
    cam1_distortion[0] = cam1_node["distortion_coeffs"][0].as<double>();
    cam1_distortion[1] = cam1_node["distortion_coeffs"][1].as<double>();
    cam1_distortion[2] = cam1_node["distortion_coeffs"][2].as<double>();
    cam1_distortion[3] = cam1_node["distortion_coeffs"][3].as<double>();
    // cam1_distortion[0] = cam1_node["distortion_coeffs"][0].as<double>();
    Eigen::Matrix3d cam1_intrinsic = Eigen::Matrix3d::Identity();
    cam1_intrinsic(0, 0) = cam1_node["intrinsics"][0].as<double>();
    cam1_intrinsic(1, 1) = cam1_node["intrinsics"][1].as<double>();
    cam1_intrinsic(0, 2) = cam1_node["intrinsics"][2].as<double>();
    cam1_intrinsic(1, 2) = cam1_node["intrinsics"][3].as<double>();

    img_width = cam1_node["resolution"][0].as<int>();
    img_height = cam1_node["resolution"][1].as<int>();

    distortion_type = cam1_node["distortion_model"].as<std::string>();

    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < cam1_node["T_cn_cnm1"].size(); ++i)
    {
        extrinsic(i, 0) = cam1_node["T_cn_cnm1"][i][0].as<double>();
        extrinsic(i, 1) = cam1_node["T_cn_cnm1"][i][1].as<double>();
        extrinsic(i, 2) = cam1_node["T_cn_cnm1"][i][2].as<double>();
        extrinsic(i, 3) = cam1_node["T_cn_cnm1"][i][3].as<double>();
    }

    stereo_param.T_rl = extrinsic;
    std::cout << "[loadKalibrResult] T_rl : \n"
              << stereo_param.T_rl << "\n";
    Eigen::Vector3d baseline = stereo_param.T_rl.block<3,1>(0,3);
    std::cout << "[loadKalibrResult] baseline norm : \n"
              << baseline.norm() << "\n";

    std::cout << "[loadKalibrResult] baseline * fx = \n" <<
                 (baseline.norm() * cam0_intrinsic(0, 0)) << "\n";

    stereo_param.cam0_intrinsic = cam0_intrinsic;
    std::cout << "[loadKalibrResult] cam0 intrinsic : \n"
              << stereo_param.cam0_intrinsic << "\n";
    memcpy(stereo_param.cam0_distortions, cam0_distortion, 5 * sizeof(double));
    std::cout << "[loadKalibrResult] cam0 distrotion : " << stereo_param.cam0_distortions[0] << ", " << stereo_param.cam0_distortions[3] << "\n";
    stereo_param.cam1_intrinsic = cam1_intrinsic;
    std::cout << "[loadKalibrResult] cam1 intrinsic : \n"
              << stereo_param.cam1_intrinsic << "\n";
    memcpy(stereo_param.cam1_distortions, cam1_distortion, 5 * sizeof(double));
    std::cout << "[loadKalibrResult] cam1 distrotion : " << stereo_param.cam1_distortions[0] << ", " << stereo_param.cam1_distortions[3] << "\n";
    stereo_param.img_width = img_width;
    stereo_param.img_height = img_height;
    stereo_param.distortion_type = distortion_type;

    return true;
}

/**
 * @brief parse kalibr yaml file support: pinhole + equidistant
 *
 * @param kalib_intrin_file
 * @param camera Camera Model param
 */
bool loadKalibrResult(const std::string &kalib_result_file, MonocularParam &mono_param)
{
    if (kalib_result_file.empty())
    {
        std::cout << "[loadKalibrResult] Empty input file name!\n";
        return false;
    }

    std::ifstream fs(kalib_result_file);
    if (!fs.is_open())
    {
        std::cout << "[loadKalibrResult] Error read " << kalib_result_file << "\n";
        return false;
    }

    YAML::Node root_node;
    YAML::Node cam0_node;
    try
    {
        root_node = YAML::LoadFile(kalib_result_file);
    }
    catch (YAML::BadFile &e)
    {
        std::cout << "[loadKalibrResult] Could not open file: " << kalib_result_file << "\n";
        return false;
    }
    catch (YAML::ParserException &e)
    {
        std::cout << "[loadKalibrResult] Invalid file format: " << kalib_result_file << "\n";
        return false;
    }
    if (root_node.IsNull())
    {
        std::cout << "[loadKalibrResult] Could not open file: " << kalib_result_file << "\n";
        return false;
    }

    int img_width = 0, img_height = 0;
    std::string distortion_type = "equi-distant";

    cam0_node = root_node["cam0"];
    double cam0_distortion[5] = {0.0};
    cam0_distortion[0] = cam0_node["distortion_coeffs"][0].as<double>();
    cam0_distortion[1] = cam0_node["distortion_coeffs"][1].as<double>();
    cam0_distortion[2] = cam0_node["distortion_coeffs"][2].as<double>();
    cam0_distortion[3] = cam0_node["distortion_coeffs"][3].as<double>();
    // cam0_distortion[0] = cam0_node["distortion_coeffs"][0].as<double>();
    Eigen::Matrix3d cam0_intrinsic = Eigen::Matrix3d::Identity();
    cam0_intrinsic(0, 0) = cam0_node["intrinsics"][0].as<double>();
    cam0_intrinsic(1, 1) = cam0_node["intrinsics"][1].as<double>();
    cam0_intrinsic(0, 2) = cam0_node["intrinsics"][2].as<double>();
    cam0_intrinsic(1, 2) = cam0_node["intrinsics"][3].as<double>();

    img_width = cam0_node["resolution"][0].as<int>();
    img_height = cam0_node["resolution"][1].as<int>();

    distortion_type = cam0_node["distortion_model"].as<std::string>();

    mono_param.cam_intrinsic = cam0_intrinsic;
    std::cout << "[loadKalibrResult] cam0 intrinsic : \n"
              << mono_param.cam_intrinsic << "\n";
    memcpy(mono_param.cam_distortions, cam0_distortion, 5 * sizeof(double));
    std::cout << "[loadKalibrResult] cam0 distrotion : " << mono_param.cam_distortions[0] << ", " << mono_param.cam_distortions[3] << "\n";

    mono_param.img_width = img_width;
    mono_param.img_height = img_height;
    mono_param.distortion_type = distortion_type;

    return true;
}

/**
 * @brief Write mono-camera parameters
 *
 * @param file_name is the output yaml file name.
 * @param mono_param is the monocular camera parameters.
 */
bool writeCamParam(const std::string &file_name, MonocularParam &mono_param)
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
    fs << "serial_num" <<  1;
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
bool writeExtrinsicParam(const std::string &file_name, StereoParam &stereo_param)
{
    if (file_name.empty())
    {
        std::cout << "[writeExtrinsicParam] Empty file name!\n";
        return false;
    }

    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        std::cout << "[writeExtrinsicParam] Fail to open " << file_name << "\n";
        return false;
    }

    cv::Mat T, R, t;
    stereo_param.getExtrinsic(R, t);
    cv::eigen2cv(stereo_param.T_rl, T);

    fs << "T_rl" << T;
    fs << "R" << R;
    fs << "T" << t.t();
    fs.release();
    return true;
}


#define MONOCULAR_RESULT 0
#define STEREO_RESULT 1

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cout << "[convert_Kalibr_output] Usage : convert_Kalibr_output kalibr_output.yaml output_dir  model \n"
                  << "This executable is used to convert multicamera kalibr calibration file into 3 files seperately!\n"
                  << "model : 0--convert monocular kalibr output yaml file \n"
                  << "model : 1--convert stereo kalibr output yaml file \n";
        return -1;
    }

    std::string input_fn(argv[1]);
    std::string output_dir(argv[2]);
    int model = std::stoi (argv[3]);

    if (output_dir.back() != '/')
        output_dir += "/";

    if(model == MONOCULAR_RESULT)
    {
        std::string output_cam0_intrinsic_fn = output_dir + "equi_intrinsic.yml";
        MonocularParam mono_param;

        bool sts = loadKalibrResult (input_fn, mono_param);
        if (!sts)
        {
            std::cout << "[convert_kalibr_result] Fail to load monocular kalibr result!\n";
            return -1;
        }

        sts = writeCamParam(output_cam0_intrinsic_fn, mono_param);
        if (!sts)
        {
            std::cout << "[convert_kalibr_result] Fail to write camera0 intrinsic!" << output_cam0_intrinsic_fn << "\n";
            return -1;
        }

        return 0;
    }


    if(model == STEREO_RESULT)
    {
        std::string output_cam0_intrinsic_fn = output_dir + "left_intrinsic.yml";
        std::string output_cam1_intrinsic_fn = output_dir + "right_intrinsic.yml";
        std::string output_stereo_fn = output_dir + "extrinsic.yml";

        StereoParam stereo_param;
        bool sts = loadKalibrResult(input_fn, stereo_param);
        if (!sts)
        {
            std::cout << "[convert_kalibr_result] Fail to load stereo kalibr result!\n";
            return -1;
        }

        sts = writeCamParam(output_cam0_intrinsic_fn, 0, stereo_param);
        if (!sts)
        {
            std::cout << "[convert_kalibr_result] Fail to write camera0 intrinsic!" << output_cam0_intrinsic_fn << "\n";
            return -1;
        }
        sts = writeCamParam(output_cam1_intrinsic_fn, 1, stereo_param);
        if (!sts)
        {
            std::cout << "[convert_kalibr_result] Fail to write camera1 intrisnic!" << output_cam1_intrinsic_fn << "\n";
            return -1;
        }

        sts = writeExtrinsicParam(output_stereo_fn, stereo_param);
        if (!sts)
        {
            std::cout << "[convert_kalibr_result] Fail to write extrinsic file " << output_stereo_fn << "\n";
            return -1;
        }
    }
    return 0;
}
