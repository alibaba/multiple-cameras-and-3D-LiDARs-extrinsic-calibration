#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>
#include <cassert>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "common.h"

/**
  * @brief parse cctag result file, image_xxx.jpg_CC.
  * @param file_name is the result file name.
  * @param img_path is the image file path.
  * @param totalFrames is keyframes vector.
  * @return vector of detected keypoints.
  */
std::vector<cv::Point2d> readDetectionResult(const std::string &file_name, std::string &img_path)
{
    if (file_name.empty())
    {
        std::cout << "[readDetectionResult] Empty input result file name!\n";
        return std::vector<cv::Point2d>();
    }
    std::vector<cv::Point2d> points;

    std::ifstream ifs(file_name);
    if (!ifs.is_open())
    {
        std::cout << "[readDetectionResult] Fail to open result file name!\n";
        return std::vector<cv::Point2d>();
    }

    std::getline(ifs, img_path);
    if (img_path.empty())
    {
        std::cout << "[readDetectionResult] Invalid image file path!\n";
        return std::vector<cv::Point2d>();
    }

    std::string line;
    while (std::getline(ifs, line))
    {
        if (line.empty())
            break;

        std::istringstream iss(line);
        double x = 0, y = 0;

        if (!(iss >> x >> y))
        {
            break;
        }
        else
        {
            if (x >= 0 && y >= 0)
                points.emplace_back(cv::Point2d(x, y));
        }
    }

    return points;
}

/**
  * @brief Get file list of cctag detection result in foler.
  * @param myPath is the folder that contains cctag result.
  * @return vector of detectiion result file names.
  */
std::vector<std::string> getResultFileList(const std::string &myPath)
{
    if (myPath.empty())
    {
        std::cout << "[getResultFileList] Empty result folder path!\n";
        return std::vector<std::string>();
    }
    std::vector<std::string> fileList;

    std::vector<boost::filesystem::path> vFileInFolder;

    std::copy(boost::filesystem::directory_iterator(myPath), boost::filesystem::directory_iterator(), std::back_inserter(vFileInFolder)); // is directory_entry, which is
    std::sort(vFileInFolder.begin(), vFileInFolder.end(), [](const boost::filesystem::path &a, const boost::filesystem::path &b) {
        std::string as = a.stem().string().substr(a.stem().string().find_last_of("_") + 1);
        std::string bs = b.stem().string().substr(b.stem().string().find_last_of("_") + 1);

        int an = 0;
        int bn = 0;
        try
        {
            an = boost::lexical_cast<int>(as);
            bn = boost::lexical_cast<int>(bs);
        }
        catch (const std::exception &e)
        {
            //std::cerr << e.what() << '\n';
        }

        return an < bn; // or some custom code
    });

    for (const auto &fileInFolder : vFileInFolder)
    {
        const std::string subExt(boost::filesystem::extension(fileInFolder));

        if (subExt == ".jpg_CC" || subExt == ".png_CC")
        {
            fileList.push_back(fileInFolder.string());

            // std::cout << fileInFolder << std::endl;
        }
    }

    return fileList;
}

/**
  * @brief Write holistic cctag detection result into one file.
  * @param file_name is the yaml file name that contains monocular camera cctag result.
  * @param v_total_kpts is the total keypoints made up of each image.
  * @param v_total_img_nams is the total image file name of monocular camera.
  * @return .
  */
bool writeCompactResultFile(const std::string &file_name,
                            const std::vector<std::vector<cv::Point2d>> &v_total_kpts,
                            const std::vector<std::string> &v_total_img_nams)
{
    if (file_name.empty())
    {
        std::cout << "[writeCompactResultFile] Empty Input file name!\n";
        return false;
    }
    if (v_total_kpts.empty())
    {
        std::cout << "[writeCompactResultFile] Empty Input keypoints!\n";
        return false;
    }
    if (v_total_img_nams.empty())
    {
        std::cout << "[writeCompactResultFile] Empty Input image names!\n";
        return false;
    }

    assert(v_total_kpts.size() == v_total_img_nams.size());

    std::ofstream yaml_out_file(file_name);
    if (!yaml_out_file.is_open())
    {
        std::cout << "[writeCompactResultFile] Fail to open yaml file!\n";
        return false;
    }

    // YAML::Node root_node;
    YAML::Node frame_nodes;

    // write frames
    for (size_t j = 0; j < v_total_img_nams.size(); ++j)
    {
        YAML::Node yaml_node;
        std::string idx_str = std::to_string(j);

        // add image file name
        yaml_node["image_file"] = v_total_img_nams[j];

        // add 2d keypoints
        const std::vector<cv::Point2d> &v_kpts = v_total_kpts[j];
        for (size_t i = 0; i < v_kpts.size(); ++i)
        {
            std::vector<double> kpt;
            kpt.push_back(v_kpts[i].x);
            kpt.push_back(v_kpts[i].y);
            yaml_node["keypoints"].push_back(kpt);
        }
        yaml_node["keypoints"].SetStyle(YAML::EmitterStyle::Flow);

        frame_nodes[idx_str] = yaml_node;
    }
    // root_node["FrameInfo"] = frame_nodes;

    // yaml_out_file << root_node;
    yaml_out_file << frame_nodes;
    yaml_out_file.close();

    return true;
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        ERROR_STREAM("Usage : cctag_detector /path/to/input_images_folder /path/to/output_folder.\n"
                     << ": input_images_folder is the undistorted images folder.\n"
                     << ": output_folder is the detection result file folder.");
        return -1;
    }

    std::string input_folder(argv[1]);
    std::string output_folder(argv[2]);

    if (input_folder.back() != '/')
        input_folder += "/";
    if (output_folder.back() != '/')
        output_folder += "/";

    std::string left_img_folder = input_folder + "undist_left/";
    std::string right_img_folder = input_folder + "undist_right/";

    auto tp_1 = std::chrono::steady_clock::now();

    std::string cmd_str = "/home/ziqianbai/Projects/vlab/CCTag/build/src/applications/detection -n 3 -i " +
                          left_img_folder + " -o " + left_img_folder;
    int sts = system(cmd_str.c_str());
    if (sts < 0)
    {
        ERROR_STREAM("Call detection application fail!");
        return -1;
    }

    // sleep 5 second to relaunch cctag detection
    cmd_str = "sleep 15";
    sts = system(cmd_str.c_str());
    if (sts < 0)
    {
        ERROR_STREAM("Call sleep fail!");
        return -1;
    }

    cmd_str = "/home/ziqianbai/Projects/vlab/CCTag/build/src/applications/detection -n 3 -i " +
              right_img_folder + " -o " + right_img_folder;
    sts = system(cmd_str.c_str());
    if (sts < 0)
    {
        ERROR_STREAM("Call detection application fail!");
        return -1;
    }

    auto tp_2 = std::chrono::steady_clock::now();
    const auto detection_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    TIMER_STREAM("CCTAG detection time is " << detection_time << "s.");

    //! generate holistic cctag detection result
    std::vector<std::string> v_left_results = getResultFileList(left_img_folder);
    std::vector<std::string> v_right_results = getResultFileList(right_img_folder);
    if (v_left_results.empty() || v_right_results.empty())
    {
        ERROR_STREAM("Empty jpg_CC reuslts after cctag detection!");
        return -1;
    }

    assert(v_left_results.size() == v_right_results.size());
    std::vector<std::string> v_left_imgs;
    std::vector<std::string> v_right_imgs;
    std::vector<std::vector<cv::Point2d>> v_left_total_kpts;
    std::vector<std::vector<cv::Point2d>> v_right_total_kpts;

    for (size_t i = 0; i < v_left_results.size(); ++i)
    {
        const std::string &left_result_fn = v_left_results[i];
        const std::string &right_result_fn = v_right_results[i];

        std::string left_img_name, right_img_name;
        std::vector<cv::Point2d> left_kpts = readDetectionResult(left_result_fn, left_img_name);
        std::vector<cv::Point2d> right_kpts = readDetectionResult(right_result_fn, right_img_name);
        if (left_kpts.empty() || right_kpts.empty())
        {
            WARN_STREAM(" Empty circle detected on image " << left_img_name << " or image " << right_img_name);
            continue;
        }

        v_left_total_kpts.emplace_back(left_kpts);
        v_right_total_kpts.emplace_back(right_kpts);
        v_left_imgs.emplace_back(left_img_name);
        v_right_imgs.emplace_back(right_img_name);
    }

    //write into yaml file
    std::string left_camera_rst = output_folder + "left_result.yaml";
    std::string right_camera_rst = output_folder + "right_result.yaml";
    bool ret = writeCompactResultFile(left_camera_rst, v_left_total_kpts, v_left_imgs);
    if (!ret)
    {
        ERROR_STREAM("FAIL to write cctag detection into " << left_camera_rst);
        return -1;
    }
    ret = writeCompactResultFile(right_camera_rst, v_right_total_kpts, v_right_imgs);
    if (!ret)
    {
        ERROR_STREAM("FAIL to write cctag detection into " << right_camera_rst);
        return -1;
    }
    return 0;
}