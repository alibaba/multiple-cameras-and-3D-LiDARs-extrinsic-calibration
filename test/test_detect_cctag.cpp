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
#include <glog/logging.h>

#include "FileSystemTools.h"


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
        LOG(ERROR) << " Empty input result file name!\n";
        return std::vector<cv::Point2d>();
    }
    std::vector<cv::Point2d> points;

    std::ifstream ifs(file_name);
    if (!ifs.is_open())
    {
        LOG(ERROR) << " Fail to open result file name!\n";
        return std::vector<cv::Point2d>();
    }

    std::getline(ifs, img_path);
    if (img_path.empty())
    {
        LOG(ERROR) << " Invalid image file path!\n";
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
        LOG(ERROR) << " Empty result folder path!\n";
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
        LOG(ERROR) << " Empty Input file name!\n";
        return false;
    }
    if (v_total_kpts.empty())
    {
        LOG(ERROR) << " Empty Input keypoints!\n";
        return false;
    }
    if (v_total_img_nams.empty())
    {
        LOG(ERROR) << " Empty Input image names!\n";
        return false;
    }

    assert(v_total_kpts.size() == v_total_img_nams.size());

    std::ofstream yaml_out_file(file_name);
    if (!yaml_out_file.is_open())
    {
        LOG(ERROR) << " Fail to open yaml file!\n";
        return false;
    }

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
    yaml_out_file << frame_nodes;
    yaml_out_file.close();

    return true;
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage : test_detect_cctag <input_img_folder> <result_filepath> <detection_exe_path>.\n";
        return -1;
    }

    std::string input_folder(argv[1]);
    std::string result_filepath(argv[2]);
    std::string exe_path = "/home/ziqianbai/Projects/vlab/CCTag/build/src/applications/detection";
    if (argc >= 4)
    {
        exe_path = argv[3];
    }

    if (!common::pathExists(input_folder))
    {
        LOG(FATAL) << " Input image folder " << input_folder << " doesnot exist!";
        return -1;
    }
    if (!common::fileExists(exe_path))
    {
        LOG(FATAL) << " detection exe doesnot exist!";
        return -1;
    }

    auto tp_1 = std::chrono::steady_clock::now();
    std::string cmd_str = exe_path + " -n 3 -i " + input_folder + " -o " + input_folder;
    int sts = system(cmd_str.c_str());
    if (sts < 0)
    {
        LOG(ERROR) << "Call detection application fail!";
        return -1;
    }
    // sleep 5 second 
    cmd_str = "sleep 5";
    sts = system(cmd_str.c_str());
    if (sts < 0)
    {
        LOG(ERROR) << "Call sleep fail!";
        return -1;
    }

    auto tp_2 = std::chrono::steady_clock::now();
    const auto detection_time = std::chrono::duration_cast<std::chrono::duration<double> >(tp_2 - tp_1).count();
    LOG(INFO) << "CCTAG detection time is " << detection_time << "s.";

    //! generate holistic cctag detection result
    std::vector<std::string> v_cctag_result = getResultFileList(input_folder);
    if (v_cctag_result.empty() )
    {
        LOG(ERROR) << "Empty jpg_CC reuslts after cctag detection!";
        return -1;
    }

    std::vector<std::string> v_img_paths;
    std::vector<std::vector<cv::Point2d> > v_total_kpts;

    for (size_t i = 0; i < v_cctag_result.size(); ++i)
    {
        const std::string &cctag_filepath = v_cctag_result[i];
        std::string img_filepath;
        std::vector<cv::Point2d> left_kpts = readDetectionResult(cctag_filepath, img_filepath);
        if (left_kpts.empty())
        {
            LOG(ERROR) << " Empty circle detected on image " << cctag_filepath;
            continue;
        }

        v_total_kpts.emplace_back(left_kpts);
        v_img_paths.emplace_back(img_filepath);
    }

    //write into result file
    bool ret = writeCompactResultFile(result_filepath, v_total_kpts, v_img_paths);
    if (!ret)
    {
        LOG(ERROR) << "FAIL to write cctag detection into " << result_filepath;
        return -1;
    }

    return 0;
}