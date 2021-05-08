#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <boost/filesystem.hpp>

struct CameraParam
{
    int image_width;
    int image_height;

    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
};

bool checkAndCreateFolder(const std::string &folder)
{
    try
    {
        if (!boost::filesystem::exists(folder))
        {
            boost::filesystem::create_directories(folder);
            boost::filesystem::permissions(folder, boost::filesystem::all_all);
        }
    }
    catch (boost::filesystem::filesystem_error &e)
    {
        std::cout << __FUNCTION__ << e.what() << " " << folder << std::endl;
        return false;
    }

    return boost::filesystem::exists(folder);
}

bool readIntrinsic(const std::string &file_name, CameraParam &cam_param)
{
    if (file_name.empty())
    {
        std::cout << "[readFileStorage] Empty input file name " << file_name << "\n";
        return false;
    }

    cv::FileStorage fsCamera(file_name, cv::FileStorage::READ);
    if (!fsCamera.isOpened())
    {
        std::cout << "invalid camera calibration file: " << file_name << "\n";
        return false;
    }

    cam_param.image_width = (int)fsCamera["image_width"];
    cam_param.image_height = (int)fsCamera["image_height"];

    fsCamera["camera_matrix"] >> cam_param.camera_matrix;
    fsCamera["distortion_coefficients"] >> cam_param.distortion_coefficients;
    std::cout << "camera_matrix : \n"
              << cam_param.camera_matrix << "\n";
    std::cout << "distortion_coefficients : \n"
              << cam_param.distortion_coefficients << "\n";

    fsCamera.release();

    return true;
}

bool getImgPaths(const std::string &imgFolder, const std::string &imgListFileName, std::vector<std::string> &imgAbsoluteName)
{
    if (imgFolder.empty())
    {
        std::cout << "[getImgPaths] Empty input folder!\n";
        return false;
    }
    if (imgListFileName.empty())
    {
        std::cout << "[getImgPaths] Empty input image list file name!\n";
        return false;
    }

    imgAbsoluteName.clear();

    std::string imgListFile = imgFolder + imgListFileName;
    std::ifstream ifs(imgListFile, std::ios::in);
    if (!ifs.is_open())
        return false;

    while (!ifs.eof())
    {
        std::string line_data;
        getline(ifs, line_data);
        // assume the image path is image_xxx.jpg or image_xxx.png
        if (line_data.empty() || line_data.size() < 5)
            continue;
        std::string extension = line_data.substr(line_data.size() - 4);
        if (extension != ".jpg" && extension != ".png")
            continue;
        imgAbsoluteName.push_back(imgFolder + line_data);
    }

    ifs.close();

    if (imgAbsoluteName.empty())
        return false;

    return true;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "[camera_undistort] Useage camera_undistor : camera_undistor img_folder distorted_model\n";
        return -1;
    }

    //! image path relative parameters
    std::string image_folder(argv[1]);
    std::string undis_image_folder = image_folder.substr(0, image_folder.find_last_of('/') + 1);
    undis_image_folder = undis_image_folder + "undist_" + image_folder.substr(image_folder.find_last_of('/') + 1) + "/";
    std::cout << "[camera_undistort] undistorted image folder " << undis_image_folder << "\n";

    if (image_folder.back() != '/')
        image_folder += "/";
    std::cout << "Image folder: " << image_folder << "\n";

    if (!checkAndCreateFolder(image_folder))
    {
        std::cout << "[camera_undistort] Image folder does not exit, quit...\n";
        return -1;
    }
    //! check and create undistorted image folder
    if (!checkAndCreateFolder(undis_image_folder))
    {
        std::cout << "[camera_undistort] Fail to create " << undis_image_folder << "\n";
        return -1;
    }

    std::string img_list_file = "image_list.txt";
    // absolute img file paths
    std::vector<std::string> v_img_paths;
    if (!getImgPaths(image_folder, img_list_file, v_img_paths))
    {
        std::cout << "[camera_undistort] Fail to get images' abosulte path!\n";
        return -1;
    }

    //! intrisnic relative parameters
    std::string distortType(argv[2]);
    std::string camera_intrin_fn;
    if (distortType == "equidistant")
    {
        camera_intrin_fn = "equi_intrinsic.yml";
    }
    else
    {
        if (distortType == "radtan")
        {
            camera_intrin_fn = "radtan_intrinsic.yml";
        }
        else
        {
            std::cerr << "[camera_undistort] Unknown camera distortion type!\n";
            return -1;
        }
    }
    // read camera intrinsic parameters
    CameraParam camParam;
    std::string intrinsic_abs_fn = image_folder + camera_intrin_fn;
    if (!readIntrinsic(intrinsic_abs_fn, camParam))
    {
        std::cout << "[camera_undistort] Fail to read camera intrinsic parameters from " << intrinsic_abs_fn << "\n";
        return -1;
    }

    std::cout << "[camera_undistort] Total distorted image number is " << v_img_paths.size() << "\n";

    // std::vector<int> compression_params;
    // compression_params.push_back(cv::ImwriteFlags::IMWRITE_JPEG_QUALITY);
    // compression_params.push_back(100);

    for (size_t i = 0; i < v_img_paths.size(); i++)
    {
        const std::string &img_name = v_img_paths[i];
        cv::Mat image = cv::imread(img_name, cv::IMREAD_UNCHANGED);
        if (image.empty())
        {
            std::cout << "[camera_undistort] Fail to load the image: " << img_name << "\n";
            continue;
        }

        cv::Mat undistorted;
        cv::Mat new_cam_matrix = cv::Mat_<double>::eye(3,3);

        if (distortType == "equidistant")
        {
            cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1), map1, map2;
            new_cam_matrix.at<double>(0,0) = camParam.camera_matrix.at<double>(0,0) * 0.72;
            new_cam_matrix.at<double>(1,1) = new_cam_matrix.at<double>(0,0);
            new_cam_matrix.at<double>(0,2) = image.cols / 2;
            new_cam_matrix.at<double>(1,2) = image.rows / 2;
            cv::fisheye::initUndistortRectifyMap(camParam.camera_matrix, camParam.distortion_coefficients,
                                                 R, new_cam_matrix, cv::Size(image.cols, image.rows), CV_32FC1, map1, map2);

            cv::remap(image, undistorted, map1, map2, cv::InterpolationFlags::INTER_CUBIC);
        }
        if (distortType == "radtan")
        {
            cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1), map1, map2;
            new_cam_matrix = camParam.camera_matrix;
            cv::initUndistortRectifyMap(camParam.camera_matrix, camParam.distortion_coefficients,
                                        R, new_cam_matrix, cv::Size(image.cols, image.rows), CV_32FC1, map1, map2);

            cv::remap(image, undistorted, map1, map2, cv::InterpolationFlags::INTER_CUBIC);
        }

        std::string undist_img_name = undis_image_folder + "undist_";
        undist_img_name += img_name.substr(img_name.find_last_of('/') + 1);
        std::cout << "[camera_undistort] Save undistorted image " << undist_img_name << "\n";
        {
            std::string save_cam_intrinsic_path = image_folder + "new_equi_intrinsic.yml";
            cv::FileStorage fs(save_cam_intrinsic_path, cv::FileStorage::WRITE);
            if(!fs.isOpened()){
                std::cout << "[camera_unditort] Fail to open " << save_cam_intrinsic_path;
            }
            fs << "camera_model" << "equidistant";
            fs << "image_width" << camParam.image_width;
            fs << "image_height" << camParam.image_height;

            fs << "camera_matrix" << new_cam_matrix;
            cv::Mat distort_coeff = cv::Mat_<double>::zeros(1, 4);
            fs << "distortion_coefficients" << distort_coeff;
            fs.release();
        }


        // cv::imwrite(undist_img_name, undistorted, compression_params);
        cv::imwrite(undist_img_name, undistorted);
    }

    return 0;
}
