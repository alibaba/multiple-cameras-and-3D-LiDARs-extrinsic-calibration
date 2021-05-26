
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/filesystem.hpp>
#include <Open3D/Open3D.h>
#include <glog/logging.h>

#include "FileSystemTools.h"
#include "YamlFileIO.h"


// fitness threshold
double min_fitness_thresh = 0.05;

struct RegistrationResult{
    // transform matrix
    Eigen::Matrix4d T;
    // fitness score
    double fitness;
    // inlier rms
    double inlier_rms;
};

// create init extrinsic file
void createLidarCamExtFile(const std::string& filepath, int lidar_id)
{
    std::string folder, filename;
    common::splitPathAndFilename(filepath, &folder, &filename);

    std::string init_T_l0_c0_filepath = common::concatenateFolderAndFileName(folder, "init_lidar0_to_camera0.yml");
    // new backpack structural value
    Eigen::Matrix4d T_l0_c0_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd l0_c0_vec1(-30 * M_PI / 180.0, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd l0_c0_vec2(90 * M_PI / 180.0, Eigen::Vector3d(1, 0, 0));
    Eigen::AngleAxisd l0_c0_vec3( M_PI, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d l0_c0_vec = l0_c0_vec1.matrix() * l0_c0_vec2.matrix() * l0_c0_vec3.matrix();
    Eigen::Vector3d t_l0_c0(0.0, -0.05657, -0.05931);
    T_l0_c0_gt.block<3, 3>(0, 0) = l0_c0_vec;
    T_l0_c0_gt.block<3, 1>(0, 3) = l0_c0_vec1.matrix() * t_l0_c0;
    // std::cout << "T_l0_c0_gt:\n" << T_l0_c0_gt << "\n";
    common::saveExtFileOpencv(init_T_l0_c0_filepath, T_l0_c0_gt);   

    Eigen::Matrix4d T_l0_l1_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd l0_l1_vec1(-30 * M_PI / 180.0, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd l0_l1_vec2(-73.5 * M_PI / 180.0, Eigen::Vector3d(0, 1, 0));
    Eigen::Matrix3d l0_l1_vec = l0_l1_vec1.matrix() * l0_l1_vec2.matrix();
    Eigen::Vector3d t_l0_l1(-0.31405, 0, -0.39803);
    T_l0_l1_gt.block<3, 3>(0, 0) = l0_l1_vec;
    T_l0_l1_gt.block<3, 1>(0, 3) = l0_l1_vec1.matrix() * t_l0_l1;
    // calc T_l1_c0
    Eigen::Matrix4d T_l1_c0_gt = T_l0_l1_gt.inverse() * T_l0_c0_gt;
    common::saveExtFileOpencv(filepath, T_l1_c0_gt);
}

bool alignTwoPointClouds(const std::string &src_pcl_file, const std::string &target_pcl_file, const Eigen::Matrix4d &T_init, 
                RegistrationResult &result){
    std::string path, file_name;
    common::splitPathAndFilename(src_pcl_file, &path, &file_name);

    auto src_pcd_ptr = open3d::io::CreatePointCloudFromFile(src_pcl_file);
    auto target_pcd_ptr = open3d::io::CreatePointCloudFromFile(target_pcl_file);

    if( src_pcd_ptr == nullptr || src_pcd_ptr == nullptr){
        LOG(ERROR) << "Fail to load src and target pointcloud file!";
        return false;
    }

    auto src_pcd_down_ptr = src_pcd_ptr->VoxelDownSample(0.05);
    src_pcd_ptr->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));

    auto target_pcd_down_ptr = target_pcd_ptr->VoxelDownSample(0.05);
    target_pcd_ptr->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));

    src_pcd_ptr->Transform(T_init);
    src_pcd_down_ptr->Transform(T_init);

    open3d::registration::RegistrationResult icp_result;

    const double max_corresp_dis = 0.03; // meter
    open3d::registration::ICPConvergenceCriteria icp_criteria(1e-6, 1e-6, 100);
    icp_result = open3d::registration::RegistrationICP(*src_pcd_ptr, *target_pcd_ptr, max_corresp_dis, Eigen::Matrix4d::Identity(),
                                                            open3d::registration::TransformationEstimationPointToPlane(), icp_criteria);
    // icp_result = open3d::registration::RegistrationICP(*src_pcd_ptr, *target_pcd_ptr, max_corresp_dis, Eigen::Matrix4d::Identity(),
    //                                                 open3d::registration::TransformationEstimationPointToPoint(false), icp_criteria);

    // transform source pointcloud with icp_transformation
    Eigen::Matrix4d T_icp = icp_result.transformation_;

    LOG(INFO) << "#############" << path << " ################";
    result.fitness = icp_result.fitness_;
    LOG(INFO) << "icp_result.fitness: " << icp_result.fitness_;
    result.inlier_rms = icp_result.inlier_rmse_;
    LOG(INFO) << "icp_result.inlier_rmse_ : " << icp_result.inlier_rmse_;

    if(result.fitness < min_fitness_thresh){
        LOG(ERROR) << "fitness score < " << min_fitness_thresh;
        return false;
    }

    // final transformation between source pointcloud and target pointcloud
    result.T = T_icp * T_init;
    src_pcd_down_ptr->Transform(T_icp);
    
    std::string save_file_path;

    save_file_path = common::concatenateFolderAndFileName(path, "transformed_source_pointcloud.ply");
    open3d::io::WritePointCloudToPLY(save_file_path, *src_pcd_down_ptr, true);
    std::shared_ptr<open3d::geometry::PointCloud> merge_points(new open3d::geometry::PointCloud);
    *merge_points = *src_pcd_down_ptr + *target_pcd_down_ptr;
    
    save_file_path = common::concatenateFolderAndFileName(path, "merged_pointcloud.ply");
    open3d::io::WritePointCloudToPLY(save_file_path, *merge_points, true);
    
    return true;
}

void evaluateExtrinsics(const std::vector<RegistrationResult> & v_results, const Eigen::Matrix4d &T_baseline){
    assert(v_results.size() > 1);

    const Eigen::Matrix4d T_0 = v_results[0].T;
    std::vector<double> v_inlier_rms = {v_results[0].inlier_rms};
    // rotation sigma
    std::vector<double> v_sigma_r;
    // translation sigma
    std::vector<double> v_sigma_t;

    int T_num = v_results.size();
    for(int i = 0; i < T_num; ++i){
        Eigen::Matrix4d T = v_results[i].T;
        Eigen::Matrix4d T_inv = T.inverse();
        
        Eigen::Matrix4d T_delt = T_inv * T_baseline;
        Eigen::AngleAxisd rot_vec(T_delt.block<3, 3>(0, 0));
        double delt_r = rot_vec.angle() * 180.0 / M_PI;
        Eigen::Vector3d t_delt = T_delt.block<3, 1>(0, 3);
        double delt_t = t_delt.norm() * 100;
        LOG(INFO) << "T_baseline and  T_" << i << " Rotation delta: "
                <<  delt_r << " °\n";
        LOG(INFO) << "T_baseline and  T_" << i << " Translation delta: " << delt_t
                << " cm\n";

        v_sigma_r.push_back(delt_r * delt_r);
        v_sigma_t.push_back(delt_t * delt_t);
        v_inlier_rms.push_back(v_results[i].inlier_rms);
    }

    double sigma_r = std::accumulate(v_sigma_r.begin(), v_sigma_r.end(), 0.0) / T_num;
    double sigma_t = std::accumulate(v_sigma_t.begin(), v_sigma_t.end(), 0.0) / T_num;
    LOG(INFO) << "Avg rotation sigma: " << sigma_r << ", Avg trans sigma: " << sigma_t;
    LOG(INFO) << "Final sigma_r" << std::sqrt(sigma_r) << ", sigma_t: " << std::sqrt(sigma_t);
    // LOG(INFO) << "Avg inlier rms: " << std::accumulate(v_inlier_rms.begin(), v_inlier_rms.end(), 0.0) / v_inlier_rms.size();

}

Eigen::Matrix4d readCameraPoseTxt(const std::string &file)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    std::ifstream fs(file);
    if (!fs.is_open())
    {
        std::cerr << "Fail to open camera pose file " << file << std::endl;
        return T;
    }

    std::string line;
    int i = 0;
    while (!fs.eof())
    {
        line.clear();
        getline(fs, line);
        if (line.empty())
            continue;

        std::stringstream ss(line);
        std::string dummy_str;
        if (i < 4)
        {
            ss >> T(i, 0) >> dummy_str >> T(i, 1) >> dummy_str >> T(i, 2) >> dummy_str >> T(i, 3);
            i++;
        }
        else
        {
            std::cerr << "Invalid camera pose format!\n";
            break;
        }
    }
    fs.close();

    return T;
}


int main(int argc, char **argv){
    if (argc < 6){
        LOG(FATAL) << "Usage: test_lidar2cam_calibration [init_T_l1_cam0.yaml] [dataset_folder] [cam_pose_file] [target_ply_file] [output_folder] [lidar_id]";
        return -1;
    }

    std::string init_ext_filepath(argv[1]);
    // dataset folder e.g. /path/data0/lidar_cam
    std::string dataset_folder(argv[2]);
    // teche0 pose w.r.t calibration reference
    std::string teche0_pose_filepath(argv[3]);
    // target pointcloud used in lidar localization
    std::string target_pcl_file_path(argv[4]);
    std::string output_folder(argv[5]);
    int lidar_id = 1;
    // specificly camera0!!!
    // int cam_id = 0;
    if (argc >= 7 ){
        lidar_id = std::atoi(argv[6]);
    }

    if(!common::fileExists(init_ext_filepath)){
        createLidarCamExtFile(init_ext_filepath, lidar_id);
    }
    if(!common::pathExists(dataset_folder)){
        LOG(FATAL) << "Input folder doesnot exist!";
        return -1;
    }
    if(!common::pathExists(output_folder)){
        if(!common::createPath(output_folder)){
            LOG(FATAL) << "Fail to create " << output_folder;
            return -1;
        }
    }
    
    // Eigen::Matrix4d T_base_l0 = horizontal_lidar_ptr->extrinsics();
    // Eigen::Matrix4d T_base_l1 = vertical_lidar_ptr->extrinsics();
    // Eigen::Matrix4d T_base_teche0 = teche0_ptr->extrinsics();
    // initial extrinsic between two lidar
    // Eigen::Matrix4d T_teche0_l1 = T_base_teche0.inverse() * T_base_l1;
    Eigen::Matrix4d T_l1_teche0 = Eigen::Matrix4d::Identity();
    if (!common::loadExtFileOpencv(init_ext_filepath, T_l1_teche0)){
        LOG(FATAL) << "Fail to load " << init_ext_filepath;
        return -1;
    }
    Eigen::Matrix4d T_teche0_l1 = T_l1_teche0.inverse();
    Eigen::Matrix4d T_w_teche0 = readCameraPoseTxt(teche0_pose_filepath);
    Eigen::Matrix4d T_w_l1_init = T_w_teche0 * T_teche0_l1;

    std::vector<RegistrationResult> v_extrinsics;
    // for (const auto & entry : boost::filesystem::directory_iterator(dataset_folder)){
    //     if(!boost::filesystem::is_directory(entry))
    //         continue;

    //     std::string data_folder = entry.path().string() + "/lidar_cam";
        std::string scan_folder_path = common::concatenateFolderAndFileName(dataset_folder, "lidar"+std::to_string(lidar_id));
        std::vector<std::string> v_pcl_paths;
        std::vector<std::string> paths = {scan_folder_path};
        common::getFileLists(paths, true, "ply", &v_pcl_paths);
        std::string src_pcl_file_path;
        for(size_t i = 0; i < v_pcl_paths.size(); ++i){
            std::string file_path = v_pcl_paths[i];
            std::string path, file_name;
            common::splitPathAndFilename(file_path, &path, &file_name);
            if(std::atoi(&file_name[0]) == lidar_id)
                src_pcl_file_path = file_path;
        }

        RegistrationResult regist_result;
        bool sts = alignTwoPointClouds(src_pcl_file_path, target_pcl_file_path, T_w_l1_init, regist_result);
        if(!sts){
            return -1;
        }
        
        Eigen::Matrix4d T_w_l1 = regist_result.T;
        // T_l1_teche0 after icp refine
        regist_result.T = T_w_l1.inverse() * T_w_teche0;
        std::cout << "Final T_l1_teche0: \n" << regist_result.T << "\n";
        std::string save_file_path = common::concatenateFolderAndFileName(output_folder, "lidar"+std::to_string(lidar_id)+"_to_camera0.yml");
        common::saveExtFileOpencv(save_file_path, regist_result.T);
        v_extrinsics.emplace_back(regist_result);
    // }

    // sort result by fitness
    // std::sort(v_extrinsics.begin(), v_extrinsics.end(), [&](RegistrationResult &res_a, RegistrationResult &res_b){
    //     return res_a.fitness > res_b.fitness;
    // });

    // evaluateExtrinsics(v_extrinsics, v_extrinsics[0].T);



    return 0;
}