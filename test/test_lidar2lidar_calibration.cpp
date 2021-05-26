
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
void createMultiLidarExtFile(const std::string& filename)
{
    // new backpack structural value
    Eigen::Matrix4d T_l0_l1_gt = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd l0_l1_vec1(-30 * M_PI / 180.0, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd l0_l1_vec2(-73.5 * M_PI / 180.0, Eigen::Vector3d(0, 1, 0));
    Eigen::Matrix3d l0_l1_vec = l0_l1_vec1.matrix() * l0_l1_vec2.matrix();
    Eigen::Vector3d t_l0_l1(-0.31405, 0, -0.39803);
    T_l0_l1_gt.block<3, 3>(0, 0) = l0_l1_vec;
    T_l0_l1_gt.block<3, 1>(0, 3) = l0_l1_vec1.matrix() * t_l0_l1;
    std::cout << "T_l0_l1_gt:\n" << T_l0_l1_gt << "\n";

    common::saveExtFileOpencv(filename, T_l0_l1_gt);   
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
    // icp_result = open3d::registration::RegistrationICP(*src_pcd_ptr, *target_pcd_ptr, max_corresp_dis, Eigen::Matrix4d::Identity(),
    //                                                         open3d::registration::TransformationEstimationPointToPlane(), icp_criteria);
    icp_result = open3d::registration::RegistrationICP(*src_pcd_ptr, *target_pcd_ptr, max_corresp_dis, Eigen::Matrix4d::Identity(),
                                                    open3d::registration::TransformationEstimationPointToPoint(false), icp_criteria);

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
    std::cout << "Final transformation:\n"
              << result.T << "\n";

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



int main(int argc, char **argv){
    if (argc < 3){
        LOG(FATAL) << "Usage: test_lidar2lidar_calibration [T_l0_l1_init.yaml] [input_dataset_folder] [output_folder] [lidar0_id] [lidar1_id]";
        return -1;
    }

    std::string init_ext_filepath(argv[1]);
    // dataset folder e.g. /path/data0/lidar_lidar
    std::string dataset_folder(argv[2]);
    std::string output_folder(argv[3]);
    int lidar0_id = 0, lidar1_id = 1;
    if (argc == 6){
        lidar0_id = std::stoi (argv[4]);
        lidar1_id = std::stoi (argv[5]);
    }

    // if initial extrinsic isn't here, create it
    if(!common::fileExists(init_ext_filepath)){
        createMultiLidarExtFile(init_ext_filepath);
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
    // initial extrinsic between two lidar
    Eigen::Matrix4d T_l0_l1_init = Eigen::Matrix4d::Identity();
    if (!common::loadExtFileOpencv(init_ext_filepath, T_l0_l1_init)){
        LOG(FATAL) << "Fail to load " << init_ext_filepath;
        return -1;
    }

    std::vector<RegistrationResult> v_extrinsics;

    // for (const auto & entry : boost::filesystem::directory_iterator(dataset_folder)){
    //     if(!boost::filesystem::is_directory(entry))
    //         continue;

    //     std::string scan_folder_path = entry.path().string() + "/lidar_lidar";
        std::string lidar0_scan_folder = common::concatenateFolderAndFileName(dataset_folder, "lidar0");
        std::string lidar1_scan_folder = common::concatenateFolderAndFileName(dataset_folder, "lidar1");
        std::vector<std::string> v_lidar0_pcl_paths,  v_lidar1_pcl_paths;
        std::vector<std::string> paths = {lidar0_scan_folder};
        common::getFileLists(paths, true, "ply", &v_lidar0_pcl_paths);
        paths = {lidar1_scan_folder};
        common::getFileLists(paths, true, "ply", &v_lidar1_pcl_paths);

        if (v_lidar0_pcl_paths.empty() || v_lidar1_pcl_paths.empty()){
            LOG(ERROR) << "lidar scan folder is empty!\n";
            return -1;
        }
        // only choose 1 scan pair for calibration
        std::string src_pcl_file_path, target_pcl_file_path;
        {
            std::string file_path = v_lidar0_pcl_paths[0];
            std::string path, file_name;
            common::splitPathAndFilename(file_path, &path, &file_name);
            if(std::atoi(&file_name[0]) == lidar0_id)
                target_pcl_file_path = file_path;
        }
        {
            std::string file_path = v_lidar1_pcl_paths[0];
            std::string path, file_name;
            common::splitPathAndFilename(file_path, &path, &file_name);
            if(std::atoi(&file_name[0]) == lidar1_id)
                src_pcl_file_path = file_path;
        }

        RegistrationResult regist_result;
        bool sts = alignTwoPointClouds(src_pcl_file_path, target_pcl_file_path, T_l0_l1_init, regist_result);
        if(!sts){
            return -1;
        }

        std::string save_file_path = common::concatenateFolderAndFileName(output_folder, "lidar0_to_lidar1.yml");
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