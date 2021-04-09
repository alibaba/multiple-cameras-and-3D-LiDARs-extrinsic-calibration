#!/usr/bin/env bash


# imu_intrinsic_file=/media/ziqianbai/DATA/Backpack/backpack_v2_dev0/imu/new_backpack_imu.yaml
# mono_camera_file=/media/ziqianbai/DATA/Backpack/backpack_v2_dev0/gmsl_cameras/cam0/monocular.yaml

# dataset path
data_path="${1}"/.
result_path=$(dirname "${1}")/result
ros_bag_file="${result_path}"/output.bag
# output folder
output_folder=$2
# IMU_YAMLS
imu_intrinsic_file=$3
# cams_CHAIN_YAML
mono_camera_file=$4

target_file=/code/kalibr_calibration/april_6x6_80x80cm.yaml

# create folder
echo "dateset folder ${data_path}"
echo "create result folder ${result_path}"
rm -rf ${result_path}
mkdir -p ${result_path}

# create bag
echo "clean result path"
cd ${result_path}
echo "kalibr_bagcreater --folder ${data_path} --output-bag ${ros_bag_file}"
kalibr_bagcreater --folder ${data_path} --output-bag ${ros_bag_file}

cd ${result_path}

# calibrate camera-IMU
echo "calib camera-IMU"
echo "kalibr_calibrate_imu_camera --bag ${ros_bag_file} --target ${target_file} --cam ${mono_camera_file} --imu ${imu_intrinsic_file}"
kalibr_calibrate_imu_camera --bag ${ros_bag_file} --target ${target_file} --cams ${mono_camera_file} --imu ${imu_intrinsic_file} --time-calibration
# --recompute-camera-chain-extrinsics #--bag-from-to 11 121   --show-extraction

mv ${result_path}/*.yaml ${output_folder}/
mv ${result_path}/*.pdf ${output_folder}/
mv ${result_path}/*.txt ${output_folder}/
mv ${ros_bag_file} ${output_folder}/
