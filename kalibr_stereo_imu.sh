#!/usr/bin/env bash

target_file=/home/ziqianbai/Projects/vlab/kalibr_calibration/april_6x6_80x80cm.yaml
data_path=/media/ziqianbai/DATA/kalibr/cams/data/.
result_path=/media/ziqianbai/DATA/kalibr/cams/result
bag_file=${result_path}/output.bag
imu_intrinsic_file=/media/ziqianbai/DATA/CALIBRATION/imu/backpack4/backpack4_imu.yaml
stereo_extrinsic_file=/media/ziqianbai/DATA/CALIBRATION/stereo/backpack4/20210111/left_00E35151656_00E35151728/s1/stereo.yaml
# output folder
output_folder=$1

# create folder
echo "dateset folder ${data_path}"
echo "create result folder ${result_path}"
rm -rf ${result_path}
mkdir -p ${result_path}

# create bag
echo "clean result path"
cd ${result_path}
echo "kalibr_bagcreater --folder ${data_path} --output-bag ${bag_file}"
kalibr_bagcreater --folder ${data_path} --output-bag ${bag_file}

cd ${result_path}

# calibrate camera-IMU
echo "calib cameras-IMU"
echo "kalibr_calibrate_imu_camera --bag ${bag_file} --target ${target_file} --cam ${stereo_extrinsic_file} --imu ${imu_intrinsic_file}"
kalibr_calibrate_imu_camera --bag ${bag_file} --target ${target_file} --cams ${stereo_extrinsic_file} --imu ${imu_intrinsic_file} --time-calibration 
# --recompute-camera-chain-extrinsics #--bag-from-to 11 121  --show-extraction

mv ${result_path}/*.yaml ${output_folder}/
mv ${result_path}/*.pdf ${output_folder}/
mv ${result_path}/*.txt ${output_folder}/
