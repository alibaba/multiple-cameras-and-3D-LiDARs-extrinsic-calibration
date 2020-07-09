#!/usr/bin/env bash

target_file=/media/ziqianbai/DATA/CALIBRATION/script/april_6x6_80x80cm.yaml
data_path=/media/ziqianbai/DATA/CALIBRATION/kalibr/cams/data/.
result_path=/media/ziqianbai/DATA/CALIBRATION/kalibr/cams/result
bag_file=${result_path}/output.bag
imu_intrinsic_file=/media/ziqianbai/DATA/CALIBRATION/backpack/backpack1_imu_xsens.yaml
stereo_extrinsic_file=/media/ziqianbai/DATA/CALIBRATION/AprilTag/2020-0707/left_00E35151666_00E35151671/20200707-095258/stereo_result.yaml
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
rm -rf *
echo "kalibr_bagcreater --folder ${data_path} --output-bag ${bag_file}"
kalibr_bagcreater --folder ${data_path} --output-bag ${bag_file}

cd ${result_path}

# calibrate camera-IMU
echo "calib cameras-IMU"
echo "kalibr_calibrate_imu_camera --bag ${bag_file} - --target ${target_file} --cam ${stereo_extrinsic_file} --imu ${imu_intrinsic_file}"
kalibr_calibrate_imu_camera --bag ${bag_file} --target ${target_file} --cam ${stereo_extrinsic_file} --imu ${imu_intrinsic_file} --time-calibration
# --recompute-camera-chain-extrinsics #--bag-from-to 11 121 # --show-extraction

mv ${result_path}/*.yaml ${output_folder}/
mv ${result_path}/*.pdf ${output_folder}/
mv ${result_path}/*.txt ${output_folder}/
