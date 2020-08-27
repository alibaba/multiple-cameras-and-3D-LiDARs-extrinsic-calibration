#!/usr/bin/env bash

target_file=/home/ziqianbai/Projects/vlab/kalibr_calibration/april_6x6_80x80cm.yaml
data_path=/home/ziqianbai/DATA_TEMP/kalibr/cams/data/.
result_path=/home/ziqianbai/DATA_TEMP/kalibr/cams/result
bag_file=${result_path}/output.bag

# output folder
output_folder=$1

# rename result file
filename=stereo

# cam_model=pinhole-radtan
cam_model=pinhole-equi
# cam_model=omni-radtan

# create folder
echo "dateset folder ${data_path}"
echo "create result folder ${result_path}"
rm -rf ${result_path}
mkdir -p ${result_path}

# create bag
echo "create bag"
echo "kalibr_bagcreater --folder ${data_path} --output-bag ${bag_file}"
kalibr_bagcreater --folder ${data_path} --output-bag ${bag_file}

cd ${result_path}

# calibrate camera
echo "calib stereo cameras"
echo "kalibr_calibrate_cameras --target ${target_file} --dont-show-report \
    --bag ${bag_file} --models ${cam_model} ${cam_model} --topics /cam0/image_raw /cam1/image_raw"
kalibr_calibrate_cameras --target ${target_file} --dont-show-report \
    --bag ${bag_file} --models ${cam_model} ${cam_model} --topics /cam0/image_raw /cam1/image_raw #--plot

cp ${result_path}/*.yaml ${output_folder}/${filename}.yaml
cp ${result_path}/*.pdf ${output_folder}/${filename}.pdf
cp ${result_path}/*.txt ${output_folder}/${filename}.txt
