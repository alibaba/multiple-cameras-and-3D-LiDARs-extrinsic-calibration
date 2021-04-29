#!/usr/bin/env bash

# camera distortion model
cam_model=pinhole-radtan
# cam_model=pinhole-equi
# cam_model=omni-radtan

# output folder
output_folder=$1

# result file
filename=monocular

target_file=/home/ziqianbai/Projects/vlab/kalibr_calibration/checkerboard_8x6_3x3cm.yaml
data_path=/media/ziqianbai/DATA/kalibr/cams/data/.
result_path=/media/ziqianbai/DATA/kalibr/cams/result
bag_file=${result_path}/output.bag

# create folder
echo "dateset folder ${data_path}"
echo "create result folder ${result_path}"
rm -rf ${result_path}
mkdir -p ${result_path}

# create bag
echo "create bag"
echo "kalibr_bagcreater --folder ${data_path} --output-bag ${bag_file}"
kalibr_bagcreater --folder ${data_path} --output-bag ${bag_file}

cd "${result_path}"

# calibrate monocular camera
if [ ${filename} = "monocular" ]; then
    echo "calib left camera"
    echo "kalibr_calibrate_cameras --target ${target_file} --dont-show-report \
    --bag ${bag_file} --models ${cam_model} --topics /cam0/image_raw"
    kalibr_calibrate_cameras --target ${target_file} --dont-show-report \
        --bag ${bag_file} --models ${cam_model} --topics /cam0/image_raw #--show-extraction
fi

cp ${result_path}/*.yaml ${output_folder}/${filename}.yaml
cp ${result_path}/*.pdf ${output_folder}/${filename}.pdf
cp ${result_path}/*.txt ${output_folder}/${filename}.txt
