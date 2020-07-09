#!/usr/bin/env bash

dataset_folder=$1

if [ $# != 1 ]; then
    echo "USAGE: $0 dataset_folder"
    echo " e.g.: $0 /dataset_folder"
    exit 1
fi

workspace_folder=/media/ziqianbai/DATA/CALIBRATION/script
input_folder=${dataset_folder}
output_folder=${dataset_folder}
target_ros_data_path=/media/ziqianbai/DATA/CALIBRATION/kalibr/cams/data
imu_raw_data_path=/media/ziqianbai/DATA/CALIBRATION/camera-IMU/backpack1_2020-0707/0707_2/imu_0.txt

# an executable python program----list_image.py
python_list_image_path=${workspace_folder}/list_images.py
# check input left/right folder and images
for element in $(ls ${dataset_folder}); do
    dir_or_file=$1"/"${element}
    if [ -d ${dir_or_file} ]; then
        echo "find folder ${dir_or_file}"
    else
        echo ${dir_or_file}
    fi
done

input_left_img_folder=${dataset_folder}"/left"
input_right_img_folder=${dataset_folder}"/right"

cd ${input_left_img_folder}
rename "s/0_/image_/" *

cd ${input_right_img_folder}
rename "s/1_/image_/" *

cd ${workspace_folder}

# create image_lists.txt for left images and right images
echo "Prepare left and right image list file!"
python ${python_list_image_path} -i ${input_left_img_folder} -o ${input_left_img_folder}
python ${python_list_image_path} -i ${input_right_img_folder} -o ${input_right_img_folder}
# check image_lists.txt
img_list_txt=${input_left_img_folder}"/image_list.txt"
if [ ! -f "${img_list_txt}" ]; then
    echo "${img_list_txt} is no exist!"
    exit
fi
img_list_txt=${input_right_img_folder}"/image_list.txt"
if [ ! -f "${img_list_txt}" ]; then
    echo "${img_list_txt} is no exist!"
    exit
fi
# copy image list file to dataset folder
cp ${img_list_txt} ${dataset_folder}

# run kalibr script
./prepare_kalibr_for_backpack.py ${input_folder} ${output_folder} ${input_folder}/image_list.txt ${imu_raw_data_path} \
    --target_data_path ${target_ros_data_path} --cam_model stereo_imu
