#!/usr/bin/env bash

dataset_folder=$1
imu_yaml_file=$2
cams_chain_yaml_file=$3

if [ $# != 3 ]; then
    echo "USAGE: $0 [dataset_folder] [imu_yaml_file] [cams_chain_yaml_file]"
    echo " e.g.: $0 [dataset_folder] [imu_yaml_file] [cams_chain_yaml_file]"
    exit 1
fi

input_folder=${dataset_folder}
output_folder=${dataset_folder}


workspace_folder=/code/kalibr_calibration
# temporary data path
target_ros_data_path=${dataset_folder}/kalibr/cams/data

# image folder path
input_left_img_folder=${dataset_folder}"/left"
# imu file path
imu_raw_data_path=${dataset_folder}/imu_0.txt

# an executable python program----list_image.py
python_list_image_path=${workspace_folder}/list_images.py

# rename image
cd ${input_left_img_folder}
rename "s/0_/image_/" *
cp timestamp_0.txt ${dataset_folder}/image_timestamp.txt

# create image_lists.txt for left images and right images
echo "Prepare image list file!"
python ${python_list_image_path} -i ${input_left_img_folder} -o ${input_left_img_folder} -r
img_list_txt=${input_folder}/image_list.txt
# copy image_lists.txt file to dataset folder
cp ${input_left_img_folder}"/image_list.txt" ${img_list_txt}

# run kalibr script
cd ${workspace_folder}
./prepare_kalibr_for_backpack.py ${input_folder} ${output_folder} ${img_list_txt} ${imu_raw_data_path} \
    --target_data_path ${target_ros_data_path} --cam_model mono_imu --imu_yaml ${imu_yaml_file}  --cams_chain ${cams_chain_yaml_file}
