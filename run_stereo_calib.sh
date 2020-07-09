#!/usr/bin/env bash

dataset_folder=$1

if [ $# != 1 ]; then
    echo "USAGE: $0 dataset_folder"
    echo " e.g.: $0 /dataset_folder"
    exit 1
fi

# an executable python program----list_image.py
python_list_image_path=list_images.py
# check input up/low folder and images
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

# create image_lists.txt for up images and low images
echo "Prepare up and low image list file!"
python ${python_list_image_path} -i ${input_left_img_folder} -o ${input_left_img_folder} -r
python ${python_list_image_path} -i ${input_right_img_folder} -o ${input_right_img_folder} -r
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

input_folder=${dataset_folder}
output_folder=${dataset_folder}
target_ros_data_path=/media/ziqianbai/DATA/CALIBRATION/kalibr/cams/data

# run kalibr script
./prepare_kalibr.py ${input_folder} ${output_folder} ${input_folder}/image_list.txt --target_data_path ${target_ros_data_path} --cam_model stereo
