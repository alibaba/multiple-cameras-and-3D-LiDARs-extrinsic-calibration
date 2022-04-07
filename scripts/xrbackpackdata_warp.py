from genericpath import exists
import os
import sys
import cv2
import time
import glob
import shutil
import argparse
import numpy as np

import mv3dhelper

def readRawTimestampFile(raw_cam_tms_filepath):
    mv3dhelper.check_file_exist(raw_cam_tms_filepath, 'raw camera timestamp')

    img_timestamps = {}
    with open(raw_cam_tms_filepath, 'r') as img_ts_fd:
        for line in img_ts_fd:
            if(len(line) > 0):
                # data: img_idx timestamp
                img_ts = line.split()
                # src time unit: ms
                img_timestamps[img_ts[0]] = img_ts[1]

    return img_timestamps

def copyRGBImages(raw_rgb_folder, new_rgb_folder):
    if not os.path.exists(new_rgb_folder):
        os.makedirs(new_rgb_folder)
    
    raw_img_list = [img for img in os.listdir(raw_rgb_folder) if img.endswith('.jpg')]
    raw_img_list.sort(key=lambda x:int(x.split('_')[1].split('.')[0]))
    raw_tms_file = os.path.join(raw_rgb_folder, 'timestamp.txt')
    new_tms_file = os.path.join(new_rgb_folder, 'timestamp_0.txt')

    # raw timestamp: ms
    v_raw_tms = readRawTimestampFile(raw_tms_file)
    tms_fd = open(new_tms_file, 'w')

    for img_name in raw_img_list:
        img_idx = img_name.split('_')[1].split('.')[0]
        # cvt timestamp from ms into ns
        raw_tms = v_raw_tms[img_idx]
        new_tms = str(int(float(raw_tms) * 1000000))
        new_img_name = new_tms + '.jpg'
        src_filepath = os.path.join(raw_rgb_folder, img_name)
        raw_img = cv2.imread(src_filepath, -1)
        gray_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
        dst_filepath = os.path.join(new_rgb_folder, '0_'+new_img_name)

        # shutil.copyfile(src_filepath, dst_filepath)
        cv2.imwrite(dst_filepath, gray_img)

        tms_fd.write(new_tms+ ' 0 ' + new_tms + '\n')
    
    tms_fd.close()

    return True

def convImuData(raw_imu_filepath, conv_imu_filepath):
    new_imu_data = []
    # raw file format
    # timestmap(ms) acc_x acc_y acc_z(m/s^2) gyr_x gyr_y gyr_z(rad/s) temp  mag_x  mag_y  mag_z  q_w  q_x  q_y  q_z  
    # target file format
    # timestamp(us) gyro_x  gyro_y  gyro_z(rad/s) acc_x  acc_y  acc_z(m/s^2)  
    with open(raw_imu_filepath, 'r') as raw_file:
        all_lines = raw_file.readlines()
        for line in all_lines:
            raw_data = line.split()
            tms = raw_data[0]
            ax = raw_data[1]
            ay = raw_data[2]
            az = raw_data[3]
            gx = raw_data[4]
            gy = raw_data[5]
            gz = raw_data[6]
            new_imu_data.append([str(int(float(tms)*1000)), gx, gy, gz, ax, ay, az])

    with open(conv_imu_filepath, 'w') as new_file:
        for data in new_imu_data:
            new_file.write(' '.join(data))
            new_file.write('\n')

    return True

def convertIMUData(old_dataset_folder, new_dataset_folder):
    raw_imu_filepath = os.path.join(old_dataset_folder, 'imu/imu_0.txt')
    if not os.path.exists(raw_imu_filepath):
        print("File {} does not exist.".format(raw_imu_filepath))
        return

    print('-----------warp imu data in {}----------------'.format(old_dataset_folder))

    output_folder = os.path.join(new_dataset_folder, 'imu')
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    new_imu_filepath = os.path.join(output_folder, 'imu_0.txt')
    if os.path.exists(raw_imu_filepath):
        convImuData(raw_imu_filepath, new_imu_filepath)

def convertCamIMUData(old_dataset_folder, new_dataset_folder):
    raw_cam_folder = os.path.join(old_dataset_folder, 'cam_imu/cam0/cam0')
    raw_imu_filepath = os.path.join(old_dataset_folder, 'cam_imu/imu_0.txt')

    if not os.path.exists(raw_cam_folder) or not os.path.exists(raw_imu_filepath):
        print("Folder {}, {} does not exist.".format(raw_cam_folder, raw_imu_filepath))
        return

    print('-----------warp cam_imu data in {}----------------'.format(old_dataset_folder))

    output_folder = os.path.join(new_dataset_folder, 'cam_imu')
    mv3dhelper.create_folder_if_not_exists(output_folder)
    new_cam_folder = os.path.join(output_folder, 'cam0')
    copyRGBImages(raw_cam_folder, new_cam_folder)

    new_imu_filepath = os.path.join(output_folder, 'imu_0.txt')
    if os.path.exists(raw_imu_filepath):
        convImuData(raw_imu_filepath, new_imu_filepath)

def convertCamsData(old_dataset_folder, new_dataset_folder):
    old_cams_folder = os.path.join(old_dataset_folder, 'cams')
    new_cams_folder = os.path.join(new_dataset_folder, 'cams')

    print('-----------warp cams data in {}----------------'.format(old_dataset_folder))

    old_camx_folder = [ d for d in os.listdir(old_cams_folder) if d.startswith('cam')]
    for d in old_camx_folder:
        old_folder = os.path.join(old_cams_folder, d) 
        new_folder = os.path.join(os.path.join(new_cams_folder, d), 'cam0')
        if not os.path.exists(new_folder):
            os.makedirs(new_folder)
        
        img_files = glob.glob(old_folder+'/cam*/*.jpg')
        for img in img_files:
            shutil.copy(img, new_folder)

def convertLidarCamData(old_dataset_folder, new_dataset_folder):
    old_lidar_cam_folder = os.path.join(old_dataset_folder, 'lidar_camera')
    new_lidar_cam_folder = os.path.join(new_dataset_folder, 'lidar_cam')

    print('-----------warp lidar_cam data in {}----------------'.format(old_dataset_folder))

    if not os.path.exists(old_lidar_cam_folder):
        print('Folder {} doesnt exist!'.format(old_lidar_cam_folder))
        exit(-1)
    shutil.copytree(old_lidar_cam_folder, new_lidar_cam_folder)

def convertLidarLidarData(old_dataset_folder, new_dataset_folder):
    old_lidar_lidar_folder = os.path.join(old_dataset_folder, 'lidar_lidar')
    new_lidar_lidar_folder = os.path.join(new_dataset_folder, 'lidar_lidar')

    print('-----------warp lidar_lidar data in {}----------------'.format(old_dataset_folder))

    if not os.path.exists(old_lidar_lidar_folder):
        print('Folder {} doesnt exist!'.format(old_lidar_lidar_folder))
        exit(-1)
    shutil.copytree(old_lidar_lidar_folder, new_lidar_lidar_folder)


def main():
    parser = argparse.ArgumentParser(description="A helper to preprocess raw data capture from XR backpack")
    parser.add_argument('datasets_folder', type=str, default='',
                        help='raw folder of multiple datasets')
    parser.add_argument('output_folder', type=str,
                        default='', help='output folder path')

    args = parser.parse_args()

    datasets_folder = args.datasets_folder
    output_folder = args.output_folder

    # warp cams folder
    b_warp_cams = True
    # warp cam_imu folder
    b_warp_cam_imu = True
    # warp imu folder
    b_warp_imu = True
    # warp lidar_lidar folder
    b_warp_lidar_lidar = True
    # warp lidar_cam folder
    b_warp_lidar_cam = True

    if not os.path.exists(datasets_folder):
        print("Folder {} does not exist.".format(datasets_folder))
        exit(1)
    
    v_dataset_folder = [os.path.join(datasets_folder, d) for d in os.listdir(datasets_folder) if d.startswith('data')]
    for dataset_folder in v_dataset_folder:
        data_name = os.path.basename(dataset_folder)
        new_dataset_folder = os.path.join(output_folder, data_name)
        if not os.path.exists(new_dataset_folder):
            os.makedirs(new_dataset_folder)

        ################### CONVERT cams #########################
        if b_warp_cams:
            convertCamsData(dataset_folder, new_dataset_folder)

        ################### CONVERT imu #########################
        if b_warp_imu:
            convertIMUData(dataset_folder, new_dataset_folder)

        ################### CONVERT cam_imu #########################
        if b_warp_cam_imu:
            convertCamIMUData(dataset_folder, new_dataset_folder)

        ################### CONVERT lidar_cam #########################
        if b_warp_lidar_cam:
            convertLidarCamData(dataset_folder, new_dataset_folder)
        
        ################### CONVERT lidar_lidar #########################
        if b_warp_lidar_lidar:
            convertLidarLidarData(dataset_folder, new_dataset_folder)

        

if __name__ == '__main__':
    main()