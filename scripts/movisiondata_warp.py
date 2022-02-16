import os
import sys
import cv2
import time
import shutil
import re
import argparse
import numpy as np


def split4FisheyeImages(raw_concat_img_folder, new_cam0_folder, new_cam1_folder, new_cam2_folder, new_cam3_folder, b_rotate):
    img_file_ext = '.png'
    if not os.path.exists(new_cam0_folder):
        os.makedirs(new_cam0_folder)
    if not os.path.exists(new_cam1_folder):
        os.makedirs(new_cam1_folder)
    if not os.path.exists(new_cam2_folder):
        os.makedirs(new_cam2_folder)
    if not os.path.exists(new_cam3_folder):
        os.makedirs(new_cam3_folder)

    raw_imgs = [img for img in os.listdir(
        raw_concat_img_folder) if img.endswith(img_file_ext)]

    cam0_tms_fname = os.path.join(new_cam0_folder, 'timestamp_0.txt')
    cam1_tms_fname = os.path.join(new_cam1_folder, 'timestamp_1.txt')
    cam2_tms_fname = os.path.join(new_cam2_folder, 'timestamp_2.txt')
    cam3_tms_fname = os.path.join(new_cam3_folder, 'timestamp_3.txt')

    cam0_tms_fd = open(cam0_tms_fname, 'w')
    cam1_tms_fd = open(cam1_tms_fname, 'w')
    cam2_tms_fd = open(cam2_tms_fname, 'w')
    cam3_tms_fd = open(cam3_tms_fname, 'w')
    for img_name in raw_imgs:
        raw_concat_img_path = os.path.join(raw_concat_img_folder, img_name)
        raw_concat_img = cv2.imread(raw_concat_img_path)
        if b_rotate:
            raw_concat_img = cv2.rotate(raw_concat_img, cv2.ROTATE_90_CLOCKWISE)
        # print("image shape:", raw_concat_img.shape)
        # cv2.imshow("original image", raw_concat_img)
        # cv2.waitKey(0)
        height, width, channel = raw_concat_img.shape
        if channel == 3: 
            raw_concat_img = cv2.cvtColor(raw_concat_img, cv2.COLOR_BGR2GRAY)
        cam0_img = raw_concat_img[:, 0:width//4]
        cam1_img = raw_concat_img[:, width//4: width//2]
        cam2_img = raw_concat_img[:, width//2: width//4*3]
        cam3_img = raw_concat_img[:, width//4*3:width]

        # cvt timestamp from us into ns
        tms = img_name.split(img_file_ext)[0]
        new_img_name = tms + img_file_ext
        new_cam0_img_path = os.path.join(new_cam0_folder, '0_' + new_img_name)
        new_cam1_img_path = os.path.join(new_cam1_folder, '1_' + new_img_name)
        new_cam2_img_path = os.path.join(new_cam2_folder, '2_' + new_img_name)
        new_cam3_img_path = os.path.join(new_cam3_folder, '3_' + new_img_name)
        

        cam0_img_name = '0 ' + tms
        cam0_tms_fd.write(tms+ ' ' + cam0_img_name + '\n')
        cam1_img_name = '1 ' + tms
        cam1_tms_fd.write(tms+ ' ' + cam1_img_name + '\n')
        cam2_img_name = '2 ' + tms
        cam2_tms_fd.write(tms+ ' ' + cam2_img_name + '\n')
        cam3_img_name = '3 ' + tms
        cam3_tms_fd.write(tms+ ' ' + cam3_img_name + '\n')

        cv2.imwrite(new_cam0_img_path, cam0_img)
        cv2.imwrite(new_cam1_img_path, cam1_img)
        cv2.imwrite(new_cam2_img_path, cam2_img)
        cv2.imwrite(new_cam3_img_path, cam3_img)

    cam0_tms_fd.close()
    cam1_tms_fd.close()
    cam2_tms_fd.close()
    cam3_tms_fd.close()


def convImuData(raw_imu_filepath, conv_imu_filepath):
    new_imu_data = []
    # raw file format
    # timestmap(ns) gyr_x gyr_y gyr_z(rad/s) acc_x acc_y acc_z(m/s^2)
    # target file format
    # timestamp(ns) gyro_x  gyro_y  gyro_z(rad/s) acc_x  acc_y  acc_z(m/s^2)  
    with open(raw_imu_filepath, 'r') as raw_file:
        all_lines = raw_file.readlines()
        for line in all_lines:
            raw_data = re.split(',| |\n', line)
            if raw_data[0].isalpha():
                continue
            tms = raw_data[0]
            gx = raw_data[1]
            gy = raw_data[2]
            gz = raw_data[3]
            ax = raw_data[4]
            ay = raw_data[5]
            az = raw_data[6]
            new_imu_data.append([tms, gx, gy, gz, ax, ay, az])

    with open(conv_imu_filepath, 'w') as new_file:
        for data in new_imu_data:
            new_file.write(' '.join(data))
            new_file.write('\n')

    return True

def main():
    parser = argparse.ArgumentParser(description="A helper to preprocess raw data capture from movision-V02A")
    parser.add_argument('dataset_folder', type=str, default='',
                        help='folder contains 4 fisheye data, imu data and hanshake data')
    parser.add_argument('output_folder', type=str,
                        default='', help='output folder path')
    parser.add_argument('--rotate_90', action='store_true', help='rotate the stereo image 180^o')

    args = parser.parse_args()

    dataset_folder = args.dataset_folder
    output_folder = args.output_folder
    b_rotate = args.rotate_90

    if not os.path.exists(dataset_folder):
        print("Folder {} does not exist.".format(dataset_folder))
        exit(1)
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 4 cameras folder exist, but we only need the cam0 folder
    raw_cam0_folder = os.path.join(dataset_folder, 'cam0')
    raw_cam1_folder = os.path.join(dataset_folder, 'cam1')
    raw_cam2_folder = os.path.join(dataset_folder, 'cam2')
    raw_cam3_folder = os.path.join(dataset_folder, 'cam3')
    raw_imu_filepath = os.path.join(dataset_folder, 'imu0.csv')

    if not os.path.exists(raw_cam0_folder) or not os.path.exists(raw_imu_filepath):
        print("Folder {}, {} does not exist.".format(raw_cam0_folder, raw_imu_filepath))
        exit(1)

    new_cam0_folder = os.path.join(output_folder, 'cam0')
    new_cam1_folder = os.path.join(output_folder, 'cam1')
    new_cam2_folder = os.path.join(output_folder, 'cam2')
    new_cam3_folder = os.path.join(output_folder, 'cam3')
    split4FisheyeImages(raw_cam0_folder, new_cam0_folder, new_cam1_folder, new_cam2_folder, new_cam3_folder, b_rotate)


    new_imu_filepath = os.path.join(output_folder, 'imu_0.txt')
    if os.path.exists(raw_imu_filepath):
        convImuData(raw_imu_filepath, new_imu_filepath)

if __name__ == '__main__':
    main()