import os
import sys
import cv2
import time
import shutil
import argparse
import numpy as np


def splitStereoImages(raw_stereo_folder, new_stereo_folder, b_rotate_stereo):
    raw_left_cam_folder = os.path.join(raw_stereo_folder, 'cam0')
    raw_right_cam_folder = os.path.join(raw_stereo_folder, 'cam1')

    new_left_cam_folder = os.path.join(new_stereo_folder, 'cam0')
    new_right_cam_folder = os.path.join(new_stereo_folder, 'cam1')
    if not os.path.exists(new_left_cam_folder):
        os.makedirs(new_left_cam_folder)
    if not os.path.exists(new_right_cam_folder):
        os.makedirs(new_right_cam_folder)

    raw_imgs = [img for img in os.listdir(
        raw_left_cam_folder) if img.endswith('.jpg')]

    left_tms_name = os.path.join(new_left_cam_folder, 'timestamp_0.txt')
    right_tms_name = os.path.join(new_right_cam_folder, 'timestamp_1.txt')

    l_tms_fd = open(left_tms_name, 'w')
    r_tms_fd = open(right_tms_name, 'w')
    for img_name in raw_imgs:
        tms = img_name.split('.jpg')[0] + '000'
        # skip images whose timestamp is invalid
        if int(tms) <= 0:
            continue

        raw_l_img_path = os.path.join(raw_left_cam_folder, img_name)
        raw_r_img_path = os.path.join(raw_right_cam_folder, img_name)
        l_img = cv2.imread(raw_l_img_path)
        r_img = cv2.imread(raw_r_img_path)
        if b_rotate_stereo:
            l_img = cv2.rotate(l_img, cv2.ROTATE_180)
            r_img = cv2.rotate(r_img, cv2.ROTATE_180)
        # print("image shape:", img.shape)
        # cv2.imshow("original image", img)
        # cv2.waitKey(0)

        # cvt timestamp from us into ns
        new_img_name = tms + '.jpg'
        left_img_path = os.path.join(new_left_cam_folder, '0_' + new_img_name)
        right_img_path = os.path.join(new_right_cam_folder, '1_' + new_img_name)
        

        l_img_name = '0 ' + tms
        l_tms_fd.write(tms+ ' ' + l_img_name + '\n')
        r_img_name = '1 ' + tms
        r_tms_fd.write(tms+ ' ' + r_img_name + '\n')

        cv2.imwrite(left_img_path, l_img)
        cv2.imwrite(right_img_path, r_img)

    l_tms_fd.close()
    r_tms_fd.close()

def copyRGBImages(raw_rgb_folder, new_rgb_folder):
    if not os.path.exists(new_rgb_folder):
        os.makedirs(new_rgb_folder)
    
    raw_img_list = [img for img in os.listdir(raw_rgb_folder) if img.endswith('.jpg')]
    tms_file = os.path.join(new_rgb_folder, 'timestamp_0.txt')

    tms_fd = open(tms_file, 'w')

    for img_name in raw_img_list:
        # cvt timestamp from us into ns
        tms = img_name.split('.jpg')[0] + '000'
        new_img_name = tms + '.jpg'
        src_filepath = os.path.join(raw_rgb_folder, img_name)
        dst_filepath = os.path.join(new_rgb_folder, '0_'+new_img_name)

        shutil.copyfile(src_filepath, dst_filepath)

        tms_fd.write(tms+ ' 0 ' + tms + '\n')
    
    tms_fd.close()

    return True

def convImuData(raw_imu_filepath, conv_imu_filepath):
    new_imu_data = []
    # raw file format
    # timestmap(us) acc_x acc_y acc_z(G) gyr_x gyr_y gyr_z(rad/s) 
    # target file format
    # timestamp(us) gyro_x  gyro_y  gyro_z(rad/s) acc_x  acc_y  acc_z(m/s^2)  
    with open(raw_imu_filepath, 'r') as raw_file:
        all_lines = raw_file.readlines()
        for line in all_lines:
            raw_data = line.split()
            tms = raw_data[0]
            # ax = str(float(raw_data[1]) * 9.81)
            # ay = str(float(raw_data[2]) * 9.81)
            # az = str(float(raw_data[3]) * 9.81)
            ax = raw_data[1]
            ay = raw_data[2]
            az = raw_data[3]
            gx = raw_data[4]
            gy = raw_data[5]
            gz = raw_data[6]
            new_imu_data.append([tms, gx, gy, gz, ax, ay, az])

    with open(conv_imu_filepath, 'w') as new_file:
        for data in new_imu_data:
            new_file.write(' '.join(data))
            new_file.write('\n')

    return True

def main():
    parser = argparse.ArgumentParser(description="A helper to preprocess raw data capture from xvision")
    parser.add_argument('dataset_folder', type=str, default='',
                        help='folder contains stereo data, rgb data and imu data')
    parser.add_argument('output_folder', type=str,
                        default='', help='output folder path')
    parser.add_argument('--rotate_180', action='store_true', help='rotate the stereo image 180^o')

    args = parser.parse_args()

    dataset_folder = args.dataset_folder
    output_folder = args.output_folder
    b_rotate = args.rotate_180

    if not os.path.exists(dataset_folder):
        print("Folder {} does not exist.".format(dataset_folder))
        exit(1)

    raw_rgb_folder = os.path.join(dataset_folder, 'color')
    raw_stereo_folder = os.path.join(dataset_folder, 'stereo')
    raw_imu_filepath = os.path.join(dataset_folder, 'imu_0.txt')

    if not os.path.exists(raw_rgb_folder) or not os.path.exists(raw_stereo_folder):
        print("Folder {}, {} does not exist.".format(raw_rgb_folder, raw_stereo_folder))
        exit(1)

    new_stereo_folder = os.path.join(output_folder, 'stereo')
    splitStereoImages(raw_stereo_folder, new_stereo_folder, b_rotate)

    new_rgb_folder = os.path.join(output_folder, 'cam0')
    copyRGBImages(raw_rgb_folder, new_rgb_folder)

    new_imu_filepath = os.path.join(output_folder, 'imu_0.txt')
    if os.path.exists(raw_imu_filepath):
        # shutil.copyfile(raw_imu_filepath, new_imu_filepath)
        convImuData(raw_imu_filepath, new_imu_filepath)

if __name__ == '__main__':
    main()